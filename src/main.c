#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "mqtt_client.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include "esp_sleep.h"
#include "esp_http_client.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "lwip/sockets.h"
#include "esp_timer.h"

// --- ספריות ה-Component Manager ---
#include "onewire_bus.h"
#include "ds18b20.h"

static const char *TAG = "BOILER_MONITOR";
static const char *APP_VERSION = "1.2.9";

#include "config.h"

// --- הגדרת פינים ---
#define ONE_WIRE_BUS GPIO_NUM_4
#define GND_SWITCH_PIN GPIO_NUM_25
#define BOILER_SIGNAL_PIN GPIO_NUM_14
#define VOLTAGE_ADC_CHANNEL ADC_CHANNEL_7

// טופיקים של MQTT
#define MQTT_TOPIC_DATA "boiler/monitor/state"
#define MQTT_TOPIC_OTA_CMD "boiler/monitor/ota_cmd"
#define MQTT_TOPIC_OTA_RES "boiler/monitor/ota_result"

// פלגים עבור WiFi ו-MQTT
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define MQTT_CONNECTED_BIT BIT1

static esp_mqtt_client_handle_t mqtt_client = NULL;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t cali_handle = NULL;
static onewire_bus_handle_t ow_bus = NULL;
static ds18b20_device_handle_t ds18b20 = NULL;

static int telnet_client_sock = -1;
static float current_battery_percent = 0.0f;

// משתנה גלובלי לסימון אם תהליך OTA רץ כעת (מונע כניסה לשינה באמצע צריבה)
static bool s_ota_in_progress = false;

// הגדרת הגבלת זמן עבודה על סוללה - 25 שניות
#define BATTERY_MODE_TIMEOUT_MS (25 * 1000) 
// מספר ניסיונות חיבור WiFi מקסימליים לפני ויתור
#define MAXIMUM_WIFI_RETRY 5
static int s_wifi_retry_num = 0;

// --- פונקציות עזר לסוללה ---
static float clampf(float x, float lo, float hi)
{
  if (x < lo)
    return lo;
  if (x > hi)
    return hi;
  return x;
}

static float battery_percent_from_voltage(float v)
{
  if (v >= 4.20f)
    return 100.0f;
  if (v <= 3.30f)
    return 0.0f;
  return (v - 3.30f) / (4.20f - 3.30f) * 100.0f;
}

// --- הפניית הלוגים ל-Telnet ---
static int telnet_log_vprintf(const char *fmt, va_list args)
{
  char buf[256];
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  if (telnet_client_sock >= 0 && len > 0)
  {
    send(telnet_client_sock, buf, len, 0);
  }
  return vprintf(fmt, args); // מדפיס גם ל-Serial הרגיל
}

// --- משימת שרת Telnet ---
static void telnet_server_task(void *pvParameters)
{
  char rx_buffer[128];
  int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  struct sockaddr_in dest_addr = {
      .sin_addr.s_addr = htonl(INADDR_ANY),
      .sin_family = AF_INET,
      .sin_port = htons(TELNET_PORT)};
  
  int opt = 1;
  setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  
  bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  listen(listen_sock, 1);

  while (1)
  {
    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);
    int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
    if (sock >= 0)
    {
      telnet_client_sock = sock;
      ESP_LOGI(TAG, "Telnet client connected. Redirecting logs...");
      while (1)
      {
        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len <= 0)
          break;
      }
      telnet_client_sock = -1;
      close(sock);
      ESP_LOGI(TAG, "Telnet client disconnected.");
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// --- אירועי WiFi משודרגים (כולל Auto-Reconnect מוגבל) ---
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    s_wifi_retry_num = 0;
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    if (s_wifi_retry_num < MAXIMUM_WIFI_RETRY) {
      s_wifi_retry_num++;
      ESP_LOGW(TAG, "WiFi connection lost. Reconnecting... (%d/%d)", s_wifi_retry_num, MAXIMUM_WIFI_RETRY);
      esp_wifi_connect();
    } else {
      ESP_LOGE(TAG, "WiFi connection failed completely after %d retries.", MAXIMUM_WIFI_RETRY);
    }
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    s_wifi_retry_num = 0; // איפוס המונה ברגע שיש חיבור יציב ו-IP
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

// --- פונקציית ביצוע ה-OTA ---
static void perform_ota(const char *url)
{
  s_ota_in_progress = true; // חסימת כניסה למצב שינה במהלך שדרוג
  ESP_LOGI(TAG, "Starting OTA update from: %s", url);
  esp_http_client_config_t config = {.url = url, .skip_cert_common_name_check = true};
  esp_https_ota_config_t ota_config = {.http_config = &config};

  esp_err_t ret = esp_https_ota(&ota_config);
  if (ret == ESP_OK)
  {
    ESP_LOGI(TAG, "OTA Successful! Rebooting...");
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_OTA_RES, "{\"status\":\"success\",\"msg\":\"Updated successfully\"}", 0, 1, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
  }
  else
  {
    ESP_LOGE(TAG, "OTA Failed! Error code: %d", ret);
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_OTA_RES, "{\"status\":\"failed\",\"msg\":\"Flash process failed\"}", 0, 1, 0);
    s_ota_in_progress = false; // שחרור נעילת השינה במקרה של כשל
  }
}

// --- אירועי MQTT ---
static void mqtt_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  esp_mqtt_event_handle_t event = event_data;
  switch ((esp_mqtt_event_id_t)event_id)
  {
  case MQTT_EVENT_CONNECTED:
    xEventGroupSetBits(s_wifi_event_group, MQTT_CONNECTED_BIT);
    esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_OTA_CMD, 1);
    break;
  case MQTT_EVENT_DISCONNECTED:
    xEventGroupClearBits(s_wifi_event_group, MQTT_CONNECTED_BIT);
    break;
  case MQTT_EVENT_DATA:
    if (strncmp(event->topic, MQTT_TOPIC_OTA_CMD, event->topic_len) == 0)
    {
      char url[256] = {0};
      memcpy(url, event->data, event->data_len < 255 ? event->data_len : 255);

      int boiler_on = gpio_get_level(BOILER_SIGNAL_PIN);
      if (!boiler_on || current_battery_percent < 50.0f)
      {
        char err_msg[128];
        snprintf(err_msg, sizeof(err_msg), "{\"status\":\"denied\",\"battery\":%.1f,\"boiler\":%d}", current_battery_percent, boiler_on);
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_OTA_RES, err_msg, 0, 1, 0);
        ESP_LOGW(TAG, "OTA Denied: Battery %.1f%%, Boiler AC: %d", current_battery_percent, boiler_on);
      }
      else
      {
        perform_ota(url);
      }
    }
    break;
  default:
    break;
  }
}

void app_main(void)
{
  // אתחול זיכרון פלאש פנימי
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  esp_log_set_vprintf(telnet_log_vprintf);

  // שמירת זמן ריצת המערכת במילישניות מתחילת האתחול לצורך ניהול ה-Timeout
  int64_t start_time = esp_timer_get_time() / 1000;

  // אתחול פינים ומחלק מתח
  gpio_reset_pin(GND_SWITCH_PIN);
  gpio_set_direction(GND_SWITCH_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(GND_SWITCH_PIN, 0); // חיבור אדמה לחיישן

  gpio_reset_pin(BOILER_SIGNAL_PIN);
  gpio_set_direction(BOILER_SIGNAL_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BOILER_SIGNAL_PIN, GPIO_FLOATING); // הגנה מפני התנגדות פנימית משתנה

  // אתחול ADC לסוללה
  adc_oneshot_unit_init_cfg_t init_config = {.unit_id = ADC_UNIT_1};
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));
  adc_oneshot_chan_cfg_t config = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VOLTAGE_ADC_CHANNEL, &config));
  adc_cali_line_fitting_config_t cali_config = {.unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
  ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));

  // אתחול רשת וארועים
  s_wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

  wifi_config_t wifi_config = {.sta = {.ssid = WIFI_SSID, .password = WIFI_PASS}};
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  // אתחול שרת Telnet למעקב
  xTaskCreate(telnet_server_task, "telnet_server", 4096, NULL, 5, NULL);

  // אתחול ה-OneWire Bus והגדרת החיישן
  onewire_bus_config_t bus_config = {.bus_gpio_num = ONE_WIRE_BUS};
  onewire_bus_rmt_config_t rmt_config = {.max_rx_bytes = 10};
  ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &ow_bus));

  // המתנה לחיבור לרשת לצורך סנכרון זמן (מוגבל ב-Timeout למצב סוללה)
  // בדיקה האם המכשיר התעורר משינה עמוקה (Deep Sleep)
  bool is_wakeup_from_sleep = (esp_sleep_get_wakeup_causes() != 0);
  uint32_t wait_ms = is_wakeup_from_sleep ? 10000 : portMAX_DELAY;
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(wait_ms));

  if (bits & WIFI_CONNECTED_BIT) {
    // אתחול NTP וסנכרון שעה
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    setenv("TZ", "IST-2IDT,M3.4.4/26,M10.5.0", 1); // הגדרת שעון ישראל
    tzset();

    // אתחול MQTT החדש
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.hostname = MQTT_BROKER_IP,
        .broker.address.port = MQTT_PORT,
        .broker.address.transport = MQTT_TRANSPORT_OVER_TCP,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASSWORD};
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    // המתנה לחיבור מלא ל-MQTT (מקסימום 5 שניות)
    xEventGroupWaitBits(s_wifi_event_group, MQTT_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(5000));
  } else {
    ESP_LOGE(TAG, "Skipping NTP and MQTT init due to WiFi connection timeout.");
  }

  while (1)
  {
    int boiler_ac_on = gpio_get_level(BOILER_SIGNAL_PIN);
    ESP_LOGI(TAG, "Boiler AC Status: %s", boiler_ac_on ? "ON (220V Connected)" : "OFF (Battery Mode)");

    int64_t current_time = esp_timer_get_time() / 1000;

    // מנגנון הגנה: יציאה ל-Deep Sleep בחיבור סוללה במקרה של עבודה ממושכת מדי
    if (!boiler_ac_on && !s_ota_in_progress && (current_time - start_time > BATTERY_MODE_TIMEOUT_MS))
    {
      ESP_LOGW(TAG, "Battery mode timeout reached (%lld ms). Force sleeping...", current_time - start_time);
      gpio_set_direction(GND_SWITCH_PIN, GPIO_MODE_INPUT); // כיבוי אדמת החיישן
      vTaskDelay(pdMS_TO_TICKS(1000)); // לוגים אחרונים
      
      // הגדרת השכמה מיידית במידה והדוד נדלק (סיגנל הופך ל-1/HIGH)
      esp_sleep_enable_ext0_wakeup(BOILER_SIGNAL_PIN, 1);
      esp_deep_sleep(5 * 60 * 1000000); // 5 דקות שינה
    }

    // הפעלת אדמה לחיישן וקריאה
    gpio_set_direction(GND_SWITCH_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(GND_SWITCH_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (ds18b20 == NULL)
    {
      onewire_device_iter_handle_t iter = NULL;
      onewire_new_device_iter(ow_bus, &iter);
      onewire_device_t next_onewire_device;
      if (onewire_device_iter_get_next(iter, &next_onewire_device) == ESP_OK)
      {
        ds18b20_config_t ds_cfg = {};
        ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20);
      }
      onewire_del_device_iter(iter);
    }

    float temp_c = 0.0f;
    if (ds18b20 != NULL)
    {
      if (ds18b20_trigger_temperature_conversion(ds18b20) == ESP_OK)
      {
        vTaskDelay(pdMS_TO_TICKS(800));
        ds18b20_get_temperature(ds18b20, &temp_c);
      }
    }

    // קריאת סוללה
    int raw = 0, voltage_mv = 0;
    adc_oneshot_read(adc1_handle, VOLTAGE_ADC_CHANNEL, &raw);
    adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv);
    float battery_voltage = (voltage_mv / 1000.0f) * 2.0f;
    current_battery_percent = clampf(battery_percent_from_voltage(battery_voltage), 0.0f, 100.0f);

    // השגת זמן מקומי מסונכרן
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);

    // שליחת נתונים רק אם יש חיבור MQTT פעיל
    EventBits_t conn_bits = xEventGroupGetBits(s_wifi_event_group);
    if ((conn_bits & MQTT_CONNECTED_BIT) && mqtt_client != NULL) {
      char json_payload[256];
      snprintf(json_payload, sizeof(json_payload),
               "{\"timestamp\":\"%s\",\"temperature\":%.1f,\"battery_v\":%.3f,\"battery_p\":%.1f,\"boiler_status\":%d,\"version\":\"%s\"}",
               strftime_buf, temp_c, battery_voltage, current_battery_percent, boiler_ac_on, APP_VERSION);

      esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DATA, json_payload, 0, 1, 0);
      ESP_LOGI(TAG, "Sent data: %s", json_payload);
    } else {
      ESP_LOGW(TAG, "MQTT not connected. Data not sent to broker.");
    }

    // כיבוי אדמת החיישן לחסכון בחשמל
    gpio_set_direction(GND_SWITCH_PIN, GPIO_MODE_INPUT);

    if (boiler_ac_on)
    {
      // אם הדוד דולק: נשארים ערים, וממתינים דקה אחת
      vTaskDelay(pdMS_TO_TICKS(60000));
    }
    else
    {
      // אם הדוד כבוי ומנגנון ה-OTA לא עובד ברקע: נכנסים ל-Deep Sleep
      if (!s_ota_in_progress) {
        ESP_LOGI(TAG, "Entering Deep Sleep (Wake up in 5m OR instantly when Boiler is ON)...");
        vTaskDelay(pdMS_TO_TICKS(1000));  // נותנים ללוגים להישלח
        
        // הגדרת השכמה מהירה מהסיגנל (פין 14 עולה ל-HIGH)
        esp_sleep_enable_ext0_wakeup(BOILER_SIGNAL_PIN, 1);
        esp_deep_sleep(5 * 60 * 1000000); 
      } else {
        // מונע השבתה במהלך עדכון קושחה
        ESP_LOGI(TAG, "OTA upgrade is active, holding active mode.");
        vTaskDelay(pdMS_TO_TICKS(5000));
      }
    }
  }
}