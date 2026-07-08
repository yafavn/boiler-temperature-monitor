#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "onewire_bus.h"
#include "ds18b20.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "BOILER_MONITOR";
static const char *APP_VERSION = "1.2.1"; // גרסה מעודכנת

#define ONE_WIRE_BUS GPIO_NUM_4
#define GND_SWITCH_PIN GPIO_NUM_25
#define VOLTAGE_ADC_CHANNEL ADC_CHANNEL_7

// פונקציות עזר לסוללה
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
  // חישוב ליניארי פשוט לטווח העבודה
  return (v - 3.30f) / (4.20f - 3.30f) * 100.0f;
}

void app_main(void)
{
  ESP_LOGI(TAG, "Boiler Monitor version %s", APP_VERSION);

  // 1. אתחול פין GND ממותג
  gpio_reset_pin(GND_SWITCH_PIN);
  gpio_set_direction(GND_SWITCH_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(GND_SWITCH_PIN, 0);

  // 2. אתחול ADC לסוללה
  adc_oneshot_unit_handle_t adc1_handle;
  adc_oneshot_unit_init_cfg_t init_config = {.unit_id = ADC_UNIT_1};
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

  adc_oneshot_chan_cfg_t config = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VOLTAGE_ADC_CHANNEL, &config));

  adc_cali_handle_t cali_handle = nullptr;
  adc_cali_line_fitting_config_t cali_config = {.unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
  ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));

  // 3. אתחול OneWire
  onewire_bus_handle_t bus = NULL;
  onewire_bus_config_t bus_config = {.bus_gpio_num = ONE_WIRE_BUS};
  onewire_bus_rmt_config_t rmt_config = {.max_rx_bytes = 10};
  ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

  ds18b20_device_handle_t ds18b20 = NULL;

  while (1)
  {
    // --- 1. הכנה למדידות (חיבור GND) ---
    gpio_set_direction(GND_SWITCH_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(GND_SWITCH_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1000)); // המתנה ארוכה להתייצבות החיישן

    // --- 2. קריאת טמפרטורה ---
    float temp_c = 0;
    // --- איתור/חיבור החיישן ---
    if (ds18b20 == NULL)
    {
      ESP_LOGI(TAG, "Attempting to find DS18B20...");
      onewire_device_iter_handle_t iter = NULL;
      onewire_new_device_iter(bus, &iter);

      onewire_device_t next_onewire_device;
      if (onewire_device_iter_get_next(iter, &next_onewire_device) == ESP_OK)
      {
        ESP_LOGI(TAG, "Device found, initializing...");
        ds18b20_config_t ds_cfg = {};
        if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20) == ESP_OK)
        {
          ESP_LOGI(TAG, "DS18B20 initialized successfully!");
        }
        else
        {
          ESP_LOGE(TAG, "Failed to initialize DS18B20 handle");
        }
      }
      else
      {
        ESP_LOGW(TAG, "No device found on the bus during scan");
      }
      onewire_del_device_iter(iter);
    }

    // --- קריאת טמפרטורה ---

    if (ds18b20 != NULL)
    {
      ESP_LOGI(TAG, "Triggering conversion...");
      if (ds18b20_trigger_temperature_conversion(ds18b20) == ESP_OK)
      {
        vTaskDelay(pdMS_TO_TICKS(800));
        if (ds18b20_get_temperature(ds18b20, &temp_c) == ESP_OK)
        {
          ESP_LOGI(TAG, "Temperature: %.1f C", temp_c);
        }
      }
      else
      {
        ESP_LOGE(TAG, "Trigger failed");
      }
    }

    // --- 3. קריאת סוללה ---
    int raw = 0;
    int voltage_mv = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, VOLTAGE_ADC_CHANNEL, &raw));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv));

    float battery_voltage = (voltage_mv / 1000.0f) * 2.0f;
    float battery_percent = clampf(battery_percent_from_voltage(battery_voltage), 0.0f, 100.0f);
    ESP_LOGI(TAG, "Battery: %.3f V (%.1f%%)", battery_voltage, battery_percent);

    // --- 4. ניהול חשמל ושינה ---
    gpio_set_direction(GND_SWITCH_PIN, GPIO_MODE_INPUT); // ניתוק ה-GND

    vTaskDelay(pdMS_TO_TICKS(59000)); // המתנה לדקה הבאה (פחות הזמן שכבר עבר)
  }
}