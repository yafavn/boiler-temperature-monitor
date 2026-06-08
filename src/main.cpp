#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "BATTERY_ADC";
static const char *APP_VERSION = "1.0.0";

struct BatteryPoint
{
  float voltage;
  float percent;
};

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
  static const BatteryPoint table[] = {
      {4.20f, 100.0f},
      {4.10f, 90.0f},
      {4.00f, 80.0f},
      {3.90f, 65.0f},
      {3.80f, 50.0f},
      {3.70f, 20.0f},
      {3.60f, 10.0f},
      {3.50f, 5.0f},
      {3.30f, 0.0f}};

  if (v >= table[0].voltage)
    return 100.0f;

  const int count = sizeof(table) / sizeof(table[0]);

  for (int i = 0; i < count - 1; i++)
  {
    if (v <= table[i].voltage && v >= table[i + 1].voltage)
    {
      float v1 = table[i].voltage;
      float p1 = table[i].percent;
      float v2 = table[i + 1].voltage;
      float p2 = table[i + 1].percent;

      float ratio = (v - v2) / (v1 - v2);
      return p2 + ratio * (p1 - p2);
    }
  }

  return 0.0f;
}

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "Boiler Temperature Monitor version %s", APP_VERSION);

  adc_oneshot_unit_handle_t adc1_handle = nullptr;

  adc_oneshot_unit_init_cfg_t init_config = {};
  init_config.unit_id = ADC_UNIT_1;
  init_config.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
  init_config.ulp_mode = ADC_ULP_MODE_DISABLE;
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

  adc_oneshot_chan_cfg_t config = {};
  config.atten = ADC_ATTEN_DB_12;
  config.bitwidth = ADC_BITWIDTH_DEFAULT;
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config));

  adc_cali_handle_t cali_handle = nullptr;
  adc_cali_line_fitting_config_t cali_config = {};
  cali_config.unit_id = ADC_UNIT_1;
  cali_config.atten = ADC_ATTEN_DB_12;
  cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;
  ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));

  const float divider_ratio = 2.0f;
  const float calibration_factor = 1.00f;

  while (1)
  {
    int raw = 0;
    int voltage_mv = 0;

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &raw));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv));

    float pin_voltage = voltage_mv / 1000.0f;
    float battery_voltage = pin_voltage * divider_ratio * calibration_factor;
    float battery_percent = clampf(battery_percent_from_voltage(battery_voltage), 0.0f, 100.0f);

    ESP_LOGI(TAG,
             "raw=%d, pin=%.3f V, battery=%.3f V, battery=%.1f%%",
             raw, pin_voltage, battery_voltage, battery_percent);

    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}