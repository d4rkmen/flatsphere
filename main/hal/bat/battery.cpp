/**
 * @file Battery.cpp
 * @brief Battery management class for ADC-based voltage and level reading
 * @author d4rkmen
 * @license Apache License 2.0
 */

#include "battery.hpp"
#include "esp_log.h"

using namespace HAL;

static const char* TAG = "Battery";

Battery::Battery()
    : _adc_handle(nullptr), _adc_raw(0), _voltage_mv(0), _read_task_handle(nullptr), _running(false)
{
    init();
}

Battery::~Battery() { deinit(); }

bool IRAM_ATTR Battery::_adc_conv_done_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t* edata, void* user_data)
{
    BaseType_t mustYield = pdFALSE;
    Battery* battery = static_cast<Battery*>(user_data);
    if (battery && battery->_read_task_handle)
    {
        vTaskNotifyGiveFromISR(battery->_read_task_handle, &mustYield);
    }
    return (mustYield == pdTRUE);
}

void Battery::_read_task(void* arg)
{
    Battery* battery = static_cast<Battery*>(arg);
    esp_err_t ret;
    uint32_t num_parsed_samples = 0;
    adc_continuous_data_t parsed_data[NUM_SAMPLES];

    while (battery->_running)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (1)
        {
            ret = adc_continuous_read_parse(battery->_adc_handle, parsed_data, NUM_SAMPLES, &num_parsed_samples, 0);
            if (ret == ESP_OK)
            {
                uint32_t sum = 0;
                uint32_t valid_count = 0;

                for (int i = 0; i < num_parsed_samples; i++)
                {
                    if (parsed_data[i].valid)
                    {
                        ESP_LOGD(TAG, "ADC%d, Channel: %d, Value: %" PRIu32, parsed_data[i].unit + 1, parsed_data[i].channel, parsed_data[i].raw_data);
                        sum += parsed_data[i].raw_data;
                        valid_count++;
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Invalid data [ADC%d_Ch%d_%" PRIu32 "]", parsed_data[i].unit + 1, parsed_data[i].channel, parsed_data[i].raw_data);
                    }
                }

                if (valid_count > 0)
                {
                    battery->_adc_raw = sum / valid_count;
                    ESP_LOGD(TAG, "Average ADC value: %" PRIu32 " (from %d valid samples)", battery->_adc_raw, valid_count);
                }
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                ESP_LOGD(TAG, "ADC read timeout - no more data available");
                break;
            }
            else
            {
                ESP_LOGE(TAG, "ADC continuous read error: 0x%x", (int)ret);
                break;
            }
            vTaskDelay(1);
        }
    }
    ESP_LOGI(TAG, "Battery ADC read task finished");
    vTaskDelete(NULL);
}

void Battery::init()
{
    esp_err_t ret;

    //-------------ADC Continuous Init---------------//
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = NUM_SAMPLES * SOC_ADC_DIGI_RESULT_BYTES * 2,
        .conv_frame_size = NUM_SAMPLES * SOC_ADC_DIGI_RESULT_BYTES,
        .flags = {},
    };

    ret = adc_continuous_new_handle(&adc_config, &_adc_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_continuous_new_handle failed, ret: %d", ret);
        return;
    }

    //-------------ADC Continuous Config---------------//
    adc_digi_pattern_config_t adc_pattern[1] = {{
        .atten = ADC_ATTEN,
        .channel = ADC_CHANNEL & 0x7,
        .unit = ADC_UNIT,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    }};

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    ret = adc_continuous_config(_adc_handle, &dig_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_continuous_config failed, ret: %d", ret);
        return;
    }

    //-------------Register Callback---------------//
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = _adc_conv_done_callback,
        .on_pool_ovf = nullptr,
    };
    ret = adc_continuous_register_event_callbacks(_adc_handle, &cbs, this);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_continuous_register_event_callbacks failed, ret: %d", ret);
        return;
    }

    //-------------Start ADC Continuous Mode---------------//
    _running = true;
    ret = adc_continuous_start(_adc_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_continuous_start failed, ret: %d", ret);
        _running = false;
        return;
    }

    //-------------Create Read Task---------------//
    xTaskCreate(_read_task, "battery_adc", 1024 * 3, this, 2, &_read_task_handle);

    ESP_LOGI(TAG, "Battery ADC continuous mode initialized");
}

void Battery::deinit()
{
    if (_adc_handle != nullptr)
    {
        esp_err_t ret;

        // Stop continuous mode
        _running = false;
        ret = adc_continuous_stop(_adc_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "adc_continuous_stop failed, ret: %d", ret);
        }

        // Wait for task to finish
        if (_read_task_handle != nullptr)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            _read_task_handle = nullptr;
        }

        // Delete handle
        ret = adc_continuous_deinit(_adc_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "adc_continuous_deinit failed, ret: %d", ret);
        }
        _adc_handle = nullptr;
    }

    ESP_LOGI(TAG, "Battery ADC deinitialized");
}

float Battery::get_voltage()
{
    if (!_running || _adc_raw == 0)
    {
        ESP_LOGW(TAG, "ADC not running or no data available");
        return 0.0f;
    }

    _voltage = static_cast<float>(_adc_raw / 4095.0f) * 3.3f * MEASUREMENT_OFFSET;
    ESP_LOGI(TAG, "%.3f V", _voltage);
    return _voltage;
}

uint8_t Battery::get_level()
{
    float voltage = get_voltage();
    return get_level(voltage);
}

uint8_t Battery::get_level(float voltage)
{
    // LiPo 1S discharge curve: voltage to percentage lookup table
    // Based on typical LiPo discharge characteristics
    static const struct
    {
        float voltage;
        uint8_t percentage;
    } lipo_curve[] = {
        {4.20f, 100},
        {4.15f, 95},
        {4.11f, 90},
        {4.08f, 85},
        {4.02f, 80},
        {3.98f, 75},
        {3.95f, 70},
        {3.91f, 65},
        {3.87f, 60},
        {3.85f, 55},
        {3.84f, 50},
        {3.82f, 45},
        {3.80f, 40},
        {3.79f, 35},
        {3.77f, 30},
        {3.75f, 25},
        {3.73f, 20},
        {3.71f, 15},
        {3.69f, 10},
        {3.61f, 5},
        {3.27f, 0}};

    const int table_size = sizeof(lipo_curve) / sizeof(lipo_curve[0]);

    // Handle out of range cases
    if (voltage >= lipo_curve[0].voltage)
    {
        return 100; // Fully charged or overcharged
    }
    if (voltage <= lipo_curve[table_size - 1].voltage)
    {
        return 0; // Empty or below cutoff
    }

    // Linear interpolation between table points
    for (int i = 0; i < table_size - 1; i++)
    {
        if (voltage >= lipo_curve[i + 1].voltage)
        {
            // Found the right segment, interpolate
            float v_high = lipo_curve[i].voltage;
            float v_low = lipo_curve[i + 1].voltage;
            uint8_t p_high = lipo_curve[i].percentage;
            uint8_t p_low = lipo_curve[i + 1].percentage;

            // Linear interpolation formula: p = p_low + (voltage - v_low) * (p_high - p_low) / (v_high - v_low)
            float percentage = p_low + (voltage - v_low) * (p_high - p_low) / (v_high - v_low);

            // Clamp to valid range and round
            if (percentage > 100.0f)
                percentage = 100.0f;
            if (percentage < 0.0f)
                percentage = 0.0f;

            return static_cast<uint8_t>(percentage + 0.5f); // Round to nearest integer
        }
    }

    // Should never reach here, but return 0 as fallback
    return 0;
}
