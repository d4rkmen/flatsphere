/**
 * @file Battery.hpp
 * @brief Battery management class for ADC-based voltage and level reading
 * @author d4rkmen
 * @license Apache License 2.0
 */
#pragma once

#include <stdint.h>
#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace HAL
{
    class Battery
    {
    private:
        adc_continuous_handle_t _adc_handle;
        int _adc_raw;
        int _voltage_mv;
        float _voltage = 0.0f;
        TaskHandle_t _read_task_handle;
        bool _running;

        static void _read_task(void* arg);
        static bool IRAM_ATTR _adc_conv_done_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t* edata, void* user_data);

    public:
        Battery();
        ~Battery();

        constexpr static const adc_unit_t ADC_UNIT = ADC_UNIT_1;
        constexpr static const adc_channel_t ADC_CHANNEL = ADC_CHANNEL_7;
        constexpr static const adc_atten_t ADC_ATTEN = ADC_ATTEN_DB_12;
        constexpr static const float MEASUREMENT_OFFSET = 3.191f;
        constexpr static const uint32_t NUM_SAMPLES = 64;
        /**
         * @brief Initialize the battery ADC
         */
        void
        init();

        /**
         * @brief Deinitialize the battery ADC
         */
        void deinit();

        /**
         * @brief Get battery voltage in volts
         * @return Battery voltage in volts
         */
        float get_voltage();

        /**
         * @brief Get battery level percentage
         * @return Battery level (0, 25, 50, 75, or 100)
         */
        uint8_t get_level();

        /**
         * @brief Get battery level based on voltage
         * @param voltage Voltage in volts
         * @return Battery level (0, 25, 50, 75, or 100)
         */
        uint8_t get_level(float voltage);
    };
} // namespace BAT
