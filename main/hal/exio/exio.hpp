/**
 * @file exio.hpp
 * @brief TCA9554PWR GPIO Expander Driver Class
 * @details 8-bit I2C I/O expander driver for HAL system
 * @author d4rkmen
 * @license Apache License 2.0
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

namespace HAL
{
    // Forward declaration
    class Hal;

    /**
     * @brief TCA9554PWR GPIO Expander Pin Definitions
     */
    enum class EXIOPin : uint8_t
    {
        EXIO1 = 0x01, // Touch reset
        EXIO2 = 0x02, // LCD reset
        EXIO3 = 0x03, // SD D3
        EXIO4 = 0x04, // RTC interrupt
        EXIO5 = 0x05, // J7 header pin 28
        EXIO6 = 0x06, // J7 header pin 26
        EXIO7 = 0x07, // J7 header pin 24
        EXIO8 = 0x08  // J7 header pin 22
    };

    /**
     * @brief Pin mode configuration
     */
    enum class EXIOMode : uint8_t
    {
        OUTPUT = 0,
        INPUT = 1
    };

    /**
     * @brief TCA9554PWR GPIO Expander Driver Class
     */
    class EXIO
    {
    private:
        // TCA9554PWR Register addresses
        static constexpr uint32_t TCA9554_I2C_SPEED_HZ = 400000;
        static constexpr uint8_t TCA9554_ADDRESS = 0x20;
        static constexpr uint8_t TCA9554_INPUT_REG = 0x00;
        static constexpr uint8_t TCA9554_OUTPUT_REG = 0x01;
        static constexpr uint8_t TCA9554_POLARITY_REG = 0x02;
        static constexpr uint8_t TCA9554_CONFIG_REG = 0x03;
        static constexpr int TCA9554_TRANS_TIMEOUT_MS = 1000;
        static constexpr int TCA9554_WAIT_TIMEOUT_MS = 1000;
        static constexpr uint32_t TCA9554_RETRY_COUNT = 5;

        Hal* _hal;
        i2c_master_dev_handle_t _dev_handle;
        i2c_master_bus_handle_t _bus_handle;
        bool _initialized;

        struct
        {
            uint8_t input;
            uint8_t output;
            uint8_t polarity;
            uint8_t config;
        } _regs;

        // Low-level I2C operations
        esp_err_t _validate_pin(uint8_t pin);
        esp_err_t _read_register(uint8_t reg, uint8_t* value);
        esp_err_t _write_register(uint8_t reg, uint8_t value);

    public:
        /**
         * @brief Constructor
         * @param hal Pointer to HAL instance for I2C bus access
         */
        EXIO(Hal* hal);
        ~EXIO();

        /**
         * @brief Initialize the EXIO device
         * @param pin_mode_state Initial pin mode configuration (0=output, 1=input)
         * @return ESP_OK on success
         */
        esp_err_t init(uint8_t pin_mode_state = 0x00);

        /**
         * @brief Deinitialize the EXIO device
         * @return ESP_OK on success
         */
        esp_err_t deinit();

        /**
         * @brief Check if EXIO is initialized
         * @return true if initialized, false otherwise
         */
        bool is_initialized() const { return _initialized; }

        /**
         * @brief Set mode for a single pin
         * @param pin Pin number (1-8)
         * @param mode Output or Input mode
         * @return ESP_OK on success
         */
        esp_err_t set_pin_mode(uint8_t pin, EXIOMode mode);

        /**
         * @brief Set mode for all pins at once
         * @param pin_state Bitmask for all pins (0=output, 1=input)
         * @return ESP_OK on success
         */
        esp_err_t set_pins_mode(uint8_t pin_state);

        /**
         * @brief Read a single pin level
         * @param pin Pin number (1-8)
         * @param state Pointer to store the pin state (0 or 1)
         * @return ESP_OK on success
         */
        esp_err_t read_pin(uint8_t pin, uint8_t* state);

        /**
         * @brief Read all pins at once
         * @param states Pointer to store all pin states
         * @return ESP_OK on success
         */
        esp_err_t read_pins(uint8_t* states);

        /**
         * @brief Set a single pin output level
         * @param pin Pin number (1-8)
         * @param state Output state (true=high, false=low)
         * @return ESP_OK on success
         */
        esp_err_t write_pin(uint8_t pin, bool state);

        /**
         * @brief Set all pins output levels at once
         * @param pin_state Bitmask for all pins
         * @return ESP_OK on success
         */
        esp_err_t write_pins(uint8_t pin_state);

        /**
         * @brief Toggle a single pin output level
         * @param pin Pin number (1-8)
         * @return ESP_OK on success
         */
        esp_err_t toggle_pin(uint8_t pin);

        /**
         * @brief Set input polarity for all pins at once
         * @param polarity Bitmask for all pins (0=normal, 1=inverted)
         * @return ESP_OK on success
         */
        esp_err_t set_polarity(uint8_t polarity);
    };

} // namespace HAL
