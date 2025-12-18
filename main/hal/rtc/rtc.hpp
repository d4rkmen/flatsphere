/**
 * @file rtc.hpp
 * @brief PCF85063 RTC Driver Class
 * @details I2C Real-Time Clock driver for HAL system
 * @author d4rkmen
 * @license Apache License 2.0
 */

 #pragma once

#include <stdint.h>
#include <time.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

namespace HAL
{
    // Forward declaration
    class Hal;

    /**
     * @brief PCF85063 RTC Driver Class
     */
    class RTC
    {
    private:
        // PCF85063 I2C address and registers
        static constexpr uint8_t PCF85063_ADDRESS = 0x51;
        static constexpr uint32_t PCF85063_I2C_SPEED_HZ = 400000;

        // Register addresses
        static constexpr uint8_t RTC_CTRL_1_ADDR = 0x00;
        static constexpr uint8_t RTC_CTRL_2_ADDR = 0x01;
        static constexpr uint8_t RTC_OFFSET_ADDR = 0x02;
        static constexpr uint8_t RTC_RAM_ADDR = 0x03;
        static constexpr uint8_t RTC_SECOND_ADDR = 0x04;
        static constexpr uint8_t RTC_MINUTE_ADDR = 0x05;
        static constexpr uint8_t RTC_HOUR_ADDR = 0x06;
        static constexpr uint8_t RTC_DAY_ADDR = 0x07;
        static constexpr uint8_t RTC_WDAY_ADDR = 0x08;
        static constexpr uint8_t RTC_MONTH_ADDR = 0x09;
        static constexpr uint8_t RTC_YEAR_ADDR = 0x0A;
        static constexpr uint8_t RTC_SECOND_ALARM = 0x0B;
        static constexpr uint8_t RTC_MINUTE_ALARM = 0x0C;
        static constexpr uint8_t RTC_HOUR_ALARM = 0x0D;
        static constexpr uint8_t RTC_DAY_ALARM = 0x0E;
        static constexpr uint8_t RTC_WDAY_ALARM = 0x0F;
        static constexpr uint8_t RTC_TIMER_VAL = 0x10;
        static constexpr uint8_t RTC_TIMER_MODE = 0x11;

        // Control register 1 bits
        static constexpr uint8_t RTC_CTRL_1_STOP = 0x20;
        static constexpr uint8_t RTC_CTRL_1_SR = 0x10;
        static constexpr uint8_t RTC_CTRL_1_CAP_SEL = 0x01; // 0=7pF, 1=12.5pF

        // Control register 2 bits
        static constexpr uint8_t RTC_CTRL_2_AIE = 0x80; // Alarm interrupt enable
        static constexpr uint8_t RTC_CTRL_2_AF = 0x40;  // Alarm flag

        // Alarm bit
        static constexpr uint8_t RTC_ALARM = 0x80;

        // Year offset
        static constexpr uint16_t YEAR_OFFSET = 1970;

        Hal* _hal;
        i2c_master_dev_handle_t _dev_handle;
        bool _initialized;

        // Low-level I2C operations
        esp_err_t _read_registers(uint8_t reg, uint8_t* data, size_t len);
        esp_err_t _write_registers(uint8_t reg, const uint8_t* data, size_t len);

        // BCD conversion helpers
        static uint8_t _dec_to_bcd(int val);
        static int _bcd_to_dec(uint8_t val);

    public:
        /**
         * @brief Constructor
         * @param hal Pointer to HAL instance for I2C bus access
         */
        RTC(Hal* hal);
        ~RTC();

        /**
         * @brief Initialize the RTC device
         * @return ESP_OK on success
         */
        esp_err_t init();

        /**
         * @brief Deinitialize the RTC device
         * @return ESP_OK on success
         */
        esp_err_t deinit();

        /**
         * @brief Check if RTC is initialized
         * @return true if initialized, false otherwise
         */
        bool is_initialized() const { return _initialized; }

        /**
         * @brief Software reset the RTC
         * @return ESP_OK on success
         */
        esp_err_t reset();

        /**
         * @brief Set time only
         * @param time struct tm with time fields
         * @return ESP_OK on success
         */
        esp_err_t set_time(const struct tm* time);

        /**
         * @brief Set date only
         * @param date struct tm with date fields
         * @return ESP_OK on success
         */
        esp_err_t set_date(const struct tm* date);

        /**
         * @brief Set both time and date
         * @param datetime struct tm with all fields
         * @return ESP_OK on success
         */
        esp_err_t set_datetime(const struct tm* datetime);

        /**
         * @brief Read current time and date
         * @param datetime Pointer to struct tm to fill
         * @return ESP_OK on success
         */
        esp_err_t read_datetime(struct tm* datetime);

        /**
         * @brief Enable alarm interrupt
         * @return ESP_OK on success
         */
        esp_err_t enable_alarm();

        /**
         * @brief Disable alarm interrupt
         * @return ESP_OK on success
         */
        esp_err_t disable_alarm();

        /**
         * @brief Get alarm flag status
         * @param flag Pointer to store alarm flag status
         * @return ESP_OK on success
         */
        esp_err_t get_alarm_flag(bool* flag);

        /**
         * @brief Clear alarm flag
         * @return ESP_OK on success
         */
        esp_err_t clear_alarm_flag();

        /**
         * @brief Set alarm time
         * @param time struct tm with alarm time
         * @return ESP_OK on success
         */
        esp_err_t set_alarm(const struct tm* time);

        /**
         * @brief Read alarm time
         * @param time Pointer to struct tm to fill
         * @return ESP_OK on success
         */
        esp_err_t read_alarm(struct tm* time);

        /**
         * @brief Format datetime to string
         * @param buffer Buffer to write formatted string
         * @param buffer_size Size of buffer
         * @param datetime struct tm to format
         * @return Number of characters written
         */
        static int datetime_to_string(char* buffer, size_t buffer_size, const struct tm* datetime);
    };

} // namespace HAL
