/**
 * @file rtc.cpp
 * @brief PCF85063 RTC Driver Class
 * @details I2C Real-Time Clock driver for HAL system
 * @author d4rkmen
 * @license Apache License 2.0
 */

#include "rtc.hpp"
#include "hal.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>
#include "time.h"

using namespace HAL;

static const char* TAG = "RTC";

RTC::RTC(Hal* hal) : _hal(hal), _dev_handle(nullptr), _initialized(false)
{
    init();
}

RTC::~RTC()
{
    deinit();
}

esp_err_t RTC::_read_registers(uint8_t reg, uint8_t* data, size_t len)
{
    if (!_initialized || _dev_handle == nullptr)
    {
        ESP_LOGE(TAG, "RTC not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2c_master_transmit_receive(_dev_handle, &reg, 1, data, len, 1000);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t RTC::_write_registers(uint8_t reg, const uint8_t* data, size_t len)
{
    if (!_initialized || _dev_handle == nullptr)
    {
        ESP_LOGE(TAG, "RTC not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t* write_buf = (uint8_t*)malloc(len + 1);
    if (!write_buf)
    {
        return ESP_ERR_NO_MEM;
    }

    write_buf[0] = reg;
    memcpy(write_buf + 1, data, len);

    esp_err_t ret = i2c_master_transmit(_dev_handle, write_buf, len + 1, 1000);
    free(write_buf);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

uint8_t RTC::_dec_to_bcd(int val)
{
    return (uint8_t)((val / 10 * 16) + (val % 10));
}

int RTC::_bcd_to_dec(uint8_t val)
{
    return (int)((val / 16 * 10) + (val % 16));
}

esp_err_t RTC::init()
{
    if (_initialized)
    {
        ESP_LOGW(TAG, "RTC already initialized");
        return ESP_OK;
    }

    if (_hal == nullptr || _hal->i2c() == nullptr)
    {
        ESP_LOGE(TAG, "HAL or I2C not available");
        return ESP_ERR_INVALID_ARG;
    }

    // Get I2C bus handle
    i2c_master_bus_handle_t bus_handle = _hal->i2c()->get_bus_handle();
    if (bus_handle == nullptr)
    {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Configure PCF85063 device
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = PCF85063_ADDRESS;
    dev_cfg.scl_speed_hz = PCF85063_I2C_SPEED_HZ;

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add PCF85063 device: %s", esp_err_to_name(ret));
        return ret;
    }

    _initialized = true;

    // Configure RTC: Normal mode, RTC run, no reset, 24hr format, 12.5pF capacitance
    uint8_t ctrl1_val = RTC_CTRL_1_CAP_SEL;
    ret = _write_registers(RTC_CTRL_1_ADDR, &ctrl1_val, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure RTC");
        deinit();
        return ret;
    }

    ESP_LOGI(TAG, "RTC initialized");
    return ESP_OK;
}

esp_err_t RTC::deinit()
{
    if (!_initialized)
    {
        return ESP_OK;
    }

    if (_dev_handle != nullptr)
    {
        esp_err_t ret = i2c_master_bus_rm_device(_dev_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to remove device: %s", esp_err_to_name(ret));
        }
        _dev_handle = nullptr;
    }

    _initialized = false;
    ESP_LOGI(TAG, "RTC deinitialized");
    return ESP_OK;
}

esp_err_t RTC::reset()
{
    uint8_t value = RTC_CTRL_1_CAP_SEL | RTC_CTRL_1_SR;
    return _write_registers(RTC_CTRL_1_ADDR, &value, 1);
}

esp_err_t RTC::set_time(const struct tm* time)
{
    if (time == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[3] = {
        _dec_to_bcd(time->tm_sec),
        _dec_to_bcd(time->tm_min),
        _dec_to_bcd(time->tm_hour)};
    return _write_registers(RTC_SECOND_ADDR, buf, 3);
}

static void fix_week_day(struct tm* date)
{
    if (date != nullptr)
    {
        time_t tmp = mktime((tm*)date);
        struct tm tmp_tm;
        localtime_r(&tmp, &tmp_tm);
        date->tm_wday = tmp_tm.tm_wday;
    }
}

esp_err_t RTC::set_date(const struct tm* date)
{
    if (date == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    // fix week day, this data could be incorrect
    fix_week_day((tm*)date);

    // tm_mon is 0-11, RTC expects 1-12
    // tm_year is years since 1900, RTC expects years since 1970
    uint8_t buf[4] = {
        _dec_to_bcd(date->tm_mday),
        _dec_to_bcd(date->tm_wday),
        _dec_to_bcd(date->tm_mon + 1),
        _dec_to_bcd((date->tm_year + 1900) - YEAR_OFFSET)};
    return _write_registers(RTC_DAY_ADDR, buf, 4);
}

esp_err_t RTC::set_datetime(const struct tm* datetime)
{
    if (datetime == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // fix week day, this data could be incorrect
    fix_week_day((tm*)datetime);

    // tm_mon is 0-11, RTC expects 1-12
    // tm_year is years since 1900, RTC expects years since 1970
    uint8_t buf[7] = {
        _dec_to_bcd(datetime->tm_sec),
        _dec_to_bcd(datetime->tm_min),
        _dec_to_bcd(datetime->tm_hour),
        _dec_to_bcd(datetime->tm_mday),
        _dec_to_bcd(datetime->tm_wday),
        _dec_to_bcd(datetime->tm_mon + 1),
        _dec_to_bcd((datetime->tm_year + 1900) - YEAR_OFFSET)};
    return _write_registers(RTC_SECOND_ADDR, buf, 7);
}

esp_err_t RTC::read_datetime(struct tm* datetime)
{
    if (datetime == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[7] = {0};
    esp_err_t ret = _read_registers(RTC_SECOND_ADDR, buf, 7);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Clear the struct first
    memset(datetime, 0, sizeof(struct tm));

    // Fill in the fields
    datetime->tm_sec = _bcd_to_dec(buf[0] & 0x7F);
    datetime->tm_min = _bcd_to_dec(buf[1] & 0x7F);
    datetime->tm_hour = _bcd_to_dec(buf[2] & 0x3F);
    datetime->tm_mday = _bcd_to_dec(buf[3] & 0x3F);
    datetime->tm_wday = _bcd_to_dec(buf[4] & 0x07);
    datetime->tm_mon = _bcd_to_dec(buf[5] & 0x1F) - 1; // RTC uses 1-12, tm_mon is 0-11
    int year = _bcd_to_dec(buf[6]) + YEAR_OFFSET;
    datetime->tm_year = year - 1900; // tm_year is years since 1900
    datetime->tm_isdst = -1;         // Unknown DST status

    return ESP_OK;
}

esp_err_t RTC::enable_alarm()
{
    uint8_t value = RTC_CTRL_2_AIE;
    return _write_registers(RTC_CTRL_2_ADDR, &value, 1);
}

esp_err_t RTC::disable_alarm()
{
    uint8_t value = 0;
    return _write_registers(RTC_CTRL_2_ADDR, &value, 1);
}

esp_err_t RTC::get_alarm_flag(bool* flag)
{
    if (flag == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t value = 0;
    esp_err_t ret = _read_registers(RTC_CTRL_2_ADDR, &value, 1);
    if (ret != ESP_OK)
    {
        return ret;
    }

    *flag = (value & RTC_CTRL_2_AF) != 0;
    return ESP_OK;
}

esp_err_t RTC::clear_alarm_flag()
{
    uint8_t value = 0;
    esp_err_t ret = _read_registers(RTC_CTRL_2_ADDR, &value, 1);
    if (ret != ESP_OK)
    {
        return ret;
    }

    value &= ~RTC_CTRL_2_AF; // Clear alarm flag
    return _write_registers(RTC_CTRL_2_ADDR, &value, 1);
}

esp_err_t RTC::set_alarm(const struct tm* time)
{
    if (time == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[5] = {
        static_cast<uint8_t>(_dec_to_bcd(time->tm_sec) & (~RTC_ALARM)),
        static_cast<uint8_t>(_dec_to_bcd(time->tm_min) & (~RTC_ALARM)),
        static_cast<uint8_t>(_dec_to_bcd(time->tm_hour) & (~RTC_ALARM)),
        RTC_ALARM, // Disable day alarm
        RTC_ALARM  // Disable weekday alarm
    };
    return _write_registers(RTC_SECOND_ALARM, buf, 5);
}

esp_err_t RTC::read_alarm(struct tm* time)
{
    if (time == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[5] = {0};
    esp_err_t ret = _read_registers(RTC_SECOND_ALARM, buf, 5);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Clear the struct first
    memset(time, 0, sizeof(struct tm));

    time->tm_sec = _bcd_to_dec(buf[0] & 0x7F);
    time->tm_min = _bcd_to_dec(buf[1] & 0x7F);
    time->tm_hour = _bcd_to_dec(buf[2] & 0x3F);
    time->tm_mday = _bcd_to_dec(buf[3] & 0x3F);
    time->tm_wday = _bcd_to_dec(buf[4] & 0x07);
    time->tm_isdst = -1;

    return ESP_OK;
}

int RTC::datetime_to_string(char* buffer, size_t buffer_size, const struct tm* datetime)
{
    if (buffer == nullptr || buffer_size == 0 || datetime == nullptr)
    {
        return 0;
    }

    // Use standard strftime for formatting
    return strftime(buffer, buffer_size, "%Y-%m-%d (%a) %H:%M:%S", datetime);
}
