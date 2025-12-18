/**
 * @file exio.cpp
 * @brief TCA9554PWR GPIO Expander Driver Class
 * @details 8-bit I2C I/O expander driver for HAL system
 * @author d4rkmen
 * @license Apache License 2.0
 */

#include "exio.hpp"
#include "hal.h"
#include "esp_log.h"
#include "esp_check.h"
#include "common_define.h"

using namespace HAL;

static const char* TAG = "EXIO";

EXIO::EXIO(Hal* hal) : _hal(hal), _dev_handle(nullptr), _bus_handle(nullptr), _initialized(false)
{
    init(0x00);
}

EXIO::~EXIO()
{
    deinit();
}
esp_err_t EXIO::_validate_pin(uint8_t pin)
{
    return (pin < 1 || pin > 8) ? ESP_ERR_INVALID_ARG : ESP_OK;
}

esp_err_t EXIO::_read_register(uint8_t reg, uint8_t* value)
{
    if (!_initialized || _dev_handle == nullptr)
    {
        ESP_LOGE(TAG, "not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    // wait all done
    esp_err_t err;
    err = _hal->i2c()->wait_all_done(TCA9554_WAIT_TIMEOUT_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to wait all done: %s", esp_err_to_name(err));
        return err;
    }

    int retries = TCA9554_RETRY_COUNT;
    do
    {
        // Write register address, then read the value
        err = i2c_master_transmit_receive(_dev_handle, &reg, 1, value, 1, TCA9554_TRANS_TIMEOUT_MS);
        if (err == ESP_OK)
        {
            // save value on success
            switch (reg)
            {
            case TCA9554_INPUT_REG:
                _regs.input = *value;
                break;
            case TCA9554_POLARITY_REG:
                _regs.polarity = *value;
                break;
            case TCA9554_CONFIG_REG:
                _regs.config = *value;
                break;
            case TCA9554_OUTPUT_REG:
                _regs.output = *value;
                break;
            }
            break;
        }
        retries--;
        delay(10);
    } while (retries > 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read register 0x%02X (0x%02X)", reg, (int)err);
    }
    return err;
}

esp_err_t EXIO::_write_register(uint8_t reg, uint8_t value)
{
    if (!_initialized || _dev_handle == nullptr)
    {
        ESP_LOGE(TAG, "not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t write_buf[2] = {reg, value};
    esp_err_t err;
    err = _hal->i2c()->wait_all_done(TCA9554_WAIT_TIMEOUT_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to wait all done (0x%02X)", (int)err);
        return err;
    }

    int retries = TCA9554_RETRY_COUNT;
    do
    {
        err = i2c_master_transmit(_dev_handle, write_buf, 2, TCA9554_TRANS_TIMEOUT_MS);
        if (err == ESP_OK)
        {
            // save value on success
            switch (reg)
            {
            case TCA9554_INPUT_REG:
                _regs.input = value;
                break;
            case TCA9554_POLARITY_REG:
                _regs.polarity = value;
                break;
            case TCA9554_CONFIG_REG:
                _regs.config = value;
                break;
            case TCA9554_OUTPUT_REG:
                _regs.output = value;
                break;
            }
            break;
        }
        delay(10);
        retries--;
    } while (retries > 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write register 0x%02X (0x%02X)", reg, (int)err);
    }
    return err;
}

esp_err_t EXIO::init(uint8_t pin_mode_state)
{
    if (_initialized)
    {
        ESP_LOGW(TAG, "already initialized");
        return ESP_OK;
    }

    if (_hal == nullptr || _hal->i2c() == nullptr)
    {
        ESP_LOGE(TAG, "HAL or I2C not available");
        return ESP_ERR_INVALID_ARG;
    }

    // Configure TCA9554 device
    esp_err_t ret = _hal->i2c()->add_device(TCA9554_ADDRESS, 0, nullptr, &_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add TCA9554 device: %s", esp_err_to_name(ret));
        return ret;
    }

    _initialized = true;
    ESP_LOGI(TAG, "I2C device added, address: 0x%02X", TCA9554_ADDRESS);
    // Set initial pin mode configuration
    ret = set_pins_mode(pin_mode_state);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set initial pin direction");
        deinit();
        return ret;
    }
    ESP_LOGI(TAG, "initialized (pin direction: 0x%02X)", pin_mode_state);
    ret = write_pins(0xFF);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write initial pins");
        deinit();
        return ret;
    }

    ESP_LOGI(TAG, "pins written (0x%02X)", 0x00);
    return ESP_OK;
}

esp_err_t EXIO::deinit()
{
    if (!_initialized)
    {
        return ESP_OK;
    }

    if (_dev_handle != nullptr)
    {
        esp_err_t ret = _hal->i2c()->remove_device(_dev_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to remove device: %s", esp_err_to_name(ret));
        }
        _dev_handle = nullptr;
    }

    _initialized = false;
    ESP_LOGI(TAG, "deinitialized");
    return ESP_OK;
}

esp_err_t EXIO::set_pin_mode(uint8_t pin, EXIOMode mode)
{
    ESP_RETURN_ON_ERROR(_validate_pin(pin), TAG, "Invalid pin number");

    uint8_t current_config = 0;
    esp_err_t ret = _read_register(TCA9554_CONFIG_REG, &current_config);
    if (ret != ESP_OK)
    {
        return ret;
    }

    uint8_t new_config;
    if (mode == EXIOMode::INPUT)
    {
        new_config = current_config | (0x01 << (pin - 1));
    }
    else
    {
        new_config = current_config & ~(0x01 << (pin - 1));
    }

    return _write_register(TCA9554_CONFIG_REG, new_config);
}

esp_err_t EXIO::set_pins_mode(uint8_t pin_state)
{
    return _write_register(TCA9554_CONFIG_REG, pin_state);
}

esp_err_t EXIO::read_pin(uint8_t pin, uint8_t* state)
{
    ESP_RETURN_ON_ERROR(_validate_pin(pin), TAG, "Invalid pin number");

    if (state == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t input_bits = 0;
    esp_err_t ret = _read_register(TCA9554_INPUT_REG, &input_bits);
    if (ret != ESP_OK)
    {
        return ret;
    }

    *state = (input_bits >> (pin - 1)) & 0x01;
    return ESP_OK;
}

esp_err_t EXIO::read_pins(uint8_t* states)
{
    if (states == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return _read_register(TCA9554_INPUT_REG, states);
}

esp_err_t EXIO::write_pin(uint8_t pin, bool state)
{
    ESP_RETURN_ON_ERROR(_validate_pin(pin), TAG, "Invalid pin number");

    uint8_t new_output;
    if (state)
    {
        new_output = _regs.output | (0x01 << (pin - 1));
    }
    else
    {
        new_output = _regs.output & ~(0x01 << (pin - 1));
    }

    ESP_RETURN_ON_ERROR(_write_register(TCA9554_OUTPUT_REG, new_output), TAG, "Failed to write pin");
    return ESP_OK;
}

esp_err_t EXIO::write_pins(uint8_t pin_state)
{
    ESP_RETURN_ON_ERROR(_write_register(TCA9554_OUTPUT_REG, pin_state), TAG, "Failed to write pins");
    return ESP_OK;
}

esp_err_t EXIO::toggle_pin(uint8_t pin)
{
    ESP_RETURN_ON_ERROR(_validate_pin(pin), TAG, "Invalid pin number");

    uint8_t new_output = _regs.output ^ (0x01 << (pin - 1));
    ESP_RETURN_ON_ERROR(write_pin(pin, new_output), TAG, "Failed to toggle pin");
    _regs.output = new_output;
    return ESP_OK;
}

esp_err_t EXIO::set_polarity(uint8_t polarity)
{
    ESP_RETURN_ON_ERROR(_write_register(TCA9554_POLARITY_REG, polarity), TAG, "Failed to set polarity");
    return ESP_OK;
}
