/**
 * @file display_panel.cpp
 * @brief ST77916 Panel Low-Level Methods
 * @author d4rkmen
 * @copyright Espressif Systems (Shanghai) CO LTD
 * @license Apache License 2.0
 */

#include "display.hpp"
#include "display_panel.hpp"
#include "driver/gpio.h"
#include "hal.h"
#include "esp_log.h"
#include "esp_check.h"
#include "common_define.h"

static const char* TAG = "Display";

#define SEND_FAILED_MESSAGE "send command failed"

// ============================================================================
// Command Transmission Helpers
// ============================================================================

esp_err_t Display::tx_param(int lcd_cmd, const void* param, size_t param_size)
{
    lcd_cmd &= 0xff;
    lcd_cmd <<= 8;
    lcd_cmd |= LCD_OPCODE_WRITE_CMD << 24;
    uint32_t value = 0;
    memcpy(&value, param, param_size);
    // ESP_LOGD(TAG, "tx_param: 0x%08X, param: 0x%08X", lcd_cmd, value);
    return esp_lcd_panel_io_tx_param(_io_handle_write, lcd_cmd, param, param_size);
}

// ! SPI bus should be on read frequency
esp_err_t Display::rx_param(int lcd_cmd, void* param, size_t param_size)
{
    lcd_cmd &= 0xff;
    lcd_cmd <<= 8;
    lcd_cmd |= LCD_OPCODE_READ_CMD << 24;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(_io_handle_read, lcd_cmd, param, param_size), TAG, "rx_param failed");

    uint32_t value = 0;
    memcpy(&value, param, param_size > sizeof(value) ? sizeof(value) : param_size);
    // ESP_LOGD(TAG, "rx_param: 0x%08X, param: 0x%08X", lcd_cmd, value);
    return ESP_OK;
}

esp_err_t Display::tx_color(int lcd_cmd, const void* param, size_t param_size)
{
    lcd_cmd &= 0xff;
    lcd_cmd <<= 8;
    lcd_cmd |= LCD_OPCODE_WRITE_COLOR << 24;
    // ESP_LOGD(TAG, "tx_color: 0x%08X", lcd_cmd);
    return esp_lcd_panel_io_tx_color(_io_handle_write, lcd_cmd, param, param_size);
}

// ============================================================================
// ST77916 Panel Low-Level Methods
// ============================================================================

esp_err_t Display::panel_reset()
{
    // Perform hardware reset if GPIO configured
    if (_reset_gpio_num >= 0)
    {
        ESP_LOGD(TAG, "panel_reset: hardware reset by GPIO");
        gpio_set_level((gpio_num_t)_reset_gpio_num, 1);
        delay(10);
        gpio_set_level((gpio_num_t)_reset_gpio_num, 0);
        delay(120);
    }
    else if (_hal->exio()->is_initialized() && DisplayConfig::EXIO::LCD_RST > 0)
    {
        ESP_LOGD(TAG, "panel_reset: hardware reset by EXIO");
        _hal->exio()->write_pin(DisplayConfig::EXIO::LCD_RST, false);
        delay(10);
        _hal->exio()->write_pin(DisplayConfig::EXIO::LCD_RST, true);
        delay(120);
    }
    else if (_io_handle_write)
    {
        ESP_LOGD(TAG, "panel_reset: software reset");
        // Perform software reset
        uint8_t cmd = CMD_SWRESET;
        ESP_RETURN_ON_ERROR(tx_param(cmd, NULL, 0), TAG, SEND_FAILED_MESSAGE);
        delay(120);
    }

    return ESP_OK;
}

esp_err_t Display::panel_init()
{
    // Send MADCTL and COLMOD commands
    uint8_t madctl_data[] = {_madctl_val};
    ESP_RETURN_ON_ERROR(tx_param(CMD_MADCTL, madctl_data, 1), TAG, SEND_FAILED_MESSAGE);

    uint8_t colmod_data[] = {_colmod_val};
    ESP_RETURN_ON_ERROR(tx_param(CMD_COLMOD, colmod_data, 1), TAG, SEND_FAILED_MESSAGE);

    // Send vendor-specific init commands
    const lcd_init_cmd_t* init_cmds = st77916_vendor_init_cmds;
    uint16_t init_cmds_size = sizeof(st77916_vendor_init_cmds) / sizeof(lcd_init_cmd_t);

    for (int i = 0; i < init_cmds_size; i++)
    {
        // Check for command overwrites
        if ((init_cmds[i].data_bytes > 0))
        {
            if (init_cmds[i].cmd == CMD_MADCTL)
            {
                _madctl_val = ((uint8_t*)init_cmds[i].data)[0];
                ESP_LOGW(TAG, "MADCTL overwritten by init sequence");
            }
            else if (init_cmds[i].cmd == CMD_COLMOD)
            {
                _colmod_val = ((uint8_t*)init_cmds[i].data)[0];
                ESP_LOGW(TAG, "COLMOD overwritten by init sequence");
            }
        }

        // Send command
        ESP_RETURN_ON_ERROR(tx_param(init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes),
                            TAG,
                            SEND_FAILED_MESSAGE);

        // Delay if specified
        if (init_cmds[i].delay_ms > 0)
        {
            delay(init_cmds[i].delay_ms);
        }
    }
    return ESP_OK;
}

esp_err_t Display::panel_draw_bitmap(int x_start, int y_start, int x_end, int y_end, const void* color_data)
{
    x_start += _x_gap;
    x_end += _x_gap;
    y_start += _y_gap;
    y_end += _y_gap;

    // Set column address
    uint8_t caset_data[] = {
        (uint8_t)((x_start >> 8) & 0xFF),
        (uint8_t)(x_start & 0xFF),
        (uint8_t)(((x_end - 1) >> 8) & 0xFF),
        (uint8_t)((x_end - 1) & 0xFF),
    };
    ESP_RETURN_ON_ERROR(tx_param(CMD_CASET, caset_data, 4), TAG, SEND_FAILED_MESSAGE);

    // Set row address
    uint8_t raset_data[] = {
        (uint8_t)((y_start >> 8) & 0xFF),
        (uint8_t)(y_start & 0xFF),
        (uint8_t)(((y_end - 1) >> 8) & 0xFF),
        (uint8_t)((y_end - 1) & 0xFF),
    };
    ESP_RETURN_ON_ERROR(tx_param(CMD_RASET, raset_data, 4), TAG, SEND_FAILED_MESSAGE);

    // Transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * _fb_bits_per_pixel / 8;
    ESP_RETURN_ON_ERROR(tx_color(CMD_RAMWR, color_data, len), TAG, SEND_FAILED_MESSAGE);
    // avoid logging here
    // ESP_LOGD(TAG, "panel_draw_bitmap: x_start: %d, y_start: %d, x_end: %d, y_end: %d, len: %ld", x_start, y_start, x_end, y_end, (long)len);
    return ESP_OK;
}

esp_err_t Display::panel_invert_color(bool invert_color_data)
{
    int command = invert_color_data ? CMD_INVON : CMD_INVOFF;
    ESP_RETURN_ON_ERROR(tx_param(command, NULL, 0), TAG, SEND_FAILED_MESSAGE);
    ESP_LOGD(TAG, "panel_invert_color: invert=%d", invert_color_data);
    return ESP_OK;
}

esp_err_t Display::panel_mirror(bool mirror_x, bool mirror_y)
{
    if (mirror_x)
    {
        _madctl_val |= MAD_MX;
    }
    else
    {
        _madctl_val &= ~MAD_MX;
    }

    if (mirror_y)
    {
        _madctl_val |= MAD_MY;
    }
    else
    {
        _madctl_val &= ~MAD_MY;
    }

    uint8_t madctl_data[] = {_madctl_val};
    ESP_RETURN_ON_ERROR(tx_param(CMD_MADCTL, madctl_data, 1), TAG, SEND_FAILED_MESSAGE);

    return ESP_OK;
}

esp_err_t Display::panel_swap_xy(bool swap_axes)
{
    if (swap_axes)
    {
        _madctl_val |= MAD_MV;
    }
    else
    {
        _madctl_val &= ~MAD_MV;
    }

    uint8_t madctl_data[] = {_madctl_val};
    ESP_RETURN_ON_ERROR(tx_param(CMD_MADCTL, madctl_data, 1), TAG, SEND_FAILED_MESSAGE);

    return ESP_OK;
}

esp_err_t Display::panel_set_gap(int x_gap, int y_gap)
{
    _x_gap = x_gap;
    _y_gap = y_gap;
    return ESP_OK;
}

esp_err_t Display::panel_on_off(bool on_off)
{
    ESP_LOGD(TAG, "panel_on_off: on_off=%s", on_off ? "true" : "false");
    int command = on_off ? CMD_DISPON : CMD_DISPOFF;
    ESP_RETURN_ON_ERROR(tx_param(command, NULL, 0), TAG, SEND_FAILED_MESSAGE);
    return ESP_OK;
}

esp_err_t Display::panel_idle(bool on_off)
{
    int command = on_off ? CMD_IDMON : CMD_IDMOFF;
    ESP_RETURN_ON_ERROR(tx_param(command, NULL, 0), TAG, SEND_FAILED_MESSAGE);
    return ESP_OK;
}