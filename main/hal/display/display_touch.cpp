/**
 * @file display_touch.cpp
 * @brief CST816 Touch Controller
 * @author d4rkmen
 * @copyright Espressif Systems (Shanghai) CO LTD
 * @license Apache License 2.0
 */

#include "display_touch.hpp"
#include "display.hpp"
#include "hal.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include <cstring>
#include <cstdlib>
#include "common_define.h"

static const char* TAG = "CST816";

// ============================================================================
// CST816 Touch Data Structure
// ============================================================================
typedef struct
{
    uint8_t num;
    uint8_t x_h : 4;
    uint8_t : 4;
    uint8_t x_l;
    uint8_t y_h : 4;
    uint8_t : 4;
    uint8_t y_l;
} __attribute__((packed)) cst816_data_t;

// ============================================================================
// Forward Declarations
// ============================================================================
static esp_err_t cst816_read_data(esp_lcd_touch_handle_t tp);
static bool cst816_get_xy(esp_lcd_touch_handle_t tp, uint16_t* x, uint16_t* y, uint16_t* strength, uint8_t* point_num, uint8_t max_point_num);
static esp_err_t cst816_del(esp_lcd_touch_handle_t tp);
static esp_err_t cst816_reset(esp_lcd_touch_handle_t tp);
static esp_err_t cst816_read_id(esp_lcd_touch_handle_t tp);
static void cst816_auto_sleep(esp_lcd_touch_handle_t tp, bool enable);
static esp_err_t i2c_read_bytes(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t* data, uint8_t len);
static esp_err_t i2c_write_bytes(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t* data, uint8_t len);

// ============================================================================
// Public API - Create CST816 Touch Controller
// ============================================================================

esp_err_t esp_lcd_touch_new_i2c_cst816s(const esp_lcd_panel_io_handle_t io,
                                        const esp_lcd_touch_config_t* config,
                                        Display* display,
                                        esp_lcd_touch_handle_t* tp)
{
    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_ARG, TAG, "Invalid io");
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Invalid config");
    ESP_RETURN_ON_FALSE(display, ESP_ERR_INVALID_ARG, TAG, "Invalid display");
    ESP_RETURN_ON_FALSE(tp, ESP_ERR_INVALID_ARG, TAG, "Invalid touch handle");

    esp_err_t ret = ESP_OK;
    esp_lcd_touch_handle_t cst816 = (esp_lcd_touch_handle_t)calloc(1, sizeof(esp_lcd_touch_t));
    ESP_GOTO_ON_FALSE(cst816, ESP_ERR_NO_MEM, err, TAG, "Touch handle malloc failed");

    // Set up callbacks
    cst816->io = io;
    cst816->display = display;
    cst816->read_data = cst816_read_data;
    cst816->get_xy = cst816_get_xy;
    cst816->del = cst816_del;
    cst816->set_swap_xy = nullptr;
    cst816->get_swap_xy = nullptr;
    cst816->set_mirror_x = nullptr;
    cst816->get_mirror_x = nullptr;
    cst816->set_mirror_y = nullptr;
    cst816->get_mirror_y = nullptr;
    cst816->enter_sleep = nullptr;
    cst816->exit_sleep = nullptr;

    // Initialize mutex
    cst816->data.lock.owner = portMUX_FREE_VAL;

    // Save config
    memcpy(&cst816->config, config, sizeof(esp_lcd_touch_config_t));

    // Configure interrupt pin if specified
    if (cst816->config.int_gpio_num != GPIO_NUM_NC)
    {
        gpio_config_t int_gpio_config = {
            .pin_bit_mask = BIT64(cst816->config.int_gpio_num),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = (cst816->config.levels.interrupt ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE),
        };
        ESP_GOTO_ON_ERROR(gpio_config(&int_gpio_config), err, TAG, "GPIO intr config failed");

        // Register interrupt callback if provided
        if (cst816->config.interrupt_callback)
        {
            esp_lcd_touch_register_interrupt_callback(cst816, cst816->config.interrupt_callback);
        }
    }

    // Reset controller
    ESP_GOTO_ON_ERROR(cst816_reset(cst816), err, TAG, "Reset failed");

    // Read chip ID
    ESP_GOTO_ON_ERROR(cst816_read_id(cst816), err, TAG, "Read ID failed");

    // Disable auto-sleep
    cst816_auto_sleep(cst816, false);

    *tp = cst816;
    ESP_LOGD(TAG, "initialized successfully");
    return ESP_OK;

err:
    if (cst816)
    {
        cst816_del(cst816);
    }
    ESP_LOGE(TAG, "initialization failed!");
    return ret;
}

// ============================================================================
// CST816 Internal Functions
// ============================================================================

static esp_err_t cst816_read_data(esp_lcd_touch_handle_t tp)
{
    cst816_data_t point;

    ESP_RETURN_ON_ERROR(i2c_read_bytes(tp, CST816_REG_DATA_START, (uint8_t*)&point, sizeof(cst816_data_t)),
                        TAG,
                        "I2C read failed");

    portENTER_CRITICAL(&tp->data.lock);

    // Limit to max supported points
    point.num = (point.num > CST816_MAX_POINTS ? CST816_MAX_POINTS : point.num);
    tp->data.points = point.num;

    // Fill coordinates
    for (int i = 0; i < point.num; i++)
    {
        tp->data.coords[i].x = ((uint16_t)point.x_h << 8) | point.x_l;
        tp->data.coords[i].y = ((uint16_t)point.y_h << 8) | point.y_l;
        tp->data.coords[i].strength = 0;
    }

    portEXIT_CRITICAL(&tp->data.lock);

    return ESP_OK;
}

static bool cst816_get_xy(esp_lcd_touch_handle_t tp, uint16_t* x, uint16_t* y, uint16_t* strength, uint8_t* point_num, uint8_t max_point_num)
{
    portENTER_CRITICAL(&tp->data.lock);

    // Get number of points
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);

    // Copy coordinates
    for (size_t i = 0; i < *point_num; i++)
    {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength)
        {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    // Invalidate points
    tp->data.points = 0;

    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t cst816_del(esp_lcd_touch_handle_t tp)
{
    if (!tp)
        return ESP_OK;

    // Reset GPIO pins
    if (tp->config.int_gpio_num != GPIO_NUM_NC)
    {
        gpio_reset_pin(tp->config.int_gpio_num);
        if (tp->config.interrupt_callback)
        {
            gpio_isr_handler_remove(tp->config.int_gpio_num);
        }
    }

    // Free memory
    free(tp);

    return ESP_OK;
}

static esp_err_t cst816_reset(esp_lcd_touch_handle_t tp)
{
    // Reset via EXIO1 (Touch Reset)
    if (tp->config.rst_gpio_num != GPIO_NUM_NC)
    {
        ESP_LOGD(TAG, " hardware reset by GPIO");
        gpio_set_level(tp->config.rst_gpio_num, 0);
        delay(10);
        gpio_set_level(tp->config.rst_gpio_num, 1);
        delay(50);
    }
    else if (tp->display->hal()->exio()->is_initialized())
    {
        ESP_LOGD(TAG, "hardware reset by EXIO");
        tp->display->hal()->exio()->write_pin(DisplayConfig::EXIO::TOUCH_RST, false); // Pull reset low
        delay(10);
        tp->display->hal()->exio()->write_pin(DisplayConfig::EXIO::TOUCH_RST, true); // Pull reset high
        delay(50);
    }
    return ESP_OK;
}

static esp_err_t cst816_read_id(esp_lcd_touch_handle_t tp)
{
    uint8_t id;
    ESP_RETURN_ON_ERROR(i2c_read_bytes(tp, CST816_REG_CHIP_ID, &id, 1), TAG, "I2C read failed");
    ESP_LOGD(TAG, "Chip ID: 0x%02X", id);
    return ESP_OK;
}

static void cst816_auto_sleep(esp_lcd_touch_handle_t tp, bool enable)
{
    ESP_LOGD(TAG, "cst816_auto_sleep: %d", enable);
    uint8_t value = enable ? 0 : 1;
    i2c_write_bytes(tp, CST816_REG_AUTO_SLEEP, &value, 1);
}

static esp_err_t i2c_read_bytes(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t* data, uint8_t len)
{
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "Invalid data pointer");
    return esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
}

static esp_err_t i2c_write_bytes(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t* data, uint8_t len)
{
    ESP_RETURN_ON_FALSE(tp, ESP_ERR_INVALID_ARG, TAG, "Invalid touch handle");
    return esp_lcd_panel_io_tx_param(tp->io, reg, data, len);
}

// ============================================================================
// Touch Framework Functions
// ============================================================================

esp_err_t esp_lcd_touch_read_data(esp_lcd_touch_handle_t tp)
{
    if (!tp || !tp->read_data)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return tp->read_data(tp);
}

bool esp_lcd_touch_get_coordinates(esp_lcd_touch_handle_t tp, uint16_t* x, uint16_t* y, uint16_t* strength, uint8_t* point_num, uint8_t max_point_num)
{
    if (!tp || !x || !y || !tp->get_xy)
    {
        return false;
    }

    bool touched = tp->get_xy(tp, x, y, strength, point_num, max_point_num);
    if (!touched)
    {
        return false;
    }

    // Process coordinates by user callback if provided
    if (tp->config.process_coordinates)
    {
        tp->config.process_coordinates(tp, x, y, strength, point_num, max_point_num);
    }

    // Software coordinate adjustment
    bool sw_adj_needed = ((tp->config.flags.mirror_x && !tp->set_mirror_x) ||
                          (tp->config.flags.mirror_y && !tp->set_mirror_y) ||
                          (tp->config.flags.swap_xy && !tp->set_swap_xy));

    // Adjust all coordinates if needed
    for (int i = 0; (sw_adj_needed && i < *point_num); i++)
    {
        // Mirror X coordinates (if not supported by HW)
        if (tp->config.flags.mirror_x && !tp->set_mirror_x)
        {
            x[i] = tp->config.x_max - x[i];
        }

        // Mirror Y coordinates (if not supported by HW)
        if (tp->config.flags.mirror_y && !tp->set_mirror_y)
        {
            y[i] = tp->config.y_max - y[i];
        }

        // Swap X and Y coordinates (if not supported by HW)
        if (tp->config.flags.swap_xy && !tp->set_swap_xy)
        {
            uint16_t tmp = x[i];
            x[i] = y[i];
            y[i] = tmp;
        }
    }

    return touched;
}

esp_err_t esp_lcd_touch_del(esp_lcd_touch_handle_t tp)
{
    if (!tp || !tp->del)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return tp->del(tp);
}

esp_err_t esp_lcd_touch_register_interrupt_callback(esp_lcd_touch_handle_t tp,
                                                    esp_lcd_touch_interrupt_callback_t callback)
{
    if (!tp)
    {
        return ESP_ERR_INVALID_ARG;
    }

    tp->config.interrupt_callback = callback;

    // If interrupt pin is configured and callback is provided, set up ISR
    if (tp->config.int_gpio_num != GPIO_NUM_NC && callback)
    {
        return gpio_isr_handler_add(tp->config.int_gpio_num,
                                    (gpio_isr_t)callback,
                                    tp);
    }

    return ESP_OK;
}
