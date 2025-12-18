/**
 * @file display_touch.hpp
 * @brief CST816 Touch Controller
 * @author d4rkmen
 * @copyright Espressif Systems (Shanghai) CO LTD
 * @license Apache License 2.0
 */

#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stdbool.h>

// Forward declaration for Display class
class Display;

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// Touch Configuration
// ============================================================================
#define ESP_LCD_TOUCH_MAX_POINTS 2

// CST816 Registers
#define CST816_REG_DATA_START 0x02
#define CST816_REG_CHIP_ID 0xA7
#define CST816_REG_AUTO_SLEEP 0xFE
#define CST816_I2C_ADDRESS 0x15
#define CST816_MAX_POINTS 1

    // ============================================================================
    // Touch Structures
    // ============================================================================

    typedef struct esp_lcd_touch_s esp_lcd_touch_t;
    typedef esp_lcd_touch_t* esp_lcd_touch_handle_t;
    typedef void (*esp_lcd_touch_interrupt_callback_t)(esp_lcd_touch_handle_t tp);

    /**
     * @brief Touch configuration structure
     */
    typedef struct
    {
        uint16_t x_max;
        uint16_t y_max;
        gpio_num_t rst_gpio_num;
        gpio_num_t int_gpio_num;

        struct
        {
            unsigned int reset : 1;
            unsigned int interrupt : 1;
        } levels;

        struct
        {
            unsigned int swap_xy : 1;
            unsigned int mirror_x : 1;
            unsigned int mirror_y : 1;
        } flags;

        void (*process_coordinates)(esp_lcd_touch_handle_t tp, uint16_t* x, uint16_t* y, uint16_t* strength, uint8_t* point_num, uint8_t max_point_num);
        esp_lcd_touch_interrupt_callback_t interrupt_callback;
    } esp_lcd_touch_config_t;

    /**
     * @brief Touch data structure
     */
    typedef struct
    {
        uint8_t points;
        struct
        {
            uint16_t x;
            uint16_t y;
            uint16_t strength;
        } coords[ESP_LCD_TOUCH_MAX_POINTS];
        portMUX_TYPE lock;
    } esp_lcd_touch_data_t;

    /**
     * @brief Touch controller structure
     */
    struct esp_lcd_touch_s
    {
        esp_err_t (*read_data)(esp_lcd_touch_handle_t tp);
        bool (*get_xy)(esp_lcd_touch_handle_t tp, uint16_t* x, uint16_t* y, uint16_t* strength, uint8_t* point_num, uint8_t max_point_num);
        esp_err_t (*del)(esp_lcd_touch_handle_t tp);
        esp_err_t (*set_swap_xy)(esp_lcd_touch_handle_t tp, bool swap);
        esp_err_t (*get_swap_xy)(esp_lcd_touch_handle_t tp, bool* swap);
        esp_err_t (*set_mirror_x)(esp_lcd_touch_handle_t tp, bool mirror);
        esp_err_t (*get_mirror_x)(esp_lcd_touch_handle_t tp, bool* mirror);
        esp_err_t (*set_mirror_y)(esp_lcd_touch_handle_t tp, bool mirror);
        esp_err_t (*get_mirror_y)(esp_lcd_touch_handle_t tp, bool* mirror);
        esp_err_t (*enter_sleep)(esp_lcd_touch_handle_t tp);
        esp_err_t (*exit_sleep)(esp_lcd_touch_handle_t tp);

        esp_lcd_touch_config_t config;
        esp_lcd_panel_io_handle_t io;
        esp_lcd_touch_data_t data;
        Display* display; // Display instance for accessing HAL/EXIO
    };

    // ============================================================================
    // Public API
    // ============================================================================

    /**
     * @brief Create a new CST816 touch driver
     *
     * @param io I2C panel IO handle
     * @param config Touch configuration
     * @param display Display instance for accessing HAL/EXIO
     * @param tp Output touch handle
     * @return ESP_OK on success
     */
    esp_err_t esp_lcd_touch_new_i2c_cst816s(const esp_lcd_panel_io_handle_t io,
                                            const esp_lcd_touch_config_t* config,
                                            Display* display,
                                            esp_lcd_touch_handle_t* tp);

    /**
     * @brief Read touch data
     *
     * @param tp Touch handle
     * @return ESP_OK on success
     */
    esp_err_t esp_lcd_touch_read_data(esp_lcd_touch_handle_t tp);

    /**
     * @brief Get touch coordinates
     *
     * @param tp Touch handle
     * @param x X coordinates array
     * @param y Y coordinates array
     * @param strength Strength array (can be NULL)
     * @param point_num Number of points touched (output)
     * @param max_point_num Maximum points to read
     * @return true if touched, false otherwise
     */
    bool esp_lcd_touch_get_coordinates(esp_lcd_touch_handle_t tp, uint16_t* x, uint16_t* y, uint16_t* strength, uint8_t* point_num, uint8_t max_point_num);

    /**
     * @brief Delete touch controller
     *
     * @param tp Touch handle
     * @return ESP_OK on success
     */
    esp_err_t esp_lcd_touch_del(esp_lcd_touch_handle_t tp);

    /**
     * @brief Register interrupt callback
     *
     * @param tp Touch handle
     * @param callback Callback function
     * @return ESP_OK on success
     */
    esp_err_t esp_lcd_touch_register_interrupt_callback(esp_lcd_touch_handle_t tp,
                                                        esp_lcd_touch_interrupt_callback_t callback);

#ifdef __cplusplus
}
#endif
