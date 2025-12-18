/**
 * @file display.hpp
 * @brief Unified Display Driver Configuration
 * ST77916 LCD (QSPI) + CST816 Touch (I2C) + LVGL Integration
 * @author d4rkmen
 * @license Apache License 2.0
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "lvgl.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"

// Forward declarations
struct esp_lcd_touch_s;
typedef struct esp_lcd_touch_s* esp_lcd_touch_handle_t;

namespace HAL
{
    class Hal;
}

// ============================================================================
// LCD Configuration - ST77916 RGB565 360x360
// ============================================================================
namespace DisplayConfig
{
    namespace LCD
    {
        // Display dimensions
        constexpr uint16_t WIDTH = 360;
        constexpr uint16_t HEIGHT = 360;
        constexpr uint8_t COLOR_BITS = 16; // RGB565

        // QSPI Interface pins
        constexpr int PIN_SCK = 40;
        constexpr int PIN_DATA0 = 46;
        constexpr int PIN_DATA1 = 45;
        constexpr int PIN_DATA2 = 42;
        constexpr int PIN_DATA3 = 41;
        constexpr int PIN_CS = 21;
        constexpr int PIN_TE = 18; // Tearing effect (optional)

        // Control pins (via TCA9554 EXIO2)
        constexpr int PIN_RST = -1; // Handled by EXIO
        constexpr int PIN_BACKLIGHT = 5;

        // SPI Configuration
        constexpr spi_host_device_t SPI_HOST = SPI2_HOST; // SPI2_HOST
        constexpr int SPI_MODE = 0;
        constexpr uint32_t SPI_CLK_WRITE_HZ = 80 * 1000 * 1000; // 80MHz for write
        constexpr uint32_t SPI_CLK_READ_HZ = 3 * 1000 * 1000;   // 3MHz for read
        constexpr int SPI_TRANS_QUEUE_SZ = 10;
        constexpr int SPI_CMD_BITS = 32;
        constexpr int SPI_PARAM_BITS = 8;
        constexpr int SPI_MAX_TRANSFER_SIZE = 2048;

        // Backlight PWM Configuration
        constexpr int BACKLIGHT_LEDC_TIMER = 0;
        constexpr int BACKLIGHT_LEDC_CHANNEL = 0;
        constexpr int BACKLIGHT_LEDC_FREQ = 5000;
        constexpr int BACKLIGHT_LEDC_RESOLUTION = 13; // 13-bit
        constexpr uint8_t BACKLIGHT_DEFAULT = 100;    // 0-100
        constexpr uint8_t BACKLIGHT_MAX = 100;
    }

    // ========================================================================
    // Touch Configuration - CST816 I2C
    // ========================================================================
    namespace Touch
    {
        // I2C Configuration
        constexpr uint8_t I2C_ADDR = 0x15;
        constexpr int I2C_NUM = 0;
        constexpr int I2C_FREQ = 400000; // 400kHz

        // GPIO pins
        constexpr int PIN_SDA = 11;
        constexpr int PIN_SCL = 10;
        constexpr int PIN_INT = 4;
        constexpr int PIN_RST = -1; // Handled by EXIO1

        // Touch settings
        constexpr uint16_t MAX_X = LCD::WIDTH - 1;
        constexpr uint16_t MAX_Y = LCD::HEIGHT - 1;
        constexpr bool SWAP_XY = false;
        constexpr bool MIRROR_X = false;
        constexpr bool MIRROR_Y = false;
    }

    // ========================================================================
    // LVGL Configuration
    // ========================================================================
    namespace LVGL
    {
        // Buffer size (1/20 of screen for double buffering)
        constexpr uint32_t BUFFER_SIZE = (LCD::WIDTH * LCD::HEIGHT * (LCD::COLOR_BITS >> 3));
        constexpr uint8_t TICK_PERIOD_MS = 1;

        // Memory allocation
        constexpr bool USE_PSRAM = true; // Use external PSRAM for buffers
    }

    // ========================================================================
    // EXIO Pin Definitions (TCA9554PWR)
    // ========================================================================
    namespace EXIO
    {
        constexpr uint8_t TOUCH_RST = 0x01; // EXIO1 for touch reset
        constexpr uint8_t LCD_RST = 0x02;   // EXIO2 for LCD reset
    }
}

// ============================================================================
// Display Driver Class
// ============================================================================
class Display
{
public:
    Display(HAL::Hal* hal = nullptr);
    ~Display();

    // Initialization
    esp_err_t init();
    esp_err_t deinit();

    // LCD Control
    esp_err_t set_backlight(uint8_t level); // 0-100
    uint8_t get_backlight() const { return _backlight_level; }

    // Display operations
    esp_err_t draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t* data);
    esp_err_t fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t color);
    esp_err_t clear(uint32_t color = 0x000000);

    // Panel control
    esp_err_t display_on(bool on);
    esp_err_t invert_colors(bool invert);
    esp_err_t set_rotation(uint8_t rotation); // 0, 1, 2, 3
    // Command transmission helpers
    esp_err_t tx_param(int lcd_cmd, const void* param, size_t param_size);
    esp_err_t rx_param(int lcd_cmd, void* param, size_t param_size);
    esp_err_t tx_color(int lcd_cmd, const void* param, size_t param_size);
    // Panel low-level operations
    esp_err_t panel_reset();
    esp_err_t panel_init();
    esp_err_t panel_draw_bitmap(int x_start, int y_start, int x_end, int y_end, const void* color_data);
    esp_err_t panel_invert_color(bool invert_color_data);
    esp_err_t panel_mirror(bool mirror_x, bool mirror_y);
    esp_err_t panel_swap_xy(bool swap_axes);
    esp_err_t panel_set_gap(int x_gap, int y_gap);
    esp_err_t panel_on_off(bool on_off);
    esp_err_t panel_idle(bool on_off);

    // Touch operations
    bool get_touch_points(uint16_t* x, uint16_t* y, uint8_t max_points = 1);
    bool is_touched();

    // LVGL access
    lv_display_t* get_lvgl_display() { return _lv_disp; }
    void lvgl_timer_handler();

    // Screenshot
    esp_err_t take_screenshot(const char* filename);

    // Getters
    uint16_t width() const { return DisplayConfig::LCD::WIDTH; }
    uint16_t height() const { return DisplayConfig::LCD::HEIGHT; }
    esp_lcd_touch_handle_t get_touch_handle() { return _touch_handle; }
    esp_lcd_panel_io_handle_t get_io_handle_read() { return _io_handle_read; }
    esp_lcd_panel_io_handle_t get_io_handle_write() { return _io_handle_write; }
    inline HAL::Hal* hal() { return _hal; }

private:
    // Hardware initialization
    esp_err_t init_lcd();
    esp_err_t init_touch();
    esp_err_t init_lvgl();
    esp_err_t init_backlight();

    // Reset functions
    void _reset_lcd();
    void _reset_touch();

    // LVGL callbacks (LVGL 9.4 API)
    static uint32_t lvgl_get_tick_cb();
    static void lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
    static void lvgl_resolution_changed_event_cb(lv_event_t* e);
    static void lvgl_touch_cb(lv_indev_t* indev, lv_indev_data_t* data);
    static void lvgl_tick_cb(void* arg);

    // Member variables
    HAL::Hal* _hal;
    esp_lcd_touch_handle_t _touch_handle;
    esp_lcd_panel_io_handle_t _io_handle_read;
    esp_lcd_panel_io_handle_t _io_handle_write;

    // LVGL 9.4 structures
    lv_display_t* _lv_disp;
    lv_indev_t* _lv_indev;
    void* _lv_buf1;
    void* _lv_buf2;

    // ST77916 panel state
    int _reset_gpio_num;
    int _x_gap;
    int _y_gap;
    uint8_t _fb_bits_per_pixel;
    uint8_t _madctl_val;
    uint8_t _colmod_val;

    uint8_t _backlight_level;
    bool _initialized;
};
