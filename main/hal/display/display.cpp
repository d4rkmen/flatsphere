/**
 * @file display.cpp
 * @brief Unified Display Driver Implementation
 * ST77916 LCD (QSPI) + CST816 Touch (I2C) + LVGL Integration
 */

#include "display.hpp"
#include "display_panel.hpp"
#include "display_touch.hpp"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "hal/hal.h"
#include "hal/i2c/i2c_master.hpp"
#include <cstring>
#include <cstdio>
#include "common_define.h"

static const char* TAG = "Display";

// ============================================================================
// Constructor / Destructor
// ============================================================================

Display::Display(HAL::Hal* hal)
    : _hal(hal), _touch_handle(nullptr), _lv_disp(nullptr), _lv_indev(nullptr), _lv_buf1(nullptr), _lv_buf2(nullptr), _backlight_level(DisplayConfig::LCD::BACKLIGHT_DEFAULT), _initialized(false)
{
    init();
}

Display::~Display()
{
    deinit();
}

// ============================================================================
// Main Initialization
// ============================================================================

esp_err_t Display::init()
{
    if (_initialized)
    {
        ESP_LOGW(TAG, "Display already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing display driver...");

    // Initialize in sequence (I2C already initialized by HAL)
    ESP_ERROR_CHECK(init_backlight());
    ESP_ERROR_CHECK(init_lcd());
    ESP_ERROR_CHECK(init_touch());
    ESP_ERROR_CHECK(init_lvgl());

    _initialized = true;
    ESP_LOGI(TAG, "Display driver initialized successfully");

    return ESP_OK;
}

esp_err_t Display::deinit()
{
    if (!_initialized)
    {
        return ESP_OK;
    }

    // Clean up LVGL
    if (_lv_indev)
    {
        lv_indev_delete(_lv_indev);
        _lv_indev = nullptr;
    }

    if (_lv_disp)
    {
        lv_display_delete(_lv_disp);
        _lv_disp = nullptr;
    }

    // Free LVGL buffers
    if (_lv_buf1)
    {
        heap_caps_free(_lv_buf1);
        _lv_buf1 = nullptr;
    }
    if (_lv_buf2)
    {
        heap_caps_free(_lv_buf2);
        _lv_buf2 = nullptr;
    }

    // Clean up touch
    if (_touch_handle)
    {
        esp_lcd_touch_del(_touch_handle);
        _touch_handle = nullptr;
    }

    // Clean up LCD

    _initialized = false;
    return ESP_OK;
}

// ============================================================================
// LCD Initialization
// ============================================================================

esp_err_t Display::init_lcd()
{
    ESP_LOGI(TAG, "Initializing ST77916 LCD...");
    // reset before init to set proper reset level
    _reset_lcd();

    // Initialize QSPI bus
    spi_bus_config_t bus_config = {
        .data0_io_num = DisplayConfig::LCD::PIN_DATA0,
        .data1_io_num = DisplayConfig::LCD::PIN_DATA1,
        .sclk_io_num = DisplayConfig::LCD::PIN_SCK,
        .data2_io_num = DisplayConfig::LCD::PIN_DATA2,
        .data3_io_num = DisplayConfig::LCD::PIN_DATA3,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = DisplayConfig::LCD::SPI_MAX_TRANSFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = 0,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(DisplayConfig::LCD::SPI_HOST,
                                       &bus_config,
                                       SPI_DMA_CH_AUTO));

    // Create LCD panel IO
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = DisplayConfig::LCD::PIN_CS,
        .dc_gpio_num = -1,
        .spi_mode = DisplayConfig::LCD::SPI_MODE,
        .pclk_hz = DisplayConfig::LCD::SPI_CLK_READ_HZ,
        .trans_queue_depth = DisplayConfig::LCD::SPI_TRANS_QUEUE_SZ,
        .on_color_trans_done = nullptr,
        .user_ctx = nullptr,
        .lcd_cmd_bits = DisplayConfig::LCD::SPI_CMD_BITS,
        .lcd_param_bits = DisplayConfig::LCD::SPI_PARAM_BITS,
        .flags = {
            .dc_low_on_data = 0,
            .octal_mode = 0,
            .quad_mode = 1, // QSPI mode
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0,
        },
    };
    uint32_t value = 0;
    // disabled to save resources, set 1 to enable LCD read
#if 1
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)DisplayConfig::LCD::SPI_HOST,
        &io_config,
        &_io_handle_read));
    // read ID from LCD
    ESP_ERROR_CHECK(rx_param(CMD_RDDID, &value, sizeof(value)));
    ESP_LOGI(TAG, "LCD ID: 0x%08X", value);
#endif

    io_config.pclk_hz = DisplayConfig::LCD::SPI_CLK_WRITE_HZ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)DisplayConfig::LCD::SPI_HOST,
        &io_config,
        &_io_handle_write));
    // Initialize ST77916 panel state
    _reset_gpio_num = DisplayConfig::LCD::PIN_RST;
    _x_gap = 0;
    _y_gap = 0;
    _fb_bits_per_pixel = DisplayConfig::LCD::COLOR_BITS;
    _madctl_val = 0; // RGB order
    _colmod_val = RGB565_2BYTE;

    // Configure reset GPIO if provided
    if (_reset_gpio_num >= 0)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << _reset_gpio_num,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }

    // Initialize panel
    ESP_ERROR_CHECK(panel_reset());
    ESP_ERROR_CHECK(panel_init());
    ESP_ERROR_CHECK(panel_invert_color(true));
    clear(0);
    ESP_ERROR_CHECK(panel_on_off(true));

    // Set backlight
    set_backlight(_backlight_level);
    ESP_LOGI(TAG, "LCD initialized successfully");
    return ESP_OK;
}

void Display::_reset_lcd()
{
    if (_hal->exio()->is_initialized())
    {
        _hal->exio()->write_pin(DisplayConfig::EXIO::LCD_RST, false);
        delay(10);
        _hal->exio()->write_pin(DisplayConfig::EXIO::LCD_RST, true);
        delay(120);
    }
}

// ============================================================================
// Touch Initialization
// ============================================================================

esp_err_t Display::init_touch()
{
    ESP_LOGI(TAG, "Initializing touch...");

    // Reset before init to set proper reset level
    _reset_touch();

    // Get I2C driver from HAL
    if (!_hal || !_hal->i2c() || !_hal->i2c()->is_initialized())
    {
        ESP_LOGE(TAG, "I2C not available in HAL");
        return ESP_ERR_INVALID_STATE;
    }

    // Get the bus handle from HAL's I2C driver
    i2c_master_bus_handle_t i2c_bus = _hal->i2c()->get_bus_handle();

    // Create touch panel IO using the new I2C master bus
    esp_lcd_panel_io_handle_t tp_io_handle = nullptr;
    esp_lcd_panel_io_i2c_config_t tp_io_config = {0};
    memset(&tp_io_config, 0, sizeof(esp_lcd_panel_io_i2c_config_t));
    tp_io_config.dev_addr = DisplayConfig::Touch::I2C_ADDR;
    tp_io_config.scl_speed_hz = DisplayConfig::Touch::I2C_FREQ;
    tp_io_config.control_phase_bytes = 1;
    tp_io_config.dc_bit_offset = 0;
    tp_io_config.lcd_cmd_bits = 8;
    tp_io_config.flags.disable_control_phase = 1;

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(
        i2c_bus,
        &tp_io_config,
        &tp_io_handle));

    // Create touch controller
    esp_lcd_touch_config_t tp_config = {
        .x_max = DisplayConfig::Touch::MAX_X,
        .y_max = DisplayConfig::Touch::MAX_Y,
        .rst_gpio_num = (gpio_num_t)DisplayConfig::Touch::PIN_RST,
        .int_gpio_num = (gpio_num_t)DisplayConfig::Touch::PIN_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = DisplayConfig::Touch::SWAP_XY,
            .mirror_x = DisplayConfig::Touch::MIRROR_X,
            .mirror_y = DisplayConfig::Touch::MIRROR_Y,
        },
        .process_coordinates = nullptr,
        .interrupt_callback = nullptr,
    };

    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_config, this, &_touch_handle));

    ESP_LOGI(TAG, "Touch initialized successfully");
    return ESP_OK;
}

void Display::_reset_touch()
{
    if (_hal->exio()->is_initialized())
    {
        _hal->exio()->write_pin(DisplayConfig::EXIO::TOUCH_RST, false);
        delay(10);
        _hal->exio()->write_pin(DisplayConfig::EXIO::TOUCH_RST, true);
        delay(50);
    }
}

// ============================================================================
// LVGL Initialization
// ============================================================================

void Display::lvgl_resolution_changed_event_cb(lv_event_t* e)
{
    lv_display_t* disp = (lv_display_t*)lv_event_get_target(e);
    Display* display = static_cast<Display*>(lv_display_get_user_data(disp));
    int32_t hor_res = lv_display_get_horizontal_resolution(disp);
    int32_t ver_res = lv_display_get_vertical_resolution(disp);
    lv_display_rotation_t rot = lv_display_get_rotation(disp);

    /* handle rotation */
    switch (rot)
    {
    case LV_DISPLAY_ROTATION_0:
        display->set_rotation(0); /* Portrait orientation */
        break;
    case LV_DISPLAY_ROTATION_90:
        display->set_rotation(1); /* Landscape orientation */
        break;
    case LV_DISPLAY_ROTATION_180:
        display->set_rotation(2); /* Portrait orientation, flipped */
        break;
    case LV_DISPLAY_ROTATION_270:
        display->set_rotation(3); /* Landscape orientation, flipped */
        break;
    }
}

uint32_t Display::lvgl_get_tick_cb()
{
    return millis();
}

esp_err_t Display::init_lvgl()
{
    ESP_LOGI(TAG, "Initializing LVGL (use: %s)...", DisplayConfig::LVGL::USE_PSRAM ? "PSRAM" : "RAM");

    // Initialize LVGL
    lv_init();
    _lv_buf1 = heap_caps_aligned_alloc(32, DisplayConfig::LVGL::BUFFER_SIZE, DisplayConfig::LVGL::USE_PSRAM ? MALLOC_CAP_SPIRAM : MALLOC_CAP_8BIT);
    _lv_buf2 = heap_caps_aligned_alloc(32, DisplayConfig::LVGL::BUFFER_SIZE, DisplayConfig::LVGL::USE_PSRAM ? MALLOC_CAP_SPIRAM : MALLOC_CAP_8BIT);
    if (!_lv_buf1 || !_lv_buf2)
    {
        ESP_LOGE(TAG, "Failed to allocate LVGL buffers");
        return ESP_ERR_NO_MEM;
    }

    // Create display (LVGL 9.4 API)
    _lv_disp = lv_display_create(DisplayConfig::LCD::WIDTH, DisplayConfig::LCD::HEIGHT);
    if (!_lv_disp)
    {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        return ESP_ERR_NO_MEM;
    }

    // Set display buffers
    lv_display_set_buffers(_lv_disp, _lv_buf1, _lv_buf2, DisplayConfig::LVGL::BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Set flush callback
    lv_display_set_flush_cb(_lv_disp, lvgl_flush_cb);

    // Set user data
    lv_display_set_user_data(_lv_disp, this);
    // ! BUG: use LV_COLOR_FORMAT_RGB565_SWAPPED lvgl greater 9.4
    lv_display_set_color_format(_lv_disp, LV_COLOR_FORMAT_RGB565);
    lv_display_add_event_cb(_lv_disp, lvgl_resolution_changed_event_cb, LV_EVENT_RESOLUTION_CHANGED, NULL);

    // Create input device
    _lv_indev = lv_indev_create();
    if (!_lv_indev)
    {
        ESP_LOGE(TAG, "Failed to create LVGL input device");
        return ESP_ERR_NO_MEM;
    }

    lv_indev_set_type(_lv_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(_lv_indev, lvgl_touch_cb);
    lv_indev_set_user_data(_lv_indev, this);
#if 1
    // Create and start LVGL tick timer
    esp_timer_create_args_t timer_args = {
        .callback = lvgl_tick_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick",
        .skip_unhandled_events = false,
    };

    esp_timer_handle_t lvgl_tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer,
                                             DisplayConfig::LVGL::TICK_PERIOD_MS * 1000));
#else
    // set tick callback
    lv_tick_set_cb(lvgl_get_tick_cb);
#endif
    ESP_LOGI(TAG, "LVGL initialized successfully");
    return ESP_OK;
}

// ============================================================================
// LVGL Callbacks
// ============================================================================

void Display::lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map)
{
    Display* display = static_cast<Display*>(lv_display_get_user_data(disp));
    // ! BUG: use LV_COLOR_FORMAT_RGB565_SWAPPED lvgl greater 9.4
    lv_draw_sw_rgb565_swap(px_map, lv_area_get_size(area));

    display->panel_draw_bitmap(area->x1,
                               area->y1,
                               area->x2 + 1,
                               area->y2 + 1,
                               px_map);

    lv_display_flush_ready(disp);
}

void Display::lvgl_touch_cb(lv_indev_t* indev, lv_indev_data_t* data)
{
    Display* display = static_cast<Display*>(lv_indev_get_user_data(indev));

    uint16_t x[1] = {0};
    uint16_t y[1] = {0};
    uint8_t cnt = 0;

    // Read touch data
    esp_lcd_touch_read_data(display->_touch_handle);

    // Get coordinates
    bool pressed = esp_lcd_touch_get_coordinates(display->_touch_handle,
                                                 x,
                                                 y,
                                                 nullptr,
                                                 &cnt,
                                                 1);

    if (pressed && cnt > 0)
    {
        data->point.x = x[0];
        data->point.y = y[0];
        data->state = LV_INDEV_STATE_PRESSED;
        // ESP_LOGD(TAG, "Touch pressed, count: %d, x=%d, y=%d", cnt, x[0], y[0]);
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
        // ESP_LOGI(TAG, "Touch released");
    }
}

void Display::lvgl_tick_cb(void* arg)
{
    lv_tick_inc(DisplayConfig::LVGL::TICK_PERIOD_MS);
}

void Display::lvgl_timer_handler()
{
    lv_timer_handler();
}

// ============================================================================
// Backlight Control
// ============================================================================

esp_err_t Display::init_backlight()
{
    ESP_LOGI(TAG, "Initializing backlight...");

    // Configure GPIO
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << DisplayConfig::LCD::PIN_BACKLIGHT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)DisplayConfig::LCD::BACKLIGHT_LEDC_RESOLUTION,
        .timer_num = (ledc_timer_t)DisplayConfig::LCD::BACKLIGHT_LEDC_TIMER,
        .freq_hz = DisplayConfig::LCD::BACKLIGHT_LEDC_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = DisplayConfig::LCD::PIN_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)DisplayConfig::LCD::BACKLIGHT_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = (ledc_timer_t)DisplayConfig::LCD::BACKLIGHT_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags = {
            .output_invert = 0,
        },
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));

    ESP_LOGI(TAG, "Backlight initialized");
    return ESP_OK;
}

esp_err_t Display::set_backlight(uint8_t level)
{
    if (level > DisplayConfig::LCD::BACKLIGHT_MAX)
    {
        level = DisplayConfig::LCD::BACKLIGHT_MAX;
    }

    _backlight_level = level;

    // Calculate duty cycle
    uint32_t max_duty = (1 << DisplayConfig::LCD::BACKLIGHT_LEDC_RESOLUTION) - 1;
    uint32_t duty = (level == 0) ? 0 : (max_duty - (81 * (DisplayConfig::LCD::BACKLIGHT_MAX - level)));

    ledc_set_duty(LEDC_LOW_SPEED_MODE,
                  (ledc_channel_t)DisplayConfig::LCD::BACKLIGHT_LEDC_CHANNEL,
                  duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,
                     (ledc_channel_t)DisplayConfig::LCD::BACKLIGHT_LEDC_CHANNEL);

    return ESP_OK;
}

// ============================================================================
// Display Operations
// ============================================================================

esp_err_t Display::draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t* data)
{
    return panel_draw_bitmap(x, y, x + w, y + h, data);
}

esp_err_t Display::fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t color)
{
    // avoid logging here
    // ESP_LOGD(TAG, "fillRect: x=%d, y=%d, w=%d, h=%d, color=0x%04X", x, y, w, h, color);
    // Simple implementation - can be optimized
    uint16_t* buffer = (uint16_t*)heap_caps_malloc(w * h * sizeof(uint16_t), DisplayConfig::LVGL::USE_PSRAM ? MALLOC_CAP_SPIRAM : MALLOC_CAP_8BIT);
    if (!buffer)
    {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        return ESP_ERR_NO_MEM;
    }
    for (uint32_t i = 0; i < w * h; i++)
    {
        buffer[i] = color;
    }

    esp_err_t ret = draw_bitmap(x, y, w, h, (uint8_t*)buffer);
    free(buffer);
    return ret;
}

esp_err_t Display::clear(uint32_t color)
{
    return fill_rect(0, 0, DisplayConfig::LCD::WIDTH, DisplayConfig::LCD::HEIGHT, color);
}

esp_err_t Display::set_rotation(uint8_t rotation)
{
    bool swap_xy = false;
    bool mirror_x = false;
    bool mirror_y = false;

    switch (rotation)
    {
    case 0: // 0 degrees
        swap_xy = false;
        mirror_x = false;
        mirror_y = false;
        break;
    case 1: // 90 degrees
        swap_xy = true;
        mirror_x = false;
        mirror_y = true;
        break;
    case 2: // 180 degrees
        swap_xy = false;
        mirror_x = true;
        mirror_y = true;
        break;
    case 3: // 270 degrees
        swap_xy = true;
        mirror_x = true;
        mirror_y = false;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(panel_swap_xy(swap_xy));
    ESP_ERROR_CHECK(panel_mirror(mirror_x, mirror_y));

    return ESP_OK;
}

// ============================================================================
// Touch Operations
// ============================================================================

bool Display::get_touch_points(uint16_t* x, uint16_t* y, uint8_t max_points)
{
    uint8_t cnt = 0;
    esp_lcd_touch_read_data(_touch_handle);
    return esp_lcd_touch_get_coordinates(_touch_handle, x, y, nullptr, &cnt, max_points);
}

bool Display::is_touched()
{
    uint16_t x, y;
    return get_touch_points(&x, &y, 1);
}

// ! not work due to SPI driver cant handle 2 contexts on the same CS line
// workaround: use low
esp_err_t Display::take_screenshot(const char* filename)
{
    ESP_LOGD(TAG, "Taking screenshot to: %s", filename);

    // Calculate buffer size: 1/10th of screen
    const uint32_t TOTAL_PIXELS = width() * height();
    // ! ST77619 always use RGB888
    const uint32_t TOTAL_BYTES = TOTAL_PIXELS * 3;
    const uint32_t SCREENSHOT_BUF_SIZE = DisplayConfig::LCD::SPI_MAX_TRANSFER_SIZE / 3 * 3;

    ESP_LOGD(TAG, "Screen: %dx%d, Total: %u bytes, Chunk: %u bytes", width(), height(), TOTAL_BYTES, SCREENSHOT_BUF_SIZE);
    // set COLMOD to RGB888
    uint8_t colmod_data = {RGB666_3BYTE};
    esp_err_t ret = tx_param(CMD_COLMOD, &colmod_data, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set COLMOD to RGB888");
        return ret;
    }
    // Allocate buffer + 1 dummy byte
    uint8_t* buffer = (uint8_t*)heap_caps_malloc(SCREENSHOT_BUF_SIZE + 1, MALLOC_CAP_8BIT);
    if (!buffer)
    {
        ESP_LOGE(TAG, "Failed to allocate screenshot buffer");
        return ESP_ERR_NO_MEM;
    }

    // Open file for writing
    FILE* file = fopen(filename, "wb");
    if (!file)
    {
        ESP_LOGE(TAG, "Failed to open file: %s", filename);
        heap_caps_free(buffer);
        return ESP_FAIL;
    }

    // Set column address (full width)
    uint8_t col_data[4] = {
        (uint8_t)(0 >> 8),
        (uint8_t)(0 & 0xFF),
        (uint8_t)((width() - 1) >> 8),
        (uint8_t)((width() - 1) & 0xFF)};
    ret = tx_param(CMD_CASET, col_data, sizeof(col_data));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set column address");
        fclose(file);
        heap_caps_free(buffer);
        return ret;
    }

    // Set row address (full height)
    uint8_t row_data[4] = {
        (uint8_t)(0 >> 8),
        (uint8_t)(0 & 0xFF),
        (uint8_t)((height() - 1) >> 8),
        (uint8_t)((height() - 1) & 0xFF)};
    ret = tx_param(CMD_RASET, row_data, sizeof(row_data));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set row address");
        fclose(file);
        heap_caps_free(buffer);
        return ret;
    }

    // Read screen data in chunks
    uint32_t bytes_read = 0;
    uint32_t chunk_count = 0;

    while (bytes_read < TOTAL_BYTES)
    {
        uint32_t bytes_to_read = (TOTAL_BYTES - bytes_read) > SCREENSHOT_BUF_SIZE
                                     ? SCREENSHOT_BUF_SIZE
                                     : (TOTAL_BYTES - bytes_read);

        // Read chunk from display RAM + 1 dummy byte
        ret = rx_param(bytes_read == 0 ? CMD_RAMRD : CMD_RDMEMC, buffer, bytes_to_read + 1);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read RAM at chunk %u", chunk_count);
            fclose(file);
            heap_caps_free(buffer);
            return ret;
        }
        // hex dump buffer
        // ESP_LOG_BUFFER_HEXDUMP(TAG, buffer + 1, bytes_to_read > 0x100 ? 0x100 : bytes_to_read, ESP_LOG_INFO);

        // Write chunk to file, skip the first dummy byte
        size_t written = fwrite(buffer + 1, 1, bytes_to_read, file);
        if (written != bytes_to_read)
        {
            ESP_LOGE(TAG, "Failed to write to file (wrote %u/%u bytes)", written, bytes_to_read);
            fclose(file);
            heap_caps_free(buffer);
            return ESP_FAIL;
        }

        bytes_read += bytes_to_read;
        chunk_count++;

        // if (chunk_count % 5 == 0)
        // {
        //     ESP_LOGI(TAG, "Progress: %u of %u bytes (%.1f%%)", bytes_read, TOTAL_BYTES, (bytes_read * 100.0f) / TOTAL_BYTES);
        // }
    }

    // Cleanup
    fclose(file);
    heap_caps_free(buffer);

    ESP_LOGI(TAG, "Screenshot saved successfully: %u bytes in %u chunks", bytes_read, chunk_count);
    // restore original COLMOD
    ESP_RETURN_ON_ERROR(tx_param(CMD_COLMOD, &_colmod_val, 1), TAG, "Failed to restore original COLMOD");
    return ESP_OK;
}
