/**
 * @file main.cpp
 * @brief Main entry point for the project
 * @author d4rkmen
 * @license Apache License 2.0
 */

#include <cstdint>
#include "hal/hal_185c.h"
#include "settings/settings.hpp"
#include "picotts.h"
#include <lvgl.h>
#include "ui/ui.h"
#include "esp_random.h"
#include "esp_check.h"
#include "esp_log.h"
#include "sys/time.h"
#include "time.h"
#include "timezone.h"
#include <format>

static const char* TAG = "MAIN";

#define BATTERY_UPDATE_INTERVAL (5 * 1000)
#define TIME_ADJUSTMENT_INTERVAL (60 * 1000)
static const char* week_day_names[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
#define GREETING_NUM 8
const char* greetings[GREETING_NUM] = {
    "Hi! It is a good time to start your day",
    "Welcome to flat sphere clock",
    "Time is on your side",
    "Don't push the horses",
    "This moment is precious",
    "Time is the only thing that is truly valuable",
    "Don't let time slip away",
    "Have a great time",
};

// same core as Speaker task
#define TTS_CPU_CORE 1

using namespace HAL;
using namespace SETTINGS;

Settings settings;
Hal185C hal(&settings);

static esp_timer_handle_t rtc_timer;
static uint32_t last_timeinfo_update;
static uint32_t last_battery_update;

// Timezone management
static uint8_t current_timezone_index = TIMEZONE_DEFAULT_INDEX;

extern const uint8_t coin16_wav_start[] asm("_binary_coin16_wav_start");
extern const uint8_t coin16_wav_end[] asm("_binary_coin16_wav_end");

void show_controls(void)
{
    lv_obj_clear_flag(ui_ImageArmSecond, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_ImageArmMinute, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_ImageArmHour, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_LabelDay, LV_OBJ_FLAG_HIDDEN);
}

void hide_controls(void)
{
    lv_obj_add_flag(ui_ImageArmSecond, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ImageArmMinute, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ImageArmHour, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_LabelDay, LV_OBJ_FLAG_HIDDEN);
}

static void read_battery_level(void)
{
    static uint8_t last_level = 0;
    uint8_t level = hal.battery()->get_level();
    lv_color_t color;
    // set color according to level
    if (level > 80)
    {
        color = lv_color_hex(0x01FF2A);
    }
    else if (level > 60)
    {
        color = lv_color_hex(0xFFF200);
    }
    else if (level > 30)
    {
        color = lv_color_hex(0xFF8800);
    }
    else
    {
        color = lv_color_hex(0xFF2600);
    }
    lv_obj_set_style_bg_color(ui_BatteryBar, color, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_bar_set_value(ui_BatteryBar, level, LV_ANIM_OFF);
    if (abs(level - last_level) > 1)
    {
        last_level = level;
        std::string level_str = std::format("battery level is {}%", level);
        ESP_LOGI(TAG, "Saying battery level: %s", level_str.c_str());
        picotts_add(level_str.c_str(), level_str.length() + 1);
    }
}

static void read_rtc_time(void* callback)
{
    struct tm timeinfo;
    hal.rtc()->read_datetime(&timeinfo);
    struct timeval timeval = {
        .tv_sec = mktime(&timeinfo),
        .tv_usec = 0};
    settimeofday(&timeval, NULL);
    if (callback != NULL)
    {
        ((void (*)(void))callback)();
    }

    ESP_LOGI(TAG, "read_rtc_time: %04d-%02d-%02d %02d:%02d:%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

extern "C" void ui_load_datetime_from_rtc(struct tm* timeinfo)
{
    if (hal.rtc()->read_datetime(timeinfo) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read datetime");
        return;
    }
    ESP_LOGI(TAG, "read_rtc_datetime: %04d-%02d-%02d %02d:%02d:%02d", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
}

// C-linkage helper function for UI event handlers
extern "C" void ui_save_time_to_rtc(int hour, int minute)
{
    struct tm timeinfo = {
        .tm_sec = 0,
        .tm_min = minute,
        .tm_hour = hour};

    ESP_LOGI(TAG, "Saving time to RTC: %02d:%02d:00", hour, minute);
    if (hal.rtc()->set_time(&timeinfo) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set time");
        return;
    }
    hal.playButtonSound();
    // set system time from RTC
    read_rtc_time(NULL);
}

extern "C" void ui_save_date_to_rtc(int day, int month, int year)
{
    struct tm timeinfo = {
        .tm_mday = day,
        .tm_mon = month - 1,
        .tm_year = year - 1900,
    };
    if (hal.rtc()->set_date(&timeinfo) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set date");
        return;
    }
    hal.playButtonSound();
    ESP_LOGI(TAG, "Saving date to RTC: %04d-%02d-%02d", year, month, day);
    // set system time from RTC
    read_rtc_time(NULL);
}

/**
 * @brief Set timezone by index
 * @param index Timezone index (0 to TIMEZONE_COUNT-1)
 * @return ESP_OK on success
 */
esp_err_t set_timezone(uint8_t index)
{
    if (index >= timezone_get_count())
    {
        ESP_LOGE(TAG, "Invalid timezone index: %d", index);
        return ESP_ERR_INVALID_ARG;
    }

    const timezone_info_t* tz = timezone_get_info(index);
    if (!tz)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Set TZ environment variable
    setenv("TZ", tz->posix_tz, 1);
    tzset(); // Apply timezone change

    current_timezone_index = index;

    ESP_LOGI(TAG, "Timezone set to: %s (%s) - %s", tz->name, tz->city, tz->posix_tz);
    ESP_LOGI(TAG, "UTC offset: %+d hours, DST: %s", tz->utc_offset_hours, tz->has_dst ? "Yes" : "No");

    return ESP_OK;
}

/**
 * @brief Get current timezone index
 */
extern "C" uint8_t ui_get_current_timezone_index(void)
{
    return current_timezone_index;
}

/**
 * @brief Get timezone count
 */
extern "C" uint8_t ui_get_timezone_count(void)
{
    return timezone_get_count();
}

/**
 * @brief Get timezone name by index
 */
extern "C" const char* ui_get_timezone_name(uint8_t index)
{
    const timezone_info_t* tz = timezone_get_info(index);
    return tz ? tz->name : "Unknown";
}

/**
 * @brief Get timezone city by index
 */
extern "C" const char* ui_get_timezone_city(uint8_t index)
{
    const timezone_info_t* tz = timezone_get_info(index);
    return tz ? tz->city : "";
}

/**
 * @brief Get timezone info string (formatted)
 * @param index Timezone index
 * @param buffer Buffer to store formatted string
 * @param buffer_size Size of buffer
 * @return Number of characters written
 */
extern "C" int ui_get_timezone_info(uint8_t index, char* buffer, size_t buffer_size)
{
    const timezone_info_t* tz = timezone_get_info(index);
    if (!tz || !buffer || buffer_size == 0)
    {
        return 0;
    }

    return snprintf(buffer, buffer_size, "%s\n%s\nUTC%+d%s", tz->name, tz->city, tz->utc_offset_hours, tz->has_dst ? " (DST)" : "");
}

/**
 * @brief Check if current time is in DST
 */
extern "C" bool ui_is_dst_active(void)
{
    time_t now;
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    return timeinfo.tm_isdst > 0;
}

/**
 * @brief Set timezone from UI
 */
extern "C" void ui_set_timezone(uint8_t index)
{
    if (set_timezone(index) == ESP_OK)
    {
        hal.playButtonSound();
        // Update display with new timezone
        read_rtc_time(NULL);
    }
}

static void on_samples(int16_t* samples, size_t size)
{
    // play to virtual channel 1
    hal.speaker()->playRaw((const int16_t*)samples, size, PICOTTS_SAMPLE_FREQ_HZ, false, 1, 1);
}

static void say_time(struct tm* timeinfo)
{
    int hour = timeinfo->tm_hour;
    int hour_12 = hour % 12;
    int minute = timeinfo->tm_min;

    std::string time_str;

    if (hour == 0 && minute == 0)
    {
        time_str = "It is midnight";
    }
    else if (hour == 12 && minute == 0)
    {
        time_str = "It is noon";
    }
    else if (minute == 0)
    {
        time_str = std::format("It is {} o'clock", hour_12);
    }
    else
    {
        std::string hour_str = hour == 0 ? "midnight" : std::format("{}", hour == 12 ? hour : hour_12);
        int next_hour = (hour + 1) % 24;
        int next_hour_12 = next_hour % 12;
        std::string next_hour_str = next_hour == 0 ? "midnight" : std::format("{}", next_hour == 12 ? next_hour : next_hour_12);
        if (minute == 15)
        {
            time_str = std::format("It is quarter past {}", hour_str);
        }
        else if (minute == 30)
        {
            time_str = std::format("It is half past {}", hour_str);
        }
        else if (minute <= 20)
        {
            time_str = std::format("It is {} {} past {}", minute, minute == 1 ? "minute" : "minutes", hour_str);
        }
        else if (minute == 45)
        {
            time_str = std::format("It is quarter to {}", next_hour_str);
        }
        else if (minute > 45)
        {
            time_str = std::format("It is {} {} to {}", 60 - minute, 60 - minute == 1 ? "minute" : "minutes", next_hour_str);
        }
        // fallback to 24 hour format
        else
        {
            time_str = std::format("It is {}  {}", hour, minute);
        }
    }

    ESP_LOGI(TAG, "Saying time: %s", time_str.c_str());
    picotts_add(time_str.c_str(), time_str.length() + 1);
}

esp_err_t setup()
{
    if (!settings.init())
    {
        ESP_LOGE(TAG, "Failed to initialize settings");
        return ESP_FAIL;
    }
    // Init hal
    hal.init();
    hal.display()->set_backlight(settings.getNumber("system", "brightness"));
    // Initialize timezone
    set_timezone(settings.getNumber("system", "timezone"));

#if 0
    ESP_RETURN_ON_ERROR(hal.mic()->begin(), TAG, "Failed to start mic");
#endif
    ESP_RETURN_ON_ERROR(hal.speaker()->begin(), TAG, "Failed to start speaker");
    hal.speaker()->setVolume(settings.getNumber("system", "volume"));
    // setup TTS
    // ! important: task prio must be high enough
    unsigned prio = 6;
    if (!picotts_init(prio, on_samples, TTS_CPU_CORE))
    {
        ESP_LOGE(TAG, "Failed to initialize TTS");
        return ESP_FAIL;
    }
    if (settings.getBool("system", "boot_sound"))
    {
        int greeting_index = esp_random() % GREETING_NUM;
        // we have to add tailing 0 to PicoTTS
        picotts_add(greetings[greeting_index], strlen(greetings[greeting_index]) + 1);
        // or play wav
        // hal.speaker()->playWav(coin16_wav_start, coin16_wav_end - coin16_wav_start);
    }
    // init LVGL UI
    ui_init();
    // hide controls before we have correct time
    hide_controls();
    // init time from rtc
    read_rtc_time((void*)show_controls);
    // set rtc_timer for 60 sec for time adjustment
    {
        esp_timer_create_args_t esp_timer_args = {
            .callback = read_rtc_time,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "read_rtc_time"};
        ESP_RETURN_ON_ERROR(esp_timer_create(&esp_timer_args, &rtc_timer), TAG, "Failed to create rtc_timer");
        ESP_RETURN_ON_ERROR(esp_timer_start_periodic(rtc_timer, TIME_ADJUSTMENT_INTERVAL * 1000), TAG, "Failed to start periodic rtc_timer");
    }
    // set volume for keyboard sound channel
    hal.speaker()->setChannelVolume(0, 70);
    // init battery level
    read_battery_level();
    return ESP_OK;
}

extern "C" void app_main(void)
{
    setup();
    std::string last_week_day = "";

    while (1)
    {
        hal.display()->lvgl_timer_handler();

        if (hal.home_button()->is_pressed())
        {
            hal.playButtonSound();
            if (lv_scr_act() == ui_WatchFace)
            {
                struct tm timeinfo;
                ui_load_datetime_from_rtc(&timeinfo);
                lv_roller_set_selected(ui_RollerDay, timeinfo.tm_mday - 1, LV_ANIM_OFF);
                lv_roller_set_selected(ui_RollerMonth, timeinfo.tm_mon, LV_ANIM_OFF);
                lv_roller_set_selected(ui_RollerYear, timeinfo.tm_year + 1900 - 2025, LV_ANIM_OFF);

                _ui_screen_change(&ui_SetDate, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 250, 0, &ui_SetDate_screen_init);
            }
            else
            {
                _ui_screen_change(&ui_WatchFace, LV_SCR_LOAD_ANIM_MOVE_TOP, 250, 0, &ui_WatchFace_screen_init);
            }
        }
        uint32_t now = millis();
        // one sec elapsed?
        if (now - last_timeinfo_update >= 1000)
        {
            time_t now_time;
            time(&now_time);
            struct tm cur;
            localtime_r(&now_time, &cur);
#if 0
            ESP_LOGI(TAG, "battery voltage: %.2fV (%d%%)", hal.battery()->get_voltage(), hal.battery()->get_level());
#endif
            int16_t angle = (cur.tm_sec) * 3600 / 60;
            lv_img_set_angle(ui_ImageArmSecond, angle);
            angle = (angle + (cur.tm_min * 3600)) / 60;
            lv_img_set_angle(ui_ImageArmMinute, angle);
            angle = (angle + (cur.tm_hour * 3600)) / 12;
            lv_img_set_angle(ui_ImageArmHour, angle);

            last_timeinfo_update = now;
            // update week/month day
            std::string week_day = std::format("{} {:02d}", week_day_names[cur.tm_wday], cur.tm_mday);
            if (week_day != last_week_day)
            {
                lv_label_set_text(ui_LabelDay, week_day.c_str());
                last_week_day = week_day;
                // set color of ui_LabelDay to red
                lv_obj_set_style_text_color(ui_LabelDay, lv_color_hex((cur.tm_wday == 0 || cur.tm_wday == 6) ? 0xFF0000 : 0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            if (cur.tm_sec == 0
                // && cur.tm_min == 0)
            )
            {
                say_time(&cur);
            }
        }
        // battery level update
        if (now - last_battery_update >= BATTERY_UPDATE_INTERVAL)
        {
            read_battery_level();
            last_battery_update = now;
        }
    }
}
