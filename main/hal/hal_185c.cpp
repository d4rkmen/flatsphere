/**
 * @file hal_185c.cpp
 * @brief HAL implementation for Waveshare KNOB 1.85C board
 * @author d4rkmen
 * @license Apache License 2.0
 * @note Based on Forairaaaaa's HAL implementation
 */

#include "hal_185c.h"
#include "esp_log.h"

static const char* TAG = "HAL";

using namespace HAL;

void Hal185C::_init_i2c()
{
    ESP_LOGI(TAG, "init i2c");
    _i2c = new I2CMaster();
}

void Hal185C::_init_display()
{
    ESP_LOGI(TAG, "init display");
    _display = new Display(this);
}

void Hal185C::_init_speaker()
{
    ESP_LOGI(TAG, "init speaker");
    _speaker = new Speaker(this);
}

void Hal185C::_init_mic()
{
    ESP_LOGI(TAG, "init microphone");
    _mic = new Mic(this);
}

void Hal185C::_init_button()
{
    ESP_LOGI(TAG, "init button");
    _home_button = new Button(0);
}

void Hal185C::_init_bat()
{
    ESP_LOGI(TAG, "init battery");
    _battery = new Battery();
}

void Hal185C::_init_exio()
{
    ESP_LOGI(TAG, "init exio");
    _exio = new EXIO(this);
}

void Hal185C::_init_rtc()
{
    ESP_LOGI(TAG, "init rtc");
    _rtc = new RTC(this);
}

void Hal185C::_init_sdcard()
{
    ESP_LOGI(TAG, "init sdcard");
    _sdcard = new SDCard();
}

void Hal185C::_init_usb()
{
    ESP_LOGI(TAG, "init usb");
    _usb = new USB(this);
}

void Hal185C::_init_wifi()
{
    ESP_LOGI(TAG, "init wifi");
    _wifi = new WiFi(_settings);
}

void Hal185C::init()
{
    ESP_LOGI(TAG, "HAL init");

    _init_i2c();
    _init_exio();
    _init_rtc();
    _init_display();
    _init_speaker();
    // _init_mic();
    _init_button();
    _init_bat();
    _init_sdcard();
    // _init_usb();
    _init_wifi();
}