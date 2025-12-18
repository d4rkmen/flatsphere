/**
 * @file hal.h
 * @author Forairaaaaa
 * @brief
 * @version 0.1
 * @date 2023-09-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include "board.h"
#include "display/display.hpp"
#include "sdcard/sdcard.hpp"
#include "button/button.hpp"
#include "speaker/speaker.hpp"
#include "mic/mic.hpp"
#include "usb/usb.hpp"
#include "wifi/wifi.hpp"
#include "settings/settings.hpp"
#include "i2c/i2c_master.hpp"
#include "bat/battery.hpp"
#include "exio/exio.hpp"
#include "rtc/rtc.hpp"

using namespace SETTINGS;

namespace HAL
{
    /**
     * @brief Hal base class
     *
     */
    class Hal
    {
    protected:
        Settings* _settings;
        Display* _display;
        I2CMaster* _i2c;
        Speaker* _speaker;
        Mic* _mic;
        Button* _home_button;
        SDCard* _sdcard;
        USB* _usb;
        WiFi* _wifi;
        Battery* _battery;
        EXIO* _exio;
        RTC* _rtc;
        BoardType _board_type;

    public:
        Hal(Settings* settings)
            : _settings(settings), _display(nullptr), _i2c(nullptr), _speaker(nullptr), _mic(nullptr), _home_button(nullptr), _sdcard(nullptr), _usb(nullptr), _wifi(nullptr), _battery(nullptr), _exio(nullptr), _rtc(nullptr), _board_type(BoardType::UNKNOWN)
        {
        }

        // Getter
        inline BoardType board_type() { return _board_type; }
        inline Settings* settings() { return _settings; }
        inline Display* display() { return _display; }
        inline I2CMaster* i2c() { return _i2c; }
        inline SDCard* sdcard() { return _sdcard; }
        inline Button* home_button() { return _home_button; }
        inline Speaker* speaker() { return _speaker; }
        inline Mic* mic() { return _mic; }
        inline WiFi* wifi() { return _wifi; }
        inline Battery* battery() { return _battery; }
        inline EXIO* exio() { return _exio; }
        inline RTC* rtc() { return _rtc; }
        inline USB* usb() { return _usb; }

        // Override
        virtual std::string type() { return "null"; }
        virtual void init() {}

        virtual void playButtonSound() {}
        virtual void playErrorSound() {}
        virtual void playDeviceConnectedSound() {}
        virtual void playDeviceDisconnectedSound() {}
    };
} // namespace HAL
