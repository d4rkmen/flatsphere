/**
 * @file hal_185c.cpp
 * @brief HAL implementation for Waveshare KNOB 1.85C board
 * @author d4rkmen
 * @license Apache License 2.0
 * @note Based on Forairaaaaa's HAL implementation
 */
#include "hal.h"
#include "common_define.h"

extern const uint8_t usb_connected_wav_start[] asm("_binary_usb_connected_wav_start");
extern const uint8_t usb_connected_wav_end[] asm("_binary_usb_connected_wav_end");
extern const uint8_t usb_disconnected_wav_start[] asm("_binary_usb_disconnected_wav_start");
extern const uint8_t usb_disconnected_wav_end[] asm("_binary_usb_disconnected_wav_end");
extern const uint8_t error_wav_start[] asm("_binary_error_wav_start");
extern const uint8_t error_wav_end[] asm("_binary_error_wav_end");

namespace HAL
{
    class Hal185C : public Hal
    {
    private:
        void _init_i2c();
        void _init_exio();
        void _init_rtc();
        void _init_display();
        void _init_speaker();
        void _init_mic();
        void _init_button();
        void _init_bat();
        void _init_sdcard();
        void _init_usb();
        void _init_wifi();

    public:
        Hal185C(SETTINGS::Settings* settings) : Hal(settings) {}
        std::string type() override
        {
            switch (_board_type)
            {
            case HAL::BoardType::WAVESHARE_KNOB_185C:
                return "LCD 1.85C";
            case HAL::BoardType::WAVESHARE_KNOB_18:
                return "KNOB 1.8";
            default:
                return "UNKNOWN";
            }
        }
        void init() override;
        void playButtonSound() override { _speaker->tone(1023, 50); }
        void playErrorSound() override { _speaker->playWav(error_wav_start, error_wav_end - error_wav_start); }
        void playDeviceConnectedSound() override
        {
            _speaker->playWav(usb_connected_wav_start, usb_connected_wav_end - usb_connected_wav_start);
        }
        void playDeviceDisconnectedSound() override
        {
            _speaker->playWav(usb_disconnected_wav_start, usb_disconnected_wav_end - usb_disconnected_wav_start);
        }

    public:
    };
} // namespace HAL
