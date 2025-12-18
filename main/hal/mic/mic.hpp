/**
 * @file mic.hpp
 * @brief Microphone class for ESP32S3 using ESP-IDF I2S driver
 * @author d4rkmen
 * @note Based on M5Unified Mic_Class API
 * @license Apache License 2.0
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "common_define.h"
#include "esp_err.h"

// Pin configuration for microphone
#define MIC_PIN_DATA_IN 39 // SD (Serial Data)
#define MIC_PIN_BCK 15     // SCK (Serial Clock)
#define MIC_PIN_WS 2       // WS (Word Select)
#define MIC_I2S_PORT I2S_NUM_1

namespace HAL
{
    // Forward declaration
    class Hal;

    /**
     * @brief Input channel configuration
     */
    enum class InputChannel : uint8_t
    {
        ONLY_RIGHT = 0,
        ONLY_LEFT = 1,
        STEREO = 2,
    };

    /**
     * @brief Microphone configuration structure
     */
    struct mic_config_t
    {
        /// I2S data in pin (for mic)
        int pin_data_in = MIC_PIN_DATA_IN;

        /// I2S bit clock pin
        int pin_bck = MIC_PIN_BCK;

        /// I2S word select pin (LRCK)
        int pin_ws = MIC_PIN_WS;

        /// Input sampling rate (Hz)
        uint32_t sample_rate = 16000;

        /// Input channel configuration
        union
        {
            struct
            {
                uint8_t left_channel : 1;
                uint8_t stereo : 1;
                uint8_t reserve : 6;
            };
            InputChannel input_channel = InputChannel::ONLY_RIGHT;
        };

        /// Sampling times to obtain the average value
        uint8_t over_sampling = 2;

        /// Multiplier for input value
        uint8_t magnification = 16;

        /// Coefficient of the previous value, used for noise filtering (0-255)
        uint8_t noise_filter_level = 0;

        /// For I2S DMA buffer length
        size_t dma_buf_len = 128;

        /// For I2S DMA buffer count
        size_t dma_buf_count = 8;

        /// Background task priority
        uint8_t task_priority = 2;

        /// Background task pinned core (-1 for no pinning)
        int8_t task_pinned_core = 1;

        /// I2S port
        i2s_port_t i2s_port = MIC_I2S_PORT;
    };

    /**
     * @brief Microphone class
     */
    class Mic
    {
    public:
        Mic(Hal* hal);
        virtual ~Mic();

        /**
         * @brief Get current configuration
         */
        mic_config_t config(void) const { return _cfg; }

        /**
         * @brief Set configuration
         */
        void config(const mic_config_t& cfg) { _cfg = cfg; }

        /**
         * @brief Initialize and start the microphone
         */
        esp_err_t begin(void);

        /**
         * @brief Stop and deinitialize the microphone
         */
        void end(void);

        /**
         * @brief Check if mic task is running
         */
        bool isRunning(void) const { return _task_running; }

        /**
         * @brief Check if microphone is enabled
         */
        bool isEnabled(void) const { return _cfg.pin_data_in >= 0; }

        /**
         * @brief Check if currently recording
         * @return 0=not recording / 1=recording (room in queue) / 2=recording (no room in queue)
         */
        size_t isRecording(void) const
        {
            return _is_recording ? ((bool)_rec_info[0].length) + ((bool)_rec_info[1].length) : 0;
        }

        /**
         * @brief Set recording sampling rate
         * @param sample_rate Sampling rate (Hz)
         */
        void setSampleRate(uint32_t sample_rate) { _cfg.sample_rate = sample_rate; }

        /**
         * @brief Record raw sound wave data (8-bit unsigned)
         * @param rec_data Recording destination array
         * @param array_len Number of array elements
         * @param sample_rate Sampling rate (Hz)
         * @param stereo true=stereo / false=mono
         */
        bool record(uint8_t* rec_data, size_t array_len, uint32_t sample_rate, bool stereo = false);

        /**
         * @brief Record raw sound wave data (16-bit signed)
         * @param rec_data Recording destination array
         * @param array_len Number of array elements
         * @param sample_rate Sampling rate (Hz)
         * @param stereo true=stereo / false=mono
         */
        bool record(int16_t* rec_data, size_t array_len, uint32_t sample_rate, bool stereo = false);

        /**
         * @brief Record using current sample rate (8-bit unsigned)
         * @param rec_data Recording destination array
         * @param array_len Number of array elements
         */
        bool record(uint8_t* rec_data, size_t array_len);

        /**
         * @brief Record using current sample rate (16-bit signed)
         * @param rec_data Recording destination array
         * @param array_len Number of array elements
         */
        bool record(int16_t* rec_data, size_t array_len);

    private:
        Hal* _hal;
        mic_config_t _cfg;

        /**
         * @brief Recording information structure
         */
        struct recording_info_t
        {
            void* data = nullptr;
            size_t length = 0;
            size_t index = 0;
            bool is_stereo = false;
            bool is_16bit = false;
        };

        recording_info_t _rec_info[2];
        volatile bool _rec_flip = false;

        uint32_t _rec_sample_rate = 0;
        int32_t _offset = 0;
        volatile bool _task_running = false;
        volatile bool _is_recording = false;

        TaskHandle_t _task_handle = nullptr;
        volatile SemaphoreHandle_t _task_semaphore = nullptr;
        i2s_chan_handle_t _rx_chan = nullptr;

        /**
         * @brief Microphone task
         */
        static void mic_task(void* args);

        /**
         * @brief Calculate recording sample rate with oversampling
         */
        uint32_t _calc_rec_rate(void) const;

        /**
         * @brief Setup I2S interface
         */
        esp_err_t _setup_i2s(void);

        /**
         * @brief Internal record raw function
         */
        bool _rec_raw(void* recdata, size_t array_len, bool flg_16bit, uint32_t sample_rate, bool stereo);
    };

} // namespace HAL
