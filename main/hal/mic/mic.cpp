/**
 * @file mic.cpp
 * @brief Microphone class for ESP32S3 using ESP-IDF I2S driver
 * @author d4rkmen
 * @note Based on M5Unified Mic_Class API
 * @license Apache License 2.0
 */
#include "mic.hpp"
#include "hal.h"
#include <cstring>
#include <algorithm>
#include "esp_log.h"

static const char* TAG = "MIC";

namespace HAL
{
    // External function from Speaker class for clock calculation
    extern void calcClockDiv(uint32_t* div_a, uint32_t* div_b, uint32_t* div_n, uint32_t baseClock, uint32_t targetFreq);

    Mic::Mic(Hal* hal) : _hal(hal)
    {
        // Configuration is set by default values in mic_config_t
    }

    Mic::~Mic()
    {
        end();
    }

    uint32_t Mic::_calc_rec_rate(void) const
    {
        return _cfg.sample_rate * _cfg.over_sampling;
    }

    esp_err_t Mic::_setup_i2s(void)
    {
        if (_cfg.pin_data_in < 0)
        {
            return ESP_ERR_INVALID_ARG;
        }

        // Configure I2S channel for RX (recording)
        i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(_cfg.i2s_port, I2S_ROLE_MASTER);
        chan_cfg.dma_desc_num = _cfg.dma_buf_count;
        chan_cfg.dma_frame_num = _cfg.dma_buf_len;
        chan_cfg.auto_clear = true;

        esp_err_t ret = i2s_new_channel(&chan_cfg, nullptr, &_rx_chan);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to create I2S RX channel: %s", esp_err_to_name(ret));
            return ret;
        }

        // Configure I2S standard mode
        i2s_std_config_t std_cfg = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_calc_rec_rate()),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_16BIT,
                _cfg.stereo ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO),
            .gpio_cfg =
                {
                    .mclk = I2S_GPIO_UNUSED,
                    .bclk = (gpio_num_t)_cfg.pin_bck,
                    .ws = (gpio_num_t)_cfg.pin_ws,
                    .dout = I2S_GPIO_UNUSED,
                    .din = (gpio_num_t)_cfg.pin_data_in,
                    .invert_flags =
                        {
                            .mclk_inv = false,
                            .bclk_inv = false,
                            .ws_inv = false,
                        },
                },
        };

        // Set slot mask based on channel selection
        if (!_cfg.stereo)
        {
            std_cfg.slot_cfg.slot_mask = _cfg.left_channel ? I2S_STD_SLOT_LEFT : I2S_STD_SLOT_RIGHT;
        }

        ret = i2s_channel_init_std_mode(_rx_chan, &std_cfg);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to init I2S std mode: %s", esp_err_to_name(ret));
            i2s_del_channel(_rx_chan);
            _rx_chan = nullptr;
            return ret;
        }

        // Enable I2S channel
        ret = i2s_channel_enable(_rx_chan);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(ret));
            i2s_del_channel(_rx_chan);
            _rx_chan = nullptr;
            return ret;
        }

        return ESP_OK;
    }

    void Mic::mic_task(void* args)
    {
        Mic* self = (Mic*)args;

        int oversampling = self->_cfg.over_sampling;
        if (oversampling < 1)
            oversampling = 1;
        else if (oversampling > 8)
            oversampling = 8;

        int32_t gain = self->_cfg.magnification;
        const float f_gain = (float)gain / (oversampling * 2.0f);

        size_t src_idx = ~0u;
        size_t src_len = 0;
        int32_t sum_value[4] = {0, 0, 0, 0};
        int32_t prev_value[2] = {0, 0};
        const bool in_stereo = self->_cfg.stereo;
        int32_t os_remain = oversampling;
        const size_t dma_buf_len = self->_cfg.dma_buf_len * sizeof(int16_t);
        int16_t* src_buf = (int16_t*)malloc(dma_buf_len);

        if (!src_buf)
        {
            ESP_LOGE(TAG, "Failed to allocate source buffer");
            self->_task_handle = nullptr;
            vTaskDelete(nullptr);
            return;
        }

        memset(src_buf, 0, dma_buf_len);

        // Read and discard first buffers to stabilize
        size_t bytes_read;
        i2s_channel_read(self->_rx_chan, src_buf, dma_buf_len, &bytes_read, pdMS_TO_TICKS(10));
        i2s_channel_read(self->_rx_chan, src_buf, dma_buf_len, &bytes_read, pdMS_TO_TICKS(10));

        while (self->_task_running)
        {
            bool rec_flip = self->_rec_flip;
            recording_info_t* current_rec = &(self->_rec_info[!rec_flip]);
            recording_info_t* next_rec = &(self->_rec_info[rec_flip]);

            size_t dst_remain = current_rec->length;
            if (dst_remain == 0)
            {
                rec_flip = !rec_flip;
                self->_rec_flip = rec_flip;
                xSemaphoreGive(self->_task_semaphore);
                std::swap(current_rec, next_rec);
                dst_remain = current_rec->length;
                if (dst_remain == 0)
                {
                    self->_is_recording = false;
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                    src_idx = ~0u;
                    src_len = 0;
                    sum_value[0] = 0;
                    sum_value[1] = 0;
                    continue;
                }
            }
            self->_is_recording = true;

            for (;;)
            {
                if (src_idx >= src_len)
                {
                    i2s_channel_read(self->_rx_chan, src_buf, dma_buf_len, &bytes_read, pdMS_TO_TICKS(100));
                    src_len = bytes_read / sizeof(int16_t);
                    src_idx = 0;
                }

                do
                {
                    sum_value[0] += src_buf[src_idx];
                    sum_value[1] += src_buf[src_idx + 1];
                    src_idx += 2;
                } while (--os_remain && (src_idx < src_len));

                if (os_remain)
                {
                    continue;
                }
                os_remain = oversampling;

                // Automatic zero level adjustment
                auto value_tmp = (sum_value[0] + sum_value[1]) << 3;
                int32_t offset = self->_offset;
                offset -= (value_tmp + offset + 16) >> 5;
                self->_offset = offset;
                offset = (offset + 8) >> 4;
                sum_value[0] += offset;
                sum_value[1] += offset;

                // Noise filtering
                int32_t noise_filter = self->_cfg.noise_filter_level;
                if (noise_filter)
                {
                    for (int i = 0; i < 2; ++i)
                    {
                        int32_t v = (sum_value[i] * (256 - noise_filter) + prev_value[i] * noise_filter + 128) >> 8;
                        prev_value[i] = v;
                        sum_value[i] = v * f_gain;
                    }
                }
                else
                {
                    for (int i = 0; i < 2; ++i)
                    {
                        sum_value[i] *= f_gain;
                    }
                }

                int output_num = 2;

                // Channel conversion
                if (in_stereo != current_rec->is_stereo)
                {
                    if (in_stereo)
                    {
                        // stereo -> mono convert
                        sum_value[0] = (sum_value[0] + sum_value[1] + 1) >> 1;
                        output_num = 1;
                    }
                    else
                    {
                        // mono -> stereo convert
                        auto tmp = sum_value[1];
                        sum_value[3] = tmp;
                        sum_value[2] = tmp;
                        sum_value[1] = sum_value[0];
                        output_num = 4;
                    }
                }

                // Write to output buffer
                for (int i = 0; i < output_num; ++i)
                {
                    auto value = sum_value[i];
                    if (current_rec->is_16bit)
                    {
                        // 16-bit signed output
                        if (value < INT16_MIN + 16)
                            value = INT16_MIN + 16;
                        else if (value > INT16_MAX - 16)
                            value = INT16_MAX - 16;
                        auto dst = (int16_t*)(current_rec->data);
                        *dst++ = value;
                        current_rec->data = dst;
                    }
                    else
                    {
                        // 8-bit unsigned output
                        value = ((value + 128) >> 8) + 128;
                        if (value < 0)
                            value = 0;
                        else if (value > 255)
                            value = 255;
                        auto dst = (uint8_t*)(current_rec->data);
                        *dst++ = value;
                        current_rec->data = dst;
                    }
                }

                sum_value[0] = 0;
                sum_value[1] = 0;
                dst_remain -= output_num;
                if ((int32_t)dst_remain <= 0)
                {
                    current_rec->length = 0;
                    break;
                }
            }
        }

        self->_is_recording = false;
        free(src_buf);
        self->_task_handle = nullptr;
        vTaskDelete(nullptr);
    }

    esp_err_t Mic::begin(void)
    {
        if (_task_running)
        {
            auto rate = _calc_rec_rate();
            if (_rec_sample_rate == rate)
            {
                return ESP_OK;
            }
            // Wait for recording to finish
            while (isRecording())
            {
                vTaskDelay(1);
            }
            end();
            _rec_sample_rate = rate;
        }

        if (!isEnabled())
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (_task_semaphore == nullptr)
        {
            _task_semaphore = xSemaphoreCreateBinary();
            if (!_task_semaphore)
            {
                ESP_LOGE(TAG, "Failed to create semaphore");
                return ESP_ERR_NO_MEM;
            }
        }

        esp_err_t ret = _setup_i2s();
        if (ret != ESP_OK)
        {
            return ret;
        }

        // Create mic task
        size_t stack_size = 2048 + (_cfg.dma_buf_len * sizeof(uint16_t));
        _task_running = true;

        BaseType_t result;
        if (_cfg.task_pinned_core >= 0 && _cfg.task_pinned_core < portNUM_PROCESSORS)
        {
            result = xTaskCreatePinnedToCore(mic_task,
                                             "mic_task",
                                             stack_size,
                                             this,
                                             _cfg.task_priority,
                                             &_task_handle,
                                             _cfg.task_pinned_core);
        }
        else
        {
            result = xTaskCreate(mic_task, "mic_task", stack_size, this, _cfg.task_priority, &_task_handle);
        }

        if (result != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to create mic task");
            _task_running = false;
            i2s_channel_disable(_rx_chan);
            i2s_del_channel(_rx_chan);
            _rx_chan = nullptr;
            return ESP_ERR_NO_MEM;
        }

        ESP_LOGI(TAG, "Microphone initialized (sample_rate=%lu Hz, oversampling=%d)", _cfg.sample_rate, _cfg.over_sampling);
        return ESP_OK;
    }

    void Mic::end(void)
    {
        if (!_task_running)
        {
            return;
        }

        _task_running = false;

        // Wake up task to exit
        if (_task_handle)
        {
            xTaskNotifyGive(_task_handle);
            // Wait for task to finish
            while (_task_handle)
            {
                vTaskDelay(1);
            }
        }

        // Delete I2S channel
        if (_rx_chan)
        {
            i2s_channel_disable(_rx_chan);
            i2s_del_channel(_rx_chan);
            _rx_chan = nullptr;
        }

        ESP_LOGI(TAG, "Microphone deinitialized");
    }

    bool Mic::_rec_raw(void* recdata, size_t array_len, bool flg_16bit, uint32_t sample_rate, bool flg_stereo)
    {
        if (!recdata || array_len == 0)
        {
            return false;
        }

        recording_info_t info;
        info.data = recdata;
        info.length = array_len;
        info.is_16bit = flg_16bit;
        info.is_stereo = flg_stereo;

        _cfg.sample_rate = sample_rate;

        esp_err_t ret = begin();
        if (ret != ESP_OK)
        {
            return false;
        }

        // Wait for previous recording to finish
        while (_rec_info[_rec_flip].length)
        {
            xSemaphoreTake(_task_semaphore, 1);
        }

        _rec_info[_rec_flip] = info;
        if (_task_handle)
        {
            xTaskNotifyGive(_task_handle);
        }
        return true;
    }

    bool Mic::record(uint8_t* rec_data, size_t array_len, uint32_t sample_rate, bool stereo)
    {
        return _rec_raw(rec_data, array_len, false, sample_rate, stereo);
    }

    bool Mic::record(int16_t* rec_data, size_t array_len, uint32_t sample_rate, bool stereo)
    {
        return _rec_raw(rec_data, array_len, true, sample_rate, stereo);
    }

    bool Mic::record(uint8_t* rec_data, size_t array_len)
    {
        return _rec_raw(rec_data, array_len, false, _cfg.sample_rate, false);
    }

    bool Mic::record(int16_t* rec_data, size_t array_len)
    {
        return _rec_raw(rec_data, array_len, true, _cfg.sample_rate, false);
    }

} // namespace HAL
