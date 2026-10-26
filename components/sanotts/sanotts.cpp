#include "flatsphere_sanotts.h"
#include "sdkconfig.h"
#include "snt_tts.h"
extern "C" {
#include "esp_g2p.h"
}
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <algorithm>
#include <atomic>
#include <cmath>

extern "C" {
bool snt_port_init(void);
extern void *snt_port_res_arena;
extern size_t snt_port_res_cap;
}
#include <cstring>

namespace {
constexpr size_t arena_bytes = 2 * 1024 * 1024;
// Decoder residency peaks at ~132 KiB (head 74 KiB + pw/film banks 58 KiB).
// Weights that do not fit stay in PSRAM and use the scalar kernels.
constexpr size_t res_arena_max = 136 * 1024;
constexpr size_t internal_reserve = 48 * 1024;
constexpr size_t pcm_capacity = 640 * 256; // capped by patched runtime
constexpr unsigned sample_rate = 22050;
// ESP-IDF task stack sizes are bytes. eSpeak MakePhonemeList alone needs
// 32,000 bytes for ph_list3[1000], plus TranslateClause and worker frames.
// The old 24 KiB stack let phonemization overwrite adjacent free heap.
constexpr uint32_t worker_stack_bytes = 64 * 1024;
struct Request { char text[192]; };
QueueHandle_t queue;
std::atomic<bool> failed{false};
void *arena;
int16_t *audio;
size_t audio_size;
flatsphere_pcm_sink output;
void *output_user;
const char *TAG = "SANOTTS";
extern const uint8_t front[] asm("_binary_front_q8_bin_start");
extern const uint8_t model[] asm("_binary_model_q8_bin_start");

int collect_pcm(const float *pcm, int count, void *)
{
    if (count < 0 || static_cast<size_t>(count) > pcm_capacity - audio_size)
        return 1;
    for (int i = 0; i < count; ++i)
    {
        if (!std::isfinite(pcm[i])) return 1;
        audio[audio_size++] = static_cast<int16_t>(
            std::clamp(pcm[i], -1.0f, 1.0f) * 32767.0f);
    }
    return 0;
}

void worker(void *)
{
    // G2P initialization also needs the larger worker stack, not app_main's.
    if (esp_g2p_init() != 0)
    {
        failed.store(true);
        ESP_LOGE(TAG, "G2P initialization failed; check flashed espeak partition");
        heap_caps_free(arena);
        heap_caps_free(audio);
        arena = nullptr;
        audio = nullptr;
        vTaskDelete(nullptr);
        return;
    }
    // Allocated after the worker stack and G2P so they keep internal RAM first.
    {
        size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        size_t free_bytes = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        size_t budget = free_bytes > internal_reserve ? free_bytes - internal_reserve : 0;
        size_t res_bytes = std::min({res_arena_max, budget, largest > 1024 ? largest - 1024 : 0}) & ~size_t(15);
        if (res_bytes >= 16 * 1024)
            snt_port_res_arena = heap_caps_aligned_alloc(16, res_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        snt_port_res_cap = snt_port_res_arena ? res_bytes : 0;
        bool dual = snt_port_init();
        ESP_LOGI(TAG, "SIMD residency %u bytes (internal free %u, largest %u); dual-core %s",
                 static_cast<unsigned>(snt_port_res_cap),
                 static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
                 static_cast<unsigned>(largest), dual ? "on" : "off");
    }
    ESP_LOGI(TAG, "sanoTTS ready (PIE SIMD + dual-core); PCM %u Hz; stack %lu bytes",
             sample_rate, static_cast<unsigned long>(worker_stack_bytes));
    ESP_LOGI(TAG, "Voice: %s", SANOTTS_VOICE_NAME);
#if SANOTTS_VOICE_EXPERIMENTAL
    ESP_LOGW(TAG, "EXPERIMENTAL: voice has not passed quality/parity acceptance");
#endif
    Request request;
    int ids[256];
    int32_t model_ids[256];
    while (xQueueReceive(queue, &request, portMAX_DELAY) == pdTRUE)
    {
        int count = esp_g2p_text_to_ids(request.text, ids, 256);
        // IDF reports this in bytes. Keep the minimum visible on-device so
        // longer utterances and language-specific call paths can be checked.
        ESP_LOGI(TAG, "G2P tokens %d; stack minimum free %lu bytes", count,
                 static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr)));
        if (count < 3)
        {
            ESP_LOGW(TAG, "Rejected utterance: phonemization failed or too many tokens");
            continue;
        }
        audio_size = 0;
        // Xtensa defines int32_t as long, not int. Copy values rather than
        // aliasing the phonemizer's int buffer through an incompatible pointer.
        for (int i = 0; i < count; ++i) model_ids[i] = ids[i];
        snt_config cfg{front, model, arena, arena_bytes, nullptr};
        snt_stats stats{};
        int rc = snt_synthesize(&cfg, model_ids, count, collect_pcm, nullptr, &stats);
        if (rc != 0)
        {
            ESP_LOGW(TAG, "Synthesis rejected (%d); no partial audio played", rc);
            continue;
        }
        // Newlib nano printf has no %lld; report milliseconds as unsigned long.
        ESP_LOGI(TAG, "%d samples (%lu ms audio) synthesized in %lu ms; stack minimum free %lu bytes",
             stats.samples, static_cast<unsigned long>(stats.samples * 1000ULL / sample_rate),
             static_cast<unsigned long>(stats.elapsed_us / 1000),
             static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr)));
        // Whole-utterance buffering keeps playback independent of synthesis speed.
        // The sink blocks until the mixer releases this buffer.
        if (audio_size && output(audio, audio_size, output_user) != 0)
            ESP_LOGW(TAG, "Playback failed");
    }
}
}

esp_err_t flatsphere_sanotts_init(flatsphere_pcm_sink sink, void *user)
{
    if (!sink) return ESP_ERR_INVALID_ARG;
    if (queue) return ESP_ERR_INVALID_STATE;
    arena = heap_caps_aligned_alloc(16, arena_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    audio = static_cast<int16_t *>(heap_caps_malloc(pcm_capacity * sizeof(int16_t),
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    queue = xQueueCreate(4, sizeof(Request));
    output = sink;
    output_user = user;
    if (arena && audio && queue &&
        xTaskCreatePinnedToCore(worker, "sanotts", worker_stack_bytes, nullptr, 4, nullptr, 1) == pdPASS)
        return ESP_OK;
    heap_caps_free(arena);
    heap_caps_free(audio);
    if (queue) vQueueDelete(queue);
    arena = nullptr;
    audio = nullptr;
    queue = nullptr;
    return ESP_ERR_NO_MEM;
}

bool flatsphere_sanotts_say(const char *text)
{
    if (!queue || failed.load() || !text) return false;
    size_t length = strnlen(text, sizeof(Request::text));
    if (!length || length >= sizeof(Request::text)) return false;
    Request request{};
    memcpy(request.text, text, length + 1);
    return xQueueSend(queue, &request, 0) == pdTRUE;
}