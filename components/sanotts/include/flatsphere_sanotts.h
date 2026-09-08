#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Called on the speech worker. PCM is mono signed 16-bit at 22050 Hz.
 * The sink must consume/copy all samples before returning. */
typedef int (*flatsphere_pcm_sink)(const int16_t *, size_t, void *);
esp_err_t flatsphere_sanotts_init(flatsphere_pcm_sink sink, void *user);
/* Nonblocking, copies one NUL-terminated UTF-8 utterance (max 191 bytes).
 * Returns false when unavailable, too long, or the four-entry queue is full. */
bool flatsphere_sanotts_say(const char *text);

#ifdef __cplusplus
}
#endif