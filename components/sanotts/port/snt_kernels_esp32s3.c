/* ESP32-S3 kernels: PIE SIMD on internal-SRAM operands, scalar reference
 * otherwise. PIE vector loads from flash/PSRAM return garbage, so both the
 * weight rows and the activation vector must be internal before dispatch. */
#include <stdint.h>
#include <string.h>
#include "esp_memory_utils.h"
#include "snt_port.h"

int32_t snt_ref_dot_s8(const int8_t *a, const int8_t *b, int len);
void snt_ref_matvec_s8(const int8_t *act, const int8_t *w, int32_t *out, int rows, int len);
int32_t snt_ref_dot_s16s8(const int16_t *a, const int8_t *b, int len);
void snt_ref_matvec_s16s8(const int16_t *act, const int8_t *w, int32_t *out, int rows, int len);

extern void sn_matvec_s8_c3(const int8_t *act, const int8_t *w, int32_t *out, int rows);
extern void sn_matvec_s8_c5(const int8_t *act, const int8_t *w, int32_t *out, int rows);
extern void sn_matvec_s8_g(const int8_t *act, const int8_t *w, int32_t *out, int rows, int chunks);
extern int snt_scratch_id(void);
extern volatile int snt_port_simd_enabled;

#define ACT_MAX 512
static int8_t act_stage[2][ACT_MAX] __attribute__((aligned(16)));

static inline int simd_ok(const void *p)
{
    return esp_ptr_internal(p) && ((uintptr_t)p & 15) == 0;
}

int snt_weights_resident(const void *p) { return simd_ok(p); }

int32_t snt_dot_s8(const int8_t *a, const int8_t *b, int len) { return snt_ref_dot_s8(a, b, len); }

void snt_matvec_s8(const int8_t *act, const int8_t *w, int32_t *out, int rows, int len)
{
    if (!snt_port_simd_enabled || rows < 1 || len < 16 || (len & 15) || !simd_ok(w))
    {
        snt_ref_matvec_s8(act, w, out, rows, len);
        return;
    }
    if (!simd_ok(act))
    {
        if (len > ACT_MAX)
        {
            snt_ref_matvec_s8(act, w, out, rows, len);
            return;
        }
        int8_t *staged = act_stage[snt_scratch_id()];
        memcpy(staged, act, (size_t)len);
        act = staged;
    }
    int chunks = len >> 4;
    if (chunks == 3) sn_matvec_s8_c3(act, w, out, rows);
    else if (chunks == 5) sn_matvec_s8_c5(act, w, out, rows);
    else sn_matvec_s8_g(act, w, out, rows, chunks);
}

int32_t snt_dot_s16s8(const int16_t *a, const int8_t *b, int len) { return snt_ref_dot_s16s8(a, b, len); }

void snt_matvec_s16s8(const int16_t *act, const int8_t *w, int32_t *out, int rows, int len)
{
    snt_ref_matvec_s16s8(act, w, out, rows, len);
}
