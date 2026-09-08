#include "snt_port.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Scalar kernels allow flash and PSRAM operands. Do not substitute S3 SIMD
 * without also implementing its internal-SRAM residency contract. */
void snt_par_run(snt_par_fn fn, int n, void *ctx)
{
    fn(0, n, ctx);
    /* Let the idle task/watchdog and the higher-priority mixer run. */
    static int64_t last_yield;
    int64_t now = esp_timer_get_time();
    if (now - last_yield >= 10000)
    {
        vTaskDelay(1);
        last_yield = esp_timer_get_time();
    }
}
int snt_scratch_id(void) { return 0; }
int64_t snt_now_us(void) { return esp_timer_get_time(); }