#include "snt_port.h"
#include <stdbool.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Two-core split for snt_par_run: the synthesis task (core 1) runs [0, mid)
 * while a worker pinned to core 0 runs [mid, n). Call sites are
 * column-disjoint, so results are identical to the serial path. */
static volatile snt_par_fn par_fn;
static void *volatile par_ctx;
static volatile int par_lo, par_hi;
static volatile int par_done;
static TaskHandle_t par_task;

/* Internal-SRAM block for resident weight copies (see runtime.cmake patch). */
void *snt_port_res_arena;
size_t snt_port_res_cap;
volatile int snt_port_simd_enabled = 1;

static void par_worker(void *arg)
{
    (void)arg;
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        __sync_synchronize();
        par_fn(par_lo, par_hi, par_ctx);
        __sync_synchronize();
        par_done = 1;
    }
}

bool snt_port_init(void)
{
    if (par_task) return true;
    return xTaskCreatePinnedToCore(par_worker, "sanotts_par", 4096, NULL, 4, &par_task, 0) == pdPASS;
}

static void yield_periodically(void)
{
    /* Let the idle task/watchdog and the higher-priority mixer run. */
    static int64_t last_yield;
    int64_t now = esp_timer_get_time();
    if (now - last_yield >= 10000)
    {
        vTaskDelay(1);
        last_yield = esp_timer_get_time();
    }
}

void snt_par_run(snt_par_fn fn, int n, void *ctx)
{
    if (!par_task || n < 8)
    {
        fn(0, n, ctx);
    }
    else
    {
        int mid = n / 2;
        par_fn = fn;
        par_ctx = ctx;
        par_lo = mid;
        par_hi = n;
        par_done = 0;
        __sync_synchronize();
        xTaskNotifyGive(par_task);
        fn(0, mid, ctx);
        /* Workers block while idle; only the caller spins for the short tail. */
        while (!par_done) { }
        __sync_synchronize();
    }
    yield_periodically();
}

int snt_scratch_id(void) { return (par_task && xTaskGetCurrentTaskHandle() == par_task) ? 1 : 0; }
int64_t snt_now_us(void) { return esp_timer_get_time(); }
