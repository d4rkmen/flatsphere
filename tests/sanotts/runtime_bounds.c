/* Host regression checks for the patched, pinned runtime (no ESP hardware). */
#include "snt_tts.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void *load(const char *dir, const char *name, size_t *size)
{
    char path[1024];
    snprintf(path, sizeof(path), "%s/%s", dir, name);
    FILE *f = fopen(path, "rb");
    assert(f);
    assert(fseek(f, 0, SEEK_END) == 0);
    long bytes = ftell(f);
    assert(bytes > 0);
    rewind(f);
    void *p = malloc((size_t)bytes);
    assert(p && fread(p, 1, (size_t)bytes, f) == (size_t)bytes);
    fclose(f);
    if (size) *size = (size_t)bytes;
    return p;
}

static int delivered;
static int sink(const float *pcm, int n, void *abort_playback)
{
    assert(n > 0);
    for (int i = 0; i < n; ++i) assert(isfinite(pcm[i]));
    delivered += n;
    return abort_playback != NULL;
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    size_t bytes;
    void *front = load(argv[1], "front_q8.bin", NULL);
    void *dec = load(argv[1], "model_q8.bin", NULL);
    int32_t *ids = load(argv[1], "e2e_ids.bin", &bytes);
    int count = (int)(bytes / sizeof(int32_t));
    int32_t *durations = load(argv[1], "e2e_durs.bin", NULL);
    void *arena = malloc(2 * 1024 * 1024);
    assert(arena);
    snt_config cfg = {front, dec, arena, 2 * 1024 * 1024, durations};
    snt_stats stats = {0};
    assert(snt_synthesize(NULL, ids, count, sink, NULL, &stats) == -1);
    assert(snt_synthesize(&cfg, ids, 257, sink, NULL, &stats) == -1);
    int32_t invalid = -1;
    assert(snt_synthesize(&cfg, &invalid, 1, sink, NULL, &stats) == -1);
    invalid = 157;
    assert(snt_synthesize(&cfg, &invalid, 1, sink, NULL, &stats) == -1);
    cfg.arena_size = 32;
    assert(snt_synthesize(&cfg, ids, count, sink, NULL, &stats) == -2);
    assert(delivered == 0);
    cfg.arena_size = 2 * 1024 * 1024;
    int32_t first_duration = durations[0];
    durations[0] = 641;
    assert(snt_synthesize(&cfg, ids, count, sink, NULL, &stats) == -3);
    durations[0] = 0;
    assert(snt_synthesize(&cfg, ids, count, sink, NULL, &stats) == -3);
    assert(delivered == 0);
    durations[0] = first_duration;
    assert(snt_synthesize(&cfg, ids, count, sink, &cfg, &stats) == 1);
    delivered = 0;
    assert(snt_synthesize(&cfg, ids, count, sink, NULL, &stats) == 0);
    assert(delivered == stats.samples && delivered > 0);
    cfg.dur_override = NULL;
    delivered = 0;
    assert(snt_synthesize(&cfg, ids, count, sink, NULL, &stats) == 0);
    assert(delivered == stats.samples && stats.frames <= 640);
    free(arena);
    free(durations);
    free(ids);
    free(dec);
    free(front);
    puts("PASS: bounds, arena recovery, cancellation, production durations");
}