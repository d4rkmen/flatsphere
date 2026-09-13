/* Project-local R7 host gate. Keep the firmware's limits and 2 MiB arena,
 * and compare ALL emitted PCM, not just its overlap with the reference.
 * The exported vectors use native little-endian int32/float32 on our hosts.
 * Only the public runtime API is needed, also allowing small stub-runtime tests.
 */
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "snt_tts.h"

enum {
    MAX_TOKENS = 256,
    MAX_FRAMES = 640,
    R7_VOCAB = 256,
    R7_HOP = 256,
    ARENA_BYTES = 2 * 1024 * 1024
};

/* No compiler alignment extension or C11 aligned_alloc required. */
static unsigned char arena_storage[ARENA_BYTES + 15];

static void *load_file(const char *dir, const char *name, size_t limit,
                       size_t *bytes) {
    FILE *file = NULL;
    void *data = NULL;
    char *path = NULL;
    size_t dir_len = strlen(dir), name_len = strlen(name);
    int ok = 0;
    *bytes = 0;
    if (dir_len > SIZE_MAX - name_len - 2) goto done;
    path = malloc(dir_len + name_len + 2);
    if (!path) goto done;
    memcpy(path, dir, dir_len);
    path[dir_len] = '/';
    memcpy(path + dir_len + 1, name, name_len + 1);
    file = fopen(path, "rb");
    if (!file || fseek(file, 0, SEEK_END) != 0) goto done;
    long length = ftell(file);
    if (length <= 0 || (uintmax_t)length > (uintmax_t)limit ||
        fseek(file, 0, SEEK_SET) != 0) goto done;
    size_t size = (size_t)length;
    data = malloc(size);
    if (!data || fread(data, 1, size, file) != size) goto done;
    /* Reject a changing/truncated file and report stream/close errors. */
    if (fgetc(file) != EOF || ferror(file)) goto done;
    *bytes = size;
    ok = 1;
done:
    if (file && fclose(file) != 0) ok = 0;
    if (!ok) {
        fprintf(stderr, "Cannot load nonempty %s (limit %zu bytes)\n",
                path ? path : name, limit);
        free(data);
        data = NULL;
        *bytes = 0;
    }
    free(path);
    return data;
}

typedef struct {
    const float *gold;
    size_t expected, emitted;
    int failed;
    double sa, sb, saa, sbb, sab;
} CorrSink;

static int corr_cb(const float *pcm, int n, void *user) {
    CorrSink *sink = (CorrSink *)user;
    if (sink->failed) return 1;
    if (n <= 0 || !pcm) {
        fprintf(stderr, "Invalid PCM callback buffer/count: %d\n", n);
        sink->failed = 1;
        return 1;
    }
    size_t start = sink->emitted;
    if ((size_t)n > SIZE_MAX - start) {
        fprintf(stderr, "PCM sample count overflow\n");
        sink->failed = 1;
        return 1;
    }
    sink->emitted += (size_t)n;
    if (sink->emitted > sink->expected) {
        fprintf(stderr, "Excess PCM: emitted %zu, expected %zu\n",
                sink->emitted, sink->expected);
        sink->failed = 1;
        return 1;
    }
    for (int i = 0; i < n; ++i) {
        double a = pcm[i], b = sink->gold[start + (size_t)i];
        if (!isfinite(a) || !isfinite(b)) {
            fprintf(stderr, "Non-finite audio at sample %zu\n", start + (size_t)i);
            sink->failed = 1;
            return 1;
        }
        sink->sa += a;
        sink->sb += b;
        sink->saa += a * a;
        sink->sbb += b * b;
        sink->sab += a * b;
    }
    return 0;
}

int main(int argc, char **argv) {
    void *front = NULL, *decoder = NULL;
    int32_t *ids = NULL, *durs = NULL;
    float *gold = NULL;
    size_t ids_bytes, durs_bytes, audio_bytes, blob_bytes;
    int pass = 0, frames = 0;
    const uint32_t endian = 1;
    if (argc != 2) {
        fprintf(stderr, "Usage: %s export-dir\n", argv[0]);
        goto done;
    }
    if (CHAR_BIT != 8 || sizeof(float) != 4 || sizeof(int32_t) != 4 ||
        *(const unsigned char *)&endian != 1) {
        fprintf(stderr, "Golden vectors require a little-endian float32/int32 host\n");
        goto done;
    }
    ids = load_file(argv[1], "e2e_ids.bin", MAX_TOKENS * sizeof(*ids), &ids_bytes);
    durs = load_file(argv[1], "e2e_durs.bin", MAX_TOKENS * sizeof(*durs), &durs_bytes);
    if (!ids || !durs) goto done;
    if (ids_bytes % sizeof(*ids) != 0 || durs_bytes != ids_bytes) {
        fprintf(stderr, "IDs/durations must have equal, nonzero int32 byte lengths\n");
        goto done;
    }
    int tokens = (int)(ids_bytes / sizeof(*ids));
    for (int i = 0; i < tokens; ++i) {
        if (ids[i] < 0 || ids[i] >= R7_VOCAB) {
            fprintf(stderr, "Token %d ID outside [0, %d)\n", i, R7_VOCAB);
            goto done;
        }
        /* Subtraction avoids overflow even for malicious int32 durations. */
        if (durs[i] < 1 || durs[i] > MAX_FRAMES - frames) {
            fprintf(stderr, "Duration %d must be positive; total frames must be <= %d\n",
                    i, MAX_FRAMES);
            goto done;
        }
        frames += durs[i];
    }
    size_t expected = (size_t)frames * R7_HOP;
    gold = load_file(argv[1], "e2e_audio.bin", expected * sizeof(*gold), &audio_bytes);
    if (!gold) goto done;
    if (audio_bytes != expected * sizeof(*gold)) {
        fprintf(stderr, "Golden audio length %zu bytes; expected %zu (%d frames x %d)\n",
                audio_bytes, expected * sizeof(*gold), frames, R7_HOP);
        goto done;
    }
    for (size_t i = 0; i < expected; ++i) {
        if (!isfinite(gold[i])) {
            fprintf(stderr, "Non-finite reference audio at sample %zu\n", i);
            goto done;
        }
    }
    front = load_file(argv[1], "front_q8.bin", SIZE_MAX, &blob_bytes);
    decoder = load_file(argv[1], "model_q8.bin", SIZE_MAX, &blob_bytes);
    if (!front || !decoder) goto done;

    void *arena = arena_storage + ((16 - (uintptr_t)arena_storage % 16) % 16);
    snt_config cfg = {front, decoder, arena, ARENA_BYTES, durs};
    snt_stats stats = {-1, -1, 0};
    CorrSink sink = {0};
    sink.gold = gold;
    sink.expected = expected;
    int rc = snt_synthesize(&cfg, ids, tokens, corr_cb, &sink, &stats);
    printf("frames %d samples %d elapsed %lld us; emitted %zu expected %zu; arena %d bytes\n",
           stats.frames, stats.samples, (long long)stats.elapsed_us,
           sink.emitted, expected, ARENA_BYTES);
    if (rc != 0 || sink.failed || sink.emitted != expected ||
        stats.frames != frames || stats.samples < 0 ||
        (size_t)stats.samples != sink.emitted) {
        fprintf(stderr, "Runtime/count gate failed: rc=%d callback_error=%d expected_frames=%d\n",
                rc, sink.failed, frames);
        goto done;
    }
    double n = (double)expected;
    double cov = sink.sab - sink.sa * sink.sb / n;
    double va = sink.saa - sink.sa * sink.sa / n;
    double vb = sink.sbb - sink.sb * sink.sb / n;
    if (!isfinite(cov) || !isfinite(va) || !isfinite(vb) || va <= 0 || vb <= 0) {
        fprintf(stderr, "Undefined correlation: non-finite or zero audio variance\n");
        goto done;
    }
    double corr = cov / sqrt(va * vb + 1e-30);
    double rms_ratio = sqrt((sink.saa + 1e-30) / (sink.sbb + 1e-30));
    printf("golden corr %.6f (%zu samples; required > 0.98)\n", corr, sink.emitted);
    printf("golden rms_ratio %.6f (bound 0.80-1.25, exclusive)\n", rms_ratio);
    pass = isfinite(corr) && isfinite(rms_ratio) &&
           corr > 0.98 && rms_ratio > 0.80 && rms_ratio < 1.25;
done:
    free(gold);
    free(durs);
    free(ids);
    free(decoder);
    free(front);
    puts(pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
