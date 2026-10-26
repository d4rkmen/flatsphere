/* Execute the production adapter + pinned engine/data with only VFS mounting
 * stubbed. The linker redirects InitializePath to a temporary host data path. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_spiffs.h"
#include "esp_g2p.h"
#include "espeak-ng/espeak_ng.h"
#include "espeak-ng/speak_lib.h"

static const char *data_path;
/* The build renames only the adapter's InitializePath call. */
void host_initialize_path(const char *ignored) {
    (void)ignored;
    espeak_ng_InitializePath(data_path);
}
esp_err_t esp_vfs_spiffs_register(const esp_vfs_spiffs_conf_t *conf) {
    assert(!conf->format_if_mount_failed);
    return ESP_OK;
}
esp_err_t esp_spiffs_info(const char *partition, size_t *total, size_t *used) {
    (void)partition;
    *total = *used = 0;
    return ESP_OK;
}
int main(int argc, char **argv) {
    assert(argc == 3);
    data_path = argv[1];
    int ids[258];
    assert(esp_g2p_text_to_ids("hello", ids, 256) == -1);
    assert(esp_g2p_init() == 0);
    const espeak_VOICE *voice = espeak_GetCurrentVoice();
    assert(voice && voice->languages);
    /* languages starts with a priority byte, then the actual language code. */
    assert(strncmp(voice->languages + 1, argv[2], 2) == 0);
    assert(esp_g2p_text_to_ids(NULL, ids, 256) == -1);
    assert(esp_g2p_text_to_ids("hello", NULL, 256) == -1);
    assert(esp_g2p_text_to_ids("hello", ids, 2) == -1);
    assert(esp_g2p_text_to_ids("", ids, 3) == 3);
    const char *text = strcmp(argv[2], "uk") == 0
        ? "Привіт! Зараз дванадцята година. Заряд батареї п'ятдесят відсотків."
        : "Hello! The time is twelve. Battery charge is fifty percent.";
    int n = esp_g2p_text_to_ids(text, ids, 256);
    assert(n > 3 && n <= 256 && ids[0] == 1 && ids[n - 1] == 2);
    for (int i = 0; i < n; ++i) assert(ids[i] >= 0 && ids[i] < 256);
    printf("PASS %s G2P: %d IDs:", argv[2], n);
    for (int i = 0; i < n; ++i) printf(" %d", ids[i]);
    puts("");
    ids[0] = 123456; ids[5] = 654321;
    assert(esp_g2p_text_to_ids(text, ids + 1, 4) == -1);
    assert(ids[0] == 123456 && ids[5] == 654321);
    n = esp_g2p_text_to_ids(text, ids, 256);
    assert(n > 3 && ids[n - 1] == 2);
    return 0;
}