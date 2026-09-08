#!/bin/sh
set -eu
# Run after an ESP-IDF sanoTTS configure/build has staged pinned sources.
root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build=${1:-"$root/build"}
mcu="$build/_deps/sano_source-src/mcu"
runtime="$build/esp-idf/sanotts/generated/snt_tts.c"
test -f "$runtime" || { echo "Configure the sanoTTS firmware first" >&2; exit 1; }
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT HUP INT TERM
for name in golden bounds; do
    test_source="$mcu/test/golden_main.c"
    if [ "$name" = bounds ]; then test_source="$root/tests/sanotts/runtime_bounds.c"; fi
    "${CC:-cc}" -O2 -std=c99 -DFSD_FAST_MATH \
        -I"$mcu/include" -I"$mcu/src" \
        "$runtime" "$mcu/src/snt_kernels_ref.c" \
        "$mcu/ports/host/snt_port_host.c" "$test_source" -lm -o "$tmp/$name"
    "$tmp/$name" "$mcu/test/fixtures/en_us_r7"
done