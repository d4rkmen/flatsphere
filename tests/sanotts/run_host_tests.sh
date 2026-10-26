#!/bin/sh
set -eu
# Entirely local host gate; no firmware configuration or downloads required.
root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
component="$root/components/sanotts"
mcu="$component/upstream"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT HUP INT TERM
cmake -DCOMPONENT="$component" -DGENERATED="$tmp" -P "$root/tests/sanotts/prepare_host.cmake"
runtime="$tmp/snt_tts.c"
voice="$component/voices/en-kristin"
for name in golden bounds; do
    test_source="$mcu/host/golden_main.c"
    if [ "$name" = bounds ]; then test_source="$root/tests/sanotts/runtime_bounds.c"; fi
    "${CC:-cc}" -O2 -std=c99 -DFSD_FAST_MATH \
        -I"$voice" -I"$mcu/include" \
        "$runtime" "$mcu/src/snt_kernels_ref.c" \
        "$mcu/host/snt_port_host.c" "$test_source" -lm -o "$tmp/$name"
    "$tmp/$name" "$voice"
done