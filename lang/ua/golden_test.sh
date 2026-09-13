#!/bin/sh
set -eu
# C golden gate for a Ukrainian (or any newly exported) R7 voice.
# Usage: lang/ua/golden_test.sh [export-dir [upstream-dir]]
# The export dir must contain front_q8.bin, model_q8.bin, e2e_{ids,durs,audio}.bin
# and a model/ subdirectory with the generated meta headers.
# upstream-dir is the sanoTTS checkout root (not its mcu/ directory).
# SNT_GOLDEN_RUNTIME may override the generated, firmware-patched runtime path.
if [ "$#" -gt 2 ]; then
    echo "Usage: $0 [export-dir [upstream-dir]]" >&2
    exit 1
fi
root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build="$root/build"
export_dir=${1:-"$root/lang/ua/work/tetiana/export-uk"}
upstream=${2:-"$build/_deps/sano_source-src"}
mcu="$upstream/mcu"
runtime=${SNT_GOLDEN_RUNTIME:-"$build/esp-idf/sanotts/generated/snt_tts.c"}
test -f "$runtime" || { echo "Configure/build the sanoTTS firmware first" >&2; exit 1; }
for header in fsd_meta.h fsd_q8_meta.h front_q8_meta.h frozen_norm.h chain_scales.h rb_scales.h; do
    test -f "$export_dir/model/$header" || {
        echo "Missing export meta header: $export_dir/model/$header" >&2
        exit 1
    }
done
tmp=$(mktemp -d)
trap 'rm -rf -- "$tmp"' 0
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM
# The generated patched runtime lives outside mcu/src, so its "model/..." includes
# fall through to -I"$export_dir" (our headers) before the English ones in mcu/src.
"${CC:-cc}" -O2 -std=c99 -DFSD_FAST_MATH \
    -I"$export_dir" -I"$mcu/include" -I"$mcu/src" \
    "$runtime" "$mcu/src/snt_kernels_ref.c" \
    "$mcu/ports/host/snt_port_host.c" "$root/lang/ua/voice_golden.c" -lm -o "$tmp/ua_golden"
"$tmp/ua_golden" "$export_dir"
