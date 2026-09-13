# sanoTTS integration milestones

Branch: `feature/sanotts` · Updated: 2026-09-08

## Ukrainian voice — export + firmware (2026-09-08)

- **int8 exporter implemented** (`lang/ua/export_int8.py`). The pinned upstream
  ships the C runtime and generated meta headers but not the exporter scripts
  (`export_front_q8.py` / `export_fsd_q8.py` / `export_fsd_golden.py`), so the
  byte format was reconstructed from the runtime's dequantization contract
  (`out[o] = s_act * sc[o] * dot(q_act, w8[o]) + bias[o]`; per-output-channel
  int8 weights `sc = max|W|/127`; conv weights laid out `[out, pad16(in*K)]`
  channel-major/kernel-minor; embeddings and decoder depthwise/norm stay fp32).
- **C golden gate** (`lang/ua/golden_test.sh`): the exported blobs reproduce the
  float PyTorch model. Decoder alone is **corr 0.9984 / rms 0.981** (Python code
  injected through the runtime's `SNT_INJECT_FRONT_PATH`). Full stack
  **corr 0.942 / rms 0.953**, below the 0.98 production gate — the residual is
  int8 quantization of the under-trained 150k-step acoustic model (its predicted
  40-code is corr 0.9998 vs float but the FSD FiLM conditioning amplifies the
  ~2% code error). Expected to improve as training converges; not a format bug.
- **Shared phoneme inventory confirmed**: the Piper `phoneme_id_map` is identical
  across English and Ukrainian (same 256-symbol IPA inventory, same IDs), so the
  on-device `CP_ID` table is reusable. espeak-ng "uk" IPA output (β, ɛ, ɪ, ɲ, ː,
  ˈ, ̪) is fully covered by the existing table.
- **Firmware integration**: `components/sanotts/voices/uk/` (int8 blobs + meta
  headers), `components/sanotts/espeak-uk/` (uk_dict + lang/zle/uk), Kconfig
  voice choice `FLATSPHERE_TTS_VOICE` (uk default), conditional espeak voice
  patch, and Ukrainian greetings/time/battery phrases with numeral agreement in
  `main.cpp`. Firmware builds: **0x276a80 bytes (2.58 MiB), 38% app free**.
- **Not yet flashed**; quality not accepted (model still under-trained).

## Follow-up: device feedback and Ukrainian voices

- **2026-09-07, user-reported:** sanoTTS works on the clock and sounds much
  better than PicoTTS. This confirms basic playback by user observation;
  measured latency, heap/stack headroom and long-run stress checks remain open.
- Ukrainian male/female preparation now lives in **`lang/ua`**. Public
  Mykyta/Tetiana recordings and matching Apache-2.0 Piper teachers were downloaded
  and checksum-verified. Both have deterministic corpus splits, 24 teacher
  listening samples, validated small teacher packs and successful 20-step
  MPS duration-training smoke runs. See `lang/ua/README.md` for counts and results.
- **Not complete:** Ukrainian sanoTTS acoustic/decoder training, int8 export and
  firmware integration. The pinned upstream repository lacks required R7
  acoustic/QAT/export tools. No Ukrainian voices were flashed; the working
  English firmware was not changed during this preparation.
- **Route A experiment completed:** Tetiana teacher decoder-cut parity passed
  (minimum cosine 0.9999998628); a 100-step R7-shaped LRC/FSD CPU run produced
  checkpoints and four held-out oracle WAVs. Added a hash-checked compatibility
  launcher and validated 40-channel target builder in `lang/ua/lrc_experiment.py`.
  Student oracle quality is **not accepted** (mean cosine 0.00980, low output
  RMS); these are execution tests, not flashable voices. Details and artifacts
  are in `lang/ua/README.md`.

## Status

### Tetiana larger training update (2026-09-07)

- **Long decoder training COMPLETED (200k steps):** `lang/ua/decoder-long.json`
  ran 20×10k weight-continuation segments on the full `duration-packs-full-v1`.
  Held-out chunk metrics jumped to **RMS ratio 0.936, cosine 0.852** (oracle
  render cosine 0.838), resolving the earlier amplitude problem. Final checkpoint
  in `lang/ua/work/tetiana/decoder-200000-cpu-v1/seg-00019/`. Summary:
  `lang/ua/experiments/tetiana-decoder-200k.json`.
- **Acoustic code targets regenerated** from the final decoder (2,969 train +
  167 eval) and **acoustic training STARTED**: 150k steps on MPS with periodic
  checkpoints via upstream `--checkpoint-steps` into
  `lang/ua/work/tetiana/acoustic-150000-mps-v1/`. Full-stack re-synthesis follows.
- **Full-stack audio is a real voice:** the `synthesize-v1` results were
  user-confirmed as good-sounding (not noise); the low RMS is a loudness-scaling
  artifact that the completed decoder training largely resolved.
- **Size budget confirmed:** the R7 architecture sizes to ~680 KB int8
  (duration ~40 KB + acoustic ~200 KB + decoder ~400 KB), matching the English
  Kristin voice and fitting the 2.6–3 MB budget with no size reduction.
  17 regression tests pass; the acoustic job is the only active training run.

- **Long duration run completed:** 100k MPS steps on 2,644/145 rows
  (2,969/167 chunks). Held-out token MAE **0.801**, token exact **54.1%**,
  frame ratio **0.946** (train 0.957, so no significant overfitting).
  Checkpoints saved every 1k steps; final model in
  `lang/ua/work/tetiana/duration-100000-mps-v1/`.
- **Full-stack synthesis works end-to-end:** `lang/ua/synthesize.py` maps text
  through learned duration → acoustic → decoder and produced 24 clock-phrase
  WAVs in `lang/ua/work/tetiana/synthesize-v1/`. Learned durations closely track
  the teacher (mean 2.466s vs 2.393s), but student amplitude is ~10× too low
  (RMS ~0.013–0.019 vs teacher ~0.13–0.20). The decoder is the amplitude
  bottleneck. Summary: `lang/ua/experiments/tetiana-fullstack-v1.json`.

- **Acoustic training now executes:** repaired only the non-QAT import path in
  a hash-checked in-memory launcher (`lang/ua/acoustic_experiment.py`). Generated
  282/37 aligned code targets and trained a 204,288-parameter acoustic40 model
  for 1,000 CPU steps, with full 256-entry vocabulary. Held-out code cosine
  **0.78099**, L1 **0.14251**. Four paired oracle/predicted-code WAVs rendered
  with teacher durations. This remains an experiment on an unapproved decoder,
  **not** learned-duration TTS or a flashable voice. Summary:
  `lang/ua/experiments/tetiana-acoustic-v1.json`; 12 regression tests pass.

- Prepared validated **256 training / 32 held-out** teacher rows (282/37 chunks).
- Completed two 3,000-step R7 LRC/FSD CPU experiments. Increasing spectral loss
  weight 0.5 → 5 improved held-out RMS ratio **0.280 → 0.504**, but waveform
  L1/cosine did not improve. Neither candidate has listening/quality approval.
- Preserved all smoke artifacts and added named pack/run controls and full
  held-out evaluation. Tracked results: `lang/ua/experiments/tetiana-decoder-v1.json`.
- Tetiana is **not flashable yet**; complete acoustic training, calibrated int8
  export and C validation remain. Per the user's sequencing, multilingual
  FlatSphere text production will be implemented after the voice is flashable,
  followed by on-device testing. Working English firmware remains unchanged.

Experimental **English text-to-speech integration implemented and build-tested**.
Both sanoTTS and the PicoTTS fallback compile for ESP32-S3 with ESP-IDF 5.5.4.
No device was flashed by the assistant. Basic playback is now user-confirmed;
latency, stack/heap headroom, and long-running stability are **not yet measured**. Additional spoken
languages are a follow-up, not a completed feature.

## Completed milestones

- [x] **M1 — Audit and isolated branch.** Created `feature/sanotts` from `main`.
  Inspected existing greeting/time/battery announcements, speaker mixer, PSRAM,
  flash layout, and upstream MCU setup. Existing local `sdkconfig` and
  `dependencies.lock` edits were not reset. ESP-IDF regenerated configuration
  during validation; review those files separately before committing.
- [x] **M2 — Reproducible upstream integration.** Added `components/sanotts`.
  CMake downloads immutable, SHA-256-verified source archives into `build/_deps`:
  sanoTTS `939d982b9faa54cbcf5d24cc878f5cd514b2646e`, espeak-ng 1.52.0
  `4870adfa25b1a32b4361592f1be8a40337c58d6c`. Includes the matching Kristin r7
  model headers, phoneme mapping, and two embedded int8 blobs (679,832 bytes).
  No manual edits inside `managed_components` are required.
- [x] **M3 — On-device phonemizer and data image.** Integrates upstream espeak
  translator sources and English data. A build-local patch accepts SPIFFS's
  flat directory layout; checked patch anchors fail configuration on drift.
  G2P rejects token overflow rather than silently truncating, always reserves
  EOS space, and avoids backward scanning before the input buffer.
- [x] **M4 — Bounded speech worker.** Four copied requests, maximum 191 UTF-8
  bytes each, nonblocking enqueue, core 1 / priority 4 / 24 KiB stack. G2P
  initializes on that worker. One serialized model instance uses a 2 MiB
  PSRAM arena and up to 327,680 bytes of PCM storage. Input is limited to 256
  phoneme IDs and 640 latent frames (about 7.43 seconds at 22,050 Hz).
  Invalid input, arena exhaustion, overlong audio, and nonfinite PCM are rejected;
  failed synthesis does not play partial utterances. Initialization failures
  leave the clock UI running rather than aborting setup.
- [x] **M5 — Clock/audio integration.** Boot greetings, time, and battery speech
  use a selectable backend. sanoTTS supplies 22,050 Hz mono signed 16-bit PCM
  to the existing channel-1 mixer, retaining resampling, volume and button tones.
  Whole-utterance buffering avoids depending on scalar synthesis keeping up
  with playback. The blocking playback bridge retains PCM until consumption;
  it also fixes Pico's reused-callback-buffer lifetime issue. Channel 1 is
  reserved from automatic allocation. Fixed mixer startup ordering, task-handle
  cleanup, and byte-offset arithmetic on partial I2S writes.
- [x] **M6 — Firmware build and host gates.** Both backend configurations built
  successfully. sanoTTS application: `0x26deb0` bytes (2.43 MiB), 39% of the
  4 MiB app partition free. Pico fallback: `0x307da0` bytes, 24% free.
  Patched runtime passes upstream's golden correlation/RMS gate:
  **correlation 0.989148**, **RMS ratio 0.935022**, 134 frames / 34,304 samples.
  Added `tests/sanotts/run_host_tests.sh` and `runtime_bounds.c`: invalid IDs,
  token limit, arena-OOM recovery, frame bounds, callback cancellation,
  recovery synthesis, and predicted-duration synthesis all passed.
  `git diff --check` passed. Existing LVGL enum/initializer warnings remain;
  legacy espeak warnings follow the upstream port's source-scoped `-w` policy.

## Build and configuration

1. Use the project's existing ESP-IDF setup (validated with 5.5.4).
2. In **ESP-IDF SDK Configuration Editor → FlatSphere speech**, select sanoTTS
   or PicoTTS. This branch defaults to sanoTTS on ESP32-S3 with PSRAM; the final
   checked working configuration selects sanoTTS. Pico remains a managed build
   dependency for fallback, but its engine/resources are not linked when unused.
3. Build through the ESP-IDF extension. Initial sanoTTS configuration requires
   network access for both pinned archives; subsequent builds reuse the cache.
   CMake validates download hashes and applies patches only to generated copies.
4. `sdkconfig.sanotts.defaults` is an optional **overlay**, not a replacement for
   all display/LVGL/hardware settings. For a separate build directory, preserve
   the existing board configuration and apply the overlay using ESP-IDF's
   `SDKCONFIG` / `SDKCONFIG_DEFAULTS` settings.
5. The host tests can be run with `sh tests/sanotts/run_host_tests.sh` after
   configuring sanoTTS; optionally pass the absolute build directory as argument.
   These are CPU-host tests, not an ESP32 timing or I2S test.

### Flash layout and first device test

Existing NVS (`0x9000`, 24 KiB) and application (`0x10000`, 4 MiB) offsets are
unchanged. `partitions.csv` adds **espeak SPIFFS at `0x410000`, size 1 MiB**;
the resulting table fits the configured 8 MiB flash. The sanoTTS build generates
`espeak.bin` and adds it to project flash arguments. Pico builds do not generate
or flash this image.

An app-only flash is **not enough for first installation**: the device also needs
the new partition table and the generated espeak image. Back up device settings
before flashing. This project's full flash also includes the generated NVS image
from `nvs.csv`, which can overwrite saved settings. Review the extension's flash
arguments and deliberately choose the required partitions; no flash or erase was
performed during this integration.

## Remaining milestones / acceptance checks

- [ ] **M7 — Hardware acceptance.** Flash the required images after reviewing
  NVS preservation. Verify `English Kristin ready` in logs, all eight greetings,
  midnight/noon, quarter/half past/to, numeric time fallback and battery percent.
  Check intelligibility, gain/clipping, gaps, button-tone mixing, UI responsiveness,
  queue-full behavior, missing data, long requests and repeated announcements.
  Measure PSRAM/internal free heap, largest block, worker stack high-water mark,
  watchdog margin, synthesis latency and long-run stability.
- [ ] **M8 — Performance/quality tuning.** This integration intentionally uses
  **scalar reference kernels**, PSRAM and a single synthesis worker. Upstream's
  optimized ESP32-S3 real-time numbers do not apply to this build. SIMD requires
  proven internal-SRAM residency and a fresh golden gate; do not run PIE loads
  directly from flash/PSRAM. Voice-specific sibilant calibration is not enabled.
  Pico's newly blocking small callbacks also need listening tests for chunk gaps.
- [ ] **M9 — Additional languages/voices.** Select target languages, translate
  announcements, obtain or train/export a matching MCU-compatible model, supply
  its phoneme vocabulary and matching espeak language data, and run numerical and
  listening gates. The shipped r7 model and its C headers are English-specific.
  Desktop/browser Vietnamese, Hindi, Nepali, Indonesian and Chinese packages
  cannot simply replace these binary blobs; some use different model graphs.
  Add a versioned voice/profile interface, storage budget and settings/UI only
  once a second compatible voice is validated. Do not expose nonworking language
  choices or assume changing espeak voice translates English clock phrases.
- [ ] **M10 — Distribution/license review and release.** Complete the notices,
  corresponding-source obligations and model-license review before distributing
  firmware. Commit the reviewed changes and push the feature branch when ready;
  neither commit nor push was performed here.

## License boundary and references

Upstream `mcu/src/snt_*.c` and `mcu/include/` inference runtime is MIT under
[`LICENSE.MIT`](https://github.com/Ampixa/sanoTTS/blob/939d982b9faa54cbcf5d24cc878f5cd514b2646e/LICENSE.MIT).
Its port/G2P layers and other project material are GPL-3.0-or-later under
[`LICENSE`](https://github.com/Ampixa/sanoTTS/blob/939d982b9faa54cbcf5d24cc878f5cd514b2646e/LICENSE);
espeak-ng is also GPLv3. Including on-device espeak means the linked firmware is
not an Apache-only distribution. Retain upstream notices and provide the required
corresponding source/build material when distributing. Existing project source
license headers have not been rewritten. Check voice/model terms separately.

- [sanoTTS README](https://github.com/Ampixa/sanoTTS)
- [ESP32-S3 port](https://github.com/Ampixa/sanoTTS/tree/939d982b9faa54cbcf5d24cc878f5cd514b2646e/mcu/ports/esp32s3)
- [espeak component setup](https://github.com/Ampixa/sanoTTS/tree/939d982b9faa54cbcf5d24cc878f5cd514b2646e/mcu/ports/esp32s3/firmware/components/espeak-ng)
- [New-language recipe](https://github.com/Ampixa/sanoTTS/blob/939d982b9faa54cbcf5d24cc878f5cd514b2646e/docs/roota-language-porting-recipe.md)