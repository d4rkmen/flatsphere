# sanoTTS integration milestones

Branch: `feature/sanotts` · Updated: 2026-09-07

## Status

Experimental **English text-to-speech integration implemented and build-tested**.
Both sanoTTS and the PicoTTS fallback compile for ESP32-S3 with ESP-IDF 5.5.4.
No device has been flashed, and playback, latency, stack/heap headroom, and
long-running stability on the clock are **not yet verified**. Additional spoken
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