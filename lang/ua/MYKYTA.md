# Mykyta: 400k training to an installable MCU voice

Entry point: `mykyta_train.py`; defaults: `mykyta-400k.json`.
Run from the repository root with the existing `.venv`. No stage runs
implicitly, and this workflow never builds or flashes hardware automatically.

## Budget and prerequisites

- Duration: **100,000** updates, MPS, full 256-symbol vocabulary.
- LRC encoder + decoder: **400,000** updates, CPU, 40 × 10,000-step segments.
- Acoustic40: **400,000** updates, MPS, frame-weighted samples.
- All prepared Mykyta train/eval rows; the test split stays reserved.
- Existing `lang/ua/requirements.txt` dependencies and pinned sanoTTS checkout
  (default `build/_deps/sano_source-src`). CPU can be selected in the JSON
  before starting; changing configuration mid-run is refused.
- A sanoTTS firmware build must have generated `build/esp-idf/sanotts/generated/snt_tts.c`
  for the final C gate. Use the ESP-IDF extension to build, not this script.
- Significant disk space is needed for full teacher tensors, 40 decoder
  segments, acoustic snapshots and preserved interrupted attempts.

400k is a training budget, **not a guarantee of better speech or int8 parity**.
Duration remains at Tetiana's established 100k budget; set `duration_steps`
to 400000 before starting if that stage should also run for 400k.

## Stages

```sh
.venv/bin/python lang/ua/mykyta_train.py plan
.venv/bin/python lang/ua/mykyta_train.py prepare
/usr/bin/caffeinate -i .venv/bin/python -u lang/ua/mykyta_train.py train
```

`plan` is read-only. `prepare` fetches teacher/source data if needed, prepares
full packs, validates train/eval separation and extracts a pipeline-local
teacher decoder. Existing Mykyta smoke runs and Tetiana artifacts are untouched.

`train` runs duration/decoder smoke checks, duration, 40 decoder segments,
fresh encoder targets, acoustic smoke, acoustic training, learned-duration
evaluation WAVs, int8 export and the C parity gate. `smoke` can also be run
separately after `prepare`. Acoustic progress is in its run's `console.log`;
pipeline logs are under `work/mykyta/mykyta-400k-v1/logs/`.

Rerun the same `train` command after interruption. Completed artifacts and
input hashes are verified, interrupted outputs are archived, and progress is
continued from verified checkpoints. Decoder can lose the current unfinished
10k segment; duration saves every 1k; acoustic saves at 1k, 5k and every 10k.
**Recovery and decoder segment continuation restore weights only, resetting
optimizer/RNG state.** This is not equivalent to an uninterrupted Adam run.
Do not edit scripts/config/data while a run is in progress: provenance is bound
to these inputs. No concurrent pipeline process is permitted for the same name.

## Export and quality gates

```sh
.venv/bin/python lang/ua/mykyta_train.py export
```

This can retry export/evaluation independently after completed training.
The male exporter uses the selected Mykyta checkpoints, the Mykyta eval pack
and the duration checkpoint's actual normalization—not Tetiana constants.

The C gate uses the exported headers, a firmware-sized 2 MiB arena, finite
samples, exact sample counts, **correlation >0.98** and RMS ratio **(0.80,1.25)**.
It uses teacher durations, so learned-duration WAV review is also mandatory.
The existing Tetiana export measures ~0.942 correlation and fails this strict
gate; it is not weakened here. If Mykyta fails, training/export remains an
experiment and no flash-ready bundle is produced. More steps alone may not
correct quantization error; further model/calibration work may be required.

Listen to `work/mykyta/mykyta-400k-v1-synthesis/index.html` and compare all
teacher/student phrases for intelligibility, pronunciation, gain and clipping.
Only after listening:

```sh
.venv/bin/python lang/ua/mykyta_train.py accept --listening-approved
.venv/bin/python lang/ua/mykyta_train.py install
```

Acceptance verifies completed training and matching audition/export checkpoint
hashes, reruns C parity, and checks evaluated phrases fit the firmware's
191-byte request / 256-ID / 640-frame limits. The flag records human approval;
it does not perform an automatic listening test. Arbitrary untested text can
still exceed firmware limits.

The accepted bundle is `work/mykyta/mykyta-400k-v1/flash-ready/` and includes
blobs, headers, golden fixtures, provenance and acceptance metadata. Installation
copies to the separate `components/sanotts/voices/uk-mykyta/` slot, refusing
overwrites. It does not replace Tetiana or change `sdkconfig`.

## Build and device validation

In ESP-IDF menuconfig select **FlatSphere speech → Ukrainian → Mykyta**, then
build and flash through the extension. Blob alignment, header precedence,
Ukrainian eSpeak overlay and the 64 KiB worker stack are retained. Preserve NVS
settings when choosing flash partitions; an existing flashed Ukrainian eSpeak
image can be reused, while first-time Ukrainian setup needs that partition.

“Flash-ready” here means an accepted model bundle for compilation, **not an
already-built firmware image or device-validated release**. Check clock UI,
speech latency, pronunciation, clipping, heap and stack headroom on hardware.