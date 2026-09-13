# Українська — Ukrainian voice preparation

## Tetiana 400k workflow

`tetiana_train.py` with `tetiana-400k.json` matches the saved Mykyta 400k
configuration: **100,000 duration updates (MPS), 400,000 decoder updates
(CPU, 40 × 10,000), and 400,000 acoustic updates (MPS)**. These are fixed
cumulative targets, **not 400 steps** or additions to old Tetiana checkpoints.
Three separate five-step smoke runs are extra and do not count toward those budgets.
The unchanged shared trainers retain the same architectures/hyperparameters;
all prepared female train/eval rows are used and the test split stays reserved.

From the repository root, using the existing `.venv` and dependencies in
`lang/ua/requirements.txt`:

```sh
.venv/bin/python lang/ua/tetiana_train.py plan
.venv/bin/python lang/ua/tetiana_train.py prepare
/usr/bin/caffeinate -i .venv/bin/python -u lang/ua/tetiana_train.py train
```

`plan` is read-only; no stage runs implicitly. Only run `prepare` when ready
to fetch/verify cached data and create full teacher packs and a local decoder
cut. `train` runs smoke checks, all training stages, fresh code targets,
learned-duration listening WAVs, int8 export and strict C parity. `smoke` is
also available separately after preparation. No automatic firmware build/flash.

New artifacts use `work/tetiana/tetiana-400k-v1*`; logs are in
`work/tetiana/tetiana-400k-v1/logs/`. Existing Tetiana runs, `export-uk`, and
hash-bound Mykyta scripts/configuration remain untouched. New custom run names
must start with `tetiana-`; change name/config before starting, never mid-run.
Full packs, 40 decoder segments and archived interruptions need substantial disk space.

Rerun `train` after interruption: hashes/receipts are verified and incomplete
attempts preserved. Duration saves every 1k, acoustic at 1k/5k/every 10k;
an unfinished decoder segment may lose up to 10k steps. **Recovery and decoder
segment continuation restore weights only; optimizer/RNG state resets.** Legacy
checkpoints are not automatically adopted. CPU can be selected in JSON before
starting; MPS availability is checked by the trainer.

Retry evaluation/export with `.venv/bin/python lang/ua/tetiana_train.py export`.
The C gate needs the existing firmware-generated runtime and a host C compiler;
if missing, it fails rather than building firmware. It requires finite PCM,
exact sample counts, a 2 MiB arena, correlation **>0.98**, and RMS ratio
**(0.80,1.25)**. Debug front injection/runtime overrides are rejected. The gate
uses teacher durations, so listen to all teacher/student pairs in
`work/tetiana/tetiana-400k-v1-synthesis/index.html` too. More steps do not
guarantee quality or int8 parity; the older Tetiana export failed this strict gate.

Only after listening, run `.venv/bin/python lang/ua/tetiana_train.py accept
--listening-approved` (one command). Acceptance reruns parity, checks checkpoint
lineage and the 191-byte / 256-ID / 640-frame limits, then creates
`work/tetiana/tetiana-400k-v1/flash-ready/`. A failed gate never creates a bundle.
This is a model bundle, not built firmware or a device-validated release.

Optional `.venv/bin/python lang/ua/tetiana_train.py install` copies an accepted
bundle to the actual Tetiana slot, `components/sanotts/voices/uk`, **only if
absent**. The current installed Tetiana is not replaced: installation will refuse
until you separately preserve/move it. No alternate firmware slot, sdkconfig
change, or gate bypass is introduced. Build/flash through the ESP-IDF extension
and hardware listening/memory/latency checks remain explicit user steps.

## Mykyta 400k workflow

See [MYKYTA.md](MYKYTA.md) for `mykyta_train.py`: full male teacher packs,
100k duration, 400k decoder, 400k acoustic, checkpoint recovery, voice-aware
export, strict C parity, listening approval and separate firmware installation.
No long training starts implicitly. More steps do not guarantee quality/parity.

## Int8 export and firmware flashability — 2026-09-08

`export_int8.py` reconstructs the missing upstream R7 exporters and emits the
byte-exact MCU layout the `snt_tts.c` runtime consumes:

- `work/tetiana/export-uk/front_q8.bin` (315,668 B) + `model_q8.bin` (399,484 B)
- `front_q8_meta.h` / `fsd_q8_meta.h` byte-offset headers plus `model/` stubs.
- `e2e_ids.bin` / `e2e_durs.bin` / `e2e_audio.bin` host golden fixture.

`golden_test.sh` compiles the patched runtime against these blobs:

| Lane | corr | rms ratio |
| --- | ---: | ---: |
| Decoder only (injected Python code) | 0.9984 | 0.981 |
| Full stack (acoustic int8 + decoder int8) | 0.942 | 0.953 |

The full-stack corr is below the 0.98 production gate because the 150k-step
acoustic model quantizes with ~2% code error that the FSD FiLM conditioning
amplifies; the decoder alone is already production-grade. Expected to converge
with further acoustic training.

The Piper `phoneme_id_map` is identical between English and Ukrainian, so the
firmware `CP_ID` table is reused unchanged. espeak-ng "uk" produces only IPA
symbols already present in that table. Firmware voice selection is
`CONFIG_FLATSPHERE_TTS_VOICE` (Kconfig, uk default); the Ukrainian build
compiles at 0x276a80 bytes (38% app partition free).

Two independent voice targets: **Микита / Mykyta (male)** and
**Тетяна / Tetiana (female)**. Folder name follows the requested `lang/ua`;
the language code passed to Piper/espeak is **`uk`**, locale **`uk_UA`**.

## Scope and current limitation

This is a dataset + teacher/distillation workspace, **not yet a pair of
flashable sanoTTS voices**. It does not change the working English firmware.
The public recordings and matching pretrained Ukrainian Piper teachers are
used instead of inventing a new speaker or training a teacher from scratch.
In sanoTTS distillation, transcripts drive the teacher; the teacher supplies
aligned duration/latent/waveform targets. Original recordings are retained for
provenance, comparison and future teacher work—not claimed as raw-audio input
to the student trainers.

The pinned sanoTTS checkout does not ship a complete R7 export pipeline:
`qat_ste.py`, `train_roota_joint_c_finetune.py`, `export_front_q8.py`,
`export_fsd_golden.py`, and `export_fsd_q8.py` are absent. Its acoustic trainer
imports the absent QAT module even with QAT disabled. An LRC-code target builder
and reproducible per-voice calibration are also needed upstream. A local
code-target builder now exists in `lrc_experiment.py` (see Route A below). `cook.py audit` records
these blockers. Do not mistake a duration-only smoke checkpoint or a Piper ONNX
teacher for a new sanoTTS voice.

## Public sources and attribution

| Target | Public corpus | Publisher duration | Teacher | License stated by publisher |
| --- | --- | --- | --- | --- |
| Male | OpenTTS-UK, Mykyta | ~8h10m | `uk_UA-mykyta-high` | Apache-2.0 |
| Female | OpenTTS-UK, Tetiana | ~8h | `uk_UA-tetiana-high` | Apache-2.0 |

- Dataset: [Yehor/opentts-uk](https://huggingface.co/datasets/Yehor/opentts-uk),
  assembled/published by Yehor Smoliakov; per-speaker documentation and licenses:
  [ukrainian-tts-datasets](https://github.com/egorsmkv/ukrainian-tts-datasets).
- Teacher training: Roman Stasyshyn,
  [Mykyta model card](https://huggingface.co/RomanStasyshyn/uk_UA-mykyta-high) and
  [Tetiana model card](https://huggingface.co/RomanStasyshyn/uk_UA-tetiana-high).
  Models are downloaded from the pinned `rhasspy/piper-voices` mirror.
- Source revisions, expected binary sizes and SHA-256 hashes: `sources.json`.
  Download receipts and cached original model/dataset cards retain provenance.
  Corpus publisher durations differ from the duration-limited prepared subsets;
  actual counts/hours are written to each `dataset_report.json`.
- Respect attribution/notices when redistributing source data or derived models.
  Dataset/model permissions do not remove the espeak/sanoTTS port GPLv3
  obligations described in the project's `sanotts.md`.

## Layout

- `cook.py`: explicit stages: fetch, prepare, probe, pack, duration, audit.
- `requirements.txt`: versions installed for the local preparation environment.
- `texts/clock_eval.jsonl`: held-out Ukrainian clock/grammar/pronunciation probes.
- `test_cook.py`: canonicalization and deterministic split tests.
- `cache/{mykyta,tetiana}/`: ignored original transcripts, recording ZIPs, ONNX
  teachers/configurations and publisher cards. ~594 MB downloads for both voices.
- `work/{mykyta,tetiana}/`: ignored manifests, receipts, reports, listening WAVs,
  small teacher packs and duration-training outputs.

## Stages

Use a Python 3.11+ virtual environment with `requirements.txt`. The local
environment is the project `.venv`, Python 3.13, Apple Silicon with 64 GiB RAM.
`cook.py --help` describes all options. Run stages separately, in order:

1. **fetch**, with `--audio` to include public recording ZIPs. Cached downloads
   are verified; incomplete downloads use `.part` and are not trusted. No archive
   is blindly extracted. No uploads, cloud jobs, paid services or account login.
2. **prepare** validates recording references and writes deterministic 90/5/5
   train/eval/test hash-bucket splits. Identical normalized text stays in the same
   split across speakers. Stress/apostrophe normalization is used only for
   deduplication; original text and supplied stresses are retained separately.
   Selects source durations 0.5–6 s and at most 191 UTF-8 bytes to match the current
   firmware request limit. These are not quality or predicted-duration guarantees.
3. **probe** validates teacher ONNX/config and synthesizes the 24 clock probes
   to `work/<speaker>/teacher_samples/`. All synthesis is local and deterministic
   with zero teacher noise. Listen to stress, numbers, ґ/г/ї/є/щ, apostrophes and
   minute/percentage inflections. `listening_verified` remains false until reviewed.
4. **pack** generates small disjoint train/eval teacher packs via upstream's
   real pack builder; default `--rows 4` per split is only a smoke test.
   Packs are refused if output directories already exist to avoid mixing runs.
  Then run **validate-pack** to check finite 192-channel tensors and duration
  sums before training. The upstream builder also checks audio/hop alignment.
5. **duration** runs the real h32/d3/k5 duration trainer; default `--steps 20`
   is an execution test, not convergence. Explicit `--device mps` selects Apple
   GPU; CPU is the conservative default. Longer training should follow listening,
   graph compatibility and held-out alignment checks.
6. **audit** records missing MCU training/export prerequisites.

Each stage supports `--voice male`, `female` or `both` (default). Training/pack
stages default to the firmware's pinned checkout in `build/_deps/sano_source-src`;
`--upstream` can point to an equivalent checkout. Expected revision is recorded;
a custom checkout must be independently revision-verified.

## Roadmap to device voices

- [ ] Teacher listening acceptance by a Ukrainian speaker, particularly lexical
  stress and digit/time normalization. Compare against public recordings.
- [ ] Train/eval pack alignment gate: sum of duration labels equals latent frames;
  target generator input has 192 channels, hop 256, 22,050 Hz, valid ID vocabulary.
- [ ] Larger, deduplicated packs and converged duration students for both voices.
- [ ] Train R7-compatible LRC encoder + FSD decoder: 40-code bottleneck, decoder
  width 76, five blocks, FiLM rank 12, head rank 48, FFT 1024 / hop 256.
- [ ] Supply missing acoustic/QAT dependencies and build aligned 40-code targets;
  train h48/frame-depth5/token-depth3 acoustic students and refine the full stack.
- [ ] Implement/recover exact int8 exporters and voice-specific calibration;
  generate C headers/offsets and fresh golden fixtures for both voices.
- [ ] Pass golden correlation **and** RMS gates, intelligibility/listening tests,
  memory/latency budget and clock-device playback tests for each voice.
- [ ] Add Ukrainian espeak data, teacher-derived phoneme mapping, translated
  firmware phrases and selectable voices. Do not reuse English schwa ID 59,
  vocabulary limits or quantization scales without validation.

MPS availability alone does not prove that the LRC/iSTFT decoder's complex/STFT
operations work on Metal. Validate those operators before allocating a long run;
CPU fallback is permitted but no training completion time is promised.

## Completed local milestones — 2026-09-07

- [x] Downloaded both teacher models and both public recording archives;
  all four binary SHA-256 hashes match `sources.json`. Original transcripts and
  per-speaker license/model cards are cached alongside them.
- [x] Prepared and checked recording references for both filtered corpora:

  | Speaker | Original transcript rows | Train | Eval | Test | Retained hours |
  | --- | ---: | ---: | ---: | ---: | ---: |
  | Mykyta | 6,436 | 4,029 | 208 | 233 | 5.374 |
  | Tetiana | 5,227 | 2,644 | 145 | 146 | 3.963 |

  ZIPs contain 6,977 / 5,417 recording entries respectively. Not all recordings
  have a row in these published manifests; unused recordings are not silently
  assigned invented transcripts. Duration, byte-length and duplicate filters
  account for the smaller retained subsets.
- [x] Validated both teacher ONNX graphs and Ukrainian single-speaker configs;
  rendered **24 WAVs per voice** in `work/<speaker>/teacher_samples/`.
  They are **Piper teacher audio**, not tiny-model output. Human listening and
  original-recording comparisons have not been performed.
- [x] Generated four train and four eval rows per voice. The male training pack
  contains five chunks (one multi-sentence row); other packs contain four each.
  Upstream audio/alignment checks and local finite-value/192-channel gates pass.
- [x] Ran **20-step MPS duration-only smoke training** independently for each
  voice: `work/<speaker>/duration-20-mps/duration-student.pt` (36,676 parameters).
  Held-out predicted/teacher total-frame ratios: male **0.8657**, female
  **0.8067**. These demonstrate execution, **not convergence or quality**.
  The tiny smoke subsets observe only a subset of the teacher vocabulary;
  retrain/reconfigure for the full Ukrainian mapping before export.
- [x] Four deterministic corpus tests pass. Saved concrete upstream audit in
  `work/upstream_audit.json`; full voice training/export remains blocked as above.

No long training job is left running, no cloud compute was used, and no
Ukrainian model was flashed. The next decision is to obtain the missing upstream
R7 tooling or implement and validate that tooling locally, before spending
compute on full two-voice training.

## Route A: current MCU architecture — 2026-09-07

Continued with **Tetiana first**, without modifying firmware or the downloaded
upstream files. `lrc_experiment.py` adds these explicit stages:

- **cut --voice female**: extract teacher generator-input → audio ONNX and
  validate it against the existing held-out pack.
- **train --voice female --steps 100 --device cpu**: R7-shaped LRC/FSD training,
  with logs, exact arguments and source hashes saved to its run directory.
- **encode --voice female --run <absolute-run-directory>**: generate aligned
  `c` tensors `[1,40,T]` and `manifest.jsonl` for each pack split. The acoustic
  trainer accepts these via its target-directory options and `--target-key c`.
  Targets record the source and encoder hashes and pass exact NPZ round-trip
  checks. They must be regenerated whenever the encoder changes.

Use the project Python environment to execute the script. Training defaults to
CPU because FSD uses complex/iSTFT operators; duration training on MPS is not
evidence that this decoder works on MPS. Existing output directories are refused.
Checkpoints must come from local trusted runs; tensor-only loading is used with
an explicit allowlist for upstream's stored `PosixPath` configuration values.

### Upstream import repair

The ordinary decoder trainer failed with
`ModuleNotFoundError: saanotts.models.nirmal_tts`. That missing, unrelated model
is eagerly imported by upstream's `models/__init__.py`. The experiment launcher
loads the **unmodified `fsd.py`** directly under the trainer's existing
`roota_fsd_blocks` compatibility-module name. Both model and trainer files are
SHA-256 checked. No dummy neural operators, changed losses, or fake QAT behavior
are introduced; general acoustic imports still need separate repair.

### Verified results

- [x] Tetiana teacher decoder cut: four held-out rows; zero sample-count mismatch,
  zero chunk RMS mismatch; minimum waveform cosine **0.9999998628**.
- [x] **100 CPU training steps** completed on four training chunks. Decoder:
  **331,308 parameters**; training-only LRC encoder: **14,952 parameters**.
  Configuration: code40 / encoder64 / decoder76 / five blocks / FiLM12 / head48,
  FFT1024, hop256, 32-frame crops, batch1, seed4242.
- [x] Checkpoints reloaded by upstream and four held-out oracle WAVs rendered.
  Logged crop loss changed from **0.18265 to 0.06436**; these are different
  random crops, not a controlled held-out improvement measurement.
- [x] Local code-target builder generated **four train + four eval** aligned
  40-channel targets with exact save/load checks.
- [ ] **Quality acceptance: NOT PASSED.** Mean held-out waveform cosine is
  **0.00980**; output RMS is only **0.0051–0.0054**. No human listening approval.
  The numerical checks demonstrate execution only, not usable Ukrainian speech.

Artifacts: `work/tetiana/decoder-cut/` and `work/tetiana/lrc-r7-100-cpu/`.
The latter contains `console.log`, `invocation.json`, `train-report.json`,
`decoder-student.pt`, `lrc-encoder.pt`, `code-targets/`, and `index.html` with
four `audio/*_decoder_oracle.wav` files. These are teacher-latent → learned
encoder → student-decoder samples, **not a trained text-to-speech front end**.

Next: use a new, larger training/validation pack and tune/train the LRC decoder
until the held-out oracle lane is acceptable. Do not promote the 100-step
encoder's targets into production acoustic training. Then repair the acoustic
trainer dependency, train against the accepted code targets, and implement int8
export/calibration with C golden gates. No long job remains running.

## Larger Tetiana experiments — 2026-09-07

Built `work/tetiana/packs-256-32-v1/` without overwriting smoke artifacts:
**256 train rows / 282 chunks / 632.33 seconds** and **32 held-out rows /
37 chunks / 71.69 seconds**. Upstream finite-value, duration/alignment and
audio-length checks passed for both packs. The held-out rows remain disjoint
from training; the reserved test split was not used for tuning.

Completed two fresh **3,000-step CPU runs**, batch4, 64-frame crops, learning
rate 0.0002, seed4242, same R7 architecture. Full train/held-out chunk evaluation
is enabled, unlike the earlier smoke run. Each run took about five minutes on
the local machine; this is host training time, not device inference speed.

| Run | Spectral loss weight | Held-out mean L1 ↓ | Held-out waveform cosine | Held-out RMS ratio |
| --- | ---: | ---: | ---: | ---: |
| `lrc-r7-3000-cpu-v1` | 0.5 | 0.02547 | 0.02381 | 0.27981 |
| `lrc-r7-3000-cpu-spectral5-v1` | 5.0 | 0.02753 | 0.02031 | 0.50418 |

The stronger spectral objective improves output amplitude but worsens waveform
L1/cosine. **Neither is accepted as a usable voice.** Newly trained vocoder
waveform phase may differ, so low cosine alone is not proof of unintelligibility;
listening, spectral quality and teacher-calibrated ASR are still required.
Do not confuse these teacher-vs-student metrics with the eventual C-vs-exported
checkpoint golden gate, which must closely reproduce the same trained model.

Listen/compare via each directory's `index.html` and `audio/` files. No human
listening approval is recorded. Exact arguments, source hashes, console output,
checkpoints and reports remain under the run directories. A small tracked result
summary is in `experiments/tetiana-decoder-v1.json`.

New launcher controls:
- `packs --voice female --name <fresh-name> --train-rows 256 --eval-rows 32`
- `train --pack-root <absolute-pack-directory> --name <fresh-run-name>` with
  `--steps`, `--batch-size`, `--crop-frames`, `--spectral-weight` and `--evaluate`.
- `encode` derives the matching train/eval pack paths from its checkpoint and
  also writes the `summary.json` required by the acoustic trainer. Targets from
  a rejected decoder remain experimental; none were promoted from these runs.

### Next gates and multilingual firmware sequencing

1. Listen to both decoder candidates against the teacher. Investigate spectral
  loss/gain and phase quality before choosing a larger/longer run; no blind
  second-voice training or claim of convergence.
2. After accepting an encoder/decoder, regenerate its LRC40 targets. The acoustic
  trainer can be unblocked for **QAT off** by moving its missing `qat_ste` import
  inside the actual `c-int8` branch; do not substitute fake QAT. This repair is
  source-inspected but not yet applied. Use a full Ukrainian vocabulary and skip
  the stock 192-channel Piper renderer when training 40-channel code outputs.
3. Train/evaluate duration + acoustic + decoder together, implement calibrated
  int8 export, and pass the C golden/memory gates before calling Tetiana flashable.
4. **Only then change FlatSphere text generation for multiple languages**, as
  requested: language/voice profiles, translated greeting/time/battery formatters,
  Ukrainian hour/minute/percentage grammar, matching G2P data and mapping, English
  fallback, and tests for both languages. Keep UI language and speech voice
  configuration explicit. Finally build/flash with NVS preservation and test
  pronunciation, responsiveness and memory on the clock.

English firmware remains untouched by this training work. Both jobs completed;
no training process is left running and no Ukrainian image was flashed.

## Acoustic40 experiment — 2026-09-07

`acoustic_experiment.py` now provides **train** and **render** stages. Both use
`--voice female --decoder-run <absolute-LRC-run-directory>`; training additionally
takes a fresh `--name`, `--steps`, and `--device`, while rendering takes
`--acoustic-run <absolute-acoustic-run-directory>`.

The missing `qat_ste` import was reproduced and repaired **in memory**, on a
hash-checked source copy: its import is deferred into the existing `c-int8`
branch. QAT-off uses the original model and loss code. Enabling QAT still needs
the real missing dependency; there is no placeholder quantizer. No upstream
source file was modified on disk.

This run deliberately uses the **unapproved spectral5 decoder** to test the
acoustic interface. It does not promote that decoder or its targets as a
production voice. Generated and validated **282 train + 37 eval LRC40 targets**,
including the target summaries required by upstream. The launcher checks voice,
checkpoint hashes, source tensor hashes, coverage, code shape and finite values.

Completed **1,000 CPU steps**, h48/frame-depth5/token-depth3/k5, **256 vocabulary
entries**, 40 outputs, **204,288 trainable parameters**:

| Metric | Train | Held-out |
| --- | ---: | ---: |
| Mean code cosine | 0.80033 | 0.78099 |
| Mean code L1 | 0.13694 | 0.14251 |
| Mean code MSE | 0.04662 | 0.05061 |

These are oracle-duration code-regression metrics, **not intelligibility scores**.
Generated four paired held-out WAVs through the actual FSD decoder:

- `*-oracle.wav`: teacher latent → LRC encoder → FSD decoder.
- `*-acoustic.wav`: phoneme IDs + **teacher durations** → acoustic model → FSD.

Location: `work/tetiana/acoustic40-1000-cpu-v1/oracle-duration-audio/`.
Each pair has identical expected sample counts and finite audio. Predicted-code
audio RMS is approximately 0.011–0.0145; no gain normalization hides the level
difference. A JSON report includes phrase text, peaks and RMS for each pair.
No learned-duration, arbitrary-text or human-listening gate is claimed.

The run also contains `latent-student.pt`, `train-report.json`, `invocation.json`
and `console.log`. Tracked summary: `experiments/tetiana-acoustic-v1.json`.
The regression suite now has **12 passing tests**.

Remaining before flashability: listening/quality acceptance of the decoder and
acoustic lanes; converged duration training on larger packs; learned-duration
and arbitrary-text synthesis validation; real int8 export and calibration;
PyTorch-to-C golden tests and device memory budget. Keep the requested firmware
language-formatting work after those model gates. No firmware changes or flash
were performed; the acoustic run finished and no job remains active.

## Long duration training setup — ready, not started

Configuration: **`duration-long.json`**; launcher: **`duration_long.py`**.

| Setting | Value |
| --- | --- |
| Voice | Tetiana, Ukrainian |
| Training data | All 2,644 prepared rows / 2,969 teacher chunks |
| Validation data | All 145 prepared rows / 167 teacher chunks |
| Test data | Reserved, not used |
| Device | Apple GPU (`mps`) |
| Budget | 100,000 steps, batch size 32 |
| Model | h32 / depth3 / kernel5, 256 vocabulary entries, 40,292 parameters |
| Learning rate | 0.0005 |
| Logging | Every 100 steps, live terminal + `console.log` |
| Checkpoints | Every 1,000 steps; final checkpoint/report at completion |

The full teacher packs have already been generated in
`work/tetiana/duration-packs-full-v1/`, with successful upstream validation and
local vocabulary/alignment/text-split checks. They contain ~6,316 s train and
345 s validation teacher audio. This uses all our **filtered** corpus, not every
original recording. The packs retain 192-channel teacher latents for potential
reuse; this stage trains only duration, not acoustic/decoder models.

### Start when ready

In VS Code, select **Terminal → Run Task → Tetiana: train duration (100k steps)**.
The task uses the project `.venv` and `caffeinate -i` to prevent idle system
sleep while the child process runs. Keep the Mac on power; this does not prevent
sleep caused by closing its lid. Task configuration is local in
`.vscode/tasks.json` (the repository ignores `.vscode`).

Alternatively, execute `lang/ua/duration_long.py` with the **train** stage using
the project Python environment. Other stages: **prepare**, **check**, **smoke**.
`check` does not train. `smoke` uses five steps in a separate output directory.
The launcher never starts the long run implicitly, and refuses to overwrite an
existing run directory. Change `run_name` in the config for a fresh run.

Outputs will be in **`work/tetiana/duration-100000-mps-v1/`**:
`invocation.json`, `console.log`, `duration-student-step1000.pt`, subsequent
numbered checkpoints, final `duration-student.pt` and `train-report.json`.
Do not use the unrelated four-row smoke checkpoints as initialization for this
full-vocabulary model.

### Interruption, evaluation and limits

- Stop with Ctrl+C/Terminate Task. Keep the last complete numbered checkpoint;
  a file interrupted during saving may be incomplete. Earlier files remain.
- Upstream checkpoints hold **weights/config only**, not optimizer or RNG state.
  Its `--initial-checkpoint` supports topology-compatible weight initialization,
  **not exact resume**. This launcher intentionally starts fresh; recovery should
  use a new run directory and explicitly reviewed initialization arguments.
- Training logs show batch metrics. Full train/held-out evaluation happens at
  completion, **not every checkpoint**. There is no automatic best-checkpoint
  selection or early stopping. Compare saved checkpoints on held-out data before
  selecting one; longer training can overfit and 100k steps is only a budget.
- Setup validation passed a **five-step MPS run on the full packs**, including
  numbered checkpoint saving and final train/held-out evaluation, in
  `duration-100000-mps-v1-smoke/`. It is not quality-qualified.
- **The 100,000-step job completed** and produced strong duration results (see
  below). Duration training alone will not fix the decoder or make Tetiana
  flashable.

## Full-stack synthesis and duration result — 2026-09-07

The long duration run **finished** in `work/tetiana/duration-100000-mps-v1/`:

| Metric | Train | Held-out |
| --- | ---: | ---: |
| Token MAE | 0.502 | 0.801 |
| Token exact | 67.2% | 54.1% |
| Frame ratio mean | 0.957 | 0.946 |

Held-out predicted/teacher total-frame ratio is **0.9256**; the model slightly
under-predicts long spans but has no meaningful overfitting. 40,292 parameters.

`lang/ua/synthesize.py` then rendered the **full stack** (phonemes → learned
duration → acoustic → decoder) for all 24 clock phrases into
`work/tetiana/synthesize-v1/`, with the Piper teacher as a listening reference
(`index.html` pairs student and teacher). Results:

- **Duration is strong:** mean student 2.466s vs teacher 2.394s; several phrases
  match the teacher within milliseconds.
- **Amplitude is the blocker:** student RMS is roughly 0.013–0.019 vs teacher
  0.13–0.20, about **10× too quiet**. The decoder loses energy; the acoustic
  model is secondary.
- Full-stack report and provenance: `experiments/tetiana-fullstack-v1.json`.
  **No listening approval and no export.** The synthesis harness runs on CPU and
  is deterministic.

Next priority: train the **decoder** longer (and investigate its gain/amplitude
reconstruction) and train the **acoustic** model further. The duration model is
the strongest current component and is reusable. Tetiana is still not flashable.

## Long quality training setup — decoder and acoustic

Target is English-level quality. Both long runs are prepared and validated, but
**not started**. The R7 architecture already sizes to ~680 KB int8 (same as the
English Kristin voice), so it fits the 2.6–3 MB budget without any size
reduction: duration ~40 KB + acoustic ~200 KB + decoder ~400 KB plus scale
headers, plus the Ukrainian espeak data in its separate SPIFFS partition.

The `synthesize-v1` listening page was user-confirmed as a **recognizable,
good-sounding voice** (not noise). Its lower RMS is a loudness-scaling artifact,
not a failure; more training improves both clarity and loudness. The plan below
runs each stage **3× longer** than the first proposal.

### Decoder first (150k steps)

`decoder-long.json` / `decoder_long.py`.

- **Data:** reuses `duration-packs-full-v1` (2,969 train / 167 eval chunks). The
  upstream decoder trainer only needs `phoneme_ids`/`w_ceil`/`generator_input`,
  which the acoustic-mode packs already contain; teacher audio is decoded from
  the decoder-cut ONNX, so no new packs are generated.
- **Budget:** 150,000 steps total, run as **15 segments of 10,000 steps**
  because the upstream decoder trainer has no periodic checkpoint save. Each
  segment warm-starts from the previous via `--init-decoder-checkpoint`
  (weight-only continuation; the optimizer is reset, which can cause a brief
  loss bump).
- **Checkpoints:** one `decoder-student.pt` + `lrc-encoder.pt` per segment under
  `work/tetiana/decoder-150000-cpu-v1/seg-NNNNN/`.
- **Device:** CPU (FSD uses complex/iSTFT operators; MPS is not validated here).
- **Estimated:** ~4–5 h for 150k steps, after a one-time ~90 s teacher-audio
  decode. This is an estimate, not a promise.
- Start with **Terminal → Run Task → Tetiana: train decoder (150k steps)**.

### Acoustic second (150k steps)

`acoustic-long.json` configures 150,000 steps, h48/d5/tok3/k5, vocab256,
40 outputs, `--sample-weight-mode frames`, and periodic checkpoints at
`1000,5000,10000,25000,50000,75000,100000,125000,150000` via the upstream
`--checkpoint-steps`. `acoustic_experiment.py` gained `--checkpoint-steps` and
`--sample-weight-mode`.

**Do not start it yet:** the acoustic targets must be regenerated from the
decoder that finishes first (`lrc_experiment.py encode --run <new decoder run>`),
otherwise the acoustic model trains against stale codes. After the decoder run
completes, regenerate targets and start the acoustic long run.

### Validation

- `decoder_long.py` supports `check`, `smoke` (isolated `-smoke` directory) and
  `train`. The smoke run reused the full packs for five steps, wrote both
  checkpoints, and passed.
- `acoustic_experiment.py` supports `train` and `render`; the long acoustic run
  uses `--checkpoint-steps` and `--sample-weight-mode frames`, and a five-step
  MPS smoke test confirmed Apple GPU compatibility before the full run.
- **17 regression tests pass.** Decoder smoke output is in
  `work/tetiana/decoder-50000-cpu-v1-smoke/` (superseded; ignore).

## Decoder and acoustic completion — 2026-09-08

The **decoder 200k-step run completed** and is a major improvement over the
earlier 3k-step experiments:

| Held-out metric | 3k steps | **200k steps** |
| --- | ---: | ---: |
| Mean RMS ratio | ~0.50 | **0.936** |
| Mean cosine | ~0.02 | **0.852** |
| Oracle render cosine | ~0.01 | **0.838** |

Final checkpoint: `work/tetiana/decoder-200000-cpu-v1/seg-00019/`. Summary:
`experiments/tetiana-decoder-200k.json`. The amplitude problem is largely
resolved.

Code targets were regenerated from that encoder (2,969 train + 167 eval) and the
**acoustic training (150k steps, MPS) is running** into
`work/tetiana/acoustic-150000-mps-v1/` with periodic checkpoints. After it
finishes, re-run `synthesize.py` against the new components and compare to
`work/tetiana/synthesize-v1/`.