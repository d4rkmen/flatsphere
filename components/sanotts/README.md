# Local sanoTTS component

## Sources and ownership

`upstream/` contains verbatim inference, scalar kernels, public headers, host
port/test and model-specific G2P from Ampixa sanoTTS commit
`939d982b9faa54cbcf5d24cc878f5cd514b2646e`. Archive SHA-256:
`68a9bcc82c568889a6d656c7efa85ba2565ba5b5681c2336229bdc364f67c048`.
`upstream/provenance.json` binds all 25 imported files, including Kristin's
fixture and **seven matching model headers**.

The sibling [espeak_ng](../espeak_ng/README.md) owns the pinned translator,
configuration, public engine headers and EN/UK data. Copy both local components
into an ESP-IDF project's `components/` directory; no eSpeak registry package,
source archive, training output or installed Piper is required to build them.
The application provides an `espeak` SPIFFS partition (here 1 MiB), ESP32-S3
PSRAM, and a blocking PCM sink via `include/flatsphere_sanotts.h`.

`cmake/runtime.cmake` applies the existing input/ID/duration/arena guards to a
generated copy; `cmake/g2p.cmake` retains capacity/EOS/loop checks, safe reverse
punctuation traversal and EN/UK selection. Upstream originals and runtime math
are unchanged. The model include path is the selected bundle only: there is no
English fallback directory to silently supply another voice's offsets.

## ESP32-S3 acceleration

`port/sn_matvec_s8_esp32s3.S` is the upstream PIE int8 matvec kernel, copied
verbatim from `mcu/ports/esp32s3/firmware/main/` at the same commit (SHA-256
`1c9114440dd2e8c6e3735fc5f907ad42b6cd0c2d2abc74f8b960f2e000c2734d`; GPL-3.0,
outside the MIT scope). `port/snt_kernels_esp32s3.c` dispatches to it only when
weights and activations are 16-byte aligned internal SRAM; otherwise it calls
the unchanged scalar reference (renamed `snt_ref_*` at compile time).
`sano_port.c` splits `snt_par_run` between the synthesis task (core 1) and a
blocking worker on core 0.

`runtime.cmake` redirects the runtime's `aa_try` weight-residency buffers to an
internal-SRAM arena (up to 136 KiB, leaving 48 KiB internal free) sized after
G2P init. To make room, the runtime's own `.bss` (`linker.lf`),
LVGL (`main/linker.lf`) and eSpeak (`../espeak_ng/linker.lf`) live in PSRAM
(`CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY`). Only `act_stage` (the SIMD
activation staging buffer) and the worker stack stay internal; G2P reads SPIFFS
with the cache disabled, so a PSRAM worker stack faults there.

On-device (an earlier Ukrainian test bundle of the same model shape), SIMD
output was bit-identical to the scalar path across a
20-sentence benchmark (`parity_diff 0` with PSRAM activations staged through
`act_stage`). On the same 75,653 ms of audio, synthesis time and real-time
factor were:

| Weights resident | Synthesis | RTF |
| --- | --- | --- |
| 113,648 B (no runtime `.bss` move) | 53,617 ms | 0.709x |
| 139,264 B (whole decoder) | 52,269 ms | 0.690x |

Against the original scalar single-core path (~2.6x real time) this is a ~3.8x
speedup, so a phrase still synthesizes slightly faster than it plays.

Safety limits: 16-byte aligned flash blobs, 64-KiB worker stack, four copied
191-byte requests, 256 IDs, 640 frames, 2-MiB PSRAM arena and 327,680-byte PCM
buffer. One worker owns the non-reentrant runtime/G2P. The sink must block until
it releases the PCM buffer. Existing speaker volume/gain behavior is retained.

## Voice bundles and evidence

### Choosing a voice

Open **SDK Configuration Editor → FlatSphere speech → Voice** and pick one entry.
That is the only setting: the announcement language follows the voice.

| Menu entry | `sdkconfig` line | Bundle | Announcements |
| --- | --- | --- | --- |
| Serhiy — Ukrainian, male (default) | `CONFIG_FLATSPHERE_TTS_VOICE_SERHIY=y` | `voices/uk-serhiy` | Ukrainian |
| Kristin — English, female | `CONFIG_FLATSPHERE_TTS_VOICE_KRISTIN=y` | `voices/en-kristin` | English |

Without the editor, put the `sdkconfig` line in `sdkconfig.defaults` (or an
overlay such as `sdkconfig.sanotts.defaults`) and run a full clean, or edit it in
`sdkconfig` directly. The build log prints `sanoTTS voice: <bundle>`; the device
logs `Voice: <bundle>` at start-up.

Derived, non-editable symbols: `FLATSPHERE_TTS_VOICE_BUNDLE` (bundle directory)
and `FLATSPHERE_TTS_LANG_UK` (phrase language in `main/`). Speaker and language
are read from the selected `bundle.json`; the configure step fails with a clear
message if the bundle is missing, its hashes do not match, or its language
disagrees with the announcement language.

Saved configurations using the pre-release names (`FLATSPHERE_TTS_VOICE_EN`,
`FLATSPHERE_TTS_VOICE_UK`) are migrated automatically by `sdkconfig.rename`.
Earlier experimental Ukrainian bundles were removed; a saved configuration
that selected one falls back to the default voice.

**Adding a voice:** install the bundle as `voices/<language>-<speaker>/` (see
`tests/sanotts/install_voice.py`), then add one `config FLATSPHERE_TTS_VOICE_<NAME>`
entry and one `default "<bundle>" if …` line in `Kconfig` (plus the
`FLATSPHERE_TTS_LANG_UK` condition if it is Ukrainian). No CMake changes needed.

User-observed playback is not numerical acceptance.

| Bundle | Existing source | Host correlation | Status |
| --- | --- | --- | --- |
| `voices/en-kristin` | Pinned upstream R7 fixture | 0.989231 | Passes >0.98; earlier user-reported playback |
| `voices/uk-serhiy` | Serhiy 400k student export | 0.959100 | Listening-quality voice; int8 parity below the 0.98 gate |

Serhiy front SHA-256:
`22ad930da117c8be52693d6c95bb69d0cfa1356612e7a06f24d6b7a821c2da96`; decoder:
`0c2cc2ed5e41db6ca82809b5420608e03475ef5aab6561557644f8d54f44bca4`.
The original export report, manifest and golden vectors are preserved in the
bundle; their checkpoint and eval-pack paths are repository-relative training
locations (`train/runs/...`) that record provenance, not firmware dependencies.
Earlier experimental Ukrainian bundles (lower host correlation, rejected on
listening quality) were removed from the tree. Legacy `espeak-uk/` remains
unused by this build.

`bundle.json` adds integrity, **not acceptance**. More training is not a proven
fix for int8 parity loss; no threshold or runtime math was changed.

## Build and verification

Use ESP-IDF 5.5.4 and **SDK Configuration Editor → FlatSphere speech**; build
through the ESP-IDF extension. Preserve board settings when applying
`sdkconfig.sanotts.defaults` (an overlay, not a full board configuration).
Factory NVS stays opt-in; leave `FLATSPHERE_FLASH_FACTORY_NVS` disabled.

- `tests/sanotts/publication_audit.py`: source/data/bundle hashes, local selection,
  publication inclusion, NVS policy, runtime bounds, EN/UK G2P and all-voice
  parity. Packaging success is not Ukrainian parity acceptance.
- `tests/sanotts/package_voices.py`: verifies original provenance and creates or
  verifies bundle metadata without replacing model/export bytes.
- `tests/sanotts/run_host_tests.sh`: local Kristin golden/bounds gate.
- `tests/sanotts/firmware_audit.py`: read-only check of the latest firmware's
  exact embedded blobs, selected headers/language, alignment and NVS exclusion.
- `tests/sanotts/import_pinned.py` and `import_espeak.py`: optional hash-pinned
  archive importers, never invoked by firmware builds.
- `tests/sanotts/install_voice.py`: explicit hash-checked export packaging into
  an empty canonical slot; refuses overwrites. Does not train or accept a voice.

Hash-bound training scripts and removed QAT work are unchanged. Device playback,
stack/heap high-water, latency, gain/clipping and long-run stress remain open.
No flash is part of this work.

Both selections build with ESP-IDF 5.5.4: Kristin `0x26e050` bytes (39% app
partition free); Ukrainian bundles of this model shape `0x276ca0` (38%). The
read-only firmware audit verifies the selected bundle; these are build results,
not device acceptance.

## Distribution obligations — not legal clearance

Retain `upstream/LICENSE` (GPLv3) and `upstream/LICENSE.MIT` verbatim, including
copyright and scope notices. MIT covers only the enumerated inference files,
**not all G2P, R7 model headers or weights**. The eSpeak-linked firmware has GPL
obligations: provide applicable Corresponding Source, modifications, build and
installation scripts and installation information where required. This minimal
runtime snapshot is not a claim of complete source compliance for derived
models or precompiled dictionaries.

Kristin provenance is retained in its original fixture README; complete weight
redistribution terms still require review. The Serhiy export records its
checkpoint lineage in `voices/uk-serhiy/export_manifest.json`; the teacher
(Apache-2.0-labelled Piper `uk_UA-serhiy-high`), the stressed training corpus
and the FP16 package are described on the
[ua-serhiy-0p58m model card](https://huggingface.co/dmytro-m/ua-serhiy-0p58m).
Those labels do not by themselves establish generated-output or derived-weight
rights.
Review model cards and dataset permissions and preserve their notices. Resolve
the [eSpeak data source-rebuild gap](../espeak_ng/README.md) before binary release.
No blanket MIT/Apache weight license or release clearance is claimed.
