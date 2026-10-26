# Local eSpeak NG (G2P only)

Repository-local ESP-IDF component, **not a registry dependency**. The translator
subset is pinned to eSpeak NG 1.52.0 commit
`4870adfa25b1a32b4361592f1be8a40337c58d6c`. `provenance.json` records the
original paths, sizes, SHA-256 hashes and both source archive identities.
Firmware builds need no source archives, training cache or Python/Piper install.

`upstream/` is verbatim. `port/config.h` is the pinned sanoTTS ESP32 phoneme-only
configuration. `cmake/prepare.cmake` verifies hashes and applies the existing
SPIFFS `phontab` file-probe patch to a generated `speech.c`. Warning suppression
is confined to the imported translator. The sanoTTS adapter owns model-specific
IPA-to-ID mapping, bounds checks, voice selection and SPIFFS mount lifetime.

## EN/UK data

`data/` retains the working pinned port English dictionary/language definitions
and 4-KiB `phondata` stub, plus the installed full `phontab`, `phonindex`,
`intonations`, Ukrainian dictionary and language definition. Both languages use
the full tables; the reduced English tables are unsuitable for UK. The stub
is for phonemization only, **not formant audio synthesis**. The imported
`data/README.md` describes the original English subset, not this combined layout.

The UK overlay originated from Piper bundled data; installed piper-tts is 1.8.0.
Its exact bytes are pinned, but the original data compiler revision/recipe has
**not** been established. Identity and host behavior do not establish complete
source-rebuild provenance or redistribution clearance. Resolve this gap before
binary distribution. There is no build-time dependency on installed Piper.

A fresh 1-MiB SPIFFS image is generated directly from local data. Including both
languages avoids stale overlays after voice selection changes. The partition
remains at `0x410000`; NVS is not managed by this component.

## Import and notices

`tests/sanotts/import_espeak.py` takes the pinned eSpeak archive, pinned sanoTTS
archive and hash-matching UK overlay directory. It checks all inputs before
writing and refuses differing destinations. This optional maintenance importer
is never invoked by firmware builds.

Preserve `upstream/COPYING` (GPLv3), `COPYING.APACHE`, `COPYING.BSD2`,
`COPYING.UCD`, Unicode subcomponent notices and `port/LICENSE.sanoTTS`, plus
source-file copyrights. The linked firmware is not Apache-only or MIT-only.
Review applicable GPL Corresponding Source, modifications, build/install
instructions and installation-information obligations before binary release.
This notice is not legal clearance.