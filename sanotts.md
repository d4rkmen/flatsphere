# Speech integration and publication

See [sanoTTS](components/sanotts/README.md) and
[eSpeak NG](components/espeak_ng/README.md) for source pins, licenses and tests.

## Local components

Both engines are repository-local, without speech build-time downloads or a
registry eSpeak dependency. Both canonical voice contracts are under
`components/sanotts/voices`. One **FlatSphere speech** choice selects Serhiy UK
or Kristin EN; the derived UK flag selects Ukrainian announcements.
Default and saved configuration: Serhiy.

The existing model bytes, runtime math, gain behavior, 16-byte blob alignment,
64-KiB worker stack, bounded queue/arena/frame/PCM lifetime are retained.
No new training outputs were substituted and no removed QAT work was restored.

## Evidence is not acceptance

Kristin correlation 0.989231 passes >0.98. Serhiy 400k measures 0.959100 on the
host int8 golden check and is the voice kept for listening quality, but numeric
parity still fails the gate. Earlier experimental Ukrainian bundles with lower
correlation were removed. No threshold was lowered; further training is not a
demonstrated remedy for quantization parity.

## Build and update safety

Select a voice in **ESP-IDF SDK Configuration Editor → FlatSphere speech**, then
build through the ESP-IDF extension. Preserve the other board settings.
Application stays at `0x10000` / 4 MiB and eSpeak at `0x410000` / 1 MiB.
The local engine generates the combined EN/UK data image. A first installation
needs matching partition table and data, not just the app.

Leave `FLATSPHERE_FLASH_FACTORY_NVS` disabled to preserve saved settings.
Component Manager owns managed dependencies; do not edit/delete them manually.
No flash, commit or push is part of this work.

Public release still needs voice/data rights review, applicable GPL source and
notices fulfillment, resolution of eSpeak data-generation provenance, and
refactored firmware playback/heap/stack/latency/stress validation.
