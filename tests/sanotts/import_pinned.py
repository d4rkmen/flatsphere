"""Extract the minimal immutable upstream artifacts from the pinned archive.

Maintenance-only importer, never invoked by firmware CMake. Refuses overwrites.
"""
import hashlib
import json
from pathlib import Path
import sys
import tarfile

ROOT = Path(__file__).resolve().parents[2]
COMMIT = "939d982b9faa54cbcf5d24cc878f5cd514b2646e"
SHA256 = "68a9bcc82c568889a6d656c7efa85ba2565ba5b5681c2336229bdc364f67c048"


def main():
    archive = Path(sys.argv[1])
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == SHA256
    component = ROOT / "components/sanotts"
    mappings = {
        "LICENSE": "upstream/LICENSE",
        "LICENSE.MIT": "upstream/LICENSE.MIT",
        "mcu/include/snt_tts.h": "upstream/include/snt_tts.h",
        "mcu/include/snt_port.h": "upstream/include/snt_port.h",
        "mcu/src/snt_tts.c": "upstream/src/snt_tts.c",
        "mcu/src/snt_kernels_ref.c": "upstream/src/snt_kernels_ref.c",
        "mcu/ports/host/snt_port_host.c": "upstream/host/snt_port_host.c",
        "mcu/test/golden_main.c": "upstream/host/golden_main.c",
    }
    for name in ("esp_g2p.c", "esp_g2p.h", "cp_id_table.h"):
        mappings[f"mcu/ports/esp32s3/firmware/main/{name}"] = f"upstream/g2p/{name}"
    for name in ("front_q8_meta.h", "fsd_meta.h", "fsd_q8_meta.h", "act_scales.h",
                 "chain_scales.h", "rb_scales.h", "frozen_norm.h"):
        mappings[f"mcu/src/model/{name}"] = f"voices/en-kristin/model/{name}"
    for name in ("front_q8.bin", "model_q8.bin", "e2e_ids.bin", "e2e_durs.bin",
                 "e2e_audio.bin", "README.md", "SHA256SUMS"):
        mappings[f"mcu/test/fixtures/en_us_r7/{name}"] = f"voices/en-kristin/{name}"
    records = {}
    with tarfile.open(archive) as tar:
        for source, destination in mappings.items():
            member = tar.getmember(f"sanoTTS-{COMMIT}/{source}")
            assert member.isfile(), source
            stream = tar.extractfile(member)
            assert stream is not None
            data = stream.read()
            path = component / destination
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("xb") as output:
                output.write(data)
            records[destination] = {"source": source, "sha256": hashlib.sha256(data).hexdigest()}
    with (component / "upstream/provenance.json").open("x") as output:
        json.dump({"repository": "https://github.com/Ampixa/sanoTTS", "commit": COMMIT,
                   "archive_sha256": SHA256, "files": records}, output, indent=2)
        output.write("\n")
    print(f"Extracted {len(records)} hash-pinned artifacts; no eSpeak source or data imported")


if __name__ == "__main__":
    main()