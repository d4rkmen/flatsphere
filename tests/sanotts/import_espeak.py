"""Maintenance-only, hash-verified import; never called by firmware builds.

Usage: import_espeak.py ESPEAK_ARCHIVE SANO_ARCHIVE UK_DATA_DIRECTORY
Copies verbatim source/data; refuses differing existing files. No extraction
paths from an archive are trusted. Patch logic stays in reviewed CMake files.
"""
import hashlib
import json
from pathlib import Path
import sys
import tarfile

ROOT = Path(__file__).resolve().parents[2]
DEST = ROOT / "components/espeak_ng"
ESPEAK = "4870adfa25b1a32b4361592f1be8a40337c58d6c"
SANO = "939d982b9faa54cbcf5d24cc878f5cd514b2646e"
ARCHIVES = {
    ESPEAK: "cd83f84c4e495f281ac14e919aecf2834306ec1ea1b498de8ef6000f3a0f90de",
    SANO: "68a9bcc82c568889a6d656c7efa85ba2565ba5b5681c2336229bdc364f67c048",
}
SOURCES = "common mnemonics error ieee80 compiledata compiledict dictionary encoding intonation langopts numbers phoneme phonemelist readclause setlengths soundicon spect ssml synthdata synthesize tr_languages translate translateword voices wavegen espeak_api speech".split()
UK = {
    "uk_dict": "a779bb0f51ce5cebb650a4e2282162fc8c44609210a2baa700f81f3e83e6e10c",
    "phontab": "cfee959088c75a2f9a8b78535f34eeea3ed7cac24615a57581ae3a9060de68c2",
    "phonindex": "093104ededac06529465db6c51769d0bf8311f9eaec7a0b881599734e40a230e",
    "intonations": "c4ff28c76c30cf7d9f07643e08ec39e68769ef648098269b5c0ab7731218e219",
    "lang/zle/uk": "df3d1a9f77bac0531e72f87be398d9eb1e903bd6f839d4f04ffec5abb4e34634",
}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def main():
    payloads = {}
    records = {}

    def add(destination, data, origin):
        payloads[destination] = data
        records[destination] = {"source": origin, "sha256": digest(data), "bytes": len(data)}

    for archive_path, commit, prefix in ((sys.argv[1], ESPEAK, "espeak-ng"),
                                          (sys.argv[2], SANO, "sanoTTS")):
        archive = Path(archive_path)
        if digest(archive.read_bytes()) != ARCHIVES[commit]:
            raise ValueError(f"Archive hash mismatch: {archive}")
        with tarfile.open(archive) as tar:
            for member in tar.getmembers():
                if not member.isfile():
                    continue
                name = member.name.removeprefix(f"{prefix}-{commit}/")
                destination = None
                if commit == ESPEAK:
                    p = Path(name)
                    if (name.startswith("src/include/") and p.suffix == ".h"
                            or name.startswith("src/libespeak-ng/") and
                            (p.suffix == ".h" or p.name in {s + ".c" for s in SOURCES})
                            or name.startswith("src/ucd-tools/src/") and p.suffix in (".h", ".c")
                            or name in ("COPYING", "COPYING.APACHE", "COPYING.BSD2", "COPYING.UCD",
                                        "src/ucd-tools/COPYING", "src/ucd-tools/COPYING.UCD")):
                        destination = "upstream/" + name
                else:
                    port = "mcu/ports/esp32s3/firmware/"
                    if name == port + "components/espeak-ng/config.h":
                        destination = "port/config.h"
                    elif name == "LICENSE":
                        destination = "port/LICENSE.sanoTTS"
                    elif name.startswith(port + "espeak-ng-data/"):
                        relative = name.removeprefix(port + "espeak-ng-data/")
                        if relative not in UK:
                            destination = "data/" + relative
                if destination is not None:
                    if ".." in Path(destination).parts:
                        raise ValueError(destination)
                    stream = tar.extractfile(member)
                    if stream is None:
                        raise ValueError(member.name)
                    add(destination, stream.read(), f"{prefix}@{commit}/{name}")
    for name, expected in UK.items():
        data = (Path(sys.argv[3]) / name).read_bytes()
        if digest(data) != expected:
            raise ValueError(f"UK overlay mismatch: {name}")
        add("data/" + name, data, "FlatSphere previously installed espeak-uk; Piper bundled data")
    provenance = {
        "repositories": {"espeak-ng": {"url": "https://github.com/espeak-ng/espeak-ng", "commit": ESPEAK,
                                      "archive_sha256": ARCHIVES[ESPEAK]},
                         "sanoTTS": {"url": "https://github.com/Ampixa/sanoTTS", "commit": SANO,
                                     "archive_sha256": ARCHIVES[SANO]}},
        "data_note": "Existing working G2P-only data: pinned port English dictionary/definitions and 4KiB phondata stub, full installed UK tables/dictionary. Original Piper build revision not established; hashes identify the exact bytes, not legal clearance.",
        "files": records,
    }
    payloads["provenance.json"] = (json.dumps(provenance, indent=2) + "\n").encode()
    # Preflight the entire import before writing anything.
    for name, data in payloads.items():
        target = DEST / name
        if target.exists() and target.read_bytes() != data:
            raise FileExistsError(f"Refusing to replace {target}")
    for name, data in payloads.items():
        target = DEST / name
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open("xb") as out:
                out.write(data)
    print(f"Verified/imported {len(records)} local eSpeak source, license and data files")


if __name__ == "__main__":
    main()