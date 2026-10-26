"""Generate relocatable bundle hashes without altering any model/export bytes."""
from pathlib import Path
import hashlib
import json
from install_voice import verify_export

ROOT = Path(__file__).resolve().parents[2]
COMPONENT = ROOT / "components/sanotts"


def main():
    provenance = json.loads((COMPONENT / "upstream/provenance.json").read_text())
    for name, entry in provenance["files"].items():
        if hashlib.sha256((COMPONENT / name).read_bytes()).hexdigest() != entry["sha256"]:
            raise ValueError(f"Upstream provenance mismatch: {name}")
    for voice, speaker, language in (("en-kristin", "kristin", "en"),
                                     ("uk-serhiy", "serhiy", "uk")):
        directory = COMPONENT / "voices" / voice
        if language == "uk":
            verify_export(directory, speaker)
        files = sorted(p for p in directory.rglob("*") if p.is_file() and p.name != "bundle.json")
        artifacts = {p.relative_to(directory).as_posix(): {
            "sha256": hashlib.sha256(p.read_bytes()).hexdigest(), "bytes": p.stat().st_size
        } for p in files}
        bundle = {"schema_version": 1, "speaker": speaker, "language": language,
                  "experimental": False, "quality_accepted": False,
                  "source": "upstream/provenance.json" if language == "en" else "export_manifest.json",
                  "artifacts": artifacts}
        target = directory / "bundle.json"
        if target.exists():
            if json.loads(target.read_text()) != bundle:
                raise ValueError(f"Existing bundle differs; review changes explicitly: {target}")
        else:
            with target.open("x") as out:
                json.dump(bundle, out, indent=2)
                out.write("\n")
        print(voice, len(artifacts), "artifacts; model/export bytes unchanged")


if __name__ == "__main__":
    main()