"""Explicit publication packaging, independent of hash-bound training scripts.

Usage: install_voice.py EXPORT_DIR --speaker <name> [--language uk|en] [--overwrite] [--select]
Refuses existing canonical slots unless --overwrite is passed. --select requires a
matching FLATSPHERE_TTS_VOICE_<NAME> entry in components/sanotts/Kconfig.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HEADERS = ("act_scales.h", "chain_scales.h", "front_q8_meta.h", "frozen_norm.h",
           "fsd_meta.h", "fsd_q8_meta.h", "rb_scales.h")
REQUIRED = {"front_q8.bin", "model_q8.bin", "e2e_audio.bin", "e2e_durs.bin",
            "e2e_ids.bin", *("model/" + name for name in HEADERS)}
SPEAKER_PATTERN = re.compile(r"^[a-z][a-z0-9-]*$")


def speaker_slug(value: str) -> str:
    if not SPEAKER_PATTERN.match(value):
        raise argparse.ArgumentTypeError(f"Speaker must match {SPEAKER_PATTERN.pattern}: {value!r}")
    return value


def verify_export(source, speaker):
    manifest = json.loads((source / "export_manifest.json").read_text())
    if manifest["speaker"] != speaker or not REQUIRED <= manifest["artifacts"].keys():
        raise ValueError("Wrong speaker or incomplete export")
    for name, record in manifest["artifacts"].items():
        path = source / name
        if (Path(name).is_absolute() or ".." in Path(name).parts or path.is_symlink()
            or not path.resolve().is_relative_to(source.resolve())):
            raise ValueError(f"Unsafe artifact: {name}")
        if hashlib.sha256(path.read_bytes()).hexdigest() != record["sha256"]:
            raise ValueError(f"Export hash mismatch: {name}")
    return manifest


def _set_sdkconfig_bool(text: str, name: str, enabled: bool) -> str:
    replacement = f"CONFIG_{name}=y" if enabled else f"# CONFIG_{name} is not set"
    pattern = rf"^(?:CONFIG_{re.escape(name)}=y|# CONFIG_{re.escape(name)} is not set)$"
    return re.sub(pattern, replacement, text, flags=re.M)


def select_voice(speaker: str, language: str = "uk", root: Path = ROOT) -> Path:
    kconfig = (root / "components/sanotts/Kconfig").read_text()
    choices = re.findall(r"^config (FLATSPHERE_TTS_VOICE_[A-Z0-9_]+)$", kconfig, re.M)
    selected = f"FLATSPHERE_TTS_VOICE_{speaker.upper().replace('-', '_')}"
    if selected not in choices:
        raise ValueError(f"Missing {selected} in components/sanotts/Kconfig")
    bundle_name = f"{language}-{speaker}"
    sdkconfig_path = root / "sdkconfig"
    if not sdkconfig_path.exists():
        return sdkconfig_path
    text = sdkconfig_path.read_text()
    voice_lines = [
        f"CONFIG_{sym}=y" if sym == selected else f"# CONFIG_{sym} is not set"
        for sym in choices
    ]
    voice_lines.append(f'CONFIG_FLATSPHERE_TTS_VOICE_BUNDLE="{bundle_name}"')
    voice_lines.append(
        "CONFIG_FLATSPHERE_TTS_LANG_UK=y"
        if language == "uk"
        else "# CONFIG_FLATSPHERE_TTS_LANG_UK is not set"
    )
    block_pattern = (
        r"(?:^(?:# )?CONFIG_FLATSPHERE_TTS_VOICE_[A-Z0-9_]+(?:=y| is not set)\n)+"
        r"^CONFIG_FLATSPHERE_TTS_VOICE_BUNDLE=\"[^\"]*\"\n"
        r"^(?:CONFIG_FLATSPHERE_TTS_LANG_UK=y|# CONFIG_FLATSPHERE_TTS_LANG_UK is not set)"
    )
    updated, count = re.subn(block_pattern, "\n".join(voice_lines), text, count=1, flags=re.M)
    if count != 1:
        raise ValueError(f"Could not locate FlatSphere speech voice block in {sdkconfig_path}")
    updated = _set_sdkconfig_bool(updated, "FLATSPHERE_TTS_VOICE_EN", speaker == "kristin")
    updated = _set_sdkconfig_bool(updated, "FLATSPHERE_TTS_VOICE_UK", language == "uk")
    sdkconfig_path.write_text(updated)
    print(f"Selected {bundle_name} ({selected}=y) in {sdkconfig_path}")
    return sdkconfig_path


def install(source, speaker, voices=None, *, language="uk", overwrite=False, select=False):
    source = Path(source).resolve()
    manifest = verify_export(source, speaker)
    voices = Path(voices) if voices else ROOT / "components/sanotts/voices"
    destination = voices / f"{language}-{speaker}"
    if destination.exists() and not overwrite:
        raise FileExistsError(f"Refusing to overwrite {destination}")
    voices.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{language}-{speaker}-", dir=voices))
    try:
        # Copy only verified artifacts and their original manifest; never follow
        # unknown training directory contents or manufacture acceptance files.
        for name in (*manifest["artifacts"], "export_manifest.json"):
            target = staging / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source / name, target)
        verify_export(staging, speaker)
        artifacts = {p.relative_to(staging).as_posix(): {
            "sha256": hashlib.sha256(p.read_bytes()).hexdigest(), "bytes": p.stat().st_size}
            for p in sorted(staging.rglob("*")) if p.is_file()}
        bundle = {"schema_version": 1, "speaker": speaker, "language": language,
                  "experimental": True, "quality_accepted": False,
                  "source": "export_manifest.json", "artifacts": artifacts}
        (staging / "bundle.json").write_text(json.dumps(bundle, indent=2) + "\n")
        if destination.exists():
            shutil.rmtree(destination)
        staging.rename(destination)
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    print(f"Installed experimental {speaker}: {destination}; no quality acceptance asserted")
    if select:
        select_voice(speaker, language)
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("export", type=Path)
    parser.add_argument("--speaker", type=speaker_slug, required=True)
    parser.add_argument("--language", choices=("uk", "en"), default="uk")
    parser.add_argument("--overwrite", action="store_true",
                        help="Replace an existing components/sanotts/voices/<lang>-<speaker> bundle")
    parser.add_argument("--select", action="store_true",
                        help="Select the installed voice in sdkconfig")
    args = parser.parse_args()
    install(args.export, args.speaker, language=args.language,
            overwrite=args.overwrite, select=args.select)