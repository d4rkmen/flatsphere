#!/usr/bin/env python3
"""Reproducible Ukrainian dataset/teacher preparation; not an MCU exporter.

Large artifacts stay in this directory's ignored cache/ and work/ trees.
Never uploads recordings, launches cloud jobs, or modifies the firmware.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import subprocess
import sys
import unicodedata
import urllib.request
import wave
import zipfile

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parents[1]
SOURCES = json.loads((ROOT / "sources.json").read_text())
CACHE = ROOT / "cache"
WORK = ROOT / "work"


def read_rows(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8-sig").splitlines() if line.strip()]


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_rows(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows), encoding="utf-8")


def digest(path: Path) -> str:
    with path.open("rb") as handle:
        return hashlib.file_digest(handle, "sha256").hexdigest()


def download(url: str, path: Path, expected_hash: str | None = None,
             expected_size: int | None = None) -> dict:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        temp = path.with_name(path.name + ".part")
        print(f"Downloading {url}", flush=True)
        try:
            request = urllib.request.Request(url, headers={"User-Agent": "FlatSphere-UA-preparation/1"})
            with urllib.request.urlopen(request, timeout=120) as source, temp.open("wb") as target:
                while chunk := source.read(1024 * 1024):
                    target.write(chunk)
            if expected_size is not None and temp.stat().st_size != expected_size:
                raise ValueError(f"Wrong size: {path}")
            if expected_hash and digest(temp) != expected_hash:
                raise ValueError(f"SHA-256 mismatch: {path}")
            temp.replace(path)
        finally:
            temp.unlink(missing_ok=True)
    sha = digest(path)
    if expected_hash and sha != expected_hash:
        raise ValueError(f"Cached SHA-256 mismatch: {path}; inspect/remove the cache file")
    if expected_size is not None and path.stat().st_size != expected_size:
        raise ValueError(f"Cached size mismatch: {path}")
    return {"url": url, "path": str(path.relative_to(ROOT)), "sha256": sha, "bytes": path.stat().st_size}


def fetch(role: str, audio: bool) -> None:
    profile = SOURCES["voices"][role]
    speaker = profile["speaker"]
    dataset = f"https://huggingface.co/datasets/{SOURCES['dataset']}/resolve/{SOURCES['dataset_revision']}"
    teacher = f"https://huggingface.co/{SOURCES['teacher_repository']}/resolve/{SOURCES['teacher_revision']}/uk/uk_UA/{speaker}/high"
    docs = f"https://raw.githubusercontent.com/egorsmkv/ukrainian-tts-datasets/{SOURCES['dataset_docs_revision']}"
    records = []
    records.append(download(f"{dataset}/voices/{speaker}/train.jsonl", CACHE / speaker / "source.jsonl"))
    records.append(download(f"{docs}/{speaker}/README.md", CACHE / speaker / "DATASET_CARD.md"))
    records.append(download(f"{teacher}/MODEL_CARD", CACHE / speaker / "MODEL_CARD.md"))
    stem = f"uk_UA-{speaker}-high.onnx"
    records.append(download(f"{teacher}/{stem}.json", CACHE / speaker / f"{stem}.json"))
    records.append(download(f"{teacher}/{stem}", CACHE / speaker / stem,
                            profile["model_sha256"], profile["model_bytes"]))
    if audio:
        records.append(download(f"{dataset}/data/{speaker}.zip", CACHE / speaker / "recordings.zip",
                                profile["archive_sha256"], profile["archive_bytes"]))
    write_json(WORK / speaker / "downloads.json", records)


def canonical_text(text: str) -> str:
    # Compare accented/unaccented versions without leaking the same sentence
    # across splits. Preserve the original transcript and stress separately.
    value = unicodedata.normalize("NFD", text).replace("\u0301", "")
    value = unicodedata.normalize("NFC", value).casefold()
    for apostrophe in ("’", "ʼ", "`", "‘"):
        value = value.replace(apostrophe, "'")
    return " ".join(value.split())


def split_for(text: str) -> str:
    bucket = int(hashlib.sha256(canonical_text(text).encode()).hexdigest()[:8], 16) % 100
    return "test" if bucket < 5 else "eval" if bucket < 10 else "train"


def prepare(role: str) -> None:
    speaker = SOURCES["voices"][role]["speaker"]
    raw = read_rows(CACHE / speaker / "source.jsonl")
    held_out = {canonical_text(row["text"]) for row in read_rows(ROOT / "texts/clock_eval.jsonl")}
    seen: set[str] = set()
    splits: dict[str, list[dict]] = {name: [] for name in ("train", "eval", "test")}
    rejected = 0
    archive_path = CACHE / speaker / "recordings.zip"
    audio_names: dict[str, str] = {}
    if archive_path.exists():
        with zipfile.ZipFile(archive_path) as archive:
            for item in archive.infolist():
                if item.is_dir() or not item.filename.endswith((".ogg", ".wav", ".opus")):
                    continue
                name = PurePosixPath(item.filename).name
                if name in audio_names:
                    raise ValueError(f"Ambiguous recording basename: {name}")
                audio_names[name] = item.filename
    for row in raw:
        text = unicodedata.normalize("NFC", row["text"]).strip()
        key = canonical_text(text)
        duration = float(row["duration"])
        audio_name = PurePosixPath(row["audio"]).name
        if (not key or key in seen or key in held_out or not math.isfinite(duration)
                or not 0.5 <= duration <= 6.0 or len(text.encode("utf-8")) > 191):
            rejected += 1
            continue
        if audio_names and audio_name not in audio_names:
            raise ValueError(f"Transcript has no matching recording: {audio_name}")
        seen.add(key)
        splits[split_for(text)].append({
            "id": f"{speaker}/{audio_name}", "text": text,
            "text_stressed": row.get("text_stressed", ""), "duration_sec": duration,
            "speaker": speaker, "voice_role": role, "language": "uk",
            "audio_member": audio_names.get(audio_name, row["audio"]),
            "dataset": SOURCES["dataset"], "revision": SOURCES["dataset_revision"],
            "license": "Apache-2.0", "text_sha256": hashlib.sha256(key.encode()).hexdigest(),
        })
    for name, rows in splits.items():
        rows.sort(key=lambda row: row["text_sha256"])
        write_rows(WORK / speaker / f"{name}.jsonl", rows)
    report = {"source_rows": len(raw), "rejected_or_duplicate": rejected,
              "archive_recordings": len(audio_names),
              "splits": {name: {"rows": len(rows), "hours": sum(r["duration_sec"] for r in rows) / 3600}
                         for name, rows in splits.items()},
              "note": "Source-duration/UTF-8 filters, not predicted-duration or perceptual quality guarantees."}
    write_json(WORK / speaker / "dataset_report.json", report)
    print(speaker, json.dumps(report, ensure_ascii=False), flush=True)


def teacher_paths(role: str) -> tuple[str, Path, Path]:
    speaker = SOURCES["voices"][role]["speaker"]
    model = CACHE / speaker / f"uk_UA-{speaker}-high.onnx"
    return speaker, model, model.with_suffix(".onnx.json")


def probe(role: str) -> None:
    import numpy as np
    import onnx
    from piper.voice import PiperVoice
    from piper.config import SynthesisConfig

    speaker, model, config_path = teacher_paths(role)
    config = json.loads(config_path.read_text())
    assert config["phoneme_type"] == "espeak" and config["espeak"]["voice"] == "uk"
    assert config["num_speakers"] == 1 and config["audio"]["sample_rate"] == 22050
    graph = onnx.load(str(model))
    onnx.checker.check_model(graph)
    voice = PiperVoice.load(str(model), config_path=str(config_path))
    synthesis = SynthesisConfig(noise_scale=0, noise_w_scale=0, length_scale=1)
    out = WORK / speaker / "teacher_samples"
    out.mkdir(parents=True, exist_ok=True)
    samples = []
    for row in read_rows(ROOT / "texts/clock_eval.jsonl"):
        path = out / f"{row['id']}.wav"
        with wave.open(str(path), "wb") as wav:
            voice.synthesize_wav(row["text"], wav, syn_config=synthesis)
        with wave.open(str(path), "rb") as wav:
            pcm = np.frombuffer(wav.readframes(wav.getnframes()), dtype="<i2")
            seconds = wav.getnframes() / wav.getframerate()
        if not pcm.size or not np.any(pcm):
            raise ValueError(f"Empty or silent teacher result: {path}")
        samples.append({"id": row["id"], "seconds": seconds, "peak": int(np.abs(pcm.astype(np.int32)).max())})
    report = {"speaker": speaker, "sample_rate": 22050,
              "inputs": [item.name for item in graph.graph.input],
              "ceil_nodes": [node.output[0] for node in graph.graph.node if node.op_type == "Ceil"],
              "mapped_id_max": max(i for ids in config["phoneme_id_map"].values() for i in ids),
              "samples": samples, "listening_verified": False}
    write_json(WORK / speaker / "teacher_report.json", report)
    write_json(WORK / speaker / "phoneme_id_map.json", config["phoneme_id_map"])
    print(f"{speaker}: generated {len(samples)} teacher listening samples", flush=True)


def upstream_path(value: str | None) -> Path:
    upstream = Path(value).resolve() if value else PROJECT / "build/_deps/sano_source-src"
    if not (upstream / "tools/build_piper_vits_roota_probe_pack.py").exists():
        raise FileNotFoundError("Pinned sanoTTS source missing; configure firmware or pass --upstream")
    return upstream


def run_tool(upstream: Path, script: str, arguments: list[str]) -> None:
    path = upstream / "tools" / script
    if not path.is_file():
        raise FileNotFoundError(f"Missing upstream tool: {script}")
    env = os.environ.copy()
    env["PYTHONPATH"] = str(upstream / "src") + os.pathsep + env.get("PYTHONPATH", "")
    subprocess.run([sys.executable, str(path), *arguments], cwd=upstream, env=env, check=True)


def pack(role: str, upstream: Path, rows: int) -> None:
    speaker, model, config = teacher_paths(role)
    out = WORK / speaker
    for split in ("train", "eval"):
        source = out / f"{split}.jsonl"
        if len(read_rows(source)) < rows:
            raise ValueError(f"Not enough {split} rows for {speaker}")
        target = out / f"pack-{split}"
        if target.exists():
            raise FileExistsError(f"Refusing to mix old/new pack artifacts: {target}")
        run_tool(upstream, "build_piper_vits_roota_probe_pack.py", [
            "--model", str(model), "--config", str(config),
            "--source-jsonl", str(source), "--out-dir", str(target),
            "--max-rows", str(rows), "--tensor-mode", "decoder",
            "--noise-scale", "0", "--noise-w", "0", "--length-scale", "1"])


def validate_pack(role: str) -> None:
    import numpy as np
    speaker = SOURCES["voices"][role]["speaker"]
    report = {}
    for split in ("train", "eval"):
        directory = WORK / speaker / f"pack-{split}"
        tensors = list(directory.rglob("*.npz"))
        if not tensors:
            raise ValueError(f"No pack tensors: {directory}")
        checked = []
        for path in tensors:
            with np.load(path, allow_pickle=False) as data:
                if "generator_input" not in data:
                    continue
                latent = data["generator_input"]
                durations = data["w_ceil"]
                ids = data["phoneme_ids"]
                if (latent.ndim != 3 or latent.shape[1] != 192 or
                        int(durations.sum()) != latent.shape[-1] or
                        not np.isfinite(latent).all() or not np.isfinite(durations).all()):
                    raise ValueError(f"Invalid latent/alignment: {path}")
                checked.append({"file": path.name, "frames": latent.shape[-1],
                                "tokens": ids.size, "max_id": int(ids.max())})
        if not checked:
            raise ValueError(f"No decoder tensors checked: {directory}")
        report[split] = checked
    write_json(WORK / speaker / "pack_validation.json", report)
    print(f"{speaker}: aligned finite 192-channel teacher packs verified")


def duration(role: str, upstream: Path, steps: int, device: str) -> None:
    speaker = SOURCES["voices"][role]["speaker"]
    out = WORK / speaker
    target = out / f"duration-{steps}-{device}"
    if target.exists():
        raise FileExistsError(f"Refusing to overwrite training run: {target}")
    run_tool(upstream, "train_roota_piper_duration_student.py", [
        "--pack-dir", str(out / "pack-train"), "--eval-pack-dir", str(out / "pack-eval"),
        "--out-dir", str(target), "--hidden", "32", "--depth", "3", "--kernel-size", "5",
        "--steps", str(steps), "--device", device])


def audit(upstream: Path) -> None:
    missing = [name for name in ("qat_ste.py", "train_roota_joint_c_finetune.py",
               "export_front_q8.py", "export_fsd_golden.py", "export_fsd_q8.py")
               if not (upstream / "tools" / name).is_file()]
    report = {"upstream": str(upstream), "expected_revision": SOURCES["sanotts_revision"],
              "missing_tools": missing, "mcu_export_ready": False,
              "other_requirements": ["LRC 40-channel code-target generation", "voice-specific calibration",
                                     "Ukrainian runtime mapping, espeak data and translated announcements",
                                     "C golden gate and listening acceptance"]}
    write_json(WORK / "upstream_audit.json", report)
    print(json.dumps(report, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("fetch", "prepare", "probe", "pack", "validate-pack", "duration", "audit"))
    parser.add_argument("--voice", choices=("male", "female", "both"), default="both")
    parser.add_argument("--audio", action="store_true", help="also download public recording archives (~364 MB total)")
    parser.add_argument("--upstream", help="path to pinned sanoTTS checkout")
    parser.add_argument("--rows", type=int, default=4, help="rows per pack split; default is a smoke test only")
    parser.add_argument("--steps", type=int, default=20, help="duration steps; default is a smoke test only")
    parser.add_argument("--device", choices=("cpu", "mps", "cuda"), default="cpu")
    args = parser.parse_args()
    if args.rows < 1 or args.steps < 1:
        parser.error("rows and steps must be positive")
    if args.stage == "audit":
        audit(upstream_path(args.upstream))
        return
    for role in (SOURCES["voices"] if args.voice == "both" else [args.voice]):
        if args.stage == "fetch": fetch(role, args.audio)
        elif args.stage == "prepare": prepare(role)
        elif args.stage == "probe": probe(role)
        elif args.stage == "pack": pack(role, upstream_path(args.upstream), args.rows)
        elif args.stage == "validate-pack": validate_pack(role)
        elif args.stage == "duration": duration(role, upstream_path(args.upstream), args.steps, args.device)


if __name__ == "__main__":
    main()