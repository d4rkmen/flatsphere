#!/usr/bin/env python3
"""Long decoder training in resumable segments; reuses the full teacher packs.

The upstream decoder trainer has no periodic checkpoint save, so this launcher
chains weight-only continuation via --init-decoder-checkpoint. Each segment
saves a final checkpoint; the next segment warm-starts from it (optimizer state
is reset, which can cause a brief loss bump). No training starts unless the
`train` stage is invoked explicitly.

`train --resume` skips only contiguous segments with verified completion.json
receipts and an exactly matching root config. Legacy completed artifacts without
a receipt are rejected, never silently adopted. Incomplete attempts are renamed
to unique .interrupted-* directories and retrained from the last completed
segment (or from scratch for segment zero); optimizer/RNG progress is not resumed.
Smoke runs use a separate -smoke root and cannot be resumed.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
from uuid import uuid4

from cook import ROOT, WORK, digest, teacher_paths, upstream_path, write_json
from lrc_experiment import fresh, load_decoder_module

DECODER_TRAINER = "tools/train_roota_piper_decoder_student.py"
DECODER_SHA = "f30f2bfedcd5ea152fc0dde7a4698d21c7c6d9ec2903859d141938f928e0662b"
COMPLETION_ARTIFACTS = ("decoder-student.pt", "lrc-encoder.pt", "train-report.json")


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    try:
        write_json(temporary, value)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def completion_receipt(segment: Path, index: int, steps: int,
                       config_sha: str, previous: Path | None) -> dict:
    """Validate outputs without loading checkpoint pickle data."""
    artifacts = {}
    for name in (*COMPLETION_ARTIFACTS, "invocation.json"):
        path = segment / name
        if not path.is_file() or path.stat().st_size == 0:
            raise ValueError(f"Missing or empty completion artifact: {path}")
        artifacts[name] = digest(path)
    report = json.loads((segment / "train-report.json").read_text())
    if not isinstance(report, dict) or report.get("steps") != steps:
        raise ValueError(f"Unexpected completed steps: {segment}")
    invocation = json.loads((segment / "invocation.json").read_text())
    if (not isinstance(invocation, dict) or invocation.get("segment") != index
            or invocation.get("trainer_sha256") != DECODER_SHA
            or invocation.get("init_from") != (str(previous) if previous else None)):
        raise ValueError(f"Unexpected invocation: {segment}")
    return {"version": 1, "segment": index, "steps": steps,
            "trainer_sha256": DECODER_SHA, "config_sha256": config_sha,
            "init_decoder_sha256": digest(previous) if previous else None,
            "artifacts_sha256": artifacts}


def resume_prefix(root: Path, segments: int, steps: int, config_sha: str) -> int:
    """Read-only preflight of the entire run, before any rename or write."""
    expected = {f"seg-{index:05d}" for index in range(segments)}
    for path in root.glob("seg-*"):
        if ".interrupted-" not in path.name and path.name not in expected:
            raise ValueError(f"Unexpected segment: {path}")
    completed = 0
    previous = None
    for index in range(segments):
        segment = root / f"seg-{index:05d}"
        if not segment.exists():
            continue
        if not segment.is_dir() or segment.is_symlink():
            raise ValueError(f"Invalid segment directory: {segment}")
        receipt = segment / "completion.json"
        if receipt.exists():
            if index != completed:
                raise ValueError(f"Non-contiguous completed segment: {segment}")
            actual = completion_receipt(segment, index, steps, config_sha, previous)
            if json.loads(receipt.read_text()) != actual:
                raise ValueError(f"Completion receipt/hash mismatch: {segment}")
            completed += 1
            previous = segment / "decoder-student.pt"
        elif all((segment / name).exists() for name in COMPLETION_ARTIFACTS):
            raise ValueError(f"Completed-looking segment lacks completion.json: {segment}; "
                             "legacy outputs cannot be resumed automatically")
    return completed


def load_config(path: Path) -> dict:
    cfg = json.loads(path.read_text())
    if cfg["voice"] not in ("male", "female") or cfg["device"] not in ("cpu", "mps", "cuda"):
        raise ValueError("Invalid voice/device")
    for key in ("pack_name", "run_name"):
        if not cfg[key] or Path(cfg[key]).name != cfg[key] or cfg[key] in (".", ".."):
            raise ValueError(f"Invalid {key}")
    for key in ("total_steps", "segment_steps", "batch_size", "crop_frames", "log_interval"):
        if type(cfg[key]) is not int or cfg[key] < 1:
            raise ValueError(f"Invalid {key}")
    if cfg["total_steps"] % cfg["segment_steps"] != 0:
        raise ValueError("total_steps must be a multiple of segment_steps")
    if not 0 < cfg["learning_rate"] <= 0.01 or not 0 <= cfg["spectral_weight"] <= 100:
        raise ValueError("Invalid learning rate or spectral weight")
    arch = cfg["architecture"]
    expected = {"variant": "lrc", "lrc_code_dim": 40, "lrc_encoder_hidden": 64,
                "fsd_dim": 76, "fsd_blocks": 5, "fsd_film_rank": 12,
                "fsd_head_rank": 48, "istft_n_fft": 1024}
    if arch != expected:
        raise ValueError("Architecture must match the R7 LRC/FSD target")
    return cfg


def pack_root(cfg: dict) -> Path:
    speaker, _, _ = teacher_paths(cfg["voice"])
    root = WORK / speaker / cfg["pack_name"]
    for split in ("train", "eval"):
        if not (root / f"pack-{split}" / "rows.json").is_file():
            raise ValueError(f"Missing {split} pack; run duration_long prepare first")
    return root


def run_root(cfg: dict) -> Path:
    speaker, _, _ = teacher_paths(cfg["voice"])
    return WORK / speaker / cfg["run_name"]


def logged(command: list[str], log_path: Path) -> None:
    with log_path.open("w") as log:
        process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1)
        try:
            assert process.stdout is not None
            for line in process.stdout:
                print(line, end="", flush=True)
                log.write(line)
                log.flush()
            if process.wait() != 0:
                raise subprocess.CalledProcessError(process.returncode, command)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait()


def check(cfg: dict, upstream: Path) -> dict:
    if digest(upstream / DECODER_TRAINER) != DECODER_SHA:
        raise ValueError("Decoder trainer revision changed; review before training")
    # Load the module once to confirm the hash-pinned compatibility import works.
    module = load_decoder_module(upstream)
    if not all(hasattr(module, name) for name in ("DecoderStudent", "LrcEncoder")):
        raise ValueError("Decoder module missing expected symbols")
    root = pack_root(cfg)
    segments = cfg["total_steps"] // cfg["segment_steps"]
    return {"run_directory": str(run_root(cfg)), "segments": segments,
            "segment_steps": cfg["segment_steps"], "total_steps": cfg["total_steps"],
            "device": cfg["device"], "pack_root": str(root),
            "checkpoint": "one decoder-student.pt + lrc-encoder.pt per segment"}


def train(cfg: dict, upstream: Path, smoke: bool, resume: bool = False) -> None:
    if smoke:
        if resume:
            raise ValueError("--resume is only supported for train, not smoke")
        cfg = dict(cfg, run_name=cfg["run_name"] + "-smoke", segment_steps=5)
    check(cfg, upstream)
    speaker, model, _ = teacher_paths(cfg["voice"])
    voice_root = WORK / speaker
    packs = pack_root(cfg)
    decoder_cut = Path(cfg.get("decoder_cut", voice_root / "decoder-cut" / f"{model.stem}-decoder-from-generator-input.onnx"))
    if not decoder_cut.is_file():
        raise FileNotFoundError("Missing decoder cut; run lrc_experiment cut first")
    segments = 1 if smoke else cfg["total_steps"] // cfg["segment_steps"]
    steps = min(cfg["segment_steps"], 5) if smoke else cfg["segment_steps"]
    root = run_root(cfg)
    config_path = root / "config.json"
    expected_config = {**cfg, "trainer_sha256": DECODER_SHA,
                       "resume": "weight-only chained continuation, optimizer reset per segment"}
    if root.exists():
        if not config_path.is_file() or json.loads(config_path.read_text()) != expected_config:
            raise ValueError(f"Root config mismatch or missing: {config_path}; no files changed")
        if not resume:
            for index in range(segments):
                fresh(root / f"seg-{index:05d}")
    else:
        root.mkdir(parents=True)
        atomic_json(config_path, expected_config)
    config_sha = digest(config_path)
    completed = resume_prefix(root, segments, steps, config_sha) if resume else 0
    previous = root / f"seg-{completed - 1:05d}" / "decoder-student.pt" if completed else None
    if completed:
        print(f"Verified {completed}/{segments} completed segments; skipping them.")
    for index in range(completed, segments):
        segment = root / f"seg-{index:05d}"
        if resume and segment.exists():
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
            archived = segment.with_name(f"{segment.name}.interrupted-{stamp}-{uuid4().hex}")
            segment.rename(archived)
            print(f"Preserved incomplete attempt: {archived}; retrying weight-only, optimizer reset.")
        fresh(segment)
        segment.mkdir()
        args = ["--pack-dir", str(packs / "pack-train"), "--eval-pack-dir", str(packs / "pack-eval"),
                "--teacher-decoder", str(decoder_cut), "--out-dir", str(segment),
                "--variant", "lrc", "--lrc-code-dim", "40", "--lrc-encoder-hidden", "64",
                "--fsd-dim", "76", "--fsd-blocks", "5", "--fsd-film-rank", "12",
                "--fsd-head-rank", "48", "--istft-n-fft", "1024",
                "--device", cfg["device"], "--steps", str(steps), "--batch-size", str(cfg["batch_size"]),
                "--crop-frames", str(cfg["crop_frames"]), "--lr", str(cfg["learning_rate"]),
                "--spectral-weight", str(cfg["spectral_weight"]), "--log-interval", str(cfg["log_interval"]),
                "--seed", str(cfg["seed"]), "--render-rows", "4" if index == segments - 1 else "0"]
        if index < segments - 1 and not smoke:
            args.append("--skip-chunk-eval")
        if previous is not None:
            args += ["--init-decoder-checkpoint", str(previous)]
        command = [sys.executable, "-u", str(Path(__file__).resolve()), "_train",
                   "--upstream", str(upstream), "--", *args]
        write_json(segment / "invocation.json", {"segment": index, "init_from": str(previous) if previous else None,
                   "args": args, "trainer_sha256": DECODER_SHA})
        logged(command, segment / "console.log")
        receipt = completion_receipt(segment, index, steps, config_sha, previous)
        atomic_json(segment / "completion.json", receipt)
        previous = segment / "decoder-student.pt"
        print(f"segment {index}/{segments} complete -> {previous}")
    print(f"latest checkpoint: {previous}")
    print("Next: regenerate code targets (lrc_experiment encode), then run long acoustic training.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("check", "smoke", "train", "_train"))
    parser.add_argument("--config", type=Path, default=ROOT / "decoder-long.json")
    parser.add_argument("--upstream")
    parser.add_argument("--resume", action="store_true",
                        help="train only: verify and skip completed segments; preserve/retry incomplete "
                             "attempts with weight-only continuation (optimizer reset)")
    args, extra = parser.parse_known_args()
    if args.resume and args.stage != "train":
        parser.error("--resume is only supported for train")
    upstream = upstream_path(args.upstream)
    if args.stage == "_train":
        if not extra or extra[0] != "--":
            parser.error("Internal training arguments must follow --")
        trainer = load_decoder_module(upstream)
        sys.argv = [str(upstream / DECODER_TRAINER), *extra[1:]]
        trainer.main()
        return
    if extra:
        parser.error("Unexpected arguments")
    cfg = load_config(args.config)
    if args.stage == "check":
        print(json.dumps(check(cfg, upstream), indent=2))
    elif args.stage == "smoke":
        train(cfg, upstream, smoke=True)
    elif args.stage == "train":
        train(cfg, upstream, smoke=False, resume=args.resume)


if __name__ == "__main__":
    main()
