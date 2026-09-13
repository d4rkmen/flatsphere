#!/usr/bin/env python3
"""R7-shaped LRC/FSD experiment using pinned upstream arithmetic unchanged.

This does not export an MCU model. Run each stage explicitly; existing runs
are never overwritten. The default 100-step run is a smoke experiment only.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

from cook import ROOT, WORK, digest, read_rows, run_tool, teacher_paths, upstream_path, write_json, write_rows

PINNED_FILES = {
    "tools/train_roota_piper_decoder_student.py": "f30f2bfedcd5ea152fc0dde7a4698d21c7c6d9ec2903859d141938f928e0662b",
    "src/saanotts/models/fsd.py": "c2c27774a3aeef81ec5016f02cb71d73a2d8b74e51de429adcc8bea1b51b0fb1",
}


def load_decoder_module(upstream: Path):
    """Load real FSD exports without the unrelated broken models/__init__.py.

    No fake neural operators or missing-module substitutes: fsd.py is loaded
    verbatim as the compatibility module the trainer already imports.
    """
    for relative, expected in PINNED_FILES.items():
        if digest(upstream / relative) != expected:
            raise ValueError(f"Upstream changed; review before training: {relative}")
    for name, relative in (
        ("roota_fsd_blocks", "src/saanotts/models/fsd.py"),
        ("ua_decoder_trainer", "tools/train_roota_piper_decoder_student.py"),
    ):
        spec = importlib.util.spec_from_file_location(name, upstream / relative)
        if spec is None or spec.loader is None:
            raise ImportError(relative)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return module


def fresh(path: Path) -> None:
    if path.exists():
        raise FileExistsError(f"Refusing to overwrite existing artifacts: {path}")


def decoder_cut(role: str, upstream: Path) -> None:
    speaker, model, _ = teacher_paths(role)
    out = WORK / speaker / "decoder-cut"
    fresh(out)
    run_tool(upstream, "extract_piper_vits_decoder_cut.py", [
        "--model", str(model), "--pack-dir", str(WORK / speaker / "pack-eval"),
        "--out-dir", str(out), "--latent-channels", "192"])


def training_args(role: str, steps: int, device: str, out: Path,
                  pack_root: Path | None = None, batch_size: int = 1,
                  crop_frames: int = 32, evaluate: bool = False,
                  spectral_weight: float = 0.5) -> list[str]:
    speaker, model, _ = teacher_paths(role)
    voice = WORK / speaker
    decoder = voice / "decoder-cut" / f"{model.stem}-decoder-from-generator-input.onnx"
    packs = pack_root or voice
    args = ["--pack-dir", str(packs / "pack-train"), "--eval-pack-dir", str(packs / "pack-eval"),
            "--teacher-decoder", str(decoder), "--out-dir", str(out),
            "--variant", "lrc", "--lrc-code-dim", "40", "--lrc-encoder-hidden", "64",
            "--fsd-dim", "76", "--fsd-blocks", "5", "--fsd-film-rank", "12",
            "--fsd-head-rank", "48", "--istft-n-fft", "1024",
            "--device", device, "--steps", str(steps), "--batch-size", str(batch_size),
            "--crop-frames", str(crop_frames), "--lr", "0.0002", "--spectral-weight", str(spectral_weight),
            "--log-interval", "100", "--render-rows", "4", "--seed", "4242"]
    if not evaluate:
        args.append("--skip-chunk-eval")
    return args


def larger_packs(role: str, upstream: Path, name: str, train_rows: int, eval_rows: int) -> None:
    speaker, model, config = teacher_paths(role)
    out = WORK / speaker / name
    fresh(out)
    for split, count in (("train", train_rows), ("eval", eval_rows)):
        source = WORK / speaker / f"{split}.jsonl"
        if len(read_rows(source)) < count:
            raise ValueError(f"Not enough {split} rows")
    out.mkdir(parents=True)
    write_json(out / "selection.json", {"speaker": speaker, "train_rows": train_rows,
               "eval_rows": eval_rows, "selection": "first rows of deterministic disjoint hash splits"})
    for split, count in (("train", train_rows), ("eval", eval_rows)):
        run_tool(upstream, "build_piper_vits_roota_probe_pack.py", [
            "--model", str(model), "--config", str(config),
            "--source-jsonl", str(WORK / speaker / f"{split}.jsonl"),
            "--out-dir", str(out / f"pack-{split}"), "--max-rows", str(count),
            "--tensor-mode", "decoder", "--noise-scale", "0", "--noise-w", "0",
            "--length-scale", "1", "--progress-interval", "32"])


def train(role: str, upstream: Path, steps: int, device: str,
          pack_root: Path | None = None, run_name: str | None = None,
          batch_size: int = 1, crop_frames: int = 32, evaluate: bool = False,
          spectral_weight: float = 0.5) -> None:
    speaker, _, _ = teacher_paths(role)
    out = WORK / speaker / (run_name or f"lrc-r7-{steps}-{device}")
    fresh(out)
    args = training_args(role, steps, device, out, pack_root, batch_size, crop_frames, evaluate, spectral_weight)
    out.mkdir(parents=True)
    write_json(out / "invocation.json", {"args": args, "source_sha256": PINNED_FILES,
               "purpose": "oracle LRC experiment; not full text-to-speech or quality acceptance"})
    # A subprocess records logs and isolates the narrow compatibility import.
    command = [sys.executable, str(Path(__file__).resolve()), "_train",
               "--upstream", str(upstream), "--", *args]
    with (out / "console.log").open("w") as log:
        subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
    print(f"Training complete; inspect {out / 'train-report.json'} and {out / 'index.html'}")


def encode_targets(role: str, upstream: Path, run: Path) -> None:
    """Create the acoustic trainer's manifest/NPZ interface, tied to one encoder.

    Only load checkpoints produced locally by this experiment. weights_only
    permits plain tensor/config checkpoints; never enable arbitrary unpickling.
    """
    import numpy as np
    import torch
    trainer = load_decoder_module(upstream)
    checkpoint = run / "decoder-student.pt"
    # Upstream stores argparse Paths; explicitly allow only that known type.
    from pathlib import PosixPath
    with torch.serialization.safe_globals([PosixPath]):
        state = torch.load(checkpoint, map_location="cpu", weights_only=True)
    speaker, _, _ = teacher_paths(role)
    train_pack = Path(state["train_args"]["pack_dir"]).resolve()
    eval_pack = Path(state["train_args"]["eval_pack_dir"]).resolve()
    voice_root = (WORK / speaker).resolve()
    if not all(p.is_relative_to(voice_root) for p in (train_pack, eval_pack)):
        raise ValueError("Checkpoint voice/training pack does not match the selected voice")
    encoder = trainer.LrcEncoder(in_channels=192, hidden=64, code_dim=40)
    encoder.load_state_dict(state["lrc_encoder_state_dict"], strict=True)
    if not all(torch.isfinite(value).all() for value in encoder.state_dict().values()):
        raise ValueError("Nonfinite encoder checkpoint")
    encoder.eval()
    destination = run / "code-targets"
    fresh(destination)
    checkpoint_sha = digest(checkpoint)
    counts = {}
    for split in ("train", "eval"):
        pack = train_pack if split == "train" else eval_pack
        rows = json.loads((pack / "rows.json").read_text())
        target = destination / split
        target.mkdir(parents=True)
        manifest = []
        for row in rows:
            for chunk in row["chunks"]:
                path = Path(chunk["tensor_npz"])
                with np.load(path, allow_pickle=False) as tensors:
                    latent = tensors["generator_input"].astype(np.float32)
                    durations = tensors["w_ceil"]
                    if (latent.ndim != 3 or latent.shape[:2] != (1, 192) or
                            not np.isfinite(latent).all() or not np.isfinite(durations).all() or
                            durations.sum() != latent.shape[-1]):
                        raise ValueError(f"Invalid input/alignment: {path}")
                with torch.inference_mode():
                    code = encoder(torch.from_numpy(latent)).numpy()
                if code.shape != (1, 40, latent.shape[-1]) or not np.isfinite(code).all():
                    raise ValueError(f"Invalid encoder result: {path}")
                output = target / f"{len(manifest):06d}.npz"
                np.savez_compressed(output, c=code)
                with np.load(output, allow_pickle=False) as saved:
                    if not np.array_equal(saved["c"], code):
                        raise ValueError(f"Target round-trip mismatch: {output}")
                manifest.append({"row_id": row["row_id"], "chunk_index": chunk["chunk_index"],
                                 "target_npz": str(output.resolve()), "source_npz": str(path),
                                 "source_sha256": digest(path), "encoder_checkpoint_sha256": checkpoint_sha,
                                 "frames": code.shape[-1], "target_key": "c"})
        if not manifest:
            raise ValueError(f"No chunks in {pack}")
        write_rows(target / "manifest.jsonl", manifest)
        write_json(target / "summary.json", {"target_key": "c", "channels": 40,
               "chunks": len(manifest), "encoder_checkpoint_sha256": checkpoint_sha,
               "source_pack": str(pack), "quality_accepted": False})
        counts[split] = len(manifest)
    write_json(destination / "report.json", {"counts": counts, "checkpoint_sha256": checkpoint_sha,
               "shape": "[1,40,T]", "round_trip_exact": True,
               "quality_accepted": False, "note": "Regenerate targets whenever the encoder changes."})
    print(f"Validated code targets: {counts}; {destination}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("cut", "packs", "train", "encode", "_train"))
    parser.add_argument("--voice", choices=("male", "female"), default="female")
    parser.add_argument("--upstream")
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--device", choices=("cpu", "mps"), default="cpu")
    parser.add_argument("--run", type=Path, help="local LRC run directory for encoding")
    parser.add_argument("--pack-root", type=Path, help="directory containing pack-train and pack-eval")
    parser.add_argument("--name", help="fresh voice-local pack/run directory name")
    parser.add_argument("--train-rows", type=int, default=256)
    parser.add_argument("--eval-rows", type=int, default=32)
    parser.add_argument("--batch-size", type=int, default=1)
    parser.add_argument("--crop-frames", type=int, default=32)
    parser.add_argument("--evaluate", action="store_true", help="evaluate complete train/eval chunks")
    parser.add_argument("--spectral-weight", type=float, default=0.5)
    args, extra = parser.parse_known_args()
    upstream = upstream_path(args.upstream)
    if args.stage == "_train":
        if not extra or extra[0] != "--":
            parser.error("Internal training arguments must follow --")
        trainer = load_decoder_module(upstream)
        sys.argv = [str(upstream / "tools/train_roota_piper_decoder_student.py"), *extra[1:]]
        trainer.main()
        return
    if (extra or min(args.steps, args.train_rows, args.eval_rows, args.batch_size) < 1
            or args.crop_frames < 5 or not 0 <= args.spectral_weight <= 100):
        parser.error("Unexpected arguments or nonpositive steps")
    if args.name and (Path(args.name).name != args.name or args.name in (".", "..")):
        parser.error("--name must be a single directory name")
    if args.stage == "cut": decoder_cut(args.voice, upstream)
    elif args.stage == "packs":
        if not args.name: parser.error("packs requires --name")
        larger_packs(args.voice, upstream, args.name, args.train_rows, args.eval_rows)
    elif args.stage == "train":
        train(args.voice, upstream, args.steps, args.device,
              args.pack_root.resolve() if args.pack_root else None, args.name,
              args.batch_size, args.crop_frames, args.evaluate, args.spectral_weight)
    elif args.stage == "encode":
        if args.run is None: parser.error("encode requires --run")
        encode_targets(args.voice, upstream, args.run.resolve())


if __name__ == "__main__":
    main()