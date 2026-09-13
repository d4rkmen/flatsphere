#!/usr/bin/env python3
"""Experimental R7-shaped acoustic40 regression; no QAT/export or quality promotion.

train --resume requires the exact original invocation and input hashes. It archives
interrupted attempts and loads verified weights for the remaining additional steps;
optimizer/RNG state resets. Canonical final paths and decoder lineage are retained.
Checkpoint/report steps are local; invocation.json and completion.json track totals.
Legacy runs without recovery provenance are refused rather than guessed.
"""
from __future__ import annotations

import argparse
import inspect
import json
from pathlib import Path
import subprocess
import sys
import types
import wave

from cook import ROOT, WORK, digest, read_rows, teacher_paths, upstream_path, write_json
from lrc_experiment import fresh, load_decoder_module
from duration_long import input_hashes, recover_training, restricted_checkpoint

TRAINER = "tools/train_roota_piper_latent_student.py"
TRAINER_SHA = "076b1ae490931c14449ab36fa9aa75c1985a1efbe196108918b1236850b5d8be"


def defer_qat_import(source: str) -> str:
    old = "import qat_ste  # noqa: E402\n"
    branch = '    if str(args.qat) == "c-int8":\n'
    if source.count(old) != 1 or source.count(branch) != 1:
        raise ValueError("QAT patch anchors changed; review upstream")
    return source.replace(old, "", 1).replace(branch, branch + "        import qat_ste\n", 1)


def load_acoustic_module(upstream: Path):
    path = upstream / TRAINER
    if digest(path) != TRAINER_SHA:
        raise ValueError("Acoustic trainer changed; review before applying import repair")
    source = defer_qat_import(path.read_text())
    module = types.ModuleType("ua_acoustic_trainer")
    module.__file__ = str(path)
    sys.modules[module.__name__] = module
    exec(compile(source, str(path), "exec"), module.__dict__)
    # The pinned helper otherwise uses weights_only=False. All child loads,
    # not merely parent preflight, must use restricted unpickling.
    module.load_torch_checkpoint_windows_safe = restricted_checkpoint
    return module


def validate_targets(run: Path, role: str) -> dict[str, Path]:
    """Reject mixed voices, stale encoders, changed source tensors and invalid codes."""
    import numpy as np
    speaker, _, _ = teacher_paths(role)
    sha = digest(run / "decoder-student.pt")
    packs = {}
    for split in ("train", "eval"):
        directory = run / "code-targets" / split
        summary = json.loads((directory / "summary.json").read_text())
        pack = Path(summary["source_pack"]).resolve()
        if not pack.is_relative_to((WORK / speaker).resolve()):
            raise ValueError("Target voice mismatch")
        if summary["encoder_checkpoint_sha256"] != sha:
            raise ValueError("Stale encoder targets")
        source_rows = json.loads((pack / "rows.json").read_text())
        expected = {(row["row_id"], chunk["chunk_index"]): Path(chunk["tensor_npz"])
                    for row in source_rows for chunk in row["chunks"]}
        seen = set()
        for row in read_rows(directory / "manifest.jsonl"):
            key = (row["row_id"], row["chunk_index"])
            if key not in expected or key in seen:
                raise ValueError("Unexpected/duplicate target chunk")
            if row["encoder_checkpoint_sha256"] != sha or digest(expected[key]) != row["source_sha256"]:
                raise ValueError("Target source or encoder mismatch")
            with np.load(row["target_npz"], allow_pickle=False) as data:
                code = data["c"]
                if code.shape != (1, 40, row["frames"]) or not np.isfinite(code).all():
                    raise ValueError("Invalid acoustic code target")
            seen.add(key)
        if not seen or seen != set(expected):
            raise ValueError("Incomplete target coverage")
        packs[split] = pack
    return packs


def train(role: str, upstream: Path, decoder_run: Path, name: str, steps: int, device: str,
          checkpoint_steps: str = "", sample_weight_mode: str = "frames", resume: bool = False) -> None:
    if (type(steps) is not int or steps < 1 or not name or name in (".", "..")
            or Path(name).name != name):
        raise ValueError("Require a valid run name and positive target steps")
    milestones = [int(item) for item in checkpoint_steps.split(",") if item.strip()]
    if milestones != sorted(set(milestones)) or any(not 0 < step <= steps for step in milestones):
        raise ValueError("Checkpoint steps must be ascending, unique, positive and <= target")
    decoder_run = decoder_run.resolve()
    packs = validate_targets(decoder_run, role)
    speaker, _, config_path = teacher_paths(role)
    config = json.loads(config_path.read_text())
    vocab = max(config["num_symbols"], 1 + max(i for ids in config["phoneme_id_map"].values() for i in ids))
    out = WORK / speaker / name
    args = ["--pack-dir", str(packs["train"]), "--eval-pack-dir", str(packs["eval"]),
            "--target-dir", str(decoder_run / "code-targets/train"),
            "--eval-target-dir", str(decoder_run / "code-targets/eval"), "--target-key", "c",
            "--architecture", "token_context", "--hidden", "48", "--depth", "5",
            "--token-depth", "3", "--kernel-size", "5", "--qat", "off", "--vocab-size", str(vocab),
            "--skip-render", "--out-dir", str(out), "--steps", str(steps),
            "--device", device, "--log-interval", "100", "--seed", "4242",
            "--sample-weight-mode", sample_weight_mode]
    if checkpoint_steps:
        args += ["--checkpoint-steps", checkpoint_steps]
    module = load_acoustic_module(upstream)
    model_config = {"architecture": "token_context", "vocab_size": vocab, "hidden": 48,
                    "depth": 5, "token_depth": 3, "kernel_size": 5, "out_channels": 40}
    extra = [config_path, decoder_run / "decoder-student.pt"]
    for split in packs:
        directory = decoder_run / "code-targets" / split
        extra.extend([directory / "summary.json", directory / "manifest.jsonl"])
        extra.extend(Path(row["target_npz"]) for row in read_rows(directory / "manifest.jsonl"))
    expected = {"args": args, "upstream_sha256": TRAINER_SHA,
                "decoder_checkpoint_sha256": digest(decoder_run / "decoder-student.pt"),
                "quality_accepted": False, "purpose": "Experimental oracle-duration code regression",
                "voice": role, "model_config": model_config, "inputs_sha256": input_hashes(packs, extra)}

    def execute(local_args, log_path):
        with log_path.open("w") as log:
            subprocess.run([sys.executable, str(Path(__file__).resolve()), "_train",
                            "--upstream", str(upstream), "--", *local_args],
                           cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)

    recover_training(out, expected, args, module, lambda: module.create_model_from_config(model_config),
                     "acoustic", resume, execute)
    print(f"Acoustic experiment complete: {out / 'train-report.json'}")


def render(role: str, upstream: Path, decoder_run: Path, acoustic_run: Path) -> None:
    """Render oracle-code vs predicted-code lanes with teacher durations.

    No inference through the 192-channel Piper renderer, no learned-duration
    or arbitrary-text claim. All checkpoint loads use restricted unpickling.
    """
    import numpy as np
    import torch
    from pathlib import PosixPath
    packs = validate_targets(decoder_run, role)
    invocation = json.loads((acoustic_run / "invocation.json").read_text())
    if invocation["decoder_checkpoint_sha256"] != digest(decoder_run / "decoder-student.pt"):
        raise ValueError("Acoustic/decoder lineage mismatch")
    with torch.serialization.safe_globals([PosixPath]):
        ac = torch.load(acoustic_run / "latent-student.pt", map_location="cpu", weights_only=True)
        dec = torch.load(decoder_run / "decoder-student.pt", map_location="cpu", weights_only=True)
    am = load_acoustic_module(upstream)
    dm = load_decoder_module(upstream)
    model = am.create_model_from_config(ac["config"])
    model.load_state_dict(ac["model_state_dict"], strict=True)
    model.eval()
    signature = inspect.signature(dm.DecoderStudent)
    kwargs = {key: value for key, value in dec["config"].items() if key in signature.parameters}
    decoder = dm.DecoderStudent(**kwargs)
    decoder.load_state_dict(dec["model_state_dict"], strict=True)
    decoder.eval()
    encoder = dm.LrcEncoder(in_channels=192, hidden=64, code_dim=40)
    encoder.load_state_dict(dec["lrc_encoder_state_dict"], strict=True)
    encoder.eval()
    out = acoustic_run / "oracle-duration-audio"
    fresh(out)
    out.mkdir()
    results = []
    rows = json.loads((packs["eval"] / "rows.json").read_text())
    for index, row in enumerate(rows[:4]):
        for chunk in row["chunks"]:
            path = Path(chunk["tensor_npz"])
            with np.load(path, allow_pickle=False) as data:
                latent = data["generator_input"].astype(np.float32)
                ids = data["phoneme_ids"].reshape(-1)
                durations = data["w_ceil"].reshape(-1)
            frames = latent.shape[-1]
            sample = am.ChunkSample(row_id=row["row_id"], row_index=index, text=row["text"],
                chunk_index=chunk["chunk_index"], phoneme_ids=ids, durations=durations,
                target=np.zeros((frames, 40), dtype=np.float32), tensor_path=path,
                audio_samples=frames * 256)
            with torch.inference_mode():
                predicted = am.predict_chunk(model, sample, torch.device("cpu"))
                oracle = encoder(torch.from_numpy(latent)).numpy()
                record = {"row_id": row["row_id"], "chunk_index": chunk["chunk_index"], "text": row["text"]}
                for lane, codes in (("oracle", oracle), ("acoustic", predicted)):
                    audio = decoder(torch.from_numpy(codes)).reshape(-1).numpy()
                    if audio.size != frames * 256 or not np.isfinite(audio).all():
                        raise ValueError("Invalid decoder output")
                    name = f"{index:02d}-{chunk['chunk_index']}-{lane}.wav"
                    with wave.open(str(out / name), "wb") as wav:
                        wav.setnchannels(1)
                        wav.setsampwidth(2)
                        wav.setframerate(22050)
                        wav.writeframes((np.clip(audio, -1, 1) * 32767).astype('<i2').tobytes())
                    record[lane] = {"file": name, "rms": float(np.sqrt(np.mean(audio**2))),
                                    "peak": float(np.max(np.abs(audio))), "samples": audio.size}
                results.append(record)
    write_json(out / "report.json", {"lanes": results, "durations": "teacher/oracle",
               "quality_accepted": False, "arbitrary_text_ready": False})
    print(f"Rendered {len(results)} paired oracle-duration chunks: {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("train", "render", "_train"))
    parser.add_argument("--voice", choices=("male", "female"), default="female")
    parser.add_argument("--decoder-run", type=Path)
    parser.add_argument("--name")
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument("--device", choices=("cpu", "mps"), default="cpu")
    parser.add_argument("--checkpoint-steps", default="", help="ascending comma-separated steps")
    parser.add_argument("--sample-weight-mode", choices=("uniform", "frames"), default="frames")
    parser.add_argument("--upstream")
    parser.add_argument("--acoustic-run", type=Path)
    parser.add_argument("--resume", action="store_true",
                        help="train only: verified weight-only recovery; optimizer/RNG reset")
    args, extra = parser.parse_known_args()
    if args.resume and args.stage != "train":
        parser.error("--resume is only supported for train")
    upstream = upstream_path(args.upstream)
    if args.stage == "_train":
        if not extra or extra[0] != "--": parser.error("Expected -- before trainer arguments")
        module = load_acoustic_module(upstream)
        sys.argv = [str(upstream / TRAINER), *extra[1:]]
        module.main()
        return
    if args.stage == "render":
        if extra or args.decoder_run is None or args.acoustic_run is None:
            parser.error("render requires --decoder-run and --acoustic-run")
        render(args.voice, upstream, args.decoder_run.resolve(), args.acoustic_run.resolve())
        return
    if (extra or args.steps < 1 or not args.name or args.name in (".", "..")
            or Path(args.name).name != args.name or args.decoder_run is None):
        parser.error("Require a fresh directory --name, --decoder-run and positive steps")
    train(args.voice, upstream, args.decoder_run.resolve(), args.name, args.steps,
            args.device, args.checkpoint_steps, args.sample_weight_mode, resume=args.resume)


if __name__ == "__main__":
    main()