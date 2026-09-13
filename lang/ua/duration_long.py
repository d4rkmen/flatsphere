#!/usr/bin/env python3
"""Prepare/check/start duration training; --resume is opt-in weight-only recovery.

The configured steps are a fixed cumulative target, not an extension request.
Prior attempts are archived intact; canonical checkpoint/config/report names stay
unchanged. Upstream reports and checkpoint steps remain LOCAL to each attempt;
invocation.json records the offset and completion.json verifies cumulative steps.
Legacy invocations without input hashes and snapshots without step metadata are
refused. Optimizer/RNG state is never restored. Smoke cannot be resumed.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import types
from uuid import uuid4

from cook import ROOT, WORK, digest, read_rows, teacher_paths, upstream_path, write_json
from lrc_experiment import fresh

TRAINER = "tools/train_roota_piper_duration_student.py"
SHA = "f4837da6f8c36a1f5061682e6c9ffb6cc0a233b1273c7bc54f48393686d6646a"


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    try:
        write_json(temporary, value)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def plain(value):
    """Normalize only Path values; retain numeric types for exact comparisons."""
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return value


def exact(left, right) -> bool:
    return json.dumps(plain(left), sort_keys=True, allow_nan=False) == json.dumps(
        plain(right), sort_keys=True, allow_nan=False)


def restricted_checkpoint(path: Path, label: str = "checkpoint") -> dict:
    """Never fall back to pickle execution, including inside upstream training."""
    import torch
    from pathlib import PosixPath
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"Invalid {label}: {path}")
    with torch.serialization.safe_globals([PosixPath]):
        checkpoint = torch.load(path, map_location="cpu", weights_only=True)

    def finite(value):
        if isinstance(value, torch.Tensor):
            if not torch.isfinite(value).all().item():
                raise ValueError(f"Nonfinite tensor in {path}")
        elif isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f"Nonfinite metadata in {path}")
        elif isinstance(value, dict):
            for item in value.values():
                finite(item)
        elif isinstance(value, (tuple, list)):
            for item in value:
                finite(item)

    if not isinstance(checkpoint, dict):
        raise ValueError(f"Invalid {label} payload: {path}")
    finite(checkpoint)
    return checkpoint


def load_duration_module(upstream: Path):
    path = upstream / TRAINER
    if digest(path) != SHA:
        raise ValueError("Duration trainer revision changed")
    source = path.read_text()
    # Only repair loading. Save instrumentation below leaves training arithmetic alone.
    for expression in ("torch.load(checkpoint_path, map_location=\"cpu\", weights_only=False)",
                       "torch.load(args.initial_checkpoint, map_location=\"cpu\", weights_only=False)"):
        if source.count(expression) != 1:
            raise ValueError("Duration load anchors changed")
        argument = "checkpoint_path" if "checkpoint_path," in expression else "args.initial_checkpoint"
        source = source.replace(expression, f"restricted_checkpoint({argument})")
    module = types.ModuleType("ua_duration_trainer")
    module.__file__ = str(path)
    module.__dict__["restricted_checkpoint"] = restricted_checkpoint
    sys.modules[module.__name__] = module
    exec(compile(source, str(path), "exec"), module.__dict__)

    def save_checkpoint(path, model, config, args):
        # The pinned save call runs after optimizer.step(). Record that local step
        # IN the payload, not by trusting a filename later during recovery.
        import torch
        match = re.fullmatch(r"duration-student-step(\d+)\.pt", path.name)
        if path.name != "duration-student.pt" and match is None:
            raise ValueError("Unexpected duration save path")
        step = int(match[1]) if match else args.steps
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        torch.save({"model_state_dict": model.cpu().state_dict(), "config": config,
                    "train_args": plain(vars(args)), "params": module.count_parameters(model),
                    "step": step}, temporary)
        temporary.replace(path)

    module.save_checkpoint = save_checkpoint
    return module


def input_hashes(packs: dict[str, Path], extra=()) -> dict:
    """Bind recovery to actual rows/tensors (not just filenames or row counts)."""
    files = set(Path(path).resolve() for path in extra)
    for pack in packs.values():
        rows_path = pack / "rows.json"
        files.add(rows_path.resolve())
        for row in json.loads(rows_path.read_text()):
            files.update(Path(chunk["tensor_npz"]).resolve() for chunk in row["chunks"])
    return {str(path): digest(path) for path in sorted(files)}


def parsed_training_args(module, args: list[str]) -> dict:
    previous = sys.argv
    try:
        sys.argv = [module.__file__, *args]
        return plain(vars(module.parse_args()))
    finally:
        sys.argv = previous


def attempt_args(original: list[str], target: int, base: int,
                 initial: Path | None, kind: str) -> list[str]:
    args = list(original)
    args[args.index("--steps") + 1] = str(target - base)
    if "--checkpoint-steps" in args:
        index = args.index("--checkpoint-steps") + 1
        milestones = [int(item) for item in args[index].split(",") if item]
        args[index] = ",".join(str(step - base) for step in milestones if step > base)
    elif base and kind == "duration":
        interval = int(args[args.index("--save-interval") + 1])
        milestones = sorted({*range((base // interval + 1) * interval, target, interval), target})
        args += ["--checkpoint-steps", ",".join(str(step - base) for step in milestones)]
    if initial is not None:
        args += ["--initial-checkpoint" if kind == "duration" else "--load-checkpoint", str(initial)]
    return args


def recover_training(out: Path, expected: dict, original: list[str], module,
                     model_factory, kind: str, resume: bool, execute) -> None:
    """Shared, fail-closed weight-only recovery. Canonical outputs are never moved
    until preflight passes. Archives retain original local steps and all artifacts.

    Legacy invocations lacking input hashes are refused. A final checkpoint without
    a successful report is not completion evidence. A target-step snapshot alone
    cannot finalize evaluation; use an earlier snapshot or refuse zero-step work.
    """
    out = out.absolute()
    target = int(original[original.index("--steps") + 1])
    final = "duration-student.pt" if kind == "duration" else "latent-student.pt"
    pattern = r"duration-student-step(\d+)\.pt" if kind == "duration" else r"checkpoint_step(\d+)\.pt"

    def read_json(path):
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"Missing/unsafe recovery metadata: {path}")
        try:
            return json.loads(path.read_text())
        except (OSError, ValueError) as error:
            raise ValueError(f"Invalid recovery metadata: {path}") from error

    def checkpoint_step(path, record):
        checkpoint = restricted_checkpoint(path)
        if (not exact(checkpoint.get("config"), expected["model_config"])
                or not exact(checkpoint.get("train_args"), record["recovery"]["train_args"])):
            raise ValueError(f"Incompatible checkpoint config/arguments: {path}")
        import torch
        model = model_factory()
        state = checkpoint.get("model_state_dict")
        wanted_state = model.state_dict()
        if not isinstance(state, dict) or state.keys() != wanted_state.keys():
            raise ValueError(f"Incompatible checkpoint state keys: {path}")
        for key, value in state.items():
            if (not isinstance(value, torch.Tensor) or value.shape != wanted_state[key].shape
                    or value.dtype != wanted_state[key].dtype or value.layout != wanted_state[key].layout):
                raise ValueError(f"Incompatible checkpoint tensor: {path}: {key}")
        model.load_state_dict(state, strict=True)
        additional = record["recovery"]["additional_steps"]
        step = checkpoint.get("step")
        if path.name == final:
            report = read_json(path.parent / "train-report.json")
            if (report.get("passed") is not True or type(report.get("steps")) is not int
                    or report["steps"] != additional or not report.get("logs")
                    or type(report["logs"][-1].get("step")) is not int
                    or report["logs"][-1]["step"] != additional):
                raise ValueError(f"Final checkpoint lacks verified successful steps: {path}")
            # Both pinned trainers save finals after the loop, with no step in
            # legacy payloads. Full train_args + final report/log establish it.
            if step is not None and (type(step) is not int or step != additional):
                raise ValueError(f"Final checkpoint step mismatch: {path}")
            step = additional
        else:
            match = re.fullmatch(pattern, path.name)
            if (match is None or type(step) is not int or step != int(match[1])
                    or not 0 < step <= additional):
                raise ValueError(f"Missing/invalid local checkpoint step: {path}")
            arguments = record["recovery"]["train_args"]
            milestones = [int(item) for item in arguments.get("checkpoint_steps", "").split(",") if item]
            if milestones:
                valid_save = step in milestones
            else:
                valid_save = kind == "duration" and (step % arguments["save_interval"] == 0 or step == additional)
            if not valid_save:
                raise ValueError(f"Checkpoint step outside save schedule: {path}")
        return step

    def invocation(directory, visited=None):
        visited = set() if visited is None else visited
        if directory in visited or directory.is_symlink() or not directory.is_dir():
            raise ValueError(f"Invalid/cyclic recovery directory: {directory}")
        visited.add(directory)
        record = read_json(directory / "invocation.json")
        if not isinstance(record, dict) or not exact({k: v for k, v in record.items() if k != "recovery"}, expected):
            raise ValueError("Resume invocation/input hash mismatch (legacy runs without hashes are refused)")
        recovery = record.get("recovery", {})
        base = recovery.get("base_steps")
        if type(base) is not int or not 0 <= base < target:
            raise ValueError("Invalid cumulative base steps")
        initial = recovery.get("initial_checkpoint")
        initial_sha = None
        if initial is not None:
            initial = Path(initial)
            if (not initial.is_absolute() or initial.parent.parent != out.parent
                    or not initial.parent.name.startswith(out.name + ".interrupted-")):
                raise ValueError("Invalid archived checkpoint lineage")
            parent = invocation(initial.parent, visited)
            if parent["recovery"]["base_steps"] + checkpoint_step(initial, parent) != base:
                raise ValueError("Cumulative checkpoint step mismatch")
            initial_sha = digest(initial)
        elif base:
            raise ValueError("Missing checkpoint for nonzero cumulative progress")
        args = attempt_args(original, target, base, initial, kind)
        wanted = recovery_record(base, initial, initial_sha, args)
        if not exact(recovery, wanted):
            raise ValueError("Resume attempt arguments/progress/hash mismatch")
        return record

    def recovery_record(base, initial, initial_sha, args):
        return {"version": 1, "mode": "weight-only; optimizer and RNG reset",
                "target_steps": target, "base_steps": base, "additional_steps": target - base,
                "initial_checkpoint": str(initial) if initial else None,
                "initial_checkpoint_sha256": initial_sha, "args": args,
                "train_args": parsed_training_args(module, args)}

    def completion(record):
        local = checkpoint_step(out / final, record)
        if record["recovery"]["base_steps"] + local != target:
            raise ValueError("Completed total does not match target")
        names = [final, "train-report.json", "invocation.json"]
        if kind == "duration":
            config = read_json(out / "config.json")
            if not exact(config.get("model_config"), expected["model_config"]):
                raise ValueError("Duration output config mismatch")
            names.append("config.json")
        for name in names:
            if (out / name).is_symlink():
                raise ValueError("Unsafe completion artifact")
        return {"version": 1, "total_steps": target, "base_steps": record["recovery"]["base_steps"],
                "local_steps": local, "mode": "weight-only; optimizer and RNG reset",
                "artifacts_sha256": {name: digest(out / name) for name in names}}

    base, initial = 0, None
    if out.exists() or out.is_symlink():
        if not resume:
            raise FileExistsError(f"Refusing existing run without --resume: {out}")
        record = invocation(out)
        receipt_path = out / "completion.json"
        if receipt_path.exists() or receipt_path.is_symlink():
            receipt = completion(record)
            if not exact(read_json(receipt_path), receipt):
                raise ValueError("Completion receipt/hash mismatch")
            print(f"Verified {target} cumulative steps; skipping completed {kind} run: {out}")
            return
        if (out / "train-report.json").exists():
            try:
                receipt = completion(record)
            except Exception as error:
                print(f"Unverified completion for {out}: {error}; scanning recovery candidates")
            else:
                atomic_json(receipt_path, receipt)
                print(f"Verified {target} cumulative steps; skipping completed {kind} run: {out}")
                return
        candidates = []
        found = False
        for path in out.iterdir():
            if re.fullmatch(pattern, path.name) or path.name == final:
                found = True
                try:
                    step = checkpoint_step(path, record)
                    total = record["recovery"]["base_steps"] + step
                    if total < target:
                        candidates.append((total, path))
                except Exception as error:
                    print(f"Excluded recovery candidate {path}: {error}")
        if candidates:
            if len({step for step, _ in candidates}) != len(candidates):
                raise ValueError("Ambiguous duplicate checkpoint steps")
            base, initial = max(candidates)
        elif record["recovery"]["initial_checkpoint"]:
            base = record["recovery"]["base_steps"]
            initial = Path(record["recovery"]["initial_checkpoint"])
        elif found:
            raise ValueError("No verified checkpoint below target; refusing ambiguous recovery")
        # With no saved progress, a verified invocation may restart from scratch.
        archived = out.with_name(f"{out.name}.interrupted-{uuid4().hex}")
        selected_sha = digest(initial) if initial else None
        out.rename(archived)
        if initial is not None and initial.parent == out:
            initial = archived / initial.name
        if initial is not None and digest(initial) != selected_sha:
            raise ValueError("Checkpoint changed during archive; no training started")
        print(f"Preserved attempt: {archived}; continuing from {base}/{target} steps (weights only)")
    elif any(out.parent.glob(out.name + ".interrupted-*")):
        raise ValueError("Canonical run missing beside archives; refusing to guess lineage")
    args = attempt_args(original, target, base, initial, kind)
    record = {**expected, "recovery": recovery_record(base, initial, digest(initial) if initial else None, args)}
    out.mkdir(parents=True)
    atomic_json(out / "invocation.json", record)
    execute(args, out / "console.log")
    atomic_json(out / "completion.json", completion(record))


def load_config(path: Path) -> dict:
    cfg = json.loads(path.read_text())
    for key in ("pack_name", "run_name"):
        if not cfg[key] or Path(cfg[key]).name != cfg[key] or cfg[key] in (".", ".."):
            raise ValueError(f"Invalid {key}")
    for key in ("train_rows", "eval_rows", "steps", "batch_size", "save_interval", "log_interval"):
        if type(cfg[key]) is not int or cfg[key] < 1:
            raise ValueError(f"Invalid {key}")
    if cfg["device"] not in ("mps", "cpu", "cuda") or cfg["voice"] not in ("female", "male"):
        raise ValueError("Invalid voice/device")
    if not 0 < cfg["learning_rate"] <= 0.01:
        raise ValueError("Invalid learning rate")
    return cfg


def paths(cfg: dict) -> tuple[Path, Path]:
    speaker, _, _ = teacher_paths(cfg["voice"])
    base = WORK / speaker
    return base / cfg["pack_name"], base / cfg["run_name"]


def logged(command: list[str], log_path: Path) -> None:
    # Stream progress to both terminal and file. Interrupted runs retain logs
    # and all completed upstream checkpoints; directories are never reused.
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


def prepare(cfg: dict, upstream: Path) -> None:
    speaker, model, config = teacher_paths(cfg["voice"])
    root, _ = paths(cfg)
    root.mkdir(parents=True, exist_ok=True)
    for split in ("train", "eval"):
        count = cfg[f"{split}_rows"]
        source = WORK / speaker / f"{split}.jsonl"
        if len(read_rows(source)) < count:
            raise ValueError(f"Not enough {split} rows")
        selection = {"source_sha256": digest(source), "rows": count,
                     "model_sha256": digest(model), "config_sha256": digest(config)}
        receipt = root / f"{split}-selection.json"
        target = root / f"pack-{split}"
        complete = root / f"{split}-complete.json"
        if complete.exists():
            if json.loads(complete.read_text()) != selection:
                raise ValueError("Prepared inputs changed; choose a new pack_name")
            continue
        fresh(target)
        write_json(receipt, selection)
        logged([sys.executable, "-u", str(upstream / "tools/build_piper_vits_roota_probe_pack.py"),
                "--model", str(model), "--config", str(config), "--source-jsonl", str(source),
                "--out-dir", str(target), "--max-rows", str(count), "--tensor-mode", "acoustic",
                "--noise-scale", "0", "--noise-w", "0", "--length-scale", "1",
                "--progress-interval", "128"], root / f"{split}-prepare.log")
        write_json(complete, selection)
    check(cfg, upstream)


def check(cfg: dict, upstream: Path) -> dict:
    import numpy as np
    if digest(upstream / TRAINER) != SHA:
        raise ValueError("Duration trainer revision changed")
    root, out = paths(cfg)
    report = {"run_directory": str(out), "steps": cfg["steps"], "device": cfg["device"], "splits": {}}
    texts = {}
    from cook import canonical_text
    for split in ("train", "eval"):
        rows = json.loads((root / f"pack-{split}/rows.json").read_text())
        if len(rows) != cfg[f"{split}_rows"]:
            raise ValueError("Prepared row count mismatch")
        texts[split] = {canonical_text(row["text"]) for row in rows}
        chunks = 0
        for row in rows:
            if row["sample_rate"] != 22050:
                raise ValueError("Unexpected teacher sample rate")
            for chunk in row["chunks"]:
                with np.load(chunk["tensor_npz"], allow_pickle=False) as data:
                    ids, durations = data["phoneme_ids"].reshape(-1), data["w_ceil"].reshape(-1)
                    if (not ids.size or ids.size != durations.size or ids.min() < 0 or ids.max() >= 256
                            or not np.isfinite(durations).all() or durations.min() < 0
                            or durations.sum() != data["generator_input"].shape[-1]):
                        raise ValueError("Invalid duration alignment/vocabulary")
                chunks += 1
        report["splits"][split] = {"rows": len(rows), "chunks": chunks}
    if texts["train"] & texts["eval"]:
        raise ValueError("Training/validation text leakage")
    write_json(root / "validation.json", report)
    print(json.dumps(report, indent=2))
    return report


def command_args(cfg: dict, upstream: Path) -> list[str]:
    root, out = paths(cfg)
    return [sys.executable, "-u", str(upstream / TRAINER),
            "--pack-dir", str(root / "pack-train"), "--eval-pack-dir", str(root / "pack-eval"),
            "--out-dir", str(out), "--steps", str(cfg["steps"]), "--device", cfg["device"],
            "--hidden", str(cfg["hidden"]), "--depth", str(cfg["depth"]),
            "--kernel-size", str(cfg["kernel_size"]), "--batch-size", str(cfg["batch_size"]),
            "--lr", str(cfg["learning_rate"]), "--save-interval", str(cfg["save_interval"]),
            "--log-interval", str(cfg["log_interval"]), "--seed", str(cfg["seed"]),
            "--max-duration", str(cfg["max_duration"]), "--duration-vocab-size", "256",
            "--duration-oov-id", "0"]


def train(cfg: dict, upstream: Path, resume: bool = False) -> None:
    check(cfg, upstream)
    import numpy as np
    import torch
    if cfg["device"] == "mps" and not torch.backends.mps.is_available():
        raise RuntimeError("MPS unavailable; explicitly choose CPU in the config")
    root, out = paths(cfg)
    module = load_duration_module(upstream)
    packs = {split: root / f"pack-{split}" for split in ("train", "eval")}
    max_tokens = 0
    for pack in packs.values():
        for row in json.loads((pack / "rows.json").read_text()):
            for chunk in row["chunks"]:
                with np.load(chunk["tensor_npz"], allow_pickle=False) as data:
                    max_tokens = max(max_tokens, data["phoneme_ids"].size)
    model_config = {"architecture": "duration_conv", "vocab_size": 256,
                    "hidden": cfg["hidden"], "depth": cfg["depth"], "kernel_size": cfg["kernel_size"],
                    "max_tokens": max_tokens, "max_duration": cfg["max_duration"],
                    "target_duration_scale": 1.0, "pause_preserve_scale": 0.0,
                    "pause_preserve_window": 0, "pause_token_ids": [4, 8, 10, 11, 12, 13],
                    "long_preserve_threshold": 0.0, "long_preserve_scale": 0.0,
                    "duration_vocab_size_forced": 256, "duration_oov_id": 0}
    speaker, model, config = teacher_paths(cfg["voice"])
    extra = [model, config, *(WORK / speaker / f"{split}.jsonl" for split in packs)]
    for split in packs:
        extra.extend(path for path in (root / f"{split}-selection.json", root / f"{split}-complete.json")
                     if path.exists())
    command = command_args(cfg, upstream)
    expected = {"configuration": cfg, "command": command, "trainer_sha256": SHA,
                "resume": "weight-only, no optimizer resume", "model_config": model_config,
                "inputs_sha256": input_hashes(packs, extra)}

    def factory():
        return module.DurationStudent(**{key: model_config[key] for key in
                                        ("vocab_size", "hidden", "depth", "kernel_size", "max_tokens")})

    def execute(args, log):
        logged([sys.executable, "-u", str(Path(__file__).resolve()), "_train",
                "--upstream", str(upstream), "--", *args], log)

    recover_training(out, expected, command[3:], module, factory, "duration", resume, execute)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("prepare", "check", "smoke", "train", "_train"))
    parser.add_argument("--config", type=Path, default=ROOT / "duration-long.json")
    parser.add_argument("--upstream")
    parser.add_argument("--resume", action="store_true",
                        help="train only: verified weight-only recovery; optimizer/RNG reset")
    args, extra = parser.parse_known_args()
    if args.resume and args.stage != "train":
        parser.error("--resume is only supported for train")
    if args.stage == "_train":
        if not extra or extra[0] != "--":
            parser.error("Expected -- before trainer arguments")
        upstream = upstream_path(args.upstream)
        module = load_duration_module(upstream)
        sys.argv = [str(upstream / TRAINER), *extra[1:]]
        module.main()
        return
    if extra:
        parser.error("Unexpected arguments")
    cfg = load_config(args.config)
    if args.stage == "smoke":
        cfg = dict(cfg, run_name=cfg["run_name"] + "-smoke", steps=5,
                   save_interval=5, log_interval=1)
    upstream = upstream_path(args.upstream)
    if args.stage == "prepare":
        prepare(cfg, upstream)
    elif args.stage == "check":
        check(cfg, upstream)
    else:
        train(cfg, upstream, resume=args.resume)


if __name__ == "__main__":
    main()