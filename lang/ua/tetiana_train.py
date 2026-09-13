#!/usr/bin/env python3
"""Explicit Tetiana preparation -> training -> gated MCU bundle.

Standalone counterpart of mykyta_train.py; shared hash-bound helpers stay intact.
No implicit stage, firmware build or flash. Resume restores weights only, not
optimizer/RNG state. Budgets are cumulative targets, not additional steps.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import shutil
import sys
import uuid

from cook import ROOT, WORK, digest, read_rows, teacher_paths, upstream_path, write_json
from duration_long import logged

DEFAULT_CONFIG = ROOT / "tetiana-400k.json"
STAGES = ("plan", "prepare", "smoke", "train", "export", "accept", "install")


def load_config(path: Path) -> dict:
    cfg = json.loads(path.read_text())
    if set(cfg) != {"name", "duration_steps", "decoder_steps", "acoustic_steps",
                    "decoder_segment_steps", "duration_device", "decoder_device", "acoustic_device"}:
        raise ValueError("Unexpected or missing pipeline configuration fields")
    name = cfg["name"]
    if (not isinstance(name, str) or not name.startswith("tetiana-")
            or name == "tetiana-" or Path(name).name != name):
        raise ValueError("Require a fresh tetiana- prefixed run name")
    for key in ("duration_steps", "decoder_steps", "acoustic_steps", "decoder_segment_steps"):
        if type(cfg[key]) is not int or cfg[key] < 1:
            raise ValueError(f"Invalid {key}")
    if cfg["decoder_steps"] % cfg["decoder_segment_steps"]:
        raise ValueError("Decoder steps must divide into whole segments")
    for key in ("duration_device", "decoder_device", "acoustic_device"):
        if cfg[key] not in ("cpu", "mps"):
            raise ValueError(f"Invalid {key}; CPU and MPS are supported")
    return cfg


def layout(cfg: dict) -> dict[str, Path]:
    base, prefix = WORK / "tetiana", cfg["name"]
    segments = cfg["decoder_steps"] // cfg["decoder_segment_steps"]
    return {"root": base / prefix, "packs": base / f"{prefix}-packs",
            "cut": base / prefix / "decoder-cut",
            "duration": base / f"{prefix}-duration", "decoder": base / f"{prefix}-decoder",
            "last_decoder": base / f"{prefix}-decoder" / f"seg-{segments - 1:05d}",
            "acoustic": base / f"{prefix}-acoustic", "synthesis": base / f"{prefix}-synthesis",
            "export": base / prefix / "export", "bundle": base / prefix / "flash-ready"}


def tree_hashes(path: Path) -> dict[str, str]:
    if not path.is_dir() or path.is_symlink():
        raise ValueError(f"Missing/unsafe artifact directory: {path}")
    files = sorted(p for p in path.rglob("*") if p.is_file())
    if not files or any(p.is_symlink() for p in path.rglob("*")):
        raise ValueError(f"Empty or symlinked artifacts: {path}")
    return {str(p.relative_to(path)): digest(p) for p in files}


def verify_receipt(receipt: Path, outputs: list[Path]) -> bool:
    if receipt.is_symlink():
        raise ValueError(f"Unsafe receipt: {receipt}")
    if not receipt.exists():
        return False
    if json.loads(receipt.read_text()) != {str(p): tree_hashes(p) for p in outputs}:
        raise ValueError(f"Completed artifacts changed: {receipt}")
    return True


def verify_required(receipt: Path, outputs: list[Path]) -> None:
    if not verify_receipt(receipt, outputs):
        raise ValueError(f"Missing completed stage: {receipt}")


def archive(path: Path) -> None:
    if path.is_symlink():
        raise ValueError(f"Refusing symlink: {path}")
    if path.exists():
        path.rename(path.with_name(f"{path.name}.interrupted-{uuid.uuid4().hex}"))


@contextmanager
def run_lock(root: Path):
    if root.is_symlink() or (root / ".lock").is_symlink():
        raise ValueError(f"Unsafe run directory: {root}")
    root.mkdir(parents=True, exist_ok=True)
    with (root / ".lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("Another Tetiana pipeline process is running") from exc
        yield


class Pipeline:
    def __init__(self, cfg: dict, upstream: Path):
        self.cfg, self.upstream, self.p = cfg, upstream, layout(cfg)

    def command(self, script: str, *args) -> list[str]:
        return [sys.executable, "-u", str(ROOT / script), *map(str, args),
                "--upstream", str(self.upstream)]

    def run(self, name: str, command: list[str]) -> None:
        print(f"\n=== {name} ===", flush=True)
        logs = self.p["root"] / "logs"
        logs.mkdir(exist_ok=True)
        logged(command, logs / f"{name}-{uuid.uuid4().hex}.log")

    def once(self, name: str, outputs: list[Path], command: list[str]) -> None:
        receipt = self.p["root"] / f"{name}-complete.json"
        if verify_receipt(receipt, outputs):
            print(f"Verified completed stage: {name}")
            return
        for path in outputs:
            archive(path)
        self.run(name, command)
        write_json(receipt, {str(p): tree_hashes(p) for p in outputs})

    def check_paths(self) -> None:
        """Never adopt unbound outputs or follow links into an older run."""
        outputs = list(self.p.values()) + [self.p[k].with_name(self.p[k].name + "-smoke")
                                          for k in ("duration", "decoder", "acoustic")]
        for path in outputs:
            if any(p.is_symlink() for p in (path, *path.parents)):
                raise ValueError(f"Unsafe pipeline path: {path}")
        binding = self.p["root"] / "pipeline.json"
        if binding.is_symlink():
            raise ValueError(f"Unsafe binding: {binding}")
        if not binding.exists():
            occupied = [p for p in outputs if p != self.p["root"] and p.exists()]
            occupied += [p for p in self.p["root"].glob("*") if p.name not in (".lock", "logs")]
            if occupied:
                raise ValueError("Unbound existing outputs; choose a fresh run name")

    def bind(self) -> None:
        self.check_paths()
        _, model, config = teacher_paths("female")
        files = [model, config, WORK / "tetiana/train.jsonl", WORK / "tetiana/eval.jsonl",
                 ROOT / "duration-long.json", ROOT / "decoder-long.json",
                 ROOT / "sources.json", ROOT / "texts/clock_eval.jsonl",
                 *[ROOT / name for name in ("tetiana_train.py", "cook.py", "duration_long.py",
                    "decoder_long.py", "lrc_experiment.py", "acoustic_experiment.py",
                    "synthesize.py", "export_int8.py", "golden_test.sh", "voice_golden.c")]]
        record = {"config": self.cfg, "upstream": str(self.upstream),
                  "inputs": {str(p): digest(p) for p in files}}
        path = self.p["root"] / "pipeline.json"
        if path.exists() and json.loads(path.read_text()) != record:
            raise ValueError("Pipeline inputs changed; restore them or choose a new run name")
        if not path.exists():
            write_json(path, record)
        duration = json.loads((ROOT / "duration-long.json").read_text())
        duration.update(voice="female", pack_name=self.p["packs"].name,
                        run_name=self.p["duration"].name, steps=self.cfg["duration_steps"],
                        device=self.cfg["duration_device"],
                        train_rows=len(read_rows(WORK / "tetiana/train.jsonl")),
                        eval_rows=len(read_rows(WORK / "tetiana/eval.jsonl")))
        decoder = json.loads((ROOT / "decoder-long.json").read_text())
        decoder.update(voice="female", pack_name=self.p["packs"].name,
                       run_name=self.p["decoder"].name, total_steps=self.cfg["decoder_steps"],
                       segment_steps=self.cfg["decoder_segment_steps"], device=self.cfg["decoder_device"],
                       decoder_cut=str(self.p["cut"] / f"{model.stem}-decoder-from-generator-input.onnx"))
        for name, value in (("duration", duration), ("decoder", decoder)):
            path = self.p["root"] / f"{name}.json"
            if path.is_symlink() or (path.exists() and json.loads(path.read_text()) != value):
                raise ValueError(f"Training configuration changed: {path}")
            if not path.exists():
                write_json(path, value)

    def long_command(self, kind: str, stage: str, resume: bool = False) -> list[str]:
        args = [stage, "--config", self.p["root"] / f"{kind}.json"]
        if resume:
            args.append("--resume")
        return self.command(f"{kind}_long.py", *args)

    def prepare(self) -> None:
        self.check_paths()
        splits = [WORK / "tetiana" / f"{s}.jsonl" for s in ("train", "eval", "test")]
        # cook.prepare rewrites every split. Do not repair a partial old corpus.
        if any(p.exists() for p in splits) and not all(p.is_file() for p in splits):
            raise ValueError("Partial Tetiana corpus; restore missing manifests before prepare")
        if (self.p["root"] / "pipeline.json").exists():
            self.bind()
        self.run("fetch", self.command("cook.py", "fetch", "--voice", "female", "--audio"))
        if not all(p.is_file() for p in splits):
            self.run("corpus", self.command("cook.py", "prepare", "--voice", "female"))
        self.bind()
        for split in ("train", "eval"):
            if not (self.p["packs"] / f"{split}-complete.json").exists():
                archive(self.p["packs"] / f"pack-{split}")
        self.run("packs", self.long_command("duration", "prepare"))
        _, model, _ = teacher_paths("female")
        cut = self.p["cut"]
        self.once("cut", [cut], [sys.executable, "-u",
                  str(self.upstream / "tools/extract_piper_vits_decoder_cut.py"),
                  "--model", str(model), "--pack-dir", str(self.p["packs"] / "pack-eval"),
                  "--out-dir", str(cut), "--latent-channels", "192"])

    def smoke(self) -> None:
        for kind in ("duration", "decoder"):
            out = self.p[kind].with_name(self.p[kind].name + "-smoke")
            self.once(f"{kind}-smoke", [out], self.long_command(kind, "smoke"))

    def acoustic_command(self, smoke: bool = False) -> list[str]:
        total = 5 if smoke else self.cfg["acoustic_steps"]
        saves = sorted({min(1000, total), min(5000, total), total, *range(10000, total, 10000)})
        args = ["train", "--voice", "female", "--decoder-run", self.p["last_decoder"],
                "--name", self.p["acoustic"].name + ("-smoke" if smoke else ""),
                "--steps", total, "--device", self.cfg["acoustic_device"],
                "--checkpoint-steps", ",".join(map(str, saves)), "--sample-weight-mode", "frames"]
        if not smoke:
            args.append("--resume")
        return self.command("acoustic_experiment.py", *args)

    def train(self) -> None:
        self.smoke()
        for kind in ("duration", "decoder"):
            self.run(kind, self.long_command(kind, "train", resume=True))
        self.once("targets", [self.p["last_decoder"] / "code-targets"],
                  self.command("lrc_experiment.py", "encode", "--voice", "female",
                               "--run", self.p["last_decoder"]))
        self.once("acoustic-smoke", [self.p["acoustic"].with_name(self.p["acoustic"].name + "-smoke")],
                  self.acoustic_command(smoke=True))
        self.run("acoustic", self.acoustic_command())
        self.export()

    def checkpoints(self) -> list[str]:
        return ["--duration-ckpt", str(self.p["duration"] / "duration-student.pt"),
                "--acoustic-ckpt", str(self.p["acoustic"] / "latent-student.pt"),
                "--decoder-ckpt", str(self.p["last_decoder"] / "decoder-student.pt")]

    def export(self) -> None:
        marker = self.p["root"] / "golden-pass.json"
        marker.unlink(missing_ok=True)
        # Front injection measures only the decoder; runtime overrides can test
        # a different implementation. Neither is full-stack acceptance.
        overrides = sorted(key for key in os.environ if key.startswith("SNT_"))
        if overrides:
            raise ValueError(f"Unset SNT debug/runtime overrides before strict parity: {overrides}")
        self.verify_training()
        self.once("synthesis", [self.p["synthesis"]], self.command("synthesize.py",
                  "--voice", "female", "--name", self.p["synthesis"].name, *self.checkpoints()))
        self.once("export", [self.p["export"]], self.command("export_int8.py", "--voice", "female",
                  "--out", self.p["export"], "--eval-pack", self.p["packs"] / "pack-eval",
                  *self.checkpoints()))
        self.verify_lineage()
        self.run("golden", ["sh", str(ROOT / "golden_test.sh"), str(self.p["export"]), str(self.upstream)])
        write_json(marker, tree_hashes(self.p["export"]))
        print(f"Parity passed. Listen to {self.p['synthesis'] / 'index.html'}; then run accept.")

    def verify_training(self) -> None:
        for kind in ("duration", "acoustic"):
            out = self.p[kind]
            receipt = json.loads((out / "completion.json").read_text())
            if receipt.get("total_steps") != self.cfg[f"{kind}_steps"]:
                raise ValueError(f"Incomplete {kind} training budget")
            for name, sha in receipt["artifacts_sha256"].items():
                if Path(name).name != name or (out / name).is_symlink() or digest(out / name) != sha:
                    raise ValueError(f"Changed completed {kind} artifact")
            final = "duration-student.pt" if kind == "duration" else "latent-student.pt"
            if not {final, "train-report.json", "invocation.json"} <= receipt["artifacts_sha256"].keys():
                raise ValueError(f"Incomplete {kind} receipt")
        from decoder_long import resume_prefix
        count = self.cfg["decoder_steps"] // self.cfg["decoder_segment_steps"]
        if resume_prefix(self.p["decoder"], count, self.cfg["decoder_segment_steps"],
                         digest(self.p["decoder"] / "config.json")) != count:
            raise ValueError("Decoder training is incomplete")

    def verify_lineage(self) -> None:
        synth = json.loads((self.p["synthesis"] / "report.json").read_text())
        exported = json.loads((self.p["export"] / "export_report.json").read_text())
        if synth.get("voice") != "tetiana" or exported.get("speaker") != "tetiana":
            raise ValueError("Wrong speaker in evaluation/export")
        args = self.checkpoints()
        for index, kind in enumerate(("duration", "acoustic", "decoder")):
            sha = digest(Path(args[index * 2 + 1]))
            if synth.get(f"{kind}_sha256") != sha or exported.get(f"{kind}_sha256") != sha:
                raise ValueError("Auditioned, exported and trained checkpoints differ; use a new run")

    def accept(self, listening_approved: bool) -> None:
        if not listening_approved:
            raise ValueError("Listen to teacher/student WAVs first, then pass --listening-approved")
        verify_required(self.p["root"] / "synthesis-complete.json", [self.p["synthesis"]])
        report = json.loads((self.p["synthesis"] / "report.json").read_text())
        if not report.get("lanes"):
            raise ValueError("Missing evaluation phrases")
        for lane in report["lanes"]:
            if not 0 < lane["learned_frames"] <= 640 or not 0 < lane["phoneme_ids"] <= 256:
                raise ValueError("Evaluation phrase exceeds firmware limits; not flash-ready")
            if len(lane["text"].encode("utf-8")) >= 192:
                raise ValueError("Evaluation phrase exceeds firmware request byte limit")
        self.export()  # Always rerun strict C parity, even with a previous PASS.
        bundle = self.p["bundle"]
        if bundle.exists() or bundle.is_symlink():
            raise FileExistsError(f"Bundle already exists: {bundle}")
        staging = bundle.with_name(f".bundle-{uuid.uuid4().hex}")
        try:
            shutil.copytree(self.p["export"], staging)
            write_json(staging / "acceptance.json", {"speaker": "tetiana", "listening_approved": True,
                       "device_validated": False, "config": self.cfg,
                       "artifacts": tree_hashes(self.p["export"]),
                       "synthesis_report_sha256": digest(self.p["synthesis"] / "report.json")})
            staging.rename(bundle)
        finally:
            if staging.exists():
                shutil.rmtree(staging)
        print(f"Flash-ready model bundle: {bundle} (firmware build/device test still required)")

    def install(self) -> None:
        bundle = self.p["bundle"]
        approval = json.loads((bundle / "acceptance.json").read_text())
        artifacts = tree_hashes(bundle)
        artifacts.pop("acceptance.json")
        if (approval.get("listening_approved") is not True or approval.get("speaker") != "tetiana"
                or artifacts != approval["artifacts"] or approval["config"] != self.cfg):
            raise ValueError("Accepted bundle changed")
        # Use the real Tetiana firmware slot, never an unselectable new slot.
        destination = ROOT.parent.parent / "components/sanotts/voices/uk"
        if destination.exists() or destination.is_symlink():
            raise FileExistsError(f"Refusing to overwrite installed Tetiana: {destination}; "
                                  "preserve it separately before explicitly installing")
        shutil.copytree(bundle, destination)
        print("Installed Tetiana in voices/uk; Mykyta and sdkconfig unchanged.")
        print("Build/flash and device validation remain separate ESP-IDF extension operations.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=STAGES)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--upstream")
    parser.add_argument("--listening-approved", action="store_true")
    args = parser.parse_args()
    cfg = load_config(args.config)
    if args.stage == "plan":
        print(json.dumps({"config": cfg, "paths": {k: str(v) for k, v in layout(cfg).items()},
                          "order": list(STAGES[1:]), "resume": "weight-only; no optimizer/RNG restore"}, indent=2))
        return
    pipeline = Pipeline(cfg, upstream_path(args.upstream))
    pipeline.check_paths()
    with run_lock(pipeline.p["root"]):
        if args.stage == "prepare":
            pipeline.prepare()
        else:
            pipeline.bind()
            if args.stage == "accept":
                pipeline.accept(args.listening_approved)
            else:
                getattr(pipeline, args.stage)()


if __name__ == "__main__":
    main()
