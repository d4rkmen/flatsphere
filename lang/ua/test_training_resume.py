"""Weight-only recovery tests with restricted checkpoints and pinned trainers.

Unit tests mock training children. UA_REAL_TRAINING_TESTS=1 additionally runs
two-step CPU trainers on existing local packs, then real one-step recovery.
All artifacts, including interruption archives, live in temporary directories.
"""
from __future__ import annotations

import contextlib
import copy
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import torch

import acoustic_experiment as acoustic
import duration_long as duration


class TrainingResumeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.upstream = Path(__file__).resolve().parents[2] / "build/_deps/sano_source-src"
        cls.dm = duration.load_duration_module(cls.upstream)
        cls.am = acoustic.load_acoustic_module(cls.upstream)

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.work = Path(temporary.name)
        self.voice = self.work / "tetiana"
        self.voice.mkdir()
        self.model = self.work / "teacher.onnx"
        self.model.write_bytes(b"teacher")
        self.config = self.work / "teacher.json"
        duration.write_json(self.config, {"num_symbols": 256, "phoneme_id_map": {"a": [1]}})
        self.packs = {}
        for split in ("train", "eval"):
            pack = self.voice / "packs" / f"pack-{split}"
            pack.mkdir(parents=True)
            tensor = pack / "chunk.npz"
            np.savez(tensor, phoneme_ids=np.array([1, 2]), w_ceil=np.array([1, 1]),
                     generator_input=np.zeros((1, 192, 2)))
            duration.write_json(pack / "rows.json", [{"row_id": split, "text": split,
                "sample_rate": 22050, "chunks": [{"chunk_index": 0, "tensor_npz": str(tensor)}]}])
            (self.voice / f"{split}.jsonl").write_text('{"source": "fixture"}\n')
            self.packs[split] = pack
        self.decoder = self.voice / "decoder"
        self.decoder.mkdir()
        (self.decoder / "decoder-student.pt").write_bytes(b"decoder lineage")
        for split in self.packs:
            directory = self.decoder / "code-targets" / split
            directory.mkdir(parents=True)
            target = directory / "target.npz"
            np.savez(target, c=np.zeros((1, 40, 2)))
            duration.write_json(directory / "summary.json", {"source_pack": str(self.packs[split])})
            (directory / "manifest.jsonl").write_text(json.dumps({"target_npz": str(target)}) + "\n")
        self.cfg = {"voice": "female", "pack_name": "packs", "run_name": "duration",
                    "train_rows": 1, "eval_rows": 1, "steps": 400000, "device": "cpu",
                    "hidden": 4, "depth": 1, "kernel_size": 3, "batch_size": 2,
                    "learning_rate": 0.0005, "save_interval": 100000,
                    "log_interval": 100, "seed": 6262, "max_duration": 80}
        for module in (duration, acoustic):
            self.enterContext(patch.object(module, "WORK", self.work))
            self.enterContext(patch.object(module, "teacher_paths",
                                          return_value=("tetiana", self.model, self.config)))
        # Input files/hashes and checkpoint compatibility are real; only dataset
        # semantic checks and child processes are mocked in these orchestration tests.
        self.check = self.enterContext(patch.object(duration, "check", return_value={}))
        self.targets = self.enterContext(patch.object(acoustic, "validate_targets", return_value=self.packs))
        self.child = self.enterContext(patch.object(duration, "logged", side_effect=self.duration_child))
        self.ac_child = self.enterContext(patch.object(acoustic.subprocess, "run", side_effect=self.acoustic_child))
        self.enterContext(contextlib.redirect_stdout(io.StringIO()))
        self.calls = []
        self.interrupt_step = None
        self.fail_empty = False

    def run_train(self, kind, resume=False, **changes):
        if kind == "duration":
            duration.train(dict(self.cfg, **changes), self.upstream, resume=resume)
        else:
            kwargs = {"role": "female", "upstream": self.upstream, "decoder_run": self.decoder,
                      "name": "acoustic", "steps": 400000, "device": "cpu",
                      "checkpoint_steps": "100000,200000,300000,400000", "sample_weight_mode": "frames"}
            acoustic.train(**dict(kwargs, **changes), resume=resume)

    def root(self, kind):
        return self.voice / kind

    def record(self, kind):
        return json.loads((self.root(kind) / "invocation.json").read_text())

    def final_name(self, kind):
        return "duration-student.pt" if kind == "duration" else "latent-student.pt"

    def snapshot_name(self, kind, step):
        return f"duration-student-step{step}.pt" if kind == "duration" else f"checkpoint_step{step:06d}.pt"

    def duration_child(self, command, log_path):
        self.artifacts("duration", command[command.index("--") + 1:], log_path)

    def acoustic_child(self, command, **kwargs):
        self.artifacts("acoustic", command[command.index("--") + 1:], Path(kwargs["stdout"].name))

    def payload(self, kind, args, step=None):
        record = self.record(kind)
        config = record["model_config"]
        module = self.dm if kind == "duration" else self.am
        if kind == "duration":
            model = module.DurationStudent(**{k: config[k] for k in
                                            ("vocab_size", "hidden", "depth", "kernel_size", "max_tokens")})
        else:
            model = module.create_model_from_config(config)
        result = {"config": config, "model_state_dict": model.state_dict(),
                  "train_args": vars(self.parse(module, args))}
        if step is not None:
            result["step"] = step
        return result

    def parse(self, module, args):
        with patch.object(sys, "argv", [module.__file__, *args]):
            return module.parse_args()

    def artifacts(self, kind, args, log_path):
        self.calls.append((kind, args))
        log_path.write_text(f"child {len(self.calls)} progress\n")
        root = log_path.parent
        steps = int(args[args.index("--steps") + 1])
        if kind == "duration":
            duration.write_json(root / "config.json", {"model_config": self.record(kind)["model_config"]})
        if self.fail_empty:
            raise KeyboardInterrupt()
        if self.interrupt_step is not None:
            step = self.interrupt_step
            torch.save(self.payload(kind, args, step), root / self.snapshot_name(kind, step))
            raise subprocess.CalledProcessError(1, args)
        # Pinned acoustic finals have no step; duration wrapper finals do.
        torch.save(self.payload(kind, args, steps if kind == "duration" else None), root / self.final_name(kind))
        duration.write_json(root / "train-report.json", {"passed": True, "steps": steps,
                            "logs": [{"step": steps}], "checkpoint": str(root / self.final_name(kind))})

    def interrupted(self, kind, step=100000, resume=False):
        self.interrupt_step = step
        with self.assertRaises(subprocess.CalledProcessError):
            self.run_train(kind, resume=resume)
        self.interrupt_step = None

    def snapshot(self, root):
        return {str(path.relative_to(root)): (path.read_bytes(), path.stat().st_mtime_ns)
                for path in root.rglob("*") if path.is_file()}

    def test_fresh_and_completed_resume_read_only_default_refuses(self):
        for kind in ("duration", "acoustic"):
            with self.subTest(kind=kind):
                self.run_train(kind)
                root = self.root(kind)
                before = self.snapshot(root)
                count = len(self.calls)
                self.run_train(kind, resume=True)
                self.assertEqual(len(self.calls), count)
                self.assertEqual(self.snapshot(root), before)
                with self.assertRaises(FileExistsError):
                    self.run_train(kind)
                receipt = json.loads((root / "completion.json").read_text())
                self.assertEqual(receipt["total_steps"], 400000)
                for name, sha in receipt["artifacts_sha256"].items():
                    self.assertEqual(duration.digest(root / name), sha)
                if kind == "acoustic":
                    self.assertEqual(self.record(kind)["decoder_checkpoint_sha256"],
                                     duration.digest(self.decoder / "decoder-student.pt"))
        self.check.assert_called()
        self.targets.assert_called()

    def test_multiple_interruptions_count_additional_steps_and_preserve_archives(self):
        for kind in ("duration", "acoustic"):
            with self.subTest(kind=kind):
                self.interrupted(kind)
                first = self.snapshot(self.root(kind))
                self.interrupted(kind, resume=True)
                archives = list(self.voice.glob(kind + ".interrupted-*"))
                self.assertEqual(len(archives), 1)
                self.assertEqual(self.snapshot(archives[0]), first)
                record = self.record(kind)
                self.assertEqual(record["recovery"]["base_steps"], 100000)
                self.assertEqual(record["recovery"]["additional_steps"], 300000)
                second = self.snapshot(self.root(kind))
                self.run_train(kind, resume=True)
                record = self.record(kind)
                recovery = record["recovery"]
                self.assertEqual(recovery["base_steps"], 200000)
                self.assertEqual(recovery["additional_steps"], 200000)
                self.assertEqual(recovery["args"][recovery["args"].index("--checkpoint-steps") + 1],
                                 "100000,200000")
                flag = "--initial-checkpoint" if kind == "duration" else "--load-checkpoint"
                initial = Path(recovery["args"][recovery["args"].index(flag) + 1])
                self.assertTrue(initial.is_absolute())
                self.assertEqual(self.snapshot(initial.parent), second)
                self.assertEqual(duration.digest(initial), recovery["initial_checkpoint_sha256"])
                self.assertEqual(record["command" if kind == "duration" else "args"]
                                 [record["command" if kind == "duration" else "args"].index("--steps") + 1],
                                 "400000")
                receipt = json.loads((self.root(kind) / "completion.json").read_text())
                self.assertEqual((receipt["base_steps"], receipt["local_steps"], receipt["total_steps"]),
                                 (200000, 200000, 400000))
                count = len(self.calls)
                self.run_train(kind, resume=True)
                self.assertEqual(len(self.calls), count)

    def test_empty_attempt_restarts_or_reuses_verified_archived_weights(self):
        for kind in ("duration", "acoustic"):
            with self.subTest(kind=kind):
                self.fail_empty = True
                with self.assertRaises(KeyboardInterrupt):
                    self.run_train(kind)
                self.fail_empty = False
                self.interrupted(kind, resume=True)
                self.assertEqual(self.record(kind)["recovery"]["base_steps"], 0)
                self.fail_empty = True
                with self.assertRaises(KeyboardInterrupt):
                    self.run_train(kind, resume=True)
                initial = self.record(kind)["recovery"]["initial_checkpoint"]
                self.fail_empty = False
                self.run_train(kind, resume=True)
                self.assertEqual(self.record(kind)["recovery"]["initial_checkpoint"], initial)
                self.assertEqual(self.record(kind)["recovery"]["base_steps"], 100000)

    def test_exact_invocation_config_and_inputs_mismatch_are_read_only(self):
        for kind in ("duration", "acoustic"):
            self.interrupted(kind)
            before = self.snapshot(self.root(kind))
            changes = [{"steps": 500000}, {"device": "mps"}]
            changes += ([{"hidden": 8}, {"learning_rate": 0.001}] if kind == "duration" else
                        [{"sample_weight_mode": "uniform"}, {"checkpoint_steps": "100000,400000"}])
            for change in changes:
                with self.subTest(kind=kind, change=change), \
                        patch.object(torch.backends.mps, "is_available", return_value=True):
                    with self.assertRaisesRegex(ValueError, "mismatch"):
                        self.run_train(kind, resume=True, **change)
                    self.assertEqual(self.snapshot(self.root(kind)), before)
            paths = [self.config, self.packs["train"] / "rows.json", self.packs["train"] / "chunk.npz"]
            if kind == "acoustic":
                paths.extend([self.decoder / "decoder-student.pt",
                              self.decoder / "code-targets/train/target.npz",
                              self.decoder / "code-targets/train/manifest.jsonl"])
            for path in paths:
                original = path.read_bytes()
                # JSON whitespace and trailing NPZ bytes still change the identity.
                path.write_bytes(original + b" \n")
                with self.subTest(kind=kind, path=path), self.assertRaisesRegex(ValueError, "mismatch"):
                    self.run_train(kind, resume=True)
                self.assertEqual(self.snapshot(self.root(kind)), before)
                path.write_bytes(original)

    def test_bad_candidates_excluded_in_favor_of_earlier_verified_checkpoint(self):
        for kind in ("duration", "acoustic"):
            self.interrupted(kind)
            root = self.root(kind)
            good = root / self.snapshot_name(kind, 100000)
            payload = duration.restricted_checkpoint(good)
            for step, damage in ((200000, "corrupt"), (300000, "nonfinite"), (400000, "shape")):
                path = root / self.snapshot_name(kind, step)
                if damage == "corrupt":
                    path.write_bytes(b"corrupt")
                else:
                    broken = copy.deepcopy(payload)
                    broken["step"] = step
                    key = next(iter(broken["model_state_dict"]))
                    broken["model_state_dict"][key] = (torch.tensor([float("nan")]) if damage == "nonfinite"
                                                        else torch.zeros(1))
                    torch.save(broken, path)
            before = self.snapshot(root)
            self.run_train(kind, resume=True)
            self.assertEqual(self.record(kind)["recovery"]["base_steps"], 100000)
            archive = next(self.voice.glob(kind + ".interrupted-*"))
            self.assertEqual(self.snapshot(archive), before)

    def test_invalid_or_missing_step_and_incompatible_metadata_refuse(self):
        for kind in ("duration", "acoustic"):
            self.interrupted(kind)
            path = self.root(kind) / self.snapshot_name(kind, 100000)
            original = duration.restricted_checkpoint(path)
            variants = []
            for step in (None, True, 100000.0, -1, 200000, 500000):
                broken = copy.deepcopy(original)
                if step is None:
                    broken.pop("step")
                else:
                    broken["step"] = step
                variants.append(broken)
            broken = copy.deepcopy(original)
            broken["train_args"]["steps"] = 999999
            variants.append(broken)
            broken = copy.deepcopy(original)
            broken["config"]["hidden"] += 1
            variants.append(broken)
            for broken in variants:
                torch.save(broken, path)
                before = self.snapshot(self.root(kind))
                with self.subTest(kind=kind, step=broken.get("step")), self.assertRaisesRegex(ValueError, "No verified"):
                    self.run_train(kind, resume=True)
                self.assertEqual(self.snapshot(self.root(kind)), before)

    def test_complete_artifacts_without_receipt_are_verified_and_adopted(self):
        for kind in ("duration", "acoustic"):
            self.run_train(kind)
            root = self.root(kind)
            (root / "completion.json").unlink()
            before = self.snapshot(root)
            count = len(self.calls)
            self.run_train(kind, resume=True)
            self.assertEqual(len(self.calls), count)
            after = self.snapshot(root)
            after.pop("completion.json")
            self.assertEqual(after, before)

    def test_partial_unreceipted_report_recovers_earlier_checkpoint(self):
        for kind in ("duration", "acoustic"):
            with self.subTest(kind=kind):
                self.run_train(kind)
                root = self.root(kind)
                payload = duration.restricted_checkpoint(root / self.final_name(kind))
                payload["step"] = 100000
                torch.save(payload, root / self.snapshot_name(kind, 100000))
                (root / "completion.json").unlink()
                (root / "train-report.json").write_text('{"passed": true, "logs": [')
                before = self.snapshot(root)
                count = len(self.calls)
                with contextlib.redirect_stdout(io.StringIO()) as output:
                    self.run_train(kind, resume=True)
                self.assertIn("Invalid recovery metadata", output.getvalue())
                self.assertEqual(len(self.calls), count + 1)
                self.assertEqual(self.record(kind)["recovery"]["base_steps"], 100000)
                self.assertEqual(self.record(kind)["recovery"]["additional_steps"], 300000)
                archive = next(self.voice.glob(kind + ".interrupted-*"))
                self.assertEqual(self.snapshot(archive), before)
                self.assertEqual(json.loads((root / "completion.json").read_text())["total_steps"], 400000)

    def test_receipted_partial_report_refuses_even_with_valid_earlier_checkpoint(self):
        for kind in ("duration", "acoustic"):
            with self.subTest(kind=kind):
                self.run_train(kind)
                root = self.root(kind)
                payload = duration.restricted_checkpoint(root / self.final_name(kind))
                payload["step"] = 100000
                torch.save(payload, root / self.snapshot_name(kind, 100000))
                (root / "train-report.json").write_text('{"passed":')
                before = self.snapshot(root)
                count = len(self.calls)
                with self.assertRaisesRegex(ValueError, "Invalid recovery metadata"):
                    self.run_train(kind, resume=True)
                self.assertEqual(len(self.calls), count)
                self.assertEqual(self.snapshot(root), before)
                self.assertFalse(list(self.voice.glob(kind + ".interrupted-*")))

    def test_target_snapshot_alone_does_not_invent_completion_or_zero_step_train(self):
        for kind in ("duration", "acoustic"):
            self.interrupted(kind, step=400000)
            before = self.snapshot(self.root(kind))
            count = len(self.calls)
            with self.assertRaisesRegex(ValueError, "No verified"):
                self.run_train(kind, resume=True)
            self.assertEqual(len(self.calls), count)
            self.assertEqual(self.snapshot(self.root(kind)), before)

    def test_local_save_schedule_mapping_and_out_of_schedule_refusal(self):
        args = ["--steps", "400000", "--save-interval", "100000"]
        mapped = duration.attempt_args(args, 400000, 150000, Path("/archive/weights.pt"), "duration")
        self.assertEqual(mapped[mapped.index("--steps") + 1], "250000")
        self.assertEqual(mapped[mapped.index("--checkpoint-steps") + 1], "50000,150000,250000")
        for kind in ("duration", "acoustic"):
            self.interrupted(kind, step=150000)
            with self.assertRaisesRegex(ValueError, "No verified"):
                self.run_train(kind, resume=True)

    def test_successful_child_without_artifacts_gets_no_completion_receipt(self):
        self.child.side_effect = lambda command, log: log.write_text("missing artifacts")
        self.ac_child.side_effect = lambda *args, **kwargs: None
        for kind in ("duration", "acoustic"):
            with self.assertRaises(ValueError):
                self.run_train(kind)
            self.assertFalse((self.root(kind) / "completion.json").exists())

    def test_existing_hash_and_semantic_checks_still_gate_training(self):
        with patch.object(duration, "check", side_effect=ValueError("invalid alignment")):
            with self.assertRaisesRegex(ValueError, "invalid alignment"):
                self.run_train("duration", resume=True)
        with patch.object(acoustic, "validate_targets", side_effect=ValueError("stale source")):
            with self.assertRaisesRegex(ValueError, "stale source"):
                self.run_train("acoustic", resume=True)
        upstream = self.work / "changed-upstream"
        for name in (duration.TRAINER, acoustic.TRAINER):
            path = upstream / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("changed trainer")
        for loader in (duration.load_duration_module, acoustic.load_acoustic_module):
            with self.assertRaises(ValueError):
                loader(upstream)
        self.assertFalse(self.calls)

    def test_incomplete_final_or_wrong_completion_steps_refuse(self):
        for kind in ("duration", "acoustic"):
            self.run_train(kind)
            root = self.root(kind)
            (root / "completion.json").unlink()
            report = root / "train-report.json"
            original = json.loads(report.read_text())
            for broken in ({**original, "steps": 100000}, {**original, "passed": False},
                           {**original, "logs": [{"step": 10}]}, {**original, "steps": True}):
                duration.write_json(report, broken)
                before = self.snapshot(root)
                with self.assertRaises(ValueError):
                    self.run_train(kind, resume=True)
                self.assertEqual(self.snapshot(root), before)
            report.unlink()
            with self.assertRaisesRegex(ValueError, "No verified"):
                self.run_train(kind, resume=True)

    def test_receipt_and_lineage_tampering_refuse_without_archive(self):
        for kind in ("duration", "acoustic"):
            self.interrupted(kind)
            self.run_train(kind, resume=True)
            root = self.root(kind)
            record = self.record(kind)
            initial = Path(record["recovery"]["initial_checkpoint"])
            for path in (root / "completion.json", root / "invocation.json", root / self.final_name(kind),
                         root / "train-report.json", initial):
                original = path.read_bytes()
                path.write_bytes(b"{}")
                before = self.snapshot(root)
                with self.subTest(kind=kind, path=path), self.assertRaises(Exception):
                    self.run_train(kind, resume=True)
                self.assertEqual(self.snapshot(root), before)
                path.write_bytes(original)
            record["recovery"]["base_steps"] += 1
            duration.write_json(root / "invocation.json", record)
            with self.assertRaisesRegex(ValueError, "Cumulative"):
                self.run_train(kind, resume=True)

    def test_legacy_missing_metadata_and_orphaned_archive_refuse(self):
        for kind in ("duration", "acoustic"):
            self.interrupted(kind)
            root = self.root(kind)
            record = self.record(kind)
            record.pop("inputs_sha256")
            duration.write_json(root / "invocation.json", record)
            before = self.snapshot(root)
            with self.assertRaisesRegex(ValueError, "legacy"):
                self.run_train(kind, resume=True)
            self.assertEqual(self.snapshot(root), before)
            root.rename(root.with_name(root.name + ".interrupted-orphan"))
            with self.assertRaisesRegex(ValueError, "Canonical run missing"):
                self.run_train(kind, resume=True)

    def test_cli_resume_is_opt_in_and_train_only(self):
        for module in (duration, acoustic):
            with patch.object(module, "upstream_path", return_value=self.upstream), \
                    patch.object(module, "train") as train:
                for flag in ([], ["--resume"]):
                    argv = [module.__file__, "train", *flag]
                    if module is acoustic:
                        argv += ["--name", "acoustic", "--decoder-run", str(self.decoder), "--steps", "400000"]
                    with patch.object(sys, "argv", argv), patch.object(duration, "load_config", return_value=self.cfg):
                        module.main()
                    self.assertEqual(train.call_args.kwargs["resume"], bool(flag))
                for stage in (("prepare", "check", "smoke", "_train") if module is duration else ("render", "_train")):
                    with patch.object(sys, "argv", [module.__file__, stage, "--resume"]), \
                            contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                        module.main()
                    self.assertEqual(error.exception.code, 2)

    def test_duration_save_instrumentation_and_restricted_child_loader(self):
        self.interrupted("duration")
        record = self.record("duration")
        args = self.parse(self.dm, record["recovery"]["args"])
        config = record["model_config"]
        model = self.dm.DurationStudent(**{k: config[k] for k in
                                         ("vocab_size", "hidden", "depth", "kernel_size", "max_tokens")})
        path = self.root("duration") / "duration-student-step200000.pt"
        self.dm.save_checkpoint(path, model, config, args)
        self.assertEqual(duration.restricted_checkpoint(path)["step"], 200000)
        with patch.object(torch, "load", wraps=torch.load) as load:
            self.dm.load_model_from_checkpoint(path, torch.device("cpu"))
            self.assertTrue(load.call_args.kwargs["weights_only"])
        self.assertIs(self.am.load_torch_checkpoint_windows_safe, duration.restricted_checkpoint)

    def test_restricted_loader_rejects_nonallowlisted_objects(self):
        path = self.work / "unsafe.pt"
        # A harmless but non-allowlisted global; no executable payload is needed.
        torch.save({"unsafe": argparse_namespace()}, path)
        with self.assertRaises(Exception):
            duration.restricted_checkpoint(path)


@unittest.skipUnless(os.environ.get("UA_REAL_TRAINING_TESTS") == "1",
                     "set UA_REAL_TRAINING_TESTS=1 to run existing-pack CPU training")
class RealTrainingResumeTests(unittest.TestCase):
    """Real wrapper subprocesses; only WORK placement is patched, not validation."""

    def setUp(self):
        self.source_work = duration.WORK
        self.upstream = Path(__file__).resolve().parents[2] / "build/_deps/sano_source-src"
        temporary = tempfile.TemporaryDirectory(prefix="resume-real-", dir=self.source_work)
        self.addCleanup(temporary.cleanup)
        self.work = Path(temporary.name)
        for module in (duration, acoustic):
            self.enterContext(patch.object(module, "WORK", self.work))

    def verify_real_recovery(self, out, kind, train):
        def read(name):
            return json.loads((out / name).read_text())

        def snapshot(directory):
            return {str(path.relative_to(directory)): (duration.digest(path), path.stat().st_mtime_ns)
                    for path in directory.rglob("*") if path.is_file()}

        train(False)
        report = read("train-report.json")
        self.assertIs(report["passed"], True)
        self.assertEqual(report["steps"], 2)
        self.assertEqual(report["logs"][-1]["step"], 2)
        self.assertEqual(read("completion.json")["total_steps"], 2)
        record = read("invocation.json")
        self.assertEqual(record["recovery"]["train_args"]["device"], "cpu")
        checkpoint = ("duration-student-step1.pt" if kind == "duration" else "checkpoint_step000001.pt")
        self.assertEqual(duration.restricted_checkpoint(out / checkpoint)["step"], 1)
        self.assertTrue((out / "console.log").read_text().strip())
        before = snapshot(out)
        train(True)
        self.assertEqual(snapshot(out), before)

        # Simulate a partial report using actual trainer-generated snapshots,
        # finals, arguments and config, never synthetic checkpoint payloads.
        (out / "train-report.json").write_text('{"passed": true, "logs": [')
        before = snapshot(out)
        with self.assertRaisesRegex(ValueError, "Invalid recovery metadata"):
            train(True)
        self.assertEqual(snapshot(out), before)
        self.assertFalse(list(out.parent.glob(out.name + ".interrupted-*")))
        (out / "completion.json").unlink()
        before = snapshot(out)
        with contextlib.redirect_stdout(io.StringIO()) as output:
            train(True)
        self.assertIn("scanning recovery candidates", output.getvalue())
        self.assertIn("Invalid recovery metadata", output.getvalue())
        recovery = read("invocation.json")["recovery"]
        self.assertEqual((recovery["base_steps"], recovery["additional_steps"]), (1, 1))
        initial = Path(recovery["initial_checkpoint"])
        self.assertEqual(initial.name, checkpoint)
        self.assertEqual(snapshot(initial.parent), before)
        self.assertEqual(duration.digest(initial), recovery["initial_checkpoint_sha256"])
        self.assertEqual(read("train-report.json")["steps"], 1)
        self.assertEqual(read("train-report.json")["logs"][-1]["step"], 1)
        receipt = read("completion.json")
        self.assertEqual((receipt["base_steps"], receipt["local_steps"], receipt["total_steps"]), (1, 1, 2))
        for name, sha in receipt["artifacts_sha256"].items():
            self.assertEqual(duration.digest(out / name), sha)
        before = snapshot(out)
        train(True)
        self.assertEqual(snapshot(out), before)
        print(f"REAL {kind}: CPU 2 steps + 1 recovery step; verified receipt, archive and read-only skip")

    def test_mykyta_duration_real_partial_report_recovery(self):
        speaker = "mykyta"
        voice = self.work / speaker
        cfg = duration.load_config(duration.ROOT / "duration-long.json")
        cfg.update(voice="male", pack_name="smoke-packs", run_name="duration-2-cpu",
                   steps=2, device="cpu", batch_size=2, save_interval=1, log_interval=1)
        for split in ("train", "eval"):
            source = self.source_work / speaker / f"pack-{split}"
            pack = voice / cfg["pack_name"] / f"pack-{split}"
            pack.mkdir(parents=True)
            shutil.copy2(source / "rows.json", pack / "rows.json")
            shutil.copy2(self.source_work / speaker / f"{split}.jsonl", voice / f"{split}.jsonl")
            cfg[f"{split}_rows"] = len(json.loads((pack / "rows.json").read_text()))
        self.verify_real_recovery(voice / cfg["run_name"], "duration",
                                  lambda resume: duration.train(cfg, self.upstream, resume=resume))

    def test_tetiana_acoustic_real_partial_report_recovery(self):
        speaker = "tetiana"
        voice = self.work / speaker
        source = self.source_work / speaker / "decoder-200000-cpu-v1/seg-00019"
        decoder = voice / "decoder-input"
        decoder.mkdir(parents=True)
        shutil.copy2(source / "decoder-student.pt", decoder / "decoder-student.pt")
        # Select one existing full-pack row per split and its real encoded targets.
        # Copy metadata only; tensors remain read-only at their original paths.
        # Rebase source_pack so the genuine WORK/voice restriction still applies.
        for split in ("train", "eval"):
            directory = source / "code-targets" / split
            summary = json.loads((directory / "summary.json").read_text())
            rows = json.loads((Path(summary["source_pack"]) / "rows.json").read_text())[:1]
            pack = voice / "smoke-packs" / f"pack-{split}"
            duration.write_json(pack / "rows.json", rows)
            summary["source_pack"] = str(pack)
            keys = {(row["row_id"], chunk["chunk_index"]) for row in rows for chunk in row["chunks"]}
            targets = [row for row in acoustic.read_rows(directory / "manifest.jsonl")
                       if (row["row_id"], row["chunk_index"]) in keys]
            summary["chunks"] = len(targets)
            target_dir = decoder / "code-targets" / split
            duration.write_json(target_dir / "summary.json", summary)
            (target_dir / "manifest.jsonl").write_text("".join(json.dumps(row) + "\n" for row in targets))
        self.verify_real_recovery(voice / "acoustic-2-cpu", "acoustic",
            lambda resume: acoustic.train("female", self.upstream, decoder, "acoustic-2-cpu",
                                           2, "cpu", checkpoint_steps="1,2", resume=resume))


def argparse_namespace():
    import argparse
    return argparse.Namespace(value=1)


if __name__ == "__main__":
    unittest.main()