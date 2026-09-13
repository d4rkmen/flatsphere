"""Temporary-fixture orchestration tests, not training or measured C parity."""
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import decoder_long
import duration_long
import mykyta_train
import tetiana_train as workflow


class TetianaWorkflowTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name).resolve()
        self.cfg = workflow.load_config(workflow.DEFAULT_CONFIG)
        self.enterContext(patch.object(workflow, "WORK", self.root / "work"))
        self.enterContext(patch.object(workflow, "logged", side_effect=AssertionError("No subprocesses")))
        self.enterContext(patch.dict(os.environ, {}, clear=True))
        self.enterContext(contextlib.redirect_stdout(io.StringIO()))
        self.p = workflow.Pipeline(self.cfg, self.root / "upstream")
        self.p.p["root"].mkdir(parents=True)

    def corpus(self):
        for split, count in (("train", 7), ("eval", 3), ("test", 2)):
            (workflow.WORK / "tetiana" / f"{split}.jsonl").write_text('{}\n' * count)
        model = self.root / "uk_UA-tetiana-high.onnx"
        model.write_text("teacher")
        config = model.with_suffix(".onnx.json")
        config.write_text("{}")
        return self.enterContext(patch.object(workflow, "teacher_paths", return_value=("tetiana", model, config)))

    def lineage(self):
        hashes = {}
        args = self.p.checkpoints()
        for index, kind in enumerate(("duration", "acoustic", "decoder")):
            path = Path(args[index * 2 + 1])
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(kind)
            hashes[f"{kind}_sha256"] = workflow.digest(path)
        workflow.write_json(self.p.p["synthesis"] / "report.json", {
            **hashes, "voice": "tetiana", "lanes": [{"text": "Тест", "phoneme_ids": 5, "learned_frames": 20}]})
        workflow.write_json(self.p.p["export"] / "export_report.json", {**hashes, "speaker": "tetiana"})
        for kind in ("synthesis", "export"):
            workflow.write_json(self.p.p["root"] / f"{kind}-complete.json",
                                {str(self.p.p[kind]): workflow.tree_hashes(self.p.p[kind])})

    def test_exact_budget_and_paths(self):
        male = mykyta_train.load_config(mykyta_train.DEFAULT_CONFIG)
        self.assertEqual({k: v for k, v in male.items() if k != "name"},
                         {k: v for k, v in self.cfg.items() if k != "name"})
        self.assertEqual([self.cfg[f"{k}_steps"] for k in ("duration", "decoder", "acoustic")],
                         [100000, 400000, 400000])
        self.assertEqual(self.p.p["last_decoder"].name, "seg-00039")
        for path in self.p.p.values():
            self.assertTrue(path.relative_to(workflow.WORK / "tetiana").parts[0].startswith("tetiana-400k-v1"))

    def test_invalid_config(self):
        for key, value in (("name", "../escape"), ("name", "duration-100000-mps-v1"), ("name", None),
                           ("duration_steps", True), ("decoder_steps", 400001), ("acoustic_steps", 0)):
            workflow.write_json(self.root / "bad.json", {**self.cfg, key: value})
            with self.assertRaises(ValueError):
                workflow.load_config(self.root / "bad.json")

    def test_plan_read_only_and_stage_required(self):
        with patch.object(sys, "argv", ["tetiana_train.py", "plan"]), \
                patch.object(workflow, "upstream_path", side_effect=AssertionError("No upstream")), \
                patch.object(workflow, "run_lock", side_effect=AssertionError("No writes")):
            workflow.main()
        self.assertEqual(list(self.p.p["root"].iterdir()), [])
        with patch.object(sys, "argv", ["tetiana_train.py"]), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                workflow.main()

    def test_binding_and_full_female_templates(self):
        teacher = self.corpus()
        self.p.bind()
        teacher.assert_called_with("female")
        duration = duration_long.load_config(self.p.p["root"] / "duration.json")
        decoder = decoder_long.load_config(self.p.p["root"] / "decoder.json")
        self.assertEqual((duration["train_rows"], duration["eval_rows"]), (7, 3))
        self.assertEqual(duration["steps"], 100000)
        self.assertEqual(decoder["total_steps"], 400000)
        for cfg in (duration, decoder):
            self.assertEqual(cfg["voice"], "female")
            self.assertEqual(cfg["pack_name"], self.p.p["packs"].name)
        self.assertTrue(Path(decoder["decoder_cut"]).is_relative_to(self.p.p["cut"]))
        for kind, cfg, excluded in (
                ("duration", duration, {"voice", "pack_name", "run_name", "steps", "device", "train_rows", "eval_rows"}),
                ("decoder", decoder, {"voice", "pack_name", "run_name", "total_steps", "segment_steps", "device", "decoder_cut"})):
            original = json.loads((workflow.ROOT / f"{kind}-long.json").read_text())
            self.assertEqual({k: v for k, v in cfg.items() if k not in excluded},
                             {k: v for k, v in original.items() if k not in excluded})
        self.p.bind()
        record = json.loads((self.p.p["root"] / "pipeline.json").read_text())
        self.assertIn(str(workflow.ROOT / "tetiana_train.py"), record["inputs"])
        self.assertNotIn(str(workflow.ROOT / "mykyta_train.py"), record["inputs"])
        (workflow.WORK / "tetiana/train.jsonl").write_text('{}\n')
        with self.assertRaisesRegex(ValueError, "inputs changed"):
            self.p.bind()

    def test_prepare_routing_and_partial_corpus_preservation(self):
        self.corpus()
        with patch.object(self.p, "run") as run, patch.object(self.p, "once") as once:
            self.p.prepare()
        self.assertEqual([c.args[0] for c in run.call_args_list], ["fetch", "packs"])
        command = run.call_args_list[0].args[1]
        self.assertEqual(command[command.index("--voice") + 1], "female")
        command = once.call_args.args[2]
        self.assertEqual(command[command.index("--pack-dir") + 1], str(self.p.p["packs"] / "pack-eval"))
        (workflow.WORK / "tetiana/test.jsonl").unlink()
        with self.assertRaisesRegex(ValueError, "Partial"):
            self.p.prepare()

    def test_unbound_outputs_and_links_refused(self):
        out = self.p.p["duration"]
        out.mkdir()
        (out / "keep").write_text("old")
        with self.assertRaisesRegex(ValueError, "Unbound"):
            self.p.check_paths()
        self.p.p["packs"].symlink_to(out)
        with self.assertRaisesRegex(ValueError, "Unsafe"):
            self.p.check_paths()
        self.assertEqual((out / "keep").read_text(), "old")

    def test_training_order_resume_and_saves(self):
        calls = []
        with patch.object(self.p, "run", side_effect=lambda n, c: calls.append((n, c))), \
                patch.object(self.p, "once", side_effect=lambda n, o, c: calls.append((n, c))), \
                patch.object(self.p, "export") as export:
            self.p.train()
        self.assertEqual([n for n, _ in calls], ["duration-smoke", "decoder-smoke", "duration", "decoder",
                         "targets", "acoustic-smoke", "acoustic"])
        for name, cmd in calls:
            self.assertEqual("--resume" in cmd, name in ("duration", "decoder", "acoustic"))
            if "--voice" in cmd:
                self.assertEqual(cmd[cmd.index("--voice") + 1], "female")
        cmd = self.p.acoustic_command()
        self.assertEqual(cmd[cmd.index("--steps") + 1], "400000")
        self.assertEqual(cmd[cmd.index("--decoder-run") + 1], str(self.p.p["last_decoder"]))
        self.assertEqual(cmd[cmd.index("--sample-weight-mode") + 1], "frames")
        self.assertEqual(cmd[cmd.index("--checkpoint-steps") + 1],
                         ",".join(map(str, [1000, 5000, *range(10000, 400001, 10000)])))
        export.assert_called_once()

    def test_once_preserves_interruption_and_verifies_hashes(self):
        out = self.root / "output"
        out.mkdir()
        (out / "old").write_text("keep")
        def produce(*_):
            out.mkdir()
            (out / "new").write_text("new")
        with patch.object(self.p, "run", side_effect=produce) as run:
            self.p.once("test", [out], ["mock"])
            self.p.once("test", [out], ["mock"])
            self.assertEqual(run.call_count, 1)
            (out / "new").write_text("changed")
            with self.assertRaises(ValueError):
                self.p.once("test", [out], ["mock"])
        archived, = self.root.glob("output.interrupted-*")
        self.assertEqual((archived / "old").read_text(), "keep")

    def test_training_completion_gates(self):
        with self.assertRaises(FileNotFoundError):
            self.p.export()
        for kind, final in (("duration", "duration-student.pt"), ("acoustic", "latent-student.pt")):
            out = self.p.p[kind]
            out.mkdir()
            for name in (final, "train-report.json", "invocation.json"):
                (out / name).write_text("fixture")
            workflow.write_json(out / "completion.json", {"total_steps": self.cfg[f"{kind}_steps"],
                                "artifacts_sha256": workflow.tree_hashes(out)})
        workflow.write_json(self.p.p["decoder"] / "config.json", {})
        with patch.object(decoder_long, "resume_prefix", return_value=40) as prefix:
            self.p.verify_training()
            self.assertEqual(prefix.call_args.args[1:3], (40, 10000))
            prefix.return_value = 39
            with self.assertRaisesRegex(ValueError, "incomplete"):
                self.p.verify_training()
        path = self.p.p["duration"] / "completion.json"
        receipt = json.loads(path.read_text())
        workflow.write_json(path, {**receipt, "total_steps": 99999})
        with self.assertRaisesRegex(ValueError, "budget"):
            self.p.verify_training()
        workflow.write_json(path, receipt)
        (path.parent / "duration-student.pt").write_text("changed")
        with self.assertRaisesRegex(ValueError, "Changed"):
            self.p.verify_training()

    def test_lineage_rejects_wrong_voice_and_checkpoint(self):
        self.lineage()
        self.p.verify_lineage()
        path = self.p.p["export"] / "export_report.json"
        original = json.loads(path.read_text())
        workflow.write_json(path, {**original, "speaker": "mykyta"})
        with self.assertRaisesRegex(ValueError, "Wrong speaker"):
            self.p.verify_lineage()
        workflow.write_json(path, original)
        Path(self.p.checkpoints()[1]).write_text("changed")
        with self.assertRaisesRegex(ValueError, "differ"):
            self.p.verify_lineage()

    def test_export_commands_and_failed_gate_no_stale_pass(self):
        self.lineage()
        marker = self.p.p["root"] / "golden-pass.json"
        marker.write_text("stale")
        with patch.object(self.p, "verify_training"), patch.object(self.p, "once") as once, \
                patch.object(self.p, "run", side_effect=subprocess.CalledProcessError(1, "mock gate")):
            with self.assertRaises(subprocess.CalledProcessError):
                self.p.export()
        for call in once.call_args_list:
            cmd = call.args[2]
            self.assertEqual(cmd[cmd.index("--voice") + 1], "female")
            self.assertNotIn("--skip-golden", cmd)
            for arg in self.p.checkpoints():
                self.assertIn(arg, cmd)
        cmd = once.call_args_list[1].args[2]
        self.assertEqual(cmd[cmd.index("--eval-pack") + 1], str(self.p.p["packs"] / "pack-eval"))
        self.assertFalse(marker.exists())

    def test_debug_overrides_refused(self):
        for key in ("SNT_INJECT_FRONT_PATH", "SNT_GOLDEN_RUNTIME"):
            with patch.dict(os.environ, {key: "debug"}):
                with self.assertRaisesRegex(ValueError, "overrides"):
                    self.p.export()

    def test_accept_requires_listening_limits_and_real_gate_success(self):
        with self.assertRaisesRegex(ValueError, "Listen"):
            self.p.accept(False)
        with self.assertRaisesRegex(ValueError, "Missing completed"):
            self.p.accept(True)
        self.lineage()
        path = self.p.p["synthesis"] / "report.json"
        original = json.loads(path.read_text())
        for lanes in ([], [{"text": "a", "phoneme_ids": 257, "learned_frames": 20}],
                      [{"text": "a", "phoneme_ids": 5, "learned_frames": 641}],
                      [{"text": "я" * 96, "phoneme_ids": 5, "learned_frames": 20}]):
            workflow.write_json(path, {**original, "lanes": lanes})
            workflow.write_json(self.p.p["root"] / "synthesis-complete.json",
                                {str(path.parent): workflow.tree_hashes(path.parent)})
            with patch.object(self.p, "export") as export:
                with self.assertRaises(ValueError):
                    self.p.accept(True)
                export.assert_not_called()
        self.lineage()
        with patch.object(self.p, "verify_training"), \
                patch.object(self.p, "run", side_effect=subprocess.CalledProcessError(1, "mock gate")):
            with self.assertRaises(subprocess.CalledProcessError):
                self.p.accept(True)
        self.assertFalse(self.p.p["bundle"].exists())

    def test_mock_acceptance_and_safe_install(self):
        self.lineage()
        # This mock verifies orchestration, not numerical parity of any voice.
        with patch.object(self.p, "verify_training"), patch.object(self.p, "run") as gate:
            self.p.accept(True)
            self.assertEqual(gate.call_args.args[0], "golden")
        bundle = self.p.p["bundle"]
        approval = json.loads((bundle / "acceptance.json").read_text())
        self.assertEqual(approval["speaker"], "tetiana")
        self.assertFalse(approval["device_validated"])
        local = self.root / "project/lang/ua"
        destination = local.parent.parent / "components/sanotts/voices/uk"
        destination.mkdir(parents=True)
        (destination / "old").write_text("keep")
        with patch.object(workflow, "ROOT", local):
            with self.assertRaisesRegex(FileExistsError, "overwrite"):
                self.p.install()
            self.assertEqual((destination / "old").read_text(), "keep")
            (destination / "old").unlink()
            destination.rmdir()
            self.p.install()
            self.assertEqual(workflow.tree_hashes(destination), workflow.tree_hashes(bundle))
            (bundle / "export_report.json").write_text("tampered")
            with self.assertRaisesRegex(ValueError, "Accepted bundle changed"):
                self.p.install()

    def test_exclusive_lock(self):
        with workflow.run_lock(self.p.p["root"]):
            with self.assertRaisesRegex(RuntimeError, "Another Tetiana"):
                with workflow.run_lock(self.p.p["root"]):
                    self.fail("concurrent lock")


if __name__ == "__main__":
    unittest.main()
