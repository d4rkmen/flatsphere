import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import mykyta_train as workflow


class MykytaWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.cfg = workflow.load_config(workflow.DEFAULT_CONFIG)
        self.work = patch.object(workflow, "WORK", self.root)
        self.work.start()
        self.addCleanup(self.work.stop)
        self.pipeline = workflow.Pipeline(self.cfg, self.root / "upstream")
        self.pipeline.p["root"].mkdir(parents=True)

    def test_budget_and_separate_voice_paths(self):
        self.assertEqual(self.cfg["decoder_steps"], 400000)
        self.assertEqual(self.cfg["acoustic_steps"], 400000)
        self.assertEqual(self.pipeline.p["last_decoder"].name, "seg-00039")
        self.assertTrue(self.pipeline.p["cut"].is_relative_to(self.pipeline.p["root"]))
        self.assertNotIn("tetiana", str(self.pipeline.p))

    def test_acoustic_explicit_voice_budget_resume(self):
        args = self.pipeline.acoustic_command()
        self.assertEqual(args[args.index("--voice") + 1], "male")
        self.assertEqual(args[args.index("--steps") + 1], "400000")
        self.assertIn("--resume", args)
        saves = list(map(int, args[args.index("--checkpoint-steps") + 1].split(",")))
        self.assertEqual(saves[-1], 400000)
        self.assertEqual(saves, sorted(set(saves)))
        self.assertNotIn("--resume", self.pipeline.acoustic_command(smoke=True))

    def test_once_skips_only_unchanged_outputs(self):
        out = self.root / "output"
        def produce(*_):
            out.mkdir()
            (out / "value").write_text("original")
        with patch.object(self.pipeline, "run", side_effect=produce) as run:
            self.pipeline.once("test", [out], ["command"])
            self.pipeline.once("test", [out], ["command"])
            self.assertEqual(run.call_count, 1)
            (out / "value").write_text("changed")
            with self.assertRaises(ValueError):
                self.pipeline.once("test", [out], ["command"])

    def test_interrupted_output_is_preserved(self):
        out = self.root / "output"
        out.mkdir()
        (out / "partial").write_text("keep me")
        def produce(*_):
            out.mkdir()
            (out / "complete").write_text("new")
        with patch.object(self.pipeline, "run", side_effect=produce):
            self.pipeline.once("retry", [out], ["command"])
        archived = list(self.root.glob("output.interrupted-*"))
        self.assertEqual(len(archived), 1)
        self.assertEqual((archived[0] / "partial").read_text(), "keep me")

    def test_accept_requires_listening(self):
        with self.assertRaisesRegex(ValueError, "Listen"):
            self.pipeline.accept(False)

    def test_export_requires_completed_training(self):
        with patch.object(self.pipeline, "once") as once:
            with self.assertRaises(FileNotFoundError):
                self.pipeline.export()
            once.assert_not_called()

    def test_lineage_rejects_different_audition(self):
        hashes = {}
        args = self.pipeline.checkpoints()
        for index, kind in enumerate(("duration", "acoustic", "decoder")):
            path = Path(args[index * 2 + 1])
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(kind)
            hashes[f"{kind}_sha256"] = workflow.digest(path)
        for key, filename, voice in (("synthesis", "report.json", {"voice": "mykyta"}),
                                      ("export", "export_report.json", {"speaker": "mykyta"})):
            self.pipeline.p[key].mkdir(parents=True, exist_ok=True)
            (self.pipeline.p[key] / filename).write_text(json.dumps({**hashes, **voice}))
        self.pipeline.verify_lineage()
        Path(args[1]).write_text("replaced")
        with self.assertRaisesRegex(ValueError, "differ"):
            self.pipeline.verify_lineage()

    def test_exclusive_lock(self):
        with workflow.run_lock(self.root):
            with self.assertRaisesRegex(RuntimeError, "Another"):
                with workflow.run_lock(self.root):
                    self.fail("concurrent lock acquired")


if __name__ == "__main__":
    unittest.main()