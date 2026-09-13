"""Resume safety tests: temporary artifacts and mocked children, no training."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import decoder_long as decoder


class DecoderResumeTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.work = Path(temporary.name)
        self.cfg = {
            "voice": "male", "pack_name": "packs", "run_name": "decoder-test",
            "total_steps": 30, "segment_steps": 10, "device": "cpu",
            "batch_size": 4, "crop_frames": 64, "learning_rate": 0.0002,
            "spectral_weight": 0.5, "log_interval": 2, "seed": 4242,
            "architecture": {"variant": "lrc", "lrc_code_dim": 40,
                             "lrc_encoder_hidden": 64, "fsd_dim": 76,
                             "fsd_blocks": 5, "fsd_film_rank": 12,
                             "fsd_head_rank": 48, "istft_n_fft": 1024},
        }
        self.upstream = self.work / "upstream"
        self.root = self.work / "mykyta" / self.cfg["run_name"]
        model = self.work / "teacher.onnx"
        cut = self.work / "mykyta/decoder-cut/teacher-decoder-from-generator-input.onnx"
        cut.parent.mkdir(parents=True)
        cut.write_bytes(b"mock cut")
        for split in ("train", "eval"):
            decoder.write_json(self.work / f"mykyta/packs/pack-{split}/rows.json", [])
        self.enterContext(patch.object(decoder, "WORK", self.work))
        self.enterContext(patch.object(decoder, "teacher_paths", return_value=("mykyta", model, model)))
        self.enterContext(patch.object(decoder, "check", return_value={}))
        self.logged = self.enterContext(patch.object(decoder, "logged", side_effect=self.artifacts))
        self.enterContext(contextlib.redirect_stdout(io.StringIO()))

    def artifacts(self, command, log_path):
        """Emulate successful child output without torch or any checkpoint loads."""
        segment = Path(command[command.index("--out-dir") + 1])
        self.assertFalse((segment / "completion.json").exists())
        steps = int(command[command.index("--steps") + 1])
        log_path.write_text("mock child finished\n")
        (segment / "decoder-student.pt").write_bytes(segment.name.encode() + b" decoder")
        (segment / "lrc-encoder.pt").write_bytes(segment.name.encode() + b" encoder")
        decoder.write_json(segment / "train-report.json", {"steps": steps})

    def run_train(self, resume=False, **changes):
        decoder.train(dict(self.cfg, **changes), self.upstream, smoke=False, resume=resume)

    def snapshot(self, root=None):
        root = root or self.root
        return {str(p.relative_to(root)): (p.read_bytes(), p.stat().st_mtime_ns)
                for p in root.rglob("*") if p.is_file()}

    def interrupted(self, index=1):
        def child(command, log_path):
            if log_path.parent.name == f"seg-{index:05d}":
                log_path.write_text("interrupted progress\n")
                raise subprocess.CalledProcessError(1, command)
            self.artifacts(command, log_path)
        self.logged.side_effect = child
        with self.assertRaises(subprocess.CalledProcessError):
            self.run_train()
        self.logged.reset_mock()
        self.logged.side_effect = self.artifacts

    def test_fresh_receipts_and_weight_only_chain(self):
        self.run_train()
        self.assertEqual(self.logged.call_count, 3)
        for index, call in enumerate(self.logged.call_args_list):
            command = call.args[0]
            segment = self.root / f"seg-{index:05d}"
            receipt = json.loads((segment / "completion.json").read_text())
            self.assertEqual(receipt["segment"], index)
            self.assertEqual(receipt["steps"], 10)
            self.assertEqual(receipt["config_sha256"], decoder.digest(self.root / "config.json"))
            self.assertEqual(set(receipt["artifacts_sha256"]),
                             {*decoder.COMPLETION_ARTIFACTS, "invocation.json"})
            for name, sha in receipt["artifacts_sha256"].items():
                self.assertEqual(sha, decoder.digest(segment / name))
            if index:
                previous = self.root / f"seg-{index - 1:05d}" / "decoder-student.pt"
                self.assertEqual(command[command.index("--init-decoder-checkpoint") + 1], str(previous))
                self.assertEqual(receipt["init_decoder_sha256"], decoder.digest(previous))
            else:
                self.assertNotIn("--init-decoder-checkpoint", command)
                self.assertIsNone(receipt["init_decoder_sha256"])

    def test_completed_resume_is_read_only_noop(self):
        self.run_train()
        before = self.snapshot()
        self.logged.reset_mock()
        self.run_train(resume=True)
        self.logged.assert_not_called()
        self.assertEqual(self.snapshot(), before)

    def test_default_rejects_existing_without_writing(self):
        self.run_train()
        before = self.snapshot()
        self.logged.reset_mock()
        with self.assertRaises(FileExistsError):
            self.run_train()
        self.logged.assert_not_called()
        self.assertEqual(self.snapshot(), before)

    def test_config_changes_reject_before_any_write(self):
        self.interrupted()
        before = self.snapshot()
        for resume in (False, True):
            for changes in ({"total_steps": 40}, {"seed": 1}, {"device": "mps"},
                            {"pack_name": "other"}, {"learning_rate": 0.001},
                            {"architecture": {"variant": "other"}}):
                with self.subTest(resume=resume, changes=changes):
                    # pack_root would otherwise reject the missing changed pack first.
                    with patch.object(decoder, "pack_root", return_value=self.work / "packs"):
                        with self.assertRaisesRegex(ValueError, "Root config"):
                            self.run_train(resume=resume, **changes)
                    self.assertEqual(self.snapshot(), before)
        self.logged.assert_not_called()

    def test_missing_or_malformed_root_config_rejects(self):
        self.root.mkdir()
        for content in (None, "{", "{}"):
            with self.subTest(content=content):
                if content is not None:
                    (self.root / "config.json").write_text(content)
                before = self.snapshot()
                with self.assertRaises(ValueError):
                    self.run_train(resume=True)
                self.assertEqual(self.snapshot(), before)
        self.logged.assert_not_called()

    def test_incomplete_preserved_and_retried_from_completed(self):
        self.interrupted()
        first = self.snapshot(self.root / "seg-00000")
        unfinished = self.root / "seg-00001"
        (unfinished / "decoder-student.pt").write_bytes(b"partial weights must not be used")
        old = self.snapshot(unfinished)
        self.run_train(resume=True)
        self.assertEqual(self.logged.call_count, 2)
        archives = list(self.root.glob("seg-00001.interrupted-*"))
        self.assertEqual(len(archives), 1)
        self.assertEqual(self.snapshot(archives[0]), old)
        self.assertEqual(self.snapshot(self.root / "seg-00000"), first)
        command = self.logged.call_args_list[0].args[0]
        self.assertEqual(command[command.index("--init-decoder-checkpoint") + 1],
                         str(self.root / "seg-00000/decoder-student.pt"))

    def test_repeated_interruptions_have_unique_archives_and_no_receipts(self):
        self.interrupted(index=0)
        for _ in range(2):
            self.logged.side_effect = KeyboardInterrupt()
            with self.assertRaises(KeyboardInterrupt):
                self.run_train(resume=True)
            self.assertFalse((self.root / "seg-00000/completion.json").exists())
        self.assertEqual(len(list(self.root.glob("seg-00000.interrupted-*"))), 2)
        self.logged.side_effect = self.artifacts
        self.logged.reset_mock()
        self.run_train(resume=True)
        self.assertEqual(len(list(self.root.glob("seg-00000.interrupted-*"))), 3)
        self.assertNotIn("--init-decoder-checkpoint", self.logged.call_args_list[0].args[0])

    def test_legacy_completed_without_receipt_rejects(self):
        self.run_train()
        (self.root / "seg-00001/completion.json").unlink()
        before = self.snapshot()
        self.logged.reset_mock()
        with self.assertRaisesRegex(ValueError, "lacks completion.json"):
            self.run_train(resume=True)
        self.logged.assert_not_called()
        self.assertEqual(self.snapshot(), before)

    def test_corrupt_completed_artifacts_or_receipt_reject_without_writes(self):
        self.run_train()
        self.logged.reset_mock()
        for name in (*decoder.COMPLETION_ARTIFACTS, "invocation.json", "completion.json"):
            for damaged in (b"corrupted", b"", None):
                with self.subTest(name=name, damaged=damaged):
                    path = self.root / "seg-00002" / name
                    original = path.read_bytes()
                    if damaged is None:
                        path.unlink()
                    else:
                        path.write_bytes(damaged)
                    before = self.snapshot()
                    with self.assertRaises(ValueError):
                        self.run_train(resume=True)
                    self.assertEqual(self.snapshot(), before)
                    path.write_bytes(original)
        self.logged.assert_not_called()

    def test_preflight_rejects_gap_before_archiving_incomplete(self):
        self.run_train()
        (self.root / "seg-00000").rename(self.root / "seg-00000.interrupted-test")
        (self.root / "seg-00000").mkdir()
        before = self.snapshot()
        self.logged.reset_mock()
        with self.assertRaisesRegex(ValueError, "Non-contiguous"):
            self.run_train(resume=True)
        self.assertEqual(self.snapshot(), before)
        self.logged.assert_not_called()

    def test_success_without_valid_artifacts_never_gets_receipt(self):
        for name in decoder.COMPLETION_ARTIFACTS:
            with self.subTest(name=name):
                def missing(command, log_path):
                    self.artifacts(command, log_path)
                    (log_path.parent / name).unlink()
                self.logged.side_effect = missing
                with self.assertRaises(ValueError):
                    self.run_train(resume=True)
                self.assertFalse((self.root / "seg-00000/completion.json").exists())

    def test_failure_after_artifacts_does_not_get_receipt(self):
        def failed(command, log_path):
            self.artifacts(command, log_path)
            raise subprocess.CalledProcessError(1, command)
        self.logged.side_effect = failed
        with self.assertRaises(subprocess.CalledProcessError):
            self.run_train()
        self.assertFalse((self.root / "seg-00000/completion.json").exists())
        self.logged.reset_mock()
        with self.assertRaisesRegex(ValueError, "lacks completion.json"):
            self.run_train(resume=True)
        self.logged.assert_not_called()

    def test_smoke_is_separate_and_cannot_resume(self):
        self.run_train()
        before = self.snapshot()
        self.logged.reset_mock()
        decoder.train(self.cfg, self.upstream, smoke=True)
        self.assertEqual(self.logged.call_count, 1)
        command = self.logged.call_args.args[0]
        self.assertEqual(command[command.index("--steps") + 1], "5")
        self.assertNotIn("--init-decoder-checkpoint", command)
        smoke_root = self.root.with_name(self.root.name + "-smoke")
        self.assertTrue((smoke_root / "seg-00000/completion.json").is_file())
        self.assertEqual(self.snapshot(), before)
        self.assertEqual(self.cfg["segment_steps"], 10)
        with self.assertRaisesRegex(ValueError, "not smoke"):
            decoder.train(self.cfg, self.upstream, smoke=True, resume=True)

    def test_cli_resume_is_opt_in_and_train_only(self):
        with patch.object(decoder, "upstream_path", return_value=self.upstream), \
                patch.object(decoder, "load_config", return_value=self.cfg), \
                patch.object(decoder, "train") as train:
            for flag in ([], ["--resume"]):
                with patch.object(sys, "argv", ["decoder_long.py", "train", *flag]):
                    decoder.main()
                train.assert_called_with(self.cfg, self.upstream, smoke=False, resume=bool(flag))
            for stage in ("smoke", "check", "_train"):
                train.reset_mock()
                with patch.object(sys, "argv", ["decoder_long.py", stage, "--resume"]), \
                        contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as error:
                        decoder.main()
                self.assertEqual(error.exception.code, 2)
                train.assert_not_called()


if __name__ == "__main__":
    unittest.main()