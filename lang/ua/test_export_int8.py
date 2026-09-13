"""Fast synthetic exporter tests; no training, teacher downloads, or firmware edits."""
from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import torch

import export_int8 as exporter


def checkpoints():
    configs = [
        dict(architecture="duration_conv", vocab_size=256, hidden=32, depth=3,
             kernel_size=5, max_tokens=155, max_duration=80),
        dict(architecture="token_context", vocab_size=256, hidden=48, depth=5,
             token_depth=3, kernel_size=5, out_channels=40),
        dict(variant="lrc", in_channels=40, source_in_channels=192,
             lrc_code_dim=40, lrc_encoder_hidden=64, fsd_dim=76, fsd_blocks=5,
             fsd_film_rank=12, fsd_head_rank=48, istft_n_fft=1024),
    ]
    shapes = exporter.tensor_shapes()
    states = [dict(config=config, model_state_dict={
        key: torch.full(shape, 0.125) for key, shape in shapes[name].items()})
        for name, config in zip(("duration", "acoustic", "decoder"), configs)]
    states[2]["lrc_encoder_state_dict"] = {
        key: torch.full(shape, 0.125) for key, shape in shapes["encoder"].items()}
    return states


class ExportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old_threads = torch.get_num_threads()
        torch.set_num_threads(1)

    @classmethod
    def tearDownClass(cls):
        torch.set_num_threads(cls.old_threads)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.out = self.root / "export"
        self.states = checkpoints()
        self.paths = [self.root / f"{name}-400000.pt" for name in ("duration", "acoustic", "decoder")]
        for path, state in zip(self.paths, self.states):
            torch.save(state, path)
        self.args = ["--voice", "male", "--out", str(self.out)]
        for name, path in zip(("duration", "acoustic", "decoder"), self.paths):
            self.args += [f"--{name}-ckpt", str(path)]
        self.work_patch = patch.object(exporter, "WORK", self.root / "work")
        self.work_patch.start()
        self.addCleanup(self.work_patch.stop)

    def assert_no_export(self):
        self.assertFalse(self.out.exists())
        self.assertEqual(list(self.root.glob(".export.tmp-*")), [])

    def make_pack(self, path=None):
        pack = path or self.root / "male-pack"
        pack.mkdir(parents=True)
        np.savez(pack / "chunk.npz", phoneme_ids=np.array([1, 2, 3]), w_ceil=np.array([1, 2, 1]))
        (pack / "rows.json").write_text(json.dumps([
            {"row_id": "mykyta-row", "text": "Тест", "chunks": [
                {"chunk_index": 0, "tensor_npz": "chunk.npz"}]}]))
        return pack

    def test_valid_topology_and_duration_normalizer(self):
        self.assertEqual(exporter.validate_checkpoints(*self.states), 155)
        front, meta = exporter.build_front(*self.states[:2])
        header = self.root / "front.h"
        exporter.write_front_header(meta, header, len(front), 155)
        self.assertIn("#define FRONT_DUR_MAX_TOKENS 155\n", header.read_text())
        self.assertEqual(meta["DUR_PROJ"][0], 256 * 32 * 4)
        model, model_meta = exporter.build_decoder(self.states[2])
        self.assertEqual(model_meta["ENC1"][0], 64 * 192 + 2 * 64 * 4)
        self.assertGreater(len(model), 0)

    def test_reject_incompatible_configs(self):
        for index, key, value in ((0, "vocab_size", 143), (0, "depth", 4),
                                  (0, "max_duration", 81), (1, "architecture", "frame_conv"),
                                  (1, "token_depth", 4), (2, "variant", "fsd"),
                                  (2, "istft_n_fft", 512), (2, "fsd_dim", 80),
                                  (2, "post_filter_channels", 4)):
            with self.subTest(key=key):
                states = copy.deepcopy(self.states)
                states[index]["config"][key] = value
                with self.assertRaisesRegex(ValueError, key):
                    exporter.validate_checkpoints(*states)
        for value in (None, 0, -1, True, 1.5, float("inf"), 2**31):
            with self.subTest(max_tokens=value):
                states = copy.deepcopy(self.states)
                states[0]["config"]["max_tokens"] = value
                with self.assertRaisesRegex(ValueError, "max_tokens"):
                    exporter.validate_checkpoints(*states)

    def test_reject_missing_extra_and_wrong_shapes(self):
        for mode in ("missing", "extra", "shape"):
            with self.subTest(mode=mode):
                states = copy.deepcopy(self.states)
                m = states[2]["model_state_dict"]
                if mode == "missing":
                    del m["fsd_blocks.0.norm.bias"]
                elif mode == "extra":
                    m["fsd_blocks.5.scale"] = torch.tensor(0.1)
                else:
                    m["fsd_blocks.0.depthwise.weight"] = torch.zeros(76, 7, 1)
                with self.assertRaisesRegex(ValueError, "tensor"):
                    exporter.validate_checkpoints(*states)

    def test_reject_nonfinite_in_every_state(self):
        for index, state_key, tensor_key in (
            (0, "model_state_dict", "embedding.weight"),
            (1, "model_state_dict", "token_blocks.0.scale"),
            (2, "model_state_dict", "fsd_blocks.0.norm.weight"),
            (2, "lrc_encoder_state_dict", "net.0.weight"),
        ):
            for value in (float("nan"), float("inf"), 1e100):
                with self.subTest(key=tensor_key, value=value):
                    states = copy.deepcopy(self.states)
                    tensor = states[index][state_key][tensor_key].double()
                    tensor.fill_(value)
                    states[index][state_key][tensor_key] = tensor
                    with self.assertRaisesRegex(ValueError, "finite"):
                        exporter.validate_checkpoints(*states)

    def test_quantization_padding_zero_and_subnormal(self):
        weight = torch.tensor([[[0., 0., 0.]], [[-1., 0., 1.]], [[1e-44, 0., 0.]]])
        q, scale, bias = exporter.quantize_conv(weight, torch.zeros(3))
        self.assertEqual(q.shape, (3, 16))
        np.testing.assert_array_equal(q[1, :3], [-127, 0, 127])
        self.assertTrue((q[:, 3:] == 0).all())
        self.assertTrue(np.isfinite(scale).all())
        self.assertTrue((scale > 0).all())
        self.assertEqual(scale[0], 1.)
        self.assertTrue((bias == 0).all())

    def test_explicit_male_export_and_manifest(self):
        with patch.object(exporter, "upstream_path", side_effect=AssertionError("not needed")):
            exporter.main(self.args + ["--skip-golden"])
        report = json.loads((self.out / "export_report.json").read_text())
        manifest = json.loads((self.out / "export_manifest.json").read_text())
        self.assertEqual(report["speaker"], "mykyta")
        self.assertEqual(report["duration_max_tokens"], 155)
        self.assertEqual(report["golden"], "skipped")
        for name, path in zip(("duration", "acoustic", "decoder"), self.paths):
            self.assertEqual(report[f"{name}_checkpoint"], str(path.resolve()))
            self.assertEqual(manifest["checkpoints"][name]["sha256"], exporter.digest(path))
        for relative, item in manifest["artifacts"].items():
            path = self.out / relative
            self.assertEqual(item["sha256"], exporter.digest(path))
            self.assertEqual(item["bytes"], path.stat().st_size)
        self.assertIn("model/front_q8_meta.h", manifest["artifacts"])
        self.assertFalse((self.out / "e2e_audio.bin").exists())
        self.assertEqual(list(self.root.glob(".export.tmp-*")), [])

    def test_legacy_female_checkpoint_defaults(self):
        for relative, state in zip((exporter.DUR_CKPT, exporter.AC_CKPT, exporter.DEC_CKPT), self.states):
            path = exporter.WORK / "tetiana" / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            torch.save(state, path)
        exporter.main(["--out", str(self.out), "--skip-golden"])
        report = json.loads((self.out / "export_report.json").read_text())
        self.assertEqual(report["speaker"], "tetiana")
        self.assertEqual(report["decoder_checkpoint"],
                         str((exporter.WORK / "tetiana" / exporter.DEC_CKPT).resolve()))

    def test_pack_selection(self):
        pack = self.make_pack()
        self.assertEqual(exporter.select_eval_pack(pack, "mykyta"), pack.resolve())
        checkpoint = {"train_args": {"eval_pack_dir": str(pack)}}
        self.assertEqual(exporter.select_eval_pack(None, "mykyta", checkpoint), pack.resolve())
        self.assertEqual(exporter.select_eval_pack(None, "mykyta"),
                         (exporter.WORK / "mykyta/pack-eval").resolve())
        legacy = exporter.WORK / "tetiana/duration-packs-full-v1/pack-eval"
        legacy.mkdir(parents=True)
        self.assertEqual(exporter.select_eval_pack(None, "tetiana"), legacy.resolve())

    def test_reject_opposite_voice_pack_or_training_metadata(self):
        female = exporter.WORK / "tetiana/pack-eval"
        with self.assertRaisesRegex(ValueError, "references tetiana"):
            exporter.validate_voice("mykyta", [female])
        with self.assertRaisesRegex(ValueError, "references tetiana"):
            exporter.validate_voice("mykyta", [], {"train_args": {"pack_dir": str(female)}})

    def test_golden_uses_selected_male_states_and_pack(self):
        pack = self.make_pack()
        acoustic_model = Mock()
        decoder_model = Mock(return_value=torch.zeros(1, 1, 4 * 256))
        am = SimpleNamespace(create_model_from_config=Mock(return_value=acoustic_model),
                             ChunkSample=lambda **kwargs: SimpleNamespace(**kwargs),
                             predict_chunk=Mock(return_value=np.zeros((1, 40, 4), np.float32)))
        dm = SimpleNamespace(DecoderStudent=lambda: decoder_model)
        with patch.object(exporter, "upstream_path", return_value=self.root), \
                patch.object(exporter, "load_acoustic_module", return_value=am), \
                patch.object(exporter, "load_decoder_module", return_value=dm), \
                patch.object(exporter, "load_state", wraps=exporter.load_state) as load:
            exporter.main(self.args + ["--eval-pack", str(pack)])
        self.assertEqual([call.args[0] for call in load.call_args_list], [p.resolve() for p in self.paths])
        acoustic_model.load_state_dict.assert_called_once()
        decoder_model.load_state_dict.assert_called_once()
        sample = am.predict_chunk.call_args.args[1]
        self.assertEqual(sample.row_id, "mykyta-row")
        np.testing.assert_array_equal(sample.phoneme_ids, [1, 2, 3])
        meta = json.loads((self.out / "golden_meta.json").read_text())
        self.assertEqual(meta["eval_pack"], str(pack.resolve()))
        self.assertEqual(meta["durations"], "teacher/oracle")
        self.assertEqual(meta["tensor_sha256"], exporter.digest(pack / "chunk.npz"))
        np.testing.assert_array_equal(np.fromfile(self.out / "e2e_durs.bin", dtype="<i4"), [1, 2, 1])

    def test_checkpoint_pack_default_is_passed_to_golden(self):
        pack = self.make_pack()
        self.states[1]["train_args"] = {"eval_pack_dir": str(pack)}
        torch.save(self.states[1], self.paths[1])
        with patch.object(exporter, "upstream_path", return_value=self.root), \
                patch.object(exporter, "build_golden") as golden:
            exporter.main(self.args)
        self.assertEqual(golden.call_args.args[4], pack.resolve())

    def test_validation_fails_before_any_artifact_writes(self):
        self.states[0]["config"]["vocab_size"] = 143
        torch.save(self.states[0], self.paths[0])
        with patch.object(exporter, "build_front") as build, \
                patch.object(exporter.tempfile, "mkdtemp") as staging:
            with self.assertRaisesRegex(ValueError, "vocab_size"):
                exporter.main(self.args + ["--skip-golden"])
        build.assert_not_called()
        staging.assert_not_called()
        self.assert_no_export()

    def test_cleanup_after_header_golden_and_manifest_failures(self):
        pack = self.make_pack()
        for function in ("write_front_header", "build_golden", "write_json"):
            with self.subTest(function=function), \
                    patch.object(exporter, "upstream_path", return_value=self.root), \
                    patch.object(exporter, function, side_effect=RuntimeError("injected failure")):
                with self.assertRaisesRegex(RuntimeError, "injected failure"):
                    exporter.main(self.args + (["--eval-pack", str(pack)] if function == "build_golden"
                                               else ["--skip-golden"]))
                self.assert_no_export()

    def test_cleanup_on_input_change(self):
        pack = self.make_pack()
        def mutate(*_):
            self.paths[1].write_bytes(b"changed checkpoint")
        with patch.object(exporter, "upstream_path", return_value=self.root), \
                patch.object(exporter, "build_golden", side_effect=mutate):
            with self.assertRaisesRegex(ValueError, "Checkpoint changed"):
                exporter.main(self.args + ["--eval-pack", str(pack)])
        self.assert_no_export()

    def test_cleanup_on_manifest_write_failure(self):
        write = exporter.write_json
        def fail_manifest(path, value):
            if path.name == "export_manifest.json":
                path.write_text("partial")
                raise OSError("manifest disk failure")
            write(path, value)
        with patch.object(exporter, "write_json", side_effect=fail_manifest):
            with self.assertRaisesRegex(OSError, "manifest disk failure"):
                exporter.main(self.args + ["--skip-golden"])
        self.assert_no_export()

    def test_missing_checkpoint_leaves_no_output(self):
        self.paths[2].unlink()
        with self.assertRaises(FileNotFoundError):
            exporter.main(self.args + ["--skip-golden"])
        self.assert_no_export()

    def test_invalid_golden_audio_cleans_export(self):
        pack = self.make_pack()
        am = SimpleNamespace(create_model_from_config=lambda _: Mock(),
                             ChunkSample=lambda **kwargs: SimpleNamespace(**kwargs),
                             predict_chunk=lambda *_: np.zeros((1, 40, 4), np.float32))
        decoder = Mock(return_value=torch.full((1, 1, 1024), float("nan")))
        dm = SimpleNamespace(DecoderStudent=lambda: decoder)
        with patch.object(exporter, "upstream_path", return_value=self.root), \
                patch.object(exporter, "load_acoustic_module", return_value=am), \
                patch.object(exporter, "load_decoder_module", return_value=dm):
            with self.assertRaisesRegex(ValueError, "Invalid golden audio"):
                exporter.main(self.args + ["--eval-pack", str(pack)])
        self.assert_no_export()

    def test_fractional_golden_ids_are_rejected_not_truncated(self):
        pack = self.make_pack()
        np.savez(pack / "chunk.npz", phoneme_ids=np.array([1.5]), w_ceil=np.array([4]))
        am = SimpleNamespace(create_model_from_config=lambda _: Mock())
        dm = SimpleNamespace(DecoderStudent=lambda: Mock())
        with patch.object(exporter, "upstream_path", return_value=self.root), \
                patch.object(exporter, "load_acoustic_module", return_value=am), \
                patch.object(exporter, "load_decoder_module", return_value=dm):
            with self.assertRaisesRegex(ValueError, "Invalid golden ids"):
                exporter.main(self.args + ["--eval-pack", str(pack)])
        self.assert_no_export()

    def test_refuses_existing_directory_file_and_dangling_symlink(self):
        for kind in ("directory", "file", "symlink"):
            with self.subTest(kind=kind):
                if kind == "directory":
                    self.out.mkdir()
                elif kind == "file":
                    self.out.write_text("keep")
                else:
                    self.out.symlink_to(self.root / "missing")
                with patch.object(exporter, "load_state") as load:
                    with self.assertRaises(FileExistsError):
                        exporter.main(self.args + ["--skip-golden"])
                    load.assert_not_called()
                if kind == "directory":
                    self.out.rmdir()
                else:
                    self.out.unlink()

    def test_publication_race_refuses_empty_directory_and_cleans_staging(self):
        publish = exporter.publish_export
        def race(staging, out):
            self.assertTrue((staging / "export_manifest.json").is_file())
            self.assertFalse(out.exists())
            out.mkdir()
            publish(staging, out)
        with patch.object(exporter, "publish_export", side_effect=race):
            with self.assertRaises(FileExistsError):
                exporter.main(self.args + ["--skip-golden"])
        self.assertTrue(self.out.is_dir())
        self.assertEqual(list(self.out.iterdir()), [])
        self.assertEqual(list(self.root.glob(".export.tmp-*")), [])


if __name__ == "__main__":
    unittest.main()