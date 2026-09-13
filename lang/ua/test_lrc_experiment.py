import tempfile
import unittest
from pathlib import Path

from cook import upstream_path
from lrc_experiment import fresh, load_decoder_module, training_args


class LrcTests(unittest.TestCase):
    def test_existing_run_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileExistsError):
                fresh(Path(directory))

    def test_architecture_matches_r7(self):
        args = training_args("female", 100, "cpu", Path("/tmp/test-lrc"))
        for flag, value in (("--variant", "lrc"), ("--lrc-code-dim", "40"),
                            ("--fsd-dim", "76"), ("--fsd-blocks", "5"),
                            ("--fsd-film-rank", "12"), ("--fsd-head-rank", "48"),
                            ("--istft-n-fft", "1024")):
            self.assertEqual(args[args.index(flag) + 1], value)

    def test_pinned_encoder_forward_backward(self):
        import torch
        module = load_decoder_module(upstream_path(None))
        model = module.LrcEncoder(in_channels=192, hidden=64, code_dim=40)
        latent = torch.randn(1, 192, 32, requires_grad=True)
        code = model(latent)
        self.assertEqual(tuple(code.shape), (1, 40, 32))
        code.square().mean().backward()
        self.assertIsNotNone(latent.grad)
        self.assertTrue(torch.isfinite(latent.grad).all())

    def test_larger_run_paths_and_evaluation(self):
        root = Path("/tmp/packs-256-32")
        args = training_args("female", 3000, "cpu", Path("/tmp/lrc"),
                             root, 4, 64, True, 5.0)
        self.assertEqual(args[args.index("--pack-dir") + 1], str(root / "pack-train"))
        self.assertEqual(args[args.index("--eval-pack-dir") + 1], str(root / "pack-eval"))
        self.assertNotIn("--skip-chunk-eval", args)
        self.assertEqual(args[args.index("--spectral-weight") + 1], "5.0")

    def test_smoke_defaults_remain_unchanged(self):
        args = training_args("female", 100, "cpu", Path("/tmp/lrc"))
        self.assertIn("--skip-chunk-eval", args)
        self.assertEqual(args[args.index("--batch-size") + 1], "1")


if __name__ == "__main__":
    unittest.main()