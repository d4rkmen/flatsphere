import unittest
from pathlib import Path
from cook import ROOT, upstream_path
from duration_long import load_config, command_args


class LongDurationTests(unittest.TestCase):
    def test_long_run_configuration(self):
        cfg = load_config(ROOT / "duration-long.json")
        args = command_args(cfg, upstream_path(None))
        for flag, value in (("--steps", "100000"), ("--save-interval", "1000"),
                            ("--duration-vocab-size", "256"), ("--device", "mps")):
            self.assertEqual(args[args.index(flag) + 1], value)
        self.assertNotIn("--initial-checkpoint", args)
        self.assertNotIn("--checkpoint-steps", args)
        self.assertIn("duration-packs-full-v1", str(Path(args[args.index('--pack-dir') + 1])))


if __name__ == '__main__':
    unittest.main()