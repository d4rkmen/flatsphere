import unittest
from pathlib import Path
from cook import ROOT, upstream_path
from decoder_long import load_config, check


class LongDecoderTests(unittest.TestCase):
    def test_long_decoder_configuration(self):
        cfg = load_config(ROOT / "decoder-long.json")
        self.assertEqual(cfg["total_steps"], 200000)
        self.assertEqual(cfg["total_steps"] % cfg["segment_steps"], 0)
        self.assertEqual(cfg["architecture"]["fsd_dim"], 76)
        self.assertEqual(cfg["architecture"]["lrc_code_dim"], 40)

    def test_check_reuses_full_packs(self):
        report = check(load_config(ROOT / "decoder-long.json"), upstream_path(None))
        self.assertEqual(report["segments"], 20)
        self.assertIn("duration-packs-full-v1", report["pack_root"])


if __name__ == "__main__":
    unittest.main()
