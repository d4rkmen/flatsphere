import unittest
from cook import canonical_text, split_for


class CorpusTests(unittest.TestCase):
    def test_apostrophes_and_stress_deduplicate(self):
        self.assertEqual(canonical_text("П’я́ть  хвилин."), canonical_text("п'ять хвилин."))

    def test_ukrainian_letters_preserved(self):
        self.assertEqual(canonical_text("Їжак, йод, ґанок."), "їжак, йод, ґанок.")

    def test_same_sentence_has_same_split_across_speakers(self):
        self.assertEqual(split_for("П’я́ть хвилин."), split_for("п'ять хвилин."))

    def test_partitions_are_nonempty(self):
        self.assertEqual({split_for(f"Речення номер {n}.") for n in range(1000)}, {"train", "eval", "test"})


if __name__ == "__main__":
    unittest.main()