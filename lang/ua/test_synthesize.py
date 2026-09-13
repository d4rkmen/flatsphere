import unittest
from cook import upstream_path
from synthesize import load_duration_module, phoneme_ids


class SynthesizeTests(unittest.TestCase):
    def test_duration_module_loads_without_side_effects(self):
        module = load_duration_module(upstream_path(None))
        self.assertTrue(callable(module.predict_durations))
        self.assertTrue(callable(module.load_model_from_checkpoint))

    def test_phoneme_ids_are_in_vocabulary(self):
        from piper.voice import PiperVoice
        from cook import teacher_paths
        speaker, model, config = teacher_paths("female")
        voice = PiperVoice.load(str(model), config_path=str(config))
        for text in ["Зараз перша година.", "Доброго ранку!"]:
            ids = phoneme_ids(voice, text)
            self.assertTrue(ids)
            self.assertTrue(all(0 <= i < 256 for i in ids))


if __name__ == "__main__":
    unittest.main()
