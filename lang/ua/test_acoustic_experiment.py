import unittest
from acoustic_experiment import defer_qat_import, load_acoustic_module
from cook import upstream_path


class AcousticTests(unittest.TestCase):
    def test_patch_is_lazy_not_fake_qat(self):
        source = 'import qat_ste  # noqa: E402\ndef train(args):\n    if str(args.qat) == "c-int8":\n        qat_ste.enable_dense_conv1d_qat(model)\n'
        patched = defer_qat_import(source)
        self.assertNotIn('\nimport qat_ste', '\n' + patched)
        self.assertIn('        import qat_ste\n', patched)
        self.assertIn('qat_ste.enable_dense_conv1d_qat(model)', patched)

    def test_patch_rejects_unexpected_source(self):
        with self.assertRaises(ValueError):
            defer_qat_import('different upstream source')

    def test_non_qat_import_works(self):
        module = load_acoustic_module(upstream_path(None))
        self.assertTrue(callable(module.main))
        self.assertNotIn('qat_ste', module.__dict__)


if __name__ == '__main__':
    unittest.main()