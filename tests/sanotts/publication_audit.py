"""Read-only publication integrity and local host parity audit.

Experimental parity failures are reported as failures, not accepted; only
packaging failures and Kristin regressions make this audit exit nonzero.
"""
from pathlib import Path
import hashlib
import json
import subprocess
import tempfile
import sys
import re

ROOT = Path(__file__).resolve().parents[2]
C = ROOT / "components/sanotts"


def run(args, **kwargs):
    return subprocess.run(args, cwd=ROOT, check=True, **kwargs)


def main():
    run([sys.executable, str(ROOT / "tests/sanotts/package_voices.py")])
    provenance = json.loads((C / "upstream/provenance.json").read_text())
    for name, entry in provenance["files"].items():
        assert hashlib.sha256((C / name).read_bytes()).hexdigest() == entry["sha256"], name
    print("PASS: 25 original pinned files including Kristin offsets and licenses", flush=True)
    engine = ROOT / "components/espeak_ng"
    provenance = json.loads((engine / "provenance.json").read_text())
    for name, entry in provenance["files"].items():
        p = engine / name
        assert hashlib.sha256(p.read_bytes()).hexdigest() == entry["sha256"], p
        assert subprocess.run(["git", "check-ignore", "--no-index", str(p)], cwd=ROOT,
                              capture_output=True).returncode == 1, p
    print("PASS: local eSpeak sources, licenses and EN/UK data hashes", flush=True)
    run(["cmake", f"-DCOMPONENT={C}", "-P", str(ROOT / "tests/sanotts/voice_selection.cmake")])
    firmware = [ROOT / "CMakeLists.txt", ROOT / "main/main.cpp", ROOT / "main/idf_component.yml",
                ROOT / "sdkconfig", ROOT / "sdkconfig.old", ROOT / "sdkconfig.sanotts.defaults",
                C / "CMakeLists.txt", C / "Kconfig", *C.glob("cmake/*.cmake"),
                engine / "CMakeLists.txt", *engine.glob("cmake/*.cmake")]
    for p in firmware:
        text = p.read_text()
        for forbidden in ("picotts", "FLATSPHERE_TTS_PICO", "FetchContent", "lang/ua/work", "build/_deps"):
            assert forbidden not in text, (p, forbidden)
    assert "# CONFIG_FLATSPHERE_FLASH_FACTORY_NVS is not set" in (ROOT / "sdkconfig").read_text()
    config = (ROOT / "sdkconfig").read_text()
    choices = ("FLATSPHERE_TTS_VOICE_KRISTIN", "FLATSPHERE_TTS_VOICE_SERHIY")
    assert sum(f"CONFIG_{name}=y" in config for name in choices) == 1
    assert len(re.findall(r'^CONFIG_FLATSPHERE_TTS_VOICE_BUNDLE="[a-z0-9-]+"$', config, re.M)) == 1
    assert "picotts" not in (ROOT / "dependencies.lock").read_text().lower()
    for voice in ("en-kristin", "uk-serhiy"):
        d = C / "voices" / voice
        manifest = json.loads((d / "bundle.json").read_text())
        for name, entry in manifest["artifacts"].items():
            p = d / name
            assert hashlib.sha256(p.read_bytes()).hexdigest() == entry["sha256"], p
            ignored = subprocess.run(["git", "check-ignore", "--no-index", str(p)], cwd=ROOT, capture_output=True)
            assert ignored.returncode == 1, (p, "ignored")
    print("PASS: all voice files publishable; no ignored firmware dependencies; NVS disabled", flush=True)
    run(["sh", str(ROOT / "tests/sanotts/run_host_tests.sh")])
    run([sys.executable, str(ROOT / "tests/sanotts/g2p_test.py")])
    with tempfile.TemporaryDirectory() as temp:
        run(["cmake", f"-DCOMPONENT={C}", f"-DGENERATED={temp}", "-P",
             str(ROOT / "tests/sanotts/prepare_host.cmake")])
        for voice in ("en-kristin", "uk-serhiy"):
            binary = str(Path(temp) / voice)
            run(["cc", "-O2", "-std=c99", "-DFSD_FAST_MATH", f"-I{C / 'voices' / voice}",
                 f"-I{C / 'upstream/include'}", str(Path(temp) / "snt_tts.c"),
                 str(C / "upstream/src/snt_kernels_ref.c"), str(C / "upstream/host/snt_port_host.c"),
                 str(ROOT / "lang/ua/voice_golden.c"), "-lm", "-o", binary])
            result = subprocess.run([binary, str(C / "voices" / voice)], cwd=ROOT, capture_output=True, text=True)
            print(f"{voice}: parity exit {result.returncode}\n{result.stdout}{result.stderr}", flush=True)
            if voice == "en-kristin":
                assert result.returncode == 0
            else:
                assert result.returncode in (0, 1), "Unexpected host failure"
                # Exit 1 alone also covers corrupt vectors/nonfinite PCM/runtime
                # failures. Only a completed finite parity measurement is allowed.
                assert not result.stderr, result.stderr
                assert re.search(r"golden corr -?\d+\.\d+ .*required > 0.98", result.stdout), result.stdout
                assert re.search(r"golden rms_ratio \d+\.\d+", result.stdout), result.stdout
    print("PASS: packaging audit (not Ukrainian parity acceptance)")


if __name__ == "__main__":
    main()