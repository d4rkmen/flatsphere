"""Host execution of the actual local engine/data and production G2P adapter."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
E = ROOT / "components/espeak_ng"
S = ROOT / "components/sanotts"


def main():
    cc = os.environ.get("CC", "cc")
    with tempfile.TemporaryDirectory() as directory:
        tmp = Path(directory)
        sources = sorted((E / "upstream/src/libespeak-ng").glob("*.c"))
        sources = [p for p in sources if p.name != "speech.c"]
        sources += [E / f"upstream/src/ucd-tools/src/{name}.c"
                    for name in ("case", "categories", "ctype", "proplist", "scripts", "tostring")]
        flags = ["-O2", "-std=gnu11", "-DHAVE_CONFIG_H", f"-I{E / 'port'}",
                 f"-I{E / 'upstream/src/include'}", f"-I{E / 'upstream/src/libespeak-ng'}",
                 f"-I{E / 'upstream/src/ucd-tools/src/include'}",
                 f"-I{S / 'upstream/g2p'}", f"-I{ROOT / 'tests/sanotts/host_stubs'}"]
        for language in ("en", "uk"):
            subprocess.run(["cmake", f"-DROOT={ROOT}", f"-DGENERATED={tmp}",
                            f"-DLANGUAGE={language}", "-P", str(ROOT / "tests/sanotts/prepare_g2p.cmake")], check=True)
            adapter = tmp / "adapter.o"
            subprocess.run([cc, *flags, "-Despeak_ng_InitializePath=host_initialize_path",
                            "-c", str(tmp / "esp_g2p.c"), "-o", str(adapter)], check=True)
            binary = tmp / language
            subprocess.run([cc, *flags, "-w", *(str(p) for p in sources), str(tmp / "speech.c"),
                            str(adapter), str(ROOT / "tests/sanotts/g2p_host.c"), "-lm", "-o", str(binary)], check=True)
            subprocess.run([str(binary), str(E / "data"), language], check=True)


if __name__ == "__main__":
    main()