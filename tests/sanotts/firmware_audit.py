"""Read-only inspection of an ESP-IDF build; never builds, configures or flashes."""
import hashlib
import json
from pathlib import Path
import re
import shlex

ROOT = Path(__file__).resolve().parents[2]


def main():
    build = ROOT / "build"
    config = (build / "config/sdkconfig.h").read_text()
    selected = re.findall(r'#define CONFIG_FLATSPHERE_TTS_VOICE_BUNDLE "([a-z0-9-]+)"', config)
    assert len(selected) == 1, selected
    voice = selected[0]
    bundle = ROOT / "components/sanotts/voices" / voice
    assert ("#define CONFIG_FLATSPHERE_TTS_LANG_UK 1" in config) == voice.startswith("uk-")
    commands = json.loads((build / "compile_commands.json").read_text())
    for entry in commands:
        assert "_deps/" not in entry["file"], entry["file"]
        assert "picotts" not in entry["command"].lower()
    runtime = [e for e in commands if e["file"].endswith("/generated/snt_tts.c")]
    assert len(runtime) == 1
    includes = [Path(arg[2:]) for arg in shlex.split(runtime[0]["command"]) if arg.startswith("-I")]
    model_dirs = [p for p in includes if (p / "model/front_q8_meta.h").exists()]
    assert model_dirs == [bundle], model_dirs
    # Verify embedded bytes, not merely assembler input paths or symbol names.
    image = (build / "flatsphere.bin").read_bytes()
    for name in ("front_q8.bin", "model_q8.bin"):
        blob = (bundle / name).read_bytes()
        assert image.count(blob) == 1, name
        print(voice, name, hashlib.sha256(blob).hexdigest())
    link_map = (build / "flatsphere.map").read_text()
    for name in ("front_q8", "model_q8"):
        match = re.search(r"0x([0-9a-f]+)\s+_binary_" + name + r"_bin_start\b", link_map)
        assert match and int(match[1], 16) % 16 == 0, name
    generated = (build / "esp-idf/sanotts/generated/esp_g2p.c").read_text()
    language = "uk" if voice.startswith("uk-") else "en"
    assert f'espeak_ng_SetVoiceByName("{language}")' in generated
    flash = json.loads((build / "flasher_args.json").read_text())["flash_files"]
    assert flash.get("0x410000") == "espeak.bin"
    assert "0x9000" not in flash and not any("nvs" in f.lower() for f in flash.values())
    assert "picotts" not in (ROOT / "dependencies.lock").read_text().lower()
    assert (build / "espeak.bin").stat().st_size == 1024 * 1024
    print(f"PASS {voice}: selected headers/G2P, exact embedded blobs, alignment, no cache/Pico inputs, NVS excluded; app {len(image):#x} bytes")


if __name__ == "__main__":
    main()