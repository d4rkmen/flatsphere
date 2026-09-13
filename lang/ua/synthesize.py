#!/usr/bin/env python3
"""Full-stack Ukrainian synthesis: text -> learned duration -> acoustic -> decoder.

This is a listening/quality harness, not a deployable exporter. It uses the
Piper teacher only for phonemization and as a listening reference. Every lane is
deterministic (zero teacher noise). Checkpoints load with restricted unpickling.
"""
from __future__ import annotations

import argparse
import html
import importlib.util
import json
from pathlib import Path
import sys
import wave

from cook import ROOT, WORK, digest, read_rows, teacher_paths, upstream_path, write_json
from lrc_experiment import PINNED_FILES, fresh, load_decoder_module
from acoustic_experiment import TRAINER_SHA as ACOUSTIC_SHA, load_acoustic_module

DURATION_TRAINER = "tools/train_roota_piper_duration_student.py"
DURATION_SHA = "f4837da6f8c36a1f5061682e6c9ffb6cc0a233b1273c7bc54f48393686d6646a"


def load_duration_module(upstream: Path):
    path = upstream / DURATION_TRAINER
    if digest(path) != DURATION_SHA:
        raise ValueError("Duration trainer changed; review before synthesis")
    spec = importlib.util.spec_from_file_location("ua_duration_trainer", path)
    if spec is None or spec.loader is None:
        raise ImportError(DURATION_TRAINER)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module.__name__] = module
    spec.loader.exec_module(module)
    return module


def load_checkpoint(path: Path) -> dict:
    import torch
    from pathlib import PosixPath
    if not path.is_file():
        raise FileNotFoundError(path)
    with torch.serialization.safe_globals([PosixPath]):
        return torch.load(path, map_location="cpu", weights_only=True)


def load_stack(upstream: Path, duration_ckpt: Path, acoustic_ckpt: Path, decoder_ckpt: Path):
    import torch
    dm = load_duration_module(upstream)
    am = load_acoustic_module(upstream)
    dmod = load_decoder_module(upstream)

    duration_state = load_checkpoint(duration_ckpt)
    acoustic_state = load_checkpoint(acoustic_ckpt)
    decoder_state = load_checkpoint(decoder_ckpt)
    for name, state in (("duration", duration_state), ("acoustic", acoustic_state), ("decoder", decoder_state)):
        if not isinstance(state.get("config"), dict) or not isinstance(state.get("model_state_dict"), dict):
            raise ValueError(f"{name}: checkpoint missing config/model_state_dict")

    duration = dm.DurationStudent(
        vocab_size=int(duration_state["config"]["vocab_size"]),
        hidden=int(duration_state["config"]["hidden"]),
        depth=int(duration_state["config"]["depth"]),
        kernel_size=int(duration_state["config"]["kernel_size"]),
        max_tokens=int(duration_state["config"]["max_tokens"]),
    )
    duration.load_state_dict(duration_state["model_state_dict"], strict=True)
    duration.eval()

    acoustic = am.create_model_from_config(acoustic_state["config"])
    acoustic.load_state_dict(acoustic_state["model_state_dict"], strict=True)
    acoustic.eval()

    import inspect
    signature = inspect.signature(dmod.DecoderStudent)
    kwargs = {key: value for key, value in decoder_state["config"].items() if key in signature.parameters}
    decoder = dmod.DecoderStudent(**kwargs)
    decoder.load_state_dict(decoder_state["model_state_dict"], strict=True)
    decoder.eval()

    return dm, am, dmod, duration, acoustic, decoder


def phoneme_ids(voice, text: str) -> list[int]:
    sentences = voice.phonemize(text)
    if not sentences:
        raise ValueError(f"Piper produced no phonemes for: {text!r}")
    ids: list[int] = []
    for sentence in sentences:
        ids.extend(voice.phonemes_to_ids(sentence))
    return ids


def synth_one(voice, dm, am, dmod, duration, acoustic, decoder, text: str, max_duration: int = 80):
    import numpy as np
    import torch
    ids = phoneme_ids(voice, text)
    if not ids or any(i < 0 or i >= 256 for i in ids):
        raise ValueError("Invalid phoneme IDs")
    ids_t = torch.tensor([ids], dtype=torch.long)
    mask = torch.ones_like(ids_t, dtype=torch.bool)
    with torch.inference_mode():
        durations = dm.predict_durations(duration, ids_t, mask, max_duration=max_duration, length_scale=1.0)[0]
    durations = durations.tolist()
    if sum(durations) <= 0:
        raise ValueError("Empty duration prediction")
    sample = am.ChunkSample(
        row_id="synthesized", row_index=0, text=text, chunk_index=0,
        phoneme_ids=np.asarray(ids, dtype=np.int64), durations=np.asarray(durations, dtype=np.int64),
        target=np.zeros((sum(durations), 40), dtype=np.float32),
        tensor_path=Path("synthesized"), audio_samples=sum(durations) * 256)
    with torch.inference_mode():
        code = am.predict_chunk(acoustic, sample, torch.device("cpu"))
        audio = decoder(torch.from_numpy(code)).reshape(-1).numpy()
    return audio, code, durations


def teacher_audio(voice, text: str):
    import io
    import numpy as np
    from piper.config import SynthesisConfig
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as wav:
        voice.synthesize_wav(text, wav, syn_config=SynthesisConfig(noise_scale=0, noise_w_scale=0, length_scale=1))
    buffer.seek(0)
    with wave.open(buffer, "rb") as wav:
        pcm = np.frombuffer(wav.readframes(wav.getnframes()), dtype="<i2").astype(np.float32) / 32767.0
        rate = wav.getframerate()
    return pcm, rate


def write_wav(path: Path, audio, rate: int = 22050) -> None:
    import numpy as np
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(rate)
        wav.writeframes((np.clip(audio, -1, 1) * 32767).astype("<i2").tobytes())


def synthesize(role: str, upstream: Path, name: str, texts: list[str],
               duration_ckpt: Path | None = None, acoustic_ckpt: Path | None = None,
               decoder_ckpt: Path | None = None) -> None:
    import numpy as np
    from piper.voice import PiperVoice
    speaker, model, config = teacher_paths(role)
    voice = PiperVoice.load(str(model), config_path=str(config))
    base = WORK / speaker
    duration_ckpt = duration_ckpt or (base / "duration-100000-mps-v1/duration-student.pt")
    acoustic_ckpt = acoustic_ckpt or (base / "acoustic-150000-mps-v1/latent-student.pt")
    decoder_ckpt = decoder_ckpt or (base / "decoder-200000-cpu-v1/seg-00019/decoder-student.pt")
    out = base / name
    fresh(out)
    out.mkdir()
    dm, am, dmod, duration, acoustic, decoder = load_stack(
        upstream, duration_ckpt, acoustic_ckpt, decoder_ckpt)
    audio_dir = out / "audio"
    audio_dir.mkdir()
    records = []
    for index, text in enumerate(texts):
        audio, code, durations = synth_one(voice, dm, am, dmod, duration, acoustic, decoder, text)
        teacher, rate = teacher_audio(voice, text)
        if audio.size == 0 or not np.isfinite(audio).all():
            raise ValueError("Empty/non-finite student audio")
        full = audio_dir / f"{index:02d}-full.wav"
        reference = audio_dir / f"{index:02d}-teacher.wav"
        write_wav(full, audio)
        write_wav(reference, teacher, rate)
        records.append({
            "text": text,
            "student_seconds": audio.size / 22050.0,
            "teacher_seconds": teacher.size / rate,
            "student_rms": float(np.sqrt(np.mean(audio ** 2))),
            "teacher_rms": float(np.sqrt(np.mean(teacher ** 2))),
            "student_peak": float(np.max(np.abs(audio))),
            "phoneme_ids": len(phoneme_ids(voice, text)),
            "learned_frames": int(sum(durations)),
            "student_file": full.name,
            "teacher_file": reference.name,
        })
    write_json(out / "report.json", {
        "voice": speaker, "role": role,
        "duration_checkpoint": str(duration_ckpt), "acoustic_checkpoint": str(acoustic_ckpt),
        "decoder_checkpoint": str(decoder_ckpt),
        "source_sha256": {**PINNED_FILES, DURATION_TRAINER: DURATION_SHA,
                          "train_roota_piper_latent_student.py": ACOUSTIC_SHA},
        "duration_sha256": digest(duration_ckpt), "acoustic_sha256": digest(acoustic_ckpt),
        "decoder_sha256": digest(decoder_ckpt),
        "lanes": records, "quality_accepted": False, "arbitrary_text_ready": False,
    })
    write_index(out, records)
    print(json.dumps({"synthesized": len(records), "out": str(out)}, ensure_ascii=False))


def write_index(out: Path, records: list[dict]) -> None:
    rows = []
    for index, record in enumerate(records):
        text = html.escape(record["text"])
        rows.append(
            f"<tr><td>{index + 1}</td><td>{text}</td>"
            f"<td><audio controls src='audio/{record['student_file']}'></audio></td>"
            f"<td><audio controls src='audio/{record['teacher_file']}'></audio></td>"
            f"<td>{record['student_seconds']:.2f}s / {record['teacher_seconds']:.2f}s</td></tr>")
    (out / "index.html").write_text(
        "<!doctype html><meta charset='utf-8'><title>FlatSphere UA synthesis</title>"
        "<style>body{font-family:sans-serif;margin:2rem}table{border-collapse:collapse}"
        "td,th{border:1px solid #ccc;padding:.5rem;text-align:left}</style>"
        "<h1>FlatSphere Ukrainian synthesis</h1><p>Left = full student, right = Piper teacher.</p>"
        "<table><tr><th>#</th><th>Text</th><th>Student</th><th>Teacher</th><th>Duration</th></tr>"
        + "".join(rows) + "</table>", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--voice", choices=("male", "female"), default="female")
    parser.add_argument("--name", default="synthesize-v1")
    parser.add_argument("--texts", type=Path, default=ROOT / "texts/clock_eval.jsonl")
    parser.add_argument("--duration-ckpt", type=Path)
    parser.add_argument("--acoustic-ckpt", type=Path)
    parser.add_argument("--decoder-ckpt", type=Path)
    parser.add_argument("--upstream")
    args = parser.parse_args()
    upstream = upstream_path(args.upstream)
    texts = [row["text"] for row in read_rows(args.texts)]
    if not texts:
        raise ValueError("No source texts")
    synthesize(args.voice, upstream, args.name, texts,
               args.duration_ckpt, args.acoustic_ckpt, args.decoder_ckpt)


if __name__ == "__main__":
    main()
