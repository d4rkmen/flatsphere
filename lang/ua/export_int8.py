#!/usr/bin/env python3
"""R7 int8 exporter for Ukrainian (Tetiana or Mykyta) voices.

Reconstructs the three trained students (duration, acoustic, decoder) and
emits the exact byte layout the MCU runtime (mcu/src/snt_tts.c) consumes:

    front_q8.bin  -- duration + acoustic (int8 weights, fp32 biases/scales)
    model_q8.bin  -- LRC encoder + FSD decoder (int8 weights, fp32 dw/norm)
    front_q8_meta.h / fsd_q8_meta.h -- byte-offset headers
    e2e_ids.bin / e2e_durs.bin / e2e_audio.bin -- host golden fixture

The upstream repo ships the C runtime and the generated *meta* headers but not
the exporter scripts (export_front_q8.py / export_fsd_q8.py /
export_fsd_golden.py). This module reproduces the format from the runtime's
dequantization contract:

    out[o] = s_act * sc[o] * dot(q_act, w8[o]) + bias[o]
    w8[o] = round(W[o] / sc[o]),  sc[o] = max|W[o,:]| / 127

Convolutions store int8 weights as [out_ch, pad16(in_ch*K)] (gather order is
channel-major, kernel-minor). Embeddings and the decoder's depthwise/norm
tensors stay fp32. Per-block residual scales are fp32 scalars.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np
import torch

from acoustic_experiment import load_acoustic_module
from cook import WORK, digest, teacher_paths, upstream_path, write_json
from lrc_experiment import load_decoder_module

DUR_CKPT = "duration-100000-mps-v1/duration-student.pt"
AC_CKPT = "acoustic-150000-mps-v1/latent-student.pt"
DEC_CKPT = "decoder-200000-cpu-v1/seg-00019/decoder-student.pt"

AC_VOCAB = 256
DUR_VOCAB = 256
AC_HIDDEN = 48
AC_DEPTH = 5
AC_TOKEN_DEPTH = 3
AC_KERNEL = 5
AC_OUT = 40
DUR_HIDDEN = 32
DUR_DEPTH = 3
DUR_KERNEL = 5
DUR_MAX_DURATION = 80

FSD_IN_CH = 192
FSD_ENC_HIDDEN = 64
FSD_CODE_DIM = 40
FSD_DIM = 76
FSD_BLOCKS = 5
FSD_FILM_RANK = 12
FSD_HEAD_RANK = 48
FSD_N_FFT = 1024
FSD_BINS = 513
FSD_HOP = 256
FSD_DW_KERNEL = 7
FSD_PW_HIDDEN = 304


def pad16(n: int) -> int:
    return (n + 15) & ~15


def load_state(path: Path):
    from pathlib import PosixPath
    with torch.serialization.safe_globals([PosixPath]):
        return torch.load(path, map_location="cpu", weights_only=True)


def tensor_shapes() -> dict[str, dict[str, tuple[int, ...]]]:
    """Exact state-dict contract of the fixed scalar MCU implementation."""
    shapes: dict[str, dict[str, tuple[int, ...]]] = {
        name: {} for name in ("duration", "acoustic", "decoder", "encoder")}

    def conv(state, prefix, out_ch, in_ch, kernel=1):
        state[prefix + ".weight"] = (out_ch, in_ch, kernel)
        state[prefix + ".bias"] = (out_ch,)

    def blocks(state, prefix, depth, hidden, kernel):
        for i in range(depth):
            p = f"{prefix}.{i}"
            for layer in (0, 2):
                conv(state, f"{p}.net.{layer}", hidden, hidden, kernel)
            state[p + ".scale"] = ()

    d, a, m, e = (shapes[n] for n in shapes)
    d["embedding.weight"] = (DUR_VOCAB, DUR_HIDDEN)
    conv(d, "input_proj", DUR_HIDDEN, DUR_HIDDEN + 3)
    blocks(d, "blocks", DUR_DEPTH, DUR_HIDDEN, DUR_KERNEL)
    conv(d, "output", 1, DUR_HIDDEN)
    a["embedding.weight"] = (AC_VOCAB, AC_HIDDEN)
    conv(a, "token_input_proj", AC_HIDDEN, AC_HIDDEN + 2)
    blocks(a, "token_blocks", AC_TOKEN_DEPTH, AC_HIDDEN, AC_KERNEL)
    conv(a, "frame_input_proj", AC_HIDDEN, AC_HIDDEN + 3)
    blocks(a, "frame_blocks", AC_DEPTH, AC_HIDDEN, AC_KERNEL)
    conv(a, "output", AC_OUT, AC_HIDDEN)
    conv(e, "net.0", FSD_ENC_HIDDEN, FSD_IN_CH)
    conv(e, "net.2", FSD_CODE_DIM, FSD_ENC_HIDDEN)
    conv(m, "pre", FSD_DIM, FSD_CODE_DIM)
    for i in range(FSD_BLOCKS):
        p = f"fsd_blocks.{i}"
        conv(m, p + ".depthwise", FSD_DIM, 1, FSD_DW_KERNEL)
        m[p + ".norm.weight"] = m[p + ".norm.bias"] = (FSD_DIM,)
        conv(m, p + ".film_reduce", FSD_FILM_RANK, FSD_CODE_DIM)
        conv(m, p + ".film_out", 2 * FSD_DIM, FSD_FILM_RANK)
        conv(m, p + ".pointwise0", FSD_PW_HIDDEN, FSD_DIM)
        conv(m, p + ".pointwise1", FSD_DIM, FSD_PW_HIDDEN)
        m[p + ".scale"] = ()
    conv(m, "fsd_head_in", FSD_HEAD_RANK, FSD_DIM)
    conv(m, "fsd_head_out", 3 * FSD_BINS, FSD_HEAD_RANK)
    return shapes


def validate_checkpoints(dur: dict, ac: dict, dec: dict) -> int:
    """Reject incompatible configs, missing/extra layers and non-fp32-finite data."""
    configs = {
        "duration": dict(architecture="duration_conv", vocab_size=DUR_VOCAB,
                         hidden=DUR_HIDDEN, depth=DUR_DEPTH, kernel_size=DUR_KERNEL,
                         max_duration=DUR_MAX_DURATION),
        "acoustic": dict(architecture="token_context", vocab_size=AC_VOCAB,
                         hidden=AC_HIDDEN, depth=AC_DEPTH, token_depth=AC_TOKEN_DEPTH,
                         kernel_size=AC_KERNEL, out_channels=AC_OUT),
        "decoder": dict(variant="lrc", in_channels=FSD_CODE_DIM,
                        source_in_channels=FSD_IN_CH, lrc_code_dim=FSD_CODE_DIM,
                        lrc_encoder_hidden=FSD_ENC_HIDDEN, fsd_dim=FSD_DIM,
                        fsd_blocks=FSD_BLOCKS, fsd_film_rank=FSD_FILM_RANK,
                        fsd_head_rank=FSD_HEAD_RANK, istft_n_fft=FSD_N_FFT),
    }
    states = {}
    for name, checkpoint in (("duration", dur), ("acoustic", ac), ("decoder", dec)):
        if not isinstance(checkpoint, dict) or not isinstance(checkpoint.get("config"), dict):
            raise ValueError(f"{name}: missing checkpoint config")
        cfg = checkpoint["config"]
        for key, expected in configs[name].items():
            if cfg.get(key) != expected or isinstance(cfg.get(key), bool):
                raise ValueError(f"{name}: config {key}={cfg.get(key)!r}; MCU requires {expected!r}")
        states[name] = checkpoint.get("model_state_dict")
    for key in ("post_filter_channels", "post_filter_layers",
                "pre_tanh_repair_channels", "pre_tanh_repair_layers"):
        if dec["config"].get(key, 0) != 0:
            raise ValueError(f"decoder: MCU does not support {key}")
    max_tokens = dur["config"].get("max_tokens")
    if type(max_tokens) is not int or not 1 <= max_tokens <= 2147483647:
        raise ValueError("duration: max_tokens must be a positive int32")
    states["encoder"] = dec.get("lrc_encoder_state_dict")
    for name, expected in tensor_shapes().items():
        state = states[name]
        if not isinstance(state, dict):
            raise ValueError(f"{name}: missing state dict")
        missing, extra = expected.keys() - state.keys(), state.keys() - expected.keys()
        if missing or extra:
            raise ValueError(f"{name}: tensor keys mismatch; missing={sorted(missing)}, extra={sorted(extra)}")
        for key, shape in expected.items():
            value = state[key]
            if not isinstance(value, torch.Tensor) or tuple(value.shape) != shape:
                raise ValueError(f"{name}.{key}: expected tensor shape {shape}")
            if (not value.is_floating_point() or value.layout != torch.strided
                    or not torch.isfinite(value.detach().float()).all().item()):
                raise ValueError(f"{name}.{key}: expected finite float32-representable tensor")
    return max_tokens


def quantize_conv(weight: torch.Tensor, bias: torch.Tensor):
    """Quantize a Conv1d weight to per-output-channel int8.

    Returns (w8 [out, pad16(in*K)] int8, scale [out] f32, bias [out] f32).
    """
    w = weight.detach().float().numpy()
    out_ch, in_ch, k = w.shape
    flat = w.reshape(out_ch, in_ch * k)
    n16 = pad16(in_ch * k)
    amax = np.abs(flat).max(axis=1)
    scale = np.where(amax > 0, np.maximum(amax / 127.0, np.finfo(np.float32).tiny), 1.0).astype(np.float32)
    w8 = np.zeros((out_ch, n16), dtype=np.int8)
    q = np.rint(flat / scale[:, None]).astype(np.float32)
    np.clip(q, -127, 127, out=q)
    w8[:, : in_ch * k] = q.astype(np.int8)
    b = bias.detach().float().numpy().astype(np.float32)
    return w8, scale, b


class Blob:
    """Sequential little-endian byte writer with offset tracking."""

    def __init__(self) -> None:
        self.parts: list[bytes] = []

    def i8(self, arr: np.ndarray) -> int:
        off = self.size()
        self.parts.append(arr.astype(np.int8).tobytes())
        return off

    def f32(self, arr: np.ndarray) -> int:
        off = self.size()
        self.parts.append(np.ascontiguousarray(arr, dtype="<f4").tobytes())
        return off

    def size(self) -> int:
        return sum(len(p) for p in self.parts)

    def bytes(self) -> bytes:
        return b"".join(self.parts)


def emit_conv(blob: Blob, weight: torch.Tensor, bias: torch.Tensor):
    w8, scale, b = quantize_conv(weight, bias)
    off_w = blob.i8(w8)
    off_s = blob.f32(scale)
    off_b = blob.f32(b)
    return off_w, off_s, off_b, pad16(weight.shape[1] * weight.shape[2])


def build_front(dur: dict, ac: dict) -> tuple[bytes, dict]:
    blob = Blob()
    meta: dict = {}
    d = dur["model_state_dict"]
    a = ac["model_state_dict"]

    # ---- duration ----
    emb = d["embedding.weight"].detach().float().numpy()  # [256, 32]
    meta["DUR_EMB_F32"] = blob.f32(emb)

    meta["DUR_PROJ"] = emit_conv(blob, d["input_proj.weight"], d["input_proj.bias"])
    for i in range(DUR_DEPTH):
        c0 = emit_conv(blob, d[f"blocks.{i}.net.0.weight"], d[f"blocks.{i}.net.0.bias"])
        c1 = emit_conv(blob, d[f"blocks.{i}.net.2.weight"], d[f"blocks.{i}.net.2.bias"])
        scale = blob.f32(d[f"blocks.{i}.scale"].detach().float().numpy())
        meta[f"DUR_B{i}"] = [c0[0], c0[1], c0[2], c1[0], c1[1], c1[2], scale]
    meta["DUR_OUT"] = emit_conv(blob, d["output.weight"], d["output.bias"])

    # ---- acoustic ----
    aemb = a["embedding.weight"].detach().float().numpy()  # [256, 48]
    meta["AC_EMB_F32"] = blob.f32(aemb)

    meta["AC_TPROJ"] = emit_conv(blob, a["token_input_proj.weight"], a["token_input_proj.bias"])
    for i in range(AC_TOKEN_DEPTH):
        c0 = emit_conv(blob, a[f"token_blocks.{i}.net.0.weight"], a[f"token_blocks.{i}.net.0.bias"])
        c1 = emit_conv(blob, a[f"token_blocks.{i}.net.2.weight"], a[f"token_blocks.{i}.net.2.bias"])
        scale = blob.f32(a[f"token_blocks.{i}.scale"].detach().float().numpy())
        meta[f"AC_TB{i}"] = [c0[0], c0[1], c0[2], c1[0], c1[1], c1[2], scale]
    meta["AC_FPROJ"] = emit_conv(blob, a["frame_input_proj.weight"], a["frame_input_proj.bias"])
    for i in range(AC_DEPTH):
        c0 = emit_conv(blob, a[f"frame_blocks.{i}.net.0.weight"], a[f"frame_blocks.{i}.net.0.bias"])
        c1 = emit_conv(blob, a[f"frame_blocks.{i}.net.2.weight"], a[f"frame_blocks.{i}.net.2.bias"])
        scale = blob.f32(a[f"frame_blocks.{i}.scale"].detach().float().numpy())
        meta[f"AC_FB{i}"] = [c0[0], c0[1], c0[2], c1[0], c1[1], c1[2], scale]
    meta["AC_OUT"] = emit_conv(blob, a["output.weight"], a["output.bias"])

    return blob.bytes(), meta


def build_decoder(dec: dict) -> tuple[bytes, dict]:
    blob = Blob()
    meta: dict = {}
    m = dec["model_state_dict"]
    e = dec["lrc_encoder_state_dict"]

    # ---- LRC encoder ----
    meta["ENC0"] = emit_conv(blob, e["net.0.weight"], e["net.0.bias"])
    meta["ENC1"] = emit_conv(blob, e["net.2.weight"], e["net.2.bias"])

    # ---- pre ----
    meta["PRE"] = emit_conv(blob, m["pre.weight"], m["pre.bias"])

    # ---- blocks ----
    for i in range(FSD_BLOCKS):
        p = f"fsd_blocks.{i}"
        dw = m[f"{p}.depthwise.weight"].detach().float().numpy().reshape(FSD_DIM, FSD_DW_KERNEL)
        dwb = m[f"{p}.depthwise.bias"].detach().float().numpy()
        nw = m[f"{p}.norm.weight"].detach().float().numpy()
        nb = m[f"{p}.norm.bias"].detach().float().numpy()
        off_dw = blob.f32(dw)
        off_dwb = blob.f32(dwb)
        off_nw = blob.f32(nw)
        off_nb = blob.f32(nb)
        filmr = emit_conv(blob, m[f"{p}.film_reduce.weight"], m[f"{p}.film_reduce.bias"])
        filmo = emit_conv(blob, m[f"{p}.film_out.weight"], m[f"{p}.film_out.bias"])
        pw0 = emit_conv(blob, m[f"{p}.pointwise0.weight"], m[f"{p}.pointwise0.bias"])
        pw1 = emit_conv(blob, m[f"{p}.pointwise1.weight"], m[f"{p}.pointwise1.bias"])
        off_scale = blob.f32(m[f"{p}.scale"].detach().float().numpy())
        meta[f"B{i}"] = (
            off_dw, off_dwb, off_nw, off_nb,
            filmr[0], filmr[1], filmr[2],
            filmo[0], filmo[1], filmo[2],
            pw0[0], pw0[1], pw0[2],
            pw1[0], pw1[1], pw1[2],
            off_scale,
        )

    # ---- head ----
    meta["HEAD_IN"] = emit_conv(blob, m["fsd_head_in.weight"], m["fsd_head_in.bias"])
    meta["HEAD_OUT"] = emit_conv(blob, m["fsd_head_out.weight"], m["fsd_head_out.bias"])

    return blob.bytes(), meta


def write_front_header(meta: dict, path: Path, total_bytes: int, max_tokens: int) -> None:
    def n16(in_ch, k):
        return pad16(in_ch * k)

    lines = ["/* generated by export_int8.py -- byte offsets into front_q8.bin */", "#pragma once", ""]
    lines += [
        f"#define FRONT_AC_VOCAB {AC_VOCAB}",
        f"#define FRONT_AC_HIDDEN {AC_HIDDEN}",
        f"#define FRONT_AC_DEPTH {AC_DEPTH}",
        f"#define FRONT_AC_TOKEN_DEPTH {AC_TOKEN_DEPTH}",
        f"#define FRONT_AC_KERNEL {AC_KERNEL}",
        f"#define FRONT_AC_OUT {AC_OUT}",
        f"#define FRONT_DUR_VOCAB {DUR_VOCAB}",
        f"#define FRONT_DUR_HIDDEN {DUR_HIDDEN}",
        f"#define FRONT_DUR_DEPTH {DUR_DEPTH}",
        f"#define FRONT_DUR_KERNEL {DUR_KERNEL}",
        f"#define FRONT_DUR_MAX_TOKENS {max_tokens}",
        f"#define FRONT_DUR_MAX_DURATION {DUR_MAX_DURATION}",
        f"#define FRONT_DUR_PROJ_N16 {n16(DUR_HIDDEN + 3, 1)}",
        f"#define FRONT_DUR_B0_C0_N16 {n16(DUR_HIDDEN, DUR_KERNEL)}",
        f"#define FRONT_DUR_B0_C1_N16 {n16(DUR_HIDDEN, DUR_KERNEL)}",
        f"#define FRONT_DUR_B1_C0_N16 {n16(DUR_HIDDEN, DUR_KERNEL)}",
        f"#define FRONT_DUR_B1_C1_N16 {n16(DUR_HIDDEN, DUR_KERNEL)}",
        f"#define FRONT_DUR_B2_C0_N16 {n16(DUR_HIDDEN, DUR_KERNEL)}",
        f"#define FRONT_DUR_B2_C1_N16 {n16(DUR_HIDDEN, DUR_KERNEL)}",
        f"#define FRONT_DUR_OUT_N16 {n16(DUR_HIDDEN, 1)}",
        f"#define FRONT_AC_TPROJ_N16 {n16(AC_HIDDEN + 2, 1)}",
        f"#define FRONT_AC_TB0_C0_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_TB0_C1_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_TB1_C0_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_TB1_C1_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_TB2_C0_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_TB2_C1_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_FPROJ_N16 {n16(AC_HIDDEN + 3, 1)}",
        f"#define FRONT_AC_FB0_C0_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_FB0_C1_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_FB1_C0_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_FB1_C1_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_FB2_C0_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_FB2_C1_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_FB3_C0_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_FB3_C1_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_FB4_C0_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_FB4_C1_N16 {n16(AC_HIDDEN, AC_KERNEL)}",
        f"#define FRONT_AC_OUT_N16 {n16(AC_HIDDEN, 1)}",
        "",
    ]

    def conv_lines(prefix, tup):
        w, s, b, _n16 = tup
        return [f"#define {prefix}_W8 {w}", f"#define {prefix}_SCALE {s}", f"#define {prefix}_BIAS {b}"]

    lines.append(f"#define FOFF_DUR_EMB_F32 {meta['DUR_EMB_F32']}")
    lines += conv_lines("FOFF_DUR_PROJ", meta["DUR_PROJ"])
    for i in range(DUR_DEPTH):
        t = meta[f"DUR_B{i}"]
        lines += conv_lines(f"FOFF_DUR_B{i}_C0", (t[0], t[1], t[2], 0))
        lines += conv_lines(f"FOFF_DUR_B{i}_C1", (t[3], t[4], t[5], 0))
        lines.append(f"#define FOFF_DUR_B{i}_SCALE_F32 {t[6]}")
    lines += conv_lines("FOFF_DUR_OUT", meta["DUR_OUT"])
    lines.append(f"#define FOFF_AC_EMB_F32 {meta['AC_EMB_F32']}")
    lines += conv_lines("FOFF_AC_TPROJ", meta["AC_TPROJ"])
    for i in range(AC_TOKEN_DEPTH):
        t = meta[f"AC_TB{i}"]
        lines += conv_lines(f"FOFF_AC_TB{i}_C0", (t[0], t[1], t[2], 0))
        lines += conv_lines(f"FOFF_AC_TB{i}_C1", (t[3], t[4], t[5], 0))
        lines.append(f"#define FOFF_AC_TB{i}_SCALE_F32 {t[6]}")
    lines += conv_lines("FOFF_AC_FPROJ", meta["AC_FPROJ"])
    for i in range(AC_DEPTH):
        t = meta[f"AC_FB{i}"]
        lines += conv_lines(f"FOFF_AC_FB{i}_C0", (t[0], t[1], t[2], 0))
        lines += conv_lines(f"FOFF_AC_FB{i}_C1", (t[3], t[4], t[5], 0))
        lines.append(f"#define FOFF_AC_FB{i}_SCALE_F32 {t[6]}")
    lines += conv_lines("FOFF_AC_OUT", meta["AC_OUT"])
    lines += ["", f"#define FRONT_MODEL_BYTES {total_bytes}", ""]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_fsd_header(meta: dict, path: Path, total_bytes: int) -> None:
    lines = ["/* generated by export_int8.py -- byte offsets into model_q8.bin */", "#pragma once", ""]
    n16 = {
        "ENC0": pad16(FSD_IN_CH),
        "ENC1": pad16(FSD_ENC_HIDDEN),
        "PRE": pad16(FSD_CODE_DIM),
        "FILMR": pad16(FSD_CODE_DIM),
        "FILMO": pad16(FSD_FILM_RANK),
        "PW0": pad16(FSD_DIM),
        "PW1": pad16(FSD_PW_HIDDEN),
        "HEAD_IN": pad16(FSD_DIM),
        "HEAD_OUT": pad16(FSD_HEAD_RANK),
    }
    lines += [
        f"#define Q8_ENC0_N16 {n16['ENC0']}",
        f"#define Q8_ENC1_N16 {n16['ENC1']}",
        f"#define Q8_PRE_N16 {n16['PRE']}",
    ]
    for i in range(FSD_BLOCKS):
        lines += [
            f"#define Q8_B{i}_FILMR_N16 {n16['FILMR']}",
            f"#define Q8_B{i}_FILMO_N16 {n16['FILMO']}",
            f"#define Q8_B{i}_PW0_N16 {n16['PW0']}",
            f"#define Q8_B{i}_PW1_N16 {n16['PW1']}",
        ]
    lines += [
        f"#define Q8_HEAD_IN_N16 {n16['HEAD_IN']}",
        f"#define Q8_HEAD_OUT_N16 {n16['HEAD_OUT']}",
        "",
    ]

    def c(prefix, tup):
        w, s, b, _ = tup
        return [f"#define {prefix}_W8 {w}", f"#define {prefix}_SCALE {s}", f"#define {prefix}_BIAS {b}"]

    lines += c("Q8OFF_ENC0", meta["ENC0"])
    lines += c("Q8OFF_ENC1", meta["ENC1"])
    lines += c("Q8OFF_PRE", meta["PRE"])
    for i in range(FSD_BLOCKS):
        t = meta[f"B{i}"]
        lines += [
            f"#define Q8OFF_B{i}_DW_W_F32 {t[0]}",
            f"#define Q8OFF_B{i}_DW_B_F32 {t[1]}",
            f"#define Q8OFF_B{i}_NORM_W_F32 {t[2]}",
            f"#define Q8OFF_B{i}_NORM_B_F32 {t[3]}",
            f"#define Q8OFF_B{i}_FILMR_W8 {t[4]}",
            f"#define Q8OFF_B{i}_FILMR_SCALE {t[5]}",
            f"#define Q8OFF_B{i}_FILMR_BIAS {t[6]}",
            f"#define Q8OFF_B{i}_FILMO_W8 {t[7]}",
            f"#define Q8OFF_B{i}_FILMO_SCALE {t[8]}",
            f"#define Q8OFF_B{i}_FILMO_BIAS {t[9]}",
            f"#define Q8OFF_B{i}_PW0_W8 {t[10]}",
            f"#define Q8OFF_B{i}_PW0_SCALE {t[11]}",
            f"#define Q8OFF_B{i}_PW0_BIAS {t[12]}",
            f"#define Q8OFF_B{i}_PW1_W8 {t[13]}",
            f"#define Q8OFF_B{i}_PW1_SCALE {t[14]}",
            f"#define Q8OFF_B{i}_PW1_BIAS {t[15]}",
            f"#define Q8OFF_B{i}_SCALE_F32 {t[16]}",
        ]
    lines += c("Q8OFF_HEAD_IN", meta["HEAD_IN"])
    lines += c("Q8OFF_HEAD_OUT", meta["HEAD_OUT"])
    lines += ["", f"#define Q8_MODEL_BYTES {total_bytes}", ""]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_aux_headers(out: Path) -> None:
    """Write the remaining model headers the runtime includes.

    fsd_meta.h carries the shared architecture dimensions. The frozen-norm,
    integer-chain and residual-bank scale headers are only consumed under
    FSD_FROZEN_NORM / SNT_INT_CHAIN, which the firmware does not define (the
    scalar float path computes per-utterance GroupNorm), so they are empty
    here. The float path is bit-consistent with the PyTorch float model.
    """
    model = out / "model"
    model.mkdir(exist_ok=True)
    (model / "fsd_meta.h").write_text(
        "/* generated by export_int8.py -- shared R7 architecture dimensions */\n"
        "#pragma once\n\n"
        f"#define FSD_IN_CH {FSD_IN_CH}\n"
        f"#define FSD_ENC_HIDDEN {FSD_ENC_HIDDEN}\n"
        f"#define FSD_CODE_DIM {FSD_CODE_DIM}\n"
        f"#define FSD_DIM {FSD_DIM}\n"
        f"#define FSD_BLOCKS {FSD_BLOCKS}\n"
        f"#define FSD_FILM_RANK {FSD_FILM_RANK}\n"
        f"#define FSD_HEAD_RANK {FSD_HEAD_RANK}\n"
        f"#define FSD_N_FFT {FSD_N_FFT}\n"
        f"#define FSD_BINS {FSD_BINS}\n"
        f"#define FSD_HOP {FSD_HOP}\n"
        f"#define FSD_DW_KERNEL {FSD_DW_KERNEL}\n"
        f"#define FSD_PW_HIDDEN {FSD_PW_HIDDEN}\n", encoding="utf-8")
    for name in ("frozen_norm.h", "chain_scales.h", "rb_scales.h", "act_scales.h"):
        (model / name).write_text(
            "/* generated by export_int8.py -- unused in the scalar float path */\n"
            "#pragma once\n", encoding="utf-8")


def build_golden(upstream: Path, out: Path, ac: dict, dec: dict, pack: Path) -> None:
    """Render selected acoustic/decoder states with selected pack's oracle durations."""
    import inspect
    am = load_acoustic_module(upstream)
    dm = load_decoder_module(upstream)
    model = am.create_model_from_config(ac["config"])
    model.load_state_dict(ac["model_state_dict"], strict=True)
    model.eval()
    sig = inspect.signature(dm.DecoderStudent)
    kwargs = {k: v for k, v in dec["config"].items() if k in sig.parameters}
    decoder = dm.DecoderStudent(**kwargs)
    decoder.load_state_dict(dec["model_state_dict"], strict=True)
    decoder.eval()

    rows = json.loads((pack / "rows.json").read_text())
    row = rows[0]
    chunk = row["chunks"][0]
    tensor_path = Path(chunk["tensor_npz"])
    if not tensor_path.is_absolute():
        tensor_path = pack / tensor_path
    tensor_sha = digest(tensor_path)
    with np.load(tensor_path, allow_pickle=False) as data:
        ids = data["phoneme_ids"].reshape(-1)
        durations = data["w_ceil"].reshape(-1)
    for name, values, limit in (("ids", ids, AC_VOCAB - 1),
                                ("durations", durations, np.iinfo(np.int32).max)):
        if (not values.size or not np.isfinite(values).all() or np.any(values < 0)
                or np.any(values > limit) or np.any(values != np.floor(values))):
            raise ValueError(f"Invalid golden {name}")
    if ids.size != durations.size:
        raise ValueError("Golden IDs/durations length mismatch")
    ids, durations = ids.astype(np.int64), durations.astype(np.int64)
    frames = int(durations.sum())
    if frames <= 0:
        raise ValueError("Empty golden audio")
    sample = am.ChunkSample(
        row_id=row["row_id"], row_index=0, text=row["text"], chunk_index=chunk["chunk_index"],
        phoneme_ids=ids, durations=durations,
        target=np.zeros((frames, AC_OUT), dtype=np.float32),
        tensor_path=tensor_path, audio_samples=frames * FSD_HOP)
    with torch.inference_mode():
        code = am.predict_chunk(model, sample, torch.device("cpu"))
        if code.shape != (1, FSD_CODE_DIM, frames) or not np.isfinite(code).all():
            raise ValueError("Invalid golden acoustic code")
        audio = decoder(torch.from_numpy(code)).reshape(-1).numpy()
    if audio.size != frames * FSD_HOP or not np.isfinite(audio).all():
        raise ValueError("Invalid golden audio")
    if digest(tensor_path) != tensor_sha:
        raise ValueError("Golden tensor changed during export")
    (out / "e2e_ids.bin").write_bytes(ids.astype("<i4").tobytes())
    (out / "e2e_durs.bin").write_bytes(durations.astype("<i4").tobytes())
    (out / "e2e_audio.bin").write_bytes(audio.astype("<f4").tobytes())
    write_json(out / "golden_meta.json", {
        "row_id": row["row_id"], "text": row["text"], "n_tokens": int(ids.size),
        "frames": frames, "samples": int(audio.size),
        "durations": "teacher/oracle", "eval_pack": str(pack),
        "tensor_path": str(tensor_path.resolve()), "tensor_sha256": tensor_sha,
    })


def select_eval_pack(explicit: Path | None, speaker: str, *checkpoints: dict) -> Path:
    if explicit is not None:
        return explicit.expanduser().resolve()
    for checkpoint in checkpoints:
        value = checkpoint.get("train_args", {}).get("eval_pack_dir")
        if value:
            return Path(value).expanduser().resolve()
    legacy = WORK / speaker / "duration-packs-full-v1" / "pack-eval"
    return (legacy if legacy.is_dir() else WORK / speaker / "pack-eval").resolve()


def validate_voice(speaker: str, paths: list[Path], *checkpoints: dict) -> None:
    """Catch known opposite-voice paths without forbidding relocated checkpoints."""
    other = "tetiana" if speaker == "mykyta" else "mykyta"
    other_root = (WORK / other).resolve()
    for checkpoint in checkpoints:
        for key in ("pack_dir", "eval_pack_dir"):
            value = checkpoint.get("train_args", {}).get(key)
            if value:
                paths = [*paths, Path(value)]
    for path in paths:
        if path.resolve().is_relative_to(other_root):
            raise ValueError(f"Selected {speaker} voice references {other}: {path}")


def publish_export(staging: Path, out: Path) -> None:
    """Atomic directory rename with kernel-enforced no-replace, including races.

    Plain Path.rename can overwrite an empty directory on POSIX. Use the
    platform's exclusive rename instead; fail closed on unsupported platforms.
    """
    libc = ctypes.CDLL(None, use_errno=True)
    if sys.platform == "darwin":
        rename = libc.renamex_np
        rename.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint]
        arguments = (os.fsencode(staging), os.fsencode(out), 0x4)  # RENAME_EXCL
    elif sys.platform.startswith("linux"):
        rename = libc.renameat2
        rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int,
                           ctypes.c_char_p, ctypes.c_uint]
        arguments = (-100, os.fsencode(staging), -100, os.fsencode(out), 1)  # RENAME_NOREPLACE
    else:
        raise RuntimeError("Atomic no-replace export requires macOS or Linux")
    rename.restype = ctypes.c_int
    if rename(*arguments) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error), str(out))


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--voice", choices=("male", "female"), default="female")
    parser.add_argument("--out", type=Path, default=None)
    for name, default in (("duration", DUR_CKPT), ("acoustic", AC_CKPT), ("decoder", DEC_CKPT)):
        parser.add_argument(f"--{name}-ckpt", type=Path,
                            help=f"Checkpoint path (relative to cwd); default: work/<speaker>/{default}")
    parser.add_argument("--eval-pack", type=Path,
                        help="Golden pack; default: selected checkpoint eval_pack_dir, then speaker pack")
    parser.add_argument("--upstream")
    parser.add_argument("--skip-golden", action="store_true")
    args = parser.parse_args(argv)
    speaker, _, _ = teacher_paths(args.voice)
    # Do not resolve the final component: even a dangling output symlink is occupied.
    out = Path(os.path.abspath((args.out or WORK / speaker / "export-uk").expanduser()))
    if os.path.lexists(out):
        raise FileExistsError(f"Refusing to overwrite: {out}")
    paths = {
        name: (value or WORK / speaker / default).expanduser().resolve()
        for name, value, default in (("duration", args.duration_ckpt, DUR_CKPT),
                                     ("acoustic", args.acoustic_ckpt, AC_CKPT),
                                     ("decoder", args.decoder_ckpt, DEC_CKPT))}
    hashes = {name: digest(path) for name, path in paths.items()}
    dur, ac, dec = (load_state(paths[name]) for name in ("duration", "acoustic", "decoder"))
    max_tokens = validate_checkpoints(dur, ac, dec)
    pack = None if args.skip_golden else select_eval_pack(args.eval_pack, speaker, ac, dec, dur)
    validate_voice(speaker, [*paths.values(), *([pack] if pack is not None else [])], dur, ac, dec)
    pack_sha = digest(pack / "rows.json") if pack is not None else None
    upstream = upstream_path(args.upstream) if pack is not None else None

    front_bytes, front_meta = build_front(dur, ac)
    model_bytes, model_meta = build_decoder(dec)
    out.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{out.name}.tmp-", dir=out.parent))
    try:
        (staging / "front_q8.bin").write_bytes(front_bytes)
        (staging / "model_q8.bin").write_bytes(model_bytes)
        write_aux_headers(staging)
        write_front_header(front_meta, staging / "model/front_q8_meta.h", len(front_bytes), max_tokens)
        write_fsd_header(model_meta, staging / "model/fsd_q8_meta.h", len(model_bytes))
        if pack is not None and upstream is not None:
            build_golden(upstream, staging, ac, dec, pack)
        if any(digest(paths[name]) != sha for name, sha in hashes.items()):
            raise ValueError("Checkpoint changed during export; retry with immutable inputs")
        if pack is not None and digest(pack / "rows.json") != pack_sha:
            raise ValueError("Evaluation pack changed during export")
        report = {
            "speaker": speaker, "front_bytes": len(front_bytes), "model_bytes": len(model_bytes),
            "duration_max_tokens": max_tokens,
            "eval_pack": str(pack) if pack is not None else None, "eval_rows_sha256": pack_sha,
            "golden": "teacher/oracle durations" if pack is not None else "skipped",
        }
        for name in paths:
            report[f"{name}_checkpoint"] = str(paths[name])
            report[f"{name}_sha256"] = hashes[name]
        write_json(staging / "export_report.json", report)
        write_json(staging / "export_manifest.json", {
            "format_version": 1, "speaker": speaker,
            "exporter_sha256": digest(Path(__file__)),
            "checkpoints": {name: {"path": str(paths[name]), "sha256": hashes[name],
                                    "config": state["config"]}
                            for name, state in (("duration", dur), ("acoustic", ac), ("decoder", dec))},
            "artifacts": {str(p.relative_to(staging)): {"bytes": p.stat().st_size, "sha256": digest(p)}
                          for p in sorted(staging.rglob("*")) if p.is_file()},
        })
        publish_export(staging, out)
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    print(f"Exported to {out}: front {len(front_bytes)}B, model {len(model_bytes)}B")


if __name__ == "__main__":
    main()
