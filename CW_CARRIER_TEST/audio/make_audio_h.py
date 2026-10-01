#!/usr/bin/env python3
"""Konwertuje plik audio na naglowki dla CW_CARRIER_TEST (odtwarzanie z flasha).
Na zywo przez USB zamiast z flasha: stream_ssb.py.

Uzycie: python3 make_audio_h.py plik.ogg [--raw] [--comp light|soft|mid|hard]
                                [--wav KATALOG] [--dump-z PLIK.npy]
        --raw     bez procesora mowy (EQ/kompresja) - do porownan A/B
        --comp    sila kompresji (domyslnie light)
        --wav     zapisuje ssb_przed.wav / ssb_po.wav (16 kHz) do odsluchu
        python3 make_audio_h.py tones:700:1900   (test dwutonowy, 3.5 s)
        --phase-only / --env-only                (diagnostyka torow)

../audio_data.h - FM/AM: 8 kHz int8, filtr 300..3400 Hz, bez DC,
                  normalizacja z lekkim miekkim ograniczeniem.
../ssb_data.h   - USB/LSB z ssb_dsp.py (procesor mowy, metoda polarna).
"""
import sys
from pathlib import Path

import numpy as np

import ssb_dsp as d

src = sys.argv[1]
here = Path(__file__).resolve().parent.parent


def arg(name):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else None


def c_array(ctype, name, length, values):
    rows = [", ".join(str(v) for v in values[i:i + 20])
            for i in range(0, len(values), 20)]
    return (f"static const {ctype} {name}[{length}] PROGMEM = {{\n  "
            + ",\n  ".join(rows) + "\n};\n")


def header(body):
    return (f"// wygenerowane przez audio/make_audio_h.py z: {Path(src).name}\n"
            f"#pragma once\n\n" + body)


speech = not src.startswith("tones:")


def load(rate, extra_af=""):
    return d.load_audio(src, rate, extra_af)


# ---------------------------------------------------------------- FM/AM
RATE = 8000
x = load(RATE, "highpass=f=300,lowpass=f=3400,")
x /= np.percentile(np.abs(x), 99.9)
x = np.tanh(1.5 * x) / np.tanh(1.5)
q = np.clip(np.round(x * 127), -127, 127).astype(np.int8)
(here / "audio_data.h").write_text(header(
    f"static constexpr uint32_t AUDIO_RATE = {RATE};\n"
    f"static constexpr uint32_t AUDIO_LEN  = {len(q)};  // {len(q) / RATE:.2f} s\n\n"
    + c_array("int8_t", "AUDIO", "AUDIO_LEN", q)))
print(f"audio_data.h: {len(q)} probek, {len(q) / RATE:.2f} s")

# ---------------------------------------------------------------- SSB
fs = d.SSB_RATE
z, x_raw, x_out = d.analytic(load(fs), fs, arg("--comp") or "light",
                             raw="--raw" in sys.argv or not speech)
if speech and "--raw" not in sys.argv:
    d.band_stats("ssb: mowa przed", x_raw, fs)
    d.band_stats("ssb: mowa po   ", x_out, fs)
if arg("--wav"):
    d.write_wav(Path(arg("--wav")) / "ssb_przed.wav", x_raw[::2], fs // 2)
    d.write_wav(Path(arg("--wav")) / "ssb_po.wav", x_out[::2], fs // 2)
if arg("--dump-z"):
    np.save(arg("--dump-z"), z)
env_q8, freq_q16 = d.polar(z, d.ask_curve(), fs, phase_only="--phase-only" in sys.argv,
                           env_only="--env-only" in sys.argv)

n = len(env_q8)
(here / "ssb_data.h").write_text(header(
    f"static constexpr uint32_t SSB_RATE = {fs};\n"
    f"static constexpr uint32_t SSB_LEN  = {n};  // {n / fs:.2f} s\n\n"
    f"// ASK w Q8: 0 = szczyt, 127*256 = podloga ~-31 dB, 128*256 = bramka OFF\n"
    + c_array("uint16_t", "SSB_ENV", "SSB_LEN", env_q8) + "\n"
    f"// chwilowa czestotliwosc USB jako offset K w Q16 (78.125 kHz/kod)\n"
    + c_array("int16_t", "SSB_FREQ", "SSB_LEN", freq_q16)))
print(f"ssb_data.h: {n} probek, {n / fs:.2f} s, {4 * n / 1024:.0f} KB we flashu")
