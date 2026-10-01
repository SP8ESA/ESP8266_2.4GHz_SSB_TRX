#!/usr/bin/env python3
"""Sygnal testowy SSB z HackRF (np. do odbioru przez esp_sdr.py).

  python3 hackrf_ssb_tx.py                      # sp8esa.mp3, USB 2400.250 MHz, w petli
  python3 hackrf_ssb_tx.py --f 2414.5 --lsb --src osr_kobieta.wav --vga 10

Audio przez ten sam procesor mowy co nadajnik ESP (ssb_dsp.analytic), potem
sygnal analityczny 32 kS/s -> 2 MS/s i przesuniecie o --offset nad srodek
HackRF, zeby jego wlasny przeciek LO (prazek na srodku) lezal daleko od
sygnalu. Plik IQ int8 nadawany w petli (hackrf_transfer -R), --gap s ciszy
miedzy powtorzeniami. SDR++ musi byc zamkniety (albo zatrzymany).
"""
import argparse
import os
import subprocess
import tempfile
from pathlib import Path

import numpy as np
from scipy.signal import resample_poly

import ssb_dsp

FS = 2_000_000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--f", type=float, default=2400.250, help="nosna (wytlumiona) SSB w MHz")
    ap.add_argument("--lsb", action="store_true")
    ap.add_argument("--src", default=str(Path(__file__).with_name("sp8esa.mp3")))
    ap.add_argument("--gap", type=float, default=1.0, help="cisza miedzy powtorzeniami [s]")
    ap.add_argument("--offset", type=float, default=600e3, help="sygnal nad srodkiem HackRF [Hz]")
    ap.add_argument("--vga", type=int, default=0, help="TX VGA HackRF 0..47 dB")
    ap.add_argument("--comp", default="light", choices=list(ssb_dsp.COMP))
    a = ap.parse_args()

    x = ssb_dsp.load_audio(a.src, ssb_dsp.SSB_RATE)
    z, _, _ = ssb_dsp.analytic(x, ssb_dsp.SSB_RATE, comp=a.comp)
    if a.lsb:
        z = np.conj(z)
    z = np.concatenate((z, np.zeros(int(a.gap * ssb_dsp.SSB_RATE))))
    z = resample_poly(z, FS // 1000, ssb_dsp.SSB_RATE // 1000)
    # cala liczba okresow przesuniecia w pliku -> petla bez skoku fazy
    per = FS // np.gcd(FS, int(a.offset))
    z = np.concatenate((z, np.zeros(-len(z) % per)))
    z *= np.exp(2j * np.pi * a.offset / FS * np.arange(len(z)))
    z *= 100 / max(np.abs(z.real).max(), np.abs(z.imag).max())
    iq = np.empty(2 * len(z), np.int8)
    iq[0::2] = np.round(z.real)
    iq[1::2] = np.round(z.imag)
    path = Path(tempfile.gettempdir()) / "hackrf_ssb_tx.iq"
    iq.tofile(path)

    fc = int(round(a.f * 1e6 - a.offset))
    print(f"{'LSB' if a.lsb else 'USB'} {a.f:.6f} MHz, {len(z) / FS:.2f} s w petli, "
          f"HackRF srodek {fc / 1e6:.6f} MHz, VGA {a.vga} dB (Ctrl+C konczy)", flush=True)
    try:
        subprocess.run(["hackrf_transfer", "-t", str(path), "-f", str(fc), "-s", str(FS),
                        "-x", str(a.vga), "-a", "0", "-R"], check=False,
                       stdout=subprocess.DEVNULL)
    except KeyboardInterrupt:
        pass
    finally:
        os.unlink(path)


if __name__ == "__main__":
    main()
