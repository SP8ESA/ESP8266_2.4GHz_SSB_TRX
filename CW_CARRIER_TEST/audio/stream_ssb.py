#!/usr/bin/env python3
"""SSB na ESP8266 z probek liczonych na PC i pchanych przez USB (tryb STREAM
w CW_CARRIER_TEST.ino) - bez limitu dlugosci i bez flashowania.

Uzycie: python3 stream_ssb.py plik.wav [plik2 ...] [--freq 2402] [--lsb]
        zrodla specjalne (z --gap 0 petla bez szwow):
          sine:F[:S]   ton F Hz przez tor DSP (S sekund, domyslnie 1)
          cw:F[:S]     stala czestotliwosc F Hz jako sam dithering K, bez DSP
          silence:S    cisza (bramka OFF)
            [--comp soft|mid|hard] [--raw] [--gap 1.0] [--seconds N]
            [--port /dev/ttyUSB0]

Pliki graja w petli (z przerwa --gap s) do Ctrl-C albo --seconds. DSP jak
dla flasha (ssb_dsp.py): procesor mowy, sygnal analityczny, obwiednia i
czestotliwosc z predystorsja z ask_cal.json.

Protokol: ESP po "STREAM f" odpowiada "OK STREAM 2000000" i przechodzi na
2 Mbaud. Pakiet: A5 5A, 64 x (obwiednia u8: ASK w 0.5 kroku, 255 = bramka
OFF; czestotliwosc i16 LE), XOR 192 bajtow. ESP co 64 probki odsyla B7 +
stan bufora/16 (bufor 4096 probek); utrzymujemy ~50%.
"""
import argparse
import time

import numpy as np
import serial

import ssb_dsp as d

BAUD, BLOCK, BUF = 2000000, 64, 4096


def stream_bytes(env_q8, freq_q16, lsb):
    e8 = np.where(env_q8 >= 128 * 256, 255,
                  np.clip(np.round(env_q8 / 128), 0, 254)).astype(np.uint8)
    f = (-freq_q16.astype(np.int32) if lsb else freq_q16.astype(np.int32)).astype("<i2")
    n = -len(e8) % BLOCK                      # dopelnienie cisza do pelnego pakietu
    e8 = np.concatenate((e8, np.full(n, 255, np.uint8)))
    f = np.concatenate((f, np.zeros(n, "<i2")))
    fb = f.view(np.uint8).reshape(-1, 2)
    payload = np.column_stack((e8, fb[:, 0], fb[:, 1])).reshape(-1, BLOCK * 3)
    out = []
    for p in payload:
        out.append(b"\xA5\x5A" + p.tobytes() + bytes([np.bitwise_xor.reduce(p)]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--freq", default="2402")
    ap.add_argument("--lsb", action="store_true")
    ap.add_argument("--comp", default="mid", choices=list(d.COMP))
    ap.add_argument("--raw", action="store_true")
    ap.add_argument("--gap", type=float, default=1.0)
    ap.add_argument("--seconds", type=float, default=0)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    a = ap.parse_args()

    cal = d.load_cal()
    blocks = []
    gap = int(a.gap * d.SSB_RATE)
    for fn in a.files:
        kind, *par = fn.split(":")
        sec = float(par[1]) if len(par) > 1 else 1.0
        n = int(round(sec * d.SSB_RATE))
        if kind == "cw":         # sam dithering K: stala czestotliwosc, bez DSP
            env_q8 = np.zeros(n, np.uint16)
            freq_q16 = np.full(n, round(float(par[0]) * 65536 / d.HZ_PER_CODE), np.int16)
        elif kind == "silence":  # bramka OFF
            n = int(round(float(par[0]) * d.SSB_RATE))
            env_q8 = np.full(n, 128 * 256, np.uint16)
            freq_q16 = np.zeros(n, np.int16)
        else:
            if kind == "sine":   # czysty ton przez tor DSP (kalibracja, AM-PM/AM-FM)
                t = np.arange(n + 1) / d.SSB_RATE
                z = np.exp(2j * np.pi * round(float(par[0])) * t)
            else:
                z, _, _ = d.analytic(d.load_audio(fn, d.SSB_RATE), d.SSB_RATE, a.comp, a.raw)
            env_q8, freq_q16 = d.polar(z, cal)
        env_q8 = np.concatenate((env_q8, np.full(gap, 128 * 256, np.uint16)))
        freq_q16 = np.concatenate((freq_q16, np.zeros(gap, np.int16)))
        blocks += stream_bytes(env_q8, freq_q16, a.lsb)
        print(f"{fn}: {len(env_q8) / d.SSB_RATE:.2f} s")
    period = len(blocks) * BLOCK / d.SSB_RATE
    print(f"petla {period:.2f} s, {len(blocks)} pakietow, "
          f"{'LSB' if a.lsb else 'USB'} {a.freq} MHz")

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = a.port, 115200, 0.05
    s.dtr = s.rts = False
    s.open()                                   # reset ESP
    time.sleep(2.5)
    s.read(65536)
    s.write(f"STREAM {a.freq}\r\n".encode())
    buf, t0 = b"", time.time()
    while b"OK STREAM" not in buf and time.time() - t0 < 6:
        buf += s.read(4096)
    if b"OK STREAM" not in buf:
        raise SystemExit(f"ESP nie wszedl w STREAM: {buf[-200:]!r}")
    time.sleep(0.03)
    s.baudrate = BAUD
    time.sleep(0.05)
    s.reset_input_buffer()

    target = BUF // 2
    fill, sent_since, i, state = 0, 0, 0, 0
    fmin, fmax, t_rep, sent_total = BUF, 0, time.time(), 0
    t_start = time.time()
    try:
        while not a.seconds or time.time() - t_start < a.seconds:
            for b in s.read(s.in_waiting or 1):
                if state == 0:
                    state = b == 0xB7
                else:
                    fill, sent_since, state = b * 16, 0, 0
                    fmin, fmax = min(fmin, fill), max(fmax, fill)
            chunk = []
            while fill + sent_since < target:
                chunk.append(blocks[i])
                i = (i + 1) % len(blocks)
                sent_since += BLOCK
            if chunk:
                s.write(b"".join(chunk))
                sent_total += len(chunk)
            if time.time() - t_rep > 2:
                print(f"  bufor ESP {fmin}..{fmax}/{BUF} probek, wyslano {sent_total * BLOCK / d.SSB_RATE:.1f} s audio")
                fmin, fmax, t_rep = BUF, 0, time.time()
    except KeyboardInterrupt:
        pass
    s.baudrate = 115200                        # ESP wraca na 115200 po 0.5 s bez danych
    buf, t0 = b"", time.time()
    while b"STREAM END" not in buf and time.time() - t0 < 3:
        buf += s.read(4096)
    line = [l for l in buf.decode(errors="replace").splitlines() if "STREAM END" in l]
    print(line[0] if line else "brak podsumowania z ESP")
    s.close()


if __name__ == "__main__":
    main()
