#!/usr/bin/env python3
"""ESP8266 jako odbiornik IQ: estymator IQ odbiornika WiFi (komendy IQ i
IQSTREAM w CW_CARRIER_TEST.ino).

Estymator sumuje n+1 probek ADC odbiornika (40 MS/s) - srednia zespolona okna
to waskopasmowa probka IQ wokol srodka kanalu WiFi (2407 + 5*ch MHz).

  python3 rx_iq.py --ch 1 --k 10 --seconds 10 --out zrzut.wav
      ciagly zrzut przez UART 2 Mbaud: okno 2^k probek ADC, fs ~ 1/(2^k*25 ns
      + 1.8 us) (k=10: ~36 kS/s, wiecej nie przejdzie przez 2 Mbaud). CH340
      gubi pojedyncze bajty przy duzym ruchu: uszkodzone paczki (16 probek)
      sa zerowane w miejscu, czas i faza zostaja ciagle.
      .wav = IQ stereo int16 (SDR++ File source, GNU Radio), .cs16 = surowe.
  python3 rx_iq.py --snap --ch 1 --n 255
      1024 pomiary do RAM ESP i odczyt (szybciej niz UART: n=31 -> 390 kS/s).
  python3 rx_iq.py --f 2414.5 ...
      LO poza srodkiem kanalu: najblizszy kanal + offset PLL (krok 1/1024 MHz),
      ESP odsyla faktyczne LO w linii "RXF <Hz> CH <n> OFF <off>".

Uwagi: na wyjsciu jest offset DC odbiornika i lustro ok. -30 dB (bez korekcji
I/Q); --dc odejmuje srednia. Czestotliwosc srodka = LO z linii RXF.
"""
import argparse
import re
import time
import wave

import numpy as np
import serial

CPU_HZ = 160e6
BAUD = 2000000


def open_esp(port):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.2
    s.dtr = s.rts = False
    s.open()
    time.sleep(2.5)
    s.read(65536)
    return s


def rxf(buf):
    """LO w Hz z linii "RXF <Hz> ..." (None, gdy jej nie ma)."""
    m = re.search(rb"RXF (\d+)", buf)
    return int(m.group(1)) if m else None


def capture(s, f, n, mode):
    """Komenda IQ: 1024 pomiary -> (t [s], z, moc, LO Hz)."""
    s.write(f"IQ {f} {n} {mode}\r\n".encode())
    buf, t0 = b"", time.time()
    while b"IQDATA " not in buf and time.time() - t0 < 5:
        buf += s.read(4096)
    if b"IQDATA " not in buf:
        raise SystemExit(f"brak IQDATA: {buf[-200:]!r}")
    lo = rxf(buf)
    buf = buf[buf.index(b"IQDATA "):]
    while b"\n" not in buf:
        buf += s.read(64)
    head, buf = buf.split(b"\n", 1)
    cnt = int(head.split()[1])
    need = cnt * 16
    while len(buf) < need and time.time() - t0 < 30:
        buf += s.read(need - len(buf))
    raw = np.frombuffer(buf[:need], "<u4").reshape(cnt, 4)
    t = np.unwrap(raw[:, 0].astype(np.float64), period=2 ** 32) / CPU_HZ
    i = raw[:, 1].view(np.int32).astype(np.float64)
    q = raw[:, 2].view(np.int32).astype(np.float64)
    return t, (i + 1j * q) / (n + 1), raw[:, 3].astype(np.float64), lo


def stream(s, f, k, mode, seconds):
    """Komenda IQSTREAM -> (z int16 Nx2, fs z naglowkow, statystyki)."""
    s.write(f"IQSTREAM {f} {k} {mode}\r\n".encode())
    buf, t0 = b"", time.time()
    while b"OK IQSTREAM" not in buf and time.time() - t0 < 5:
        buf += s.read(4096)
    if b"OK IQSTREAM" not in buf:
        raise SystemExit(f"ESP nie wszedl w IQSTREAM: {buf[-200:]!r}")
    time.sleep(0.01)
    s.baudrate = BAUD
    s.reset_input_buffer()
    raw = bytearray()
    t0 = time.time()
    while time.time() - t0 < seconds:
        raw += s.read(max(1, s.in_waiting))
    s.write(b"x")                                   # koniec strumienia
    s.baudrate = 115200
    tail, t1 = b"", time.time()
    while b"IQSTREAM END" not in tail and time.time() - t1 < 3:
        tail += s.read(4096)
    end = [l for l in tail.decode(errors="replace").splitlines() if "IQSTREAM END" in l]

    iq, fs, lost, total = parse(bytes(raw))
    return iq, fs, dict(packets=total, lost=lost, lo=rxf(buf),
                        esp=end[0] if end else "brak podsumowania")


SPP = 16                 # probek na paczke
PK = 5 + SPP * 4         # A5 5A, ccount 24 bity, 16 x (I, Q) i16


def parse(d):
    """Paczka jest wazna, gdy nastepny naglowek stoi dokladnie PK dalej (CH340
    gubi pojedyncze bajty). Paczki ukladane na osi czasu wg ccount, brakujace
    = zera, wiec czas i faza sie nie rozjezdzaja. -> (iq Nx2, fs, zgubione, wszystkie)"""
    pos = [i for i in range(len(d) - PK - 1) if d[i] == 0xA5 and d[i + 1] == 0x5A
           and d[i + PK] == 0xA5 and d[i + PK + 1] == 0x5A]
    ts = np.array([int.from_bytes(d[i + 2:i + 5], "little") for i in pos], np.float64)
    dts = np.diff(ts) % 2 ** 24
    per = np.median(dts)                           # cykli na paczke
    ok = np.concatenate(([True], np.abs(dts - np.round(dts / per) * per) < 0.01 * per))
    pos, ts = [p for p, k in zip(pos, ok) if k], ts[ok]
    idx = (np.unwrap(ts, period=2 ** 24) - ts[0]) / per
    idx = np.round(idx).astype(int)
    iq = np.zeros((SPP * (idx[-1] + 1), 2), np.int16)
    for p, k in zip(pos, idx):
        iq[SPP * k:SPP * k + SPP] = np.frombuffer(d[p + 5:p + PK], "<i2").reshape(SPP, 2)
    return iq, CPU_HZ * SPP / per, idx[-1] + 1 - len(pos), idx[-1] + 1


def save(path, iq, fs):
    if path.endswith(".wav"):
        with wave.open(path, "wb") as w:
            w.setnchannels(2)
            w.setsampwidth(2)
            w.setframerate(int(round(fs)))
            w.writeframes(iq.astype("<i2").tobytes())
    else:
        iq.astype("<i2").tofile(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ch", type=int, default=1)
    ap.add_argument("--f", help="LO w MHz (np. 2414.5), zamiast --ch")
    ap.add_argument("--k", type=int, default=10, help="okno 2^k probek ADC (strumien)")
    ap.add_argument("--mode", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=10)
    ap.add_argument("--out", default="zrzut.wav")
    ap.add_argument("--dc", action="store_true", help="odejmij srednia (offset DC odbiornika)")
    ap.add_argument("--snap", action="store_true", help="1024 pomiary przez RAM zamiast strumienia")
    ap.add_argument("--n", type=int, default=255, help="probek ADC na pomiar w --snap")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    a = ap.parse_args()
    f = a.f or a.ch
    s = open_esp(a.port)
    if a.snap:
        t, z, p, lo = capture(s, f, a.n, a.mode)
        s.close()
        dt = np.diff(t)
        print(f"{1 / np.median(dt) / 1e3:.1f} kS/s, LO {lo} Hz, I sr {z.real.mean():+.1f}, "
              f"Q sr {z.imag.mean():+.1f}")
        np.savez(a.out.rsplit(".", 1)[0] + ".npz", t=t, z=z, p=p, n=a.n, lo=lo or 0)
        return
    iq, fs, info = stream(s, f, a.k, a.mode, a.seconds)
    s.close()
    if a.dc:
        iq = np.round(iq - iq.mean(0)).astype(np.int16)
    save(a.out, iq, fs)
    print(f"{a.out}: {len(iq)} probek, fs {fs:.1f} S/s ({len(iq) / fs:.1f} s), LO "
          f"{info['lo']} Hz | paczek {info['packets']}, "
          f"zgubionych (zera) {info['lost']} ({info['lost'] / info['packets'] * 100:.1f}%) | {info['esp']}")


if __name__ == "__main__":
    main()
