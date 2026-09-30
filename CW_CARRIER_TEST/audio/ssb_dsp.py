"""DSP do SSB na ESP8266 (metoda polarna, Kahn): procesor mowy, sygnal
analityczny i tablice obwiedni/czestotliwosci dla modulatora w firmware.

Uzywane przez make_audio_h.py (tablice we flashu) i stream_ssb.py (probki
pchane przez USB).

Obwiednia: ASK w Q8 (0 = szczyt, 127*256 = podloga ~-31 dB, 128*256 =
bramka OFF), wg zmierzonej krzywej toru ASK (ask_cal.json z cal_hackrf.py).
Czestotliwosc: chwilowa czestotliwosc USB jako offset K w Q16 (1 kod =
78.125 kHz), minus zmierzone AM-PM i AM-FM toru ASK; LSB = minus to samo.
"""
import json
import subprocess
import wave
from pathlib import Path

import numpy as np
from scipy.ndimage import maximum_filter1d
from scipy.signal import (filtfilt, firwin, firwin2, hilbert, lfilter,
                          savgol_filter, welch)

SSB_RATE = 32000
DB_PER_ASK = 0.2429     # pomiar z repo (M-ASK reference), APWR 64
HZ_PER_CODE = 78125.0
# pasmo i EQ mowy SSB: cel = nachylenie EQ_TILT dB/oktawe od 400 Hz (0 = plasko,
# jasno; 3 = cieplej, blizej naturalnej mowy), podbicie max EQ_BOOST dB
SSB_LO, SSB_HI = 200, 2800
EQ_TILT = 1.5
EQ_BOOST = 10.0
# kompresja: AF ratio i prog [dB pod glosna mowa], RF (obwiednia SSB) ratio
# i sufit limitera [percentyl |z|, 100 = brak]
COMP = {"soft": (3, 20, 2, 100.0), "mid": (6, 25, 4, 98.0), "hard": (8, 30, 6, 95.0)}
TRIM = ("silenceremove=start_periods=1:start_threshold=-45dB,areverse,"
        "silenceremove=start_periods=1:start_threshold=-45dB,areverse")
CAL = Path(__file__).resolve().parent / "ask_cal.json"


def load_audio(src, rate, extra_af="", trim=True):
    """Plik audio (cokolwiek czyta ffmpeg) albo "tones:f1:f2..." (3.5 s)."""
    if src.startswith("tones:"):
        t = np.arange(int(3.5 * rate)) / rate
        return sum(np.sin(2 * np.pi * float(f) * t) for f in src.split(":")[1:])
    pcm = subprocess.run(
        ["ffmpeg", "-v", "error", "-i", src, "-ac", "1", "-ar", str(rate),
         "-af", extra_af + (TRIM if trim else "anull"), "-f", "s16le", "-"],
        check=True, capture_output=True).stdout
    x = np.frombuffer(pcm, dtype="<i2").astype(np.float64)
    return x - x.mean()


def follow(v, att, rel, fs):
    """Sledzenie obwiedni: filtr 1 rzedu, osobne stale ataku i zwolnienia [s]."""
    aa, ar = np.exp(-1 / (att * fs)), np.exp(-1 / (rel * fs))
    e = np.empty_like(v)
    acc = v[0]
    for i, u in enumerate(v):
        k = aa if u > acc else ar
        acc = k * acc + (1 - k) * u
        e[i] = acc
    return e


def smooth(v, fs, ms):
    n = max(1, int(ms * 1e-3 * fs))
    return np.convolve(v, np.ones(n) / n, "same")


def speech_gate(xe, fs):
    """Maska mowy (0..1) liczona offline, wiec moze patrzec w obie strony:
    prog 10 dB nad szumem tla nagrania, maska poszerzona o +-40 ms (bez
    obcinania poczatkow i koncowek slow), zbocza 10 ms. Nakladana na samym
    koncu, zeby zadna kompresja nie podniosla tla w pauzach."""
    lvl = 10 * np.log10(smooth(xe * xe, fs, 20) + 1e-20)
    thr = min(np.percentile(lvl, 90) - 20, np.percentile(lvl, 3) + 10)
    mask = maximum_filter1d((lvl > thr).astype(float), size=int(0.08 * fs) + 1)
    return smooth(mask, fs, 10)


def limiter(v, fs, ceil, look_ms=5, rel_ms=60):
    """Limiter z wyprzedzeniem: wzmocnienie schodzi plynnie juz przed szczytem
    i wraca w rel_ms - nic nie jest obcinane (clippery dawaly charczenie)."""
    n = max(1, int(look_ms * 1e-3 * fs))
    pk = maximum_filter1d(np.abs(v), size=2 * n + 1)
    g = np.minimum(1.0, ceil / np.maximum(pk, 1e-12))
    g = 1 - follow(1 - g, 0.0002, rel_ms * 1e-3, fs)   # szybko w dol, wolno w gore
    return v * smooth(g, fs, look_ms)


def speech_eq(x, fs, tilt=EQ_TILT):
    """Pasmo SSB_LO..SSB_HI i auto-EQ: dlugoterminowe widmo (1/3 okt.) do
    nachylenia -tilt dB/okt. od 400 Hz, korekta -10..+EQ_BOOST dB."""
    bp = firwin(1023, [SSB_LO, SSB_HI], pass_zero=False, fs=fs)
    xb = filtfilt(bp, 1, x)
    f, P = welch(xb, fs=fs, nperseg=4096)
    Ps = np.array([P[(f >= fc * 2 ** (-1 / 6)) & (f <= fc * 2 ** (1 / 6))].mean()
                   if fc > 0 else P[0] for fc in f])
    band = (f >= 250) & (f <= 2600)
    target = -tilt * np.log2(np.maximum(f, 1) / 400)
    g = target - 10 * np.log10(np.maximum(Ps, 1e-30))
    g_db = np.clip(g - np.mean(g[band]), -10, EQ_BOOST)
    g_db[(f < SSB_LO - 30) | (f > SSB_HI)] = -30
    eq = firwin2(1025, f / (fs / 2), 10 ** (g_db / 40))  # filtfilt = kwadrat
    return filtfilt(eq, 1, xb)


def speech_proc(x, fs, comp="mid", tilt=EQ_TILT):
    """Procesor mowy: auto-EQ, kompresor (detektor RMS 10 / 150 ms, wolniejszy
    niz okres tonu krtaniowego, wzmocnienie wygladzone), limiter z
    wyprzedzeniem zamiast clippera. Zwraca (audio, maska mowy)."""
    ratio, thr = COMP[comp][:2]
    xe = speech_eq(x, fs, tilt)
    lv = 10 * np.log10(follow(xe * xe, 0.010, 0.150, fs) + 1e-20)
    T = np.percentile(lv, 90) - thr
    gdb = np.where(lv > T, (T + (lv - T) / ratio) - lv, 0.0)
    y = xe * 10 ** (smooth(gdb, fs, 5) / 20)
    y /= np.percentile(np.abs(y), 99.9)
    return limiter(y, fs, np.percentile(np.abs(y), 99)), speech_gate(xe, fs)


def rf_proc(z, fs, bpf, comp="mid"):
    """Obrobka obwiedni sygnalu SSB: kompresja (detektor 10 / 100 ms,
    wzmocnienie wygladzone) i limiter obwiedni z wyprzedzeniem, potem
    ponowny filtr pasma i Hilbert. Mniej czasu przy podlodze -31 dB i
    nizszy PAPR = wiecej sredniej mocy."""
    ratio, clip = COMP[comp][2:]
    a = np.abs(z)
    e = follow(a, 0.010, 0.100, fs)
    T = np.percentile(e, 95) * 10 ** (-18 / 20)
    gdb = np.where(e > T, 20 * np.log10(np.maximum(e, 1e-12) / T) * (1 / ratio - 1), 0.0)
    z = z * 10 ** (smooth(gdb, fs, 5) / 20)
    if clip < 100:
        z = limiter(z, fs, np.percentile(np.abs(z), clip), look_ms=3, rel_ms=50)
    return hilbert(filtfilt(bpf, 1, z.real))


def analytic(x, fs=SSB_RATE, comp="mid", raw=False, tilt=EQ_TILT):
    """Audio -> znormalizowany sygnal analityczny USB. Zwraca
    (z, audio przed obrobka, audio po obrobce)."""
    bpf = firwin(1023, [SSB_LO, SSB_HI], pass_zero=False, fs=fs)
    x_raw = filtfilt(bpf, 1, x)
    if raw:
        z = hilbert(x_raw)
    else:
        y, gate = speech_proc(x, fs, comp, tilt)
        z = rf_proc(hilbert(filtfilt(bpf, 1, y)), fs, bpf, comp) * gate  # pauzy = cisza
    z /= np.percentile(np.abs(z), 99.9)
    return z, x_raw, z.real


def load_cal(no_amfm=False, log=print):
    """Krzywe toru ASK: amplituda (liniowo), AM-PM [rad], AM-FM [Hz], tau AM-FM."""
    levels = np.arange(128)
    if not CAL.exists():
        log("ssb: brak ask_cal.json - prawo z repo, bez korekcji AM-PM")
        return dict(levels=levels, amp_lin=10 ** (-DB_PER_ASK * levels / 20),
                    ampm=np.zeros(128), amfm=np.zeros(128), amfm_tau=0.0)
    j = json.loads(CAL.read_text())
    amp_db = savgol_filter(np.array(j["amp_db"][:128]), 9, 2)
    amp_db = np.minimum.accumulate(amp_db - amp_db[0])  # malejaca od 0 dB
    if "ampm_step_deg" in j:  # skok fazy na krawedzi (bez udzialu AM-FM)
        ampm = np.polyval(np.polyfit(j["amfm_levels"], j["ampm_step_deg"], 2), levels)
    else:
        ampm = np.polyval(np.polyfit(levels, np.array(j["phase_deg"][:128]), 3,
                                     w=10 ** (amp_db / 20)), levels)
    ampm = np.radians(ampm - ampm[0])
    # AM->FM: czestotliwosc wyjscia spada do ~-70 Hz przy nizszym poziomie,
    # z opoznieniem 1 rzedu - kompensowane w tablicy czestotliwosci
    amfm = np.array(j["amfm_hz"]) - j["amfm_hz"][0] if "amfm_hz" in j else np.zeros(128)
    if no_amfm:
        amfm = np.zeros(128)
    log(f"ssb: krzywa z {CAL.name} ({j.get('esp_cmds')}), zakres {amp_db[-1]:.1f} dB, "
        f"AM-PM do {np.degrees(ampm).max():+.1f} deg, AM-FM do {amfm.min():+.0f} Hz")
    return dict(levels=levels, amp_lin=10 ** (amp_db / 20), ampm=ampm, amfm=amfm,
                amfm_tau=j.get("amfm_tau_ms", 0.0) * 1e-3)


def polar(z, cal, fs=SSB_RATE, phase_only=False, env_only=False, log=print):
    """Sygnal analityczny -> (env_q8 uint16, freq_q16 int16) dla modulatora."""
    levels, amp_lin = cal["levels"], cal["amp_lin"]
    # obwiednia na srodku probki (ZOH amplitudy trwa [n, n+1)); ASK n/n+1 jest
    # ditherowane, wiec srednia amplituda jest liniowa miedzy punktami krzywej
    zm = (z[:-1] + z[1:]) / 2
    env = np.clip(np.abs(zm), amp_lin[-1], 1.0)
    ask = np.interp(env, amp_lin[::-1], levels[::-1].astype(float))
    if phase_only:   # diagnostyka: stala obwiednia, sama faza
        ask = np.zeros_like(ask)
    env_q8 = np.clip(np.round(ask * 256), 0, 127 * 256).astype(np.uint16)
    # pauzy: obwiednia ponizej podlogi dluzej niz 5 ms -> bramka OFF (128), z 1 ms
    # marginesu na podlodze po obu stronach (szybko przelaczana bramka nie dziala)
    quiet = np.abs(zm) < amp_lin[-1]
    edges = np.flatnonzero(np.diff(np.concatenate(([0], quiet.astype(int), [0]))))
    gated = 0
    for a, b in zip(edges[0::2], edges[1::2]):
        if b - a >= fs * 5 // 1000:
            m = fs // 1000
            env_q8[a + m:b - m] = 128 * 256
            gated += b - a - 2 * m
    log(f"ssb: bramka OFF w pauzach przez {gated / fs * 1000:.0f} ms "
        f"({gated / len(env_q8) * 100:.0f}% czasu)")
    # czestotliwosc z przyrostu fazy minus AM-PM i AM-FM toru, kwantyzacja ze
    # sprzezeniem bledu: faza koncowa trafia co do Q16, bez dryfu
    phase = np.unwrap(np.angle(z))
    phase[1:] -= np.interp(ask, levels, cal["ampm"])
    qf = np.diff(phase) * fs / (2 * np.pi * HZ_PER_CODE) * 65536
    fam = np.interp(ask, levels, cal["amfm"])
    if cal["amfm_tau"] > 0:  # ten sam filtr 1 rzedu co w ESP (tau ~0.27 ms)
        al = 1 - np.exp(-1 / (cal["amfm_tau"] * fs))
        fam = lfilter([al], [1, al - 1], fam, zi=[fam[0] * (1 - al)])[0]
    qf -= fam / HZ_PER_CODE * 65536
    assert np.abs(qf).max() < 32767
    cum = np.round(np.cumsum(qf)).astype(np.int64)
    freq_q16 = np.diff(np.concatenate(([0], cum))).astype(np.int16)
    if env_only:     # diagnostyka: sama obwiednia, stala czestotliwosc
        freq_q16[:] = 0
    return env_q8, freq_q16


def band_stats(name, x, fs, log=print):
    f, P = welch(x, fs=fs, nperseg=4096)
    b = lambda lo, hi: 10 * np.log10(P[(f >= lo) & (f < hi)].sum() / P.sum())
    log(f"{name}: 200-500 {b(200,500):5.1f} | 500-1k {b(500,1000):5.1f} | 1-2k {b(1000,2000):5.1f} | "
        f"2-2.8k {b(2000,2800):5.1f} dB | crest {20*np.log10(np.abs(x).max()/np.sqrt(np.mean(x*x))):4.1f} dB")


def write_wav(path, x, fs):
    x = x / np.abs(x).max() * 0.9
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(fs)
        w.writeframes((x * 32767).astype("<i2").tobytes())
