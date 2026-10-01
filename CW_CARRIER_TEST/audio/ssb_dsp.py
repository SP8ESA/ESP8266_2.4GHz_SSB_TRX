"""DSP do SSB na ESP8266 (metoda polarna, Kahn): procesor mowy, sygnal
analityczny i tablice obwiedni/czestotliwosci dla modulatora w firmware.

Uzywane przez make_audio_h.py (tablice we flashu) i stream_ssb.py (probki
pchane przez USB).

Obwiednia: ASK w Q8 (0 = szczyt, 127*256 = podloga ~-31 dB, 128*256 =
bramka OFF), wg nominalnego prawa tlumika (DB_PER_ASK na krok), bez
predystorsji. Czestotliwosc: chwilowa czestotliwosc USB jako offset K w Q16
(1 kod = 78.125 kHz); LSB = minus to samo.
"""
import subprocess
import wave

import numpy as np
from scipy.ndimage import maximum_filter1d
from scipy.signal import filtfilt, firwin, firwin2, hilbert, welch

SSB_RATE = 32000
DB_PER_ASK = 0.2429     # pomiar z repo (M-ASK reference), APWR 64
HZ_PER_CODE = 78125.0
# pasmo i EQ mowy SSB: cel = nachylenie EQ_TILT dB/oktawe od 400 Hz (0 = plasko,
# jasno; 3 = cieplej, blizej naturalnej mowy), podbicie max EQ_BOOST dB
SSB_LO, SSB_HI = 200, 2800
EQ_TILT = 1.5
EQ_BOOST = 10.0
# kompresja: AF ratio i prog [dB pod glosna mowa], RF (obwiednia SSB) ratio
# i sufit limitera [percentyl |z|, 100 = brak], sufit limitera AF [percentyl
# |audio|], szczyt obwiedni = ASK 0 przy percentylu |z| (wyzej twarde ciecie)
COMP = {"light": (3, 15, 1.5, 100.0, 99.9, 99.99),
        "soft": (3, 20, 2, 100.0, 99, 99.9),
        "mid": (6, 25, 4, 98.0, 99, 99.9),
        "hard": (8, 30, 6, 95.0, 99, 99.9)}
TRIM = ("silenceremove=start_periods=1:start_threshold=-45dB,areverse,"
        "silenceremove=start_periods=1:start_threshold=-45dB,areverse")


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
    wyprzedzeniem zamiast clippera."""
    ratio, thr = COMP[comp][:2]
    lim = COMP[comp][4]
    xe = speech_eq(x, fs, tilt)
    lv = 10 * np.log10(follow(xe * xe, 0.010, 0.150, fs) + 1e-20)
    T = np.percentile(lv, 90) - thr
    gdb = np.where(lv > T, (T + (lv - T) / ratio) - lv, 0.0)
    y = xe * 10 ** (smooth(gdb, fs, 5) / 20)
    y /= np.percentile(np.abs(y), 99.9)
    return limiter(y, fs, np.percentile(np.abs(y), lim))


def rf_proc(z, fs, bpf, comp="mid"):
    """Obrobka obwiedni sygnalu SSB: kompresja (detektor 10 / 100 ms,
    wzmocnienie wygladzone) i limiter obwiedni z wyprzedzeniem, potem
    ponowny filtr pasma i Hilbert. Mniej czasu przy podlodze -31 dB i
    nizszy PAPR = wiecej sredniej mocy."""
    ratio, clip = COMP[comp][2:4]
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
        y = speech_proc(x, fs, comp, tilt)
        z = rf_proc(hilbert(filtfilt(bpf, 1, y)), fs, bpf, comp)
    z /= np.percentile(np.abs(z), 99.9 if raw else COMP[comp][5])
    return z, x_raw, z.real


def ask_curve():
    """Nominalna krzywa tlumika ASK: poziomy 0..127 i amplituda liniowa."""
    levels = np.arange(128)
    return dict(levels=levels, amp_lin=10 ** (-DB_PER_ASK * levels / 20))


def polar(z, cal, fs=SSB_RATE, phase_only=False, env_only=False, log=print, gate_ms=5):
    """Sygnal analityczny -> (env_q8 uint16, freq_q16 int16) dla modulatora.
    gate_ms: bramka OFF dopiero przy obwiedni pod podloga dluzej niz tyle ms;
    0 = nigdy (pauzy i dolki zostaja na podlodze -31 dB)."""
    levels, amp_lin = cal["levels"], cal["amp_lin"]
    # obwiednia na srodku probki (ZOH amplitudy trwa [n, n+1)); ASK n/n+1 jest
    # ditherowane, wiec srednia amplituda jest liniowa miedzy punktami krzywej
    zm = (z[:-1] + z[1:]) / 2
    env = np.clip(np.abs(zm), amp_lin[-1], 1.0)
    ask = np.interp(env, amp_lin[::-1], levels[::-1].astype(float))
    if phase_only:   # diagnostyka: stala obwiednia, sama faza
        ask = np.zeros_like(ask)
    # max ASK 126.5, nie 127: ditherBurst miesza an z an+1, a przy an = 127
    # an+1 = 128 to bramka OFF - resztka bledu ditheringu przy wejsciu na
    # podloge wylaczala bramke na chwile, a kazde wlaczenie bramki to losowa
    # faza nosnej (gate_test.py) -> trwale skoki fazy = trzeszczenie
    env_q8 = np.clip(np.round(ask * 256), 0, 126.5 * 256).astype(np.uint16)
    # pauzy: obwiednia ponizej podlogi dluzej niz 5 ms -> bramka OFF (128), z 1 ms
    # marginesu na podlodze po obu stronach (szybko przelaczana bramka nie dziala)
    quiet = np.abs(zm) < amp_lin[-1]
    edges = np.flatnonzero(np.diff(np.concatenate(([0], quiet.astype(int), [0]))))
    gated = 0
    for a, b in zip(edges[0::2], edges[1::2]):
        if gate_ms and b - a >= fs * gate_ms / 1000:
            m = fs // 1000
            env_q8[a + m:b - m] = 128 * 256
            gated += b - a - 2 * m
    log(f"ssb: bramka OFF w pauzach przez {gated / fs * 1000:.0f} ms "
        f"({gated / len(env_q8) * 100:.0f}% czasu)")
    # czestotliwosc z przyrostu fazy, kwantyzacja ze sprzezeniem bledu:
    # faza koncowa trafia co do Q16, bez dryfu
    phase = np.unwrap(np.angle(z))
    qf = np.diff(phase) * fs / (2 * np.pi * HZ_PER_CODE) * 65536
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
