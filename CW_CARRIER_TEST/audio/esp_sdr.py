#!/usr/bin/env python3
"""ESP8266 jako odbiornik SSB: widmo, waterfall, demodulator USB/LSB, dzwiek.

  python3 esp_sdr.py                     # start na 2400.250 MHz USB
  python3 esp_sdr.py --f 2414.5 --lsb
  python3 esp_sdr.py --wav test.wav --seconds 10   # bez okna: zapis audio (test)

Zrodlo: IQSTREAM z CW_CARRIER_TEST.ino (estymator IQ odbiornika, okno 2^k
probek ADC, k=10 -> ~35.7 kS/s, pasmo ok. +-17 kHz wokol LO). LO ustawia ESP
(najblizszy kanal + offset PLL, krok 1/1024 MHz); czestotliwosc odbioru w
pasmie to cyfrowy NCO. Gdy odbior wyjdzie poza bezpieczny srodek pasma (albo
wejdzie na kreske DC), LO przestawia sie samo - przestrojenie to ok. 0.1 s
przerwy. Klik / przeciagniecie w widmie albo waterfallu stroi, kolko myszy
na polu czestotliwosci zmienia ja o krok.

Zgubione przez CH340 paczki (16 probek) sa wypelniane biezacym DC, wiec po
filtrze DC nie trzaskaja. Dzwiek: liniowa interpolacja do 48 kHz z lekka
korekta proporcji wg zapelnienia bufora (zegary ESP i karty dzwiekowej sie
rozjezdzaja).
"""
import argparse
import queue
import re
import sys
import threading
import time
import wave

import numpy as np
import serial
from scipy.signal import firwin, lfilter

CPU_HZ = 160e6
BAUD = 2_000_000
SPP = 16
PK = 5 + SPP * 4
HDR = b"\xA5\x5A"
AUDIO_FS = 48000
NFFT = 2048
WF_ROWS = 400
FILTERS = {"1.8 kHz": (400, 2200), "2.4 kHz": (300, 2700), "2.8 kHz": (200, 3000),
           "3.5 kHz": (100, 3600)}


# ---------------------------------------------------------------- ESP

class EspReader(threading.Thread):
    """Watek z portem: IQSTREAM, skladanie paczek na osi czasu, przestrajanie.
    Wynik w self.out: (generacja, LO Hz, fs, probki complex64)."""

    def __init__(self, port, k=10, mode=0):
        super().__init__(daemon=True)
        self.port, self.k, self.mode = port, k, mode
        self.out = queue.Queue(maxsize=200)
        self.req = queue.Queue()
        self.gen = 0
        self.lo = None
        self.fs = None
        self.lost = self.packets = 0
        self.err = None
        self.running = True

    def tune(self, khz):
        self.req.put(int(round(khz)))

    def stop(self):
        self.running = False

    def _open(self):
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = self.port, 115200, 0.05
        s.dtr = s.rts = False
        s.open()
        time.sleep(2.5)
        s.read(65536)
        return s

    def _start(self, s, khz):
        s.baudrate = 115200
        s.reset_input_buffer()
        s.write(f"IQSTREAM {khz // 1000}.{khz % 1000:03d} {self.k} {self.mode}\r\n".encode())
        buf, t0 = b"", time.time()
        while b"OK IQSTREAM" not in buf and time.time() - t0 < 3:
            buf += s.read(4096)
        m = re.search(rb"RXF (\d+)", buf)
        if b"OK IQSTREAM" not in buf or not m:
            raise RuntimeError(f"ESP: {buf[-200:]!r}")
        time.sleep(0.01)
        s.baudrate = BAUD
        s.reset_input_buffer()
        return int(m.group(1))

    def _stop(self, s):
        s.write(b"x")
        time.sleep(0.02)
        s.baudrate = 115200
        buf, t0 = b"", time.time()
        while b"IQSTREAM END" not in buf and time.time() - t0 < 2:
            buf += s.read(4096)

    def run(self):
        try:
            s = self._open()
        except Exception as e:  # noqa: BLE001
            self.err = str(e)
            return
        khz = self.req.get()
        while self.running:
            try:
                lo = self._start(s, khz)
            except Exception as e:  # noqa: BLE001
                self.err = str(e)
                return
            self.gen += 1
            self.lo, self.err = lo, None
            khz = self._pump(s)
            self._stop(s)
        s.close()

    def _pump(self, s):
        """Czyta strumien do zadania przestrojenia; zwraca nowe kHz albo None."""
        buf = bytearray()
        per = (2 ** self.k * 25e-9 + 1.8e-6) * CPU_HZ * SPP   # cykli na paczke (start)
        prev = None
        dc = 0j
        pend = []
        t_flush = time.time()
        while self.running:
            try:
                new = self.req.get_nowait()
                while not self.req.empty():
                    new = self.req.get_nowait()
                return new
            except queue.Empty:
                pass
            buf += s.read(max(1, s.in_waiting))
            pos = 0
            while True:
                i = buf.find(HDR, pos)
                if i < 0 or i + PK + 2 > len(buf):
                    pos = max(pos, len(buf) - PK - 2) if i < 0 else i
                    break
                if buf[i + PK:i + PK + 2] != HDR:
                    pos = i + 1
                    continue
                ts = buf[i + 2] | buf[i + 3] << 8 | buf[i + 4] << 16
                v = np.frombuffer(bytes(buf[i + 5:i + PK]), "<i2").astype(np.float32)
                z = (v[0::2] + 1j * v[1::2]).astype(np.complex64)
                if prev is not None:
                    dt = (ts - prev) & 0xFFFFFF
                    n = int(round(dt / per))
                    if n == 1:
                        per += (dt - per) * 0.002
                    elif 1 < n < 200:
                        self.lost += n - 1
                        self.packets += n - 1
                        pend.append(np.full(SPP * (n - 1), dc, np.complex64))
                prev = ts
                dc += (z.mean() - dc) * 0.02
                pend.append(z)
                self.packets += 1
                pos = i + PK
            del buf[:pos]
            if pend and time.time() - t_flush > 0.02:
                self.fs = CPU_HZ * SPP / per
                try:
                    self.out.put_nowait((self.gen, self.lo, self.fs, np.concatenate(pend)))
                except queue.Full:
                    pass
                pend = []
                t_flush = time.time()
        return None


# ---------------------------------------------------------------- DSP

class Resampler:
    """Liniowa interpolacja ciagla miedzy blokami."""

    def __init__(self):
        self.prev = 0.0
        self.t = 0.0

    def process(self, x, step):
        y = np.concatenate(([self.prev], x))
        last = len(y) - 1
        if self.t > last:
            self.t -= len(x)
            self.prev = y[-1]
            return np.zeros(0, np.float32)
        idx = self.t + step * np.arange(int((last - self.t) / step) + 1)
        i0 = np.minimum(idx.astype(int), last - 1) if last >= 1 else idx.astype(int)
        fr = idx - i0
        out = y[i0] * (1 - fr) + y[np.minimum(i0 + 1, last)] * fr
        self.t = idx[-1] + step - last
        self.prev = y[-1]
        return out.astype(np.float32)


class AudioFifo:
    def __init__(self, cap=AUDIO_FS * 2):
        self.buf = np.zeros(cap, np.float32)
        self.r = self.w = 0
        self.lock = threading.Lock()

    def fill(self):
        return (self.w - self.r) % len(self.buf)

    def write(self, x):
        with self.lock:
            n = len(self.buf)
            if len(x) >= n - self.fill():
                return
            j = np.arange(self.w, self.w + len(x)) % n
            self.buf[j] = x
            self.w = (self.w + len(x)) % n

    def read(self, m):
        with self.lock:
            n = len(self.buf)
            k = min(m, self.fill())
            j = np.arange(self.r, self.r + k) % n
            out = np.zeros(m, np.float32)
            out[:k] = self.buf[j]
            self.r = (self.r + k) % n
            return out


class Demod:
    """DC -> NCO -> zespolony filtr pasmowy (jedna wstega) -> Re -> AGC."""

    def __init__(self):
        self.fs = None
        self.off = 0.0          # czestotliwosc odbioru wzgledem LO [Hz]
        self.lsb = False
        self.band = FILTERS["2.4 kHz"]
        self.volume = 0.5
        self.agc_peak = 1e-3
        self.gain = 0.0
        self.smeter = -120.0
        self._dirty = True
        self.reset()

    def reset(self):
        self.phase = 0.0
        self.dc = None
        self.zi = None
        self.agc_peak = 1e-3
        self.res = Resampler()

    def set(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)
        self._dirty = True

    def _design(self):
        lo, hi = self.band
        bw, fc = (hi - lo) / 2, (hi + lo) / 2 * (-1 if self.lsb else 1)
        h = firwin(255, bw, fs=self.fs, window=("kaiser", 7))
        self.h = (h * np.exp(2j * np.pi * fc / self.fs * np.arange(255))).astype(np.complex64)
        self.zi = np.zeros(254, np.complex64)
        self._dirty = False

    def process(self, z, fs):
        if self.fs is None or abs(fs - self.fs) > 0.002 * self.fs:
            self.fs = fs
            self._dirty = True
        if self._dirty:
            self._design()
        # filtr DC (stala ok. 0.3 s), pierwszy blok startuje od jego sredniej
        if self.dc is None:
            self.dc = complex(z.mean())
        a = 1 / (0.3 * fs)
        dc, zf = lfilter([a], [1, -(1 - a)], z, zi=[self.dc * (1 - a)])
        self.dc = complex(zf[0]) / (1 - a)
        x = (z - dc).astype(np.complex64)
        n = np.arange(len(x))
        w = 2 * np.pi * self.off / fs
        x = x * np.exp(-1j * (self.phase + w * n)).astype(np.complex64)
        self.phase = (self.phase + w * len(x)) % (2 * np.pi)
        b, self.zi = lfilter(self.h, 1, x, zi=self.zi)
        p = float(np.mean(np.abs(b) ** 2)) + 1e-12
        self.smeter = 10 * np.log10(p)
        a = b.real
        # AGC: szczyt z szybkim atakiem i wolnym (ok. 1.5 s) zwolnieniem
        pk = float(np.max(np.abs(a))) if len(a) else 0.0
        self.agc_peak = max(pk, self.agc_peak * np.exp(-len(a) / (1.5 * fs)))
        g = min(0.5 / max(self.agc_peak, 1e-6), 2000.0)
        gg = np.linspace(self.gain or g, g, len(a), dtype=np.float32)
        self.gain = g
        return a.astype(np.float32) * gg * self.volume


def spectrum_rows(x, fs):
    """Widma (dB) kolejnych okien NFFT, bez nakladania."""
    win = np.blackman(NFFT).astype(np.float32)
    k = len(x) // NFFT
    if k == 0:
        return []
    f = np.fft.fftshift(np.fft.fft(x[:k * NFFT].reshape(k, NFFT) * win, axis=1), axes=1)
    return list(10 * np.log10(np.abs(f) ** 2 / (NFFT * (win ** 2).sum()) + 1e-12))


class Engine(threading.Thread):
    """Laczy ESP z demodulatorem, dzwiekiem i widmem; strojenie i auto-LO."""

    def __init__(self, port, f_hz, lsb, ppm=0.0):
        super().__init__(daemon=True)
        self.ppm = ppm          # kwarc ESP: prawdziwe LO = nominalne * (1 + ppm)
        self.esp_lo = None      # LO nominalne (z RXF)
        self.esp = EspReader(port)
        self.demod = Demod()
        self.demod.lsb = lsb
        self.fifo = AudioFifo()
        self.rows = queue.Queue(maxsize=100)
        self.f_rx = f_hz
        self.lo_req = None
        self.gen = 0
        self.lo = None
        self.fs = 35700.0
        self.spec_buf = np.zeros(0, np.complex64)
        self.running = True
        self.lock = threading.RLock()

    # -- strojenie
    def lo_for(self, f_hz):
        """LO 5 kHz obok sygnalu, po stronie bez wstegi (USB: LO nizej), zeby
        kreska DC i spur ESP (-0.75 kHz) nie wpadly w pasmo audio."""
        side = 1 if self.demod.lsb else -1
        return int(round((f_hz + side * 5000) / (1 + self.ppm * 1e-6) / 1000))

    def set_ppm(self, ppm):
        with self.lock:
            self.ppm = ppm
            if self.esp_lo:
                self.lo = self.esp_lo * (1 + ppm * 1e-6)
                self.demod.set(off=self.f_rx - self.lo)
                try:
                    self.rows.put_nowait(("clear", self.lo, self.fs))
                except queue.Full:
                    pass
            self.set_freq(self.f_rx)

    def band_edges(self, off):
        lo_b, hi_b = self.demod.band
        return (off - hi_b, off - lo_b) if self.demod.lsb else (off + lo_b, off + hi_b)

    def set_freq(self, f_hz):
        """Odbior w pasmie -> tylko NCO; poza bezpiecznym srodkiem pasma albo
        z DC w filtrze -> nowe LO."""
        with self.lock:
            self.f_rx = f_hz
            ok = False
            if self.lo:
                a, b = self.band_edges(f_hz - self.lo)
                ok = (a > 400 or b < -400) and max(abs(a), abs(b)) < self.fs / 2 - 1500
            if ok:
                self.demod.set(off=f_hz - self.lo)
            else:
                khz = self.lo_for(f_hz)
                if self.lo_req != khz:
                    self.lo_req = khz
                    self.esp.tune(khz)

    def set_mode(self, lsb):
        with self.lock:
            self.demod.set(lsb=lsb)
            self.set_freq(self.f_rx)

    def set_band(self, band):
        with self.lock:
            self.demod.set(band=band)
            self.set_freq(self.f_rx)

    def set_lo(self, khz):
        with self.lock:
            self.lo_req = int(khz)
            self.esp.tune(int(khz))

    def run(self):
        self.esp.start()
        self.set_freq(self.f_rx)
        while self.running:
            try:
                gen, lo, fs, z = self.esp.out.get(timeout=0.2)
            except queue.Empty:
                continue
            with self.lock:
                if gen != self.gen:
                    self.esp_lo = lo
                    lo = lo * (1 + self.ppm * 1e-6)
                    self.gen, self.lo, self.fs = gen, lo, fs
                    self.lo_req = None
                    self.demod.reset()
                    self.demod.set(off=self.f_rx - lo)
                    self.spec_buf = np.zeros(0, np.complex64)
                    try:
                        self.rows.put_nowait(("clear", lo, fs))
                    except queue.Full:
                        pass
                self.fs = fs
                lo = self.lo
                a = self.demod.process(z, fs)
            fill = self.fifo.fill()
            corr = np.clip((fill - 0.25 * AUDIO_FS) / (0.25 * AUDIO_FS), -1, 1) * 0.005
            self.fifo.write(self.demod.res.process(a, fs / AUDIO_FS * (1 + corr)))
            self.spec_buf = np.concatenate((self.spec_buf, z))
            k = len(self.spec_buf) // NFFT
            if k:
                for r in spectrum_rows(self.spec_buf - self.spec_buf.mean(), fs):
                    try:
                        self.rows.put_nowait(("row", lo, fs, r))
                    except queue.Full:
                        pass
                self.spec_buf = self.spec_buf[k * NFFT:]

    def stop(self):
        self.running = False
        self.esp.stop()


# ---------------------------------------------------------------- GUI

def run_gui(eng, args):
    import pyqtgraph as pg
    import sounddevice as sd
    from PyQt5 import QtCore, QtWidgets

    class FreqAxis(pg.AxisItem):
        """Os w kHz wzgledem LO, podpisy jako czestotliwosc bezwzgledna."""
        lo = 0

        def tickValues(self, minVal, maxVal, size):
            out = []
            for sp in (5000.0, 1000.0):
                a = np.ceil((self.lo + minVal) / sp) * sp
                vals = np.arange(a, self.lo + maxVal + 1, sp) - self.lo
                out.append((sp, list(vals)))
            return out

        def tickStrings(self, values, scale, spacing):
            return [f"{(self.lo + v) / 1e6:.3f}" for v in values]

    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=False, imageAxisOrder="row-major")
    win = QtWidgets.QWidget()
    win.setWindowTitle("ESP8266 SDR")
    lay = QtWidgets.QVBoxLayout(win)

    # -- pasek sterowania
    bar = QtWidgets.QHBoxLayout()
    lay.addLayout(bar)
    bar.addWidget(QtWidgets.QLabel("Odbiór [MHz]:"))
    freq = QtWidgets.QDoubleSpinBox()
    freq.setDecimals(6)
    freq.setRange(2300.0, 2600.0)
    freq.setSingleStep(0.0001)
    freq.setValue(eng.f_rx / 1e6)
    freq.setKeyboardTracking(False)
    freq.setMinimumWidth(170)
    f = freq.font()
    f.setPointSize(f.pointSize() + 4)
    f.setBold(True)
    freq.setFont(f)
    bar.addWidget(freq)
    bar.addWidget(QtWidgets.QLabel("krok:"))
    step = QtWidgets.QComboBox()
    steps = {"10 Hz": 10, "50 Hz": 50, "100 Hz": 100, "500 Hz": 500, "1 kHz": 1000,
             "5 kHz": 5000, "100 kHz": 100000}
    step.addItems(list(steps))
    step.setCurrentText("100 Hz")
    bar.addWidget(step)
    mode = QtWidgets.QComboBox()
    mode.addItems(["USB", "LSB"])
    mode.setCurrentText("LSB" if eng.demod.lsb else "USB")
    bar.addWidget(mode)
    filt = QtWidgets.QComboBox()
    filt.addItems(list(FILTERS))
    filt.setCurrentText("2.4 kHz")
    bar.addWidget(filt)
    bar.addWidget(QtWidgets.QLabel("głośność:"))
    vol = QtWidgets.QSlider(QtCore.Qt.Horizontal)
    vol.setRange(0, 100)
    vol.setValue(50)
    vol.setMaximumWidth(120)
    bar.addWidget(vol)
    bar.addWidget(QtWidgets.QLabel("kwarc ESP [ppm]:"))
    ppm = QtWidgets.QDoubleSpinBox()
    ppm.setDecimals(2)
    ppm.setRange(-50, 50)
    ppm.setSingleStep(0.01)
    ppm.setValue(eng.ppm)
    ppm.setKeyboardTracking(False)
    ppm.valueChanged.connect(eng.set_ppm)
    bar.addWidget(ppm)
    bar.addStretch(1)
    smeter = QtWidgets.QProgressBar()
    smeter.setRange(0, 60)
    smeter.setFormat("S %v dB")
    smeter.setMaximumWidth(160)
    bar.addWidget(smeter)

    # -- widmo i waterfall
    gl = pg.GraphicsLayoutWidget()
    lay.addWidget(gl, 1)
    ax1 = FreqAxis("bottom")
    sp = gl.addPlot(row=0, col=0, axisItems={"bottom": ax1})
    sp.setLabel("left", "dB")
    sp.showGrid(x=True, y=True, alpha=0.3)
    sp.setMouseEnabled(x=False, y=False)
    sp.hideButtons()
    curve = sp.plot(pen=pg.mkPen("#ffd54a", width=1))
    avg_curve = sp.plot(pen=pg.mkPen("#4fc3f7", width=1))
    ax2 = FreqAxis("bottom")
    wp = gl.addPlot(row=1, col=0, axisItems={"bottom": ax2})
    wp.setMouseEnabled(x=False, y=False)
    wp.hideButtons()
    wp.hideAxis("left")
    wp.setXLink(sp)
    gl.ci.layout.setRowStretchFactor(0, 2)
    gl.ci.layout.setRowStretchFactor(1, 3)
    img = pg.ImageItem()
    wp.addItem(img)
    img.setLookupTable(pg.colormap.get("inferno").getLookupTable(nPts=256))
    wf = np.full((WF_ROWS, NFFT), -100.0, np.float32)
    regions = []
    lines = []
    for p in (sp, wp):
        r = pg.LinearRegionItem(movable=False, brush=pg.mkBrush(80, 200, 120, 50),
                                pen=pg.mkPen(None))
        p.addItem(r)
        regions.append(r)
        ln = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen("#ff5252", width=2))
        p.addItem(ln)
        lines.append(ln)
    status = QtWidgets.QLabel("łączenie z ESP…")
    lay.addWidget(status)

    st = dict(lo=None, fs=eng.fs, avg=None, lvl=None, rows=0)

    def apply_freq(hz, from_box=False):
        eng.set_freq(hz)
        if not from_box:
            freq.blockSignals(True)
            freq.setValue(hz / 1e6)
            freq.blockSignals(False)
        draw_marker()

    def draw_marker():
        if st["lo"] is None:
            return
        off = eng.f_rx - st["lo"]
        lo_b, hi_b = eng.demod.band
        a, b = (off - hi_b, off - lo_b) if eng.demod.lsb else (off + lo_b, off + hi_b)
        for r in regions:
            r.setRegion((a, b))
        for ln in lines:
            ln.blockSignals(True)
            ln.setValue(off)
            ln.blockSignals(False)

    def on_line(ln):
        if st["lo"] is not None:
            apply_freq(st["lo"] + ln.value())

    for ln in lines:
        ln.sigPositionChangeFinished.connect(on_line)

    def on_click(ev):
        if ev.button() != QtCore.Qt.LeftButton or st["lo"] is None:
            return
        for p in (sp, wp):
            if p.sceneBoundingRect().contains(ev.scenePos()):
                x = p.vb.mapSceneToView(ev.scenePos()).x()
                apply_freq(st["lo"] + round(x / 10) * 10)
                return

    gl.scene().sigMouseClicked.connect(on_click)

    freq.valueChanged.connect(lambda v: apply_freq(round(v * 1e6), from_box=True))
    step.currentTextChanged.connect(lambda t: freq.setSingleStep(steps[t] / 1e6))
    freq.setSingleStep(steps[step.currentText()] / 1e6)

    def on_mode(t):
        eng.set_mode(t == "LSB")
        draw_marker()

    mode.currentTextChanged.connect(on_mode)

    def on_filt(t):
        eng.set_band(FILTERS[t])
        draw_marker()

    filt.currentTextChanged.connect(on_filt)
    vol.valueChanged.connect(lambda v: setattr(eng.demod, "volume", (v / 100) ** 2 * 2))
    eng.demod.volume = 0.5

    def tick():
        new = []
        while True:
            try:
                new.append(eng.rows.get_nowait())
            except queue.Empty:
                break
        for item in new:
            if item[0] == "row" and abs(item[2] - st["fs"]) > 0.003 * st["fs"]:
                item = ("clear", item[1], item[2])     # fs sie ustalilo: nowa skala
            if item[0] == "clear":
                _, lo, fs = item[:3]
                st.update(lo=lo, fs=fs, avg=None)
                wf[:] = st["lvl"][0] if st["lvl"] else -100
                ax1.lo = ax2.lo = lo
                sp.setXRange(-fs / 2, fs / 2, padding=0)
                img.setImage(wf, autoLevels=False, levels=st["lvl"] or (-100, -40))
                img.setRect(QtCore.QRectF(-fs / 2, 0, fs, WF_ROWS))
                wp.setYRange(0, WF_ROWS, padding=0)
                ax1.picture = ax2.picture = None
                ax1.update()
                ax2.update()
                draw_marker()
                continue
            _, lo, fs, r = item
            wf[1:] = wf[:-1]
            wf[0] = r
            st["avg"] = r if st["avg"] is None else st["avg"] * 0.85 + r * 0.15
            st["rows"] += 1
        if new and st["avg"] is not None:
            x = (np.arange(NFFT) - NFFT / 2) * st["fs"] / NFFT
            curve.setData(x, wf[0])
            avg_curve.setData(x, st["avg"])
            floor = float(np.median(st["avg"]))
            lv = (floor - 5, floor + 55)
            st["lvl"] = lv if st["lvl"] is None else tuple(
                0.95 * a + 0.05 * b for a, b in zip(st["lvl"], lv))
            sp.setYRange(st["lvl"][0] - 5, st["lvl"][1] + 10, padding=0)
            img.setImage(wf[::-1], autoLevels=False, levels=st["lvl"])
            noise = floor + 10 * np.log10(NFFT / st["fs"] * (eng.demod.band[1] - eng.demod.band[0]))
            smeter.setValue(int(np.clip(eng.demod.smeter - noise, 0, 60)))
        e = eng.esp
        if e.err:
            status.setText(f"BŁĄD ESP: {e.err}")
        elif st["lo"] is not None:
            lost = e.lost / max(e.packets, 1) * 100
            status.setText(
                f"LO {st['lo'] / 1e6:.6f} MHz (ESP {eng.esp_lo}) | fs {st['fs']:.0f} S/s (pasmo ±{st['fs'] / 2e3:.1f} kHz)"
                f" | odbiór {eng.f_rx / 1e6:.6f} MHz {'LSB' if eng.demod.lsb else 'USB'}"
                f" | zgubione paczki {lost:.1f}% | bufor audio {eng.fifo.fill() / AUDIO_FS * 1000:.0f} ms")

    def audio_cb(outdata, frames, t, stt):
        outdata[:, 0] = np.clip(eng.fifo.read(frames), -1, 1)

    stream = sd.OutputStream(samplerate=AUDIO_FS, channels=1, blocksize=1024,
                             dtype="float32", callback=audio_cb)
    stream.start()
    timer = QtCore.QTimer()
    timer.timeout.connect(tick)
    timer.start(30)
    win.resize(1200, 750)
    win.show()
    eng.start()
    rc = app.exec_()
    stream.stop()
    eng.stop()
    time.sleep(0.3)
    return rc


def run_wav(eng, path, seconds):
    """Bez okna: demodulowane audio do WAV (do testow)."""
    eng.start()
    t0 = time.time()
    out = []
    while time.time() - t0 < seconds:
        time.sleep(0.05)
        out.append(eng.fifo.read(eng.fifo.fill()))
    eng.stop()
    a = np.concatenate(out)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(AUDIO_FS)
        w.writeframes((np.clip(a, -1, 1) * 32767).astype("<i2").tobytes())
    e = eng.esp
    print(f"{path}: {len(a) / AUDIO_FS:.1f} s, LO {eng.lo}, fs {eng.fs:.0f}, "
          f"zgubione {e.lost}/{e.packets}, blad: {e.err}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--f", type=float, default=2400.250, help="czestotliwosc odbioru [MHz]")
    ap.add_argument("--lsb", action="store_true")
    ap.add_argument("--ppm", type=float, default=-0.5,
                    help="kwarc ESP wzgledem wzorca (HackRF: ok. -0.5 ppm)")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--wav", help="bez okna: zapisz demodulowane audio")
    ap.add_argument("--seconds", type=float, default=10)
    a = ap.parse_args()
    eng = Engine(a.port, int(round(a.f * 1e6)), a.lsb, a.ppm)
    if a.wav:
        run_wav(eng, a.wav, a.seconds)
    else:
        sys.exit(run_gui(eng, a))


if __name__ == "__main__":
    main()
