/*
  CW_CARRIER_TEST.ino

  Niemodulowana nosna (CW) z ESP8266 + opcjonalne kluczowanie Morse.
  Sterowanie PHY wziete z kaboom748/esp8266_WLAN_PHY (TX_REFERENCE_v1,
  TEST-TONEv5): slot tonu 0x600005B8, PBUS, ROM set_txclk_en /
  set_ana_inf_tx_scale.

  Po zasileniu cisza (nadajnik wylaczony). SSB na zywo z PC: komenda STREAM +
  audio/stream_ssb.py (probki przez USB, 2 Mbaud). Z flasha na komende:
  w petli nagranie z audio_data.h / ssb_data.h + 1 s ciszy, bez auto-OFF.
  Serial 115200, komendy:
    ON          nosna ON (auto-OFF po TIME sekundach)
    OFF         nosna OFF
    CH n        kanal 1..14 (srodek 2412..2484 MHz)
    K n         kod tonu 0..1023 (0 = srodek kanalu), na zywo
    ASK n       skala cyfrowa 0..63, na zywo
    APWR n      skala analogowa 0..255 (to NIE sa dBm), przy ON
    TIME s      auto-OFF po s sekundach, 0 = bez limitu
    WPM n       predkosc Morse 5..40
    CW tekst    nadaje tekst Morsem (bramka tonu), dowolny znak przerywa
    SWEEP a b   trojkatny sweep a..b MHz, do 1 kHz (np. 2434.75 2435.25)
    SWEEP ON    sweep z ostatnim zakresem
    SWEEP OFF   koniec sweepu, powrot na K (nosna zostaje)
    PERIOD ms   okres trojkata (gora + dol) 20..600000 ms
    FM f        FM na f MHz (np. 2425.1), audio z audio_data.h w petli
    FM ON/OFF   FM z ostatnia czestotliwoscia / koniec (nosna zostaje)
    DEV hz      dewiacja FM 100..75000 Hz (5000 = NFM, 75000 = WFM)
    AM f        AM na f MHz, to samo audio; obwiednia przez pole ASK
    AM ON/OFF   AM z ostatnia czestotliwoscia / koniec (nosna zostaje)
    DEPTH n     glebokosc AM 0..95 %
    USB f       SSB gorna wstega, f = czestotliwosc (wytlumionej) nosnej
    LSB f       SSB dolna wstega; USB/LSB ON/OFF jak przy FM/AM
    CAL AM|DLY  sygnaly kalibracyjne pod SSB (AM-AM/AM-PM, opoznienie)
    IQ f n t    odbior: 1024 probki IQ, f = kanal 1..14 albo MHz (2300..2600)
    IQSTREAM f k t  odbior: ciagly strumien IQ 2 Mbaud (audio/rx_iq.py)
    STATUS, ?   stan i pomoc

  Sweep zmienia tylko pole K w slocie tonu (K ze znakiem, ~78.125 kHz/kod
  wg 2-FSK/ESP8266_2FSK_GUIDE_DETAILLE_FR_v2.0.md), a posrednie
  czestotliwosci daje sigma-delta z ditheringiem (ditherBurst); FM to ta
  sama sciezka z czestotliwoscia zmieniana co probke audio. AM zmienia co
  probke pole ASK (tlumik ~0.243 dB/krok wg M-ASK/ESP8266_ASK_M-ASK_REFERENCE_EN.md),
  z tablica log dla liniowej obwiedni i ditheringiem ulamka. USB/LSB to
  modulacja polarna (Kahn): obwiednia |z| przez ASK (+ bramka OFF ponizej
  -31 dB) i jednoczesnie chwilowa czestotliwosc d(arg z)/dt przez K, obie
  z ssb_data.h (Hilbert liczony offline). Kanal jest wybierany
  raz przy starcie tak, zeby zakres byl jak najblizej jego srodka; potem
  bramka, ASK i APWR stoja w miejscu - bez przestrajania PLL i bez przerw.

  Audio: audio/make_audio_h.py (tu: audio/sp8esa.mp3).

  UWAGA: dlugie ciagle TX mocno grzeje uklad - nie zostawiaj bez nadzoru.
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
extern "C" {
  #include "user_interface.h"
}
#include "audio_data.h"
#include "ssb_data.h"

static uint8_t  gChannel  = 6;
static uint16_t gK        = 0;
static uint8_t  gAsk      = 32;
static uint8_t  gApwr     = 127;  // liniowy PA, najmniej zaklocen fazy od AM (cal_hackrf.py am)
static uint32_t gTimeoutS = 0;
static uint8_t  gWpm      = 15;
static uint32_t gSweepLo  = 2424750;  // kHz
static uint32_t gSweepHi  = 2425250;  // kHz
static uint32_t gPeriodMs = 10000;
static uint32_t gModKHz    = 2402000;  // kHz
static uint32_t gDevHz    = 5000;
static uint8_t  gDepth    = 80;       // % glebokosci AM

static bool     phyReady  = false;
static bool     txOn      = false;
static uint32_t txStartMs = 0;

enum Mode : uint8_t { MODE_NONE, MODE_SWEEP, MODE_FM, MODE_AM, MODE_USB, MODE_LSB,
                     MODE_CAL };
static Mode     mode       = MODE_NONE;

static uint32_t sweepT0    = 0;
static int32_t  sweepQLo   = 0;  // K w Q16
static int32_t  sweepQHi   = 0;

static int32_t  modQc      = 0;  // nosna FM/AM, K w Q16
static int32_t  fmDevQ     = 0;  // dewiacja przy pelnej skali, K w Q16
static uint32_t auRate     = 0;  // probek audio na sekunde
static uint32_t auCps      = 0;  // cykle CPU na probke audio
static uint32_t auNext     = 0;  // ccount konca biezacej probki
static uint32_t auPos      = 0;
static uint32_t auGap      = 0;  // probki ciszy do odczekania
static int32_t  amAskQ[255];     // probka -127..127 -> ASK w Q16

static constexpr uint32_t TONE1      = 0x600005B8u;
static constexpr uint32_t TONE2      = 0x600005BCu;
static constexpr uint32_t TONE3      = 0x600005C4u;
static constexpr uint32_t PBUS_CMD   = 0x60000594u;
static constexpr uint32_t PBUS_STAT  = 0x600005A0u;
static constexpr uint32_t RX_CTRL    = 0x60009B08u;

static constexpr uint32_t K_MASK      = 0x000003FFu;
static constexpr uint32_t SCALE_MASK  = 0x0003FC00u;
static constexpr uint32_t GATE_MASK   = 0x00040000u;
static constexpr uint32_t SCALE_SHIFT = 10u;
static constexpr uint32_t RX_STOP     = 0x08000000u;

static constexpr uint32_t ROM_SET_TXCLK_EN         = 0x4000650Cu;
static constexpr uint32_t ROM_SET_ANA_INF_TX_SCALE = 0x4000678Cu;

using SetTxClkFn    = void(*)(int);
using SetAnaScaleFn = uint8_t(*)(uint8_t);

static SetTxClkFn set_txclk =
  reinterpret_cast<SetTxClkFn>(ROM_SET_TXCLK_EN);
static SetAnaScaleFn set_ana_scale =
  reinterpret_cast<SetAnaScaleFn>(ROM_SET_ANA_INF_TX_SCALE);

static inline void memw() { __asm__ volatile("memw" ::: "memory"); }
static inline uint32_t rd32(uint32_t a) {
  return *reinterpret_cast<volatile uint32_t*>(a);
}
static inline void wr32(uint32_t a, uint32_t v) {
  *reinterpret_cast<volatile uint32_t*>(a) = v;
}
static inline uint8_t askCode(uint8_t ask) {
  return static_cast<uint8_t>(0u - ask);
}
static int chMHz(uint8_t ch) {
  return ch == 14 ? 2484 : 2407 + 5 * ch;
}

// ---------------------------------------------------------------- PHY

static bool pbusWrite(uint8_t sel, uint8_t bank, uint16_t value) {
  uint32_t cmd = rd32(PBUS_CMD);
  cmd &= 0xFFFF0001u;
  cmd |= (uint32_t)(bank & 3u) << 14;
  cmd |= (uint32_t)(value & 0x1FFu) << 5;
  cmd |= (uint32_t)(sel & 7u) << 2;
  cmd |= 2u;
  wr32(PBUS_CMD, cmd);
  memw();

  const uint32_t t0 = micros();
  bool ok = true;
  while (rd32(PBUS_STAT) & 0x80000000u) {
    if ((uint32_t)(micros() - t0) > 2500u) { ok = false; break; }
    delay(0);
  }

  uint32_t r = rd32(PBUS_CMD);
  r &= ~2u;
  wr32(PBUS_CMD, r);
  memw();
  return ok;
}

static void enterManual() {
  uint32_t r = rd32(RX_CTRL);
  r |= RX_STOP;
  wr32(RX_CTRL, r);
  memw();

  r = rd32(PBUS_CMD);
  r |= 1u;
  wr32(PBUS_CMD, r);
  memw();
}

static void leaveManual() {
  uint32_t r = rd32(PBUS_CMD);
  r &= ~1u;
  wr32(PBUS_CMD, r);
  memw();

  r = rd32(RX_CTRL);
  r &= ~RX_STOP;
  wr32(RX_CTRL, r);
  memw();
}

static bool txPathOn() {
  return pbusWrite(2,1,1) &&
         pbusWrite(7,1,95) &&
         pbusWrite(1,1,127) &&
         pbusWrite(6,1,127);
}

static void txPathOff() {
  pbusWrite(6,1,0);
  pbusWrite(1,1,12);
  pbusWrite(2,1,0);
}

static void toneProgram() {
  uint32_t r = rd32(TONE1);
  r &= ~(K_MASK | SCALE_MASK | GATE_MASK);
  r |= (uint32_t)gK & K_MASK;
  r |= (uint32_t)askCode(gAsk) << SCALE_SHIFT;
  r |= GATE_MASK;
  wr32(TONE1, r);

  r = rd32(TONE2);
  r &= ~GATE_MASK;
  wr32(TONE2, r);

  r = rd32(TONE3);
  r &= ~GATE_MASK;
  wr32(TONE3, r);
  memw();
}

static inline void toneField(uint32_t mask, uint32_t value) {
  uint32_t r = rd32(TONE1);
  r = (r & ~mask) | (value & mask);
  wr32(TONE1, r);
  memw();
}

static inline void gate(bool on) {
  toneField(GATE_MASK, on ? GATE_MASK : 0u);
}

static bool setChannel(uint8_t ch) {
  if (!wifi_set_channel(ch)) return false;
  delay(40);
  gChannel = wifi_get_channel();
  return gChannel == ch;
}

static bool phyStart() {
  if (phyReady) return true;
  // Core 3.x startuje z WiFi w forced sleep; WiFi.mode() go wybudza.
  WiFi.persistent(false);
  if (!WiFi.mode(WIFI_STA)) return false;
  wifi_station_set_auto_connect(0);
  wifi_station_disconnect();
  if (!wifi_set_sleep_type(NONE_SLEEP_T)) return false;
  delay(100);
  if (!setChannel(gChannel)) return false;
  phyReady = true;
  return true;
}

static void rxUntune();

static bool txStart() {
  if (txOn) return true;
  if (!phyStart()) return false;
  rxUntune();
  enterManual();
  if (!txPathOn()) {
    leaveManual();
    return false;
  }
  set_ana_scale(gApwr);
  set_txclk(1);
  toneProgram();
  txOn = true;
  txStartMs = millis();
  return true;
}

static void txStop() {
  if (!txOn) return;
  gate(false);
  set_txclk(0);
  txPathOff();
  leaveManual();
  txOn = false;
}

// ---------------------------------------------------------------- Sweep / FM

static constexpr int32_t K_LIMIT = 500;  // |K| < 512, ~39 MHz

// offset kHz -> K w Q16 (1 kod = 78.125 kHz = 5000/64 kHz)
static int32_t khzToQ16(int32_t khz) {
  return (int32_t)(((int64_t)khz * 64 * 65536) / 5000);
}

static uint8_t bestChannel(uint32_t loKHz, uint32_t hiKHz) {
  uint8_t best = 1;
  int32_t bestOff = INT32_MAX;
  for (uint8_t ch = 1; ch <= 14; ++ch) {
    const int32_t c = chMHz(ch) * 1000;
    const int32_t off = max(abs((int32_t)loKHz - c), abs((int32_t)hiKHz - c));
    if (off < bestOff) {
      bestOff = off;
      best = ch;
    }
  }
  return best;
}

// wybiera kanal pod zakres lo..hi kHz i wlacza nosna; zwraca srodek kanalu
// w kHz albo 0 przy bledzie
static int32_t tuneFor(uint32_t loKHz, uint32_t hiKHz) {
  const uint8_t ch = bestChannel(loKHz, hiKHz);
  const int32_t c = chMHz(ch) * 1000;
  if (khzToQ16((int32_t)loKHz - c) < -(K_LIMIT << 16) ||
      khzToQ16((int32_t)hiKHz - c) > (K_LIMIT << 16)) {
    Serial.println(F("ERR ZAKRES ZA DALEKO OD KANALU"));
    return 0;
  }

  if (!phyStart()) return 0;
  if (ch != gChannel) {
    const bool wasOn = txOn;
    txStop();
    if (!setChannel(ch)) {
      Serial.println(F("ERR SET CHANNEL"));
      return 0;
    }
    if (wasOn && !txStart()) return 0;
  }
  if (!txStart()) {
    Serial.println(F("ERR TX START"));
    return 0;
  }
  return c;
}

static void modeStop() {
  if (mode == MODE_NONE) return;
  mode = MODE_NONE;
  if (txOn) toneProgram();
}

// Modulator czestotliwosci: w rejestrze kod K najblizszy celowi q (K w Q16),
// a blad fazy (cel - faktyczna, Q16 kod * cykl CPU) odrabiany impulsem k+-1
// o dokladnie wyliczonej dlugosci, gdy przekroczy prog. Faza trzyma sie
// celu z dokladnoscia do progu (~3.5 deg), a nie kroku calej iteracji petli
// (~17 deg) - przy malych ulamkach (typowe dla SSB) krok iteracji dawal
// piloksztalt fazy w pasmie audio. Czas liczony od faktycznych chwil zapisu
// (po memw), wiec przestoje (flash, NMI) sa rozliczane.
static int32_t  sdErr    = 0;   // blad fazy, Q16 kod * cykl
static int32_t  sdAErr   = 0;   // blad amplitudy, Q16 ASK
static int32_t  sdCode   = 0;   // kod aktualnie w rejestrze
static uint32_t sdLast   = 0;   // ccount ostatniego zapisu
static uint32_t sdRng    = 0x9E3779B9u;
static int32_t  sdThr    = 20 << 16;  // prog impulsu ~ 20 cykli przy kodzie +-1
static uint32_t gBurstMs = 1;   // SWEEP: dlugosc burstu bez przerw (diagnostyka)
static int32_t  gSweepAq = -1;  // SWEEP: ASK w Q8 (diagnostyka), -1 = ASK
static uint32_t sdIters  = 0;   // do pomiaru szybkosci w STATUS
static uint32_t sdCycles = 0;

// poziom ASK 0..127, 128 = bramka OFF
static inline uint32_t levelBits(uint32_t level) {
  return level >= 128 ? 0u : GATE_MASK | ((uint32_t)askCode(level) << SCALE_SHIFT);
}

// trzyma srednia czestotliwosc q i srednie tlumienie aq (ASK w Q16, 0..128,
// ulamek ditherowany miedzy n i n+1) az ccount dojdzie do endCycle
static IRAM_ATTR __attribute__((noinline)) void ditherBurst(int32_t q, int32_t aq,
                                                             uint32_t endCycle) {
  const int32_t kb = (q + 0x8000) >> 16;  // kod najblizszy celowi
  const int32_t r = q - (kb << 16);       // dryf bledu na cykl przy kb, |r| <= 32768
  const uint32_t base = rd32(TONE1) & ~(K_MASK | SCALE_MASK | GATE_MASK);
  const uint32_t kN = base | ((uint32_t)kb & K_MASK);
  const uint32_t kU = base | ((uint32_t)(kb + 1) & K_MASK);
  const uint32_t kD = base | ((uint32_t)(kb - 1) & K_MASK);
  const uint32_t an = (uint32_t)aq >> 16;
  const uint32_t s0 = levelBits(an);
  const uint32_t s1 = levelBits(an + 1);
  const int32_t aFrac = aq & 0xFFFF;
  const int32_t thr = sdThr;

  int32_t err = sdErr;
  int32_t aErr = sdAErr;
  bool aUp = false;
  int32_t code = sdCode;
  uint32_t last = sdLast;
  uint32_t rng = sdRng;
  uint32_t n = 0;

  const uint32_t ps = xt_rsil(15);
  const uint32_t t0 = ESP.getCycleCount();
  do {
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    aErr += aFrac - (aUp ? 65536 : 0);
    aUp = aErr + (int32_t)(rng & 0xFFFF) - 32768 > 0;
    const uint32_t sb = aUp ? s1 : s0;

    uint32_t dt = ESP.getCycleCount() - last;
    if (dt > 8192u) dt = 8192u;  // dluga przerwa (yield): nie nadrabiaj
    const int32_t errNow = err + (q - (code << 16)) * (int32_t)dt;

    if (errNow > thr || errNow < -thr) {
      // impuls k+-1 dokladnie tak dlugi, zeby sprowadzic blad do zera
      const bool up = errNow > 0;
      const int32_t rate = up ? 65536 - r : 65536 + r;
      int32_t len = (up ? errNow : -errNow) / rate;
      if (len > 4000) len = 4000;
      wr32(TONE1, (up ? kU : kD) | sb);
      memw();
      const uint32_t tp = ESP.getCycleCount();
      dt = tp - last;
      if (dt > 8192u) dt = 8192u;
      err += (q - (code << 16)) * (int32_t)dt;
      code = up ? kb + 1 : kb - 1;
      while ((int32_t)(ESP.getCycleCount() - tp) < len) {}
      last = tp;
    }
    wr32(TONE1, kN | sb);
    memw();
    const uint32_t tw = ESP.getCycleCount();
    dt = tw - last;
    if (dt > 8192u) dt = 8192u;
    err += (q - (code << 16)) * (int32_t)dt;
    err = constrain(err, -(1 << 29), 1 << 29);
    code = kb;
    last = tw;
    ++n;
  } while ((int32_t)(last - endCycle) < 0);
  xt_wsr_ps(ps);
  memw();

  sdErr = err;
  sdAErr = aErr;
  sdCode = code;
  sdLast = last;
  sdRng = rng;
  sdIters += n;
  sdCycles += last - t0;
}

static bool sweepStart() {
  const int32_t c = tuneFor(gSweepLo, gSweepHi);
  if (!c) return false;
  sweepQLo = khzToQ16((int32_t)gSweepLo - c);
  sweepQHi = khzToQ16((int32_t)gSweepHi - c);
  sweepT0 = micros();
  mode = MODE_SWEEP;
  return true;
}

// polozenie na trojkacie liczone z czasu, jitter loop() go nie przesuwa
static void sweepTick() {
  const uint32_t periodUs = gPeriodMs * 1000u;
  uint32_t el = micros() - sweepT0;
  while (el >= periodUs) {
    sweepT0 += periodUs;
    el -= periodUs;
  }

  const uint32_t half = periodUs / 2;
  const uint32_t pos = el < half ? el : periodUs - el;  // 0..half
  const uint32_t range = (uint32_t)(sweepQHi - sweepQLo);
  const int32_t q = sweepQLo + (int32_t)((uint64_t)range * pos / half);

  ditherBurst(q, gSweepAq < 0 ? (int32_t)gAsk << 16 : gSweepAq << 8,
              ESP.getCycleCount() + ESP.getCpuFreqMHz() * 1000u * gBurstMs);
}

// Audio z AUDIO[] w petli z 1 s ciszy (sama nosna), jedna probka na burst.
// FM: czestotliwosc = nosna + DEV * probka/128, ASK stale.
// AM: czestotliwosc stala, obwiednia 1 + DEPTH * probka/127 przez ASK.
static inline bool isSsb(Mode m) { return m == MODE_USB || m == MODE_LSB; }

static bool audioStart(Mode m) {
  const uint32_t spanKHz = m == MODE_FM ? (gDevHz + 999) / 1000 : 5;
  const int32_t c = tuneFor(gModKHz - spanKHz, gModKHz + spanKHz);
  if (!c) return false;
  modQc = khzToQ16((int32_t)gModKHz - c);
  fmDevQ = (int32_t)((int64_t)gDevHz * 64 * 65536 / 5000000);

  // ASK to tlumik ~0.2429 dB/krok (ASK 0 = max, pomiar z repo przy APWR 64),
  // wiec liniowa obwiednia -> ASK = -20*log10(e/e_max)/0.2429; szczyt = ASK 0
  const float peak = 1.0f + gDepth / 100.0f;
  for (int i = -127; i <= 127; ++i) {
    const float e = (1.0f + gDepth / 100.0f * i / 127.0f) / peak;
    float ask = -20.0f * log10f(e) / 0.2429f;
    ask = constrain(ask, 0.0f, 127.0f);
    amAskQ[i + 127] = (int32_t)(ask * 65536.0f);
  }

  if (isSsb(m) != isSsb(mode)) {  // inna tablica - od poczatku
    auPos = 0;
    auGap = 0;
  }
  auRate = isSsb(m) ? SSB_RATE : AUDIO_RATE;
  auCps = ESP.getCpuFreqMHz() * 1000000u / auRate;
  auNext = ESP.getCycleCount();
  mode = m;
  return true;
}

// jedna probka: FM/AM z AUDIO[] (8 kHz), USB/LSB z SSB_ENV/SSB_FREQ (32 kHz);
// w 1 s przerwie FM/AM daja sama nosna, SSB nic (bramka OFF)
static void audioSample(uint32_t endCycle) {
  const uint32_t len = isSsb(mode) ? SSB_LEN : AUDIO_LEN;
  const bool gap = auGap > 0;
  const uint32_t pos = auPos;
  if (gap) {
    --auGap;
  } else if (++auPos >= len) {
    auPos = 0;
    auGap = auRate;
  }

  if (isSsb(mode)) {
    if (gap) {
      ditherBurst(modQc, 128 << 16, endCycle);
      return;
    }
    const int32_t f = (int16_t)pgm_read_word(&SSB_FREQ[pos]);
    const int32_t env = pgm_read_word(&SSB_ENV[pos]);
    ditherBurst(mode == MODE_USB ? modQc + f : modQc - f, env << 8, endCycle);
    return;
  }

  const int32_t a = gap ? 0 : (int8_t)pgm_read_byte(&AUDIO[pos]);
  if (mode == MODE_FM) {
    ditherBurst(modQc + ((a * fmDevQ) >> 7), (int32_t)gAsk << 16, endCycle);
  } else {
    ditherBurst(modQc, amAskQ[a + 127], endCycle);
  }
}

// SSB: cale zdanie jednym ciagiem z wylaczonymi przerwaniami. Kazda przerwa
// (powrot do SDK, ISR) zostawia w rejestrze k albo k+1 zamiast sredniej, co
// przesuwa faze - dla SSB (faza przez calke czestotliwosci) to smietnik
// po obu stronach. Czas dla systemu i komend jest w 1 s ciszy (bramka OFF).
static void ssbSentence() {
  const uint32_t ps = xt_rsil(15);
  auNext = ESP.getCycleCount();
  do {  // audioSample na koncu tablicy wraca na 0 i ustawia auGap
    auNext += auCps;
    audioSample(auNext);
    if ((auPos & 1023) == 0) ESP.wdtFeed();
  } while (auGap == 0);
  xt_wsr_ps(ps);
}

static void audioTick() {
  if (isSsb(mode) && auGap == 0) {
    ssbSentence();  // konczy sie ustawieniem auGap (1 s ciszy)
    return;
  }
  // po dlugiej przerwie (np. obsluga komendy) nie nadrabiaj, tylko graj dalej
  if ((int32_t)(ESP.getCycleCount() - auNext) > (int32_t)(8 * auCps)) {
    auNext = ESP.getCycleCount();
  }
  for (uint32_t i = 0; i < auRate / 1000; ++i) {  // ~1 ms, potem loop()
    auNext += auCps;
    audioSample(auNext);
  }
}

// Kalibracja pod SSB, nagrywana HackRF-em (analiza offline), w kolko:
// CAL AM:  OFF 4 ms, REF (ASK 0) 4 ms, potem dla kazdego poziomu z listy
//          poziom 0.5 ms + REF 0.5 ms -> AM-AM i AM-PM toru ASK/bramki
// CAL DLY: co 0.25 ms naraz (K, ASK 0) <-> (K+1, ASK 20) -> opoznienie
//          zmiany amplitudy wzgledem zmiany czestotliwosci
static const uint16_t CAL_EXTRA_Q8[] = {
  127 * 256 + 64, 127 * 256 + 128, 127 * 256 + 192,  // dithering z bramka OFF
  20 * 256 + 128, 60 * 256 + 128, 100 * 256 + 128   // dithering ASK n/n+1
};
static constexpr uint32_t CAL_N = 129 + sizeof(CAL_EXTRA_Q8) / sizeof(CAL_EXTRA_Q8[0]);
static bool     calDly   = false;
static uint32_t calPhase = 0;
static uint32_t calPulse = 0;   // CAL P n: impuls K+1 na n cykli co 100 us

static bool calStart(bool dly) {
  const int32_t c = tuneFor(gModKHz - 100, gModKHz + 100);
  if (!c) return false;
  modQc = khzToQ16((int32_t)gModKHz - c);
  calDly = dly;
  calPhase = 0;
  auNext = ESP.getCycleCount();
  mode = MODE_CAL;
  return true;
}

// nastepny segment: q (K w Q16), aq (ASK w Q16, 128 = OFF), czas w us
static uint32_t calSegment(int32_t& q, int32_t& aq) {
  q = modQc;
  aq = 0;
  if (calDly) {
    if (calPhase++ & 1) {
      q += 65536;
      aq = 20 << 16;
    }
    return 250;
  }
  const uint32_t p = calPhase;
  calPhase = p + 1 >= 2 + 2 * CAL_N ? 0 : p + 1;
  if (p == 0) {
    aq = 128 << 16;
    return 4000;
  }
  if (p == 1) return 4000;
  const uint32_t i = (p - 2) / 2;
  if (!((p - 2) & 1)) {
    aq = (int32_t)(i < 129 ? i << 8 : CAL_EXTRA_Q8[i - 129]) << 8;
  }
  return 500;
}

static IRAM_ATTR __attribute__((noinline)) void calPulseTick() {
  const uint32_t mhz = ESP.getCpuFreqMHz();
  const int32_t k = modQc >> 16;
  const uint32_t base = (rd32(TONE1) & ~(K_MASK | SCALE_MASK)) | GATE_MASK;
  const uint32_t w0 = base | ((uint32_t)k & K_MASK);
  const uint32_t w1 = base | ((uint32_t)(k + 1) & K_MASK);
  const uint32_t width = calPulse;
  if ((int32_t)(ESP.getCycleCount() - auNext) > (int32_t)(1000 * mhz)) {
    auNext = ESP.getCycleCount();
  }
  for (uint8_t i = 0; i < 10; ++i) {  // 10 x 100 us, sztywny harmonogram
    auNext += 100 * mhz;
    while ((int32_t)(ESP.getCycleCount() - auNext) < 0) {}
    const uint32_t ps = xt_rsil(15);
    const uint32_t t0 = ESP.getCycleCount();
    wr32(TONE1, w1);
    while (ESP.getCycleCount() - t0 < width) {}
    wr32(TONE1, w0);
    memw();
    xt_wsr_ps(ps);
  }
}

static void calTick() {
  if (calPulse) {
    calPulseTick();
    return;
  }
  const uint32_t mhz = ESP.getCpuFreqMHz();
  if ((int32_t)(ESP.getCycleCount() - auNext) > (int32_t)(8000 * mhz)) {
    auNext = ESP.getCycleCount();
  }
  const uint32_t tEnd = ESP.getCycleCount() + 1000 * mhz;
  do {
    int32_t q, aq;
    auNext += calSegment(q, aq) * mhz;
    ditherBurst(q, aq, auNext);
  } while ((int32_t)(auNext - tEnd) < 0);
}

// ---------------------------------------------------------------- Stream
// SSB z probek liczonych na PC (audio/stream_ssb.py) i pchanych przez USB.
// Po "STREAM f" UART przechodzi na ST_BAUD. Pakiet: A5 5A, 64 x (obwiednia
// u8: ASK w 0.5 kroku 0..254, 255 = bramka OFF; czestotliwosc i16 LE: offset
// K w Q16 wzgledem nosnej f), XOR 192 bajtow. Modulator jak przy SSB z
// flasha (przerwania wylaczone, bez przerw w fazie), FIFO UART czytane
// wprost z rejestrow miedzy probkami. Co 64 probki raport do PC: B7, stan
// bufora / 16. Koniec: 0.5 s bez danych -> powrot na 115200.
static constexpr uint32_t ST_BAUD = 2000000;
static constexpr uint32_t ST_RATE = 32000;
static constexpr uint16_t ST_BUF  = 4096;       // probek (128 ms)
static constexpr uint16_t ST_START = 1024;      // start odtwarzania od 32 ms zapasu
static uint32_t stBuf[ST_BUF];                  // (obwiednia << 16) | czestotliwosc
static uint16_t stHead = 0, stTail = 0;
static uint8_t  stState = 0, stSum = 0;
static uint16_t stPos = 0;
static uint8_t  stBlk[192];
static uint32_t stBlocks = 0, stBad = 0, stOvf = 0, stUnder = 0, stFifo = 0;

static inline uint16_t stFill() { return (uint16_t)(stHead - stTail) & (ST_BUF - 1); }

static void stParse(uint8_t b) {
  switch (stState) {
    case 0:
      if (b == 0xA5) stState = 1;
      break;
    case 1:
      stState = b == 0x5A ? 2 : (b == 0xA5 ? 1 : 0);
      stPos = 0;
      stSum = 0;
      break;
    case 2:
      stBlk[stPos++] = b;
      stSum ^= b;
      if (stPos == sizeof(stBlk)) stState = 3;
      break;
    default:
      stState = 0;
      if (b != stSum) {
        ++stBad;
        break;
      }
      ++stBlocks;
      for (uint16_t i = 0; i < sizeof(stBlk); i += 3) {
        const uint16_t nh = (stHead + 1) & (ST_BUF - 1);
        if (nh == stTail) {
          ++stOvf;
          continue;
        }
        stBuf[stHead] = ((uint32_t)stBlk[i] << 16) | stBlk[i + 1] | ((uint32_t)stBlk[i + 2] << 8);
        stHead = nh;
      }
  }
}

static void streamRun(uint32_t fkhz) {
  const int32_t c = tuneFor(fkhz - 5, fkhz + 5);
  if (!c) return;
  modeStop();
  gModKHz = fkhz;
  modQc = khzToQ16((int32_t)fkhz - c);
  stHead = stTail = 0;
  stState = 0;
  stBlocks = stBad = stOvf = stUnder = stFifo = 0;
  Serial.printf("OK STREAM %lu\r\n", (unsigned long)ST_BAUD);
  Serial.flush();
  delay(20);
  Serial.updateBaudRate(ST_BAUD);
  while (Serial.available()) Serial.read();

  const uint32_t mhz = ESP.getCpuFreqMHz();
  const uint32_t cps = mhz * 1000000u / ST_RATE;
  const uint32_t ps = xt_rsil(15);
  uint32_t next = ESP.getCycleCount();
  uint32_t lastRx = next;
  bool got = false, playing = false;
  uint32_t samples = 0;
  for (uint32_t n = 0;; ++n) {
    uint32_t cnt = (USS(0) >> USRXC) & 0xFF;
    if (cnt) {
      lastRx = ESP.getCycleCount();
      got = true;
    }
    while (cnt--) stParse(USF(0));

    next += cps;
    int32_t q = modQc, aq = 128 << 16;  // cisza: bramka OFF
    if (!playing && stFill() >= ST_START) playing = true;
    if (playing) {
      if (stTail == stHead) {
        ++stUnder;
        playing = false;  // niedobor: cisza az bufor znow sie napelni
      } else {
        const uint32_t v = stBuf[stTail];
        stTail = (stTail + 1) & (ST_BUF - 1);
        const uint32_t e = v >> 16;
        if (e < 255) aq = (int32_t)e << 15;
        q = modQc + (int16_t)(v & 0xFFFF);
        ++samples;
      }
    }
    ditherBurst(q, aq, next);

    if ((n & 63) == 0) {
      ESP.wdtFeed();
      if (((USS(0) >> USTXC) & 0xFF) < 120) {
        USF(0) = 0xB7;
        USF(0) = stFill() >> 4;
      }
      if (USIS(0) & (1 << UIOF)) {
        ++stFifo;
        USIC(0) = 1 << UIOF;
      }
    }
    const uint32_t idle = (got ? 500u : 3000u) * 1000u * mhz;
    if (!playing && ESP.getCycleCount() - lastRx > idle) break;
  }
  xt_wsr_ps(ps);

  txStop();  // koniec strumienia: nadajnik wylaczony
  delay(50);
  Serial.updateBaudRate(115200);
  while (Serial.available()) Serial.read();
  Serial.printf("STREAM END probek=%lu (%lu.%02lu s) pakietow=%lu blad_sumy=%lu "
                "przepelnienie=%lu niedobor=%lu fifo_uart=%lu\r\n",
                (unsigned long)samples, (unsigned long)(samples / ST_RATE),
                (unsigned long)(samples % ST_RATE * 100 / ST_RATE),
                (unsigned long)stBlocks, (unsigned long)stBad, (unsigned long)stOvf,
                (unsigned long)stUnder, (unsigned long)stFifo);
}

// ---------------------------------------------------------------- RX IQ
// Odbior przez estymator IQ odbiornika (ROM: rom_iq_est_enable/disable,
// sterowanie 0x6000057C, bit31 = gotowe). Po n+1 probkach ADC rejestry
// 0x600005DC/0x600005E0 trzymaja sumy I i Q (ze znakiem, << 6), 0x600005E4
// moc. rom_dc_iq_est dzieli je przez n+1 - srednia zespolona okna, czyli
// waskopasmowa probka IQ wokol LO. "IQ f n tryb" (f = kanal albo MHz): 1024 pomiary
// z czasem w cyklach CPU do stBuf, potem binarnie: "IQDATA 1024\n" +
// 1024 x (t u32, I i32, Q i32, P u32) LE.
using IqEnFn  = void(*)(uint32_t, uint32_t);
using IqDisFn = void(*)(void);
static IqEnFn  iq_est_enable  = reinterpret_cast<IqEnFn>(0x40006430u);
static IqDisFn iq_est_disable = reinterpret_cast<IqDisFn>(0x40006400u);
static constexpr uint32_t IQ_SUM_I = 0x600005DCu;
static constexpr uint32_t IQ_SUM_Q = 0x600005E0u;
static constexpr uint32_t IQ_POW   = 0x600005E4u;
static constexpr uint16_t IQ_N     = ST_BUF / 4;

// Strojenie poza srodki kanalow (libphy.a, SDK 2.2.x): set_rf_freq_offset
// (xtal, MHz, off) liczy PLL na MHz + off/1024 (ram_rfpll_set_freq: F =
// 0.75 * xtal * (N + frac/65536)), wpisuje SDM i czeka na kalibracje PLL.
// xtal: 0 = 40, 1 = 26, 2 = 24 MHz (chip6_phy_init_ctrl[1]). Oficjalna
// sciezka chip_v6_set_chan_offset sie nie nadaje: chip_60_set_channel obcina
// offset do +-300 (~293 kHz) i zeruje go, gdy freq_correct_en w init data
// jest wylaczone (domyslnie). Najpierw kanal (kalibracje RX dla najblizszego
// srodka), potem PLL; txStart wraca na srodek kanalu (rxUntune).
extern "C" void set_rf_freq_offset(int xtal, int mhz, int off);
extern "C" uint8_t chip6_phy_init_ctrl[];
static bool rxOffSet = false;

static void rxUntune() {
  if (!rxOffSet) return;
  set_rf_freq_offset(chip6_phy_init_ctrl[1], chMHz(gChannel), 0);
  rxOffSet = false;
}

// najblizszy kanal 1..13 (14 blokuje domyslny kraj) + offset PLL; wypisuje
// "RXF <Hz> CH <n> OFF <off>"
static bool rxTune(uint32_t fkhz) {
  const uint8_t ch = (uint8_t)constrain(((int32_t)fkhz - 2407000 + 2500) / 5000, 1, 13);
  const int32_t d = (int32_t)fkhz - chMHz(ch) * 1000;
  const int32_t off = (d * 1024 + (d < 0 ? -500 : 500)) / 1000;
  if (!phyStart() || !setChannel(ch)) return false;
  set_rf_freq_offset(chip6_phy_init_ctrl[1], chMHz(ch), off);
  rxOffSet = off != 0;
  Serial.printf("RXF %lu CH %u OFF %ld\r\n",
                (unsigned long)(chMHz(ch) * 1000000u + (int32_t)(off * 1000000LL / 1024)),
                ch, (long)off);
  return true;
}

// Wzmocnienie odbiornika: pola PBUS w trybie debug (rom_pbus_debugmode, RX
// zostaje wlaczony; NIE rom_pbus_enter_debugmode - ten wylacza RX), wtedy AGC
// nie moze ich zmienic. PBUS(3,1) = stopien RF (LNA/mieszacz), 7 bitow w
// odwrotnej kolejnosci niz "termometr" rf: rf 0..6 = 0x00,0x40,0x60,...,0x7F,
// poziom szumu -14 -> +6 dB (~21 dB), S/N slabego SSB 29..32 dB na kazdym
// kroku. PBUS(3,2) bity 5..3 = VGA pasma podstawowego, 8 krokow, ~1 dB. Celowo bez rom_pbus_set_rxgain (przepisuje tez PBUS(2,1) -
// wlaczniki toru RX, gasi LNA) i bez rom_pbus_exit_debugmode (robi "TX off" i
// zostawia PBUS(2,1) = 0x184 zamiast 0x1FE). AGC sam: bez sygnalu rf 2 vga 2,
// przy silnym rf 0 vga 5. rf < 0 = powrot do AGC (rom_pbus_workmode).
static void rxGain(int rf, int bb) {
  using V0 = void(*)(void);
  using RdFn = uint32_t(*)(uint32_t, uint32_t);
  using WrFn = void(*)(uint32_t, uint32_t, uint32_t);
  const RdFn pbus_rd = reinterpret_cast<RdFn>(0x400074D8u);
  const WrFn pbus_force = reinterpret_cast<WrFn>(0x4000747Cu);
  if (rf < 0) {
    if (rd32(PBUS_CMD) & 1u) reinterpret_cast<V0>(0x40007648u)();  // rom_pbus_workmode
    return;
  }
  static const uint8_t RF_31[7] = {0x00, 0x40, 0x60, 0x70, 0x78, 0x7C, 0x7F};
  reinterpret_cast<V0>(0x4000737Cu)();  // rom_pbus_debugmode (pomija, gdy juz jest)
  pbus_force(3, 1, RF_31[rf < 6 ? rf : 6]);
  pbus_force(3, 2, (pbus_rd(3, 2) & 0x1C7u) | (uint32_t)(bb & 7) << 3);
}

// "ch" 1..14 (srodek kanalu) albo czestotliwosc w MHz, np. 2414.5
static bool parseRxFreq(const String& s, uint32_t& fkhz) {
  long ch = 0;
  if (parseArg(s, 1, 14, ch)) {
    fkhz = (uint32_t)chMHz((uint8_t)ch) * 1000u;
    return true;
  }
  return parseKHz(s, fkhz);
}

// Wewnetrzna magistrala I2C bloków analogowych (ROM). Bloki wg wywolan w
// libphy.a: 97 tor RX, 98 PLL RF (reg 3 = wlaczniki: sen 0x01, praca 0xF1),
// 101 (sen 0x06, praca 0xC6), 103 BBPLL (zegar CPU - nie ruszac), 108 SAR,
// 119 styk analog/cyfra. "I2C blk host reg [val]".
using I2cRdFn = uint8_t(*)(uint8_t, uint8_t, uint8_t);
using I2cWrFn = void(*)(uint8_t, uint8_t, uint8_t, uint8_t);
static I2cRdFn i2c_rd = reinterpret_cast<I2cRdFn>(0x40007268u);
static I2cWrFn i2c_wr = reinterpret_cast<I2cWrFn>(0x400072d8u);

// f = 0: bez strojenia (zostaja ustawienia z I2C)
static void iqCapture(uint32_t fkhz, uint32_t n, uint32_t mode) {
  txStop();
  modeStop();
  if (fkhz && !rxTune(fkhz)) {
    Serial.println(F("ERR TUNE"));
    return;
  }
  delay(20);
  const uint32_t ps = xt_rsil(15);
  for (uint16_t i = 0; i < IQ_N; ++i) {
    iq_est_enable(mode, n);
    stBuf[4 * i]     = ESP.getCycleCount();
    stBuf[4 * i + 1] = rd32(IQ_SUM_I);
    stBuf[4 * i + 2] = rd32(IQ_SUM_Q);
    stBuf[4 * i + 3] = rd32(IQ_POW);
    iq_est_disable();
  }
  xt_wsr_ps(ps);
  Serial.printf("IQDATA %u\n", IQ_N);
  Serial.write(reinterpret_cast<const uint8_t*>(stBuf), sizeof(stBuf));
  Serial.flush();
  Serial.println();
}

// Eksperyment: czasy estymatora. "IQT n wariant": 1024 pomiary jak "IQ 0",
// ale w polu P zapisuje ccount gotowosci. Wariant 0 = ROM enable/disable,
// 1 = ta sama sekwencja wpisana bezposrednio, 2 = bit 0 wlaczony raz, potem
// tylko start (bit 1) / odczyt / kasowanie startu.
static constexpr uint32_t IQ_CTRL = 0x6000057Cu;
static constexpr uint32_t IQ_MASK = 0xFFFA0001u;
static void IRAM_ATTR __attribute__((noinline)) iqTest(uint32_t n, uint32_t variant) {
  const uint32_t ps = xt_rsil(15);
  if (variant == 2) wr32(IQ_CTRL, rd32(IQ_CTRL) | 1u);
  for (uint16_t i = 0; i < IQ_N; ++i) {
    const uint32_t t0 = ESP.getCycleCount();
    if (variant == 0) {
      iq_est_enable(0, n);
    } else if (variant == 1) {
      wr32(IQ_CTRL, rd32(IQ_CTRL) | 1u);
      wr32(IQ_CTRL, (rd32(IQ_CTRL) & IQ_MASK) | n << 2 | 2u);
      while (!(rd32(IQ_CTRL) & 0x80000000u)) {}
    } else {
      wr32(IQ_CTRL, (rd32(IQ_CTRL) & IQ_MASK) | n << 2 | 2u);
      while (!(rd32(IQ_CTRL) & 0x80000000u)) {}
    }
    const uint32_t t1 = ESP.getCycleCount();
    stBuf[4 * i] = t0;
    stBuf[4 * i + 1] = rd32(IQ_SUM_I);
    stBuf[4 * i + 2] = rd32(IQ_SUM_Q);
    stBuf[4 * i + 3] = t1;
    if (variant == 0) {
      iq_est_disable();
    } else if (variant == 1) {
      wr32(IQ_CTRL, (rd32(IQ_CTRL) & IQ_MASK) | 0x1000u);
      wr32(IQ_CTRL, rd32(IQ_CTRL) & ~1u);
    } else {
      wr32(IQ_CTRL, (rd32(IQ_CTRL) & IQ_MASK) | n << 2);
    }
  }
  if (variant == 2) wr32(IQ_CTRL, (rd32(IQ_CTRL) & IQ_MASK & ~1u) | 0x1000u);
  xt_wsr_ps(ps);
  Serial.printf("IQDATA %u\n", IQ_N);
  Serial.write(reinterpret_cast<const uint8_t*>(stBuf), sizeof(stBuf));
  Serial.flush();
  Serial.println();
}

// "IQSTREAM f k tryb": ciagly odbior, okno n+1 = 2^k probek ADC (40 MS/s),
// srednia I/Q = suma >> k jako i16. UART na ST_BAUD; co 16 probek naglowek
// A5 5A + ccount 24 bity LE (z niego PC liczy fs i uklada paczki na osi
// czasu), potem 16 x (I, Q) i16 LE. Male paczki: zgubiony bajt (CH340) psuje
// tylko 16 probek. Gdy FIFO nie ma miejsca, czeka (licznik przestojow).
// Bajt od PC: 0xFF = AGC, 0x80 | rf << 3 | vga = wzmocnienie reczne (rf 0..6,
// vga 0..7, bez przerwy w strumieniu), inny konczy strumien (powrot na 115200
// + podsumowanie).
static void iqStream(uint32_t fkhz, uint32_t k, uint32_t mode) {
  txStop();
  modeStop();
  if (!rxTune(fkhz)) {
    Serial.println(F("ERR TUNE"));
    return;
  }
  const uint32_t n = (1u << k) - 1;
  Serial.printf("OK IQSTREAM %lu\r\n", (unsigned long)ST_BAUD);
  Serial.flush();
  delay(20);
  Serial.updateBaudRate(ST_BAUD);
  // 2 bity stopu: ciagly strumien 1-bitowy CH340 gubil co ~7 kB bajt
  USC0(0) = (USC0(0) & ~(3u << UCSBN)) | (3u << UCSBN);
  delay(20);
  while (Serial.available()) Serial.read();

  uint32_t samples = 0, stalls = 0;
  const uint32_t ps = xt_rsil(15);
  for (uint32_t i = 0;; ++i) {
    iq_est_enable(mode, n);
    const int32_t si = (int32_t)rd32(IQ_SUM_I) >> k;
    const int32_t sq = (int32_t)rd32(IQ_SUM_Q) >> k;
    iq_est_disable();
    const int16_t vi = (int16_t)constrain(si, -32768, 32767);
    const int16_t vq = (int16_t)constrain(sq, -32768, 32767);
    const bool head = (i & 15) == 0;
    const uint32_t need = head ? 9 : 4;
    if (128 - ((USS(0) >> USTXC) & 0xFF) < need) {
      ++stalls;
      while (128 - ((USS(0) >> USTXC) & 0xFF) < need) {}
    }
    if (head) {
      const uint32_t t = ESP.getCycleCount();
      USF(0) = 0xA5; USF(0) = 0x5A;
      USF(0) = t; USF(0) = t >> 8; USF(0) = t >> 16;
    }
    USF(0) = vi; USF(0) = vi >> 8; USF(0) = vq; USF(0) = vq >> 8;
    ++samples;
    if ((i & 1023) == 0) ESP.wdtFeed();
    if ((USS(0) >> USRXC) & 0xFF) {
      const uint8_t c = USF(0);
      if (!(c & 0x80)) break;              // PC konczy
      if (c == 0xFF) rxGain(-1, 0);
      else rxGain((c >> 3) & 7, c & 7);
    }
  }
  xt_wsr_ps(ps);
  delay(50);
  USC0(0) = (USC0(0) & ~(3u << UCSBN)) | (1u << UCSBN);
  Serial.updateBaudRate(115200);
  while (Serial.available()) Serial.read();
  Serial.printf("IQSTREAM END probek=%lu przestojow_fifo=%lu\r\n",
                (unsigned long)samples, (unsigned long)stalls);
}

// ---------------------------------------------------------------- RX low-IF
// "IQLIF f if D": odbior z LO przesunietym o if Hz (LO = f - if, domyslnie
// 100 kHz), zeby kreska DC, garb szumu przy LO i spur -0.75 kHz lezaly poza
// pasmem. Estymator z krotkim oknem 32 probek ADC (0.8 us, sterowany
// bezposrednio: start, w czasie pomiaru (~1.2 us) obrobka poprzedniej probki,
// odczyt) daje ~550 kS/s. Na CPU: odjecie DC (IIR), mnozenie przez
// exp(-j*2*pi*f_nco*t) z faza z ccount (CPU i ADC na wspolnym kwarcu, wiec
// nierowne odstepy probek nie psuja fazy), CIC 2. rzedu z decymacja /D
// (domyslnie 15 -> ~36 kS/s). Wyjscie jak IQSTREAM: co 16 probek A5 5A +
// ccount 24 bity, 16 x (I, Q) i16 LE. Bajty od PC: 0xFF = AGC, 0x80 | rf << 3
// | vga = wzmocnienie, 0xFE + 4 bajty (int32 LE) = nowe K NCO (f_nco = K *
// 160 MHz / 2^32) bez przerwy w strumieniu, inny bajt konczy.
static int16_t lifSin[1024];

static void lifTable() {
  if (lifSin[256]) return;
  for (int i = 0; i < 1024; ++i) lifSin[i] = (int16_t)lroundf(16383.0f * sinf(i * 6.2831853f / 1024));
}

static int32_t lifK(double fHz) {
  return (int32_t)llround(fHz * 4294967296.0 / ((double)ESP.getCpuFreqMHz() * 1e6));
}

static void IRAM_ATTR __attribute__((noinline)) lifLoop(int32_t k0, uint32_t dec, uint32_t* samples, uint32_t* stalls) {
  const uint32_t nsum = 31;
  const uint32_t shift = 5 + (uint32_t)ceilf(2 * log2f((float)dec));
  uint32_t k = (uint32_t)k0;
  int32_t dci = 0, dcq = 0;                 // DC w Q8
  uint32_t i1 = 0, i2 = 0, q1 = 0, q2 = 0;  // integratory CIC (modulo 2^32)
  uint32_t pi2 = 0, pq2 = 0, pci = 0, pcq = 0;
  int32_t xi = 0, xq = 0;
  uint32_t tprev = 0, cnt = 0, nout = 0;
  bool have = false;
  wr32(IQ_CTRL, rd32(IQ_CTRL) | 1u);
  for (uint32_t loop = 0;; ++loop) {
    const uint32_t t0 = ESP.getCycleCount();
    wr32(IQ_CTRL, (rd32(IQ_CTRL) & IQ_MASK) | nsum << 2 | 2u);
    if (have) {                             // poprzednia probka w czasie pomiaru
      dci += ((xi << 8) - dci) >> 11;
      dcq += ((xq << 8) - dcq) >> 11;
      const int32_t ri = xi - (dci >> 8), rq = xq - (dcq >> 8);
      const uint32_t ph = (tprev * k) >> 22;
      const int32_t sn = lifSin[ph], cs = lifSin[(ph + 256) & 1023];
      i1 += (uint32_t)((ri * cs + rq * sn) >> 4);
      q1 += (uint32_t)((rq * cs - ri * sn) >> 4);
      i2 += i1;
      q2 += q1;
      if (++cnt == dec) {
        cnt = 0;
        const uint32_t ci = i2 - pi2, cq = q2 - pq2;
        pi2 = i2;
        pq2 = q2;
        int32_t oi = (int32_t)(ci - pci) >> shift, oq = (int32_t)(cq - pcq) >> shift;
        pci = ci;
        pcq = cq;
        oi = oi > 32767 ? 32767 : oi < -32768 ? -32768 : oi;
        oq = oq > 32767 ? 32767 : oq < -32768 ? -32768 : oq;
        const bool head = (nout & 15) == 0;
        const uint32_t need = head ? 9 : 4;
        if (128 - ((USS(0) >> USTXC) & 0xFF) < need) {
          ++*stalls;
          while (128 - ((USS(0) >> USTXC) & 0xFF) < need) {}
        }
        if (head) {
          const uint32_t t = ESP.getCycleCount();
          USF(0) = 0xA5; USF(0) = 0x5A;
          USF(0) = t; USF(0) = t >> 8; USF(0) = t >> 16;
        }
        USF(0) = oi; USF(0) = oi >> 8; USF(0) = oq; USF(0) = oq >> 8;
        ++nout;
        ++*samples;
      }
    }
    while (!(rd32(IQ_CTRL) & 0x80000000u)) {}
    xi = (int32_t)rd32(IQ_SUM_I) >> 8;      // srednia z 32 probek ADC x8
    xq = (int32_t)rd32(IQ_SUM_Q) >> 8;
    wr32(IQ_CTRL, (rd32(IQ_CTRL) & IQ_MASK) | nsum << 2);
    tprev = t0;
    have = true;
    if ((loop & 4095) == 0) ESP.wdtFeed();
    if ((USS(0) >> USRXC) & 0xFF) {
      const uint8_t c = USF(0);
      if (!(c & 0x80)) break;
      if (c == 0xFF) {
        rxGain(-1, 0);
      } else if (c == 0xFE) {
        uint32_t nk = 0;
        for (int b = 0; b < 4; ++b) {
          const uint32_t tw = ESP.getCycleCount();
          while (!((USS(0) >> USRXC) & 0xFF)) {
            if (ESP.getCycleCount() - tw > 160000) break;   // 1 ms
          }
          nk |= (uint32_t)(uint8_t)USF(0) << (8 * b);
        }
        k = nk;
      } else {
        rxGain((c >> 3) & 7, c & 7);
      }
    }
  }
  wr32(IQ_CTRL, (rd32(IQ_CTRL) & IQ_MASK & ~1u) | 0x1000u);
}

static void iqLif(uint32_t fkhz, int32_t ifHz, uint32_t dec) {
  txStop();
  modeStop();
  const int32_t lokhz = (int32_t)fkhz - ifHz / 1000;
  if (!rxTune((uint32_t)lokhz)) {
    Serial.println(F("ERR TUNE"));
    return;
  }
  lifTable();
  const uint8_t ch = (uint8_t)constrain(((int32_t)lokhz - 2407000 + 2500) / 5000, 1, 13);
  // LO faktyczne jak w rxTune: kanal + off/1024 MHz
  const int32_t d = lokhz - chMHz(ch) * 1000;
  const int32_t o = (d * 1024 + (d < 0 ? -500 : 500)) / 1000;
  const double loHz = chMHz(ch) * 1e6 + o * 1e6 / 1024;
  const double nco = fkhz * 1e3 - loHz;
  const int32_t k = lifK(nco);
  Serial.printf("OK IQLIF %lu NCO %ld K %ld\r\n", (unsigned long)ST_BAUD, (long)lround(nco), (long)k);
  Serial.flush();
  delay(20);
  Serial.updateBaudRate(ST_BAUD);
  USC0(0) = (USC0(0) & ~(3u << UCSBN)) | (3u << UCSBN);
  delay(20);
  while (Serial.available()) Serial.read();
  uint32_t samples = 0, stalls = 0;
  const uint32_t ps = xt_rsil(15);
  lifLoop(k, dec, &samples, &stalls);
  xt_wsr_ps(ps);
  delay(50);
  USC0(0) = (USC0(0) & ~(3u << UCSBN)) | (1u << UCSBN);
  Serial.updateBaudRate(115200);
  while (Serial.available()) Serial.read();
  Serial.printf("IQLIF END probek=%lu przestojow_fifo=%lu\r\n",
                (unsigned long)samples, (unsigned long)stalls);
}

// ---------------------------------------------------------------- Morse

static const char* morse(char c) {
  static const char* const letters[26] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..",
    ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.",
    "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--.."
  };
  static const char* const digits[10] = {
    "-----", ".----", "..---", "...--", "....-",
    ".....", "-....", "--...", "---..", "----."
  };
  if (c >= 'A' && c <= 'Z') return letters[c - 'A'];
  if (c >= '0' && c <= '9') return digits[c - '0'];
  switch (c) {
    case '/': return "-..-.";
    case '?': return "..--..";
    case '.': return ".-.-.-";
    case ',': return "--..--";
    case '=': return "-...-";
    default:  return nullptr;
  }
}

// false = przerwane znakiem z Seriala (CR/LF po komendzie ignorowane)
static bool keyWait(uint32_t ms) {
  const uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < ms) {
    while (Serial.available()) {
      const int c = Serial.peek();
      if (c != '\r' && c != '\n') return false;
      Serial.read();
    }
    delay(1);
  }
  return true;
}

static void sendCw(const String& text) {
  const bool wasOn = txOn;
  if (!txStart()) {
    Serial.println(F("ERR TX START"));
    return;
  }
  gate(false);

  const uint32_t dit = 1200u / gWpm;
  bool ok = keyWait(3 * dit);

  for (size_t i = 0; ok && i < text.length(); ++i) {
    const char c = text[i];
    if (c == ' ') {
      ok = keyWait(4 * dit);  // 3 dit po znaku + 4 = odstep 7 dit
      continue;
    }
    const char* p = morse(c);
    if (!p) continue;
    for (; ok && *p; ++p) {
      gate(true);
      ok = keyWait(*p == '.' ? dit : 3 * dit);
      gate(false);
      if (ok) ok = keyWait(dit);
    }
    if (ok) ok = keyWait(2 * dit);
  }

  if (!ok) {
    while (Serial.available()) Serial.read();
    Serial.println(F("CW ABORT"));
  } else {
    Serial.println(F("CW DONE"));
  }

  if (wasOn) {
    gate(true);
  } else {
    txStop();
  }
}

// ---------------------------------------------------------------- Serial

static void printStatus() {
  Serial.printf(
    "TX=%s CH=%u (%d MHz) K=%u ASK=%u APWR=%u TIME=%lus WPM=%u SLOT=%08lX\r\n",
    txOn ? "ON" : "OFF", gChannel, chMHz(gChannel), gK, gAsk, gApwr,
    (unsigned long)gTimeoutS, gWpm, (unsigned long)rd32(TONE1)
  );
  Serial.printf(
    "SWEEP%s %lu.%03lu-%lu.%03lu MHz PERIOD=%lums\r\n",
    mode == MODE_SWEEP ? "=ON" : "",
    (unsigned long)(gSweepLo / 1000), (unsigned long)(gSweepLo % 1000),
    (unsigned long)(gSweepHi / 1000), (unsigned long)(gSweepHi % 1000),
    (unsigned long)gPeriodMs
  );
  Serial.printf(
    "%s %lu.%03lu MHz DEV=%luHz DEPTH=%u%% AUDIO=%lu/%lu probek @%luHz\r\n",
    mode == MODE_FM ? "FM=ON" : mode == MODE_AM ? "AM=ON" :
    mode == MODE_USB ? "USB=ON" : mode == MODE_LSB ? "LSB=ON" : "FM/AM/SSB",
    (unsigned long)(gModKHz / 1000), (unsigned long)(gModKHz % 1000),
    (unsigned long)gDevHz, gDepth, (unsigned long)auPos,
    (unsigned long)(isSsb(mode) ? SSB_LEN : AUDIO_LEN), (unsigned long)auRate
  );
  if (sdCycles) {
    Serial.printf("DITHER=%lukHz\r\n", (unsigned long)(
      (uint64_t)sdIters * ESP.getCpuFreqMHz() * 1000u / sdCycles));
    sdIters = 0;
    sdCycles = 0;
  }
}

static void printHelp() {
  Serial.println(F("ON | OFF | CH 1..14 | K 0..1023 | ASK 0..63 | APWR 0..255"));
  Serial.println(F("TIME s (0=bez limitu) | WPM 5..40 | CW tekst | STATUS | ?"));
  Serial.println(F("SWEEP a b (MHz) | SWEEP ON | SWEEP OFF | PERIOD 20..600000 ms"));
  Serial.println(F("FM f (MHz) | FM ON | FM OFF | DEV 100..75000 Hz"));
  Serial.println(F("AM f (MHz) | AM ON | AM OFF | DEPTH 0..95 %"));
  Serial.println(F("USB f | LSB f (MHz, czestotliwosc nosnej) | USB/LSB ON | OFF"));
  Serial.println(F("CAL AM | CAL DLY | CAL OFF (pomiar toru pod SSB)"));
  Serial.println(F("STREAM f (MHz) - SSB z probek z PC przez USB (audio/stream_ssb.py)"));
  Serial.println(F("IQ f n tryb | IQSTREAM f k tryb - odbior IQ, f = kanal albo MHz (audio/rx_iq.py)"));
  Serial.println(F("GAIN AGC | GAIN rf 0..6 vga 0..7 - wzmocnienie odbiornika"));
  Serial.println(F("IQLIF f [if_Hz [D]] - odbior low-IF (LO = f - if, NCO w ESP)"));
}

static bool parseArg(const String& s, long lo, long hi, long& out) {
  if (s.length() == 0) return false;
  for (size_t i = 0; i < s.length(); ++i) {
    if (!isDigit(s[i])) return false;
  }
  out = s.toInt();
  return out >= lo && out <= hi;
}

// "2434.75" -> 2434750 kHz
static bool parseKHz(const String& s, uint32_t& out) {
  const int dot = s.indexOf('.');
  const String ip = dot < 0 ? s : s.substring(0, dot);
  String fp = dot < 0 ? String() : s.substring(dot + 1);
  long mhz = 0;
  if (!parseArg(ip, 2300, 2600, mhz) || fp.length() > 3) return false;
  for (size_t i = 0; i < fp.length(); ++i) {
    if (!isDigit(fp[i])) return false;
  }
  while (fp.length() < 3) fp += '0';
  out = (uint32_t)mhz * 1000u + (uint32_t)fp.toInt();
  return true;
}

static void handle(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  Serial.print(F("> "));
  Serial.println(cmd);

  const int sp = cmd.indexOf(' ');
  const String name = sp < 0 ? cmd : cmd.substring(0, sp);
  String arg = sp < 0 ? String() : cmd.substring(sp + 1);
  arg.trim();
  long v = 0;

  if (name == "ON") {
    if (!txStart()) { Serial.println(F("ERR TX START")); return; }
    printStatus();
  } else if (name == "OFF") {
    txStop();
    modeStop();
    Serial.println(F("OK RF OFF"));
  } else if (name == "CH") {
    if (!parseArg(arg, 1, 14, v)) { Serial.println(F("ERR CH 1..14")); return; }
    modeStop();
    const bool wasOn = txOn;
    txStop();
    if (!phyStart() || !setChannel((uint8_t)v)) {
      Serial.println(F("ERR SET CHANNEL"));
      return;
    }
    if (wasOn && !txStart()) Serial.println(F("ERR TX START"));
    printStatus();
  } else if (name == "K") {
    if (!parseArg(arg, 0, 1023, v)) { Serial.println(F("ERR K 0..1023")); return; }
    modeStop();
    gK = (uint16_t)v;
    if (txOn) toneField(K_MASK, gK);
    printStatus();
  } else if (name == "ASK") {
    if (!parseArg(arg, 0, 63, v)) { Serial.println(F("ERR ASK 0..63")); return; }
    gAsk = (uint8_t)v;
    if (txOn) toneField(SCALE_MASK, (uint32_t)askCode(gAsk) << SCALE_SHIFT);
    printStatus();
  } else if (name == "APWR") {
    if (!parseArg(arg, 0, 255, v)) { Serial.println(F("ERR APWR 0..255")); return; }
    gApwr = (uint8_t)v;
    if (txOn) {
      // analog ustawiany raz na sesje TX - restart toru
      txStop();
      if (!txStart()) { Serial.println(F("ERR TX START")); return; }
    }
    printStatus();
  } else if (name == "TIME") {
    if (!parseArg(arg, 0, 86400, v)) { Serial.println(F("ERR TIME 0..86400")); return; }
    gTimeoutS = (uint32_t)v;
    txStartMs = millis();
    printStatus();
  } else if (name == "WPM") {
    if (!parseArg(arg, 5, 40, v)) { Serial.println(F("ERR WPM 5..40")); return; }
    gWpm = (uint8_t)v;
    printStatus();
  } else if (name == "CW") {
    if (arg.length() == 0) { Serial.println(F("ERR CW tekst")); return; }
    sendCw(arg);
  } else if (name == "SWEEP") {
    if (arg == "OFF") {
      if (mode == MODE_SWEEP) modeStop();
    } else if (arg != "ON") {
      const int sp2 = arg.indexOf(' ');
      uint32_t lo = 0, hi = 0;
      if (sp2 < 0 ||
          !parseKHz(arg.substring(0, sp2), lo) ||
          !parseKHz(arg.substring(sp2 + 1), hi) || hi <= lo) {
        Serial.println(F("ERR SWEEP a b (MHz, np. 2434.75, a<b) | ON | OFF"));
        return;
      }
      const uint32_t prevLo = gSweepLo, prevHi = gSweepHi;
      gSweepLo = lo;
      gSweepHi = hi;
      if (!sweepStart()) {
        gSweepLo = prevLo;
        gSweepHi = prevHi;
        return;
      }
    } else if (!sweepStart()) {
      return;
    }
    printStatus();
  } else if (name == "PERIOD") {
    if (!parseArg(arg, 20, 600000, v)) { Serial.println(F("ERR PERIOD 20..600000")); return; }
    gPeriodMs = (uint32_t)v;
    printStatus();
  } else if (name == "FM" || name == "AM" || name == "USB" || name == "LSB") {
    const Mode m = name == "FM" ? MODE_FM : name == "AM" ? MODE_AM :
                   name == "USB" ? MODE_USB : MODE_LSB;
    if (arg == "OFF") {
      if (mode == m) modeStop();
    } else if (arg != "ON") {
      uint32_t f = 0;
      if (!parseKHz(arg, f)) {
        Serial.println(F("ERR FM|AM|USB|LSB f (MHz, np. 2400.05) | ON | OFF"));
        return;
      }
      const uint32_t prev = gModKHz;
      gModKHz = f;
      if (!audioStart(m)) {
        gModKHz = prev;
        return;
      }
    } else if (!audioStart(m)) {
      return;
    }
    printStatus();
  } else if (name == "DEV") {
    if (!parseArg(arg, 100, 75000, v)) { Serial.println(F("ERR DEV 100..75000")); return; }
    const uint32_t prev = gDevHz;
    gDevHz = (uint32_t)v;
    if (mode == MODE_FM && !audioStart(MODE_FM)) {
      gDevHz = prev;
      return;
    }
    printStatus();
  } else if (name == "DEPTH") {
    if (!parseArg(arg, 0, 95, v)) { Serial.println(F("ERR DEPTH 0..95")); return; }
    gDepth = (uint8_t)v;
    if (mode == MODE_AM) audioStart(MODE_AM);
    printStatus();
  } else if (name == "I2C") {
    long v[4] = {-1, -1, -1, -1};
    int cnt = 0, pos = 0;
    while (cnt < 4 && pos < (int)arg.length()) {
      int e = arg.indexOf(' ', pos);
      if (e < 0) e = arg.length();
      if (!parseArg(arg.substring(pos, e), 0, 255, v[cnt])) break;
      ++cnt;
      pos = e + 1;
    }
    if (cnt < 3 || (int)arg.length() >= pos + 1) {
      Serial.println(F("ERR I2C blk host reg [val]"));
      return;
    }
    if (v[0] == 103) {
      Serial.println(F("ERR I2C 103 = BBPLL (zegar CPU)"));
      return;
    }
    const uint8_t old = i2c_rd(v[0], v[1], v[2]);
    if (cnt == 4) i2c_wr(v[0], v[1], v[2], v[3]);
    Serial.printf("I2C %ld %ld %ld = 0x%02X -> 0x%02X\r\n", v[0], v[1], v[2], old,
                  i2c_rd(v[0], v[1], v[2]));
  } else if (name == "PLL") {
    // eksperyment: PLL RF na dowolne MHz (takze poza zakresem VCO)
    long mhz = 0, off = 0;
    const int s1 = arg.indexOf(' ');
    String so = s1 < 0 ? String("0") : arg.substring(s1 + 1);
    const bool neg = so.startsWith("-");
    if (s1 < 0 || !parseArg(arg.substring(0, s1), 100, 8000, mhz) ||
        !parseArg(neg ? so.substring(1) : so, 0, 30000, off)) {
      Serial.println(F("ERR PLL mhz 100..8000 off [-]0..30000 (1/1024 MHz)"));
      return;
    }
    const uint32_t t0 = micros();
    set_rf_freq_offset(chip6_phy_init_ctrl[1], mhz, neg ? -off : off);
    rxOffSet = true;
    Serial.printf("PLL %ld MHz %+ld, %lu us, 98/7=0x%02X\r\n", mhz, neg ? -off : off,
                  (unsigned long)(micros() - t0), i2c_rd(98, 1, 7));
  } else if (name == "IQLIF") {
    // "IQLIF f [if_Hz [D]]": f jak w IQSTREAM (kanal albo MHz)
    long ifv = 100000, dec = 15;
    uint32_t f = 0;
    int s1 = arg.indexOf(' ');
    const String a0 = s1 < 0 ? arg : arg.substring(0, s1);
    String rest = s1 < 0 ? String() : arg.substring(s1 + 1);
    bool ok = parseRxFreq(a0, f);
    if (ok && rest.length()) {
      const int s2 = rest.indexOf(' ');
      String si = s2 < 0 ? rest : rest.substring(0, s2);
      const bool neg = si.startsWith("-");
      ok = parseArg(neg ? si.substring(1) : si, 20000, 400000, ifv);
      if (neg) ifv = -ifv;
      if (ok && s2 > 0) ok = parseArg(rest.substring(s2 + 1), 4, 64, dec);
    }
    if (!ok) {
      Serial.println(F("ERR IQLIF f [if 20000..400000 Hz, +-] [D 4..64]"));
      return;
    }
    iqLif(f, (int32_t)ifv, (uint32_t)dec);
  } else if (name == "IQT") {
    long n = 0, v = 0;
    const int s1 = arg.indexOf(' ');
    if (s1 < 0 || !parseArg(arg.substring(0, s1), 0, 32767, n) ||
        !parseArg(arg.substring(s1 + 1), 0, 2, v)) {
      Serial.println(F("ERR IQT n 0..32767 wariant 0..2"));
      return;
    }
    iqTest((uint32_t)n, (uint32_t)v);
  } else if (name == "GAIN") {
    long rf = 0, bb = 0;
    const int s1 = arg.indexOf(' ');
    if (arg == "AGC") {
      rxGain(-1, 0);
    } else if (s1 > 0 && parseArg(arg.substring(0, s1), 0, 6, rf) &&
               parseArg(arg.substring(s1 + 1), 0, 7, bb)) {
      rxGain(rf, bb);
    } else {
      Serial.println(F("ERR GAIN AGC | GAIN rf 0..6 vga 0..7"));
      return;
    }
    Serial.printf("GAIN %s\r\n", arg.c_str());
  } else if (name == "PB") {
    // eksperyment: PBUS przez ROM. "PB ENTER" / "PB EXIT" / "PB sel bank val" (val 0..511)
    using V0 = void(*)(void);
    using V3 = void(*)(uint32_t, uint32_t, uint32_t);
    if (arg == "ENTER") {
      reinterpret_cast<V0>(0x4000737Cu)();   // rom_pbus_debugmode (RX zostaje wlaczony)
    } else if (arg == "DUMP") {
      using R2 = uint32_t(*)(uint32_t, uint32_t);
      for (uint32_t sel = 0; sel < 8; ++sel) {
        Serial.printf("PBUS %lu:", (unsigned long)sel);
        for (uint32_t bank = 0; bank < 4; ++bank)
          Serial.printf(" %03lX", (unsigned long)reinterpret_cast<R2>(0x400074D8u)(sel, bank));  // rom_pbus_rd
        Serial.println();
      }
    } else if (arg == "EXIT") {
      reinterpret_cast<V0>(0x40007448u)();   // rom_pbus_exit_debugmode
    } else {
      long v[3];
      int pos = 0;
      for (int k = 0; k < 3; ++k) {
        int e = arg.indexOf(' ', pos);
        if (e < 0) e = arg.length();
        if (!parseArg(arg.substring(pos, e), 0, 511, v[k])) {
          Serial.println(F("ERR PB ENTER | EXIT | sel 0..7 bank 0..3 val 0..511"));
          return;
        }
        pos = e + 1;
      }
      reinterpret_cast<V3>(0x4000747Cu)(v[0] & 7, v[1] & 3, v[2]);  // rom_pbus_force_test
    }
    Serial.printf("PB OK %08lX\r\n", (unsigned long)rd32(PBUS_CMD));
  } else if (name == "RD" || name == "WR") {
    // eksperyment: rejestry 0x6000xxxx, "RD adr" / "WR adr wartosc" (hex)
    const int s1 = arg.indexOf(' ');
    char* e = nullptr;
    const uint32_t adr = strtoul(arg.substring(0, s1 < 0 ? arg.length() : s1).c_str(), &e, 16);
    if ((adr & 0xFFFF0003u) != 0x60000000u || (name == "WR" && s1 < 0)) {
      Serial.println(F("ERR RD adr | WR adr val (hex, 0x6000xxxx)"));
      return;
    }
    const uint32_t old = rd32(adr);
    if (name == "WR") wr32(adr, strtoul(arg.substring(s1 + 1).c_str(), nullptr, 16));
    Serial.printf("REG %08lX = %08lX -> %08lX\r\n", (unsigned long)adr, (unsigned long)old,
                  (unsigned long)rd32(adr));
  } else if (name == "IQ") {
    long n = 0, md = 0;
    uint32_t f = 0;
    const int s1 = arg.indexOf(' '), s2 = arg.lastIndexOf(' ');
    if (s1 < 0 || s2 <= s1 || !(arg.substring(0, s1) == "0" || parseRxFreq(arg.substring(0, s1), f)) ||
        !parseArg(arg.substring(s1 + 1, s2), 0, 32767, n) ||
        !parseArg(arg.substring(s2 + 1), 0, 1, md)) {
      Serial.println(F("ERR IQ f (kanal 1..14, MHz albo 0 = bez strojenia) n 0..32767 tryb 0..1"));
      return;
    }
    iqCapture(f, (uint32_t)n, (uint32_t)md);
  } else if (name == "IQSTREAM") {
    long k = 0, md = 0;
    uint32_t f = 0;
    const int s1 = arg.indexOf(' '), s2 = arg.lastIndexOf(' ');
    if (s1 < 0 || s2 <= s1 || !parseRxFreq(arg.substring(0, s1), f) ||
        !parseArg(arg.substring(s1 + 1, s2), 5, 14, k) ||
        !parseArg(arg.substring(s2 + 1), 0, 1, md)) {
      Serial.println(F("ERR IQSTREAM f (kanal 1..14 albo MHz) k 5..14 tryb 0..1"));
      return;
    }
    iqStream(f, (uint32_t)k, (uint32_t)md);
  } else if (name == "STREAM") {
    uint32_t f = gModKHz;
    if (arg.length() && !parseKHz(arg, f)) {
      Serial.println(F("ERR STREAM f (MHz, np. 2402)"));
      return;
    }
    streamRun(f);
  } else if (name == "CAL") {
    if (arg == "OFF") {
      if (mode == MODE_CAL) modeStop();
    } else if (arg == "AM" || arg == "DLY") {
      calPulse = 0;
      if (!calStart(arg == "DLY")) return;
    } else if (arg.startsWith("P ") && parseArg(arg.substring(2), 1, 16000, v)) {
      calPulse = (uint32_t)v;
      if (!calStart(false)) return;
    } else {
      Serial.println(F("ERR CAL AM | DLY | P cykle | OFF"));
      return;
    }
    printStatus();
  } else if (name == "BURST") {
    if (!parseArg(arg, 1, 2000, v)) { Serial.println(F("ERR BURST 1..2000 ms")); return; }
    gBurstMs = (uint32_t)v;
    Serial.printf("OK BURST=%lums\r\n", (unsigned long)gBurstMs);
  } else if (name == "ASKQ") {
    if (!parseArg(arg, 0, 32768, v)) { Serial.println(F("ERR ASKQ 0..32768 (ASK*256)")); return; }
    gSweepAq = v;
    Serial.printf("OK ASKQ=%ld\r\n", (long)gSweepAq);
  } else if (name == "SDTHR") {
    if (!parseArg(arg, 1, 2000, v)) { Serial.println(F("ERR SDTHR 1..2000 (cykle)")); return; }
    sdThr = (int32_t)v << 16;
    Serial.printf("OK SDTHR=%ld\r\n", (long)v);
  } else if (name == "STATUS") {
    printStatus();
  } else if (name == "?" || name == "HELP") {
    printHelp();
  } else {
    Serial.println(F("ERR ? = pomoc"));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("BOOT CW_CARRIER_TEST"));

  // po zasileniu cisza: SSB leci ze strumienia z PC (audio/stream_ssb.py);
  // z flasha nadal na komende USB/LSB/AM/FM
  if (!phyStart()) {
    Serial.println(F("ERR PHY INIT"));
  }
  printStatus();
  printHelp();
}

void loop() {
  static String line;

  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (line.length()) {
        handle(line);
        line = "";
      }
    } else if (line.length() < 80) {
      line += c;
    }
  }

  if (txOn && mode == MODE_SWEEP) sweepTick();
  if (txOn && (mode == MODE_FM || mode == MODE_AM || isSsb(mode))) audioTick();
  if (txOn && mode == MODE_CAL) calTick();

  if (txOn && gTimeoutS &&
      (uint32_t)(millis() - txStartMs) >= gTimeoutS * 1000u) {
    txStop();
    modeStop();
    Serial.println(F("AUTO OFF (TIME)"));
  }
}
