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

static bool txStart() {
  if (txOn) return true;
  if (!phyStart()) return false;
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
