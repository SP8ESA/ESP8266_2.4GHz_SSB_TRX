# ESP8266 jako odbiornik IQ: estymator IQ odbiornika Wi-Fi

Stan na 2026-10-01. Komendy `IQ` i `IQSTREAM` w `CW_CARRIER_TEST.ino`,
skrypt na PC: `CW_CARRIER_TEST/audio/rx_iq.py`.

## Wynik w skrócie

ESP8266 daje spójne próbki IQ (z fazą, nie samą moc) wokół LO, które można
ustawić z krokiem ok. 1 kHz w zakresie co najmniej 2300–2600 MHz (offset
ułamkowego PLL względem najbliższego kanału Wi-Fi). Źródłem jest sprzętowy estymator DC/IQ odbiornika, używany
normalnie do kalibracji. Sumuje on n+1 próbek ADC odbiornika (40 MS/s), a
średnia z okna to jedna wąskopasmowa próbka zespolona. Powtarzając pomiar,
dostajemy strumień IQ do kilkuset kS/s. Przy ciągłym zrzucie przez USB jest
to ok. 36 kS/s, czyli pasmo ±17.8 kHz, w którym mieści się NFM.

## Rejestry i funkcje ROM

| adres | co to jest | skąd wiadomo |
|---|---|---|
| `0x6000057C` | sterowanie estymatorem: bit 0 = włącz, bit 1 = start, bity 2–16 = n (15 bitów), bit 18 = tryb, bit 31 = gotowe | deasemblacja `rom_iq_est_enable` |
| `0x600005DC` | **suma I** z n+1 próbek, ze znakiem, skala ×64 | `rom_dc_iq_est` |
| `0x600005E0` | **suma Q**, jak wyżej | `rom_dc_iq_est` |
| `0x600005E4` | moc (rośnie z n), u kaboom748 „E4”, używana do OOK | pomiar + notatki kaboom748 |
| `0x60000580..58C` | `RX_IQ_0..3`: sumy 2. rzędu (I², Q², wyrazy krzyżowe), bez fazy | `ram_rxiq_get_mis` w `libphy.a` |
| `0x60000D80..` | 8 × 11 bitów z przetwornika SAR (detektor mocy TX / TOUT), to nie jest tor RX | `read_sar_dout` w `libphy.a` |

Funkcje w ROM:
- **`rom_iq_est_enable(tryb, n)` (`0x40006430`):** ustawia bit 0, potem zapisuje
  `(ctrl & 0xFFFA0001) | (tryb << 18) | (n << 2) | 2` i czeka na bit 31.
- **`rom_iq_est_disable()` (`0x40006400`):** zapisuje
  `(ctrl & 0xFFFA0001) | 0x1000`, potem kasuje bit 0.
- **`rom_dc_iq_est(tryb, n, out)` (`0x4000615C`):** woła po kolei start i stop
  z tablicy funkcji PHY (pozycje 13 i 12), a między nimi zapisuje
  `out[0] = (suma_I >> 6) / (n+1)` i `out[1] = (suma_Q >> 6) / (n+1)`.

Funkcje z `libphy.a` (SDK 2.2.x) do strojenia PLL:
- **`ram_rfpll_set_freq(MHz, xtal, off, out[3])`:** liczy
  `v = 4·(1024·MHz + off) / (3·1024·xtal_MHz) − 32` (xtal: 0 = 40, 1 = 26,
  2 = 24 MHz), `out[0]` = część całkowita, `out[1..2]` = ułamek ×65536.
  Czyli F = 0.75·xtal·(v + 32), a `off` ma jednostkę 1/1024 MHz (≈ 977 Hz).
  Krok samego SDM przy 26 MHz to ≈ 297 Hz.
- **`set_rf_freq_offset(xtal, MHz, off)`:** `ram_rfpll_set_freq`, potem
  zapis SDM i kalibracja PLL przez `g_phyFuns` (pozycje 55, 56, 54) i
  `wait_rfpll_cal_end`. Nikt w SDK jej nie woła, ale jest eksportowana.
  Tego używa `rxTune` w szkicu; `xtal` = `chip6_phy_init_ctrl[1]`.
- **`chip_v6_set_chan_offset(ch, off)`:** oficjalna ścieżka (korekcja
  kwarcu, `phy_freq_offset`), ale **nie nadaje się**: `chip_60_set_channel`
  obcina offset do ±300 (≈ ±293 kHz) i zeruje go, gdy `freq_correct_en`
  (bit 0 bajtu 113 `chip6_phy_init_ctrl`) jest wyłączone, a domyślnie jest.
  Sprawdzone: offset ląduje w `phy_freq_offset`, PLL stoi.

Tablica funkcji PHY: wskaźnik w RAM pod `0x3FFFC730`, a domyślna kopia
leży w ROM od `0x4000EABC`. Zrzut ROM (64 KB) zrobiłem przez
`esptool dump_mem 0x40000000 0x10000`.

Tryb (bit 18) to w praktyce jeden bit. Oba tryby dają spójne IQ; w jednym
teście tryb 1 miał ok. 3 dB lepszy stosunek sygnału do szumu. Co ten bit
dokładnie wybiera, nie wiem.

## Czas pomiaru, czyli częstotliwość próbkowania

Czas jednego pomiaru rośnie o 25 ns na każdą próbkę okna, więc ADC
odbiornika pracuje z **40 MS/s**. Do tego dochodzi stały narzut ok. 1.8 µs
na pomiar. Rozrzut odstępów między pomiarami wynosi 0.12 µs.

| n+1 (okno) | czas okna | próbki IQ | tryb |
|---|---|---|---|
| 32 | 0.8 µs | 390 kS/s | tylko `IQ` (1024 pomiary do RAM) |
| 256 | 6.4 µs | 122 kS/s | tylko `IQ` |
| 1024 (k=10) | 25.6 µs | **35.7 kS/s** | `IQSTREAM`, domyślnie |
| 2048 (k=11) | 51 µs | 18.7 kS/s | `IQSTREAM` |
| 4096 (k=12) | 102 µs | 9.5 kS/s | `IQSTREAM` |

Okno uśrednia, więc działa jak filtr sinc. Przy k=10 tłumienie wynosi
0.6 dB przy ±8 kHz i ok. 3 dB na brzegu pasma (±17.8 kHz).

## Testy z HackRF jako nadajnikiem

ESP na kanale 1 (2412 MHz), HackRF nadawał słabą nośną (VGA 0 dB, bez
wzmacniacza):

| sygnał | prążek w IQ z ESP | nad szumem | lustro |
|---|---|---|---|
| HackRF wyłączony | tylko DC (offset odbiornika) | — | — |
| +10 kHz | +10.867 kHz | 44–48 dB | ok. −33 dBc |
| +30 kHz | +30.905 kHz | 42 dB | ok. −30 dBc |
| −10 kHz | −9.111 kHz | 51 dB | ok. −31 dBc |
| +5 kHz, strumień k=10 | +6.098 kHz | 58.7 dB | −31.4 dBc |

Wszystkie prążki mają ten sam przesuw (+0.9…+1.1 kHz), czyli rozjazd
kryształów ESP i HackRF. Znak częstotliwości jest poprawny. Mowa w SSB
nadana HackRF‑em została odebrana i zdemodulowana (nagrania w
`~/Pulpit/esp8266_odbior/`).

## Strojenie poza środki kanałów

`IQ f …` i `IQSTREAM f …`: `f` to numer kanału (1–14) albo częstotliwość
w MHz (np. `2414.5`, do 3 miejsc po przecinku). `rxTune` wybiera najbliższy
kanał 1–13 (14 blokuje domyślny kraj), ustawia go przez `wifi_set_channel`
(kalibracje RX dla tego środka), a potem przestawia PLL przez
`set_rf_freq_offset`. ESP odsyła `RXF <Hz> CH <n> OFF <off>`. `txStart`
wraca na środek kanału (`rxUntune`), więc nadawanie działa jak wcześniej
(sprawdzone HackRF-em: nośna po odbiorze na 2600 MHz w tym samym miejscu
co bez odbioru).

Test HackRF → ESP (nośna CW `hackrf_transfer -c 127 -x 0`, 5 kHz nad LO,
k=10, 2 s):

| LO ESP | kanał | HackRF | prążek | błąd | SNR | lustro |
|---|---|---|---|---|---|---|
| 2412.000 (ch 1) | 1 | 2412.005 | +6.24 kHz | +1.24 kHz | 59 dB | −31 dBc |
| 2414.500 | 1 | 2414.505 | +6.23 kHz | +1.23 kHz | 66 dB | −32 dBc |
| 2414.500 | 1 | 2412.005 | −2.77 kHz (alias z −2.495 MHz) | — | 25 dB | — |
| 2414.501 (= …500977) | 1 | 2414.505 | +5.31 kHz | +1.29 kHz | 61 dB | −31 dBc |
| 2400.100 (= …099610) | 1 | 2400.105 | +6.57 kHz | +1.18 kHz | 56 dB | −32 dBc |
| 2400.250 (QO-100) | 1 | 2400.255 | +6.32 kHz | +1.32 kHz | 59 dB | −32 dBc |
| 2300.000 | 1 | 2300.005 | +6.21 kHz | +1.21 kHz | 63 dB | −31 dBc |
| 2484.000 | 13 | 2484.005 | +6.26 kHz | +1.26 kHz | 60 dB | −32 dBc |
| 2600.000 | 13 | 2600.005 | +6.05 kHz | +1.05 kHz | 58 dB | −31 dBc |

Błąd +1.0…+1.3 kHz to rozjazd kwarców ESP i HackRF, jednakowy dla
wszystkich LO. Krok 1/1024 MHz działa (2414.501 przesuwa prążek o ok.
0.9 kHz). Nawet 128 MHz od kanału 13 czułość i lustro się nie pogarszają.
Granice 2300/2600 MHz to tylko zakres parsera (`parseKHz`), nie PLL.
Przestrojenie to kalibracja PLL, więc w strumieniu nie da się przestrajać
bez przerwy; dokładne strojenie (poniżej 1 kHz) robi się cyfrowo na PC.

## Strumień przez USB (`IQSTREAM f k tryb`)

- **Okno:** n+1 = 2^k, więc dzielenie zamienia się w przesunięcie:
  I, Q = suma >> k, wysyłane jako int16.
- **Łącze:** UART 2 Mbaud z **2 bitami stopu**. Przy jednym bicie stopu CH340
  gubił więcej bajtów (4.3% paczek zamiast 2.6% przy dużych paczkach).
- **Paczka (69 B):** `A5 5A`, ccount (24 bity LE), 16 × (I, Q) int16 LE.
  Ze znaczników czasu PC liczy dokładne fs i układa paczki na osi czasu.
  Uszkodzona paczka (zgubiony bajt) jest zastępowana zerami, więc czas
  i faza zostają ciągłe.
- **Straty CH340:** k=10 ok. 0.6% paczek, k=11 ok. 0.7%, k=12 zero. Przy
  paczkach po 64 próbki straty były ok. 4× większe.
- **Przebieg:** przerwania są wyłączone, a dowolny bajt od PC kończy
  strumień.

```sh
cd CW_CARRIER_TEST/audio
python3 rx_iq.py --ch 1 --k 10 --seconds 10 --out zrzut.wav   # IQ stereo int16 (SDR++: File source)
python3 rx_iq.py --snap --ch 1 --n 255                        # 1024 pomiary przez RAM, do 390 kS/s
python3 rx_iq.py --f 2400.25 --k 10 --seconds 10 --dc         # LO poza kanałem (np. QO-100)
```

## Odbiornik SSB z oknem (`esp_sdr.py`)

`CW_CARRIER_TEST/audio/esp_sdr.py`: widmo, waterfall, demodulator USB/LSB
(NCO, zespolony filtr FIR 255, AGC), dźwięk przez sounddevice.
- **Strojenie:** pole „Odbiór” (kółko myszy, krok do wyboru) albo klik lub
  przeciągnięcie czerwonej linii w widmie. W paśmie stroi NCO. LO ESP
  przestawia się samo, gdy filtr zbliży się do brzegu pasma albo do DC;
  ustawia się wtedy 5 kHz obok sygnału, po stronie bez wstęgi.
- **Korekcja kwarcu ESP:** pole ppm, domyślnie −0.5 ppm względem HackRF
  (+1.24 kHz przy 2412 MHz). Bez niej głos jest przesunięty o ok. 1.2 kHz.
- **Sygnał testowy:** `hackrf_ssb_tx.py` (domyślnie `sp8esa.mp3`, USB
  2400.250 MHz, w pętli, HackRF 600 kHz niżej, VGA 0).
- **Sprawdzone:** nagranie z `--wav` ma rozkład energii w pasmach zgodny
  z oryginałem co do ok. 1 dB, zgubione paczki 0.7%.

```sh
python3 hackrf_ssb_tx.py &            # nadawanie testowe (SDR++ zamknięty)
python3 esp_sdr.py                    # okno, start na 2400.250 MHz USB
python3 esp_sdr.py --wav t.wav --seconds 10   # bez okna, do testów
```

## Ograniczenia

- **Strojenie skokowe:** krok PLL ok. 1 kHz, każda zmiana to kalibracja
  PLL (przerwa i skok fazy).
- **Słaby antyaliasing:** okno tłumi sygnały daleko od kanału mniej więcej
  jak 1/(π·f·T). Przy k=10 to ok. −32 dB przy 500 kHz. Przykład z testu:
  przeciek nośnej HackRF z −500 kHz zawinął się na −1.5 kHz
  (−501.1 kHz + 14 × 35.685 kHz).
- **Offset DC** odbiornika (kreska na środku): odejmować średnią
  (`--dc`) albo filtrować.
- **Niedopasowanie I/Q:** lustro na ok. −31 dBc. Bez korekcji na PC.
- **Wzmocnienie odbiornika** nie jest ustawiane; zostaje takie, jakie
  zostawił stos Wi‑Fi.
- **Jedno naraz:** odbiór i nadawanie nie działają jednocześnie.
- **Przepustowość:** ograniczona przez CH340 (ok. 2 Mbaud), a nie przez
  sam estymator.

## Otwarte pytania

- Co wybiera bit 18 (tryb)?
- Czy da się ustawić wzmocnienie odbiornika ręcznie? Notatki kaboom748
  wskazują `0x60000590` jako sterowanie wzmocnieniem.
- Korekcja DC i I/Q na PC, demodulatory (NFM, SSB) w `rx_iq.py`.
- Nadawanie z offsetem PLL (`set_rf_freq_offset` także przy TX): nośna
  QO-100 bez przesuwania polem K o −12 MHz od kanału 1. Nie testowane.

## Źródła

- **kaboom748/esp8266_WLAN_PHY:** notatki o IQ_EST, „E4”, `RX_IQ_0..3`
  i adresach funkcji ROM (dziennik CCA, referencje RX OOK i FSK).
- **Moja analiza:** zrzut ROM z płytki, deasemblacja `rom_dc_iq_est`,
  `rom_iq_est_enable`, `rom_iq_est_disable` oraz `ram_rxiq_get_mis`
  i `read_sar_dout` z `libphy.a` (SDK NONOS 2.2.x), pomiary z HackRF One.
