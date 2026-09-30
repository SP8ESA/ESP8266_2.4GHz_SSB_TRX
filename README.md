# ESP8266: SSB na 2,4 GHz

Zwykły ESP8266 nadaje mowę w SSB (USB/LSB) na 2,4 GHz, bez żadnego
dodatkowego sprzętu RF. Wykorzystuje tylko wewnętrzny generator tonu
toru PHY, sterowany bezpośrednio z rejestrów. PC liczy próbki SSB i
wysyła je przez USB, a ESP je moduluje. Do odbioru wystarczy dowolny SDR
w trybie USB (testowane na HackRF One + SDR++).

> **Sterowanie radiem ESP8266 oparte jest na
> [kaboom748/esp8266_WLAN_PHY](https://github.com/kaboom748/esp8266_WLAN_PHY).**
> Stamtąd pochodzą adresy rejestrów toru tonu, PBUS i funkcji ROM,
> sekwencja włączania toru TX oraz skale pól K i ASK. Bez tamtego
> reverse engineeringu ten projekt by nie powstał. Szczegóły są w sekcji
> [Źródła](#źródła-i-licencje).

## Uwaga

- To prawdziwy nadajnik. Nadawaj tylko tam, gdzie masz do tego prawo:
  z licencją krótkofalarską w paśmie 13 cm albo w granicach ISM/SRD.
- Długie ciągłe nadawanie mocno grzeje układ. Nie zostawiaj go bez nadzoru.
- W czasie nadawania SSB WiFi nie działa: tor PHY jest w trybie ręcznym,
  a przerwania są wyłączone.

## Zawartość

| Plik | Do czego |
|---|---|
| `CW_CARRIER_TEST/CW_CARRIER_TEST.ino` | firmware: modulator, tryb `STREAM`, SSB z flasha |
| `CW_CARRIER_TEST/ssb_data.h` | nagranie SSB we flashu (obwiednia + częstotliwość, 32 kS/s) |
| `CW_CARRIER_TEST/audio_data.h` | to samo nagranie dla trybów FM/AM (8 kS/s); szkic go dołącza |
| `CW_CARRIER_TEST/audio/stream_ssb.py` | SSB na żywo: pliki audio z PC przez USB |
| `CW_CARRIER_TEST/audio/ssb_dsp.py` | DSP: procesor mowy, sygnał analityczny, predystorsja |
| `CW_CARRIER_TEST/audio/make_audio_h.py` | generuje `ssb_data.h` / `audio_data.h` z pliku audio |
| `CW_CARRIER_TEST/audio/ask_cal.json` | zmierzone nieliniowości toru (AM-AM, AM-PM, AM-FM) |
| `CW_CARRIER_TEST/audio/osr_*.wav` | nagrania testowe (głos kobiecy i męski) |

Szkic nazywa się `CW_CARRIER_TEST`, bo zaczął życie jako test nośnej CW.
Oprócz SSB umie też CW/Morse, sweep, FM i AM (pełna lista komend: `?`).

## Wymagania

Sprzęt:

- płytka ESP8266 z 4 MB flash (testowane: moduł w stylu Wemos D1 mini
  z CH340; konwerter USB-UART musi obsługiwać 2 Mbaud),
- odbiornik SDR na 2,4 GHz.

Oprogramowanie:

- `arduino-cli` (albo Arduino IDE) z rdzeniem `esp8266:esp8266` 3.1.2,
- Python 3 z numpy, scipy i pyserial (`pip install -r requirements.txt`),
- `ffmpeg` w `PATH` (do wczytywania plików audio).

## Uruchomienie

### 1. Firmware

```sh
URL=https://arduino.esp8266.com/stable/package_esp8266com_index.json
arduino-cli core update-index --additional-urls $URL
arduino-cli core install esp8266:esp8266@3.1.2 --additional-urls $URL

FQBN=esp8266:esp8266:d1_mini:xtal=160,eesz=4M
arduino-cli compile --fqbn $FQBN CW_CARRIER_TEST
arduino-cli upload  --fqbn $FQBN -p /dev/ttyUSB0 CW_CARRIER_TEST
```

W Arduino IDE wybierz płytkę „LOLIN(WEMOS) D1 R2 & mini” i CPU Frequency
160 MHz. Na tym zegarze wszystko było testowane, a modulator odmierza czas
w cyklach CPU.

Po zasileniu ESP nic nie nadaje, tylko czeka na komendy na porcie
szeregowym (115200).

### 2. SSB na żywo z PC

```sh
cd CW_CARRIER_TEST/audio
python3 stream_ssb.py osr_kobieta.wav osr_mezczyzna.wav --freq 2402
```

W SDR ustaw 2402,000 MHz, USB, filtr około 3 kHz. Kwarce ESP i odbiornika
mogą się rozjeżdżać o kilka ppm, co na 2,4 GHz daje kilka kHz (u mnie
wyszło około −4,7 kHz), więc trzeba dostroić odbiornik na słuch.

Skrypt otwiera port (co resetuje ESP), wysyła `STREAM <f>`, przechodzi
na 2 Mbaud i gra pliki w pętli, z 1 s przerwy, aż do Ctrl-C. Po 0,5 s bez
danych ESP wyłącza nadajnik, wraca na 115200 i wypisuje podsumowanie:
liczbę próbek, błędy sumy kontrolnej i niedobory bufora.

| Opcja | Znaczenie |
|---|---|
| `--freq 2402` | częstotliwość (wytłumionej) nośnej w MHz, do 1 kHz, np. `2402.35` |
| `--lsb` | dolna wstęga zamiast górnej |
| `--comp soft\|mid\|hard` | siła procesora mowy (domyślnie `mid`) |
| `--raw` | bez procesora mowy, tylko filtr pasma (do porównań A/B) |
| `--gap 1.0` | przerwa między plikami w sekundach |
| `--seconds N` | zakończ po N sekundach |
| `--port /dev/ttyUSB0` | port szeregowy |

Jako plik można podać wszystko, co czyta ffmpeg, albo jedno ze źródeł
testowych:

- `sine:F[:S]`: ton F Hz przez cały tor DSP,
- `cw:F[:S]`: stała częstotliwość F Hz, sam dithering K bez DSP,
- `silence:S`: cisza (bramka wyłączona).

### 3. SSB z flasha, bez PC

W monitorze portu szeregowego (115200):

```
USB 2402      górna wstęga, nośna 2402,000 MHz
LSB 2402      dolna wstęga
OFF           koniec
```

ESP gra w pętli nagranie z `ssb_data.h`, z 1 s przerwy. W trakcie zdania
przerwania są wyłączone, więc komendy są odczytywane dopiero w przerwie.

Własne nagranie wgrywa się tak:

```sh
cd CW_CARRIER_TEST/audio
python3 make_audio_h.py moje_nagranie.wav   # nadpisuje ../ssb_data.h i ../audio_data.h
```

Potem trzeba przekompilować szkic. Każda sekunda SSB zajmuje około 125 KB
flasha.

## Jak to działa

### Radio: generator tonu PHY

Tor PHY ESP8266 ma „slot tonu” (rejestr `0x600005B8`), czyli generator
nośnej używany do testów RF. Ma trzy pola:

- **K** (bity 0–9): przesunięcie od środka kanału WiFi, ze znakiem,
  78,125 kHz na kod,
- **skala / ASK** (bity 10–17): tłumik cyfrowy, około 0,24 dB na krok,
- **bramka** (bit 18): nośna włączona lub wyłączona.

Firmware ustawia sekwencję startową tak:

1. `wifi_set_channel()` stroi PLL na kanał najbliższy żądanej
   częstotliwości (zakres K to ±39 MHz od środka kanału),
2. zatrzymuje odbiornik (`0x60009B08`, bit 27) i przełącza PBUS w tryb
   ręczny (`0x60000594`, bit 0),
3. włącza tor TX zapisami PBUS,
4. ustawia skalę analogową PA funkcją ROM `set_ana_inf_tx_scale`
   (`0x4000678C`; w komendach to `APWR`) i włącza zegar TX przez
   `set_txclk_en` (`0x4000650C`).

Od tej chwili nośna zależy tylko od zapisów do slotu tonu. CPU może je
robić z dowolną szybkością, bez przestrajania PLL i bez przerw.

### SSB metodą polarną (Kahn / EER)

Sygnał SSB to sygnał analityczny mowy, z(t) = a(t)·e^{jφ(t)}. ESP nie ma
przetwornika I/Q, ale ma sterowanie amplitudą (ASK) i częstotliwością (K),
więc SSB robi się metodą polarną:

- obwiednia a(t) idzie na tłumik ASK,
- chwilowa częstotliwość dφ/dt idzie na K. Dla LSB ta sama częstotliwość
  ma przeciwny znak.

Obie wielkości liczy PC. Próbkowanie wynosi 32 kS/s.

### Ułamkowe K: modulator impulsowy

K ma krok 78 kHz, a mowa wymaga rozdzielczości około 1 Hz. Robi to funkcja
`ditherBurst()`:

- w rejestrze siedzi kod k najbliższy celowi,
- błąd fazy (cel minus faktyczna częstotliwość) jest całkowany co cykl CPU,
- gdy błąd przekroczy próg (20 cykli, czyli około 3,5° przy 160 MHz),
  firmware wpisuje k+1 albo k−1 na dokładnie wyliczoną liczbę cykli,
  po której błąd wraca do zera.

Faza trzyma się celu z dokładnością około 3,5°, a produkty przełączania
lądują daleko od pasma audio. Ułamkowa amplituda działa podobnie: ASK
przeskakuje losowo między n i n+1 (dithering xorshiftem).

Cały strumień leci z **wyłączonymi przerwaniami** (`xt_rsil(15)`). Każda
przerwa (SDK, ISR) zostawia w rejestrze k albo k+1 na zbyt długo, a skoro
faza to całka częstotliwości, skok fazy daje śmieci po obu stronach nośnej.
Dlatego watchdog jest karmiony ręcznie, a UART jest czytany wprost z
rejestrów FIFO między próbkami.

### Nieliniowości toru i predystorsja

Tor ASK/PA nie jest idealny. Nieliniowości zmierzyłem HackRF-em na
sygnałach testowych: `CAL AM` w firmware (sweep poziomów ASK przeplatany
poziomem odniesienia) oraz schody ASK z `make_audio_h.py envstair:`.
Skrypty analizy nie wchodzą do tej paczki, a wynik jest w `ask_cal.json`
(przy APWR 127):

- **AM-AM:** rzeczywista krzywa tłumika. ASK 0..127 daje około 30,7 dB
  zakresu, a obwiednia przechodzi przez krzywą odwrotną.
- **AM-PM:** faza przesuwa się do +9° wraz z tłumieniem. Ta wartość jest
  odejmowana od fazy.
- **AM-FM:** przy niższym poziomie częstotliwość wyjściowa spada do
  około −68 Hz, z opóźnieniem pierwszego rzędu τ ≈ 0,27 ms. Jest
  odejmowana od częstotliwości przez ten sam filtr.
- **APWR 127:** najbardziej liniowy punkt pracy PA.
- **Podłoga:** ASK 127 daje około −31 dB. Jeśli obwiednia leży poniżej
  podłogi dłużej niż 5 ms, bramka jest wyłączana, z 1 ms marginesu.
  Bez tego w pauzach szedł szum fazy na −31 dB, który AGC odbiornika
  wyciągał jako chrapanie. Bramki się nie dithera, bo jest nieliniowa.

Bez `ask_cal.json` DSP przyjmuje prawo z repozytorium upstream
(0,2429 dB/krok) i nie koryguje AM-PM ani AM-FM. Kalibracja pochodzi
z jednego egzemplarza ESP, więc na innej płytce może wyglądać inaczej.

### DSP na PC (`ssb_dsp.py`)

1. Filtr pasma 200–2800 Hz i auto-EQ: długoterminowe widmo (w 1/3 oktawy)
   jest ściągane do nachylenia −1,5 dB/okt. od 400 Hz (korekta −10..+10 dB).
2. Kompresor z detektorem RMS 10/150 ms i wygładzonym wzmocnieniem, potem
   limiter z wyprzedzeniem zamiast clippera (clipper dawał charczenie).
3. Bramka mowy liczona offline, więc może patrzeć w obie strony: maska
   poszerzona o ±40 ms, zbocza 10 ms.
4. Transformata Hilberta daje sygnał analityczny. Na nim działa kompresja
   i limiter obwiedni (niższy PAPR daje więcej średniej mocy nad podłogą),
   potem drugi raz filtr pasma i Hilbert.
5. `polar()`: obwiednia zamieniana na ASK przez odwróconą zmierzoną krzywą,
   częstotliwość to przyrost fazy minus AM-PM i AM-FM. Kwantyzacja ma
   sprzężenie błędu, więc faza końcowa trafia co do LSB i nie dryfuje.

### Transmisja przez USB (tryb `STREAM`)

- UART 2 Mbaud. Pakiet: `A5 5A`, 64 próbki po 3 bajty i XOR tych 192
  bajtów, razem 195 B na 64 próbki, czyli około 97,5 kB/s (połowa łącza).
  Każda próbka to:
  - obwiednia u8: ASK w pół kroku, 0..254, a 255 oznacza wyłączoną bramkę,
  - częstotliwość i16 LE: przesunięcie K w Q16, około 1,19 Hz na LSB,
    względem nośnej.
- ESP ma bufor 4096 próbek (128 ms) i zaczyna grać od 1024 (32 ms).
  Co 64 próbki odsyła `B7` i zapełnienie/16, a PC utrzymuje bufor
  w około 50%. Przy niedoborze jest cisza, aż bufor się znowu napełni.
- Koniec: 0,5 s bez danych (albo 3 s przed pierwszym pakietem).

## Wyniki i ograniczenia

Pomiar HackRF-em na mowie, w wersji z predystorsją, bramką w pauzach
i kompensacją AM-FM:

- przeciwna wstęga: około −32 dB (w pierwszych próbach, bez predystorsji: −15,7 dB),
- śmieci w ±3,5–10 kHz od nośnej: około −35 dB,
- pauzy: około −50 dB.

Ograniczenia:

- Tłumienie przeciwnej wstęgi około 32 dB to dużo mniej, niż daje
  nadajnik z filtrem kwarcowym.
- W trakcie mowy zostają zniekształcenia fazy (około 25° w ostatnim
  pomiarze), których jeszcze nie rozgryzłem.

## Źródła i licencje

Ten projekt jest na licencji MIT (zob. [LICENSE](LICENSE)).

- **[kaboom748/esp8266_WLAN_PHY](https://github.com/kaboom748/esp8266_WLAN_PHY)**
  (stan z 2026-09-29, commit `754cbc3`) to źródło całej wiedzy o sterowaniu
  radiem ESP8266:
  - `TEST-TONEv5.yaml` i `2-FSK/TX_REFERENCE_v1.ino`: slot tonu
    `0x600005B8`, PBUS (`0x60000594` / `0x600005A0`), stop RX
    (`0x60009B08`), funkcje ROM `set_txclk_en` i `set_ana_inf_tx_scale`,
    sekwencja włączania i wyłączania toru TX,
  - `2-FSK/ESP8266_2FSK_GUIDE_DETAILLE_FR_v2.0.md`: 78,125 kHz na kod K,
  - `M-ASK/ESP8266_ASK_M-ASK_REFERENCE_EN.md`: około 0,2429 dB na krok ASK.

  Funkcje PHY w firmware (`pbusWrite`, `enterManual`/`leaveManual`,
  `txPathOn`/`txPathOff`, `toneProgram`) są oparte na kodzie z tego
  repozytorium. Jest on udostępniony na licencji MIT,
  Copyright (c) 2026 kaboom748.
- **Nagrania testowe:** [Open Speech Repository](https://www.voiptroubleshooter.com/open_speech/american.html),
  American English. `osr_kobieta.wav` to fragment 0,40–3,10 s z
  `OSR_us_000_0010_8k.wav`, a `osr_mezczyzna.wav` fragment 1,90–4,75 s
  z `OSR_us_000_0030_8k.wav`. `ssb_data.h` i `audio_data.h` są
  wygenerowane z `osr_kobieta.wav`.
- **Metoda polarna:** L. R. Kahn, „Single-Sideband Transmission by Envelope
  Elimination and Restoration”, *Proc. IRE*, vol. 40, no. 7, 1952.
