# Zrzuty ekranu (2026-10-01)

## Nadajnik ESP8266 (SSB, tryb STREAM, USB 2402.000 MHz)

Odbiór HackRF One w SDR++. Sygnały testowe z `CW_CARRIER_TEST/audio/stream_ssb.py` (bez
procesora mowy, szczyt = ASK 0).

**`tx_szum_szeroko.png`:** biały szum w paśmie SSB (`noise`, 200–2800 Hz),
szeroki widok. Pośrodku pasmo szumu (w tej skali jeden wąski prążek), po
bokach symetryczne prążki od ruchu UART CH340 w trybie STREAM (±153 kHz,
słabsze ±51 i ±306 kHz). Odtwarzanie z flasha ich nie ma.

![biały szum, szeroki widok](tx_szum_szeroko.png)

**`tx_dwuton_szeroko.png`:** test dwutonowy (`twotone`, 700 + 1900 Hz,
każdy ton −6 dB względem PEP), ten sam widok. Moc skupiona w dwóch
prążkach, więc szczyt jest wyżej niż przy szumie. Widać te same prążki od
UART i dodatkowe produkty obce.

![test dwutonowy, szeroki widok](tx_dwuton_szeroko.png)

**`tx_dwuton_zblizenie.png`:** test dwutonowy z bliska. Dwa tony ok. 1.2 kHz
od siebie nad wytłumioną nośną 2402.000 MHz. Słaby prążek z lewej, ok.
0.5 kHz pod nośną, to najpewniej produkt intermodulacji 3. rzędu (2·f1 − f2).

![test dwutonowy, zbliżenie](tx_dwuton_zblizenie.png)

**`tx_szum_zblizenie.png`:** biały szum z bliska. Płaskie pasmo ok. 2.6 kHz
nad nośną ze stromymi zboczami: kształt pasma nadajnika, a obok niego
poziom produktów poza pasmem.

![biały szum, zbliżenie](tx_szum_zblizenie.png)

## Odbiornik ESP8266 (`CW_CARRIER_TEST/audio/esp_sdr.py`)

**`rx_esp_sdr_sp8esa.png`:** ESP odbiera nagranie `sp8esa.mp3` nadawane
HackRF-em w USB na 2400.250 MHz (`CW_CARRIER_TEST/audio/hackrf_ssb_tx.py`).
- **Czerwona linia:** częstotliwość odbioru 2400.2476 MHz.
- **Zielone pole:** filtr USB 2.4 kHz.
- **Waterfall:** mowa z 1 s przerwami między powtórzeniami.
- **Pionowa kreska przy 2400.245 MHz:** LO ESP (resztka DC odbiornika).

Sygnał wypadł ok. 2.5 kHz niżej niż nominalne 2400.250 MHz, dlatego
odbiór jest dostrojony ręcznie. To najpewniej dryf kwarcu HackRF: zrzut jest
zrobiony zaraz po jego ponownym podłączeniu, po tym jak rozłączał się po
ok. 20 min nadawania.

![odbiór sp8esa w esp_sdr.py](rx_esp_sdr_sp8esa.png)
