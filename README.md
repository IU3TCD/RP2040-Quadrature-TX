# RP2040 SSB/AM/CW DSP Modulator

progetto MOOOOOLTO SPERIMENTALE

Modulatore digitale multi-modo per radio amatoriale basato su Raspberry Pi Pico
(RP2040). Il microcontrollore genera in DSP i segnali in quadratura **I/Q** a
partire da un microfono (o da toni di test interni) e pilota un mixer a
commutazione **FST3235** in configurazione QSE (Quadrature Sampling Exciter,
principio Tayloe). Uscita a frequenza fissa **20.4 MHz** (IF).

Modi supportati: **USB, LSB, AM, CW**.

## Come funziona

Il segnale audio viene campionato, filtrato in banda voce (300–2700 Hz) e
trasformato in una coppia I/Q tramite un filtro di Hilbert FIR. Il metodo di
generazione della banda laterale è il **phasing** (metodo a reiezione di banda):

- **USB / LSB** — si scambiano invertendo il segno del ramo Q (`q = -q`), senza
  alcuna commutazione analogica.
- **CW** — tono fisso a 700 Hz manipolato nel DSP con envelope a rampa
  (anti-click).
- **AM** — portante generata nel DSP come offset DC sul ramo I, con le due
  bande laterali (Q = 0). Livello di portante e indice di modulazione
  regolabili via I2C.

Le quattro fasi in quadratura (**0°, 90°, 180°, 270°** = I, Q, −I, −Q) escono su
quattro uscite PWM, ciascuna seguita da un filtro RC a due stadi, e pilotano
direttamente l'FST3235. Le fasi di commutazione RF (0°/90°) provengono da un
**Si5351** esterno; il filtro a quarzo a valle (7 kHz) seleziona il canale e
rifinisce la reiezione della banda indesiderata.

## Architettura
             ┌─────────────── RP2040 (DSP) ───────────────┐
Mic ──ADC──▶ │ bandpass ─▶ Hilbert ─▶ I/Q ─▶ modo TX     │
             │                                        │   │
                                      │ 4 uscite PWM (I / -I / Q / -Q) │
                               └──────────────────┬─────────────────────────┘
                                     │ (ciascuna: RC doppio stadio)
                                     ▼
      Si5351 (0°/90°) ──────────▶ FST3235 (QSE, doppio switch 2:1)
                                     │
                                     ▼
                              nodo di somma  ─▶ RF out


Il controllo (modo, sorgente, parametri AM) avviene via **I2C** da un
microcontrollore host (es. Arduino Nano). Il DSP gira sul **core0**, lo slave
I2C sul **core1**, così le transazioni di controllo non disturbano il
campionamento audio.

## Specifiche DSP

| Parametro            | Valore                                  |
|----------------------|-----------------------------------------|
| System clock         | 250 MHz (overclock, Vcore 1.20 V)       |
| Sample rate (fs)     | 48828.125 Hz (wrap PWM ÷5)              |
| PWM DAC              | 10 bit, ~244 kHz                        |
| Filtro di Hilbert    | FIR 101 tap, passabanda 300–2700 Hz, finestra Blackman |
| Banda audio          | 300–2700 Hz (bandpass RBJ biquad)       |
| Aritmetica           | Q15 intera nell'ISR (no floating point) |
| Latenza              | ~1.1 ms (ritardo di gruppo del FIR)     |

Il clock di sistema e la tensione core vengono impostati dal firmware stesso,
quindi lo sketch è autosufficiente indipendentemente dalle impostazioni dell'IDE.

## Filtro RC di uscita (per ognuna delle 4 uscite)

GPIO ──[330Ω]──┬──[220Ω]──┬──▶ FST3235
               │          │
            [47nF]     [220nF] 
               │          │
              GND        GND


Doppio stadio: 330 Ω/47 nF (fc ≈ 10.3 kHz) + 220 Ω/220 nF (fc ≈ 3.29 kHz).
Banda voce piatta (~2.4 dB a 2.7 kHz), portante PWM (244 kHz) attenuata ~60 dB,
impedenza di uscita ~550 Ω. I quattro rami devono usare componenti appaiati
(1% o misurati) per una buona reiezione d'immagine.

## Mappa dei pin

| GPIO   | Funzione                                   |
|--------|--------------------------------------------|
| 6      | I    (0°)   — PWM, stesso slice di GPIO7    |
| 7      | −I   (180°) — PWM                          |
| 8      | Q    (90°)  — PWM, stesso slice di GPIO9    |
| 9      | −Q   (270°) — PWM                          |
| 28     | Ingresso microfono (ADC2)                  |
| 12     | Tasto CW (input, pull-up, attivo basso)    |
| 16     | Pin di test (input, pull-up, attivo basso) |
| 2 / 3  | I2C1 SDA / SCL (slave)                     |

## Protocollo I2C

Slave sul bus **I2C1**, indirizzo **0x42**. Comandi in scrittura `[reg][val]`:

| Registro     | Cod. | Valore                                  |
|--------------|------|-----------------------------------------|
| `REG_MODE`   | 0x01 | 0 = USB, 1 = LSB, 2 = AM, 3 = CW         |
| `REG_SOURCE` | 0x02 | 0 = mic, 1 = 1 tono, 2 = 2 toni          |
| `REG_CARRIER`| 0x03 | livello portante AM (0–255)             |
| `REG_MODIDX` | 0x04 | indice di modulazione AM (0–255)        |
| `REG_STATUS` | 0x10 | lettura → `[mode][source]`              |

L'host (5 V, es. Arduino Nano) si collega tramite un **level shifter
bidirezionale I2C** (BSS138) verso i 3.3 V dell'RP2040.

## Sorgenti e test

- **Microfono** — ingresso normale (ADC su GPIO28, segnale centrato a metà scala).
- **1 tono** — sinusoide di test a 1 kHz (DDS interno).
- **2 toni** — 700 + 1900 Hz per la misura di intermodulazione (IMD).

Il **pin di test** (GPIO16, attivo basso) forza il tono singolo mentre è a
massa e ripristina la sorgente comandata via I2C al rilascio — utile per il
collaudo (es. il classico cerchio in modalità X-Y sull'oscilloscopio verifica
la quadratura).

## Hardware richiesto

- Raspberry Pi Pico (RP2040)
- Mixer FST3235 (alimentato a 3.3 V)
- Sintetizzatore Si5351 (LO in quadratura 0°/90°)
- Filtro a quarzo a 20.4 MHz (banda ~7 kHz, terminazione 800 Ω)
- Componenti dei filtri RC (vedi sopra)
- Microcontrollore host per il controllo I2C (es. Arduino Nano) + level shifter

## Build

Sketch per **Arduino IDE** con il core [Earle Philhower per
RP2040](https://github.com/earlephilhower/arduino-pico). Seleziona la board
Raspberry Pi Pico e carica; clock e tensione core sono impostati dal codice.

## Stato del progetto

Progetto sperimentale in sviluppo. L'architettura ha attraversato diverse
iterazioni (Weaver, NE602 phasing con trasformatori, QSE con FST3235); quella
attuale è il modulatore QSE con FST3235 pilotato direttamente dalle quattro
uscite in quadratura del DSP.

Stay-Tuned

