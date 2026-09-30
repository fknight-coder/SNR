# Hardware — ZSPARK Adaptive Sonar Transmitter

This document describes the physical components, MCU peripheral configuration and wiring architecture used in the current prototype, as implemented in [`firmware/main.c`](../firmware/main.c).

> **Prototype note:** In this revision, the five environmental inputs are emulated using potentiometers wired to ADC1 channels. This lets the adaptive decision logic (Mackenzie sound speed, Francois-Garrison absorption, two-stage frequency search) be exercised and validated on the bench before real sensors are integrated. See [Sensor Emulation → Real Sensors](#sensor-emulation--real-sensors) below.

---

## Target Hardware

| Item | Detail |
|---|---|
| Board | STM32G474RE (Nucleo) |
| MCU | STM32G474RE (ARM Cortex-M4, 170 MHz) |
| System clock | HSE (8 MHz) → PLL → 168 MHz SYSCLK (PLLM=2, PLLN=84, PLLP=2) |
| Voltage scaling | RANGE 1 BOOST |

---

## Bill of Materials (Current Prototype)

| Component | Qty | Function |
|---|---|---|
| STM32G474RE board | 1 | Main compute — ADC sampling, adaptive DSP, DAC output |
| Linear potentiometer (10 kΩ) | 5 | Emulate Temperature, Depth, Turbidity, Salinity, Battery voltage inputs |


Onboard peripherals inherited from the Discovery board's default CubeMX configuration (I2S3/audio codec, MEMS accelerometer SPI, USB OTG FS, user button B1) are initialized by the board-support code but are **not** part of the sonar signal chain — see [Unused Onboard Peripherals](#unused-onboard-peripherals).

---

## Signal Chain Overview

```
5x Potentiometer          STM32F407VGT6                          Output
(sensor emulators)   ┌─────────────────────────┐
                      │                         │
PA1 ─────────────────┤ ADC1_IN1  Temperature    │
PA2 ─────────────────┤ ADC1_IN2  Depth          │
PA3 ─────────────────┤ ADC1_IN3  Turbidity      │   ADC1 (scan + DMA2_Stream0)
PB0 ─────────────────┤ ADC1_IN8  Salinity       │
PB1 ─────────────────┤ ADC1_IN9  Battery        │
                      │            │             │
                      │            ▼             │
                      │  Calibration + Averaging │
                      │            ▼             │
                      │  RAM Adaptive LUT/Cache  │
                      │            ▼             │
                      │  Mackenzie + F-Garrison  │
                      │  Two-Stage Freq Search   │
                      │            ▼             │
                      │  DDS: 1024-pt Sine LUT + │
                      │  Adaptive Window         │
                      │  (Hann/Hamming/Blackman) │
                      │            ▼             │
                      │      DAC Buffer          │
                      │            │             │
                      │   TIM6 (2 MHz trigger)   │
                      │            │             │
                      │            ▼             │
                      │   DAC1 Ch.1 (DMA1_Stream5)├──── PA4 (DAC1_OUT1) ──► Analog waveform out
                      │                         │
                      │  LD3/LD4/LD5/LD6 (GPIOD)├──── Waveform-mode indicator LEDs
                      └─────────────────────────┘
```

---

## Pin Map

| Pin | Peripheral | Connected To | Function |
|---|---|---|---|
| PA1 | ADC1_IN1 | Potentiometer 1 | Temperature (emulated) |
| PA2 | ADC1_IN2 | Potentiometer 2 | Depth (emulated) |
| PA3 | ADC1_IN3 | Potentiometer 3 | Turbidity (emulated) |
| PB0 | ADC1_IN8 | Potentiometer 4 | Salinity (emulated) |
| PB1 | ADC1_IN9 | Potentiometer 5 | Battery voltage (emulated) |
| PA4 | DAC1_OUT1 | Signal output / scope probe | Synthesized sonar waveform output |
| GPIOD (LD3) | GPIO Output | Onboard orange LED | CW waveform active |
| GPIOD (LD4) | GPIO Output | Onboard green LED | LFM waveform active |
| GPIOD (LD5) | GPIO Output | Onboard red LED | Geometric Sweep waveform active |
| GPIOD (LD6) | GPIO Output | Onboard blue LED | Barker-13 waveform active |

All five ADC channels are sampled in a single scan-mode conversion group and moved to RAM via **DMA2_Stream0**; the DAC buffer is streamed out via **DMA1_Stream5**, paced by **TIM6** as the hardware trigger source (TIM6 is a trigger only — it does not move data itself).

---

## MCU Peripheral Configuration

### ADC1 — Environmental Sensing

| Setting | Value |
|---|---|
| Resolution | 12-bit |
| Mode | Scan, continuous, DMA circular |
| Channels (rank order) | IN1 → IN2 → IN3 → IN8 → IN9 |
| Sampling time | 480 cycles/channel |
| Data alignment | Right-aligned |

### DAC1 — Waveform Output

| Setting | Value |
|---|---|
| Channel | DAC_CHANNEL_1 (PA4) |
| Trigger source | TIM6 TRGO |
| Output buffer | Enabled |
| Buffer size | 32,768 samples (16,384-sample ping-pong halves) |

### TIM6 — Sample-Rate Clock

```
Fs = 84 MHz / (PSC + 1) / (ARR + 1)
   = 84 MHz / 1 / 42
   ≈ 2 MHz sample rate
```

TIM6 runs purely as a hardware trigger for the DAC's DMA-driven conversions — it has no DMA channel of its own.

### DMA

| Stream | Peripheral | Direction |
|---|---|---|
| DMA2_Stream0 | ADC1 | Peripheral → Memory (adc_buffer) |
| DMA1_Stream5 | DAC1 | Memory (dac_buffer) → Peripheral |

USART DMA is configured in the project but is not used by the sonar application path.

---

## Onboard Signal Processing (Firmware Side)

| Stage | Detail |
|---|---|
| Sine LUT | 1024-point precomputed sine table for DDS waveform synthesis |
| Adaptive RAM LUT/cache | 32-entry fixed-size cache keyed on quantized (temp, depth, turbidity, salinity, battery) state; LRU eviction on a miss |
| Windowing | Hann / Hamming / Blackman selected adaptively from turbidity and battery thresholds |
| Waveform types | CW, LFM Chirp, Geometric Sweep, Barker-13 (13-chip phase code) |
| Update rate | Adaptive parameters recomputed every 100 ms |

---

## Unused Onboard Peripherals

The STM32F407G-DISC1's default CubeMX board-support init also brings up the following, which are **not** part of the sonar signal path and can be disabled or repurposed on a custom PCB:

| Peripheral | Board Function | Status |
|---|---|---|
| I2C1 | CS43L22 audio codec control interface | Initialized, unused by sonar logic |
| SPI1 | Onboard MEMS motion sensor | Initialized, unused by sonar logic |
| I2S3 (SCK/SD) | Audio codec data | GPIO configured only |
| USB OTG FS | USB host stack | Present in project (`usb_host.h`), unused by sonar logic |
| B1 / BOOT1 | User button / boot select | Board default GPIO only |

---

## Sensor Emulation → Real Sensors

| Emulated (current) | Target real sensor (future) |
|---|---|
| Potentiometer — Temperature | DS18B20 or thermistor probe |
| Potentiometer — Depth | Pressure/depth transducer |
| Potentiometer — Turbidity | Turbidity sensor (e.g. SEN0189-class) |
| Potentiometer — Salinity | Salinity/TDS probe |
| Potentiometer — Battery | Direct battery-rail voltage divider into ADC |

Because all adaptation logic keys off calibrated float values (`x = a·ADC + b`) rather than raw ADC counts, swapping a potentiometer for a real sensor only requires updating that channel's calibration constants — no change to the decision engine, DDS, or DMA pipeline.

---

## Bring-Up / Test Setup

1. Power the STM32F407G-DISC1 via USB (ST-Link + application power share the same rail on the Discovery board).
2. Wire the five 10 kΩ potentiometer wipers to PA1, PA2, PA3, PB0, PB1 (with ends to 3.3 V and GND).
3. Probe **PA4 (DAC1_OUT1)** with an oscilloscope to observe the synthesized waveform.
4. Watch **LD3–LD6** to confirm the adaptive engine is selecting the expected waveform mode as the potentiometers are varied.
