# SparkX — Ocean-Physics-Optimized Self Tunning Low Power Sonar Transmitter for AUVs

**Smart India Hackathon 2026 — Problem Statement 26058**

| Team | Team ID | PS Category | Theme |
|------|---------|-------------|-------|
| SparkX | 137 | Hardware | Robotics and Drones |
---

## Problem Statement

> **Development of a Low-Power, Real-Time Adaptive Software-Defined Sonar Transmitter Payload for Autonomous Underwater Vehicles (AUVs)**

Conventional sonar transmitters run on fixed frequency, pulse duration and amplitude settings. They cannot react to changing underwater conditions (temperature, salinity, depth) and therefore waste battery, lose signal quality, or fail to balance range against resolution.

SparkX proposes a **software-defined sonar transmitter** that senses its environment, computes the acoustically optimal transmission parameters in real time, and synthesizes a matched waveform entirely in firmware — with no hardware changes required to reconfigure it.

---

## Solution Overview

Five environmental sensors are continuously sampled, scaled and calibrated via ADC+DMA.
Mackenzie and Francois-Garrison models compute live acoustic conditions for adaptive frequency selection.
A two-stage (coarse-to-fine) numerical search evaluates transmission loss across candidate frequencies and selects the lowest-loss option.
The sonar equation derives source level and calibrated DAC amplitude.
CW / LFM / Geometric / Barker-13 and adaptive windowing provide flexible waveform generation.
DDS + sine-LUT with timer-triggered DMA streams the waveform directly to the DAC.

**Core pipeline:** `Sense → Calibrate → Real Oceanographic Calculation → Transmission loss calculation → Select Best Candidate Frequency → Source-Level Requirement & SNR Gap → Parameter, Modulation & Window Selection →  Waveform Generation - DDS phase accumulator + sine lookup table genneration
→ DMA + TIM → DAC`

### Hardware-Validated Waveforms

Captured on a Keysight InfiniiVision MSOX3014T (time domain + FFT):

| ![LFM Waveform](docs/images/LFM_waveform.jpeg) | ![CW Waveform](docs/images/CW_waveform.jpeg) |
|:---:|:---:|
| **LFM Waveform — Time Domain & FFT** | **CW Waveform — Time Domain & FFT** |

| ![GS](docs/images/GS_waveform.jpeg) | ![Barker-13](docs/images/Barker-13.jpeg) |
|:---:|:---:|
| **GS — Time Domain & FFT** | **Barker 13  — Time Domain & FFT** |

### Key Innovations

| # | Innovation | Description |
|---|---|---|
| 1 | **Low-Power Hardware DDS Architecture** | DDS + sine-LUT + Timer-triggered DMA streams samples to the DAC with minimal CPU involvement |
| 2 | **Real-Time Physics-Based Adaptation** | Live Mackenzie (sound speed) and Francois–Garrison (absorption) model evaluation drives every decision |
| 3 | **Adaptive Multi-Modulation** | Runtime selection between CW, LFM, Geometric Sweep and Barker-13 waveforms |
| 4 | **Two-Stage Frequency Selection** | Coarse-to-fine numerical search over candidate frequencies for lowest transmission loss at lowest power |

---

## System Architecture

The control-flow below mirrors the firmware decision logic (sensing → calibration → LUT cache check → adaptive computation → waveform synthesis → output):

```mermaid
flowchart TD
    A["Sense: T, S, D, Battery"] --> B["Core Compute:<br/>Calibration + Averaging"]
    B --> C{Battery OK?}
    C -- No --> D["Fallback: CW mode"]
    C -- Yes --> E{LUT cache hit?<br/>RAM + SD card}

    E -- Yes --> F["Waveform Generation<br/>(cached parameters)"]
    E -- No --> G["Calculate Sound Speed<br/>(Mackenzie)"]
    G --> H["Calculate Transmission Loss<br/>(Francois-Garrison)"]
    H --> I["Two-Stage Frequency Selection"]

    subgraph TS["Two-Stage Candidate Search"]
        I1["Coarse Frequency Selection"] --> I2["Promising Region"]
        I2 --> I3["Fine Frequency Selection"]
    end
    I --> I1
    I3 --> J["Parameter Selection<br/>B, Tp, Fs, fc"]
    J --> K["Modulation Select<br/>CW / LFM / Geometric Sweep / Barker-13"]
    K --> L["Windowing Select<br/>(Hann / Hamming / Blackman)"]
    L --> M["DDS Generation<br/>(Phase Accumulator + LUT)"]
    M --> N["Waveform Generation"]

    F --> O["Timer + DMA"]
    N --> O
    D --> O
    O --> P["DAC Output"]
```

---
```markdown
## Acoustic & Signal-Processing Model

All formulas are implemented for on-device (STM32G474) computation and are traceable end-to-end from raw sensor readings to the final DAC sample stream. Physical constants below (pH, range, SNR target, noise level, directivity index, target strength, range resolution, time-bandwidth product, sample rate) are **mission-specified inputs**, not part of the derivation — they are substituted into the formulas at run time or during procedure, never hard-coded into the equations themselves.

### Mission-Specified Parameters

| Symbol | Meaning | Units |
|---|---|---|
| `pH` | Water acidity (affects boric-acid relaxation term) | — |
| `R` | Target/operating range | m |
| `SNR_required` | Minimum detectable SNR for the mission | dB |
| `NL` | Ambient/receiver noise level | dB |
| `DI` | Transducer directivity index | dB |
| `TS` | Expected target strength | dB |
| `ΔR` | Required range resolution | m |
| `TB` | Time-bandwidth product (pulse-compression gain target) | — |
| `Fs` | DAC/DMA sample rate | Hz |

---

### 1. Sensor Calibration

Raw ADC counts are converted into physical units (temperature, depth, salinity, battery voltage) before being used anywhere else in the model:

```
x = a·ADC + b
```

`a` (scale/slope) and `b` (offset) are **predefined constants taken from the respective sensor's datasheet**, so no separate calibration procedure is needed. The firmware applies these fixed coefficients to every ADC reading.

### 2. Sound Speed — Mackenzie (1981) Equation

Valid for T: 2–30 °C, S: 25–40 ppt, D: 0–8000 m.

```
c(T,S,D) = 1448.96 + 4.591T − 0.05304T² + 0.0002374T³
         + 1.340(S−35) + 0.01630D + 1.675×10⁻⁷D²
         − 0.01025T(S−35) − 7.139×10⁻¹³T·D³
```

### 3. Range & Time-of-Flight (Monostatic)

For a monostatic system, the pulse travels to the target and back over the same range `R`:

```
t_flight = 2R / c
```

`R` is mission-specified; `c` comes from Section 2.

### 4. Frequency-Dependent Absorption — Francois & Garrison (1982)

Absorption α(f) (dB/km) is the sum of three relaxation contributions — boric acid, magnesium sulfate, and pure water — each frequency-dependent:

```
α(f) = (A1·P1·f1·f²)/(f1² + f²) + (A2·P2·f2·f²)/(f2² + f²) + A3·P3·f²
```

**Boric acid contribution:**
```
A1 = (8.86/c)·10^(0.78·pH − 5)
f1 = 2.8·√(S/35)·10^(4 − 1245/(T+273))
P1 = 1
```

**Magnesium sulfate contribution:**
```
A2 = 21.44·(S/c)·(1 + 0.025T)
f2 = 8.17·10^(8 − 1990/(T+273)) / (1 + 0.0018(S−35))
P2 = 1 − 1.37×10⁻⁴D + 6.2×10⁻⁹D²
```

**Pure water contribution:**
```
if T ≤ 20:  A3 = 4.937×10⁻⁴ − 2.59×10⁻⁵T + 9.11×10⁻⁷T² − 1.5×10⁻⁸T³
if T > 20:  A3 = 3.964×10⁻⁴ − 1.146×10⁻⁵T + 1.45×10⁻⁷T² − 6.5×10⁻¹⁰T³
P3 = 1 − 3.83×10⁻⁵D + 4.9×10⁻¹⁰D²
```

`f` is in kHz, `c` in m/s (from Mackenzie), `T` in °C, `S` in ppt, `D` in m, `pH` mission-specified.

### 5. Transmission Loss (One-Way)

```
TL(f) = 20·log10(R) + α(f)·R_km,      R_km = R / 1000
```

`R` is mission-specified; `α(f)` from Section 4. The active sonar equation (Section 6) doubles this to account for the outbound and return paths.

### 6. Active Sonar Equation

```
SNR = SL − 2·TL − (NL − DI) + TS

∴ SL = SNR_required + 2·TL + (NL − DI) − TS
```

- `SL` — required source level
- `SNR_required` — mission-specified
- `NL` — noise level (mission-specified, later replaced with measured receiver noise)
- `DI` — directivity index (transducer property, mission-specified)
- `TS` — target strength (mission-specified)

The resulting `SL` (dB re 1 µPa @ 1 m) is mapped to the DAC drive amplitude.

### 7. Frequency Optimization (Coarse → Fine)

Candidate frequencies are scored and the lowest-cost, hardware-feasible option is selected:

```
J(f) = 2·TL(f) + E_penalty(f) + H_hardware(f)
f* = argmin J(f)
```

Hard constraints (Nyquist limit from `Fs`, amplifier range) reject candidates outright rather than merely penalizing them.

### 8. Bandwidth

```
B = c / (2·ΔR)
```

`c` from Section 2; `ΔR` mission-specified (required range resolution).

### 9. Pulse Duration & Sample Count

```
Tp = TB / B
N  = Tp · Fs
```

`TB` and `Fs` are mission-specified.

### 10. Waveform Synthesis — LFM Chirp Rate

```
K = (f1 − f0) / Tp
```

`f0`, `f1` are the chirp's start/end frequencies (derived from `f*` ± B/2); `Tp` from Section 9.

### 11. DDS Phase Accumulator

```
Δφ[n] = 2π·f[n] / Fs
φ[n+1] = φ[n] + Δφ[n]
```

`Fs` mission-specified.

### 12. Sine Lookup Table Indexing

```
index = floor((φ mod 2π) / 2π · M)
```

`M` = number of entries in the precomputed sine LUT.

### 13. Windowing

```
w[n] = 0.5·(1 − cos(2π·n / (N−1)))      # Hann window
```

`N` from Section 9.

### 14. DAC Mapping

```
DAC[n] = 2047.5 + G·A·w[n]·sin(φ[n])    # 12-bit unsigned output
```

- `G` — calibration gain constant (bench-measured)
- `A` — amplitude derived from `SL` (Section 6)
- `w[n]` — window value (Section 13)

---

> **Note:** Sections 2, 3, 5, 6, 8–14 are pure math, computable on the STM32 with no extra hardware. Amplitude calibration (Section 14's gain constant `G`) and the SNR feedback loop (Section 15) require a hydrophone for measured, rather than assumed, values.
```
## Acoustic & Signal-Processing Model

All formulas are implemented for on-device (STM32G474) computation and are traceable end-to-end from raw sensor readings to the final DAC sample stream.

### 1. Sensor Calibration
 
Raw ADC counts are converted into physical units (temperature, depth, salinity, battery voltage) before being used anywhere else in the model:
 
```
x = a·ADC + b
```
 
Here `a` (scale/slope) and `b` (offset) are **predefined constants taken from the respective sensor's datasheet**, so no separate calibration procedure is needed. The firmware simply applies these fixed coefficients to every ADC reading.

### 2. Sound Speed — Mackenzie (1981) Equation

Valid for T: 2–30 °C, S: 25–40 ppt, D: 0–8000 m.

```
c(T,S,D) = 1448.96 + 4.591T − 0.05304T² + 0.0002374T³
         + 1.340(S−35) + 0.01630D + 1.675×10⁻⁷D²
         − 0.01025T(S−35) − 7.139×10⁻¹³T·D³
```

### 3. Range & Time-of-Flight (Monostatic)

```
# Predefined (Mission specified)
```

### 4–5. Frequency-Dependent Absorption & Transmission Loss

Absorption α(f, T, S, D, pH) is evaluated via the Francois–Garrison model; combined with spreading loss:

```
TL = 20·log10(R) + α·R_km
α = 
```

## 4. Active SONAR equation

$$SNR = SL - 2\,TL - (NL - DI) + TS$$

$$\therefore\; SL = SNR_{required} + 2\,TL + (NL - DI) - TS$$

- $SL$: source level
- $SNR_{required}$: mission spec
- $NL$: noise level (later, actual receiver noise level)
- $DI$: directivity index (transducer's property)
- $TS$: target strength
 
The factor of 2 on TL accounts for the signal losing energy on both the outbound and return paths. The resulting SL (in dB re 1 µPa @ 1 m) is then mapped to the DAC drive amplitude.

### 7. Frequency Optimization (Coarse → Fine)

Candidate frequencies are scored and the lowest-cost, hardware-feasible option is selected:

```
J(f) = 2·TL(f) + E_penalty(f) + H_hardware(f)
```

Hard constraints (Nyquist limit, amplifier range) reject candidates outright rather than merely penalizing them.

### 8–9. Bandwidth, Pulse Duration & Sample Count

```
B  = c / (2·ΔR)                 # from required range resolution
Tp = TB_required / B            # from time-bandwidth product
N  = Tp · Fs
```

### 10–12. Waveform Synthesis — LFM / DDS / LUT

```
K = (f1 − f0) / Tp                          # chirp rate
Δφ[n] = 2π·f[n] / Fs
φ[n+1] = φ[n] + Δφ[n]
index  = floor((φ mod 2π) / 2π · M)         # M-entry sine LUT
```

### 13–14. Windowing & DAC Mapping

```
w[n]    = 0.5·(1 − cos(2π·n / (N−1)))       # Hann window
DAC[n]  = 2047.5 + G · A · w[n] · sin(φ[n]) # 12-bit unsigned output
```


> **Note:** Sections 2, 3, 5, 6, 8–14 are pure math, computable on the STM32 with no extra hardware. Amplitude calibration (Section 14's gain constant `G`) and the SNR feedback loop (Section 15) require a hydrophone for measured, rather than assumed, values.


---

## Adaptive Modulation

| Waveform | Role |
|---|---|
| **CW** | Fallback / low-battery mode, narrowband |
| **LFM (Chirp)** | Pulse compression for range resolution in higher-loss conditions |
| **Geometric Sweep (GS)** | Alternate wideband sweep profile |
| **Barker-13** | Phase-coded pulse compression for clutter rejection |

Modulation and windowing choice are driven by the live acoustic-loss and SNR estimates produced by the models above, not fixed at compile time.

---

## Technology Stack

| Category | Tools |
|---|---|
| Firmware | STM32CubeIDE, STM32CubeMX, STM32CubeProgrammer |
| Target MCU | STM32G474 |
| Simulation & Modeling | MATLAB, LTspice |
| PCB / Schematic | KiCad |
| Signal Analysis | DSP with FFT, Oscilloscope, Spectrum Analyzer |
| Mechanical Design | Autodesk Fusion 360 |
| Prototyping | STM32F407VGT6 (Discovery 1) |

---

## Repository Structure

```
SparkX-SIH26058/
│
├── firmware/                 STM32G474 embedded C firmware
│   └── main.c                  DDS engine, sensor acquisition, decision logic
│                               Headers / configuration
│
├── docs/                     Design documentation
│   └── images/                Flowcharts, oscilloscope captures, CAD renders
│
├── simulation/                MATLAB models (waveform + propagation simulation)
│
├── hardware/                   wiring reference
│
├── mechanical/cad/             AUV payload CAD (Fusion 360 exports)
│
├── LICENSE
└── README.md
```

> Adjust folder names above to match your actual firmware/hardware layout before pushing.

---

## Getting Started

### Prerequisites

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- STM32G474 Nucleo/dev board (or target hardware)
- MATLAB (for simulation/verification, optional)


### Build & Flash Firmware

1. Open `firmware/` as a project in STM32CubeIDE.
2. Build the project.
3. Flash to the STM32G474 target via ST-Link.

### Run the MATLAB Simulation

Open `simulation/` in MATLAB and run the top-level script to verify waveform generation and propagation-loss calculations before deploying to hardware.

---

## Feasibility & Risk Mitigation

| Challenge | Mitigation |
|---|---|
| Illustrative mission values used in design | Replace with measured mission data during field testing |
| Amplitude calibration uncertainty | Calibrate using actual hydrophone measurements |
| SNR estimation vs. measured SNR | Validate experimentally with transmitter + receiver pair |
| Absorption model validity | Apply Francois-Garrison only within its validated frequency/environmental ranges |
| LUT extrapolation beyond validated conditions | Restrict adaptive LUT to validated frequency, amplitude and environmental regions |

---

## Impact & Applications

- Underwater mapping and bathymetric surveying
- Inspection of underwater infrastructure (pipelines, ports, offshore installations)
- Marine research and oceanography
- Search and rescue operations
- Environmental / ecosystem monitoring
- Defence and maritime security

---

## Current Prototype Scope

**Demonstrated**
- Environmental sensing + calibration pipeline
- Mackenzie sound-speed and Francois-Garrison absorption computation
- Two-stage coarse-to-fine frequency optimization
- DDS + LUT waveform synthesis (CW, LFM, Geometric Sweep, Barker-13)
- Timer + DMA driven DAC output
- MATLAB simulation of the full signal chain

**Future Development**
- Physical hydrophone-based amplitude and SNR calibration
- Dedicated transmitter PCB
- Power amplifier and transducer matching network
- Pressure-rated waterproof payload enclosure
- End-to-end in-water acoustic field testing

---

## License

This project is released under the [MIT License](LICENSE).

---

**Team SparkX | SIH26058 | K. K. Wagh Institute of Engineering Education & Research, Nashik**
