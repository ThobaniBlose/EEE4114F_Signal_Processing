# SHARC Wave Parameter Processor — STM32 Embedded DSP Subsystem

Embedded signal processing subsystem for the SHARC (Smart Heave And Roll Characteriser) wave measurement buoy. Runs on the STM32L4R5ZI (NUCLEO-L4R5ZI-P) and computes non-directional and directional wave parameters from IMU acceleration data.

## What It Does

Processes raw 3-axis accelerometer data (100 Hz, in g) and produces a complete wave measurement output:

```
T1,S001,UTC=NO_GPS,GPS=0,LAT=nan,LON=nan,
Hm0=1.014,Tp=18.204,Tm01=16.606,Tm02=16.373,
DIR=18.9,REF=BODY_RELATIVE,Q=1,R1=0.681,COH=0.398
```

## Processing Pipeline

```
IMU CSV (ax, ay, az in g)
    ↓
Mean removal + g → m/s² conversion
    ↓
Butterworth bandpass filtfilt (0.02–0.50 Hz, SOS biquad cascade)
    ↓
Trapezoidal integration → velocity
    ↓
High-pass filtfilt (0.02 Hz) → drift suppression
    ↓
Integration → displacement (eta)
    ↓
High-pass filtfilt → clean eta
    ↓
Welch PSD (win=8192, nfft=16384) → Hm0, Tp, Tm01, Tm02
    ↓
Cross-spectral direction estimation → mean direction, r1, coherence
    ↓
Automatic mode decision + Tier-1 packet output
```

## Validation Results

Validated against MATLAB golden reference (S001_IMU.csv truncated replay segment):

| Parameter | STM32 | MATLAB | Error |
|-----------|--------|--------|-------|
| Hm0 | 1.013655 m | 1.013638 m | 0.002% |
| Tp | 18.204445 s | 18.204444 s | 0.000% |
| Tm01 | 16.606028 s | 16.606034 s | 0.000% |
| Tm02 | 16.372871 s | 16.372861 s | 0.000% |
| Direction | 18.940° | 18.940° | <0.001° |
| r1 band | 0.6814 | 0.6814 | <0.001 |

## Acceptance Test Results

| Test | Description | Result |
|------|-------------|--------|
| ATP3 | Body-to-North/East rotation (5 cases) | All errors < 0.000001 |
| ATP4A | Geographic peak-bin direction | Error: 0.000011° |
| ATP4B | Band-integrated direction (0.08–0.40 Hz) | Error: 0.000029° |

## Project Structure

```
Core/Inc/
  sharc_process.h         — Single-pass window processor API
  wave_full_pipeline.h    — Filter coefficients, SOS, integration, filtfilt
  wave_mode.h             — Mode decision (GEOGRAPHIC/BODY_RELATIVE/FALLBACK)
  wave_packet.h           — Tier-1 output packet formatter
  wave_types.h            — Shared enums
  direction_processor.h   — Body→Earth frame rotation
  csv_imu_reader.h        — CSV parser for sensing subsystem format
  wave_processor.h        — Legacy Welch PSD processor (acceleration-domain)
  s001_replay_segment.h   — Real S001_IMU.csv replay data (32768 samples)

Core/Src/
  sharc_process.c         — Production single-pass processor
  wave_full_pipeline.c    — SOS filtfilt, cumtrapz, cross-spectra
  wave_mode.c             — Automatic mode decision from data
  wave_packet.c           — Tier-1 ASCII packet formatter
  direction_processor.c   — Heading-based frame rotation
  csv_imu_reader.c        — CSV parser with pluggable line source
  wave_processor.c        — Legacy acceleration-domain processor
  s001_replay_segment.c   — Replay data arrays (all 13 IMU columns)
  main.c                  — Test harness + ATP validation
```

## Hardware

- **MCU:** STM32L4R5ZIT6P (Cortex-M4, 120 MHz, 640 KB RAM, 2 MB Flash)
- **Board:** NUCLEO-L4R5ZI-P
- **Debug output:** LPUART1 via ST-LINK VCP (115200 8N1)
- **FPU:** Hardware single-precision + software double for filter state

## Build

Open in STM32CubeIDE. Only GPIO and LPUART1 peripherals are enabled. Build the Debug configuration.

## Mode Decision Logic

The processor automatically selects the processing mode from the data:

- **GEOGRAPHIC:** heading, roll, pitch, and magnetometer all valid → full Earth-frame direction
- **BODY_RELATIVE:** motion valid but heading/mag missing → body-frame relative direction
- **FALLBACK:** motion data unusable → vertical-only (no direction)

## Key Design Decisions

- **SOS biquad cascade** for IIR filters — numerically stable in single precision
- **Reflection padding + initial conditions** for filtfilt — matches MATLAB to 7 significant figures
- **Recursive trig oscillator** for DFT — eliminates per-sample cos/sin calls
- **Memory-efficient cross-spectra** — eta stored once, vx/vy computed sequentially, reusing one buffer
- **No CMSIS-DSP dependency** — pure C implementation, portable

## Reference Algorithm

`PIPE_2GNSS_Wave_Direction.m` — MATLAB golden reference algorithm for the full directional wave processing pipeline.

## Author

Thobani Blose — EEE4114F Signal Processing Subsystem, EEE4113F University of Cape Town, 2026
