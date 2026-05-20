# SHARC Wave Parameter Processor

Embedded signal processing subsystem for the SHARC wave measurement buoy. This runs on the STM32L4R5ZI and computes wave height, period, and direction from raw IMU acceleration data.

## Overview

The processor takes 3-axis accelerometer data sampled at 100 Hz and outputs a compact measurement packet containing significant wave height (Hm0), peak period (Tp), mean periods (Tm01, Tm02), wave direction, and quality indicators. It automatically detects whether heading information is available and selects the appropriate processing mode.

Example output:

    T1,S001,UTC=NO_GPS,GPS=0,LAT=nan,LON=nan,Hm0=1.014,Tp=18.204,
    Tm01=16.606,Tm02=16.373,DIR=18.9,REF=BODY_RELATIVE,Q=1,R1=0.681,COH=0.398

## How It Works

The processing chain converts raw acceleration into wave parameters through these stages:

1. Convert acceleration from g to m/s squared and remove the mean (gravity)
2. Bandpass filter the dynamic acceleration (0.02 to 0.50 Hz) using a forward-backward Butterworth filter implemented as second-order sections
3. Integrate filtered acceleration to get velocity, then high-pass filter to suppress drift
4. Integrate velocity to get surface displacement (eta), high-pass filter again
5. Compute the displacement power spectrum using Welch's method and extract Hm0, Tp, Tm01, Tm02 from spectral moments
6. Compute cross-spectra between eta and horizontal velocity channels to estimate wave direction, spreading (r1), and coherence
7. Format a Tier-1 output packet

## Validation

The STM32 implementation was validated against a MATLAB golden reference using a truncated segment from real ocean data (S001_IMU.csv, WS23 dataset). All wave parameters match to better than 0.01 percent. The directional output was verified using synthetic test cases with known incoming wave direction.

Key results:

- Hm0 error: 0.002 percent
- Tp error: less than 0.001 percent
- Direction error on synthetic cases: less than 0.001 degrees
- Body-to-Earth frame rotation error: less than 0.000001 on all test cases

## Processing Modes

The system decides its mode automatically by scanning the input data:

- Geographic mode: all sensor channels valid including heading and magnetometer. Produces Earth-referenced wave direction.
- Body-relative mode: motion data valid but heading or magnetometer missing. Produces direction relative to the buoy body frame.
- Fallback mode: motion data itself is unusable. Produces vertical-only wave parameters with no direction.

## Project Files

The main processing modules are in Core/Inc and Core/Src:

- sharc_process: the single-pass window processor that runs the full pipeline
- wave_full_pipeline: filter coefficients, SOS filtfilt, integration, cross-spectral DFT
- wave_mode: automatic mode decision logic
- wave_packet: Tier-1 output packet formatter
- direction_processor: body-frame to Earth-frame rotation
- csv_imu_reader: CSV parser for the sensing subsystem data format
- s001_replay_segment: real ocean data replay arrays for validation

## Hardware

- MCU: STM32L4R5ZIT6P (Cortex-M4, 640 KB RAM, 2 MB Flash)
- Board: NUCLEO-L4R5ZI-P
- Debug output: LPUART1 via ST-LINK virtual COM port at 115200 baud
- FPU: hardware single-precision, with double-precision used for filter state variables

## Design Notes

- IIR filters use second-order sections (biquad cascade) for numerical stability in single precision
- The filtfilt implementation uses reflection padding and approximate initial conditions to match MATLAB output to seven significant figures
- Cross-spectral DFT uses a recursive trig oscillator to avoid calling cos/sin on every sample
- Memory is managed carefully: eta is stored once, horizontal channels are computed sequentially reusing a single buffer
- No external DSP library dependency. The implementation is pure C and portable.

## Building

Open the project in STM32CubeIDE and build the Debug configuration. Only GPIO and LPUART1 are enabled for the validation build. The SD card integration branch adds SPI1 and FATFS.

## Author

Thobani Blose, EEE4113F Signal Processing Subsystem, University of Cape Town, 2026.
