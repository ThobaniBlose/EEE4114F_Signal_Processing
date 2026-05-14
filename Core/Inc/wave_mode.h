#ifndef WAVE_MODE_H
#define WAVE_MODE_H

#include "wave_types.h"
#include "csv_imu_reader.h"

/* ---------------------------------------------------------------------------
 * wave_mode — determines processing mode from a parsed CSV sample.
 *
 * Logic (matches PIPE_2GNSS_Wave_Direction.m fallback conditions):
 *   GEOGRAPHIC    : heading, roll, pitch all valid AND magnetometer valid
 *   BODY_RELATIVE : heading valid, but magnetometer missing/NaN
 *   FALLBACK      : heading missing/NaN — vertical only, no direction
 *
 * NaN detection: a field is considered missing if it is NaN or exactly 0.0
 * when all three of heading/roll/pitch are zero (sensing subsystem placeholder).
 * --------------------------------------------------------------------------- */

typedef struct {
    WaveMode_t mode;
    uint8_t    heading_valid;
    uint8_t    mag_valid;
    uint8_t    roll_valid;
    uint8_t    pitch_valid;
} WaveModeResult_t;

/* Determine processing mode from one parsed CSV sample.
 * Returns 0 always (result always filled). */
int wave_mode_decide(const CsvImuSample_t *sample,
                     WaveModeResult_t     *result);

/* Return a short string label for the mode (for UART printing). */
const char *wave_mode_label(WaveMode_t mode);

#endif /* WAVE_MODE_H */
