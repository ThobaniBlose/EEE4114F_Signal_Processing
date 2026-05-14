/*
 * wave_mode.c
 *
 * Determines wave processing mode from sensing subsystem CSV fields.
 *
 * MATLAB-equivalent logic:
 * - GEOGRAPHIC only if heading, roll, pitch and magnetometer are usable.
 * - BODY_RELATIVE if motion is available but geographic heading is not reliable.
 * - FALLBACK / VERTICAL_ONLY only if motion itself is unusable.
 */

#include "wave_mode.h"
#include <math.h>
#include <stdint.h>

#define MAG_NORM_MIN_UT    20.0f
#define MAG_NORM_MAX_UT    90.0f
#define MAG_HORIZ_MIN_UT    5.0f
#define MAX_ABS_ROLL_DEG   45.0f
#define MAX_ABS_PITCH_DEG  45.0f

static int field_valid(float v)
{
    return isfinite(v);
}

static int motion_valid(const CsvImuSample_t *s)
{
    return field_valid(s->ax_g) &&
           field_valid(s->ay_g) &&
           field_valid(s->az_g);
}

static int heading_ok(float h)
{
    return field_valid(h) && h >= 0.0f && h < 360.0f;
}

static int attitude_valid(const CsvImuSample_t *s)
{
    if (!field_valid(s->roll_deg) || !field_valid(s->pitch_deg))
        return 0;
    if (fabsf(s->roll_deg) > MAX_ABS_ROLL_DEG)
        return 0;
    if (fabsf(s->pitch_deg) > MAX_ABS_PITCH_DEG)
        return 0;
    return 1;
}

static int mag_valid(const CsvImuSample_t *s)
{
    if (!field_valid(s->mx) || !field_valid(s->my) || !field_valid(s->mz))
        return 0;
    float mag_norm  = sqrtf(s->mx*s->mx + s->my*s->my + s->mz*s->mz);
    float mag_horiz = sqrtf(s->mx*s->mx + s->my*s->my);
    if (mag_norm < MAG_NORM_MIN_UT || mag_norm > MAG_NORM_MAX_UT)
        return 0;
    if (mag_horiz < MAG_HORIZ_MIN_UT)
        return 0;
    return 1;
}

int wave_mode_decide(const CsvImuSample_t *sample,
                     WaveModeResult_t     *result)
{
    if (sample == 0 || result == 0) return -1;

    uint8_t motion_ok_f   = motion_valid(sample)            ? 1U : 0U;
    uint8_t heading_ok_f  = heading_ok(sample->heading_deg) ? 1U : 0U;
    uint8_t attitude_ok_f = attitude_valid(sample)          ? 1U : 0U;
    uint8_t mag_ok_f      = mag_valid(sample)               ? 1U : 0U;

    result->heading_valid = heading_ok_f;
    result->roll_valid    = field_valid(sample->roll_deg)  ? 1U : 0U;
    result->pitch_valid   = field_valid(sample->pitch_deg) ? 1U : 0U;
    result->mag_valid     = mag_ok_f;

    if (!motion_ok_f) {
        /* No usable motion data — true vertical-only fallback */
        result->mode = WAVE_MODE_FALLBACK;
    }
    else if (heading_ok_f && attitude_ok_f && mag_ok_f) {
        /* Full geographic mode */
        result->mode = WAVE_MODE_GEOGRAPHIC;
    }
    else {
        /* Motion exists but Earth-frame heading not reliable.
         * Matches MATLAB BODY_FRAME_FALLBACK / BODY_RELATIVE. */
        result->mode = WAVE_MODE_BODY_RELATIVE;
    }

    return 0;
}

const char *wave_mode_label(WaveMode_t mode)
{
    switch (mode) {
        case WAVE_MODE_GEOGRAPHIC:    return "GEOGRAPHIC";
        case WAVE_MODE_BODY_RELATIVE: return "BODY_RELATIVE";
        case WAVE_MODE_FALLBACK:      return "VERTICAL_ONLY";
        default:                      return "UNKNOWN";
    }
}
