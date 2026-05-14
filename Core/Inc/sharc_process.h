#ifndef SHARC_PROCESS_H
#define SHARC_PROCESS_H

#include <stdint.h>
#include "wave_full_pipeline.h"
#include "wave_types.h"

/* ---------------------------------------------------------------------------
 * sharc_process — single-pass window processor.
 *
 * Performs the complete SHARC wave processing pipeline in one call:
 *   1. Mode decision from data (GEOGRAPHIC / BODY_RELATIVE / FALLBACK)
 *   2. Reconstruct eta, vx_body, vy_body (each computed once)
 *   3. Non-directional wave parameters: Hm0, Tp, Tm01, Tm02
 *   4. Directional estimation: mean direction, r1, coherence
 *   5. Confidence assessment
 *
 * Input: raw acceleration arrays in g, plus heading/mag/roll/pitch arrays.
 * Output: complete result struct ready for Tier-1 packet formatting.
 * --------------------------------------------------------------------------- */

typedef struct {
    /* Non-directional */
    float Hm0;
    float fp;
    float Tp;
    float Tm01;
    float Tm02;

    /* Directional */
    float mean_from_deg;
    float peak_from_deg;
    float r1_band;
    float r1_peak;
    float mean_vector_coh;
    uint32_t direction_confident;
    uint32_t valid_bins;

    /* Mode */
    WaveMode_t mode;

    /* Metadata */
    uint32_t n_samples;
    uint32_t n_segs;
} SharcResult_t;

/* ---------------------------------------------------------------------------
 * sharc_process_window
 *
 * Process one window of IMU data. All arrays must have n_samples elements.
 * ax_g, ay_g, az_g are in units of g.
 * heading_deg, roll_deg, pitch_deg, mx, my, mz are used for mode decision.
 *
 * Returns 0 on success, negative on error.
 * --------------------------------------------------------------------------- */
int sharc_process_window(const float *ax_g,
                         const float *ay_g,
                         const float *az_g,
                         const float *mx,
                         const float *my,
                         const float *mz,
                         const float *heading_deg,
                         const float *roll_deg,
                         const float *pitch_deg,
                         uint32_t     n_samples,
                         SharcResult_t *result);

#endif /* SHARC_PROCESS_H */
