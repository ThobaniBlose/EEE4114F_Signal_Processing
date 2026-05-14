#ifndef DIRECTION_PROCESSOR_H
#define DIRECTION_PROCESSOR_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Direction processor — horizontal frame rotation and directional estimation.
 *
 * Convention:
 *   Body frame:  x = forward axis, y = right axis
 *   heading_deg: clockwise from North [deg]
 *
 * Rotation to Earth frame (North/East):
 *   a_N = ax*cos(psi) - ay*sin(psi)
 *   a_E = ax*sin(psi) + ay*cos(psi)
 *
 * Reference: PIPE_2GNSS_Wave_Direction.m, STEP 7 frame rotation.
 * --------------------------------------------------------------------------- */

typedef struct {
    uint32_t n_samples;
} DirectionProcessor_Result;

/* ---------------------------------------------------------------------------
 * direction_processor_rotate_horizontal
 *
 * Rotate body-frame horizontal acceleration into Earth-frame North/East.
 * All arrays must be caller-allocated with size >= n_samples.
 *
 * Returns 0 on success, -1 bad args, -2 n_samples == 0.
 * --------------------------------------------------------------------------- */
int direction_processor_rotate_horizontal(const float              *ax_body_ms2,
                                           const float              *ay_body_ms2,
                                           const float              *heading_deg,
                                           float                    *a_north_ms2,
                                           float                    *a_east_ms2,
                                           uint32_t                  n_samples,
                                           DirectionProcessor_Result *result);

#endif /* DIRECTION_PROCESSOR_H */
