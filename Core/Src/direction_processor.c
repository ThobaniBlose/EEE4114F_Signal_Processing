/*
 * direction_processor.c
 *
 * Horizontal frame rotation: body frame → Earth frame (North/East).
 *
 * This is the first stage of the directional wave processing pipeline.
 * It does not compute the directional spectrum yet — it only prepares
 * the horizontal acceleration channels in the correct Earth frame,
 * which are needed for cross-spectral direction estimation.
 *
 * Reference: PIPE_2GNSS_Wave_Direction.m, STEP 7 frame rotation.
 */

#include "direction_processor.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int direction_processor_rotate_horizontal(const float              *ax_body_ms2,
                                           const float              *ay_body_ms2,
                                           const float              *heading_deg,
                                           float                    *a_north_ms2,
                                           float                    *a_east_ms2,
                                           uint32_t                  n_samples,
                                           DirectionProcessor_Result *result)
{
    if (ax_body_ms2 == 0 || ay_body_ms2 == 0 || heading_deg == 0) return -1;
    if (a_north_ms2 == 0 || a_east_ms2  == 0 || result      == 0) return -1;
    if (n_samples == 0U) return -2;

    for (uint32_t i = 0U; i < n_samples; i++) {
        float psi = heading_deg[i] * ((float)M_PI / 180.0f);
        float c   = cosf(psi);
        float s   = sinf(psi);
        float ax  = ax_body_ms2[i];
        float ay  = ay_body_ms2[i];

        a_north_ms2[i] = ax * c - ay * s;
        a_east_ms2[i]  = ax * s + ay * c;
    }

    result->n_samples = n_samples;
    return 0;
}
