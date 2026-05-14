#ifndef WAVE_TYPES_H
#define WAVE_TYPES_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Shared types used across wave processing modules.
 * --------------------------------------------------------------------------- */

/* Processing mode — determined by which sensing subsystem fields are present */
typedef enum {
    WAVE_MODE_GEOGRAPHIC    = 0,  /* Full directional: heading + mag available  */
    WAVE_MODE_BODY_RELATIVE = 1,  /* Heading available, no mag — body-relative  */
    WAVE_MODE_FALLBACK      = 2,  /* No heading/mag — vertical only, no direction */
} WaveMode_t;

/* Quality flag for the output packet */
typedef enum {
    WAVE_QUALITY_GOOD     = 0,  /* All channels valid                          */
    WAVE_QUALITY_DEGRADED = 1,  /* Some channels missing, fallback used        */
    WAVE_QUALITY_INVALID  = 2,  /* Insufficient data for any estimate          */
} WaveQuality_t;

#endif /* WAVE_TYPES_H */
