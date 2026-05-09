#ifndef WAVE_PARAMS_H
#define WAVE_PARAMS_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Configuration
 *
 * fs = 100 Hz, segment = 1024 samples = 10.24 s
 * df = 100 / 1024 = 0.0977 Hz
 * Wave band 0.05–0.50 Hz → bins k=1 to k=5
 *
 * 30-minute Welch accumulation:
 *   180,000 samples, 50% overlap → ~351 segments averaged
 *   RAM: segment(4KB) + overlap(2KB) + PSD(2KB) = ~8 KB total
 * --------------------------------------------------------------------------- */
#define WAVE_N      1024U       /* Segment length — must be power of 2        */
#define WAVE_FS     100.0f      /* Sample rate [Hz]                           */
#define WAVE_F_LO   0.05f       /* Lower wave-band limit [Hz]                 */
#define WAVE_F_HI   0.50f       /* Upper wave-band limit [Hz]                 */
#define WAVE_OVERLAP (WAVE_N / 2U)   /* 50% overlap                          */

/* Debug: print intermediate results every N segments (0 = disabled) */
#define WAVE_DEBUG_INTERVAL  10U

/* ---------------------------------------------------------------------------
 * Result struct
 * --------------------------------------------------------------------------- */
typedef struct {
    float    Hm0;      /* Significant wave height  4*sqrt(m0)   [m]  */
    float    Tp;       /* Peak period              1/f_peak     [s]  */
    float    Tm01;     /* Mean period              m0/m1        [s]  */
    float    Tm02;     /* Mean zero-crossing       sqrt(m0/m2)  [s]  */
    uint32_t n_segs;   /* Segments accumulated so far                */
} WaveParams_t;

/* ---------------------------------------------------------------------------
 * Public API
 *
 *   wave_init()          reset all state
 *   wave_push_sample()   feed one sample [m] at WAVE_FS Hz
 *                        returns 1 each time a segment is processed
 *   wave_get_params()    compute and return current averaged result
 * --------------------------------------------------------------------------- */
void wave_init(void);
int  wave_push_sample(float eta);
void wave_get_params(WaveParams_t *out);

#endif /* WAVE_PARAMS_H */
