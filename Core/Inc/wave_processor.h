#ifndef WAVE_PROCESSOR_H
#define WAVE_PROCESSOR_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Wave processor configuration
 *
 * These values are set to match the WS23 MATLAB replay reference.
 * To use with a different dataset, update these defines and regenerate
 * the Hann window by calling wave_processor_init() again.
 * --------------------------------------------------------------------------- */
#define WP_FS_HZ        100.0f      /* sample rate [Hz]                        */
#define WP_WIN_LEN      8192U       /* segment length [samples]                */
#define WP_NFFT         32768U      /* zero-padded DFT length                  */
#define WP_HOP_LEN      (WP_WIN_LEN / 2U)   /* 50% overlap                    */
#define WP_F1_HZ        0.040f      /* low-freq taper start [Hz]               */
#define WP_F2_HZ        0.050f      /* low-freq taper end [Hz]                 */
#define WP_F_HI_HZ      0.500f      /* upper wave-band limit [Hz]              */

/* ---------------------------------------------------------------------------
 * Result struct — filled by wave_processor_run()
 * --------------------------------------------------------------------------- */
typedef struct {
    float    Hm0;       /* significant wave height  4*sqrt(m0)   [m]  */
    float    Tp;        /* peak period              1/f_peak     [s]  */
    float    Tm01;      /* mean period              m0/m1        [s]  */
    float    Tm02;      /* mean zero-crossing       sqrt(m0/m2)  [s]  */
    float    f_peak;    /* peak frequency                        [Hz] */
    float    m0;        /* zeroth spectral moment                [m²] */
    float    m1;        /* first spectral moment                 [m²·Hz] */
    float    m2;        /* second spectral moment                [m²·Hz²] */
    uint32_t n_segs;    /* number of segments processed               */
} WaveProcessor_Result;

/* ---------------------------------------------------------------------------
 * Public API
 *
 *   wave_processor_init()   — build Hann window, compute bin limits
 *                             call once after peripheral init
 *
 *   wave_processor_run()    — process az_ms2[n_samples] acceleration data
 *                             fills *result with Hm0, Tp, Tm01, Tm02
 *                             az_ms2 must already have mean removed
 * --------------------------------------------------------------------------- */
void wave_processor_init(void);

int  wave_processor_run(const float          *az_ms2,
                        uint32_t              n_samples,
                        WaveProcessor_Result *result);

#endif /* WAVE_PROCESSOR_H */
