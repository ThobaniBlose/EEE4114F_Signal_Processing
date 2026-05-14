#ifndef WAVE_FULL_PIPELINE_H
#define WAVE_FULL_PIPELINE_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Full-pipeline wave processor — matches accepted MATLAB S001 pipeline.
 *
 * Processing chain:
 *   ax/ay/az [g] → m/s² → mean removal → bandpass filter → integrate →
 *   high-pass → integrate → high-pass → Welch cross-spectra → wave params
 *
 * Filter design: Butterworth, matches MATLAB butter() exactly.
 * Integration: cumulative trapezoidal, matches MATLAB cumtrapz().
 * Filtering: filtfilt (forward + backward), matches MATLAB filtfilt().
 * --------------------------------------------------------------------------- */

#define WFP_FS_HZ          (100.0f)
#define WFP_G0             (9.80665f)

#define WFP_BP_ORDER       (4U)       /* 2nd-order bandpass = 4th-order IIR */
#define WFP_BP_NCOEFF      (5U)

#define WFP_HP_ORDER       (2U)
#define WFP_HP_NCOEFF      (3U)

#define WFP_WELCH_WIN_LEN  (8192U)
#define WFP_WELCH_NOVERLAP (4096U)
#define WFP_WELCH_NFFT     (16384U)

#define WFP_FILTER_LO_HZ   (0.02f)
#define WFP_FILTER_HI_HZ   (0.50f)
#define WFP_WAVE_LO_HZ     (0.04f)
#define WFP_WAVE_HI_HZ     (0.40f)

/* Derived bin mapping for directional processing */
#define WFP_DF_HZ          (WFP_FS_HZ / (float)WFP_WELCH_NFFT)
#define WFP_WAVE_K_MIN     ((uint32_t)(WFP_WAVE_LO_HZ / WFP_DF_HZ + 0.999f))
#define WFP_WAVE_K_MAX     ((uint32_t)(WFP_WAVE_HI_HZ / WFP_DF_HZ))
#define WFP_WAVE_N_BINS    (WFP_WAVE_K_MAX - WFP_WAVE_K_MIN + 1U)
#define WFP_N_SEGS_32768   (7U)

/* Filter coefficients — exported for verification in Step 7 */
extern const float wfp_bp_b[WFP_BP_NCOEFF];
extern const float wfp_bp_a[WFP_BP_NCOEFF];
extern const float wfp_hp_b[WFP_HP_NCOEFF];
extern const float wfp_hp_a[WFP_HP_NCOEFF];

/* ---------------------------------------------------------------------------
 * Processing functions
 * --------------------------------------------------------------------------- */

/* Compute RMS of a float array */
float wfp_rms(const float *x, uint32_t n);

/* Convert g array to mean-removed m/s² */
void wfp_make_dynamic_accel_ms2(const float *x_g,
                                float       *x_dyn_ms2,
                                uint32_t     n);

#define WFP_FILTFILT_PAD_BP  (12U)    /* reflection padding samples per edge */
#define WFP_FILTFILT_PAD_HP  (6U)    /* reflection padding for high-pass */

/* Forward-backward (filtfilt) bandpass filter using SOS, in-place.
 * Buffer must be at least n + 2*WFP_FILTFILT_PAD_BP floats. */
void wfp_filtfilt_bp_order4_inplace(float *x, uint32_t n);

/* Forward-backward (filtfilt) high-pass filter using SOS, in-place.
 * Buffer must be at least n + 2*WFP_FILTFILT_PAD_HP floats. */
void wfp_filtfilt_hp_order2_inplace(float *x, uint32_t n);

/* Cumulative trapezoidal integration, in-place. */
void wfp_cumtrapz_inplace(float *x, uint32_t n, float dt_s);

#endif /* WAVE_FULL_PIPELINE_H */
