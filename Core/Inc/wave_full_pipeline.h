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

/* Peak bin for S001 truncated segment validation */
#define WFP_PEAK_K_S001_TRUNC   (9U)

/* Direction confidence thresholds */
#define WFP_DIR_R1_THRESH        (0.20f)
#define WFP_DIR_COH_THRESH       (0.30f)
#define WFP_DIR_MIN_VALID_BINS   (3U)

/* Cross-spectral bin result */
typedef struct {
    double p_eta;
    double p_h;
    double cross_re;
    double cross_im;
    uint32_t n_segs;
} WfpCrossBin_t;

/* Peak-bin direction result */
typedef struct {
    float peak_from_deg;
    float peak_toward_deg;
    float r1_peak;
    float vector_coh_peak;
    uint32_t n_segs;
} WfpDirectionPeak_t;

/* Band-integrated direction result */
typedef struct {
    float mean_from_deg;
    float mean_toward_deg;
    float peak_from_deg;
    float peak_toward_deg;
    float r1_band;
    float r1_peak;
    float mean_vector_coh;
    uint32_t direction_confident;
    uint32_t valid_bins;
    uint32_t peak_k;
    float peak_freq_hz;
} WfpDirectionBand_t;

/* Forward-backward (filtfilt) bandpass filter using SOS, in-place.
 * Buffer must be at least n + 2*WFP_FILTFILT_PAD_BP floats. */
void wfp_filtfilt_bp_order4_inplace(float *x, uint32_t n);

/* Forward-backward (filtfilt) high-pass filter using SOS, in-place.
 * Buffer must be at least n + 2*WFP_FILTFILT_PAD_HP floats. */
void wfp_filtfilt_hp_order2_inplace(float *x, uint32_t n);

/* Cumulative trapezoidal integration, in-place. */
void wfp_cumtrapz_inplace(float *x, uint32_t n, float dt_s);

/* Reconstruct body-frame velocity from acceleration in g */
void wfp_reconstruct_velocity_from_accel_g(const float *accel_g,
                                           float       *out_velocity,
                                           uint32_t     n);

/* Reconstruct vertical displacement from az in g */
void wfp_reconstruct_eta_from_az_g(const float *az_g,
                                   float       *out_eta,
                                   uint32_t     n);

/* Compute cross-spectral bin between eta and horizontal channel h */
void wfp_cross_bin_eta_h(const float   *eta,
                         const float   *h,
                         uint32_t       n,
                         uint32_t       k_bin,
                         WfpCrossBin_t *out);

/* Compute peak-bin direction from eta-x and eta-y cross bins */
void wfp_direction_peak_from_cross(const WfpCrossBin_t *eta_x,
                                   const WfpCrossBin_t *eta_y,
                                   WfpDirectionPeak_t  *out);

/* Compute band-integrated direction from arrays of cross bins */
void wfp_direction_band_from_cross(const WfpCrossBin_t *eta_x_bins,
                                   const WfpCrossBin_t *eta_y_bins,
                                   WfpDirectionBand_t  *out);

#endif /* WAVE_FULL_PIPELINE_H */
