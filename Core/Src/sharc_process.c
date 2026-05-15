/*
 * sharc_process.c
 *
 * Single-pass window processor for the SHARC wave buoy DSP subsystem.
 * Computes non-directional wave parameters and body-relative direction
 * from one window of IMU data.
 *
 * Each signal (eta, vx, vy) is reconstructed exactly once.
 */

#include "sharc_process.h"
#include "wave_full_pipeline.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------------------------------------------------------------------------
 * Internal buffers — shared with main.c for ATP tests via extern.
 * buf_a: primary working buffer (reused for each channel)
 * buf_eta: stores eta for cross-spectral computation
 * --------------------------------------------------------------------------- */
float s_buf_a[32768U + 2U * WFP_FILTFILT_PAD_BP];
float s_buf_eta[32768U + 2U * WFP_FILTFILT_PAD_BP];

/* Cross-spectral bin accumulators */
static WfpCrossBin_t s_eta_x_bins[WFP_WAVE_N_BINS];
static WfpCrossBin_t s_eta_y_bins[WFP_WAVE_N_BINS];

/* ---------------------------------------------------------------------------
 * Mode decision from window data — scans finite fractions
 * --------------------------------------------------------------------------- */
static WaveMode_t decide_mode_from_window(const float *ax_g,
                                          const float *ay_g,
                                          const float *az_g,
                                          const float *mx,
                                          const float *my,
                                          const float *mz,
                                          const float *heading_deg,
                                          const float *roll_deg,
                                          const float *pitch_deg,
                                          uint32_t     n)
{
    uint32_t motion_ok = 0, mag_ok = 0, hdg_ok = 0, roll_ok = 0, pitch_ok = 0;

    for (uint32_t i = 0; i < n; i++) {
        if (isfinite(ax_g[i]) && isfinite(ay_g[i]) && isfinite(az_g[i])) motion_ok++;
        if (isfinite(mx[i]) && isfinite(my[i]) && isfinite(mz[i]))       mag_ok++;
        if (isfinite(heading_deg[i])) hdg_ok++;
        if (isfinite(roll_deg[i]))    roll_ok++;
        if (isfinite(pitch_deg[i]))   pitch_ok++;
    }

    float motion_frac  = (float)motion_ok / (float)n;
    float mag_frac     = (float)mag_ok / (float)n;
    float heading_frac = (float)hdg_ok / (float)n;
    float roll_frac    = (float)roll_ok / (float)n;
    float pitch_frac   = (float)pitch_ok / (float)n;

    if (motion_frac < 0.95f) return WAVE_MODE_FALLBACK;
    if (mag_frac >= 0.95f && heading_frac >= 0.95f &&
        roll_frac >= 0.95f && pitch_frac >= 0.95f)
        return WAVE_MODE_GEOGRAPHIC;
    return WAVE_MODE_BODY_RELATIVE;
}

/* ---------------------------------------------------------------------------
 * Compute non-directional wave params from eta in s_buf_eta
 * --------------------------------------------------------------------------- */
static void compute_wave_params(uint32_t n, SharcResult_t *r)
{
    /* Detrend */
    double sum = 0.0;
    for (uint32_t i = 0; i < n; i++) sum += (double)s_buf_eta[i];
    float mean_eta = (float)(sum / (double)n);
    for (uint32_t i = 0; i < n; i++) s_buf_eta[i] -= mean_eta;

    const uint32_t win_len = WFP_WELCH_WIN_LEN;
    const uint32_t hop     = WFP_WELCH_NOVERLAP;
    const uint32_t nfft    = WFP_WELCH_NFFT;
    const float    df      = WFP_FS_HZ / (float)nfft;
    const uint32_t k_lo    = WFP_WAVE_K_MIN;
    const uint32_t k_hi    = WFP_WAVE_K_MAX;

    static float psd_acc[70];
    memset(psd_acc, 0, sizeof(psd_acc));

    /* Hann power correction */
    double hpwr = 0.0;
    for (uint32_t j = 0; j < win_len; j++) {
        double w = 0.5 * (1.0 - cos(2.0 * M_PI * (double)j / (double)win_len));
        hpwr += w * w;
    }
    float hann_pwr = (float)(hpwr / (double)win_len);

    uint32_t n_segs = 0;
    for (uint32_t start = 0; (start + win_len) <= n; start += hop) {
        for (uint32_t k = k_lo; k <= k_hi; k++) {
            float step = 2.0f * (float)M_PI * (float)k / (float)nfft;
            float c_s = cosf(step), s_s = sinf(step);
            float c = 1.0f, s = 0.0f;
            float w_step = 2.0f * (float)M_PI / (float)win_len;
            float wc_s = cosf(w_step), ws_s = sinf(w_step);
            float wc = 1.0f, ws = 0.0f;
            float re = 0.0f, im = 0.0f;

            for (uint32_t j = 0; j < win_len; j++) {
                float w = 0.5f * (1.0f - wc);
                float x = w * s_buf_eta[start + j];
                re += x * c; im -= x * s;
                float cn = c*c_s - s*s_s; float sn = s*c_s + c*s_s; c = cn; s = sn;
                float wcn = wc*wc_s - ws*ws_s; float wsn = ws*wc_s + wc*ws_s; wc = wcn; ws = wsn;
            }
            psd_acc[k - k_lo] += 2.0f * (re*re + im*im) / ((float)win_len * hann_pwr * WFP_FS_HZ);
        }
        n_segs++;
    }

    float m0 = 0.0f, m1 = 0.0f, m2 = 0.0f, S_peak = 0.0f;
    uint32_t k_peak = k_lo;
    for (uint32_t k = k_lo; k <= k_hi; k++) {
        float f_k = (float)k * df;
        float S_k = psd_acc[k - k_lo] / (float)n_segs;
        float tw = (k == k_lo || k == k_hi) ? 0.5f : 1.0f;
        m0 += tw * S_k * df; m1 += tw * f_k * S_k * df; m2 += tw * f_k * f_k * S_k * df;
        if (S_k > S_peak) { S_peak = S_k; k_peak = k; }
    }

    r->Hm0  = 4.0f * sqrtf(m0);
    r->fp   = (float)k_peak * df;
    r->Tp   = (r->fp > 0.0f) ? 1.0f / r->fp : 0.0f;
    r->Tm01 = (m1 > 0.0f) ? m0 / m1 : 0.0f;
    r->Tm02 = (m2 > 0.0f) ? sqrtf(m0 / m2) : 0.0f;
    r->n_segs = n_segs;
}

/* ---------------------------------------------------------------------------
 * sharc_process_window
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
                         SharcResult_t *result)
{
    if (!ax_g || !ay_g || !az_g || !result) return -1;
    if (n_samples < WFP_WELCH_WIN_LEN) return -2;

    memset(result, 0, sizeof(SharcResult_t));
    result->n_samples = n_samples;

    /* 1. Mode decision */
    result->mode = decide_mode_from_window(ax_g, ay_g, az_g,
                                           mx, my, mz,
                                           heading_deg, roll_deg, pitch_deg,
                                           n_samples);

    /* 2. Reconstruct eta (stored in s_buf_eta) */
    wfp_reconstruct_eta_from_az_g(az_g, s_buf_eta, n_samples);

    /* 3. Non-directional wave parameters */
    compute_wave_params(n_samples, result);

    /* 4. Directional processing (if motion available) */
    if (result->mode != WAVE_MODE_FALLBACK) {
        /* Reconstruct vx_body into s_buf_a */
        wfp_reconstruct_velocity_from_accel_g(ax_g, s_buf_a, n_samples);

        /* Compute eta-vx cross spectra */
        for (uint32_t i = 0; i < WFP_WAVE_N_BINS; i++) {
            wfp_cross_bin_eta_h(s_buf_eta, s_buf_a, n_samples,
                                WFP_WAVE_K_MIN + i, &s_eta_x_bins[i]);
        }

        /* Reconstruct vy_body into s_buf_a (reuse) */
        wfp_reconstruct_velocity_from_accel_g(ay_g, s_buf_a, n_samples);

        /* Compute eta-vy cross spectra */
        for (uint32_t i = 0; i < WFP_WAVE_N_BINS; i++) {
            wfp_cross_bin_eta_h(s_buf_eta, s_buf_a, n_samples,
                                WFP_WAVE_K_MIN + i, &s_eta_y_bins[i]);
        }

        /* Band-integrated direction */
        WfpDirectionBand_t dir;
        wfp_direction_band_from_cross(s_eta_x_bins, s_eta_y_bins, &dir);

        result->mean_from_deg       = dir.mean_from_deg;
        result->peak_from_deg       = dir.peak_from_deg;
        result->r1_band             = dir.r1_band;
        result->r1_peak             = dir.r1_peak;
        result->mean_vector_coh     = dir.mean_vector_coh;
        result->direction_confident = dir.direction_confident;
        result->valid_bins          = dir.valid_bins;
    }

    return 0;
}
