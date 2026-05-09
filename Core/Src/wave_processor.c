/*
 * wave_processor.c
 *
 * Vertical wave parameter processor — non-directional branch.
 *
 * Algorithm:
 *   1. Welch's method: overlapping segments, Hann window, 50% hop.
 *   2. Zero-padded DFT (WP_WIN_LEN real samples → WP_NFFT frequency grid).
 *      Recursive trig recurrence: cosf/sinf called once per bin, not per sample.
 *   3. Acceleration-to-displacement PSD conversion:
 *        S_eta(f) = S_aa(f) * [R(f) / (2*pi*f)^2]^2
 *      where R(f) is a cosine taper from WP_F1_HZ to WP_F2_HZ.
 *   4. Spectral moments m0, m1, m2 via trapezoidal integration.
 *   5. Wave parameters: Hm0, Tp, Tm01, Tm02.
 *
 * Validated against MATLAB STM replay reference (WS23 dataset):
 *   Hm0=0.9834 m, Tp=19.28 s, Tm01=16.71 s, Tm02=16.45 s
 */

#include "wave_processor.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------------------------------------------------------------------------
 * Internal state — all static to avoid stack pressure
 * --------------------------------------------------------------------------- */
static float    s_hann[WP_WIN_LEN];
static float    s_windowed[WP_WIN_LEN];
static float    s_psd_accum[WP_NFFT / 2U + 1U];
static float    s_hann_pwr = 0.0f;
static uint32_t s_k_lo     = 0U;
static uint32_t s_k_hi     = 0U;

/* ---------------------------------------------------------------------------
 * spectral_taper
 *
 * Cosine taper R(f): 0 below F1, smooth rise F1→F2, 1 above F2.
 * Suppresses low-frequency integration noise.
 * --------------------------------------------------------------------------- */
static float spectral_taper(float f)
{
    if (f < WP_F1_HZ)  return 0.0f;
    if (f >= WP_F2_HZ) return 1.0f;
    float x = (f - WP_F1_HZ) / (WP_F2_HZ - WP_F1_HZ);
    return 0.5f * (1.0f - cosf((float)M_PI * x));
}

/* ---------------------------------------------------------------------------
 * wave_processor_init
 *
 * Build the periodic Hann window and compute wave-band bin limits.
 * Must be called once before wave_processor_run().
 * --------------------------------------------------------------------------- */
void wave_processor_init(void)
{
    float wsum2 = 0.0f;
    for (uint32_t n = 0U; n < WP_WIN_LEN; n++) {
        /* Periodic Hann — matches MATLAB hann(N,'periodic') */
        s_hann[n] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI
                                         * (float)n / (float)WP_WIN_LEN));
        wsum2 += s_hann[n] * s_hann[n];
    }
    s_hann_pwr = wsum2 / (float)WP_WIN_LEN;

    float df = WP_FS_HZ / (float)WP_NFFT;
    s_k_lo = (uint32_t)ceilf (WP_F1_HZ   / df);
    s_k_hi = (uint32_t)floorf(WP_F_HI_HZ / df);
}

/* ---------------------------------------------------------------------------
 * wave_processor_run
 *
 * Process az_ms2[n_samples] (mean-removed vertical acceleration, m/s²).
 * Fills *result with Hm0, Tp, Tm01, Tm02, f_peak, m0, n_segs.
 * --------------------------------------------------------------------------- */
int wave_processor_run(const float          *az_ms2,
                       uint32_t              n_samples,
                       WaveProcessor_Result *result)
{
    if (az_ms2 == 0 || result == 0) return -1;
    if (n_samples < WP_WIN_LEN)     return -2;
    memset(s_psd_accum, 0, sizeof(s_psd_accum));
    uint32_t seg_count = 0U;

    uint32_t last_start = (n_samples >= WP_WIN_LEN)
                          ? (n_samples - WP_WIN_LEN) : 0U;

    for (uint32_t start = 0U; start <= last_start; start += WP_HOP_LEN) {

        /* Apply Hann window to this segment */
        for (uint32_t n = 0U; n < WP_WIN_LEN; n++) {
            s_windowed[n] = az_ms2[start + n] * s_hann[n];
        }

        /* Zero-padded DFT over wave-band bins.
         * Recursive trig recurrence: only 2 trig calls per bin (not per sample).
         * Reference: Goertzel-style recurrence for arbitrary frequency grids. */
        for (uint32_t k = s_k_lo; k <= s_k_hi; k++) {

            float step   = 2.0f * (float)M_PI * (float)k / (float)WP_NFFT;
            float c_step = cosf(step);
            float s_step = sinf(step);
            float c = 1.0f, s = 0.0f;
            float re = 0.0f, im = 0.0f;

            for (uint32_t n = 0U; n < WP_WIN_LEN; n++) {
                float x  = s_windowed[n];
                re += x * c;
                im -= x * s;
                float c_new = c * c_step - s * s_step;
                float s_new = s * c_step + c * s_step;
                c = c_new;
                s = s_new;
            }

            float f_k = (float)k * WP_FS_HZ / (float)WP_NFFT;

            /* One-sided acceleration PSD [m²/s⁴/Hz] */
            float Saa = 2.0f * (re*re + im*im)
                        / ((float)WP_WIN_LEN * s_hann_pwr * WP_FS_HZ);

            /* Frequency-domain double integration with low-freq taper:
             *   S_eta(f) = Saa(f) * [R(f) / (2*pi*f)^2]^2              */
            float R = spectral_taper(f_k);
            float H = 0.0f;
            if (f_k > 0.0f) {
                float twopif = 2.0f * (float)M_PI * f_k;
                H = R / (twopif * twopif);
            }
            s_psd_accum[k] += Saa * H * H;
        }
        seg_count++;
    }

    /* Spectral moments — trapezoidal integration (matches MATLAB trapz) */
    float df   = WP_FS_HZ / (float)WP_NFFT;
    float m0   = 0.0f, m1 = 0.0f, m2 = 0.0f;
    float S_pk = 0.0f;
    uint32_t k_peak = s_k_lo;

    for (uint32_t k = s_k_lo; k <= s_k_hi; k++) {
        float f_k = (float)k * df;
        float S_k = s_psd_accum[k] / (float)seg_count;
        float tw  = (k == s_k_lo || k == s_k_hi) ? 0.5f : 1.0f;
        m0 += tw * S_k * df;
        m1 += tw * f_k * S_k * df;
        m2 += tw * f_k * f_k * S_k * df;
        if (S_k > S_pk) { S_pk = S_k; k_peak = k; }
    }

    result->m0     = m0;
    result->m1     = m1;
    result->m2     = m2;
    result->f_peak = (float)k_peak * df;
    result->n_segs = seg_count;
    result->Hm0    = 4.0f * sqrtf(m0);
    result->Tp     = (result->f_peak > 0.0f) ? 1.0f / result->f_peak : 0.0f;
    result->Tm01   = (m1 > 0.0f) ? m0 / m1          : 0.0f;
    result->Tm02   = (m2 > 0.0f) ? sqrtf(m0 / m2)   : 0.0f;

    return 0;
}
