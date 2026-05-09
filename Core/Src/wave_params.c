/*
 * wave_params.c
 *
 * Streaming Welch PSD accumulator for vertical wave parameter extraction.
 * No external dependencies — uses only <math.h> and <string.h>.
 *
 * Algorithm:
 *   1. Samples are pushed one at a time into a 1024-sample segment buffer.
 *   2. When the buffer is full, a Hann-windowed direct DFT is computed over
 *      the wave-band bins only (k=1 to k=5 at 100 Hz / 1024 pts).
 *   3. The one-sided PSD is accumulated across segments (Welch averaging).
 *   4. wave_get_params() divides by segment count and computes spectral
 *      moments m0, m1, m2 → Hm0, Tp, Tm01, Tm02.
 *   5. 50% overlap: the second half of each segment seeds the next.
 */

#include "wave_params.h"
#include <math.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Internal state
 * --------------------------------------------------------------------------- */
static float    s_seg[WAVE_N];                  /* current segment buffer     */
static uint32_t s_seg_idx = 0;                  /* write position             */
static uint32_t s_n_segs  = 0;                  /* segments accumulated       */
static float    s_psd_acc[WAVE_N / 2U + 1U];   /* accumulated one-sided PSD  */
static float    s_hann[WAVE_N];                 /* Hann window coefficients   */
static float    s_hann_power;                   /* window power correction    */

/* ---------------------------------------------------------------------------
 * build_hann  — called once at init
 * --------------------------------------------------------------------------- */
static void build_hann(void)
{
    float sum = 0.0f;
    for (uint32_t k = 0; k < WAVE_N; k++) {
        s_hann[k] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI
                                         * (float)k / (float)(WAVE_N - 1U)));
        sum += s_hann[k] * s_hann[k];
    }
    /* Power correction: normalise PSD by mean squared window value */
    s_hann_power = sum / (float)WAVE_N;   /* ≈ 0.375 for Hann */
}

/* ---------------------------------------------------------------------------
 * process_segment
 *
 * Compute one-sided PSD over wave-band bins and accumulate.
 *
 * df = WAVE_FS / WAVE_N = 100 / 1024 ≈ 0.0977 Hz
 * k_lo = ceil(0.05 / 0.0977) = 1
 * k_hi = floor(0.50 / 0.0977) = 5
 *
 * Only 5 DFT bins computed per segment — ~5 × 1024 FPU ops ≈ 0.5 ms at 32 MHz.
 * --------------------------------------------------------------------------- */
static void process_segment(void)
{
    const float df = WAVE_FS / (float)WAVE_N;

    uint32_t k_lo = (uint32_t)ceilf (WAVE_F_LO / df);
    uint32_t k_hi = (uint32_t)floorf(WAVE_F_HI / df);
    if (k_hi >= WAVE_N / 2U) k_hi = WAVE_N / 2U - 1U;

    for (uint32_t k = k_lo; k <= k_hi; k++) {

        float re = 0.0f, im = 0.0f;
        const float step = 2.0f * (float)M_PI * (float)k / (float)WAVE_N;

        /* Apply Hann window inline — no local array needed, no stack pressure */
        for (uint32_t n = 0; n < WAVE_N; n++) {
            float w = s_hann[n] * s_seg[n];
            re += w * cosf(step * (float)n);
            im -= w * sinf(step * (float)n);
        }

        float S_k = 2.0f * (re * re + im * im)
                    / ((float)WAVE_N * s_hann_power * WAVE_FS);

        s_psd_acc[k] += S_k;
    }

    s_n_segs++;
}

/* ---------------------------------------------------------------------------
 * wave_init
 * --------------------------------------------------------------------------- */
void wave_init(void)
{
    memset(s_seg,     0, sizeof(s_seg));
    memset(s_psd_acc, 0, sizeof(s_psd_acc));
    s_seg_idx = 0;
    s_n_segs  = 0;
    build_hann();
}

/* ---------------------------------------------------------------------------
 * wave_push_sample
 *
 * Feed one vertical displacement sample [m] at WAVE_FS Hz.
 * Returns 1 each time a complete segment has been processed, 0 otherwise.
 * --------------------------------------------------------------------------- */
int wave_push_sample(float eta)
{
    s_seg[s_seg_idx++] = eta;

    if (s_seg_idx < WAVE_N) {
        return 0;
    }

    /* Segment full — process it */
    process_segment();

    /* 50% overlap: copy second half to start of next segment */
    memcpy(s_seg, s_seg + WAVE_OVERLAP, WAVE_OVERLAP * sizeof(float));
    s_seg_idx = WAVE_OVERLAP;

    return 1;
}

/* ---------------------------------------------------------------------------
 * wave_get_params
 *
 * Compute wave parameters from the accumulated averaged PSD.
 * Safe to call after any number of segments >= 1.
 * --------------------------------------------------------------------------- */
void wave_get_params(WaveParams_t *out)
{
    out->n_segs = s_n_segs;

    if (s_n_segs == 0U) {
        out->Hm0  = 0.0f;
        out->Tp   = 0.0f;
        out->Tm01 = 0.0f;
        out->Tm02 = 0.0f;
        return;
    }

    const float df    = WAVE_FS / (float)WAVE_N;
    const float scale = 1.0f / (float)s_n_segs;

    uint32_t k_lo = (uint32_t)ceilf (WAVE_F_LO / df);
    uint32_t k_hi = (uint32_t)floorf(WAVE_F_HI / df);
    if (k_hi >= WAVE_N / 2U) k_hi = WAVE_N / 2U - 1U;

    float m0 = 0.0f, m1 = 0.0f, m2 = 0.0f;
    float S_peak = 0.0f, f_peak = WAVE_F_LO;

    for (uint32_t k = k_lo; k <= k_hi; k++) {
        float f_k = (float)k * df;
        float S_k = s_psd_acc[k] * scale;   /* averaged PSD */

        m0 += S_k * df;
        m1 += f_k * S_k * df;
        m2 += f_k * f_k * S_k * df;

        if (S_k > S_peak) {
            S_peak = S_k;
            f_peak = f_k;
        }
    }

    out->Hm0  = 4.0f * sqrtf(m0);
    out->Tp   = (f_peak > 0.0f) ? (1.0f / f_peak) : 0.0f;
    out->Tm01 = (m1     > 0.0f) ? (m0 / m1)        : 0.0f;
    out->Tm02 = (m2     > 0.0f) ? sqrtf(m0 / m2)   : 0.0f;
}
