/*
 * wave_full_pipeline.c
 *
 * Full-pipeline wave processor — matches accepted MATLAB S001 pipeline.
 * Filter coefficients from:
 *   [b_bp, a_bp] = butter(2, [0.02 0.50]/(100/2), 'bandpass');
 *   [b_hp, a_hp] = butter(2, 0.02/(100/2), 'high');
 */

#include "wave_full_pipeline.h"
#include <math.h>
#include <string.h>

/* Bandpass: 0.02–0.50 Hz, 2nd-order Butterworth (4th-order IIR) */
const float wfp_bp_b[WFP_BP_NCOEFF] = {
     2.226313688492e-04f,
     0.000000000000e+00f,
    -4.452627376984e-04f,
     0.000000000000e+00f,
     2.226313688492e-04f
};

const float wfp_bp_a[WFP_BP_NCOEFF] = {
     1.000000000000e+00f,
    -3.957276466172e+00f,
     5.872799694766e+00f,
    -3.873768339428e+00f,
     9.582451123601e-01f
};

/* High-pass: 0.02 Hz, 2nd-order Butterworth */
const float wfp_hp_b[WFP_HP_NCOEFF] = {
     9.991118180796e-01f,
    -1.998223636159e+00f,
     9.991118180796e-01f
};

const float wfp_hp_a[WFP_HP_NCOEFF] = {
     1.000000000000e+00f,
    -1.998222847292e+00f,
     9.982244250264e-01f
};

/* ---------------------------------------------------------------------------
 * wfp_rms
 * --------------------------------------------------------------------------- */
float wfp_rms(const float *x, uint32_t n)
{
    double sumsq = 0.0;
    for (uint32_t i = 0U; i < n; i++) {
        sumsq += (double)x[i] * (double)x[i];
    }
    return (float)sqrt(sumsq / (double)n);
}

/* ---------------------------------------------------------------------------
 * wfp_make_dynamic_accel_ms2
 * --------------------------------------------------------------------------- */
void wfp_make_dynamic_accel_ms2(const float *x_g,
                                float       *x_dyn_ms2,
                                uint32_t     n)
{
    double sum = 0.0;
    for (uint32_t i = 0U; i < n; i++) {
        sum += (double)x_g[i] * (double)WFP_G0;
    }
    double mean = sum / (double)n;
    for (uint32_t i = 0U; i < n; i++) {
        x_dyn_ms2[i] = (float)(((double)x_g[i] * (double)WFP_G0) - mean);
    }
}

/* ---------------------------------------------------------------------------
 * SOS coefficients from MATLAB tf2sos() — numerically stable for embedded
 * --------------------------------------------------------------------------- */

/* Bandpass: butter(2, [0.02 0.50]/(100/2), 'bandpass') */
static const double wfp_bp_gain = 2.226313688491914e-04;
static const double wfp_bp_sos[2][6] = {
    { 1.000000000000000e+00, -2.000000000000002e+00,  1.000000000000002e+00,
      1.000000000000000e+00, -1.959041329201376e+00,  9.599376365994913e-01 },
    { 1.000000000000000e+00,  2.000000000000002e+00,  1.000000000000002e+00,
      1.000000000000000e+00, -1.998235136970346e+00,  9.982368393791310e-01 }
};

/* High-pass: butter(2, 0.02/(100/2), 'high') */
static const double wfp_hp_gain = 9.991118180795607e-01;
static const double wfp_hp_sos[1][6] = {
    { 1.000000000000000e+00, -2.000000000000000e+00,  1.000000000000000e+00,
      1.000000000000000e+00, -1.998222847291842e+00,  9.982244250264011e-01 }
};

/* ---------------------------------------------------------------------------
 * sos_filter_inplace_zi — SOS cascade with approximate initial conditions
 * Mimics MATLAB filtfilt's steady-state IC handling.
 * --------------------------------------------------------------------------- */
static void sos_filter_inplace_zi(float *x, uint32_t n,
                                  const double sos[][6],
                                  uint32_t n_sections,
                                  double gain)
{
    for (uint32_t i = 0U; i < n; i++) {
        x[i] = (float)((double)x[i] * gain);
    }

    for (uint32_t section = 0U; section < n_sections; section++) {
        double b0 = sos[section][0];
        double b1 = sos[section][1];
        double b2 = sos[section][2];
        double a1 = sos[section][4];
        double a2 = sos[section][5];

        double x0_val = (double)x[0];

        /* Approximate initial condition for steady-state response */
        double hdc_den = 1.0 + a1 + a2;
        double hdc = 0.0;
        if (fabs(hdc_den) > 1.0e-18) {
            hdc = (b0 + b1 + b2) / hdc_den;
        }
        double z1 = (hdc - b0) * x0_val;
        double z2 = (b2 - a2 * hdc) * x0_val;

        for (uint32_t i = 0U; i < n; i++) {
            double input  = (double)x[i];
            double output = b0 * input + z1;
            z1 = b1 * input - a1 * output + z2;
            z2 = b2 * input - a2 * output;
            x[i] = (float)output;
        }
    }
}

/* ---------------------------------------------------------------------------
 * reverse_float_array — in-place reversal
 * --------------------------------------------------------------------------- */
static void reverse_float_array(float *x, uint32_t n)
{
    for (uint32_t i = 0U; i < n / 2U; i++) {
        float tmp = x[i];
        x[i] = x[n - 1U - i];
        x[n - 1U - i] = tmp;
    }
}

/* ---------------------------------------------------------------------------
 * wfp_filtfilt_bp_order4_inplace
 *
 * MATLAB-like filtfilt with reflection padding and initial conditions.
 * Buffer must have space for n + 2*WFP_FILTFILT_PAD_BP floats.
 * --------------------------------------------------------------------------- */
void wfp_filtfilt_bp_order4_inplace(float *x, uint32_t n)
{
    const uint32_t pad = WFP_FILTFILT_PAD_BP;
    const uint32_t m   = n + 2U * pad;

    /* Move original data to make room for left padding */
    for (uint32_t i = n; i > 0U; i--) {
        x[pad + i - 1U] = x[i - 1U];
    }

    /* Left reflection: 2*x(1) - x(nfact+1:-1:2) */
    float x_first = x[pad];
    for (uint32_t i = 0U; i < pad; i++) {
        x[i] = 2.0f * x_first - x[pad + pad - i];
    }

    /* Right reflection: 2*x(end) - x(end-1:-1:end-nfact) */
    float x_last = x[pad + n - 1U];
    for (uint32_t i = 0U; i < pad; i++) {
        x[pad + n + i] = 2.0f * x_last - x[pad + n - 2U - i];
    }

    /* Forward + backward SOS filter on padded signal */
    sos_filter_inplace_zi(x, m, wfp_bp_sos, 2U, wfp_bp_gain);
    reverse_float_array(x, m);
    sos_filter_inplace_zi(x, m, wfp_bp_sos, 2U, wfp_bp_gain);
    reverse_float_array(x, m);

    /* Remove padding — shift result back */
    for (uint32_t i = 0U; i < n; i++) {
        x[i] = x[pad + i];
    }
}

/* ---------------------------------------------------------------------------
 * wfp_filtfilt_hp_order2_inplace
 * Simple forward-backward high-pass (no padding yet).
 * --------------------------------------------------------------------------- */
void wfp_filtfilt_hp_order2_inplace(float *x, uint32_t n)
{
    sos_filter_inplace_zi(x, n, wfp_hp_sos, 1U, wfp_hp_gain);
    reverse_float_array(x, n);
    sos_filter_inplace_zi(x, n, wfp_hp_sos, 1U, wfp_hp_gain);
    reverse_float_array(x, n);
}
