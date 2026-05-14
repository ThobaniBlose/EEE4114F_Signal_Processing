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
 * MATLAB-like filtfilt with reflection padding for high-pass.
 * --------------------------------------------------------------------------- */
void wfp_filtfilt_hp_order2_inplace(float *x, uint32_t n)
{
    const uint32_t pad = WFP_FILTFILT_PAD_HP;
    const uint32_t m   = n + 2U * pad;

    /* Move original data to make room for left padding */
    for (uint32_t i = n; i > 0U; i--) {
        x[pad + i - 1U] = x[i - 1U];
    }

    /* Left reflection */
    float x_first = x[pad];
    for (uint32_t i = 0U; i < pad; i++) {
        x[i] = 2.0f * x_first - x[pad + pad - i];
    }

    /* Right reflection */
    float x_last = x[pad + n - 1U];
    for (uint32_t i = 0U; i < pad; i++) {
        x[pad + n + i] = 2.0f * x_last - x[pad + n - 2U - i];
    }

    /* Forward + backward SOS filter */
    sos_filter_inplace_zi(x, m, wfp_hp_sos, 1U, wfp_hp_gain);
    reverse_float_array(x, m);
    sos_filter_inplace_zi(x, m, wfp_hp_sos, 1U, wfp_hp_gain);
    reverse_float_array(x, m);

    /* Remove padding */
    for (uint32_t i = 0U; i < n; i++) {
        x[i] = x[pad + i];
    }
}

/* ---------------------------------------------------------------------------
 * wfp_cumtrapz_inplace — cumulative trapezoidal integration
 * Matches MATLAB cumtrapz(t, x) with uniform dt.
 * --------------------------------------------------------------------------- */
void wfp_cumtrapz_inplace(float *x, uint32_t n, float dt_s)
{
    if (n == 0U) return;

    float prev_input = x[0];
    float integral   = 0.0f;
    x[0] = 0.0f;

    for (uint32_t i = 1U; i < n; i++) {
        float current_input = x[i];
        integral += 0.5f * dt_s * (prev_input + current_input);
        x[i] = integral;
        prev_input = current_input;
    }
}

/* ---------------------------------------------------------------------------
 * wfp_reconstruct_velocity_from_accel_g
 * accel[g] → m/s² → mean removal → BP filtfilt → integrate → HP filtfilt
 * --------------------------------------------------------------------------- */
void wfp_reconstruct_velocity_from_accel_g(const float *accel_g,
                                           float       *out_velocity,
                                           uint32_t     n)
{
    const float dt_s = 1.0f / WFP_FS_HZ;
    wfp_make_dynamic_accel_ms2(accel_g, out_velocity, n);
    wfp_filtfilt_bp_order4_inplace(out_velocity, n);
    wfp_cumtrapz_inplace(out_velocity, n, dt_s);
    wfp_filtfilt_hp_order2_inplace(out_velocity, n);
}

/* ---------------------------------------------------------------------------
 * wfp_reconstruct_eta_from_az_g
 * az[g] → m/s² → mean removal → BP → integrate → HP → integrate → HP
 * --------------------------------------------------------------------------- */
void wfp_reconstruct_eta_from_az_g(const float *az_g,
                                   float       *out_eta,
                                   uint32_t     n)
{
    const float dt_s = 1.0f / WFP_FS_HZ;
    wfp_make_dynamic_accel_ms2(az_g, out_eta, n);
    wfp_filtfilt_bp_order4_inplace(out_eta, n);
    wfp_cumtrapz_inplace(out_eta, n, dt_s);
    wfp_filtfilt_hp_order2_inplace(out_eta, n);
    wfp_cumtrapz_inplace(out_eta, n, dt_s);
    wfp_filtfilt_hp_order2_inplace(out_eta, n);
}

/* ---------------------------------------------------------------------------
 * Helper: mean of float array as double
 * --------------------------------------------------------------------------- */
static double mean_float_as_double(const float *x, uint32_t n)
{
    double sum = 0.0;
    for (uint32_t i = 0U; i < n; i++) sum += (double)x[i];
    return sum / (double)n;
}

/* ---------------------------------------------------------------------------
 * wfp_cross_bin_eta_h
 * Welch cross-spectral estimate at a single frequency bin k_bin.
 * Uses recursive trig oscillators — no cos/sin per sample.
 * --------------------------------------------------------------------------- */
void wfp_cross_bin_eta_h(const float   *eta,
                         const float   *h,
                         uint32_t       n,
                         uint32_t       k_bin,
                         WfpCrossBin_t *out)
{
    const uint32_t win_len = WFP_WELCH_WIN_LEN;
    const uint32_t hop     = WFP_WELCH_NOVERLAP;
    const uint32_t nfft    = WFP_WELCH_NFFT;

    float eta_mean = (float)mean_float_as_double(eta, n);
    float h_mean   = (float)mean_float_as_double(h, n);

    double p_eta_sum = 0.0, p_h_sum = 0.0;
    double cross_re_sum = 0.0, cross_im_sum = 0.0;
    uint32_t n_segs = 0U;

    /* DFT oscillator step: exp(-j*2*pi*k/nfft) */
    float dft_step = -2.0f * (float)M_PI * (float)k_bin / (float)nfft;
    float dft_c_step = cosf(dft_step);
    float dft_s_step = sinf(dft_step);

    /* Hann window oscillator step: cos(2*pi/win_len) */
    float win_step = 2.0f * (float)M_PI / (float)win_len;
    float win_c_step = cosf(win_step);
    float win_s_step = sinf(win_step);

    for (uint32_t start = 0U; (start + win_len) <= n; start += hop) {
        float e_re = 0.0f, e_im = 0.0f;
        float h_re = 0.0f, h_im = 0.0f;

        float dft_c = 1.0f, dft_s = 0.0f;
        float win_c = 1.0f, win_s = 0.0f;

        for (uint32_t j = 0U; j < win_len; j++) {
            /* Periodic Hann: w = 0.5*(1 - cos(2*pi*j/win_len)) */
            float win = 0.5f * (1.0f - win_c);

            float eta_val = (eta[start + j] - eta_mean) * win;
            float h_val   = (h[start + j]   - h_mean)   * win;

            e_re += eta_val * dft_c;
            e_im += eta_val * dft_s;
            h_re += h_val * dft_c;
            h_im += h_val * dft_s;

            /* Update DFT oscillator */
            float dft_c_new = dft_c * dft_c_step - dft_s * dft_s_step;
            float dft_s_new = dft_s * dft_c_step + dft_c * dft_s_step;
            dft_c = dft_c_new;
            dft_s = dft_s_new;

            /* Update Hann window oscillator */
            float win_c_new = win_c * win_c_step - win_s * win_s_step;
            float win_s_new = win_s * win_c_step + win_c * win_s_step;
            win_c = win_c_new;
            win_s = win_s_new;
        }

        double e_re_d = (double)e_re, e_im_d = (double)e_im;
        double h_re_d = (double)h_re, h_im_d = (double)h_im;

        p_eta_sum    += e_re_d*e_re_d + e_im_d*e_im_d;
        p_h_sum      += h_re_d*h_re_d + h_im_d*h_im_d;
        cross_re_sum += e_re_d*h_re_d + e_im_d*h_im_d;
        cross_im_sum += e_im_d*h_re_d - e_re_d*h_im_d;
        n_segs++;
    }

    out->p_eta    = p_eta_sum;
    out->p_h      = p_h_sum;
    out->cross_re = cross_re_sum;
    out->cross_im = cross_im_sum;
    out->n_segs   = n_segs;
}

/* ---------------------------------------------------------------------------
 * wfp_direction_peak_from_cross
 * Compute peak-bin direction from eta-x and eta-y cross-spectral results.
 * --------------------------------------------------------------------------- */
void wfp_direction_peak_from_cross(const WfpCrossBin_t *eta_x,
                                   const WfpCrossBin_t *eta_y,
                                   WfpDirectionPeak_t  *out)
{
    double p_eta = eta_x->p_eta;
    double p_x   = eta_x->p_h;
    double p_y   = eta_y->p_h;

    double q_eta_x = eta_x->cross_im;
    double q_eta_y = eta_y->cross_im;

    double den = sqrt(p_eta * (p_x + p_y));
    double comp1 = 0.0, comp2 = 0.0;
    if (den > 0.0) {
        comp1 = -q_eta_x / den;
        comp2 = -q_eta_y / den;
    }

    double r1 = sqrt(comp1*comp1 + comp2*comp2);
    if (r1 > 0.999) r1 = 0.999;

    double theta_toward = atan2(comp2, comp1) * 180.0 / M_PI;
    while (theta_toward < 0.0)   theta_toward += 360.0;
    while (theta_toward >= 360.0) theta_toward -= 360.0;

    double theta_from = theta_toward + 180.0;
    if (theta_from >= 360.0) theta_from -= 360.0;

    double coh_den = p_eta * (p_x + p_y);
    double vector_coh = 0.0;
    if (coh_den > 0.0) {
        double cex2 = eta_x->cross_re * eta_x->cross_re +
                      eta_x->cross_im * eta_x->cross_im;
        double cey2 = eta_y->cross_re * eta_y->cross_re +
                      eta_y->cross_im * eta_y->cross_im;
        vector_coh = (cex2 + cey2) / coh_den;
    }
    if (vector_coh < 0.0) vector_coh = 0.0;
    if (vector_coh > 1.0) vector_coh = 1.0;

    out->peak_from_deg    = (float)theta_from;
    out->peak_toward_deg  = (float)theta_toward;
    out->r1_peak          = (float)r1;
    out->vector_coh_peak  = (float)vector_coh;
    out->n_segs           = eta_x->n_segs;
}

/* ---------------------------------------------------------------------------
 * Helper: wrap angle to [0, 360)
 * --------------------------------------------------------------------------- */
static double wrap360(double x)
{
    while (x < 0.0)   x += 360.0;
    while (x >= 360.0) x -= 360.0;
    return x;
}

/* ---------------------------------------------------------------------------
 * Helper: compute direction components from one cross-bin pair
 * --------------------------------------------------------------------------- */
static void dir_components_from_cross(const WfpCrossBin_t *eta_x,
                                      const WfpCrossBin_t *eta_y,
                                      double *comp1, double *comp2,
                                      double *r1_out,
                                      double *theta_from, double *theta_toward,
                                      double *vector_coh)
{
    double p_eta = eta_x->p_eta;
    double p_x   = eta_x->p_h;
    double p_y   = eta_y->p_h;
    double q_eta_x = eta_x->cross_im;
    double q_eta_y = eta_y->cross_im;

    double den = sqrt(p_eta * (p_x + p_y));
    *comp1 = 0.0; *comp2 = 0.0;
    if (den > 0.0) { *comp1 = -q_eta_x / den; *comp2 = -q_eta_y / den; }

    double r1 = sqrt((*comp1)*(*comp1) + (*comp2)*(*comp2));
    if (r1 > 0.999) r1 = 0.999;
    *r1_out = r1;

    *theta_toward = wrap360(atan2(*comp2, *comp1) * 180.0 / M_PI);
    *theta_from   = wrap360(*theta_toward + 180.0);

    double coh_den = p_eta * (p_x + p_y);
    *vector_coh = 0.0;
    if (coh_den > 0.0) {
        double cex2 = eta_x->cross_re*eta_x->cross_re + eta_x->cross_im*eta_x->cross_im;
        double cey2 = eta_y->cross_re*eta_y->cross_re + eta_y->cross_im*eta_y->cross_im;
        *vector_coh = (cex2 + cey2) / coh_den;
    }
    if (*vector_coh < 0.0) *vector_coh = 0.0;
    if (*vector_coh > 1.0) *vector_coh = 1.0;
}

/* ---------------------------------------------------------------------------
 * wfp_direction_band_from_cross
 * Band-integrated direction from arrays of per-bin cross-spectral results.
 * --------------------------------------------------------------------------- */
void wfp_direction_band_from_cross(const WfpCrossBin_t *eta_x_bins,
                                   const WfpCrossBin_t *eta_y_bins,
                                   WfpDirectionBand_t  *out)
{
    /* Find peak bin */
    double p_eta_max = 0.0;
    uint32_t peak_i = 0U;
    for (uint32_t i = 0U; i < WFP_WAVE_N_BINS; i++) {
        if (eta_x_bins[i].p_eta > p_eta_max) {
            p_eta_max = eta_x_bins[i].p_eta;
            peak_i = i;
        }
    }
    double energy_thresh = 0.01 * p_eta_max;

    /* Peak-bin direction */
    double c1_pk, c2_pk, r1_pk, from_pk, toward_pk, coh_pk;
    dir_components_from_cross(&eta_x_bins[peak_i], &eta_y_bins[peak_i],
                              &c1_pk, &c2_pk, &r1_pk, &from_pk, &toward_pk, &coh_pk);

    /* Band integration using trapezoidal rule */
    double trap_num_c1 = 0.0, trap_num_c2 = 0.0, trap_den = 0.0;
    double coh_sum = 0.0;
    uint32_t valid_bins = 0U;
    uint8_t have_prev = 0U;
    double prev_f = 0.0, prev_w = 0.0, prev_wc1 = 0.0, prev_wc2 = 0.0;

    for (uint32_t i = 0U; i < WFP_WAVE_N_BINS; i++) {
        double comp1, comp2, r1, theta_from, theta_toward, vcoh;
        dir_components_from_cross(&eta_x_bins[i], &eta_y_bins[i],
                                  &comp1, &comp2, &r1, &theta_from, &theta_toward, &vcoh);

        double p_eta = eta_x_bins[i].p_eta;
        uint8_t valid = (p_eta >= energy_thresh) && isfinite(comp1) &&
                        isfinite(comp2) && isfinite(vcoh);

        if (valid) {
            uint32_t k = WFP_WAVE_K_MIN + i;
            double f = (double)k * (double)WFP_DF_HZ;
            double coh_w = vcoh < 0.01 ? 0.01 : vcoh;
            double w   = p_eta * coh_w;
            double wc1 = w * comp1;
            double wc2 = w * comp2;

            if (have_prev) {
                double df = f - prev_f;
                trap_num_c1 += 0.5 * df * (prev_wc1 + wc1);
                trap_num_c2 += 0.5 * df * (prev_wc2 + wc2);
                trap_den    += 0.5 * df * (prev_w   + w);
            }
            prev_f = f; prev_w = w; prev_wc1 = wc1; prev_wc2 = wc2;
            have_prev = 1U;
            coh_sum += vcoh;
            valid_bins++;
        }
    }

    double comp1_band = 0.0, comp2_band = 0.0;
    if (trap_den > 0.0) {
        comp1_band = trap_num_c1 / trap_den;
        comp2_band = trap_num_c2 / trap_den;
    }

    double theta_toward_band = wrap360(atan2(comp2_band, comp1_band) * 180.0 / M_PI);
    double theta_from_band   = wrap360(theta_toward_band + 180.0);
    double r1_band = sqrt(comp1_band*comp1_band + comp2_band*comp2_band);
    double mean_coh = (valid_bins > 0U) ? coh_sum / (double)valid_bins : 0.0;

    uint32_t confident = (valid_bins >= WFP_DIR_MIN_VALID_BINS) &&
                         (r1_band >= (double)WFP_DIR_R1_THRESH) &&
                         (mean_coh >= (double)WFP_DIR_COH_THRESH);

    out->mean_from_deg       = (float)theta_from_band;
    out->mean_toward_deg     = (float)theta_toward_band;
    out->peak_from_deg       = (float)from_pk;
    out->peak_toward_deg     = (float)toward_pk;
    out->r1_band             = (float)r1_band;
    out->r1_peak             = (float)r1_pk;
    out->mean_vector_coh     = (float)mean_coh;
    out->direction_confident = confident;
    out->valid_bins          = valid_bins;
    out->peak_k              = WFP_WAVE_K_MIN + peak_i;
    out->peak_freq_hz        = (float)out->peak_k * WFP_DF_HZ;
}
