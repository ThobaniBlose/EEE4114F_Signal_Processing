/*
 * wave_full_pipeline.c
 *
 * Full-pipeline wave processor — matches accepted MATLAB S001 pipeline.
 * Filter coefficients from:
 *   [b_bp, a_bp] = butter(2, [0.02 0.50]/(100/2), 'bandpass');
 *   [b_hp, a_hp] = butter(2, 0.02/(100/2), 'high');
 */

#include "wave_full_pipeline.h"

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
