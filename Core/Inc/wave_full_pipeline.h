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

/* Filter coefficients — exported for verification in Step 7 */
extern const float wfp_bp_b[WFP_BP_NCOEFF];
extern const float wfp_bp_a[WFP_BP_NCOEFF];
extern const float wfp_hp_b[WFP_HP_NCOEFF];
extern const float wfp_hp_a[WFP_HP_NCOEFF];

#endif /* WAVE_FULL_PIPELINE_H */
