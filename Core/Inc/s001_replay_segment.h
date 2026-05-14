#ifndef S001_REPLAY_SEGMENT_H
#define S001_REPLAY_SEGMENT_H

#include <stdint.h>

#define S001_REPLAY_N_SAMPLES  (32768U)
#define S001_REPLAY_START_IDX  (65537U)
#define S001_REPLAY_END_IDX    (98304U)

/* MATLAB golden values for this exact truncated segment. */
#define S001_REF_HM0_M       (1.156854000f)
#define S001_REF_FP_HZ       (0.051880000f)
#define S001_REF_TP_S        (19.275294000f)
#define S001_REF_TM01_S      (18.034925000f)
#define S001_REF_TM02_S      (17.710739000f)

/* Temporary no-SD replay arrays.
 * Units match S001_IMU.csv:
 *   timestamp_ms        [ms]
 *   ax, ay, az          [g]
 *   gx, gy, gz          [deg/s]
 *   mx, my, mz          [uT]
 *   heading, roll, pitch [deg]
 *
 * These arrays allow the STM to decide GEOGRAPHIC vs BODY_RELATIVE
 * automatically from the replay data, not from hardcoded assumptions.
 *
 * The full 13-column replay is also saved separately as:
 *   S001_STM_REPLAY_FULL.csv
 */

extern const uint32_t s001_timestamp_ms[S001_REPLAY_N_SAMPLES];

extern const float s001_ax_g[S001_REPLAY_N_SAMPLES];
extern const float s001_ay_g[S001_REPLAY_N_SAMPLES];
extern const float s001_az_g[S001_REPLAY_N_SAMPLES];

extern const float s001_gx_dps[S001_REPLAY_N_SAMPLES];
extern const float s001_gy_dps[S001_REPLAY_N_SAMPLES];
extern const float s001_gz_dps[S001_REPLAY_N_SAMPLES];

extern const float s001_mx_uT[S001_REPLAY_N_SAMPLES];
extern const float s001_my_uT[S001_REPLAY_N_SAMPLES];
extern const float s001_mz_uT[S001_REPLAY_N_SAMPLES];

extern const float s001_heading_deg[S001_REPLAY_N_SAMPLES];
extern const float s001_roll_deg[S001_REPLAY_N_SAMPLES];
extern const float s001_pitch_deg[S001_REPLAY_N_SAMPLES];

#endif /* S001_REPLAY_SEGMENT_H */
