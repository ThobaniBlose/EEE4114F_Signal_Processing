#ifndef S001_REPLAY_SEGMENT_H
#define S001_REPLAY_SEGMENT_H

#include <stdint.h>

#define S001_REPLAY_N_SAMPLES  (32768U)
#define S001_REPLAY_START_IDX  (65537U)
#define S001_REPLAY_END_IDX    (98304U)
#define S001_REPLAY_START_MS   (655360U)
#define S001_REPLAY_END_MS     (983030U)

/* STM wave-processor configuration used to create this reference. */
#define S001_REF_FS_HZ       (100.000000f)
#define S001_REF_WIN_LEN     (8192U)
#define S001_REF_HOP_LEN     (4096U)
#define S001_REF_NFFT        (32768U)
#define S001_REF_F1_HZ       (0.020000f)
#define S001_REF_F2_HZ       (0.040000f)
#define S001_REF_F_HI_HZ     (0.400000f)

/* MATLAB golden values for this exact truncated segment. */
#define S001_REF_HM0_M       (1.156854191f)
#define S001_REF_FP_HZ       (0.051879883f)
#define S001_REF_TP_S        (19.275294118f)
#define S001_REF_TM01_S      (18.034924678f)
#define S001_REF_TM02_S      (17.710738693f)

extern const float s001_az_ms2[S001_REPLAY_N_SAMPLES];

#endif /* S001_REPLAY_SEGMENT_H */
