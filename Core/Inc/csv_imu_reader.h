#ifndef CSV_IMU_READER_H
#define CSV_IMU_READER_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * CSV IMU reader — adapter between the sensing subsystem SD card CSV output and the
 * wave_processor module.
 *
 * Expected CSV format (S001_IMU.CSV):
 *   timestamp_ms, ax, ay, az, gx, gy, gz, mx, my, mz, heading_deg, roll_deg, pitch_deg
 *
 * az is in g. Conversion: az_ms2 = (az_g - mean_az_g) * 9.81
 *
 * File I/O is isolated behind CsvImuGetlineFn. Swap for f_gets() when FATFS
 * is available — everything else stays unchanged.
 * --------------------------------------------------------------------------- */

/* Column indices (0-based) */
#define CSV_IMU_COL_TIMESTAMP   0U
#define CSV_IMU_COL_AX          1U
#define CSV_IMU_COL_AY          2U
#define CSV_IMU_COL_AZ          3U
#define CSV_IMU_COL_GX          4U
#define CSV_IMU_COL_GY          5U
#define CSV_IMU_COL_GZ          6U
#define CSV_IMU_COL_MX          7U
#define CSV_IMU_COL_MY          8U
#define CSV_IMU_COL_MZ          9U
#define CSV_IMU_COL_HEADING     10U
#define CSV_IMU_COL_ROLL        11U
#define CSV_IMU_COL_PITCH       12U

#define CSV_IMU_G_TO_MS2        9.81f
#define CSV_IMU_MAX_LINE        256U
#define CSV_IMU_MAX_SAMPLES     32768U

/* ---------------------------------------------------------------------------
 * Full sample struct — all 13 columns
 * --------------------------------------------------------------------------- */
typedef struct {
    uint32_t timestamp_ms;
    float    ax_g;
    float    ay_g;
    float    az_g;
    float    gx_dps;
    float    gy_dps;
    float    gz_dps;
    float    mx;
    float    my;
    float    mz;
    float    heading_deg;
    float    roll_deg;
    float    pitch_deg;
} CsvImuSample_t;

/* ---------------------------------------------------------------------------
 * Motion result struct — all channels needed for directional processing
 * --------------------------------------------------------------------------- */
typedef struct {
    uint32_t n_samples;
    float    mean_az_g;
} CsvImuResult_t;

typedef struct {
    uint32_t n_samples;
    float    mean_ax_g;
    float    mean_ay_g;
    float    mean_az_g;
} CsvMotionResult_t;

/* ---------------------------------------------------------------------------
 * Generic line-source function pointer
 *
 * Signature matches both the in-memory stub and FATFS f_gets():
 *   buf      — destination buffer
 *   buf_size — buffer size in bytes
 *   ctx      — caller context (e.g. FIL* or generator state)
 *
 * Returns buf on success, NULL when exhausted.
 * --------------------------------------------------------------------------- */
typedef const char *(*CsvImuGetlineFn)(char    *buf,
                                       uint32_t buf_size,
                                       void    *ctx);

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

/* Parse one full sensing subsystem CSV line into a sample struct.
 * Returns 0 on success, negative on error or skip (header/empty). */
int csv_imu_parse_sample_line(const char     *line,
                               CsvImuSample_t *sample);

/* Parse az from a generic line source, mean-remove, convert to m/s².
 * Returns 0 on success, -1 bad args, -2 no samples. */
int csv_imu_parse_az_from_source(CsvImuGetlineFn  getline_fn,
                                  void            *ctx,
                                  float           *az_ms2_out,
                                  uint32_t         max_samples,
                                  CsvImuResult_t  *result);

/* Parse all motion channels needed for directional processing.
 * Fills ax/ay/az (m/s², mean-removed) and heading/roll/pitch (deg).
 * All output arrays must be caller-allocated with size >= max_samples.
 * Returns 0 on success, -1 bad args, -2 no samples. */
int csv_imu_parse_motion_from_source(CsvImuGetlineFn   getline_fn,
                                      void             *ctx,
                                      float            *ax_ms2_out,
                                      float            *ay_ms2_out,
                                      float            *az_ms2_out,
                                      float            *heading_out,
                                      float            *roll_out,
                                      float            *pitch_out,
                                      uint32_t          max_samples,
                                      CsvMotionResult_t *result);

/* Convenience wrapper — uses built-in in-memory stub source. */
int csv_imu_parse_az(float          *az_ms2_out,
                     uint32_t        max_samples,
                     CsvImuResult_t *result);

#endif /* CSV_IMU_READER_H */
