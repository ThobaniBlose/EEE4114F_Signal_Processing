#ifndef CSV_IMU_READER_H
#define CSV_IMU_READER_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * CSV IMU reader — adapter between Batsi's SD card CSV output and the
 * wave_processor module.
 *
 * Expected CSV format (S001_IMU.CSV):
 *   timestamp_ms, ax, ay, az, gx, gy, gz, mx, my, mz, heading_deg, roll_deg, pitch_deg
 *
 * Column indices (0-based):
 *   0  = timestamp_ms
 *   1  = ax [g]
 *   2  = ay [g]
 *   3  = az [g]   <-- used for vertical wave processing
 *   4  = gx [deg/s]
 *   5  = gy [deg/s]
 *   6  = gz [deg/s]
 *   7  = mx
 *   8  = my
 *   9  = mz
 *   10 = heading_deg
 *   11 = roll_deg
 *   12 = pitch_deg
 *
 * Units: az is in g. Conversion to m/s² and mean removal are applied
 * inside csv_imu_parse_az() before passing to wave_processor_run().
 *
 * File I/O stub:
 *   Currently uses an in-memory array of CSV line strings for testing.
 *   When FATFS is enabled, replace csv_imu_getline() with f_gets().
 * --------------------------------------------------------------------------- */

#define CSV_IMU_COL_AZ       3U     /* az column index (0-based)              */
#define CSV_IMU_G_TO_MS2     9.81f  /* g to m/s² conversion                  */
#define CSV_IMU_MAX_LINE     128U   /* max characters per CSV line            */
#define CSV_IMU_MAX_SAMPLES  32768U /* max samples to parse in one session    */

/* ---------------------------------------------------------------------------
 * Result struct
 * --------------------------------------------------------------------------- */
typedef struct {
    uint32_t n_samples;     /* number of az samples successfully parsed       */
    float    mean_az_g;     /* mean az in g (removed before processing)       */
} CsvImuResult_t;

/* ---------------------------------------------------------------------------
 * Public API
 *
 * csv_imu_parse_az()
 *   Parse az from CSV lines (stub or real source), mean-remove, convert to
 *   m/s², and store in az_ms2_out[].
 *
 *   Parameters:
 *     az_ms2_out  — output buffer, caller-allocated, size >= CSV_IMU_MAX_SAMPLES
 *     max_samples — maximum samples to read
 *     result      — filled with n_samples and mean_az_g
 *
 *   Returns:
 *     0  on success
 *    -1  if az_ms2_out or result is NULL
 *    -2  if no valid samples were parsed
 * --------------------------------------------------------------------------- */
int csv_imu_parse_az(float          *az_ms2_out,
                     uint32_t        max_samples,
                     CsvImuResult_t *result);

#endif /* CSV_IMU_READER_H */
