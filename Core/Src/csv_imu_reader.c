/*
 * csv_imu_reader.c
 *
 * Parses sensing subsystem S001_IMU.CSV format and prepares az data for wave_processor.
 *
 * File I/O is isolated in csv_imu_getline() (stub) and behind CsvImuGetlineFn.
 * To connect real SD card / FATFS, pass an f_gets()-backed function to
 * csv_imu_parse_az_from_source(). Nothing else changes.
 */

#include "csv_imu_reader.h"
#include <string.h>
#include <stdlib.h>

/* ---------------------------------------------------------------------------
 * Stub in-memory CSV source (16 rows for parser validation)
 * --------------------------------------------------------------------------- */
static const char * const s_test_csv_lines[] = {
    "timestamp_ms,ax,ay,az,gx,gy,gz,mx,my,mz,heading_deg,roll_deg,pitch_deg",
    "0,0.012,-0.005,1.0023,0.11,-0.32,0.02,23.1,-12.0,45.2,127.1,1.1,-0.7",
    "10,0.013,-0.004,1.0156,0.12,-0.31,0.02,23.2,-12.1,45.3,127.2,1.2,-0.8",
    "20,0.011,-0.006,1.0287,0.10,-0.33,0.01,23.0,-12.2,45.1,127.0,1.0,-0.6",
    "30,0.014,-0.003,1.0412,0.13,-0.30,0.03,23.3,-11.9,45.4,127.3,1.3,-0.9",
    "40,0.012,-0.005,1.0523,0.11,-0.32,0.02,23.1,-12.0,45.2,127.1,1.1,-0.7",
    "50,0.010,-0.007,1.0612,0.09,-0.34,0.01,22.9,-12.3,45.0,126.9,0.9,-0.5",
    "60,0.013,-0.004,1.0678,0.12,-0.31,0.02,23.2,-12.1,45.3,127.2,1.2,-0.8",
    "70,0.015,-0.002,1.0712,0.14,-0.29,0.03,23.4,-11.8,45.5,127.4,1.4,-1.0",
    "80,0.012,-0.005,1.0698,0.11,-0.32,0.02,23.1,-12.0,45.2,127.1,1.1,-0.7",
    "90,0.011,-0.006,1.0634,0.10,-0.33,0.01,23.0,-12.2,45.1,127.0,1.0,-0.6",
    "100,0.009,-0.008,1.0523,0.08,-0.35,0.00,22.8,-12.4,44.9,126.8,0.8,-0.4",
    "110,0.013,-0.004,1.0378,0.12,-0.31,0.02,23.2,-12.1,45.3,127.2,1.2,-0.8",
    "120,0.014,-0.003,1.0212,0.13,-0.30,0.03,23.3,-11.9,45.4,127.3,1.3,-0.9",
    "130,0.012,-0.005,1.0045,0.11,-0.32,0.02,23.1,-12.0,45.2,127.1,1.1,-0.7",
    "140,0.011,-0.006,0.9889,0.10,-0.33,0.01,23.0,-12.2,45.1,127.0,1.0,-0.6",
    "150,0.010,-0.007,0.9745,0.09,-0.34,0.01,22.9,-12.3,45.0,126.9,0.9,-0.5",
};

static const uint32_t s_test_n_lines =
    sizeof(s_test_csv_lines) / sizeof(s_test_csv_lines[0]);

static uint32_t s_line_index = 0U;

/* STUB — replace body with: return f_gets(buf, buf_size, ctx) ? buf : NULL; */
static const char *csv_imu_getline(char *buf, uint32_t buf_size, void *ctx)
{
    (void)ctx;
    if (s_line_index >= s_test_n_lines) return NULL;
    const char *src = s_test_csv_lines[s_line_index++];
    uint32_t len = (uint32_t)strlen(src);
    if (len >= buf_size) len = buf_size - 1U;
    memcpy(buf, src, len);
    buf[len] = '\0';
    return buf;
}

/* ---------------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------------- */
static int csv_parse_column_float(const char *line, uint32_t col_idx, float *out)
{
    const char *p = line;
    uint32_t col = 0U;
    while (col < col_idx) {
        while (*p != '\0' && *p != ',') p++;
        if (*p == '\0') return 0;
        p++;
        col++;
    }
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0' || *p == '\n' || *p == '\r') return 0;
    char *end;
    *out = strtof(p, &end);
    return (end != p) ? 1 : 0;
}

static int csv_parse_column_uint32(const char *line, uint32_t col_idx, uint32_t *out)
{
    const char *p = line;
    uint32_t col = 0U;
    while (col < col_idx) {
        while (*p != '\0' && *p != ',') p++;
        if (*p == '\0') return 0;
        p++;
        col++;
    }
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0' || *p == '\n' || *p == '\r') return 0;
    char *end;
    unsigned long val = strtoul(p, &end, 10);
    if (end == p) return 0;
    *out = (uint32_t)val;
    return 1;
}

/* ---------------------------------------------------------------------------
 * csv_imu_parse_sample_line
 * --------------------------------------------------------------------------- */
int csv_imu_parse_sample_line(const char *line, CsvImuSample_t *sample)
{
    if (line == 0 || sample == 0)                              return -1;
    if (line[0] == 't' || line[0] == 'T' || line[0] == '#')   return -2;
    if (line[0] == '\0' || line[0] == '\n' || line[0] == '\r') return -2;

    if (!csv_parse_column_uint32(line, CSV_IMU_COL_TIMESTAMP, &sample->timestamp_ms)) return -3;
    if (!csv_parse_column_float (line, CSV_IMU_COL_AX,        &sample->ax_g))         return -4;
    if (!csv_parse_column_float (line, CSV_IMU_COL_AY,        &sample->ay_g))         return -5;
    if (!csv_parse_column_float (line, CSV_IMU_COL_AZ,        &sample->az_g))         return -6;
    if (!csv_parse_column_float (line, CSV_IMU_COL_GX,        &sample->gx_dps))       return -7;
    if (!csv_parse_column_float (line, CSV_IMU_COL_GY,        &sample->gy_dps))       return -8;
    if (!csv_parse_column_float (line, CSV_IMU_COL_GZ,        &sample->gz_dps))       return -9;
    if (!csv_parse_column_float (line, CSV_IMU_COL_MX,        &sample->mx))           return -10;
    if (!csv_parse_column_float (line, CSV_IMU_COL_MY,        &sample->my))           return -11;
    if (!csv_parse_column_float (line, CSV_IMU_COL_MZ,        &sample->mz))           return -12;
    if (!csv_parse_column_float (line, CSV_IMU_COL_HEADING,   &sample->heading_deg))  return -13;
    if (!csv_parse_column_float (line, CSV_IMU_COL_ROLL,      &sample->roll_deg))     return -14;
    if (!csv_parse_column_float (line, CSV_IMU_COL_PITCH,     &sample->pitch_deg))    return -15;

    return 0;
}

/* ---------------------------------------------------------------------------
 * csv_imu_parse_motion_from_source
 *
 * Parses all motion channels needed for directional wave processing:
 *   ax, ay, az  — mean-removed, converted from g to m/s²
 *   heading, roll, pitch — in degrees, passed through as-is
 * --------------------------------------------------------------------------- */
int csv_imu_parse_motion_from_source(CsvImuGetlineFn   getline_fn,
                                      void             *ctx,
                                      float            *ax_ms2_out,
                                      float            *ay_ms2_out,
                                      float            *az_ms2_out,
                                      float            *heading_out,
                                      float            *roll_out,
                                      float            *pitch_out,
                                      uint32_t          max_samples,
                                      CsvMotionResult_t *result)
{
    if (getline_fn == 0 || result == 0) return -1;
    if (ax_ms2_out == 0 || ay_ms2_out == 0 || az_ms2_out == 0) return -1;
    if (heading_out == 0 || roll_out == 0 || pitch_out == 0)    return -1;

    char           line[CSV_IMU_MAX_LINE];
    CsvImuSample_t sample;
    uint32_t       n      = 0U;
    float          sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;

    while (n < max_samples) {
        if (getline_fn(line, sizeof(line), ctx) == NULL) break;
        if (csv_imu_parse_sample_line(line, &sample) != 0) continue;

        /* Store raw g values temporarily for mean removal */
        ax_ms2_out[n]  = sample.ax_g;
        ay_ms2_out[n]  = sample.ay_g;
        az_ms2_out[n]  = sample.az_g;
        heading_out[n] = sample.heading_deg;
        roll_out[n]    = sample.roll_deg;
        pitch_out[n]   = sample.pitch_deg;

        sum_ax += sample.ax_g;
        sum_ay += sample.ay_g;
        sum_az += sample.az_g;
        n++;
    }

    if (n == 0U) return -2;

    float mean_ax = sum_ax / (float)n;
    float mean_ay = sum_ay / (float)n;
    float mean_az = sum_az / (float)n;

    /* Mean-remove and convert g → m/s² for all acceleration channels */
    for (uint32_t i = 0U; i < n; i++) {
        ax_ms2_out[i] = (ax_ms2_out[i] - mean_ax) * CSV_IMU_G_TO_MS2;
        ay_ms2_out[i] = (ay_ms2_out[i] - mean_ay) * CSV_IMU_G_TO_MS2;
        az_ms2_out[i] = (az_ms2_out[i] - mean_az) * CSV_IMU_G_TO_MS2;
    }

    result->n_samples = n;
    result->mean_ax_g = mean_ax;
    result->mean_ay_g = mean_ay;
    result->mean_az_g = mean_az;

    return 0;
}

/* ---------------------------------------------------------------------------
 * csv_imu_parse_az_from_source
 * --------------------------------------------------------------------------- */
int csv_imu_parse_az_from_source(CsvImuGetlineFn  getline_fn,
                                  void            *ctx,
                                  float           *az_ms2_out,
                                  uint32_t         max_samples,
                                  CsvImuResult_t  *result)
{
    if (getline_fn == 0 || az_ms2_out == 0 || result == 0) return -1;

    char           line[CSV_IMU_MAX_LINE];
    CsvImuSample_t sample;
    uint32_t       n     = 0U;
    float          sum_g = 0.0f;

    while (n < max_samples) {
        if (getline_fn(line, sizeof(line), ctx) == NULL) break;
        if (csv_imu_parse_sample_line(line, &sample) != 0) continue;
        az_ms2_out[n] = sample.az_g;
        sum_g += sample.az_g;
        n++;
    }

    if (n == 0U) return -2;

    float mean_g = sum_g / (float)n;
    for (uint32_t i = 0U; i < n; i++) {
        az_ms2_out[i] = (az_ms2_out[i] - mean_g) * CSV_IMU_G_TO_MS2;
    }

    result->n_samples = n;
    result->mean_az_g = mean_g;
    return 0;
}

/* ---------------------------------------------------------------------------
 * csv_imu_parse_az — convenience wrapper using built-in stub
 * --------------------------------------------------------------------------- */
int csv_imu_parse_az(float          *az_ms2_out,
                     uint32_t        max_samples,
                     CsvImuResult_t *result)
{
    s_line_index = 0U;
    return csv_imu_parse_az_from_source(csv_imu_getline, 0,
                                        az_ms2_out, max_samples, result);
}
