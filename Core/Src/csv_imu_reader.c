/*
 * csv_imu_reader.c
 *
 * CSV IMU adapter — parses az column from Batsi's S001_IMU.CSV format,
 * removes mean, converts g to m/s², and prepares data for wave_processor_run().
 *
 * File I/O layer:
 *   csv_imu_getline() is the only function that touches the data source.
 *   Currently it reads from an in-memory test array (stub mode).
 *   To switch to real SD card / FATFS, replace csv_imu_getline() with:
 *       f_gets(line, sizeof(line), &fil);
 *   Everything else stays the same.
 *
 * Test CSV format (matches Batsi's S001_IMU.CSV spec):
 *   timestamp_ms, ax, ay, az, gx, gy, gz, mx, my, mz, heading_deg, roll_deg, pitch_deg
 *   1000,0.0123,-0.0045,0.9987,0.12,-0.34,0.01,23.4,-12.1,45.6,127.3,1.2,-0.8
 */

#include "csv_imu_reader.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * Stub in-memory CSV source
 *
 * Replace this section with FATFS file handle when SD card is available.
 * The test lines below simulate a small segment of Batsi-style IMU data.
 * az values are near 1.0 g (gravity) with small wave-induced variation.
 * --------------------------------------------------------------------------- */

/* Forward declaration — defined at bottom of file */
static const char * const s_test_csv_lines[];
static const uint32_t     s_test_csv_n_lines;

static uint32_t s_line_index = 0U;

/*
 * csv_imu_getline — returns next CSV line or NULL when exhausted.
 *
 * STUB VERSION: reads from s_test_csv_lines[].
 * FATFS VERSION: replace body with:
 *     return f_gets(buf, buf_size, &fil) ? buf : NULL;
 */
static const char *csv_imu_getline(char *buf, uint32_t buf_size)
{
    if (s_line_index >= s_test_csv_n_lines) {
        return NULL;
    }
    const char *src = s_test_csv_lines[s_line_index++];
    uint32_t len = (uint32_t)strlen(src);
    if (len >= buf_size) len = buf_size - 1U;
    memcpy(buf, src, len);
    buf[len] = '\0';
    return buf;
}

/* ---------------------------------------------------------------------------
 * csv_parse_column_float
 *
 * Extract the float value at column col_idx (0-based) from a CSV line.
 * Returns 1 on success, 0 on failure.
 * --------------------------------------------------------------------------- */
static int csv_parse_column_float(const char *line, uint32_t col_idx, float *out)
{
    const char *p = line;
    uint32_t col = 0U;

    while (col < col_idx) {
        /* Advance to next comma */
        while (*p != '\0' && *p != ',') p++;
        if (*p == '\0') return 0;   /* not enough columns */
        p++;   /* skip comma */
        col++;
    }

    /* Skip leading whitespace */
    while (*p == ' ' || *p == '\t') p++;

    if (*p == '\0' || *p == '\n' || *p == '\r') return 0;

    char *end;
    *out = strtof(p, &end);
    return (end != p) ? 1 : 0;
}

/* ---------------------------------------------------------------------------
 * csv_imu_parse_az
 * --------------------------------------------------------------------------- */
int csv_imu_parse_az(float          *az_ms2_out,
                     uint32_t        max_samples,
                     CsvImuResult_t *result)
{
    if (az_ms2_out == 0 || result == 0) return -1;

    char     line[CSV_IMU_MAX_LINE];
    uint32_t n       = 0U;
    float    sum_g   = 0.0f;
    float    az_g    = 0.0f;

    /* Reset stub line index for repeatable test runs */
    s_line_index = 0U;

    /* Pass 1: read az_g values into output buffer, accumulate sum for mean */
    while (n < max_samples) {
        if (csv_imu_getline(line, sizeof(line)) == NULL) break;

        /* Skip header line if present */
        if (line[0] == 't' || line[0] == 'T' || line[0] == '#') continue;

        /* Skip empty lines */
        if (line[0] == '\0' || line[0] == '\n' || line[0] == '\r') continue;

        if (!csv_parse_column_float(line, CSV_IMU_COL_AZ, &az_g)) continue;

        az_ms2_out[n] = az_g;   /* store raw g value temporarily */
        sum_g += az_g;
        n++;
    }

    if (n == 0U) return -2;

    /* Compute mean and remove it, then convert g → m/s² */
    float mean_g = sum_g / (float)n;

    for (uint32_t i = 0U; i < n; i++) {
        az_ms2_out[i] = (az_ms2_out[i] - mean_g) * CSV_IMU_G_TO_MS2;
    }

    result->n_samples  = n;
    result->mean_az_g  = mean_g;

    return 0;
}

/* ---------------------------------------------------------------------------
 * Stub test data
 *
 * 16 lines of synthetic IMU CSV matching Batsi's column format.
 * az values oscillate around 1.0 g to simulate a buoy in small waves.
 * Replace with real SD card reads when FATFS is available.
 * --------------------------------------------------------------------------- */
static const char * const s_test_csv_lines[] = {
    /* header */
    "timestamp_ms,ax,ay,az,gx,gy,gz,mx,my,mz,heading_deg,roll_deg,pitch_deg",
    /* data rows — az column (index 3) oscillates around 1.0 g */
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

static const uint32_t s_test_csv_n_lines =
    sizeof(s_test_csv_lines) / sizeof(s_test_csv_lines[0]);
