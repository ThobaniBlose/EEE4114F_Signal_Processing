/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wave_mode.h"
#include "wave_full_pipeline.h"
#include "sharc_process.h"
#include "wave_packet.h"
#include "direction_processor.h"
#include "s001_replay_segment.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

COMP_HandleTypeDef hcomp1;

I2C_HandleTypeDef hi2c1;
SMBUS_HandleTypeDef hsmbus2;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

SAI_HandleTypeDef hsai_BlockB1;
SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockA2;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_COMP1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_SMBUS_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SAI1_Init(void);
static void MX_SAI2_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

static void test_full_pipeline_mean_removal(void)
{
    char buf[256];
    const float g0 = 9.80665f;
    const uint32_t n = S001_REPLAY_N_SAMPLES;

    double sum_ax = 0.0, sum_ay = 0.0, sum_az = 0.0;
    for (uint32_t i = 0U; i < n; i++) {
        sum_ax += (double)s001_ax_g[i] * (double)g0;
        sum_ay += (double)s001_ay_g[i] * (double)g0;
        sum_az += (double)s001_az_g[i] * (double)g0;
    }
    double mean_ax_ms2 = sum_ax / (double)n;
    double mean_ay_ms2 = sum_ay / (double)n;
    double mean_az_ms2 = sum_az / (double)n;

    double dyn_sumsq_ax = 0.0, dyn_sumsq_ay = 0.0, dyn_sumsq_az = 0.0;
    double dyn_sum_ax = 0.0, dyn_sum_ay = 0.0, dyn_sum_az = 0.0;
    for (uint32_t i = 0U; i < n; i++) {
        double ax_dyn = ((double)s001_ax_g[i] * (double)g0) - mean_ax_ms2;
        double ay_dyn = ((double)s001_ay_g[i] * (double)g0) - mean_ay_ms2;
        double az_dyn = ((double)s001_az_g[i] * (double)g0) - mean_az_ms2;
        dyn_sum_ax += ax_dyn;
        dyn_sum_ay += ay_dyn;
        dyn_sum_az += az_dyn;
        dyn_sumsq_ax += ax_dyn * ax_dyn;
        dyn_sumsq_ay += ay_dyn * ay_dyn;
        dyn_sumsq_az += az_dyn * az_dyn;
    }
    double dyn_mean_ax = dyn_sum_ax / (double)n;
    double dyn_mean_ay = dyn_sum_ay / (double)n;
    double dyn_mean_az = dyn_sum_az / (double)n;
    double dyn_std_ax = sqrt(dyn_sumsq_ax / (double)(n - 1U));
    double dyn_std_ay = sqrt(dyn_sumsq_ay / (double)(n - 1U));
    double dyn_std_az = sqrt(dyn_sumsq_az / (double)(n - 1U));

    uart_print("\r\n============================================================\r\n");
    uart_print("STEP 6: FULL-PIPELINE G-CONVERSION AND MEAN REMOVAL\r\n");
    uart_print("============================================================\r\n");

    snprintf(buf, sizeof(buf),
             "Mean accel before removal [m/s2]:\r\n"
             "  [%.9f, %.9f, %.9f]\r\n",
             mean_ax_ms2, mean_ay_ms2, mean_az_ms2);
    uart_print(buf);

    snprintf(buf, sizeof(buf),
             "\r\nDynamic accel after removal [m/s2]:\r\n"
             "  mean = [%.6e, %.6e, %.6e]\r\n",
             dyn_mean_ax, dyn_mean_ay, dyn_mean_az);
    uart_print(buf);

    snprintf(buf, sizeof(buf),
             "  std  = [%.6e, %.6e, %.6e]\r\n",
             dyn_std_ax, dyn_std_ay, dyn_std_az);
    uart_print(buf);

    uart_print("\r\nMATLAB expected dynamic std:\r\n");
    uart_print("  [3.660e-02, 3.366e-02, 5.569e-02] m/s2\r\n");

    if (fabs(dyn_mean_az) < 1.0e-6) {
        uart_print("\r\nPASS: Mean removal produces near-zero dynamic mean.\r\n");
    } else {
        uart_print("\r\nCHECK: Dynamic mean larger than expected.\r\n");
    }
    uart_print("STEP 6 COMPLETE.\r\n");
}

static void test_full_pipeline_filter_coefficients(void)
{
    char buf[256];

    uart_print("\r\n============================================================\r\n");
    uart_print("STEP 7: FULL-PIPELINE FILTER COEFFICIENT CHECK\r\n");
    uart_print("============================================================\r\n");

    snprintf(buf, sizeof(buf),
             "BP b: [%.6e, %.6e, %.6e, %.6e, %.6e]\r\n",
             (double)wfp_bp_b[0], (double)wfp_bp_b[1], (double)wfp_bp_b[2],
             (double)wfp_bp_b[3], (double)wfp_bp_b[4]);
    uart_print(buf);

    snprintf(buf, sizeof(buf),
             "BP a: [%.6e, %.6e, %.6e, %.6e, %.6e]\r\n",
             (double)wfp_bp_a[0], (double)wfp_bp_a[1], (double)wfp_bp_a[2],
             (double)wfp_bp_a[3], (double)wfp_bp_a[4]);
    uart_print(buf);

    snprintf(buf, sizeof(buf),
             "HP b: [%.6e, %.6e, %.6e]\r\n",
             (double)wfp_hp_b[0], (double)wfp_hp_b[1], (double)wfp_hp_b[2]);
    uart_print(buf);

    snprintf(buf, sizeof(buf),
             "HP a: [%.6e, %.6e, %.6e]\r\n",
             (double)wfp_hp_a[0], (double)wfp_hp_a[1], (double)wfp_hp_a[2]);
    uart_print(buf);

    uart_print("PASS: Filter coefficients loaded.\r\n");
    uart_print("STEP 7 COMPLETE.\r\n");
}

/* Single shared DSP buffer — includes padding for filtfilt edge handling */
/* These are defined in sharc_process.c — reuse them here for ATP tests */
extern float s_buf_a[];
extern float s_buf_eta[];
#define wfp_buf     s_buf_a
#define wfp_eta_buf s_buf_eta
/* Cross-spectral bin arrays for band direction */
static WfpCrossBin_t wfp_eta_x_bins[WFP_WAVE_N_BINS];
static WfpCrossBin_t wfp_eta_y_bins[WFP_WAVE_N_BINS];

/* ATP4B: band direction over 0.08-0.40 Hz */
#define ATP4_K_MIN     (14U)
#define ATP4_K_MAX     (65U)
#define ATP4_N_BINS    (ATP4_K_MAX - ATP4_K_MIN + 1U)
static float atp4_eta_buf[WFP_WELCH_WIN_LEN];
static float atp4_h_buf[WFP_WELCH_WIN_LEN];
static WfpCrossBin_t atp4_eta_north_bins[ATP4_N_BINS];
static WfpCrossBin_t atp4_eta_east_bins[ATP4_N_BINS];

static void test_full_pipeline_bandpass_filter(void)
{
    char buf[256];

    uart_print("\r\n============================================================\r\n");
    uart_print("STEP 8: FULL-PIPELINE BANDPASS FILTER TEST\r\n");
    uart_print("============================================================\r\n");

    wfp_make_dynamic_accel_ms2(s001_ax_g, wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_filtfilt_bp_order4_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    float ax_rms = wfp_rms(wfp_buf, S001_REPLAY_N_SAMPLES);

    wfp_make_dynamic_accel_ms2(s001_ay_g, wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_filtfilt_bp_order4_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    float ay_rms = wfp_rms(wfp_buf, S001_REPLAY_N_SAMPLES);

    wfp_make_dynamic_accel_ms2(s001_az_g, wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_filtfilt_bp_order4_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    float az_rms = wfp_rms(wfp_buf, S001_REPLAY_N_SAMPLES);

    snprintf(buf, sizeof(buf),
             "Bandpass filtered accel RMS [m/s2]:\r\n"
             "  ax_filt = %.9e\r\n"
             "  ay_filt = %.9e\r\n"
             "  az_filt = %.9e\r\n",
             (double)ax_rms, (double)ay_rms, (double)az_rms);
    uart_print(buf);

    uart_print("\r\nMATLAB targets:\r\n");
    uart_print("  ax_filt = 6.930560786e-03\r\n");
    uart_print("  ay_filt = 6.219773947e-03\r\n");
    uart_print("  az_filt = 4.109526795e-02\r\n");

    uart_print("\r\nSTEP 8 COMPLETE.\r\n");
}

static void test_full_pipeline_velocity_stage(void)
{
    char buf[256];
    const float dt_s = 1.0f / WFP_FS_HZ;

    uart_print("\r\n============================================================\r\n");
    uart_print("STEP 9: FULL-PIPELINE VELOCITY RECOVERY TEST\r\n");
    uart_print("============================================================\r\n");

    /* vx_body from ax */
    wfp_make_dynamic_accel_ms2(s001_ax_g, wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_filtfilt_bp_order4_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_cumtrapz_inplace(wfp_buf, S001_REPLAY_N_SAMPLES, dt_s);
    wfp_filtfilt_hp_order2_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    float vx_rms = wfp_rms(wfp_buf, S001_REPLAY_N_SAMPLES);

    /* vy_body from ay */
    wfp_make_dynamic_accel_ms2(s001_ay_g, wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_filtfilt_bp_order4_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_cumtrapz_inplace(wfp_buf, S001_REPLAY_N_SAMPLES, dt_s);
    wfp_filtfilt_hp_order2_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    float vy_rms = wfp_rms(wfp_buf, S001_REPLAY_N_SAMPLES);

    /* vz from az */
    wfp_make_dynamic_accel_ms2(s001_az_g, wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_filtfilt_bp_order4_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_cumtrapz_inplace(wfp_buf, S001_REPLAY_N_SAMPLES, dt_s);
    wfp_filtfilt_hp_order2_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    float vz_rms = wfp_rms(wfp_buf, S001_REPLAY_N_SAMPLES);

    snprintf(buf, sizeof(buf),
             "Recovered velocity RMS [m/s]:\r\n"
             "  vx_body = %.9e\r\n"
             "  vy_body = %.9e\r\n"
             "  vz      = %.9e\r\n",
             (double)vx_rms, (double)vy_rms, (double)vz_rms);
    uart_print(buf);

    uart_print("\r\nMATLAB targets:\r\n");
    uart_print("  vx_body = 1.226652456e-02\r\n");
    uart_print("  vy_body = 1.188385420e-02\r\n");
    uart_print("  vz      = 1.019634046e-01\r\n");

    uart_print("\r\nSTEP 9 COMPLETE.\r\n");
}

static float local_max_abs_float(const float *x, uint32_t n)
{
    float max_abs = 0.0f;
    for (uint32_t i = 0U; i < n; i++) {
        float a = fabsf(x[i]);
        if (a > max_abs) max_abs = a;
    }
    return max_abs;
}

static void test_full_pipeline_displacement_stage(void)
{
    char buf[256];
    const float dt_s = 1.0f / WFP_FS_HZ;

    uart_print("\r\n============================================================\r\n");
    uart_print("STEP 10: FULL-PIPELINE VERTICAL DISPLACEMENT TEST\r\n");
    uart_print("============================================================\r\n");

    /* az_dyn → bandpass → integrate → HP → integrate → HP = eta */
    wfp_make_dynamic_accel_ms2(s001_az_g, wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_filtfilt_bp_order4_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_cumtrapz_inplace(wfp_buf, S001_REPLAY_N_SAMPLES, dt_s);
    wfp_filtfilt_hp_order2_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_cumtrapz_inplace(wfp_buf, S001_REPLAY_N_SAMPLES, dt_s);
    wfp_filtfilt_hp_order2_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);

    float eta_rms     = wfp_rms(wfp_buf, S001_REPLAY_N_SAMPLES);
    float eta_max_abs = local_max_abs_float(wfp_buf, S001_REPLAY_N_SAMPLES);

    snprintf(buf, sizeof(buf),
             "Recovered vertical displacement:\r\n"
             "  eta RMS     = %.9e m\r\n"
             "  eta max abs = %.9e m\r\n",
             (double)eta_rms, (double)eta_max_abs);
    uart_print(buf);

    uart_print("\r\nMATLAB targets:\r\n");
    uart_print("  eta RMS     = 2.680921055e-01 m\r\n");
    uart_print("  eta max abs = 6.595198828e-01 m\r\n");

    uart_print("\r\nSTEP 10 COMPLETE.\r\n");
}

static void test_full_pipeline_wave_params(void)
{
    char buf[256];
    const float dt_s = 1.0f / WFP_FS_HZ;

    uart_print("\r\n============================================================\r\n");
    uart_print("STEP 11: FULL-PIPELINE NON-DIRECTIONAL WAVE PARAMETERS\r\n");
    uart_print("============================================================\r\n");

    /* Reconstruct eta into wfp_buf (same chain as Step 10) */
    wfp_make_dynamic_accel_ms2(s001_az_g, wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_filtfilt_bp_order4_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_cumtrapz_inplace(wfp_buf, S001_REPLAY_N_SAMPLES, dt_s);
    wfp_filtfilt_hp_order2_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_cumtrapz_inplace(wfp_buf, S001_REPLAY_N_SAMPLES, dt_s);
    wfp_filtfilt_hp_order2_inplace(wfp_buf, S001_REPLAY_N_SAMPLES);

    /* Detrend eta (remove any residual DC) */
    {
        double sum = 0.0;
        for (uint32_t i = 0; i < S001_REPLAY_N_SAMPLES; i++) sum += (double)wfp_buf[i];
        float mean_eta = (float)(sum / (double)S001_REPLAY_N_SAMPLES);
        for (uint32_t i = 0; i < S001_REPLAY_N_SAMPLES; i++) wfp_buf[i] -= mean_eta;
    }

    /* Welch PSD of eta — direct DFT on wave-band bins only.
     * Settings: win=8192, hop=4096, nfft=16384, wave band 0.04-0.40 Hz */
    const uint32_t win_len = WFP_WELCH_WIN_LEN;    /* 8192 */
    const uint32_t hop_len = WFP_WELCH_NOVERLAP;   /* 4096 */
    const uint32_t nfft    = WFP_WELCH_NFFT;       /* 16384 */
    const float    df      = WFP_FS_HZ / (float)nfft;
    const uint32_t k_lo    = (uint32_t)ceilf(WFP_WAVE_LO_HZ / df);
    const uint32_t k_hi    = (uint32_t)floorf(WFP_WAVE_HI_HZ / df);

    /* Hann window (periodic) — compute on the fly to save static RAM */
    /* PSD accumulator — reuse a portion of memory after eta */
    static float psd_acc[820];  /* enough for k_hi - k_lo + 1 bins (max ~66) */
    for (uint32_t k = 0; k < 820; k++) psd_acc[k] = 0.0f;

    /* Hann power correction */
    double hann_pwr_sum = 0.0;
    for (uint32_t n = 0; n < win_len; n++) {
        double w = 0.5 * (1.0 - cos(2.0 * M_PI * (double)n / (double)win_len));
        hann_pwr_sum += w * w;
    }
    float hann_pwr = (float)(hann_pwr_sum / (double)win_len);

    uint32_t n_segs = 0;
    uint32_t last_start = S001_REPLAY_N_SAMPLES - win_len;

    for (uint32_t start = 0; start <= last_start; start += hop_len) {
        for (uint32_t k = k_lo; k <= k_hi; k++) {
            float step = 2.0f * (float)M_PI * (float)k / (float)nfft;
            float c_step = cosf(step);
            float s_step = sinf(step);
            float c = 1.0f, s = 0.0f;
            float re = 0.0f, im = 0.0f;

            for (uint32_t n = 0; n < win_len; n++) {
                /* Apply Hann window inline */
                float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI
                                               * (float)n / (float)win_len));
                float x = w * wfp_buf[start + n];
                re += x * c;
                im -= x * s;
                float c_new = c * c_step - s * s_step;
                float s_new = s * c_step + c * s_step;
                c = c_new;
                s = s_new;
            }

            float S_k = 2.0f * (re*re + im*im)
                        / ((float)win_len * hann_pwr * WFP_FS_HZ);
            psd_acc[k - k_lo] += S_k;
        }
        n_segs++;
    }

    /* Spectral moments with trapezoidal weights */
    float m0 = 0.0f, m1 = 0.0f, m2 = 0.0f;
    float S_peak = 0.0f;
    uint32_t k_peak = k_lo;

    for (uint32_t k = k_lo; k <= k_hi; k++) {
        float f_k = (float)k * df;
        float S_k = psd_acc[k - k_lo] / (float)n_segs;
        float tw  = (k == k_lo || k == k_hi) ? 0.5f : 1.0f;
        m0 += tw * S_k * df;
        m1 += tw * f_k * S_k * df;
        m2 += tw * f_k * f_k * S_k * df;
        if (S_k > S_peak) { S_peak = S_k; k_peak = k; }
    }

    float Hm0  = 4.0f * sqrtf(m0);
    float fp   = (float)k_peak * df;
    float Tp   = (fp > 0.0f) ? 1.0f / fp : 0.0f;
    float Tm01 = (m1 > 0.0f) ? m0 / m1 : 0.0f;
    float Tm02 = (m2 > 0.0f) ? sqrtf(m0 / m2) : 0.0f;

    snprintf(buf, sizeof(buf),
             "Wave parameters from eta:\r\n"
             "  Hm0  = %.6f m\r\n"
             "  fp   = %.6f Hz\r\n"
             "  Tp   = %.6f s\r\n"
             "  Tm01 = %.6f s\r\n"
             "  Tm02 = %.6f s\r\n"
             "  segs = %lu\r\n",
             (double)Hm0, (double)fp, (double)Tp,
             (double)Tm01, (double)Tm02, (unsigned long)n_segs);
    uart_print(buf);

    uart_print("\r\nMATLAB golden (truncated full-pipeline):\r\n");
    uart_print("  Hm0  = 1.013638 m\r\n");
    uart_print("  fp   = 0.054932 Hz\r\n");
    uart_print("  Tp   = 18.204444 s\r\n");
    uart_print("  Tm01 = 16.606034 s\r\n");
    uart_print("  Tm02 = 16.372861 s\r\n");

    float err_Hm0  = 100.0f * fabsf((Hm0  - 1.013638f)  / 1.013638f);
    float err_Tp   = 100.0f * fabsf((Tp   - 18.204444f) / 18.204444f);
    float err_Tm01 = 100.0f * fabsf((Tm01 - 16.606034f) / 16.606034f);
    float err_Tm02 = 100.0f * fabsf((Tm02 - 16.372861f) / 16.372861f);

    snprintf(buf, sizeof(buf),
             "\r\nErrors: Hm0=%.3f%%  Tp=%.3f%%  Tm01=%.3f%%  Tm02=%.3f%%\r\n",
             (double)err_Hm0, (double)err_Tp,
             (double)err_Tm01, (double)err_Tm02);
    uart_print(buf);

    if (err_Hm0 < 5.0f && err_Tp < 5.0f && err_Tm01 < 5.0f && err_Tm02 < 5.0f) {
        uart_print("\r\nPASS: Full-pipeline wave params match MATLAB golden.\r\n");
    } else {
        uart_print("\r\nCHECK: Full-pipeline wave params differ from MATLAB.\r\n");
    }
    uart_print("STEP 11 COMPLETE.\r\n");
}

static void test_full_pipeline_direction_bin_setup(void)
{
    char buf[256];

    uart_print("\r\n============================================================\r\n");
    uart_print("STEP 12A: DIRECTIONAL WAVE-BAND BIN SETUP\r\n");
    uart_print("============================================================\r\n");

    float f_min = (float)WFP_WAVE_K_MIN * WFP_DF_HZ;
    float f_max = (float)WFP_WAVE_K_MAX * WFP_DF_HZ;

    snprintf(buf, sizeof(buf),
             "Spectral setup:\r\n"
             "  fs     = %.1f Hz\r\n"
             "  nfft   = %lu\r\n"
             "  df     = %.9f Hz\r\n"
             "  k_min  = %lu\r\n"
             "  k_max  = %lu\r\n"
             "  n_bins = %lu\r\n"
             "  f_min  = %.9f Hz\r\n"
             "  f_max  = %.9f Hz\r\n",
             (double)WFP_FS_HZ,
             (unsigned long)WFP_WELCH_NFFT,
             (double)WFP_DF_HZ,
             (unsigned long)WFP_WAVE_K_MIN,
             (unsigned long)WFP_WAVE_K_MAX,
             (unsigned long)WFP_WAVE_N_BINS,
             (double)f_min, (double)f_max);
    uart_print(buf);

    uart_print("\r\nMATLAB wave band: 0.040 to 0.400 Hz\r\n");

    if (f_min >= 0.04f && f_max <= 0.40f) {
        uart_print("PASS: Directional wave-bin mapping correct.\r\n");
    } else {
        uart_print("CHECK: Bin mapping outside target band.\r\n");
    }
    uart_print("STEP 12A COMPLETE.\r\n");
}

static void test_full_pipeline_peak_direction(void)
{
    char buf[256];

    uart_print("\r\n============================================================\r\n");
    uart_print("STEP 12B: BODY-RELATIVE PEAK-BIN DIRECTION TEST\r\n");
    uart_print("============================================================\r\n");

    WfpCrossBin_t eta_x, eta_y;
    WfpDirectionPeak_t dir_peak;

    /* Reconstruct eta and keep it in wfp_eta_buf */
    wfp_reconstruct_eta_from_az_g(s001_az_g, wfp_eta_buf, S001_REPLAY_N_SAMPLES);

    /* Reconstruct vx_body, compute eta-vx cross at peak bin */
    wfp_reconstruct_velocity_from_accel_g(s001_ax_g, wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_cross_bin_eta_h(wfp_eta_buf, wfp_buf, S001_REPLAY_N_SAMPLES,
                        WFP_PEAK_K_S001_TRUNC, &eta_x);

    /* Reconstruct vy_body (reuses wfp_buf), compute eta-vy cross */
    wfp_reconstruct_velocity_from_accel_g(s001_ay_g, wfp_buf, S001_REPLAY_N_SAMPLES);
    wfp_cross_bin_eta_h(wfp_eta_buf, wfp_buf, S001_REPLAY_N_SAMPLES,
                        WFP_PEAK_K_S001_TRUNC, &eta_y);

    wfp_direction_peak_from_cross(&eta_x, &eta_y, &dir_peak);

    snprintf(buf, sizeof(buf),
             "Peak-bin direction:\r\n"
             "  k_peak      = %lu\r\n"
             "  f_peak      = %.6f Hz\r\n"
             "  segs        = %lu\r\n"
             "  peak from   = %.6f deg\r\n"
             "  peak toward = %.6f deg\r\n"
             "  r1 peak     = %.6f\r\n"
             "  vector coh  = %.6f\r\n",
             (unsigned long)WFP_PEAK_K_S001_TRUNC,
             (double)((float)WFP_PEAK_K_S001_TRUNC * WFP_DF_HZ),
             (unsigned long)dir_peak.n_segs,
             (double)dir_peak.peak_from_deg,
             (double)dir_peak.peak_toward_deg,
             (double)dir_peak.r1_peak,
             (double)dir_peak.vector_coh_peak);
    uart_print(buf);

    uart_print("\r\nMATLAB target:\r\n");
    uart_print("  peak from = 13.530366 deg\r\n");
    uart_print("  r1 peak   = 0.753947\r\n");

    uart_print("\r\nSTEP 12B COMPLETE.\r\n");
}

static void test_full_pipeline_band_direction(void)
{
    char buf[256];

    uart_print("\r\n============================================================\r\n");
    uart_print("STEP 12C: BODY-RELATIVE BAND-INTEGRATED DIRECTION TEST\r\n");
    uart_print("============================================================\r\n");

    WfpDirectionBand_t dir_band;

    uart_print("12C.1 Reconstructing eta from az...\r\n");
    wfp_reconstruct_eta_from_az_g(s001_az_g, wfp_eta_buf, S001_REPLAY_N_SAMPLES);
    uart_print("12C.1 done.\r\n");

    uart_print("12C.2 Reconstructing vx_body from ax...\r\n");
    wfp_reconstruct_velocity_from_accel_g(s001_ax_g, wfp_buf, S001_REPLAY_N_SAMPLES);
    uart_print("12C.2 done.\r\n");

    uart_print("12C.3 Computing eta-vx cross spectra...\r\n");
    for (uint32_t i = 0U; i < WFP_WAVE_N_BINS; i++) {
        uint32_t k = WFP_WAVE_K_MIN + i;
        if ((i % 10U) == 0U) {
            snprintf(buf, sizeof(buf), "  eta-vx bin i=%lu k=%lu\r\n",
                     (unsigned long)i, (unsigned long)k);
            uart_print(buf);
        }
        wfp_cross_bin_eta_h(wfp_eta_buf, wfp_buf, S001_REPLAY_N_SAMPLES,
                            k, &wfp_eta_x_bins[i]);
    }
    uart_print("12C.3 done.\r\n");

    uart_print("12C.4 Reconstructing vy_body from ay...\r\n");
    wfp_reconstruct_velocity_from_accel_g(s001_ay_g, wfp_buf, S001_REPLAY_N_SAMPLES);
    uart_print("12C.4 done.\r\n");

    uart_print("12C.5 Computing eta-vy cross spectra...\r\n");
    for (uint32_t i = 0U; i < WFP_WAVE_N_BINS; i++) {
        uint32_t k = WFP_WAVE_K_MIN + i;
        if ((i % 10U) == 0U) {
            snprintf(buf, sizeof(buf), "  eta-vy bin i=%lu k=%lu\r\n",
                     (unsigned long)i, (unsigned long)k);
            uart_print(buf);
        }
        wfp_cross_bin_eta_h(wfp_eta_buf, wfp_buf, S001_REPLAY_N_SAMPLES,
                            k, &wfp_eta_y_bins[i]);
    }
    uart_print("12C.5 done.\r\n");

    uart_print("12C.6 Integrating directional result...\r\n");
    wfp_direction_band_from_cross(wfp_eta_x_bins, wfp_eta_y_bins, &dir_band);
    uart_print("12C.6 done.\r\n");

    snprintf(buf, sizeof(buf),
             "Band direction result:\r\n"
             "  peak k     = %lu\r\n"
             "  peak f     = %.6f Hz\r\n"
             "  mean from  = %.6f deg\r\n"
             "  peak from  = %.6f deg\r\n"
             "  r1 band    = %.6f\r\n"
             "  r1 peak    = %.6f\r\n"
             "  mean coh   = %.6f\r\n"
             "  confident  = %lu\r\n"
             "  valid bins = %lu\r\n",
             (unsigned long)dir_band.peak_k,
             (double)dir_band.peak_freq_hz,
             (double)dir_band.mean_from_deg,
             (double)dir_band.peak_from_deg,
             (double)dir_band.r1_band,
             (double)dir_band.r1_peak,
             (double)dir_band.mean_vector_coh,
             (unsigned long)dir_band.direction_confident,
             (unsigned long)dir_band.valid_bins);
    uart_print(buf);

    uart_print("\r\nMATLAB target:\r\n");
    uart_print("  mean from  = 18.940496 deg\r\n");
    uart_print("  peak from  = 13.530366 deg\r\n");
    uart_print("  r1 band    = 0.681391\r\n");
    uart_print("  r1 peak    = 0.753947\r\n");
    uart_print("  mean coh   = 0.397559\r\n");
    uart_print("  confident  = 1\r\n");
    uart_print("  valid bins = 11\r\n");

    uart_print("\r\nSTEP 12C COMPLETE.\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  // MX_ADC1_Init();
  // MX_CAN1_Init();
  // MX_COMP1_Init();
  // MX_I2C1_Init();
  // MX_I2C2_SMBUS_Init();
  // MX_USART2_UART_Init();
  // MX_USART3_UART_Init();
  // MX_SAI1_Init();
  // MX_SAI2_Init();
  // MX_SDMMC1_SD_Init();
  // MX_SPI1_Init();
  // MX_SPI3_Init();
  // MX_TIM1_Init();
  // MX_TIM2_Init();
  // MX_TIM3_Init();
  // MX_TIM4_Init();
  // MX_TIM15_Init();
  // MX_USB_OTG_FS_USB_Init();

  /* USER CODE BEGIN 2 */
  char buf[256];

  /* Set to 1 to run the full step-by-step validation suite.
   * Set to 0 for production single-pass processing. */
  #define SHARC_RUN_VALIDATION  0

#if SHARC_RUN_VALIDATION
  /* --- Full validation suite (Steps 4-12C) lives here --- */
  /* Enable this only when you need to re-prove the build. */
  uart_print("VALIDATION MODE: Enable SHARC_RUN_VALIDATION=1 to run.\r\n");
#else
  /* --- Production single-pass processor --- */
  uart_print("\r\n============================================================\r\n");
  uart_print("SHARC SINGLE-PASS WINDOW PROCESSOR\r\n");
  uart_print("============================================================\r\n");

  SharcResult_t sharc_result;
  int sharc_status = sharc_process_window(
      s001_ax_g, s001_ay_g, s001_az_g,
      s001_mx_uT, s001_my_uT, s001_mz_uT,
      s001_heading_deg, s001_roll_deg, s001_pitch_deg,
      S001_REPLAY_N_SAMPLES,
      &sharc_result);

  if (sharc_status != 0) {
    snprintf(buf, sizeof(buf), "SHARC process error: %d\r\n", sharc_status);
    uart_print(buf);
    Error_Handler();
  }

  snprintf(buf, sizeof(buf),
           "Mode: %s\r\n\r\n"
           "Non-directional:\r\n"
           "  Hm0  = %.6f m\r\n"
           "  Tp   = %.6f s\r\n"
           "  Tm01 = %.6f s\r\n"
           "  Tm02 = %.6f s\r\n",
           wave_mode_label(sharc_result.mode),
           (double)sharc_result.Hm0, (double)sharc_result.Tp,
           (double)sharc_result.Tm01, (double)sharc_result.Tm02);
  uart_print(buf);

  snprintf(buf, sizeof(buf),
           "\r\nDirectional:\r\n"
           "  mean from  = %.3f deg\r\n"
           "  peak from  = %.3f deg\r\n"
           "  r1 band    = %.4f\r\n"
           "  r1 peak    = %.4f\r\n"
           "  mean coh   = %.4f\r\n"
           "  confident  = %lu\r\n"
           "  valid bins = %lu\r\n",
           (double)sharc_result.mean_from_deg,
           (double)sharc_result.peak_from_deg,
           (double)sharc_result.r1_band,
           (double)sharc_result.r1_peak,
           (double)sharc_result.mean_vector_coh,
           (unsigned long)sharc_result.direction_confident,
           (unsigned long)sharc_result.valid_bins);
  uart_print(buf);

  /* Format Tier-1 packet */
  WavePacket_t final_pkt = {
      .gps_fix = 0U, .lat = NAN, .lon = NAN,
      .Hm0  = sharc_result.Hm0,
      .Tp   = sharc_result.Tp,
      .Tm01 = sharc_result.Tm01,
      .Tm02 = sharc_result.Tm02,
      .dir_deg = sharc_result.mean_from_deg,
      .mode = sharc_result.mode,
      .quality = (WaveQuality_t)sharc_result.direction_confident,
      .r1  = sharc_result.r1_band,
      .coh = sharc_result.mean_vector_coh,
  };
  strncpy(final_pkt.session_id, "S001", sizeof(final_pkt.session_id));

  char pkt_out[WAVE_PACKET_MAX_LEN];
  wave_packet_format(&final_pkt, pkt_out, sizeof(pkt_out));
  uart_print("\r\nTier-1 packet:\r\n");
  uart_print(pkt_out);
  uart_print("\r\n");

  uart_print("\r\nPROCESSING COMPLETE.\r\n");

  /* =======================================================================
   * ATP3: BODY-FRAME TO NORTH/EAST ROTATION TEST
   * Uses direction_processor_rotate_horizontal() — the same function
   * that the geographic directional branch uses.
   * ======================================================================= */
  uart_print("\r\n============================================================\r\n");
  uart_print("ATP3: BODY-FRAME TO NORTH/EAST ROTATION TEST\r\n");
  uart_print("============================================================\r\n");

  {
    #define ATP3_N  5U
    float atp3_ax[ATP3_N]  = {1.0f, 1.0f, 1.0f, 0.0f, 0.0f};
    float atp3_ay[ATP3_N]  = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f};
    float atp3_hdg[ATP3_N] = {0.0f, 45.0f, 90.0f, 0.0f, 90.0f};
    float atp3_north[ATP3_N];
    float atp3_east[ATP3_N];

    DirectionProcessor_Result atp3_dir;
    direction_processor_rotate_horizontal(
        atp3_ax, atp3_ay, atp3_hdg,
        atp3_north, atp3_east,
        ATP3_N, &atp3_dir);

    for (uint32_t i = 0; i < ATP3_N; i++) {
      snprintf(buf, sizeof(buf),
               "  Case %lu: ax=%.1f ay=%.1f hdg=%.1f -> N=%.6f E=%.6f\r\n",
               (unsigned long)(i + 1U),
               (double)atp3_ax[i], (double)atp3_ay[i], (double)atp3_hdg[i],
               (double)atp3_north[i], (double)atp3_east[i]);
      uart_print(buf);
    }
    uart_print("ATP3 COMPLETE.\r\n");
  }

  /* =======================================================================
   * ATP4A: SYNTHETIC HEADING-REFERENCED PEAK DIRECTION TEST
   * ======================================================================= */
  {
    const uint32_t n = S001_REPLAY_N_SAMPLES;
    const float fs = WFP_FS_HZ;
    const uint32_t k_peak = 20U;
    const float f0 = (float)k_peak * WFP_DF_HZ;
    const float known_from_geo_deg = 60.0f;
    const float heading_deg_val = 30.0f;
    const float known_from_body_deg = known_from_geo_deg - heading_deg_val;
    const float eta_amp = 0.50f;
    const float h_amp   = 0.15f;

    WfpCrossBin_t eta_north, eta_east;
    WfpDirectionPeak_t dir_peak;

    uart_print("\r\n============================================================\r\n");
    uart_print("ATP4A: SYNTHETIC HEADING-REFERENCED PEAK DIRECTION TEST\r\n");
    uart_print("============================================================\r\n");

    snprintf(buf, sizeof(buf),
             "Synthetic case:\r\n"
             "  known geo coming-from = %.1f deg\r\n"
             "  heading               = %.1f deg\r\n"
             "  expected body-rel     = %.1f deg\r\n"
             "  k_peak = %lu  f0 = %.6f Hz\r\n",
             (double)known_from_geo_deg, (double)heading_deg_val,
             (double)known_from_body_deg,
             (unsigned long)k_peak, (double)f0);
    uart_print(buf);

    /* Build eta + North channel */
    for (uint32_t i = 0U; i < n; i++) {
      float t = (float)i / fs;
      float phase = 2.0f * (float)M_PI * f0 * t;
      float vx_body = h_amp * sinf(phase) * cosf(known_from_body_deg * (float)M_PI / 180.0f);
      float vy_body = h_amp * sinf(phase) * sinf(known_from_body_deg * (float)M_PI / 180.0f);
      float hdg = heading_deg_val * (float)M_PI / 180.0f;
      wfp_eta_buf[i] = eta_amp * cosf(phase);
      wfp_buf[i] = vx_body * cosf(hdg) - vy_body * sinf(hdg);
    }
    wfp_cross_bin_eta_h(wfp_eta_buf, wfp_buf, n, k_peak, &eta_north);

    /* Build East channel */
    for (uint32_t i = 0U; i < n; i++) {
      float t = (float)i / fs;
      float phase = 2.0f * (float)M_PI * f0 * t;
      float vx_body = h_amp * sinf(phase) * cosf(known_from_body_deg * (float)M_PI / 180.0f);
      float vy_body = h_amp * sinf(phase) * sinf(known_from_body_deg * (float)M_PI / 180.0f);
      float hdg = heading_deg_val * (float)M_PI / 180.0f;
      wfp_buf[i] = vx_body * sinf(hdg) + vy_body * cosf(hdg);
    }
    wfp_cross_bin_eta_h(wfp_eta_buf, wfp_buf, n, k_peak, &eta_east);

    wfp_direction_peak_from_cross(&eta_north, &eta_east, &dir_peak);

    float err_deg = dir_peak.peak_from_deg - known_from_geo_deg;
    while (err_deg > 180.0f)  err_deg -= 360.0f;
    while (err_deg < -180.0f) err_deg += 360.0f;

    snprintf(buf, sizeof(buf),
             "\r\nATP4A result:\r\n"
             "  estimated coming-from = %.6f deg\r\n"
             "  known reference       = %.6f deg\r\n"
             "  error                 = %.6f deg\r\n"
             "  r1 peak               = %.6f\r\n"
             "  vector coherence      = %.6f\r\n",
             (double)dir_peak.peak_from_deg,
             (double)known_from_geo_deg,
             (double)err_deg,
             (double)dir_peak.r1_peak,
             (double)dir_peak.vector_coh_peak);
    uart_print(buf);

    uart_print("ATP4A COMPLETE.\r\n");
  }

  /* =======================================================================
   * ATP4B: SYNTHETIC HEADING-REFERENCED BAND-INTEGRATED DIRECTION TEST
   * Tests geographic directional branch over trusted 0.08-0.40 Hz band.
   * ======================================================================= */
  {
    const uint32_t n = WFP_WELCH_WIN_LEN;  /* 8192 samples */
    const float fs = WFP_FS_HZ;
    const uint32_t k_peak = 20U;
    const float f0 = (float)k_peak * WFP_DF_HZ;
    const float known_from_geo_deg = 60.0f;
    const float heading_deg_val = 30.0f;
    const float known_from_body_deg = known_from_geo_deg - heading_deg_val;
    const float eta_amp = 0.50f;
    const float h_amp   = 0.15f;

    uart_print("\r\n============================================================\r\n");
    uart_print("ATP4B: SYNTHETIC HEADING-REFERENCED BAND DIRECTION TEST\r\n");
    uart_print("============================================================\r\n");

    snprintf(buf, sizeof(buf),
             "Synthetic case:\r\n"
             "  known geo coming-from = %.1f deg\r\n"
             "  heading               = %.1f deg\r\n"
             "  trusted band          = 0.08 to 0.40 Hz\r\n"
             "  k range               = %lu to %lu\r\n"
             "  k_peak = %lu  f0 = %.6f Hz\r\n",
             (double)known_from_geo_deg, (double)heading_deg_val,
             (unsigned long)ATP4_K_MIN, (unsigned long)ATP4_K_MAX,
             (unsigned long)k_peak, (double)f0);
    uart_print(buf);

    /* Build eta + North channel */
    for (uint32_t i = 0U; i < n; i++) {
      float t = (float)i / fs;
      float phase = 2.0f * (float)M_PI * f0 * t;
      float vx_body = h_amp * sinf(phase) * cosf(known_from_body_deg * (float)M_PI / 180.0f);
      float vy_body = h_amp * sinf(phase) * sinf(known_from_body_deg * (float)M_PI / 180.0f);
      float hdg = heading_deg_val * (float)M_PI / 180.0f;
      atp4_eta_buf[i] = eta_amp * cosf(phase);
      atp4_h_buf[i]   = vx_body * cosf(hdg) - vy_body * sinf(hdg);
    }
    for (uint32_t i = 0U; i < ATP4_N_BINS; i++) {
      wfp_cross_bin_eta_h(atp4_eta_buf, atp4_h_buf, n, ATP4_K_MIN + i, &atp4_eta_north_bins[i]);
    }

    /* Build East channel */
    for (uint32_t i = 0U; i < n; i++) {
      float t = (float)i / fs;
      float phase = 2.0f * (float)M_PI * f0 * t;
      float vx_body = h_amp * sinf(phase) * cosf(known_from_body_deg * (float)M_PI / 180.0f);
      float vy_body = h_amp * sinf(phase) * sinf(known_from_body_deg * (float)M_PI / 180.0f);
      float hdg = heading_deg_val * (float)M_PI / 180.0f;
      atp4_h_buf[i] = vx_body * sinf(hdg) + vy_body * cosf(hdg);
    }
    for (uint32_t i = 0U; i < ATP4_N_BINS; i++) {
      wfp_cross_bin_eta_h(atp4_eta_buf, atp4_h_buf, n, ATP4_K_MIN + i, &atp4_eta_east_bins[i]);
    }

    /* Band-integrated direction over 0.08-0.40 Hz */
    double p_eta_max = 0.0;
    uint32_t peak_i = 0U;
    for (uint32_t i = 0U; i < ATP4_N_BINS; i++) {
      if (atp4_eta_north_bins[i].p_eta > p_eta_max) {
        p_eta_max = atp4_eta_north_bins[i].p_eta;
        peak_i = i;
      }
    }
    double energy_thresh = 0.01 * p_eta_max;

    double trap_num_c1 = 0.0, trap_num_c2 = 0.0, trap_den = 0.0;
    double coh_sum = 0.0;
    uint32_t valid_bins = 0U;
    uint8_t have_prev = 0U;
    double prev_f = 0.0, prev_w = 0.0, prev_wc1 = 0.0, prev_wc2 = 0.0;
    double peak_from_deg = 0.0, peak_r1 = 0.0;

    for (uint32_t i = 0U; i < ATP4_N_BINS; i++) {
      double p_eta = atp4_eta_north_bins[i].p_eta;
      double p_n   = atp4_eta_north_bins[i].p_h;
      double p_e   = atp4_eta_east_bins[i].p_h;
      double q_n   = atp4_eta_north_bins[i].cross_im;
      double q_e   = atp4_eta_east_bins[i].cross_im;
      double den   = sqrt(p_eta * (p_n + p_e));
      double comp1 = 0.0, comp2 = 0.0;
      if (den > 0.0) { comp1 = -q_n / den; comp2 = -q_e / den; }
      double r1 = sqrt(comp1*comp1 + comp2*comp2);
      if (r1 > 0.999) r1 = 0.999;

      double theta_toward = atan2(comp2, comp1) * 180.0 / M_PI;
      while (theta_toward < 0.0) theta_toward += 360.0;
      while (theta_toward >= 360.0) theta_toward -= 360.0;
      double theta_from = theta_toward + 180.0;
      if (theta_from >= 360.0) theta_from -= 360.0;

      double coh_den = p_eta * (p_n + p_e);
      double vcoh = 0.0;
      if (coh_den > 0.0) {
        double cn2 = atp4_eta_north_bins[i].cross_re*atp4_eta_north_bins[i].cross_re +
                     atp4_eta_north_bins[i].cross_im*atp4_eta_north_bins[i].cross_im;
        double ce2 = atp4_eta_east_bins[i].cross_re*atp4_eta_east_bins[i].cross_re +
                     atp4_eta_east_bins[i].cross_im*atp4_eta_east_bins[i].cross_im;
        vcoh = (cn2 + ce2) / coh_den;
      }
      if (vcoh > 1.0) vcoh = 1.0;

      if (i == peak_i) { peak_from_deg = theta_from; peak_r1 = r1; }

      uint8_t valid = (p_eta >= energy_thresh) && isfinite(comp1) && isfinite(comp2);
      if (valid) {
        double f = (double)(ATP4_K_MIN + i) * (double)WFP_DF_HZ;
        double cw = vcoh < 0.01 ? 0.01 : vcoh;
        double w = p_eta * cw;
        double wc1 = w * comp1, wc2 = w * comp2;
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

    double c1_band = 0.0, c2_band = 0.0;
    if (trap_den > 0.0) { c1_band = trap_num_c1 / trap_den; c2_band = trap_num_c2 / trap_den; }
    double from_band = atan2(c2_band, c1_band) * 180.0 / M_PI + 180.0;
    while (from_band < 0.0) from_band += 360.0;
    while (from_band >= 360.0) from_band -= 360.0;
    double r1_band = sqrt(c1_band*c1_band + c2_band*c2_band);
    double mean_coh = (valid_bins > 0U) ? coh_sum / (double)valid_bins : 0.0;

    double err_deg = from_band - (double)known_from_geo_deg;
    while (err_deg > 180.0) err_deg -= 360.0;
    while (err_deg < -180.0) err_deg += 360.0;

    snprintf(buf, sizeof(buf),
             "\r\nATP4B result:\r\n"
             "  mean coming-from = %.6f deg\r\n"
             "  peak coming-from = %.6f deg\r\n"
             "  known reference  = %.6f deg\r\n"
             "  mean error       = %.6f deg\r\n"
             "  r1 band          = %.6f\r\n"
             "  r1 peak          = %.6f\r\n"
             "  mean coherence   = %.6f\r\n"
             "  valid bins       = %lu\r\n",
             from_band, peak_from_deg,
             (double)known_from_geo_deg, err_deg,
             r1_band, peak_r1, mean_coh,
             (unsigned long)valid_bins);
    uart_print(buf);

    uart_print("ATP4B COMPLETE.\r\n");
  }
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK
                              |RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_VREFINT;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO2;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_SMBUS_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hsmbus2.Instance = I2C2;
  hsmbus2.Init.Timing = 0x00B07CB4;
  hsmbus2.Init.AnalogFilter = SMBUS_ANALOGFILTER_ENABLE;
  hsmbus2.Init.OwnAddress1 = 2;
  hsmbus2.Init.AddressingMode = SMBUS_ADDRESSINGMODE_7BIT;
  hsmbus2.Init.DualAddressMode = SMBUS_DUALADDRESS_DISABLE;
  hsmbus2.Init.OwnAddress2 = 0;
  hsmbus2.Init.OwnAddress2Masks = SMBUS_OA2_NOMASK;
  hsmbus2.Init.GeneralCallMode = SMBUS_GENERALCALL_DISABLE;
  hsmbus2.Init.NoStretchMode = SMBUS_NOSTRETCH_DISABLE;
  hsmbus2.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
  hsmbus2.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_SLAVE;
  hsmbus2.Init.SMBusTimeout = 0x00008186;
  if (HAL_SMBUS_Init(&hsmbus2) != HAL_OK)
  {
    Error_Handler();
  }

  /** configuration Alert Mode
  */
  if (HAL_SMBUS_EnableAlert_IT(&hsmbus2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockB1.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockB1.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockB1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockB1.FrameInit.FrameLength = 8;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB1.SlotInit.SlotNumber = 1;
  hsai_BlockB1.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA2.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA2.Init.PdmInit.Activation = DISABLE;
  hsai_BlockA2.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockA2.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA2.FrameInit.FrameLength = 8;
  hsai_BlockA2.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA2.SlotInit.FirstBitOffset = 0;
  hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA2.SlotInit.SlotNumber = 1;
  hsai_BlockA2.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  hsd1.Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK2, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_ENABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA8 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
