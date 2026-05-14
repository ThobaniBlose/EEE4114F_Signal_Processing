%% STEP 1: Truth-model geometry check for directional processing
% Goal of this step:
% 1) Generate a wave-like vertical displacement eta(t)
% 2) Generate Earth-frame horizontal truth channels aligned with a chosen wave direction
% 3) Rotate those horizontal channels into the buoy/body frame using heading
% 4) Rotate them back into the Earth frame
% 5) Check that the geometry is internally consistent before any spectra are computed

clear; clc; close all;

%% 1) Simulation settings
fs = 100;                  % Sampling frequency [Hz]
Ts = 1/fs;                 % Sampling period [s]
T_total = 30*60;           % Total duration [s] = 30 min
t = (0:Ts:T_total-Ts)';    % Time vector [s]
g = 9.81;   % gravitational acceleration [m/s^2]

%% 2) Sea-state definition (reused from earlier work)
N = 100;                           % Number of wave components
f_min = 0.01;                     % Minimum wave frequency [Hz]
f_max = 0.40;                     % Maximum wave frequency [Hz]
% f_min = 0.05;                     % Minimum wave frequency [Hz]
% f_max = 0.4;                     % Maximum wave frequency [Hz]


f_vec = linspace(f_min, f_max, N)';

f_peak_shape = 0.16;              % Envelope centre [Hz]
sigma_f = 0.06;                   % Envelope spread [Hz]
A_shape = exp(-0.5*((f_vec - f_peak_shape)/sigma_f).^2);

Hm0_target = 0.30;                % Target significant wave height [m]
m0_target = (Hm0_target/4)^2;     % Zeroth spectral moment target [m^2]

% Scale amplitudes so that 0.5*sum(A_i^2) = m0_target
A_vec = sqrt(2*m0_target) * A_shape / norm(A_shape);

rng(10);                          % Repeatable phases
phi_vec = 2*pi*rand(N,1);

%% 3) Wave-direction convention
% IMPORTANT:
% alpha_from_deg = direction the waves are COMING FROM
% measured CLOCKWISE from North
%
% Examples:
% 0 deg   = coming from North
% 90 deg  = coming from East
% 180 deg = coming from South
% 270 deg = coming from West

alpha_from_deg = 30;                    % Chosen incoming wave direction [deg]
alpha_to_deg = mod(alpha_from_deg + 180, 360);   % Direction wave travels TOWARD [deg]

% Convert "toward" direction into Earth-frame unit vector components
% Earth frame = [East, North]
dirE = sind(alpha_to_deg);
dirN = cosd(alpha_to_deg);

%% 4) Build vertical truth channel eta(t)
eta = zeros(size(t));      % Vertical displacement [m]

for i = 1:N
    omega_i = 2*pi*f_vec(i);
    phase_i = omega_i*t + phi_vec(i);

    eta = eta + A_vec(i)*sin(phase_i);
end

%% 4A) Vertical channel sensor simulation

% --- Filter coefficients defined here for vertical channel ---
fc_lo_v = 0.05;
fc_hi_v = 0.50;
[b_bp_v, a_bp_v] = butter(2, [fc_lo_v fc_hi_v]/(fs/2), 'bandpass');

% --- Truth vertical velocity and acceleration ---
eta_vel_true   = [diff(eta); 0] * fs;
eta_accel_true = [diff(eta_vel_true); 0] * fs;

% --- MPU6050 z-axis accelerometer ---
accel_noise_z = 400e-6 * sqrt(fs/2) * g;
accel_bias_z  = 0.012 * g;

rng(202);
accel_noise_z_sig = accel_noise_z * randn(size(t));

az_drift = 0.005 * (t/T_total) + 0.003*sin(2*pi*(1/900)*t);
az_raw = g + eta_accel_true + accel_bias_z + accel_noise_z_sig + az_drift;

% --- Remove gravity and bias with realistic calibration error ---
% In reality calibration is done on land and has residual error
accel_bias_z_estimated = accel_bias_z * 0.85;  % 15% calibration error
az_corrected = az_raw - g - accel_bias_z_estimated;

% --- Band-pass filter ---
az_filt = filtfilt(b_bp_v, a_bp_v, az_corrected);

% Add slow thermal drift — realistic for field deployment
az_drift = 0.005 * (t/T_total) + 0.003*sin(2*pi*(1/900)*t);
az_raw = g + eta_accel_true + accel_bias_z + accel_noise_z_sig + az_drift;

% --- IMU-only vertical velocity (integrate az_filt) ---
vz_imu_raw = cumtrapz(t, az_filt);
fc_hp_v = 0.05;
[b_hp_v, a_hp_v] = butter(2, fc_hp_v/(fs/2), 'high');
vz_imu = filtfilt(b_hp_v, a_hp_v, vz_imu_raw);

% --- IMU-only vertical displacement (integrate vz_imu) ---
eta_imu_raw = cumtrapz(t, vz_imu);
eta_imu = filtfilt(b_hp_v, a_hp_v, eta_imu_raw);

fprintf('\n--- STEP 4A: Vertical IMU channel ---\n');
fprintf('Truth vertical accel RMS:   %.6f m/s^2\n', rms(eta_accel_true));
fprintf('Raw az_raw RMS:             %.6f m/s^2\n', rms(az_raw));
fprintf('Corrected az RMS:           %.6f m/s^2\n', rms(az_corrected));
fprintf('Filtered az RMS:            %.6f m/s^2\n', rms(az_filt));
fprintf('IMU velocity RMS:           %.6f m/s\n',   rms(vz_imu));
fprintf('Truth velocity RMS:         %.6f m/s\n',   rms(eta_vel_true));
fprintf('IMU displacement RMS:       %.6f m\n',     rms(eta_imu));
fprintf('Truth displacement RMS:     %.6f m\n',     rms(eta));
fprintf('IMU/truth velocity ratio:   %.4f\n', rms(vz_imu)/rms(eta_vel_true));
fprintf('IMU/truth displace ratio:   %.4f\n', rms(eta_imu)/rms(eta));

%% 4A) Plots
figure;
subplot(3,1,1);
plot(t(t<=20), eta_accel_true(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), az_filt(t<=20), 'LineWidth', 1.0);
grid on;
ylabel('m/s^2');
title('STEP 4A: Vertical acceleration — truth vs filtered IMU');
legend('Truth', 'IMU filtered', 'Location', 'best');

subplot(3,1,2);
plot(t(t<=20), eta_vel_true(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), vz_imu(t<=20), 'LineWidth', 1.0);
grid on;
ylabel('m/s');
title('STEP 4A: Vertical velocity — truth vs IMU only');
legend('Truth', 'IMU only', 'Location', 'best');

subplot(3,1,3);
plot(t(t<=20), eta(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), eta_imu(t<=20), 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('m');
title('STEP 4A: Vertical displacement — truth vs IMU only');
legend('Truth \eta', 'IMU only', 'Location', 'best');

%% 4B) GNSS vertical velocity simulation

% --- GNSS settings (u-blox typical) ---
fs_gnss  = 10;                    % GNSS update rate [Hz]
Ts_gnss  = 1/fs_gnss;
t_gnss   = (0:Ts_gnss:T_total-Ts_gnss)';

% --- Interpolate truth vertical velocity to GNSS rate ---
eta_vel_gnss_true = interp1(t, eta_vel_true, t_gnss, 'linear');

% --- GNSS vertical velocity noise ---
% u-blox M8 series: ~0.05 m/s RMS vertical velocity noise
gnss_vel_noise_std = 0.10;   % [m/s] AT6558 speed precision

rng(203);
gnss_vel_noise = gnss_vel_noise_std * randn(size(t_gnss));

% --- GNSS vertical velocity measurement ---
vz_gnss_meas = eta_vel_gnss_true + gnss_vel_noise;

% --- Upsample GNSS to full rate by holding last sample ---
vz_gnss_held = interp1(t_gnss, vz_gnss_meas, t, 'previous', 'extrap');

fprintf('\n--- STEP 4B: GNSS vertical velocity simulation ---\n');
fprintf('GNSS sample rate:          %.1f Hz\n', fs_gnss);
fprintf('GNSS velocity noise std:   %.4f m/s\n', gnss_vel_noise_std);
fprintf('Truth velocity RMS:        %.6f m/s\n', rms(eta_vel_true));
fprintf('GNSS velocity RMS:         %.6f m/s\n', rms(vz_gnss_meas));
fprintf('GNSS velocity error RMS:   %.6f m/s\n', rms(vz_gnss_meas - eta_vel_gnss_true));

%% 4B) Plots
figure;
subplot(2,1,1);
plot(t(t<=60), eta_vel_true(t<=60), 'LineWidth', 1.2); hold on;
plot(t(t<=60), vz_imu(t<=60), 'LineWidth', 1.0);
plot(t_gnss(t_gnss<=60), vz_gnss_meas(t_gnss<=60), 'o', ...
     'MarkerSize', 4, 'LineWidth', 1.0);
grid on;
ylabel('m/s');
title('STEP 4B: Vertical velocity — truth vs IMU vs GNSS samples');
legend('Truth', 'IMU only', 'GNSS samples', 'Location', 'best');

subplot(2,1,2);
plot(t(t<=60), eta_vel_true(t<=60), 'LineWidth', 1.2); hold on;
plot(t(t<=60), vz_gnss_held(t<=60), 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('m/s');
title('STEP 4B: GNSS held vertical velocity vs truth');
legend('Truth', 'GNSS held', 'Location', 'best');

%% 4C) Kalman filter — IMU + GNSS vertical velocity fusion

% State:     x = [vz, bias]
%            vz   = vertical velocity estimate
%            bias = IMU accelerometer bias estimate
%
% Propagation (IMU integration):
%            vz(k+1)   = vz(k) + (az_filt(k) - bias(k)) * Ts
%            bias(k+1) = bias(k)   (bias walks slowly)
%
% Measurement (GNSS velocity at 10 Hz):
%            z(k) = vz(k) + noise

% --- Kalman filter tuning ---
% Process noise
q_vz   = (accel_noise_z * Ts)^2 * 0.01;  % trust IMU strongly
q_bias = (1e-7)^2;                         % bias changes very slowly
r_gnss = (gnss_vel_noise_std * 3)^2;      % trust GNSS less
% --- Initialise ---
x_kf    = [0; 0];                      % [vz; bias]
P_kf    = diag([0.01, 1e-6]);          % initial covariance

F = [1, -Ts; 0, 1];                    % state transition
H = [1, 0];                            % measurement matrix
Q = diag([q_vz, q_bias]);              % process noise covariance
R = r_gnss;                            % measurement noise covariance

vz_kf   = zeros(size(t));              % fused velocity output
bias_kf = zeros(size(t));              % estimated bias output

% --- GNSS sample index tracker ---
gnss_sample_times = t_gnss;
gnss_idx = 1;

for k = 1:length(t)

    % --- Predict ---
    x_kf = F * x_kf + [az_filt(k) * Ts; 0];
    P_kf = F * P_kf * F' + Q;

    % --- Update when GNSS sample is available ---
    current_time = t(k);
    if gnss_idx <= length(t_gnss) && ...
       abs(current_time - t_gnss(gnss_idx)) < Ts/2

        z   = vz_gnss_meas(gnss_idx);
        y   = z - H * x_kf;
        S   = H * P_kf * H' + R;
        K   = P_kf * H' / S;
        x_kf = x_kf + K * y;
        P_kf = (eye(2) - K * H) * P_kf;
        gnss_idx = gnss_idx + 1;
    end

    vz_kf(k)   = x_kf(1);
    bias_kf(k) = x_kf(2);
end

% --- Integrate fused velocity to get displacement ---
eta_kf_raw = cumtrapz(t, vz_kf);
[b_hp_v, a_hp_v] = butter(2, fc_hp_v/(fs/2), 'high');
eta_kf = filtfilt(b_hp_v, a_hp_v, eta_kf_raw);

% --- RMSE ---
rmse_imu = sqrt(mean((vz_imu - eta_vel_true).^2));
rmse_kf  = sqrt(mean((vz_kf  - eta_vel_true).^2));
rmse_imu_disp = sqrt(mean((eta_imu - eta).^2));
rmse_kf_disp  = sqrt(mean((eta_kf  - eta).^2));

fprintf('\n--- STEP 4C: Kalman filter IMU+GNSS fusion ---\n');
fprintf('Velocity RMSE  — IMU only:      %.6f m/s\n', rmse_imu);
fprintf('Velocity RMSE  — IMU+GNSS fused: %.6f m/s\n', rmse_kf);
fprintf('Displacement RMSE — IMU only:    %.6f m\n', rmse_imu_disp);
fprintf('Displacement RMSE — IMU+GNSS:    %.6f m\n', rmse_kf_disp);
fprintf('Velocity improvement:            %.2f%%\n', ...
        (rmse_imu - rmse_kf)/rmse_imu * 100);
fprintf('Displacement improvement:        %.2f%%\n', ...
        (rmse_imu_disp - rmse_kf_disp)/rmse_imu_disp * 100);
fprintf('Estimated bias (end):            %.6f m/s^2\n', bias_kf(end));

%% 4C) Plots — the comparison Robyn asked for
figure('Position', [100 100 900 700]);

subplot(3,1,1);
plot(t(t<=60), eta_vel_true(t<=60), 'k', 'LineWidth', 1.4); hold on;
plot(t(t<=60), vz_imu(t<=60), 'b', 'LineWidth', 1.0);
plot(t(t<=60), vz_kf(t<=60), 'r', 'LineWidth', 1.0);
grid on;
ylabel('Vertical velocity [m/s]');
title('STEP 4C: Vertical velocity — truth vs IMU only vs IMU+GNSS fused');
legend('Truth', ...
       sprintf('IMU only  (RMSE=%.4f m/s)', rmse_imu), ...
       sprintf('IMU+GNSS  (RMSE=%.4f m/s)', rmse_kf), ...
       'Location', 'best');

subplot(3,1,2);
plot(t(t<=60), eta(t<=60), 'k', 'LineWidth', 1.4); hold on;
plot(t(t<=60), eta_imu(t<=60), 'b', 'LineWidth', 1.0);
plot(t(t<=60), eta_kf(t<=60), 'r', 'LineWidth', 1.0);
grid on;
ylabel('Vertical displacement [m]');
title('STEP 4C: Vertical displacement — truth vs IMU only vs IMU+GNSS fused');
legend('Truth \eta', ...
       sprintf('IMU only  (RMSE=%.4f m)', rmse_imu_disp), ...
       sprintf('IMU+GNSS  (RMSE=%.4f m)', rmse_kf_disp), ...
       'Location', 'best');

subplot(3,1,3);
plot(t(t<=60), bias_kf(t<=60), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Estimated bias [m/s^2]');
title('STEP 4C: Kalman filter estimated accelerometer bias');
%% 5) Build a clean Earth-frame horizontal truth signal
% For this FIRST step, I am not yet modelling the full buoy physics.
% I am only building a wave-like horizontal truth channel aligned with the chosen
% propagation direction so that I can verify the frame-rotation logic.
%
% I use a velocity-like horizontal magnitude:
% h_dir_true(t) = sum gamma_h * omega_i * A_i * cos(phase_i)
%
% This gives me a smooth wave-like horizontal signal that is tied to the same
% wave components as eta(t).

gamma_h = 1.0;             % Horizontal scaling factor for this geometry check
h_dir_true = zeros(size(t));

for i = 1:N
    omega_i = 2*pi*f_vec(i);
    phase_i = omega_i*t + phi_vec(i);

    h_dir_true = h_dir_true + gamma_h * omega_i * A_vec(i) * cos(phase_i);
end

% Project that horizontal signal into Earth-frame East/North channels
uE_true = h_dir_true * dirE;     % East-frame horizontal truth channel
vN_true = h_dir_true * dirN;     % North-frame horizontal truth channel

%% 6) Define buoy heading
% psi_true_deg = buoy heading, measured CLOCKWISE from North
% This is only for the frame-rotation check.
%
% I let it vary slowly so that I test the rotation over time.

psi_true_deg = 20 + 5*sin(2*pi*(1/300)*t);   % [deg]
psi_true = deg2rad(psi_true_deg);            % [rad]

%% 6A) Truth roll and pitch angles
% Small wave-induced tilt oscillations
% phi   = roll  [rotation about forward axis]
% theta = pitch [rotation about side axis]

phi_true_deg   = 3.0 * sin(2*pi*0.14*t + 0.8);
theta_true_deg = 2.0 * sin(2*pi*0.17*t + 1.3);

phi_true   = deg2rad(phi_true_deg);
theta_true = deg2rad(theta_true_deg);

fprintf('\n--- STEP 6A: Truth attitude ---\n');
fprintf('Roll  amplitude: %.2f deg\n', max(abs(phi_true_deg)));
fprintf('Pitch amplitude: %.2f deg\n', max(abs(theta_true_deg)));
fprintf('Roll  std: %.6f deg\n', std(phi_true_deg));
fprintf('Pitch std: %.6f deg\n', std(theta_true_deg));

%% 6B) Simulated MPU6050 raw sensor outputs
% Accelerometer measures:
%   1) gravity projected onto tilted axes
%   2) wave-induced horizontal acceleration (not yet available, added after 7)
%   3) sensor bias
%   4) broadband noise
%
% Gyroscope measures:
%   1) true angular rates (derivative of attitude angles)
%   2) sensor bias
%   3) broadband noise

g = 9.81;   % gravitational acceleration [m/s^2]

% --- Gravity projection onto body axes from truth tilt ---
grav_x = -g * sin(theta_true);   % pitch effect on x-axis [m/s^2]
grav_y =  g * sin(phi_true);     % roll  effect on y-axis [m/s^2]

% --- MPU6050 accelerometer bias ---
accel_bias_x =  0.015 * g;   % [m/s^2]
accel_bias_y = -0.010 * g;   % [m/s^2]

% --- MPU6050 accelerometer noise ---
% 400 uG/rtHz noise density at fs=100 Hz
accel_noise_std = 400e-6 * sqrt(fs/2) * g;   % [m/s^2]

rng(200);
accel_noise_x = accel_noise_std * randn(size(t));
accel_noise_y = accel_noise_std * randn(size(t));

% --- Truth angular rates from attitude angles ---
dphi_dt   = [diff(phi_true);   0] * fs;
dtheta_dt = [diff(theta_true); 0] * fs;
dpsi_dt   = [diff(psi_true);   0] * fs;

gx_true_dps = rad2deg(dphi_dt);
gy_true_dps = rad2deg(dtheta_dt);
gz_true_dps = rad2deg(dpsi_dt);

% --- MPU6050 gyroscope bias ---
gyro_bias_x =  0.05;    % [deg/s] post-calibration residual
gyro_bias_y = -0.04;    % [deg/s] post-calibration residual
gyro_bias_z =  0.03;    % [deg/s] post-calibration residual
% --- MPU6050 gyroscope noise ---
% 0.005 deg/s/rtHz at fs=100 Hz
gyro_noise_std = 0.005 * sqrt(fs/2);   % [deg/s]

rng(201);
gyro_noise_x = gyro_noise_std * randn(size(t));
gyro_noise_y = gyro_noise_std * randn(size(t));
gyro_noise_z = gyro_noise_std * randn(size(t));

% --- Total gyroscope output ---
gx_meas_dps = gx_true_dps + gyro_bias_x + gyro_noise_x;
gy_meas_dps = gy_true_dps + gyro_bias_y + gyro_noise_y;
gz_meas_dps = gz_true_dps + gyro_bias_z + gyro_noise_z;

fprintf('\n--- STEP 6B: MPU6050 raw sensor outputs ---\n');
fprintf('Accel noise std: %.6f m/s^2 (%.2f mg)\n', accel_noise_std, accel_noise_std/g*1000);
fprintf('Gyro  noise std: %.6f deg/s\n', gyro_noise_std);
fprintf('Gravity on x: max=%.4f  rms=%.4f m/s^2\n', max(abs(grav_x)), rms(grav_x));
fprintf('Gravity on y: max=%.4f  rms=%.4f m/s^2\n', max(abs(grav_y)), rms(grav_y));
fprintf('gx truth rate: max=%.4f  rms=%.4f deg/s\n', max(abs(gx_true_dps)), rms(gx_true_dps));
fprintf('gy truth rate: max=%.4f  rms=%.4f deg/s\n', max(abs(gy_true_dps)), rms(gy_true_dps));
fprintf('gz truth rate: max=%.4f  rms=%.4f deg/s\n', max(abs(gz_true_dps)), rms(gz_true_dps));
fprintf('gx measured:   max=%.4f  rms=%.4f deg/s\n', max(abs(gx_meas_dps)), rms(gx_meas_dps));
fprintf('gy measured:   max=%.4f  rms=%.4f deg/s\n', max(abs(gy_meas_dps)), rms(gy_meas_dps));
fprintf('gz measured:   max=%.4f  rms=%.4f deg/s\n', max(abs(gz_meas_dps)), rms(gz_meas_dps));

%% 6B) Plots

% Plot 1: gravity leakage on both axes
figure;
subplot(2,1,1);
plot(t(t<=20), grav_x(t<=20), 'LineWidth', 1.2);
grid on;
ylabel('m/s^2');
title('STEP 6B: Gravity leakage on body x-axis (pitch effect)');

subplot(2,1,2);
plot(t(t<=20), grav_y(t<=20), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('m/s^2');
title('STEP 6B: Gravity leakage on body y-axis (roll effect)');

% Plot 2: gyroscope truth vs measured
figure;
subplot(3,1,1);
plot(t(t<=20), gx_true_dps(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), gx_meas_dps(t<=20), 'LineWidth', 1.0);
grid on;
ylabel('deg/s');
title('STEP 6B: Roll rate — truth vs measured');
legend('Truth', 'Measured', 'Location', 'best');

subplot(3,1,2);
plot(t(t<=20), gy_true_dps(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), gy_meas_dps(t<=20), 'LineWidth', 1.0);
grid on;
ylabel('deg/s');
title('STEP 6B: Pitch rate — truth vs measured');
legend('Truth', 'Measured', 'Location', 'best');

subplot(3,1,3);
plot(t(t<=20), gz_true_dps(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), gz_meas_dps(t<=20), 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('deg/s');
title('STEP 6B: Yaw rate — truth vs measured');
legend('Truth', 'Measured', 'Location', 'best');

%% 7) Rotate Earth-frame horizontal truth into body frame
% Body-frame convention:
% x_b = buoy-forward axis
% y_b = buoy-right axis
%
% With heading psi measured clockwise from North:
% e_xb in Earth = [sin(psi), cos(psi)]
% e_yb in Earth = [cos(psi), -sin(psi)]

x_b = uE_true .* sin(psi_true) + vN_true .* cos(psi_true);
y_b = uE_true .* cos(psi_true) - vN_true .* sin(psi_true);

%% 7A) Complete raw accelerometer output and gravity removal

% --- Wave-induced horizontal acceleration from body-frame velocity signals ---
ax_wave = [diff(x_b); 0] * fs;   % [m/s^2]
ay_wave = [diff(y_b); 0] * fs;   % [m/s^2]

% --- Total raw accelerometer output ---
% Now we can assemble the full signal:
% gravity + wave acceleration + bias + noise
ax_raw = ax_wave + grav_x + accel_bias_x + accel_noise_x;
ay_raw = ay_wave + grav_y + accel_bias_y + accel_noise_y;

%% 6C) Complementary filter — using truth gravity for tilt correction

alpha_cf = 0.98;

% Preallocate
phi_est    = zeros(size(t));
theta_est  = zeros(size(t));
psi_est_cf = zeros(size(t));

% Initialise from truth gravity projection
phi_est(1)    = atan2(grav_y(1), g);
theta_est(1)  = atan2(-grav_x(1), g);
psi_est_cf(1) = psi_true(1);

for k = 2:length(t)

    % Gyroscope integration
    phi_gyro   = phi_est(k-1)    + deg2rad(gx_meas_dps(k-1)) * Ts;
    theta_gyro = theta_est(k-1)  + deg2rad(gy_meas_dps(k-1)) * Ts;
    psi_gyro   = psi_est_cf(k-1) + deg2rad(gz_meas_dps(k-1)) * Ts;

    % Tilt correction from truth gravity only
    phi_accel   = atan2(grav_y(k), g);
    theta_accel = atan2(-grav_x(k), g);

    % Magnetometer heading
    psi_mag = psi_true(k);

    % Fusion
    phi_est(k)    = alpha_cf * phi_gyro   + (1-alpha_cf) * phi_accel;
    theta_est(k)  = alpha_cf * theta_gyro + (1-alpha_cf) * theta_accel;
    psi_est_cf(k) = alpha_cf * psi_gyro   + (1-alpha_cf) * psi_mag;

end

phi_est_deg    = rad2deg(phi_est);
theta_est_deg  = rad2deg(theta_est);
psi_est_cf_deg = rad2deg(psi_est_cf);

phi_err_deg    = phi_est_deg   - phi_true_deg;
theta_err_deg  = theta_est_deg - theta_true_deg;
psi_cf_err_deg = mod(psi_est_cf_deg - psi_true_deg + 180, 360) - 180;

fprintf('\n--- STEP 6C: Complementary filter ---\n');
fprintf('Alpha: %.2f\n', alpha_cf);
fprintf('Roll  error: mean=%.4f  std=%.4f  max=%.4f deg\n', ...
        mean(phi_err_deg), std(phi_err_deg), max(abs(phi_err_deg)));
fprintf('Pitch error: mean=%.4f  std=%.4f  max=%.4f deg\n', ...
        mean(theta_err_deg), std(theta_err_deg), max(abs(theta_err_deg)));
fprintf('Yaw   error: mean=%.4f  std=%.4f  max=%.4f deg\n', ...
        mean(psi_cf_err_deg), std(psi_cf_err_deg), max(abs(psi_cf_err_deg)));

%% 6C) Plots
figure;
subplot(3,1,1);
plot(t(t<=60), phi_true_deg(t<=60), 'LineWidth', 1.2); hold on;
plot(t(t<=60), phi_est_deg(t<=60), '--', 'LineWidth', 1.0);
grid on;
ylabel('Roll [deg]');
title('STEP 6C: Roll estimate vs truth');
legend('Truth', 'Estimated', 'Location', 'best');

subplot(3,1,2);
plot(t(t<=60), theta_true_deg(t<=60), 'LineWidth', 1.2); hold on;
plot(t(t<=60), theta_est_deg(t<=60), '--', 'LineWidth', 1.0);
grid on;
ylabel('Pitch [deg]');
title('STEP 6C: Pitch estimate vs truth');
legend('Truth', 'Estimated', 'Location', 'best');

subplot(3,1,3);
plot(t(t<=60), psi_true_deg(t<=60), 'LineWidth', 1.2); hold on;
plot(t(t<=60), psi_est_cf_deg(t<=60), '--', 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('Yaw [deg]');
title('STEP 6C: Yaw estimate vs truth');
legend('Truth', 'Estimated', 'Location', 'best');

%% 7A) Gravity removal using truth gravity
grav_x_est = grav_x;
grav_y_est = grav_y;

ax_corrected = ax_raw - grav_x_est;
ay_corrected = ay_raw - grav_y_est;

grav_x_residual = grav_x_est - grav_x;
grav_y_residual = grav_y_est - grav_y;

fprintf('\n--- STEP 7A: Raw accelerometer and gravity removal ---\n');
fprintf('Wave accel x: max=%.4f  rms=%.4f m/s^2\n', max(abs(ax_wave)), rms(ax_wave));
fprintf('Wave accel y: max=%.4f  rms=%.4f m/s^2\n', max(abs(ay_wave)), rms(ay_wave));
fprintf('Raw ax_raw:   max=%.4f  rms=%.4f m/s^2\n', max(abs(ax_raw)), rms(ax_raw));
fprintf('Raw ay_raw:   max=%.4f  rms=%.4f m/s^2\n', max(abs(ay_raw)), rms(ay_raw));
fprintf('Gravity residual x: max=%.4f  rms=%.4f m/s^2\n', max(abs(grav_x_residual)), rms(grav_x_residual));
fprintf('Gravity residual y: max=%.4f  rms=%.4f m/s^2\n', max(abs(grav_y_residual)), rms(grav_y_residual));
fprintf('Corrected ax: max=%.4f  rms=%.4f m/s^2\n', max(abs(ax_corrected)), rms(ax_corrected));
fprintf('Corrected ay: max=%.4f  rms=%.4f m/s^2\n', max(abs(ay_corrected)), rms(ay_corrected));
%% 7B) Bias removal and filtering of corrected accelerometer channels

% --- Remove accelerometer bias (known from calibration) ---
ax_debiased = ax_corrected - accel_bias_x;
ay_debiased = ay_corrected - accel_bias_y;

% --- Band-pass filter to keep only wave band (0.05 to 0.50 Hz) ---
fc_lo = 0.05;
fc_hi = 0.50;
[b_bp, a_bp] = butter(2, [fc_lo fc_hi]/(fs/2), 'bandpass');

ax_filt = filtfilt(b_bp, a_bp, ax_debiased);
ay_filt = filtfilt(b_bp, a_bp, ay_debiased);

fprintf('\n--- STEP 7B: Bias removal and filtering ---\n');
fprintf('ax debiased: max=%.4f  rms=%.4f m/s^2\n', max(abs(ax_debiased)), rms(ax_debiased));
fprintf('ay debiased: max=%.4f  rms=%.4f m/s^2\n', max(abs(ay_debiased)), rms(ay_debiased));
fprintf('ax filtered: max=%.4f  rms=%.4f m/s^2\n', max(abs(ax_filt)), rms(ax_filt));
fprintf('ay filtered: max=%.4f  rms=%.4f m/s^2\n', max(abs(ay_filt)), rms(ay_filt));
fprintf('ax wave truth rms: %.4f m/s^2\n', rms(ax_wave));
fprintf('ay wave truth rms: %.4f m/s^2\n', rms(ay_wave));
fprintf('ax ratio filtered/truth: %.4f\n', rms(ax_filt)/rms(ax_wave));
fprintf('ay ratio filtered/truth: %.4f\n', rms(ay_filt)/rms(ay_wave));
%% 7B) Plots
figure;
subplot(3,1,1);
plot(t(t<=20), ax_wave(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), ax_debiased(t<=20), 'LineWidth', 1.0);
grid on;
ylabel('m/s^2');
title('STEP 7B: x-axis — wave truth vs debiased');
legend('Wave truth', 'Debiased', 'Location', 'best');

subplot(3,1,2);
plot(t(t<=20), ax_wave(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), ax_filt(t<=20), 'LineWidth', 1.0);
grid on;
ylabel('m/s^2');
title('STEP 7B: x-axis — wave truth vs filtered');
legend('Wave truth', 'Filtered', 'Location', 'best');

subplot(3,1,3);
plot(t(t<=20), ay_wave(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), ay_filt(t<=20), 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('m/s^2');
title('STEP 7B: y-axis — wave truth vs filtered');
legend('Wave truth', 'Filtered', 'Location', 'best');
%% 7A) Plots
figure;
subplot(3,1,1);
plot(t(t<=20), ax_raw(t<=20), 'LineWidth', 1.0); hold on;
plot(t(t<=20), ax_wave(t<=20), 'LineWidth', 1.2);
plot(t(t<=20), grav_x(t<=20), 'LineWidth', 1.2);
grid on;
ylabel('m/s^2');
title('STEP 7A: x-axis raw accelerometer breakdown');
legend('Raw total', 'Wave accel', 'Gravity leakage', 'Location', 'best');

subplot(3,1,2);
plot(t(t<=20), ax_raw(t<=20), 'LineWidth', 1.0); hold on;
plot(t(t<=20), ax_corrected(t<=20), 'LineWidth', 1.2);
plot(t(t<=20), ax_wave(t<=20), '--', 'LineWidth', 1.2);
grid on;
ylabel('m/s^2');
title('STEP 7A: x-axis before and after gravity removal');
legend('Raw', 'Corrected', 'Wave truth', 'Location', 'best');

subplot(3,1,3);
plot(t(t<=20), ay_raw(t<=20), 'LineWidth', 1.0); hold on;
plot(t(t<=20), ay_corrected(t<=20), 'LineWidth', 1.2);
plot(t(t<=20), ay_wave(t<=20), '--', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('m/s^2');
title('STEP 7A: y-axis before and after gravity removal');
legend('Raw', 'Corrected', 'Wave truth', 'Location', 'best');

%% 7C) Integrate filtered acceleration to get velocity-like channels

% Use cumulative trapezoidal integration in time domain
% Then high-pass filter to remove integration drift

vx_int_raw = cumtrapz(t, ax_filt);
vy_int_raw = cumtrapz(t, ay_filt);

% High-pass filter to remove drift from integration
% Use same low cutoff as wave band
fc_hp = 0.05;
[b_hp, a_hp] = butter(2, fc_hp/(fs/2), 'high');

vx_int = filtfilt(b_hp, a_hp, vx_int_raw);
vy_int = filtfilt(b_hp, a_hp, vy_int_raw);

fprintf('\n--- STEP 7C: Time-domain integration ---\n');
fprintf('vx_int rms: %.6f m/s\n', rms(vx_int));
fprintf('vy_int rms: %.6f m/s\n', rms(vy_int));
fprintf('x_b rms (truth velocity): %.6f m/s\n', rms(x_b));
fprintf('y_b rms (truth velocity): %.6f m/s\n', rms(y_b));
fprintf('vx ratio integrated/truth: %.4f\n', rms(vx_int)/rms(x_b));
fprintf('vy ratio integrated/truth: %.4f\n', rms(vy_int)/rms(y_b));
%% 8) Recover Earth-frame channels from body frame
uE_rec = x_b .* sin(psi_true) + y_b .* cos(psi_true);
vN_rec = x_b .* cos(psi_true) - y_b .* sin(psi_true);

%% 9) Recovery errors
uE_err = uE_rec - uE_true;
vN_err = vN_rec - vN_true;

fprintf('\n--- STEP 1: Geometry check ---\n');
fprintf('Chosen incoming wave direction (coming from): %.2f deg clockwise from North\n', alpha_from_deg);
fprintf('Wave propagation direction (toward): %.2f deg clockwise from North\n', alpha_to_deg);
fprintf('Earth-frame direction vector [East, North] = [%.4f, %.4f]\n', dirE, dirN);
fprintf('Max abs East-channel recovery error:  %.6e\n', max(abs(uE_err)));
fprintf('Max abs North-channel recovery error: %.6e\n', max(abs(vN_err)));

%% 10) Plots over a short window
t_plot = 0:Ts:20;
idx = t <= 20;

figure;
plot(t(idx), eta(idx), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('\eta(t) [m]');
title('STEP 1: Vertical truth signal');

figure;
plot(t(idx), uE_true(idx), 'LineWidth', 1.2); hold on;
plot(t(idx), vN_true(idx), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Horizontal truth signal');
title('STEP 1: Earth-frame horizontal truth channels');
legend('u_E true', 'v_N true', 'Location', 'best');

figure;
plot(t(idx), x_b(idx), 'LineWidth', 1.2); hold on;
plot(t(idx), y_b(idx), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Body-frame horizontal signal');
title('STEP 1: Body-frame horizontal channels');
legend('x_b', 'y_b', 'Location', 'best');

figure;
plot(t(idx), uE_true(idx), 'LineWidth', 1.2); hold on;
plot(t(idx), uE_rec(idx), '--', 'LineWidth', 1.2);
plot(t(idx), vN_true(idx), 'LineWidth', 1.2);
plot(t(idx), vN_rec(idx), '--', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Recovered vs true');
title('STEP 1: Earth-frame recovery check');
legend('u_E true', 'u_E recovered', 'v_N true', 'v_N recovered', 'Location', 'best');

%% STEP 2: Add realistic measured channels (still no directional spectrum yet)
% Goal of this step:
% 1) Keep the STEP 1 truth channels
% 2) Create noisy / biased body-frame horizontal measurement channels
% 3) Create a realistic sampled magnetometer heading measurement
% 4) Rotate the noisy body-frame channels back into the Earth frame
% 5) Check how well the Earth-frame channels are recovered

%% 2A) Noisy / biased body-frame horizontal measurement channels
% For this step, I treat x_b and y_b as the horizontal IMU-type measurement channels.
% I am not yet forcing the exact full MPU accelerometer physics here.
% I only add realistic bias, slow drift, and random measurement noise.

rng(100);

bx_bias = 0.003;                              % constant bias on body x channel
by_bias = -0.002;                             % constant bias on body y channel

bx_drift = 0.0015*sin(2*pi*(1/900)*t);        % slow drift
by_drift = -0.0010*sin(2*pi*(1/750)*t);

bx_noise_std = 0.0020;                        % random noise std
by_noise_std = 0.0020;

x_b_meas = x_b + bx_bias + bx_drift + bx_noise_std*randn(size(t));
y_b_meas = y_b + by_bias + by_drift + by_noise_std*randn(size(t));

%% 2B) Sampled magnetometer heading model (HMC5883L-style)
% HMC5883L can run much faster, but for this first realistic model
% I choose a moderate magnetometer output rate.
%
% I keep:
% - a constant heading bias
% - a slow heading drift
% - random heading noise
%
% Later we can tune these values once we decide how aggressive we want the model to be.

fs_mag = 75;                         % magnetometer sample rate [Hz]
Ts_mag = 1/fs_mag;
t_mag = (0:Ts_mag:T_total-Ts_mag)';

psi_true_deg_mag = interp1(t, psi_true_deg, t_mag, 'linear');

mag_bias_deg = 1.0;                                  % constant heading bias [deg]
mag_drift_deg = 0.3*sin(2*pi*(1/600)*t_mag);         % slow heading drift [deg]
mag_noise_std_deg = 0.5;                             % heading noise std [deg]

rng(101);
psi_mag_meas_deg = psi_true_deg_mag + mag_bias_deg + ...
                   mag_drift_deg + mag_noise_std_deg*randn(size(t_mag));

% Make a full-rate heading estimate by holding the latest magnetometer sample
psi_est_deg = interp1(t_mag, psi_mag_meas_deg, t, 'previous', 'extrap');
psi_est = deg2rad(psi_est_deg);

%% 2C) Recover Earth-frame channels from noisy body-frame channels
uE_est_noisy = x_b_meas .* sin(psi_est) + y_b_meas .* cos(psi_est);
vN_est_noisy = x_b_meas .* cos(psi_est) - y_b_meas .* sin(psi_est);

uE_est_real = vx_int .* sin(psi_est_cf) + vy_int .* cos(psi_est_cf);
vN_est_real = vx_int .* cos(psi_est_cf) - vy_int .* sin(psi_est_cf);

%% 2D) Errors
uE_err_noisy = uE_est_noisy - uE_true;
vN_err_noisy = vN_est_noisy - vN_true;

uE_rmse = sqrt(mean(uE_err_noisy.^2));
vN_rmse = sqrt(mean(vN_err_noisy.^2));

psi_est_err_deg = psi_est_deg - psi_true_deg;
psi_est_err_deg = mod(psi_est_err_deg + 180, 360) - 180;

fprintf('\n--- STEP 2: Realistic measurement stage ---\n');
fprintf('Magnetometer sample rate: %.1f Hz\n', fs_mag);
fprintf('Injected heading bias: %.2f deg\n', mag_bias_deg);
fprintf('Heading error mean: %.2f deg\n', mean(psi_est_err_deg));
fprintf('Heading error std:  %.2f deg\n', std(psi_est_err_deg));
fprintf('Heading max abs error: %.2f deg\n', max(abs(psi_est_err_deg)));
fprintf('Recovered East-channel RMSE:  %.6e\n', uE_rmse);
fprintf('Recovered North-channel RMSE: %.6e\n', vN_rmse);

%% 2E) Plots

% Plot 1: body-frame truth vs measured channels
figure;
subplot(2,1,1);
plot(t(t<=20), x_b(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), x_b_meas(t<=20), 'LineWidth', 1.0);
grid on;
ylabel('x_b');
title('STEP 2: Body-frame x channel - truth vs measured');
legend('x_b truth', 'x_b measured', 'Location', 'best');

subplot(2,1,2);
plot(t(t<=20), y_b(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), y_b_meas(t<=20), 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('y_b');
title('STEP 2: Body-frame y channel - truth vs measured');
legend('y_b truth', 'y_b measured', 'Location', 'best');

% Plot 2: heading truth vs sampled magnetometer heading
figure;
plot(t(t<=120), psi_true_deg(t<=120), 'LineWidth', 1.2); hold on;
plot(t_mag(t_mag<=120), psi_mag_meas_deg(t_mag<=120), 'o', 'LineWidth', 1.0);
plot(t(t<=120), psi_est_deg(t<=120), '--', 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('Heading [deg]');
title('STEP 2: True vs sampled magnetometer heading');
legend('True heading', 'Mag samples', 'Held heading estimate', 'Location', 'best');

% Plot 3: Earth-frame truth vs recovered channels
figure;
subplot(2,1,1);
plot(t(t<=20), uE_true(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), uE_est_noisy(t<=20), 'LineWidth', 1.0);
grid on;
ylabel('u_E');
title('STEP 2: East-channel recovery with realistic measurement effects');
legend('u_E true', 'u_E recovered', 'Location', 'best');

subplot(2,1,2);
plot(t(t<=20), vN_true(t<=20), 'LineWidth', 1.2); hold on;
plot(t(t<=20), vN_est_noisy(t<=20), 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('v_N');
title('STEP 2: North-channel recovery with realistic measurement effects');
legend('v_N true', 'v_N recovered', 'Location', 'best');

%% STEP 3: First spectra and cross-spectra of recovered horizontal channels
% Goal of this step:
% 1) Compare the spectra of true vs recovered Earth-frame horizontal channels
% 2) Check whether the main wave band is preserved after realistic measurement effects
% 3) Inspect the cross-spectrum and coherence between recovered u_E and v_N
% 4) Decide whether the recovered channels are still usable for directional DSP

%% 3A) Spectral settings
win_len = 4096;
noverlap = win_len/2;
nfft = 8192;

f_min_wave = 0.08;
f_max_wave = 0.40;
idx_wave = [];

%% 3B) Detrend signals before spectral analysis
uE_true_psd = detrend(uE_true, 'constant');
vN_true_psd = detrend(vN_true, 'constant');

uE_rec_psd = detrend(uE_est_noisy, 'constant');
vN_rec_psd = detrend(vN_est_noisy, 'constant');

uE_real_psd = detrend(uE_est_real, 'constant');
vN_real_psd = detrend(vN_est_real, 'constant');

%% 3C) Auto-spectra
[Suu_true, f_spec] = pwelch(uE_true_psd, win_len, noverlap, nfft, fs);
[Svv_true, ~]      = pwelch(vN_true_psd, win_len, noverlap, nfft, fs);

[Suu_rec, ~] = pwelch(uE_rec_psd, win_len, noverlap, nfft, fs);
[Svv_rec, ~] = pwelch(vN_rec_psd, win_len, noverlap, nfft, fs);

[Suu_real, ~] = pwelch(uE_real_psd, win_len, noverlap, nfft, fs);
[Svv_real, ~] = pwelch(vN_real_psd, win_len, noverlap, nfft, fs);

idx_wave = (f_spec >= f_min_wave) & (f_spec <= f_max_wave);

%% 3D) Cross-spectrum and coherence of recovered channels
[Suv_rec, ~] = cpsd(uE_rec_psd, vN_rec_psd, win_len, noverlap, nfft, fs);
coh_uv_rec = mscohere(uE_rec_psd, vN_rec_psd, win_len, noverlap, nfft, fs);

Cuv_rec = real(Suv_rec);      % co-spectrum
Quv_rec = imag(Suv_rec);      % quad-spectrum
Suv_mag = abs(Suv_rec);       % cross-spectrum magnitude
Suv_phase_deg = rad2deg(angle(Suv_rec));

%% 3E) Spectral diagnostics
[~, idx_Suu_true_peak_local] = max(Suu_true(idx_wave));
[~, idx_Svv_true_peak_local] = max(Svv_true(idx_wave));
[~, idx_Suu_rec_peak_local]  = max(Suu_rec(idx_wave));
[~, idx_Svv_rec_peak_local]  = max(Svv_rec(idx_wave));

f_wave = f_spec(idx_wave);

f_Suu_true_peak = f_wave(idx_Suu_true_peak_local);
f_Svv_true_peak = f_wave(idx_Svv_true_peak_local);
f_Suu_rec_peak  = f_wave(idx_Suu_rec_peak_local);
f_Svv_rec_peak  = f_wave(idx_Svv_rec_peak_local);

% Relative spectral errors in the wave band
Suu_rel_err_wave = norm(Suu_rec(idx_wave) - Suu_true(idx_wave)) / norm(Suu_true(idx_wave));
Svv_rel_err_wave = norm(Svv_rec(idx_wave) - Svv_true(idx_wave)) / norm(Svv_true(idx_wave));

% Coherence summary over the wave band
coh_uv_wave_mean = mean(coh_uv_rec(idx_wave));
coh_uv_wave_max  = max(coh_uv_rec(idx_wave));

fprintf('\n--- STEP 3: Spectral check of recovered horizontal channels ---\n');
fprintf('Wave band analysed: %.2f to %.2f Hz\n', f_min_wave, f_max_wave);
fprintf('u_E true peak frequency: %.6f Hz\n', f_Suu_true_peak);
fprintf('u_E recovered peak frequency: %.6f Hz\n', f_Suu_rec_peak);
fprintf('v_N true peak frequency: %.6f Hz\n', f_Svv_true_peak);
fprintf('v_N recovered peak frequency: %.6f Hz\n', f_Svv_rec_peak);
fprintf('u_E relative spectral error over wave band: %.6e\n', Suu_rel_err_wave);
fprintf('v_N relative spectral error over wave band: %.6e\n', Svv_rel_err_wave);
fprintf('Mean u_E-v_N coherence over wave band: %.6f\n', coh_uv_wave_mean);
fprintf('Max u_E-v_N coherence over wave band: %.6f\n', coh_uv_wave_max);

Suu_real_rel_err = norm(Suu_real(idx_wave) - Suu_true(idx_wave)) / norm(Suu_true(idx_wave));
Svv_real_rel_err = norm(Svv_real(idx_wave) - Svv_true(idx_wave)) / norm(Svv_true(idx_wave));

fprintf('\n--- STEP 3 REAL channels spectral check ---\n');
fprintf('u_E real relative spectral error: %.6e\n', Suu_real_rel_err);
fprintf('v_N real relative spectral error: %.6e\n', Svv_real_rel_err);

[coh_uv_real, ~] = mscohere(uE_real_psd, vN_real_psd, win_len, noverlap, nfft, fs);

Suu_real_rel_err = norm(Suu_real(idx_wave) - Suu_true(idx_wave)) / norm(Suu_true(idx_wave));
Svv_real_rel_err = norm(Svv_real(idx_wave) - Svv_true(idx_wave)) / norm(Svv_true(idx_wave));

fprintf('\n--- STEP 3 REAL channels spectral check ---\n');
fprintf('u_E real relative spectral error: %.6e\n', Suu_real_rel_err);
fprintf('v_N real relative spectral error: %.6e\n', Svv_real_rel_err);
fprintf('Mean u_E-v_N real coherence over wave band: %.6f\n', mean(coh_uv_real(idx_wave)));
fprintf('Max  u_E-v_N real coherence over wave band: %.6f\n', max(coh_uv_real(idx_wave)));
%% 3F) Plots

% Plot 1: auto-spectra comparison
figure;
subplot(2,1,1);
plot(f_spec, Suu_true, 'LineWidth', 1.2); hold on;
plot(f_spec, Suu_rec,  'LineWidth', 1.0);
grid on;
xlim([0 0.5]);
ylabel('PSD');
title('STEP 3: East-channel spectrum');
legend('u_E true', 'u_E recovered', 'Location', 'best');

subplot(2,1,2);
plot(f_spec, Svv_true, 'LineWidth', 1.2); hold on;
plot(f_spec, Svv_rec,  'LineWidth', 1.0);
grid on;
xlim([0 0.5]);
xlabel('Frequency [Hz]');
ylabel('PSD');
title('STEP 3: North-channel spectrum');
legend('v_N true', 'v_N recovered', 'Location', 'best');

% Plot 2: recovered cross-spectrum parts
figure;
subplot(3,1,1);
plot(f_spec, Cuv_rec, 'LineWidth', 1.2);
grid on;
xlim([0 0.5]);
ylabel('Co-spec');
title('STEP 3: Recovered cross-spectrum between u_E and v_N');

subplot(3,1,2);
plot(f_spec, Quv_rec, 'LineWidth', 1.2);
grid on;
xlim([0 0.5]);
ylabel('Quad-spec');

subplot(3,1,3);
plot(f_spec, Suv_mag, 'LineWidth', 1.2);
grid on;
xlim([0 0.5]);
xlabel('Frequency [Hz]');
ylabel('|S_{uv}|');

% Plot 3: coherence
figure;
plot(f_spec, coh_uv_rec, 'LineWidth', 1.2); hold on;
xline(f_min_wave, '--', 'LineWidth', 1.0);
xline(f_max_wave, '--', 'LineWidth', 1.0);
grid on;
xlim([0 0.5]);
ylim([0 1.05]);
xlabel('Frequency [Hz]');
ylabel('Coherence');
title('STEP 3: Coherence between recovered u_E and v_N');

% Plot 4: cross-spectrum phase
figure;
plot(f_spec, Suv_phase_deg, 'LineWidth', 1.2);
grid on;
xlim([0 0.5]);
xlabel('Frequency [Hz]');
ylabel('Phase [deg]');
title('STEP 3: Phase of recovered cross-spectrum S_{uv}');

%% STEP 4: First directional estimate from the recovered horizontal pair only
% Goal of this step:
% 1) Use the recovered horizontal spectra to estimate the principal direction axis
% 2) Compare that axis with the true wave axis
% 3) Be explicit that this is an AXIS estimate (mod 180 deg), not yet a full
%    signed "coming from" direction

%% 4A) True axis angle
% alpha_from_deg is measured clockwise from North.
% An axis has 180-degree ambiguity, so reduce it to [0, 180).

alpha_axis_true_deg = mod(alpha_from_deg, 180);

%% 4B) Estimate principal axis from recovered horizontal spectra
% The covariance / spectrum matrix in the horizontal plane is:
% [Suu  Cuv
%  Cuv  Svv]
%
% The principal-axis angle measured from the +East axis is:
% theta_E = 0.5 * atan2(2*Cuv, Suu - Svv)
%
% Then convert to "clockwise from North" and wrap to [0, 180).

theta_axis_fromEast_deg = 0.5 * rad2deg(atan2(2*Cuv_rec, Suu_rec - Svv_rec));
alpha_axis_est_deg = mod(90 - theta_axis_fromEast_deg, 180);

%% 4C) Axis error (wrapped to [-90, 90])
axis_err_deg = alpha_axis_est_deg - alpha_axis_true_deg;
axis_err_deg = mod(axis_err_deg + 90, 180) - 90;

axis_err_band_mean = mean(axis_err_deg(idx_wave));
axis_err_band_std  = std(axis_err_deg(idx_wave));

% Estimate at dominant recovered u_E peak
[~, idx_peak_axis_local] = max(Suu_rec(idx_wave));
f_wave = f_spec(idx_wave);
f_axis_peak = f_wave(idx_peak_axis_local);

[~, idx_axis_peak] = min(abs(f_spec - f_axis_peak));
alpha_axis_peak_est_deg = alpha_axis_est_deg(idx_axis_peak);

axis_peak_err_deg = alpha_axis_peak_est_deg - alpha_axis_true_deg;
axis_peak_err_deg = mod(axis_peak_err_deg + 90, 180) - 90;

fprintf('\n--- STEP 4: Horizontal-axis directional estimate ---\n');
fprintf('True wave axis (mod 180): %.2f deg clockwise from North\n', alpha_axis_true_deg);
fprintf('Band-mean estimated axis: %.2f deg clockwise from North\n', mean(alpha_axis_est_deg(idx_wave)));
fprintf('Estimated axis at dominant peak: %.2f deg clockwise from North\n', alpha_axis_peak_est_deg);
fprintf('Band-mean axis error: %.6f deg\n', axis_err_band_mean);
fprintf('Band-std axis error: %.6f deg\n', axis_err_band_std);
fprintf('Peak-frequency axis error: %.6f deg\n', axis_peak_err_deg);

%% 4D) Plot estimated axis vs frequency
figure;
plot(f_spec, alpha_axis_est_deg, 'LineWidth', 1.3); hold on;
yline(alpha_axis_true_deg, '--', 'LineWidth', 1.2);
xline(f_min_wave, '--', 'LineWidth', 1.0);
xline(f_max_wave, '--', 'LineWidth', 1.0);
grid on;
xlim([0 0.5]);
ylim([0 180]);
xlabel('Frequency [Hz]');
ylabel('Estimated axis [deg clockwise from North]');
title('STEP 4: Principal horizontal direction axis from recovered u_E and v_N');
legend('Estimated axis', 'True axis', 'Wave-band limits', 'Location', 'best');

%% 4E) Plot axis error vs frequency
figure;
plot(f_spec, axis_err_deg, 'LineWidth', 1.3); hold on;
xline(f_min_wave, '--', 'LineWidth', 1.0);
xline(f_max_wave, '--', 'LineWidth', 1.0);
grid on;
xlim([0 0.5]);
xlabel('Frequency [Hz]');
ylabel('Axis error [deg]');
title('STEP 4: Horizontal-axis estimation error');

%% STEP 5: First directional spreading slice using vertical energy + horizontal axis
% Goal of this step:
% 1) Use the vertical channel as the 1D energy backbone
% 2) Use the recovered horizontal pair to build a 180-deg-ambiguous spreading function
% 3) Plot the first polar directional slice at the dominant vertical frequency
%
% IMPORTANT:
% This uses only the horizontal pair for directionality, so it gives an AXIS
% distribution, not yet a signed "coming from" direction.

%% 5A) Vertical energy spectrum from eta(t)
eta_psd = detrend(eta, 'constant');
eta_kf_psd = detrend(eta_kf, 'constant');
[Setaeta, f_eta] = pwelch(eta_psd, win_len, noverlap, nfft, fs);
[Setaeta_kf, ~] = pwelch(eta_kf_psd, win_len, noverlap, nfft, fs);

idx_wave_eta = (f_eta >= f_min_wave) & (f_eta <= f_max_wave);

[~, idx_eta_peak_local] = max(Setaeta(idx_wave_eta));
f_eta_wave = f_eta(idx_wave_eta);
f_eta_peak = f_eta_wave(idx_eta_peak_local);

[~, idx_peak_common] = min(abs(f_spec - f_eta_peak));

%% 5B) Second-order horizontal directional coefficients
% These come from the horizontal spectral matrix only.
% They define the principal axis and how concentrated the axis distribution is.

den_h = Suu_rec + Svv_rec + eps;

a2_rec = (Suu_rec - Svv_rec) ./ den_h;
b2_rec = 2*Cuv_rec ./ den_h;

% Strength of the second-order directional structure
r2_rec = sqrt(a2_rec.^2 + b2_rec.^2);

% Keep it physically safe
r2_rec = min(r2_rec, 0.999);

a2_peak = a2_rec(idx_peak_common);
b2_peak = b2_rec(idx_peak_common);
r2_peak = r2_rec(idx_peak_common);

alpha_axis_peak_deg = alpha_axis_est_deg(idx_peak_common);
coh_peak = coh_uv_rec(idx_peak_common);
Setaeta_peak = Setaeta(idx_peak_common);

%% 5C) Build the first axis-based spreading slice at the dominant frequency
alpha_grid_deg = (0:1:359)';
alpha_grid_rad = deg2rad(alpha_grid_deg);

% Axis-symmetric spreading:
% D(alpha) = 1/(2*pi) * [1 + r2*cos(2*(alpha - alpha0))]
%
% alpha and alpha0 are both measured clockwise from North here.
D_axis_peak = (1/(2*pi)) * ...
    (1 + r2_peak * cosd(2*(alpha_grid_deg - alpha_axis_peak_deg)));

% Numerical safety in case of tiny negative roundoff
D_axis_peak(D_axis_peak < 0) = 0;

% Normalize so integral over direction is 1
D_axis_peak = D_axis_peak / trapz(alpha_grid_rad, D_axis_peak);

% Directional spectrum slice at the dominant frequency
S_dir_peak = Setaeta_peak * D_axis_peak;

%% 5D) Diagnostics
fprintf('\n--- STEP 5: First directional spreading slice ---\n');
fprintf('Dominant vertical-spectrum frequency: %.6f Hz\n', f_eta_peak);
fprintf('Vertical spectrum value at peak: %.6e\n', Setaeta_peak);
fprintf('Recovered coherence at peak: %.6f\n', coh_peak);
fprintf('a2 at peak: %.6f\n', a2_peak);
fprintf('b2 at peak: %.6f\n', b2_peak);
fprintf('r2 at peak: %.6f\n', r2_peak);
fprintf('Estimated axis at peak: %.6f deg clockwise from North\n', alpha_axis_peak_deg);
fprintf('NOTE: This is an axis-based distribution with 180-deg ambiguity.\n');

% Compare vertical spectra
Seta_rel_err = norm(Setaeta_kf(idx_wave) - Setaeta(idx_wave)) / norm(Setaeta(idx_wave));
fprintf('Vertical spectrum error (fused vs truth): %.4f%%\n', Seta_rel_err*100);
fprintf('Hm0 from truth eta:      %.4f m\n', 4*std(eta));
fprintf('Hm0 from IMU only eta:   %.4f m\n', 4*std(eta_imu));
fprintf('Hm0 from fused eta:      %.4f m\n', 4*std(eta_kf));

%% 5E) Plot the spreading slice as a standard line plot
figure;
plot(alpha_grid_deg, D_axis_peak, 'LineWidth', 1.3); hold on;
xline(alpha_axis_peak_deg, '--', 'LineWidth', 1.1);
xline(mod(alpha_axis_peak_deg + 180, 360), '--', 'LineWidth', 1.1);
grid on;
xlim([0 360]);
xlabel('Direction [deg clockwise from North]');
ylabel('D(\alpha) [1/rad]');
title('STEP 5: First axis-based directional spreading slice at dominant frequency');
legend('D(\alpha)', 'Estimated axis', 'Opposite axis', 'Location', 'best');

%% 5F) Plot as a polar directional slice
figure;
pax = polaraxes;
polarplot(pax, alpha_grid_rad, S_dir_peak, 'LineWidth', 1.4);
pax.ThetaZeroLocation = 'top';
pax.ThetaDir = 'clockwise';
title('STEP 5: First polar directional slice (180-deg ambiguous)');

%% STEP 6: Use the vertical channel to resolve the 180-deg ambiguity
% Goal of this step:
% 1) Compute cross-spectra between eta and the recovered u_E, v_N channels
% 2) Extract the first-order directional vector from the quadrature parts
% 3) Estimate a unique propagation direction and then the "coming-from" direction
%
% IMPORTANT:
% For THIS synthetic model:
% - eta(t) was built in quadrature with the horizontal channels
% - so the useful first-order information lives in imag(S_eta_u) and imag(S_eta_v)

%% 6A) Cross-spectra between eta and recovered horizontal channels
[Seta_u_rec, f_eta_u] = cpsd(eta_psd, uE_rec_psd, win_len, noverlap, nfft, fs);
[Seta_v_rec, f_eta_v] = cpsd(eta_psd, vN_rec_psd, win_len, noverlap, nfft, fs);

% Cross-spectra using fused vertical channel
[Seta_u_kf, ~] = cpsd(eta_kf_psd, uE_real_psd, win_len, noverlap, nfft, fs);
[Seta_v_kf, ~] = cpsd(eta_kf_psd, vN_real_psd, win_len, noverlap, nfft, fs);

Qeta_u_kf = imag(Seta_u_kf);
Qeta_v_kf = imag(Seta_v_kf);

Qeta_u_rec = imag(Seta_u_rec);
Qeta_v_rec = imag(Seta_v_rec);

%% 6B) First-order directional coefficients
% For the current synthetic model:
%   -Q_eta_u is proportional to the East component of propagation
%   -Q_eta_v is proportional to the North component of propagation
%
% Normalize to make dimensionless first-order directional quantities.

den1 = sqrt(Setaeta .* (Suu_rec + Svv_rec) + eps);

a1_rec = -Qeta_u_rec ./ den1;    % East-like first-order directional component
b1_rec = -Qeta_v_rec ./ den1;    % North-like first-order directional component

r1_rec = sqrt(a1_rec.^2 + b1_rec.^2);
r1_rec = min(r1_rec, 0.999);

% First-order coefficients using fused channels
den1_kf = sqrt(Setaeta_kf .* (Suu_real + Svv_real) + eps);
a1_kf = -Qeta_u_kf ./ den1_kf;
b1_kf = -Qeta_v_kf ./ den1_kf;
r1_kf = min(sqrt(a1_kf.^2 + b1_kf.^2), 0.999);

% Direction from fused channels
theta_toward_kf_deg = mod(atan2d(a1_kf, b1_kf), 360);
theta_from_kf_deg   = mod(theta_toward_kf_deg + 180, 360);

% Error
theta_from_kf_err = theta_from_kf_deg - alpha_from_deg;
theta_from_kf_err = mod(theta_from_kf_err + 180, 360) - 180;

% Peak frequency direction
theta_from_kf_peak = theta_from_kf_deg(idx_peak_common);
theta_from_kf_peak_err = theta_from_kf_peak - alpha_from_deg;
theta_from_kf_peak_err = mod(theta_from_kf_peak_err + 180, 360) - 180;

% Band mean
theta_kf_band_mean = mod(rad2deg(atan2(...
    mean(sind(theta_from_kf_deg(idx_wave))), ...
    mean(cosd(theta_from_kf_deg(idx_wave))))), 360);

theta_kf_band_err = theta_kf_band_mean - alpha_from_deg;
theta_kf_band_err = mod(theta_kf_band_err + 180, 360) - 180;

fprintf('\n--- STEP 6 FUSED: Direction from fused vertical + real horizontal ---\n');
fprintf('Peak-freq direction: %.4f deg   error: %.4f deg\n', ...
        theta_from_kf_peak, theta_from_kf_peak_err);
fprintf('Band-mean direction: %.4f deg   error: %.4f deg\n', ...
        theta_kf_band_mean, theta_kf_band_err);
%% 6C) Convert first-order vector into direction
% Direction TOWARD, measured clockwise from North:
theta_toward_est_deg = mod(atan2d(a1_rec, b1_rec), 360);

% Convert to COMING-FROM direction
theta_from_est_deg = mod(theta_toward_est_deg + 180, 360);

%% 6D) Wrapped directional errors
theta_from_err_deg = theta_from_est_deg - alpha_from_deg;
theta_from_err_deg = mod(theta_from_err_deg + 180, 360) - 180;

theta_from_band_mean = mean(theta_from_est_deg(idx_wave));
theta_from_band_err_mean = mean(theta_from_err_deg(idx_wave));
theta_from_band_err_std  = std(theta_from_err_deg(idx_wave));

theta_from_peak_est_deg = theta_from_est_deg(idx_peak_common);
theta_from_peak_err_deg = theta_from_peak_est_deg - alpha_from_deg;
theta_from_peak_err_deg = mod(theta_from_peak_err_deg + 180, 360) - 180;

%% 6E) Diagnostics at dominant frequency
a1_peak = a1_rec(idx_peak_common);
b1_peak = b1_rec(idx_peak_common);
r1_peak = r1_rec(idx_peak_common);

fprintf('\n--- STEP 6: First unique directional estimate from eta + horizontal channels ---\n');
fprintf('True coming-from direction: %.6f deg clockwise from North\n', alpha_from_deg);
fprintf('Dominant vertical-spectrum frequency: %.6f Hz\n', f_eta_peak);
fprintf('a1 at peak: %.6f\n', a1_peak);
fprintf('b1 at peak: %.6f\n', b1_peak);
fprintf('r1 at peak: %.6f\n', r1_peak);
fprintf('Estimated toward-direction at peak: %.6f deg clockwise from North\n', theta_toward_est_deg(idx_peak_common));
fprintf('Estimated coming-from direction at peak: %.6f deg clockwise from North\n', theta_from_peak_est_deg);
fprintf('Band-mean coming-from direction: %.6f deg clockwise from North\n', theta_from_band_mean);
fprintf('Band-mean coming-from error: %.6f deg\n', theta_from_band_err_mean);
fprintf('Band-std coming-from error: %.6f deg\n', theta_from_band_err_std);
fprintf('Peak-frequency coming-from error: %.6f deg\n', theta_from_peak_err_deg);

%% 6F) Plots

% Plot 1: first-order coefficients
figure;
subplot(3,1,1);
plot(f_spec, a1_rec, 'LineWidth', 1.2);
grid on;
xlim([0 0.5]);
ylabel('a_1');
title('STEP 6: First-order directional coefficients');

subplot(3,1,2);
plot(f_spec, b1_rec, 'LineWidth', 1.2);
grid on;
xlim([0 0.5]);
ylabel('b_1');

subplot(3,1,3);
plot(f_spec, r1_rec, 'LineWidth', 1.2);
grid on;
xlim([0 0.5]);
xlabel('Frequency [Hz]');
ylabel('r_1');

% Plot 2: unique coming-from direction vs frequency
figure;
plot(f_spec, theta_from_est_deg, 'LineWidth', 1.3); hold on;
yline(alpha_from_deg, '--', 'LineWidth', 1.2);
xline(f_min_wave, '--', 'LineWidth', 1.0);
xline(f_max_wave, '--', 'LineWidth', 1.0);
grid on;
xlim([0 0.5]);
ylim([0 360]);
xlabel('Frequency [Hz]');
ylabel('Estimated coming-from direction [deg clockwise from North]');
title('STEP 6: First unique directional estimate');
legend('Estimated coming-from direction', 'True coming-from direction', ...
       'Wave-band limits', 'Location', 'best');

% Plot 3: unique directional error vs frequency
figure;
plot(f_spec, theta_from_err_deg, 'LineWidth', 1.3); hold on;
xline(f_min_wave, '--', 'LineWidth', 1.0);
xline(f_max_wave, '--', 'LineWidth', 1.0);
grid on;
xlim([0 0.5]);
xlabel('Frequency [Hz]');
ylabel('Direction error [deg]');
title('STEP 6: Unique directional error');

%% STEP 7: Full directional spreading slice at dominant frequency
% Goal of this step:
% 1) Combine first-order and second-order directional information
% 2) Build a fuller directional spreading function at the dominant frequency
% 3) Show that the 180-deg ambiguity is now resolved
%
% This is still only a single-frequency slice, not yet the full 2D spectrum.

%% 7A) Weighted Fourier-style coefficients at the dominant frequency
% Using the weighting factors we discussed earlier:
% first-order weighted more strongly than second-order

a1_w_peak = (2/3) * a1_peak;
b1_w_peak = (2/3) * b1_peak;

a2_w_peak = (1/6) * a2_peak;
b2_w_peak = (1/6) * b2_peak;

%% 7B) Build full directional spreading slice
% alpha is measured clockwise from North.
%
% First-order term in this convention:
%   a1*sin(alpha) + b1*cos(alpha)
%
% Second-order term in this convention:
%   -a2*cos(2alpha) + b2*sin(2alpha)

term1_peak = -(a1_w_peak * sind(alpha_grid_deg) + b1_w_peak * cosd(alpha_grid_deg));
term2_peak = -a2_w_peak * cosd(2*alpha_grid_deg) + b2_w_peak * sind(2*alpha_grid_deg);

D_full_peak = (1/(2*pi)) * (1 + 2*term1_peak + 2*term2_peak);

% Prevent tiny non-physical negatives from numerical effects
D_full_peak(D_full_peak < 0) = 0;

% Normalize so integral over direction is 1
D_full_peak = D_full_peak / trapz(alpha_grid_rad, D_full_peak);

% Directional spectrum slice at dominant frequency
S_dir_full_peak = Setaeta_peak * D_full_peak;

%% 7C) Extract peak direction from the full slice
[~, idx_Dfull_max] = max(D_full_peak);
theta_from_slice_peak_deg = alpha_grid_deg(idx_Dfull_max);

theta_from_slice_err_deg = theta_from_slice_peak_deg - alpha_from_deg;
theta_from_slice_err_deg = mod(theta_from_slice_err_deg + 180, 360) - 180;

fprintf('\n--- STEP 7: Full directional slice at dominant frequency ---\n');
fprintf('Dominant frequency used: %.6f Hz\n', f_eta_peak);
fprintf('Weighted a1 at peak: %.6f\n', a1_w_peak);
fprintf('Weighted b1 at peak: %.6f\n', b1_w_peak);
fprintf('Weighted a2 at peak: %.6f\n', a2_w_peak);
fprintf('Weighted b2 at peak: %.6f\n', b2_w_peak);
fprintf('Peak direction from full slice: %.6f deg clockwise from North\n', theta_from_slice_peak_deg);
fprintf('Peak-direction error from full slice: %.6f deg\n', theta_from_slice_err_deg);

%% 7D) Compare axis-only and full directional slices
figure;
plot(alpha_grid_deg, D_axis_peak, 'LineWidth', 1.2); hold on;
plot(alpha_grid_deg, D_full_peak, 'LineWidth', 1.4);
xline(alpha_from_deg, '--', 'LineWidth', 1.1);
grid on;
xlim([0 360]);
xlabel('Direction [deg clockwise from North]');
ylabel('D(\alpha) [1/rad]');
title('STEP 7: Axis-only vs full directional spreading slice');
legend('Axis-only slice', 'Full slice', 'True coming-from direction', 'Location', 'best');

%% 7E) Full directional slice as a polar plot
figure;
pax = polaraxes;
polarplot(pax, alpha_grid_rad, S_dir_full_peak, 'LineWidth', 1.5);
pax.ThetaZeroLocation = 'top';
pax.ThetaDir = 'clockwise';
title('STEP 7: Full polar directional slice at dominant frequency');

%% 7F) Optional: show both polar slices together
figure;
pax2 = polaraxes;
polarplot(pax2, alpha_grid_rad, S_dir_peak, 'LineWidth', 1.2); hold on;
polarplot(pax2, alpha_grid_rad, S_dir_full_peak, 'LineWidth', 1.5);
pax2.ThetaZeroLocation = 'top';
pax2.ThetaDir = 'clockwise';
title('STEP 7: Axis-only vs full polar directional slice');
legend('Axis-only', 'Full slice', 'Location', 'best');

%% STEP 8: Full directional spectrum S(f, alpha) over the trusted wave band
% Goal of this step:
% 1) Build D(f, alpha) for every frequency bin in the valid wave band
% 2) Form S(f, alpha) = S_etaeta(f) * D(f, alpha)
% 3) Extract practical summary quantities from the full directional spectrum
%
% IMPORTANT:
% - alpha is the COMING-FROM direction, clockwise from North
% - I only trust bins inside the wave band and above a coherence threshold

%% 8A) Valid frequency mask
coh_thresh = 0.90;   % first practical threshold for this proof of concept
valid_mask = idx_wave & (coh_uv_rec >= coh_thresh);

fprintf('\n--- STEP 8: Directional-spectrum setup ---\n');
fprintf('Wave-band bins: %d\n', sum(idx_wave));
fprintf('Valid bins after coherence threshold %.2f: %d\n', coh_thresh, sum(valid_mask));

%% 8B) Allocate directional spreading and directional spectrum arrays
nAlpha = length(alpha_grid_deg);
nFreq  = length(f_spec);

D_full_all = zeros(nAlpha, nFreq);    % D(f, alpha)
S_dir_all  = zeros(nAlpha, nFreq);    % S(f, alpha)

%% 8C) Build D(f, alpha) for each valid frequency bin
for k = 1:nFreq

    if ~valid_mask(k)
        continue;
    end

    % Weighted coefficients at this frequency
    a1_w = (2/3) * a1_rec(k);
    b1_w = (2/3) * b1_rec(k);

    a2_w = (1/6) * a2_rec(k);
    b2_w = (1/6) * b2_rec(k);

    % Full spreading function using COMING-FROM angle convention
    term1_k = -(a1_w * sind(alpha_grid_deg) + b1_w * cosd(alpha_grid_deg));
    term2_k = -a2_w * cosd(2*alpha_grid_deg) + b2_w * sind(2*alpha_grid_deg);

    D_k = (1/(2*pi)) * (1 + 2*term1_k + 2*term2_k);

    % Prevent tiny non-physical negatives
    D_k(D_k < 0) = 0;

    % Normalize so integral over alpha is 1
    area_k = trapz(alpha_grid_rad, D_k);
    if area_k > 0
        D_k = D_k / area_k;
    end

    D_full_all(:,k) = D_k;
    S_dir_all(:,k)  = Setaeta(k) * D_k;
end

%% 8D) Frequency-dependent peak direction
theta_from_peak_byfreq_deg = nan(nFreq,1);

for k = 1:nFreq
    if ~valid_mask(k)
        continue;
    end

    [~, idx_pk] = max(S_dir_all(:,k));
    theta_from_peak_byfreq_deg(k) = alpha_grid_deg(idx_pk);
end

%% 8E) Band-integrated directional distribution
S_dir_band = trapz(f_spec(valid_mask), S_dir_all(:,valid_mask), 2);

% Normalize only for shape display if desired
if trapz(alpha_grid_rad, S_dir_band) > 0
    S_dir_band_norm = S_dir_band / trapz(alpha_grid_rad, S_dir_band);
else
    S_dir_band_norm = S_dir_band;
end

[~, idx_band_pk] = max(S_dir_band);
theta_from_band_peak_deg = alpha_grid_deg(idx_band_pk);

theta_band_peak_err_deg = theta_from_band_peak_deg - alpha_from_deg;
theta_band_peak_err_deg = mod(theta_band_peak_err_deg + 180, 360) - 180;

%% 8F) Direction at dominant vertical-spectrum frequency
theta_from_fpeak_deg = theta_from_peak_byfreq_deg(idx_peak_common);

theta_fpeak_err_deg = theta_from_fpeak_deg - alpha_from_deg;
theta_fpeak_err_deg = mod(theta_fpeak_err_deg + 180, 360) - 180;

%% 8G) A simple band-mean direction from valid bins
theta_valid = theta_from_peak_byfreq_deg(valid_mask);

% circular mean for angles in degrees
theta_valid_rad = deg2rad(theta_valid);
theta_band_mean_deg = mod(rad2deg(atan2(mean(sind(theta_valid)), mean(cosd(theta_valid)))), 360);

theta_band_mean_err_deg = theta_band_mean_deg - alpha_from_deg;
theta_band_mean_err_deg = mod(theta_band_mean_err_deg + 180, 360) - 180;

%% 8H) Print summary
fprintf('True coming-from direction: %.6f deg clockwise from North\n', alpha_from_deg);
fprintf('Dominant vertical-spectrum frequency: %.6f Hz\n', f_eta_peak);
fprintf('Peak direction at dominant frequency: %.6f deg\n', theta_from_fpeak_deg);
fprintf('Peak-direction error at dominant frequency: %.6f deg\n', theta_fpeak_err_deg);
fprintf('Band-integrated peak direction: %.6f deg\n', theta_from_band_peak_deg);
fprintf('Band-integrated peak-direction error: %.6f deg\n', theta_band_peak_err_deg);
fprintf('Band-mean direction from valid bins: %.6f deg\n', theta_band_mean_deg);
fprintf('Band-mean direction error: %.6f deg\n', theta_band_mean_err_deg);

%% FULL CIRCULAR DIRECTIONAL SPECTRUM (manual polar heatmap)

% Keep only valid bins
f_valid = f_spec(valid_mask);
S_valid = S_dir_all(:, valid_mask);

% Close the direction loop so 0 deg and 360 deg join properly
alpha_plot_deg = [alpha_grid_deg; 360];
alpha_plot_rad = deg2rad(alpha_plot_deg);

S_valid_plot = [S_valid; S_valid(1,:)];   % size: (nAlpha+1) x nFreq

% Build polar grid
[Theta, R] = meshgrid(alpha_plot_rad, f_valid);

% Convert polar -> Cartesian
% clockwise from North:
X = R .* sin(Theta);
Y = R .* cos(Theta);

% Transpose for plotting
Z = S_valid_plot.';   % size: nFreq x (nAlpha+1)

% Optional: suppress very low-energy background so the ridge stands out
Zplot = Z;
Zplot(Zplot < 0.05*max(Zplot(:))) = NaN;

figure;
surf(X, Y, Zplot, 'EdgeColor', 'none');
view(2);
axis equal tight;
colormap(turbo);
colorbar;
hold on;

% Radial circles
rmax = max(f_valid);
ang = linspace(0, 2*pi, 361);
for rr = [0.1 0.2 0.3 0.4]
    plot(rr*sin(ang), rr*cos(ang), 'k:', 'LineWidth', 0.5);
end

% Direction spokes every 30 deg
for a = 0:30:330
    plot([0 rmax*sind(a)], [0 rmax*cosd(a)], 'k:', 'LineWidth', 0.5);
    text(1.08*rmax*sind(a), 1.08*rmax*cosd(a), sprintf('%d°', a), ...
        'HorizontalAlignment', 'center', 'FontSize', 9);
end

% True direction
plot([0 rmax*sind(alpha_from_deg)], [0 rmax*cosd(alpha_from_deg)], ...
    '--w', 'LineWidth', 1.4);

title('Full directional spectrum S(f,\alpha)');
xlabel('East');
ylabel('North');

%% 8J) Plot 2: Peak direction vs frequency
figure;
plot(f_spec, theta_from_peak_byfreq_deg, 'LineWidth', 1.3); hold on;
xline(f_min_wave, '--', 'LineWidth', 1.0);
xline(f_max_wave, '--', 'LineWidth', 1.0);
yline(alpha_from_deg, '--', 'LineWidth', 1.2);
grid on;
xlim([0 0.5]);
ylim([0 360]);
xlabel('Frequency [Hz]');
ylabel('Peak direction [deg clockwise from North]');
title('STEP 8: Peak direction from full directional spectrum');
legend('Estimated peak direction', 'Wave-band limits', 'True coming-from direction', ...
       'Location', 'best');

%% 8K) Plot 3: Band-integrated directional distribution
figure;
plot(alpha_grid_deg, S_dir_band_norm, 'LineWidth', 1.4); hold on;
xline(alpha_from_deg, '--', 'LineWidth', 1.2);
xline(theta_from_band_peak_deg, ':', 'LineWidth', 1.2);
grid on;
xlim([0 360]);
xlabel('Direction [deg clockwise from North]');
ylabel('Normalized band-integrated energy');
title('STEP 8: Band-integrated directional distribution');
legend('Integrated directional distribution', 'True coming-from direction', ...
       'Estimated integrated peak', 'Location', 'best');

%% 8L) Plot 4: Band-integrated directional distribution in polar form
figure;
pax3 = polaraxes;
polarplot(pax3, alpha_grid_rad, S_dir_band_norm, 'LineWidth', 1.5);
pax3.ThetaZeroLocation = 'top';
pax3.ThetaDir = 'clockwise';
title('STEP 8: Band-integrated polar directional distribution');

%% STEP 8B: Add ridge overlay and stability diagnostics to the circular plot

% Keep only valid bins
f_ridge = f_spec(valid_mask);
theta_ridge_deg = theta_from_peak_byfreq_deg(valid_mask);

% Convert ridge from polar to Cartesian
x_ridge = f_ridge .* sind(theta_ridge_deg);
y_ridge = f_ridge .* cosd(theta_ridge_deg);

% True direction line
rmax = max(f_ridge);
x_true = [0, rmax*sind(alpha_from_deg)];
y_true = [0, rmax*cosd(alpha_from_deg)];

% Estimated band-integrated peak line
x_est = [0, rmax*sind(theta_from_band_peak_deg)];
y_est = [0, rmax*cosd(theta_from_band_peak_deg)];

% Overlay on the existing circular spectrum figure
hold on;
plot(x_ridge, y_ridge, 'k.-', 'LineWidth', 1.8, 'MarkerSize', 12);
plot(x_true, y_true, '--w', 'LineWidth', 2.0);
plot(x_est, y_est, ':k', 'LineWidth', 2.0);

legend('Spectrum', 'Estimated ridge', 'True direction', 'Estimated band peak', ...
    'Location', 'bestoutside');

%% Ridge stability diagnostics
theta_valid = theta_ridge_deg;

fprintf('\n--- STEP 8B: Ridge stability diagnostics ---\n');
fprintf('Min valid peak direction: %.6f deg\n', min(theta_valid));
fprintf('Max valid peak direction: %.6f deg\n', max(theta_valid));
fprintf('Mean valid peak direction: %.6f deg\n', mean(theta_valid));
fprintf('Std valid peak direction: %.6f deg\n', std(theta_valid));

% Wrapped error relative to truth
theta_valid_err = theta_valid - alpha_from_deg;
theta_valid_err = mod(theta_valid_err + 180, 360) - 180;

fprintf('Mean valid peak-direction error: %.6f deg\n', mean(theta_valid_err));
fprintf('Std valid peak-direction error: %.6f deg\n', std(theta_valid_err));
fprintf('Max abs valid peak-direction error: %.6f deg\n', max(abs(theta_valid_err)));

%% Optional: separate clean figure with only ridge + truth lines
figure;
hold on;
axis equal;
grid on;

% draw radial circles
ang = linspace(0, 2*pi, 361);
for rr = [0.1 0.2 0.3 0.4]
    plot(rr*sin(ang), rr*cos(ang), 'k:', 'LineWidth', 0.6);
end

% draw direction spokes every 30 deg
for a = 0:30:330
    plot([0 rmax*sind(a)], [0 rmax*cosd(a)], 'k:', 'LineWidth', 0.6);
    text(1.07*rmax*sind(a), 1.07*rmax*cosd(a), sprintf('%d°', a), ...
        'HorizontalAlignment', 'center', 'FontSize', 9);
end

plot(x_ridge, y_ridge, 'b.-', 'LineWidth', 1.8, 'MarkerSize', 12);
plot(x_true, y_true, '--r', 'LineWidth', 2.0);
plot(x_est, y_est, ':k', 'LineWidth', 2.0);

xlabel('East');
ylabel('North');
title('Ridge line of peak direction vs frequency');
legend('0.1/0.2/0.3/0.4 Hz circles', '', '', '', '', ...
       'Estimated ridge', 'True direction', 'Estimated band peak', ...
       'Location', 'bestoutside');
xlim([-rmax rmax]);
ylim([-rmax rmax]);

%% STEP 9: Directional pipeline on real sensor channels

% Cross-spectrum and coherence of real channels
[Suv_real, ~] = cpsd(uE_real_psd, vN_real_psd, win_len, noverlap, nfft, fs);
coh_uv_real_full = mscohere(uE_real_psd, vN_real_psd, win_len, noverlap, nfft, fs);

Cuv_real = real(Suv_real);
Quv_real = imag(Suv_real);

% Second-order directional coefficients
den_h_real = Suu_real + Svv_real + eps;
a2_real = (Suu_real - Svv_real) ./ den_h_real;
b2_real = 2*Cuv_real ./ den_h_real;

% Principal axis
theta_axis_real_deg = 0.5 * rad2deg(atan2(2*Cuv_real, Suu_real - Svv_real));
alpha_axis_real_deg = mod(90 - theta_axis_real_deg, 180);

% Vertical spectrum (reuse existing Setaeta from eta)
eta_psd_real = detrend(eta, 'constant');

% Cross-spectra between eta and real horizontal channels
[Seta_u_real, ~] = cpsd(eta_psd_real, uE_real_psd, win_len, noverlap, nfft, fs);
[Seta_v_real, ~] = cpsd(eta_psd_real, vN_real_psd, win_len, noverlap, nfft, fs);

Qeta_u_real = imag(Seta_u_real);
Qeta_v_real = imag(Seta_v_real);

% First-order directional coefficients
den1_real = sqrt(Setaeta .* (Suu_real + Svv_real) + eps);
a1_real = -Qeta_u_real ./ den1_real;
b1_real = -Qeta_v_real ./ den1_real;

r1_real = min(sqrt(a1_real.^2 + b1_real.^2), 0.999);
r2_real = min(sqrt(a2_real.^2 + b2_real.^2), 0.999);

% Unique coming-from direction
theta_toward_real_deg = mod(atan2d(a1_real, b1_real), 360);
theta_from_real_deg   = mod(theta_toward_real_deg + 180, 360);

% Direction error
theta_from_real_err = theta_from_real_deg - alpha_from_deg;
theta_from_real_err = mod(theta_from_real_err + 180, 360) - 180;

% Valid mask using real coherence
valid_mask_real = idx_wave & (coh_uv_real_full >= coh_thresh);

% Band mean direction
theta_real_band_mean = mod(rad2deg(atan2(...
    mean(sind(theta_from_real_deg(valid_mask_real))), ...
    mean(cosd(theta_from_real_deg(valid_mask_real))))), 360);

theta_real_band_err = theta_real_band_mean - alpha_from_deg;
theta_real_band_err = mod(theta_real_band_err + 180, 360) - 180;

% Peak at dominant frequency
theta_from_real_peak = theta_from_real_deg(idx_peak_common);
theta_from_real_peak_err = theta_from_real_peak - alpha_from_deg;
theta_from_real_peak_err = mod(theta_from_real_peak_err + 180, 360) - 180;

fprintf('\n--- STEP 9: Directional pipeline on real sensor channels ---\n');
fprintf('True coming-from direction: %.4f deg\n', alpha_from_deg);
fprintf('Valid bins (real channels): %d\n', sum(valid_mask_real));
fprintf('Band-mean direction (real): %.4f deg\n', theta_real_band_mean);
fprintf('Band-mean error (real):     %.4f deg\n', theta_real_band_err);
fprintf('Peak-freq direction (real): %.4f deg\n', theta_from_real_peak);
fprintf('Peak-freq error (real):     %.4f deg\n', theta_from_real_peak_err);
fprintf('\n--- Comparison: idealised vs real ---\n');
fprintf('Band-mean error idealised: %.4f deg\n', theta_from_band_err_mean);
fprintf('Band-mean error real:      %.4f deg\n', theta_real_band_err);

%% STEP 9: Plot — coming-from direction vs frequency
figure;
plot(f_spec, theta_from_est_deg, 'LineWidth', 1.3); hold on;
plot(f_spec, theta_from_real_deg, 'LineWidth', 1.3);
yline(alpha_from_deg, '--', 'LineWidth', 1.2);
xline(f_min_wave, ':', 'LineWidth', 1.0);
xline(f_max_wave, ':', 'LineWidth', 1.0);
grid on;
xlim([0 0.5]);
ylim([0 360]);
xlabel('Frequency [Hz]');
ylabel('Coming-from direction [deg]');
title('STEP 9: Directional estimate — idealised vs real sensor channels');
legend('Idealised channels', 'Real channels', 'True direction', ...
       'Wave-band limits', 'Location', 'best');


%% STEP 10: Final pipeline summary

fprintf('\n');
fprintf('================================================================\n');
fprintf('         WAVE BUOY DIRECTIONAL PIPELINE — FINAL SUMMARY        \n');
fprintf('================================================================\n');

fprintf('\n--- Simulation parameters ---\n');
fprintf('True coming-from direction:  %.2f deg clockwise from North\n', alpha_from_deg);
fprintf('Significant wave height Hm0: %.2f m\n', 4*std(eta));
fprintf('Wave band:                   %.2f to %.2f Hz\n', f_min_wave, f_max_wave);
fprintf('Dominant frequency:          %.4f Hz\n', f_eta_peak);
fprintf('Simulation duration:         %.0f min\n', T_total/60);
fprintf('Sample rate:                 %.0f Hz\n', fs);

fprintf('\n--- Sensor model (MPU6050 + HMC5883L) ---\n');
fprintf('Accelerometer noise std:     %.2f mg\n', accel_noise_std/g*1000);
fprintf('Accelerometer bias:          %.1f mg (x)   %.1f mg (y)\n', ...
        accel_bias_x/g*1000, accel_bias_y/g*1000);
fprintf('Gyroscope noise std:         %.4f deg/s\n', gyro_noise_std);
fprintf('Gyroscope bias (calibrated): %.2f deg/s (x)   %.2f deg/s (y)\n', ...
        gyro_bias_x, gyro_bias_y);
fprintf('Magnetometer noise:          %.2f deg std\n', mag_noise_std_deg);
fprintf('Magnetometer bias:           %.2f deg\n', mag_bias_deg);
fprintf('Magnetometer sample rate:    %.0f Hz\n', fs_mag);

fprintf('\n--- Attitude estimation (complementary filter) ---\n');
fprintf('Filter alpha:                %.2f\n', alpha_cf);
fprintf('Roll  error:  mean=%.4f  std=%.4f  max=%.4f deg\n', ...
        mean(phi_err_deg), std(phi_err_deg), max(abs(phi_err_deg)));
fprintf('Pitch error:  mean=%.4f  std=%.4f  max=%.4f deg\n', ...
        mean(theta_err_deg), std(theta_err_deg), max(abs(theta_err_deg)));
fprintf('Yaw   error:  mean=%.4f  std=%.4f  max=%.4f deg\n', ...
        mean(psi_cf_err_deg), std(psi_cf_err_deg), max(abs(psi_cf_err_deg)));

fprintf('\n--- Signal recovery ---\n');
fprintf('Gravity leakage RMS (x):     %.4f m/s^2\n', rms(grav_x));
fprintf('Gravity leakage RMS (y):     %.4f m/s^2\n', rms(grav_y));
fprintf('Wave acceleration RMS (x):   %.4f m/s^2\n', rms(ax_wave));
fprintf('Wave acceleration RMS (y):   %.4f m/s^2\n', rms(ay_wave));
fprintf('Gravity/wave ratio (x):      %.2fx\n', rms(grav_x)/rms(ax_wave));
fprintf('Gravity/wave ratio (y):      %.2fx\n', rms(grav_y)/rms(ay_wave));
fprintf('Integrated velocity ratio x: %.4f\n', rms(vx_int)/rms(x_b));
fprintf('Integrated velocity ratio y: %.4f\n', rms(vy_int)/rms(y_b));

fprintf('\n--- Spectral quality ---\n');
fprintf('Idealised channels:\n');
fprintf('  u_E spectral error:        %.2f%%\n', Suu_rel_err_wave*100);
fprintf('  v_N spectral error:        %.2f%%\n', Svv_rel_err_wave*100);
fprintf('  Mean coherence:            %.4f\n', coh_uv_wave_mean);
fprintf('Real sensor channels:\n');
fprintf('  u_E spectral error:        %.2f%%\n', Suu_real_rel_err*100);
fprintf('  v_N spectral error:        %.2f%%\n', Svv_real_rel_err*100);
fprintf('  Mean coherence:            %.4f\n', mean(coh_uv_real_full(idx_wave)));

fprintf('\n--- Directional estimation ---\n');
fprintf('                        Idealised    Real sensor  Fully fused\n');
fprintf('Valid frequency bins:   %5d        %5d        %5d\n', ...
        sum(valid_mask), sum(valid_mask_real), sum(idx_wave));
fprintf('Band-mean error:        %7.3f deg  %7.3f deg  %7.3f deg\n', ...
        theta_from_band_err_mean, theta_real_band_err, theta_kf_band_err);
fprintf('Peak-freq error:        %7.3f deg  %7.3f deg  %7.3f deg\n', ...
        theta_fpeak_err_deg, theta_from_real_peak_err, theta_from_kf_peak_err);

fprintf('\n--- Vertical channel ---\n');
fprintf('Hm0 truth:              %.4f m\n', 4*std(eta));
fprintf('Hm0 IMU only:           %.4f m\n', 4*std(eta_imu));
fprintf('Hm0 IMU+GNSS fused:     %.4f m\n', 4*std(eta_kf));
fprintf('Velocity RMSE IMU only: %.6f m/s\n', rmse_imu);
fprintf('Velocity RMSE fused:    %.6f m/s\n', rmse_kf);
fprintf('Displacement RMSE IMU:  %.6f m\n', rmse_imu_disp);
fprintf('Displacement RMSE fused:%.6f m\n', rmse_kf_disp);

fprintf('\n================================================================\n');
fprintf('NOTE: Gravity removal uses truth attitude (perfect calibration).\n');
fprintf('Real hardware will require static calibration before deployment.\n');
fprintf('Antarctic deployment requires body-frame fallback mode.\n');
fprintf('================================================================\n');

%% STEP 11: Polar fallback mode — body-frame directional spectrum

% --- Deployment flag ---
polar_mode = 1;   % 0 = normal georeferenced mode
                  % 1 = Antarctic polar fallback mode

if polar_mode == 0
    fprintf('\n--- STEP 11: Normal georeferenced mode active ---\n');
    fprintf('Magnetometer heading used for Earth-frame rotation.\n');
    fprintf('Directional results are in geographic NED frame.\n');

else
    fprintf('\n--- STEP 11: POLAR FALLBACK MODE ACTIVE ---\n');
    fprintf('Magnetometer heading NOT used.\n');
    fprintf('Directional spectrum computed in buoy body frame.\n');
    fprintf('Results are relative to buoy forward axis, not geographic North.\n');

    %% 11A) Body-frame horizontal channels
    % Use integrated velocity channels in body frame directly
    % vx_int and vy_int are already in body frame before Earth rotation
    % No heading rotation applied

    vx_body = vx_int;   % body forward axis velocity
    vy_body = vy_int;   % body right axis velocity

    % Detrend
    vx_body_psd = detrend(vx_body, 'constant');
    vy_body_psd = detrend(vy_body, 'constant');

    %% 11B) Auto-spectra in body frame
    [Sxx_body, ~] = pwelch(vx_body_psd, win_len, noverlap, nfft, fs);
    [Syy_body, ~] = pwelch(vy_body_psd, win_len, noverlap, nfft, fs);
    [Sxy_body, ~] = cpsd(vx_body_psd, vy_body_psd, win_len, noverlap, nfft, fs);
    coh_body      = mscohere(vx_body_psd, vy_body_psd, win_len, noverlap, nfft, fs);

    Cxy_body = real(Sxy_body);
    Qxy_body = imag(Sxy_body);

    %% 11C) Cross-spectra with vertical channel
    [Seta_x_body, ~] = cpsd(eta_kf_psd, vx_body_psd, win_len, noverlap, nfft, fs);
    [Seta_y_body, ~] = cpsd(eta_kf_psd, vy_body_psd, win_len, noverlap, nfft, fs);

    Qeta_x_body = imag(Seta_x_body);
    Qeta_y_body = imag(Seta_y_body);

    %% 11D) First and second order directional coefficients
    den_body  = Sxx_body + Syy_body + eps;
    a2_body   = (Sxx_body - Syy_body) ./ den_body;
    b2_body   = 2*Cxy_body ./ den_body;
    r2_body   = min(sqrt(a2_body.^2 + b2_body.^2), 0.999);

    den1_body = sqrt(Setaeta_kf .* (Sxx_body + Syy_body) + eps);
    a1_body   = -Qeta_x_body ./ den1_body;
    b1_body   = -Qeta_y_body ./ den1_body;
    r1_body   = min(sqrt(a1_body.^2 + b1_body.^2), 0.999);

    %% 11E) Body-frame coming-from direction
    theta_toward_body_deg = mod(atan2d(a1_body, b1_body), 360);
    theta_from_body_deg   = mod(theta_toward_body_deg + 180, 360);

    %% 11F) Valid mask
    valid_mask_body = idx_wave & (coh_body >= coh_thresh);

    %% 11G) Full directional spectrum in body frame
    nAlpha_b = length(alpha_grid_deg);
    nFreq_b  = length(f_spec);
    S_dir_body = zeros(nAlpha_b, nFreq_b);

    for k = 1:nFreq_b
        if ~valid_mask_body(k)
            continue;
        end

        a1_w = (2/3) * a1_body(k);
        b1_w = (2/3) * b1_body(k);
        a2_w = (1/6) * a2_body(k);
        b2_w = (1/6) * b2_body(k);

        term1_k = -(a1_w * sind(alpha_grid_deg) + b1_w * cosd(alpha_grid_deg));
        term2_k = -a2_w * cosd(2*alpha_grid_deg) + b2_w * sind(2*alpha_grid_deg);

        D_k = (1/(2*pi)) * (1 + 2*term1_k + 2*term2_k);
        D_k(D_k < 0) = 0;

        area_k = trapz(alpha_grid_rad, D_k);
        if area_k > 0
            D_k = D_k / area_k;
        end

        S_dir_body(:,k) = Setaeta_kf(k) * D_k;
    end

    %% 11H) Peak direction per frequency bin
    theta_body_byfreq = nan(nFreq_b, 1);
    for k = 1:nFreq_b
        if ~valid_mask_body(k)
            continue;
        end
        [~, idx_pk] = max(S_dir_body(:,k));
        theta_body_byfreq(k) = alpha_grid_deg(idx_pk);
    end

    %% 11I) Band-integrated distribution
    S_body_band = trapz(f_spec(valid_mask_body), ...
                        S_dir_body(:,valid_mask_body), 2);
    if trapz(alpha_grid_rad, S_body_band) > 0
        S_body_band_norm = S_body_band / trapz(alpha_grid_rad, S_body_band);
    else
        S_body_band_norm = S_body_band;
    end

    [~, idx_body_pk] = max(S_body_band);
    theta_body_peak_deg = alpha_grid_deg(idx_body_pk);

    % Band mean
    theta_body_valid = theta_body_byfreq(valid_mask_body);
    theta_body_mean_deg = mod(rad2deg(atan2(...
        mean(sind(theta_body_valid)), ...
        mean(cosd(theta_body_valid)))), 360);

    %% 11J) Diagnostics
    fprintf('\n--- STEP 11: Body-frame directional results ---\n');
    fprintf('Valid frequency bins:              %d\n', sum(valid_mask_body));
    fprintf('Peak direction at dominant freq:   %.4f deg (body frame)\n', ...
            theta_from_body_deg(idx_peak_common));
    fprintf('Band-integrated peak direction:    %.4f deg (body frame)\n', ...
            theta_body_peak_deg);
    fprintf('Band-mean direction:               %.4f deg (body frame)\n', ...
            theta_body_mean_deg);
    fprintf('\nNOTE: All directions are relative to buoy forward axis.\n');
    fprintf('0 deg   = waves coming from directly ahead of buoy\n');
    fprintf('90 deg  = waves coming from buoy right side\n');
    fprintf('180 deg = waves coming from directly behind buoy\n');
    fprintf('270 deg = waves coming from buoy left side\n');
    fprintf('\n--- STEP 11: Geometry verification ---\n');
fprintf('True wave from (geographic):     %.2f deg\n', mod(alpha_from_deg, 360));
fprintf('Mean buoy heading:               %.2f deg\n', mean(psi_true_deg));
fprintf('Expected body-frame from:        %.2f deg\n', ...
        mod(mod(alpha_from_deg,360) - mean(psi_true_deg), 360));
fprintf('Received body-frame from:        %.2f deg\n', theta_body_mean_deg);
fprintf('Difference:                      %.2f deg\n', ...
        mod(theta_body_mean_deg - mod(mod(alpha_from_deg,360) - mean(psi_true_deg),360) + 180, 360) - 180);

    %% 11K) Plots
    figure('Position', [100 100 900 700]);

    subplot(2,1,1);
    plot(f_spec, theta_from_body_deg, 'b.', 'MarkerSize', 8); hold on;
    xline(f_min_wave, ':', 'LineWidth', 1.0);
    xline(f_max_wave, ':', 'LineWidth', 1.0);
    grid on;
    xlim([0 0.5]);
    ylim([0 360]);
    xlabel('Frequency [Hz]');
    ylabel('Direction [deg relative to buoy forward axis]');
    title('STEP 11: Body-frame directional estimate vs frequency (POLAR MODE)');
    yticks(0:30:360);

    subplot(2,1,2);
    plot(alpha_grid_deg, S_body_band_norm, 'k', 'LineWidth', 1.4);
    grid on;
    xlim([0 360]);
    xticks(0:30:360);
    xlabel('Direction [deg relative to buoy forward axis]');
    ylabel('Normalised energy');
    title('STEP 11: Band-integrated body-frame directional distribution');

    % Polar plot
    figure;
    ax_body = polaraxes;
    polarplot(ax_body, alpha_grid_rad, S_body_band_norm, ...
              'k', 'LineWidth', 1.5);
    ax_body.ThetaZeroLocation = 'top';
    ax_body.ThetaDir = 'clockwise';
    ax_body.ThetaTick = 0:30:330;
    ax_body.ThetaTickLabel = {'Fwd','30°','60°','Right', ...
                              '120°','150°','Aft','210°', ...
                              '240°','Left','300°','330°'};
    title('STEP 11: Polar body-frame directional distribution (POLAR MODE)');

end

%% STEP 3D: Full 3D directional spectrum S(f, alpha)
% Run this AFTER your existing pipeline script.
% All required variables (S_dir_all, f_spec, alpha_grid_deg, valid_mask,
% alpha_from_deg, f_eta_peak, F_BAND_MIN, F_BAND_MAX) must be in the workspace.
 
%% --- 1) Gather valid-band data ---
f_valid      = f_spec(valid_mask);
S_valid      = S_dir_all(:, valid_mask);   % (N_alpha x N_f_valid)
alpha_rad    = deg2rad(alpha_grid_deg);    % (N_alpha x 1)
 
N_f     = length(f_valid);
N_alpha = length(alpha_grid_deg);
 
%% --- 2) Band-integrated distribution for polar/2D reference ---
S_band = trapz(f_valid, S_valid, 2);      % (N_alpha x 1)
if trapz(alpha_rad, S_band) > 0
    S_band_norm = S_band / trapz(alpha_rad, S_band);
else
    S_band_norm = S_band;
end
 
%% --- 3) Meshgrid: alpha (rows) x frequency (cols) ---
[F_mesh, A_mesh] = meshgrid(f_valid, alpha_grid_deg);   % both (N_alpha x N_f)
S_mesh = S_valid;                                        % (N_alpha x N_f)
 
% Suppress very low energy so the ridge stands out
S_plot = S_mesh;
S_plot(S_plot < 0.02 * max(S_mesh(:))) = NaN;
 
%% === FIGURE 1: 3D surface — frequency x direction x energy ===
figure('Name','3D Directional Spectrum','Position',[80 80 1000 700]);
 
surf(F_mesh, A_mesh, S_plot, 'EdgeColor','none');
 
colormap(turbo);
cb = colorbar;
cb.Label.String = 'S(f,\alpha)  [m^2/Hz/rad]';
 
xlabel('Frequency  f  [Hz]', 'FontSize', 12);
ylabel('Direction \alpha  [deg, CW from North]', 'FontSize', 12);
zlabel('S(f,\alpha)  [m^2/Hz/rad]', 'FontSize', 12);
title(sprintf('Full 3D directional wave spectrum\nTrue coming-from: %.0f° | Hm0 = %.2f m', ...
    alpha_from_deg, 4*std(eta)), 'FontSize', 13);
 
xlim([min(f_valid) max(f_valid)]);
ylim([0 360]);
yticks(0:30:360);
view([-40 35]);
grid on;
box on;
shading interp;
 
% Mark true direction plane
hold on;
zmax = max(S_mesh(:), [], 'omitnan');
patch([min(f_valid) max(f_valid) max(f_valid) min(f_valid)], ...
      [alpha_from_deg alpha_from_deg alpha_from_deg alpha_from_deg], ...
      [0 0 zmax zmax], ...
      'r', 'FaceAlpha', 0.15, 'EdgeColor', 'r', 'LineWidth', 1.4);
text(max(f_valid)*0.98, alpha_from_deg, zmax*0.95, ...
    sprintf('%.0f° true', alpha_from_deg), ...
    'Color','r','FontSize',10,'HorizontalAlignment','right');
hold off;
 
%% === FIGURE 2: Plan view contour (top-down) ===
figure('Name','S(f,alpha) Contour','Position',[100 100 900 600]);
 
contourf(F_mesh, A_mesh, S_mesh, 20, 'LineColor','none');
colormap(turbo);
colorbar;
xlabel('Frequency  f  [Hz]', 'FontSize', 12);
ylabel('Direction  [deg CW from North]', 'FontSize', 12);
title(sprintf('S(f,\\alpha) — contour view  |  true direction = %.0f°', alpha_from_deg), ...
    'FontSize', 13);
xlim([min(f_valid) max(f_valid)]);
ylim([0 360]);
yticks(0:30:360);
grid on;
 
hold on;
yline(alpha_from_deg, 'r--', 'LineWidth', 1.8);
yline(mod(alpha_from_deg+180,360), 'r:', 'LineWidth', 1.2);
xline(f_eta_peak, 'w--', 'LineWidth', 1.4);
legend('','True coming-from','Opposite axis','Dominant f_p','Location','best');
hold off;
 
%% === FIGURE 3: Polar plot — circular spectrum at dominant frequency ===
[~, idx_fp] = min(abs(f_valid - f_eta_peak));
 
S_fp     = S_valid(:, idx_fp);
S_fp_norm = S_fp / max(S_fp(:));
 
figure('Name','Polar slice at f_p','Position',[120 120 700 650]);
pax = polaraxes;
polarplot(pax, alpha_rad, S_fp_norm, 'LineWidth', 2.0, 'Color', [0.15 0.45 0.80]);
hold(pax,'on');
 
% True direction spoke
polarplot(pax, [deg2rad(alpha_from_deg) deg2rad(alpha_from_deg)], [0 1.05], ...
    'r--', 'LineWidth', 1.8);
 
pax.ThetaZeroLocation = 'top';
pax.ThetaDir          = 'clockwise';
pax.ThetaTick         = 0:30:330;
title(sprintf('S(f_p, \\alpha) at f_p = %.3f Hz  |  true from %.0f°', ...
    f_eta_peak, alpha_from_deg), 'FontSize', 12);
legend('Normalised S(f_p,\alpha)','True direction','Location','best');
 
%% === FIGURE 4: Polar plot — band-integrated distribution ===
figure('Name','Band-integrated polar','Position',[140 140 700 650]);
pax2 = polaraxes;
polarplot(pax2, alpha_rad, S_band_norm, 'LineWidth', 2.0, 'Color', [0.10 0.65 0.45]);
hold(pax2,'on');
polarplot(pax2, [deg2rad(alpha_from_deg) deg2rad(alpha_from_deg)], ...
    [0 max(S_band_norm)*1.05], 'r--', 'LineWidth', 1.8);
pax2.ThetaZeroLocation = 'top';
pax2.ThetaDir          = 'clockwise';
pax2.ThetaTick         = 0:30:330;
title(sprintf('Band-integrated S(\\alpha)  [%.2f–%.2f Hz]  |  true from %.0f°', ...
    f_min_wave, f_max_wave, alpha_from_deg), 'FontSize', 12);
legend('Band-integrated','True direction','Location','best');
 
%% === FIGURE 5: Waterfall — frequency slices stacked ===
figure('Name','Waterfall','Position',[160 160 1000 650]);
 
stride = max(1, floor(N_f/40));    % plot ~40 slices max
idx_slices = 1:stride:N_f;
cmap = turbo(length(idx_slices));
 
hold on;
for ii = 1:length(idx_slices)
    j = idx_slices(ii);
    fj = f_valid(j);
    Sj = S_valid(:, j);
    Sj(Sj < 0.01*max(S_mesh(:))) = NaN;
    plot3(repmat(fj, N_alpha, 1), alpha_grid_deg, Sj, ...
        'Color', cmap(ii,:), 'LineWidth', 1.0);
end
hold off;
 
xlabel('Frequency  [Hz]', 'FontSize', 12);
ylabel('Direction  [deg CW from N]', 'FontSize', 12);
zlabel('S(f,\alpha)', 'FontSize', 12);
title('Waterfall: directional energy slices vs frequency', 'FontSize', 13);
ylim([0 360]);
yticks(0:30:360);
view([-50 30]);
grid on;
colormap(turbo);
 
%% === FIGURE 6: Circular polar heatmap (radius = frequency, angle = direction) ===
figure('Name','Circular directional spectrum','Position',[180 180 750 750]);
 
% Close the angular loop
alpha_plot = [alpha_grid_deg; 360];
alpha_plot_rad = deg2rad(alpha_plot);
S_plot_circ = [S_valid; S_valid(1,:)];
 
[Theta, R] = meshgrid(alpha_plot_rad, f_valid);
X = R .* sin(Theta);
Y = R .* cos(Theta);
Z = S_plot_circ.';             % (N_f x N_alpha+1)
 
Z_circ = Z;
Z_circ(Z_circ < 0.03*max(Z(:))) = NaN;
 
surf(X, Y, Z_circ, 'EdgeColor','none');
view(2);
axis equal tight;
colormap(turbo);
colorbar;
shading interp;
hold on;
 
% Radial frequency rings
ang_ring = linspace(0, 2*pi, 361);
for rr = [0.1 0.2 0.3 0.4]
    plot3(rr*sin(ang_ring), rr*cos(ang_ring), repmat(max(Z(:),[],'omitnan'), 1, 361), ...
        'k:', 'LineWidth', 0.6);
    text(0, rr*1.04, max(Z(:),[],'omitnan'), sprintf('%.1f Hz', rr), ...
        'HorizontalAlignment','center','FontSize',8,'Color','k');
end
 
% Direction spokes
rmax = max(f_valid);
for a = 0:30:330
    plot3([0 rmax*sind(a)], [0 rmax*cosd(a)], [0 0], 'k:', 'LineWidth', 0.5);
    text(1.10*rmax*sind(a), 1.10*rmax*cosd(a), 0, sprintf('%d°',a), ...
        'HorizontalAlignment','center','FontSize',9);
end
 
% True direction line
zmax_circ = max(Z(:),[],'omitnan');
plot3([0 rmax*sind(alpha_from_deg)], [0 rmax*cosd(alpha_from_deg)], ...
    [zmax_circ zmax_circ], 'r--', 'LineWidth', 2.0);
 
% Peak ridge overlay
theta_from_peak_deg_circ = nan(length(f_spec),1);
for k = 1:length(f_spec)
    if ~valid_mask(k), continue; end
    [~,ip] = max(S_dir_all(:,k));
    theta_from_peak_deg_circ(k) = alpha_grid_deg(ip);
end
fridge = f_spec(valid_mask);
tridge = theta_from_peak_deg_circ(valid_mask);
x_ridge = fridge .* sind(tridge);
y_ridge = fridge .* cosd(tridge);
plot3(x_ridge, y_ridge, repmat(zmax_circ,size(x_ridge)), ...
    'k.-', 'LineWidth', 2.0, 'MarkerSize', 10);
 
title(sprintf('S(f,\\alpha) circular spectrum  |  true from %.0f°', alpha_from_deg), ...
    'FontSize', 13);
xlabel('East'); ylabel('North');
legend('Spectrum','','','','','','','','','','','','','','True direction','Ridge', ...
    'Location','bestoutside','AutoUpdate','off');
hold off;
 
%% --- Summary printout ---
fprintf('\n=== 3D DIRECTIONAL SPECTRUM SUMMARY ===\n');
fprintf('Valid frequency bins:       %d\n',   sum(valid_mask));
fprintf('Frequency range:            %.3f – %.3f Hz\n', min(f_valid), max(f_valid));
fprintf('Direction resolution:       %.1f deg\n', alpha_grid_deg(2)-alpha_grid_deg(1));
fprintf('Dominant frequency f_p:     %.4f Hz\n', f_eta_peak);
fprintf('True coming-from direction: %.1f deg\n', alpha_from_deg);
[~,ib] = max(S_band);
fprintf('Band-integrated peak dir:   %.1f deg\n', alpha_grid_deg(ib));
fprintf('Peak error:                 %.2f deg\n', ...
    mod(alpha_grid_deg(ib)-alpha_from_deg+180,360)-180);
fprintf('S(f,alpha) peak value:      %.4e m^2/Hz/rad\n', max(S_mesh(:),[],'omitnan'));
fprintf('========================================\n');
 