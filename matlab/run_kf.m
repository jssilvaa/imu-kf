%% Load CSV + calib.json, run KF, plot, RMSE

T = readtable("../data/raw/run1.csv");
t = (T.t_us - T.t_us(1)) * 1e-6;
dt = median(diff(t));

% Load calib.json
cal = jsondecode(fileread("../data/calib/calib.json"));

ax = double(T.ax_raw); ay = double(T.ay_raw); az = double(T.az_raw);
gx = double(T.gx_raw); gy = double(T.gy_raw);

% Accel 6-face correction: a_g = (raw - b)/s
bx = cal.acc.bx; by = cal.acc.by; bz = cal.acc.bz;
sx = cal.acc.sx; sy = cal.acc.sy; sz = cal.acc.sz;

ax_g = (ax - bx) / sx;
ay_g = (ay - by) / sy;
az_g = (az - bz) / sz;

% Gyro bias raw then nominal sensitivity (±250 dps)
bgx = cal.gyro.bgx; bgy = cal.gyro.bgy;
GYRO_LSB_PER_DPS = 131.0;

gx_dps = (gx - bgx) / GYRO_LSB_PER_DPS;
gy_dps = (gy - bgy) / GYRO_LSB_PER_DPS;

gx = deg2rad(gx_dps);
gy = deg2rad(gy_dps);

% Accel tilt (rad)
z_roll  = atan2(ay_g, az_g);
z_pitch = atan2(-ax_g, sqrt(ay_g.^2 + az_g.^2));

% Static gating
a_norm = sqrt(ax_g.^2 + ay_g.^2 + az_g.^2);
static_ok = abs(a_norm - 1.0) < 0.10;

% Protocol truth from phase (if present)
truth_pitch = zeros(size(t));
truth_roll  = zeros(size(t));
if any(strcmp("phase", T.Properties.VariableNames)) && any(strcmp("mode", T.Properties.VariableNames))
    mprot = (T.mode == 2);
    truth_pitch(mprot & (T.phase == 1)) = pi/2;
    truth_roll(mprot  & (T.phase == 1)) = pi/2;
end

% Evaluate on stationary protocol samples
eval_mask = static_ok;
if any(strcmp("mode", T.Properties.VariableNames))
    eval_mask = eval_mask & (T.mode == 2);
end

% Initial R from stationary variance (pitch)
r_meas = var(z_pitch(eval_mask));

% Tunables
q_theta = 1e-5;
q_bias  = 1e-7;

[hat_pitch, b_pitch] = kf_angle_bias(gx, z_pitch, dt, q_theta, q_bias, r_meas, static_ok);
[hat_roll,  b_roll ] = kf_angle_bias(gy, z_roll,  dt, q_theta, q_bias, r_meas, static_ok);

% RMSE
e = hat_pitch(eval_mask) - truth_pitch(eval_mask);
rmse_deg = sqrt(mean(e.^2)) * 180/pi;
fprintf("Pitch RMSE (stationary protocol samples): %.3f deg\n", rmse_deg);

% Plots
figure; plot(t, rad2deg(z_pitch), t, rad2deg(hat_pitch), t, rad2deg(truth_pitch)); grid on;
legend("Accel meas", "KF est", "Truth"); xlabel("t [s]"); ylabel("Pitch [deg]");

figure; plot(t, rad2deg(z_roll), t, rad2deg(hat_roll), t, rad2deg(truth_roll)); grid on;
legend("Accel meas", "KF est", "Truth"); xlabel("t [s]"); ylabel("Roll [deg]");

% this loads data runs kf and plots the results 