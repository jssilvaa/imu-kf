# docs.md — Methodology and Implementation Notes

## 1) Problem and scope
We estimate **pitch** and **roll** from an MPU6050 using IMU sensor fusion.
- Gyroscope provides angular rate (smooth, drifts).
- Accelerometer provides gravity direction (bounded, noisy; corrupted by linear acceleration).
We fuse them with a **Kalman filter** to obtain stable, low-drift angle estimates.

Yaw is excluded (not observable without magnetometer).

---

## 2) Data acquisition (Arduino)
### 2.1 What is logged
Each sample should include:
- timestamp (µs)
- raw accel: ax_raw, ay_raw, az_raw (int16)
- raw gyro:  gx_raw, gy_raw, gz_raw (int16)
- protocol phase label (optional but recommended)
- accel norm and a `static_ok` flag for gating
If you use a different protocol, make sure to label accordingly.

### 2.2 Sampling
Run at fixed rate (e.g. 100 Hz) using `micros()` scheduling.
This makes dt constant and removes timing ambiguity from filter design.

### 2.3 Motion protocol (use easily known angles)
Per axis run (pitch-only or roll-only):
- wait 2 s at 0°
- rotate to 90° (transition)
- wait 2 s at 90°
- rotate to 0° (transition)
- wait 2 s at 0°

---

## 3) Sensor conversion (units)
Assuming we are using accel ±2g and gyro ±250 dps:
- accel sensitivity: 16384 LSB/g
- gyro sensitivity: 131 LSB/(°/s)

Convert:
a[g] = (a_raw - b_a)/16384
w[rad/s] = deg2rad((w_raw - b_w)/131)

---

## 4) Calibration
### 4.1 Gyro bias
Record 10 s stationary.
b_w = mean(w_raw) per axis.
Subtract online and offline.

### 4.2 Accelerometer bias/scale
Bias-only calibration is insufficient for rigor because it cannot separate scale from offset.
Recommended: 6-face calibration (+X,−X,+Y,−Y,+Z,−Z).
For each axis i:
b_i = (a_{i,+} + a_{i,-})/2
s_i = (a_{i,+} - a_{i,-})/(2g)
Correct:
a_i[g] = (a_i_raw - b_i)/s_i

Minimum upgrade if time-constrained: add −Z pose (upside-down) so z scale/bias is correct.

---

## 5) Measurement model (accelerometer tilt)
Angles are computed in radians:

Roll:
z_phi = atan2(a_y, a_z)

Pitch:
z_theta = atan2(-a_x, sqrt(a_y^2 + a_z^2))

These are valid **when linear acceleration is small** (approx. a ≈ g).

---

## 6) Kalman filter model (correct 2-state)
We run one independent KF for pitch and one for roll.

State:
x = [theta; b]  (angle + gyro bias)

Input:
u = w_meas (gyro rate)

Dynamics:
theta_k = theta_{k-1} + dt*(u_k - b_{k-1}) + w_theta
b_k     = b_{k-1} + w_b

Matrix form:
A = [1, -dt; 0, 1]
B = [dt; 0]
H = [1, 0]

Noise:
Q = diag([q_theta, q_bias])
R = r_meas

---

## 7) Gating / robustness
Accelerometer measurements are gated using:
a_norm = ||a[g]||
static_ok = |a_norm - 1| < eps

If static_ok == 0:
- skip measurement update, OR
- inflate R by a large factor (e.g. R <- 1000*R)

This prevents tilt estimates from being corrupted during motion.

---

## 8) Tuning and evaluation (MATLAB first)
1) Estimate measurement variance:
R ≈ Var(z_theta) (on stationary windows)

2) Estimate gyro noise from stationary gyro:
sigma_w = std(w_meas)

Initialize:
q_theta ~ (sigma_w*dt)^2
q_bias  << q_theta (bias random walk, slow)

3) Optimize by minimizing mean squared error against protocol truth on stationary windows.
Report RMSE and include plots:
- accel measurement vs KF estimate vs truth
- bias estimate evolution

---

## 9) Deployment (Arduino + GUI)
Port the same 2-state KF.
- Maintain fixed dt (100 Hz).
- Use radians internally.
- Output roll/pitch in degrees over Serial in the exact format expected by `carRacing.py`.
- Verify COM port and baud rate match the GUI script.
