# IMU Pitch/Roll Estimation with MPU6050 + Kalman Filter

This project implements a pitch/roll estimator for an MPU6050 IMU using a **2-state Kalman filter (angle + gyro bias)**. The workflow is:
1) acquire raw IMU data with a controlled motion protocol (must be set beforehand, try different protocols),
2) calibrate gyro bias + accel bias/scale,
3) tune and validate the Kalman filter in MATLAB, i.e. check the correctness of the filter 
4) deploy the same KF on Arduino and connect to the provided GUI (`carRacing.py`).
5) check that the controller is good for the game!

## Hardware
- MPU6050 (I2C)
- Arduino (UNO/Nano/MEGA, etc.)

## Sensor configuration 
The following options are selected with FS_SEL = 0: 
- Accelerometer range: ±2g (16384 LSB/g) 
- Gyro range: ±250 dps (131 LSB/(°/s))
The sample rate is set with DLPF and SMPLRT_DIV = 9 
- Sample rate: 100 Hz (fixed dt = 0.01 s)
- DLPF enabled (reduce accel jitter)

## Repository layout
- `arduino/imu_logger/` : logs raw accel/gyro + protocol phases to Serial
- `arduino/imu_kf/` : implements the KF + sends serial output for GUI
- `matlab/` : offline KF + tuning + plots (mainly for analysis)
- `python/` : optional, bluetooth serial logger + calibration + offline KF
- `data/` : raw logs, calibration outputs
- `results/` : figures and metrics for reporting

## Quick start

### 1) Acquire raw data (Arduino)
Flash `arduino/imu_logger/imu_logger.ino`.
- Keep IMU still for 10 s (gyro bias + accel baseline).
- Run the protocol: wait 2 s → rotate 90° → wait 2 s → rotate 90° → wait 2 s.
- Save the Serial output as CSV into `data/raw/`.

### 2) Calibrate
Option A (recommended): run 6-face accel calibration and export `data/calib/calib.json`.

Option B (minimum): use flat (+Z) + upside-down (−Z) to get z scale/bias.

### 3) Run KF in MATLAB
Open `matlab/run_kf.m`:
- load CSV
- apply calibration
- compute accel tilt measurement z(t)
- run KF for pitch and roll
- generate plots and RMSE vs protocol truth
If calibration is off, go over the calibration protocol once more. 

### 4) Deploy realtime KF (Arduino)
Flash `arduino/imu_kf/imu_kf.ino`.
Output pitch/roll over Serial in the format expected by the GUI.

## Notes
- Pitch/roll are observable from gravity; **yaw is not** without magnetometer.
- Accelerometer angles are unreliable under linear acceleration; the KF gates accel updates using ‖a‖ ≈ 1g.

See `docs.md` for methodology, equations, calibration, and tuning procedure.
