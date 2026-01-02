import argparse
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def kf_angle_bias(omega_rad_s, z_rad, dt, q_theta, q_bias, r_meas, static_ok):
    """
    2-state KF for angle + gyro bias.
    x = [theta, b]^T
    theta_k = theta_{k-1} + dt*(omega - b) + w_theta
    b_k = b_{k-1} + w_b
    z = theta + v
    """
    N = len(z_rad)
    theta_hat = np.zeros(N)
    bias_hat  = np.zeros(N)

    x = np.array([z_rad[0], 0.0], dtype=float)
    P = np.eye(2)

    A = np.array([[1.0, -dt],
                  [0.0,  1.0]])
    B = np.array([dt, 0.0])
    H = np.array([1.0, 0.0])

    Q = np.diag([q_theta, q_bias])
    R = r_meas

    for k in range(1, N):
        # Predict
        x = A @ x + B * omega_rad_s[k]
        P = A @ P @ A.T + Q

        # Update (gated)
        if static_ok[k]:
            y = z_rad[k] - (H @ x)
            S = (H @ P @ H.T) + R
            K = (P @ H.T) / S
            x = x + K * y
            P = (np.eye(2) - np.outer(K, H)) @ P

        theta_hat[k] = x[0]
        bias_hat[k]  = x[1]

    return theta_hat, bias_hat

def accel_tilt(ax_g, ay_g, az_g):
    # roll = atan2(ay, az)
    roll  = np.arctan2(ay_g, az_g)
    # pitch = atan2(-ax, sqrt(ay^2+az^2))
    pitch = np.arctan2(-ax_g, np.sqrt(ay_g**2 + az_g**2))
    return roll, pitch

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--calib", required=True)
    ap.add_argument("--axis", choices=["pitch","roll"], default="pitch",
                    help="which axis was excited in the protocol (for truth + RMSE)")
    ap.add_argument("--dt_override", type=float, default=0.0)
    ap.add_argument("--eps_static", type=float, default=0.10, help="| ||a|| - 1 | < eps")
    ap.add_argument("--q_theta", type=float, default=1e-5)
    ap.add_argument("--q_bias", type=float, default=1e-7)
    ap.add_argument("--r_scale", type=float, default=1.0, help="scale R estimate (tuning knob)")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    t = (df["t_us"].to_numpy() - df["t_us"].iloc[0]) * 1e-6
    dt = float(np.median(np.diff(t))) if args.dt_override <= 0 else args.dt_override

    with open(args.calib, "r") as f:
        calib = json.load(f)

    # Raw
    ax = df["ax_raw"].to_numpy(dtype=float)
    ay = df["ay_raw"].to_numpy(dtype=float)
    az = df["az_raw"].to_numpy(dtype=float)
    gx = df["gx_raw"].to_numpy(dtype=float)
    gy = df["gy_raw"].to_numpy(dtype=float)
    gz = df["gz_raw"].to_numpy(dtype=float)

    # Apply accel 6-face calibration: a_g = (raw - b)/s
    bx, by, bz = calib["acc"]["bx"], calib["acc"]["by"], calib["acc"]["bz"]
    sx, sy, sz = calib["acc"]["sx"], calib["acc"]["sy"], calib["acc"]["sz"]

    ax_g = (ax - bx) / sx
    ay_g = (ay - by) / sy
    az_g = (az - bz) / sz

    # Gyro bias from calib (raw), then scale by nominal sensitivity (±250 dps)
    bgx, bgy, bgz = calib["gyro"]["bgx"], calib["gyro"]["bgy"], calib["gyro"]["bgz"]
    GYRO_LSB_PER_DPS = 131.0

    gx_dps = (gx - bgx) / GYRO_LSB_PER_DPS
    gy_dps = (gy - bgy) / GYRO_LSB_PER_DPS

    gx_rad = np.deg2rad(gx_dps)
    gy_rad = np.deg2rad(gy_dps)

    # Measurements from accel
    roll_z, pitch_z = accel_tilt(ax_g, ay_g, az_g)

    # Gating
    a_norm = np.sqrt(ax_g**2 + ay_g**2 + az_g**2)
    static_ok = np.abs(a_norm - 1.0) < args.eps_static

    # Protocol truth from phase (if present)
    truth_pitch = np.zeros_like(t)
    truth_roll  = np.zeros_like(t)

    if "phase" in df.columns and "mode" in df.columns:
        # mode==2 means protocol in our Arduino sketch
        mprot = (df["mode"].to_numpy() == 2)
        ph = df["phase"].to_numpy()
        # default: 0deg (phase0), 90deg (phase1), 0deg (phase2)
        truth_pitch[(mprot) & (ph == 1)] = np.pi/2
        truth_roll[(mprot) & (ph == 1)]  = np.pi/2

    # Evaluate only on stationary samples in protocol window
    eval_mask = static_ok.copy()
    if "mode" in df.columns:
        eval_mask &= (df["mode"].to_numpy() == 2)

    # Estimate R from stationary measurement variance, then scale
    z_for_R = pitch_z[eval_mask] if args.axis == "pitch" else roll_z[eval_mask]
    if z_for_R.size < 50:
        raise RuntimeError("Not enough stationary protocol samples to estimate R. Check your logging/gating.")
    R = float(np.var(z_for_R)) * args.r_scale

    # Run KF
    if args.axis == "pitch":
        theta_hat, bias_hat = kf_angle_bias(gx_rad, pitch_z, dt, args.q_theta, args.q_bias, R, static_ok)
        truth = truth_pitch
        z = pitch_z
        title = "Pitch"
    else:
        theta_hat, bias_hat = kf_angle_bias(gy_rad, roll_z, dt, args.q_theta, args.q_bias, R, static_ok)
        truth = truth_roll
        z = roll_z
        title = "Roll"

    # RMSE on eval mask (truth defined by protocol)
    e = theta_hat[eval_mask] - truth[eval_mask]
    rmse_deg = np.rad2deg(np.sqrt(np.mean(e**2)))
    print(f"{title} RMSE (stationary protocol samples): {rmse_deg:.3f} deg")
    print(f"dt={dt:.6f}s, R={R:.3e}, q_theta={args.q_theta:.3e}, q_bias={args.q_bias:.3e}")

    # Plots
    plt.figure()
    plt.plot(t, np.rad2deg(z), label="accel meas")
    plt.plot(t, np.rad2deg(theta_hat), label="KF estimate")
    plt.plot(t, np.rad2deg(truth), label="truth")
    plt.grid(True)
    plt.xlabel("t [s]")
    plt.ylabel(f"{title} [deg]")
    plt.legend()

    plt.figure()
    plt.plot(t, bias_hat, label="bias estimate [rad/s]")
    plt.grid(True)
    plt.xlabel("t [s]")
    plt.ylabel("gyro bias estimate [rad/s]")
    plt.legend()

    plt.show()

if __name__ == "__main__":
    main()

# runs the kf on python, can also do this l8er in matlab 
# run like:
# python python/run_kf.py --csv data/raw/run1.csv --calib data/calib/calib.json --axis pitch
