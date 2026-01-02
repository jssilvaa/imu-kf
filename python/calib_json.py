import argparse
import json
import numpy as np
import pandas as pd

POSES = ["XP","XN","YP","YN","ZP","ZN"]

def mean_pose(df, pose):
    d = df[df["pose"] == pose]
    if len(d) == 0:
        raise RuntimeError(f"Missing pose {pose} in data")
    return d[["ax_raw","ay_raw","az_raw"]].mean().to_numpy()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--out", default="data/calib/calib.json")
    ap.add_argument("--calib_seconds", type=float, default=10.0,
                    help="duration of initial flat calibration segment")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)

    # Time in seconds from t_us
    t = (df["t_us"].to_numpy() - df["t_us"].iloc[0]) * 1e-6

    # ----- Gyro bias from initial stationary interval -----
    m_cal = t <= args.calib_seconds
    if not np.any(m_cal):
        raise RuntimeError("No samples in initial calibration window. Did you log from power-on?")

    gyro_bias = df.loc[m_cal, ["gx_raw","gy_raw","gz_raw"]].mean().to_numpy()
    gyro_std  = df.loc[m_cal, ["gx_raw","gy_raw","gz_raw"]].std().to_numpy()

    # ----- Accel 6-face bias+scale -----
    # For axis i, use the reading of that axis in its + and - pose.
    # Means are raw counts.
    m_xp = mean_pose(df, "XP")[0]
    m_xn = mean_pose(df, "XN")[0]
    m_yp = mean_pose(df, "YP")[1]
    m_yn = mean_pose(df, "YN")[1]
    m_zp = mean_pose(df, "ZP")[2]
    m_zn = mean_pose(df, "ZN")[2]

    bx = 0.5 * (m_xp + m_xn)
    sx = 0.5 * (m_xp - m_xn)  # LSB per +1g
    by = 0.5 * (m_yp + m_yn)
    sy = 0.5 * (m_yp - m_yn)
    bz = 0.5 * (m_zp + m_zn)
    sz = 0.5 * (m_zp - m_zn)

    calib = {
        "acc": {"bx": float(bx), "by": float(by), "bz": float(bz),
                "sx": float(sx), "sy": float(sy), "sz": float(sz)},
        "gyro": {"bgx": float(gyro_bias[0]), "bgy": float(gyro_bias[1]), "bgz": float(gyro_bias[2]),
                 "std_gx": float(gyro_std[0]), "std_gy": float(gyro_std[1]), "std_gz": float(gyro_std[2])},
        "notes": {
            "acc_model": "a_g = (a_raw - b) / s  (s in LSB/g)",
            "gyro_model": "w_raw_bias_subtracted then scaled by nominal sensitivity (set in run script)"
        }
    }

    with open(args.out, "w") as f:
        json.dump(calib, f, indent=2)

    print(f"Wrote {args.out}")
    print("Accel params (LSB/g):", calib["acc"])
    print("Gyro bias (raw):", calib["gyro"]["bgx"], calib["gyro"]["bgy"], calib["gyro"]["bgz"])

if __name__ == "__main__":
    main()

# run like: 
# python python/calib_json.py --csv data/raw/run1.csv --out data/calib/calib.json
