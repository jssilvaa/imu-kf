import argparse
import sys
import time
import serial

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", required=True, help="output CSV path")
    ap.add_argument("--seconds", type=float, default=0.0, help="0 => until Ctrl+C")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    t0 = time.time()

    with open(args.out, "w", newline="") as f:
        while True:
            if args.seconds > 0 and (time.time() - t0) >= args.seconds:
                break
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            # Keep header lines + data; skip comment lines starting with '#'
            if line.startswith("#"):
                continue

            # Write exactly what comes (already CSV)
            f.write(line + "\n")
            f.flush()

    ser.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)

# run likee:
#python python/log_serial.py --port COM5 --out data/raw/run1.csv
