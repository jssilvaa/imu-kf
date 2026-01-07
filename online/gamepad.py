import argparse
import threading
import time
from collections import deque

import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# -----------------------------
# Control mapping (same logic)
# -----------------------------
def controller_interpreter(roll_deg: float, pitch_deg: float):
    """
    Maps roll/pitch to controls:
      a[0] steer in [-1,1]
      a[1] gas   in [0,1]
      a[2] brake in [0,1]
    """
    a = np.array([0.0, 0.0, 0.0], dtype=np.float32)

    # Gas / Brake from pitch
    if -8.0 < pitch_deg < 8.0:
        gas = 0.0
        brk = 0.0
    elif 8.0 <= pitch_deg < 15.0:
        gas = 0.15
        brk = 0.0
    elif 15.0 <= pitch_deg < 40.0:
        gas = 0.35
        brk = 0.0
    elif pitch_deg >= 40.0:
        gas = 0.60
        brk = 0.0
    elif -8.0 <= pitch_deg < 0.0:
        gas = 0.0
        brk = 0.15
    elif -40.0 <= pitch_deg < -15.0:
        gas = 0.0
        brk = 0.35
    else:  # pitch_deg < -40
        gas = 0.0
        brk = 0.60

    # Steering from roll
    if -15.0 <= roll_deg <= 15.0:
        steer = 0.0
    elif roll_deg < -15.0:
        steer = -0.5
    else:
        steer = +0.5

    a[0] = float(np.clip(steer, -1.0, 1.0))
    a[1] = float(np.clip(gas,   0.0, 1.0))
    a[2] = float(np.clip(brk,   0.0, 1.0))
    return a


# -----------------------------
# Serial worker
# -----------------------------
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = True
        self.status = "DISCONNECTED"   # DISCONNECTED / CALIBRATING / STREAMING / ERROR
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.last_line = ""
        self.t_stream_start = None
        self.err_msg = ""
        self.csv_f = None

    def set_csv(self, path: str):
        if path:
            self.csv_f = open(path, "w", newline="")
            self.csv_f.write("t_s,roll_deg,pitch_deg,steer,gas,brake,raw_line\n")
            self.csv_f.flush()

    def close(self):
        if self.csv_f:
            self.csv_f.close()
            self.csv_f = None


def serial_reader(port: str, baud: int, shared: SharedState, calib_timeout_s: float = 25.0):
    ser = None
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=1)
        time.sleep(2.0)  # Arduino reset on connect

        with shared.lock:
            shared.status = "CALIBRATING"
            shared.err_msg = ""

        # Trigger calibration (10s) -> Arduino prints "1" then starts streaming roll,pitch
        ser.write(b'a')

        t0 = time.time()
        while shared.running:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                if time.time() - t0 > calib_timeout_s:
                    raise RuntimeError("Calibration handshake timed out (expected '1').")
                continue
            if line == "1":
                break

        with shared.lock:
            shared.status = "STREAMING"
            shared.t_stream_start = time.time()

        # Streaming loop
        while shared.running:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) != 2:
                continue

            try:
                r = float(parts[0])
                p = float(parts[1])
            except ValueError:
                continue

            a = controller_interpreter(r, p)

            with shared.lock:
                shared.roll_deg = r
                shared.pitch_deg = p
                shared.last_line = line

                if shared.csv_f is not None and shared.t_stream_start is not None:
                    t_s = time.time() - shared.t_stream_start
                    shared.csv_f.write(f"{t_s:.6f},{r:.6f},{p:.6f},{a[0]:.6f},{a[1]:.6f},{a[2]:.6f},\"{line}\"\n")
                    shared.csv_f.flush()

    except Exception as e:
        with shared.lock:
            shared.status = "ERROR"
            shared.err_msg = str(e)
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


# -----------------------------
# GUI (matplotlib)
# -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. COM3 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--history_s", type=float, default=10.0, help="seconds shown in the scrolling plot")
    ap.add_argument("--fps", type=int, default=30, help="GUI update rate")
    ap.add_argument("--csv", default="", help="optional: log to CSV (e.g. data/raw/monitor.csv)")
    args = ap.parse_args()

    shared = SharedState()
    shared.set_csv(args.csv)

    th = threading.Thread(target=serial_reader, args=(args.port, args.baud, shared), daemon=True)
    th.start()

    # Ring buffers for plot history
    hist_n = max(10, int(args.history_s * args.fps))
    t_buf = deque(maxlen=hist_n)
    r_buf = deque(maxlen=hist_n)
    p_buf = deque(maxlen=hist_n)

    t0_gui = time.time()

    # Figure layout
    fig = plt.figure(figsize=(10, 6))
    fig.canvas.manager.set_window_title("IMU Gamepad Monitor (roll/pitch + controls)")

    ax1 = plt.subplot2grid((2, 2), (0, 0), colspan=2)  # time series
    ax2 = plt.subplot2grid((2, 2), (1, 0))            # bars
    ax3 = plt.subplot2grid((2, 2), (1, 1))            # text panel
    ax3.axis("off")

    # Time-series lines
    (line_r,) = ax1.plot([], [], label="roll [deg]")
    (line_p,) = ax1.plot([], [], label="pitch [deg]")
    ax1.set_xlabel("t [s]")
    ax1.set_ylabel("angle [deg]")
    ax1.grid(True)
    ax1.legend(loc="upper right")

    # Bars for steer/gas/brake
    labels = ["steer", "gas", "brake"]
    x = np.arange(len(labels))
    bars = ax2.bar(x, [0, 0, 0])
    ax2.set_xticks(x, labels)
    ax2.set_ylim(-1.05, 1.05)
    ax2.grid(True, axis="y")

    # Text panel
    text_status = ax3.text(0.02, 0.90, "", fontsize=12, transform=ax3.transAxes)
    text_angles = ax3.text(0.02, 0.68, "", fontsize=12, transform=ax3.transAxes)
    text_raw    = ax3.text(0.02, 0.46, "", fontsize=10, transform=ax3.transAxes)
    text_hint   = ax3.text(0.02, 0.18,
                           "Hints:\n- If STREAMING but angles jump: check axis/sign mapping.\n"
                           "- If always steer/gas/brake nonzero: deadbands too tight.\n"
                           "- If calibration times out: check port/baud/firmware.\n",
                           fontsize=9, transform=ax3.transAxes)

    def update(_):
        # Read shared
        with shared.lock:
            status = shared.status
            err = shared.err_msg
            r = shared.roll_deg
            p = shared.pitch_deg
            raw = shared.last_line
            t_stream_start = shared.t_stream_start

        # Time base
        if t_stream_start is None:
            t_s = time.time() - t0_gui
        else:
            t_s = time.time() - t_stream_start

        # Update buffers only if streaming
        if status == "STREAMING":
            t_buf.append(t_s)
            r_buf.append(r)
            p_buf.append(p)

        # Update time series plot
        if len(t_buf) >= 2:
            tt = np.array(t_buf)
            rr = np.array(r_buf)
            pp = np.array(p_buf)
            line_r.set_data(tt, rr)
            line_p.set_data(tt, pp)

            # Autoscale x to current window; y keep reasonable
            ax1.set_xlim(max(0.0, tt[-1] - args.history_s), tt[-1] + 1e-6)
            y_min = float(min(rr.min(), pp.min(), -5.0))
            y_max = float(max(rr.max(), pp.max(),  5.0))
            pad = 0.10 * max(1.0, y_max - y_min)
            ax1.set_ylim(y_min - pad, y_max + pad)

        # Compute control bars from current angles (even if not streaming yet)
        a = controller_interpreter(r, p)
        steer, gas, brk = float(a[0]), float(a[1]), float(a[2])

        # Bar heights
        bars[0].set_height(steer)
        bars[1].set_height(gas)
        bars[2].set_height(brk)

        # Text panel
        if status == "ERROR":
            text_status.set_text(f"Status: ERROR\n{err}")
        else:
            text_status.set_text(f"Status: {status}")

        text_angles.set_text(f"roll  = {r:+7.3f} deg\npitch = {p:+7.3f} deg\n\n"
                             f"steer = {steer:+.3f}\n"
                             f"gas   = {gas:+.3f}\n"
                             f"brake = {brk:+.3f}")

        # Raw line (for debugging parsing / baud issues)
        text_raw.set_text(f"last raw: {raw[:60]}" + ("..." if len(raw) > 60 else ""))

        return line_r, line_p, *bars, text_status, text_angles, text_raw

    ani = FuncAnimation(fig, update, interval=int(1000 / max(5, args.fps)), blit=False)

    try:
        plt.show()
    finally:
        # stop thread
        with shared.lock:
            shared.running = False
        time.sleep(0.2)
        shared.close()


if __name__ == "__main__":
    main()

# pip install pyserial numpy matplotlib
# python online/gamepad_gui.py --port COM3 --baud 115200
# if logging is needed add --csv data/raw/monitor.csv