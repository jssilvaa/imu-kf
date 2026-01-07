import argparse
import threading
import time
import serial
import numpy as np
import gymnasium as gym

# Shared state (updated by serial thread)
_lock = threading.Lock()
_roll_deg = 0.0
_pitch_deg = 0.0
_running = True


def serial_reader(port: str, baud: int):
    global _roll_deg, _pitch_deg, _running

    ser = serial.Serial(port, baudrate=baud, timeout=1)
    time.sleep(2.0)  # allow Arduino reset on connect

    # Trigger calibration on Arduino (10 s). Arduino replies with "1" then starts streaming.
    ser.write(b'a')

    t_start = time.time()
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line == "1":
            break
        # avoid infinite hang if something’s wrong
        if time.time() - t_start > 20.0:
            raise RuntimeError("Calibration handshake timed out (expected '1'). Check baud/port/firmware.")

    # Stream reading loop
    while _running:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        # Expected: "roll_deg,pitch_deg"
        try:
            parts = line.split(",")
            if len(parts) != 2:
                continue
            r = float(parts[0])
            p = float(parts[1])
        except ValueError:
            continue

        with _lock:
            _roll_deg = r
            _pitch_deg = p

    ser.close()


def controller_interpreter(roll_deg: float, pitch_deg: float):
    """
    Maps roll/pitch to CarRacing controls.
    a[0] steer in [-1,1], a[1] gas in [0,1], a[2] brake in [0,1]
    """
    a = np.array([0.0, 0.0, 0.0], dtype=np.float32)

    # ---- Gas / Brake from pitch ----
    # Keep your thresholds, but clamp to [0,1]
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

    # ---- Steering from roll ----
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


def main():
    global _running

    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. COM3 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    # Start reader thread
    th = threading.Thread(target=serial_reader, args=(args.port, args.baud), daemon=True)
    th.start()

    env = gym.make("CarRacing-v2", render_mode="human")
    env.reset()

    try:
        while True:
            with _lock:
                r = _roll_deg
                p = _pitch_deg

            a = controller_interpreter(r, p)

            _, _, terminated, truncated, _ = env.step(a)
            if terminated or truncated:
                env.reset()
    except KeyboardInterrupt:
        pass
    finally:
        _running = False
        time.sleep(0.2)
        env.close()


if __name__ == "__main__":
    main()

# run like python python/car_racing_realtime.py --port COM3 --baud 115200