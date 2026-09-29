# runtime/main.py (rewrite: fixed imports, correct init order, PID JSON only, live debug toggles)

import threading
import time
import cv2
import signal
import numpy as np
import os
import json
from pathlib import Path
from dataclasses import dataclass

from .camera import Camera
from .controller import RobotController
from .robotKinematics import RobotKinematics
from .PID import PIDcontroller
from .debug_overlay import draw_legend

#-----------------------------
# FOR HEADLESS
#-----------------------------
HEADLESS = (os.environ.get("DISPLAY") is None) or (os.environ.get("SSH_CONNECTION") is not None)
# Optional override:
# export BALLBOT_HEADLESS=0 to force GUI
if os.environ.get("BALLBOT_HEADLESS") == "0":
    HEADLESS = False
if os.environ.get("BALLBOT_HEADLESS") == "1":
    HEADLESS = True

# ----------------------------
# Debug flags + monitor
# ----------------------------

@dataclass
class DebugFlags:
    master: bool = False
    cam: bool = False
    cam_mask: bool = False
    ctrl: bool = False
    pid: bool = False
    help: bool = False
    overlay: bool = True


class DebugMonitor:
    def __init__(self):
        self.last_loop_print = 0.0
        self.last_pid_print = 0.0
        self.last_valid = None

    def loop_update(self, *, valid, x, y, elapsed_ms, sleep_ms, period_s=2.0):
        now = time.time()

        # Print immediately on validity changes
        if self.last_valid is None or valid != self.last_valid:
            print(f"[VISION] valid={valid} x={x} y={y}", flush=True)
            self.last_valid = valid
            self.last_loop_print = now
            return

        # Periodic heartbeat
        if now - self.last_loop_print >= period_s:
            print(
                f"[LOOP] valid={valid} x={x} y={y} elapsed={elapsed_ms:.1f}ms sleep={sleep_ms:.1f}ms",
                flush=True,
            )
            self.last_loop_print = now

    def pid_update(self, *, theta, phi, valid, period_s=0.5):
        now = time.time()
        if now - self.last_pid_print >= period_s:
            print(f"[PID] theta={theta:.2f} phi={phi:.1f} valid={valid}", flush=True)
            self.last_pid_print = now


dbg = DebugFlags()
mon = DebugMonitor()


# ----------------------------
# PID config (JSON) loader
# ----------------------------

REPO_ROOT = Path(__file__).resolve().parents[3]
PID_DEFAULTS = {"kp": 0.0063, "ki": 0.00005, "kd": 0.006025, "alpha": 0.65, "beta": 0.3}
PID_PATH = os.getenv("BALLBOT_PID", str(REPO_ROOT / "config" / "pid.json"))


def load_pid(path: str, defaults: dict) -> dict:
    p = Path(path)
    if not p.exists():
        print(f"[PID] file not found: {p} — using defaults", flush=True)
        return defaults.copy()
    try:
        data = json.loads(p.read_text())
        out = defaults.copy()
        for k in defaults.keys():
            if k in data:
                out[k] = float(data[k])
        return out
    except Exception as e:
        print(f"[PID] failed to read {p}: {e} — using defaults", flush=True)
        return defaults.copy()


# ----------------------------
# Initialize subsystems (order matters)
# ----------------------------

cam = Camera()
model = RobotKinematics()
robot = RobotController(model)   # calibration.json auto-loads inside controller
robot.arm()

pid_cfg = load_pid(PID_PATH, PID_DEFAULTS)
kp, ki, kd = pid_cfg["kp"], pid_cfg["ki"], pid_cfg["kd"]
alpha, beta = pid_cfg["alpha"], pid_cfg["beta"]

print(f"[PID] kp={kp} ki={ki} kd={kd} alpha={alpha} beta={beta}", flush=True)

PID = PIDcontroller(kp, ki, kd, alpha, beta, max_theta=model.maxtheta, conversion="tanh")

# Ensure subsystems start quiet (foreground keys toggle these)
cam.debug = False
cam.debug_mask = False
robot.debug = False


# ----------------------------
# Shared state / threads
# ----------------------------

latest_frame = None
frame_lock = threading.Lock()

preview_frame = None
preview_lock = threading.Lock()

running = True
mask_windows_open = False

# State
x, y = 100, 75
x_t, y_t = (100, 75)


def update_robot_pos(x_t, y_t, x, y, valid):
    theta, phi = PID.pid((x_t, y_t), (x, y), valid=valid)
    robot.Goto_N_time_spherical(theta, phi, 8.26)

    if dbg.master and dbg.pid:
        mon.pid_update(theta=theta, phi=phi, valid=valid)


def capture():
    global latest_frame
    hz = 60
    period = 1.0 / hz

    while running:
        t0 = time.perf_counter()
        frame = cam.take_picture()
        with frame_lock:
            latest_frame = frame
        dt = time.perf_counter() - t0
        sleep_time = period - dt
        if sleep_time > 0:
            time.sleep(sleep_time)


def process():
    global x, y, preview_frame
    hz = 50
    period = 1.0 / hz
    preview_div = 5  # ~10 fps preview
    count = 0

    while running:
        loop_start = time.perf_counter()

        with frame_lock:
            frame_copy = None if latest_frame is None else latest_frame.copy()

        if frame_copy is None:
            time.sleep(0.001)
            continue

        # Vision
        x, y, valid = cam.coordinate(frame_copy)

        # Control
        update_robot_pos(x_t, y_t, x, y, valid)

        elapsed = time.perf_counter() - loop_start
        sleep_time = period - elapsed

        # Preview handoff
        count += 1
        if count % preview_div == 0:
            with preview_lock:
                preview_frame = frame_copy.copy()

        # Loop debug
        if dbg.master and dbg.cam:
            mon.loop_update(
                valid=valid,
                x=x, y=y,
                elapsed_ms=elapsed * 1000,
                sleep_ms=max(sleep_time, 0) * 1000,
                period_s=2.0,
            )

        if sleep_time > 0:
            time.sleep(sleep_time)


# ----------------------------
# Shutdown handling
# ----------------------------

def _request_shutdown(signum, frame):
    global running
    print(f"\n[INFO] Signal {signum} received. Shutting down...", flush=True)
    running = False


signal.signal(signal.SIGTERM, _request_shutdown)
signal.signal(signal.SIGINT, _request_shutdown)


# ----------------------------
# Run
# ----------------------------

threading.Thread(target=capture, daemon=True).start()
threading.Thread(target=process, daemon=True).start()
time.sleep(0.5)

print("=" * 60)
print("[INFO] Robot running. Keys: q=quit  t=master  h=help  o=overlay", flush=True)
print("[INFO] When master ON: c=camera  m=mask  r=controller  p=pid", flush=True)
print("=" * 60, flush=True)

try:
    while running:
        # --- Only do OpenCV preview + key polling if GUI is available ---
        if not HEADLESS:
            with preview_lock:
                pf = None if preview_frame is None else preview_frame.copy()

            if pf is not None:
                cam.display_draw(pf, (x, y))
                if dbg.overlay and (dbg.master or dbg.help):
                    draw_legend(pf, dbg)
                cv2.imshow("Tracked Output", pf)

            key = cv2.waitKey(1) & 0xFF

            # mask windows logic only makes sense with GUI
            want_mask = (dbg.master and dbg.cam_mask)
            if want_mask:
                if not mask_windows_open:
                    cv2.namedWindow("mask_dilated", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("grey", cv2.WINDOW_NORMAL)
                    mask_windows_open = True

                m = getattr(cam, "_dbg_mask", None)
                g = getattr(cam, "_dbg_gray", None)
                if m is not None:
                    cv2.imshow("mask_dilated", m)
                if g is not None:
                    cv2.imshow("grey", g)
            else:
                if mask_windows_open:
                    cv2.destroyWindow("mask_dilated")
                    cv2.destroyWindow("grey")
                    mask_windows_open = False

            # handle keys (same as you already have)
            if key == ord("q"):
                break
            if key == ord("h"):
                dbg.help = not dbg.help
                print(f"[DEBUG] help={'ON' if dbg.help else 'OFF'}", flush=True)
            if key == ord("o"):
                dbg.overlay = not dbg.overlay
                print(f"[DEBUG] overlay={'ON' if dbg.overlay else 'OFF'}", flush=True)
            if key == ord("t"):
                dbg.master = not dbg.master
                print(f"[DEBUG] master={'ON' if dbg.master else 'OFF'}", flush=True)
                if not dbg.master:
                    dbg.cam = dbg.cam_mask = dbg.ctrl = dbg.pid = False
                    dbg.help = False
                    cam.debug = cam.debug_mask = False
                    robot.debug = False
            elif dbg.master:
                if key == ord("c"):
                    dbg.cam = not dbg.cam
                    cam.debug = dbg.cam
                elif key == ord("m"):
                    dbg.cam_mask = not dbg.cam_mask
                    cam.debug_mask = dbg.cam_mask
                elif key == ord("r"):
                    dbg.ctrl = not dbg.ctrl
                    robot.debug = dbg.ctrl
                elif key == ord("p"):
                    dbg.pid = not dbg.pid

        else:
            # Headless mode: no imshow / no waitKey.
            # Just keep the robot running and print a heartbeat.
            time.sleep(0.25)

except KeyboardInterrupt:
    print("\n[INFO] Ctrl+C received. Exiting...", flush=True)


finally:
    running = False
    time.sleep(0.05)
    if not HEADLESS:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
    robot.disarm()
    cam.terminate()
    print("[INFO] Robot disarmed. Camera closed. Bye.", flush=True)
