import os
import time
import json
import curses
import math
from pathlib import Path

from .controller import RobotController, apply_cal
from .robotKinematics import RobotKinematics

# calibration.py lives in src/ballbot/runtime/
REPO_ROOT = Path(__file__).resolve().parents[3]

CAL_FILE = Path(
    os.getenv(
        "BALLBOT_CAL",
        str(REPO_ROOT / "config" / "calibration.json")
    )
)

REFERENCE_POSE = {"theta": 0.0, "phi": 0.0, "h": 8.26}

STEP_SUPER_COARSE = 6.0
STEP_COARSE = 3.0
STEP_FINE = 1.0
STEP_MICRO = 0.5

step = STEP_FINE

KEY_MAPPING = {
    ord('q'): ("s1", -1),
    ord('a'): ("s1",  1),
    ord('w'): ("s2", -1),
    ord('s'): ("s2",  1),
    ord('e'): ("s3", -1),
    ord('d'): ("s3",  1),
}

STEP_KEYS = {
    ord('z'): STEP_SUPER_COARSE,
    ord('x'): STEP_COARSE,
    ord('c'): STEP_FINE,
    ord('v'): STEP_MICRO,
}

PRINT_KEY = ord('p')
SAVE_KEY = ord('S')           # Shift+S
RELOAD_KEY = ord('r')
RESET_OFFSET_KEY = ord('0')
CAPTURE_ZERO_KEY = ord('Z')   # Shift+Z
DEBUG_KEY = ord('t')

DEBOUNCE_SEC = 0.15
last_servo_press = {"s1": 0.0, "s2": 0.0, "s3": 0.0}


def save_cal(rc):
    with open(CAL_FILE, "w") as f:
        json.dump(rc.cal, f, indent=2)


def safe_exit(rc, stdscr, put):
    put(12, 0, "Moving to safe pose before shutdown...")
    stdscr.refresh()

    theta = REFERENCE_POSE["theta"]
    phi = REFERENCE_POSE["phi"]
    h = REFERENCE_POSE["h"]

    rc.robot.solve_inverse_kinematics_spherical(theta, phi, h)
    angles = [
        math.degrees(math.pi * 0.5 - rc.robot.theta1),
        math.degrees(math.pi * 0.5 - rc.robot.theta2),
        math.degrees(math.pi * 0.5 - rc.robot.theta3),
    ]
    rc.set_motor_angles(*angles)
    time.sleep(0.5)

    rc.disarm()
    save_cal(rc)

    put(13, 0, f"Calibration saved to {CAL_FILE}")
    stdscr.refresh()
    time.sleep(1.0)


def main(stdscr):
    global step

    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)
    stdscr.clear()

    H, W = stdscr.getmaxyx()

    def put(row, col, text):
        # Safe fixed-width write that clears old text on that line
        width = max(1, W - col - 1)
        stdscr.addstr(row, col, text.ljust(width)[:width])

    # --- Fixed UI layout (rows) ---
    ROW_TITLE = 0
    ROW_KEYS1 = 1
    ROW_KEYS2 = 2
    ROW_KEYS3 = 3
    ROW_KEYS4 = 4
    ROW_HINTS = 5

    ROW_STEP  = 6
    ROW_OFFS  = 7
    ROW_STAT  = 8
    ROW_LAST  = 9
    ROW_DBG   = 10

    ROW_EXIT1 = 12
    ROW_EXIT2 = 13

    put(ROW_TITLE, 0, "=== REAL-TIME CALIBRATION ===")
    put(ROW_KEYS1, 0, "q/a: s1   w/s: s2   e/d: s3")
    put(ROW_KEYS2, 0, "z: super coarse   x: coarse   c: fine   v: micro")
    put(ROW_KEYS3, 0, "p: print offsets   S: save   r: reload   0: reset offsets   t: debug")
    put(ROW_KEYS4, 0, "Z: capture ZERO (one-time, at reference pose)")
    put(ROW_HINTS, 0, "Ctrl+C to exit")
    stdscr.refresh()

    model = RobotKinematics()
    rc = RobotController(model, calibration_file=CAL_FILE)
    rc.arm()

    # Initial UI state
    put(ROW_STEP, 0, f"Step size: {step}")
    put(ROW_OFFS, 0,
        f"Offsets: s1={rc.cal['s1'].get('offset', 0.0):+.4f}  "
        f"s2={rc.cal['s2'].get('offset', 0.0):+.4f}  "
        f"s3={rc.cal['s3'].get('offset', 0.0):+.4f}"
    )
    put(ROW_STAT, 0, "Ready.")
    put(ROW_LAST, 0, "")
    put(ROW_DBG, 0, "")
    stdscr.refresh()

    try:
        while True:
            # --- Drive reference pose continuously ---
            theta = REFERENCE_POSE["theta"]
            phi = REFERENCE_POSE["phi"]
            h = REFERENCE_POSE["h"]

            rc.robot.solve_inverse_kinematics_spherical(theta, phi, h)
            angles = [
                math.degrees(math.pi * 0.5 - rc.robot.theta1),
                math.degrees(math.pi * 0.5 - rc.robot.theta2),
                math.degrees(math.pi * 0.5 - rc.robot.theta3),
            ]
            rc.set_motor_angles(*angles)

            # --- Debug line (curses, no stdout spam) ---
            if rc.debug:
                c1 = apply_cal(angles[0], rc.cal["s1"])
                c2 = apply_cal(angles[1], rc.cal["s2"])
                c3 = apply_cal(angles[2], rc.cal["s3"])
                put(ROW_DBG, 0, f"CMD servo: {c1:7.3f}  {c2:7.3f}  {c3:7.3f}")
            else:
                put(ROW_DBG, 0, "")

            key = stdscr.getch()
            now = time.time()

            if key != -1:
                # Step size keys
                if key in STEP_KEYS:
                    step = STEP_KEYS[key]
                    put(ROW_STEP, 0, f"Step size: {step}")
                    put(ROW_STAT, 0, "Step updated.")
                    stdscr.refresh()

                # Print offsets
                elif key == PRINT_KEY:
                    put(ROW_OFFS, 0,
                        f"Offsets: s1={rc.cal['s1'].get('offset', 0.0):+.4f}  "
                        f"s2={rc.cal['s2'].get('offset', 0.0):+.4f}  "
                        f"s3={rc.cal['s3'].get('offset', 0.0):+.4f}"
                    )
                    put(ROW_STAT, 0, "Offsets printed.")
                    stdscr.refresh()

                # Save
                elif key == SAVE_KEY:
                    save_cal(rc)
                    put(ROW_STAT, 0, f"Saved -> {CAL_FILE}")
                    stdscr.refresh()

                # Reload
                elif key == RELOAD_KEY:
                    try:
                        with open(CAL_FILE, "r") as f:
                            rc.cal = json.load(f)
                        put(ROW_STAT, 0, "Reloaded calibration from disk (reverted).")
                        put(ROW_OFFS, 0,
                            f"Offsets: s1={rc.cal['s1'].get('offset', 0.0):+.4f}  "
                            f"s2={rc.cal['s2'].get('offset', 0.0):+.4f}  "
                            f"s3={rc.cal['s3'].get('offset', 0.0):+.4f}"
                        )
                    except Exception as e:
                        put(ROW_STAT, 0, f"Reload failed: {e}")
                    stdscr.refresh()

                # Reset offsets
                elif key == RESET_OFFSET_KEY:
                    for s in ("s1", "s2", "s3"):
                        rc.cal[s]["offset"] = 0.0
                    put(ROW_OFFS, 0,
                        f"Offsets: s1={0.0:+.4f}  s2={0.0:+.4f}  s3={0.0:+.4f}"
                    )
                    put(ROW_STAT, 0, "Offsets reset to 0.0.")
                    stdscr.refresh()

                # Debug toggle
                elif key == DEBUG_KEY:
                    rc.debug = not rc.debug
                    put(ROW_STAT, 0, f"Debug {'ON' if rc.debug else 'OFF'}.")
                    stdscr.refresh()

                # Capture zero
                elif key == CAPTURE_ZERO_KEY:
                    rc.robot.solve_inverse_kinematics_spherical(
                        REFERENCE_POSE["theta"],
                        REFERENCE_POSE["phi"],
                        REFERENCE_POSE["h"]
                    )
                    a1 = math.degrees(math.pi * 0.5 - rc.robot.theta1)
                    a2 = math.degrees(math.pi * 0.5 - rc.robot.theta2)
                    a3 = math.degrees(math.pi * 0.5 - rc.robot.theta3)

                    rc.cal["s1"]["zero"] = round(a1, 4)
                    rc.cal["s2"]["zero"] = round(a2, 4)
                    rc.cal["s3"]["zero"] = round(a3, 4)

                    save_cal(rc)
                    put(ROW_STAT, 0, f"Captured ZERO: {a1:.2f}, {a2:.2f}, {a3:.2f}")
                    stdscr.refresh()

                # Offset adjust
                elif key in KEY_MAPPING:
                    servo, direction = KEY_MAPPING[key]

                    # Safety net: require ZERO before allowing offset tweaks
                    if abs(float(rc.cal[servo].get("zero", 0.0))) < 1e-6:
                        put(ROW_STAT, 0, "ERROR: Capture ZERO (Z) before offset calibration.")
                        stdscr.refresh()
                        continue

                    if (now - last_servo_press[servo]) >= DEBOUNCE_SEC:
                        new_off = rc.cal[servo]["offset"] + direction * step
                        new_off = round(new_off / step) * step   # quantize
                        new_off = round(new_off, 4)

                        rc.cal[servo]["offset"] = new_off
                        last_servo_press[servo] = now

                        put(ROW_LAST, 0, f"Updated {servo} offset: {new_off:+.4f}   (step={step})")

                        # Keep offsets always visible
                        put(ROW_OFFS, 0,
                            f"Offsets: s1={rc.cal['s1'].get('offset', 0.0):+.4f}  "
                            f"s2={rc.cal['s2'].get('offset', 0.0):+.4f}  "
                            f"s3={rc.cal['s3'].get('offset', 0.0):+.4f}"
                        )
                        put(ROW_STAT, 0, "Offset updated.")
                        stdscr.refresh()

            time.sleep(0.05)

    except KeyboardInterrupt:
        put(ROW_EXIT1, 0, "Ctrl+C detected, exiting safely...")
        stdscr.refresh()
        safe_exit(rc, stdscr, put)


if __name__ == "__main__":
    curses.wrapper(main)
