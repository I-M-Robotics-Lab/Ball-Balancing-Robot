# src/ballbot/runtime/jog.py
import os
import time
import curses

from ballbot.runtime.robotKinematics import RobotKinematics
from ballbot.runtime.controller import RobotController, servo_home


HELP = [
    "=== BallBot Jog Mode (SERVO ONLY, NO IK) ===",
    "R: Arm (no motion)        F: Disarm",
    "SPACE: Deadman toggle (required for jog keys)",
    "Jog keys (match calibration):",
    "  s1: Q (-) / A (+)     s2: W (-) / S (+)     s3: E (-) / D (+)",
    "Step presets: Z=6.0  X=3.0  C=1.0  V=0.5",
    "H: HOME ALL (servo ref = neutral+offset)",
    "ESC or Ctrl+C: Quit (HOME all -> disarm)",
]


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def run(stdscr):
    curses.curs_set(0)
    stdscr.timeout(50)
    stdscr.keypad(True)

    # Step presets (match calibration)
    STEP_SUPER_COARSE = 6.0
    STEP_COARSE = 3.0
    STEP_FINE = 1.0
    STEP_MICRO = 0.5
    step = STEP_FINE

    # Rate-limit writes to avoid bus spam/twitch
    MIN_DT = 0.03
    last_write_t = 0.0

    os.environ.setdefault("BALLBOT_HW", "1")

    model = RobotKinematics()
    rc = RobotController(model)

    armed = False
    deadman = False

    # Track last commanded / synced servo-space angles (no motion at startup)
    pos = {
        1: servo_home(rc.cal["s1"]),
        2: servo_home(rc.cal["s2"]),
        3: servo_home(rc.cal["s3"]),
    }

    last = "START (disarmed; no motion)"

    # Dedicated jog keys
    KEY_MAPPING = {
        ord("q"): (1, -1),
        ord("a"): (1,  1),
        ord("w"): (2, -1),
        ord("s"): (2,  1),
        ord("e"): (3, -1),
        ord("d"): (3,  1),
    }

    STEP_KEYS = {
        ord("z"): STEP_SUPER_COARSE,
        ord("x"): STEP_COARSE,
        ord("c"): STEP_FINE,
        ord("v"): STEP_MICRO,
    }

    def cal_for(i: int) -> dict:
        return rc.cal[f"s{i}"]

    def sync_pos_from_hw():
        try:
            if rc.s1 and rc.s1.angle is not None: pos[1] = float(rc.s1.angle)
            if rc.s2 and rc.s2.angle is not None: pos[2] = float(rc.s2.angle)
            if rc.s3 and rc.s3.angle is not None: pos[3] = float(rc.s3.angle)
        except Exception:
            pass

    def set_pos(i: int, angle: float, *, force: bool = False):
        nonlocal last_write_t
        c = cal_for(i)
        angle = clamp(angle, c["min"], c["max"])

        now = time.time()
        if (not force) and (now - last_write_t) < MIN_DT:
            pos[i] = angle
            return

        rc.set_servo_angle_raw(i, angle)
        pos[i] = angle
        last_write_t = now


    def home_all():
        for i in (1, 2, 3):
            set_pos(i, servo_home(cal_for(i)), force=True)
            time.sleep(MIN_DT)  # optional but nice insurance

    def draw():
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        stdscr.addstr(0, 0, HELP[0][: w - 1])
        stdscr.addstr(
            1, 0,
            f"ARMED={armed}  DEADMAN={deadman}  step={step:.2f}   CAL={getattr(rc,'calibration_file','')}"[: w - 1]
        )

        row = 3
        for i in (1, 2, 3):
            c = cal_for(i)
            stdscr.addstr(
                row, 0,
                f"s{i}: pos={pos[i]:7.3f}  ref={servo_home(c):7.3f}  (min={c['min']:.1f} max={c['max']:.1f})"[: w - 1]
            )
            row += 1

        row += 1
        for line in HELP[1:]:
            stdscr.addstr(row, 0, line[: w - 1])
            row += 1

        stdscr.addstr(h - 2, 0, f"Last: {last}"[: w - 1])
        stdscr.refresh()

    draw()

    try:
        while True:
            key = stdscr.getch()
            if key == -1:
                continue

            # ESC exits (Ctrl+C will raise KeyboardInterrupt and hit finally)
            if key == 27:
                last = "QUIT"
                break

            # normalize letters
            k = ord(chr(key).lower()) if 0 <= key <= 255 else key

            if key == ord(" "):
                deadman = not deadman
                last = f"DEADMAN={'ON' if deadman else 'OFF'}"
                draw()
                continue

            if k == ord("r"):
                if not armed:
                    rc.arm()
                    armed = True
                    deadman = False
                    sync_pos_from_hw()
                    last = "ARMED (deadman OFF)"
                else:
                    last = "Already armed"
                draw()
                continue

            if k == ord("f"):
                if armed:
                    rc.disarm(go_home=False)
                    armed = False
                    deadman = False
                    last = "DISARMED"
                else:
                    last = "Already disarmed"
                draw()
                continue

            if k in STEP_KEYS:
                step = STEP_KEYS[k]
                last = f"STEP={step:.1f}"
                draw()
                continue

            if k == ord("h"):
                if not armed:
                    last = "HOME ALL requires armed"
                else:
                    home_all()
                    last = "HOME ALL"
                draw()
                continue

            if k in KEY_MAPPING:
                if not armed:
                    last = "Jog blocked: not armed"
                elif not deadman:
                    last = "Jog blocked: deadman OFF"
                else:
                    i, d = KEY_MAPPING[k]
                    delta = d * step
                    set_pos(i, pos[i] + delta)
                    last = f"s{i} {'+' if d>0 else '-'}{abs(delta):.1f}"
                draw()
                continue

            last = f"key={key}"
            draw()

    finally:
        try:
            if armed:
                home_all()
                time.sleep(0.15)
            rc.disarm(go_home=False)
        except Exception:
            pass


def main():
    curses.wrapper(run)


if __name__ == "__main__":
    main()
