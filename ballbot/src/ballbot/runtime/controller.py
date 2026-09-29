import os
import json
import time
import math
import logging
from pathlib import Path
from adafruit_servokit import ServoKit


# =============================
# Paths / Config
# =============================

def _repo_root() -> Path:
    # controller.py is in src/ballbot/runtime/
    return Path(__file__).resolve().parents[3]

def _cfg_path(env_key: str, default_rel: str) -> Path:
    return Path(os.getenv(env_key, str(_repo_root() / default_rel)))

CAL_PATH = _cfg_path("BALLBOT_CAL", "config/calibration.json")


# =============================
# Calibration
# =============================

def default_calibration() -> dict:
    """Defaults are SIM-ONLY. HW mode must use a real file."""
    return {
        "s1": {"neutral": 54.0, "gain": 1.0, "offset": 0.0, "zero": 0.0, "min": 10.0, "max": 100.0},
        "s2": {"neutral": 54.0, "gain": 1.0, "offset": 0.0, "zero": 0.0, "min": 10.0, "max": 100.0},
        "s3": {"neutral": 54.0, "gain": 1.0, "offset": 0.0, "zero": 0.0, "min": 10.0, "max": 100.0},
    }

def load_calibration(path: str | Path) -> dict:
    p = Path(path)
    if not p.exists():
        raise RuntimeError(f"Calibration file not found: {p}")
    try:
        return json.loads(p.read_text())
    except Exception as e:
        raise RuntimeError(f"Failed to parse calibration file {p}: {e}")

def _ensure_fields(cal: dict) -> dict:
    """Ensure required keys exist and coerce numeric types."""
    for s in ("s1", "s2", "s3"):
        cal.setdefault(s, {})
        c = cal[s]
        c.setdefault("neutral", 54.0)
        c.setdefault("gain", 1.0)
        c.setdefault("offset", 0.0)
        c.setdefault("zero", 0.0)
        c.setdefault("min", 0.0)
        c.setdefault("max", 180.0)

        # Coerce types
        for k in ("neutral", "gain", "offset", "zero", "min", "max"):
            c[k] = float(c.get(k, 0.0))
    return cal

def apply_cal(angle_cmd: float, cal: dict) -> float:
    """
    angle_cmd: angle from kinematics (MODEL space)
    cal: per-servo calibration dict

    Mapping:
      servo = neutral + gain * (model - zero) + offset
    """
    neutral = float(cal.get("neutral", 54.0))
    gain = float(cal.get("gain", 1.0))
    offset = float(cal.get("offset", 0.0))
    amin = float(cal.get("min", 0.0))
    amax = float(cal.get("max", 180.0))
    zero = float(cal.get("zero", 0.0))

    a = neutral + gain * (angle_cmd - zero) + offset
    if a < amin: a = amin
    if a > amax: a = amax
    return a

def servo_home(cal: dict) -> float:
    """
    A safe-ish servo-space home used for explicit HOME commands (not required on ARM).
    Uses neutral+offset and clamps.
    """
    neutral = float(cal.get("neutral", 54.0))
    offset = float(cal.get("offset", 0.0))
    amin = float(cal.get("min", 0.0))
    amax = float(cal.get("max", 180.0))
    a = neutral + offset
    return max(amin, min(amax, a))


# =============================
# Controller
# =============================

class RobotController:
    def __init__(self, model, calibration_file=None):
        self.robot = model

        # Logging (HMI should capture stdout/stderr; logging still helps)
        self.log = logging.getLogger("ballbot.controller")

        # Hardware enable
        self.hw_enabled = os.getenv("BALLBOT_HW", "1") == "1"
        self.armed = False

        # Setup hardware (if enabled)
        self.kit = None
        self.s1 = self.s2 = self.s3 = None
        if self.hw_enabled:
            self.kit = ServoKit(channels=16)
            self.s1 = self.kit.servo[13]
            self.s2 = self.kit.servo[15]
            self.s3 = self.kit.servo[14]
            for s in (self.s1, self.s2, self.s3):
                s.actuation_range = 270
                s.set_pulse_width_range(500, 2500)

        # Debug / bookkeeping
        self.debug = False
        self._last_cmd = None
        self._last_print_t = 0.0

        # Select calibration source
        cal_mode = os.getenv("BALLBOT_CAL_MODE", "").strip().lower()

        if calibration_file is None:
            calibration_file = "default" if cal_mode in ("default", "none") else CAL_PATH

        if isinstance(calibration_file, str) and calibration_file.strip().lower() in ("default", "none"):
            self.calibration_file = "<defaults>"
            if self.hw_enabled:
                # Hard fail: never allow guessed calibration on real hardware
                raise RuntimeError("Refusing to run with default calibration while BALLBOT_HW=1")
            self.cal = _ensure_fields(default_calibration())
        else:
            self.calibration_file = str(calibration_file)
            self.cal = _ensure_fields(load_calibration(self.calibration_file))

        # Basic print for visibility (HMI may not show; logs should capture)
        print(
            f"RobotController ready. HW={'ON' if self.hw_enabled else 'OFF'} "
            f"Calibration={self.calibration_file}",
            flush=True,
        )
        print(
            f"[CAL] neutrals: "
            f"s1={self.cal['s1'].get('neutral')} "
            f"s2={self.cal['s2'].get('neutral')} "
            f"s3={self.cal['s3'].get('neutral')}",
            flush=True,
        )

    # -----------------------------
    # Servo lifecycle
    # -----------------------------
    def arm(self):
        """
        Arm should NOT move the robot. It simply enables motion commands.
        Make HOME an explicit user action (or ramp-to-home elsewhere).
        """
        if self.armed:
            return
        self.armed = True
        print("Servos armed (no motion)", flush=True)

    def disarm(self, go_home: bool = False):
        if not self.armed:
            return
        print("Disarming servos", flush=True)
        try:
            if self.hw_enabled and go_home:
                # Explicit HOME on disarm if requested (clamped)
                h1 = servo_home(self.cal["s1"])
                h2 = servo_home(self.cal["s2"])
                h3 = servo_home(self.cal["s3"])
                self._force_servo_angles(h1, h2, h3)
                time.sleep(0.25)

            if self.hw_enabled:
                for s in (self.s1, self.s2, self.s3):
                    s.angle = None
        except Exception as e:
            print("Disarm error:", e, flush=True)
        self.armed = False

    # Optional explicit HOME action (safe-ish)
    def home_servos(self, hold_s: float = 0.25):
        if not self.armed or not self.hw_enabled:
            return
        h1 = servo_home(self.cal["s1"])
        h2 = servo_home(self.cal["s2"])
        h3 = servo_home(self.cal["s3"])
        self._force_servo_angles(h1, h2, h3)
        time.sleep(hold_s)

    # -----------------------------
    # Low-level setters
    # -----------------------------
    def _force_angles(self, a1, a2, a3):
        """Input is MODEL space angles."""
        if not self.hw_enabled:
            return

        c1 = apply_cal(a1, self.cal["s1"])
        c2 = apply_cal(a2, self.cal["s2"])
        c3 = apply_cal(a3, self.cal["s3"])

        self.s1.angle = c1
        self.s2.angle = c2
        self.s3.angle = c3

        if self.debug:
            now = time.time()
            cmd = (round(c1, 3), round(c2, 3), round(c3, 3))
            if cmd != self._last_cmd and (now - self._last_print_t) > 0.2:
                print(f"CMD servo: {cmd[0]:.3f}, {cmd[1]:.3f}, {cmd[2]:.3f}", flush=True)
                self._last_cmd = cmd
                self._last_print_t = now

    def _force_servo_angles(self, s1, s2, s3):
        """Input is SERVO space angles."""
        if not self.hw_enabled:
            return
        self.s1.angle = s1
        self.s2.angle = s2
        self.s3.angle = s3

    def set_motor_angles(self, a1, a2, a3):
        if not self.armed:
            return
        self._force_angles(a1, a2, a3)
        
    def set_servo_angle_raw(self, idx: int, angle: float):
        """idx: 1..3 for s1..s3. Requires armed."""
        if not self.armed or not self.hw_enabled:
            return
        if idx == 1:
            self.s1.angle = angle
        elif idx == 2:
            self.s2.angle = angle
        elif idx == 3:
            self.s3.angle = angle


    # -----------------------------
    # Motion commands
    # -----------------------------
    def Goto_N_time_spherical(self, theta, phi, h):
        if not self.armed:
            return
        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)
        angles = [
            math.degrees(math.pi * 0.5 - self.robot.theta1),
            math.degrees(math.pi * 0.5 - self.robot.theta2),
            math.degrees(math.pi * 0.5 - self.robot.theta3),
        ]
        self.set_motor_angles(*angles)

    def Goto_time_spherical(self, theta, phi, h, t=0.5):
        if not self.armed:
            return
        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)
        angles = [
            math.degrees(math.pi * 0.5 - self.robot.theta1),
            math.degrees(math.pi * 0.5 - self.robot.theta2),
            math.degrees(math.pi * 0.5 - self.robot.theta3),
        ]
        self.interpolate_time(angles, duration=t)

    def interpolate_time(self, target_angles, duration=0.3):
        if not self.armed:
            return
        if not self.hw_enabled:
            return

        current = [self.s1.angle or 0.0, self.s2.angle or 0.0, self.s3.angle or 0.0]
        steps = max(1, int(duration / 0.01))
        for i in range(steps + 1):
            u = i / steps
            angles = [c + (ta - c) * u for c, ta in zip(current, target_angles)]
            self._force_angles(*angles)
            time.sleep(duration / steps)
