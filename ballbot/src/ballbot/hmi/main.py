# src/ballbot/hmi/main.py
import curses
import os
import signal
import subprocess
import time
from pathlib import Path

# File is .../src/ballbot/hmi/main.py
# parents:
#   [0]=hmi, [1]=ballbot, [2]=src, [3]=REPO_ROOT
ROOT = Path(__file__).resolve().parents[3]
LOGS = ROOT / "logs"
LOGS.mkdir(exist_ok=True)

HW_LOCK = Path("/tmp/ballbot_hw.lock")

# Prefer venv python so we always run with the same environment
PY = ROOT / ".venv" / "bin" / "python"
PY_CMD = str(PY) if PY.exists() else "python3"

PROCS: dict[str, subprocess.Popen] = {}  # name -> Popen

CFG_ENV = {
  "BALLBOT_CAL": str(ROOT / "config" / "calibration.json"),
  # reserved for future tuning/config plumbing
  "BALLBOT_PID": str(ROOT / "config" / "pid.json"),
  "BALLBOT_CAM": str(ROOT / "config" / "camera.json"),
}


def _tail_bytes(path: Path, max_bytes: int = 5000) -> str:
    try:
        data = path.read_bytes()
        return data[-max_bytes:].decode(errors="replace")
    except Exception:
        return ""

def _spawn(name: str, cmd: list[str], log_name: str, env: dict | None = None):
    """Spawn a process and tee stdout/stderr to logs."""
    if name in PROCS and PROCS[name].poll() is None:
        return False, f"{name} already running"

    log_path = LOGS / log_name
    f = open(log_path, "ab", buffering=0)

    merged_env = os.environ.copy()
    merged_env["PYTHONPATH"] = f"{ROOT/'src'}:{merged_env.get('PYTHONPATH','')}".rstrip(":")
    if env:
        merged_env.update(env)

    p = subprocess.Popen(
        cmd,
        cwd=str(ROOT),
        stdout=f,
        stderr=subprocess.STDOUT,
        env=merged_env,
        start_new_session=True,
    )
    PROCS[name] = p
    f.close()

    # --- NEW: detect immediate failure ---
    time.sleep(0.25)
    rc = p.poll()
    if rc is not None:
        snippet = _tail_bytes(log_path, 5000)
        # keep lock handling in caller (run_start) as you already do
        return False, f"{name} exited immediately (code {rc}). See {log_path}\n--- tail ---\n{snippet}"

    return True, f"Started {name} (pid {p.pid}) -> {log_path}"


def _spawn_module(name: str, module: str, log_name: str, env: dict | None = None):
    """Spawn: python -m <module> (best for src/ layout)."""
    return _spawn(name, [PY_CMD, "-m", module], log_name, env=env)


def _spawn_script(name: str, script_rel: str, log_name: str, env: dict | None = None):
    """Spawn: bash <script> (for install, etc)."""
    return _spawn(name, ["bash", str(ROOT / script_rel)], log_name, env=env)


def _run_foreground(cmd: list[str], cwd: Path, env: dict | None = None) -> int:
    # Save current curses mode so we can restore after foreground app exits
    curses.def_prog_mode()
    curses.endwin()

    merged = os.environ.copy()
    merged["PYTHONPATH"] = f"{ROOT/'src'}:{merged.get('PYTHONPATH','')}".rstrip(":")  # critical for -m ballbot...
    if env:
        merged.update(env)

    p = None
    try:
        # Start child in a new process group so we can signal it cleanly
        p = subprocess.Popen(
            cmd,
            cwd=str(cwd),
            env=merged,
            start_new_session=True,
        )
        return p.wait()

    except KeyboardInterrupt:
        # User hit Ctrl+C while the tool was running.
        # Forward SIGINT to the child's process group instead of killing the HMI.
        try:
            if p and p.poll() is None:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
                return p.wait()
        except Exception:
            return 130  # conventional Ctrl+C exit code

    finally:
        # Restore curses "program mode" and repaint
        try:
            curses.reset_prog_mode()
            curses.noecho()
            curses.cbreak()
            curses.curs_set(0)
            curses.flushinp()
            curses.doupdate()
        except Exception:
            pass


def _stop(name: str, timeout_s: float = 2.0):
    p = PROCS.get(name)
    if not p:
        return False, f"{name} not running"
    if p.poll() is not None:
        PROCS.pop(name, None)
        return False, f"{name} not running"

    try:
        pgid = os.getpgid(p.pid)

        # 1) polite stop
        os.killpg(pgid, signal.SIGINT)

        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if p.poll() is not None:
                PROCS.pop(name, None)
                return True, f"Stopped {name} (SIGINT)."
            time.sleep(0.05)

        # 2) escalate
        os.killpg(pgid, signal.SIGTERM)

        t0 = time.time()
        while time.time() - t0 < 1.0:
            if p.poll() is not None:
                PROCS.pop(name, None)
                return True, f"Stopped {name} (SIGTERM)."
            time.sleep(0.05)

        # 3) last resort
        os.killpg(pgid, signal.SIGKILL)
        time.sleep(0.1)

        PROCS.pop(name, None)
        return True, f"Killed {name} (SIGKILL)."

    except Exception as e:
        return False, f"Stop failed: {e}"



def _is_running(name: str) -> bool:
    p = PROCS.get(name)
    if not p:
        return False
    if p.poll() is None:
        return True
    PROCS.pop(name, None)
    return False


def _hw_busy() -> bool:
    return HW_LOCK.exists()


def _hw_lock_acquire() -> bool:
    if _hw_busy():
        return False
    try:
        HW_LOCK.write_text(str(os.getpid()))
        return True
    except Exception:
        return False


def _hw_lock_release():
    try:
        if HW_LOCK.exists():
            HW_LOCK.unlink()
    except Exception:
        pass


MENU = [
    ("Install/Update deps (scripts/install.sh)", "install"),
    ("Run: Start balance runtime", "run_start"),
    ("Run: Stop balance runtime", "run_stop"),
    ("Jog: Start jog", "jog_start"),
    ("Calibrate: Servo offsets", "cal_servo"),
    ("Tune PID: (not wired yet)", "tune"),
    ("Tools: GUI (needs wired)", "GUI"),
    ("Tools: Run Debug", "run_debug"),
    ("Status", "status"),
    ("Exit", "exit"),
]


def draw(stdscr, idx, msg):
    stdscr.clear()
    h, w = stdscr.getmaxyx()

    title = "BallBot HMI"
    stdscr.addstr(0, max(0, (w - len(title)) // 2), title, curses.A_BOLD)

    status = [
        f"runtime={'ON' if _is_running('runtime') else 'off'}",
        f"jog={'ON' if _is_running('jog') else 'off'}",
        f"webui={'ON' if _is_running('webui') else 'off'}",
        f"HW_LOCK={'BUSY' if _hw_busy() else 'free'}",
    ]
    stdscr.addstr(2, 2, " | ".join(status))

    stdscr.addstr(4, 2, "Menu (↑/↓, Enter). Press 'q' or Esc to exit.")
    for i, (label, _) in enumerate(MENU):
        attr = curses.A_REVERSE if i == idx else curses.A_NORMAL
        stdscr.addstr(6 + i, 4, label[: w - 8], attr)

    if msg:
        stdscr.addstr(h - 2, 2, msg[: w - 4])

    stdscr.addstr(3, 2, f"CAL={CFG_ENV['BALLBOT_CAL']}")

    stdscr.refresh()


def handle(action: str):
    # -------- Install --------
    if action == "install":
        # install uses sudo; will prompt in the terminal (log still captures output)
        _run_foreground(["bash", str(ROOT / "scripts" / "Install.sh")], ROOT)
        return "Install finished (foreground)."

    # -------- Balance runtime --------
    if action == "run_start":
        if _hw_busy():
            return "Hardware busy (lock exists). Stop other HW process first."
        if not _hw_lock_acquire():
            return "Could not acquire hardware lock."

        # NEW: module launch for src/ layout
        ok, msg = _spawn_module("runtime", "ballbot.runtime.main", "runtime.log", env=CFG_ENV)
        if not ok:
            _hw_lock_release()
        return msg

    if action == "run_stop":
        ok, msg = _stop("runtime")
        _hw_lock_release()
        return msg

    # --- Run Debug (FOREGROUND) ---
    if action == "run_debug":
        if _hw_busy():
            return "Hardware busy (lock exists). Stop other HW process first."
        if not _hw_lock_acquire():
            return "Could not acquire hardware lock."
        try:
            rc = _run_foreground([PY_CMD, "-m", "ballbot.runtime.main"], ROOT, env=CFG_ENV)
            return f"Run Debug exited (code {rc})."
        finally:
            _hw_lock_release()

    # --- Jog (FOREGROUND) ---
    if action == "jog_start":
        if _hw_busy():
            return "Hardware busy (lock exists). Stop other HW process first."
        if not _hw_lock_acquire():
            return "Could not acquire hardware lock."
        try:
            rc = _run_foreground([PY_CMD, "-m", "ballbot.runtime.jog"], ROOT, env=CFG_ENV)
            return f"Jog exited (code {rc})."
        finally:
            _hw_lock_release()

    # -------- Calibration: Servo Offsets (foreground) --------
    if action == "cal_servo":
        if _hw_busy():
            return "Hardware busy (lock exists). Stop other HW process first."
        if not _hw_lock_acquire():
            return "Could not acquire hardware lock."
        try:
            _run_foreground([PY_CMD, "-m", "ballbot.runtime.calibration", "--servo"], ROOT, env=CFG_ENV)
            return "Calibration finished (foreground)."
        finally:
            _hw_lock_release()
            
   #-------- GUI --------
    if action == "GUI":
        # sim-only: no hardware lock needed
        env = dict(CFG_ENV)
        env["BALLBOT_HW"] = "0"
        ok, msg = _spawn_module("GUI", "ballbot.tools.pyqt_gui.GUI", "gui.log", env=env)
        return msg           
            

    # -------- Web UI --------
  #  if action == "web_sim":
   #     # sim-only: no hardware lock needed
    #    env = dict(CFG_ENV)
     #   env["BALLBOT_HW"] = "0"
      #  ok, msg = _spawn_module("webui", "ballbot.tools.webui.app", "webui.log", env=env)
       # return msg

 #   if action == "web_hw":
  #      if _hw_busy():
   #         return "Hardware busy (lock exists). Stop other HW process first."
    #    if not _hw_lock_acquire():
     #       return "Could not acquire hardware lock."
      #  env = dict(CFG_ENV)
       # env["BALLBOT_HW"] = "1"
   #     ok, msg = _spawn_module("webui", "ballbot.tools.webui.app", "webui.log", env=env)
    #    if not ok:
     #       _hw_lock_release()
      #  return msg

 #   if action == "web_stop":
  #      ok, msg = _stop("webui")
   #     # safest: always release lock after stopping webui
    #    _hw_lock_release()
       # return msg      

    # -------- Status / misc --------
    if action == "status":
        lines = []
        for name in ["install", "runtime", "jog", "webui"]:
            p = PROCS.get(name)
            if p and p.poll() is None:
                lines.append(f"{name}: RUNNING pid={p.pid}")
            else:
                lines.append(f"{name}: off")
        return " | ".join(lines)

    if action == "tune":
        return "Tune menu not wired yet (next step)."

    if action == "exit":
        return "exit"

    return "Unknown action"


def _curses_main(stdscr):
    curses.curs_set(0)
    stdscr.keypad(True)
    curses.noecho()
    curses.cbreak()
    idx = 0
    msg = ""

    while True:
        # Reap lock if runtime/webui/jog are not running
        if not (_is_running("runtime") or _is_running("webui") or _is_running("jog")):
            _hw_lock_release()

        draw(stdscr, idx, msg)

        key = stdscr.getch()
        msg = ""

        if key in (ord("q"), 27):
            break
        if key == curses.KEY_UP:
            idx = (idx - 1) % len(MENU)
        elif key == curses.KEY_DOWN:
            idx = (idx + 1) % len(MENU)
        elif key in (curses.KEY_ENTER, 10, 13):
            action = MENU[idx][1]
            out = handle(action)
            stdscr.clear()
            stdscr.refresh()
            if out == "exit":
                break
            msg = out

    # best-effort cleanup
    for name in ["runtime", "jog", "webui"]:
        _stop(name)
    _hw_lock_release()


def main() -> int:
    """Entry point for `python -m ballbot.hmi`."""
    curses.wrapper(_curses_main)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
