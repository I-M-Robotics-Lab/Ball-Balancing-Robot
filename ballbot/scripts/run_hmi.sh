#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# curses needs TERM
export TERM="${TERM:-xterm}"

# If stdout isn't a tty but stdin is, fix it
if [[ -t 0 && ! -t 1 ]]; then
  exec >/dev/tty 2>/dev/tty
fi

# If we still don't have a tty, relaunch inside a terminal emulator
if [[ ! -t 0 || ! -t 1 ]]; then
  if command -v lxterminal >/dev/null 2>&1; then
    exec lxterminal -e "$0"
  elif command -v xterm >/dev/null 2>&1; then
    exec xterm -e "$0"
  fi
  echo "[HMI] No TTY available."
  exit 2
fi

if [[ ! -x ".venv/bin/python" ]]; then
  echo "Venv not found at: $ROOT/.venv"
  echo "Run: bash scripts/install.sh"
  exit 1
fi

export PYTHONPATH="$ROOT/src"
#echo "TERM=$TERM tty=$(tty) in=$(test -t 0 && echo yes || echo no) out=$(test -t 1 && echo yes || echo no)"
#read -r -p "Press Enter..."
exec "$ROOT/.venv/bin/python" -m ballbot.hmi
