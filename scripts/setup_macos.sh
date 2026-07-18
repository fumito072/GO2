#!/bin/bash
# Build a reproducible native Apple Silicon runtime for GO2 Cockpit.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

if [ "$(uname -s)" != "Darwin" ]; then
    echo "ERROR: this setup script is for macOS (Darwin)." >&2
    exit 1
fi
if [ "$(uname -m)" != "arm64" ]; then
    echo "ERROR: the verified runtime targets Apple Silicon (arm64)." >&2
    exit 1
fi
if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    echo "ERROR: $PYTHON_BIN was not found. Install CPython 3.10 first." >&2
    exit 1
fi

PY_VERSION="$($PYTHON_BIN -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
if [ "$PY_VERSION" != "3.10" ]; then
    cat >&2 <<EOF
ERROR: Python 3.10 is required; found $PY_VERSION.
CycloneDDS 0.10.2 provides the verified macOS ARM64 wheel for CPython 3.10.
Set PYTHON_BIN to a Python 3.10 executable and retry.
EOF
    exit 1
fi

cd "$ROOT"
echo "[1/4] Creating .venv with $($PYTHON_BIN --version 2>&1)"
"$PYTHON_BIN" -m venv .venv
VENV_PY="$ROOT/.venv/bin/python"

echo "[2/4] Installing pinned macOS dependencies"
"$VENV_PY" -m pip install --upgrade pip setuptools wheel
"$VENV_PY" -m pip install -r requirements-macos.txt

echo "[3/4] Checking imports and dependency consistency"
"$VENV_PY" -m pip check
"$VENV_PY" - <<'PY'
import importlib

modules = (
    "aiohttp", "anthropic", "av", "cv2", "cyclonedds",
    "faster_whisper", "numpy", "onnxruntime", "sounddevice",
    "torch", "unitree_sdk2py",
)
for name in modules:
    module = importlib.import_module(name)
    print("  %-18s OK %s" % (name, getattr(module, "__version__", "")))
PY

echo "[4/4] Running the non-actuating policy wiring check"
"$VENV_PY" -m m3_rl.test_obs_builder

cat <<'EOF'

macOS runtime is ready.

Read-only hardware probe (does not move the robot):
  GO2_IFACE=en10 .venv/bin/python -m m0_teleop.check_robot --video --lidar

Safe real-sensor UI (commands are blocked):
  COCKPIT_NO_VOICE=1 cockpit/launch.sh --real --read-only

Normal real mode (Mac motion is not yet field-tested; clear the area first):
  cockpit/launch.sh --real
EOF
