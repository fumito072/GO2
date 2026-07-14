#!/usr/bin/env bash
# go2-runtime 分離環境の構築(WSL2 Ubuntu-24.04、robot 非接続)
# docs/CLAUDE.md §6.2: 環境を分離する。torch は CPU 版(policy.pt は 1.1MB MLP)。
set -eu
VENV="$HOME/go2-runtime"
if [ ! -x "$VENV/bin/python" ]; then
    python3 -m venv "$VENV"
fi
"$VENV/bin/pip" install --quiet --no-input numpy torch --index-url https://download.pytorch.org/whl/cpu
"$VENV/bin/python" - <<'EOF'
import numpy, torch
print("go2-runtime ready: torch", torch.__version__, "/ numpy", numpy.__version__)
EOF
