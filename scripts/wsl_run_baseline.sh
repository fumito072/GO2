#!/usr/bin/env bash
# torch 依存の offline baseline(robot 非接続)を go2-runtime venv で実行
set -u
VENV="$HOME/go2-runtime"
cd /mnt/c/Users/FUJIFILM/colapis/GO2
export PYTHONDONTWRITEBYTECODE=1
echo "--- m3_rl.joint_map ---"
"$VENV/bin/python" -m m3_rl.joint_map; echo "exit=$?"
echo "--- m3_rl.test_obs_builder ---"
"$VENV/bin/python" -m m3_rl.test_obs_builder; echo "exit=$?"
