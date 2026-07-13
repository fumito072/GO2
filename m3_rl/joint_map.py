"""joint_map.py — IsaacLab関節順 ↔ Go2 SDK(LowCmd/LowState)モータ順の変換。

真値: policy/policy_spec.json（simからダンプ）。並びを1つでも間違えると歩けないため、
配列はハードコードせず名前から生成する。単体テスト: python -m m3_rl.joint_map
"""
import json
import os

import numpy as np

_SPEC = json.load(open(os.path.join(os.path.dirname(__file__), "..", "policy", "policy_spec.json"),
                       encoding="utf-8"))

# IsaacLab順（policyの入出力順）: FL,FR,RL,RR × hip → thigh → calf
ISAAC_JOINT_NAMES = list(_SPEC["joint_names_isaac_order"])
# Go2 SDK順: FR,FL,RR,RL × (hip,thigh,calf)
SDK_JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]

# SDK配列[j] に入れるべき Isaac index / Isaac配列[i] に入れるべき SDK index
SDK_FROM_ISAAC = np.array([ISAAC_JOINT_NAMES.index(n) for n in SDK_JOINT_NAMES], dtype=int)
ISAAC_FROM_SDK = np.array([SDK_JOINT_NAMES.index(n) for n in ISAAC_JOINT_NAMES], dtype=int)

DEFAULT_POS_ISAAC = np.array(_SPEC["default_joint_pos"], dtype=np.float32)
DEFAULT_POS_SDK = DEFAULT_POS_ISAAC[SDK_FROM_ISAAC]

ACTION_SCALE = float(_SPEC["action_scale"])       # 0.25
KP = float(_SPEC["actuators"]["base_legs"]["kp"][0])   # 25.0
KD = float(_SPEC["actuators"]["base_legs"]["kd"][0])   # 0.5
CONTROL_HZ = float(_SPEC["control_hz"])           # 50


def isaac_to_sdk(arr_isaac):
    return np.asarray(arr_isaac)[SDK_FROM_ISAAC]


def sdk_to_isaac(arr_sdk):
    return np.asarray(arr_sdk)[ISAAC_FROM_SDK]


if __name__ == "__main__":
    # 往復テスト
    x = np.arange(12, dtype=float)
    assert np.allclose(sdk_to_isaac(isaac_to_sdk(x)), x)
    assert np.allclose(isaac_to_sdk(sdk_to_isaac(x)), x)
    # 名前対応の目視確認
    for j, n in enumerate(SDK_JOINT_NAMES):
        print("SDK[%2d] %-16s <- ISAAC[%2d] %-16s  default=%+.2f" %
              (j, n, SDK_FROM_ISAAC[j], ISAAC_JOINT_NAMES[SDK_FROM_ISAAC[j]], DEFAULT_POS_SDK[j]))
    print("OK: mapping round-trip passed")
