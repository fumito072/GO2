#!/usr/bin/env python
"""オフライン検証: joint_map + obs_builder + policy.pt の疎通（ロボット不要・要torch/numpy）。

実行: python -m m3_rl.test_obs_builder
検証内容:
  1) 関節マッピングの往復一致
  2) 立位静止+平地height_scanのobs(235)で policy.pt が有限のactionを返す
  3) actionをフィードバックしながら100step回して発散しない（|q_target-default|が常識範囲）
  ※これは「配線の疎通」であり物理の検証ではない。物理は unitree_mujoco の sim2sim で。
"""
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("m3_rl", 1)[0])
from m3_rl import joint_map as jm  # noqa: E402
from m3_rl.obs_builder import ObsBuilder  # noqa: E402


def standing_state():
    return {"quat": [1.0, 0.0, 0.0, 0.0],
            "gyro": [0.0, 0.0, 0.0],
            "vel": [0.0, 0.0, 0.0],
            "q": jm.DEFAULT_POS_SDK.tolist(),
            "dq": [0.0] * 12}


def main():
    import torch
    # 1) mapping
    x = np.arange(12.0)
    assert np.allclose(jm.sdk_to_isaac(jm.isaac_to_sdk(x)), x), "mapping round-trip NG"
    print("[1] joint mapping round-trip OK")

    pol_path = __file__.rsplit("m3_rl", 1)[0] + "policy/policy.pt"
    policy = torch.jit.load(pol_path, map_location="cpu")
    policy.eval()
    ob = ObsBuilder()

    # 2) 1step
    obs = ob.build(standing_state(), np.zeros(3, np.float32))
    assert obs.shape == (235,), obs.shape
    assert abs(float(obs[48:].mean()) - (-0.225)) < 1e-3, "height_scan flat値NG"
    assert np.allclose(obs[6:9], [0, 0, -1], atol=1e-6), "projected_gravity NG"
    with torch.inference_mode():
        act = policy(torch.from_numpy(obs).unsqueeze(0)).squeeze(0).numpy()
    assert act.shape == (12,) and np.isfinite(act).all(), "action NG"
    print("[2] policy 1step OK  |act|max=%.3f" % float(np.abs(act).max()))

    # 3) 100step フィードバック（静止コマンド）: 目標角が常識範囲に留まるか
    st = standing_state()
    max_dev = 0.0
    for i in range(100):
        obs = ob.build(st, np.zeros(3, np.float32))
        with torch.inference_mode():
            act = policy(torch.from_numpy(obs).unsqueeze(0)).squeeze(0).numpy()
        ob.set_action(act)
        q_t = ob.action_to_q_target_isaac(act)
        # 「即時追従する理想モータ」近似で次stateへ
        st["q"] = jm.isaac_to_sdk(q_t).tolist()
        dev = float(np.abs(q_t - jm.DEFAULT_POS_ISAAC).max())
        max_dev = max(max_dev, dev)
        assert np.isfinite(q_t).all()
    assert max_dev < 1.2, "発散疑い max_dev=%.2f" % max_dev
    print("[3] 100step feedback OK  max|q_target-default|=%.3f rad" % max_dev)

    # 4) 前進コマンドでactionが変わるか（感度チェック）
    obs0 = ob.build(standing_state(), np.zeros(3, np.float32))
    obs1 = ob.build(standing_state(), np.array([0.5, 0, 0], np.float32))
    with torch.inference_mode():
        a0 = policy(torch.from_numpy(obs0).unsqueeze(0)).numpy()
        a1 = policy(torch.from_numpy(obs1).unsqueeze(0)).numpy()
    diff = float(np.abs(a1 - a0).max())
    assert diff > 1e-3, "コマンド感度なし？"
    print("[4] command sensitivity OK (Δact=%.3f)" % diff)
    print("ALL OK - 配線疎通に問題なし（物理検証は sim2sim で）")


if __name__ == "__main__":
    main()
