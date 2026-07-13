"""obs_builder.py — 実機信号から policy 観測235次元を組み立てる。

並び（policy_spec.json / simダンプ準拠, 値はSI・スケール無し）:
  [0:3]    base_lin_vel   機体座標 [m/s]
  [3:6]    base_ang_vel   機体座標 [rad/s]（IMUジャイロ）
  [6:9]    projected_gravity  R^T·(0,0,-1)
  [9:12]   velocity_commands  (vx, vy, wz)
  [12:24]  joint_pos - default   （Isaac順）
  [24:36]  joint_vel             （Isaac順）
  [36:48]  last_actions          （Isaac順・スケール前の生action）
  [48:235] height_scan 187点 = clip(base_z - ground_z - 0.5, -1, 1)
"""
import numpy as np

from . import joint_map as jm


def quat_rotate_inverse(q_wxyz, v):
    """world→bodyベクトル変換。q=(w,x,y,z)。"""
    w, x, y, z = q_wxyz
    q_vec = np.array([x, y, z])
    a = v * (2.0 * w * w - 1.0)
    b = np.cross(q_vec, v) * w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c


class ObsBuilder:
    DIM = 235

    def __init__(self):
        self.last_action = np.zeros(12, np.float32)  # Isaac順・生action
        self.flat_hs = np.full(187, -0.225, np.float32)

    def build(self, state, cmd, height_scan=None, lin_vel_world=None):
        """state: go2_iface.state() dict / cmd: (vx,vy,wz) /
        height_scan: 187 or None(平地仮定) / lin_vel_world: 速度ソース上書き(x,y,z)"""
        q_wxyz = state["quat"]
        v_world = np.asarray(lin_vel_world if lin_vel_world is not None
                             else state.get("vel", [0, 0, 0]), np.float64)
        obs = np.empty(self.DIM, np.float32)
        obs[0:3] = quat_rotate_inverse(q_wxyz, v_world)
        obs[3:6] = state["gyro"]
        obs[6:9] = quat_rotate_inverse(q_wxyz, np.array([0.0, 0.0, -1.0]))
        obs[9:12] = cmd
        q_isaac = jm.sdk_to_isaac(np.asarray(state["q"], np.float32))
        dq_isaac = jm.sdk_to_isaac(np.asarray(state["dq"], np.float32))
        obs[12:24] = q_isaac - jm.DEFAULT_POS_ISAAC
        obs[24:36] = dq_isaac
        obs[36:48] = self.last_action
        obs[48:235] = self.flat_hs if height_scan is None else np.asarray(height_scan, np.float32)
        return obs

    def set_action(self, action_isaac):
        self.last_action = np.asarray(action_isaac, np.float32).copy()

    def action_to_q_target_isaac(self, action_isaac):
        return jm.DEFAULT_POS_ISAAC + jm.ACTION_SCALE * np.asarray(action_isaac, np.float32)
