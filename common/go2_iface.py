"""go2_iface.py — unitree_sdk2py の薄いラッパ + ロボット無しで動くMock。

実機: `pip install git+https://github.com/unitreerobotics/unitree_sdk2_python.git`
（cyclonedds が必要。README_JP.md の M0 セットアップ参照）

使い方:
    from common.go2_iface import make_robot
    bot = make_robot(mock=False)   # 実機 / True でMock
    bot.move(0.3, 0, 0); bot.stop_move(); st = bot.state()

※ SDK の import は関数内で遅延させ、Mockモードでは SDK 不要。
※ 低レベル(LowCmd)は m3_rl/rl_stair_controller.py が本ラッパの low_* を使う。
"""
import math
import threading
import time

import numpy as np

from . import config

# Go2 SDK(LowCmd/LowState) のモータ順（unitree_sdk2 go2 例と同一）
SDK_JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]

_dds_inited = False


def _init_dds():
    global _dds_inited
    if _dds_inited:
        return
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    if config.NET_IFACE:
        ChannelFactoryInitialize(0, config.NET_IFACE)
    else:
        ChannelFactoryInitialize(0)
    _dds_inited = True


class RealGo2:
    """高レベル(Sport)・状態購読・映像。M0-M2 はこれだけで足りる。"""

    def __init__(self):
        _init_dds()
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

        self.sport = SportClient()
        self.sport.SetTimeout(5.0)
        self.sport.Init()

        self._low = None
        self._sms = None
        self._low_ts = 0.0
        self._sms_ts = 0.0

        self._sub_low = ChannelSubscriber(config.TOPIC_LOWSTATE, LowState_)
        self._sub_low.Init(self._on_low, 10)
        self._sub_sms = ChannelSubscriber(config.TOPIC_SPORTSTATE, SportModeState_)
        self._sub_sms.Init(self._on_sms, 10)
        self._video = None

    def _on_low(self, msg):
        self._low = msg
        self._low_ts = time.monotonic()

    def _on_sms(self, msg):
        self._sms = msg
        self._sms_ts = time.monotonic()

    # --- 高レベル速度制御（Sport mode 有効時のみ）---
    def move(self, vx, vy, wz):
        vx = max(config.VEL_LIMIT["vx"][0], min(config.VEL_LIMIT["vx"][1], vx))
        vy = max(config.VEL_LIMIT["vy"][0], min(config.VEL_LIMIT["vy"][1], vy))
        wz = max(config.VEL_LIMIT["wz"][0], min(config.VEL_LIMIT["wz"][1], wz))
        self.sport.Move(vx, vy, wz)

    def stop_move(self):
        self.sport.StopMove()

    def damp(self):
        self.sport.Damp()

    def stand_up(self):
        self.sport.RecoveryStand()

    def stand_down(self):
        self.sport.StandDown()

    def balance_stand(self):
        self.sport.BalanceStand()

    # --- 状態 ---
    def state(self):
        """dict: rpy, gyro, quat(wxyz), q/dq(SDK順12), pos(odom), vel, low_age, battery"""
        s = {"low_age": time.monotonic() - self._low_ts if self._low_ts else 1e9,
             "sms_age": time.monotonic() - self._sms_ts if self._sms_ts else 1e9}
        if self._low is not None:
            imu = self._low.imu_state
            s["rpy"] = list(imu.rpy)
            s["gyro"] = list(imu.gyroscope)
            s["quat"] = list(imu.quaternion)  # (w,x,y,z)
            s["q"] = [self._low.motor_state[i].q for i in range(12)]
            s["dq"] = [self._low.motor_state[i].dq for i in range(12)]
            s["tau"] = [self._low.motor_state[i].tau_est for i in range(12)]
            try:
                s["battery"] = self._low.power_v
            except Exception:
                pass
        if self._sms is not None:
            s["pos"] = list(self._sms.position)      # world odom [m]
            s["vel"] = list(self._sms.velocity)      # world [m/s]
            s["yaw_speed"] = self._sms.yaw_speed
            s["body_height"] = self._sms.body_height
        return s

    # --- 前面カメラ ---
    def get_frame(self):
        """BGR np.ndarray or None"""
        import cv2
        if self._video is None:
            from unitree_sdk2py.go2.video.video_client import VideoClient
            self._video = VideoClient()
            self._video.SetTimeout(3.0)
            self._video.Init()
        code, data = self._video.GetImageSample()
        if code != 0 or data is None:
            return None
        arr = np.frombuffer(bytes(data), dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    # --- モード切替（M3で使用）---
    # NOTE: SDKの実体は unitree_sdk2py.comm.motion_switcher にある
    #       (go2.motion_switcher は存在しない)。
    @staticmethod
    def _motion_switcher():
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        ms = MotionSwitcherClient()
        ms.SetTimeout(5.0)
        ms.Init()
        return ms

    def release_sport_mode(self):
        """低レベル制御の前に必須。0=成功、それ以外は失敗コード。

        解除前のモード名(例 'mcf','normal','ai')を self._prev_mode に記憶し、
        restore_sport_mode() が同じモードへ戻せるようにする。
        """
        ms = self._motion_switcher()
        _c, result = ms.CheckMode()
        prev = (result or {}).get("name")
        if prev:
            self._prev_mode = prev
        code = -1
        for _ in range(10):
            code, _ = ms.ReleaseMode()   # ReleaseMode は (code, None) を返す
            time.sleep(0.5)
            _c2, result = ms.CheckMode()
            if not result or not result.get("name"):
                return 0                  # モード名が消えた = 解除できた
        return code

    def restore_sport_mode(self, name=None):
        """低レベル制御のあと sport(高レベル)へ戻す。0=成功。
        name未指定なら解除前のモード(既定 'normal')へ戻す。"""
        target = name or getattr(self, "_prev_mode", None) or "normal"
        ms = self._motion_switcher()
        code, _ = ms.SelectMode(target)
        time.sleep(1.0)
        return code

    def mode_name(self):
        """現在のモーションモード名('normal'等)。解除中は空/None。"""
        try:
            _code, result = self._motion_switcher().CheckMode()
            return (result or {}).get("name") or ""
        except Exception:
            return "?"

    # --- 低レベル（LowCmd）---
    def low_publisher(self):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        pub = ChannelPublisher(config.TOPIC_LOWCMD, LowCmd_)
        pub.Init()
        crc = CRC()

        cmd = unitree_go_msg_dds__LowCmd_()
        cmd.head[0], cmd.head[1] = 0xFE, 0xEF
        cmd.level_flag = 0xFF  # low-level
        for m in cmd.motor_cmd:
            m.mode = 0x01
            m.q, m.dq, m.tau, m.kp, m.kd = 0.0, 0.0, 0.0, 0.0, 0.0

        def send(q_sdk, kp, kd, dq=None, tau=None):
            for i in range(12):
                mc = cmd.motor_cmd[i]
                mc.q = float(q_sdk[i])
                mc.dq = float(dq[i]) if dq is not None else 0.0
                mc.tau = float(tau[i]) if tau is not None else 0.0
                mc.kp = float(kp if np.isscalar(kp) else kp[i])
                mc.kd = float(kd if np.isscalar(kd) else kd[i])
            cmd.crc = crc.Crc(cmd)
            pub.Write(cmd)

        return send


class MockGo2:
    """ロボット無しで全スクリプトを疎通させる簡易シミュレータ（一輪車運動+雑音）。"""

    def __init__(self):
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.vx = self.vy = self.wz = 0.0
        # SDK順(FR,FL,RR,RL × hip,thigh,calf)の立位姿勢（R脚hipは-0.1, L脚hipは+0.1）
        self.q = [-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5]
        self._t0 = time.monotonic()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def _run(self):
        dt = 0.02
        while True:
            self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
            self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
            self.yaw += self.wz * dt
            time.sleep(dt)

    def move(self, vx, vy, wz):
        self.vx, self.vy, self.wz = vx, vy, wz

    def stop_move(self):
        self.vx = self.vy = self.wz = 0.0

    def damp(self):
        self.stop_move()
        print("[mock] DAMP")

    def stand_up(self):
        print("[mock] stand_up")

    def stand_down(self):
        print("[mock] stand_down")

    def balance_stand(self):
        print("[mock] balance_stand")

    def state(self):
        n = np.random.randn
        return {"low_age": 0.001, "sms_age": 0.001,
                "rpy": [0.002 * n(), 0.002 * n(), self.yaw],
                "gyro": [0.01 * n(), 0.01 * n(), self.wz],
                "quat": [math.cos(self.yaw / 2), 0.0, 0.0, math.sin(self.yaw / 2)],
                "q": [q + 0.002 * n() for q in self.q],
                "dq": [0.01 * n() for _ in range(12)],
                "pos": [self.x, self.y, 0.31], "vel": [self.vx, self.vy, 0.0],
                "yaw_speed": self.wz, "battery": 33.0}

    def get_frame(self):
        import cv2
        img = np.full((480, 640, 3), 60, np.uint8)
        cv2.putText(img, "MOCK GO2  t=%.1fs" % (time.monotonic() - self._t0),
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(img, "x=%.2f y=%.2f yaw=%.2f" % (self.x, self.y, self.yaw),
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.rectangle(img, (200, 250), (440, 480), (90, 90, 90), -1)
        for i in range(5):  # 階段っぽい絵
            cv2.rectangle(img, (200, 430 - i * 45), (440, 460 - i * 45), (140, 140, 140), 2)
        return img

    def release_sport_mode(self):
        print("[mock] release_sport_mode")
        return 0

    def restore_sport_mode(self, name="normal"):
        print("[mock] restore_sport_mode(%s)" % name)
        return 0

    def mode_name(self):
        return "normal"

    def low_publisher(self):
        def send(q_sdk, kp, kd, dq=None, tau=None):
            self.q = list(q_sdk)  # モックは即時追従
        return send


def make_robot(mock: bool):
    return MockGo2() if mock else RealGo2()
