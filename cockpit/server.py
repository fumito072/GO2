#!/usr/bin/env python
"""cockpit — Go2 統合コックピット(ブラウザUI)。

カメラ / LiDAR点群(3D) / ハイトマップ / テレメトリ をリアルタイム表示し、
Sport歩容の速度コマンド操作パッドを提供する(M0テレオペのUI版)。

実行:
  python -m cockpit.server --mock            # ロボット無しで疎通(合成階段)
  GO2_IFACE=enp46s0 python -m cockpit.server # 実機。http://<このPC>:8080 を開く

安全:
  - 起動直後は DISARM 状態。UIのARMスイッチを入れるまで移動コマンドは送らない。
  - 移動コマンドが0.5s途絶えたら自動で速度0(stop_move)。
  - 停止/DAMP はARM状態に関係なく常に受け付ける。
  - 全コマンドは deploy_log.jsonl に記録。
  - 本サーバは Sport(高レベル)のみ。M3の低レベル制御は rl_stair_controller が担当。
"""
import argparse
import asyncio
import json
import math
import os
import socket
import struct
import sys
import threading
import time

import numpy as np

sys.path.insert(0, __file__.rsplit("cockpit", 1)[0])
from common import config  # noqa: E402
from common.go2_iface import make_robot, SDK_JOINT_NAMES  # noqa: E402
from common.safety import deploy_log  # noqa: E402
from cockpit.mission import (  # noqa: E402
    DEFAULT_MODEL, MissionAgent, classify_exploration_request,
)
from cockpit.rl_bridge import RlController  # noqa: E402
from cockpit.stair import detect_stair  # noqa: E402
from cockpit.stair_task import StairTask  # noqa: E402
from cockpit.explore_task import ExploreTask  # noqa: E402
from cockpit.voice import Transcriber, parse_intent  # noqa: E402
from m2_navila.elevation_node import RollingElevationMap, parse_pointcloud2, quat_to_yaw  # noqa: E402

STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")

# WSバイナリフレーム種別(先頭1バイト)
BIN_LIDAR = 1      # [u8 type][u32 n][f32 x,y,z ×n]  odom系
BIN_HEIGHTMAP = 2  # [u8 type][f32 cx][f32 cy][f32 res][u16 n][f32 h n*n] NaN=未観測


class RobotBridge:
    """ロボット(実機/Mock)と各センサの集約。コマンドは専用スレッドで10Hz送信。"""

    # UI は壁・天井を含む空間点群、elevation は足元地形だけが必要。
    # 同じ閾値を共用すると UI から base より 1m上の構造が消えてしまう。
    UI_CLOUD_RADIUS_M = 10.0
    UI_CLOUD_Z_BELOW_M = 2.5
    UI_CLOUD_Z_ABOVE_M = 3.0
    ELEV_CLOUD_RADIUS_M = 6.0
    ELEV_CLOUD_Z_BELOW_M = 1.5
    ELEV_CLOUD_Z_ABOVE_M = 1.0

    def __init__(self, mock: bool, publish_hs: bool = True):
        self.mock = mock
        self.bot = make_robot(mock=mock)
        self.elev = RollingElevationMap(size_m=8.0, res=0.05)
        self.pose = None          # (x,y,z,yaw) odom系
        self.pose_src = "none"
        self.pose_ts = 0.0        # robot_odom callback受信 monotonic時刻
        self.cloud_pts = None     # 直近点群 (N,3) odom系
        self.cloud_ts = 0.0
        self.cloud_rx_ts = 0.0
        self.cloud_hz = 0.0
        self.cloud_frame = "--"
        self.cloud_raw_n = 0
        self.cloud_ui_n = 0
        self.cloud_elev_n = 0
        self.cloud_parse_errors = 0
        self.cloud_error = None
        self.cloud_bounds = None
        self.cloud_scan_valid = False
        self._cloud_warn_ts = 0.0
        self.latest_jpeg = None   # 前面カメラJPEG bytes
        self.cam_ts = 0.0

        # --- コマンド状態(armed が False の間は移動を送らない) ---
        self.armed = False
        self.cmd = [0.0, 0.0, 0.0]
        self.cmd_ts = 0.0
        self._moving = False
        self._lock = threading.Lock()

        self.stair = {"kind": "none", "reason": "起動直後"}  # 段差検出の最新結果
        self.hs_cover = 0.0        # height_scan の観測率(RL前チェックで使う)
        self.vel_world = [0.0, 0.0, 0.0]   # odom微分で作る world系 base線速度
        self._pose_prev = None
        self.rl_active = False     # True の間、速度指令は sport ではなく UDP へ流す
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if mock:
            threading.Thread(target=self._mock_world, daemon=True).start()
        else:
            self._init_real_sensors()
        threading.Thread(target=self._cam_loop, daemon=True).start()
        threading.Thread(target=self._cmd_loop, daemon=True).start()
        threading.Thread(target=self._stair_loop, daemon=True).start()
        if publish_hs:
            threading.Thread(target=self._hs_loop, daemon=True).start()

    # ---------- 段差検出 (5Hz) ----------
    def _stair_loop(self):
        while True:
            try:
                if self.pose is not None and self.cloud_ts:
                    self.stair = detect_stair(self.elev.lookup, self.pose)
            except Exception as e:
                self.stair = {"kind": "none", "reason": "検出エラー: %r" % (e,)}
            time.sleep(0.2)

    # ---------- height_scan(187点) をUDP配信 → m3_rl.rl_stair_controller --hs elev ----------
    def _hs_loop(self):
        """elevation_node と同じ契約。コックピットをRL方策の「目」として使える。"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ix = np.arange(config.GRID_NX) * config.GRID_RES + config.GRID_X0
        iy = np.arange(config.GRID_NY) * config.GRID_RES + config.GRID_Y0
        Xl = np.tile(ix, config.GRID_NY)      # i = iy*17 + ix (x内側ループ)
        Yl = np.repeat(iy, config.GRID_NX)
        while True:
            time.sleep(1.0 / 20)
            if self.pose is None:
                continue
            try:
                x, y, z, yaw = self.pose
                c, s = math.cos(yaw), math.sin(yaw)
                h = self.elev.lookup(x + c * Xl - s * Yl, y + s * Xl + c * Yl)
                cover = float(np.isfinite(h).mean())
                self.hs_cover = cover
                h = np.where(np.isfinite(h), h, z - 0.31)   # 未観測は足元と同高
                hs = np.clip(z - h - config.HEIGHT_SCAN_OFFSET, *config.HEIGHT_SCAN_CLIP)
                # "vel": odom微分の world系線速度。sport解除後の base_lin_vel 供給源
                # (rl_stair_controller --linvel elev/auto が読む)
                pkt = json.dumps({"hs": [round(float(v), 4) for v in hs],
                                  "topped": False, "base_z": round(z, 3),
                                  "cover": round(cover, 2),
                                  "vel": [round(v, 4) for v in self.vel_world],
                                  "ts": time.time()}).encode()
                sock.sendto(pkt, config.ELEV_UDP_ADDR)
            except Exception:
                pass

    # ---------- 実機センサ ----------
    def _init_real_sensors(self):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
        self._cloud_n = 0
        self._cloud_t0 = time.monotonic()
        # subscriberをinstanceに保持する。GCでDDS callbackが止まる実装差を避ける。
        self._cloud_sub = ChannelSubscriber(config.TOPIC_LIDAR_CLOUD, PointCloud2_)
        self._cloud_sub.Init(self._on_cloud, 5)
        try:
            from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
            self._odom_sub = ChannelSubscriber(config.TOPIC_LIDAR_ODOM, Odometry_)
            self._odom_sub.Init(self._on_odom, 10)
        except Exception as e:
            print("[cockpit] odom購読失敗(%r) → sportmodestateにフォールバック" % (e,))
        # LiDAR keepalive(実機 2026-07-18 03:02: cloud_deskewed が途中で配信
        # 停止 → rt/utlidar/switch に ON を送ると復旧した)。ON は冪等な
        # センサ有効化コマンドなので5秒毎に送り続けて再発を防ぐ
        try:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
            self._lidar_sw = ChannelPublisher("rt/utlidar/switch", String_)
            self._lidar_sw.Init()
            self._lidar_sw_msg = String_
            threading.Thread(target=self._lidar_keepalive,
                             daemon=True).start()
        except Exception as e:
            print("[cockpit] LiDAR keepalive初期化失敗(%r)" % (e,))

    def _lidar_keepalive(self):
        while True:
            try:
                self._lidar_sw.Write(self._lidar_sw_msg(data="ON"))
            except Exception:
                pass
            time.sleep(5.0)

    def _update_vel(self, x, y, z):
        """odom位置の微分 → world系 base線速度(1次ローパス)。

        sport解除後は sportmodestate が止まるため、RL方策の base_lin_vel の供給源になる。
        """
        t = time.monotonic()
        if self._pose_prev is not None:
            t0, x0, y0, z0 = self._pose_prev
            dt = t - t0
            if 0.005 < dt < 0.5:
                raw = ((x - x0) / dt, (y - y0) / dt, (z - z0) / dt)
                a = 0.35
                self.vel_world = [a * r + (1 - a) * v
                                  for r, v in zip(raw, self.vel_world)]
        self._pose_prev = (t, x, y, z)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, p.z, quat_to_yaw(q.w, q.x, q.y, q.z))
        self.pose_src = "lidar_odom"
        self.pose_ts = time.monotonic()
        self._update_vel(p.x, p.y, p.z)

    def _filter_cloud(self, pts, *, radius_m, z_below_m, z_above_m):
        """pose近傍の有限な点だけを残す共通フィルタ。入力はodom系。"""
        if self.pose is None or pts.shape[0] == 0:
            return pts
        x, y, z = self.pose[0], self.pose[1], self.pose[2]
        ok = (pts[:, 2] > z - z_below_m) & (pts[:, 2] < z + z_above_m)
        ok &= ((pts[:, 0] - x) ** 2 + (pts[:, 1] - y) ** 2) < radius_m ** 2
        return pts[ok]

    def _filter_cloud_ui(self, pts):
        """UI用: 実測のz=-14m級外れ値だけ落とし、壁・棚・天井は残す。"""
        return self._filter_cloud(
            pts, radius_m=self.UI_CLOUD_RADIUS_M,
            z_below_m=self.UI_CLOUD_Z_BELOW_M, z_above_m=self.UI_CLOUD_Z_ABOVE_M)

    def _filter_cloud_elevation(self, pts):
        """階段検出用: 従来の狭い地面帯。UI用フィルタとは意図的に分離。"""
        if self.pose is None:
            return np.empty((0, 3), dtype=np.float32)
        return self._filter_cloud(
            pts, radius_m=self.ELEV_CLOUD_RADIUS_M,
            z_below_m=self.ELEV_CLOUD_Z_BELOW_M, z_above_m=self.ELEV_CLOUD_Z_ABOVE_M)

    @staticmethod
    def _pointcloud_frame(msg):
        """IDL差を吸収して header.frame_id を診断用文字列にする。"""
        frame = getattr(getattr(msg, "header", None), "frame_id", "")
        if isinstance(frame, bytes):
            frame = frame.decode("utf-8", errors="replace")
        return str(frame or "--")

    @staticmethod
    def _is_world_cloud_frame(frame):
        """robot_odom poseと同じodom frameか。map等は変換がないため許可しない。"""
        name = str(frame or "").strip("/\x00").lower()
        # prefix/suffix許可は fake/odom, map/odom, odom/lidar まで誤受理する。
        # 実機で確認済みの cloud_deskewed 契約は frame_id=odom の完全一致。
        return name == "odom"

    def _warn_cloud(self, message):
        """LiDAR callbackを止めず、同じ警告は最大2秒に1回だけ出す。"""
        self.cloud_error = message
        now = time.monotonic()
        if now - self._cloud_warn_ts > 2.0:
            print("[cockpit] LiDAR: %s" % message)
            self._cloud_warn_ts = now

    def _on_cloud(self, msg):
        self.cloud_frame = self._pointcloud_frame(msg)
        self.cloud_rx_ts = time.monotonic()
        self._cloud_n += 1
        dt = self.cloud_rx_ts - self._cloud_t0
        if dt > 2.0:
            self.cloud_hz = self._cloud_n / dt
            self._cloud_n = 0
            self._cloud_t0 = self.cloud_rx_ts
        try:
            self._process_cloud(msg)
        except Exception as e:
            self.cloud_scan_valid = False
            self.cloud_ui_n = 0
            self.cloud_elev_n = 0
            self.cloud_bounds = None
            self.cloud_parse_errors += 1
            self._warn_cloud("PointCloud2処理失敗: %r" % (e,))

    def _process_cloud(self, msg):
        """1 scanを処理する。例外は _on_cloud で捕捉しDDS callbackを生存させる。"""
        raw_pts = parse_pointcloud2(msg)  # cloud_deskewed は odom系

        self.cloud_raw_n = int(raw_pts.shape[0])
        self.cloud_error = None
        if not self._is_world_cloud_frame(self.cloud_frame):
            self.cloud_ui_n = 0
            self.cloud_elev_n = 0
            self.cloud_scan_valid = False
            self.cloud_bounds = None
            self._warn_cloud(
                "frame=%s はrobot_odomと同じodom系ではないため破棄しました。cloud_deskewedを指定してください"
                % self.cloud_frame)
            return
        if not self.cloud_raw_n:
            self.cloud_ui_n = 0
            self.cloud_elev_n = 0
            self.cloud_scan_valid = False
            self.cloud_bounds = None
            self._warn_cloud("finiteなXYZ点が0件です (frame=%s)" % self.cloud_frame)
            return

        # odom cloudとSportModeState poseは原点/時刻の同一性が保証されない。
        # robot_odomがfreshでないscanをworld mapへ混ぜずfail-closedにする。
        pose_age = time.monotonic() - float(getattr(self, "pose_ts", 0.0) or 0.0)
        if self.pose is None or self.pose_src != "lidar_odom" \
                or pose_age < 0 or pose_age > 0.6:
            self.cloud_ui_n = 0
            self.cloud_elev_n = 0
            self.cloud_scan_valid = False
            self.cloud_bounds = None
            self._warn_cloud(
                "freshなrobot_odom poseがないためscanを破棄しました "
                "(frame=%s pose_src=%s age=%.3fs)" %
                (self.cloud_frame, self.pose_src, pose_age))
            return

        ui_pts = self._filter_cloud_ui(raw_pts)
        elev_pts = self._filter_cloud_elevation(raw_pts)
        self.cloud_ui_n = int(ui_pts.shape[0])
        self.cloud_elev_n = int(elev_pts.shape[0])
        self.cloud_scan_valid = bool(self.cloud_ui_n)
        if not self.cloud_scan_valid:
            self.cloud_bounds = None
        if self.cloud_ui_n:
            self.cloud_pts = ui_pts
            self.cloud_ts = self.cloud_rx_ts
            self.cloud_bounds = {
                "min": [round(float(v), 3) for v in np.min(ui_pts, axis=0)],
                "max": [round(float(v), 3) for v in np.max(ui_pts, axis=0)],
            }
        if self.pose is not None:
            self.elev.recenter(self.pose[0], self.pose[1])
        self.elev.insert(elev_pts)
        if self.cloud_raw_n and not self.cloud_ui_n:
            self._warn_cloud(
                "raw=%d点を全reject (frame=%s / pose=%s)。座標系を確認してください"
                % (self.cloud_raw_n, self.cloud_frame, self.pose_src))

    # ---------- Mockワールド(合成階段。段高は COCKPIT_MOCK_STEP で変更可) ----------
    MOCK_STEP_H = float(os.environ.get("COCKPIT_MOCK_STEP", "0.12"))
    MOCK_STEP_X0 = 1.0     # 階段の開始位置 [m]
    MOCK_TREAD = 0.30      # 踏面の奥行き [m]
    MOCK_N_STEP = 5

    @classmethod
    def _mock_ground(cls, x):
        """odom x → 地面高さ(階段)。"""
        i = np.clip(np.floor((np.asarray(x) - cls.MOCK_STEP_X0) / cls.MOCK_TREAD) + 1,
                    0, cls.MOCK_N_STEP)
        return np.where(np.asarray(x) < cls.MOCK_STEP_X0, 0.0, i * cls.MOCK_STEP_H)

    @classmethod
    def _make_mock_scene(cls):
        """ローカル視覚確認用の固定3D fixtureを作る。

        旧Mockは ``z=ground(x)`` の水平面しかなく、3D描画が壊れていても
        上面図としては正しく見えてしまった。床/踏面に加えて左右壁、正面壁、
        階段の蹴上げ、左右非対称の箱を明示的にサンプリングする。

        Returns:
            (scene_pts, ground_pts): UI用全表面 / elevation用床・踏面のみ。
        """
        spacing = 0.06

        # 床と階段踏面。階段は x=1.0m から5段、その先は踊り場。
        gx = np.arange(-3.0, 5.01, spacing, dtype=np.float32)
        gy = np.arange(-2.16, 2.161, spacing, dtype=np.float32)
        X, Y = np.meshgrid(gx, gy, indexing="ij")
        floor = np.column_stack(
            [X.ravel(), Y.ravel(), cls._mock_ground(X.ravel())]).astype(np.float32)
        surfaces = [floor]

        # 左右壁。高さは各xでの床/踏面から2.35m。
        wall_h = np.arange(0.0, 2.351, spacing, dtype=np.float32)
        WX, WH = np.meshgrid(gx, wall_h, indexing="ij")
        wall_z = cls._mock_ground(WX.ravel()) + WH.ravel()
        for wall_y in (-2.2, 2.2):
            surfaces.append(np.column_stack([
                WX.ravel(), np.full(WX.size, wall_y, np.float32), wall_z
            ]).astype(np.float32))

        # 正面壁には中央のドア開口を残し、奥行き/左右方向を読み取れるようにする。
        front_x = np.float32(5.0)
        FY, FH = np.meshgrid(gy, wall_h, indexing="ij")
        keep = ~((np.abs(FY.ravel()) < 0.52) & (FH.ravel() < 1.82))
        front = np.column_stack([
            np.full(int(keep.sum()), front_x, np.float32),
            FY.ravel()[keep],
            cls._mock_ground(front_x) + FH.ravel()[keep],
        ]).astype(np.float32)
        surfaces.append(front)

        # 背面壁(x=-3)。地上フロアを閉じ、自律探索が有限領域で
        # frontier枯渇 → EXPLORATION_COMPLETE に到達できるようにする(E2検証)。
        back_x = np.float32(-3.0)
        BY, BH = np.meshgrid(gy, wall_h, indexing="ij")
        surfaces.append(np.column_stack([
            np.full(BY.size, back_x, np.float32),
            BY.ravel(),
            cls._mock_ground(back_x) + BH.ravel(),
        ]).astype(np.float32))

        # z=f(x)では生成できない垂直な蹴上げ面を各段へ追加。
        riser_y = np.arange(-1.08, 1.081, spacing, dtype=np.float32)
        for i in range(cls.MOCK_N_STEP):
            rx = cls.MOCK_STEP_X0 + i * cls.MOCK_TREAD
            rz = np.arange(i * cls.MOCK_STEP_H,
                           (i + 1) * cls.MOCK_STEP_H + spacing / 2,
                           spacing / 2, dtype=np.float32)
            RY, RZ = np.meshgrid(riser_y, rz, indexing="ij")
            surfaces.append(np.column_stack([
                np.full(RY.size, rx, np.float32), RY.ravel(), RZ.ravel()
            ]).astype(np.float32))

        def add_box(cx, cy, width, depth, height):
            """床から立つ直方体の上面・4側面をsceneへ追加する。"""
            base = float(cls._mock_ground(cx))
            xs = np.arange(cx - width / 2, cx + width / 2 + spacing / 2,
                           spacing, dtype=np.float32)
            ys = np.arange(cy - depth / 2, cy + depth / 2 + spacing / 2,
                           spacing, dtype=np.float32)
            hs = np.arange(0.0, height + spacing / 2, spacing, dtype=np.float32)
            BX, BY = np.meshgrid(xs, ys, indexing="ij")
            surfaces.append(np.column_stack([
                BX.ravel(), BY.ravel(), np.full(BX.size, base + height, np.float32)
            ]).astype(np.float32))
            for bx in (xs[0], xs[-1]):
                BY2, BH = np.meshgrid(ys, hs, indexing="ij")
                surfaces.append(np.column_stack([
                    np.full(BY2.size, bx, np.float32), BY2.ravel(), base + BH.ravel()
                ]).astype(np.float32))
            for by in (ys[0], ys[-1]):
                BX2, BH = np.meshgrid(xs, hs, indexing="ij")
                surfaces.append(np.column_stack([
                    BX2.ravel(), np.full(BX2.size, by, np.float32), base + BH.ravel()
                ]).astype(np.float32))

        # 左は低い箱、右は高い柱。左右反転/平面化を画面で発見しやすい。
        add_box(0.15, 1.45, 0.55, 0.48, 0.55)
        add_box(3.55, -1.25, 0.42, 0.42, 1.25)

        scene = np.concatenate(surfaces).astype(np.float32, copy=False)
        return scene, floor

    def _mock_world(self):
        scene_pts, ground_pts = self._make_mock_scene()
        while True:
            st = self.bot.state()
            x, y = st["pos"][0], st["pos"][1]
            yaw = st["rpy"][2]
            # 段の上に乗れば実際に base_z が上がる(登坂完了判定のテストに必要)
            base_z = float(self._mock_ground(x)) + 0.31
            self.pose = (x, y, base_z, yaw)
            self.pose_src = "mock"
            self.pose_ts = time.monotonic()
            self._update_vel(x, y, base_z)
            near_ui = ((scene_pts[:, 0] - x) ** 2 + (scene_pts[:, 1] - y) ** 2
                       < self.UI_CLOUD_RADIUS_M ** 2)
            near_ground = ((ground_pts[:, 0] - x) ** 2 + (ground_pts[:, 1] - y) ** 2
                           < self.ELEV_CLOUD_RADIUS_M ** 2)
            ui_pts = scene_pts[near_ui]
            elev_pts = ground_pts[near_ground]
            self.cloud_pts = ui_pts
            self.cloud_ts = time.monotonic()
            self.cloud_rx_ts = self.cloud_ts
            self.cloud_hz = 10.0
            self.cloud_frame = "odom/mock-room"
            self.cloud_raw_n = int(scene_pts.shape[0])
            self.cloud_ui_n = int(ui_pts.shape[0])
            self.cloud_elev_n = int(elev_pts.shape[0])
            self.cloud_error = None
            self.cloud_scan_valid = True
            self.cloud_bounds = {
                "min": [round(float(v), 3) for v in np.min(ui_pts, axis=0)],
                "max": [round(float(v), 3) for v in np.max(ui_pts, axis=0)],
            }
            self.elev.recenter(x, y)
            self.elev.insert(elev_pts)
            time.sleep(0.1)

    # ---------- カメラ ----------
    def _cam_loop(self):
        import cv2
        if self.mock:
            while True:
                frame = self.bot.get_frame()
                ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    self.latest_jpeg = jpg.tobytes()
                    self.cam_ts = time.monotonic()
                time.sleep(1 / 12)
            return
        # 実機: VideoClient はJPEGを返すのでそのまま流す(再エンコード不要)
        try:
            from unitree_sdk2py.go2.video.video_client import VideoClient
            v = VideoClient()
            v.SetTimeout(3.0)
            v.Init()
        except Exception as e:
            print("[cockpit] VideoClient初期化失敗: %r" % (e,))
            return
        while True:
            try:
                code, data = v.GetImageSample()
                if code == 0 and data is not None:
                    self.latest_jpeg = bytes(data)
                    self.cam_ts = time.monotonic()
            except Exception:
                pass
            time.sleep(1 / 12)

    # ---------- コマンド送信スレッド(sport_teleop.spin と同じ思想) ----------
    def _cmd_loop(self):
        while True:
            with self._lock:
                armed = self.armed
                vx, vy, wz = self.cmd
                stale = time.monotonic() - self.cmd_ts > 0.5
                rl = self.rl_active
            active = (vx or vy or wz) and not stale and armed
            try:
                if rl:
                    # 低レベル制御中: sportは解除済み。速度は rl_stair_controller へUDP送信。
                    v = (vx, vy, wz) if active else (0.0, 0.0, 0.0)
                    self._cmd_sock.sendto(
                        json.dumps({"vx": v[0], "vy": v[1], "wz": v[2],
                                    "ts": time.time()}).encode(), config.CMD_UDP_ADDR)
                    if self.mock and active:
                        self.bot.move(*v)   # Mockのみ: 方策の代わりに機体を進める(検証用)
                elif active:
                    self.bot.move(vx, vy, wz)
                    self._moving = True
                elif self._moving:
                    self.bot.stop_move()
                    self._moving = False
            except Exception as e:
                print("[cockpit] move error: %r" % (e,))
            time.sleep(0.1)

    def set_cmd(self, vx, vy, wz):
        lim = config.VEL_LIMIT
        try:
            values = (float(vx), float(vy), float(wz))
        except (TypeError, ValueError):
            values = (float("nan"),) * 3
        valid = all(math.isfinite(v) for v in values)
        with self._lock:
            if valid:
                self.cmd = [
                    max(lim["vx"][0], min(lim["vx"][1], values[0])),
                    max(lim["vy"][0], min(lim["vy"][1], values[1])),
                    max(lim["wz"][0], min(lim["wz"][1], values[2])),
                ]
            else:
                # NaNはPythonのmin/maxで上限値へ化け得る。非有限/変換不能は
                # clampせず、必ずzero commandへ倒す。
                self.cmd = [0.0, 0.0, 0.0]
            self.cmd_ts = time.monotonic()
        if not valid:
            try:
                self.bot.stop_move()
            except Exception:
                pass
            deploy_log("cockpit_cmd_rejected", reason="non-finite command")
        return valid

    def set_armed(self, on: bool):
        valid = isinstance(on, bool)
        on = on if valid else False
        with self._lock:
            self.armed = on
            self.cmd = [0.0, 0.0, 0.0]
        if not on:
            try:
                self.bot.stop_move()
            except Exception:
                pass
        deploy_log("cockpit_arm", on=on, valid=valid)
        return valid

    def do_action(self, name: str) -> str:
        """stop/damp はARM不問。姿勢系はARM時のみ。"""
        always = {"stop", "damp"}
        gated = {"stand_up", "stand_down", "balance_stand"}
        if name not in always | gated:
            return "unknown action"
        if name in gated and not self.armed:
            return "DISARM中は実行できません(ARMしてください)"
        with self._lock:
            self.cmd = [0.0, 0.0, 0.0]
        try:
            if name == "stop":
                self.bot.stop_move()
            elif name == "damp":
                self.bot.damp()
            elif name == "stand_up":
                self.bot.stand_up()
            elif name == "stand_down":
                self.bot.stand_down()
            elif name == "balance_stand":
                self.bot.balance_stand()
        except Exception as e:
            return "error: %r" % (e,)
        deploy_log("cockpit_action", name=name)
        return "ok"

    # ---------- 配信用スナップショット ----------
    def telemetry(self):
        st = self.bot.state()
        now = time.monotonic()
        if self.cloud_error:
            cloud_status = "error"
        elif not self.cloud_rx_ts:
            cloud_status = "waiting"
        elif now - self.cloud_rx_ts > 2.0:
            cloud_status = "stale"
        elif not self.cloud_ui_n:
            cloud_status = "empty"
        else:
            cloud_status = "ok"
        t = {"type": "telemetry", "ts": time.time(),
             "mock": self.mock, "armed": self.armed,
             "cmd": [round(v, 2) for v in self.cmd],
             "low_age_ms": round(st.get("low_age", 1e9) * 1e3, 1),
             "pose_src": self.pose_src,
             "pose_age": round(now - self.pose_ts, 3) if self.pose_ts else None,
             "cloud_hz": round(self.cloud_hz, 1),
             "cloud_age": round(now - self.cloud_ts, 2) if self.cloud_ts else None,
             "cloud_rx_age": round(now - self.cloud_rx_ts, 2) if self.cloud_rx_ts else None,
             "cloud_status": cloud_status,
             "cloud_frame": self.cloud_frame,
             "cloud_raw_n": self.cloud_raw_n,
             "cloud_ui_n": self.cloud_ui_n,
             "cloud_elev_n": self.cloud_elev_n,
             "cloud_reject_pct": round(
                 100.0 * (1.0 - self.cloud_ui_n / self.cloud_raw_n), 1)
                 if self.cloud_raw_n else None,
             "cloud_parse_errors": self.cloud_parse_errors,
             "cloud_error": self.cloud_error,
             "cloud_bounds": self.cloud_bounds,
             "cam_age": round(now - self.cam_ts, 2) if self.cam_ts else None}
        if "rpy" in st:
            t["rpy"] = [round(v, 4) for v in st["rpy"]]
            t["gyro"] = [round(v, 3) for v in st["gyro"]]
            t["q"] = [round(v, 3) for v in st["q"]]
            t["dq"] = [round(v, 2) for v in st["dq"]]
            t["tau"] = [round(v, 2) for v in st.get("tau", [0] * 12)]
            t["battery"] = round(st.get("battery", 0), 2)
        if "pos" in st:
            t["pos"] = [round(v, 3) for v in st["pos"]]
            t["vel"] = [round(v, 3) for v in st["vel"]]
            t["body_height"] = round(st.get("body_height", 0), 3)
        if self.pose is not None:
            t["pose"] = [round(v, 3) for v in self.pose]
        t["joint_names"] = SDK_JOINT_NAMES
        return t

    def lidar_frame(self, max_pts=8000):
        pts = self.cloud_pts
        if (not self.cloud_scan_valid or pts is None or not len(pts)
                or time.monotonic() - self.cloud_ts > 2.0):
            return None
        if len(pts) > max_pts:
            idx = np.random.choice(len(pts), max_pts, replace=False)
            pts = pts[idx]
        return struct.pack("<BI", BIN_LIDAR, len(pts)) + pts.astype("<f4").tobytes()

    def heightmap_frame(self):
        m = self.elev
        # 8m四方 0.05m格子は 160×160。UI転送用に2×2平均で 80×80 (0.1m) に縮約。
        h = m.h
        n2 = m.n // 2
        blocks = h[:n2 * 2, :n2 * 2].reshape(n2, 2, n2, 2)
        valid_n = np.isfinite(blocks).sum(axis=(1, 3))
        h4 = np.full((n2, n2), np.nan, np.float32)
        np.divide(np.nansum(blocks, axis=(1, 3)), valid_n, out=h4, where=valid_n > 0)
        hdr = struct.pack("<Bfff H", BIN_HEIGHTMAP, m.cx, m.cy, m.res * 2, n2)
        return hdr + h4.astype("<f4").tobytes()


# ================= aiohttp app =================

def build_app(bridge: RobotBridge, transcriber: Transcriber = None,
              mission: MissionAgent = None, stair: StairTask = None,
              rl: RlController = None, explore: ExploreTask = None):
    from aiohttp import web, WSMsgType

    def abort_autonomy(why):
        """自律系(AI任務/登坂タスク/RL方策/探索)をまとめて中断。"""
        if mission is not None:
            mission.abort(why)
        if stair is not None:
            stair.abort(why)
        if rl is not None and rl.is_running():
            rl.stop(why)
        if explore is not None:
            explore.abort(why)

    def stair_busy():
        return getattr(stair, "state", "idle") in (
            "starting", "scan", "align", "approach", "confirm",
            "climb", "settle", "handoff")

    def _asset_ver(name):
        try:
            return str(int(os.path.getmtime(os.path.join(STATIC_DIR, name))))
        except OSError:
            return "0"

    async def index(_req):
        # CSS/JSリンクに更新時刻ベースの ?v= を注入 → コード更新後はリロードだけで反映される
        with open(os.path.join(STATIC_DIR, "index.html"), encoding="utf-8") as f:
            html = f.read()
        html = html.replace("/static/style.css", "/static/style.css?v=" + _asset_ver("style.css"))
        html = html.replace("/static/app.js", "/static/app.js?v=" + _asset_ver("app.js"))
        return web.Response(text=html, content_type="text/html",
                            headers={"Cache-Control": "no-cache"})

    async def voice_status(_req):
        if transcriber is None:
            return web.json_response({"ready": False, "error": "--no-voice で無効化されています"})
        return web.json_response({"ready": transcriber.ready, "error": transcriber.error})

    async def voice(req):
        """音声(webm/wav等) → {"text", "intent"}。実行はクライアント側(ARMゲート経由)。"""
        if transcriber is None:
            return web.json_response({"error": "音声機能は無効です"}, status=503)
        if not transcriber.ready:
            return web.json_response(
                {"error": transcriber.error or "音声モデルをロード中です。数十秒後に再試行してください"},
                status=503)
        body = await req.read()
        if len(body) < 100:
            return web.json_response({"error": "音声が空です"}, status=400)
        import tempfile
        with tempfile.NamedTemporaryFile(suffix=".webm", delete=False) as f:
            f.write(body)
            path = f.name
        try:
            loop = asyncio.get_event_loop()
            text, evidence = await loop.run_in_executor(
                None, transcriber.transcribe_ex, path)
        except Exception as e:
            return web.json_response({"error": "認識失敗: %r" % (e,)}, status=500)
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass
        # 一段目: 契約パーサ(voice_gateway.intent_parser)。STOP_NOW 即時、
        # 移動 goal は確認フロー。命令でない発話のみ旧ルールベースへ。
        goal = None
        if explore is not None:
            goal = explore.route_text(text, modality="voice", evidence=evidence)
        if goal is not None and goal.get("handled"):
            intent = ({"action": "stop", "say": goal["say"]}
                      if goal["kind"] == "stop_now"
                      else {"action": "none", "say": goal["say"]})
            deploy_log("cockpit_voice", text=text, intent=intent.get("action"),
                       goal_kind=goal["kind"])
            return web.json_response({"text": text, "intent": intent,
                                      "goal": goal})
        intent = parse_intent(text)
        deploy_log("cockpit_voice", text=text, intent=intent.get("action"))
        return web.json_response({"text": text, "intent": intent})

    async def video(req):
        """MJPEGストリーム。"""
        resp = web.StreamResponse(headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-store"})
        await resp.prepare(req)
        last = 0.0
        try:
            while True:
                jpg = bridge.latest_jpeg
                if jpg is not None and bridge.cam_ts != last:
                    last = bridge.cam_ts
                    await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     + ("Content-Length: %d\r\n\r\n" % len(jpg)).encode()
                                     + jpg + b"\r\n")
                await asyncio.sleep(1 / 15)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp

    async def snapshot(_req):
        if bridge.latest_jpeg is None:
            return web.Response(status=503, text="no frame")
        return web.Response(body=bridge.latest_jpeg, content_type="image/jpeg")

    async def ws_handler(req):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(req)
        print("[cockpit] WS client connected: %s" % req.remote)

        async def sender():
            n = 0
            while not ws.closed:
                try:
                    t = bridge.telemetry()
                    if mission is not None:
                        t["mission"] = mission.snapshot()
                    t["stair"] = bridge.stair
                    if stair is not None:
                        t["stair_task"] = stair.snapshot()
                    if rl is not None:
                        t["rl"] = rl.snapshot()
                    if explore is not None:
                        t["explore"] = explore.snapshot()
                    await ws.send_str(json.dumps(t))
                    if n % 2 == 0:  # 5Hz
                        lf = bridge.lidar_frame()
                        if lf:
                            await ws.send_bytes(lf)
                        hf = bridge.heightmap_frame()
                        if hf:
                            await ws.send_bytes(hf)
                    if n % 10 == 0 and explore is not None:  # 1Hz
                        mf = explore.map_frame()
                        if mf:
                            await ws.send_bytes(mf)
                except (ConnectionResetError, RuntimeError):
                    break
                n += 1
                await asyncio.sleep(0.1)  # telemetry 10Hz

        task = asyncio.ensure_future(sender())
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    d = json.loads(msg.data)
                except Exception:
                    continue
                t = d.get("type")
                if t == "cmd":
                    try:
                        cmd = (float(d.get("vx", 0)), float(d.get("vy", 0)),
                               float(d.get("wz", 0)))
                    except (TypeError, ValueError):
                        cmd = (float("nan"), 0.0, 0.0)
                    if not all(math.isfinite(v) for v in cmd):
                        abort_autonomy("invalid manual command")
                        bridge.set_cmd(0, 0, 0)
                        continue
                    if any(abs(v) > 1e-9 for v in cmd):
                        # 手動入力は明示的override。自律keeperとのlast-write-winsを
                        # 許さず、全世代を失効・zero化してから手動commandを適用。
                        abort_autonomy("manual override")
                    bridge.set_cmd(*cmd)
                elif t == "arm":
                    on = d.get("on", False)
                    if not isinstance(on, bool) or not on:
                        abort_autonomy("DISARM")
                    bridge.set_armed(on)
                    await ws.send_str(json.dumps({"type": "ack", "what": "arm",
                                                  "armed": bridge.armed}))
                elif t == "action":
                    name = d.get("name", "")
                    if name in ("stop", "damp"):
                        abort_autonomy(name)
                    r = bridge.do_action(name)
                    await ws.send_str(json.dumps({"type": "ack", "what": name,
                                                  "result": r}))
                elif t == "mission":
                    instruction = d.get("instruction", "")
                    if mission is None:
                        err = "無効です"
                        what = "mission"
                    elif (classify_exploration_request(instruction) is not None
                          and explore is not None):
                        # AI MISSION欄からの探索も同じproposal/confirm経路へ統一。
                        err = explore.route_text(instruction, modality="ui")
                        what = "explore"
                    elif stair_busy():
                        err = "段差タスクの実行中です(先に停止してください)"
                        what = "mission"
                    elif rl is not None and rl.is_running():
                        err = "RL方策の実行中です(先に停止してください)"
                        what = "mission"
                    else:
                        err = mission.start(instruction)
                        what = "mission"
                    await ws.send_str(json.dumps({"type": "ack", "what": what,
                                                  "result": err or "ok"}))
                elif t == "mission_stop":
                    if mission is not None:
                        mission.abort("user")
                    await ws.send_str(json.dumps({"type": "ack", "what": "mission_stop",
                                                  "result": "ok"}))
                elif t == "stair_start":
                    if stair is None:
                        err = "無効です"
                    elif mission is not None and mission.status == "running":
                        err = "自律探索の実行中です(先に探索を停止してください)"
                    else:
                        if mission is not None:
                            mission.abort("stair task開始")  # 自律系の二重実行を防ぐ
                        err = stair.start(confirm=d.get("confirm", True),
                                          multi=d.get("multi", True),
                                          scan=d.get("scan", False),
                                          backend=d.get("backend", "sport"),
                                          dry_run=d.get("dry_run", True),
                                          policy=d.get("policy", "wave5"))
                    await ws.send_str(json.dumps({"type": "ack", "what": "stair_start",
                                                  "result": err or "ok"}))
                elif t == "stair_stop":
                    if stair is not None:
                        stair.abort("user")
                    await ws.send_str(json.dumps({"type": "ack", "what": "stair_stop",
                                                  "result": "ok"}))
                elif t == "rl_start":   # 手動でRL方策だけ起動(登坂タスクを介さない)
                    if rl is None:
                        err = "RLバックエンドが無効です"
                    elif mission is not None and mission.status == "running":
                        # 探索(自動登坂含む)と RL 方策の二重駆動を防ぐ
                        err = "自律探索の実行中です(先に探索を停止してください)"
                    else:
                        if mission is not None:
                            mission.abort("RL開始")
                        if stair is not None:
                            stair.abort("RL開始")
                        err = rl.start(dry_run=d.get("dry_run", True),
                                       policy=d.get("policy", "wave5"),
                                       hs=d.get("hs", "elev"),
                                       linvel=d.get("linvel", "auto"))
                    await ws.send_str(json.dumps({"type": "ack", "what": "rl_start",
                                                  "result": err or "ok"}))
                elif t == "rl_stop":
                    if rl is not None:
                        rl.stop("user", restore=d.get("restore", False))
                    await ws.send_str(json.dumps({"type": "ack", "what": "rl_stop",
                                                  "result": "ok"}))
                elif t == "rl_restore":
                    err = rl.restore_sport() if rl is not None else "無効です"
                    await ws.send_str(json.dumps({"type": "ack", "what": "rl_restore",
                                                  "result": err or "ok"}))
                elif t == "explore":     # 自然言語テキスト → 契約パーサ → 提案
                    if explore is None:
                        r = {"kind": "error", "say": "探索機能が無効です"}
                    else:
                        r = explore.route_text(d.get("text", ""), modality="text")
                        if not r.get("handled"):
                            r = {"kind": "non_command",
                                 "say": "命令として解釈できませんでした(探索/移動/停止の指示のみ)"}
                    await ws.send_str(json.dumps({"type": "ack", "what": "explore",
                                                  "result": r}))
                elif t == "explore_auto":  # 🤖 自律モードボタン(確認は別途必要)
                    if explore is None:
                        r = {"kind": "error", "say": "探索機能が無効です"}
                    else:
                        r = explore.route_text("部屋を探索してマップを作って",
                                               modality="ui")
                    await ws.send_str(json.dumps({"type": "ack",
                                                  "what": "explore_auto",
                                                  "result": r}))
                elif t == "explore_confirm":
                    r = (explore.confirm() if explore is not None
                         else {"kind": "error", "say": "無効です"})
                    await ws.send_str(json.dumps({"type": "ack",
                                                  "what": "explore_confirm",
                                                  "result": r}))
                elif t == "explore_cancel":
                    r = (explore.cancel_proposal("user")
                         if explore is not None else {"kind": "error"})
                    await ws.send_str(json.dumps({"type": "ack",
                                                  "what": "explore_cancel",
                                                  "result": r}))
                elif t == "explore_stop":
                    if explore is not None:
                        explore.abort("user")
                    await ws.send_str(json.dumps({"type": "ack",
                                                  "what": "explore_stop",
                                                  "result": "ok"}))
        finally:
            task.cancel()
            # クライアント切断=操縦者喪失 → 自律系を止めて停止(安全)
            abort_autonomy("WS切断")
            bridge.set_cmd(0, 0, 0)
            try:
                bridge.bot.stop_move()
            except Exception:
                pass
            print("[cockpit] WS client disconnected")
        return ws

    @web.middleware
    async def no_cache_static(request, handler):
        resp = await handler(request)
        if request.path.startswith("/static/"):
            resp.headers["Cache-Control"] = "no-cache, must-revalidate"
        return resp

    app = web.Application(client_max_size=32 * 1024 * 1024, middlewares=[no_cache_static])
    app.router.add_get("/", index)
    app.router.add_get("/video", video)
    app.router.add_get("/snapshot", snapshot)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/voice/status", voice_status)
    app.router.add_post("/voice", voice)
    app.router.add_static("/static", STATIC_DIR)
    return app


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true", help="ロボット無し(合成階段)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--no-voice", action="store_true", help="音声認識を無効化")
    ap.add_argument("--whisper", default="small", help="faster-whisperモデル (tiny/base/small/medium)")
    ap.add_argument("--vlm-model", default=None,
                    help="ミッション用claudeモデル (既定 claude-sonnet-5 / 高速なのは haiku)")
    ap.add_argument("--no-publish-hs", action="store_true",
                    help="height_scan(187点)のUDP配信を止める(既定は配信=RL方策の目になる)")
    args = ap.parse_args()

    try:
        bridge = RobotBridge(mock=args.mock, publish_hs=not args.no_publish_hs)
    except Exception as e:
        print("[cockpit] ロボット接続失敗: %s" % (e,))
        print("  - LANケーブル・ロボットの電源・GO2_IFACE(現在:'%s')を確認してください" % config.NET_IFACE)
        print("  - ロボット無しで試す場合: python3 -m cockpit.server --mock")
        sys.exit(1)
    transcriber = None if args.no_voice else Transcriber(args.whisper)
    mission = MissionAgent(bridge, model=args.vlm_model or DEFAULT_MODEL)
    rl = RlController(bridge)
    stair = StairTask(bridge, vlm_model=args.vlm_model or DEFAULT_MODEL, rl=rl)
    explore = ExploreTask(bridge, stair_task=stair, mission=mission)
    # 自律系の相互排他: AI任務/段差タスクの実行中は探索開始を拒否
    # (stair_task の属性は state。"status" ではない — 2026-07-18 修正)
    _STAIR_BUSY = ("starting", "scan", "align", "approach", "confirm",
                   "climb", "settle", "handoff")
    explore.external_busy = lambda: (
        "AI任務の実行中です(先に停止してください)"
        if mission.status == "running" else
        "段差タスクの実行中です(先に停止してください)"
        if getattr(stair, "state", "idle") in _STAIR_BUSY else
        "RL方策の実行中です(先に停止してください)"
        if rl is not None and rl.is_running() else None)
    deploy_log("cockpit_start", mock=args.mock, port=args.port)
    from aiohttp import web
    app = build_app(bridge, transcriber, mission, stair, rl, explore)
    print("=" * 60)
    print(" Go2 COCKPIT  →  http://localhost:%d  (mock=%s)" % (args.port, args.mock))
    print("   起動直後はDISARM。UIのARMスイッチONで操縦可能になります。")
    print("=" * 60)
    web.run_app(app, host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    main()
