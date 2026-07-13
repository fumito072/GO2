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
from cockpit.mission import DEFAULT_MODEL, MissionAgent  # noqa: E402
from cockpit.rl_bridge import RlController  # noqa: E402
from cockpit.stair import detect_stair  # noqa: E402
from cockpit.stair_task import StairTask  # noqa: E402
from cockpit.voice import Transcriber, parse_intent  # noqa: E402
from m2_navila.elevation_node import RollingElevationMap, parse_pointcloud2, quat_to_yaw  # noqa: E402

STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")

# WSバイナリフレーム種別(先頭1バイト)
BIN_LIDAR = 1      # [u8 type][u32 n][f32 x,y,z ×n]  odom系
BIN_HEIGHTMAP = 2  # [u8 type][f32 cx][f32 cy][f32 res][u16 n][f32 h n*n] NaN=未観測


class RobotBridge:
    """ロボット(実機/Mock)と各センサの集約。コマンドは専用スレッドで10Hz送信。"""

    def __init__(self, mock: bool, publish_hs: bool = True):
        self.mock = mock
        self.bot = make_robot(mock=mock)
        self.elev = RollingElevationMap(size_m=8.0, res=0.05)
        self.pose = None          # (x,y,z,yaw) odom系
        self.pose_src = "none"
        self.cloud_pts = None     # 直近点群 (N,3) odom系
        self.cloud_ts = 0.0
        self.cloud_hz = 0.0
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
        sub = ChannelSubscriber(config.TOPIC_LIDAR_CLOUD, PointCloud2_)
        sub.Init(self._on_cloud, 5)
        try:
            from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
            so = ChannelSubscriber(config.TOPIC_LIDAR_ODOM, Odometry_)
            so.Init(self._on_odom, 10)
        except Exception as e:
            print("[cockpit] odom購読失敗(%r) → sportmodestateにフォールバック" % (e,))

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
        self._update_vel(p.x, p.y, p.z)

    def _filter_cloud(self, pts):
        """L1点群の外れ値除去(実測でz=-14m級のゴミが混ざる)。"""
        if self.pose is None or pts.shape[0] == 0:
            return pts
        x, y, z = self.pose[0], self.pose[1], self.pose[2]
        ok = (pts[:, 2] > z - 1.5) & (pts[:, 2] < z + 1.0)
        ok &= ((pts[:, 0] - x) ** 2 + (pts[:, 1] - y) ** 2) < 36.0  # 半径6m
        return pts[ok]

    def _on_cloud(self, msg):
        pts = self._filter_cloud(parse_pointcloud2(msg))  # cloud_deskewed は odom系
        self.cloud_pts = pts
        self.cloud_ts = time.monotonic()
        self._cloud_n += 1
        dt = time.monotonic() - self._cloud_t0
        if dt > 2.0:
            self.cloud_hz = self._cloud_n / dt
            self._cloud_n = 0
            self._cloud_t0 = time.monotonic()
        if self.pose is not None:
            self.elev.recenter(self.pose[0], self.pose[1])
        self.elev.insert(pts)
        # odomが無い場合のposeフォールバック(sms position + IMU yaw)
        if self.pose_src != "lidar_odom":
            st = self.bot.state()
            if "pos" in st:
                q = st.get("quat", [1, 0, 0, 0])
                self.pose = (st["pos"][0], st["pos"][1], st["pos"][2],
                             quat_to_yaw(q[0], q[1], q[2], q[3]))
                self.pose_src = "sms"
                self._update_vel(*self.pose[:3])

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

    def _mock_world(self):
        rng = np.random.default_rng(0)
        while True:
            st = self.bot.state()
            x, y = st["pos"][0], st["pos"][1]
            yaw = st["rpy"][2]
            # 段の上に乗れば実際に base_z が上がる(登坂完了判定のテストに必要)
            base_z = float(self._mock_ground(x)) + 0.31
            self.pose = (x, y, base_z, yaw)
            self.pose_src = "mock"
            self._update_vel(x, y, base_z)
            n = 2500
            px = rng.uniform(x - 4, x + 4, n)
            py = rng.uniform(y - 4, y + 4, n)
            pz = self._mock_ground(px) + rng.normal(0, 0.005, n)
            pts = np.column_stack([px, py, pz]).astype(np.float32)
            self.cloud_pts = pts
            self.cloud_ts = time.monotonic()
            self.cloud_hz = 10.0
            self.elev.recenter(x, y)
            self.elev.insert(pts)
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
        with self._lock:
            self.cmd = [max(lim["vx"][0], min(lim["vx"][1], float(vx))),
                        max(lim["vy"][0], min(lim["vy"][1], float(vy))),
                        max(lim["wz"][0], min(lim["wz"][1], float(wz)))]
            self.cmd_ts = time.monotonic()

    def set_armed(self, on: bool):
        with self._lock:
            self.armed = bool(on)
            self.cmd = [0.0, 0.0, 0.0]
        if not on:
            try:
                self.bot.stop_move()
            except Exception:
                pass
        deploy_log("cockpit_arm", on=bool(on))

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
        t = {"type": "telemetry", "ts": time.time(),
             "mock": self.mock, "armed": self.armed,
             "cmd": [round(v, 2) for v in self.cmd],
             "low_age_ms": round(st.get("low_age", 1e9) * 1e3, 1),
             "pose_src": self.pose_src,
             "cloud_hz": round(self.cloud_hz, 1),
             "cloud_age": round(time.monotonic() - self.cloud_ts, 2) if self.cloud_ts else None,
             "cam_age": round(time.monotonic() - self.cam_ts, 2) if self.cam_ts else None}
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
        if pts is None or time.monotonic() - self.cloud_ts > 2.0:
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
        with np.errstate(invalid="ignore"):
            h4 = np.nanmean(h[:n2 * 2, :n2 * 2].reshape(n2, 2, n2, 2), axis=(1, 3))
        hdr = struct.pack("<Bfff H", BIN_HEIGHTMAP, m.cx, m.cy, m.res * 2, n2)
        return hdr + h4.astype("<f4").tobytes()


# ================= aiohttp app =================

def build_app(bridge: RobotBridge, transcriber: Transcriber = None,
              mission: MissionAgent = None, stair: StairTask = None,
              rl: RlController = None):
    from aiohttp import web, WSMsgType

    def abort_autonomy(why):
        """自律系(AI任務/登坂タスク/RL方策)をまとめて中断。"""
        if mission is not None:
            mission.abort(why)
        if stair is not None:
            stair.abort(why)
        if rl is not None and rl.is_running():
            rl.stop(why)

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
            text = await loop.run_in_executor(None, transcriber.transcribe, path)
        except Exception as e:
            return web.json_response({"error": "認識失敗: %r" % (e,)}, status=500)
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass
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
                    await ws.send_str(json.dumps(t))
                    if n % 2 == 0:  # 5Hz
                        lf = bridge.lidar_frame()
                        if lf:
                            await ws.send_bytes(lf)
                        hf = bridge.heightmap_frame()
                        if hf:
                            await ws.send_bytes(hf)
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
                    bridge.set_cmd(d.get("vx", 0), d.get("vy", 0), d.get("wz", 0))
                elif t == "arm":
                    on = d.get("on", False)
                    if not on:
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
                    err = mission.start(d.get("instruction", "")) if mission else "無効です"
                    await ws.send_str(json.dumps({"type": "ack", "what": "mission",
                                                  "result": err or "ok"}))
                elif t == "mission_stop":
                    if mission is not None:
                        mission.abort("user")
                    await ws.send_str(json.dumps({"type": "ack", "what": "mission_stop",
                                                  "result": "ok"}))
                elif t == "stair_start":
                    if stair is None:
                        err = "無効です"
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
    deploy_log("cockpit_start", mock=args.mock, port=args.port)
    from aiohttp import web
    app = build_app(bridge, transcriber, mission, stair, rl)
    print("=" * 60)
    print(" Go2 COCKPIT  →  http://localhost:%d  (mock=%s)" % (args.port, args.mock))
    print("   起動直後はDISARM。UIのARMスイッチONで操縦可能になります。")
    print("=" * 60)
    web.run_app(app, host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    main()
