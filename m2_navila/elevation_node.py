#!/usr/bin/env python
"""M2/M3: 標高マップノード — L1点群→ローカル標高格子→height_scan(187点)をUDP配信。

これが Wave4/5 方策の「目」。sim の RayCaster と同じ意味の 187 点
（base ヨー座標系 17×11 グリッド, 値 = clip(base_z - 地面z - 0.5, -1, 1)）を作る。

実行:
  python -m m2_navila.elevation_node            # 実機（L1点群+odom）
  python -m m2_navila.elevation_node --mock     # 合成階段でテスト（受信側の疎通用）
出力:
  UDP(config.ELEV_UDP_ADDR) に JSON {"hs":[187], "topped":bool, "base_z":..,"cover":0-1} を20Hzで送る。
幾何終了判定（M2）:
  0.3m以上登った後、base_z上昇が止まり(1.5sで<0.02m)、前方グリッドが平坦(σ<0.03m)なら topped=True。
要M0確認:
  - 点群トピック名/フレーム（config.TOPIC_LIDAR_CLOUD, 既定は odom系とみなす）
  - odomトピック（無ければ sportmodestate の position+IMU yaw を使う: --pose sms）
"""
import argparse
import json
import math
import socket
import struct
import sys
import time

import numpy as np

sys.path.insert(0, __file__.rsplit("m2_navila", 1)[0])
from common import config  # noqa: E402

NX, NY = config.GRID_NX, config.GRID_NY
RES = config.GRID_RES


class RollingElevationMap:
    """ロボット周辺 SIZE×SIZE [m] の world(odom)系 標高格子（max-z + 忘却）。"""

    def __init__(self, size_m=8.0, res=0.05):
        self.res = res
        self.n = int(size_m / res)
        self.h = np.full((self.n, self.n), np.nan, np.float32)
        self.t = np.zeros((self.n, self.n), np.float64)  # 最終更新時刻
        self.cx = self.cy = 0.0  # 格子中心(world)

    def recenter(self, x, y):
        dx = int(round((x - self.cx) / self.res))
        dy = int(round((y - self.cy) / self.res))
        if abs(dx) < self.n // 4 and abs(dy) < self.n // 4:
            return
        self.h = np.roll(self.h, (-dx, -dy), axis=(0, 1))
        self.t = np.roll(self.t, (-dx, -dy), axis=(0, 1))
        if dx > 0:
            self.h[-dx:, :] = np.nan
        elif dx < 0:
            self.h[:-dx, :] = np.nan
        if dy > 0:
            self.h[:, -dy:] = np.nan
        elif dy < 0:
            self.h[:, :-dy] = np.nan
        self.cx += dx * self.res
        self.cy += dy * self.res

    def insert(self, pts_world):
        """pts: (N,3) world座標。"""
        if pts_world.shape[0] == 0:
            return
        ix = np.round((pts_world[:, 0] - self.cx) / self.res).astype(int) + self.n // 2
        iy = np.round((pts_world[:, 1] - self.cy) / self.res).astype(int) + self.n // 2
        ok = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        ix, iy, z = ix[ok], iy[ok], pts_world[ok, 2]
        now = time.monotonic()
        # 同一セル複数点は max。古い値(>3s)は上書き、それ以外は max-merge。
        stale = (now - self.t[ix, iy]) > 3.0
        cur = self.h[ix, iy]
        new = np.where(stale | np.isnan(cur), z, np.maximum(cur, z))
        self.h[ix, iy] = new
        self.t[ix, iy] = now

    def lookup(self, xs, ys):
        ix = np.round((xs - self.cx) / self.res).astype(int) + self.n // 2
        iy = np.round((ys - self.cy) / self.res).astype(int) + self.n // 2
        ix = np.clip(ix, 0, self.n - 1)
        iy = np.clip(iy, 0, self.n - 1)
        return self.h[ix, iy]


def parse_pointcloud2(msg):
    """sensor_msgs/PointCloud2 → (N,3) float32。x,y,z は float32 前提。"""
    offs = {}
    for f in msg.fields:
        offs[f.name] = f.offset
    step = msg.point_step
    buf = bytes(msg.data)
    n = len(buf) // step
    arr = np.frombuffer(buf, dtype=np.uint8).reshape(n, step)
    out = np.empty((n, 3), np.float32)
    for k, name in enumerate(("x", "y", "z")):
        o = offs[name]
        out[:, k] = arr[:, o:o + 4].copy().view(np.float32)[:, 0]
    ok = np.isfinite(out).all(axis=1)
    return out[ok]


def quat_to_yaw(w, x, y, z):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class ElevationNode:
    def __init__(self, args):
        self.args = args
        self.map = RollingElevationMap()
        self.pose = None  # (x,y,z,yaw)
        self.base_z_hist = []  # (t, z)
        self.climbed = 0.0
        self.z0 = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if not args.mock:
            self._init_dds()

    def _init_dds(self):
        from common.go2_iface import _init_dds
        _init_dds()
        from unitree_sdk2py.core.channel import ChannelSubscriber
        if self.args.pose == "odom":
            try:
                from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
                sub = ChannelSubscriber(config.TOPIC_LIDAR_ODOM, Odometry_)
                sub.Init(self._on_odom, 10)
            except Exception as e:
                print("[elev] odom購読失敗(%r) → --pose sms を使ってください" % (e,))
                raise
        else:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_, LowState_
            sub = ChannelSubscriber(config.TOPIC_SPORTSTATE, SportModeState_)
            sub.Init(self._on_sms, 10)
            self._quat = [1, 0, 0, 0]
            sub2 = ChannelSubscriber(config.TOPIC_LOWSTATE, LowState_)
            sub2.Init(self._on_low, 10)
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
        subc = ChannelSubscriber(config.TOPIC_LIDAR_CLOUD, PointCloud2_)
        subc.Init(self._on_cloud, 5)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, p.z, quat_to_yaw(q.w, q.x, q.y, q.z))

    def _on_low(self, msg):
        self._quat = list(msg.imu_state.quaternion)

    def _on_sms(self, msg):
        q = self._quat
        self.pose = (msg.position[0], msg.position[1], msg.position[2],
                     quat_to_yaw(q[0], q[1], q[2], q[3]))

    def _on_cloud(self, msg):
        if self.pose is None:
            return
        pts = parse_pointcloud2(msg)
        if self.args.cloud_frame == "body":
            x, y, z, yaw = self.pose
            c, s = math.cos(yaw), math.sin(yaw)
            R = np.array([[c, -s], [s, c]], np.float32)
            xy = pts[:, :2] @ R.T + np.array([x, y], np.float32)
            pts = np.column_stack([xy, pts[:, 2] + z])
        self.map.recenter(self.pose[0], self.pose[1])
        self.map.insert(pts)

    # --- mock: 20cm階段を合成 ---
    def mock_tick(self, t):
        vx = 0.3
        x = vx * t
        z = 0.0
        if x > 1.0:  # 1m先から0.2m段×5
            step_i = min(5, int((x - 1.0) / 0.3) + 1)
            z = 0.2 * step_i
        self.pose = (x, 0.0, z + 0.31, 0.0)
        gx = np.arange(-2, 4, 0.05) + x
        gy = np.arange(-2, 2, 0.05)
        X, Y = np.meshgrid(gx, gy, indexing="ij")
        H = np.clip(np.floor((X - 1.0) / 0.3) + 1, 0, 5) * 0.2
        H[X < 1.0] = 0.0
        self.map.recenter(x, 0)
        self.map.insert(np.column_stack([X.ravel(), Y.ravel(), H.ravel()]))

    def compute_height_scan(self):
        x, y, z, yaw = self.pose
        ix = np.arange(NX) * RES + config.GRID_X0
        iy = np.arange(NY) * RES + config.GRID_Y0
        # 並び: i = iy*17 + ix（x内側ループ / policy_spec準拠）
        Xl = np.tile(ix, NY)
        Yl = np.repeat(iy, NX)
        c, s = math.cos(yaw), math.sin(yaw)
        Xw = x + c * Xl - s * Yl
        Yw = y + s * Xl + c * Yl
        h = self.map.lookup(Xw, Yw)
        cover = float(np.isfinite(h).mean())
        # 未観測セルは「足元と同じ高さ」とみなす（保守的）
        ground_ref = z - 0.31  # base高さの公称値ぶん下
        h = np.where(np.isfinite(h), h, ground_ref)
        hs = np.clip(z - h - config.HEIGHT_SCAN_OFFSET, *config.HEIGHT_SCAN_CLIP)
        return hs.astype(np.float32), cover

    def update_termination(self):
        x, y, z, yaw = self.pose
        now = time.monotonic()
        if self.z0 is None:
            self.z0 = z
        self.climbed = max(self.climbed, z - self.z0)
        self.base_z_hist.append((now, z))
        self.base_z_hist = [(t, v) for (t, v) in self.base_z_hist if now - t < 1.6]
        rise = (self.base_z_hist[-1][1] - self.base_z_hist[0][1]) if len(self.base_z_hist) > 3 else 1.0
        # 前方0.2-0.8mの平坦度
        ix = np.arange(0.2, 0.8, 0.1)
        c, s = math.cos(yaw), math.sin(yaw)
        h = self.map.lookup(x + c * ix, y + s * ix)
        h = h[np.isfinite(h)]
        flat = (h.std() < 0.03) if h.size >= 3 else False
        return bool(self.climbed > 0.3 and abs(rise) < 0.02 and flat)

    def run(self):
        print("[elev] start (mock=%s) → UDP %s:%d" % (self.args.mock, *config.ELEV_UDP_ADDR))
        t0 = time.monotonic()
        while True:
            if self.args.mock:
                self.mock_tick(time.monotonic() - t0)
            if self.pose is None:
                time.sleep(0.05)
                continue
            hs, cover = self.compute_height_scan()
            topped = self.update_termination()
            pkt = json.dumps({"hs": [round(float(v), 4) for v in hs],
                              "topped": topped, "base_z": round(self.pose[2], 3),
                              "cover": round(cover, 2), "ts": time.time()}).encode()
            self.sock.sendto(pkt, config.ELEV_UDP_ADDR)
            time.sleep(1.0 / 20)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true", help="合成階段でテスト")
    ap.add_argument("--pose", choices=["odom", "sms"], default="sms",
                    help="自己位置ソース: LiDARオドメトリ or sportmodestate")
    ap.add_argument("--cloud-frame", choices=["world", "body"], default="world",
                    help="点群の座標系（M0で実物確認して合わせる）")
    args = ap.parse_args()
    ElevationNode(args).run()


if __name__ == "__main__":
    main()
