"""config.py — real_mac_GO2 共通設定。

実機側で環境が違う場合はまずここを直す。トピック名やポートはFWバージョンで
変わり得るので、M0の check_robot.py で実物を確認してから進むこと。
"""
import os

# ---------- ネットワーク ----------
# Go2 とEthernet直結したときのPC側インタフェース名。
# Windows例: "イーサネット" / Linux例: "enp2s0" / 空文字ならDDSのデフォルト。
NET_IFACE = os.environ.get("GO2_IFACE", "")
# Go2 本体のIP（既定 192.168.123.161）。PC側は 192.168.123.x/24 に静的設定する。
GO2_IP = os.environ.get("GO2_IP", "192.168.123.161")

# ---------- DDS トピック（unitree_sdk2 既定） ----------
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_SPORTSTATE = "rt/sportmodestate"
# M0確認済み(2026-07-09, Go2 X実機): cloud_deskewed は frame_id=odom(world系)で配信。
# 生の rt/utlidar/cloud は frame_id=utlidar_lidar(センサ系)なので使わない。
TOPIC_LIDAR_CLOUD = os.environ.get(
    "GO2_LIDAR_CLOUD_TOPIC", "rt/utlidar/cloud_deskewed")  # L1点群 odom系
TOPIC_LIDAR_ODOM = os.environ.get(
    "GO2_LIDAR_ODOM_TOPIC", "rt/utlidar/robot_odom")       # LiDARオドメトリ

# ---------- プロセス間通信（このバンドル内） ----------
# elevation_node → rl_stair_controller: height_scan 187点 (float32×187, リトルエンディアン)
ELEV_UDP_ADDR = ("127.0.0.1", 43211)
# agent/teleop → rl_stair_controller: 速度コマンド JSON {"vx","vy","wz","ts"}
CMD_UDP_ADDR = ("127.0.0.1", 43210)
# NaVILA server (5090側で起動)
NAVILA_URL = os.environ.get("NAVILA_URL", "http://127.0.0.1:8890/act")

# ---------- height_scan グリッド（policy_spec.json と一致必須） ----------
# 並び: i = iy*17 + ix, x(前方)が内側ループ。x∈[-0.8,0.8], y∈[-0.5,0.5], 0.1m刻み。
GRID_NX, GRID_NY = 17, 11
GRID_RES = 0.1
GRID_X0, GRID_Y0 = -0.8, -0.5
HEIGHT_SCAN_OFFSET = 0.5          # 値 = clip(base_z - ground_z - 0.5, -1, 1)
HEIGHT_SCAN_FLAT = -0.225         # 平地静止時の実測値（policy_spec.flat_sample）
HEIGHT_SCAN_CLIP = (-1.0, 1.0)

# ---------- 制御 ----------
CONTROL_HZ = 50.0                 # RL方策
ACTION_SCALE = 0.25
KP, KD = 25.0, 0.5

# ---------- 安全 ----------
WATCHDOG_STATE_MS = 40.0          # lowstateがこの時間途絶えたらDamp
WATCHDOG_CMD_S = 0.6              # 上位コマンドがこの時間来なければ速度(0,0,0)
MAX_ROLL_PITCH = 0.8              # [rad] 転倒判定→Damp
VEL_LIMIT = {"vx": (-0.6, 1.0), "vy": (-0.4, 0.4), "wz": (-1.0, 1.0)}  # 上位コマンドのクランプ

# デプロイログ（全マイルストーン共通）
DEPLOY_LOG = os.path.join(os.path.dirname(__file__), "..", "deploy_log.jsonl")
