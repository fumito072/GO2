#!/usr/bin/env python
"""M0-1: 接続確認 — 状態/映像/LiDARの非駆動疎通を表示。

実行(実機):  python -m m0_teleop.check_robot            [バンドル直下から]
実行(Mock):  python -m m0_teleop.check_robot --mock
事前: PCを192.168.123.51/24等に設定しGo2(192.168.123.161)とEthernet直結。
      Secondary Development をアプリで有効化して再起動しておく。
"""
import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit("m0_teleop", 1)[0])
from common.go2_iface import make_robot, SDK_JOINT_NAMES  # noqa: E402
from common.safety import deploy_log  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--video", action="store_true", help="前面カメラも1枚取得して保存")
    ap.add_argument("--lidar", action="store_true",
                    help="LiDAR点群とrobot_odomを購読のみで確認")
    ap.add_argument("--lidar-seconds", type=float, default=3.0,
                    help="LiDARを待つ秒数 (既定3秒、最大10秒)")
    args = ap.parse_args()

    print("[check] connecting ...")
    bot = make_robot(mock=args.mock)
    time.sleep(2.0)  # 購読が届くまで待つ

    ok = True
    st = bot.state()
    if st.get("low_age", 1e9) < 1.0:
        print("[OK] LowState 受信 (age %.0f ms)" % (st["low_age"] * 1e3))
        print("     rpy = %s" % [round(v, 3) for v in st["rpy"]])
        print("     battery = %s V" % st.get("battery", "?"))
        print("     joints (SDK順):")
        for n, q in zip(SDK_JOINT_NAMES, st["q"]):
            print("       %-16s q=%+.3f" % (n, q))
    else:
        ok = False
        print("[NG] LowState が来ない → Secondary Development有効化/ネットワーク/IFACE を確認")

    if st.get("sms_age", 1e9) < 1.0:
        print("[OK] SportModeState 受信: pos=%s body_h=%.3f" %
              ([round(v, 2) for v in st.get("pos", [])], st.get("body_height", -1)))
    else:
        print("[WARN] SportModeState が来ない（sportサービス停止中なら正常）")

    if args.video:
        frame = bot.get_frame()
        if frame is not None:
            import cv2
            cv2.imwrite("check_frame.jpg", frame)
            print("[OK] 前面カメラ取得 → check_frame.jpg (%dx%d)" % (frame.shape[1], frame.shape[0]))
        else:
            ok = False
            print("[NG] 前面カメラ取得失敗")

    lidar_result = {}
    if args.lidar:
        if args.mock:
            print("[SKIP] --lidar は実機モード専用です")
        else:
            # ChannelFactory は make_robot() で初期化済み。ここではrobot向けの
            # publisher/RPCを作らず、2 topicを購読するだけに限定する。
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
            from common import config

            result = {"cloud_n": 0, "odom_n": 0, "cloud": None,
                      "odom": None, "error": None}

            def on_cloud(msg):
                try:
                    result["cloud_n"] += 1
                    result["cloud"] = {
                        "frame": str(msg.header.frame_id),
                        "points": int(msg.width) * int(msg.height),
                        "point_step": int(msg.point_step),
                        "bytes": len(msg.data),
                    }
                except Exception as exc:
                    result["error"] = "cloud callback: %r" % (exc,)

            def on_odom(msg):
                try:
                    result["odom_n"] += 1
                    p = msg.pose.pose.position
                    result["odom"] = [float(p.x), float(p.y), float(p.z)]
                except Exception as exc:
                    result["error"] = "odom callback: %r" % (exc,)

            # subscriberは待機終了まで参照を保持する。SDK/GC実装差でcallbackが
            # 途中停止するのを防ぐ。
            cloud_sub = ChannelSubscriber(config.TOPIC_LIDAR_CLOUD, PointCloud2_)
            odom_sub = ChannelSubscriber(config.TOPIC_LIDAR_ODOM, Odometry_)
            cloud_sub.Init(on_cloud, 5)
            odom_sub.Init(on_odom, 10)
            wait_s = min(10.0, max(0.5, float(args.lidar_seconds)))
            deadline = time.monotonic() + wait_s
            while time.monotonic() < deadline:
                if result["cloud_n"] >= 2 and result["odom_n"] >= 2:
                    break
                time.sleep(0.05)

            lidar_result = dict(result)
            if result["error"]:
                ok = False
                print("[NG] LiDAR callback失敗: %s" % result["error"])
            if result["cloud_n"] and result["cloud"]:
                c = result["cloud"]
                print("[OK] LiDAR点群 %d件: frame=%s points=%d step=%d bytes=%d" %
                      (result["cloud_n"], c["frame"], c["points"],
                       c["point_step"], c["bytes"]))
                if c["frame"] != "odom":
                    ok = False
                    print("[NG] 点群frameがodomではありません: %s" % c["frame"])
            else:
                ok = False
                print("[NG] LiDAR点群が来ない: topic=%s" % config.TOPIC_LIDAR_CLOUD)
            if result["odom_n"] and result["odom"]:
                print("[OK] LiDAR odom %d件: pos=%s" %
                      (result["odom_n"], [round(v, 3) for v in result["odom"]]))
            else:
                ok = False
                print("[NG] LiDAR odomが来ない: topic=%s" % config.TOPIC_LIDAR_ODOM)

    deploy_log("m0_check", ok=ok, mock=args.mock,
               video=args.video, lidar=args.lidar,
               cloud_n=lidar_result.get("cloud_n", 0),
               odom_n=lidar_result.get("odom_n", 0))
    print("[check] " + ("ALL OK" if ok else "NG あり（上のメッセージ参照）"))


if __name__ == "__main__":
    main()
