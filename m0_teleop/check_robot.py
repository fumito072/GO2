#!/usr/bin/env python
"""M0-1: 接続確認 — LowState/SportModeState/映像の疎通と基本情報を表示。

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

    deploy_log("m0_check", ok=ok, mock=args.mock)
    print("[check] " + ("ALL OK" if ok else "NG あり（上のメッセージ参照）"))


if __name__ == "__main__":
    main()
