#!/usr/bin/env python
"""M0-3: 前面カメラ映像を表示/保存（5090側VLM/NaVILAへの入力パイプの確認）。

実行: python -m m0_teleop.video_stream [--mock] [--save out.mp4] [--fps 10]
qキーで終了。--udp host:port を付けるとJPEGフレームをUDP送信（別PCの受信用）。
"""
import argparse
import socket
import sys
import time

import cv2

sys.path.insert(0, __file__.rsplit("m0_teleop", 1)[0])
from common.go2_iface import make_robot  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--save", default="")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--udp", default="", help="host:port へJPEG送信（任意）")
    args = ap.parse_args()

    bot = make_robot(mock=args.mock)
    vw = None
    sock = None
    addr = None
    if args.udp:
        host, port = args.udp.rsplit(":", 1)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = (host, int(port))

    n, t0 = 0, time.monotonic()
    while True:
        frame = bot.get_frame()
        if frame is None:
            time.sleep(0.2)
            continue
        n += 1
        if args.save and vw is None:
            vw = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"),
                                 args.fps, (frame.shape[1], frame.shape[0]))
        if vw is not None:
            vw.write(frame)
        if sock is not None:
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok and len(jpg) < 60000:
                sock.sendto(jpg.tobytes(), addr)
        fps = n / max(1e-3, time.monotonic() - t0)
        cv2.putText(frame, "%.1f fps" % fps, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("go2 front", frame)
        if cv2.waitKey(max(1, int(1000 / args.fps))) & 0xFF == ord("q"):
            break
    if vw is not None:
        vw.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
