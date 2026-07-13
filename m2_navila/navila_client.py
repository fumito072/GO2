#!/usr/bin/env python
"""M2: NaVILAクライアント — 前面カメラ履歴→サーバ推論→中間コマンド→速度API実行。

実行:
  python -m m2_navila.navila_client --instruction "go to the stairs and climb up" [--mock]
  （サーバ: navila_server.py を先に起動。--mock はロボットもサーバもモック）
中間コマンドの実行則:
  forward x[m] → Move(0.35,0,0) を x/0.35 秒   / turn θ[deg] → Move(0.1,0,±0.5) を θ/(0.5rad/s)
終了:
  NaVILAが stop / elevation_node の topped=True（幾何判定, 「登る」を含む任務のみ）
"""
import argparse
import base64
import json
import socket
import sys
import threading
import time
import urllib.request

import cv2

sys.path.insert(0, __file__.rsplit("m2_navila", 1)[0])
from common import config  # noqa: E402
from common.go2_iface import make_robot  # noqa: E402
from common.safety import deploy_log  # noqa: E402


class ElevListener(threading.Thread):
    """elevation_node のUDPを受けて topped フラグを保持。"""

    def __init__(self):
        super().__init__(daemon=True)
        self.topped = False
        self.base_z = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(config.ELEV_UDP_ADDR)
        self.sock.settimeout(0.5)

    def run(self):
        while True:
            try:
                pkt, _ = self.sock.recvfrom(65535)
                d = json.loads(pkt)
                self.topped = bool(d.get("topped"))
                self.base_z = d.get("base_z")
            except socket.timeout:
                pass
            except Exception:
                pass


def call_server(url, instruction, frames_jpg):
    body = json.dumps({"instruction": instruction,
                       "frames_b64": [base64.b64encode(f).decode() for f in frames_jpg]}).encode()
    req = urllib.request.Request(url, data=body, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instruction", required=True)
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--url", default=config.NAVILA_URL)
    ap.add_argument("--use-elev", action="store_true", help="幾何終了判定(elevation_node)を併用")
    args = ap.parse_args()

    bot = make_robot(mock=args.mock)
    elev = None
    if args.use_elev:
        elev = ElevListener()
        elev.start()

    frames = []  # jpegバイト列の履歴(古→新)
    deploy_log("m2_start", instruction=args.instruction)
    climbing_task = any(k in args.instruction.lower() for k in ("climb", "登", "のぼ"))

    try:
        while True:
            f = bot.get_frame()
            if f is None:
                time.sleep(0.2)
                continue
            ok, jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frames.append(jpg.tobytes())
            frames = frames[-8:]

            act = call_server(args.url, args.instruction, frames)
            print("[navila] %s" % act)
            deploy_log("m2_act", act={k: v for k, v in act.items() if k != "raw"}, raw=act.get("raw", "")[:80])

            if act["type"] == "stop":
                bot.stop_move()
                print("[navila] STOP → 任務終了")
                break
            elif act["type"] == "forward":
                dur = act["value"] / 0.35
                t0 = time.monotonic()
                while time.monotonic() - t0 < dur:
                    bot.move(0.35, 0.0, 0.0)
                    if elev and climbing_task and elev.topped:
                        break
                    time.sleep(0.1)
                bot.stop_move()
            elif act["type"] in ("turn_left", "turn_right"):
                w = 0.5 if act["type"] == "turn_left" else -0.5
                dur = (act["value"] * 3.14159 / 180.0) / 0.5
                t0 = time.monotonic()
                while time.monotonic() - t0 < dur:
                    bot.move(0.1, 0.0, w)
                    time.sleep(0.1)
                bot.stop_move()

            if elev and climbing_task and elev.topped:
                bot.stop_move()
                print("[navila] 幾何判定: 登坂完了(平坦化+高度上昇停止) → 停止")
                deploy_log("m2_topped_geometric")
                break
    except KeyboardInterrupt:
        pass
    finally:
        bot.stop_move()
        deploy_log("m2_end")


if __name__ == "__main__":
    main()
