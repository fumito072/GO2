#!/usr/bin/env python
"""M0-2: SportClient 速度テレオペ（純正歩容）。純正登坂モードの実力測定にも使う。

実行: python -m m0_teleop.sport_teleop [--mock]
キー:
  w/s 前進/後退   a/d 左/右平行移動   q/e 左/右旋回   space 停止
  1/2 速度スケール down/up            u 立ち上がる     j 伏せる
  x   Damp(脱力・緊急用)              ESC 終了(停止して抜ける)
安全: キーを離すと0.4sで自動停止（コマンドは10Hzで送り続ける方式）。
純正の階段登坂テスト: アプリで登坂(階段)モードをONにしてから w で正対前進。
  SDKからのモード切替APIはFW依存のため、M0ではアプリ併用が確実。
"""
import argparse
import sys
import threading
import time

sys.path.insert(0, __file__.rsplit("m0_teleop", 1)[0])
from common.go2_iface import make_robot  # noqa: E402
from common.safety import deploy_log  # noqa: E402

HELP = __doc__


class Teleop:
    def __init__(self, bot):
        self.bot = bot
        self.scale = 0.4
        self.vx = self.vy = self.wz = 0.0
        self.last_key = time.monotonic()
        self.running = True

    def on_key(self, ch):
        s = self.scale
        m = {"w": (s, 0, 0), "s": (-0.6 * s, 0, 0), "a": (0, 0.5 * s, 0), "d": (0, -0.5 * s, 0),
             "q": (0, 0, 1.2 * s), "e": (0, 0, -1.2 * s)}
        if ch in m:
            self.vx, self.vy, self.wz = m[ch]
            self.last_key = time.monotonic()
        elif ch == " ":
            self.vx = self.vy = self.wz = 0.0
            self.bot.stop_move()
        elif ch == "1":
            self.scale = max(0.2, self.scale - 0.1)
            print("scale=%.1f" % self.scale)
        elif ch == "2":
            self.scale = min(1.0, self.scale + 0.1)
            print("scale=%.1f" % self.scale)
        elif ch == "u":
            self.bot.stand_up()
        elif ch == "j":
            self.bot.stand_down()
        elif ch == "x":
            print("!! DAMP !!")
            self.bot.damp()
            deploy_log("m0_damp_manual")
        elif ch == "\x1b":  # ESC
            self.running = False

    def spin(self):
        # 10Hzで送信。キー入力が0.4s無ければ停止（安全）。
        while self.running:
            if time.monotonic() - self.last_key > 0.4 and (self.vx or self.vy or self.wz):
                self.vx = self.vy = self.wz = 0.0
                self.bot.stop_move()
            if self.vx or self.vy or self.wz:
                self.bot.move(self.vx, self.vy, self.wz)
            time.sleep(0.1)
        self.bot.stop_move()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true")
    args = ap.parse_args()
    print(HELP)
    bot = make_robot(mock=args.mock)
    tel = Teleop(bot)
    deploy_log("m0_teleop_start", mock=args.mock)

    th = threading.Thread(target=tel.spin, daemon=True)
    th.start()

    # 端末キー入力（Windows: msvcrt / POSIX: termios raw）
    try:
        if sys.platform == "win32":
            import msvcrt
            while tel.running:
                if msvcrt.kbhit():
                    ch = msvcrt.getwch()
                    tel.on_key(ch.lower())
                else:
                    time.sleep(0.02)
        else:
            import termios
            import tty
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                while tel.running:
                    ch = sys.stdin.read(1)
                    tel.on_key(ch.lower())
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
    except KeyboardInterrupt:
        pass
    finally:
        tel.running = False
        bot.stop_move()
        deploy_log("m0_teleop_end")
        print("bye")


if __name__ == "__main__":
    main()
