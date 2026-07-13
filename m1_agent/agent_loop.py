#!/usr/bin/env python
"""M1: 音声+VLMエージェントループ — 「階段まで行って登って止まれ」を言葉で。

実行:
  python -m m1_agent.agent_loop --mock                 # ロボット/API無しで疎通
  python -m m1_agent.agent_loop --mock --vlm           # Mockロボット+本物VLM
  python -m m1_agent.agent_loop                        # 実機+VLM（純正歩容/M1）
  python -m m1_agent.agent_loop --rl-backend           # M3のRL制御ループへ速度を送る版

構成:
  voice_input(PCマイク/キーボード) → 指示テキスト
  → 1.2s毎: 前面カメラ → VLM(decide) → move/turn/stop/climb/done
  → 実行: SportClient(既定) or UDP→rl_stair_controller(--rl-backend)
安全:
  - 「止まって/ストップ/stop」は VLM を経由せず即時停止（反射）。
  - VLM応答が途絶えたら watchdog が0.6sで速度0。
  - 登坂中はIMUピッチで頂上検出を補助（上り→水平が2秒続いたらdone扱い）。
"""
import argparse
import json
import socket
import sys
import time

sys.path.insert(0, __file__.rsplit("m1_agent", 1)[0])
from common import config  # noqa: E402
from common.go2_iface import make_robot  # noqa: E402
from common.safety import Watchdog, deploy_log  # noqa: E402
from m1_agent.vlm_client import VLMClient  # noqa: E402
from m1_agent.voice_input import VoiceInput  # noqa: E402

STOP_WORDS = ("止まって", "とまって", "ストップ", "stop", "やめ", "停止")


class CmdSender:
    """速度コマンドの送り先: sport(純正歩容) or rl(UDP→rl_stair_controller)。"""

    def __init__(self, bot, rl_backend=False):
        self.bot = bot
        self.rl = rl_backend
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if rl_backend else None

    def send(self, vx, vy, wz):
        if self.rl:
            self.sock.sendto(json.dumps({"vx": vx, "vy": vy, "wz": wz, "ts": time.time()}).encode(),
                             config.CMD_UDP_ADDR)
        else:
            if vx == 0 and vy == 0 and wz == 0:
                self.bot.stop_move()
            else:
                self.bot.move(vx, vy, wz)

    def stop(self):
        self.send(0.0, 0.0, 0.0)


class PitchMonitor:
    """登坂中の頂上検出補助: ピッチが+0.12rad超(上り)を経験後、|pitch|<0.05が2秒続いたらTrue。"""

    def __init__(self):
        self.saw_incline = False
        self.flat_since = None

    def update(self, pitch):
        # IsaacLab/Go2のIMU: 上りでpitchが負になる機体もあるため絶対値で判定
        if abs(pitch) > 0.12:
            self.saw_incline = True
            self.flat_since = None
        elif self.saw_incline:
            if self.flat_since is None:
                self.flat_since = time.monotonic()
            elif time.monotonic() - self.flat_since > 2.0:
                return True
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true", help="Mockロボット")
    ap.add_argument("--vlm", action="store_true", help="mock時も本物VLMを使う")
    ap.add_argument("--rl-backend", action="store_true", help="速度をM3 RLループへUDP送信")
    ap.add_argument("--decide-hz", type=float, default=0.8)
    ap.add_argument("--instruction", default="", help="音声の代わりに固定指示で開始")
    args = ap.parse_args()

    bot = make_robot(mock=args.mock)
    vlm = VLMClient(mock=(args.mock and not args.vlm))
    sender = CmdSender(bot, rl_backend=args.rl_backend)
    voice = VoiceInput() if not args.instruction else None
    wd = Watchdog(config.WATCHDOG_CMD_S, sender.stop, "agent").start()

    print("=== M1 agent loop ===  (話しかける例: 「階段まで行って登って止まって」)")
    deploy_log("m1_start", mock=args.mock, rl_backend=args.rl_backend)

    task = args.instruction or None
    climbing = False
    pm = PitchMonitor()
    last_decide = 0.0

    try:
        while True:
            # --- 音声(または新指示) ---
            if voice is not None:
                text = voice.get(timeout=0.05)
                if text == "__quit__":
                    break
                if text:
                    if any(w in text.lower() for w in STOP_WORDS):
                        print("[reflex] 停止")
                        sender.stop()
                        task, climbing = None, False
                        deploy_log("m1_stop_reflex", text=text)
                        continue
                    task = text
                    climbing = False
                    pm = PitchMonitor()
                    deploy_log("m1_task", text=text)

            if task is None:
                time.sleep(0.05)
                continue

            # --- 登坂中のピッチ補助判定 ---
            st = bot.state()
            if climbing and pm.update(st["rpy"][1]):
                print("[pitch] 頂上とみなして停止")
                sender.stop()
                deploy_log("m1_top_by_pitch")
                task, climbing = None, False
                continue

            # --- 一定周期でVLM判断 ---
            if time.monotonic() - last_decide < 1.0 / args.decide_hz:
                time.sleep(0.02)
                wd.kick()  # 実行中はコマンド維持（直近速度をsport側が保持）
                continue
            last_decide = time.monotonic()

            frame = bot.get_frame()
            if frame is None:
                sender.stop()
                continue
            status = "climbing" if climbing else "approach"
            t0 = time.time()
            act = vlm.decide(frame, task, status)
            dt = time.time() - t0
            print("[vlm %.1fs] %s" % (dt, act))
            deploy_log("m1_vlm", act=act, latency=round(dt, 2))

            a = act["action"]
            if a == "move":
                sender.send(act["vx"], 0.0, 0.0)
            elif a == "turn":
                sender.send(0.15, 0.0, act["wz"])
            elif a == "climb":
                climbing = True
                # M1(純正): 低速前進で段へ。M3(RL backend): RL方策が脚を上げる。
                sender.send(0.35, 0.0, 0.0)
            elif a == "stop":
                sender.stop()
            elif a == "done":
                print("[agent] 任務完了")
                sender.stop()
                deploy_log("m1_done", task=task)
                task, climbing = None, False
            wd.kick()
    except KeyboardInterrupt:
        pass
    finally:
        sender.stop()
        if not args.mock:
            bot.stop_move()
        deploy_log("m1_end")
        print("bye")


if __name__ == "__main__":
    main()
