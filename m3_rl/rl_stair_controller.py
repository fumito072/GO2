#!/usr/bin/env python
"""M3: 学習方策(Wave4/5)の実機実行 — LowCmd 50Hz 制御ループ。

*** 必ず人間立会い・緊急停止(物理)を手元に。初回は --dry-run と吊り下げから ***

実行例:
  python -m m3_rl.rl_stair_controller --mock --dry-run          # PCだけで疎通
  python -m m3_rl.rl_stair_controller --dry-run                 # 実機state読取のみ(指令送らず)
  python -m m3_rl.rl_stair_controller --hs flat                 # 平地歩行(知覚なし=平地仮定)
  python -m m3_rl.rl_stair_controller --hs elev                 # 階段(elevation_node併用)

流れ（モード切替の振付）:
  1) sport歩容で階段の手前へ(M1/M2) → その場で停止
  2) 本スクリプト起動 → 確認プロンプト → StandDown(伏せ) → ReleaseMode(sport解除)
  3) 現姿勢→立位へ3秒ランプ(kpも0→25へ) → 2秒ホールド
  4) 50HzでRL方策実行。速度コマンドはUDP(m1_agentの --rl-backend か teleop)から
  5) 終了/異常 → Damp(kp0,kd2)で安全に脱力
安全: lowstate>40ms途絶/|roll,pitch|>0.8rad/目標角リミット/1stepあたり変化率制限。
"""
import argparse
import json
import math
import socket
import sys
import threading
import time

import numpy as np

sys.path.insert(0, __file__.rsplit("m3_rl", 1)[0])
from common import config  # noqa: E402
from common.go2_iface import make_robot  # noqa: E402
from common.safety import Watchdog, clamp_joint_targets, deploy_log  # noqa: E402
from m3_rl import joint_map as jm  # noqa: E402
from m3_rl.obs_builder import ObsBuilder  # noqa: E402

MAX_DQ_PER_STEP = 0.30  # [rad/step] 目標角の変化率制限（安全）


class UdpCmd(threading.Thread):
    """m1_agent/teleop からの速度コマンド受信。途絶時は(0,0,0)。"""

    def __init__(self):
        super().__init__(daemon=True)
        self.cmd = np.zeros(3, np.float32)
        self.ts = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(config.CMD_UDP_ADDR)
        self.sock.settimeout(0.5)

    def run(self):
        while True:
            try:
                pkt, _ = self.sock.recvfrom(4096)
                d = json.loads(pkt)
                vx = max(config.VEL_LIMIT["vx"][0], min(config.VEL_LIMIT["vx"][1], float(d.get("vx", 0))))
                vy = max(config.VEL_LIMIT["vy"][0], min(config.VEL_LIMIT["vy"][1], float(d.get("vy", 0))))
                wz = max(config.VEL_LIMIT["wz"][0], min(config.VEL_LIMIT["wz"][1], float(d.get("wz", 0))))
                self.cmd = np.array([vx, vy, wz], np.float32)
                self.ts = time.monotonic()
            except socket.timeout:
                pass
            except Exception:
                pass

    def get(self):
        if time.monotonic() - self.ts > config.WATCHDOG_CMD_S:
            return np.zeros(3, np.float32)
        return self.cmd


class UdpElev(threading.Thread):
    """elevation_node / cockpit からの height_scan 受信。古い(>0.3s)場合は None。

    パケットに "vel"(world系 base線速度)が含まれていれば base_lin_vel の供給源になる
    (sport解除後は sportmodestate が止まるため。--linvel elev/auto で使用)。
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.hs = None
        self.vel = None
        self.ts = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(config.ELEV_UDP_ADDR)
        self.sock.settimeout(0.5)

    def run(self):
        while True:
            try:
                pkt, _ = self.sock.recvfrom(65535)
                d = json.loads(pkt)
                self.hs = np.array(d["hs"], np.float32)
                v = d.get("vel")
                self.vel = np.array(v, np.float32) if v is not None else None
                self.ts = time.monotonic()
            except socket.timeout:
                pass
            except Exception:
                pass

    def _fresh(self):
        return self.ts and time.monotonic() - self.ts <= 0.3

    def get(self):
        if self.hs is None or not self._fresh():
            return None
        return self.hs

    def get_vel(self):
        if self.vel is None or not self._fresh():
            return None
        return self.vel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default=None, help="TorchScript .pt（既定: policy/policy.pt）")
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="LowCmdを送らず表示のみ")
    ap.add_argument("--hs", choices=["flat", "elev"], default="flat",
                    help="height_scanソース: flat=平地仮定 / elev=elevation_node(UDP)")
    ap.add_argument("--linvel", choices=["sms", "zero", "elev", "auto"], default="sms",
                    help="base線速度の源: sms=sportmodestate / zero=0固定 / "
                         "elev=elevationパケットのvel(LiDARオドメトリ微分) / "
                         "auto=smsが生きていればsms、途絶ならelev→zeroへフォールバック")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--yes", action="store_true", help="確認プロンプトをスキップ")
    args = ap.parse_args()

    import torch
    pol_path = args.policy or (__file__.rsplit("m3_rl", 1)[0] + "policy/policy.pt")
    policy = torch.jit.load(pol_path, map_location=args.device)
    policy.eval()
    print("[rl] policy: %s" % pol_path)

    bot = make_robot(mock=args.mock)
    time.sleep(1.5)
    st = bot.state()
    if st.get("low_age", 1e9) > 1.0 and not args.mock:
        print("[rl] LowState未受信 — 接続/Secondary Development を確認")
        return

    cmd_rx = UdpCmd(); cmd_rx.start()
    # height_scan が不要でも、--linvel elev/auto なら elevation パケットを購読する
    elev_rx = UdpElev() if (args.hs == "elev" or args.linvel in ("elev", "auto")) else None
    if elev_rx:
        elev_rx.start()
    ob = ObsBuilder()

    # ---- 危険操作の確認 ----
    if not args.yes and not args.dry_run:
        print("\n*** 低レベル制御を開始します ***\n"
              "  - ロボットは吊り下げ or 周囲2mクリアですか？\n"
              "  - 物理的な緊急停止（電源/Dampボタン）は手元にありますか？\n"
              "続行するには 'go' を入力: ", end="")
        if input().strip().lower() != "go":
            print("中止")
            return

    aborted = {"flag": False, "why": ""}

    def do_damp(why=""):
        if aborted["flag"]:
            return
        aborted["flag"] = True
        aborted["why"] = why
        print("\n[rl] !! DAMP (%s) !!" % why)
        deploy_log("m3_damp", why=why)

    # lowstate途絶watchdog（Pythonループのジッタを考慮し下限100ms）
    wd_period = max(0.1, config.WATCHDOG_STATE_MS / 1e3)
    wd_state = Watchdog(wd_period, lambda: do_damp("lowstate stale"), "state")

    # ---- モード切替: 伏せ → sport解除 ----
    if not args.dry_run and not args.mock:
        print("[rl] StandDown → ReleaseMode ...")
        bot.stand_down()
        time.sleep(3.0)
    code = bot.release_sport_mode() if not args.dry_run else 0
    if code != 0:
        print("[rl] ReleaseMode失敗 code=%s（FW/型番の開放状態を確認）" % code)
        return
    deploy_log("m3_lowlevel_start", hs=args.hs, dry=args.dry_run, policy=pol_path)

    send = bot.low_publisher()
    dt = 1.0 / config.CONTROL_HZ

    def lowcmd(q_sdk, kp, kd):
        if not args.dry_run:
            send(q_sdk, kp, kd)

    try:
        # ---- 立位ランプ(3s) + ホールド(2s) ----
        st = bot.state()
        q0_sdk = np.asarray(st["q"], np.float32)
        target_sdk = jm.DEFAULT_POS_SDK.copy()
        wd_state.start()  # ここから途絶監視を開始
        T = int(3.0 * config.CONTROL_HZ)
        for i in range(T):
            a = (i + 1) / T
            q = (1 - a) * q0_sdk + a * target_sdk
            lowcmd(q, kp=a * jm.KP, kd=jm.KD)
            if bot.state().get("low_age", 1) < wd_period:
                wd_state.kick()
            if aborted["flag"]:
                raise RuntimeError("aborted in ramp")
            time.sleep(dt)
        for _ in range(int(2.0 * config.CONTROL_HZ)):
            lowcmd(target_sdk, jm.KP, jm.KD)
            if bot.state().get("low_age", 1) < wd_period:
                wd_state.kick()
            if aborted["flag"]:
                raise RuntimeError("aborted in hold")
            time.sleep(dt)
        print("[rl] 立位完了 → 方策開始（UDPで速度コマンドを送ってください。Ctrl+Cで終了）")
        deploy_log("m3_policy_start")

        # ---- 50Hz 方策ループ ----
        import torch
        q_prev_target = target_sdk.copy()
        hs_warned = False
        linvel_src = None
        n = 0
        while not aborted["flag"]:
            t0 = time.perf_counter()
            st = bot.state()
            if st.get("low_age", 1e9) < wd_period:
                wd_state.kick()
            r, p = st["rpy"][0], st["rpy"][1]
            if abs(r) > config.MAX_ROLL_PITCH or abs(p) > config.MAX_ROLL_PITCH:
                do_damp("tipover rpy=(%.2f,%.2f)" % (r, p))
                break

            hs = None
            if args.hs == "elev" and elev_rx is not None:
                hs = elev_rx.get()
                if hs is None and not hs_warned:
                    print("[rl] WARN: elevation途絶 → 平地仮定にフォールバック")
                    hs_warned = True

            # ---- base_lin_vel の供給源を決める(方策性能を左右する最重要の観測) ----
            if args.linvel == "sms":
                lin, src = None, "sms"
            elif args.linvel == "zero":
                lin, src = np.zeros(3), "zero"
            elif args.linvel == "elev":
                v = elev_rx.get_vel() if elev_rx else None
                lin, src = (v, "elev") if v is not None else (np.zeros(3), "zero(elev途絶)")
            else:  # auto
                if st.get("sms_age", 1e9) < 0.3:
                    lin, src = None, "sms"
                else:
                    v = elev_rx.get_vel() if elev_rx else None
                    lin, src = (v, "elev") if v is not None else (np.zeros(3), "zero")
            if src != linvel_src:
                linvel_src = src
                print("[rl] base_lin_vel源: %s" % src)
                deploy_log("m3_linvel_src", src=src)

            obs = ob.build(st, cmd_rx.get(), height_scan=hs, lin_vel_world=lin)

            with torch.inference_mode():
                act = policy(torch.from_numpy(obs).unsqueeze(0).to(args.device))
            act = act.squeeze(0).cpu().numpy().astype(np.float32)
            ob.set_action(act)

            q_t_isaac = ob.action_to_q_target_isaac(act)
            q_t_sdk = jm.isaac_to_sdk(q_t_isaac)
            q_t_sdk = np.asarray(clamp_joint_targets(q_t_sdk, jm.SDK_JOINT_NAMES), np.float32)
            q_t_sdk = np.clip(q_t_sdk, q_prev_target - MAX_DQ_PER_STEP, q_prev_target + MAX_DQ_PER_STEP)
            q_prev_target = q_t_sdk
            lowcmd(q_t_sdk, jm.KP, jm.KD)

            n += 1
            if args.dry_run and n % 50 == 0:
                print("[dry] cmd=%s act|max|=%.2f q_t[0:3]=%s hs=%s" %
                      (cmd_rx.get().round(2).tolist(), float(np.abs(act).max()),
                       q_t_sdk[:3].round(2).tolist(), "elev" if hs is not None else "flat"))
            rest = dt - (time.perf_counter() - t0)
            if rest > 0:
                time.sleep(rest)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        # ---- Damp: 2秒かけて脱力（kd減衰のみ）----
        print("[rl] damp exit ...")
        st = bot.state()
        qn = np.asarray(st["q"], np.float32)
        for _ in range(int(2.0 * config.CONTROL_HZ)):
            lowcmd(qn, kp=0.0, kd=2.0)
            time.sleep(dt)
        deploy_log("m3_end", why=aborted["why"])
        print("bye (%s)" % (aborted["why"] or "normal"))


if __name__ == "__main__":
    main()
