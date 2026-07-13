"""rl_bridge.py — コックピットから M3 学習方策(rl_stair_controller)を起動・監視・停止する。

*** 低レベル制御(rt/lowcmd 50Hz)はこのプロジェクトで最も危険な操作 ***
方策の実行そのものは既存の監査済みスクリプト m3_rl/rl_stair_controller.py に任せ、
本モジュールは「起動 / 状態監視 / 安全な停止 / sportモード復帰」だけを担当する。
自前で LowCmd を出すのは、SIGINTで止まらなかった場合の非常用Dampのみ。

停止の仕組み:
  SIGINT → rl_stair_controller の `except KeyboardInterrupt` → finally で2秒かけてDamp(kp=0,kd=2)
  → プロセス終了。5秒で終わらなければ SIGKILL し、本モジュールが非常Dampを直接送る。

sport(高レベル)と低レベル制御は排他。RL中はコックピットの速度指令はUDPで方策へ渡る
(RobotBridge.rl_active が True の間)。終了後 restore_sport_mode() で純正歩容に戻す。
"""
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque

import numpy as np

from common import config
from common.safety import deploy_log

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
POLICIES = {
    "wave5": "policy/policy.pt",                    # 既定(push外乱DR, 実機向け)
    "wave4": "policy/policy_wave4_maxclimb.pt",     # 0.23m級に挑むとき(外乱裕度は低い)
}
RL_MAX_STEP = 0.25          # これを超える段はRLでも拒否(訓練分布外)
MIN_HS_COVER = 0.35         # height_scan の最低観測率(方策の目が塞がっていないか)
SIGINT_WAIT_S = 5.0


class RlController:
    """rl_stair_controller サブプロセスのライフサイクル管理。"""

    def __init__(self, bridge):
        self.bridge = bridge
        self.proc = None
        self.state = "idle"        # idle|preflight|starting|ramping|running|stopping|stopped|error
        self.detail = ""
        self.dry_run = True
        self.policy = "wave5"
        self.log = deque(maxlen=60)
        self.started_at = 0.0
        self.policy_started = False
        self._lock = threading.Lock()

    # ---------- 状態 ----------
    def snapshot(self):
        return {"state": self.state, "detail": self.detail,
                "dry_run": self.dry_run, "policy": self.policy,
                "running": self.is_running(),
                "policy_started": self.policy_started,
                "elapsed": round(time.monotonic() - self.started_at, 1) if self.started_at else 0,
                "log": list(self.log)[-12:],
                "mode": None}

    def is_running(self):
        return self.proc is not None and self.proc.poll() is None

    # ---------- 事前チェック ----------
    def preflight(self, dry_run: bool, policy: str, hs: str):
        """開始してよいか判定。問題があれば理由文字列、OKなら None。"""
        if self.is_running():
            return "RL方策はすでに実行中です"
        if not self.bridge.armed:
            return "ARMしてください"
        if policy not in POLICIES:
            return "不明なpolicy: %s" % policy
        pol_path = os.path.join(REPO, POLICIES[policy])
        if not os.path.exists(pol_path):
            return "方策ファイルがありません: %s" % pol_path
        try:
            import torch  # noqa: F401
        except ImportError:
            return "torchが入っていません (pip install torch)"
        if self.bridge.pose is None:
            return "自己位置が取れていません(LiDARオドメトリ未受信)"
        st = self.bridge.bot.state()
        if not self.bridge.mock and st.get("low_age", 1e9) > 0.5:
            return "LowStateが途絶しています"
        if hs == "elev":
            if self.bridge.hs_cover < MIN_HS_COVER:
                return ("height_scanの観測率が低すぎます (%.0f%% < %.0f%%)。"
                        "少し歩いて地図を作ってから再試行してください"
                        % (self.bridge.hs_cover * 100, MIN_HS_COVER * 100))
        stair = self.bridge.stair or {}
        if stair.get("kind") == "drop":
            return "前方に落差があります: " + stair.get("reason", "")
        if stair.get("kind") in ("step", "stairs") and stair.get("height", 0) > RL_MAX_STEP:
            return ("段高 %.2fm は方策の訓練範囲(<=%.2fm)を超えます"
                    % (stair["height"], RL_MAX_STEP))
        if not dry_run and not self.bridge.mock:
            # 実弾。ここまで来たら呼び出し側(UI)で明示的な確認が済んでいる前提。
            deploy_log("rl_live_preflight_ok", policy=policy, hs=hs,
                       hs_cover=round(self.bridge.hs_cover, 2))
        return None

    # ---------- 起動 ----------
    def start(self, dry_run=True, policy="wave5", hs="elev", linvel="auto"):
        err = self.preflight(dry_run, policy, hs)
        if err:
            self.state, self.detail = "error", err
            return err
        with self._lock:
            self.dry_run = dry_run
            self.policy = policy
            self.log.clear()
            self.policy_started = False
            self.started_at = time.monotonic()
            self.state, self.detail = "starting", "rl_stair_controller 起動中…"

            cmd = [sys.executable, "-m", "m3_rl.rl_stair_controller",
                   "--hs", hs, "--linvel", linvel, "--yes",
                   "--policy", os.path.join(REPO, POLICIES[policy])]
            if dry_run:
                cmd.append("--dry-run")
            if self.bridge.mock:
                cmd.append("--mock")
            env = dict(os.environ, PYTHONUNBUFFERED="1")
            self.proc = subprocess.Popen(
                cmd, cwd=REPO, env=env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1,
                start_new_session=True)   # SIGINTを自分のプロセス群に波及させない
            # 方策サブプロセスは常にUDPで velocity_commands を待ち受ける。dry/liveとも
            # 速度指令はUDP経由で方策へ渡す(sportには送らない=dry-run中は機体が歩かない)。
            self.bridge.rl_active = True
        deploy_log("rl_start", dry_run=dry_run, policy=policy, hs=hs, linvel=linvel)
        threading.Thread(target=self._reader, daemon=True).start()
        return None

    def _reader(self):
        """サブプロセスの出力を読んで状態を推定する。"""
        for line in self.proc.stdout:
            line = line.rstrip()
            if not line:
                continue
            self.log.append(line)
            # rl_stair_controller の実際の出力フレーズに対応(m3_rl/rl_stair_controller.py)
            if "ReleaseMode失敗" in line or "LowState未受信" in line:
                self.state, self.detail = "error", line
            elif "StandDown" in line or "ReleaseMode ..." in line:
                self.state, self.detail = "ramping", "sport解除・立位ランプ中…"
            elif "立位完了" in line or "方策開始" in line:
                self.state, self.detail = "running", "方策実行中 (50Hz)"
                self.policy_started = True
                deploy_log("rl_policy_running")
            elif "DAMP" in line or "damp exit" in line:
                self.state, self.detail = "stopping", line
        rc = self.proc.wait()
        self.bridge.rl_active = False
        self.bridge.set_cmd(0, 0, 0)
        if self.state != "error":
            self.state = "stopped"
            self.detail = "終了 (rc=%d)%s" % (rc, "" if rc == 0 else " — ログを確認")
        deploy_log("rl_exit", rc=rc, dry_run=self.dry_run)

    # ---------- 停止 ----------
    def stop(self, why="user", restore=False):
        """SIGINT で正規のDamp退出をさせる。ダメなら強制終了+非常Damp。"""
        if not self.is_running():
            self.bridge.rl_active = False
            return "実行していません"
        self.state, self.detail = "stopping", "停止中 (SIGINT→Damp): " + why
        deploy_log("rl_stop", why=why)
        self.bridge.set_cmd(0, 0, 0)     # まず速度0
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except Exception:
            try:
                self.proc.send_signal(signal.SIGINT)
            except Exception:
                pass
        t0 = time.monotonic()
        while time.monotonic() - t0 < SIGINT_WAIT_S:
            if self.proc.poll() is not None:
                break
            time.sleep(0.1)
        else:
            self.detail = "SIGINTで終了せず → 強制終了 + 非常Damp"
            deploy_log("rl_force_kill")
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except Exception:
                self.proc.kill()
            self.proc.wait(timeout=3)
            self._emergency_damp()
        self.bridge.rl_active = False
        if restore:
            self.restore_sport()
        return None

    def _emergency_damp(self):
        """SIGKILL後の保険: 自前で LowCmd(kp=0,kd=2) を2秒送って脱力させる。"""
        if self.dry_run:
            return
        try:
            send = self.bridge.bot.low_publisher()
            q = np.asarray(self.bridge.bot.state()["q"], np.float32)
            for _ in range(int(2.0 * config.CONTROL_HZ)):
                send(q, kp=0.0, kd=2.0)
                time.sleep(1.0 / config.CONTROL_HZ)
            deploy_log("rl_emergency_damp")
        except Exception as e:
            deploy_log("rl_emergency_damp_failed", error=repr(e))

    def restore_sport(self):
        """低レベル制御のあと純正歩容へ戻す(ロボットは伏せた状態から立たせる必要がある)。"""
        if self.is_running():
            return "RL実行中は復帰できません"
        try:
            code = self.bridge.bot.restore_sport_mode()
        except Exception as e:
            return "復帰失敗: %r" % (e,)
        deploy_log("rl_restore_sport", code=code)
        if code != 0:
            return "SelectMode失敗 code=%s" % code
        self.detail = "sportモードへ復帰しました(必要なら「立ち上がる」を押してください)"
        return None
