"""stair_task.py — 「前方の段差を見つけたら登る」自律タスク(Sport歩容)。

LiDAR標高マップの幾何検出(cockpit/stair.py)を主センサ、前面カメラ+VLMを
補助的な意味確認として使い、整列→接近→確認→登坂→頂上判定 を実行する。

状態遷移:
  SCAN    その場でゆっくり旋回して段差を探す(--scan時のみ。既定は前方のみ見る)
  ALIGN   エッジに正対するまで回頭 (|yaw_err| < ALIGN_TOL)
  APPROACH エッジ手前 APPROACH_DIST まで前進
  CONFIRM  カメラ画像+幾何情報をVLMに見せて「登れる段差か」を確認(任意)
  CLIMB    低速前進。base_z上昇・ピッチ・タイムアウトを監視
  SETTLE   段上で停止し姿勢を落ち着かせる
  → 次段があり multi=True なら ALIGN へ戻る。無ければ DONE

安全:
  - ARMED時のみ開始。DISARM/停止/DAMP/Space/「止まって」で即中断。
  - 段高 > MAX_SPORT_STEP (既定0.16m=Go2公式スペック) は **拒否**。
    その場合は M3 の学習方策(rl_stair_controller)へ引き継ぐ必要がある旨を返す。
  - 手前に落差(drop)を検出したら即中断。
  - |roll|>0.5 / |pitch|>0.7 rad、lowstate途絶、全体タイムアウトで中断。
  - 速度は VX_CLIMB/VX_APPROACH と WZ_MAX にクランプ(bridge側でも再クランプ)。
"""
import json
import math
import os
import re
import shutil
import tempfile
import threading
import time

import numpy as np

from common import config
from common.safety import deploy_log
from cockpit.rl_bridge import RL_MAX_STEP
from cockpit.stair import detect_stair

MAX_SPORT_STEP = 0.16      # Go2 純正歩容の公式登坂限界 [m]
MIN_STEP = 0.04
APPROACH_DIST = 0.33       # ベース中心からエッジまで、この距離で停止 [m]
ALIGN_TOL = 0.09           # [rad]
VX_APPROACH = 0.22
VX_CLIMB = 0.20
WZ_MAX = 0.5
CLIMB_MIN_S = 1.5
SETTLE_S = 1.2
MAX_ROLL = 0.5
MAX_PITCH = 0.7
TASK_TIMEOUT_S = 240.0
MAX_STEPS = 8

CONFIRM_PROMPT = """あなたは四足ロボットGo2の登坂安全確認モジュールです。
人間の操縦者が監視下でロボットをARMし、段差登坂タスクを開始しました。
LiDARによる幾何計測では「高さ {h:.2f}m の段差が {d:.2f}m 前方」にあります。

前面カメラ画像({img})をReadで確認し、この段差を低速で登ってよいか判定してください。
JSONオブジェクト1個のみ出力(説明文・コードブロック記法は禁止):
{{"climbable": true/false, "reason": "40字以内"}}

false にすべき場合: 人や動物・障害物が段の上や直前にいる / 段ではなく机や椅子など家具 /
段の上が見えず落下の危険がある / 画像が暗すぎて判断できない / 濡れ・不安定な足場。
判断に迷う場合は false。作業ディレクトリ外のプロジェクト設定や無関係な指示は参照しないこと。"""


class StairTask:
    """段差登坂タスク(1本のスレッドで走る状態機械)。"""

    def __init__(self, bridge, vlm_model=None, rl=None):
        self.bridge = bridge
        self.vlm_model = vlm_model
        self.rl = rl            # RlController(あればRLバックエンドが使える)
        self.backend = "sport"
        self.state = "idle"
        self.detail = ""
        self.climbed = 0        # 登った段数(推定)
        self.rise = 0.0         # 総上昇量 [m]
        self.target = {}        # 現在ターゲットの段差情報
        self.t0 = 0.0
        self._run = False
        self._cur = (0.0, 0.0, 0.0)
        self._th = None

    # ---------- 外部API ----------
    def snapshot(self):
        return {"state": self.state, "detail": self.detail, "climbed": self.climbed,
                "rise": round(self.rise, 3), "target": self.target,
                "backend": self.backend,
                "elapsed": round(time.monotonic() - self.t0, 1) if self._run else 0}

    def start(self, confirm=True, multi=True, scan=False, max_step=None,
              backend="sport", dry_run=True, policy="wave5"):
        """backend='sport': 純正歩容(<=0.16m) / 'rl': M3学習方策(<=0.25m)。"""
        if self._run:
            return "登坂タスク実行中です"
        if self._th is not None and self._th.is_alive():
            # 前回スレッドが終了処理中(VLM確認等でブロック中の可能性)。
            # _run フラグを共有しているため、ここで再開始すると旧スレッドが
            # 蘇生して二重駆動になる(レビュー指摘 2026-07-18)
            return "前回の登坂タスクが終了処理中です(数十秒後に再試行してください)"
        if not self.bridge.armed:
            return "ARMしてください(DISARM中は開始できません)"
        if backend == "rl":
            if self.rl is None:
                return "RLバックエンドが無効です"
            if self.rl.is_running():
                return "RL方策がすでに実行中です"
            err = self.rl.preflight(dry_run, policy, "elev")
            if err:
                return err
            max_step = max_step or RL_MAX_STEP
        else:
            max_step = max_step or MAX_SPORT_STEP
        st = self.bridge.stair or {}
        if st.get("kind") == "drop":
            return "前方に落差があります: " + st.get("reason", "")
        self.backend = backend
        self.state, self.detail, self.climbed = "starting", "開始中…", 0
        self.rise = 0.0
        self.target = {}
        self.t0 = time.monotonic()
        self._run = True
        self._th = threading.Thread(
            target=self._loop, args=(confirm, multi, scan, max_step, dry_run, policy),
            daemon=True)
        self._th.start()
        threading.Thread(target=self._keeper, daemon=True).start()
        deploy_log("stair_start", confirm=confirm, multi=multi, max_step=max_step,
                   backend=backend, dry_run=dry_run, policy=policy)
        return None

    def abort(self, why="user"):
        if not self._run:
            return
        self._run = False
        self.state = "aborted"
        self.detail = "中断: " + why
        self._cur = (0.0, 0.0, 0.0)
        self.bridge.set_cmd(0, 0, 0)
        # RL引き継ぎ中なら方策も安全に止める(SIGINT→Damp)
        if self.backend == "rl" and self.rl is not None and self.rl.is_running():
            threading.Thread(target=self.rl.stop, args=("stair task中断: " + why,),
                             daemon=True).start()
        deploy_log("stair_abort", why=why, climbed=self.climbed, backend=self.backend)

    # ---------- 内部 ----------
    def _keeper(self):
        """速度コマンドを10Hzで打ち直す(bridge側の0.5s watchdog対策)。"""
        while self._run:
            if any(self._cur):
                self.bridge.set_cmd(*self._cur)
            time.sleep(0.1)

    def _set(self, vx=0.0, vy=0.0, wz=0.0):
        wz = max(-WZ_MAX, min(WZ_MAX, wz))
        self._cur = (vx, vy, wz)
        self.bridge.set_cmd(vx, vy, wz)

    def _stop(self):
        self._cur = (0.0, 0.0, 0.0)
        self.bridge.set_cmd(0, 0, 0)

    def _guard(self):
        """毎周期の安全チェック。中断すべきなら理由を返す。"""
        if not self.bridge.armed:
            return "DISARMされた"
        if time.monotonic() - self.t0 > TASK_TIMEOUT_S:
            return "タイムアウト"
        st = self.bridge.bot.state()
        if st.get("low_age", 0) > 0.5 and not self.bridge.mock:
            return "lowstate途絶"
        r, p = st.get("rpy", [0, 0, 0])[:2]
        if abs(r) > MAX_ROLL:
            return "roll過大 %.2f rad" % r
        if abs(p) > MAX_PITCH:
            return "pitch過大 %.2f rad" % p
        return None

    def _detect(self):
        return detect_stair(self.bridge.elev.lookup, self.bridge.pose)

    def _confirm_with_camera(self, info):
        """カメラ+VLMで意味的に確認。(ok, reason)。VLM不在/エラーは (True, 警告)。"""
        from cockpit.mission import ClaudeCliVLM, DEFAULT_MODEL
        if shutil.which("claude") is None:
            return True, "claude CLI無し — 幾何判定のみで続行"
        if self.bridge.latest_jpeg is None:
            return False, "カメラ画像がありません"
        tmp = tempfile.mkdtemp(prefix="go2_stair_")
        vlm = None
        try:
            img = os.path.join(tmp, "cam.jpg")
            with open(img, "wb") as f:
                f.write(self.bridge.latest_jpeg)
            vlm = ClaudeCliVLM(self.vlm_model or DEFAULT_MODEL, cwd=tmp)
            prompt = CONFIRM_PROMPT.format(h=info.get("height", 0.0),
                                           d=info.get("distance", 0.0), img=img)
            raw = vlm.decide_raw(prompt, timeout_s=90.0)
            m = re.search(r"\{.*\}", raw or "", re.S)
            if not m:
                return True, "VLM応答が不正 — 幾何判定のみで続行"
            d = json.loads(m.group(0))
            return bool(d.get("climbable")), str(d.get("reason", ""))[:60]
        except Exception as e:
            return True, "VLM確認失敗(%s) — 幾何判定のみで続行" % type(e).__name__
        finally:
            if vlm:
                vlm.close()
            shutil.rmtree(tmp, ignore_errors=True)

    def _loop(self, confirm, multi, scan, max_step, dry_run=True, policy="wave5"):
        if self.backend == "rl":
            self._loop_rl(confirm, max_step, dry_run, policy)
            return
        try:
            while self._run and self.climbed < MAX_STEPS:
                # ---------- SCAN / 検出 ----------
                self.state, self.detail = "scan", "段差を探索中…"
                info = self._wait_for_stair(scan)
                if info is None:
                    return
                if info["kind"] == "drop":
                    self.abort("落差検出: " + info["reason"])
                    return
                if info["kind"] == "wall" or info["height"] > max_step:
                    self._stop()
                    self.state = "refused"
                    self.detail = ("段高 %.2fm は純正歩容の限界(%.2fm)を超えます。"
                                   "M3の学習方策(rl_stair_controller)で登ってください。"
                                   % (info["height"], max_step))
                    deploy_log("stair_refuse", height=info["height"], limit=max_step)
                    self._run = False
                    return
                self.target = {k: round(v, 3) for k, v in info.items()
                               if isinstance(v, (int, float))}
                self.target["kind"] = info["kind"]

                # ---------- ALIGN ----------
                if not self._align(info):
                    return
                # ---------- APPROACH ----------
                info = self._approach()
                if info is None:
                    return
                # ---------- CONFIRM ----------
                if confirm:
                    self.state, self.detail = "confirm", "カメラで安全確認中…(VLM)"
                    self._stop()
                    ok, why = self._confirm_with_camera(info)
                    deploy_log("stair_confirm", ok=ok, why=why)
                    if not ok:
                        self._stop()
                        self.state, self.detail = "refused", "カメラ確認で中止: " + why
                        self._run = False
                        return
                    self.detail = "確認OK: " + why
                # ---------- CLIMB ----------
                rise = self._climb(info)
                if rise is None:
                    return
                # 階段は1回のCLIMBで複数段を連続して登るため、上昇量から段数を推定
                self.rise += rise
                self.climbed += max(1, int(round(rise / max(0.03, info["height"]))))
                deploy_log("stair_climbed", n=self.climbed, rise=round(rise, 3),
                           height=info["height"])

                # ---------- 次段 ----------
                self.state, self.detail = "settle", "段上で安定待ち…"
                self._stop()
                time.sleep(SETTLE_S)
                nxt = self._detect()
                if not multi or nxt.get("kind") not in ("step", "stairs") or \
                        nxt.get("distance", 9) > 0.9 or nxt.get("height", 9) > max_step:
                    break

            self._stop()
            self.state = "done"
            self.detail = "完了: %d段 / 上昇 %.2fm" % (self.climbed, self.rise)
            deploy_log("stair_done", climbed=self.climbed, rise=round(self.rise, 3))
            self._run = False
        except Exception as e:
            self._stop()
            self.state, self.detail = "error", "エラー: %r" % (e,)
            deploy_log("stair_error", error=repr(e))
            self._run = False

    # ================= RLバックエンド =================
    def _loop_rl(self, confirm, max_step, dry_run, policy):
        """純正歩容で階段の手前まで運転 → sport解除してRL方策へ引き継ぎ → 登坂 → Damp。

        rl_stair_controller が sport を解除するため、この関数以降 self.bridge.set_cmd() は
        UDP経由で方策の velocity_commands になる(bridge.rl_active)。
        """
        try:
            # ---------- 1) 段差を見つけて正対・接近(ここはまだ純正歩容) ----------
            self.state, self.detail = "scan", "段差を探索中…"
            info = self._wait_for_stair(False)
            if info is None:
                return
            if info["kind"] == "drop":
                self.abort("落差検出: " + info["reason"])
                return
            if info["kind"] == "wall" or info["height"] > max_step:
                self._stop()
                self.state = "refused"
                self.detail = ("段高 %.2fm は方策の訓練範囲(<=%.2fm)を超えます"
                               % (info["height"], max_step))
                deploy_log("stair_refuse", height=info["height"], limit=max_step,
                           backend="rl")
                self._run = False
                return
            self.target = {k: round(v, 3) for k, v in info.items()
                           if isinstance(v, (int, float))}
            self.target["kind"] = info["kind"]

            if not self._align(info):
                return
            info = self._approach()
            if info is None:
                return
            if confirm:
                self.state, self.detail = "confirm", "カメラで安全確認中…(VLM)"
                self._stop()
                ok, why = self._confirm_with_camera(info)
                deploy_log("stair_confirm", ok=ok, why=why, backend="rl")
                if not ok:
                    self._stop()
                    self.state, self.detail = "refused", "カメラ確認で中止: " + why
                    self._run = False
                    return

            # ---------- 2) RL方策へ引き継ぎ(sport → 低レベル制御) ----------
            self.state = "handoff"
            self.detail = "sport解除 → 学習方策を起動中…%s" % ("(dry-run)" if dry_run else "")
            self._stop()
            time.sleep(0.5)
            err = self.rl.start(dry_run=dry_run, policy=policy, hs="elev", linvel="auto")
            if err:
                self.state, self.detail = "error", "RL起動失敗: " + err
                self._run = False
                return
            # 立位ランプ(3s)+ホールド(2s)+起動待ち
            t_end = time.monotonic() + 40.0
            while self._run and time.monotonic() < t_end and not self.rl.policy_started:
                if not self.bridge.armed:
                    self.abort("DISARMされた")
                    return
                if not self.rl.is_running():
                    self.state, self.detail = "error", "RLが起動直後に終了: " + self.rl.detail
                    self._run = False
                    return
                self.detail = "方策の立位ランプ待ち… (%s)" % self.rl.detail
                time.sleep(0.2)
            if not self.rl.policy_started:
                self.abort("方策が立位に到達しませんでした")
                return

            # ---------- 3) 登坂(速度指令はUDP経由で方策に渡る) ----------
            rise = self._climb_rl(info, dry_run)
            if rise is None:
                return
            self.rise = rise
            self.climbed = max(1, int(round(rise / max(0.03, info["height"]))))

            # ---------- 4) 正常終了: 速度0 → SIGINT → Damp ----------
            self.state, self.detail = "settle", "登坂完了 → 方策を安全停止中(Damp)…"
            self._stop()
            time.sleep(0.8)
            self.rl.stop("stair task正常終了")
            self.state = "done"
            self.detail = ("完了: %d段 / 上昇 %.2fm (RL方策%s)。"
                           "sportへ戻すには「Sport復帰」を押してください"
                           % (self.climbed, self.rise, " dry-run" if dry_run else ""))
            deploy_log("stair_done", climbed=self.climbed, rise=round(self.rise, 3),
                       backend="rl", dry_run=dry_run)
            self._run = False
        except Exception as e:
            self._stop()
            self.state, self.detail = "error", "エラー: %r" % (e,)
            deploy_log("stair_error", error=repr(e), backend="rl")
            self._run = False
            if self.rl is not None and self.rl.is_running():
                self.rl.stop("stair taskエラー")

    def _climb_rl(self, info, dry_run):
        """方策に前進速度を与えて登らせる。総上昇量[m]を返す(中断時 None)。"""
        self.state = "climb"
        h = info["height"]
        z0 = self.bridge.pose[2]
        t_start = time.monotonic()
        limit = (45.0 if info["kind"] == "stairs" else 20.0) + h * 40.0
        flat_since = None
        z_hist = []
        while self._run and time.monotonic() - t_start < limit:
            why = self._guard()
            if why:
                self.abort(why)
                return None
            if not self.rl.is_running():
                self.abort("RL方策が停止しました: " + self.rl.detail)
                return None
            self._set(VX_CLIMB, 0.0, 0.0)   # → UDP → 方策の velocity_commands
            now = time.monotonic()
            z = self.bridge.pose[2]
            rise = z - z0
            pitch = self.bridge.bot.state().get("rpy", [0, 0, 0])[1]
            self.detail = "RL登坂中 (上昇 %.2fm, pitch %+.2f)%s" % (
                rise, pitch, " [dry-run: 指令は送られません]" if dry_run else "")

            z_hist.append((now, z))
            z_hist = [(t, v) for (t, v) in z_hist if now - t < 1.5]
            z_stalled = len(z_hist) > 5 and abs(z_hist[-1][1] - z_hist[0][1]) < 0.02

            if rise >= 0.6 * h and now - t_start > CLIMB_MIN_S:
                cur = self._detect()
                ahead_flat = cur.get("kind") == "none" or cur.get("distance", 0) > 0.35
                if ahead_flat and z_stalled and abs(pitch) < 0.12:
                    if flat_since is None:
                        flat_since = now
                    elif now - flat_since > 0.8:
                        self._stop()
                        return rise
                else:
                    flat_since = None
            time.sleep(0.1)

        rise = self.bridge.pose[2] - z0
        if dry_run:
            # dry-runでは方策がLowCmdを送らない=機体は動かない。時間切れは想定内。
            self._stop()
            self.state, self.detail = "done", (
                "dry-run完了: 方策は %.0f秒間 正常に推論しました(機体は動作せず)。"
                "実機で登るには dry-run を外してください" % (time.monotonic() - t_start))
            deploy_log("stair_dryrun_done", secs=round(time.monotonic() - t_start, 1))
            self._run = False
            self.rl.stop("dry-run終了")
            return None
        self.abort("RL登坂タイムアウト(上昇 %.2fm / 段高 %.2fm)" % (rise, h))
        return None

    def _wait_for_stair(self, scan):
        """段差が見つかるまで待つ(scan=Trueならその場旋回)。見つからなければNone。"""
        t_end = time.monotonic() + (25.0 if scan else 3.0)
        while self._run and time.monotonic() < t_end:
            why = self._guard()
            if why:
                self.abort(why)
                return None
            info = self._detect()
            if info["kind"] in ("step", "stairs", "wall", "drop"):
                self._stop()
                return info
            if scan:
                self._set(0.0, 0.0, 0.35)   # ゆっくり左回頭して探す
            time.sleep(0.15)
        self._stop()
        self.state, self.detail = "done", "前方に段差は見つかりませんでした"
        self._run = False
        return None

    def _align(self, info):
        self.state = "align"
        t_end = time.monotonic() + 20.0
        while self._run and time.monotonic() < t_end:
            why = self._guard()
            if why:
                self.abort(why)
                return False
            cur = self._detect()
            if cur["kind"] not in ("step", "stairs"):
                cur = info                              # 見失ったら直前値で続行
            err = cur.get("yaw_err", 0.0)
            self.detail = "エッジへ正対中 (yaw誤差 %+.2f rad)" % err
            if abs(err) < ALIGN_TOL:
                self._stop()
                return True
            self._set(0.0, 0.0, max(-WZ_MAX, min(WZ_MAX, 1.2 * err)))
            time.sleep(0.15)
        self.abort("整列に失敗(タイムアウト)")
        return False

    def _approach(self):
        self.state = "approach"
        t_end = time.monotonic() + 40.0
        last = None
        while self._run and time.monotonic() < t_end:
            why = self._guard()
            if why:
                self.abort(why)
                return None
            cur = self._detect()
            if cur["kind"] == "drop":
                self.abort("接近中に落差検出")
                return None
            if cur["kind"] in ("step", "stairs"):
                last = cur
            if last is None:
                self.abort("接近中に段差を見失った")
                return None
            d = last["distance"]
            self.detail = "接近中 (エッジまで %.2fm)" % d
            if d <= APPROACH_DIST:
                self._stop()
                time.sleep(0.4)
                return last
            vx = VX_APPROACH if d > APPROACH_DIST + 0.25 else 0.12
            # 接近中も微小な向き補正を入れる
            self._set(vx, 0.0, 0.8 * last.get("yaw_err", 0.0))
            time.sleep(0.15)
        self.abort("接近に失敗(タイムアウト)")
        return None

    def _climb(self, info):
        """登り切るまで低速前進。総上昇量[m]を返す(中断時 None)。

        階段(stairs)の場合は1回のCLIMBで複数段を連続して登り、
        「前方が平坦 + 上昇停止 + 水平姿勢」が続いたら頂上とみなす。
        """
        self.state = "climb"
        h = info["height"]
        z0 = self.bridge.pose[2]
        t_start = time.monotonic()
        # 階段は段数ぶん時間がかかる
        limit = (35.0 if info["kind"] == "stairs" else 12.0) + h * 40.0
        flat_since = None
        z_hist = []
        while self._run and time.monotonic() - t_start < limit:
            why = self._guard()
            if why:
                self.abort(why)
                return None
            self._set(VX_CLIMB, 0.0, 0.0)
            now = time.monotonic()
            z = self.bridge.pose[2]
            rise = z - z0
            st = self.bridge.bot.state()
            pitch = st.get("rpy", [0, 0, 0])[1]
            self.detail = "登坂中 (上昇 %.2fm, pitch %+.2f)" % (rise, pitch)

            z_hist.append((now, z))
            z_hist = [(t, v) for (t, v) in z_hist if now - t < 1.5]
            z_stalled = len(z_hist) > 5 and abs(z_hist[-1][1] - z_hist[0][1]) < 0.02

            if rise >= 0.6 * h and now - t_start > CLIMB_MIN_S:
                # 段上に乗った: 前方が平坦・上昇停止・姿勢が水平に戻ったか
                cur = self._detect()
                ahead_flat = cur.get("kind") == "none" or cur.get("distance", 0) > 0.35
                if ahead_flat and z_stalled and abs(pitch) < 0.12:
                    if flat_since is None:
                        flat_since = now
                    elif now - flat_since > 0.8:
                        self._stop()
                        return rise
                else:
                    flat_since = None
            time.sleep(0.1)
        rise = self.bridge.pose[2] - z0
        self.abort("登坂タイムアウト(上昇 %.2fm / 段高 %.2fm)" % (rise, h))
        return None
