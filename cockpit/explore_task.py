"""explore_task — 音声/自然言語 → 契約パーサ → Mission FSM → 自律探索マッピング。

docs/12 §2 のエッジ単体統合ランタイム。mission.py / stair_task.py と同じ
「bridge を持つスレッド駆動タスク」の流儀で、新パイプライン
(contracts / voice_gateway / mission / navigation / perception / realtime)を
実機コックピットへ接続する。

invariant(docs/CLAUDE.md §4):
  1. パーサ/UI/explorer は GoalSpec / goal pose の提案のみ。actuator owner にならない。
  2. 実行経路は GoalSpec → Mission FSM → Command Arbiter →
     Exclusive Actuation Gateway → Sport backend(bridge.set_cmd)の一方向。
  8. 「止まって」/完了は Damp ではなく Controlled Stop → ACTIVE_HOLD。
  9. unknown ≠ free。pose/点群の途絶は fail-closed(新規移動を発行しない)。
     例外(操作者承認 2026-07-17): EXPLORE_AND_MAP の移動計画のみ optimistic
     (非OCCUPIED=通行可)。未踏域へ踏み出せるようにするため。障害物/落差/
     inflation は引き続きブロックし、走行中も mapper が実況で OCCUPIED 化する。
     waypoint 復帰・地図表示・その他の消費側は従来どおり FREE のみ。

開始条件: ARM 中のみ + 確認済み GoalSpec のみ。自動再開なし。

自動登坂(操作者承認 2026-07-18):
  探索中に段差候補(幾何検出 or 行き詰まり時のVLM探索)を見つけたら、
  停止して VLM(stair_scout)で「登れる階段か」を判定し、登れるなら
  stair_task(sport backend, 段高≤0.16m)へ引き渡して登坂、成功後は
  地図を新フロアへ切替えて探索を続行する。確認済み探索 GoalSpec の
  読み上げに登坂を含む旨を明記しており、FSM は登坂中も EXPLORING の
  まま(登坂=探索のサブ行動)。STOP_NOW/DISARM は登坂中も即時有効。
"""
import json
import math
import os
import random
import re
import threading
import time
import struct
import uuid

import numpy as np

from common.safety import deploy_log
from contracts.command_envelope import (
    ArbiterPriority, CommandEnvelope, RequestedMode, ServerAttribution,
    LocomotionBackend,
)
from contracts.goal_spec import (
    SCHEMA_VERSION, Confirmation, ConfirmationStatus, GoalProposal, GoalSpec,
    Intent, Modality, Precondition, TranscriptEvidence,
)
from mission.command_arbiter import CommandArbiter
from mission.executive import AffordanceContext, MissionExecutive, MissionState
from navigation.frontier_explorer import (
    ExplorationDecision, ExplorationStatus, next_goal,
)
from navigation.waypoint_follower import (
    SMOOTH_OFFSETS, compute_command, compute_command_smooth,
)
from perception.cloud_projector import (
    apply_cloud, clearance_multi, free_clearance,
)
from perception.global_map import GlobalOccupancyMap, FREE, OCCUPIED
from realtime.exclusive_actuation_gateway import (
    ActionKind, Channel, ExclusiveActuationGateway, RobotStatusFlags,
    RunManifestControl,
)
from voice_gateway.intent_parser import ParseKind, ParserContext, parse_utterance

BIN_EXPMAP = 3            # WSバイナリ frame type(1=点群, 2=標高格子に続く)

TICK_S = 0.2              # 5Hz ランタイム(bridge watchdog 0.5s より十分速い)
REPLAN_S = 20.0           # goal の保持上限。到達/閉塞/なし の場合のみ早期再計画。
                          # 2026-07-17 19:54: 3s 毎の再計画は旋回中の新観測で
                          # ベスト frontier が毎回入れ替わり、方向転換だけで
                          # 前進できなかった(goal スラッシング)。コミット制へ。
                          # 2026-07-18: 12s では 180°旋回(≈6s)後の前進時間が
                          # 足りず、整列中に goal が切り替わった → 20s へ。
VX_FLOOR = 0.08           # 実効前進速度の下限(Go2 Move デッドバンド対策)
EXPLORE_TIMEOUT_S = 600.0 # 探索全体タイムアウト(docs/12 §6)
POSE_STALE_S = 1.0        # pose 途絶 → fail-closed
CLOUD_STALE_S = 2.0       # 点群途絶 → fail-closed
MAX_ROLL = 0.5            # stair_task と同値
MAX_PITCH = 0.7
BLOCKED_REPLAN_S = 4.0    # 前方閉塞がこの秒数続いたら goal を捨てて再計画
STALL_WINDOW_S = 60.0     # 「地図も育たず移動もしていない」がこの秒数続いたら
                          # 打ち切り(2026-07-18: 25s は既知領域の横断中に誤発火
                          # した。移動中は成長しなくて正常 — 下の STALL_MOVE_M)
STALL_MIN_GROWTH = 8      # 成長とみなす最小 cell 数
STALL_MOVE_M = 0.3        # この距離動いたら「進捗あり」として stall 計時をリセット

MAP_SIZE_M = (20.0, 20.0)
MAP_RES_M = 0.05
MAP_ORIGIN = (-10.0, -10.0)
# 狭室調整(実機 2026-07-17 19:37 run: clearance 0.2-0.4 で全方位閉塞し
# 局所ループ化した。膨張0.30m+必要クリアランス0.45mは開けた空間向けで過大):
INFLATE_CELLS = 4         # 0.05m × 4 = 0.20m(機体半幅0.155m+余裕。goal計画用)
CLEAR_INFLATE = 3         # 前方回廊の膨張 0.15m(直進監視は薄め+MIN_CLEAR で担保)
MIN_CLEAR_M = 0.30        # 前進に必要な前方クリアランス(中心基準。鼻先→障害物
                          # 実ギャップ ≈ 0.30+0.15-0.35 = 0.10m で停止、直前は
                          # vx ≤ clearance-0.30 で這い速度まで自動減速)
MIN_CLUSTER = 10
MAX_STEP_M = 2.5          # 1 goal の移動上限(docs/10 §5 の ≤3m 内)
MIN_GOAL_M = 0.4          # これより近い goal は境界の先へ押し出す(即到達ループ防止)
AVOID_TTL_S = 30.0        # 閉塞で断念した goal を再選択しない時間
AVOID_R_M = 0.35          # 断念 goal の再選択禁止半径(狭室で全frontierを覆わない値)
SCAN_WZ = 0.35            # 到達可能frontierなし時のその場スキャン旋回 [rad/s]
PROGRESS_WINDOW_S = 150.0 # この秒数で(2026-07-18: 90s→150s。スキャン旋回・
PROGRESS_MIN_M = 0.30     #   ランダムgoal・回避リストの打開手段を試し切る時間を
                          #   与える) これ以上動けなければ stalled 終了
TICK_LOG_EVERY = 10       # 2秒毎に explore_tick を deploy_log(実機診断用)

# --- 探索中の自動登坂(操作者承認 2026-07-18: 「探索中にも登れるように」) ---
STAIR_AUTO = True           # 探索中の階段判定+自動登坂を有効化
STAIR_TRIG_DIST = 1.2       # 幾何検出(step/stairs)がこの距離内 → 判定へ [m]
STAIR_MAX_AUTO_H = 0.16     # sport 登坂の上限段高。超は候補記録のみ(RLは
                            # 登坂後 Damp になり探索を自動継続できないため)
STAIR_MIN_CONF = 0.6        # VLM 判定の採用最低確信度
STAIR_STUCK_S = 6.0         # 行き詰まりがこの秒数続いたら VLM に階段を探させる
STAIR_DEDUP_R = 1.2         # 判定済み地点の再判定禁止半径 [m]
STAIR_CLIMB_TIMEOUT_S = 300.0  # 登坂ハンドオフの上限(stair_task 自身は240s)
STAIR_MAX_CLIMBS = 3        # 1探索あたりの自動登坂回数上限(フロア数の上限)
LEVEL_UP_MIN_RISE = 0.25    # これ以上の上昇のみ「フロア変更」として地図切替。
                            # 敷居等の1段(<0.25m)は同一フロアとして継続

_CONFIRM_RE = re.compile(
    r"^(はい|うん|ええ|おっけー?|オッケー?|ok|okay|確認|実行|やって|"
    r"お願い(します)?|ゴー|go)$", re.IGNORECASE)
_CANCEL_RE = re.compile(
    r"^(いいえ|いや(だ)?|やめて|やめる|キャンセル|中止|取り消し|取消|だめ|ノー|no)$",
    re.IGNORECASE)

_INTENT_LABEL = {
    Intent.EXPLORE_AND_MAP: "自律探索してマップを作成",
    Intent.NAVIGATE_TO_WAYPOINT: "ウェイポイントへ移動",
}
_AREA_LABEL = {"current_room": "この部屋", "all_reachable": "到達可能な全域"}


def _norm(text: str) -> str:
    return re.sub(r"[\s。、．，!！?？]", "", (text or "")).strip().lower()


class ExploreTask:
    """自然言語/音声 → 確認 → 自律探索/waypoint移動 の実行系。

    状態: idle | proposal | running | done | stalled | stopped | aborted |
          refused | error
    """

    def __init__(self, bridge, artifacts_dir: str = None, stair_task=None):
        self.bridge = bridge
        # 探索中の自動登坂(操作者承認 2026-07-18)。登坂の実行は stair_task
        # (整列→接近→幾何確認→登坂)が担い、本タスクは判定と引き渡しのみ。
        # FSM は登坂中も EXPLORING のまま(登坂は探索のサブ行動)。
        self._stair = stair_task
        self._stair_mine = False    # 自分が開始した登坂のみ abort してよい
        self._judge = None          # StairJudge(遅延生成)
        self.level = 0              # 現在のフロア(登坂成功で+1)
        self.lease = str(uuid.uuid4())      # 単一操作者セッションの lease
        self.session = str(uuid.uuid4())
        self.status = "idle"
        self.detail = ""
        self.say = ""                       # UI/音声へ返す最新メッセージ
        self.gmap = None                    # 探索間で継続(同一 odom run 内)
        self.trace = []                     # ロボット軌跡(間引き)
        self.goal = None                    # 現在の goal (gx, gy) or None
        self._pending = None                # (GoalProposal, expire_monotonic)
        self._run = False
        self._th = None
        self._lock = threading.Lock()
        self._t0 = 0.0
        self._seq = 0
        self._now_last = 0
        self._artifacts = artifacts_dir or os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "artifacts", "maps")
        # server が設定する相互排他フック(他の自律系が動作中なら理由を返す)
        self.external_busy = None
        # UIログへ流す意思決定イベント [(seq, text)](sender が snapshot 経由で配る)
        self._ev_seq = 0
        self._events = []
        # 常時マッピング(SLAM表示): 地図への書き込みは mapper スレッドのみ
        # (single writer)。ミッションループは読むだけ。
        self._z_floor = None
        self._map_lock = threading.Lock()
        threading.Thread(target=self._mapper_loop, daemon=True).start()

    def _event(self, text: str):
        """探索の意思決定を UI ログ/deploy_log の両方へ記録する。"""
        with self._lock:
            self._ev_seq += 1
            self._events.append((self._ev_seq, text))
            del self._events[:-30]
        deploy_log("explore_event", text=text)

    def _abort_my_stair(self, why: str):
        """自分が開始した登坂だけを中断する(操作者の手動登坂は触らない)。"""
        if self._stair_mine and self._stair is not None:
            try:
                self._stair.abort(why)
            except Exception:
                pass
        self._stair_mine = False

    def _run_judge(self, out):
        """VLM 判定(別スレッド。数十秒〜2分ブロックするため)。"""
        try:
            if self._judge is None:
                from cockpit.stair_scout import StairJudge
                self._judge = StairJudge()
            out[0] = self._judge.judge(self.bridge)
        except Exception as e:
            out[0] = {"climbable": False, "stairs": False,
                      "confidence": 0.0, "reason": "判定例外: %r" % (e,)}

    def _rotate_map_level(self):
        """登坂成功後: 旧フロアの地図を保存し、新フロア用に地図を初期化する。
        (上下フロアは2D座標が重なるため、同一グリッドには統合できない)"""
        with self._map_lock:
            if self.gmap is not None:
                self._save_map(completed=False)
            self.level += 1
            self.gmap = None      # mapper が新フロアで再作成(home も再登録)
            self._z_floor = None
        self.trace = []
        deploy_log("explore_level_up", level=self.level)

    # ---------- 常時マッピング(受動。コマンドは一切送らない) ----------

    def _mapper_loop(self):
        """pose+点群が生きている間、ミッションと無関係に地図を更新し続ける。
        テレオペ中もリアルタイムに占有格子が育つ(ユーザー要望 2026-07-15)。"""
        last_diag = 0.0
        last_stats = 0.0
        stats_acc = {}
        zf_fail_since = None
        while True:
            try:
                pose = self.bridge.pose
                pts = self.bridge.cloud_pts
                cloud_ts = self.bridge.cloud_rx_ts
                fresh = (pose is not None and pts is not None and len(pts)
                         and cloud_ts
                         and time.monotonic() - cloud_ts < CLOUD_STALE_S)
                if fresh:
                    zf = self._estimate_z_floor(pts, pose)
                    if zf is not None:
                        self._z_floor = zf
                        zf_fail_since = None
                    elif self._z_floor is None:
                        # 推定不能が続く場合(実機 19:44 run: 80s以上初期化不能)、
                        # ARM中=立位に限り標準立位高でフォールバック。以後
                        # 実推定が成功すれば毎tick上書きされる。
                        now_m = time.monotonic()
                        if zf_fail_since is None:
                            zf_fail_since = now_m
                        elif (now_m - zf_fail_since > 8.0
                              and getattr(self.bridge, "armed", False)):
                            self._z_floor = float(pose[2]) - 0.31
                            deploy_log("explore_zfloor_fallback",
                                       z=round(self._z_floor, 3))
                    if self._z_floor is not None:
                        with self._map_lock:
                            if self.gmap is None:
                                mid = ("cockpit_explore" if self.level == 0
                                       else "cockpit_explore_L%d" % self.level)
                                self.gmap = GlobalOccupancyMap(
                                    size_m=MAP_SIZE_M, resolution_m=MAP_RES_M,
                                    origin_xy=MAP_ORIGIN,
                                    map_id=mid)
                                # 起動地点を home として登録(「ホームに戻って」用)
                                self.gmap.set_waypoint(
                                    "home", (pose[0], pose[1], pose[3]))
                                deploy_log("explore_map_init",
                                           home=[round(pose[0], 2),
                                                 round(pose[1], 2)])
                            st = apply_cloud(self.gmap, (pose[0], pose[1]),
                                             pts, self._z_floor,
                                             self._now_ns())
                            for k, v in st.items():
                                stats_acc[k] = stats_acc.get(k, 0) + v
                            # 分類統計を30s毎に記録(z_floor異常の検出用:
                            # drop/obstacle が floor より多ければ誤分類を疑う)
                            if time.monotonic() - last_stats > 30.0:
                                last_stats = time.monotonic()
                                deploy_log("explore_map_stats",
                                           z_floor=round(self._z_floor, 3),
                                           **stats_acc)
                                stats_acc = {}
                # 初期化できない間は 10s 毎に理由を記録(実機診断 2026-07-17)
                if self.gmap is None and time.monotonic() - last_diag > 10.0:
                    last_diag = time.monotonic()
                    deploy_log("explore_mapper_wait",
                               pose=(pose is not None),
                               pts=(0 if pts is None else int(len(pts))),
                               cloud_fresh=bool(fresh),
                               z_floor=(None if self._z_floor is None
                                        else round(self._z_floor, 2)))
            except Exception as e:
                # 地図が壊れても操縦系には影響させない(受動系)
                deploy_log("explore_mapper_error", err=repr(e))
                time.sleep(2.0)
            time.sleep(0.4)  # ≒2.5Hz

    # ---------- 時刻(全契約が厳格 monotonic を要求。複数スレッドから呼ばれる) ----------

    def _now_ns(self) -> int:
        with self._lock:
            n = time.monotonic_ns()
            if n <= self._now_last:
                n = self._now_last + 1
            self._now_last = n
            return n

    # ---------- 入口: テキスト/音声の共通ルーティング ----------

    def route_text(self, text: str, modality: str = "text",
                   evidence: dict = None, utterance_id: str = None) -> dict:
        """認識テキスト/入力テキストを契約パーサへ通す。

        戻り値 dict: {kind, say, handled} —
          kind ∈ stop_now | proposal | confirmed | cancelled | clarification |
                 rejected | non_command | busy | error
        non_command のときのみ呼び出し側が旧ルールベース解釈へフォールバック可。
        「止まって」系(STOP_NOW)は ARM 状態に関係なく即時停止する(常時受理)。
        """
        norm = _norm(text)
        with self._lock:
            pending = self._pending
        # 確認待ち中の「はい/いいえ」(パーサは NON_COMMAND を返すため前処理)
        if pending is not None:
            if _CONFIRM_RE.match(norm):
                return self.confirm()
            if _CANCEL_RE.match(norm):
                return self.cancel_proposal("操作者が取消")

        ev = None
        if evidence:
            ev = TranscriptEvidence(
                asr_quality_score=float(evidence.get("quality", 0.0)),
                no_speech_probability=float(evidence.get("no_speech", 1.0)))
        mod = {"voice": Modality.VOICE, "ui": Modality.UI}.get(
            modality, Modality.TEXT)
        pctx = ParserContext(
            modality=mod,
            operator_lease_id=self.lease,
            session_id=self.session,
            utterance_id=(utterance_id or (str(uuid.uuid4())
                                           if mod is Modality.VOICE else None)),
            asr_model_id=("faster-whisper" if mod is Modality.VOICE
                          else "text-parser-v1"),
            now_monotonic_ns=self._now_ns(),
            created_at_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            id_factory=lambda: str(uuid.uuid4()),
            evidence=ev)
        try:
            res = parse_utterance(text, pctx)
        except Exception as e:
            return {"kind": "error", "handled": True,
                    "say": "解釈に失敗しました: %r" % (e,)}

        if res.kind is ParseKind.STOP_NOW:
            self._operator_stop(res.stop_spec)
            return {"kind": "stop_now", "handled": True,
                    "say": "停止します(STOP_NOW)"}
        if res.kind is ParseKind.CANCEL:
            if pending is not None:
                return self.cancel_proposal("操作者が取消")
            return {"kind": "cancelled", "handled": True, "say": "取り消しました"}
        if res.kind is ParseKind.PROPOSAL:
            return self._stage_proposal(res.proposal)
        if res.kind is ParseKind.CLARIFICATION:
            return {"kind": "clarification", "handled": True,
                    "say": "指示が曖昧です: %s" % res.reason}
        if res.kind is ParseKind.REJECTED:
            return {"kind": "rejected", "handled": True,
                    "say": "実行できない指示です: %s" % res.reason}
        return {"kind": "non_command", "handled": False, "say": ""}

    def _stage_proposal(self, prop: GoalProposal) -> dict:
        if self._run:
            return {"kind": "busy", "handled": True,
                    "say": "実行中です。先に停止してください"}
        if prop.intent not in _INTENT_LABEL:
            # 階段系(NAVIGATE_TO_STAIR_APPROACH/ASCEND/DESCEND)は本統合の対象外
            return {"kind": "rejected", "handled": True,
                    "say": "%s は探索統合の対象外です(段差操作パネルを使用)"
                           % prop.intent.name}
        if prop.intent is Intent.NAVIGATE_TO_WAYPOINT:
            g = self.gmap   # 一度だけ読む(フロア切替でNoneになり得る)
            wp = (g.waypoints if g is not None else {}).get(prop.target.ref)
            if wp is None:
                return {"kind": "rejected", "handled": True,
                        "say": "ウェイポイント %r が未登録です(先に探索を完了してください)"
                               % prop.target.ref}
        area = _AREA_LABEL.get(prop.target.ref, prop.target.ref)
        stair_note = ("階段候補があればAI判定の上で自動登坂します(段高0.16mまで)。"
                      if (STAIR_AUTO and self._stair is not None
                          and prop.intent is Intent.EXPLORE_AND_MAP
                          and not self.bridge.mock) else "")
        say = ("%s します(対象: %s / 最大速度 %.2f m/s)。%s"
               "よろしければ「確認」を押すか「はい」と答えてください"
               % (_INTENT_LABEL[prop.intent], area,
                  prop.constraints.max_speed_mps, stair_note))
        with self._lock:
            self._pending = (prop, time.monotonic() + prop.confirm_ttl_s)
            self.status = "proposal"
            self.detail = say
            self.say = say
        deploy_log("explore_proposal", intent=prop.intent.name,
                   target=prop.target.ref, proposal_id=prop.proposal_id)
        return {"kind": "proposal", "handled": True, "say": say,
                "intent": prop.intent.name, "target": prop.target.ref}

    # ---------- 確認 / 取消 ----------

    def confirm(self) -> dict:
        """確認 → CONFIRMED GoalSpec → FSM 受理 → 実行スレッド開始。"""
        with self._lock:
            pending = self._pending
            self._pending = None
        if pending is None:
            return {"kind": "error", "handled": True, "say": "確認待ちの提案がありません"}
        prop, expire = pending
        if time.monotonic() > expire:
            self.status, self.detail = "idle", ""
            return {"kind": "error", "handled": True,
                    "say": "提案の確認期限(30s)が切れました。もう一度指示してください"}
        if self._run:
            return {"kind": "busy", "handled": True, "say": "実行中です"}
        if not self.bridge.armed:
            self.status, self.detail = "idle", ""
            return {"kind": "error", "handled": True,
                    "say": "ARMしてください(開始は ARM 中のみ)"}
        if self.external_busy is not None:
            busy = self.external_busy()
            if busy:
                self.status, self.detail = "idle", ""
                return {"kind": "error", "handled": True, "say": busy}
        why = self._health_check()
        if why:
            self.status, self.detail = "refused", why
            return {"kind": "error", "handled": True, "say": "開始拒否: " + why}

        required = prop.intent in (Intent.EXPLORE_AND_MAP,)
        spec = GoalSpec(
            schema_version=SCHEMA_VERSION,
            goal_id=str(uuid.uuid4()),
            source=prop.source,
            transcript=prop.transcript,
            intent=prop.intent,
            target=prop.target,
            completion=prop.completion,
            constraints=prop.constraints,
            confidence=prop.confidence,
            confirmation=(Confirmation(required=True,
                                       status=ConfirmationStatus.CONFIRMED,
                                       proposal_id=prop.proposal_id,
                                       challenge_id=prop.challenge_id)
                          if required else
                          Confirmation(required=False,
                                       status=ConfirmationStatus.NOT_REQUIRED,
                                       proposal_id=None, challenge_id=None)),
            preconditions=(Precondition.OPERATOR_LEASE_VALID,
                           Precondition.ROBOT_ARMED,
                           Precondition.SAFETY_SUPERVISOR_OK),
            created_at_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            expires_after_ms=5000,
        )
        execu = MissionExecutive(expected_operator_lease_id=self.lease)
        now = self._now_ns()
        execu.arm(self_check_ok=True, now_ns=now)
        ctx = AffordanceContext(operator_lease_valid=True, supervisor_ok=True)
        dec = execu.accept_goal(spec, ctx, self._now_ns())
        if not dec.accepted:
            self.status, self.detail = "refused", dec.reason
            deploy_log("explore_refused", reason=dec.reason)
            return {"kind": "error", "handled": True, "say": "FSMが拒否: " + dec.reason}

        self._run = True
        self.status = "running"
        self.detail = "開始"
        self.say = "開始します"
        self.goal = None
        self._t0 = time.monotonic()
        self._th = threading.Thread(target=self._loop, args=(spec, execu),
                                    daemon=True)
        self._th.start()
        deploy_log("explore_start", intent=spec.intent.name,
                   goal_id=spec.goal_id, target=spec.target.ref)
        return {"kind": "confirmed", "handled": True, "say": "開始します",
                "goal_id": spec.goal_id}

    def cancel_proposal(self, why: str = "user") -> dict:
        with self._lock:
            had = self._pending is not None
            self._pending = None
            if self.status == "proposal":
                self.status, self.detail = "idle", ""
        if had:
            deploy_log("explore_proposal_cancel", why=why)
        return {"kind": "cancelled", "handled": True,
                "say": "提案を取り消しました" if had else "確認待ちの提案はありません"}

    # ---------- 停止 / 中断 ----------

    def _operator_stop(self, stop_spec: GoalSpec):
        """STOP_NOW: ARM 不問・確認不要の即時停止(invariant 7/8)。"""
        with self._lock:
            self._pending = None
        self._stop_spec_pending = stop_spec  # ループ側で arbiter へ latch
        self._run = False
        self._abort_my_stair("STOP_NOW(操作者)")
        self.bridge.set_cmd(0, 0, 0)
        try:
            self.bridge.bot.stop_move()
        except Exception:
            pass
        if self.status in ("running", "proposal"):
            self.status = "stopped"
            self.detail = "STOP_NOW(操作者)"
        deploy_log("explore_stop_now", goal_id=stop_spec.goal_id)

    def abort(self, why: str = "user"):
        """server の abort_autonomy / DISARM / WS切断から呼ばれる。"""
        with self._lock:
            self._pending = None
        # loop が例外死した後でも、孤児化した登坂を止められるように
        # _run 判定の外で必ず実行する(レビュー指摘 2026-07-18)
        self._abort_my_stair("explore中断: " + why)
        if self._run or self.status == "proposal":
            self._run = False
            self.status = "aborted"
            self.detail = why
            self.bridge.set_cmd(0, 0, 0)
            deploy_log("explore_abort", why=why)

    # ---------- 状態 ----------

    def snapshot(self) -> dict:
        # gmap はフロア切替(_rotate_map_level)で None に戻ることがある。
        # 別スレッド(WS sender)から呼ばれるため、一度だけ読んで使う
        g = self.gmap
        counts = g.counts() if g is not None else None
        with self._lock:
            pend = self._pending
            events = list(self._events)
        return {
            "status": self.status,
            "detail": self.detail,
            "say": self.say,
            "pending": (pend[0].intent.name if pend else None),
            "goal": ([round(v, 2) for v in self.goal] if self.goal else None),
            "counts": counts,
            "waypoints": (sorted(g.waypoints) if g is not None else []),
            "elapsed": (round(time.monotonic() - self._t0, 1)
                        if self._run else 0.0),
            "trace": self.trace[-120:],
            "events": events,
            "level": self.level,
        }

    def map_frame(self):
        """占有格子のWSバイナリ frame(type=3)。0.1m へ 2×2 max-pool 縮約。
        [u8 type][f32 ox][f32 oy][f32 res][u16 W][u16 H][u8 cell...]
        cell: 0=UNKNOWN 1=FREE 2=OCCUPIED。max-pool = OCCUPIED 優先(値の大小が
        そのまま unknown<free<occupied の優先順になっている)。"""
        gm = self.gmap   # フロア切替で None に戻り得る — 一度だけ読む
        if gm is None:
            return None
        g = gm.grid
        h2, w2 = g.shape[0] // 2, g.shape[1] // 2
        g4 = g[:h2 * 2, :w2 * 2].reshape(h2, 2, w2, 2).max(axis=(1, 3))
        head = struct.pack("<BfffHH", BIN_EXPMAP,
                           float(gm.origin_xy[0]),
                           float(gm.origin_xy[1]),
                           float(gm.resolution_m * 2), w2, h2)
        return head + g4.astype(np.uint8).tobytes()

    # ---------- 内部: 健全性 ----------

    def _health_check(self) -> str:
        """開始前/毎tickの fail-closed 検査。異常なら理由を返す。"""
        st = self.bridge.bot.state()
        if not self.bridge.mock and st.get("low_age", 1e9) > 0.5:
            return "lowstate途絶"
        rpy = st.get("rpy")
        if rpy:
            if abs(rpy[0]) > MAX_ROLL:
                return "roll過大(%.2f)" % rpy[0]
            if abs(rpy[1]) > MAX_PITCH:
                return "pitch過大(%.2f)" % rpy[1]
        if self.bridge.pose is None:
            return "pose未取得"
        pose_ts = getattr(self.bridge, "pose_ts", None)
        if pose_ts is not None and time.monotonic() - pose_ts > POSE_STALE_S:
            return "pose途絶(>%.1fs)" % POSE_STALE_S
        cloud_ts = self.bridge.cloud_rx_ts
        if not cloud_ts or time.monotonic() - cloud_ts > CLOUD_STALE_S:
            return "点群途絶(>%.1fs)" % CLOUD_STALE_S
        return ""

    def _estimate_z_floor(self, pts, pose):
        """robot 近傍(<1.8m)の低い点の中央値で床 z を推定(fail-closed)。

        2026-07-17: 半径1.2m/30点では起動直後の点群蓄積次第で80s以上
        推定不能になった(実機ログ)。1.8m/15点へ緩和(中央値なので頑健)。"""
        if pts is None or len(pts) == 0:
            return None
        p = np.asarray(pts)
        d = np.hypot(p[:, 0] - pose[0], p[:, 1] - pose[1])
        near = p[(d < 1.8) & (p[:, 2] < pose[2] - 0.05)]
        if len(near) < 15:
            return None
        return float(np.median(near[:, 2]))

    # ---------- 実行ループ(5Hz) ----------

    def _loop(self, spec: GoalSpec, execu: MissionExecutive):
        try:
            self._loop_inner(spec, execu)
        except Exception as e:
            self.status = "error"
            self.detail = "%r" % (e,)
            deploy_log("explore_error", err=repr(e))
        finally:
            self._run = False
            self.goal = None
            # 例外死でも自分が開始した登坂を必ず止める(孤児化防止 —
            # stair keeper が 10Hz で駆動し続けるため。レビュー指摘 2026-07-18)
            self._abort_my_stair("explore loop終了")
            self.bridge.set_cmd(0, 0, 0)
            try:
                self.bridge.bot.stop_move()   # Controlled Stop(立位維持=ACTIVE_HOLD)
            except Exception:
                pass

    def _loop_inner(self, spec: GoalSpec, execu: MissionExecutive):
        # --- arbiter + gateway(COMMON_NAV authority、4段 handshake) ---
        arb = CommandArbiter()
        manifest = RunManifestControl(
            run_id="cockpit_explore",
            selected_backend=LocomotionBackend.SPORT_STAIR_API,
            policy_hash="not_applicable",
            operator_lease_id=self.lease)
        gw = ExclusiveActuationGateway(manifest, arb)
        gw.request_channel(Channel.COMMON_NAV, self._now_ns())
        gw.ack_inactive(Channel.COMMON_NAV, self._now_ns())
        gw.assign_generation(Channel.COMMON_NAV, self._now_ns())
        gw.enable(Channel.COMMON_NAV, self._now_ns())

        # --- 地図は mapper スレッドが作る(single writer)。初期化を待つ ---
        # 起動直後は点群蓄積に時間がかかることがある(実機 19:44 run)。
        # 待機中は移動コマンドを出さないので安全。STOP/DISARM は即応する。
        t_wait = time.monotonic()
        while self.gmap is None and self._run:
            if not self.bridge.armed:
                return self._finish(execu, "aborted", "DISARMされた", abandon=True)
            waited = time.monotonic() - t_wait
            if waited > 30.0:
                return self._finish(execu, "error",
                                    "地図未初期化(30s待っても点群から床を推定"
                                    "できない — LiDAR/odom を確認)",
                                    abandon=True)
            self.detail = "地図初期化待ち(LiDAR点群を蓄積中 %.0fs/30s)" % waited
            time.sleep(0.1)
        if not self._run:
            return self._finish(execu, "stopped", "初期化待ち中に停止", abandon=True)
        gmap = self.gmap

        waypoint_goal = None
        if spec.intent is Intent.NAVIGATE_TO_WAYPOINT:
            waypoint_goal = gmap.waypoints[spec.target.ref]

        self._stop_spec_pending = None
        tick = 0
        goal = None
        goal_ts = 0.0
        blocked_since = None
        last_growth_ts = time.monotonic()
        last_cells = 0
        avoid = []               # [(x, y, expire_monotonic)] 閉塞で断念した goal
        last_dec = None          # 直近の next_goal 判定(テレメトリ用)
        next_replan_ts = 0.0     # goalなし時の再計画スロットル(1Hz)
        prog_xy = None           # 前進進捗の基準点(局所解の検出)
        prog_ts = time.monotonic()
        grow_xy = None           # stall 計時用の移動基準点(横断中の誤発火防止)
        # --- 自動登坂の状態(4c) ---
        stair_phase = None       # None | "judging" | "climbing"
        judge_th = None
        judge_t0 = 0.0
        judge_scan = False
        judge_result = [None]
        judged_sites = []        # 判定済み地点(不採用含む — 再判定防止)
        climbs = 0
        climb_t0 = 0.0
        stuck_since = None
        # 確率的探索(操作者要望 2026-07-18): goal 選択とスキャン方向に
        # seed 付き乱数を注入。seed はログに残す(再現可能)
        rng_seed = int.from_bytes(os.urandom(4), "little")
        rng = random.Random(rng_seed)
        scan_dir = None          # 行き詰まりスキャンの旋回方向(episode毎に抽選)
        self._event("探索開始(goal_id=%s, seed=%d)"
                    % (spec.goal_id[:8], rng_seed))

        while self._run:
            t_tick = time.monotonic()
            tick += 1

            # --- 1) ガード(fail-closed) ---
            if not self.bridge.armed:
                return self._finish(execu, "aborted", "DISARMされた", abandon=True)
            if t_tick - self._t0 > EXPLORE_TIMEOUT_S:
                return self._finish(execu, "aborted", "全体タイムアウト(%.0fs)"
                                    % EXPLORE_TIMEOUT_S, abandon=True)
            why = self._health_check()
            if why:
                return self._finish(execu, "aborted", why, abandon=True,
                                    critical=("roll" in why or "pitch" in why
                                              or "lowstate" in why))

            # --- 2) 操作者 STOP_NOW の latch(route_text から予約) ---
            stop_spec = self._stop_spec_pending
            if stop_spec is not None:
                self._stop_spec_pending = None
                now = self._now_ns()
                self._seq += 1
                env = CommandEnvelope(
                    schema_version="1.0", source_id="operator_voice",
                    goal_id=stop_spec.goal_id,
                    actuation_request_id=str(uuid.uuid4()),
                    sender_timestamp=now, sequence=self._seq,
                    expires_after_ms=stop_spec.expires_after_ms,
                    requested_mode=RequestedMode.STOP_NOW,
                    vx=0.0, vy=0.0, wz=0.0, phase="EXPLORE",
                    policy_hash="not_applicable")
                arb.submit(env, ServerAttribution(
                    "operator_voice", ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                    now), now)
                execu.notify_stop_now(self._now_ns())
                return self._finish(execu, "stopped", "STOP_NOW(操作者)",
                                    abandon=False)

            # --- 3) 地図統合は mapper スレッドが常時実施(single writer)。
            #        ここでは pose と軌跡のみ扱う ---
            pose = tuple(self.bridge.pose)   # (x, y, z, yaw)
            if tick % 5 == 0:
                self.trace.append([round(pose[0], 2), round(pose[1], 2)])
                if len(self.trace) > 2000:
                    del self.trace[:1000]

            # --- 4) 打ち切り検出(地図が成長しない探索を誤完了にしない) ---
            counts = gmap.counts()
            known = counts["free"] + counts["occupied"]
            if known >= last_cells + STALL_MIN_GROWTH:
                last_cells = known
                last_growth_ts = t_tick
            # 既知領域の横断中は地図が育たないのが正常 — 移動も進捗と
            # みなして stall 計時をリセットする(2026-07-18)
            if (grow_xy is None
                    or math.hypot(pose[0] - grow_xy[0],
                                  pose[1] - grow_xy[1]) > STALL_MOVE_M):
                grow_xy = (pose[0], pose[1])
                last_growth_ts = t_tick
            if (spec.intent is Intent.EXPLORE_AND_MAP
                    and stair_phase is None
                    and t_tick - last_growth_ts > STALL_WINDOW_S):
                return self._finish(execu, "stalled",
                                    "地図が%.0fs成長しない(打ち切り。完了ではない)"
                                    % STALL_WINDOW_S, abandon=True)

            # --- 4b) 前進進捗の監視(旋回だけで場所が変わらない局所解を申告)
            #        判定/登坂フェーズ中は停止が正常なので監視しない ---
            if spec.intent is Intent.EXPLORE_AND_MAP and stair_phase is None:
                if (prog_xy is None
                        or math.hypot(pose[0] - prog_xy[0],
                                      pose[1] - prog_xy[1]) > PROGRESS_MIN_M):
                    prog_xy = (pose[0], pose[1])
                    prog_ts = t_tick
                elif t_tick - prog_ts > PROGRESS_WINDOW_S:
                    return self._finish(
                        execu, "stalled",
                        "%.0fs間 %.1fm以上前進できない(局所解 — 打ち切り)"
                        % (PROGRESS_WINDOW_S, PROGRESS_MIN_M), abandon=True)

            # --- 4c) 階段判定・自動登坂(操作者承認 2026-07-18) ---
            #   trigger(幾何 or 行き詰まり) → 停止して VLM 判定(judging)
            #   → 登れるなら stair_task へ引き渡し(climbing。こちらは送信停止)
            #   → 成功: 地図を新フロアへ切替えて探索続行 / 失敗: 記録して続行
            if (STAIR_AUTO and self._stair is not None
                    and spec.intent is Intent.EXPLORE_AND_MAP
                    and not self.bridge.mock):
                if stair_phase == "climbing":
                    snap = self._stair.snapshot()
                    self.detail = "🪜 登坂中: %s" % snap.get("detail", "")
                    if t_tick - climb_t0 > STAIR_CLIMB_TIMEOUT_S:
                        self._stair.abort("explore側タイムアウト(%.0fs)"
                                          % STAIR_CLIMB_TIMEOUT_S)
                    if snap.get("state") in ("done", "refused",
                                             "aborted", "error"):
                        self._stair_mine = False
                        st_state = snap.get("state")
                        climbed = int(snap.get("climbed") or 0)
                        rise = float(snap.get("rise") or 0.0)
                        judged_sites.append((pose[0], pose[1]))
                        if (st_state in ("aborted", "error") and rise > 0.04):
                            # 登坂の途中で中断 — ロボットが階段の途中に
                            # いる可能性がある。その場で探索を再開すると
                            # 段上でのその場旋回等の転落リスクがあるため、
                            # 自動再開せず操作者へ引き渡す(レビュー指摘)
                            return self._finish(
                                execu, "aborted",
                                "登坂中断(%s)。階段上で停止している可能性 — "
                                "手動操縦で安全な場所へ移動してください"
                                % snap.get("detail", st_state), abandon=True)
                        if (st_state == "done" and climbed >= 1
                                and rise >= LEVEL_UP_MIN_RISE):
                            climbs += 1
                            self._event("🪜 登坂成功: %d段 / 上昇%.2fm — "
                                        "フロア%d の地図を開始"
                                        % (climbed, rise, self.level + 1))
                            self._rotate_map_level()
                            # 新フロアの地図初期化を待つ(頂上で立位・静止)
                            t_w = time.monotonic()
                            while (self.gmap is None and self._run
                                    and self.bridge.armed
                                    and time.monotonic() - t_w < 30.0):
                                self.detail = "新フロアの地図を初期化中…"
                                time.sleep(0.2)
                            if not self._run:
                                break   # STOP/abort — while後段の停止処理へ
                            if not self.bridge.armed:
                                return self._finish(
                                    execu, "aborted",
                                    "DISARMされた(新フロア初期化中)",
                                    abandon=True)
                            if self.gmap is None:
                                return self._finish(
                                    execu, "error",
                                    "新フロアの地図を初期化できない",
                                    abandon=True)
                            gmap = self.gmap
                            goal, goal_ts = None, 0.0
                            avoid, judged_sites = [], []
                            blocked_since, stuck_since = None, None
                            last_cells, counts = 0, gmap.counts()
                            last_growth_ts = time.monotonic()
                            prog_xy, prog_ts = None, time.monotonic()
                            grow_xy = None
                            next_replan_ts = 0.0
                        else:
                            if st_state == "done" and rise > 0.04:
                                climbs += 1
                                self._event("小さな段(上昇%.2fm)を乗り越え — "
                                            "同一フロアとして探索続行" % rise)
                            else:
                                self._event("登坂せず(%s: %s) — 探索を続行"
                                            % (st_state,
                                               snap.get("detail", "")))
                            # 登坂に費やした時間で stall/進捗監視を誤発火
                            # させない(フェーズ中の停止は正常)
                            last_growth_ts = time.monotonic()
                            prog_xy, prog_ts = None, time.monotonic()
                            goal, goal_ts = None, 0.0
                            next_replan_ts = 0.0
                        # COMMON_NAV を取り直す(登坂前に disable 済み)
                        gw.request_channel(Channel.COMMON_NAV, self._now_ns())
                        gw.ack_inactive(Channel.COMMON_NAV, self._now_ns())
                        gw.assign_generation(Channel.COMMON_NAV,
                                             self._now_ns())
                        gw.enable(Channel.COMMON_NAV, self._now_ns())
                        stair_phase = None
                elif stair_phase == "judging":
                    self.detail = "🔎 階段判定中(VLM)… 停止して待機"
                    if judge_th is not None and not judge_th.is_alive():
                        res = judge_result[0] or {}
                        judge_th = None
                        judged_sites.append((pose[0], pose[1]))
                        if (res.get("climbable")
                                and res.get("confidence", 0.0)
                                >= STAIR_MIN_CONF):
                            # 幾何trigger: 目標は正面でVLM判定済み → confirm不要。
                            # 行き詰まりtrigger(scan=True): stair_task が旋回
                            # して見つける対象はVLMが見た画角と別方向の可能性が
                            # あるため、stair_task 自身のカメラ確認を有効にする
                            # (レビュー指摘 2026-07-18)
                            # STOP競合窓対策(レビュー指摘): _stair_mine を
                            # start前に立て、start後に _run を再確認する。
                            # STOPがどのタイミングで来ても、abort() が拾うか
                            # 直後の再確認で必ず止まる
                            self._stair_mine = True
                            err = (self._stair.start(
                                       confirm=judge_scan,
                                       multi=True, scan=judge_scan,
                                       backend="sport", dry_run=False,
                                       max_step=STAIR_MAX_AUTO_H)
                                   if self._run else "探索停止済み")
                            if err:
                                self._stair_mine = False
                                self._event("登坂を開始できず: %s" % err)
                                stair_phase = None
                            elif not self._run:
                                self._abort_my_stair("開始直後にSTOP/中断")
                                stair_phase = None
                            else:
                                stair_phase = "climbing"
                                climb_t0 = t_tick
                                self._event("🪜 自動登坂を開始: %s (conf %.2f)"
                                            % (res.get("reason", ""),
                                               res.get("confidence", 0.0)))
                                # stair_task が bridge を専有する — 明け渡し
                                gw.disable(Channel.COMMON_NAV, self._now_ns())
                        else:
                            self._event("登坂しない: %s (conf %.2f)"
                                        % (res.get("reason", "階段ではない"),
                                           res.get("confidence", 0.0)))
                            stair_phase = None
                        if stair_phase != "climbing":
                            last_growth_ts = time.monotonic()
                            prog_xy, prog_ts = None, time.monotonic()
                    elif t_tick - judge_t0 > 150.0:
                        self._event("VLM判定タイムアウト — 探索を続行")
                        judged_sites.append((pose[0], pose[1]))
                        judge_th = None
                        stair_phase = None
                        last_growth_ts = time.monotonic()
                        prog_xy, prog_ts = None, time.monotonic()
                else:
                    # trigger 判定
                    det = self.bridge.stair or {}
                    cloud_ok = (self.bridge.cloud_rx_ts and
                                time.monotonic() - self.bridge.cloud_rx_ts
                                < 1.0)
                    near_judged = any(
                        math.hypot(pose[0] - jx, pose[1] - jy) < STAIR_DEDUP_R
                        for jx, jy in judged_sites)
                    geo_trig = (cloud_ok
                                and det.get("kind") in ("step", "stairs")
                                and det.get("distance", 9.0) <= STAIR_TRIG_DIST
                                and det.get("height", 9.0) <= STAIR_MAX_AUTO_H)
                    if last_dec in ("NO_CLEAR_GOAL", "NO_REACHABLE_FRONTIER"):
                        stuck_since = stuck_since or t_tick
                    else:
                        stuck_since = None
                    sem_trig = (stuck_since is not None
                                and t_tick - stuck_since > STAIR_STUCK_S)
                    if (climbs < STAIR_MAX_CLIMBS and not near_judged
                            and (geo_trig or sem_trig)):
                        stair_phase = "judging"
                        judge_scan = not geo_trig  # 幾何で未捕捉なら scan させる
                        judge_t0 = t_tick
                        judge_result = [None]
                        judge_th = threading.Thread(
                            target=self._run_judge, args=(judge_result,),
                            daemon=True)
                        judge_th.start()
                        goal = None
                        self._event("🔎 階段候補(%s) — VLM判定中(最大2分、"
                                    "その場停止)"
                                    % ("幾何検出: %s h=%.2fm d=%.2fm"
                                       % (det.get("kind"),
                                          det.get("height", 0.0),
                                          det.get("distance", 0.0))
                                       if geo_trig else "行き詰まり"))

            # --- 5) goal 決定 ---
            arrived_now = False
            if spec.intent is Intent.NAVIGATE_TO_WAYPOINT:
                goal = (waypoint_goal[0], waypoint_goal[1])
            elif stair_phase is not None:
                goal = None          # 判定/登坂中は移動計画を止める
            else:  # EXPLORE_AND_MAP
                need_replan = ((goal is None and t_tick >= next_replan_ts)
                               or (goal is not None
                                   and t_tick - goal_ts > REPLAN_S))
                if need_replan and self._z_floor is not None:
                    next_replan_ts = t_tick + 1.0   # goalなし時の再計画は1Hzまで
                    avoid = [a for a in avoid if a[2] > t_tick]
                    prev_goal, prev_dec = goal, last_dec
                    # goal 方向が地図上で既に塞がっている候補は即座に捨てて
                    # 次候補を試す(2026-07-18: 閉塞goalに4s費やす往復の排除)
                    tmp_avoid = [(a[0], a[1]) for a in avoid]
                    d = None
                    for _try in range(3):
                        d = next_goal(gmap, (pose[0], pose[1]),
                                      min_cluster_cells=MIN_CLUSTER,
                                      max_step_m=MAX_STEP_M,
                                      inflate_cells=INFLATE_CELLS,
                                      optimistic=True,
                                      min_goal_dist_m=MIN_GOAL_M,
                                      avoid_xy=tmp_avoid,
                                      avoid_radius_m=AVOID_R_M,
                                      rng=rng)
                        if d.status is not ExplorationStatus.GOAL:
                            break
                        yaw_g = math.atan2(d.goal.y - pose[1],
                                           d.goal.x - pose[0])
                        c_g = free_clearance(gmap, pose[0], pose[1], yaw_g,
                                             max_m=1.0,
                                             inflate_cells=CLEAR_INFLATE,
                                             optimistic=True)
                        if c_g >= MIN_CLEAR_M:
                            break
                        tmp_avoid.append((d.goal.x, d.goal.y))
                        d = None
                    if d is None:
                        # 全候補の方向が塞がっている → 今回はgoalなし(scan旋回)
                        last_dec = "NO_CLEAR_GOAL"
                        if prev_dec != "NO_CLEAR_GOAL":
                            self._event("全goal候補の方向が塞がっている — "
                                        "その場旋回で観測を継続")
                        goal = None
                        goal_ts = t_tick
                        self.goal = None
                        self.detail = "goal候補なし(方向閉塞)"
                        d = ExplorationDecision(
                            ExplorationStatus.NO_REACHABLE_FRONTIER, None,
                            "候補方向が全て閉塞")
                    last_dec = (last_dec if last_dec == "NO_CLEAR_GOAL"
                                else d.status.name)
                    goal_ts = t_tick
                    if d.status is ExplorationStatus.GOAL:
                        g = (d.goal.x, d.goal.y)
                        if (prev_goal is None
                                or math.hypot(g[0] - prev_goal[0],
                                              g[1] - prev_goal[1]) > 0.3):
                            self._event("goal (%.2f, %.2f) へ %.1fm 前進"
                                        "(frontier %d cells)"
                                        % (g[0], g[1], d.goal.distance_m,
                                           d.goal.frontier_cells))
                    elif d.status.name != prev_dec:
                        self._event("frontier判定: %s — %s"
                                    % (d.status.name, d.reason))
                    if d.status is ExplorationStatus.COMPLETE:
                        gmap.set_waypoint("explore_end",
                                          (pose[0], pose[1], pose[3]))
                        execu.notify_completion(spec.completion.predicate,
                                                self._now_ns())
                        self._save_map(completed=True)
                        return self._finish(execu, "done",
                                            "EXPLORATION_COMPLETE(frontier枯渇)",
                                            abandon=False)
                    elif d.status is ExplorationStatus.GOAL:
                        goal = (d.goal.x, d.goal.y)
                    else:
                        # NO_OBSERVATIONS / NO_REACHABLE_FRONTIER ≠ 完了
                        goal = None
                        self.detail = "frontier待ち: %s" % d.status.name

            self.goal = goal

            # --- 6) 追従 → envelope → arbiter ---
            vx = wz = 0.0
            clearance = None
            if goal is not None:
                scan_dir = None      # goal があればスキャン episode 終了
                # 探索中は未踏(UNKNOWN)域もクリアランスに数える(optimistic)。
                # 障害物/落差は mapper が OCCUPIED 化するので従来どおり止まる。
                # waypoint 復帰は観測済み FREE のみ(保守側のまま)。
                optimistic = spec.intent is Intent.EXPLORE_AND_MAP
                yaw_ref = math.atan2(goal[1] - pose[1], goal[0] - pose[0])
                if optimistic:
                    # ルンバ風スムーズ操舵(2026-07-18): goal 方向±80°の
                    # 9方向を測り、通行可能な最良方向へ「進みながら」曲がる。
                    # 壁に正対しても停止せず弧で回り込む
                    heads = [yaw_ref + o for o in SMOOTH_OFFSETS]
                    cls = clearance_multi(gmap, pose[0], pose[1], heads,
                                          max_m=2.0,
                                          inflate_cells=CLEAR_INFLATE,
                                          optimistic=True)
                    fc = compute_command_smooth(
                        pose[0], pose[1], pose[3], goal[0], goal[1], cls,
                        vx_max=spec.constraints.max_speed_mps,
                        min_clearance=MIN_CLEAR_M, vx_floor=VX_FLOOR)
                    clearance = fc.clearance
                else:
                    # waypoint 復帰は従来の保守的な直線追従(FREEのみ)
                    clearance = free_clearance(gmap, pose[0], pose[1],
                                               yaw_ref, max_m=2.0,
                                               inflate_cells=CLEAR_INFLATE,
                                               optimistic=False)
                    fc = compute_command(pose[0], pose[1], pose[3],
                                         goal[0], goal[1],
                                         front_clearance=clearance,
                                         vx_max=spec.constraints.max_speed_mps,
                                         min_clearance=MIN_CLEAR_M,
                                         vx_floor=VX_FLOOR)
                if fc.arrived:
                    arrived_now = True
                    if spec.intent is Intent.NAVIGATE_TO_WAYPOINT:
                        execu.notify_completion(spec.completion.predicate,
                                                self._now_ns())
                        return self._finish(execu, "done", "WAYPOINT_REACHED",
                                            abandon=False)
                    goal = None       # 次 tick で再計画
                    goal_ts = 0.0
                else:
                    vx, wz = fc.vx, fc.wz
                    if fc.blocked:
                        if blocked_since is None:
                            blocked_since = t_tick
                        elif t_tick - blocked_since > BLOCKED_REPLAN_S:
                            # この goal は塞がっている — しばらく再選択しない
                            self._event("goal (%.2f, %.2f) は前方閉塞"
                                        "(clearance %.2fm) — %.0fs 回避"
                                        % (goal[0], goal[1], clearance,
                                           AVOID_TTL_S))
                            avoid.append((goal[0], goal[1],
                                          t_tick + AVOID_TTL_S))
                            goal, goal_ts, blocked_since = None, 0.0, None
                    else:
                        blocked_since = None
            elif (spec.intent is Intent.EXPLORE_AND_MAP
                    and stair_phase is None
                    and last_dec in ("NO_REACHABLE_FRONTIER",
                                     "NO_CLEAR_GOAL")):
                # 行き詰まり: 立ち尽くさず、その場旋回で LiDAR 観測を稼いで
                # 打開を試みる(前進はしない。旋回のみ)。方向は episode 毎に
                # 抽選(常に左回りだと同じ側ばかり観測する)。90s 前進なしなら
                # 4b) の進捗監視が stalled で正直に終了させる。
                if scan_dir is None:
                    scan_dir = rng.choice((-1.0, 1.0))
                wz = SCAN_WZ * scan_dir

            if stair_phase == "climbing":
                # stair_task が bridge を専有中(10Hz keeper)。こちらから
                # set_cmd すると last-write-wins で交互混入するため一切送らない
                act = None
            else:
                now = self._now_ns()
                self._seq += 1
                env = CommandEnvelope(
                    schema_version="1.0", source_id="frontier_explorer",
                    goal_id=spec.goal_id,
                    actuation_request_id=str(uuid.uuid4()),
                    sender_timestamp=now, sequence=self._seq,
                    expires_after_ms=500,
                    requested_mode=RequestedMode.COMMON_NAV,
                    vx=round(float(vx), 4), vy=0.0, wz=round(float(wz), 4),
                    phase="EXPLORE", policy_hash="not_applicable")
                arb.submit(env, ServerAttribution(
                    "frontier_explorer", ArbiterPriority.NAV_LOCAL_PLANNER,
                    now), now)

                # --- 7) gateway → bridge(唯一の actuator 経路) ---
                st = self.bridge.bot.state()
                vel = st.get("vel") or (0.0, 0.0, 0.0)
                speed = math.hypot(float(vel[0]), float(vel[1]))
                flags = RobotStatusFlags(
                    settled_below_thresholds=(speed < 0.05),
                    stable_contact_verified=(st.get("low_age", 1e9) < 0.5
                                             or self.bridge.mock),
                    sport_authority_active=True)
                act = gw.tick(flags, self._now_ns())
                if act.kind is ActionKind.FORWARD and self._run:
                    e = act.envelope
                    self.bridge.set_cmd(e.vx, e.vy, e.wz)
                else:
                    self.bridge.set_cmd(0, 0, 0)

            if stair_phase is None:
                self.detail = ("goal=%s clearance可 vx=%.2f wz=%.2f known=%d"
                               % (["%.1f,%.1f" % goal if goal else "なし"][0],
                                  vx, wz, known)) if not arrived_now \
                    else "goal到達"

            # 実機診断テレメトリ(2秒毎): なぜ動かないかを事後解析できるようにする
            if tick % TICK_LOG_EVERY == 0:
                deploy_log("explore_tick",
                           goal=([round(goal[0], 2), round(goal[1], 2)]
                                 if goal else None),
                           dec=last_dec,
                           clr=(round(clearance, 2)
                                if clearance is not None else None),
                           vx=round(float(vx), 3), wz=round(float(wz), 3),
                           act=(act.kind.name if act is not None
                                else "STAIR_HANDOFF"),
                           known=int(known),
                           avoid=len(avoid), stair=stair_phase,
                           level=self.level)

            # --- 8) 5Hz 維持 ---
            dt = time.monotonic() - t_tick
            if dt < TICK_S:
                time.sleep(TICK_S - dt)

        # _run が外部(_operator_stop/abort)で落とされた場合の後処理。
        # STOP_NOW はここで arbiter へ latch し FSM へ通知する(従来は
        # _run=False で loop が即終了し、section 2 の監査経路に到達しなかった
        # — レビュー指摘 2026-07-18)
        stop_spec = self._stop_spec_pending
        if stop_spec is not None:
            self._stop_spec_pending = None
            now = self._now_ns()
            self._seq += 1
            env = CommandEnvelope(
                schema_version="1.0", source_id="operator_voice",
                goal_id=stop_spec.goal_id,
                actuation_request_id=str(uuid.uuid4()),
                sender_timestamp=now, sequence=self._seq,
                expires_after_ms=stop_spec.expires_after_ms,
                requested_mode=RequestedMode.STOP_NOW,
                vx=0.0, vy=0.0, wz=0.0, phase="EXPLORE",
                policy_hash="not_applicable")
            arb.submit(env, ServerAttribution(
                "operator_voice", ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                now), now)
            try:
                execu.notify_stop_now(self._now_ns())
            except Exception:
                pass
            return self._finish(execu, "stopped", "STOP_NOW(操作者)",
                                abandon=False)
        if self.status not in ("stopped", "aborted", "done", "stalled"):
            self.status, self.detail = "aborted", "外部中断"

    # ---------- 終了処理 ----------

    def _finish(self, execu: MissionExecutive, status: str, detail: str,
                abandon: bool, critical: bool = False):
        self._run = False
        self.goal = None
        self._abort_my_stair("explore終了: " + detail)
        self.bridge.set_cmd(0, 0, 0)
        try:
            if critical:
                execu.notify_critical_fault(detail, self._now_ns())
            elif abandon and execu.state in (MissionState.EXPLORING,
                                             MissionState.NAVIGATING):
                execu.abandon_goal(detail, self._now_ns())
        except Exception:
            pass
        if status in ("stalled", "aborted", "stopped") and self.gmap is not None:
            self._save_map(completed=False)
        self.status = status
        self.detail = detail
        self.say = detail
        deploy_log("explore_end", status=status, detail=detail,
                   fsm=execu.state.name)

    def _save_map(self, completed: bool):
        """artifacts/maps/<map_id>/ へ保存(docs/10 §4)。"""
        try:
            d = self.gmap.to_dict()
            d["provenance"] = {"run_id": "cockpit_explore",
                               "odom_source": self.bridge.pose_src,
                               "completed": bool(completed),
                               "saved_at": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                                         time.gmtime())}
            path = os.path.join(self._artifacts, self.gmap.map_id)
            os.makedirs(path, exist_ok=True)
            with open(os.path.join(path, "map.json"), "w",
                      encoding="utf-8") as f:
                json.dump(d, f, ensure_ascii=False)
            deploy_log("explore_map_saved", path=path, completed=completed,
                       counts=self.gmap.counts())
        except Exception as e:
            deploy_log("explore_map_save_error", err=repr(e))
