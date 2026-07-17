"""mission.py — 自然言語ミッション実行 (VLA: 言語+カメラ+LiDAR → 行動)。

`claude -p`(Claude Codeヘッドレス, stream-json持続セッション)をVLM/プランナとして使い、
毎ターン「前面カメラ画像 + ハイトマップ画像 + 数値コンテキスト」を渡して
次の1手 {move/turn/stop/done} を決めさせる。M1 agent_loop のコックピット統合版。

安全:
  - ミッション開始は ARMED 時のみ。DISARM/停止/DAMP/「止まって」で即中断。
  - 速度は vx≤0.3, |wz|≤0.6 にクランプ(通常テレオペよりさらに保守的)。
  - VLMの1判断は最大0.8秒だけ実行し、全commandをLiDAR guardianが10Hzで再検査。
  - EXPLORE_AND_MAPはVLM open-loopではなくglobal map + A* frontier探索を使う。
  - ミッション全体のタイムアウト(既定180s)。全判断を deploy_log に記録。
"""
import json
import os
import re
import shutil
import subprocess
import tempfile
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np

from common import config
from common.safety import deploy_log
from contracts.goal_spec import ConfirmationStatus, GoalSpec, Intent, Modality
from navigation.collision_guard import CollisionGuard
from navigation.exploration_controller import (
    ControlStatus, ExplorationController, ExplorationControllerConfig,
)
from perception.global_map import GlobalOccupancyMap
from voice_gateway.intent_parser import ParseKind, ParserContext, parse_utterance

DEFAULT_MODEL = "claude-sonnet-5"
VX_MAX = 0.3
VX_MIN = -0.15   # 後退(「下がって」系)。後方はカメラに映らないため前進より保守的
WZ_MAX = 0.6
# VLM判断間のopen-loop距離を制限する。旧値8秒では0.3m/s時に2.4m進み、
# 次の画像/LiDAR判断より先に衝突できた。探索は下の10Hz controllerを使う。
HOLD_MOVE_S = 0.8
HOLD_TURN_S = 0.8
MISSION_TIMEOUT_S = 180.0
VLM_TIMEOUT_S = 90.0  # sonnetは画像2枚Readで30〜60秒かかることがある
AUTONOMY_SENSOR_MAX_AGE_S = 0.60
AUTONOMY_SENSOR_ABORT_S = 2.0
AUTONOMY_LOWSTATE_MAX_AGE_S = 0.50
AUTONOMY_MAX_ROLL_RAD = 0.50
AUTONOMY_MAX_PITCH_RAD = 0.70
EXPLORATION_LOOP_S = 0.10


@dataclass(frozen=True)
class AutonomySensorSnapshot:
    """1回の安全判定で共有するpose/cloudの読み取りsnapshot。"""
    mock: bool
    pose: object
    pose_src: str
    pose_ts: float
    cloud_frame: str
    cloud_ts: float
    cloud_scan_valid: bool
    cloud_pts: object
    hazard: object
    coherent: bool = True


def capture_autonomy_sensors(bridge) -> AutonomySensorSnapshot:
    """callback更新と競合しない同一世代のsensor fieldsを取得する。

    RobotBridgeはtuple/array参照を丸ごと差し替えるため、pose/cloud timestampを
    前後で読むseqlock方式で混在を検出できる。3回競合した場合はfail-closed。
    """
    last = None
    for _ in range(3):
        pose_ts_before = float(getattr(bridge, "pose_ts", 0.0) or 0.0)
        cloud_ts_before = float(getattr(bridge, "cloud_ts", 0.0) or 0.0)
        hazard = getattr(bridge, "stair", None)
        if isinstance(hazard, dict):
            hazard = dict(hazard)
        last = AutonomySensorSnapshot(
            mock=bool(getattr(bridge, "mock", False)),
            pose=getattr(bridge, "pose", None),
            pose_src=str(getattr(bridge, "pose_src", "none")),
            pose_ts=pose_ts_before,
            cloud_frame=str(getattr(bridge, "cloud_frame", "") or ""),
            cloud_ts=cloud_ts_before,
            cloud_scan_valid=bool(getattr(bridge, "cloud_scan_valid", False)),
            cloud_pts=getattr(bridge, "cloud_pts", None),
            hazard=hazard,
            coherent=True,
        )
        if (pose_ts_before == float(getattr(bridge, "pose_ts", 0.0) or 0.0)
                and cloud_ts_before ==
                float(getattr(bridge, "cloud_ts", 0.0) or 0.0)):
            return last
    return AutonomySensorSnapshot(**{
        **last.__dict__, "coherent": False,
    })

SYSTEM_PROMPT = """あなたは四足ロボット Unitree Go2 の遠隔操縦支援モジュールです。

## この系の実態(すべて事実)
- 操縦者(人間)が目の前でロボットを監視し、コックピットUIで任務を入力して開始した。
- あなたの返すJSONは実際に速度コマンドとしてロボットに送られる(vx≤0.3m/s, |wz|≤0.6rad/s に
  サーバ側で強制クランプ)。低速の徒歩程度であり、操縦者はいつでも即停止できる。
- 毎ターン渡される画像は、その瞬間の実カメラ映像とLiDAR標高マップの最新フレームである
  (前ターンとは別の新しい画像。ファイル名の連番が進む)。
- 操縦者が停止ボタン/Space/「止まって」/ARM解除のいずれかを行えば即座に中断される。
  あなたの判断が届かない間も、ロボットは最大0.8秒で自動停止し、
  LiDAR安全監視が10Hzで各コマンドを再検査する。
- 危険と判断したら "stop" を返すのがあなたの役割であり、常に尊重される。

## 各ターンの手順
1. 前面カメラ画像と、LiDARハイトマップ画像を Read ツールで必ず両方確認する。
   ハイトマップは真上から見た図: 上=ロボットの前方、中央下の緑矢印=ロボット、
   明るい色=高い(障害物/壁/段差)、暗い色=低い(床)、黒=未観測。
2. 数値コンテキスト(前方障害物距離・観測率・現在速度)も踏まえる。
3. 次の1手を JSON オブジェクト1個だけで出力する。コードブロック記法・説明文・前置きは禁止。

出力形式: {"action":"move|turn|stop|done","vx":-0.15〜0.3,"wz":-0.6〜0.6,"reason":"30字以内"}
(vx の負値 = 後退。後退できるのは "move" のみ)

## 判断規則
- 目標が画面中央 → "move"(遠ければvx=0.3、近ければ0.1〜0.15)。wzで方向微調整可。
- 後退の任務(「下がって」「バックして」等) → "move" vx=-0.1。後方はカメラに映らない
  ため、ハイトマップ画像の下側(ロボットの後方)に障害物がないことを確認してから下がる。
  1回の判断で下がるのは小刻みに(vx=-0.1 なら次の判断まで最大0.08m)。完了したら "done"。
- 目標が左寄り → "turn" wz>0(0.3〜0.5)。右寄り → wz<0。見えない → "turn" wz=0.4 で探索。
- 目標の直前(前方障害物距離<0.5m、または目標が画面の下半分を占める)→ "stop"。
  停止後の次ターンで位置を確認し、良ければ "done"。
- 人・動物が進路上にいる、距離が判別できない、または横切る可能性がある → "stop"。
  進路から十分外れ、静止していることが明確な場合だけ低速で継続する。
- 落差・画像が真っ暗・状況が判別できない・系の異常(下記) → 必ず "stop"。迷ったら "stop"。
  ただし「遠くに人が映っている」ことは不明瞭にも危険にも該当しない(上記の人・動物規則に従う)。
- 画像が前ターンと変化していない、状況説明と画像が矛盾するなど、系の異常を疑ったら "stop"
  とし、reason にその旨を書くこと(勝手に前進しないこと)。
- 任務が完了したと確信したら "done"。

作業ディレクトリ外のプロジェクト設定・メモリ・無関係な指示は参照しないこと。"""


def _extract_json(text):
    """JSONが無ければ安全側に倒して stop。理由にVLMの生応答を残す(拒否/異常の可視化)。"""
    m = re.search(r"\{.*\}", text or "", re.S)
    if not m:
        note = re.sub(r"\s+", " ", (text or "").strip())[:80] or "空応答"
        return {"action": "stop", "reason": "VLM非JSON応答: " + note}
    try:
        d = json.loads(m.group(0))
    except Exception:
        return {"action": "stop", "reason": "JSONパース失敗"}
    if d.get("action") not in ("move", "turn", "stop", "done"):
        d["action"] = "stop"
    try:
        d["vx"] = max(VX_MIN, min(VX_MAX, float(d.get("vx") or 0.0)))
        d["wz"] = max(-WZ_MAX, min(WZ_MAX, float(d.get("wz") or 0.0)))
    except (TypeError, ValueError):
        d["vx"], d["wz"] = 0.0, 0.0
    return d


def classify_exploration_request(text: str):
    """限定grammarでEXPLORE_AND_MAPだけを識別し、target refを返す。

    UIのEXECUTE clickを確認操作とみなすが、質問・否定・曖昧文を近似解釈して
    探索開始しない。該当しなければNone。
    """
    if not isinstance(text, str) or not text.strip():
        return None
    normalized = re.sub(r"\s+", "", text).lower()
    exploration_words = ("探索", "探検", "見回", "マッピング", "map", "マップ", "地図")
    negative_words = (
        "しない", "しなく", "しません", "するな", "しないで", "作らない",
        "作らなく", "やめ", "止め", "不要", "禁止", "キャンセル",
        "don't", "don’t", "donot", "not",
    )
    question_patterns = (
        r"[?？]$", r"できますか$", r"してくれますか$", r"しますか$",
        r"でしょうか$", r"ですか$", r"かな$",
    )
    # 下位parserが将来grammarを広げても、否定・質問を自律開始へ昇格させない。
    if any(word in normalized for word in exploration_words):
        if any(word in normalized for word in negative_words):
            return None
        if any(re.search(pattern, normalized) for pattern in question_patterns):
            return None
    lease, session = str(uuid.uuid4()), str(uuid.uuid4())

    def make_id():
        return str(uuid.uuid4())

    ctx = ParserContext(
        modality=Modality.TEXT,
        operator_lease_id=lease,
        session_id=session,
        utterance_id=None,
        asr_model_id="cockpit-text-ui",
        now_monotonic_ns=time.monotonic_ns(),
        created_at_utc=datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        id_factory=make_id,
    )
    result = parse_utterance(text, ctx)
    if result.kind is ParseKind.PROPOSAL \
            and result.proposal.intent is Intent.EXPLORE_AND_MAP:
        return result.proposal.target.ref
    return None


def autonomy_sensor_error(bridge, now_s=None):
    """自律運動に必要なLiDAR/odom契約をfail-closedで検査する。"""
    now = time.monotonic() if now_s is None else float(now_s)
    if not np.isfinite(now):
        return "現在時刻が不正です"
    sensor = (bridge if isinstance(bridge, AutonomySensorSnapshot)
              else capture_autonomy_sensors(bridge))
    if not sensor.coherent:
        return "pose/LiDAR snapshotが更新中です"
    pose = sensor.pose
    try:
        pose_values = np.asarray(pose[:4], dtype=np.float64)
    except (TypeError, ValueError, IndexError):
        pose_values = np.asarray([], dtype=np.float64)
    if pose_values.shape != (4,) or not np.isfinite(pose_values).all():
        return "odom poseがありません"
    if not sensor.mock:
        if sensor.pose_src != "lidar_odom":
            return "pose sourceがlidar_odomではありません"
        frame = sensor.cloud_frame.strip("/\x00").lower()
        if frame != "odom":
            return "LiDAR frameがodomではありません(%s)" % (frame or "missing")
    pose_ts = sensor.pose_ts
    if pose_ts <= 0 or now - pose_ts < 0 or now - pose_ts > AUTONOMY_SENSOR_MAX_AGE_S:
        return "odom poseがstaleです"
    if not sensor.cloud_scan_valid:
        return "LiDAR scanがinvalidです"
    cloud_ts = sensor.cloud_ts
    if cloud_ts <= 0 or now - cloud_ts < 0 or now - cloud_ts > AUTONOMY_SENSOR_MAX_AGE_S:
        return "LiDAR scanがstaleです"
    pts = np.asarray(sensor.cloud_pts)
    if pts.ndim != 2 or pts.shape[1] < 3 or len(pts) < 20:
        return "LiDAR点密度が不足しています"
    if int(np.count_nonzero(np.isfinite(pts[:, :3]).all(axis=1))) < 20:
        return "finiteなLiDAR点が不足しています"
    return None


def autonomy_robot_state_error(bridge):
    """自律command中に必要なLowState/姿勢契約をfail-closedで検査する。

    LiDAR/odomが正常でも、LowState途絶や転倒姿勢では速度を許可しない。
    実機はlow_ageと有限なroll/pitchを必須にし、mockでも提供された姿勢値は
    同じ上限で検査する。
    """
    mock = bool(getattr(bridge, "mock", False))
    bot = getattr(bridge, "bot", None)
    if bot is None or not callable(getattr(bot, "state", None)):
        return None if mock else "robot stateがありません"
    try:
        state = bot.state()
    except Exception as exc:
        return "robot state取得失敗: %s" % (exc,)
    if not isinstance(state, dict):
        return "robot stateが不正です"

    if not mock:
        try:
            low_age = float(state.get("low_age", float("inf")))
        except (TypeError, ValueError):
            low_age = float("inf")
        if (not np.isfinite(low_age) or low_age < 0 or
                low_age > AUTONOMY_LOWSTATE_MAX_AGE_S):
            return "lowstateがstaleです"

    rpy = state.get("rpy")
    if rpy is None:
        return None if mock else "roll/pitchがありません"
    try:
        attitude = np.asarray(rpy[:2], dtype=np.float64)
    except (TypeError, ValueError, IndexError):
        attitude = np.asarray([], dtype=np.float64)
    if attitude.shape != (2,) or not np.isfinite(attitude).all():
        return "roll/pitchが不正です"
    if abs(float(attitude[0])) > AUTONOMY_MAX_ROLL_RAD:
        return "roll過大(%.2f rad)" % float(attitude[0])
    if abs(float(attitude[1])) > AUTONOMY_MAX_PITCH_RAD:
        return "pitch過大(%.2f rad)" % float(attitude[1])
    return None


class ClaudeCliVLM:
    """`claude -p --input-format stream-json` の持続プロセス。1ミッション=1セッション。

    cwd は画像を置く一時ディレクトリにする。リポジトリ直下で起動すると本プロジェクトの
    CLAUDE.md/メモリ(開発時の取り決め)を読み込んでしまい、操縦判断が阻害されるため。
    """

    def __init__(self, model=DEFAULT_MODEL, cwd=None):
        self.model = model
        self.proc = subprocess.Popen(
            ["claude", "-p",
             "--input-format", "stream-json", "--output-format", "stream-json",
             "--verbose", "--model", model,
             "--append-system-prompt", SYSTEM_PROMPT,
             "--allowed-tools", "Read"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, bufsize=1, cwd=cwd)

    def decide_raw(self, user_text: str, timeout_s=VLM_TIMEOUT_S) -> str:
        """1ターン投げて最終テキストを返す。"""
        msg = {"type": "user",
               "message": {"role": "user",
                           "content": [{"type": "text", "text": user_text}]}}
        self.proc.stdin.write(json.dumps(msg) + "\n")
        self.proc.stdin.flush()
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError("claudeプロセスが終了しました")
            try:
                d = json.loads(line)
            except Exception:
                continue
            if d.get("type") == "result":
                if d.get("is_error"):
                    raise RuntimeError("claudeエラー: %s" % str(d.get("result"))[:200])
                return d.get("result", "")
        raise RuntimeError("VLM応答タイムアウト")

    def decide(self, user_text: str, timeout_s=VLM_TIMEOUT_S) -> dict:
        return _extract_json(self.decide_raw(user_text, timeout_s))

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.terminate()
        except Exception:
            pass


# ---------- コンテキスト生成(カメラ縮小 + ハイトマップ画像 + 数値) ----------

def _body_grid_heights(bridge, xs, ys):
    """body系座標(x前方,y左)の格子点の標高をworld格子から引く。(len(ys),len(xs))"""
    x0, y0, _z, yaw = bridge.pose
    c, s = np.cos(yaw), np.sin(yaw)
    X, Y = np.meshgrid(xs, ys)  # body系
    Xw = x0 + c * X - s * Y
    Yw = y0 + s * X + c * Y
    return bridge.elev.lookup(Xw.ravel(), Yw.ravel()).reshape(X.shape)


def build_context(bridge, outdir, step):
    """(cam_path, map_path, stats_text) を生成。カメラ未受信ならRuntimeError。"""
    import cv2
    if bridge.latest_jpeg is None:
        raise RuntimeError("カメラ画像がありません")
    arr = np.frombuffer(bridge.latest_jpeg, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    h, w = img.shape[:2]
    if w > 768:
        img = cv2.resize(img, (768, int(h * 768 / w)))
    cam_path = os.path.join(outdir, "cam_%03d.jpg" % step)
    cv2.imwrite(cam_path, img, [cv2.IMWRITE_JPEG_QUALITY, 80])

    map_path = None
    stats = []
    if bridge.pose is not None:
        # body系 前方-1〜+4m × 左右±2.5m を0.05m格子で
        xs = np.arange(-1.0, 4.0, 0.05)
        ys = np.arange(2.5, -2.5, -0.05)  # 画像の左=+y(ロボットの左)
        H = _body_grid_heights(bridge, xs, ys)
        ground = bridge.pose[2] - 0.31
        Hrel = H - ground
        known = np.isfinite(Hrel)
        norm = np.clip((np.nan_to_num(Hrel, nan=0.0) + 0.2) / 1.2, 0, 1)
        gray = (norm * 255).astype(np.uint8)
        vis = cv2.applyColorMap(gray, cv2.COLORMAP_VIRIDIS)
        vis[~known] = (20, 20, 20)
        # 画像座標: 上=+x(前方)。行=ys(y), 列=xs(x) なので転置して上下反転
        vis = cv2.rotate(vis, cv2.ROTATE_90_COUNTERCLOCKWISE)
        vis = cv2.resize(vis, (300, 300), interpolation=cv2.INTER_NEAREST)
        # ロボット位置 (x=0,y=0) は下から 1/5 の中央
        ry = int(300 * (1 - (0.0 - (-1.0)) / 5.0))
        cv2.arrowedLine(vis, (150, ry), (150, ry - 22), (80, 255, 120), 3, tipLength=0.5)
        map_path = os.path.join(outdir, "map_%03d.png" % step)
        cv2.imwrite(map_path, vis)

        # 数値: 前方回廊(|y|<0.35m)の最近接障害物と最大段差
        cor = np.abs(np.arange(2.5, -2.5, -0.05)) < 0.35
        fwd = xs > 0.25
        Hc = Hrel[np.ix_(cor, fwd)]
        xc = xs[fwd]
        obst_d = None
        col_has = np.any(np.nan_to_num(Hc, nan=-9) > 0.15, axis=0)
        if col_has.any():
            obst_d = float(xc[col_has.argmax()])
        stats.append("前方障害物(高さ0.15m超): %s" %
                     ("%.2fm先" % obst_d if obst_d is not None else "3m以内になし/未観測"))
        stats.append("地図の観測率: %d%%" % int(known.mean() * 100))
    st = bridge.bot.state()
    if "vel" in st:
        stats.append("現在速度: vx=%.2f m/s" % st["vel"][0])
    return cam_path, map_path, " / ".join(stats)


class MissionAgent:
    """自然言語ミッションの実行ループ(スレッド)。"""

    def __init__(self, bridge, model=DEFAULT_MODEL):
        self.bridge = bridge
        self.model = model
        self.status = "idle"       # idle|running|done|aborted|error
        self.instruction = ""
        self.step = 0
        self.last = {}             # 直近の判断 {action, reason, latency}
        self.detail = ""
        self.t0 = 0.0
        self.history = deque(maxlen=20)
        self._cur = (0.0, 0.0, 0.0)
        self._hold_until = 0.0
        self._command_lock = threading.Lock()
        self._command_generation = 0
        self._lifecycle_lock = threading.RLock()
        self._run_flag = False
        self._run_id = 0
        self._th = None
        self.mode = "idle"        # idle|vlm|explore
        self.exploration_target = None
        self.gmap = None
        self.controller = None
        self.map_lock = threading.RLock()
        self.goal_spec = None
        self.guard = CollisionGuard()
        self.safety = {"safe": True, "reason": "hold"}
        self._last_safety_log = None
        self.available = self._check_cli()

    @staticmethod
    def _check_cli():
        from shutil import which
        return which("claude") is not None

    def snapshot(self):
        return {"status": self.status, "instruction": self.instruction,
                "step": self.step, "last": self.last, "detail": self.detail,
                "elapsed": round(time.monotonic() - self.t0, 1) if self._run_flag else 0,
                "mode": self.mode, "safety": dict(self.safety),
                "exploration": self.controller.metrics() if self.controller else None}

    # ---------- 開始/中断 ----------
    def start(self, instruction: str):
        instruction = (instruction or "").strip()
        if not instruction:
            return "指示が空です"
        target = classify_exploration_request(instruction)
        if target is not None:
            return "探索はEXPLORE MAPで提案内容を確認してから開始してください"
        with self._lifecycle_lock:
            if self._run_flag:
                return "ミッション実行中です(先に中断してください)"
            if not self.bridge.armed:
                return "ARMしてください(DISARM中はミッション開始不可)"
            if not self.available:
                return "claude CLIが見つかりません"
            sensor_error = (autonomy_robot_state_error(self.bridge) or
                            autonomy_sensor_error(self.bridge))
            if sensor_error:
                return "自律走行を開始できません: " + sensor_error
            return self._launch_locked(instruction, "vlm", None, None)

    def start_goal(self, spec: GoalSpec, executive=None):
        """確認済み探索GoalSpecを再解釈せず、安全runnerへ一度だけ渡す。"""
        if not isinstance(spec, GoalSpec):
            return "GoalSpecが必要です"
        if spec.intent is not Intent.EXPLORE_AND_MAP:
            return "このrunnerはEXPLORE_AND_MAPのみ対応します"
        if (not spec.confirmation.required or
                spec.confirmation.status is not ConfirmationStatus.CONFIRMED):
            return "探索GoalSpecは明示確認済みである必要があります"
        if executive is None or getattr(
                getattr(executive, "state", None), "name", "") != "EXPLORING":
            return "Mission FSMがEXPLORING状態ではありません"
        with self._lifecycle_lock:
            if self._run_flag:
                return "ミッション実行中です(先に中断してください)"
            if not self.bridge.armed:
                return "ARMしてください(DISARM中は探索開始不可)"
            sensor_error = (autonomy_robot_state_error(self.bridge) or
                            autonomy_sensor_error(self.bridge))
            if sensor_error:
                return "自律走行を開始できません: " + sensor_error
            instruction = spec.transcript.text or "確認済み自律探索"
            return self._launch_locked(
                instruction, "explore", spec.target.ref, spec)

    def _launch_locked(self, instruction, mode, target, spec):
        """_lifecycle_lock保持中に1世代のrunner/keeperを起動する。"""
        # 前runのhold/commandを次世代へ持ち越さない。lifecycle lock保持中に
        # zero化してから新runを公開する。
        self._stop_command()
        self.instruction = instruction
        self.mode = mode
        self.exploration_target = target
        self.goal_spec = spec
        self.gmap = None
        self.controller = None
        self.safety = {"safe": True, "reason": "preflight passed"}
        self._last_safety_log = None
        self.status = "running"
        self.detail = "探索controller起動中…" if mode == "explore" else "VLM起動中…"
        self.step = 0
        self.last = {}
        self.t0 = time.monotonic()
        self._run_flag = True
        self._run_id += 1
        run_id = self._run_id
        self._th = threading.Thread(target=self._run, args=(run_id,), daemon=True)
        self._th.start()
        threading.Thread(target=self._keeper, args=(run_id,), daemon=True).start()
        deploy_log("mission_start", instruction=instruction, model=self.model,
                   mode=mode, goal_id=(spec.goal_id if spec else None))
        return None

    def abort(self, why="user", expected_run_id=None):
        with self._lifecycle_lock:
            if not self._run_flag:
                return
            if expected_run_id is not None and expected_run_id != self._run_id:
                return
            self._run_flag = False
            self._run_id += 1  # blocked旧threadが次missionへ復活しないよう失効
            self.status = "aborted"
            self.detail = "中断: " + why
            # run失効とzero commandをatomicにする。次run開始後に旧abortが
            # commandを上書きする隙間を作らない。
            self._stop_command()
        deploy_log("mission_abort", why=why)

    # ---------- 実行ループ ----------
    def _run_is_active(self, run_id):
        return self._run_flag and (run_id is None or run_id == self._run_id)

    def _ensure_command_sync(self):
        """旧fixture/復元stateでも安全helperを利用できるよう初期化する。"""
        if not hasattr(self, "_command_lock"):
            self._command_lock = threading.Lock()
        if not hasattr(self, "_command_generation"):
            self._command_generation = 0

    def _set_held_command(self, command, hold_s):
        """keeperが検査するcommandを世代付きでatomicに公開する。"""
        self._ensure_command_sync()
        cmd = tuple(float(v) for v in command)
        if len(cmd) != 3 or not np.isfinite(np.asarray(cmd)).all():
            raise ValueError("command must be finite (vx,vy,wz)")
        with self._command_lock:
            self._cur = cmd
            self._hold_until = time.monotonic() + max(0.0, float(hold_s))
            self._command_generation += 1

    def _stop_command(self):
        """自律commandを無効化し、watchdogを待たず即座に0を送る。"""
        self._ensure_command_sync()
        # bridge送信まで同じlock内に置く。keeperの旧command送信が、このzeroを
        # 後から追い越すTOCTOUを防ぐ。
        with self._command_lock:
            self._cur = (0.0, 0.0, 0.0)
            self._hold_until = 0.0
            self._command_generation += 1
            self.bridge.set_cmd(0, 0, 0)

    def _keeper(self, run_id=None):
        """全自律commandをLiDAR guardianで検査してから再送する。"""
        self._ensure_command_sync()
        while self._run_is_active(run_id):
            now = time.monotonic()
            with self._command_lock:
                command = tuple(self._cur)
                deadline = self._hold_until
                generation = self._command_generation
            if now < deadline and any(command):
                robot_error = autonomy_robot_state_error(self.bridge)
                sensor = (capture_autonomy_sensors(self.bridge)
                          if robot_error is None else None)
                err = (robot_error if robot_error is not None
                       else autonomy_sensor_error(sensor, now))
                assessment = None
                if err is None:
                    try:
                        assessment = self.guard.assess(
                            sensor.pose, sensor.cloud_pts, command,
                            now_s=now,
                            cloud_timestamp_s=sensor.cloud_ts,
                            scan_valid=sensor.cloud_scan_valid,
                            hazard=sensor.hazard)
                        if not assessment.safe:
                            err = assessment.reason
                    except Exception as e:
                        err = "collision guardian error: %r" % (e,)
                if err:
                    # LowState/姿勢異常は一時停止ではなくrun世代ごと失効する。
                    # VLM判断待ち中に一度転倒して復帰した場合も、古い判断を
                    # 自動再開させない。run_id=Noneはkeeper単体test用。
                    if robot_error is not None and run_id is not None:
                        self.safety = {"safe": False, "reason": robot_error}
                        self._last_safety_log = robot_error
                        deploy_log("mission_guard_stop", reason=robot_error,
                                   mode=self.mode, critical=True)
                        self.abort("機体状態異常: " + robot_error,
                                   expected_run_id=run_id)
                        return
                    with self._command_lock:
                        unchanged = generation == self._command_generation
                    if unchanged:
                        self._stop_command()
                    self.safety = {"safe": False, "reason": err}
                    if err != self._last_safety_log:
                        deploy_log("mission_guard_stop", reason=err, mode=self.mode)
                        self._last_safety_log = err
                    self.detail = "安全停止: " + err
                else:
                    self.safety = {
                        "safe": True,
                        "reason": assessment.reason if assessment else "sensor ready",
                        "clearance_m": assessment.clearance_m if assessment else None,
                    }
                    # 評価したsnapshotだけを送る。producerがhelper経由で新世代を
                    # 公開した場合は、この旧評価を破棄して次tickで再検査する。
                    with self._command_lock:
                        if (generation == self._command_generation and
                                self._run_is_active(run_id) and
                                time.monotonic() < self._hold_until):
                            self.bridge.set_cmd(*command)
            elif any(command):
                # refresh停止だけではbridge watchdogの0.5秒間、旧速度が残る。
                with self._command_lock:
                    unchanged = generation == self._command_generation
                if unchanged:
                    self._stop_command()
            time.sleep(0.1)

    def _run(self, run_id=None):
        if self.mode == "explore":
            self._run_exploration(run_id)
        else:
            self._run_vlm(run_id)

    def _run_exploration(self, run_id=None):
        """LiDAR global map + frontier/A* + guardianによる決定的な全域探索。"""
        try:
            sensor0 = capture_autonomy_sensors(self.bridge)
            initial_error = (autonomy_robot_state_error(self.bridge) or
                             autonomy_sensor_error(sensor0))
            if initial_error:
                self.abort("探索開始時センサ異常: " + initial_error,
                           expected_run_id=run_id)
                return
            pose0 = sensor0.pose
            with self.map_lock:
                self.gmap = GlobalOccupancyMap(
                    size_m=(20.0, 20.0), resolution_m=0.10,
                    origin_xy=(float(pose0[0]) - 10.0,
                               float(pose0[1]) - 10.0),
                    map_id="cockpit_explore_%d" % int(time.time()),
                    frame_id="odom")
                self.gmap.set_waypoint("home", (pose0[0], pose0[1], pose0[3]))
                self.controller = ExplorationController(
                    self.gmap,
                    ExplorationControllerConfig(
                        max_speed_mps=0.20,
                        max_yaw_rate_rps=0.45,
                        inflation_radius_m=0.30,
                        max_goal_step_m=2.0,
                        frontier_standoff_m=0.25,
                        progress_timeout_s=3.0,
                        complete_confirmations=3,
                    ),
                    collision_guard=self.guard)
            last_cloud_ts = -1.0
            sensor_bad_since = None
            self.detail = "LiDAR global mapを構築中…"
            deploy_log("exploration_controller_start",
                       target=self.exploration_target, map_id=self.gmap.map_id)

            while self._run_is_active(run_id):
                now_s = time.monotonic()
                if not self.bridge.armed:
                    self.abort("DISARMされた", expected_run_id=run_id)
                    return
                if now_s - self.t0 > MISSION_TIMEOUT_S:
                    self.abort("探索タイムアウト(%ds)" % MISSION_TIMEOUT_S,
                               expected_run_id=run_id)
                    return
                robot_error = autonomy_robot_state_error(self.bridge)
                if robot_error:
                    self.abort("機体状態異常: " + robot_error,
                               expected_run_id=run_id)
                    return
                sensor = capture_autonomy_sensors(self.bridge)
                err = autonomy_sensor_error(sensor, now_s)
                if err:
                    with self._lifecycle_lock:
                        if not self._run_is_active(run_id):
                            return
                        self._stop_command()
                        self.safety = {"safe": False, "reason": err}
                        self.detail = "センサ待機/停止: " + err
                    sensor_bad_since = sensor_bad_since or now_s
                    if now_s - sensor_bad_since > AUTONOMY_SENSOR_ABORT_S:
                        self.abort("センサ異常が継続: " + err,
                                   expected_run_id=run_id)
                        return
                    time.sleep(EXPLORATION_LOOP_S)
                    continue
                sensor_bad_since = None

                pose = tuple(float(v) for v in sensor.pose[:4])
                points = np.asarray(sensor.cloud_pts, dtype=np.float32)
                cloud_ts = sensor.cloud_ts
                now_ns = time.monotonic_ns()
                with self.map_lock:
                    if cloud_ts != last_cloud_ts:
                        self.controller.integrate_point_cloud(
                            pose, points, now_ns, max_range_m=8.0)
                        last_cloud_ts = cloud_ts
                        self.step += 1

                    ctl = self.controller.step(
                        pose, now_ns, points_xyz=points,
                        cloud_timestamp_s=cloud_ts,
                        scan_valid=sensor.cloud_scan_valid,
                        hazard=sensor.hazard)
                completed_metrics = None
                with self._lifecycle_lock:
                    # map計算中にSTOP/DISARMされた旧threadは、UI状態もcommandも
                    # 一切更新せず終了する。
                    if not self._run_is_active(run_id):
                        return
                    self.last = {
                        "action": ctl.status.value.lower(),
                        "reason": ctl.reason,
                        "vx": round(ctl.vx, 3), "wz": round(ctl.wz, 3),
                        "goal": ([round(ctl.goal.x, 2), round(ctl.goal.y, 2)]
                                 if ctl.goal else None),
                        "map_revision": ctl.map_revision,
                    }

                    if ctl.status is ControlStatus.COMPLETE:
                        self._stop_command()
                        self._run_flag = False
                        self.status = "done"
                        self.detail = "探索完了: 到達可能frontierなし(安定確認済み)"
                        self.safety = {"safe": True, "reason": "active hold"}
                        completed_metrics = self.controller.metrics()
                    elif ctl.moving:
                        # lifecycle→command lock順でpublish。abortとの前後関係を固定。
                        self._set_held_command((ctl.vx, ctl.vy, ctl.wz), 0.30)
                        self.safety = {"safe": True, "reason": ctl.reason}
                        self.detail = ("探索 %s / %s / map=%s" %
                                       (ctl.status.value, ctl.reason,
                                        self.gmap.counts()))
                    else:
                        self._stop_command()
                        if ctl.status is ControlStatus.STOP_SENSOR:
                            self.safety = {"safe": False, "reason": ctl.reason}
                        self.detail = ("探索 %s / %s / map=%s" %
                                       (ctl.status.value, ctl.reason,
                                        self.gmap.counts()))
                        if ctl.status is ControlStatus.BLOCKED \
                                and self.controller.blocked_cycles >= 20:
                            self.abort(
                                "到達可能なfrontierがありません: " + ctl.reason,
                                expected_run_id=run_id)
                            return
                if completed_metrics is not None:
                    deploy_log("exploration_done", **completed_metrics)
                    return
                time.sleep(EXPLORATION_LOOP_S)
        except Exception as e:
            with self._lifecycle_lock:
                if not self._run_is_active(run_id):
                    return
                self._run_flag = False
                self.status = "error"
                self.detail = "探索エラー: %s" % (e,)
                self._stop_command()
            deploy_log("exploration_error", error=repr(e))

    def _run_vlm(self, run_id=None):
        vlm = None
        tmpdir = tempfile.mkdtemp(prefix="go2_mission_")
        try:
            vlm = ClaudeCliVLM(self.model, cwd=tmpdir)
            last_note = "(開始直後)"
            while self._run_is_active(run_id):
                if not self.bridge.armed:
                    self.abort("DISARMされた", expected_run_id=run_id)
                    return
                if time.monotonic() - self.t0 > MISSION_TIMEOUT_S:
                    self.abort("タイムアウト(%ds)" % MISSION_TIMEOUT_S,
                               expected_run_id=run_id)
                    return
                sensor_error = (autonomy_robot_state_error(self.bridge) or
                                autonomy_sensor_error(self.bridge))
                if sensor_error:
                    self.abort("センサ異常: " + sensor_error,
                               expected_run_id=run_id)
                    return
                self.step += 1
                self.detail = "ステップ%d: 状況把握中…" % self.step
                cam, hmap, stats = build_context(self.bridge, tmpdir, self.step)
                prompt = ("任務: %s\nステップ%d (経過%.0f秒)\nカメラ画像: %s\n%s\n%s\n直前の行動: %s\n"
                          "画像を確認して次の1手をJSONのみで。" %
                          (self.instruction, self.step, time.monotonic() - self.t0, cam,
                           ("ハイトマップ画像: %s" % hmap) if hmap else "(ハイトマップなし)",
                           stats, last_note))
                self.detail = "ステップ%d: VLM判断中…" % self.step
                t0 = time.time()
                d = vlm.decide(prompt)
                lat = time.time() - t0
                done = False
                with self._lifecycle_lock:
                    if not self._run_is_active(run_id):
                        return
                    self.last = {
                        "action": d["action"], "reason": d.get("reason", ""),
                        "vx": d.get("vx", 0), "wz": d.get("wz", 0),
                        "latency": round(lat, 1),
                    }
                    self.history.append(dict(self.last, step=self.step))
                    deploy_log("mission_step", step=self.step, **self.last)

                    a = d["action"]
                    if a == "move":
                        self._set_held_command(
                            (d["vx"], 0.0, d["wz"] * 0.5), HOLD_MOVE_S)
                        last_note = "move vx=%.2f wz=%.2f (%s)" % (
                            d["vx"], d["wz"], d.get("reason", ""))
                    elif a == "turn":
                        # 見失い時の探索旋回へ前進を混ぜない。
                        self._set_held_command(
                            (0.0, 0.0, d["wz"] if d["wz"] else 0.4),
                            HOLD_TURN_S)
                        last_note = "turn wz=%.2f (%s)" % (
                            d["wz"], d.get("reason", ""))
                    elif a == "stop":
                        self._stop_command()
                        last_note = "stop (%s)" % d.get("reason", "")
                    elif a == "done":
                        self._stop_command()
                        self._run_flag = False
                        self.status = "done"
                        self.detail = "完了: " + d.get("reason", "")
                        done = True
                    if not done:
                        self.detail = "ステップ%d: %s実行中 (%s)" % (
                            self.step, a, d.get("reason", ""))
                if done:
                    deploy_log("mission_done", steps=self.step)
                    return
        except Exception as e:
            with self._lifecycle_lock:
                if not self._run_is_active(run_id):
                    return
                self._run_flag = False
                self.status = "error"
                self.detail = "エラー: %s" % (e,)
                self._stop_command()
            deploy_log("mission_error", error=repr(e))
        finally:
            if vlm:
                vlm.close()
            shutil.rmtree(tmpdir, ignore_errors=True)
