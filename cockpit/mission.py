"""mission.py — 自然言語ミッション実行 (VLA: 言語+カメラ+LiDAR → 行動)。

`claude -p`(Claude Codeヘッドレス, stream-json持続セッション)をVLM/プランナとして使い、
毎ターン「前面カメラ画像 + ハイトマップ画像 + 数値コンテキスト」を渡して
次の1手 {move/turn/stop/done} を決めさせる。M1 agent_loop のコックピット統合版。

安全:
  - ミッション開始は ARMED 時のみ。DISARM/停止/DAMP/「止まって」で即中断。
  - 速度は vx≤0.3, |wz|≤0.6 にクランプ(通常テレオペよりさらに保守的)。
  - 1判断の実行は最大 HOLD_S 秒 — 次の判断が来なければ自動停止(VLM停止対策)。
  - ミッション全体のタイムアウト(既定180s)。全判断を deploy_log に記録。
"""
import json
import os
import re
import subprocess
import tempfile
import threading
import time
from collections import deque

import numpy as np

from common import config
from common.safety import deploy_log

DEFAULT_MODEL = "claude-sonnet-5"
VX_MAX = 0.3
WZ_MAX = 0.6
HOLD_MOVE_S = 8.0    # 1判断あたりの最大実行時間(次の判断がこれより遅れたら停止)
HOLD_TURN_S = 4.0
MISSION_TIMEOUT_S = 180.0
VLM_TIMEOUT_S = 90.0  # sonnetは画像2枚Readで30〜60秒かかることがある

SYSTEM_PROMPT = """あなたは四足ロボット Unitree Go2 の遠隔操縦支援モジュールです。

## この系の実態(すべて事実)
- 操縦者(人間)が目の前でロボットを監視し、コックピットUIで任務を入力して開始した。
- あなたの返すJSONは実際に速度コマンドとしてロボットに送られる(vx≤0.3m/s, |wz|≤0.6rad/s に
  サーバ側で強制クランプ)。低速の徒歩程度であり、操縦者はいつでも即停止できる。
- 毎ターン渡される画像は、その瞬間の実カメラ映像とLiDAR標高マップの最新フレームである
  (前ターンとは別の新しい画像。ファイル名の連番が進む)。
- 操縦者が停止ボタン/Space/「止まって」/ARM解除のいずれかを行えば即座に中断される。
  あなたの判断が届かない間も、ロボットは最大8秒で自動停止する(暴走防止)。
- 危険と判断したら "stop" を返すのがあなたの役割であり、常に尊重される。

## 各ターンの手順
1. 前面カメラ画像と、LiDARハイトマップ画像を Read ツールで必ず両方確認する。
   ハイトマップは真上から見た図: 上=ロボットの前方、中央下の緑矢印=ロボット、
   明るい色=高い(障害物/壁/段差)、暗い色=低い(床)、黒=未観測。
2. 数値コンテキスト(前方障害物距離・観測率・現在速度)も踏まえる。
3. 次の1手を JSON オブジェクト1個だけで出力する。コードブロック記法・説明文・前置きは禁止。

出力形式: {"action":"move|turn|stop|done","vx":0.0〜0.3,"wz":-0.6〜0.6,"reason":"30字以内"}

## 判断規則
- 目標が画面中央 → "move"(遠ければvx=0.3、近ければ0.1〜0.15)。wzで方向微調整可。
- 目標が左寄り → "turn" wz>0(0.3〜0.5)。右寄り → wz<0。見えない → "turn" wz=0.4 で探索。
- 目標の直前(前方障害物距離<0.5m、または目標が画面の下半分を占める)→ "stop"。
  停止後の次ターンで位置を確認し、良ければ "done"。
- 人や動物が近い・落差・画像が真っ暗・状況が不明瞭 → 必ず "stop"。迷ったら "stop"。
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
        d["vx"] = max(0.0, min(VX_MAX, float(d.get("vx") or 0.0)))
        d["wz"] = max(-WZ_MAX, min(WZ_MAX, float(d.get("wz") or 0.0)))
    except (TypeError, ValueError):
        d["vx"], d["wz"] = 0.0, 0.0
    return d


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
        self._run_flag = False
        self._th = None
        self.available = self._check_cli()

    @staticmethod
    def _check_cli():
        from shutil import which
        return which("claude") is not None

    def snapshot(self):
        return {"status": self.status, "instruction": self.instruction,
                "step": self.step, "last": self.last, "detail": self.detail,
                "elapsed": round(time.monotonic() - self.t0, 1) if self._run_flag else 0}

    # ---------- 開始/中断 ----------
    def start(self, instruction: str):
        if not self.available:
            return "claude CLIが見つかりません"
        if self._run_flag:
            return "ミッション実行中です(先に中断してください)"
        if not self.bridge.armed:
            return "ARMしてください(DISARM中はミッション開始不可)"
        instruction = (instruction or "").strip()
        if not instruction:
            return "指示が空です"
        self.instruction = instruction
        self.status = "running"
        self.detail = "VLM起動中…"
        self.step = 0
        self.last = {}
        self.t0 = time.monotonic()
        self._run_flag = True
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()
        threading.Thread(target=self._keeper, daemon=True).start()
        deploy_log("mission_start", instruction=instruction, model=self.model)
        return None

    def abort(self, why="user"):
        if not self._run_flag:
            return
        self._run_flag = False
        self.status = "aborted"
        self.detail = "中断: " + why
        self._cur = (0.0, 0.0, 0.0)
        self.bridge.set_cmd(0, 0, 0)
        deploy_log("mission_abort", why=why)

    # ---------- 実行ループ ----------
    def _keeper(self):
        """判断間も速度を維持(HOLD上限まで)。cmd watchdog(0.5s)より速く打ち直す。"""
        while self._run_flag:
            if time.monotonic() < self._hold_until and any(self._cur):
                self.bridge.set_cmd(*self._cur)
            time.sleep(0.2)

    def _run(self):
        vlm = None
        tmpdir = tempfile.mkdtemp(prefix="go2_mission_")
        try:
            vlm = ClaudeCliVLM(self.model, cwd=tmpdir)
            last_note = "(開始直後)"
            while self._run_flag:
                if not self.bridge.armed:
                    self.abort("DISARMされた")
                    return
                if time.monotonic() - self.t0 > MISSION_TIMEOUT_S:
                    self.abort("タイムアウト(%ds)" % MISSION_TIMEOUT_S)
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
                if not self._run_flag:
                    return
                self.last = {"action": d["action"], "reason": d.get("reason", ""),
                             "vx": d.get("vx", 0), "wz": d.get("wz", 0),
                             "latency": round(lat, 1)}
                self.history.append(dict(self.last, step=self.step))
                deploy_log("mission_step", step=self.step, **self.last)

                a = d["action"]
                if a == "move":
                    self._cur = (d["vx"], 0.0, d["wz"] * 0.5)
                    self._hold_until = time.monotonic() + HOLD_MOVE_S
                    last_note = "move vx=%.2f wz=%.2f (%s)" % (d["vx"], d["wz"], d.get("reason", ""))
                elif a == "turn":
                    self._cur = (0.1, 0.0, d["wz"] if d["wz"] else 0.4)
                    self._hold_until = time.monotonic() + HOLD_TURN_S
                    last_note = "turn wz=%.2f (%s)" % (d["wz"], d.get("reason", ""))
                elif a == "stop":
                    self._cur = (0.0, 0.0, 0.0)
                    self.bridge.set_cmd(0, 0, 0)
                    last_note = "stop (%s)" % d.get("reason", "")
                elif a == "done":
                    self._cur = (0.0, 0.0, 0.0)
                    self.bridge.set_cmd(0, 0, 0)
                    self._run_flag = False
                    self.status = "done"
                    self.detail = "完了: " + d.get("reason", "")
                    deploy_log("mission_done", steps=self.step)
                    return
                self.detail = "ステップ%d: %s実行中 (%s)" % (self.step, a, d.get("reason", ""))
        except Exception as e:
            self._run_flag = False
            self.status = "error"
            self.detail = "エラー: %s" % (e,)
            self._cur = (0.0, 0.0, 0.0)
            self.bridge.set_cmd(0, 0, 0)
            deploy_log("mission_error", error=repr(e))
        finally:
            if vlm:
                vlm.close()
