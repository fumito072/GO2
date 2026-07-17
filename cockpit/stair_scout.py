"""stair_scout — 探索中の階段/段差の意味判定(VLA)。

幾何検出(cockpit.stair.detect_stair)が拾えないパターン(絨毯の段・低い一段・
螺旋など)を、カメラ画像+ハイトマップ画像で VLM に判定させる。
判定には時間をかけてよい(操作者了承 2026-07-18: 探索中の自動登坂のため)。

invariant(docs/CLAUDE.md):
  - 本モジュールは「判定」のみ。actuator を所有せず、速度も姿勢も送らない。
  - 「登れる」判定は登坂を直接解禁しない — 呼び出し側(explore_task)が
    stair_task(整列→接近→幾何確認→登坂)へ引き渡し、そこの安全検査を通る。
  - 「危険/不明」判定は常に尊重(fail-closed)。パース不能も登坂不可扱い。
"""
import json
import os
import re
import shutil
import subprocess
import tempfile

from common.safety import deploy_log

DEFAULT_MODEL = "claude-sonnet-5"
JUDGE_TIMEOUT_S = 120.0     # 操作者了承のもと長め(sonnet の画像2枚 Read 対応)

_PROMPT = """あなたは四足ロボット Unitree Go2 の地形判定モジュールです。
ロボットは自律探索中に前方の構造物で停止しました。これが「Go2が登れる階段/段差」か
どうかを判定してください。

前面カメラ画像: %(cam)s
LiDARハイトマップ画像: %(hmap)s
(ハイトマップは真上から見た図: 上=ロボット前方、中央下の緑矢印=ロボット、
 明るい=高い、暗い=低い、黒=未観測)
数値コンテキスト: %(stats)s

Go2 が登れる目安: 1段の高さ 0.08〜0.16m、奥行き 0.24m 以上、幅 0.5m 以上、
乾いた滑らない面。人・動物・可動物・崩れそうな物の上は不可。
ガラス/手すりのみ/エスカレーター/下り段差しか見えない場合も不可。

両方の画像を Read ツールで必ず確認し、次の JSON オブジェクト1個だけを出力すること。
コードブロック記法・説明文・前置きは禁止。
{"stairs": true|false, "climbable": true|false, "steps": 段数の推定(不明なら0),
 "step_height_m": 1段の高さ推定(不明なら0), "confidence": 0.0〜1.0,
 "reason": "40字以内"}

確信が持てない場合は climbable=false とすること(安全側)。"""


def _extract_json(text):
    """fail-closed: JSON が取れなければ「登れない」。"""
    m = re.search(r"\{.*\}", text or "", re.S)
    if not m:
        return {"stairs": False, "climbable": False, "steps": 0,
                "step_height_m": 0.0, "confidence": 0.0,
                "reason": "VLM非JSON応答"}
    try:
        d = json.loads(m.group(0))
    except Exception:
        return {"stairs": False, "climbable": False, "steps": 0,
                "step_height_m": 0.0, "confidence": 0.0,
                "reason": "JSONパース失敗"}
    try:
        out = {
            "stairs": bool(d.get("stairs", False)),
            "climbable": bool(d.get("climbable", False)),
            "steps": int(d.get("steps") or 0),
            "step_height_m": float(d.get("step_height_m") or 0.0),
            "confidence": max(0.0, min(1.0,
                                       float(d.get("confidence") or 0.0))),
            "reason": str(d.get("reason", ""))[:80],
        }
    except (TypeError, ValueError):
        return {"stairs": False, "climbable": False, "steps": 0,
                "step_height_m": 0.0, "confidence": 0.0,
                "reason": "数値フィールド不正"}
    # 矛盾は安全側に潰す: 階段でないのに登れる、は認めない
    if not out["stairs"]:
        out["climbable"] = False
    return out


class StairJudge:
    """1回きりの `claude -p` 呼び出しで登坂可否を判定する(同期・時間許容)。

    呼び出し側は必ず別スレッド/停止状態から呼ぶこと(数十秒ブロックする)。
    """

    def __init__(self, model: str = DEFAULT_MODEL):
        self.model = model
        self.available = shutil.which("claude") is not None

    def judge(self, bridge, timeout_s: float = JUDGE_TIMEOUT_S) -> dict:
        """現在のカメラ+ハイトマップで判定。戻り値は _extract_json の dict に
        judge_ok(判定プロセス自体が成功したか)を足したもの。"""
        if not self.available:
            return dict(_extract_json(None), judge_ok=False,
                        reason="claude CLIなし")
        tmpdir = tempfile.mkdtemp(prefix="go2_stairjudge_")
        try:
            from cockpit.mission import build_context
            cam, hmap, stats = build_context(bridge, tmpdir, 0)
            prompt = _PROMPT % {"cam": cam,
                                "hmap": hmap or "(ハイトマップなし)",
                                "stats": stats}
            # 一時ディレクトリを cwd に(本プロジェクトの CLAUDE.md を読ませない)
            p = subprocess.run(
                ["claude", "-p", prompt, "--model", self.model,
                 "--allowed-tools", "Read"],
                capture_output=True, text=True, timeout=timeout_s, cwd=tmpdir)
            d = _extract_json(p.stdout)
            d["judge_ok"] = (p.returncode == 0)
            deploy_log("stair_judge", **d)
            return d
        except subprocess.TimeoutExpired:
            deploy_log("stair_judge", judge_ok=False, reason="タイムアウト")
            return dict(_extract_json(None), judge_ok=False,
                        reason="VLM判定タイムアウト")
        except Exception as e:
            deploy_log("stair_judge_error", err=repr(e))
            return dict(_extract_json(None), judge_ok=False,
                        reason="判定エラー: %r" % (e,))
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)
