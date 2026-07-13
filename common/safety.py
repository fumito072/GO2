"""safety.py — watchdog・クランプ・関節リミット（全マイルストーン共通）。"""
import json
import os
import threading
import time

# Go2 関節リミット [rad]（URDF準拠、SDK順ではなく「関節種別」で表記）
JOINT_LIMITS = {
    "hip": (-1.0472, 1.0472),
    "thigh": (-1.5708, 3.4907),
    "calf": (-2.7227, -0.83776),
}
LIMIT_MARGIN = 0.05  # [rad] リミット手前で止める余白


def joint_kind(name: str) -> str:
    if "hip" in name:
        return "hip"
    if "thigh" in name:
        return "thigh"
    return "calf"


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def clamp_joint_targets(q_target, joint_names):
    """関節目標角をリミット内にクランプ（joint_namesと同順の配列を返す）。"""
    out = []
    for q, n in zip(q_target, joint_names):
        lo, hi = JOINT_LIMITS[joint_kind(n)]
        out.append(clamp(float(q), lo + LIMIT_MARGIN, hi - LIMIT_MARGIN))
    return out


class Watchdog:
    """kick() が period 秒来なければ on_timeout() を一度呼ぶ。"""

    def __init__(self, period_s: float, on_timeout, name="wd"):
        self.period = period_s
        self.on_timeout = on_timeout
        self.name = name
        self._last = time.monotonic()
        self._fired = False
        self._stop = False
        self._th = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._th.start()
        return self

    def kick(self):
        self._last = time.monotonic()
        self._fired = False

    def stop(self):
        self._stop = True

    def _run(self):
        while not self._stop:
            time.sleep(self.period / 4.0)
            if not self._fired and (time.monotonic() - self._last) > self.period:
                self._fired = True
                try:
                    self.on_timeout()
                except Exception:
                    pass


def deploy_log(event: str, **kw):
    """deploy_log.jsonl へ1行追記（実験ledgerとは別の実機ログ）。"""
    from . import config
    rec = {"ts": time.time(), "t": time.strftime("%Y-%m-%dT%H:%M:%S"), "event": event}
    rec.update(kw)
    path = os.path.abspath(config.DEPLOY_LOG)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
