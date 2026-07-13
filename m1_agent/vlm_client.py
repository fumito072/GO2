"""vlm_client.py — 前面カメラ画像+指示 → 次の行動(JSON) を返すVLM呼び出し。

バックエンド:
  - anthropic (既定): 環境変数 ANTHROPIC_API_KEY。モデルは VLM_MODEL (既定 claude-sonnet-5)
  - openai互換: OPENAI_BASE_URL/OPENAI_API_KEY を設定（ローカルQwen2.5-VL(vLLM)等）
  - mock: --mock 時。前進→(5回目で)done を返すだけのテスト用

返り値スキーマ（VLMにもこの形で強制する）:
  {"action": "move"|"turn"|"stop"|"climb"|"done",
   "vx": m/s (moveのみ), "wz": rad/s (turnのみ), "reason": "短い説明"}
"""
import base64
import json
import os
import re

SYSTEM_PROMPT = """あなたは四足ロボット(Unitree Go2)の操縦AIです。前面カメラ画像と任務を受け取り、
次の1手をJSONだけで返します。説明文は書かず、JSON以外を出力しないこと。

スキーマ:
{"action":"move|turn|stop|climb|done", "vx":数値, "wz":数値, "reason":"20字以内"}

規則:
- 任務対象(例:階段)が画面中央に見える→ "move" (vx 0.3〜0.5)で接近。
- 対象が左寄り→ "turn" wz>0 (0.3〜0.6)。右寄り→ wz<0。見えない→ "turn" wz=0.5 で探す。
- 対象の直前(画面下半分を占める)まで来たら:
  - 任務が「行くだけ」なら "stop" → 次に "done"。
  - 任務に「登る」が含まれるなら "climb"（登坂は下位層が実行する。あなたは開始判断のみ）。
- 登坂中に画像が平坦な床ばかりになり傾きが無くなったら "done"。
- 危険(人が近い/落差/画像が真っ暗)なら必ず "stop"。判断に迷ったら "stop"。
- vxは最大0.5、wzは最大0.6。"""


def _extract_json(text):
    m = re.search(r"\{.*\}", text, re.S)
    if not m:
        return {"action": "stop", "reason": "parse fail"}
    try:
        d = json.loads(m.group(0))
    except Exception:
        return {"action": "stop", "reason": "parse fail"}
    d.setdefault("vx", 0.0)
    d.setdefault("wz", 0.0)
    d["vx"] = max(-0.3, min(0.5, float(d.get("vx") or 0.0)))
    d["wz"] = max(-0.6, min(0.6, float(d.get("wz") or 0.0)))
    if d.get("action") not in ("move", "turn", "stop", "climb", "done"):
        d["action"] = "stop"
    return d


class VLMClient:
    def __init__(self, backend="auto", mock=False):
        self.mock = mock
        self._n = 0
        if mock:
            self.backend = "mock"
            return
        if backend == "auto":
            backend = "anthropic" if os.environ.get("ANTHROPIC_API_KEY") else "openai"
        self.backend = backend
        self.model = os.environ.get("VLM_MODEL",
                                    "claude-sonnet-5" if backend == "anthropic" else "Qwen/Qwen2.5-VL-7B-Instruct")
        if backend == "anthropic":
            import anthropic
            self.client = anthropic.Anthropic()
        else:
            import openai
            self.client = openai.OpenAI(
                base_url=os.environ.get("OPENAI_BASE_URL", "http://127.0.0.1:8000/v1"),
                api_key=os.environ.get("OPENAI_API_KEY", "dummy"))

    def decide(self, frame_bgr, instruction, status_text=""):
        """frame(BGR np array) + 指示 → action dict"""
        if self.backend == "mock":
            self._n += 1
            if self._n % 6 == 0:
                return {"action": "done", "reason": "mock"}
            return {"action": "move", "vx": 0.4, "wz": 0.0, "reason": "mock forward"}

        import cv2
        ok, jpg = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        b64 = base64.b64encode(jpg.tobytes()).decode()
        user_text = "任務: %s\n状態: %s\n次の1手をJSONで。" % (instruction, status_text or "-")

        if self.backend == "anthropic":
            msg = self.client.messages.create(
                model=self.model, max_tokens=200, system=SYSTEM_PROMPT,
                messages=[{"role": "user", "content": [
                    {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg", "data": b64}},
                    {"type": "text", "text": user_text}]}])
            text = "".join(b.text for b in msg.content if b.type == "text")
        else:
            r = self.client.chat.completions.create(
                model=self.model, max_tokens=200,
                messages=[{"role": "system", "content": SYSTEM_PROMPT},
                          {"role": "user", "content": [
                              {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + b64}},
                              {"type": "text", "text": user_text}]}])
            text = r.choices[0].message.content
        return _extract_json(text)
