"""voice.py — cockpit音声操縦: faster-whisper文字起こし + 日本語コマンド解析。

ブラウザから届いた音声(webm/opus等, PyAVが読める形式なら何でも)をテキスト化し、
ルールベースで速度コマンド/姿勢アクションに変換する。VLMは使わない(決定的で安全)。

intentの形式(クライアント側で速度スケールを掛ける):
  {"action": "move"|"turn"|"strafe"|"stop"|"stand_up"|"stand_down"|"balance_stand"|"none",
   "vx","vy","wz": 単位速度(キー操作と同じ倍率),  "dur": 継続秒, "say": 表示用説明}
"""
import re
import threading

STOP_WORDS = ("止まって", "とまって", "止まれ", "とまれ", "ストップ", "stop", "停止", "やめて", "やめろ")


class Transcriber:
    """faster-whisperをバックグラウンドでロード。ロード完了までready=False。"""

    def __init__(self, model_size="small"):
        self.model_size = model_size
        self._model = None
        self.device = None
        self.error = None
        self._lock = threading.Lock()
        threading.Thread(target=self._load, daemon=True).start()

    def _load(self):
        # エッジ機のGPU(int8)を優先し、CUDA不可ならCPUへフォールバック(docs/12 §4)。
        from faster_whisper import WhisperModel
        for device in ("cuda", "cpu"):
            try:
                m = WhisperModel(self.model_size, device=device,
                                 compute_type="int8")
                self._model = m
                self.device = device
                print("[voice] whisper '%s' ロード完了 (device=%s)"
                      % (self.model_size, device))
                return
            except Exception as e:
                self.error = repr(e)
                print("[voice] whisper(%s)ロード失敗: %s" % (device, self.error))

    @property
    def ready(self):
        return self._model is not None

    def transcribe(self, path: str) -> str:
        return self.transcribe_ex(path)[0]

    def transcribe_ex(self, path: str):
        """文字起こし + 品質エビデンス。

        戻り値: (text, {"quality": 0..1, "no_speech": 0..1})
        quality は segment avg_logprob の指数平均、no_speech は
        no_speech_prob の平均。契約パーサの TranscriptEvidence 用
        (VOICE modality では必須, contracts/goal_spec.py)。
        """
        import math
        if self._model is None:
            raise RuntimeError(self.error or "モデルロード中です。少し待ってください")
        with self._lock:  # ctranslate2は同時実行しない
            segments, _info = self._model.transcribe(
                path, language="ja", beam_size=1, vad_filter=True,
                condition_on_previous_text=False)
            segs = list(segments)
        text = "".join(s.text for s in segs).strip()
        if segs:
            avg_lp = sum(s.avg_logprob for s in segs) / len(segs)
            quality = max(0.0, min(1.0, math.exp(avg_lp)))
            no_speech = max(0.0, min(1.0, sum(s.no_speech_prob for s in segs)
                                     / len(segs)))
        else:
            quality, no_speech = 0.0, 1.0
        return text, {"quality": round(quality, 4),
                      "no_speech": round(no_speech, 4)}


def _norm(text: str) -> str:
    t = text.strip().lower()
    return re.sub(r"[、。．，,.!！?？\s]+", "", t)


def parse_intent(text: str) -> dict:
    """認識テキスト → intent。優先度: stop > 姿勢 > 後退 > 旋回/平行 > 前進。"""
    t = _norm(text)
    if not t:
        return {"action": "none", "say": "(無音)"}
    if any(w in t for w in STOP_WORDS):
        return {"action": "stop", "say": "停止"}

    # 姿勢
    if re.search(r"立って|立ち上が|たって|起きて|スタンドアップ", t):
        return {"action": "stand_up", "say": "立ち上がる"}
    if re.search(r"伏せ|ふせ|座って|すわって|おすわり|しゃが|ダウン", t):
        return {"action": "stand_down", "say": "伏せる"}
    if "バランス" in t:
        return {"action": "balance_stand", "say": "バランス立位"}

    # 継続時間: 「N秒」/「ずっと」(上限8s) / 既定3s
    dur = 3.0
    m = re.search(r"(\d+)秒", t)
    if m:
        dur = min(8.0, max(0.5, float(m.group(1))))
    elif re.search(r"ずっと|進み続け", t):
        dur = 8.0
    # 速さ修飾
    speed = 1.0
    if re.search(r"ゆっくり|そっと|少しだけ|ちょっと", t):
        speed = 0.5
    elif re.search(r"速く|はやく|急いで|ダッシュ", t):
        speed = 1.5

    back = re.search(r"後ろ|うしろ|下がっ|さがっ|後退|バック", t)
    left = re.search(r"左|ひだり", t)
    right = re.search(r"右|みぎ", t)
    strafe = re.search(r"平行|横|ステップ|スライド|カニ", t)
    fwd = re.search(r"前|まえ|進|すす|直進|まっすぐ|歩い", t)

    if back:
        return {"action": "move", "vx": -0.6 * speed, "vy": 0, "wz": 0, "dur": dur,
                "say": "後退 %.0f秒" % dur}
    if (left or right) and strafe:
        vy = 0.5 * speed * (1 if left else -1)
        return {"action": "strafe", "vx": 0, "vy": vy, "wz": 0, "dur": dur,
                "say": "%s平行移動 %.0f秒" % ("左" if left else "右", dur)}
    if left or right:
        wz = 1.2 * speed * (1 if left else -1)
        return {"action": "turn", "vx": 0, "vy": 0, "wz": wz, "dur": dur,
                "say": "%s旋回 %.0f秒" % ("左" if left else "右", dur)}
    if fwd:
        return {"action": "move", "vx": 1.0 * speed, "vy": 0, "wz": 0, "dur": dur,
                "say": "前進 %.0f秒" % dur}
    return {"action": "none", "say": "コマンド解釈不能"}


if __name__ == "__main__":
    tests = [
        "前に進んで", "5秒前進", "ゆっくり後ろに下がって", "右に曲がって", "左旋回",
        "右に平行移動して", "止まって", "ストップ", "立って", "伏せて", "おすわり",
        "バランス立位", "ずっとまっすぐ歩いて", "速く前へ", "こんにちは",
    ]
    for s in tests:
        print("%-14s -> %s" % (s, parse_intent(s)))
