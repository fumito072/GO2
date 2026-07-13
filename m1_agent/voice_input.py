"""voice_input.py — PCマイク→Whisper→テキスト（プッシュトゥトーク方式）。

Enterで録音開始→もう一度Enterで停止→文字起こしをqueueへ。
faster-whisper が無い/マイクが無い環境では自動でキーボード入力にフォールバック。
単体テスト: python -m m1_agent.voice_input
"""
import queue
import sys
import threading


class VoiceInput:
    def __init__(self, model_size="small", lang="ja"):
        self.q: "queue.Queue[str]" = queue.Queue()
        self.lang = lang
        self._ok = False
        try:
            import sounddevice  # noqa: F401
            from faster_whisper import WhisperModel
            self.model = WhisperModel(model_size, device="auto", compute_type="auto")
            self._ok = True
            print("[voice] faster-whisper(%s) 準備OK — Enterで録音開始/停止" % model_size)
        except Exception as e:
            print("[voice] 音声が使えないためキーボード入力モード (%r)" % (e,))
        threading.Thread(target=self._run, daemon=True).start()

    def _record_once(self):
        import numpy as np
        import sounddevice as sd
        sr = 16000
        chunks = []
        stop = threading.Event()

        def cb(indata, frames, t, status):
            chunks.append(indata.copy())

        input()  # 開始Enter待ち
        print("[voice] ●録音中… もう一度Enterで停止")
        with sd.InputStream(samplerate=sr, channels=1, dtype="float32", callback=cb):
            input()
            stop.set()
        if not chunks:
            return ""
        audio = np.concatenate(chunks)[:, 0]
        print("[voice] 認識中…")
        segs, _ = self.model.transcribe(audio, language=self.lang, vad_filter=True)
        return "".join(s.text for s in segs).strip()

    def _run(self):
        while True:
            try:
                if self._ok:
                    text = self._record_once()
                else:
                    text = input("指示> ").strip()
                if text:
                    print("[voice] → %s" % text)
                    self.q.put(text)
            except (EOFError, KeyboardInterrupt):
                self.q.put("__quit__")
                return
            except Exception as e:
                print("[voice] err %r" % (e,))

    def get(self, timeout=None):
        try:
            return self.q.get(timeout=timeout)
        except queue.Empty:
            return None


if __name__ == "__main__":
    v = VoiceInput()
    while True:
        t = v.get(timeout=1.0)
        if t == "__quit__":
            sys.exit(0)
        if t:
            print("GOT:", t)
