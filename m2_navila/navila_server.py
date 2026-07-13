#!/usr/bin/env python
"""M2: NaVILA 推論サーバ（5090側で起動。VRAM 24GB+ 必要 → RTX5090 OK）。

セットアップは INSTALL_NAVILA_JP.md 参照（AnjieCheng/NaVILA を pip -e で入れ、
チェックポイント a8cheng/navila-llama3-8b-8f を HF から取得）。

実行:
  python navila_server.py --ckpt a8cheng/navila-llama3-8b-8f [--port 8890]
  python navila_server.py --mock        # モデル無しで配管テスト（前進→停止を返す）

API: POST /act  {"instruction": str, "frames_b64": [jpeg base64 ×最大8枚(古→新)]}
  → {"raw": "...", "type": "forward|turn_left|turn_right|stop", "value": 0.75}
     value: forward=距離[m] / turn=角度[deg]
"""
import argparse
import base64
import io
import json
import re
from http.server import BaseHTTPRequestHandler, HTTPServer

MODEL = {"m": None, "proc": None, "mock_n": 0, "args": None}

PROMPT = ("You are navigating a quadruped robot. Task: {instr}\n"
          "You are given historical frames and the current observation. "
          "Choose exactly one action: move forward <x> cm / turn left <x> degrees / "
          "turn right <x> degrees / stop.")


def parse_action(text):
    t = text.lower()
    m = re.search(r"forward\D*(\d+(?:\.\d+)?)\s*(cm|m)?", t)
    if m:
        v = float(m.group(1))
        v = v / 100.0 if (m.group(2) or "cm") == "cm" else v
        return {"type": "forward", "value": min(v, 1.0)}
    m = re.search(r"turn\s+(left|right)\D*(\d+(?:\.\d+)?)", t)
    if m:
        return {"type": "turn_" + m.group(1), "value": min(float(m.group(2)), 45.0)}
    return {"type": "stop", "value": 0.0}


def load_model(ckpt):
    """NaVILA(VILA/llavaスタイル)のロード。NaVILAリポジトリのインストールが前提。"""
    from llava.model.builder import load_pretrained_model  # NaVILA repo が提供
    from llava.mm_utils import get_model_name_from_path
    name = get_model_name_from_path(ckpt)
    tokenizer, model, image_processor, _ = load_pretrained_model(ckpt, name, None)
    MODEL["m"] = (tokenizer, model, image_processor)


def infer(instruction, frames):
    if MODEL["args"].mock:
        MODEL["mock_n"] += 1
        return "stop" if MODEL["mock_n"] % 8 == 0 else "move forward 50 cm"
    from PIL import Image
    import torch
    from llava.conversation import conv_templates
    from llava.mm_utils import process_images, tokenizer_image_token
    from llava.constants import IMAGE_TOKEN_INDEX, DEFAULT_IMAGE_TOKEN

    tokenizer, model, image_processor = MODEL["m"]
    imgs = [Image.open(io.BytesIO(b)).convert("RGB") for b in frames]
    prompt_text = PROMPT.format(instr=instruction)
    conv = conv_templates["llama_3"].copy()
    conv.append_message(conv.roles[0], (DEFAULT_IMAGE_TOKEN + "\n") * len(imgs) + prompt_text)
    conv.append_message(conv.roles[1], None)
    full = conv.get_prompt()
    input_ids = tokenizer_image_token(full, tokenizer, IMAGE_TOKEN_INDEX,
                                      return_tensors="pt").unsqueeze(0).to(model.device)
    image_tensor = process_images(imgs, image_processor, model.config).to(
        model.device, dtype=model.dtype)
    with torch.inference_mode():
        out = model.generate(input_ids, images=image_tensor, max_new_tokens=48, do_sample=False)
    return tokenizer.decode(out[0, input_ids.shape[1]:], skip_special_tokens=True)


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path != "/act":
            self.send_error(404)
            return
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        frames = [base64.b64decode(b) for b in body.get("frames_b64", [])][-8:]
        try:
            raw = infer(body.get("instruction", "go forward"), frames)
            res = parse_action(raw)
            res["raw"] = raw
        except Exception as e:
            res = {"type": "stop", "value": 0.0, "raw": "ERROR %r" % (e,)}
        data = json.dumps(res).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *a):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="a8cheng/navila-llama3-8b-8f")
    ap.add_argument("--port", type=int, default=8890)
    ap.add_argument("--mock", action="store_true")
    args = ap.parse_args()
    MODEL["args"] = args
    if not args.mock:
        print("[navila] loading %s ..." % args.ckpt)
        load_model(args.ckpt)
    print("[navila] serving on :%d  (mock=%s)" % (args.port, args.mock))
    HTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
