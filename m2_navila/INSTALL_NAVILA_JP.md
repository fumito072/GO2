# NaVILA セットアップ（5090マシン側）

NaVILA = 足つきロボット向け Vision-Language-Action（RSS'25, UCSD/NVIDIA）。
RGB＋言語指示 → 「move forward 75 cm / turn left 30 degrees / stop」の中間コマンドを出す。
公式実装: https://github.com/AnjieCheng/NaVILA / チェックポイント: `a8cheng/navila-llama3-8b-8f`（8B, bf16でVRAM約17GB → 5090の32GBでOK）

## 手順（Linux/WSL推奨。conda）
```bash
git clone https://github.com/AnjieCheng/NaVILA && cd NaVILA
# リポジトリREADMEの environment_setup.sh に従う（VILA系: torch, transformers, flash-attn等）
./environment_setup.sh navila   # conda env "navila" が作られる
conda activate navila
# チェックポイントはHFから自動DL（初回のみ、~16GB）
```

## サーバ起動（このバンドルの navila_server.py を NaVILA リポジトリ環境で）
```bash
conda activate navila
python <bundle>/m2_navila/navila_server.py --ckpt a8cheng/navila-llama3-8b-8f --port 8890
# 配管テストだけなら（モデル不要・どのenvでも）:
python <bundle>/m2_navila/navila_server.py --mock
```

## 動作確認
```bash
# ロボットPC側（またはmock）:
python -m m2_navila.navila_client --instruction "walk to the stairs and climb up" --mock
```

## 注意
- NaVILAのプロンプト/会話テンプレートはリポジトリ更新で変わることがある。
  `navila_server.py` の `infer()` は llama_3 テンプレート想定 — 公式の
  inference例と食い違ったらそちらに合わせて `infer()` を修正する。
- 推論レイテンシは1コマンド約1秒前後。ロボットは各コマンド実行完了まで待つ設計
  （navila_client.py）なので、遅くても安全側。
- 日本語指示も通るが、学習は英語なので英語指示のほうが成功率が高い。
  音声(日本語)→英訳→NaVILA という接続は m1_agent の VLM に翻訳させれば可能。
