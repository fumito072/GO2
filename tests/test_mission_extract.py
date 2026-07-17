"""mission._extract_json のクランプ検証(VLM出力→速度契約)。

後退対応(2026-07-17): 「一歩後ろに下がって」が vx>=0 クランプで
実行不能だった(VLMは move vx=0 を出すしかなく無動作→異常停止)。
"""
import unittest

from cockpit.mission import VX_MAX, VX_MIN, _extract_json


class TestExtractJson(unittest.TestCase):
    def test_backward_allowed_and_clamped(self):
        d = _extract_json('{"action":"move","vx":-0.5,"wz":0}')
        self.assertEqual(d["action"], "move")
        self.assertAlmostEqual(d["vx"], VX_MIN)   # -0.15 へクランプ
        d = _extract_json('{"action":"move","vx":-0.1,"wz":0}')
        self.assertAlmostEqual(d["vx"], -0.1)     # 範囲内はそのまま

    def test_forward_clamp_unchanged(self):
        d = _extract_json('{"action":"move","vx":0.9,"wz":0}')
        self.assertAlmostEqual(d["vx"], VX_MAX)

    def test_non_json_fails_to_stop(self):
        d = _extract_json("動けません")
        self.assertEqual(d["action"], "stop")

    def test_unknown_action_becomes_stop(self):
        d = _extract_json('{"action":"jump","vx":0.3}')
        self.assertEqual(d["action"], "stop")


if __name__ == "__main__":
    unittest.main()
