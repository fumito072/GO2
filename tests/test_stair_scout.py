"""stair_scout の判定パーサ(fail-closed)の unit test。VLM 呼び出しはしない。"""
import unittest

from cockpit.stair_scout import _extract_json


class TestExtract(unittest.TestCase):
    def test_valid_climbable(self):
        d = _extract_json('{"stairs": true, "climbable": true, "steps": 4, '
                          '"step_height_m": 0.12, "confidence": 0.9, '
                          '"reason": "標準的な屋内階段"}')
        self.assertTrue(d["climbable"])
        self.assertEqual(d["steps"], 4)

    def test_non_json_fails_closed(self):
        d = _extract_json("これは登れると思います")
        self.assertFalse(d["climbable"])
        self.assertFalse(d["stairs"])

    def test_empty_fails_closed(self):
        self.assertFalse(_extract_json("")["climbable"])
        self.assertFalse(_extract_json(None)["climbable"])

    def test_not_stairs_cannot_be_climbable(self):
        # 矛盾出力(階段でないのに登れる)は安全側に潰す
        d = _extract_json('{"stairs": false, "climbable": true, '
                          '"confidence": 0.8, "reason": "x"}')
        self.assertFalse(d["climbable"])

    def test_confidence_clamped(self):
        d = _extract_json('{"stairs": true, "climbable": true, '
                          '"confidence": 1.7, "reason": "x"}')
        self.assertEqual(d["confidence"], 1.0)

    def test_bad_numbers_fail_closed(self):
        d = _extract_json('{"stairs": true, "climbable": true, '
                          '"steps": "many", "confidence": 0.9}')
        # int("many") は例外 → パーサは例外を投げず fail-closed すべき
        self.assertFalse(d["climbable"])


if __name__ == "__main__":
    unittest.main()
