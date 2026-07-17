"""navigation.waypoint_follower の unit test (E0)。"""
import math
import unittest

from navigation.waypoint_follower import (
    ALIGN_YAW, ARRIVE_RADIUS, MIN_CLEARANCE, SMOOTH_OFFSETS, VX_MAX, WZ_MAX,
    FollowCommand, compute_command, compute_command_smooth, wrap_angle,
)


class TestWrapAngle(unittest.TestCase):
    def test_identity_in_range(self):
        for a in (-3.0, -1.0, 0.0, 1.0, 3.0):
            self.assertAlmostEqual(wrap_angle(a), a)

    def test_wraps_over_pi(self):
        self.assertAlmostEqual(wrap_angle(math.pi + 0.1), -math.pi + 0.1)
        self.assertAlmostEqual(wrap_angle(-math.pi - 0.1), math.pi - 0.1)
        self.assertAlmostEqual(wrap_angle(2 * math.pi), 0.0)


class TestArrival(unittest.TestCase):
    def test_inside_radius_is_arrived_and_stopped(self):
        c = compute_command(0, 0, 0, ARRIVE_RADIUS * 0.9, 0)
        self.assertTrue(c.arrived)
        self.assertEqual((c.vx, c.wz), (0.0, 0.0))

    def test_outside_radius_not_arrived(self):
        c = compute_command(0, 0, 0, ARRIVE_RADIUS * 1.5, 0)
        self.assertFalse(c.arrived)


class TestYawAlignment(unittest.TestCase):
    def test_large_yaw_error_rotates_in_place(self):
        # goal は真後ろ → その場旋回のみ
        c = compute_command(0, 0, 0.0, -2.0, 0.0)
        self.assertEqual(c.vx, 0.0)
        self.assertNotEqual(c.wz, 0.0)
        self.assertFalse(c.blocked)

    def test_rotation_direction_matches_error_sign(self):
        left = compute_command(0, 0, 0.0, 0.0, 2.0)   # goal は左(+y)
        right = compute_command(0, 0, 0.0, 0.0, -2.0)  # goal は右(-y)
        self.assertGreater(left.wz, 0.0)
        self.assertLess(right.wz, 0.0)

    def test_small_yaw_error_moves_forward(self):
        c = compute_command(0, 0, 0.0, 2.0, 0.1)
        self.assertGreater(c.vx, 0.0)

    def test_boundary_just_below_align_threshold_moves(self):
        yaw_err = ALIGN_YAW * 0.95
        c = compute_command(0, 0, 0.0, 2 * math.cos(yaw_err), 2 * math.sin(yaw_err))
        self.assertGreater(c.vx, 0.0)


class TestClamps(unittest.TestCase):
    def test_vx_never_exceeds_max(self):
        c = compute_command(0, 0, 0, 100.0, 0)
        self.assertLessEqual(c.vx, VX_MAX)

    def test_wz_never_exceeds_max(self):
        c = compute_command(0, 0, 0, -100.0, 1e-6)
        self.assertLessEqual(abs(c.wz), WZ_MAX)

    def test_vx_nonnegative_always(self):
        # 全周の goal 方位で後退提案が出ないこと
        for i in range(16):
            a = i * math.pi / 8
            c = compute_command(0, 0, 0, 3 * math.cos(a), 3 * math.sin(a))
            self.assertGreaterEqual(c.vx, 0.0, msg=f"bearing={a}")


class TestClearanceFailClosed(unittest.TestCase):
    def test_blocked_stops_forward_but_allows_turn(self):
        c = compute_command(0, 0, 0.0, 2.0, 0.3, front_clearance=0.2)
        self.assertTrue(c.blocked)
        self.assertEqual(c.vx, 0.0)

    def test_clearance_at_threshold_still_creeps(self):
        c = compute_command(0, 0, 0.0, 2.0, 0.0,
                            front_clearance=MIN_CLEARANCE + 0.05)
        self.assertFalse(c.blocked)
        self.assertLessEqual(c.vx, 0.05 + 1e-9)  # 残クリアランスで頭打ち

    def test_infinite_clearance_full_speed_far_goal(self):
        c = compute_command(0, 0, 0.0, 10.0, 0.0)
        self.assertAlmostEqual(c.vx, VX_MAX)

    def test_zero_clearance_zero_forward(self):
        c = compute_command(0, 0, 0.0, 2.0, 0.0, front_clearance=0.0)
        self.assertEqual(c.vx, 0.0)
        self.assertTrue(c.blocked)


class TestConvergence(unittest.TestCase):
    def test_closed_loop_reaches_goal(self):
        """簡易運動学で 5Hz 追従して goal に収束すること。"""
        x, y, yaw = 0.0, 0.0, 2.5   # goal とほぼ逆向きから開始
        gx, gy = 1.5, -1.0
        dt = 0.2
        for _ in range(400):
            c = compute_command(x, y, yaw, gx, gy)
            if c.arrived:
                break
            yaw = wrap_angle(yaw + c.wz * dt)
            x += c.vx * math.cos(yaw) * dt
            y += c.vx * math.sin(yaw) * dt
        else:
            self.fail("400 step 以内に到達しない")
        self.assertLessEqual(math.hypot(gx - x, gy - y), ARRIVE_RADIUS)


class TestVxFloor(unittest.TestCase):
    """実効速度フロア(2026-07-17/18): Go2 の Move デッドバンド未満の微速は
    実効速度まで底上げして這い進む(vx=0.025 を50s送り続けて無動作 stall、
    その後「ブロック扱い」では clearance 0.30-0.38 の狭室で全goalが
    閉塞扱いになり一歩も動けなかった実機事象の再発防止)。"""

    def test_creep_band_is_boosted_to_floor(self):
        # min_clearance は満たすがデッドバンド未満 → 底上げして這う
        fc = compute_command(0, 0, 0.0, 2.0, 0.0, front_clearance=0.33,
                             min_clearance=0.30, vx_floor=0.08)
        self.assertFalse(fc.blocked)
        self.assertAlmostEqual(fc.vx, 0.08)

    def test_below_min_clearance_still_blocked(self):
        fc = compute_command(0, 0, 0.0, 2.0, 0.0, front_clearance=0.28,
                             min_clearance=0.30, vx_floor=0.08)
        self.assertTrue(fc.blocked)
        self.assertEqual(fc.vx, 0.0)

    def test_above_floor_moves(self):
        fc = compute_command(0, 0, 0.0, 2.0, 0.0, front_clearance=0.60,
                             min_clearance=0.30, vx_floor=0.08)
        self.assertFalse(fc.blocked)
        self.assertGreaterEqual(fc.vx, 0.08)

    def test_default_floor_zero_keeps_legacy(self):
        # 既定 vx_floor=0 では従来挙動(微速も許可 — 他呼び出し元の互換)
        fc = compute_command(0, 0, 0.0, 2.0, 0.0, front_clearance=0.48)
        self.assertFalse(fc.blocked)
        self.assertGreater(fc.vx, 0.0)


class TestSmoothSteer(unittest.TestCase):
    """ルンバ風スムーズ操舵(2026-07-18): 壁に正対しても停止せず弧で抜ける。"""

    def _cls(self, mapping):
        # SMOOTH_OFFSETS と同順のクリアランス列を作る
        return [mapping.get(round(o, 2), 2.0) for o in SMOOTH_OFFSETS]

    def test_open_space_goes_straight(self):
        fc = compute_command_smooth(0, 0, 0.0, 2.0, 0.0, self._cls({}),
                                    min_clearance=0.30, vx_floor=0.08)
        self.assertFalse(fc.blocked)
        self.assertGreater(fc.vx, 0.1)
        self.assertAlmostEqual(fc.heading, 0.0, places=3)  # 直進が最優先

    def test_wall_ahead_arcs_without_stopping(self):
        # 正面±0.3radは塞がり、右0.6radが開いている → 止まらず右へ弧
        cls = self._cls({0.0: 0.1, 0.3: 0.1, -0.3: 0.1})
        fc = compute_command_smooth(0, 0, 0.0, 2.0, 0.0, cls,
                                    min_clearance=0.30, vx_floor=0.08)
        self.assertFalse(fc.blocked)
        self.assertGreater(fc.vx, 0.0)     # ← 停止しない(核心)
        self.assertNotEqual(fc.wz, 0.0)    # 曲がりながら
        self.assertNotAlmostEqual(fc.heading, 0.0, places=2)

    def test_all_blocked_turns_toward_most_open(self):
        cls = [0.1] * len(SMOOTH_OFFSETS)
        cls[SMOOTH_OFFSETS.index(-1.4)] = 0.25   # 右80°が相対的に一番開く
        fc = compute_command_smooth(0, 0, 0.0, 2.0, 0.0, cls,
                                    min_clearance=0.30, vx_floor=0.08)
        self.assertTrue(fc.blocked)
        self.assertEqual(fc.vx, 0.0)
        self.assertLess(fc.wz, 0.0)        # 右(負方向)へ旋回

    def test_arrive(self):
        fc = compute_command_smooth(0, 0, 0.0, 0.05, 0.0, self._cls({}))
        self.assertTrue(fc.arrived)

    def test_prefers_straight_when_equal(self):
        # 全方向同じクリアランスなら直進(ペナルティで側方が不利)
        fc = compute_command_smooth(0, 0, 0.0, 2.0, 0.0,
                                    [1.5] * len(SMOOTH_OFFSETS),
                                    min_clearance=0.30)
        self.assertAlmostEqual(fc.heading, 0.0, places=3)


if __name__ == "__main__":
    unittest.main()
