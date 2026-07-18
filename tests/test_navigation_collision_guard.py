"""Local collision guardian の回帰試験（実機・DDSなし）。"""
import math
import unittest

import numpy as np

from navigation.collision_guard import CollisionGuard


POSE = (1.0, -2.0, 0.31, 0.0)
NOW_S = 10.0


def _body_points_to_world(body_xyz, pose=POSE):
    """body-frame fixtureをguardian入力のodom/world frameへ変換する。"""
    points = np.asarray(body_xyz, dtype=np.float64)
    x, y, _, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    out = points.copy()
    out[:, 0] = x + c * points[:, 0] - s * points[:, 1]
    out[:, 1] = y + s * points[:, 0] + c * points[:, 1]
    return out


def _dense_floor(pose=POSE):
    """停止回廊を十分な密度で覆う、障害物ではない床return。"""
    xs = np.linspace(0.05, 0.65, 13)
    ys = np.linspace(-0.35, 0.35, 7)
    bx, by = np.meshgrid(xs, ys, indexing="ij")
    ground_z = pose[2] - 0.31
    body = np.column_stack((bx.ravel(), by.ravel(),
                            np.full(bx.size, ground_z)))
    return _body_points_to_world(body, pose)


class TestCollisionGuardLinearMotion(unittest.TestCase):
    def setUp(self):
        self.guard = CollisionGuard()

    def assess(self, points, *, pose=POSE, command=(0.2, 0.0, 0.0),
               cloud_timestamp_s=NOW_S - 0.1, scan_valid=True):
        return self.guard.assess(
            pose, points, command, now_s=NOW_S,
            cloud_timestamp_s=cloud_timestamp_s, scan_valid=scan_valid)

    def test_fresh_dense_floor_allows_forward_motion(self):
        result = self.assess(_dense_floor())

        self.assertTrue(result.safe, result.reason)
        self.assertEqual(result.reason, "corridor clear")
        self.assertGreaterEqual(
            result.evidence_points,
            self.guard.config.min_corridor_evidence_points)
        self.assertEqual(result.obstacle_points, 0)
        self.assertAlmostEqual(result.required_clearance_m, 0.51)
        self.assertAlmostEqual(result.sensor_age_s, 0.1)

    def test_obstacle_inside_required_clearance_stops(self):
        required = self.guard.required_clearance(0.2)
        ground_z = POSE[2] - self.guard.config.nominal_base_height_m
        obstacle = _body_points_to_world(
            [[required - 0.05, 0.0, ground_z + 0.20]])
        points = np.vstack((_dense_floor(), obstacle))

        result = self.assess(points)

        self.assertFalse(result.safe)
        self.assertIn("obstacle inside stopping corridor", result.reason)
        self.assertGreaterEqual(result.obstacle_points, 1)
        self.assertLessEqual(result.clearance_m, result.required_clearance_m)

    def test_stale_and_low_density_scans_fail_closed(self):
        with self.subTest("stale"):
            stale = self.assess(
                _dense_floor(),
                cloud_timestamp_s=NOW_S - self.guard.config.max_cloud_age_s - 0.01)
            self.assertFalse(stale.safe)
            self.assertIn("LiDAR stale", stale.reason)

        with self.subTest("low density"):
            too_few = _dense_floor()[:self.guard.config.min_finite_points - 1]
            sparse = self.assess(too_few)
            self.assertFalse(sparse.safe)
            self.assertIn("point density insufficient", sparse.reason)

    def test_side_obstacle_outside_swept_corridor_does_not_block(self):
        ground_z = POSE[2] - self.guard.config.nominal_base_height_m
        corridor_half_width = (self.guard.config.robot_radius_m
                               + self.guard.config.static_margin_m)
        side_obstacle = _body_points_to_world(
            [[0.30, corridor_half_width + 0.08, ground_z + 0.20]])
        points = np.vstack((_dense_floor(), side_obstacle))

        result = self.assess(points)

        self.assertTrue(result.safe, result.reason)
        self.assertEqual(result.obstacle_points, 0)

    def test_world_points_are_rotated_by_robot_yaw(self):
        # yaw=+90degではbody前方はworld +Y。world +Yの障害物が停止対象になる。
        pose = (1.0, -2.0, 0.31, math.pi / 2.0)
        ground_z = pose[2] - self.guard.config.nominal_base_height_m
        obstacle = _body_points_to_world([[0.30, 0.0, ground_z + 0.20]], pose)
        points = np.vstack((_dense_floor(pose), obstacle))

        result = self.assess(points, pose=pose)

        self.assertFalse(result.safe)
        self.assertIn("obstacle inside stopping corridor", result.reason)
        self.assertAlmostEqual(result.clearance_m, 0.30, places=6)

    def test_zero_hold_is_always_safe_but_rotation_requires_observed_footprint(self):
        hold = self.guard.assess(
            POSE, None, (0.0, 0.0, 0.0), now_s=NOW_S,
            cloud_timestamp_s=0.0, scan_valid=False)
        self.assertTrue(hold.safe)

        # 全体点数は十分でも足元の回転swept areaが未観測ならfail-closed。
        ground_z = POSE[2] - self.guard.config.nominal_base_height_m
        far = _body_points_to_world([
            [2.0 + 0.01 * i, 1.5, ground_z] for i in range(30)
        ])
        turn = self.assess(far, command=(0.0, 0.0, 0.3))
        self.assertFalse(turn.safe)
        self.assertIn("rotation footprint unobserved", turn.reason)

    def test_rotation_stops_for_obstacle_inside_swept_footprint(self):
        ground_z = POSE[2] - self.guard.config.nominal_base_height_m
        obstacle = _body_points_to_world([[0.25, 0.0, ground_z + 0.20]])
        result = self.assess(
            np.vstack((_dense_floor(), obstacle)),
            command=(0.0, 0.0, 0.3))
        self.assertFalse(result.safe)
        self.assertIn("rotation footprint", result.reason)


if __name__ == "__main__":
    unittest.main()
