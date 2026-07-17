"""E2E collision fixture自体の幾何回帰試験。"""
import unittest

from demo.synthetic_world import SyntheticWorld


class TestSegmentCollision(unittest.TestCase):
    def test_collinear_but_disjoint_segments_do_not_intersect(self):
        self.assertFalse(SyntheticWorld._segments_intersect(
            (0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)))

    def test_crossing_and_touching_segments_intersect(self):
        self.assertTrue(SyntheticWorld._segments_intersect(
            (0.0, 0.0), (2.0, 0.0), (1.0, -1.0), (1.0, 1.0)))
        self.assertTrue(SyntheticWorld._segments_intersect(
            (0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (2.0, 1.0)))

    def test_robot_radius_collision_uses_clearance(self):
        world = SyntheticWorld([(0.0, -1.0, 0.0, 1.0)])
        self.assertTrue(world.motion_collides((-1.0, 0.0), (-0.20, 0.0), 0.20))
        self.assertFalse(world.motion_collides((-1.0, 0.0), (-0.21, 0.0), 0.20))


if __name__ == "__main__":
    unittest.main()
