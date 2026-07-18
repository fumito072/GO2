"""ExplorationController の停止・再計画・完了安定化の回帰試験。"""
import unittest

from navigation.exploration_controller import (
    ControlStatus, ExplorationController, ExplorationControllerConfig,
)
from perception.global_map import FREE, GlobalOccupancyMap


START_NS = 1_000_000_000


def _config(**overrides):
    values = {
        "max_speed_mps": 0.20,
        "max_yaw_rate_rps": 0.50,
        "heading_before_move_rad": 0.28,
        "goal_tolerance_m": 0.18,
        "lookahead_m": 0.40,
        "inflation_radius_m": 0.05,
        "max_goal_step_m": 1.0,
        "frontier_standoff_m": 0.10,
        "free_max_age_s": 10.0,
        "progress_timeout_s": 0.50,
        "progress_epsilon_m": 0.06,
        "complete_confirmations": 3,
    }
    values.update(overrides)
    return ExplorationControllerConfig(**values)


def _corridor_map(now_ns=START_NS):
    """UNKNOWN内に東向きの一本道を作り、確実にfrontier goalを生成する。"""
    gmap = GlobalOccupancyMap(
        size_m=(4.0, 4.0), resolution_m=0.1,
        origin_xy=(-2.0, -2.0), map_id="controller_corridor")
    row = gmap.height // 2
    start_x = 5
    gmap.grid[row, start_x:31] = FREE
    gmap.age_ns[row, start_x:31] = now_ns
    gmap.revision = 1
    x, y = gmap.cell_to_world(start_x, row)
    return gmap, (x, y, 0.31, 0.0)


def _fully_observed_map(now_ns=START_NS):
    gmap = GlobalOccupancyMap(
        size_m=(4.0, 4.0), resolution_m=0.1,
        origin_xy=(-2.0, -2.0), map_id="controller_complete")
    gmap.grid[:, :] = FREE
    gmap.age_ns[:, :] = now_ns
    gmap.revision = 1
    return gmap, (0.05, 0.05, 0.31, 0.0)


class TestExplorationControllerSafetyStops(unittest.TestCase):
    def test_map_revision_invalidates_blocked_current_path(self):
        gmap, pose = _corridor_map()
        controller = ExplorationController(gmap, _config())
        first = controller.step(pose, START_NS)
        self.assertIn(first.status, (ControlStatus.MOVE, ControlStatus.TURN))
        self.assertIsNotNone(first.goal)
        self.assertGreaterEqual(len(first.goal.path), 2)

        # 現goal cellを新規hazardにし、map revisionを進める。
        blocked_xy = gmap.cell_to_world(*first.goal.goal_cell)
        old_revision = gmap.revision
        gmap.mark_hazard([blocked_xy], START_NS + 1)
        self.assertGreater(gmap.revision, old_revision)

        stopped = controller.step(pose, START_NS + 2)

        self.assertIs(stopped.status, ControlStatus.STOP_REPLAN)
        self.assertFalse(stopped.moving)
        self.assertIn("map更新でpath無効化", stopped.reason)
        self.assertIsNone(controller.current_goal)

    def test_no_pose_progress_times_out_and_clears_goal(self):
        gmap, pose = _corridor_map()
        controller = ExplorationController(
            gmap, _config(progress_timeout_s=0.50))
        first = controller.step(pose, START_NS)
        self.assertIn(first.status, (ControlStatus.MOVE, ControlStatus.TURN))
        attempted_cell = first.goal.goal_cell

        stopped = controller.step(pose, START_NS + 500_000_001)

        self.assertIs(stopped.status, ControlStatus.STOP_REPLAN)
        self.assertFalse(stopped.moving)
        self.assertIn("progress timeout", stopped.reason)
        self.assertIsNone(controller.current_goal)
        self.assertGreaterEqual(controller.safety_stops, 1)
        self.assertIn(attempted_cell, controller.explorer.recent_goal_cells)

    def test_path_age_expiry_stops_without_map_revision_change(self):
        gmap, pose = _corridor_map()
        controller = ExplorationController(
            gmap, _config(free_max_age_s=0.10, progress_timeout_s=1.0))
        first = controller.step(pose, START_NS)
        self.assertIn(first.status, (ControlStatus.MOVE, ControlStatus.TURN))
        old_revision = gmap.revision

        stopped = controller.step(pose, START_NS + 100_000_001)

        self.assertEqual(gmap.revision, old_revision)
        self.assertIs(stopped.status, ControlStatus.STOP_REPLAN)
        self.assertIn("stale", stopped.reason)
        self.assertFalse(stopped.moving)


class TestExplorationControllerCompletion(unittest.TestCase):
    def test_fully_stale_map_is_blocked_not_complete(self):
        gmap, pose = _fully_observed_map()
        controller = ExplorationController(
            gmap, _config(free_max_age_s=0.10, complete_confirmations=3))

        result = controller.step(pose, START_NS + 100_000_001)

        self.assertIs(result.status, ControlStatus.BLOCKED)
        self.assertNotIn("COMPLETE", result.reason)

    def test_complete_requires_stable_unchanged_map_confirmations(self):
        gmap, pose = _fully_observed_map()
        controller = ExplorationController(
            gmap, _config(complete_confirmations=3))

        first = controller.step(pose, START_NS)
        self.assertIs(first.status, ControlStatus.VERIFYING_COMPLETE)
        self.assertFalse(first.moving)

        # frontierなしのままmap revisionだけが変わっても、
        # 確認回数はリセットされる。
        far_corner = gmap.cell_to_world(0, 0)
        gmap.mark_hazard([far_corner], START_NS + 1)
        reset = controller.step(pose, START_NS + 2)
        self.assertIs(reset.status, ControlStatus.VERIFYING_COMPLETE)
        self.assertIn("1/3", reset.reason)

        # 同じmap/scanでcontrol tickだけ重ねても確認回数は増えない。
        same_scan = controller.step(pose, START_NS + 3)
        self.assertIn("1/3", same_scan.reason)

        controller.integrate_planar_scan(pose[:2], [], START_NS + 4)
        second = controller.step(pose, START_NS + 5)
        controller.integrate_planar_scan(pose[:2], [], START_NS + 6)
        complete = controller.step(pose, START_NS + 7)

        self.assertIs(second.status, ControlStatus.VERIFYING_COMPLETE)
        self.assertIn("2/3", second.reason)
        self.assertIs(complete.status, ControlStatus.COMPLETE)
        self.assertFalse(complete.moving)
        self.assertIn("3回確認", complete.reason)


if __name__ == "__main__":
    unittest.main()
