"""Global frontier plannerとlocal collision guardianを束ねる探索controller。

body-frameの低速commandを返す純ロジック層。I/O、SportClient、threadを持たない。
callerはfreshなodom poseと同一frameの点群を渡し、返されたSTOPを必ず尊重する。
"""
import math
from dataclasses import dataclass
from enum import Enum, unique
from typing import Mapping, Optional, Sequence

import numpy as np

from contracts.errors import ContractViolation
from navigation.collision_guard import CollisionAssessment, CollisionGuard
from navigation.frontier_explorer import (
    ExplorationGoal, ExplorationStatus, FrontierExplorer,
)
from perception.global_map import GlobalOccupancyMap


@unique
class ControlStatus(Enum):
    MOVE = "MOVE"
    TURN = "TURN"
    STOP_REPLAN = "STOP_REPLAN"
    STOP_SENSOR = "STOP_SENSOR"
    BLOCKED = "BLOCKED"
    VERIFYING_COMPLETE = "VERIFYING_COMPLETE"
    COMPLETE = "COMPLETE"


@dataclass(frozen=True)
class ExplorationControllerConfig:
    max_speed_mps: float = 0.20
    max_yaw_rate_rps: float = 0.50
    heading_before_move_rad: float = 0.28
    goal_tolerance_m: float = 0.18
    lookahead_m: float = 0.40
    inflation_radius_m: float = 0.30
    max_goal_step_m: float = 2.0
    frontier_standoff_m: float = 0.25
    free_max_age_s: float = 60.0
    progress_timeout_s: float = 3.0
    progress_epsilon_m: float = 0.06
    complete_confirmations: int = 3

    def __post_init__(self):
        for name in (
            "max_speed_mps", "max_yaw_rate_rps", "heading_before_move_rad",
            "goal_tolerance_m", "lookahead_m", "inflation_radius_m",
            "max_goal_step_m", "frontier_standoff_m", "free_max_age_s",
            "progress_timeout_s", "progress_epsilon_m",
        ):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or isinstance(value, bool) \
                    or not math.isfinite(value) or value <= 0:
                raise ContractViolation("controller.%s" % name, "正の有限値が必要")
        if not isinstance(self.complete_confirmations, int) \
                or isinstance(self.complete_confirmations, bool) \
                or self.complete_confirmations < 2:
            raise ContractViolation("controller.complete_confirmations", "2以上のintが必要")


@dataclass(frozen=True)
class ExplorationCommand:
    status: ControlStatus
    vx: float
    vy: float
    wz: float
    reason: str
    goal: Optional[ExplorationGoal]
    safety: Optional[CollisionAssessment]
    map_revision: int

    @property
    def moving(self) -> bool:
        return abs(self.vx) > 1e-9 or abs(self.vy) > 1e-9 or abs(self.wz) > 1e-9


def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ExplorationController:
    def __init__(self, gmap: GlobalOccupancyMap,
                 config: ExplorationControllerConfig = ExplorationControllerConfig(),
                 collision_guard: Optional[CollisionGuard] = None):
        if not isinstance(gmap, GlobalOccupancyMap):
            raise ContractViolation("gmap", "GlobalOccupancyMapが必要")
        if not isinstance(config, ExplorationControllerConfig):
            raise ContractViolation("config", "ExplorationControllerConfigが必要")
        self.gmap = gmap
        self.config = config
        self.guard = collision_guard
        self.explorer = FrontierExplorer(
            gmap,
            inflation_radius_m=config.inflation_radius_m,
            max_step_m=config.max_goal_step_m,
            standoff_m=config.frontier_standoff_m,
        )
        self.current_goal: Optional[ExplorationGoal] = None
        self._planned_revision = -1
        self._last_progress_ns: Optional[int] = None
        self._best_goal_distance = math.inf
        self._complete_streak = 0
        self._last_complete_revision = -1
        self._observation_revision = 0
        self._last_complete_observation = -1
        self.safety_stops = 0
        self.replans = 0
        self.blocked_cycles = 0

    def integrate_planar_scan(self, robot_xy, points_xy, now_ns: int,
                              *, max_range_m: float = 8.0, hit_mask=None) -> int:
        updated = self.gmap.integrate_scan(robot_xy, points_xy, now_ns,
                                           max_range_m=max_range_m, hit_mask=hit_mask)
        self._observation_revision += 1
        return updated

    def integrate_point_cloud(self, robot_pose, points_xyz, now_ns: int,
                              *, max_range_m: float = 8.0) -> int:
        updated = self.gmap.integrate_point_cloud(
            robot_pose, points_xyz, now_ns, max_range_m=max_range_m)
        self._observation_revision += 1
        return updated

    def _stop(self, status: ControlStatus, reason: str,
              safety: Optional[CollisionAssessment] = None):
        return ExplorationCommand(status, 0.0, 0.0, 0.0, reason,
                                  self.current_goal, safety, self.gmap.revision)

    def _planner_freshness(self, now_ns: int):
        return {
            "now_ns": now_ns,
            "max_age_ns": int(self.config.free_max_age_s * 1_000_000_000),
        }

    def _clear_goal(self, failed: bool) -> None:
        if failed:
            self.explorer.goal_failed()
        else:
            self.explorer.goal_reached()
        self.current_goal = None
        self._planned_revision = -1
        self._best_goal_distance = math.inf
        self._last_progress_ns = None

    def _plan_new_goal(self, pose, now_ns: int) -> Optional[ExplorationCommand]:
        self.replans += 1
        decision = self.explorer.plan(
            pose[:2], **self._planner_freshness(now_ns))
        if decision.status is ExplorationStatus.COMPLETE:
            if self._last_complete_revision != self.gmap.revision:
                self._complete_streak = 1
                self._last_complete_revision = self.gmap.revision
                self._last_complete_observation = self._observation_revision
            elif self._last_complete_observation != self._observation_revision:
                # 同じscanをcontrol tickだけ重ねても完了確認を進めない。
                self._complete_streak += 1
                self._last_complete_observation = self._observation_revision
            else:
                self._complete_streak = max(1, self._complete_streak)
            if self._complete_streak >= self.config.complete_confirmations:
                return self._stop(ControlStatus.COMPLETE,
                                  "frontier枯渇を%d回確認" % self._complete_streak)
            return self._stop(
                ControlStatus.VERIFYING_COMPLETE,
                "frontier枯渇の安定確認 %d/%d" %
                (self._complete_streak, self.config.complete_confirmations))
        self._complete_streak = 0
        self._last_complete_observation = -1
        if decision.status is not ExplorationStatus.GOAL:
            self.blocked_cycles += 1
            return self._stop(ControlStatus.BLOCKED, decision.reason)
        self.blocked_cycles = 0
        self.current_goal = decision.goal
        self._planned_revision = self.gmap.revision
        self._best_goal_distance = math.hypot(
            decision.goal.x - pose[0], decision.goal.y - pose[1])
        self._last_progress_ns = now_ns
        return None

    def _refresh_for_map_change(self, pose, now_ns: int) -> Optional[ExplorationCommand]:
        if self.current_goal is None:
            return None
        if self._planned_revision != self.gmap.revision:
            refreshed = self.explorer.refresh_path(
                pose[:2], **self._planner_freshness(now_ns))
            if refreshed is None:
                old = self.current_goal
                self._clear_goal(failed=True)
                self.replans += 1
                return self._stop(
                    ControlStatus.STOP_REPLAN,
                    "map更新でpath無効化(goal=%r)" % (old.goal_cell,))
            self.current_goal = refreshed
            self._planned_revision = self.gmap.revision
        # ageによるFREE→staleはmap revisionを変えない。各tickで残りpathの
        # traversabilityを検査し、古い地図の上を走り続けない。
        traversable = self.gmap.traversable_mask(
            inflation_radius_m=self.config.inflation_radius_m,
            **self._planner_freshness(now_ns))
        path_valid = True
        for wx, wy in self.current_goal.path:
            cell = self.gmap.world_to_cell(wx, wy)
            if cell is None or not traversable[cell[1], cell[0]]:
                path_valid = False
                break
        if not path_valid:
            old = self.current_goal
            self._clear_goal(failed=True)
            self.replans += 1
            return self._stop(
                ControlStatus.STOP_REPLAN,
                "pathがstaleまたはblocked(goal=%r)" % (old.goal_cell,))
        return None

    def _lookahead(self, pose) -> Sequence[float]:
        path = self.current_goal.path
        if not path:
            return (self.current_goal.x, self.current_goal.y)
        # 現在poseに最も近いpath indexから、同一直線上だけlookaheadする。
        # 曲がり角を跨いだlookaheadはinflated pathの内側をショートカットして
        # 壁へ接近するため禁止する。
        nearest = min(range(len(path)),
                      key=lambda i: (math.hypot(path[i][0] - pose[0],
                                                path[i][1] - pose[1]), i))
        if nearest >= len(path) - 1:
            return path[-1]
        target = path[nearest + 1]
        first_dx = target[0] - path[nearest][0]
        first_dy = target[1] - path[nearest][1]
        first_norm = math.hypot(first_dx, first_dy)
        if first_norm <= 1e-12:
            return target
        first_dx, first_dy = first_dx / first_norm, first_dy / first_norm
        travelled = math.hypot(target[0] - pose[0], target[1] - pose[1])
        prev = target
        for point in path[nearest + 2:]:
            dx, dy = point[0] - prev[0], point[1] - prev[1]
            edge = math.hypot(dx, dy)
            if edge <= 1e-12:
                continue
            dx, dy = dx / edge, dy / edge
            if abs(dx * first_dy - dy * first_dx) > 1e-6 \
                    or dx * first_dx + dy * first_dy < 0.999:
                break
            if travelled + edge > self.config.lookahead_m:
                break
            travelled += edge
            target = point
            prev = point
        return target

    def step(self, robot_pose, now_ns: int, *, points_xyz=None,
             cloud_timestamp_s: Optional[float] = None,
             scan_valid: bool = True,
             hazard: Optional[Mapping] = None) -> ExplorationCommand:
        """最新map/point cloudから1 tickのbody-frame commandを返す。"""
        if len(robot_pose) < 4:
            raise ContractViolation("robot_pose", "(x,y,z,yaw)が必要")
        pose = np.asarray(robot_pose[:4], dtype=np.float64)
        if not np.isfinite(pose).all():
            raise ContractViolation("robot_pose", "有限値が必要")
        if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns <= 0:
            raise ContractViolation("now_ns", "正のmonotonic nsが必要")
        self.explorer.observe_pose(pose[:2])

        if self.current_goal is not None:
            goal_distance = math.hypot(self.current_goal.x - pose[0],
                                       self.current_goal.y - pose[1])
            if goal_distance <= self.config.goal_tolerance_m:
                self._clear_goal(failed=False)
            else:
                if goal_distance <= self._best_goal_distance - self.config.progress_epsilon_m:
                    self._best_goal_distance = goal_distance
                    self._last_progress_ns = now_ns
                elif self._last_progress_ns is not None and \
                        now_ns - self._last_progress_ns > \
                        int(self.config.progress_timeout_s * 1_000_000_000):
                    failed = self.current_goal.goal_cell
                    self._clear_goal(failed=True)
                    self.safety_stops += 1
                    return self._stop(ControlStatus.STOP_REPLAN,
                                      "progress timeout(goal=%r)" % (failed,))

        refresh_stop = self._refresh_for_map_change(pose, now_ns)
        if refresh_stop is not None:
            return refresh_stop
        if self.current_goal is None:
            plan_stop = self._plan_new_goal(pose, now_ns)
            if plan_stop is not None:
                return plan_stop
        assert self.current_goal is not None

        target = self._lookahead(pose)
        desired_yaw = math.atan2(target[1] - pose[1], target[0] - pose[0])
        heading_error = _wrap_angle(desired_yaw - pose[3])
        if abs(heading_error) > self.config.heading_before_move_rad:
            cmd = (0.0, 0.0, max(-self.config.max_yaw_rate_rps,
                                  min(self.config.max_yaw_rate_rps,
                                      1.2 * heading_error)))
            status = ControlStatus.TURN
        else:
            goal_distance = math.hypot(self.current_goal.x - pose[0],
                                       self.current_goal.y - pose[1])
            speed = min(self.config.max_speed_mps,
                        max(0.06, 0.7 * goal_distance))
            cmd = (speed, 0.0,
                   max(-0.35, min(0.35, 0.9 * heading_error)))
            status = ControlStatus.MOVE

        assessment = None
        if self.guard is not None:
            if points_xyz is None or cloud_timestamp_s is None:
                self.safety_stops += 1
                return self._stop(ControlStatus.STOP_SENSOR,
                                  "collision guardian input missing")
            assessment = self.guard.assess(
                pose, points_xyz, cmd,
                now_s=now_ns / 1_000_000_000.0,
                cloud_timestamp_s=cloud_timestamp_s,
                scan_valid=scan_valid, hazard=hazard)
            if not assessment.safe:
                self.safety_stops += 1
                # 幾何障害物ならgoalをcooldownし、次tickで別pathを探す。
                if "obstacle" in assessment.reason or "wall" in assessment.reason:
                    self._clear_goal(failed=True)
                return self._stop(ControlStatus.STOP_SENSOR,
                                  assessment.reason, assessment)

        return ExplorationCommand(status, cmd[0], cmd[1], cmd[2],
                                  "path追従", self.current_goal, assessment,
                                  self.gmap.revision)

    def metrics(self) -> dict:
        out = self.explorer.metrics()
        out.update({
            "map_revision": self.gmap.revision,
            "replans": self.replans,
            "safety_stops": self.safety_stops,
            "blocked_cycles": self.blocked_cycles,
        })
        return out
