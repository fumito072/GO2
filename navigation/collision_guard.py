"""LiDAR点群で自律速度commandを毎tick遮断するlocal collision guardian。

VLMやglobal plannerの判断とは独立して動き、sensor stale、未観測の進行回廊、
制動距離内の障害物、drop/wall hazardをfail-closedで停止させる。入力点群とposeは
同一odom frameで時刻同期済みであることをI/O adapterが保証する。
"""
import math
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence

import numpy as np

from contracts.errors import ContractViolation


@dataclass(frozen=True)
class CollisionGuardConfig:
    max_cloud_age_s: float = 0.60
    robot_radius_m: float = 0.30
    static_margin_m: float = 0.12
    reaction_time_s: float = 0.25
    braking_deceleration_mps2: float = 0.50
    nominal_base_height_m: float = 0.31
    min_obstacle_height_m: float = 0.04
    max_obstacle_height_m: float = 1.50
    floor_tolerance_m: float = 0.10
    min_finite_points: int = 20
    min_corridor_evidence_points: int = 3

    def __post_init__(self):
        positive = (
            "max_cloud_age_s", "robot_radius_m", "static_margin_m",
            "reaction_time_s", "braking_deceleration_mps2",
            "nominal_base_height_m", "min_obstacle_height_m",
            "max_obstacle_height_m", "floor_tolerance_m",
        )
        for name in positive:
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or isinstance(value, bool) \
                    or not math.isfinite(value) or value <= 0:
                raise ContractViolation("guard.%s" % name, "正の有限値が必要")
        if self.max_obstacle_height_m <= self.min_obstacle_height_m:
            raise ContractViolation("guard.max_obstacle_height_m",
                                    "minより大きい値が必要")
        for name in ("min_finite_points", "min_corridor_evidence_points"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise ContractViolation("guard.%s" % name, "正のintが必要")


@dataclass(frozen=True)
class CollisionAssessment:
    safe: bool
    reason: str
    clearance_m: Optional[float]
    required_clearance_m: float
    sensor_age_s: Optional[float]
    obstacle_points: int
    evidence_points: int


class CollisionGuard:
    def __init__(self, config: CollisionGuardConfig = CollisionGuardConfig()):
        if not isinstance(config, CollisionGuardConfig):
            raise ContractViolation("config", "CollisionGuardConfigが必要")
        self.config = config

    def required_clearance(self, speed_mps: float) -> float:
        speed = abs(float(speed_mps))
        if not math.isfinite(speed):
            raise ContractViolation("speed_mps", "有限値が必要")
        c = self.config
        braking = speed * speed / (2.0 * c.braking_deceleration_mps2)
        return c.robot_radius_m + c.static_margin_m \
            + c.reaction_time_s * speed + braking

    @staticmethod
    def _stop(reason: str, required: float, age: Optional[float],
              clearance=None, obstacle_points=0, evidence_points=0):
        return CollisionAssessment(False, reason, clearance, required, age,
                                   int(obstacle_points), int(evidence_points))

    def assess(self, pose_xyz_yaw: Sequence[float], points_xyz,
               command_body: Sequence[float], *, now_s: float,
               cloud_timestamp_s: float, scan_valid: bool,
               hazard: Optional[Mapping] = None) -> CollisionAssessment:
        """command_body=(vx,vy,wz)を許可できるか判定する。

        stop/hold commandはsensor状態によらず許可する。運動commandはfreshな点群と
        進行回廊の観測証拠を必須とする。
        """
        if len(command_body) != 3:
            raise ContractViolation("command_body", "(vx,vy,wz)が必要")
        cmd = np.asarray(command_body, dtype=np.float64)
        if not np.isfinite(cmd).all():
            raise ContractViolation("command_body", "有限値が必要")
        vx, vy, wz = (float(v) for v in cmd)
        linear_speed = math.hypot(vx, vy)
        moving = linear_speed > 1e-6 or abs(wz) > 1e-6
        required = self.required_clearance(linear_speed)
        if not moving:
            return CollisionAssessment(True, "hold", None, required, None, 0, 0)

        if not isinstance(scan_valid, (bool, np.bool_)):
            raise ContractViolation("scan_valid", "boolが必要")
        if not isinstance(now_s, (int, float)) or isinstance(now_s, bool) \
                or not math.isfinite(now_s):
            raise ContractViolation("now_s", "有限値が必要")
        if not isinstance(cloud_timestamp_s, (int, float)) \
                or isinstance(cloud_timestamp_s, bool) \
                or not math.isfinite(cloud_timestamp_s):
            raise ContractViolation("cloud_timestamp_s", "有限値が必要")
        age = float(now_s - cloud_timestamp_s)
        if not scan_valid:
            return self._stop("LiDAR scan invalid", required, age)
        if cloud_timestamp_s <= 0 or age < 0 or age > self.config.max_cloud_age_s:
            return self._stop("LiDAR stale(age=%.3fs)" % age, required, age)

        if len(pose_xyz_yaw) < 4:
            raise ContractViolation("pose_xyz_yaw", "(x,y,z,yaw)が必要")
        pose = np.asarray(pose_xyz_yaw[:4], dtype=np.float64)
        if not np.isfinite(pose).all():
            raise ContractViolation("pose_xyz_yaw", "有限値が必要")
        pts = np.asarray(points_xyz, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] < 3:
            return self._stop("LiDAR points missing", required, age)
        pts = pts[:, :3]
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) < self.config.min_finite_points:
            return self._stop("LiDAR point density insufficient(%d)" % len(pts),
                              required, age)

        if hazard:
            kind = str(hazard.get("kind", "")).lower()
            distance = hazard.get("distance")
            if kind == "drop":
                return self._stop("drop hazard", required, age)
            if kind == "wall":
                try:
                    wall_d = float(distance)
                except (TypeError, ValueError):
                    wall_d = 0.0
                if not math.isfinite(wall_d) or wall_d <= required + 0.10:
                    return self._stop("wall hazard", required, age,
                                      clearance=wall_d if math.isfinite(wall_d) else None)

        x, y, z, yaw = (float(v) for v in pose)
        dx = pts[:, 0] - x
        dy = pts[:, 1] - y
        c, s = math.cos(yaw), math.sin(yaw)
        # odom/world -> body(x前, y左)
        bx = c * dx + s * dy
        by = -s * dx + c * dy
        ground_z = z - self.config.nominal_base_height_m
        height = pts[:, 2] - ground_z
        vertical = (height >= -self.config.floor_tolerance_m) \
            & (height <= self.config.max_obstacle_height_m)
        obstacle_height = vertical & (height >= self.config.min_obstacle_height_m)

        if linear_speed > 1e-6:
            ux, uy = vx / linear_speed, vy / linear_speed
            along = bx * ux + by * uy
            lateral = np.abs(-bx * uy + by * ux)
            corridor_half_width = self.config.robot_radius_m + self.config.static_margin_m
            corridor = (along > 0.02) & (along <= required + 0.15) \
                & (lateral <= corridor_half_width) & vertical
            evidence_count = int(np.count_nonzero(corridor))
            obstacle = corridor & obstacle_height
            obstacle_count = int(np.count_nonzero(obstacle))
            if obstacle_count:
                clearance = float(np.min(along[obstacle]))
                if clearance <= required:
                    return self._stop(
                        "obstacle inside stopping corridor(%.3fm <= %.3fm)" %
                        (clearance, required), required, age,
                        clearance=clearance, obstacle_points=obstacle_count,
                        evidence_points=evidence_count)
            if evidence_count < self.config.min_corridor_evidence_points:
                return self._stop(
                    "motion corridor unobserved(%d points)" % evidence_count,
                    required, age, obstacle_points=obstacle_count,
                    evidence_points=evidence_count)
            clearance = (float(np.min(along[obstacle])) if obstacle_count else None)
            return CollisionAssessment(True, "corridor clear", clearance, required,
                                       age, obstacle_count, evidence_count)

        # pure rotation: body/leg swept circle内の障害物を拒否。
        radial = np.hypot(bx, by)
        rotation_observed = vertical & (radial <= required + 0.15)
        evidence_count = int(np.count_nonzero(rotation_observed))
        swept = obstacle_height & (radial <= required)
        count = int(np.count_nonzero(swept))
        if count:
            clearance = float(np.min(radial[swept]))
            return self._stop("obstacle inside rotation footprint", required, age,
                              clearance=clearance, obstacle_points=count,
                              evidence_points=evidence_count)
        if evidence_count < self.config.min_corridor_evidence_points:
            return self._stop(
                "rotation footprint unobserved(%d points)" % evidence_count,
                required, age, evidence_points=evidence_count)
        return CollisionAssessment(True, "rotation footprint clear", None, required,
                                   age, 0, evidence_count)
