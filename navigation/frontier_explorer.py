"""到達可能性と探索履歴を考慮した frontier exploration baseline。

重要な安全契約:
  - UNKNOWN / stale FREE は通行不可。
  - goal cellだけでなく、robotからgoalまでのinflate済みFREE pathを必ず探索する。
  - max_step_mは直線距離ではなく計画pathの累積距離へ適用する。
  - stateless ``next_goal`` は互換API。実運用は履歴を持つ ``FrontierExplorer`` を使う。
  - 本moduleはgoal/pathを提案するだけで、最終的な速度許可はlocal collision
    guardianが毎tick判断する。
  - 旧APIのrng引数は互換のため受理するが、LIVE選択は決定的にする。
"""
import heapq
import math
from collections import deque
from dataclasses import dataclass, replace
from enum import Enum, unique
from typing import Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from contracts.errors import ContractViolation
from perception.global_map import GlobalOccupancyMap, UNKNOWN, FREE

Cell = Tuple[int, int]
WorldPoint = Tuple[float, float]


@unique
class ExplorationStatus(Enum):
    GOAL = "GOAL"
    COMPLETE = "COMPLETE"
    NO_OBSERVATIONS = "NO_OBSERVATIONS"
    NO_REACHABLE_FRONTIER = "NO_REACHABLE_FRONTIER"


@dataclass(frozen=True)
class ExplorationGoal:
    x: float
    y: float
    yaw: float
    frontier_cells: int
    distance_m: float
    # controllerが直線で壁を抜けないためのglobal path。先頭はrobot近傍、末尾はgoal。
    path: Tuple[WorldPoint, ...] = ()
    path_length_m: float = 0.0
    goal_cell: Optional[Cell] = None
    frontier_cell: Optional[Cell] = None


@dataclass(frozen=True)
class ExplorationDecision:
    status: ExplorationStatus
    goal: Optional[ExplorationGoal]
    reason: str


def find_frontier_clusters(gmap: GlobalOccupancyMap,
                           traversable: np.ndarray) -> List[List[Cell]]:
    """FREEかつUNKNOWNと4近傍で接するcellを8近傍clusterへまとめる。"""
    if traversable.shape != gmap.grid.shape:
        raise ContractViolation("traversable", "mapと同じshapeが必要")
    g = gmap.grid
    h, w = g.shape
    unknown = (g == UNKNOWN)
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    frontier = traversable & near_unknown

    clusters: List[List[Cell]] = []
    seen = np.zeros_like(frontier)
    ys, xs = np.nonzero(frontier)
    for iy, ix in sorted(zip(ys.tolist(), xs.tolist())):
        if seen[iy, ix]:
            continue
        stack = [(ix, iy)]
        seen[iy, ix] = True
        cluster = []
        while stack:
            cx, cy = stack.pop()
            cluster.append((cx, cy))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h \
                            and frontier[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        stack.append((nx, ny))
        clusters.append(sorted(cluster))
    return clusters


def _neighbors(cell: Cell, traversable: np.ndarray):
    """8近傍。対角移動ではcorner cuttingを禁止する。"""
    x, y = cell
    h, w = traversable.shape
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h and traversable[ny, nx]):
                continue
            if dx and dy:
                # diagonalの両脇がFREEでなければrobot footprintが角を横切る。
                if not (traversable[y, nx] and traversable[ny, x]):
                    continue
                cost = math.sqrt(2.0)
            else:
                cost = 1.0
            yield (nx, ny), cost


def _dijkstra(traversable: np.ndarray, start: Cell):
    h, w = traversable.shape
    dist = np.full((h, w), np.inf, dtype=np.float64)
    parent_x = np.full((h, w), -1, dtype=np.int32)
    parent_y = np.full((h, w), -1, dtype=np.int32)
    if not traversable[start[1], start[0]]:
        return dist, parent_x, parent_y
    dist[start[1], start[0]] = 0.0
    heap = [(0.0, start[1], start[0])]
    while heap:
        d, y, x = heapq.heappop(heap)
        if d != dist[y, x]:
            continue
        for (nx, ny), edge in _neighbors((x, y), traversable):
            nd = d + edge
            # deterministic tie: parent cellの辞書順を固定する。
            old = dist[ny, nx]
            better_tie = (abs(nd - old) <= 1e-12 and
                          (y, x) < (parent_y[ny, nx], parent_x[ny, nx]))
            if nd < old - 1e-12 or better_tie:
                dist[ny, nx] = nd
                parent_x[ny, nx] = x
                parent_y[ny, nx] = y
                heapq.heappush(heap, (nd, ny, nx))
    return dist, parent_x, parent_y


def _restore_path(start: Cell, goal: Cell,
                  parent_x: np.ndarray, parent_y: np.ndarray) -> List[Cell]:
    if goal == start:
        return [start]
    if parent_x[goal[1], goal[0]] < 0:
        return []
    out = [goal]
    cur = goal
    limit = parent_x.size + 1
    while cur != start and len(out) <= limit:
        px = int(parent_x[cur[1], cur[0]])
        py = int(parent_y[cur[1], cur[0]])
        if px < 0 or py < 0:
            return []
        cur = (px, py)
        out.append(cur)
    if cur != start:
        return []
    out.reverse()
    return out


def _path_length_cells(path: Sequence[Cell], resolution_m: float) -> float:
    total = 0.0
    for a, b in zip(path, path[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1]) * resolution_m
    return total


def _truncate_path(path: Sequence[Cell], resolution_m: float,
                   max_length_m: float) -> List[Cell]:
    if not path:
        return []
    out = [path[0]]
    travelled = 0.0
    for a, b in zip(path, path[1:]):
        edge = math.hypot(b[0] - a[0], b[1] - a[1]) * resolution_m
        if travelled + edge > max_length_m + 1e-12:
            break
        travelled += edge
        out.append(b)
    return out


def _unknown_gain(gmap: GlobalOccupancyMap, cell: Cell, radius_cells: int = 5) -> int:
    x, y = cell
    y0, y1 = max(0, y - radius_cells), min(gmap.height, y + radius_cells + 1)
    x0, x1 = max(0, x - radius_cells), min(gmap.width, x + radius_cells + 1)
    return int(np.sum(gmap.grid[y0:y1, x0:x1] == UNKNOWN))


def _near_any(cell: Cell, others: Iterable[Cell], radius_cells: int) -> bool:
    r2 = radius_cells * radius_cells
    return any((cell[0] - p[0]) ** 2 + (cell[1] - p[1]) ** 2 <= r2
               for p in others)


def path_to_cell(gmap: GlobalOccupancyMap, robot_xy, goal_cell: Cell,
                 *, inflation_radius_m: float = 0.30,
                 now_ns=None, max_age_ns=None) -> Tuple[WorldPoint, ...]:
    """現在poseから指定FREE cellまでのcollision-inflated shortest path。"""
    rc = gmap.world_to_cell(float(robot_xy[0]), float(robot_xy[1]))
    if rc is None:
        raise ContractViolation("robot_xy", "地図範囲外: %r" % (robot_xy,))
    if not (0 <= goal_cell[0] < gmap.width and 0 <= goal_cell[1] < gmap.height):
        raise ContractViolation("goal_cell", "地図範囲外: %r" % (goal_cell,))
    traversable = gmap.traversable_mask(
        inflation_radius_m=inflation_radius_m,
        now_ns=now_ns, max_age_ns=max_age_ns)
    dist, px, py = _dijkstra(traversable, rc)
    if not np.isfinite(dist[goal_cell[1], goal_cell[0]]):
        return ()
    cells = _restore_path(rc, goal_cell, px, py)
    return tuple(gmap.cell_to_world(x, y) for x, y in cells)


def next_goal(gmap: GlobalOccupancyMap, robot_xy,
              min_cluster_cells: int = 5,
              max_step_m: float = 3.0,
              inflate_cells: Optional[int] = None,
              *, inflation_radius_m: float = 0.30,
              standoff_m: float = 0.35,
              min_progress_m: float = 0.20,
              visit_counts: Optional[np.ndarray] = None,
              recent_goal_cells: Sequence[Cell] = (),
              failed_goal_cells: Iterable[Cell] = (),
              now_ns=None, max_age_ns=None,
              optimistic: bool = False,
              min_goal_dist_m: float = 0.0,
              avoid_xy: Sequence[WorldPoint] = (),
              avoid_radius_m: float = 0.5,
              rng=None) -> ExplorationDecision:
    """reachable frontierへ至るpath付きgoalを決定的に返す。

    小clusterは優先度を下げるだけで捨てない。frontierが存在するのにfilter後が
    空になった状態をCOMPLETEと誤判定しない。

    optimisticは旧API互換で受理するが、LIVE plannerではUNKNOWNを通行可に
    しない。min_goal_dist_m/avoid_xyはFREE-only A*候補へだけ適用する。
    rngは旧呼び出し互換のため検証だけ行い、goal選択には使わない。
    """
    if not isinstance(gmap, GlobalOccupancyMap):
        raise ContractViolation("gmap", "GlobalOccupancyMap が必要")
    if min_cluster_cells <= 0 or max_step_m <= 0 or standoff_m < 0 \
            or min_progress_m < 0 or min_goal_dist_m < 0 \
            or avoid_radius_m < 0:
        raise ContractViolation("planner_config", "距離/cluster設定が不正")
    if not isinstance(optimistic, (bool, np.bool_)):
        raise ContractViolation("optimistic", "boolが必要")
    if rng is not None and not callable(getattr(rng, "random", None)):
        raise ContractViolation("rng", "random()を持つ必要")
    counts = gmap.counts()
    if counts["free"] == 0:
        return ExplorationDecision(ExplorationStatus.NO_OBSERVATIONS, None,
                                   "FREE cell がまだ存在しない")
    rc = gmap.world_to_cell(float(robot_xy[0]), float(robot_xy[1]))
    if rc is None:
        raise ContractViolation("robot_xy", "地図範囲外: %r" % (robot_xy,))
    if visit_counts is not None and visit_counts.shape != gmap.grid.shape:
        raise ContractViolation("visit_counts", "mapと同じshapeが必要")

    kwargs = {}
    if inflate_cells is not None:
        kwargs["inflate_cells"] = inflate_cells
    else:
        kwargs["inflation_radius_m"] = inflation_radius_m
    if now_ns is not None or max_age_ns is not None:
        kwargs.update(now_ns=now_ns, max_age_ns=max_age_ns)
    traversable = gmap.traversable_mask(**kwargs)
    # start自体がfresh FREEでなければ、frontier有無や完了を論じられない。
    # この判定をCOMPLETEより先に置き、全FREEの期限切れを誤完了にしない。
    if not traversable[rc[1], rc[0]]:
        return ExplorationDecision(ExplorationStatus.NO_REACHABLE_FRONTIER, None,
                                   "robot cellがstaleまたはinflation領域")
    raw_clusters = find_frontier_clusters(gmap, traversable)
    if not raw_clusters:
        # freshnessを有効にした結果だけfrontierが消えた場合、それは探索完了では
        # なく「既知FREEが古く、経路を証明できない」状態。stale mapを根拠に
        # COMPLETEを出すと、LiDAR停止直後に未探索領域を残して終了してしまう。
        if now_ns is not None:
            base_kwargs = {k: v for k, v in kwargs.items()
                           if k not in ("now_ns", "max_age_ns")}
            base_traversable = gmap.traversable_mask(**base_kwargs)
            if find_frontier_clusters(gmap, base_traversable):
                return ExplorationDecision(
                    ExplorationStatus.NO_REACHABLE_FRONTIER, None,
                    "frontierはあるがFREE pathがstale")
        return ExplorationDecision(ExplorationStatus.COMPLETE, None,
                                   "reachable-map上のfrontier枯渇")
    dist, parent_x, parent_y = _dijkstra(traversable, rc)
    failed = set(failed_goal_cells)
    revisit_radius = max(1, int(math.ceil(0.50 / gmap.resolution_m)))
    candidates = []
    for cluster_index, cluster in enumerate(raw_clusters):
        preferred_size = len(cluster) >= min_cluster_cells
        for fc in cluster:
            cell_dist = dist[fc[1], fc[0]]
            if not np.isfinite(cell_dist) or fc in failed:
                continue
            full_path = _restore_path(rc, fc, parent_x, parent_y)
            if len(full_path) < 2:
                continue
            full_length = _path_length_cells(full_path, gmap.resolution_m)
            # frontierへ直接踏み込まず、かつ1 goalのpath長をmax_stepへ制限する。
            # failure/recent/visitは遠方の最終viewpointではなく、このrunが実際に
            # 目指す切り詰め後goalへ適用する。同じ中間goalの再発行を防ぐ。
            target_length = min(
                max_step_m,
                max(min_progress_m, full_length - standoff_m),
            )
            candidate_path = _truncate_path(
                full_path, gmap.resolution_m, target_length)
            if len(candidate_path) < 2:
                continue
            goal_cell = candidate_path[-1]
            # failure stateにはfrontier cellではなく実際に停止したviewpoint
            # (goal_cell)を保存する。同じ近傍のfrontier表現へ名前を変えて
            # 即再試行する抜け道もradiusで塞ぐ。
            if _near_any(goal_cell, failed, revisit_radius):
                continue
            path_len = _path_length_cells(candidate_path, gmap.resolution_m)
            if path_len < min_progress_m - 1e-9:
                continue
            gain = _unknown_gain(gmap, fc)
            visits = int(visit_counts[goal_cell[1], goal_cell[0]]) \
                if visit_counts is not None else 0
            recent = _near_any(goal_cell, recent_goal_cells, revisit_radius)
            # utility: 情報利得/実path cost。小cluster・再訪・高visitを強く減点。
            utility = (len(cluster) + 0.20 * gain) / (1.0 + full_length)
            utility -= 2.0 * visits
            if recent:
                utility -= 25.0
            if not preferred_size:
                utility -= 2.0
            goal_xy = gmap.cell_to_world(*goal_cell)
            if any(math.hypot(goal_xy[0] - float(ax),
                              goal_xy[1] - float(ay)) < avoid_radius_m
                   for ax, ay in avoid_xy):
                continue
            if path_len < max(min_progress_m, min_goal_dist_m) - 1e-9:
                continue
            candidates.append((-utility, visits, recent, full_length,
                               fc[1], fc[0], cluster_index, cluster,
                               fc, candidate_path))

    if not candidates:
        return ExplorationDecision(ExplorationStatus.NO_REACHABLE_FRONTIER, None,
                                   "frontierはあるがinflate済みFREE pathがない")
    candidates.sort(key=lambda row: row[:7])
    _, _, _, _, _, _, _, cluster, frontier_cell, path_cells = candidates[0]
    goal_cell = path_cells[-1]
    path_world = tuple(gmap.cell_to_world(x, y) for x, y in path_cells)
    gx, gy = path_world[-1]
    fwx, fwy = gmap.cell_to_world(*frontier_cell)
    yaw = math.atan2(fwy - gy, fwx - gx)
    path_length = _path_length_cells(path_cells, gmap.resolution_m)
    direct = math.hypot(gx - float(robot_xy[0]), gy - float(robot_xy[1]))
    reason = ("frontier %d cells / path %.2fm / direct %.2fm%s" %
              (len(cluster), path_length, direct,
               " / small-frontier fallback" if len(cluster) < min_cluster_cells else ""))
    return ExplorationDecision(
        ExplorationStatus.GOAL,
        ExplorationGoal(gx, gy, yaw, len(cluster), direct,
                        path=path_world, path_length_m=path_length,
                        goal_cell=goal_cell, frontier_cell=frontier_cell),
        reason)


def _clip_along_ray(gmap: GlobalOccupancyMap, traversable: np.ndarray,
                    start_xy, goal_xy, max_step_m: float):
    """互換用の保守的な直線clip。

    渡されたmask上で最初の非通行cellより先へ進まない。LIVEのfrontier計画は
    このhelperではなく、上のFREE-only Dijkstra/A* pathを使う。
    """
    if traversable.shape != gmap.grid.shape:
        raise ContractViolation("traversable", "mapと同じshapeが必要")
    if max_step_m <= 0:
        raise ContractViolation("max_step_m", "正の値が必要")
    sx, sy = float(start_xy[0]), float(start_xy[1])
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    distance = math.hypot(gx - sx, gy - sy)
    if distance <= 0:
        return None
    samples = max(2, int(math.ceil(distance / gmap.resolution_m)) * 2)
    best = None
    for i in range(1, samples + 1):
        t = i / samples
        x = sx + (gx - sx) * t
        y = sy + (gy - sy) * t
        if math.hypot(x - sx, y - sy) > max_step_m + 1e-12:
            break
        cell = gmap.world_to_cell(x, y)
        if cell is None or not traversable[cell[1], cell[0]]:
            break
        best = (x, y)
    return best


class FrontierExplorer:
    """robot trace、visit density、goal retry/cooldownを保持するplanner state。"""

    def __init__(self, gmap: GlobalOccupancyMap, *, recent_goal_limit: int = 12,
                 failure_cooldown_plans: int = 30, **planner_kwargs):
        if not isinstance(gmap, GlobalOccupancyMap):
            raise ContractViolation("gmap", "GlobalOccupancyMap が必要")
        self.gmap = gmap
        self.planner_kwargs = dict(planner_kwargs)
        self.visit_counts = np.zeros_like(gmap.grid, dtype=np.uint16)
        self.robot_trace: List[WorldPoint] = []
        self.recent_goal_cells = deque(maxlen=recent_goal_limit)
        self._failed_until: Mapping[Cell, int] = {}
        self._plan_index = 0
        self.failure_cooldown_plans = int(failure_cooldown_plans)
        self.current_goal: Optional[ExplorationGoal] = None
        self.goal_attempts = {}
        self._last_trace_cell: Optional[Cell] = None

    def observe_pose(self, robot_xy) -> None:
        cell = self.gmap.world_to_cell(float(robot_xy[0]), float(robot_xy[1]))
        if cell is None:
            raise ContractViolation("robot_xy", "地図範囲外: %r" % (robot_xy,))
        # 制御tick数ではなく、実際に新しいcellへ移った時だけvisitを加算する。
        if cell != self._last_trace_cell:
            cur = int(self.visit_counts[cell[1], cell[0]])
            self.visit_counts[cell[1], cell[0]] = min(np.iinfo(np.uint16).max, cur + 1)
            self.robot_trace.append((float(robot_xy[0]), float(robot_xy[1])))
            self._last_trace_cell = cell

    def _active_failures(self):
        return {cell for cell, until in self._failed_until.items()
                if until > self._plan_index}

    def plan(self, robot_xy, **overrides) -> ExplorationDecision:
        self.observe_pose(robot_xy)
        self._plan_index += 1
        args = dict(self.planner_kwargs)
        args.update(overrides)
        decision = next_goal(
            self.gmap, robot_xy,
            visit_counts=self.visit_counts,
            recent_goal_cells=tuple(self.recent_goal_cells),
            failed_goal_cells=self._active_failures(),
            **args)
        if decision.status is ExplorationStatus.GOAL:
            goal = decision.goal
            if self.current_goal is None or goal.goal_cell != self.current_goal.goal_cell:
                self.goal_attempts[goal.goal_cell] = self.goal_attempts.get(goal.goal_cell, 0) + 1
            self.current_goal = goal
        else:
            self.current_goal = None
        return decision

    def refresh_path(self, robot_xy, **overrides) -> Optional[ExplorationGoal]:
        """map更新後に現在goalへのpathを再計算。塞がれたらNone。"""
        if self.current_goal is None or self.current_goal.goal_cell is None:
            return None
        args = dict(self.planner_kwargs)
        args.update(overrides)
        allowed = {"inflation_radius_m", "now_ns", "max_age_ns"}
        path = path_to_cell(self.gmap, robot_xy, self.current_goal.goal_cell,
                            **{k: v for k, v in args.items() if k in allowed})
        if not path:
            return None
        path_length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                          for a, b in zip(path, path[1:]))
        goal = replace(self.current_goal, path=path, path_length_m=path_length,
                       distance_m=math.hypot(path[-1][0] - float(robot_xy[0]),
                                             path[-1][1] - float(robot_xy[1])))
        self.current_goal = goal
        return goal

    def goal_reached(self) -> None:
        if self.current_goal is not None and self.current_goal.goal_cell is not None:
            self.recent_goal_cells.append(self.current_goal.goal_cell)
        self.current_goal = None

    def goal_failed(self) -> None:
        if self.current_goal is not None and self.current_goal.goal_cell is not None:
            cell = self.current_goal.goal_cell
            self.recent_goal_cells.append(cell)
            # 同じstuck goalを即再発行しない。有界plan回数後に再評価可能。
            failed = dict(self._failed_until)
            failed[cell] = self._plan_index + self.failure_cooldown_plans
            self._failed_until = failed
        self.current_goal = None

    def metrics(self) -> dict:
        moves = max(0, len(self.robot_trace) - 1)
        unique = int(np.count_nonzero(self.visit_counts))
        repeated = int(np.sum(np.maximum(self.visit_counts.astype(np.int64) - 1, 0)))
        return {
            "trace_points": len(self.robot_trace),
            "unique_cells": unique,
            "repeat_visits": repeated,
            "revisit_ratio": (repeated / max(1, moves + repeated)),
            "goals_attempted": int(sum(self.goal_attempts.values())),
        }
