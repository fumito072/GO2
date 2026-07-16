"""navigation.frontier_explorer — frontier ベース探索の決定的 baseline(docs/10 §5)。

純ロジック・決定的(同じ地図+同じ pose → 同じ goal)。時刻や乱数を使わない。
explorer は「次に観測すべき goal pose」を提案するだけで、
実行可否は Mission FSM / arbiter / gateway が決める(invariant 1, 2)。

安全規則(docs/10 §5):
  - 通行判定は FREE cell のみ(UNKNOWN は不可 — invariant 9)、inflate 済み。
  - 1 goal の移動は max_step_m 以内に制限し、goal ごとに arbiter を通す。
  - frontier 枯渇 → COMPLETE(EXPLORATION_COMPLETE へ)。観測ゼロや
    到達可能 frontier なしは COMPLETE と区別する(誤完了の防止)。
"""
import math
from dataclasses import dataclass
from enum import Enum, unique
from typing import List, Optional, Tuple

import numpy as np

from contracts.errors import ContractViolation
from perception.global_map import GlobalOccupancyMap, UNKNOWN, FREE


@unique
class ExplorationStatus(Enum):
    GOAL = "GOAL"                          # 次の観測 goal を提案
    COMPLETE = "COMPLETE"                  # frontier 枯渇 = 探索完了
    NO_OBSERVATIONS = "NO_OBSERVATIONS"    # まだ何も観測していない(完了ではない)
    NO_REACHABLE_FRONTIER = "NO_REACHABLE_FRONTIER"  # frontier はあるが到達経路なし


@dataclass(frozen=True)
class ExplorationGoal:
    x: float
    y: float
    yaw: float                 # goal で向くべき方向(frontier centroid へ)
    frontier_cells: int        # 対象 cluster の大きさ(情報利得の代理)
    distance_m: float          # robot からの直線距離(clip 後)


@dataclass(frozen=True)
class ExplorationDecision:
    status: ExplorationStatus
    goal: Optional[ExplorationGoal]
    reason: str


def find_frontier_clusters(gmap: GlobalOccupancyMap,
                           traversable: np.ndarray) -> List[List[Tuple[int, int]]]:
    """frontier cell(= 通行可能 FREE cell で UNKNOWN に 4近傍で接する)を
    抽出し、8近傍で連結 cluster にまとめる。決定的順序で返す。"""
    g = gmap.grid
    h, w = g.shape
    unknown = (g == UNKNOWN)
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    frontier = traversable & near_unknown

    clusters: List[List[Tuple[int, int]]] = []
    visited = np.zeros_like(frontier)
    ys, xs = np.nonzero(frontier)
    # 決定的な走査順(iy, ix 昇順)
    for iy, ix in sorted(zip(ys.tolist(), xs.tolist())):
        if visited[iy, ix]:
            continue
        stack = [(ix, iy)]
        visited[iy, ix] = True
        cluster = []
        while stack:
            cx, cy = stack.pop()
            cluster.append((cx, cy))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h \
                            and frontier[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        stack.append((nx, ny))
        clusters.append(sorted(cluster))
    return clusters


def next_goal(gmap: GlobalOccupancyMap, robot_xy,
              min_cluster_cells: int = 5,
              max_step_m: float = 3.0,
              inflate_cells: int = 3) -> ExplorationDecision:
    """次の観測 goal を決定的に選ぶ。

    score = cluster サイズ / (1 + 距離)。tie は cell 座標順で決定的に解く。
    """
    if not isinstance(gmap, GlobalOccupancyMap):
        raise ContractViolation("gmap", "GlobalOccupancyMap が必要")
    counts = gmap.counts()
    if counts["free"] == 0:
        # 観測ゼロ。探索「完了」と混同しない(誤完了の防止 — docs/10 §5)
        return ExplorationDecision(ExplorationStatus.NO_OBSERVATIONS, None,
                                   "FREE cell がまだ存在しない")
    rc = gmap.world_to_cell(float(robot_xy[0]), float(robot_xy[1]))
    if rc is None:
        raise ContractViolation("robot_xy", "地図範囲外: %r" % (robot_xy,))

    traversable = gmap.traversable_mask(inflate_cells=inflate_cells)
    clusters = [c for c in find_frontier_clusters(gmap, traversable)
                if len(c) >= min_cluster_cells]
    if not clusters:
        return ExplorationDecision(ExplorationStatus.COMPLETE, None,
                                   "frontier 枯渇(min_cluster=%d)" % min_cluster_cells)

    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    scored = []
    for c in clusters:
        cxs = [p[0] for p in c]
        cys = [p[1] for p in c]
        centroid = (sum(cxs) / len(c), sum(cys) / len(c))
        wx, wy = gmap.cell_to_world(int(round(centroid[0])), int(round(centroid[1])))
        dist = math.hypot(wx - rx, wy - ry)
        score = len(c) / (1.0 + dist)
        scored.append((-score, centroid[1], centroid[0], c, centroid, dist))
    scored.sort(key=lambda s: (s[0], s[1], s[2]))

    for _, _, _, cluster, centroid, _ in scored:
        # goal 候補 = cluster 内で robot に最も近い cell(FREE by construction)
        best = min(cluster,
                   key=lambda p: (math.hypot(*(np.subtract(
                       gmap.cell_to_world(p[0], p[1]), (rx, ry)))), p[1], p[0]))
        gx, gy = gmap.cell_to_world(best[0], best[1])
        d = math.hypot(gx - rx, gy - ry)
        if d > max_step_m:
            # robot→goal の直線上、max_step 以内で最遠の通行可能 cell に clip
            clipped = _clip_along_ray(gmap, traversable, (rx, ry), (gx, gy),
                                      max_step_m)
            if clipped is None:
                continue  # この cluster へは今回踏み出せない → 次の cluster
            gx, gy = clipped
            d = math.hypot(gx - rx, gy - ry)
            if d <= 1e-6:
                continue
        cwx, cwy = gmap.cell_to_world(int(round(centroid[0])),
                                      int(round(centroid[1])))
        yaw = math.atan2(cwy - gy, cwx - gx)
        return ExplorationDecision(
            ExplorationStatus.GOAL,
            ExplorationGoal(x=gx, y=gy, yaw=yaw,
                            frontier_cells=len(cluster), distance_m=d),
            "frontier cluster(%d cells)へ前進" % len(cluster))

    return ExplorationDecision(ExplorationStatus.NO_REACHABLE_FRONTIER, None,
                               "frontier はあるが今回踏み出せる経路がない")


def _clip_along_ray(gmap: GlobalOccupancyMap, traversable: np.ndarray,
                    start_xy, goal_xy, max_step_m: float):
    """start→goal の直線上、max_step 以内で最も遠い通行可能 cell の world 座標。
    直線上に通行不可 cell が現れたらそこで打ち切る(その手前まで)。"""
    sx, sy = start_xy
    gx, gy = goal_xy
    d = math.hypot(gx - sx, gy - sy)
    if d <= 0:
        return None
    n = max(2, int(d / gmap.resolution_m))
    best = None
    for i in range(1, n + 1):
        t = i / n
        x = sx + (gx - sx) * t
        y = sy + (gy - sy) * t
        if math.hypot(x - sx, y - sy) > max_step_m:
            break
        c = gmap.world_to_cell(x, y)
        if c is None or not traversable[c[1], c[0]]:
            break
        best = (x, y)
    return best
