"""navigation.frontier_explorer — frontier ベース探索の決定的 baseline(docs/10 §5)。

純ロジック。既定は決定的(同じ地図+同じ pose → 同じ goal)で時刻や乱数を
使わない。呼び出し側が seed 済み rng を注入した場合のみ、cluster 試行順を
score 重み付きで確率化する(局所偏重の回避 — 操作者要望 2026-07-18。
同じ seed なら再現可能なので、テスト容易性は維持される)。
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
              inflate_cells: int = 3,
              optimistic: bool = False,
              min_goal_dist_m: float = 0.0,
              avoid_xy=(),
              avoid_radius_m: float = 0.5,
              rng=None) -> ExplorationDecision:
    """次の観測 goal を選ぶ。rng=None(既定)なら完全に決定的。

    score = cluster サイズ / (1 + 距離)。tie は cell 座標順で決定的に解く。

    rng(random.Random)を渡すと、cluster の試行順を score 重み付きで
    シャッフルする(Efraimidis–Spirakis)。score が高い cluster ほど先に
    試されやすいが常に同じ順ではない — 毎回同じ frontier に向かう局所
    偏重を避ける(操作者要望 2026-07-18)。同じ seed なら再現可能。

    optimistic=True(探索計画専用): goal への踏み出し判定(_clip_along_ray)を
    非OCCUPIED 通行可で行う — 未踏(UNKNOWN)域を通って frontier へ向かえる
    (操作者要望 2026-07-17)。frontier の定義自体は従来どおり
    「観測済み FREE と UNKNOWN の境界」のまま(UNKNOWN 全体を frontier に
    しないため)。

    min_goal_dist_m: goal がこれより近い場合、frontier 境界の先(cluster
    centroid 方向)へ押し出す。床観測が疎な実機では robot 自身が frontier 上に
    立つことがあり、押し出しがないと「即到達→同じ goal 再選択」で動けなくなる。

    avoid_xy: 最近閉塞などで断念した goal 座標のリスト。半径 avoid_radius_m
    以内の goal を出す cluster はスキップする(同じ袋小路の再選択防止)。
    呼び出し側が渡す限り決定性は保たれる。
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
    walkable = (gmap.traversable_mask(inflate_cells=inflate_cells,
                                      optimistic=True)
                if optimistic else traversable)
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
    if rng is not None and len(scored) > 1:
        # score 重み付きシャッフル: key = u^(1/score) の降順(E–S 法)。
        # 期待順位は score 順に近いが、低 score cluster も時々先頭に来る
        scored = sorted(
            scored,
            key=lambda s: rng.random() ** (1.0 / max(-s[0], 1e-9)),
            reverse=True)

    for _, _, _, cluster, centroid, _ in scored:
        # goal 候補 = cluster 内で robot に最も近い cell(FREE by construction)
        best = min(cluster,
                   key=lambda p: (math.hypot(*(np.subtract(
                       gmap.cell_to_world(p[0], p[1]), (rx, ry)))), p[1], p[0]))
        gx, gy = gmap.cell_to_world(best[0], best[1])
        d = math.hypot(gx - rx, gy - ry)
        cwx, cwy = gmap.cell_to_world(int(round(centroid[0])),
                                      int(round(centroid[1])))
        if d > max_step_m:
            # robot→goal の直線上、max_step 以内で最遠の通行可能 cell に clip
            clipped = _clip_along_ray(gmap, walkable, (rx, ry), (gx, gy),
                                      max_step_m)
            if clipped is None:
                continue  # この cluster へは今回踏み出せない → 次の cluster
            gx, gy = clipped
            d = math.hypot(gx - rx, gy - ry)
            if d <= 1e-6:
                continue
        if d < min_goal_dist_m:
            # robot が frontier 上/直近に立っている(床観測が疎な実機で頻発)。
            # centroid 方向へ境界の先(未知側)まで押し出し、通行可能域で clip
            ux, uy = cwx - rx, cwy - ry
            n = math.hypot(ux, uy)
            if n <= 1e-6:
                continue
            pushed = _clip_along_ray(
                gmap, walkable, (rx, ry),
                (rx + ux / n * max_step_m, ry + uy / n * max_step_m),
                max_step_m)
            if pushed is None:
                continue
            gx, gy = pushed
            d = math.hypot(gx - rx, gy - ry)
            if d < min_goal_dist_m:
                continue  # 押し出しても近すぎる(すぐ塞がる方向) → 次の cluster
        if any(math.hypot(gx - float(ax), gy - float(ay)) < avoid_radius_m
               for ax, ay in avoid_xy):
            continue  # 最近断念した goal の近傍 → 次の cluster
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
