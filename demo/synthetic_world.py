"""demo.synthetic_world — 壁線分の合成世界と ray cast LiDAR(決定的)。"""
import math
from typing import List, Sequence, Tuple

Segment = Tuple[float, float, float, float]  # (x1, y1, x2, y2)


class SyntheticWorld:
    def __init__(self, segments: Sequence[Segment]):
        self.segments = [tuple(float(v) for v in s) for s in segments]

    def _ray_hit(self, ox: float, oy: float, dx: float, dy: float,
                 max_range: float) -> float:
        """origin から方向 (dx,dy) の最近接壁までの距離(なければ inf)。"""
        best = math.inf
        for (x1, y1, x2, y2) in self.segments:
            ex, ey = x2 - x1, y2 - y1
            denom = dx * ey - dy * ex
            if abs(denom) < 1e-12:
                continue
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / denom      # ray param
            u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom      # segment param
            if t > 1e-9 and 0.0 <= u <= 1.0 and t < best:
                best = t
        return best if best <= max_range else math.inf

    def scan(self, robot_xy, n_rays: int = 72,
             max_range: float = 8.0) -> List[Tuple[float, float]]:
        """全周 scan。hit は壁上の点、miss は max_range*1.5 の点を返す
        (GlobalOccupancyMap.integrate_scan が miss を free-ray として扱うため)。"""
        ox, oy = float(robot_xy[0]), float(robot_xy[1])
        pts = []
        for i in range(n_rays):
            a = 2.0 * math.pi * i / n_rays
            dx, dy = math.cos(a), math.sin(a)
            d = self._ray_hit(ox, oy, dx, dy, max_range)
            r = d if math.isfinite(d) else max_range * 1.5
            pts.append((ox + dx * r, oy + dy * r))
        return pts

    def scan_with_hits(self, robot_xy, n_rays: int = 72,
                       max_range: float = 3.0):
        """beam endpointとhit maskを返す。no-returnはmax_range端までFREE。"""
        ox, oy = float(robot_xy[0]), float(robot_xy[1])
        pts, hits = [], []
        for i in range(n_rays):
            a = 2.0 * math.pi * i / n_rays
            dx, dy = math.cos(a), math.sin(a)
            d = self._ray_hit(ox, oy, dx, dy, max_range)
            hit = math.isfinite(d)
            r = d if hit else max_range
            pts.append((ox + dx * r, oy + dy * r))
            hits.append(hit)
        return pts, hits

    @staticmethod
    def _point_segment_distance(px, py, x1, y1, x2, y2):
        dx, dy = x2 - x1, y2 - y1
        denom = dx * dx + dy * dy
        if denom <= 1e-18:
            return math.hypot(px - x1, py - y1)
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / denom))
        return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))

    @staticmethod
    def _segments_intersect(a, b, c, d):
        eps = 1e-12

        def orient(p, q, r):
            return ((q[0] - p[0]) * (r[1] - p[1])
                    - (q[1] - p[1]) * (r[0] - p[0]))

        def on_segment(p, q, r):
            # qがcollinearなp-r線分のbounding box内にあるか。
            return (min(p[0], r[0]) - eps <= q[0] <= max(p[0], r[0]) + eps
                    and min(p[1], r[1]) - eps <= q[1] <= max(p[1], r[1]) + eps)

        o1, o2 = orient(a, b, c), orient(a, b, d)
        o3, o4 = orient(c, d, a), orient(c, d, b)
        if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) \
                and ((o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)):
            return True
        return ((abs(o1) <= eps and on_segment(a, c, b))
                or (abs(o2) <= eps and on_segment(a, d, b))
                or (abs(o3) <= eps and on_segment(c, a, d))
                or (abs(o4) <= eps and on_segment(c, b, d)))

    def motion_clearance(self, start_xy, end_xy) -> float:
        """移動線分と最も近い壁線分の距離。cross時は0。"""
        a, b = tuple(start_xy[:2]), tuple(end_xy[:2])
        best = math.inf
        for x1, y1, x2, y2 in self.segments:
            c, d = (x1, y1), (x2, y2)
            if self._segments_intersect(a, b, c, d):
                return 0.0
            best = min(
                best,
                self._point_segment_distance(a[0], a[1], x1, y1, x2, y2),
                self._point_segment_distance(b[0], b[1], x1, y1, x2, y2),
                self._point_segment_distance(x1, y1, a[0], a[1], b[0], b[1]),
                self._point_segment_distance(x2, y2, a[0], a[1], b[0], b[1]),
            )
        return best

    def motion_collides(self, start_xy, end_xy, robot_radius: float = 0.25) -> bool:
        return self.motion_clearance(start_xy, end_xy) <= robot_radius


def two_room_world() -> SyntheticWorld:
    """2部屋(各 3m×4m)をドア幅 1.0m で接続した世界。
    robot は部屋A(x<0)から開始し、探索で部屋Bまで地図化できる。"""
    return SyntheticWorld([
        # 外周(-3,-2)-(3,2)
        (-3, -2, 3, -2), (3, -2, 3, 2), (3, 2, -3, 2), (-3, 2, -3, -2),
        # 仕切り壁 x=0(ドア: y ∈ [-0.5, 0.5])
        (0, -2, 0, -0.5), (0, 0.5, 0, 2),
    ])
