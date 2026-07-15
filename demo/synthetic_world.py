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


def two_room_world() -> SyntheticWorld:
    """2部屋(各 3m×4m)をドア幅 1.0m で接続した世界。
    robot は部屋A(x<0)から開始し、探索で部屋Bまで地図化できる。"""
    return SyntheticWorld([
        # 外周(-3,-2)-(3,2)
        (-3, -2, 3, -2), (3, -2, 3, 2), (3, 2, -3, 2), (-3, 2, -3, -2),
        # 仕切り壁 x=0(ドア: y ∈ [-0.5, 0.5])
        (0, -2, 0, -0.5), (0, 0.5, 0, 2),
    ])
