"""stair.py — LiDAR標高マップから前方の段差を幾何的に検出する。

RollingElevationMap(= M2/M3方策と同じ地図)の body系プロファイルから、
「段差のエッジ距離 / 段高 / エッジの傾き(yaw誤差) / 種別」を推定する。

種別 kind:
  none   段差なし(平坦)
  step   単段の段差(登坂対象)
  stairs 連続する階段(2段目が近くにある)
  wall   高すぎる(登坂不可 → 壁/家具)
  drop   手前に落差がある(危険 → 前進禁止)

実データ対策:
  - L1点群は外れ値(z=-14mなど)を含むため |h - 地面| > 1.2m のセルは捨てる。
  - 椅子脚のようなスパイクを段差と誤認しないよう、エッジの先に「平坦な踏面」が
    tread_min 続くことと、左右 width_min にわたってエッジが直線状に並ぶことを要求する。

単体テスト: python3 -m cockpit.stair
"""
import math

import numpy as np

# 既定パラメータ(すべてメートル/ラジアン)
CFG = {
    "corridor_hw": 0.22,   # 足元回廊の半幅(この幅の中央値でプロファイルを作る)
    "x_min": 0.15,         # プロファイル開始(ベース中心から)
    "x_max": 2.00,
    "dx": 0.05,
    "min_rise": 0.04,      # これ未満は段差とみなさない
    "tread_min": 0.18,     # エッジの先にこれだけ平坦な踏面が要る
    "tread_flat": 0.05,    # 踏面の許容凹凸(標準偏差)
    "height_win": 0.22,    # 段高を測る窓(エッジ+0.05 〜 +0.05+これ)
    "max_step": 0.30,      # これ以上は wall 扱い
    "drop": -0.07,         # これ以下の凹みは落差(危険)
    "min_cover": 0.45,     # プロファイル1点あたりの最低観測率
    "yaw_span": 0.35,      # エッジ直線あてはめに使う左右範囲
    "width_min": 0.35,     # エッジがこの幅以上まっすぐ続くこと
    "next_step_win": 0.55,  # エッジからこの距離内に次段があれば stairs
    "sane_rel": 1.20,      # |rel| がこれを超えるセルは外れ値として捨てる
}


def _body_to_world(pose, bx, by):
    x0, y0, _z, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x0 + c * bx - s * by, y0 + s * bx + c * by


def _sample(lookup, pose, bx, by):
    """body系座標配列 → 標高配列(未観測/外れ値はnan)。"""
    wx, wy = _body_to_world(pose, np.asarray(bx, float), np.asarray(by, float))
    h = np.asarray(lookup(wx, wy), float).copy()
    h[~np.isfinite(h)] = np.nan
    return h


def _ground_ref(lookup, pose, cfg):
    """足元〜真後ろの標高中央値を地面基準にする(pose_zより信頼できる)。"""
    bx = np.arange(-0.35, 0.16, 0.05)
    by = np.arange(-0.20, 0.21, 0.05)
    X, Y = np.meshgrid(bx, by, indexing="ij")
    h = _sample(lookup, pose, X.ravel(), Y.ravel())
    h = h[np.isfinite(h)]
    if h.size >= 5:
        g = float(np.median(h))
        # 明らかに変ならposeから推定した地面に戻す
        if abs(g - (pose[2] - 0.31)) < 0.5:
            return g
    return pose[2] - 0.31


def _profile(lookup, pose, ground, cfg, hw=None):
    """前方プロファイル: (xs, rel中央値, 観測率)。relは地面基準の相対高さ。"""
    hw = cfg["corridor_hw"] if hw is None else hw
    xs = np.arange(cfg["x_min"], cfg["x_max"] + 1e-9, cfg["dx"])
    ys = np.arange(-hw, hw + 1e-9, 0.05)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    h = _sample(lookup, pose, X.ravel(), Y.ravel()).reshape(X.shape) - ground
    h[np.abs(h) > cfg["sane_rel"]] = np.nan          # 外れ値除去
    cover = np.isfinite(h).mean(axis=1)
    rel = np.full(len(xs), np.nan)
    seen = cover > 0
    if seen.any():  # 全nan行に nanmedian を掛けると警告が出るので避ける
        rel[seen] = np.nanmedian(h[seen], axis=1)
    return xs, rel, cover


def _first_edge(xs, rel, cover, cfg, need_tread=True):
    """rel が min_rise を超え、その先に平坦な踏面が続く最初の index。無ければ None。"""
    dx = cfg["dx"]
    n_tread = max(2, int(round(cfg["tread_min"] / dx)))
    for i in range(len(xs)):
        if not np.isfinite(rel[i]) or rel[i] < cfg["min_rise"]:
            continue
        if cover[i] < cfg["min_cover"]:
            continue
        seg = rel[i:i + n_tread]
        if len(seg) < n_tread or not np.all(np.isfinite(seg)):
            continue
        if np.any(seg < cfg["min_rise"] * 0.75):
            continue                                   # 尖った物(椅子脚など)
        if need_tread and float(np.nanstd(seg)) > cfg["tread_flat"] * 2.0:
            continue                                   # 踏面が平坦でない = 斜面/雑物
        return i
    return None


def _edge_yaw(lookup, pose, ground, cfg, edge_x):
    """エッジ直線を左右に追い、yaw誤差(このぶん回頭すれば正対)と幅を返す。"""
    dx = cfg["dx"]
    ys = np.arange(-cfg["yaw_span"], cfg["yaw_span"] + 1e-9, 0.05)
    xs = np.arange(max(cfg["x_min"], edge_x - 0.35), edge_x + 0.36, dx)
    pts = []
    for y in ys:
        h = _sample(lookup, pose, xs, np.full_like(xs, y)) - ground
        h[np.abs(h) > cfg["sane_rel"]] = np.nan
        idx = None
        for i in range(len(xs) - 1):
            if np.isfinite(h[i]) and np.isfinite(h[i + 1]) and \
               h[i] >= cfg["min_rise"] and h[i + 1] >= cfg["min_rise"] * 0.75:
                idx = i
                break
        if idx is not None:
            pts.append((y, xs[idx]))
    if len(pts) < 5:
        return 0.0, 0.0, 0.0
    pts = np.array(pts)
    med = np.median(pts[:, 1])
    keep = pts[np.abs(pts[:, 1] - med) < 0.20]        # 直線から外れる点を捨てる
    if len(keep) < 5:
        return 0.0, 0.0, 0.0
    a, _b = np.polyfit(keep[:, 0], keep[:, 1], 1)     # x_edge = a*y + b
    width = float(keep[:, 0].max() - keep[:, 0].min())
    # 傾き a を消す回頭量: δ = -atan(a)(CCW正)
    yaw_err = float(-math.atan(a))
    fit = float(np.std(keep[:, 1] - (a * keep[:, 0] + _b)))
    return yaw_err, width, fit


def detect_stair(lookup, pose, cfg=None):
    """lookup(xs,ys)->標高配列, pose=(x,y,z,yaw) → 検出結果 dict。"""
    cfg = dict(CFG, **(cfg or {}))
    if pose is None:
        return {"kind": "none", "reason": "自己位置なし"}
    ground = _ground_ref(lookup, pose, cfg)
    xs, rel, cover = _profile(lookup, pose, ground, cfg)
    if np.isfinite(rel).sum() < 5:
        return {"kind": "none", "reason": "地図が未観測", "ground": ground}

    # 危険: 手前1.2m以内の落差
    near = xs < 1.2
    rel_near = rel[near]
    if np.isfinite(rel_near).any() and float(np.nanmin(rel_near)) <= cfg["drop"]:
        j = int(np.nanargmin(rel_near))
        return {"kind": "drop", "distance": float(xs[j]),
                "depth": float(rel_near[j]), "ground": ground,
                "reason": "%.2fm先に落差 %.2fm" % (xs[j], rel_near[j])}

    i = _first_edge(xs, rel, cover, cfg)
    if i is None:
        return {"kind": "none", "reason": "段差なし(平坦)", "ground": ground}

    edge_x = float(xs[i])
    w0 = edge_x + 0.05
    win = (xs >= w0) & (xs <= w0 + cfg["height_win"])
    hs = rel[win]
    hs = hs[np.isfinite(hs)]
    if hs.size == 0:
        return {"kind": "none", "reason": "踏面が未観測", "ground": ground}
    height = float(np.median(hs))

    yaw_err, width, fit = _edge_yaw(lookup, pose, ground, cfg, edge_x)
    conf = float(np.clip(cover[i:i + 4].mean(), 0, 1))
    if width < cfg["width_min"]:
        return {"kind": "none", "distance": edge_x, "height": height, "width": width,
                "ground": ground, "confidence": conf,
                "reason": "幅%.2fmしかない(段差ではなく物体の可能性)" % width}

    if height > cfg["max_step"]:
        kind, reason = "wall", "高さ%.2fm — 登坂対象外(壁/家具)" % height
    else:
        # 2段目があるか(踏面の先がさらに上がる)
        far = (xs > edge_x + cfg["tread_min"]) & (xs <= edge_x + cfg["next_step_win"])
        rel_far = rel[far]
        rel_far = rel_far[np.isfinite(rel_far)]
        has_next = rel_far.size > 0 and float(np.max(rel_far)) > height + cfg["min_rise"]
        kind = "stairs" if has_next else "step"
        reason = "段高%.2fm 距離%.2fm" % (height, edge_x)

    return {"kind": kind, "distance": edge_x, "height": height,
            "yaw_err": yaw_err, "width": width, "edge_fit": fit,
            "confidence": conf, "ground": ground, "reason": reason}


# ---------------- 合成地形での単体テスト ----------------

def _make_lookup(fn):
    return lambda xs, ys: fn(np.asarray(xs, float), np.asarray(ys, float))


def _selftest():
    POSE = (0.0, 0.0, 0.31, 0.0)
    rng = np.random.default_rng(0)

    def noisy(z):
        return z + rng.normal(0, 0.004, np.shape(z))

    cases = []

    # 1. 平坦
    cases.append(("平坦", _make_lookup(lambda x, y: noisy(np.zeros_like(x))), "none", None))

    # 2. 0.15m の単段 (x=0.8)
    def step15(x, y):
        return noisy(np.where(x > 0.8, 0.15, 0.0))
    cases.append(("単段0.15m@0.8m", _make_lookup(step15), "step", (0.8, 0.15)))

    # 3. 0.20m×5段の階段 (x=1.0から, 踏面0.30m)
    def stairs20(x, y):
        i = np.clip(np.floor((x - 1.0) / 0.3) + 1, 0, 5)
        return noisy(np.where(x < 1.0, 0.0, i * 0.2))
    cases.append(("階段0.20m×5@1.0m", _make_lookup(stairs20), "stairs", (1.0, 0.2)))

    # 4. 壁 (0.6m)
    def wall(x, y):
        return noisy(np.where(x > 0.9, 0.6, 0.0))
    cases.append(("壁0.6m", _make_lookup(wall), "wall", (0.9, 0.6)))

    # 5. 落差 (x>0.7 で -0.3m)
    def ledge(x, y):
        return noisy(np.where(x > 0.7, -0.3, 0.0))
    cases.append(("落差-0.3m", _make_lookup(ledge), "drop", None))

    # 6. 椅子脚(幅0.06mのスパイク4本) → 段差ではない
    def chair(x, y):
        z = np.zeros_like(x)
        for cx in (0.7, 1.0):
            for cy in (-0.15, 0.15):
                z = np.where((np.abs(x - cx) < 0.03) & (np.abs(y - cy) < 0.03), 0.45, z)
        return noisy(z)
    cases.append(("椅子脚", _make_lookup(chair), "none", None))

    # 7. 外れ値ノイズ入りの単段 (z=-14mのゴミ点)
    def step_noisy(x, y):
        z = np.where(x > 0.8, 0.12, 0.0)
        bad = (np.abs(x - 1.4) < 0.06) & (np.abs(y) < 0.06)
        return noisy(np.where(bad, -14.7, z))
    cases.append(("単段0.12m+外れ値", _make_lookup(step_noisy), "step", (0.8, 0.12)))

    # 8. 斜めのエッジ (yaw誤差 +0.2rad 相当: x_e = a*y + b, a = -tan(0.2))
    a = -math.tan(0.2)
    def skew(x, y):
        return noisy(np.where(x > 0.8 + a * y, 0.15, 0.0))
    cases.append(("斜めエッジ(+0.20rad)", _make_lookup(skew), "step", (0.8, 0.15)))

    # 9. 緩やかなスロープ(段差ではない)
    def slope(x, y):
        return noisy(np.clip((x - 0.5) * 0.12, 0, None))
    cases.append(("緩斜面", _make_lookup(slope), None, None))  # step扱いでも可

    # 10. 未観測(全nan)
    cases.append(("未観測", _make_lookup(lambda x, y: np.full_like(x, np.nan)), "none", None))

    ok = 0
    for name, lk, want, geom in cases:
        r = detect_stair(lk, POSE)
        good = (want is None) or (r["kind"] == want)
        if good and geom:
            d, h = geom
            good = abs(r.get("distance", -9) - d) <= 0.12 and abs(r.get("height", -9) - h) <= 0.04
        ok += good
        extra = ""
        if "distance" in r:
            extra = " d=%.2f h=%.3f w=%.2f yaw_err=%+.3f conf=%.2f" % (
                r["distance"], r.get("height", float("nan")), r.get("width", 0),
                r.get("yaw_err", 0), r.get("confidence", 0))
        print("%-4s %-22s -> %-6s%s  | %s" %
              ("OK" if good else "NG", name, r["kind"], extra, r["reason"]))

    # 斜めエッジのyaw_errが+0.2rad付近か
    r = detect_stair(cases[7][1], POSE)
    yerr = r.get("yaw_err", 0)
    yaw_ok = abs(yerr - 0.2) < 0.06
    ok += yaw_ok
    print("%-4s yaw_err推定: %+.3f rad (期待 +0.200)" % ("OK" if yaw_ok else "NG", yerr))

    total = len(cases) + 1
    print("\n%d/%d PASS" % (ok, total))
    return 0 if ok == total else 1


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
