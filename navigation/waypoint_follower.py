"""waypoint_follower — goal pose への保守的な追従制御(純関数)。

docs/12 §5: frontier explorer が提案した観測 goal pose を (vx, wz) へ変換する。
本モジュールは actuator を所有しない。出力は CommandEnvelope の材料となる
速度「提案」であり、必ず arbiter → gateway を経由して送信される(invariant 1/2)。

設計規則:
- 純関数のみ。時刻・状態を持たない(テスト容易性、docs/10 §5 の流儀)。
- ヨー整列優先: 目標方位との誤差が大きい間はその場旋回し、前進しない。
- 前方クリアランスが確保できなければ前進 0(fail-closed。unknown ≠ free)。
- クランプは mission.py の保守値(vx≤0.3)よりさらに低い既定
  VX_MAX=0.25 / WZ_MAX=0.5 とする(docs/12 §5)。
"""
from __future__ import annotations

import math
from typing import NamedTuple

# 既定パラメータ(docs/12 §5)。安全側の変更以外は根拠と test を伴うこと。
VX_MAX = 0.25          # m/s   前進上限(mission.py の 0.3 より保守的)
WZ_MAX = 0.5           # rad/s 旋回上限
ALIGN_YAW = 0.45       # rad   これを超えるヨー誤差ではその場旋回(≈26°)
ARRIVE_RADIUS = 0.15   # m     到達判定半径
K_YAW = 1.2            # ヨー誤差 → wz ゲイン
K_DIST = 0.8           # 距離 → vx ゲイン
MIN_CLEARANCE = 0.45   # m     前進に必要な前方クリアランス(機体長+余裕)


class FollowCommand(NamedTuple):
    """追従計算の結果。速度は提案値であり送信可否は上位が判断する。"""
    vx: float          # m/s (>=0。後退は提案しない)
    wz: float          # rad/s
    arrived: bool      # goal 半径内に入った
    blocked: bool      # 前方クリアランス不足で前進を止めた


# ルンバ風スムーズ操舵(2026-07-18)の候補方向: goal 方向からのオフセット [rad]。
# ±80°まで。0 が先頭(直進優先のタイブレーク)
SMOOTH_OFFSETS = (0.0, 0.3, -0.3, 0.6, -0.6, 0.95, -0.95, 1.4, -1.4)
SMOOTH_CAP_M = 1.5      # 効用計算でのクリアランス頭打ち(それ以上は同価値)
SMOOTH_TURN_PEN = 0.45  # 方向逸脱ペナルティ [m/rad](大=直進優先)


class SmoothCommand(NamedTuple):
    """スムーズ操舵の結果。heading は選ばれた進行方向(world yaw)。"""
    vx: float
    wz: float
    arrived: bool
    blocked: bool      # 全候補方向が min_clearance 未満
    heading: float
    clearance: float   # 選択方向のクリアランス(blocked 時は最も開いた方向)


def wrap_angle(a: float) -> float:
    """[-pi, pi) へ正規化。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compute_command(x: float, y: float, yaw: float,
                    gx: float, gy: float,
                    front_clearance: float = float("inf"),
                    *,
                    vx_max: float = VX_MAX,
                    wz_max: float = WZ_MAX,
                    align_yaw: float = ALIGN_YAW,
                    arrive_radius: float = ARRIVE_RADIUS,
                    min_clearance: float = MIN_CLEARANCE,
                    vx_floor: float = 0.0) -> FollowCommand:
    """現在 pose (x, y, yaw) から goal (gx, gy) への次の速度提案を返す。

    front_clearance: 上位(explore_task)が costmap から算出した進行方向の
    FREE 連続距離 [m]。未観測(unknown)はクリアランスに数えないこと。
    """
    dx, dy = gx - x, gy - y
    dist = math.hypot(dx, dy)
    if dist <= arrive_radius:
        return FollowCommand(0.0, 0.0, arrived=True, blocked=False)

    yaw_err = wrap_angle(math.atan2(dy, dx) - yaw)
    wz = max(-wz_max, min(wz_max, K_YAW * yaw_err))

    # ヨー誤差が大きい間はその場旋回(横滑りの誤マップ化と接触を避ける)
    if abs(yaw_err) > align_yaw:
        return FollowCommand(0.0, wz, arrived=False, blocked=False)

    # 前方クリアランス不足なら前進しない(fail-closed)。旋回は許す。
    if front_clearance < min_clearance:
        return FollowCommand(0.0, wz, arrived=False, blocked=True)

    # 距離とヨー誤差で減速。クリアランス残に応じても頭打ち。
    vx = min(vx_max, K_DIST * dist, max(0.0, front_clearance - min_clearance))
    vx *= max(0.0, math.cos(yaw_err))
    if 0.0 < vx < vx_floor:
        # Go2 Sport Move のデッドバンド未満は物理的に進まない(実機 2026-07-17
        # 19:55: vx=0.025 を50s送信し続け無動作→stall)。min_clearance は
        # 上で保証済みなので、実効速度まで底上げして這い進む(2026-07-18:
        # 「ブロック扱い」だと clearance 0.30-0.38 の狭室で一歩も動けない)。
        vx = vx_floor
    return FollowCommand(vx, wz, arrived=False, blocked=False)


def compute_command_smooth(x: float, y: float, yaw: float,
                           gx: float, gy: float,
                           clearances,
                           offsets=SMOOTH_OFFSETS,
                           *,
                           vx_max: float = VX_MAX,
                           wz_max: float = WZ_MAX,
                           arrive_radius: float = ARRIVE_RADIUS,
                           min_clearance: float = MIN_CLEARANCE,
                           vx_floor: float = 0.0) -> SmoothCommand:
    """ルンバ風スムーズ操舵(2026-07-18): 停止せず弧を描いて障害物を回り込む。

    clearances: offsets(goal 方向からの相対角)と同順の前方クリアランス [m]。
    呼び出し側が clearance_multi 等で測って渡す(本関数は純関数)。

    効用 = min(クリアランス, 1.5m) − 0.45×|方向逸脱|。効用最大の通行可能な
    方向へ「進みながら」旋回するため、壁に正対しても横に開いた方向へ弧で
    抜ける。全候補が min_clearance 未満のときだけ blocked(従来の停止→
    再計画へフォールバック)。"""
    dx, dy = gx - x, gy - y
    dist = math.hypot(dx, dy)
    if dist <= arrive_radius:
        return SmoothCommand(0.0, 0.0, True, False, yaw, 0.0)
    goal_dir = math.atan2(dy, dx)

    best_i, best_u = None, None
    open_i = 0
    for i, (off, c) in enumerate(zip(offsets, clearances)):
        if c > clearances[open_i]:
            open_i = i
        if c < min_clearance:
            continue
        u = min(c, SMOOTH_CAP_M) - SMOOTH_TURN_PEN * abs(off)
        if best_u is None or u > best_u:
            best_i, best_u = i, u

    if best_i is None:
        # 全方向閉塞: 最も開いた方向へその場旋回(壁に正対したまま止まらない)
        h = wrap_angle(goal_dir + offsets[open_i])
        yaw_err = wrap_angle(h - yaw)
        wz = max(-wz_max, min(wz_max, K_YAW * yaw_err))
        return SmoothCommand(0.0, wz, False, True, h, clearances[open_i])

    h = wrap_angle(goal_dir + offsets[best_i])
    c = clearances[best_i]
    yaw_err = wrap_angle(h - yaw)
    wz = max(-wz_max, min(wz_max, K_YAW * yaw_err))
    vx = min(vx_max, K_DIST * dist, max(0.0, c - min_clearance))
    vx *= max(0.0, math.cos(yaw_err))   # 大きく曲がる間は自然に減速(弧)
    if 0.0 < vx < vx_floor:
        vx = vx_floor
    return SmoothCommand(vx, wz, False, False, h, c)
