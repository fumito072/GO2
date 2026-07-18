"""demo.explore_e2e — offline E2E: 音声/テキスト → 自律探索 → マップ構築。

実行: python -m demo.explore_e2e
robot 非接続・決定的(時刻/ID は counter 注入)。経路は本番と同一:
  発話 → intent_parser → GoalProposal → (復唱確認=CONFIRMED GoalSpec 発行)
  → MissionExecutive(EXPLORING) → frontier_explorer → CommandEnvelope
  → CommandArbiter → ExclusiveActuationGateway → 合成世界 kinematic sim
  → GlobalOccupancyMap 更新 → EXPLORATION_COMPLETE → ACTIVE_HOLD

STOP_NOW(「止まれ」)は途中で注入でき、arbiter latch により即停止する。
"""
import math
import uuid
from dataclasses import dataclass
from typing import Optional

import numpy as np

from contracts.command_envelope import (
    ArbiterPriority, CommandEnvelope, LocomotionBackend, RequestedMode,
    ServerAttribution,
)
from contracts.goal_spec import (
    Confirmation, ConfirmationStatus, GoalProposal, GoalSpec, Intent,
    Modality, Precondition, TranscriptEvidence, SCHEMA_VERSION,
)
from contracts.stop_states import StopState
from mission.command_arbiter import CommandArbiter, DirectiveKind
from mission.executive import AffordanceContext, MissionExecutive, MissionState
from navigation.exploration_controller import (
    ControlStatus, ExplorationController, ExplorationControllerConfig,
)
from perception.global_map import GlobalOccupancyMap, FREE
from realtime.exclusive_actuation_gateway import (
    ActionKind, Channel, ExclusiveActuationGateway, RobotStatusFlags,
    RunManifestControl,
)
from voice_gateway.intent_parser import ParseKind, ParserContext, parse_utterance
from demo.synthetic_world import SyntheticWorld, two_room_world

MS = 1_000_000
CREATED_AT = "2026-07-15T12:00:00Z"


class _Ids:
    """決定的 UUID factory(canonical 小文字形式)。"""

    def __init__(self):
        self._n = 0

    def __call__(self) -> str:
        self._n += 1
        return str(uuid.UUID(int=self._n))


@dataclass
class E2EResult:
    completed: bool
    stopped_by_operator: bool
    final_mission_state: MissionState
    steps: int
    map_counts: dict
    room_b_mapped: bool
    room_b_entered: bool
    robot_xy: tuple
    narrative: list
    collision_count: int
    coverage_ratio: float
    unique_visited_cells: int
    revisit_ratio: float


def _confirm_proposal(prop: GoalProposal, ids: _Ids) -> GoalSpec:
    """復唱確認後の CONFIRMED GoalSpec 発行(docs/02 §4.1)。
    demo では operator の確認応答を即時成立とみなす。"""
    return GoalSpec(
        schema_version=SCHEMA_VERSION,
        goal_id=ids(),
        source=prop.source,
        transcript=prop.transcript,
        intent=prop.intent,
        target=prop.target,
        completion=prop.completion,
        constraints=prop.constraints,
        confidence=prop.confidence,
        confirmation=Confirmation(required=True,
                                  status=ConfirmationStatus.CONFIRMED,
                                  proposal_id=prop.proposal_id,
                                  challenge_id=prop.challenge_id),
        preconditions=(Precondition.OPERATOR_LEASE_VALID,
                       Precondition.ROBOT_ARMED,
                       Precondition.ACTIVE_STAIR_GEOMETRY_VALID,
                       Precondition.SAFETY_SUPERVISOR_OK),
        created_at_utc=CREATED_AT,
        expires_after_ms=5000,
    )


def run_e2e(utterance: str = "部屋を探索してマップを作って",
            world: Optional[SyntheticWorld] = None,
            start_xy=(-1.5, 0.0),
            stop_after_steps: Optional[int] = None,
            max_steps: int = 600,
            speed_mps: float = 0.25,
            dt_s: float = 0.2) -> E2EResult:
    world = world or two_room_world()
    ids = _Ids()
    lease, session, utt = ids(), ids(), ids()
    now = [1_000]

    def tick_ns(advance_ms=0):
        now[0] += advance_ms * MS + 1
        return now[0]

    narrative = []

    # --- 音声 → GoalProposal ---
    pctx = ParserContext(
        modality=Modality.VOICE, operator_lease_id=lease, session_id=session,
        utterance_id=utt, asr_model_id="faster-whisper-sim",
        now_monotonic_ns=tick_ns(), created_at_utc=CREATED_AT, id_factory=ids,
        evidence=TranscriptEvidence(asr_quality_score=0.97,
                                    no_speech_probability=0.01))
    parsed = parse_utterance(utterance, pctx)
    if parsed.kind is not ParseKind.PROPOSAL:
        raise RuntimeError("E2E: 発話が PROPOSAL にならない: %s(%s)"
                           % (parsed.kind, parsed.reason))
    narrative.append("発話「%s」→ %s の提案" % (utterance, parsed.proposal.intent.name))

    # --- 復唱確認 → CONFIRMED GoalSpec → Mission FSM ---
    spec = _confirm_proposal(parsed.proposal, ids)
    execu = MissionExecutive(expected_operator_lease_id=lease)
    execu.arm(self_check_ok=True, now_ns=tick_ns())
    ctx = AffordanceContext(operator_lease_valid=True, supervisor_ok=True)
    dec = execu.accept_goal(spec, ctx, tick_ns())
    if not dec.accepted:
        raise RuntimeError("E2E: goal 拒否: %s" % dec.reason)
    narrative.append("確認済み goal 受理 → %s" % execu.state.name)

    # --- 安全基盤(arbiter + gateway)と COMMON_NAV authority ---
    arb = CommandArbiter()
    manifest = RunManifestControl(
        run_id="e2e_explore", selected_backend=LocomotionBackend.SPORT_STAIR_API,
        policy_hash="not_applicable", operator_lease_id=lease)
    gw = ExclusiveActuationGateway(manifest, arb)
    t = tick_ns()
    gw.request_channel(Channel.COMMON_NAV, t)
    gw.ack_inactive(Channel.COMMON_NAV, tick_ns())
    gw.assign_generation(Channel.COMMON_NAV, tick_ns())
    gw.enable(Channel.COMMON_NAV, tick_ns())

    # --- 探索 loop(kinematic sim) ---
    gmap = GlobalOccupancyMap(size_m=(16.0, 12.0), resolution_m=0.1,
                              origin_xy=(-8.0, -6.0), map_id="e2e_map")
    gmap.set_waypoint("home", (start_xy[0], start_xy[1], 0.0))
    pos = [float(start_xy[0]), float(start_xy[1])]
    yaw = 0.0
    controller = ExplorationController(
        gmap,
        ExplorationControllerConfig(
            max_speed_mps=speed_mps,
            max_yaw_rate_rps=0.5,
            inflation_radius_m=0.30,
            max_goal_step_m=2.0,
            frontier_standoff_m=0.20,
            free_max_age_s=120.0,
            progress_timeout_s=3.0,
        ))
    flags = RobotStatusFlags(settled_below_thresholds=True,
                             stable_contact_verified=True)
    seq = 0
    completed = False
    stopped = False
    steps = 0
    collision_count = 0
    entered_room_b = False

    for step in range(max_steps):
        steps = step + 1
        t = tick_ns(int(dt_s * 1000))
        points, hits = world.scan_with_hits(pos, n_rays=120, max_range=2.2)
        controller.integrate_planar_scan(tuple(pos), points, now_ns=t,
                                         max_range_m=2.2, hit_mask=hits)

        # 操作者の途中停止(試験注入)
        if stop_after_steps is not None and step == stop_after_steps:
            sctx = ParserContext(
                modality=Modality.VOICE, operator_lease_id=lease,
                session_id=session, utterance_id=ids(),
                asr_model_id="faster-whisper-sim", now_monotonic_ns=t,
                created_at_utc=CREATED_AT, id_factory=ids,
                evidence=TranscriptEvidence(asr_quality_score=0.97,
                                            no_speech_probability=0.01))
            sres = parse_utterance("止まれ", sctx)
            assert sres.kind is ParseKind.STOP_NOW
            seq += 1
            stop_env = CommandEnvelope(
                schema_version="1.0", source_id="operator_voice",
                goal_id=sres.stop_spec.goal_id, actuation_request_id=ids(),
                sender_timestamp=t, sequence=seq,
                expires_after_ms=sres.stop_spec.expires_after_ms,
                requested_mode=RequestedMode.STOP_NOW,
                vx=0.0, vy=0.0, wz=0.0, phase="EXPLORE",
                policy_hash="not_applicable")
            arb.submit(stop_env, ServerAttribution(
                trusted_source_id="operator_voice",
                priority=ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                accepted_monotonic_timestamp_ns=t), t)
            execu.notify_stop_now(t)
            stopped = True
            narrative.append("step %d: 「止まれ」→ STOP_NOW latch" % step)

        if not stopped and execu.state is MissionState.EXPLORING:
            ctl = controller.step((pos[0], pos[1], 0.31, yaw), t)
            if ctl.status is ControlStatus.COMPLETE:
                execu.notify_completion(spec.completion.predicate, tick_ns())
                completed = True
                narrative.append("step %d: frontier 枯渇を安定確認 → %s"
                                 % (step, execu.state.name))
            elif ctl.status in (ControlStatus.STOP_REPLAN,
                                ControlStatus.STOP_SENSOR):
                narrative.append("step %d: safety stop: %s" % (step, ctl.reason))
            seq += 1
            env = CommandEnvelope(
                schema_version="1.0", source_id="frontier_explorer",
                goal_id=spec.goal_id, actuation_request_id=ids(),
                sender_timestamp=t, sequence=seq, expires_after_ms=300,
                requested_mode=RequestedMode.COMMON_NAV,
                vx=round(ctl.vx, 4), vy=round(ctl.vy, 4), wz=round(ctl.wz, 4),
                phase="EXPLORE", policy_hash="not_applicable")
            arb.submit(env, ServerAttribution(
                trusted_source_id="frontier_explorer",
                priority=ArbiterPriority.NAV_LOCAL_PLANNER,
                accepted_monotonic_timestamp_ns=t), t)

        act = gw.tick(flags, tick_ns())
        if act.kind is ActionKind.FORWARD:
            # Sport Moveと同じbody-frame commandをyaw込みで積分する。
            old = tuple(pos)
            yaw_mid = yaw + 0.5 * act.envelope.wz * dt_s
            yaw = (yaw + act.envelope.wz * dt_s + math.pi) % (2 * math.pi) - math.pi
            nx = pos[0] + (act.envelope.vx * math.cos(yaw_mid)
                           - act.envelope.vy * math.sin(yaw_mid)) * dt_s
            ny = pos[1] + (act.envelope.vx * math.sin(yaw_mid)
                           + act.envelope.vy * math.cos(yaw_mid)) * dt_s
            if world.motion_collides(old, (nx, ny), robot_radius=0.22):
                collision_count += 1
                # simulatorは壁を通さない。count>0ならE2E testを失敗させる。
            else:
                pos[0], pos[1] = nx, ny
        entered_room_b = entered_room_b or pos[0] > 0.35
        if completed:
            break

    room_b = gmap.world_to_cell(1.5, 0.0)
    room_b_mapped = bool(gmap.grid[room_b[1], room_b[0]] == FREE)
    # 外周壁内を対象areaとし、map全体の未知領域でcoverageを薄めない。
    x0, y0 = gmap.world_to_cell(-2.8, -1.8)
    x1, y1 = gmap.world_to_cell(2.8, 1.8)
    roi = gmap.grid[y0:y1 + 1, x0:x1 + 1]
    coverage = float(np.mean(roi != 0))
    metrics = controller.metrics()
    return E2EResult(
        completed=completed, stopped_by_operator=stopped,
        final_mission_state=execu.state, steps=steps,
        map_counts=gmap.counts(), room_b_mapped=room_b_mapped,
        room_b_entered=entered_room_b,
        robot_xy=(round(pos[0], 3), round(pos[1], 3)), narrative=narrative,
        collision_count=collision_count, coverage_ratio=coverage,
        unique_visited_cells=metrics["unique_cells"],
        revisit_ratio=metrics["revisit_ratio"])


def _ascii_map(gmap: GlobalOccupancyMap, every: int = 4) -> str:
    rows = []
    for iy in range(gmap.height - 1, -1, -every):
        row = []
        for ix in range(0, gmap.width, every):
            v = gmap.grid[iy, ix]
            row.append("#" if v == 2 else ("." if v == 1 else " "))
        rows.append("".join(row))
    return "\n".join(rows)


def main():
    r = run_e2e()
    for line in r.narrative:
        print("  " + line)
    print("completed=%s state=%s steps=%d pos=%s" %
          (r.completed, r.final_mission_state.name, r.steps, r.robot_xy))
    print("map: %s / room_b_mapped=%s" % (r.map_counts, r.room_b_mapped))
    print("coverage=%.1f%% collisions=%d unique=%d revisit=%.3f" %
          (100 * r.coverage_ratio, r.collision_count,
           r.unique_visited_cells, r.revisit_ratio))


if __name__ == "__main__":
    main()
