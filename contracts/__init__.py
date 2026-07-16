"""contracts — safety-critical typed schema(Phase 1 最初の contract task)。

正本:
  - GoalSpec / GoalProposal   : docs/02_TARGET_ARCHITECTURE.md §4.1-4.3(canonical)
                                + docs/06_VOICE_AIRPODS.md §8.1(音声 evidence 拡張)
  - StairModel                : docs/05_ASCENT_DESCENT_DESIGN.md §3.2-3.3
  - CommandEnvelope           : docs/08_SAFETY_TEST_EVALUATION.md §4.2
  - LocomotionCommand         : docs/02_TARGET_ARCHITECTURE.md §7
  - 停止状態の分離            : docs/CLAUDE.md invariant 7

設計規則(docs/CLAUDE.md §10):
  - typed / versioned / finite・range checked。
  - 新規依存ゼロ(stdlib のみ)。mock・replay・simulation は同じ契約を使う。
  - LLM/VLM/ASR/UI はこれらの型を生成するだけで actuator owner にならない。
"""

from contracts.errors import ContractViolation
from contracts.stop_states import StopState, TERMINAL_ACTION_FOR_NORMAL_COMPLETION
from contracts.goal_spec import (
    GoalSpec, GoalProposal, Modality, Intent, CompletionPredicate,
    ConfirmationStatus, Precondition,
)
from contracts.stair_model import StairModel, Plane, FreshCoverage, TerrainClass, StairDirection
from contracts.command_envelope import (
    CommandEnvelope, LocomotionCommand, StairGeometrySummary,
    RequestedMode, LocomotionBackend, LocomotionMode, ArbiterPriority, ServerAttribution,
)

__all__ = [
    "ContractViolation",
    "StopState", "TERMINAL_ACTION_FOR_NORMAL_COMPLETION",
    "GoalSpec", "GoalProposal", "Modality", "Intent", "CompletionPredicate",
    "ConfirmationStatus", "Precondition",
    "StairModel", "Plane", "FreshCoverage", "TerrainClass", "StairDirection",
    "CommandEnvelope", "LocomotionCommand", "StairGeometrySummary",
    "RequestedMode", "LocomotionBackend", "LocomotionMode", "ArbiterPriority",
    "ServerAttribution",
]
