"""voice_gateway.intent_parser — 限定 grammar の日本語 intent parser。

正本: docs/06 §8.2(基本変換)、§8.3(conditional HOLD と STOP_NOW の分離規則)、
      §12.2(固定 regression 文)、docs/02 §4.2(正規化表)。

置換対象: cockpit/voice.py の parse_intent(部分文字列 stop 最優先)と
m1_agent/agent_loop.py の STOP_WORDS reflex。本 parser は現行実装を変更せず、
新しい経路として追加する(段階移行 — CLAUDE.md §10)。

規則(docs/06 §8.3):
  - 単独の停止命令、または「今すぐ」「そこで」「直ちに」を伴う stop は STOP_NOW。
  - 「〜したら止まれ」「〜まで行って止まれ」は前半 skill の completion 後
    ACTIVE_HOLD(STOP_NOW ではない)。
  - 「止まらず」「止まらないで」は STOP ではない。ただし「止まらずに登れ」は
    安全制約を弱めるため task 自体を拒否する。
  - 引用・質問・否定・メタ発話は command として実行しない。
  - 「上って」+「下りて」の複合 mission は分割確認(CLARIFICATION)。
  - 訂正(いや/やっぱり)を含む発話は全体を解釈し、取り消しは CANCEL。

出力は GoalProposal(復唱確認へ)か STOP_NOW の GoalSpec(arbiter 直送)のみ。
実行可否の最終判断は Mission state / affordance validator の責務であり、
本 parser は許可を出さない(invariant 1)。
"""
import re
import unicodedata
from dataclasses import dataclass
from enum import Enum, unique
from typing import Callable, Optional

from contracts.goal_spec import (
    GoalProposal, GoalSpec, Intent, CompletionPredicate, ConfirmationStatus,
    Modality, Confidence, Confirmation, Completion, Constraints, Source,
    Target, Transcript, TranscriptEvidence, INTENT_TO_PREDICATE,
    CONFIRMATION_REQUIRED_INTENTS, SCHEMA_VERSION,
)

PARSER_VERSION = "grammar-ja-v1"

# STOP 語彙(単語として。ストップウォッチ等の false friend は先にマスクする)
_STOP_WORDS = ("止まれ", "止まって", "とまれ", "とまって", "ストップ", "停止")
_FALSE_FRIENDS = ("ストップウォッチ", "ストッパー", "バス停")
_IMMEDIATE_MARKERS = ("今すぐ", "いますぐ", "直ちに", "ただちに", "すぐに", "そこで")
_QUESTION_RE = re.compile(r"(の?[??]$|ですか$|ますか$|かな[??]?$|の$)")
_CANCEL_RE = re.compile(r"(やめて|やめる|キャンセル|中止|やっぱりいい)")
_CORRECTION_RE = re.compile(r"(いや、|いや\s|やっぱり|じゃなくて|ではなく)")
# 否定された stop(「止まらず(に)」「止まらないで」「止まるな」)
_NO_STOP_RE = re.compile(r"(止|と)まら(ず|ない)|(止|と)まるな")
# skill の禁止形(「登らないで」「下りるな」等)。曖昧な短縮 pattern は
# 「一番上なら」等への誤爆源になるため、否定形を明示列挙する
_PROHIBIT_RE = re.compile(
    r"(登らな|上らな|のぼらな|下りな|降りな|おりな|行かな|進まな|来な)"
    r"|(登|上|のぼ|下り|降り)るな")
# 探索/地図作成の禁止形。skill検出より先に拒否しないと、例えば
# 「探索しないで」の先頭語だけがEXPLORE_AND_MAPへ昇格してしまう。
_PROHIBIT_EXPLORE_RE = re.compile(
    r"(探索|探検|見回)(は|を)?(し)?(ない|なく|ません|るな)"
    r"|見(て)?回ら(ない|なく)|マッピングし(ない|なく|ません)"
    r"|(マップ|地図)を?(作ら|構築し|生成し)(ない|なく|ません)"
    r"|(探索|探検|見回|マッピング).*(やめ|不要|禁止|キャンセル)")

_ASCEND_RE = re.compile(r"(登|上|のぼ)(っ|れ|る|り)")
# ひらがな「おり」は「とおりに」等に誤一致するため活用語尾を必須にする
_DESCEND_RE = re.compile(r"(下り|降り)|おり(て|ろ)")
_NAV_STAIR_RE = re.compile(r"階段[のにへ]?(前|ところ|とこ)?(まで|に|へ)(行|来|進|移動)")
_EXPLORE_RE = re.compile(r"(探索|見て回|見回)|((マップ|地図)を?(作|構築|生成))")
_EXPLORE_ALL_RE = re.compile(r"(全部|全体|すべて|家中|部屋全部)")
_WAYPOINT_RE = re.compile(r"(戻って|戻れ|帰って|帰れ|ホームに|ホームへ|スタート地点)")


@unique
class ParseKind(Enum):
    STOP_NOW = "STOP_NOW"          # GoalSpec を arbiter 高優先入力へ直送
    PROPOSAL = "PROPOSAL"          # GoalProposal(復唱確認へ)
    CANCEL = "CANCEL"              # 提案/現 task の取り消し要求(mission 層が処理)
    CLARIFICATION = "CLARIFICATION"  # 曖昧 — 問い返しが必要
    REJECTED = "REJECTED"          # 安全上実行しない(理由つき)
    NON_COMMAND = "NON_COMMAND"    # 命令ではない(質問/引用/否定/OOD)


@dataclass(frozen=True)
class ParserContext:
    """parser を決定的・テスト可能にするための注入 context。
    id_factory は uuid 文字列を返す callable(本番は uuid.uuid4)。"""
    modality: Modality
    operator_lease_id: str
    session_id: str
    utterance_id: Optional[str]
    asr_model_id: str
    now_monotonic_ns: int
    created_at_utc: str
    id_factory: Callable[[], str]
    evidence: Optional[TranscriptEvidence] = None  # VOICE では必須
    stop_expires_ms: int = 1000


@dataclass(frozen=True)
class ParseResult:
    kind: ParseKind
    reason: str
    proposal: Optional[GoalProposal] = None
    stop_spec: Optional[GoalSpec] = None


def _normalize(text: str) -> str:
    t = unicodedata.normalize("NFKC", text).strip()
    return t


def _mask_false_friends(t: str) -> str:
    for w in _FALSE_FRIENDS:
        t = t.replace(w, "■" * len(w))
    return t


def _mask_quotes(t: str) -> str:
    # 引用内の語は command 判定に使わない(docs/06 §8.3)
    return re.sub(r"[「『][^」』]*[」』]", lambda m: "■" * len(m.group(0)), t)


def _transcript(ctx: ParserContext, text: str) -> Transcript:
    return Transcript(text=text, language="ja", model_id=ctx.asr_model_id,
                      evidence=ctx.evidence)


def _confidence(semantic: float) -> Confidence:
    # context_score は Mission FSM との整合で mission 層が再評価する(docs/06 §7.2)。
    # parser 段階では 1.0 を置く。監査 metadata であり実行判断に使わない。
    return Confidence(semantic_score=semantic, context_score=1.0,
                      parser_version=PARSER_VERSION)


def _proposal(ctx: ParserContext, raw: str, intent: Intent, target: Target,
              semantic: float = 1.0) -> ParseResult:
    prop = GoalProposal(
        schema_version=SCHEMA_VERSION,
        proposal_id=ctx.id_factory(),
        challenge_id=ctx.id_factory(),
        source=Source(modality=ctx.modality,
                      operator_lease_id=ctx.operator_lease_id,
                      session_id=ctx.session_id,
                      utterance_id=ctx.utterance_id),
        transcript=_transcript(ctx, raw),
        intent=intent,
        target=target,
        completion=Completion(predicate=INTENT_TO_PREDICATE[intent],
                              terminal_action="ACTIVE_HOLD", dwell_s=1.0),
        constraints=Constraints(),
        confidence=_confidence(semantic),
        created_monotonic_ns=ctx.now_monotonic_ns,
    )
    return ParseResult(ParseKind.PROPOSAL, "parsed: %s" % intent.name,
                       proposal=prop)


def _stop_now(ctx: ParserContext, raw: str) -> ParseResult:
    spec = GoalSpec(
        schema_version=SCHEMA_VERSION,
        goal_id=ctx.id_factory(),
        source=Source(modality=ctx.modality,
                      operator_lease_id=ctx.operator_lease_id,
                      session_id=ctx.session_id,
                      utterance_id=ctx.utterance_id),
        transcript=_transcript(ctx, raw),
        intent=Intent.STOP_NOW,
        target=None,
        completion=Completion(predicate=CompletionPredicate.IMMEDIATE,
                              terminal_action="ACTIVE_HOLD", dwell_s=1.0),
        constraints=Constraints(),
        confidence=_confidence(1.0),
        confirmation=Confirmation(required=False,
                                  status=ConfirmationStatus.NOT_REQUIRED,
                                  proposal_id=None, challenge_id=None),
        preconditions=(),
        created_at_utc=ctx.created_at_utc,
        expires_after_ms=ctx.stop_expires_ms,
    )
    # STOP_NOW は task queue / LLM / navigator を通さず arbiter 高優先入力へ直送
    # (docs/06 §8.2)。直送の配線は gateway の責務。
    return ParseResult(ParseKind.STOP_NOW, "immediate stop", stop_spec=spec)


def parse_utterance(text: str, ctx: ParserContext) -> ParseResult:
    """発話ひとつを解釈する。実行許可は出さない(invariant 1)。

    fail-closed: 解釈できない/曖昧な入力は必ず NON_COMMAND / CLARIFICATION /
    REJECTED になり、勝手に「最も近い解釈」で実行しない。
    """
    if not isinstance(text, str) or not text.strip():
        return ParseResult(ParseKind.NON_COMMAND, "空入力")
    raw = _normalize(text)
    t = _mask_quotes(_mask_false_friends(raw))

    # --- 質問・メタ発話(引用マスク後に判定) ---
    if _QUESTION_RE.search(t) or "?" in t or "?" in t:
        return ParseResult(ParseKind.NON_COMMAND, "質問/メタ発話は実行しない")

    # --- 訂正・取り消し(全体を parse し、取り消しが最後の意図なら CANCEL) ---
    if _CANCEL_RE.search(t):
        return ParseResult(ParseKind.CANCEL, "取り消し要求(mission 層で処理)")

    # --- 否定・禁止 ---
    no_stop = _NO_STOP_RE.search(t) is not None
    if no_stop:
        # 「止まらずに登れ」— 安全制約の弱体化要求は goal ごと拒否(docs/06 §12.2)
        if _ASCEND_RE.search(t) or _DESCEND_RE.search(t) or _NAV_STAIR_RE.search(t):
            return ParseResult(ParseKind.REJECTED,
                               "「止まらず」は安全制約を弱めるため task を拒否")
        return ParseResult(ParseKind.NON_COMMAND, "否定された stop は命令ではない")
    if _PROHIBIT_RE.search(t):
        return ParseResult(ParseKind.NON_COMMAND, "禁止形は task ではない(記録のみ)")
    if _PROHIBIT_EXPLORE_RE.search(t):
        return ParseResult(ParseKind.NON_COMMAND,
                           "探索/地図作成の禁止形は task ではない(記録のみ)")

    # --- skill 検出(stop 判定より先に必要 — 単独 stop かどうかを決めるため) ---
    has_ascend = _ASCEND_RE.search(t) is not None
    has_descend = _DESCEND_RE.search(t) is not None
    has_explore = _EXPLORE_RE.search(t) is not None
    has_waypoint = _WAYPOINT_RE.search(t) is not None
    has_nav_stair = _NAV_STAIR_RE.search(t) is not None
    has_skill = any((has_ascend, has_descend, has_explore, has_waypoint, has_nav_stair))

    # --- stop 判定(docs/06 §8.3) ---
    stop_pos = -1
    for w in _STOP_WORDS:
        p = t.find(w)
        if p >= 0 and (stop_pos < 0 or p < stop_pos):
            stop_pos = p
    if stop_pos >= 0:
        immediate = any(m in t for m in _IMMEDIATE_MARKERS)
        if immediate or not has_skill:
            # 単独 stop / 「今すぐ」等つき → STOP_NOW
            return _stop_now(ctx, raw)
        # skill + stop → stop は completion 条件(「〜たら止まれ」「〜て止まって」)。
        # completion.predicate は skill 側の正準対応で決まる(ACTIVE_HOLD 終端)。
        # conditional marker がなくても STOP_NOW にはしない(§8.3: STOP_NOW は
        # 単独 or immediate marker のみ)。

    # --- 複合 mission の分割確認(§8.3: 上って+下りては一括実行しない) ---
    if has_ascend and has_descend:
        return ParseResult(ParseKind.CLARIFICATION,
                           "上りと下りの複合命令は分割確認(上端 HOLD 後に別命令)")

    # --- 訂正を含む発話は全体が曖昧 — 問い返し(§8.3) ---
    if has_skill and _CORRECTION_RE.search(t):
        return ParseResult(ParseKind.CLARIFICATION, "訂正を含む発話 — 意図を確認")

    # --- intent 決定 ---
    if has_ascend:
        return _proposal(ctx, raw, Intent.ASCEND_STAIRS,
                         Target(type="stairs", ref="current", resolved_id=None))
    if has_descend:
        return _proposal(ctx, raw, Intent.DESCEND_STAIRS,
                         Target(type="stairs", ref="current", resolved_id=None))
    if has_explore:
        area = "all_reachable" if _EXPLORE_ALL_RE.search(t) else "current_room"
        return _proposal(ctx, raw, Intent.EXPLORE_AND_MAP,
                         Target(type="area", ref=area, resolved_id=None))
    if has_waypoint:
        return _proposal(ctx, raw, Intent.NAVIGATE_TO_WAYPOINT,
                         Target(type="waypoint", ref="home", resolved_id=None))
    if has_nav_stair:
        return _proposal(ctx, raw, Intent.NAVIGATE_TO_STAIR_APPROACH,
                         Target(type="stairs", ref="current", resolved_id=None))

    # 対象参照だけの発話(「前の階段ではなく右の階段」)は問い返し
    if "階段" in t:
        return ParseResult(ParseKind.CLARIFICATION,
                           "対象参照のみ — 実行する skill を確認")

    return ParseResult(ParseKind.NON_COMMAND, "未対応の発話(OOD)")
