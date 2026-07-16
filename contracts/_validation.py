"""contracts._validation — 共通検証ヘルパ(stdlib のみ)。

方針:
  - すべて fail-closed。型・範囲・有限性(NaN/Inf 拒否)を必ず検査する。
  - dict 入力は「未知キー拒否」を既定にする。これにより `source_priority` や
    `max_top_hold_s` のような禁止 field の注入(docs/08 §4.2, docs/02 §4.3)を
    schema 層で機械的に弾く。
  - 正規表現の終端は `\\Z`(絶対末尾)を使う。`$` は末尾改行を許容するため
    strict parser では使わない。
"""
import math
import re
from enum import Enum
from typing import Any, Mapping, Sequence, Type, TypeVar

from contracts.errors import ContractViolation

E = TypeVar("E", bound=Enum)

# 識別子 token(stair_id, frame_id, source_id, phase 等)
_TOKEN_RE = re.compile(r"^[A-Za-z0-9_\-]{1,64}\Z")
# policy_hash: sha256 hex を明示 prefix 付きで。Branch S 等ハッシュ非適用は明示 token。
_POLICY_HASH_RE = re.compile(r"^(sha256:[0-9a-f]{64}|not_applicable)\Z")
# created_at_utc(監査用 wall clock。安全判定には使わない — docs/02 §4.1)
_UTC_RE = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d{1,6})?Z\Z")
# UUID は canonical 形式(小文字・ハイフン付き)のみ受理する。
# uuid.UUID() の寛容パース(urn:uuid:/波括弧/ハイフンなし/大文字)は、同一 UUID の
# 複数表記を別文字列として通し、dedup key(docs/06 §4.2)を迂回できるため使わない。
_UUID_CANONICAL_RE = re.compile(
    r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}\Z")


def fail(path: str, reason: str) -> None:
    raise ContractViolation(path, reason)


def req_mapping(v: Any, path: str) -> Mapping:
    if not isinstance(v, Mapping):
        fail(path, "object(mapping) が必要: %r" % type(v).__name__)
    return v


def no_unknown_keys(d: Mapping, allowed: Sequence[str], path: str) -> None:
    unknown = set(d.keys()) - set(allowed)
    if unknown:
        fail(path, "未知キーを拒否(fail-closed): %s" % sorted(unknown))


def req_keys(d: Mapping, required: Sequence[str], path: str) -> None:
    missing = [k for k in required if k not in d]
    if missing:
        fail(path, "必須キー欠落: %s" % missing)


def req_bool(v: Any, path: str) -> bool:
    if not isinstance(v, bool):
        fail(path, "bool が必要: %r" % (v,))
    return v


def req_int(v: Any, path: str, lo: int = None, hi: int = None) -> int:
    # bool は int の subclass なので明示拒否
    if isinstance(v, bool) or not isinstance(v, int):
        fail(path, "int が必要: %r" % (v,))
    if lo is not None and v < lo:
        fail(path, "範囲外(%d < %d)" % (v, lo))
    if hi is not None and v > hi:
        fail(path, "範囲外(%d > %d)" % (v, hi))
    return v


def req_finite(v: Any, path: str, lo: float = None, hi: float = None) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        fail(path, "有限の数値が必要: %r" % (v,))
    f = float(v)
    if not math.isfinite(f):
        fail(path, "NaN/Inf を拒否(fail-closed)")
    if lo is not None and f < lo:
        fail(path, "範囲外(%g < %g)" % (f, lo))
    if hi is not None and f > hi:
        fail(path, "範囲外(%g > %g)" % (f, hi))
    return f


def req_str(v: Any, path: str, max_len: int = 256, allow_empty: bool = False) -> str:
    if not isinstance(v, str):
        fail(path, "str が必要: %r" % (v,))
    if not allow_empty and not v:
        fail(path, "空文字列を拒否")
    if len(v) > max_len:
        fail(path, "長さ上限超過(%d > %d)" % (len(v), max_len))
    return v


def req_token(v: Any, path: str) -> str:
    s = req_str(v, path, max_len=64)
    if not _TOKEN_RE.match(s):
        fail(path, "識別子 token 形式(A-Za-z0-9_-、64文字以内)が必要: %r" % (s,))
    return s


def req_uuid(v: Any, path: str) -> str:
    s = req_str(v, path, max_len=36)
    if not _UUID_CANONICAL_RE.match(s):
        fail(path, "canonical UUID(小文字・ハイフン付き)が必要: %r" % (s,))
    return s


def opt_uuid(v: Any, path: str):
    if v is None:
        return None
    return req_uuid(v, path)


def req_enum(v: Any, enum_cls: Type[E], path: str) -> E:
    if isinstance(v, enum_cls):
        return v
    if isinstance(v, str):
        try:
            return enum_cls[v]
        except KeyError:
            pass
    fail(path, "%s のいずれかが必要: %r(許容: %s)"
         % (enum_cls.__name__, v, [m.name for m in enum_cls]))


def req_enum_member(v: Any, enum_cls: Type[E], path: str) -> E:
    """validate() 用: 既に enum member であることを要求(文字列を許容しない)。

    直接コンストラクタで文字列 "VOICE" 等を入れた場合の検証迂回
    (str は `is Modality.VOICE` に決して一致せず分岐をすり抜ける)を防ぐ。"""
    if not isinstance(v, enum_cls):
        fail(path, "%s enum が必要(文字列不可): %r" % (enum_cls.__name__, v))
    return v


def req_policy_hash(v: Any, path: str) -> str:
    s = req_str(v, path, max_len=80)
    if not _POLICY_HASH_RE.match(s):
        fail(path, "policy_hash は 'sha256:<64hex>' か 'not_applicable': %r" % (s,))
    return s


def req_utc(v: Any, path: str) -> str:
    s = req_str(v, path, max_len=32)
    if not _UTC_RE.match(s):
        fail(path, "UTC ISO8601(...Z)形式が必要: %r" % (s,))
    return s


def req_float_list(v: Any, path: str, n: int = None,
                   lo: float = None, hi: float = None) -> list:
    if not isinstance(v, (list, tuple)):
        fail(path, "配列が必要: %r" % (v,))
    if n is not None and len(v) != n:
        fail(path, "要素数不一致(%d != %d)" % (len(v), n))
    return [req_finite(x, "%s[%d]" % (path, i), lo, hi) for i, x in enumerate(v)]


def req_score(v: Any, path: str) -> float:
    return req_finite(v, path, 0.0, 1.0)
