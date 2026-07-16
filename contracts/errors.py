"""contracts.errors — 契約違反の例外型。

safety-critical schema の検証失敗は必ずこの例外で fail-closed にする。
握りつぶし(except: pass)は Gate 0 の No-Go 条件(docs/08 §9)。
"""


class ContractViolation(ValueError):
    """schema/契約違反。field パスと理由を必ず持つ。"""

    def __init__(self, path: str, reason: str):
        self.path = path
        self.reason = reason
        super().__init__("%s: %s" % (path, reason))
