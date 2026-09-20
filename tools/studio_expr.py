"""
Vibe Settings expressions — the STRICT little language behind
`@studio control … show="…"` and `@studio require expr="…" when="…"`.

Driver authors write expressions in a header comment; nothing downstream ever
interprets that string. The manifest generator parses it HERE into a JSON AST,
resolves every name against the driver's own functions and enums, and fails the
build on anything it cannot resolve. Studio only ever evaluates the AST.

Grammar (C-like, deliberately small):

    expr    := or [ '?' expr ':' expr ]
    or      := and { '||' and }
    and     := cmp { '&&' cmp }
    cmp     := sum [ ('<' | '<=' | '>' | '>=' | '==' | '!=') sum ]
    sum     := term { ('+' | '-') term }
    term    := unary { ('*' | '/') unary }
    unary   := '!' unary | '-' unary | atom
    atom    := NUMBER | '(' expr ')' | NAME '(' expr { ',' expr } ')' | ref | MEMBER
    ref     := NAME            — an argument of THIS function
             | NAME '.' NAME   — <function>.<argument> on the same tile

    functions: min(a, b, …)  max(a, b, …)  value(ref)

A bare ref is the argument's raw (register) value. `value(ref)` is the physical
quantity of the enum member currently selected for it (`@studio value=100` in the
member's doc comment) — so `value(set_accel_odr.odr)` is the data rate in Hz.

AST nodes (all plain JSON):
    {"num": 1.5}
    {"ref": "set_accel_odr.odr"}            always fully qualified
    {"member": "SENSE_…_LN", "num": 3}      an enum constant, already a number
    {"value": "set_accel_odr.odr"}
    {"op": "<=", "args": [a, b]}            + - * / < <= > >= == != && ||
    {"not": a}   {"neg": a}
    {"call": "max", "args": [...]}
    {"cond": [test, then, else]}
"""

import re

_TOKEN_RE = re.compile(
    r"\s*(?:(\d+\.\d+|\d+|0[xX][0-9A-Fa-f]+)|([A-Za-z_]\w*)|(\|\||&&|<=|>=|==|!=|[-+*/<>!?:(),.]))"
)
FUNCTIONS = {"min", "max", "value"}


class ExprError(ValueError):
    pass


def tokenize(text):
    pos, out = 0, []
    text = text.strip()
    while pos < len(text):
        m = _TOKEN_RE.match(text, pos)
        if not m or m.end() == pos:
            raise ExprError(f"unexpected character at {pos}: {text[pos:pos + 8]!r}")
        num, name, op = m.groups()
        if num is not None:
            out.append(("num", float(num) if "." in num else int(num, 0)))
        elif name is not None:
            out.append(("name", name))
        else:
            out.append(("op", op))
        pos = m.end()
    return out


class _Parser:
    def __init__(self, tokens, resolve_ref, resolve_member):
        self.t, self.i = tokens, 0
        self.resolve_ref, self.resolve_member = resolve_ref, resolve_member

    def peek(self):
        return self.t[self.i] if self.i < len(self.t) else (None, None)

    def take(self, kind=None, val=None):
        k, v = self.peek()
        if k is None or (kind and k != kind) or (val is not None and v != val):
            raise ExprError(f"expected {val or kind}, found {v!r}")
        self.i += 1
        return v

    def at(self, *ops):
        k, v = self.peek()
        return k == "op" and v in ops

    def expr(self):
        test = self.binary(0)
        if self.at("?"):
            self.take()
            then = self.expr()
            self.take("op", ":")
            return {"cond": [test, then, self.expr()]}
        return test

    LEVELS = [("||",), ("&&",), ("<", "<=", ">", ">=", "==", "!="), ("+", "-"), ("*", "/")]

    def binary(self, level):
        if level == len(self.LEVELS):
            return self.unary()
        left = self.binary(level + 1)
        while self.at(*self.LEVELS[level]):
            op = self.take()
            left = {"op": op, "args": [left, self.binary(level + 1)]}
            if level == 2:  # comparisons don't chain
                break
        return left

    def unary(self):
        if self.at("!"):
            self.take()
            return {"not": self.unary()}
        if self.at("-"):
            self.take()
            return {"neg": self.unary()}
        return self.atom()

    def ref(self):
        name = self.take("name")
        if self.at("."):
            self.take()
            name = f"{name}.{self.take('name')}"
        return name

    def atom(self):
        k, v = self.peek()
        if k == "num":
            self.take()
            return {"num": v}
        if self.at("("):
            self.take()
            inner = self.expr()
            self.take("op", ")")
            return inner
        if k != "name":
            raise ExprError(f"unexpected {v!r}")
        if v in FUNCTIONS and self.i + 1 < len(self.t) and self.t[self.i + 1] == ("op", "("):
            self.take()
            self.take("op", "(")
            if v == "value":
                target = self.resolve_ref(self.ref())
                self.take("op", ")")
                return {"value": target}
            args = [self.expr()]
            while self.at(","):
                self.take()
                args.append(self.expr())
            self.take("op", ")")
            return {"call": v, "args": args}
        member = self.resolve_member(v) if not (
            self.i + 1 < len(self.t) and self.t[self.i + 1] == ("op", ".")
        ) else None
        if member is not None:
            self.take()
            return {"member": v, "num": member}
        return {"ref": self.resolve_ref(self.ref())}


def parse(text, resolve_ref, resolve_member):
    """Parse `text` → AST.

    resolve_ref(name)    → the fully qualified "<function>.<argument>", or raise ExprError
    resolve_member(name) → the enum constant's integer value, or None if not a member
    """
    p = _Parser(tokenize(text), resolve_ref, resolve_member)
    ast = p.expr()
    if p.i != len(p.t):
        raise ExprError(f"unexpected trailing {p.peek()[1]!r}")
    return ast


def refs_of(ast):
    """Every fully qualified ref an AST reads (bare or through value())."""
    out = set()

    def walk(n):
        if isinstance(n, dict):
            if "ref" in n:
                out.add(n["ref"])
            if "value" in n:
                out.add(n["value"])
            for v in n.values():
                walk(v)
        elif isinstance(n, list):
            for v in n:
                walk(v)

    walk(ast)
    return out
