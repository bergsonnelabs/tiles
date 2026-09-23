#!/usr/bin/env python3
"""Fingerprint of what the build server COMPILES from a tiles commit.

The build server (web/services/build) bakes a tiles commit into its image and
compiles every Studio project against it. Redeploying it is only worth doing
when that commit would build something different. This fingerprint answers
that: same fingerprint, same firmware for any project; a new one, redeploy.

Inputs are what `make` and coregen read: the Makefile, sdk/, hal/, drivers/,
the Core definitions (a peripheral's definition is never read by the build),
third_party submodule pins, tools/coregen/, tools/gen_studio_natives.py (coregen
imports it) and manifests/sdk-docs/ (coregen reads them for the WAMR natives).

What does NOT count: comments and whitespace in C, docstrings in Python, prose
fields in JSON (descriptions, briefs, labels, timestamps), and documentation
files. It errs toward "changed": a new declaration counts even if no binary
changes, because a project may start calling it.

  python3 tools/build_fingerprint.py            # HEAD
  python3 tools/build_fingerprint.py REF [REF…] # several commits
"""
import ast
import hashlib
import io
import json
import re
import subprocess
import sys
import tokenize

INPUTS = re.compile(
    r'^(Makefile|tiles[^/]*\.h|sdk/|hal/|drivers/|definitions/Core-|third_party/|'
    r'tools/coregen/|tools/gen_studio_natives\.py|manifests/sdk-docs/)'
)
SKIP = re.compile(r'\.(md|txt|pdf|png|jpe?g|svg|gif|rst|html)$|/(docs?|examples?|tests?)/', re.I)
PROSE = {
    'description', 'brief', 'label', 'doc', 'details', 'notes', 'note', 'source',
    'headline', 'summary', 'title', 'features', 'twin', 'long_description',
    'updated_at', 'created_at', 'json_version', 'application_notes',
}

_C_TOKEN = re.compile(
    r'//[^\n]*'                    # line comment
    r'|/\*.*?\*/'                  # block comment
    r'|"(?:\\.|[^"\\\n])*"'        # string literal (kept)
    r"|'(?:\\.|[^'\\\n])*'",       # char literal (kept)
    re.S,
)


def strip_c(src: bytes) -> bytes:
    """C/C++ with comments removed and whitespace collapsed. Strings and char
    literals are kept verbatim, so a '//' inside a string is not a comment."""
    text = src.decode('utf-8', errors='replace').replace('\\\r\n', '').replace('\\\n', '')
    text = _C_TOKEN.sub(lambda m: m.group(0) if m.group(0)[0] in '"\'' else ' ', text)
    lines = (re.sub(r'\s+', ' ', line).strip() for line in text.splitlines())
    return '\n'.join(line for line in lines if line).encode()


def strip_py(src: bytes) -> bytes:
    """Python without comments, docstrings or layout, as TEXT, so the result is
    the same on every Python 3 (an AST dump is not: its format changes between
    versions, which once made CI and a laptop disagree about the same commit)."""
    text = src.decode('utf-8', errors='replace')
    lines = text.splitlines()
    # Docstrings: the first statement of a module / class / function, if it is a
    # bare string. They sit on lines of their own, so blank those lines.
    for node in ast.walk(ast.parse(text)):
        body = getattr(node, 'body', None)
        if (isinstance(body, list) and body and isinstance(body[0], ast.Expr)
                and isinstance(getattr(body[0], 'value', None), ast.Constant)
                and isinstance(body[0].value.value, str)):
            doc = body[0]
            for i in range(doc.lineno - 1, doc.end_lineno):
                lines[i] = ''
    # Comments: where the tokenizer says they are (never inside a string).
    comments = {}
    for tok in tokenize.generate_tokens(io.StringIO(text).readline):
        if tok.type == tokenize.COMMENT:
            comments[tok.start[0] - 1] = tok.start[1]
    for i, col in comments.items():
        if lines[i]:
            lines[i] = lines[i][:col]
    out = (re.sub(r'\s+', ' ', line).strip() for line in lines)
    return '\n'.join(line for line in out if line).encode()


def strip_json(src: bytes) -> bytes:
    """JSON without its prose fields, canonically ordered."""
    def walk(v):
        if isinstance(v, dict):
            return {k: walk(x) for k, x in v.items() if k not in PROSE}
        if isinstance(v, list):
            return [walk(x) for x in v]
        return v
    return json.dumps(walk(json.loads(src)), sort_keys=True, separators=(',', ':')).encode()


def normalize(path: str, src: bytes) -> bytes:
    if re.search(r'\.(c|h|cpp|hpp)$', path):
        return strip_c(src)
    try:
        if path.endswith('.py'):
            return strip_py(src)
        if path.endswith('.json'):
            return strip_json(src)
    except (SyntaxError, ValueError):
        pass  # not parseable: the raw bytes are the input
    return src


def _git(*args: str) -> str:
    return subprocess.run(['git', *args], check=True, capture_output=True, text=True).stdout


def fingerprint(ref: str = 'HEAD') -> str:
    h = hashlib.sha256()
    entries = []
    for line in _git('ls-tree', '-r', ref).splitlines():
        meta, path = line.split('\t', 1)
        _, kind, obj = meta.split()
        if INPUTS.match(path) and not SKIP.search(path):
            entries.append((path, kind, obj))
    for path, kind, obj in sorted(entries):
        if kind == 'commit':  # a submodule: its pinned commit IS the input
            digest = obj.encode()
        else:
            src = subprocess.run(['git', 'cat-file', 'blob', obj],
                                 capture_output=True, check=True).stdout
            digest = hashlib.sha256(normalize(path, src)).digest()
        h.update(path.encode() + b'\0' + digest)
    return h.hexdigest()[:16]


if __name__ == '__main__':
    refs = sys.argv[1:] or ['HEAD']
    if len(refs) == 1:
        print(fingerprint(refs[0]))
    else:
        for ref in refs:
            print(ref, fingerprint(ref))
