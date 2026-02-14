#!/usr/bin/env python3
import argparse
import re
from pathlib import Path

CONST_RE = re.compile(r"^\s*static\s+const\s+uint8_t\s+([A-Z0-9_]+)\s*=\s*([^;]+);")
STRUCT_RE = re.compile(r"typedef\s+struct\s*\{(.*?)\}\s*([A-Za-z0-9_]+)\s*;", re.S)
FIELD_RE = re.compile(r"\s*([A-Za-z_][A-Za-z0-9_<>\s\*]+?)\s+([A-Za-z_][A-Za-z0-9_]*)\s*(\[[^\]]+\])?\s*;")


def parse(path: Path):
    txt = path.read_text(encoding="utf-8", errors="ignore")
    consts = {}
    for ln in txt.splitlines():
        m = CONST_RE.match(ln)
        if m:
            consts[m.group(1)] = m.group(2).strip()

    structs = {}
    for m in STRUCT_RE.finditer(txt):
        body, name = m.groups()
        fields = []
        for raw in body.splitlines():
            raw = raw.split("//", 1)[0].strip()
            if not raw:
                continue
            f = FIELD_RE.match(raw)
            if not f:
                continue
            typ, fname, arr = f.groups()
            fields.append((" ".join(typ.split()), fname, arr or ""))
        structs[name] = fields
    return consts, structs


def print_diff(title, left, right):
    print(f"\n== {title} ==")
    keys = sorted(set(left) | set(right))
    has_diff = False
    for k in keys:
        if k not in left:
            has_diff = True
            print(f"+ only MASTER: {k} = {right[k]}")
        elif k not in right:
            has_diff = True
            print(f"- only SLAVE : {k} = {left[k]}")
        elif left[k] != right[k]:
            has_diff = True
            print(f"~ {k}: SLAVE={left[k]} | MASTER={right[k]}")
    if not has_diff:
        print("No differences.")


def main():
    ap = argparse.ArgumentParser(description="Compare protocol constants/packet structs between SLAVE and MASTER main.cpp files")
    ap.add_argument("master_main", type=Path, help="Path to MASTER src/main.cpp")
    ap.add_argument("--slave-main", type=Path, default=Path("src/main.cpp"), help="Path to SLAVE src/main.cpp")
    args = ap.parse_args()

    s_consts, s_structs = parse(args.slave_main)
    m_consts, m_structs = parse(args.master_main)

    # Focus on protocol-ish constants
    s_consts = {k: v for k, v in s_consts.items() if k.endswith("_TYPE") or k.startswith("MAX_")}
    m_consts = {k: v for k, v in m_consts.items() if k.endswith("_TYPE") or k.startswith("MAX_")}

    print(f"SLAVE:  {args.slave_main}")
    print(f"MASTER: {args.master_main}")

    print_diff("Protocol constants", s_consts, m_consts)
    print_diff("Packet structs", {k: s_structs[k] for k in sorted(s_structs)}, {k: m_structs[k] for k in sorted(m_structs)})


if __name__ == "__main__":
    main()
