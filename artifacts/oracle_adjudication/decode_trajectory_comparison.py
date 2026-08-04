"""Декодирует точные (t, точка) обеих вершин из ГОТОВОГО свидетельства
artifacts/field_gate_freeze/locus_certificate.json (ядро НЕ перегоняется).

t = dividend / (sum_i c_i * sqrt(radicand_i))  -- kernel/.../event_time.py:
EventTimeV1.canonical(); point.x/y -- та же каноническая сумма корней.
Здесь только ЧТЕНИЕ уже вычисленных точных чисел в float для отчёта:
округление вносится ТОЛЬКО на этом последнем шаге (форматирование), не в
самом свидетельстве.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

WT = Path("/tmp/claude-0/-home-user-CFTUV/4255c1b9-7873-5749-a1f5-eb79eb899ce6/scratchpad/wt-oracle")
OUT = Path("/tmp/claude-0/-home-user-CFTUV/4255c1b9-7873-5749-a1f5-eb79eb899ce6/scratchpad/oracle_adjudication")


def decode_sum(terms):
    total = 0.0
    for radicand, (num, den) in terms:
        total += (num / den) * math.sqrt(radicand)
    return total


def decode_time(t):
    dnum, dden = t["dividend"]
    dividend = dnum / dden
    divisor = decode_sum(t["divisor"])
    return dividend / divisor


def decode_point(p):
    return decode_sum(p["x"]), decode_sum(p["y"])


def classify(participants):
    """Помечает участников как EDGE_A / EDGE_B / EDGE_BOTTOM / FAN по геометрии
    названной в DECISIONS.md (2026-08-04, «НАЗВАННОЕ ИЗМЕНЕНИЕ ТРАЕКТОРИИ»)."""

    tags = []
    for p in participants:
        if len(p) == 5:
            tags.append(f"FAN(point=({p[0]},{p[1]}),idx={p[4]})")
        elif len(p) == 4:
            x1, y1, x2, y2 = p
            if y1 == 0 and y2 == 0:
                tags.append(f"EDGE_BOTTOM(({x1},{y1})-({x2},{y2}))")
            elif y1 == 8558 and y2 == 32768:
                tags.append(f"EDGE_DIAGONAL(({x1},{y1})-({x2},{y2}))")
            else:
                tags.append(f"EDGE(({x1},{y1})-({x2},{y2}))")
        else:
            tags.append(str(p))
    return tags


def main():
    cert = json.loads((WT / "artifacts" / "field_gate_freeze" / "locus_certificate.json").read_text())

    def rows(loci, label):
        out = []
        for loc in loci:
            t = decode_time(loc["time"])
            x, y = decode_point(loc["point"])
            out.append({
                "trajectory": label,
                "t_float": t,
                "point_float": [x, y],
                "kind": loc["kind"],
                "participants_raw": loc["participants"],
                "participants_tagged": classify(loc["participants"]),
            })
        return out

    removed = rows(cert["removed_loci"], "6ce0227_only (removed on new vertex)")
    added = rows(cert["added_loci"], "new_vertex_only (821995f/1dbf712 descendant, added)")

    result = {
        "source": "artifacts/field_gate_freeze/locus_certificate.json (готовое свидетельство, ядро не перегонялось)",
        "commits": {
            "old_trajectory": "6ce0227 (45 skeleton nodes on wall 2.001, до слияния транзакции одновременного уровня)",
            "new_trajectory": "821995f9ae3e70b599471259851c15dcd4de70b8, потомок 1dbf712 (49 skeleton nodes, RELEASE CANDIDATE REJECTED_FIELD_GATE)",
        },
        "counts": cert["counts"],
        "removed_loci_6ce0227_only": removed,
        "added_loci_new_vertex_only": added,
        "reading": (
            "На 6ce0227 (removed): каждый локус несёт ДВА FAN-участника (оба веера "
            "одного оконного выреза) плюс ОДНО обычное ребро (EDGE_DIAGONAL при "
            "t~14531/14536, EDGE_BOTTOM при t~18089/18090) -- то есть два веера "
            "встречаются ДРУГ С ДРУГОМ в одной точке вместе с этим ребром. "
            "На новой вершине (added): каждый локус несёт ОДИН FAN-участник плюс "
            "ОДНО ребро 'A' (участник вида (x,y,x2,y2) БЕЗ суффикса веера, отличный "
            "от EDGE_DIAGONAL/EDGE_BOTTOM выше) -- то есть каждый веер встречается "
            "порознь с рёбром A, а не со вторым веером; ребро A доживает до "
            "MULTIWAY-события при t~22373."
        ),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "trajectory_comparison.json").write_text(json.dumps(result, indent=1, ensure_ascii=False))
    print("removed:", len(removed), "added:", len(added))
    for r in removed:
        print(f"  [OLD 6ce0227] t={r['t_float']:.4f} kind={r['kind']} {r['participants_tagged']}")
    for r in added:
        print(f"  [NEW vertex ] t={r['t_float']:.4f} kind={r['kind']} {r['participants_tagged']}")


if __name__ == "__main__":
    main()
