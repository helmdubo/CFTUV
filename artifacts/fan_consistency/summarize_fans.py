"""Сводка перечня вееров: группы ~90° углов по механизму, число вееров, пример (меш/патч/вершина).

  python artifacts/fan_consistency/summarize_fans.py [--write]

Читает JSON из RESULTS (enumerate_fans.py). С `--write` кладёт компактную сводку рядом со скриптами.
"""
from __future__ import annotations

import json
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths  # noqa: E402

RIGHT_BAND_DEG = 0.5  # только для ОТБОРА углов в «около-прямые» группы; на решения ядра не влияет


def eval_class(corner) -> str:
    if corner["eval_dot0"]:
        return "EVAL_EXACT_90"
    return "EVAL_ABOVE_90" if abs(corner["eval_turn_deg"]) > 90.0 else "EVAL_BELOW_90"


def raw_class(corner) -> str:
    if corner["raw_exact_half"]:
        return "RAW_EXACT_90"
    tag = "RAW_ABOVE" if corner["raw_dev_deg"] > 0 else "RAW_BELOW"
    return f"{tag}+RESTORED" if corner["restored"] else f"{tag}+NOT_RESTORED"


def load(mesh: str, density: int):
    return json.loads(
        (_paths.RESULTS / f"fans_{mesh.replace('.', '_')}_d{density}.json").read_text(encoding="utf-8")
    )


def summarize(mesh: str, density: int) -> dict:
    rows = load(mesh, density)
    groups = defaultdict(list)
    other = defaultdict(list)
    for row in rows:
        for c in row["corners"]:
            if abs(abs(c["src_turn_deg"]) - 90.0) < RIGHT_BAND_DEG:
                key = (
                    raw_class(c),
                    "SRC_EXACT_90" if c["src_dot0"] else "SRC_NOT_EXACT",
                    eval_class(c),
                    f"selH={c['sel_H']}",
                    f"specH={c['spec_H']}",
                    f"lift={c['lift_law'] or '-'}",
                    f"canon_auth={'Y' if c['canonical_authority'] else 'N'}",
                    f"cov_faces={c['coverage_fan_faces']}",
                    f"part_faces={c['partition_fan_faces']}",
                )
                groups[key].append((row["patch"], c["vertex"], c["eval_turn_deg"]))
            else:
                other[(f"turn~{round(abs(c['src_turn_deg']))}", f"specH={c['spec_H']}", f"lift={c['lift_law'] or '-'}")].append(
                    (row["patch"], c["vertex"])
                )
    return {"groups": groups, "other": other, "domains": len(rows)}


def consistency_table() -> dict:
    """Одинаковые точно-прямые углы (сырая доля ровно 1/2, исходная карта точна): разброс итогового H по плотностям."""

    table: dict = {}
    print("=== ТОЧНО-ПРЯМЫЕ вогнутые углы (raw == 1/2, src dot == 0): итоговый H по плотностям")
    for mesh in _paths.MESHES:
        for density in range(5):
            try:
                rows = load(mesh, density)
            except FileNotFoundError:
                continue
            histogram: dict[int, int] = defaultdict(int)
            by_class: dict[str, dict[int, int]] = defaultdict(lambda: defaultdict(int))
            for row in rows:
                for c in row["corners"]:
                    if c["raw_exact_half"] and c["src_dot0"]:
                        histogram[c["spec_H"]] += 1
                        by_class[eval_class(c)][c["spec_H"]] += 1
            if not histogram:
                continue
            verdict = "СОГЛАСОВАНО" if len(histogram) == 1 else "РАСХОДИТСЯ"
            print(
                f"  {mesh:13s} d{density}: H={dict(sorted(histogram.items()))}  {verdict}  "
                f"по классу решётки: { {k: dict(sorted(v.items())) for k, v in sorted(by_class.items())} }"
            )
            table[f"{mesh}/d{density}"] = {
                "H_histogram": dict(sorted(histogram.items())),
                "by_eval_class": {k: dict(sorted(v.items())) for k, v in sorted(by_class.items())},
                "consistent": len(histogram) == 1,
            }
    return table


def main() -> None:
    write = "--write" in sys.argv
    if "--consistency" in sys.argv:
        table = consistency_table()
        if write:
            (Path(__file__).resolve().parent / "summary_consistency.json").write_text(
                json.dumps(table, ensure_ascii=False, indent=1), encoding="utf-8"
            )
        return
    out: dict = {}
    for mesh in _paths.MESHES:
        for density in (1, 2, 4):
            try:
                s = summarize(mesh, density)
            except FileNotFoundError:
                continue
            print(f"=== {mesh} d{density}: domains={s['domains']}")
            tab = []
            for key, items in sorted(s["groups"].items(), key=lambda kv: (-len(kv[1]), kv[0])):
                ex = items[0]
                print(f"  {len(items):4d}  {' | '.join(key)}   e.g. patch {ex[0]} v{ex[1]} eval_turn={ex[2]:+.6f}")
                tab.append({"count": len(items), "key": list(key), "example": {"patch": ex[0], "vertex": ex[1], "eval_turn_deg": ex[2]}})
            for key, items in sorted(s["other"].items(), key=lambda kv: (-len(kv[1]), kv[0])):
                print(f"  {len(items):4d}  NON-RIGHT {' | '.join(key)}   e.g. patch {items[0][0]} v{items[0][1]}")
                tab.append({"count": len(items), "key": ["NON_RIGHT", *key], "example": {"patch": items[0][0], "vertex": items[0][1]}})
            out[f"{mesh}/d{density}"] = tab
    if write:
        target = Path(__file__).resolve().parent / "summary_groups.json"
        target.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
        print("written", target)


if __name__ == "__main__":
    main()
