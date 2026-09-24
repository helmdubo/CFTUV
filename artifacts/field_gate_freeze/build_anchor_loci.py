"""Якорные локусы: пересечение двух математик по ТОЧНЫМ (t, точка).

ЗАЧЕМ ОНИ ВООБЩЕ. Полевая регрессия сдвинула счёт узлов стены 2.001 с 45 на
49, и вопрос «какое число верно» ОТКРЫТ: `SkeletonOutcome.EXACT` про 49 узлов
ничего не доказывает — исход и `proof_status` независимы. Заморозить 45 значит
объявить властью старую математику; заморозить 49 — новую. Ни того, ни другого
свидетельства нет.

Якорь снимает развилку, не решая её: якорный локус — тот, который ОБЕ вершины
выдают в ПОБИТОВО одинаковых точных `(t, точка)`. Такой локус не есть
историческая случайность ни одной из математик, и требовать его сохранения
можно, ничего не постулируя про спорные.

КАК СНЯТЬ. Тем же `field_route.py`, запущенным в двух worktree:

    git worktree add --detach <РАБОЧАЯ> 6ce0227
    (в каждом worktree)
    python3 artifacts/field_gate_freeze/field_route.py \
        wall_2_001_snapshot.json 0.254 0 - <вывод>.json
    python3 artifacts/field_gate_freeze/build_anchor_loci.py <выход> \
        <снимок>=<рабочая>.json,<эта>.json ...

Скрипт НЕ ходит в git сам: маршрут стоит секунды, и прятать выбор вершины
внутрь генератора значило бы, что расписка не показывает, что с чем сравнили.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

WORKING_VERTEX = "6ce0227"
REJECTED_VERTEX = "1dbf712"


def locus_key(locus: dict) -> str:
    return json.dumps([locus["time"], locus["point"]], sort_keys=True)


def pair(working: dict, rejected: dict) -> list:
    rows = []
    by_domain = {record["domain_id"]: record for record in rejected["domains"]}
    for left in working["domains"]:
        right = by_domain.get(left["domain_id"])
        if right is None:
            raise SystemExit(
                f"DOMAIN_ABSENT_ON_REJECTED_VERTEX: {left['domain_id']}"
            )
        first = {locus_key(l): l for l in left.get("loci", [])}
        second = {locus_key(l): l for l in right.get("loci", [])}
        shared = sorted(set(first) & set(second))
        anchors = [
            {
                "time": first[key]["time"],
                "point": first[key]["point"],
                "participants": first[key]["participants"],
                "participants_agree": (
                    first[key]["participants"] == second[key]["participants"]
                ),
                f"kind_{WORKING_VERTEX}": first[key]["kind"],
                f"kind_{REJECTED_VERTEX}": second[key]["kind"],
                f"converging_{WORKING_VERTEX}": first[key][
                    "converging_vertices"
                ],
                f"converging_{REJECTED_VERTEX}": second[key][
                    "converging_vertices"
                ],
                f"incidences_{REJECTED_VERTEX}": second[key][
                    "incidence_count"
                ],
            }
            for key in shared
        ]
        rows.append(
            {
                "patch_id": left["patch_id"],
                "domain_id": left["domain_id"],
                f"loci_{WORKING_VERTEX}": len(first),
                f"loci_{REJECTED_VERTEX}": len(second),
                "anchor_loci": len(shared),
                f"only_{WORKING_VERTEX}": len(set(first) - set(second)),
                f"only_{REJECTED_VERTEX}": len(set(second) - set(first)),
                "participants_agree_on_all_anchors": all(
                    row["participants_agree"] for row in anchors
                ),
                "anchors": anchors,
            }
        )
    return rows


def _main() -> None:
    out = Path(sys.argv[1])
    table: dict = {}
    for argument in sys.argv[2:]:
        snapshot, paths = argument.split("=", 1)
        left_path, right_path = paths.split(",", 1)
        table[snapshot] = pair(
            json.loads(Path(left_path).read_text(encoding="utf-8")),
            json.loads(Path(right_path).read_text(encoding="utf-8")),
        )
    out.write_text(
        json.dumps(table, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    for snapshot, rows in table.items():
        for row in rows:
            print(
                f"{snapshot:34s} п{row['patch_id']:<4} "
                f"якорей {row['anchor_loci']:3d} "
                f"({WORKING_VERTEX} {row[f'loci_{WORKING_VERTEX}']}, "
                f"{REJECTED_VERTEX} {row[f'loci_{REJECTED_VERTEX}']}, "
                f"только-{WORKING_VERTEX} {row[f'only_{WORKING_VERTEX}']}, "
                f"только-{REJECTED_VERTEX} {row[f'only_{REJECTED_VERTEX}']}) "
                f"participants совпали: "
                f"{row['participants_agree_on_all_anchors']}"
            )


if __name__ == "__main__":
    _main()
