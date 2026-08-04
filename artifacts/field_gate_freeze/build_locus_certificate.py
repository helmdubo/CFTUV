"""Сертификация четырёх новых локусов: 45 ↔ 49 ПО СУЩЕСТВУ, а не по числу.

Вопрос аудитора: четыре новых локуса стены 2.001 — это (а) настоящие
рождения/смерти траекторий, (б) лишние эмиссии ОДНОГО топологического
контакта или (в) внутренние записи транзакции, которым не место в публичном
наборе узлов.

Сертификат отвечает ЧИСЛАМИ, а не словом. Он сопоставляет локусы двух вершин
попарно по ТОЧНЫМ `(t, точка)` и раскладывает разницу на четыре класса:

* ЯКОРЬ — локус есть на обеих вершинах, состав совпал побитово;
* ЯКОРЬ СО СМЕНОЙ СОСТАВА — точка и время те же, `participants` те же,
  а `kind`/`converging_vertices`/`incidences` изменились;
* СНЯТЫЙ — есть на рабочей вершине, нет на отвергнутой;
* ДОБАВЛЕННЫЙ — наоборот.

И отдельно — КРАТНОСТЬ: у скольких узлов `incidences` длиннее одного и
сколько среди них РАЗЛИЧНЫХ. Кратность с одним различным значением есть
буквальное определение исхода (б): один контакт, эмитированный несколько раз.

Вход: два JSON маршрута (`field_route.py` на двух вершинах) и свидетельство
сборки граней. Вывод: `locus_certificate.json`.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

WORKING = "6ce0227"
REJECTED = "1dbf712"


def locus_key(locus: dict) -> str:
    return json.dumps([locus["time"], locus["point"]], sort_keys=True)


def composition(locus: dict) -> tuple:
    return (
        locus["kind"],
        tuple(tuple(p) for p in locus["participants"]),
        locus["converging_vertices"],
        tuple(locus["kinds"]),
        locus["incidence_count"],
    )


def certify(working: dict, rejected: dict, evidence: dict) -> dict:
    left = {locus_key(l): l for l in working["domains"][0]["loci"]}
    right = {locus_key(l): l for l in rejected["domains"][0]["loci"]}

    anchors_identical, anchors_recomposed = [], []
    for key in sorted(set(left) & set(right)):
        row = {
            "point_approx": None,
            "participants": left[key]["participants"],
            f"{WORKING}": {
                "kind": left[key]["kind"],
                "converging_vertices": left[key]["converging_vertices"],
                "kinds": left[key]["kinds"],
                "incidence_count": left[key]["incidence_count"],
            },
            f"{REJECTED}": {
                "kind": right[key]["kind"],
                "converging_vertices": right[key]["converging_vertices"],
                "kinds": right[key]["kinds"],
                "incidence_count": right[key]["incidence_count"],
            },
            "participants_agree": (
                left[key]["participants"] == right[key]["participants"]
            ),
            "time": left[key]["time"],
            "point": left[key]["point"],
        }
        if composition(left[key]) == composition(right[key]):
            anchors_identical.append(row)
        else:
            anchors_recomposed.append(row)

    removed = [left[key] for key in sorted(set(left) - set(right))]
    added = [right[key] for key in sorted(set(right) - set(left))]

    multiplicity = [
        {
            "node_index": node["node_index"],
            "time_approx": node["time_approx"],
            "point_approx": node["point_approx"],
            "kinds": node["kinds"],
            "participants": node["participants"],
            "incidence_count": len(node["incidences"]),
            "distinct_incidences": node["distinct_incidences"],
            "converging_vertices": node["converging_vertices"],
        }
        for node in evidence["multiway_nodes"]
    ]
    repeated = [row for row in multiplicity if row["incidence_count"] > 1]

    return {
        "question": (
            "45 -> 49: (а) настоящие рождения; (б) лишние эмиссии одного "
            "контакта; (в) внутренние записи транзакции"
        ),
        "snapshot": working["snapshot"],
        "sha256": working["sha256"],
        "domain_id": working["domains"][0]["domain_id"],
        "counts": {
            f"loci_{WORKING}": len(left),
            f"loci_{REJECTED}": len(right),
            "anchor_loci": len(anchors_identical) + len(anchors_recomposed),
            "anchor_loci_byte_identical": len(anchors_identical),
            "anchor_loci_recomposed": len(anchors_recomposed),
            "removed": len(removed),
            "added": len(added),
            "net": len(right) - len(left),
        },
        "participants_agree_on_every_anchor": all(
            row["participants_agree"]
            for row in anchors_identical + anchors_recomposed
        ),
        "multiway_multiplicity": {
            "multiway_nodes": len(multiplicity),
            "with_repeated_incidences": len(repeated),
            "all_repeats_are_one_distinct_incidence": all(
                row["distinct_incidences"] == 1 for row in repeated
            ),
            "every_multiway_folds_to_one_union_incidence": all(
                row["distinct_incidences"] == 1 for row in multiplicity
            ),
            "rows": multiplicity,
        },
        "anchors_recomposed": anchors_recomposed,
        "removed_loci": removed,
        "added_loci": added,
    }


def _main() -> None:
    out = Path(sys.argv[1])
    working = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
    rejected = json.loads(Path(sys.argv[3]).read_text(encoding="utf-8"))
    evidence = json.loads(Path(sys.argv[4]).read_text(encoding="utf-8"))
    result = certify(working, rejected, evidence)
    out.write_text(
        json.dumps(result, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    print(json.dumps(result["counts"], ensure_ascii=False))
    print(
        "participants совпали на каждом якоре:",
        result["participants_agree_on_every_anchor"],
    )
    print(
        "MULTIWAY с кратными incidences:",
        result["multiway_multiplicity"]["with_repeated_incidences"],
        "| все кратные — ОДИН различный incidence:",
        result["multiway_multiplicity"]["all_repeats_are_one_distinct_incidence"],
        "| каждый MULTIWAY свёрнут в один union-incidence:",
        result["multiway_multiplicity"][
            "every_multiway_folds_to_one_union_incidence"
        ],
    )


if __name__ == "__main__":
    _main()
