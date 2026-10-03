"""Переснятие строк таблицы якорных локусов под ТЕКУЩИМИ законами продукта.

ЗАЧЕМ. `anchor_loci.json` был снят как пересечение двух математик (`build_anchor_loci.py`). Когда закон
продукта сдвигает точные `(t, точка)` по замыслу, строка перестаёт быть свидетельством чего-либо, кроме
старого закона, и держать ворота на закрепке старого закона нельзя: закрепка ослепляет их на продукте.
Переснятие заменяет строку локусами ТЕКУЩЕГО ядра (одна математика, как для патча 17 после DECAL-WELD C1) и
записывает, КАКОЙ закон сдвинул каждый старый якорь: ворота остаются чувствительными, а сдвиг — названным.

КАК СНЯТЬ. Маршрутом `field_route.py` на каждом слепке, без закрепок (текущие локусы) и с закрепками
старых законов (атрибуция):

    python artifacts/field_gate_freeze/field_route.py <слепок> <alpha> 0 <патчи|-> <вывод>.json
    python artifacts/field_gate_freeze/field_route.py ... --pin-fans <ИМЯ[,ИМЯ]> --pin-join JOIN_SOFT_BEND_THRESHOLD_30_V1

и планом (JSON-список строк), где у каждой строки таблицы:

    {"snapshot": "...", "patch_id": 0, "current": "<маршрут без закрепок>.json",
     "attribution": [["МЕТКА", "<маршрут с закрепкой>.json"], ...],   # порядок = приоритет метки
     "join_off": "<маршрут только с --pin-join>.json"}                # необязателен

    python artifacts/field_gate_freeze/refreeze_anchor_loci.py <таблица>.json <выход>.json <план>.json "<пометка>"

Старый якорь, которого нет в текущем маршруте, получает ПЕРВУЮ метку, чей маршрут его содержит; якорь без метки
записывается как `UNATTRIBUTED` и печатается — это дефект переснятия, а не принятое состояние. `retired_by_join` —
локусы, которые порог JOIN 30° даёт на ТЕКУЩИХ законах веера и которых нет при 45°: ворота обязаны видеть, что они
не вернулись (красный контроль `--pin-join`). Строки, которых нет в плане, не трогаются.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

#: Ключи пересечения двух математик: после переснятия строка — одна математика, и эти счётчики врали бы.
TWO_MATH_KEYS = ("loci_6ce0227", "loci_1dbf712", "only_6ce0227", "only_1dbf712")


def locus_key(locus: dict) -> str:
    return json.dumps([locus["time"], locus["point"]], sort_keys=True)


def domain_loci(route_path: str, patch_id: int) -> dict:
    """Локусы домена патча в выводе `field_route.py`: ключ -> запись."""

    route = json.loads(Path(route_path).read_text(encoding="utf-8"))
    for record in route["domains"]:
        if record["patch_id"] == patch_id:
            return {locus_key(locus): locus for locus in record.get("loci", [])}
    raise SystemExit(f"DOMAIN_ABSENT: патч {patch_id} в {route_path}")


def anchor_record(locus: dict) -> dict:
    return {
        "time": locus["time"],
        "point": locus["point"],
        "participants": locus["participants"],
        "participants_agree": True,
    }


def refreeze_row(row: dict, spec: dict, note: str) -> dict:
    current = domain_loci(spec["current"], row["patch_id"])
    old_keys = [
        json.dumps([anchor["time"], anchor["point"]], sort_keys=True)
        for anchor in row["anchors"]
    ]
    attribution = [
        (label, domain_loci(path, row["patch_id"])) for label, path in spec["attribution"]
    ]
    moved_by: dict[str, int] = {}
    kept = 0
    for key in old_keys:
        if key in current:
            kept += 1
            continue
        label = next((name for name, loci in attribution if key in loci), "UNATTRIBUTED")
        moved_by[label] = moved_by.get(label, 0) + 1
    retired = []
    if spec.get("join_off"):
        join_off = domain_loci(spec["join_off"], row["patch_id"])
        retired = [
            anchor_record(join_off[key]) for key in sorted(set(join_off) - set(current))
        ]
    rerecorded = {
        "by": note,
        "previous_anchor_loci": len(old_keys),
        "kept": kept,
        "moved": len(old_keys) - kept,
        "moved_by": dict(sorted(moved_by.items())),
    }
    fresh = {
        key: value
        for key, value in row.items()
        if key not in TWO_MATH_KEYS and key not in ("anchors", "anchor_loci", "participants_agree_on_all_anchors")
    }
    fresh["rerecorded"] = rerecorded
    fresh["anchor_loci"] = len(current)
    fresh["participants_agree_on_all_anchors"] = True
    if retired:
        fresh["retired_by_join"] = retired
    fresh["anchors"] = [anchor_record(current[key]) for key in sorted(current)]
    return fresh


def main() -> None:
    table_path, out_path, plan_path, note = sys.argv[1:5]
    table = json.loads(Path(table_path).read_text(encoding="utf-8"))
    plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
    for spec in plan:
        rows = table[spec["snapshot"]]
        index = next(i for i, row in enumerate(rows) if row["patch_id"] == spec["patch_id"])
        rows[index] = refreeze_row(rows[index], spec, note)
        record = rows[index]["rerecorded"]
        print(
            f"{spec['snapshot']:34s} п{spec['patch_id']:<4} было {record['previous_anchor_loci']:3d} "
            f"осталось {record['kept']:3d} сдвинуто {record['moved']:3d} {record['moved_by']} "
            f"стало {rows[index]['anchor_loci']:3d} снято JOIN {len(rows[index].get('retired_by_join', []))}"
        )
    Path(out_path).write_text(
        json.dumps(table, ensure_ascii=False, indent=1), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
