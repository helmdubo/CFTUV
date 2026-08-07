"""Сколько ЕДИНИЦ точной работы съедает домен. Цена потолка, снятая на 6ce0227.

Почему на 6ce0227, а не на вершине. Вершина аудит-ветки несёт две полевые
регрессии (walls.012 обрывается на 2 узлах вместо 12, wall 2.001 жуёт prepare
20 s вместо 2.3 s). Потолок, снятый там, унаследовал бы ЦЕНУ РЕГРЕССИИ и
объявил бы её нормой здорового домена. Модули, которые считают работу
(`exact_sqrt_sum`, `event_time`, `motorcycle`, `coverage`), на 6ce0227 и на
вершине ПОБИТОВО одинаковы — проверено `git diff --quiet`, — поэтому счётчик
переносится сюда без единой правки семантики.

Счёт идёт в `UNBUDGETED_WORK`: на этой вершине скелет бюджета не называет,
поэтому вся работа домена попадает в телеметрию неоплаченного. Это ровно тот
же счётчик и те же статьи, что у транзакционного бюджета.

Прогон:
    python3 ../exact_work_budget/measure_units.py <куда.json> [потолок]
из каталога `artifacts/perf_prepare_diag`. Второй аргумент занижает объявленный
потолок — это НЕГАТИВНЫЕ ворота карточки В ПОЛЕ, а не только на корпусе: на
здоровых доменах заниженный кап обязан дать ИМЕНОВАННЫЙ отказ с полной деталью
вместо ответа. Проверка, у которой нельзя построить нарушение, не проверяет
ничего.
"""

from __future__ import annotations

import json
import sys
import time

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as canon


def _measure(label, run):
    canon.reset_factorization_memory()
    canon.reset_unbudgeted_work()
    started = time.perf_counter()
    outcome, charged, detail = run()
    seconds = time.perf_counter() - started
    # Транзакционный бюджет, если ветка его называет; иначе телеметрия
    # неоплаченного. Обе величины — один и тот же счётчик и те же статьи.
    row = dict(charged if charged else canon.UNBUDGETED_WORK.counters())
    row["seconds"] = round(seconds, 3)
    row["outcome"] = outcome
    row["leaked_unbudgeted"] = canon.UNBUDGETED_WORK.spent
    # Деталь нужна ровно негативным воротам: у отказа по бюджету она несёт
    # стадию, операцию, ширину радиканда и все шесть счётчиков. У здорового
    # домена она пустая, и пустота — тоже утверждение.
    row["detail"] = detail
    print(label, json.dumps(row, ensure_ascii=False), flush=True)
    return label, row


def _snapshot_rows(name, density):
    from run_domain import bundle_from_field_snapshot, snapshot_and_request
    from snapshot_bmesh import load_snapshot

    path = env.SNAPSHOTS / name
    payload = load_snapshot(path)
    selected = frozenset(payload["raw"]["selected_edges"])
    _, _, bundle = bundle_from_field_snapshot(path)
    return snapshot_and_request(bundle, selected, density=density)


def _building_rows(patches, density):
    import big_scene
    from cftuv.envelope_request_export import (
        _typed_value,
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )
    from cftuv.envelope_topology_export import stage_domain_inputs

    _, bundle, selected, _ = big_scene.survey()
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(
        bundle, selected
    )
    rows = []
    for patch_id in patches:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        snapshot = build_envelope_analysis_snapshot(
            bundle, included_patch_ids=frozenset({patch_id})
        )
        request = build_envelope_decal_request(
            snapshot,
            frozenset(by_domain[domain_id]),
            0.45,
            decal_request_id_value=request_id,
            density=density,
        )
        rows.append((patch_id, domain_id, snapshot, request))
    return rows


def main():
    out = sys.argv[1]
    from cftuv.envelope_queue_export import run_queue_domain

    def runner(row, alpha_text):
        patch_id, domain_id, snapshot, request = row

        def run():
            prepared, domain = run_queue_domain(
                patch_id, domain_id, snapshot, request, alpha_text
            )
            # `getattr`, а не точка: этот же файл считает единицы и на
            # 6ce0227, где поля `work_budget` у подготовки ещё нет. Там весь
            # расход законно уходит в телеметрию неоплаченного, и харнесс
            # обязан это пережить, иначе «здоровый домен» померить нечем.
            named = getattr(prepared, "work_budget", None)
            charged = () if named is None else named.counters()
            if not charged:
                # ОТКАЗ несёт свой расход в счётчиках подготовки и в детали —
                # так объявлено в `prepare_conveyor`. Читаем оттуда, иначе
                # негативные ворота показывали бы нули там, где расход как раз
                # и есть смысл прочитать.
                charged = tuple(
                    (name, value)
                    for name, value in prepared.counters
                    if name.startswith("EXACT_WORK_")
                )
            return (
                f"{domain.preparation_outcome}/{domain.coverage_outcome}",
                charged,
                domain.detail,
            )

        return run

    if len(sys.argv) > 2:
        # Потолок правится ПРЯМО В МОДУЛЕ, потому что хост зовёт
        # `prepare_conveyor` без бюджета и штатный кап читается по имени в
        # момент вызова. Второй провод «занижённый кап» через хост завёл бы в
        # продуктовый путь параметр, нужный только воротам.
        canon._EXACT_CANONICALIZATION_WORK_CAP = int(sys.argv[2])

    results = {}
    plans = [
        ("walls_012_d0_a045", _snapshot_rows("walls_012_snapshot.json", 0), "0.45"),
        ("walls_012_d1_a045", _snapshot_rows("walls_012_snapshot.json", 1), "0.45"),
        ("wall_2_001_d0_a045", _snapshot_rows("wall_2_001_snapshot.json", 0), "0.45"),
        ("wall_2_001_d0_a0254", _snapshot_rows("wall_2_001_snapshot.json", 0), "0.254"),
        ("building_d0_a045", _building_rows([109, 121], 0), "0.45"),
    ]
    for label, rows, alpha_text in plans:
        for row in rows:
            key = f"{label}:patch{row[0]}"
            _, value = _measure(key, runner(row, alpha_text))
            results[key] = value
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(results, handle, ensure_ascii=False, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
