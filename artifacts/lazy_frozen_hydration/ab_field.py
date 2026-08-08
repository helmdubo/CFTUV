"""A/B пяти полевых доменов: плотная гидратация против ленивой.

Один прогон даёт ТРИ величины на домен и на режим: единицы бюджета по шести
статьям, секунды и наблюдаемый ответ (исход обеих ступеней плюс отпечаток
скелета). Третья здесь не для отчёта, а для ворот: A/B, у которого ответы не
сверяются, мерил бы скорость двух РАЗНЫХ вычислений.

Третий режим — КОНТРФАКТ `--free-hashes`. Он подменяет `Fraction.__hash__`
кэширующим и НИЧЕГО больше не трогает. Ответы при этом обязаны совпасть до
байта (хэш кэшируется, а не меняется), а разница секунд называет ЧИСЛОМ, во
сколько обходится тождество точных величин. Это измерение, а не поставка:
подмена живёт внутри процесса замера и в ядро не идёт.

Прогон из каталога `artifacts/perf_prepare_diag`:

    PYTHONPATH=. python3 ../lazy_frozen_hydration/ab_field.py <out.json> [--free-hashes]
"""

from __future__ import annotations

import fractions
import json
import sys
import time

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as canon


def _install_cached_fraction_hash():
    """Кэш хэша дроби В ПАМЯТИ ЗАМЕРА. Значение то же — цена другая."""

    cache: dict[int, int] = {}
    original = fractions.Fraction.__hash__

    def cached(self):
        key = id(self)
        value = cache.get(key)
        if value is None:
            value = original(self)
            cache[key] = value
        return value

    fractions.Fraction.__hash__ = cached
    return original


def _domains():
    from run_domain import bundle_from_field_snapshot, snapshot_and_request
    from snapshot_bmesh import load_snapshot

    for name, density in (
        ("walls_012_snapshot.json", 0),
        ("walls_012_snapshot.json", 1),
        ("wall_2_001_snapshot.json", 0),
    ):
        path = env.SNAPSHOTS / name
        payload = load_snapshot(path)
        selected = frozenset(payload["raw"]["selected_edges"])
        _, _, bundle = bundle_from_field_snapshot(path)
        for row in snapshot_and_request(bundle, selected, density=density):
            yield f"{name}:d{density}", row


def _building():
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
    for patch_id in (109, 121):
        domain_id = _typed_value("patch-domain", revision, patch_id)
        snapshot = build_envelope_analysis_snapshot(
            bundle, included_patch_ids=frozenset({patch_id})
        )
        request = build_envelope_decal_request(
            snapshot,
            frozenset(by_domain[domain_id]),
            0.45,
            decal_request_id_value=request_id,
            density=0,
        )
        yield "building:d0", (patch_id, domain_id, snapshot, request)


def _answer(prepared, domain):
    """Наблюдаемый ответ домена. Сверяется между режимами, а не печатается."""

    from cftuv_envelope.wavefront.digest import semantic_digest

    rows = []
    for region in prepared.regions:
        rows.append(
            (
                repr(region.skeleton_outcome),
                repr(region.face_outcome),
                None if region.skeleton is None
                else semantic_digest(region.skeleton),
                None if region.partition is None
                else len(region.partition.faces),
                None if region.partition is None
                else repr(region.partition.doubled_area.terms),
            )
        )
    return (
        str(domain.preparation_outcome),
        str(domain.coverage_outcome),
        tuple(rows),
    )


def _provider(dense: bool):
    from cftuv_envelope.wavefront import prepare_conveyor

    def provide(patch_id, domain_id, selected, snapshot, request):
        return prepare_conveyor(snapshot, request, dense_hydration=dense)

    return provide


def main():
    out = sys.argv[1]
    if "--free-hashes" in sys.argv:
        _install_cached_fraction_hash()
    from cftuv.envelope_queue_export import run_queue_domain

    results = {}
    rows = list(_domains()) + list(_building())
    for label, (patch_id, domain_id, snapshot, request) in rows:
        key = f"{label}:patch{patch_id}"
        results[key] = {}
        answers = {}
        for mode in ("dense", "lazy"):
            canon.reset_factorization_memory()
            canon.reset_unbudgeted_work()
            started = time.perf_counter()
            prepared, domain = run_queue_domain(
                patch_id, domain_id, snapshot, request, "0.45",
                # Режим гидратации входит через объявленный хук подготовки:
                # хост про него не знает и знать не обязан, это свойство ядра.
                preparation_provider=_provider(mode == "dense"),
            )
            seconds = time.perf_counter() - started
            budget = getattr(prepared, "work_budget", None)
            counters = dict(budget.counters()) if budget else {}
            counters["seconds"] = round(seconds, 3)
            counters["outcome"] = (
                f"{domain.preparation_outcome}/{domain.coverage_outcome}"
            )
            results[key][mode] = counters
            answers[mode] = _answer(prepared, domain)
        # ВОРОТА A/B: два режима обязаны отвечать одинаково, иначе секунды
        # сравнивались бы у двух разных вычислений.
        results[key]["answers_agree"] = answers["dense"] == answers["lazy"]
        assert answers["dense"] == answers["lazy"], key
        print(key, json.dumps(results[key], ensure_ascii=False), flush=True)
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(results, handle, ensure_ascii=False, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
