"""A/B полевых доменов: СТАРОЕ представление тождества против нового.

Один прогон даёт на домен и на режим три величины: шесть статей бюджета,
секунды и наблюдаемый ответ. Третья — не для отчёта, а для ворот: A/B, у
которого ответы не сверяются, мерил бы скорость двух РАЗНЫХ вычислений.

Порядок режимов чередуется (`legacy, cached, legacy`) по той же причине, что и
в `where_the_seconds_go.py`: третий прогон — контроль дрейфа машины. Если
контрольный `legacy` разошёлся с первым сильнее, чем `cached` разошёлся с ними
обоими, замер мерил загрузку, а не код, и такой замер докладывать нельзя.

Прогон из каталога `artifacts/perf_prepare_diag`:

    PYTHONPATH=. python3 ../exact_identity_intern/ab_identity.py <out.json> \
        [--only=<подстрока метки>] [--repeats=N]
"""

from __future__ import annotations

import json
import sys
import time

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as canon
from cftuv_envelope.wavefront.exact_identity import (
    ExactIdentityModeV1,
    set_identity_mode,
)


def _domains():
    from run_domain import bundle_from_field_snapshot, snapshot_and_request
    from snapshot_bmesh import load_snapshot

    for name, density, alpha in (
        ("walls_012_snapshot.json", 0, "0.45"),
        ("walls_012_snapshot.json", 1, "0.45"),
        ("wall_2_001_snapshot.json", 0, "0.45"),
        ("wall_2_001_snapshot.json", 0, "0.254"),
    ):
        path = env.SNAPSHOTS / name
        payload = load_snapshot(path)
        selected = frozenset(payload["raw"]["selected_edges"])
        _, _, bundle = bundle_from_field_snapshot(path)
        for row in snapshot_and_request(bundle, selected, density=density):
            yield f"{name}:d{density}:a{alpha}", alpha, row


def _building(patches=(109, 121)):
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
            density=0,
        )
        yield "building:d0:a0.45", "0.45", (patch_id, domain_id, snapshot, request)


def _answer(prepared, domain):
    """Наблюдаемый ответ домена ЦЕЛИКОМ, а не его исход."""

    from cftuv_envelope.wavefront.digest import semantic_digest

    rows = []
    for region in prepared.regions:
        rows.append(
            (
                repr(region.skeleton_outcome),
                repr(region.face_outcome),
                None if region.skeleton is None
                else semantic_digest(region.skeleton),
                None if region.skeleton is None
                else region.skeleton.levels,
                None if region.skeleton is None
                else tuple(tuple(item) for item in region.skeleton.counters),
                None if region.partition is None
                else len(region.partition.faces),
                None if region.partition is None
                else repr(region.partition.doubled_area.terms),
            )
        )
    return (
        str(domain.preparation_outcome),
        str(domain.coverage_outcome),
        str(domain.detail),
        tuple(rows),
    )


def main() -> None:
    out = sys.argv[1]
    only = next(
        (arg.split("=", 1)[1] for arg in sys.argv[2:] if arg.startswith("--only=")),
        None,
    )
    patches = next(
        (arg.split("=", 1)[1] for arg in sys.argv[2:] if arg.startswith("--patches=")),
        None,
    )
    from cftuv.envelope_queue_export import run_queue_domain

    building = (
        tuple(int(item) for item in patches.split(",")) if patches else (109, 121)
    )
    rows = list(_domains()) + list(_building(building))
    results = {}
    for label, alpha, (patch_id, domain_id, snapshot, request) in rows:
        key = f"{label}:patch{patch_id}"
        if only is not None and only not in key:
            continue
        results[key] = {}
        answers = {}
        for run_label, mode in (
            ("legacy", ExactIdentityModeV1.LEGACY_TUPLE),
            ("cached", ExactIdentityModeV1.CACHED),
            ("legacy_control", ExactIdentityModeV1.LEGACY_TUPLE),
        ):
            previous = set_identity_mode(mode)
            canon.reset_factorization_memory()
            canon.reset_unbudgeted_work()
            started = time.perf_counter()
            try:
                prepared, domain = run_queue_domain(
                    patch_id, domain_id, snapshot, request, alpha
                )
            finally:
                seconds = time.perf_counter() - started
                set_identity_mode(previous)
            budget = getattr(prepared, "work_budget", None)
            counters = dict(budget.counters()) if budget else {}
            counters["seconds"] = round(seconds, 3)
            counters["outcome"] = (
                f"{domain.preparation_outcome}/{domain.coverage_outcome}"
            )
            counters["leaked_unbudgeted"] = canon.UNBUDGETED_WORK.spent
            results[key][run_label] = counters
            answers[run_label] = _answer(prepared, domain)
        results[key]["answers_agree"] = len(set(answers.values())) == 1
        assert len(set(answers.values())) == 1, ("ANSWERS_DIVERGED", key)
        print(key, json.dumps(results[key], ensure_ascii=False), flush=True)
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(results, handle, ensure_ascii=False, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
