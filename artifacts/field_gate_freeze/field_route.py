"""Полный безблендерный маршрут полевого слепка — тот же, которым идёт Blender.

decode -> request policy -> source snap -> metric -> angular profile ->
conveyor preparation -> skeleton -> faces -> coverage.

ЧТО ЗДЕСЬ ПЕРЕИСПОЛЬЗОВАНО, А НЕ НАПИСАНО ЗАНОВО. Всё, кроме двух вещей ниже,
берётся из существующих харнессов: `artifacts/perf_prepare_diag/env.py`
(пути + заглушки mathutils/bmesh/bpy), `snapshot_bmesh.py` (BMesh-срез над
слепком владельца), `run_domain.py` (`bundle_from_field_snapshot`,
`timed_queue`, `report`), `big_scene.py` (именованная подмена UV-классификации
многопетлевых патчей для building). Ядро зовётся своими публичными дверями
`prepare_conveyor`/`conveyor_coverage` через хостовый `run_queue_domain` —
ровно как в поле.

ДВЕ ВЕЩИ, КОТОРЫХ В ХАРНЕССАХ НЕ БЫЛО, и почему они здесь.

1. ПО-ДОМЕННЫЙ ЗАХВАТ ИМЕНОВАННОГО ОТКАЗА ХОСТА. `run_domain.snapshot_and_request`
   строит снимок и запрос ДЛЯ ВСЕХ доменов меша одним циклом без try. У
   walls.001 это фатально: домен-склон законно отвергается
   `NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED` на стадии экспорта, исключение
   уносит весь меш, и домен-дверь, который в поле СТРОИТСЯ, становится
   невидим. Здесь тот же цикл с тем же порядком вызовов
   (`stage_domain_inputs` -> `build_envelope_analysis_snapshot` ->
   `build_envelope_decal_request`), но отказ каждого домена ловится и
   становится ЗАПИСЬЮ этого домена. Тот же приём уже применён в
   `big_scene.py` (`except EnvelopeHostAdapterError` вокруг пары builder'ов) —
   это перенос его на слепки поменьше, а не новая политика.

2. КАП РАБОТЫ. Полевой сигнал владельца про building — «висит». Ворота,
   которые ждут бесконечно, не ворота. Кап здесь ВНЕШНИЙ и honest: домен
   считается в отдельном процессе с жёстким пределом секунд, превышение
   становится ИМЕНОВАННЫМ исходом `DOMAIN_WORK_CAP_EXCEEDED`, а не зависанием.
   Wall-clock назван явно: детерминированный `ExactWorkBudgetV1` в единицах
   работы — отдельная карточка (ремонт №2 порядка работ), и подменять её
   секундомером эта карточка не вправе. Секунды здесь сторожат ВОРОТА, а не
   выносят суждение о математике.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
DIAG = REPO_ROOT / "artifacts" / "perf_prepare_diag"
SNAPSHOTS = REPO_ROOT / "artifacts" / "field_snapshots"

if str(DIAG) not in sys.path:
    sys.path.insert(0, str(DIAG))

import env  # noqa: E402,F401  (пути + заглушки; ставит и kernel/src, и корень)
from run_domain import bundle_from_field_snapshot  # noqa: E402
from snapshot_bmesh import selected_edge_ids  # noqa: E402


HOST_REJECT = "HOST_EXPORT_REJECTED"
STAGE_RAISED = "STAGE_RAISED"
WORK_CAP_EXCEEDED = "DOMAIN_WORK_CAP_EXCEEDED"


def snapshot_sha256(name: str) -> str:
    import hashlib

    return hashlib.sha256((SNAPSHOTS / name).read_bytes()).hexdigest()


def install_building_substitution() -> str:
    """Именованная подмена UV-классификации: только для building_full."""

    import big_scene  # noqa: F401  (подмена ставится на импорте)

    return big_scene.SUBSTITUTION


def staged_domains(bundle, selected, *, alpha: float, density: int):
    """Домены меша: (patch_id, domain_id, snapshot, request) либо именованный отказ.

    Порядок и аргументы вызовов совпадают с `run_domain.snapshot_and_request`;
    отличие ровно одно — отказ домена не уносит остальные (см. докстринг №1).
    """

    from cftuv.envelope_request_export import (
        EnvelopeHostAdapterError,
        _typed_value,
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )
    from cftuv.envelope_topology_export import stage_domain_inputs

    (
        _scene,
        revision,
        patch_ids,
        request_id,
        selected_by_domain,
    ) = stage_domain_inputs(bundle, frozenset(selected))

    out = []
    for patch_id in patch_ids:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        try:
            snapshot = build_envelope_analysis_snapshot(
                bundle, included_patch_ids=frozenset({patch_id})
            )
            request = build_envelope_decal_request(
                snapshot,
                frozenset(selected_by_domain[domain_id]),
                alpha,
                decal_request_id_value=request_id,
                density=density,
            )
        except EnvelopeHostAdapterError as error:
            out.append(
                {
                    "patch_id": int(patch_id),
                    "domain_id": domain_id,
                    "stage": "HOST_EXPORT",
                    "outcome": f"{HOST_REJECT}:{error.outcome.value}",
                    "detail": str(error),
                    "ready": None,
                }
            )
            continue
        out.append(
            {
                "patch_id": int(patch_id),
                "domain_id": domain_id,
                "stage": "QUEUE",
                "outcome": None,
                "detail": "",
                "ready": (patch_id, domain_id, snapshot, request),
            }
        )
    return out


def terms(value) -> list:
    """Каноническая сумма корней в JSON: [[радиканд, [числитель, знаменатель]], ...]."""

    return [
        [int(radicand), [int(c.numerator), int(c.denominator)]]
        for radicand, c in value.terms
    ]


def locus_record(node) -> dict:
    """Локус узла скелета: ТОЧНЫЕ (t, точка) и семантические participants.

    Ни `kind`, ни `converging_vertices`, ни `incidences` в ключ локуса не
    входят: ключ — это ГДЕ и КОГДА, а состав — то, что вокруг ключа может
    законно измениться при ремонте.
    """

    time = node.time.canonical()
    return {
        "time": {
            "dividend": [
                int(time.dividend.numerator),
                int(time.dividend.denominator),
            ],
            "divisor": terms(time.divisor),
        },
        "point": {"x": terms(node.point.x), "y": terms(node.point.y)},
        "participants": sorted(
            [int(v) for v in key] for key in node.participants
        ),
        "kind": node.kind.value,
        "kinds": [kind.value for kind in node.kinds],
        "converging_vertices": int(node.converging_vertices),
        "incidence_count": len(node.incidences),
    }


def run_domain_record(entry, *, alpha: float):
    """Одна ступень очереди на готовом домене. Возвращает плоскую запись."""

    from cftuv.envelope_queue_export import run_queue_domain

    patch_id, domain_id, snapshot, request = entry["ready"]
    started = time.perf_counter()
    prepared, domain = run_queue_domain(
        patch_id, domain_id, snapshot, request, str(alpha)
    )
    total = time.perf_counter() - started
    region = prepared.regions[0] if prepared.regions else None
    skeleton = None if region is None else region.skeleton
    return {
        "patch_id": int(patch_id),
        "domain_id": domain_id,
        "stage": "QUEUE",
        "outcome": domain.preparation_outcome,
        "coverage_outcome": domain.coverage_outcome,
        "detail": domain.detail,
        "faces": len(domain.faces),
        "lattice_scale": domain.lattice_scale,
        "counters": {name: int(value) for name, value in prepared.counters},
        "region_id": None if region is None else region.region_id,
        "bridge_outcome": None if region is None else region.bridge_outcome.value,
        "skeleton_outcome": (
            None if skeleton is None else skeleton.outcome.value
        ),
        "skeleton_nodes": 0 if skeleton is None else len(skeleton.nodes),
        "skeleton_proof_status": (
            None if skeleton is None else skeleton.proof_status.value
        ),
        "skeleton_obligations": (
            0 if skeleton is None else len(skeleton.proof_obligations)
        ),
        "face_outcome": (
            None
            if region is None or region.partition is None
            else region.partition.outcome.value
        ),
        "face_detail": (
            None
            if region is None or region.partition is None
            else region.partition.detail
        ),
        "owner_specs": (
            []
            if region is None
            else sorted({name for _, name in region.owner_by_edge})
        ),
        # Владение: КАЖДОЕ ребро-источник со своим спеком, а не только их набор.
        "owner_by_edge": (
            []
            if region is None
            else sorted(
                [[int(v) for v in key], name]
                for key, name in region.owner_by_edge
            )
        ),
        "wall_spans": (
            []
            if region is None
            else sorted([int(v) for v in key] for key in region.wall_spans)
        ),
        "ambiguous_owner_spans": (
            []
            if region is None
            else sorted(
                [int(v) for v in key] for key in region.ambiguous_owner_spans
            )
        ),
        "loci": (
            []
            if skeleton is None
            else [locus_record(node) for node in skeleton.nodes]
        ),
        "total_ms": round(total * 1000.0, 1),
        "prepare_ms": round(domain.prepare_seconds * 1000.0, 1),
        "coverage_ms": round(domain.coverage_seconds * 1000.0, 1),
        "contour_ms": round(domain.contour_seconds * 1000.0, 1),
    }


def run_snapshot(name: str, *, alpha: float, density: int, patch_filter=None):
    """Полный маршрут на слепке. Каждый домен — своя запись, отказ тоже."""

    payload, bm, bundle = bundle_from_field_snapshot(SNAPSHOTS / name)
    selected = selected_edge_ids(payload)
    records = []
    for entry in staged_domains(
        bundle, selected, alpha=alpha, density=density
    ):
        if patch_filter is not None and entry["patch_id"] not in patch_filter:
            continue
        if entry["ready"] is None:
            entry.pop("ready")
            records.append(entry)
            continue
        records.append(run_domain_record(entry, alpha=alpha))
    return {
        "snapshot": name,
        "object_name": payload["object_name"],
        "sha256": snapshot_sha256(name),
        "alpha": alpha,
        "density": density,
        "mesh": {
            "vertices": len(bm.verts),
            "edges": len(bm.edges),
            "faces": len(bm.faces),
            "selected_edges": len(selected),
        },
        "domains": records,
    }


def _main() -> None:
    """CLI: `field_route.py <snapshot> <alpha> <density> [patch,patch] [out]`."""

    name = sys.argv[1]
    alpha = float(sys.argv[2])
    density = int(sys.argv[3])
    patches = (
        frozenset(int(v) for v in sys.argv[4].split(","))
        if len(sys.argv) > 4 and sys.argv[4] not in ("", "-")
        else None
    )
    if name == "building_full_snapshot.json":
        install_building_substitution()
    result = run_snapshot(name, alpha=alpha, density=density, patch_filter=patches)
    text = json.dumps(result, ensure_ascii=False, indent=1)
    if len(sys.argv) > 5:
        Path(sys.argv[5]).write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    _main()
