"""Входы тестов материализатора: настоящий домен очереди из честной метрики.

Два источника, и оба доходят до `prepare_conveyor` и `conveyor_coverage` ТЕМ
ЖЕ путём, что и поле:

* полевые фикстуры (`kernel/fixtures/building_002_*`): снапшот и запрос
  выгружены хостом, метрика — `RationalAffinePlanarMetricV2`;
* синтетика `affine_domain`: многоугольник и маршруты-источники собираются
  `straight_snapshot`, а его старый плоский кадр заменяется дескриптором
  `build_rational_affine_planar_metric` — ровно тем, что строит хост. Без этого
  очередь отвечает `CHART_LATTICE_IS_NOT_DECLARED`: у плоского кадра нет
  сертификата решётки.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import cftuv_envelope as kernel
from cftuv_envelope.exact_sqrt_sum import exact_work_budget
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

from reference_factories import straight_snapshot


FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures"

#: Полевые домены, у которых очередь доходит до `EXACT`. Порядок фиксирован.
FIELD_FIXTURES = (
    "building_002_weighted_normals_v1",
    "building_002_point_contact_v1",
    "building_002_full_selection_v1",
)

TWO_EDGE_FACE = (
    ((0.0, 0.0), (6.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),
)
TWO_EDGE_ROUTE = ((0.0, 0.0), (6.0, 0.0), (10.0, 0.0))


def load_fixture(name: str):
    root = FIXTURE_ROOT / name
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    return snapshot, request


def affine_domain(
    *,
    faces,
    routes,
    alpha: str = "1",
    grid_policy=kernel.GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
):
    """Снапшот и запрос с честной аффинной метрикой вместо плоского кадра."""

    snapshot, request = straight_snapshot(
        faces=faces, source_routes=routes, alpha=alpha
    )
    frame = next(iter(snapshot.surface_metric_descriptors))
    domain = next(iter(snapshot.patch_domains))
    metric = kernel.build_rational_affine_planar_metric(
        source_revision=snapshot.source_revision,
        patch_domain_id=frame.patch_domain_id,
        owner_patch_id=domain.owner_patch_id,
        source_vertices=snapshot.source_vertices,
        source_faces=snapshot.surface_ir.source_faces,
        grid_policy=grid_policy,
    )
    return (
        dataclasses.replace(
            snapshot, surface_metric_descriptors=frozenset({metric})
        ),
        request,
    )


def prepare_and_cover(snapshot, request, *, domain_id=None, alpha=None):
    """`(подготовка, покрытие)` одним вызовом; оба обязаны быть `EXACT`."""

    prepared = prepare_conveyor(snapshot, request, patch_domain_id=domain_id)
    assert prepared.outcome.value == "EXACT", prepared.detail
    coverage = conveyor_coverage(prepared, alpha)
    assert coverage.outcome.value == "EXACT", coverage.detail
    return prepared, coverage


def field_domain(name: str, alpha=None):
    snapshot, request = load_fixture(name)
    return prepare_and_cover(snapshot, request, alpha=alpha) + (request,)


def two_edge_chain_domain(alpha: str = "1", orientation: str = "A_START_TO_END"):
    """Цепь из двух коллинеарных рёбер `(0,0)-(6,0)` и `(6,0)-(10,0)`: один пробег."""

    snapshot, request = affine_domain(
        faces=TWO_EDGE_FACE,
        routes=(
            {
                "name": "source",
                "points": TWO_EDGE_ROUTE,
                "uses": (("use", orientation),),
            },
        ),
        alpha=alpha,
    )
    return prepare_and_cover(snapshot, request) + (request,)


def straight_chain_domain(alpha: str = "1"):
    """Цепь из трёх коллинеарных рёбер: один пробег."""

    snapshot, request = affine_domain(
        faces=(
            (
                (0.0, 0.0),
                (4.0, 0.0),
                (6.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
            ),
        ),
        routes=(
            {
                "name": "source",
                "points": ((0.0, 0.0), (4.0, 0.0), (6.0, 0.0), (10.0, 0.0)),
            },
        ),
        alpha=alpha,
    )
    return prepare_and_cover(snapshot, request) + (request,)


def budget(*, cap: int | None = None):
    return exact_work_budget(stage="MATERIALIZE_TEST", domain_id="test", cap=cap)
