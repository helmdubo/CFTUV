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


def with_affine_metric(
    snapshot,
    *,
    grid_policy=kernel.GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
    planarity_policy=kernel.PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1,
    with_triangles=True,
    near_planar_lift_law=None,
    near_planar_frame_policy=None,
    curvature_ladder=None,
):
    """Снапшот с честной аффинной метрикой вместо его плоского кадра.

    `with_triangles` — как хост: строитель получает треугольники поверхности и
    пишет сертификат искажения ширины у near-planar домена. Без них записи нет.
    """

    frame = next(iter(snapshot.surface_metric_descriptors))
    domain = next(iter(snapshot.patch_domains))
    metric = kernel.build_rational_affine_planar_metric(
        source_revision=snapshot.source_revision,
        patch_domain_id=frame.patch_domain_id,
        owner_patch_id=domain.owner_patch_id,
        source_vertices=snapshot.source_vertices,
        source_faces=snapshot.surface_ir.source_faces,
        planarity_policy=planarity_policy,
        grid_policy=grid_policy,
        surface_triangles=(
            snapshot.surface_ir.surface_triangles if with_triangles else None
        ),
        **(
            {}
            if near_planar_lift_law is None
            else {"near_planar_lift_law": near_planar_lift_law}
        ),
        **(
            {}
            if near_planar_frame_policy is None
            else {"near_planar_frame_policy": near_planar_frame_policy}
        ),
        **(
            {}
            if curvature_ladder is None
            else {"curvature_ladder": curvature_ladder}
        ),
    )
    return dataclasses.replace(
        snapshot, surface_metric_descriptors=frozenset({metric})
    )


def affine_domain(
    *,
    faces,
    routes,
    alpha: str = "1",
    grid_policy=kernel.GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
    planarity_policy=kernel.PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1,
    lift=None,
    with_triangles=True,
    near_planar_lift_law=None,
    near_planar_frame_policy=None,
):
    """Снапшот и запрос с честной аффинной метрикой вместо плоского кадра.

    `lift` — `{индекс_вершины: dz}`: смещение вершины `v<индекс>` по Z. Домен с
    ненулевым `lift` и политикой `NEAR_PLANAR_PROJECTION_V1` — near-planar.
    """

    snapshot, request = straight_snapshot(
        faces=faces, source_routes=routes, alpha=alpha
    )
    if lift:
        snapshot = dataclasses.replace(
            snapshot,
            source_vertices=frozenset(
                dataclasses.replace(
                    item,
                    position=dataclasses.replace(
                        item.position,
                        z=item.position.z + lift.get(int(item.vertex_id.value[1:]), 0.0),
                    ),
                )
                for item in snapshot.source_vertices
            ),
        )
    return (
        with_affine_metric(
            snapshot,
            grid_policy=grid_policy,
            planarity_policy=planarity_policy,
            with_triangles=with_triangles,
            near_planar_lift_law=near_planar_lift_law,
            near_planar_frame_policy=near_planar_frame_policy,
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


#: Параллелограмм, а не прямоугольник: карта домена КОСАЯ. Базис — `A = (10,0,0)`,
#: `B = (13,8,0)` (первая вершина вне прямой `v0 v1`), Грам `[[100,130],[130,233]]`:
#: ни ортогональности, ни единичных длин. Единица решётки по `u` и по `v` — разные
#: физические длины, и метрическая ошибка станции на такой карте видна сразу.
SKEW_FACE = ((0.0, 0.0), (10.0, 0.0), (13.0, 8.0), (3.0, 8.0))
SKEW_BOTTOM = ((0.0, 0.0), (10.0, 0.0))
SKEW_SIDE = ((10.0, 0.0), (13.0, 8.0))


def skew_chain_domain(alpha: str = "1", route=SKEW_BOTTOM):
    """Одна цепь на косой карте: нижнее ребро либо (route=SKEW_SIDE) боковое."""

    snapshot, request = affine_domain(
        faces=(SKEW_FACE,),
        routes=({"name": "source", "points": route},),
        alpha=alpha,
    )
    return prepare_and_cover(snapshot, request) + (request,)


def l_chains_domain(alpha: str = "1"):
    """Две цепи углом «Г» в квадрате: `(0,0)-(10,0)` и `(10,0)-(10,10)`.

    Это и есть форма «Г», которую ядро принимает: ОДНА цепь с изломом в
    `straight_snapshot` не собирается (`SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR`
    — излом цепи объявляется углом с сертификатом), поэтому каждое плечо — своя
    `PhysicalChain` со своим `ChainUse`. Карта косая: базис `A = (10,0,0)`,
    `B = (10,10,0)`.
    """

    snapshot, request = affine_domain(
        faces=(((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),),
        routes=(
            {"name": "arm0", "points": ((0.0, 0.0), (10.0, 0.0))},
            {"name": "arm1", "points": ((10.0, 0.0), (10.0, 10.0))},
        ),
        alpha=alpha,
    )
    return prepare_and_cover(snapshot, request) + (request,)


RING_OUTER = ((0.0, 0.0), (12.0, 0.0), (12.0, 12.0), (0.0, 12.0))
RING_INNER = ((4.0, 4.0), (8.0, 4.0), (8.0, 8.0), (4.0, 8.0))


def ring_domain(alpha: str = "1"):
    """Домен с ДЫРОЙ: квадратное кольцо из четырёх трапеций, источник — вся дыра.

    Четыре стороны внутреннего контура — четыре цепи (замкнутая цепь на
    `straight_snapshot` не принимается: `PLANAR_CHAIN_SUPPORT_NOT_LINEAR`).
    """

    outer, inner = RING_OUTER, RING_INNER
    faces = tuple(
        (outer[i], outer[(i + 1) % 4], inner[(i + 1) % 4], inner[i]) for i in range(4)
    )
    routes = tuple(
        {"name": f"side{i}", "points": (inner[i], inner[(i + 1) % 4])}
        for i in range(4)
    )
    snapshot, request = affine_domain(faces=faces, routes=routes, alpha=alpha)
    return prepare_and_cover(snapshot, request) + (request,)


def near_planar_domain(alpha: str = "1"):
    """Near-planar домен: вершина `v3` приподнята на 0.002 над плоскостью остальных.

    Подъём БОЛЬШЕ полушага исходной сетки (1/4096): меньший сетка округляет в
    ноль, и домен остаётся точно плоским. Сертификат — `NearPlanarProjection...`,
    метрика строится по проекциям на точную плоскость.
    """

    snapshot, request = affine_domain(
        faces=(SKEW_FACE,),
        routes=({"name": "source", "points": SKEW_BOTTOM},),
        alpha=alpha,
        planarity_policy=kernel.PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        lift={3: 0.002},
    )
    return prepare_and_cover(snapshot, request) + (request,)


def budget(*, cap: int | None = None):
    return exact_work_budget(stage="MATERIALIZE_TEST", domain_id="test", cap=cap)


def assemble_polygon_batch(polygon, alpha, *, plane=None, diagnostics=None):
    """Батч прямо из разбиения многоугольника корпуса: без снапшота и метрики.

    Корпус стенда (`wavefront_cases.named_corpus`) — решёточные многоугольники
    без цепей и без дескриптора, поэтому станции здесь простейшие: у каждого
    ребра-источника СВОЙ пробег с единичной метрикой, у каждого веера свой кадр
    с нулевой станцией. Всё, что идёт после кадров (вершины, факты, UV,
    тесселяция, цепи, батч), — те же функции, что у `materialize_domain`.
    Возвращает `(батч, грани с кадрами)` либо `None`, если разбиение не `EXACT`.
    `plane` и `diagnostics` (функция без аргументов) подменяют подъём и запись
    диагностик — для теста порядка «сначала подъём, потом диагностика».
    """

    from dataclasses import replace
    from fractions import Fraction
    from types import SimpleNamespace

    from cftuv_envelope.canonical import geometry_batch_semantic_digest
    from cftuv_envelope.contracts.envelopes import StationModelId
    from cftuv_envelope.ids import PatchDomainId, SemanticDigestValue, SourceRevision
    from cftuv_envelope.materialize import assemble
    from cftuv_envelope.materialize.coalesce import CoveredFaceV1
    from cftuv_envelope.materialize.frames import FrameFaceV1
    from cftuv_envelope.materialize.lift import PlaneLiftV1
    from cftuv_envelope.materialize.stations import StationRunV1
    from cftuv_envelope.wavefront.coverage import coverage_at
    from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces
    from cftuv_envelope.wavefront.skeleton import SkeletonOutcome, build_skeleton
    from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

    from reference_factories import reference_request

    guard = budget()
    skeleton = build_skeleton(polygon, work_budget=guard)
    if skeleton.outcome is not SkeletonOutcome.EXACT:
        return None
    partition = build_faces(polygon, skeleton, guard)
    if partition.outcome is not FaceOutcome.EXACT:
        return None
    covered = coverage_at(partition, Fraction(alpha), guard)
    lines = {face.owner: face.line for face in partition.faces}
    frame_faces = []
    for item in covered.faces:
        if len(item.points) < 3:
            continue
        face = CoveredFaceV1(
            region_id="corpus",
            owner=tuple(item.owner),
            envelope_spec_id="spec",
            envelope_instance_id="instance",
            points=tuple(item.points),
            doubled_area=item.doubled_area,
        )
        fan = len(item.owner) == 5
        dx, dy = (
            (0, 0)
            if fan
            else (item.owner[2] - item.owner[0], item.owner[3] - item.owner[1])
        )
        squared = Fraction(dx * dx + dy * dy)
        run = StationRunV1(
            run_id=f"run:{item.owner}",
            chain_id="chain",
            chain_use_id="use",
            run_index=0,
            origin=(item.owner[0], item.owner[1]),
            direction=(dx, dy),
            s_origin=SqrtSumV1.zero(),
            inverse_length=(
                SqrtSumV1.zero()
                if not squared
                else SqrtSumV1.radical(1, Fraction(1) / squared, guard)
            ),
            covector=(Fraction(dx), Fraction(dy)),
            physical_edge_ids=(),
            lineage_ids=(),
        )
        frame_faces.append(
            FrameFaceV1(
                face=face,
                line=lines[item.owner],
                claim_key="claim",
                frame_key=run.run_id,
                station_model=(
                    StationModelId.CONSTANT_PHYSICAL_ENDPOINT_S
                    if fan
                    else StationModelId.SEMANTIC_CHAIN_USE_S
                ),
                run=run,
                fan_station=SqrtSumV1.zero() if fan else None,
                physical_edge_ids=frozenset(),
                chain_use_ids=frozenset(),
                chain_ids=frozenset(),
            )
        )
    table = SimpleNamespace(node_vertex_ids={})
    layout = assemble.Layout(frame_faces)
    cycles, points = assemble.intern_vertices(
        [("corpus", item) for item in frame_faces], table
    )
    lattice_alpha = Fraction(alpha)
    facts = assemble.station_values(
        frame_faces, cycles, layout, table, lattice_alpha, guard
    )
    triangles = assemble.tessellate_faces(frame_faces, cycles, guard, reverse=False)
    positions = assemble.lift_vertices(
        points,
        plane
        or PlaneLiftV1(
            (Fraction(0),) * 3,
            (Fraction(1), Fraction(0), Fraction(0)),
            (Fraction(0), Fraction(1), Fraction(0)),
            1,
        ),
    )
    batch = assemble.assemble_batch(
        frame_faces=frame_faces,
        cycles=cycles,
        positions=positions,
        polygons=triangles,
        facts=facts,
        layout=layout,
        scale=1,
        lattice_alpha=lattice_alpha,
        edge_faces={},
        request=reference_request(),
        source_revision=SourceRevision("corpus"),
        patch_domain_id=PatchDomainId("corpus"),
        contract_versions=("cftuv.envelope.geometry_batch.v1",),
        diagnostics=diagnostics or (lambda: ()),
    )
    batch = replace(
        batch,
        semantic_digest=SemanticDigestValue(
            geometry_batch_semantic_digest(batch).sha256_hex
        ),
    )
    return batch, frame_faces
