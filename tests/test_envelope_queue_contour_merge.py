"""Слой ОТОБРАЖЕНИЯ очереди: внутренний разделитель одной цепи подавлен.

Полевой дефект, ради которого слой заведён: меш `building.004`, seam-рёбра 18
и 38 точно коллинеарны, общая вершина 6, других швов в ней нет — одна
`PhysicalChain` с внутренней прямой вершиной. Ядро строит по грани на ребро,
разделяя их перпендикулярной дугой из прямой вершины скелета; разбиение при
этом ВЕРНО. Дефект — в том, что контур покрытия рисовал внутреннюю границу
двух граней ОДНОЙ цепи как ребро декали.

Вход здесь синтетический и целочисленный, а не полевой, и это не упрощение:
слияние решается ЧЕТЫРЬМЯ точными предикатами (регион, класс несущей прямой,
`PhysicalChain`, семантика владельца), и синтетика позволяет пошевелить каждый
из них по отдельности. Полевая фикстура рядом отвечает на другой вопрос —
«верное не тронуто» — и стоит своим тестом.

Blender здесь не нужен: слой отображает контракты и с bpy не разговаривает.
"""

from __future__ import annotations

from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
KERNEL_SRC = REPO_ROOT / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv.envelope_queue_export import (  # noqa: E402
    CONTOUR_MERGED_SAME_CHAIN_GROUPS,
    CONTOUR_MERGED_SAME_CHAIN_SEPARATORS,
    CONTOUR_MERGE_BOUNDARY_UNRESOLVED,
    QUEUE_SKELETON_LAYER,
    EnvelopeQueueCoveredFaceV1,
    _integer_line_class,
    _skeleton_segments,
    merge_same_chain_faces,
    queue_domain_payload,
    run_queue_domain,
    source_chain_by_span,
    undirected_span,
)

from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1  # noqa: E402


CHAIN = "physical-chain:one"
OTHER_CHAIN = "physical-chain:two"


def _point(x, y):
    return (SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y)))


def _face(owner, points, *, chain=CHAIN, spec="", instance=None):
    """Грань стадии контура: контур ПРОТИВ часовой, первое ребро — источник.

    Порядок точек тот же, что объявлен ядром (`build_faces`): `points[0]` и
    `points[1]` — концы ребра домена, дальше узлы скелета.
    """

    contour = tuple(_point(x, y) for x, y in points)
    doubled = SqrtSumV1.zero()
    size = len(contour)
    for index in range(size):
        x0, y0 = contour[index]
        x1, y1 = contour[(index + 1) % size]
        doubled = doubled + (x0 * y1) - (x1 * y0)
    return EnvelopeQueueCoveredFaceV1(
        region_id="region",
        owner=owner,
        envelope_spec_id=spec,
        envelope_instance_id=instance,
        points=contour,
        doubled_area=doubled,
        source_chain_id=chain,
    )


#: Грань левого ребра прямой вершины: источник `(0,0) -> (2,0)`.
def _left(**kwargs):
    return _face((0, 0, 2, 0), ((0, 0), (2, 0), (2, 1), (0, 1)), **kwargs)


#: Грань правого ребра той же прямой: источник `(2,0) -> (4,0)`.
#: Общая граница с левой — отрезок `(2,0) - (2,1)`, та самая дуга из прямой
#: вершины, которая и рисовалась лишним поперечным ребром декали.
def _right(**kwargs):
    return _face((2, 0, 4, 0), ((2, 0), (4, 0), (4, 1), (2, 1)), **kwargs)


#: Грань ВЕРТИКАЛЬНОГО ребра той же цепи: источник `(2,1) -> (2,0)`. Границу с
#: левой делит ту же самую, но прямая у неё ДРУГАЯ — это угол внутри цепи.
def _corner(**kwargs):
    return _face((2, 1, 2, 0), ((2, 1), (2, 0), (4, 0), (4, 1)), **kwargs)


#: Грань скрытой опоры веера: ключ пятиместный, отрезка не задаёт.
def _fan(**kwargs):
    return _face((2, 1, 2, 1, 1), ((2, 1), (2, 0), (4, 0), (4, 1)), **kwargs)


def test_integer_line_class_matches_the_kernel_predicate():
    """Класс прямой считается ТОЙ ЖЕ формулой, что у моста, и без float'ов."""

    from cftuv_envelope.wavefront.bridge import line_class

    for owner in ((0, 0, 2, 0), (2, 0, 4, 0), (2, 1, 2, 0), (0, 0, 3, 5)):
        x0, y0, x1, y1 = owner
        a, b = y0 - y1, x1 - x0
        expected = line_class(
            (Fraction(a), Fraction(b), Fraction(a * x0 + b * y0))
        )
        assert _integer_line_class(owner) == expected

    # Ключ веера отрезка не задаёт, вырожденное ребро — тоже.
    assert _integer_line_class((2, 1, 2, 1, 1)) is None
    assert _integer_line_class((2, 1, 2, 1)) is None
    # Встречные рёбра одной прямой в ОДИН класс не попадают: знак сохранён.
    assert _integer_line_class((0, 0, 2, 0)) != _integer_line_class((2, 0, 0, 0))


def test_undirected_span_ignores_traversal_direction_but_not_the_fan():
    assert undirected_span((2, 0, 0, 0)) == undirected_span((0, 0, 2, 0))
    assert undirected_span((2, 1, 2, 1, 1)) is None
    assert undirected_span((2, 1, 2, 1)) is None


def test_same_chain_collinear_faces_merge_into_one_contour():
    """Полевой случай: две грани одной цепи на одной прямой — один контур."""

    merged, stats = merge_same_chain_faces((_left(), _right()))

    assert len(merged) == 1
    assert stats.merged_separators == 1
    assert stats.merged_groups == 1
    assert stats.unresolved_groups == 0

    face = merged[0]
    # Владелец назван наименьшим ключом, второй не исчез — он перечислен.
    assert face.owner == (0, 0, 2, 0)
    assert face.merged_owners == ((2, 0, 4, 0),)
    # Площадь слитого контура ТОЧНО равна сумме площадей: 2 + 2 = 4 (удвоенная
    # площадь прямоугольника 4x1). Это же равенство и есть условие принятия.
    assert face.doubled_area.as_rational() == Fraction(8)
    # Разделитель `(2,0) - (2,1)` в контуре не остался ни одним из концов
    # ПОДРЯД: обход идёт по внешней границе объединения.
    corners = [
        (point[0].as_rational(), point[1].as_rational())
        for point in face.points
    ]
    assert len(corners) == 6
    assert set(corners) == {
        (0, 0), (2, 0), (4, 0), (4, 1), (2, 1), (0, 1),
    }
    following = dict(zip(corners, corners[1:] + corners[:1]))
    assert following[(2, 0)] == (4, 0)
    assert following[(2, 1)] == (0, 1)


def test_three_collinear_faces_of_one_chain_report_two_separators():
    """Одна группа, два разделителя. Одним числом эти два случая не различить."""

    far = _face((4, 0, 6, 0), ((4, 0), (6, 0), (6, 1), (4, 1)))
    merged, stats = merge_same_chain_faces((_left(), _right(), far))

    assert len(merged) == 1
    assert stats.merged_groups == 1
    assert stats.merged_separators == 2
    assert merged[0].merged_owners == ((2, 0, 4, 0), (4, 0, 6, 0))
    assert merged[0].doubled_area.as_rational() == Fraction(12)


def test_collinear_faces_of_different_chains_are_not_merged():
    """Негативный контроль: коллинеарность СЛУЧАЙНАЯ, цепи разные."""

    merged, stats = merge_same_chain_faces(
        (_left(), _right(chain=OTHER_CHAIN))
    )

    assert len(merged) == 2
    assert stats == type(stats)(0, 0, 0)
    assert all(face.merged_owners == () for face in merged)


def test_a_corner_inside_one_chain_is_not_merged():
    """Негативный контроль: цепь одна, а прямая другая — это угол цепи."""

    merged, stats = merge_same_chain_faces((_left(), _corner()))

    assert len(merged) == 2
    assert stats.merged_separators == 0
    assert stats.merged_groups == 0
    assert stats.unresolved_groups == 0


def test_a_fan_support_face_is_not_merged():
    """Негативный контроль: у скрытой опоры веера отрезка нет вовсе."""

    merged, stats = merge_same_chain_faces((_left(), _fan()))

    assert len(merged) == 2
    assert stats.merged_separators == 0
    assert stats.merged_groups == 0


def test_faces_of_different_owners_are_not_merged():
    """Негативный контроль: цепь и прямая одни, а огибающие разные."""

    by_spec, stats = merge_same_chain_faces(
        (_left(spec="strip-a"), _right(spec="strip-b"))
    )
    assert len(by_spec) == 2
    assert stats.merged_groups == 0

    # Имя ЭКЗЕМПЛЯРА тоже различает: оно стоит на эффективной alpha, и две
    # разные эффективные alpha — это две огибающие, а не одна.
    by_instance, stats = merge_same_chain_faces(
        (
            _left(spec="strip-a", instance="strip-a@1"),
            _right(spec="strip-a", instance="strip-a@2"),
        )
    )
    assert len(by_instance) == 2
    assert stats.merged_groups == 0


def test_faces_without_a_named_chain_are_not_merged():
    """Цепь не названа — условие (c) не доказано, значит слияния нет."""

    merged, stats = merge_same_chain_faces(
        (_left(chain=None), _right(chain=None))
    )

    assert len(merged) == 2
    assert stats.merged_groups == 0
    assert stats.unresolved_groups == 0


def test_collinear_same_chain_faces_that_do_not_touch_are_not_a_failure():
    """Не смежные грани — не неудача слияния, и кричать о них нельзя.

    Два коллинеарных ребра одной цепи могут стоять по разные стороны домена.
    Сливать там нечего; `CONTOUR_MERGE_BOUNDARY_UNRESOLVED` обязан остаться
    именем настоящей аномалии, а не срабатывать на законном входе.
    """

    apart = _face((10, 0, 12, 0), ((10, 0), (12, 0), (12, 1), (10, 1)))
    merged, stats = merge_same_chain_faces((_left(), apart))

    assert len(merged) == 2
    assert stats.merged_groups == 0
    assert stats.unresolved_groups == 0


def test_a_boundary_that_does_not_prove_itself_stays_unmerged_and_counted():
    """Смежность есть, а обход не сложился: грани остаются, число растёт.

    Вход — аномалия разбиения: два ребра-источника одной цепи претендуют на
    ОДНУ И ТУ ЖЕ грань, то есть одно полуребро приходит дважды в одном
    направлении. Молчаливого второго порядка обхода здесь нет: группа остаётся
    неслитой и названа числом.
    """

    twin = _face((4, 0, 6, 0), ((2, 0), (4, 0), (4, 1), (2, 1)))
    merged, stats = merge_same_chain_faces((_left(), _right(), twin))

    assert len(merged) == 3
    assert stats.merged_groups == 0
    assert stats.merged_separators == 0
    assert stats.unresolved_groups == 1


def test_a_merged_contour_that_is_not_simple_is_refused_by_the_kernel_bound():
    """Обход сложился, а контур перекручен — граница 1 ядра его отвергает.

    Проверяется именно та проверка, которая тождеством НЕ является: площадь на
    перекрученном контуре сходится (сокращение вычитает ноль), а
    трансверсальное самопересечение — нет.
    """

    from cftuv_envelope.wavefront.faces import contour_crossings

    twisted = _face(
        (2, 0, 4, 0), ((2, 0), (4, 0), (4, 2), (3, -1), (2, 1))
    )
    merged, stats = merge_same_chain_faces((_left(), twisted))

    assert stats.unresolved_groups == 1
    assert stats.merged_groups == 0
    assert len(merged) == 2
    # Перекрут лежит во ВХОДЕ, и это доказано тем же предикатом ядра: без него
    # тест мерил бы отказ, не назвав его причины.
    assert contour_crossings(twisted.points)


def test_the_suppressed_separator_stays_visible_on_the_skeleton_layer():
    """Слои разделяют роли: подавлено ребро ДЕКАЛИ, дуга скелета осталась.

    `_skeleton_segments` читает ПОЛНЫЕ грани разбиения (`region.partition`) и
    про слияние контуров покрытия не знает вовсе. Поэтому разделитель
    `(2,0) - (2,1)` исчезает только со слоя владельца и остаётся на слое
    скелета — там, где он и есть настоящая дуга.
    """

    region = SimpleNamespace(
        region_id="region",
        partition=SimpleNamespace(
            faces=(_left(), _right()),
        ),
    )
    segments = _skeleton_segments(region, 1)

    assert all(item.layer == QUEUE_SKELETON_LAYER for item in segments)
    drawn = {tuple(sorted(item.points)) for item in segments}
    separator = tuple(sorted(((2.0, 0.0), (2.0, 1.0))))
    assert separator in drawn

    # И того же отрезка нет ни одной СТОРОНОЙ слитого контура покрытия.
    merged, _stats = merge_same_chain_faces((_left(), _right()))
    contour = [
        (float(point[0].as_rational()), float(point[1].as_rational()))
        for point in merged[0].points
    ]
    sides = {
        tuple(sorted((contour[index], contour[(index + 1) % len(contour)])))
        for index in range(len(contour))
    }
    assert separator not in sides


# --- стадия QUEUE_CONTOUR целиком, на НАСТОЯЩЕМ скелете ядра ---------------


def _straight_vertex_partition():
    """Домен с ПРЯМОЙ вершиной между двумя коллинеарными рёбрами-источниками.

    Это и есть полевая конфигурация `building.004` в наименьшем виде: вершина
    `(2, 0)` — та самая вершина 6, рёбра `(0,0)-(2,0)` и `(2,0)-(4,0)` — рёбра
    18 и 38. Скелет, грани и усечение считает САМО ЯДРО; синтетичен только
    домен.
    """

    from cftuv_envelope.wavefront.faces import build_faces
    from cftuv_envelope.wavefront.polygon import (
        LoopV1,
        PolygonV1,
        with_edge_speeds,
    )
    from cftuv_envelope.wavefront.skeleton import build_skeleton

    polygon = with_edge_speeds(
        PolygonV1.build(LoopV1(((0, 0), (2, 0), (4, 0), (4, 3), (0, 3)))),
        (((0, 0), (2, 0), 4), ((2, 0), (4, 0), 4)),
    )
    skeleton = build_skeleton(polygon)
    return polygon, skeleton, build_faces(polygon, skeleton)


def _domain_with_chains(chain_by_edge):
    """Домен с провенансом: `PhysicalChain` у каждого сегмента граничной петли.

    `chain_by_edge` — цепь по индексу ребра внешней петли. Разные имена в нём
    и дают негативный контроль «коллинеарны, но цепи разные» на том же самом
    пути, что и положительный случай.
    """

    from cftuv_envelope.reference.planar_types import (
        BoundedSupportSegment,
        ExactPlanarPoint,
        PlanarLoop,
        PlanarRegion,
    )

    nodes = ((0, 0), (2, 0), (4, 0), (4, 3), (0, 3))
    segments = tuple(
        BoundedSupportSegment(
            segment_id=f"segment-{index}",
            start=ExactPlanarPoint.from_values(*nodes[index]),
            end=ExactPlanarPoint.from_values(*nodes[(index + 1) % len(nodes)]),
            support_ids=frozenset(),
            boundary_constraint_ids=frozenset(),
            provenance=SimpleNamespace(
                physical_chain_ids=frozenset({chain_by_edge[index]})
            ),
            start_constructions=frozenset(),
            end_constructions=frozenset(),
        )
        for index in range(len(nodes))
    )
    region = PlanarRegion(
        region_id="region",
        outer=PlanarLoop("outer", segments),
        holes=(),
    )
    return SimpleNamespace(domain_regions=(region,))


def _queue_domain(chain_by_edge, alpha=Fraction(1)):
    """Полный путь стадии контура: ядро считает, хост отображает."""

    from cftuv_envelope.robust.grid import GridSpecV1
    from cftuv_envelope.wavefront.bridge import BridgeOutcome, BridgeReportV1
    from cftuv_envelope.wavefront.conveyor import (
        ConveyorCoverageV1,
        ConveyorFaceCoverageV1,
        ConveyorOutcome,
        ConveyorPreparationV1,
        ConveyorRegionCoverageV1,
        PreparedRegionV1,
    )
    from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at
    from cftuv_envelope.wavefront.faces import FaceOutcome
    from cftuv_envelope.wavefront.skeleton import SkeletonOutcome

    from cftuv.envelope_queue_export import build_queue_domain

    polygon, skeleton, partition = _straight_vertex_partition()
    assert partition.outcome is FaceOutcome.EXACT
    assert len(partition.faces) == 2

    bridge = BridgeReportV1(
        outcome=BridgeOutcome.EXACT,
        findings=(),
        polygon=polygon,
        edge_count=5,
        law_count=2,
        matched_edge_count=2,
        off_lattice_points=(),
        non_unit_speed_laws=(),
        unmatched_laws=(),
        surplus_laws=(),
        owner_by_edge=(),
        lattice_scale=1,
    )
    region = PreparedRegionV1(
        region_id="region",
        bridge=bridge,
        skeleton=skeleton,
        partition=partition,
        bridge_outcome=BridgeOutcome.EXACT,
        skeleton_outcome=SkeletonOutcome.EXACT,
        face_outcome=FaceOutcome.EXACT,
        owner_by_edge=(),
        wall_spans=(),
        wall_edge_count=3,
        ambiguous_owner_spans=(
            (0, 0, 2, 0),
            (2, 0, 4, 0),
        ),
        degraded_miter_corners=(),
    )
    prepared = ConveyorPreparationV1(
        outcome=ConveyorOutcome.EXACT,
        regions=(region,),
        lattice=GridSpecV1(1),
        law_names=("strip-spec:one",),
        counters=(),
        timings=(),
        domain=_domain_with_chains(chain_by_edge),
    )
    covered = coverage_at(partition, alpha)
    coverage = ConveyorCoverageV1(
        outcome=ConveyorOutcome.EXACT,
        alpha=alpha,
        lattice_alpha=alpha,
        regions=(
            ConveyorRegionCoverageV1(
                region_id="region",
                outcome=CoverageOutcome.EXACT,
                faces=tuple(
                    # Владелец НЕ определён — ровно как в поле: класс
                    # коллинеарных рёбер даёт `AMBIGUOUS_OWNER_EDGES=2` и
                    # `NAMED_OWNERS=0`.
                    ConveyorFaceCoverageV1(
                        region_id="region",
                        owner=face.owner,
                        envelope_spec_id="",
                        envelope_instance_id=None,
                        doubled_area=face.doubled_area,
                    )
                    for face in covered.faces
                ),
                doubled_area=covered.doubled_area,
                polygon_doubled_area=covered.polygon_doubled_area,
                wall_spans=(),
            ),
        ),
        doubled_area=covered.doubled_area,
        polygon_doubled_area=covered.polygon_doubled_area,
        preparation=prepared,
        counters=(),
        timings=(),
    )
    return build_queue_domain(
        1,
        "patch-domain:one",
        prepared,
        coverage,
        prepare_seconds=0.0,
        coverage_seconds=0.0,
    )


ONE_CHAIN = ("chain:seam", "chain:seam", "chain:other", "chain:other", "chain:other")
TWO_CHAINS = ("chain:a", "chain:b", "chain:other", "chain:other", "chain:other")


def test_the_straight_vertex_of_one_chain_loses_its_separator_on_the_face_layer():
    """Полевой дефект целиком: два ребра одной цепи — одна грань декали.

    Ядро по-прежнему строит ДВЕ грани (`CONVEYOR_FACES = 2`, оба ребра на одной
    несущей прямой), и это верная внутренняя структура. Слой отображения хоста
    рисует их одним контуром и говорит это числом.
    """

    domain = _queue_domain(ONE_CHAIN)

    assert len(domain.faces) == 1
    face = domain.faces[0]
    assert face.owner == (0, 0, 2, 0)
    assert face.merged_owners == ((2, 0, 4, 0),)
    assert face.source_chain_id == "chain:seam"
    assert dict(domain.host_counters) == {
        CONTOUR_MERGED_SAME_CHAIN_SEPARATORS: 1,
        CONTOUR_MERGED_SAME_CHAIN_GROUPS: 1,
        CONTOUR_MERGE_BOUNDARY_UNRESOLVED: 0,
    }
    # Дуга скелета из прямой вершины на своём слое ОСТАЛАСЬ: слои разделяют
    # роли, подавлено ребро декали, а не граница разбиения.
    assert any(
        item.layer == QUEUE_SKELETON_LAYER and item.label == "SKELETON_ARC"
        for item in domain.segments
    )
    separator = {(2.0, 0.0)}
    assert any(
        separator & set(item.points)
        for item in domain.segments
        if item.layer == QUEUE_SKELETON_LAYER
    )


def test_the_same_straight_vertex_across_two_chains_keeps_its_separator():
    """Негативный контроль на ТОМ ЖЕ пути: геометрия та же, цепи разные."""

    domain = _queue_domain(TWO_CHAINS)

    assert len(domain.faces) == 2
    assert {face.owner for face in domain.faces} == {
        (0, 0, 2, 0),
        (2, 0, 4, 0),
    }
    assert all(face.merged_owners == () for face in domain.faces)
    assert dict(domain.host_counters) == {
        CONTOUR_MERGED_SAME_CHAIN_SEPARATORS: 0,
        CONTOUR_MERGED_SAME_CHAIN_GROUPS: 0,
        CONTOUR_MERGE_BOUNDARY_UNRESOLVED: 0,
    }


def test_the_merged_contour_keeps_the_area_the_kernel_measured():
    """Нарисованная область равна измеренной — на обеих alpha, точно."""

    for alpha in (Fraction(1, 2), Fraction(1), Fraction(3)):
        merged = _queue_domain(ONE_CHAIN, alpha)
        split = _queue_domain(TWO_CHAINS, alpha)
        assert len(merged.faces) == 1 and len(split.faces) == 2
        assert merged.faces[0].doubled_area == sum(
            face.doubled_area for face in split.faces
        )


# --- полевой контроль: верное не тронуто -----------------------------------

FIXTURE = (
    REPO_ROOT / "kernel" / "fixtures" / "building_002_point_contact_v1"
)


def _field_domain(alpha: str = "0.25"):
    import cftuv_envelope as kernel

    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (FIXTURE / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (FIXTURE / "decal_request.json").read_bytes()
    )
    domain_id = next(iter(snapshot.patch_domains)).patch_domain_id.value
    return run_queue_domain(7, domain_id, snapshot, request, alpha)


def test_host_names_the_chain_of_every_source_edge_of_the_field_domain():
    """Цепь ребра-источника выводится, и это не теория: 12 рёбер из 12.

    Условие (c) без этой выгрузки было бы недоказуемо всегда, то есть слияние
    молча никогда бы не срабатывало. Поэтому число здесь заморожено.
    """

    prepared, domain = _field_domain()
    spans = source_chain_by_span(prepared)

    region_id = prepared.regions[0].region_id
    assert len(spans[region_id]) == 12
    assert all(value for value in spans[region_id].values())

    # У трёх юбок цепь названа, у двух вееров её нет — и это разные ответы.
    named = [face for face in domain.faces if face.source_chain_id]
    unnamed = [face for face in domain.faces if not face.source_chain_id]
    assert len(named) == 3 and all(
        face.envelope_spec_id.startswith("strip-spec:") for face in named
    )
    assert len(unnamed) == 2 and all(
        face.envelope_spec_id.startswith("angular-spec:") for face in unnamed
    )


def test_the_field_domain_merges_nothing_and_says_so_by_number():
    """«Чинит сломанное» без «не трогает верное» было бы другим правилом."""

    _prepared, domain = _field_domain()

    assert dict(domain.host_counters) == {
        CONTOUR_MERGED_SAME_CHAIN_SEPARATORS: 0,
        CONTOUR_MERGED_SAME_CHAIN_GROUPS: 0,
        CONTOUR_MERGE_BOUNDARY_UNRESOLVED: 0,
    }
    assert len(domain.faces) == 5
    assert all(face.merged_owners == () for face in domain.faces)


def test_host_counters_reach_the_sidecar_under_their_own_key():
    """Числа хоста названы отдельно от чисел ядра, а не подмешаны к ним."""

    _prepared, domain = _field_domain()
    payload = queue_domain_payload(domain)

    host = {item["name"]: item["value"] for item in payload["host_counters"]}
    assert host == {
        CONTOUR_MERGED_SAME_CHAIN_SEPARATORS: 0,
        CONTOUR_MERGED_SAME_CHAIN_GROUPS: 0,
        CONTOUR_MERGE_BOUNDARY_UNRESOLVED: 0,
    }
    kernel_counters = {item["name"] for item in payload["counters"]}
    assert not (kernel_counters & set(host))
