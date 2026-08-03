"""Ворота плоской редукции: ДВЕ КОЛОНКИ В ОДНОМ ПРОГОНЕ, без замороженных чисел.

Что здесь доказывается. `SurfaceArrivalComplexV1` — форма ответа для бэкендов,
которых ещё нет, и без единого производителя она была бы слухом. Производитель,
уже доказанный P0-1..P0-3, один — планарная очередь, и требование к контракту
формулируется исполняемо:

    КОЛОНКА A — планарное ядро (`build_skeleton` -> `build_faces` ->
    `coverage_at`), эталон.
    КОЛОНКА C — тот же результат, ПЕРЕЛОЖЕННЫЙ в `SurfaceArrivalComplexV1` и
    прочитанный ОБРАТНО из одних только байт комплекса.

Обе колонки считаются В ОДНОМ ПРОГОНЕ, и ни одного дайджеста здесь не
заморожено: до единой интеграционной вершины замораживать нечего — заморозка
превратила бы сверку двух колонок в сверку с прошлогодним файлом (`DECISIONS`,
2026-08-03, «AUTH развилок S0/S2»).

Проекции ОСЕЙ ПЕРЕИСПОЛЬЗУЮТСЯ из `test_wavefront_weighted_wall_differential`,
а не переписываются: колонка, читающая свою копию проекции, доказывала бы
согласие двух копий. Девять осей P0-3 те же, что у дифференциала
MOTORCYCLE/EXHAUSTIVE, плюс три собственные оси surface-контракта —
`triangle_owner_map`, `cut_locus`, `station_witnesses`.

Чего эти ворота НЕ доказывают, и это записано, а не подразумевается: что
непланарный вход разойдётся с планарным ядром (G-N1). Непланарного бэкенда не
существует, множество таких входов пусто, и тест на пустом множестве зеленеет
молча. Поэтому G-N1 стоит именованным placeholder'ом со skip-причиной
`SURFACE_BACKEND_NOT_IMPLEMENTED`, а не отсутствует.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import replace
from decimal import Decimal
from fractions import Fraction
from functools import cache
import json

import pytest

from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.analysis import SurfaceRegime
from cftuv_envelope.contracts.metric import ExactRationalV1
from cftuv_envelope.contracts.surface_adjacency import (
    SurfaceAdjacencyDigestValue,
    SurfaceAdjacencyScopeV1,
)
from cftuv_envelope.contracts.surface_arrival import (
    SURFACE_ARRIVAL_COMPLEX_V1_SCHEMA,
    ArrivalCellId,
    ArrivalCellLawV1,
    ArrivalCellV1,
    ArrivalMetricDigestValue,
    ArrivalTieResolutionV1,
    ExactAlgebraicSumV1,
    SurfaceArrivalBackendV1,
    SurfaceArrivalComplexV1,
    SurfaceArrivalContractError,
    SurfaceMetricRefV1,
    arrival_metric_digest,
    exact_rational,
    require_alpha_within_horizon,
    validate_arrival_against_metric,
)
from cftuv_envelope.contracts.metric import GridSnappingLawV1
from cftuv_envelope.contracts.surface_metric_v2 import (
    NamedEpsilonV1,
    SurfaceAdjacencyRefV1,
    SurfaceMetricPrecisionTierV1,
)
from cftuv_envelope.ids import LawId, PatchDomainId, SourceRevision
from cftuv_envelope.surface_adjacency_compat import read_surface_adjacency
from cftuv_envelope.surface_metric_build import build_surface_metric_v2
from cftuv_envelope.surface_arrival_planar_queue import (
    PLANAR_CHART_CELL_PREFIX,
    build_planar_queue_arrival_complex,
    edge_key_of,
    from_algebraic_sum,
    from_algebraic_time,
    owner_key_of,
    rebuild_face_partition,
    rebuild_skeleton_node,
)
from cftuv_envelope.wavefront.digest import node_record
from cftuv_envelope.wavefront.faces import build_faces
from cftuv_envelope.wavefront.polygon import (
    PolygonV1,
    with_edge_speeds,
    with_vertex_fans,
)
from cftuv_envelope.wavefront.skeleton import SplitSearch, build_skeleton

from surface_adjacency_factories import two_triangle_quad, vertex
from wavefront_cases import named_corpus, partial_source_corpus
from weighted_wall_differential_cases import weighted_wall_differential_corpus
from test_wavefront_weighted_wall_differential import (
    ALPHAS,
    AXES,
    _coverage_record,
    _reading,
)


DOMAIN = PatchDomainId("flat-reduction:domain")
REVISION = SourceRevision("flat-reduction:revision")

#: Горизонт комплекса РАВЕН наибольшей сверяемой alpha, а не «с запасом»:
#: запас означал бы, что дверь G-N9 никогда не проверяется настоящим запросом.
ALPHA_HORIZON = exact_rational(max(ALPHAS))


def flat_reduction_corpus() -> tuple[tuple[str, PolygonV1], ...]:
    """Три корпуса подряд, порядок фиксирован — значит, воспроизводим.

    Именованный корпус даёт полный источник, корпус частичного источника —
    стены, корпус взвешенной стены — веса, дыры, зеркало, веера и
    одновременность. Перечень берётся целиком и стендом не дополняется
    (`DECISIONS.md`, 2026-07-27).
    """

    entries = [(f"named:{name}", polygon) for name, polygon in named_corpus()]
    entries += [
        (f"partial:{name}", polygon) for name, polygon in partial_source_corpus()
    ]
    entries += [
        (f"weighted:{case.name}", case.polygon)
        for case in weighted_wall_differential_corpus()
    ]
    return tuple(entries)


CORPUS = flat_reduction_corpus()
BY_NAME = dict(CORPUS)
EXTRA_AXES = ("triangle_owner_map", "cut_locus", "station_witnesses")


# --------------------------------------------------------------------------
# Две колонки
# --------------------------------------------------------------------------


def _complex_of(polygon, skeleton, partition) -> SurfaceArrivalComplexV1:
    return build_planar_queue_arrival_complex(
        polygon,
        skeleton,
        partition,
        patch_domain_id=DOMAIN,
        source_revision=REVISION,
        alpha_horizon=ALPHA_HORIZON,
    )


@cache
def _columns(name: str):
    """Обе колонки за один прогон одного и того же входа."""

    polygon = BY_NAME[name]
    skeleton = build_skeleton(polygon)
    partition = build_faces(polygon, skeleton)
    complex_ = _complex_of(polygon, skeleton, partition)
    return polygon, skeleton, partition, complex_


def _obligation_projection(complex_: SurfaceArrivalComplexV1) -> Counter:
    """Та же стабильная запись долга, что у P0-3: runtime-ids исключены."""

    return Counter(
        (
            obligation.cause.value,
            obligation.disposition.value,
            from_algebraic_time(obligation.level),
            tuple(edge_key_of(item) for item in obligation.participant_edge_keys),
            tuple(edge_key_of(item) for item in obligation.target_edge_keys),
            None if obligation.event_kind is None else obligation.event_kind.value,
        )
        for obligation in complex_.proof_obligations
    )


def _reading_from_complex(complex_: SurfaceArrivalComplexV1) -> dict:
    """Девять осей ИЗ ОДНИХ БАЙТ КОМПЛЕКСА. Планарные объекты сюда не входят."""

    partition = rebuild_face_partition(complex_)
    nodes = tuple(rebuild_skeleton_node(event) for event in complex_.events)
    return {
        "outcome": complex_.front_outcome.value,
        "node_record": Counter(
            json.dumps(node_record(node), sort_keys=True) for node in nodes
        ),
        "times_points": Counter(
            (
                node.time.canonical().dividend,
                node.time.canonical().divisor.terms,
                node.point.x.terms,
                node.point.y.terms,
            )
            for node in nodes
        ),
        "participants": Counter(node.participants for node in nodes),
        "face_owners": (
            partition.outcome.value,
            Counter(face.owner for face in partition.faces),
        ),
        "face_areas": (
            partition.outcome.value,
            Counter(
                (face.owner, face.doubled_area.terms) for face in partition.faces
            ),
            partition.doubled_area.terms,
            partition.polygon_doubled_area,
        ),
        "coverage": _coverage_record(partition),
        "proof_status": complex_.proof_status.value,
        "proof_obligations": _obligation_projection(complex_),
    }


# --------------------------------------------------------------------------
# Три собственные оси surface-контракта
# --------------------------------------------------------------------------


def _owner_map_reference(partition) -> dict[str, tuple[str, ...]]:
    return {
        PLANAR_CHART_CELL_PREFIX + owner_key_of(face.owner).value: (
            owner_key_of(face.owner).value,
        )
        for face in partition.faces
    }


def _owner_map_from_complex(complex_) -> dict[str, tuple[str, ...]]:
    return {
        cell.cell_id.value: tuple(
            sorted(item.owner_key.value for item in cell.candidates)
        )
        for cell in complex_.cells
    }


def _cut_locus_reference(skeleton) -> Counter:
    """Место И МОМЕНТ встречи с объединением владельцев — выводом из скелета.

    Вывод здесь СВОЙ, а не вызов адаптера: ось, у которой обе стороны считает
    одна функция, проверяет только то, что функция детерминирована.
    """

    grouped: dict[tuple, set] = {}
    for node in skeleton.nodes:
        time = node.time.canonical()
        key = (
            time.dividend,
            time.divisor.terms,
            node.point.x.terms,
            node.point.y.terms,
        )
        grouped.setdefault(key, set()).update(
            owner_key_of(item).value for item in node.participants
        )
    return Counter(
        (key, tuple(sorted(owners))) for key, owners in grouped.items()
    )


def _cut_locus_from_complex(complex_) -> Counter:
    return Counter(
        (
            (
                Fraction(point.time.dividend.numerator, point.time.dividend.denominator),
                from_algebraic_sum(point.time.divisor).terms,
                from_algebraic_sum(point.place.x).terms,
                from_algebraic_sum(point.place.y).terms,
            ),
            tuple(item.value for item in point.owner_keys),
        )
        for point in complex_.cut_locus
    )


def _station_witnesses_reference(partition) -> Counter:
    return Counter(
        (
            owner_key_of(face.owner).value,
            tuple((point[0].terms, point[1].terms) for point in face.points[2:]),
        )
        for face in partition.faces
    )


def _station_witnesses_from_complex(complex_) -> Counter:
    return Counter(
        (
            candidate.owner_key.value,
            tuple(
                (
                    from_algebraic_sum(witness.place.x).terms,
                    from_algebraic_sum(witness.place.y).terms,
                )
                for witness in candidate.station_witnesses
            ),
        )
        for cell in complex_.cells
        for candidate in cell.candidates
    )


def _extra_reference(partition, skeleton) -> dict:
    return {
        "triangle_owner_map": _owner_map_reference(partition),
        "cut_locus": _cut_locus_reference(skeleton),
        "station_witnesses": _station_witnesses_reference(partition),
    }


def _extra_from_complex(complex_) -> dict:
    return {
        "triangle_owner_map": _owner_map_from_complex(complex_),
        "cut_locus": _cut_locus_from_complex(complex_),
        "station_witnesses": _station_witnesses_from_complex(complex_),
    }


# --------------------------------------------------------------------------
# Ворота
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", tuple(BY_NAME))
def test_planar_result_survives_the_surface_contract_on_all_nine_axes(name):
    polygon, skeleton, _partition, complex_ = _columns(name)
    reference = _reading(polygon, skeleton)
    restored = _reading_from_complex(complex_)
    assert tuple(restored) == AXES
    for axis in AXES:
        if reference[axis] != restored[axis]:
            pytest.fail(
                f"{name}: SURFACE_FLAT_REDUCTION_MISMATCH axis={axis}\n"
                f"PLANAR_KERNEL={reference[axis]!r}\n"
                f"ARRIVAL_COMPLEX={restored[axis]!r}"
            )


@pytest.mark.parametrize("name", tuple(BY_NAME))
def test_surface_axes_of_the_complex_reproduce_the_planar_reading(name):
    _polygon, skeleton, partition, complex_ = _columns(name)
    reference = _extra_reference(partition, skeleton)
    restored = _extra_from_complex(complex_)
    assert tuple(restored) == EXTRA_AXES
    for axis in EXTRA_AXES:
        if reference[axis] != restored[axis]:
            pytest.fail(
                f"{name}: SURFACE_FLAT_REDUCTION_MISMATCH axis={axis}\n"
                f"PLANAR_KERNEL={reference[axis]!r}\n"
                f"ARRIVAL_COMPLEX={restored[axis]!r}"
            )


@pytest.mark.parametrize("name", tuple(BY_NAME))
def test_every_requested_alpha_stays_inside_the_declared_horizon(name):
    _polygon, _skeleton, _partition, complex_ = _columns(name)
    for alpha in ALPHAS:
        require_alpha_within_horizon(complex_, exact_rational(alpha))


def _rotated(polygon: PolygonV1, shift: int = 1) -> PolygonV1:
    """Тот же вход с другого начала обхода. Веера и скорости сохраняются."""

    loops = []
    for loop in polygon.loops:
        points = loop.points
        step = shift % len(points)
        loops.append(points[step:] + points[:step])
    rebuilt = PolygonV1.build(loops[0], tuple(loops[1:]))
    speeds = {frozenset((start, end)): value for start, end, value in polygon.edges()}
    rebuilt = with_edge_speeds(
        rebuilt,
        tuple(
            (start, end, speeds[frozenset((start, end))])
            for start, end, _ in rebuilt.edges()
        ),
    )
    if polygon.vertex_fans:
        rebuilt = with_vertex_fans(rebuilt, polygon.vertex_fans)
    return rebuilt


@pytest.mark.parametrize("name", tuple(BY_NAME))
def test_g_n3_input_permutation_gives_the_same_complex_byte_for_byte(name):
    """G-N3: сдвиг начала обхода — не другой вход, и ответ обязан не заметить."""

    polygon = BY_NAME[name]
    _polygon, _skeleton, _partition, complex_ = _columns(name)
    rotated = _rotated(polygon)
    skeleton = build_skeleton(rotated)
    other = _complex_of(rotated, skeleton, build_faces(rotated, skeleton))
    assert canonical_json_bytes(other) == canonical_json_bytes(complex_), name


@pytest.mark.parametrize("name", tuple(BY_NAME))
def test_g_n4_both_split_modes_give_one_complex(name):
    """G-N4: MOTORCYCLE и EXHAUSTIVE — два поиска ОДНОГО ответа, не два ответа."""

    polygon = BY_NAME[name]
    built = {}
    for search in SplitSearch:
        skeleton = build_skeleton(polygon, split_search=search)
        built[search] = _complex_of(
            polygon, skeleton, build_faces(polygon, skeleton)
        )
    motorcycle = canonical_json_bytes(built[SplitSearch.MOTORCYCLE])
    exhaustive = canonical_json_bytes(built[SplitSearch.EXHAUSTIVE])
    assert motorcycle == exhaustive, name


def test_the_gate_runs_on_a_named_non_empty_corpus_of_three_families():
    """Ворота на пустом множестве зеленеют молча, поэтому множество названо."""

    assert len(CORPUS) == len(BY_NAME) == 86
    families = Counter(name.split(":", 1)[0] for name in BY_NAME)
    assert set(families) == {"named", "partial", "weighted"}
    assert all(count > 0 for count in families.values())
    complexes = tuple(_columns(name)[3] for name in BY_NAME)
    # Исходы материализации владения несутся ДОСЛОВНО, и их в корпусе три —
    # включая оба отказа. Один только EXACT означал бы, что запись отказа не
    # проверена ничем.
    assert set(item.ownership_outcome.value for item in complexes) == {
        "EXACT",
        "SKELETON_IS_NOT_EXACT",
        "FACE_CHAIN_DOES_NOT_CLOSE",
    }
    # Обе стороны каждой двоичной оси присутствуют. Ось, у которой в корпусе
    # встречается одно значение, проверена ровно наполовину.
    assert set(item.front_outcome.value for item in complexes) == {
        "EXACT",
        "WAVEFRONT_LEFT_UNRESOLVED",
    }
    assert set(item.proof_status.value for item in complexes) == {
        "COMPLETE",
        "INCOMPLETE",
    }
    assert sum(len(item.proof_obligations) for item in complexes) > 0
    # MULTIWAY-узел — единственный путь, на котором `node_record` спрашивает
    # `validate_multiway_node`; без него ось `node_record` не трогала бы
    # инцидентности вовсе.
    assert (
        sum(
            1
            for item in complexes
            for event in item.events
            if event.kind.value == "MULTIWAY"
        )
        > 0
    )
    seed_kinds = Counter(
        seed.kind.value for item in complexes for seed in item.seeds
    )
    assert {
        "PARALLEL_SEGMENT",
        "BOUNDARY_CONDITION",
        "DIRECTIONAL_PARALLEL",
    } <= set(seed_kinds)


# --------------------------------------------------------------------------
# Отрицательный контроль представимости
# --------------------------------------------------------------------------


def _first_exact_case() -> str:
    for name in BY_NAME:
        if _columns(name)[3].ownership_outcome.value == "EXACT":
            return name
    raise AssertionError("в корпусе нет ни одного EXACT-разбиения")


def test_a_complex_that_lost_one_face_owner_fails_the_axes():
    """Сконструированная потеря обязана ПРОВАЛИТЬ сверку, а не пройти её.

    Без этого контроля девять зелёных осей означали бы только, что обе колонки
    читают одну и ту же переменную. Здесь у комплекса отнимают ровно один
    фрагмент владения и его ячейку — контракт при этом остаётся валидным, — и
    оси обязаны это увидеть.
    """

    name = _first_exact_case()
    polygon, skeleton, _partition, complex_ = _columns(name)
    victim = min(complex_.owner_fragments, key=lambda item: item.owner_key.value)
    mutilated = replace(
        complex_,
        owner_fragments=frozenset(
            item for item in complex_.owner_fragments if item is not victim
        ),
        cells=frozenset(
            cell
            for cell in complex_.cells
            if cell.cell_id != victim.cell_id
        ),
    )
    reference = _reading(polygon, skeleton)
    restored = _reading_from_complex(mutilated)
    broken = [axis for axis in AXES if reference[axis] != restored[axis]]
    assert {"face_owners", "face_areas", "coverage"} <= set(broken), broken
    extra_broken = [
        axis
        for axis in EXTRA_AXES
        if _extra_reference(_columns(name)[2], skeleton)[axis]
        != _extra_from_complex(mutilated)[axis]
    ]
    assert {"triangle_owner_map", "station_witnesses"} <= set(extra_broken)


# --------------------------------------------------------------------------
# Именованные двери контракта
# --------------------------------------------------------------------------


def _planar_complex():
    return _columns(_first_exact_case())[3]


def _adjacency_ref(value: str) -> SurfaceAdjacencyRefV1:
    return SurfaceAdjacencyRefV1(
        REVISION,
        SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH,
        SurfaceAdjacencyDigestValue(value),
    )


def _surface_complex(**overrides) -> SurfaceArrivalComplexV1:
    """Минимальный комплекс на МЕШЕ — носитель отрицательных контролей."""

    adjacency = overrides.pop("adjacency", _adjacency_ref("a" * 64))
    regime = overrides.pop("surface_regime", SurfaceRegime.DEVELOPABLE)
    fields = {
        "schema_version": SURFACE_ARRIVAL_COMPLEX_V1_SCHEMA,
        "patch_domain_id": DOMAIN,
        "source_revision": REVISION,
        "backend": SurfaceArrivalBackendV1.DEVELOPABLE_UNFOLD,
        "metric_ref": SurfaceMetricRefV1(
            REVISION,
            DOMAIN,
            regime,
            ArrivalMetricDigestValue("b" * 64),
            adjacency,
        ),
        "adjacency_ref": adjacency,
        "cell_law": ArrivalCellLawV1.MESH_TRIANGLE_V1,
        "alpha_horizon": exact_rational(Fraction(1)),
        "seeds": frozenset(),
        "cells": frozenset(),
        "cut_locus": frozenset(),
        "owner_fragments": frozenset(),
        "events": (),
        "domain_doubled_area": ExactAlgebraicSumV1(()),
        "front_outcome": _planar_complex().front_outcome,
        "ownership_outcome": LawId("EXACT"),
        "proof_status": _planar_complex().proof_status,
        "proof_obligations": (),
        "precision_tier": SurfaceMetricPrecisionTierV1.REFERENCE_EXACT_SMALL,
        "named_epsilon": None,
        "counters": (),
    }
    fields.update(overrides)
    return SurfaceArrivalComplexV1(**fields)


def _refusal(callable_, *args, **kwargs) -> str:
    with pytest.raises(SurfaceArrivalContractError) as refusal:
        callable_(*args, **kwargs)
    return refusal.value.invariant


def test_the_minimal_mesh_complex_is_accepted_so_the_doors_have_a_baseline():
    assert _surface_complex().cell_law is ArrivalCellLawV1.MESH_TRIANGLE_V1


def test_g_n7_certified_without_epsilon_is_refused():
    assert (
        _refusal(
            _surface_complex,
            precision_tier=SurfaceMetricPrecisionTierV1.REFERENCE_CERTIFIED,
        )
        == "CERTIFIED_WITHOUT_EPSILON"
    )
    assert (
        _refusal(
            _surface_complex,
            precision_tier=SurfaceMetricPrecisionTierV1.RUNTIME_APPROXIMATE,
        )
        == "CERTIFIED_WITHOUT_EPSILON"
    )


def test_g_n7_epsilon_under_the_exact_tier_is_refused_too():
    """`None ⇔ exact-tier` — эквивалентность, а не импликация."""

    assert (
        _refusal(
            _surface_complex,
            named_epsilon=NamedEpsilonV1(LawId("LAW"), Decimal("0.5")),
        )
        == "EPSILON_UNDER_EXACT_TIER"
    )


def test_g_n8_disagreeing_refs_are_refused():
    assert (
        _refusal(_surface_complex, adjacency_ref=_adjacency_ref("c" * 64))
        == "METRIC_REF_ADJACENCY_DISAGREEMENT"
    )


def test_g_n8_a_mesh_without_an_adjacency_table_is_refused():
    assert (
        _refusal(_surface_complex, adjacency_ref=None)
        == "SURFACE_METRIC_REQUIRES_ADJACENCY"
    )


def test_g_n8_a_planar_chart_carrying_an_adjacency_table_is_refused():
    """У одной плоской карты склейки треугольников нет ПРЕДМЕТА, а не записи."""

    planar = _planar_complex()
    assert (
        _refusal(replace, planar, adjacency_ref=_adjacency_ref("d" * 64))
        == "PLANAR_CHART_HAS_NO_ADJACENCY"
    )


def test_g_n8_cell_law_must_agree_with_the_declared_authority():
    planar = _planar_complex()
    assert (
        _refusal(replace, planar, cell_law=ArrivalCellLawV1.MESH_TRIANGLE_V1)
        == "CELL_LAW_DISAGREES_WITH_METRIC_REF"
    )
    assert (
        _refusal(
            _surface_complex,
            cell_law=ArrivalCellLawV1.PLANAR_SINGLE_CHART_REGION_V1,
        )
        == "CELL_LAW_DISAGREES_WITH_METRIC_REF"
    )


def test_g_n9_alpha_beyond_the_horizon_is_a_named_refusal():
    planar = _planar_complex()
    beyond = exact_rational(max(ALPHAS) + Fraction(1, 1024))
    assert (
        _refusal(require_alpha_within_horizon, planar, beyond)
        == "ALPHA_BEYOND_HORIZON"
    )
    assert (
        _refusal(require_alpha_within_horizon, planar, exact_rational(Fraction(-1)))
        == "ALPHA_IS_NEGATIVE"
    )


def test_a_non_positive_horizon_is_refused():
    for horizon in (ExactRationalV1(0, 1), ExactRationalV1(-1, 4)):
        assert (
            _refusal(_surface_complex, alpha_horizon=horizon)
            == "ALPHA_HORIZON_NOT_POSITIVE"
        )


def test_general_curved_is_never_reference_exact_small():
    assert (
        _refusal(_surface_complex, surface_regime=SurfaceRegime.GENERAL_CURVED)
        == "GENERAL_CURVED_IS_NEVER_EXACT"
    )


def test_a_preserved_tie_carries_at_least_two_candidates():
    planar = _planar_complex()
    cell = min(planar.cells, key=lambda item: item.cell_id.value)
    for resolution in (
        ArrivalTieResolutionV1.MULTIWAY_PRESERVED,
        ArrivalTieResolutionV1.UNDECIDED_FAIL_CLOSED,
    ):
        lonely = ArrivalCellV1(cell.cell_id, cell.candidates, resolution)
        assert (
            _refusal(replace, planar, cells=frozenset({lonely}))
            == "TIE_NEEDS_TWO_CANDIDATES"
        )


def test_a_candidate_whose_owner_is_not_a_seed_is_refused():
    planar = _planar_complex()
    cell = min(planar.cells, key=lambda item: item.cell_id.value)
    stranger = replace(
        cell,
        cell_id=ArrivalCellId(PLANAR_CHART_CELL_PREFIX + "stranger"),
        candidates=(
            replace(cell.candidates[0], owner_key=owner_key_of((9, 9, 9, 9))),
        ),
    )
    assert (
        _refusal(replace, planar, cells=frozenset({stranger}))
        == "CANDIDATE_OWNER_IS_NOT_A_SEED"
    )


def test_a_fragment_pointing_at_an_undeclared_cell_is_refused():
    planar = _planar_complex()
    victim = min(planar.owner_fragments, key=lambda item: item.owner_key.value)
    dangling = replace(
        victim, cell_id=ArrivalCellId(PLANAR_CHART_CELL_PREFIX + "nowhere")
    )
    assert (
        _refusal(
            replace,
            planar,
            owner_fragments=frozenset(
                {dangling}
                | {item for item in planar.owner_fragments if item is not victim}
            ),
        )
        == "FRAGMENT_CELL_IS_NOT_DECLARED"
    )


# --------------------------------------------------------------------------
# Рукопожатие с S0: ссылки сверяются с НАСТОЯЩИМИ байтами метрики
# --------------------------------------------------------------------------


def _s0_metric_and_adjacency():
    """Настоящая V2-метрика S0 на плоском квадрате из двух треугольников."""

    surface = two_triangle_quad()
    adjacency = read_surface_adjacency(
        surface, scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH
    )
    metric = build_surface_metric_v2(
        patch_domain_id=DOMAIN,
        surface=surface,
        adjacency=adjacency,
        source_positions={
            vertex(0): (Fraction(0), Fraction(0), Fraction(0)),
            vertex(1): (Fraction(1), Fraction(0), Fraction(0)),
            vertex(2): (Fraction(1), Fraction(1), Fraction(0)),
            vertex(3): (Fraction(0), Fraction(1), Fraction(0)),
        },
        surface_regime=SurfaceRegime.DEVELOPABLE,
        snapping_law=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
    )
    return metric, adjacency


def _mesh_complex_on(metric, **overrides) -> SurfaceArrivalComplexV1:
    return _surface_complex(
        source_revision=metric.source_revision,
        metric_ref=SurfaceMetricRefV1(
            metric.source_revision,
            metric.patch_domain_id,
            metric.surface_regime,
            arrival_metric_digest(metric),
            metric.adjacency_ref,
        ),
        adjacency_ref=metric.adjacency_ref,
        precision_tier=metric.precision_tier,
        named_epsilon=metric.named_epsilon,
        **overrides,
    )


def test_the_mesh_arm_agrees_with_real_s0_bytes():
    """Ссылка — сравнение пересчитанного дайджеста, а не доверие к полю."""

    metric, adjacency = _s0_metric_and_adjacency()
    complex_ = _mesh_complex_on(metric)
    validate_arrival_against_metric(
        complex_, metric=metric, adjacency=adjacency
    )


def test_a_tampered_metric_digest_is_caught_by_recomputation():
    metric, adjacency = _s0_metric_and_adjacency()
    complex_ = _mesh_complex_on(metric)
    tampered = replace(
        complex_,
        metric_ref=replace(
            complex_.metric_ref,
            metric_digest=ArrivalMetricDigestValue("e" * 64),
        ),
    )
    assert (
        _refusal(
            validate_arrival_against_metric,
            tampered,
            metric=metric,
            adjacency=adjacency,
        )
        == "METRIC_DIGEST"
    )


def test_a_cell_that_is_not_a_mesh_triangle_is_caught_against_the_table():
    """Адрес ячейки под `MESH_TRIANGLE_V1` обязан БЫТЬ треугольником таблицы."""

    metric, adjacency = _s0_metric_and_adjacency()
    planar = _planar_complex()
    borrowed = min(planar.cells, key=lambda item: item.cell_id.value)
    complex_ = _mesh_complex_on(
        metric,
        seeds=planar.seeds,
        cells=frozenset(
            {
                ArrivalCellV1(
                    ArrivalCellId("synthetic:triangle:9999"),
                    borrowed.candidates,
                    ArrivalTieResolutionV1.RESOLVED_EXACT,
                )
            }
        ),
    )
    assert (
        _refusal(
            validate_arrival_against_metric,
            complex_,
            metric=metric,
            adjacency=adjacency,
        )
        == "CELL_IS_NOT_A_MESH_TRIANGLE"
    )


# --------------------------------------------------------------------------
# G-N1: невыполним сегодня, и это записано
# --------------------------------------------------------------------------


@pytest.mark.parametrize("fixture", ("half_sphere", "fold_near_smooth"))
def test_g_n1_non_planar_input_must_diverge_from_the_planar_kernel(fixture):
    """G-N1 НЕВЫПОЛНИМ: непланарного бэкенда не существует.

    Требование записано полностью (`DECISIONS.md`, 2026-08-03): непланарный
    вход ОБЯЗАН разойтись с планарным ядром, иначе где-то происходит молчаливое
    проецирование. Проверить его нечем: `SurfaceArrivalComplexV1` умеет
    выразить только результат `PLANAR_QUEUE`, а трёх остальных бэкендов нет.

    Тест стоит здесь именно потому, что его отсутствие было бы неотличимо от
    его прохождения: ворота на пустом множестве зеленеют молча.
    """

    pytest.skip(f"SURFACE_BACKEND_NOT_IMPLEMENTED: {fixture}")
