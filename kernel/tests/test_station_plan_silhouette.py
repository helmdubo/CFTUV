"""План станций цепей в проходе силуэта (`CHAIN_STATION_PLAN_V1`, S2): `FREE`-вершины цепей решаются по плану, а не по сетке домена.

Прежний общий проход по готовым батчам всех доменов прогона (`SILHOUETTE_SOURCE_DOTS_V1`) удалён: какие вершины цепи декаль не несёт,
решено при компиляции, а проход домена только исполняет решение. Что доказано на синтетике (прямоугольник `columns x rows`, источник —
нижняя сторона, стена — боковые, фронт — верхняя):

| что проверяется                                                                           | тест |
|-------------------------------------------------------------------------------------------|------|
| `FREE`-вершина цепи источника растворена, остальные вершины цепи остаются                  | `..._a_free_vertex_of_the_source_chain_is_dissolved_...` |
| допуска UV у плана нет: сдвиг сверх допуска запроса растворяет, сдвиг записан отдельно      | `..._the_uv_tolerance_does_not_act_...` |
| допуск хорды остаётся: вершина за допуском выживает под именем, проверка сходится           | `..._a_free_vertex_beyond_the_chord_...` |
| копия разреза кольца остаётся и названа                                                     | `..._a_twin_copy_...` |
| прикреплённое ребро (степень не два) — выживает под именем                                  | `..._an_attached_edge_...` |
| конец цепи границы — угол декали, плана там нет                                             | `..._a_corner_...` |
| проверка: выживший без имени, имя без выжившего, сдвиг сверх записанного — красные          | `..._verification_...` |
| бюджет кончился — названный пропуск, плана не требуют                                       | `..._a_skipped_pass_...` |
| поле (`mesh2`): ни одной `FREE`-вершины в батче, закон до плана их несёт, батч валиден       | `..._a_field_domain_...` |
"""

from __future__ import annotations

import dataclasses
from fractions import Fraction

import pytest

from cftuv_envelope._chain_station import free_vertices
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import CUT_RIGHT_COPY_MARK
from cftuv_envelope.exact_sqrt_sum import exact_work_budget
from cftuv_envelope.materialize import silhouette
from cftuv_envelope.materialize.silhouette import verify_silhouette
from cftuv_envelope.validation import validate_geometry_batch

from test_silhouette_topology import CHORD, _faces, _field, _grid, _key, _materialize, _run, _shifted


def _free(source, *keys):
    return dataclasses.replace(source, free=frozenset(keys))


def test_a_free_vertex_of_the_source_chain_is_dissolved_and_the_others_of_the_chain_stay():
    source = _free(_grid(4, 1, merged=True), _key(2, 0))

    result, counters = _run(source)

    assert _key(2, 0) in result.dissolved and not result.dissolved & {_key(1, 0), _key(3, 0)}
    assert counters[silhouette.PLAN_DISSOLVED] == 1
    assert counters[silhouette.VERTICES_DISSOLVED] == 3  # вершины фронта растворяет закон вершин, как прежде
    assert not any(name in counters for name in silhouette.PLAN_KEPT_NAMES)
    assert not verify_silhouette(source, result)
    assert len(_faces(result)[0]) == 4 + 2  # четыре угла и две оставшиеся вершины источника


def test_without_a_plan_the_pass_is_what_it_was():
    plain = _grid(4, 1, merged=True)
    with_empty_plan = _free(plain)

    first, _ = _run(plain)
    second, _ = _run(with_empty_plan)

    assert first.dissolved == second.dissolved and first.counters == second.counters
    assert not any(name.startswith("MATERIALIZE_STATION_PLAN_") for name, _value in first.counters)


def test_the_uv_tolerance_does_not_act_on_a_free_vertex_and_its_slide_is_recorded_apart():
    shift = Fraction(1, 100)  # сверх допуска запроса 1/256
    plain = _grid(4, 1, merged=True, station=lambda i, j: _shifted(i, j, shift, (2, 0)))
    source = _free(plain, _key(2, 0))

    kept, _kept_counters = _run(plain)
    result, counters = _run(source)

    assert _key(2, 0) not in kept.dissolved
    assert _key(2, 0) in result.dissolved
    assert counters[silhouette.PLAN_MAX_UV_SLIDE_MILLI_ALPHA] == 10  # 1/100 alpha
    assert counters[silhouette.MAX_UV_SLIDE_MILLI_ALPHA] <= 1  # допуск запроса для остальных вершин прежний
    assert not verify_silhouette(source, result)


def test_a_free_vertex_beyond_the_chord_depth_survives_by_name_and_the_verification_agrees():
    source = _free(_grid(4, 1, merged=True, lift=lambda i, j: 4 * CHORD if (i, j) == (2, 0) else 0.0), _key(2, 0))

    result, counters = _run(source)

    assert _key(2, 0) not in result.dissolved
    assert counters[silhouette.PLAN_KEPT_CHORD] == 1 and silhouette.PLAN_DISSOLVED not in counters
    assert not verify_silhouette(source, result)


def test_a_twin_copy_of_the_ring_cut_stays_and_is_named():
    source = _free(_grid(4, 1, merged=True), _key(1, 0), _key(2, 0))
    mesh = silhouette._Mesh(source, exact_work_budget(stage="MATERIALIZE"))
    incident = {key: {0} for key in source.positions}
    incident[_key(2, 0) + CUT_RIGHT_COPY_MARK] = {0}

    fixed = mesh._fixed_vertices(incident)

    assert {_key(2, 0), _key(2, 0) + CUT_RIGHT_COPY_MARK} <= fixed
    assert mesh.tally[silhouette.PLAN_KEPT_TWINNED] == 1
    assert _key(1, 0) not in fixed and mesh.planned == {_key(1, 0)}


def test_an_attached_edge_keeps_the_vertex_by_name_instead_of_silently():
    """Ребро между двумя регионами не растворяется, и вершина источника с таким ребром — не точка на прямой: выжила и названа."""

    source = _free(_grid(4, 1, split=2), _key(2, 0))

    result, counters = _run(source)

    assert _key(2, 0) not in result.dissolved
    assert counters[silhouette.PLAN_KEPT_ATTACHED] == 1
    assert not verify_silhouette(source, result)


def test_a_corner_of_the_chain_run_is_no_station_and_is_never_dissolved_even_when_the_plan_names_it():
    """Вершина, где граница переходит с источника на стену, — угол декали: плана там нет, правило прежнее."""

    source = _free(_grid(4, 1, merged=True), _key(0, 0), _key(4, 0))
    mesh = silhouette._Mesh(source, exact_work_budget(stage="MATERIALIZE"))
    incident = {key: {0} for key in source.positions}

    assert mesh.chain_interior(incident) == set()
    assert {_key(0, 0), _key(4, 0)} <= mesh._fixed_vertices(incident)
    result, counters = _run(source)
    assert not result.dissolved & {_key(0, 0), _key(4, 0)} and silhouette.PLAN_DISSOLVED not in counters


def test_the_verification_names_a_free_vertex_that_survives_without_a_name_and_a_name_without_a_survivor():
    source = _free(_grid(4, 1, merged=True), _key(2, 0))
    honest, _ = _run(source)
    plain, _ = _run(_grid(4, 1, merged=True))

    # выживший без имени: итог без плана при плане на входе
    assert "STATION_PLAN_VIOLATED" in verify_silhouette(source, plain)
    # имя без выжившего: названа вершина, которой в итоговой сетке нет
    named = (*honest.counters, (silhouette.PLAN_KEPT_CHORD, 1))
    assert "STATION_PLAN_VIOLATED" in verify_silhouette(source, dataclasses.replace(honest, counters=named))
    # растворённая вершина плана, которая всё же стоит в итоговой сетке
    assert "STATION_PLAN_VIOLATED" in verify_silhouette(source, dataclasses.replace(plain, dissolved=plain.dissolved | {_key(2, 0)}))
    assert not verify_silhouette(source, honest)


def test_the_verification_names_a_plan_slide_beyond_the_recorded_maximum():
    source = _free(_grid(4, 1, merged=True, station=lambda i, j: _shifted(i, j, Fraction(1, 100), (2, 0))), _key(2, 0))
    result, _counters = _run(source)
    lowered = tuple((name, 0 if name == silhouette.PLAN_MAX_UV_SLIDE_MILLI_ALPHA else value) for name, value in result.counters)

    assert "PLAN_UV_SLIDE_BEYOND_RECORDED_MAXIMUM" in verify_silhouette(source, dataclasses.replace(result, counters=lowered))


@pytest.mark.parametrize("skip", [silhouette.SKIPPED_WORK_BUDGET, silhouette.SKIPPED_NOT_MANIFOLD])
def test_a_skipped_pass_is_named_and_the_plan_is_not_demanded_of_it(skip):
    source = _free(_grid(4, 1, merged=True), _key(2, 0))

    result = silhouette._unchanged(source, ((skip, 1),))

    assert not verify_silhouette(source, result)


def _free_keys_in(batch, free):
    return {vertex.vert_key.value for vertex in batch.vertices if vertex.vert_key.value.startswith("src:") and vertex.vert_key.value[4:] in free}


def test_a_field_domain_carries_none_of_the_free_vertices_while_the_law_before_the_plan_still_carries_them():
    pair = _field("mesh2_patch0_cut_fans_v1")
    free = free_vertices(pair[0].compilation.chain_station_plans)
    planar = _materialize(pair, DecalTopologyLawV1.PLANAR_POLYGONS_V1)
    law = _materialize(pair, DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)
    counters = dict(law.counters)

    carried = _free_keys_in(planar.batch, free)
    assert free and carried, "the fixture must have free vertices the planar law carries"
    assert not _free_keys_in(law.batch, free)
    assert counters[silhouette.PLAN_DISSOLVED] == len(carried)
    assert not any(name in counters for name in silhouette.PLAN_KEPT_NAMES)
    assert counters[silhouette.PLAN_MAX_UV_SLIDE_MILLI_ALPHA] <= 4
    assert not validate_geometry_batch(law.batch)
    assert any(line.startswith("SILHOUETTE_TOPOLOGY_V1:") and "plan_dissolved=" in line for line in law.diagnostics)
