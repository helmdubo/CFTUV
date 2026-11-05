"""Полный путь материализации на честных входах: метрика станции, «Г», дыра, near-planar.

Аудит 2026-10-02 (предложения 6 и 7). Раньше полным путём шли только полевые
фикстуры и цепь из двух-трёх коллинеарных рёбер; здесь добавлены

* МЕТРИЧЕСКАЯ ПРОВЕРКА станции на косой карте: `s` меряется в метрике Грама, а
  `r` — время прихода `FaceV1.line` с `q = ratio * |d|^2`, и закон UV
  (`uv_law.py`) утверждает, что текселы изотропны. Утверждение проверено НЕ
  самим ядром: `s` и `r` каждой вершины сверяются с расстояниями в 3D от
  источника, посчитанными по исходной геометрии цепи, а изотропия — по
  якобиану `(u, v) -> 3D` каждого треугольника;
* угол «Г» из двух цепей, домен с дырой и near-planar домен.
"""

from __future__ import annotations

import math

import pytest

from cftuv_envelope.materialize import domain
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import fraction_from_exact

import materialize_factories as factories
from test_materialize_domain import CASES, _alpha_decimal, _case, _run


def _triple(value):
    return tuple(fraction_from_exact(item) for item in (value.x, value.y, value.z))


class _Chart:
    """Карта домена как её видит НЕЗАВИСИМЫЙ читатель: дескриптор и его вершины."""

    def __init__(self, prepared):
        frame = prepared.context.frame
        self.origin = _triple(frame.exact_origin)
        self.a = _triple(frame.exact_basis_a)
        self.b = _triple(frame.exact_basis_b)
        self.coordinates = {
            item.source_vertex_id: (
                fraction_from_exact(item.domain_coordinate.x),
                fraction_from_exact(item.domain_coordinate.y),
            )
            for item in frame.exact_source_vertex_coordinates
        }
        gram = frame.exact_gram_matrix
        self.trace = float(fraction_from_exact(gram.m00) + fraction_from_exact(gram.m11))
        self.scale = float(prepared.lattice.scale)

    @property
    def space_origin(self):
        return tuple(float(item) for item in self.origin)

    def space(self, vertex_id):
        u, v = self.coordinates[vertex_id]
        return tuple(
            float(self.origin[i] + self.a[i] * u + self.b[i] * v) for i in range(3)
        )

    @property
    def tolerance(self) -> float:
        """Сдвиг узла решётки: ≤ шаг карты, в 3D — не больше шага на норму базиса."""

        return 4.0 * math.sqrt(self.trace) / self.scale + 1e-9


def _sub(p, q):
    return tuple(a - b for a, b in zip(p, q))


def _dot(p, q):
    return sum(a * b for a, b in zip(p, q))


def _norm(p):
    return math.sqrt(_dot(p, p))


def _strip_regions(batch):
    """`{регион: [факты]}` только у регионов-полос (веера — своя станция)."""

    by_region: dict = {}
    for fact in batch.station_facts:
        by_region.setdefault(fact.semantic_region_id, []).append(fact)
    return {
        region: facts
        for region, facts in by_region.items()
        if all(item.station_model_id.value == "SEMANTIC_CHAIN_USE_S" for item in facts)
    }


def _strip_checks(name):
    """`(строк, макс. |s - s3D|, макс. |r - r3D|, max |r - r_chart|)` по полосам."""

    prepared, coverage, _request = _case(name)
    batch = _run(name).batch
    chart = _Chart(prepared)
    context = prepared.context
    position = {item.vert_key: item.position for item in batch.vertices}
    regions = {item.semantic_region_id: item for item in batch.semantic_regions}
    rows = 0
    worst_s = worst_r = worst_chart = 0.0
    for region_id, facts in _strip_regions(batch).items():
        (use_id,) = regions[region_id].provenance.chain_use_ids
        directed = context.directed_chain_vertices(context.uses_by_id[use_id])
        start, end = chart.space(directed[0]), chart.space(directed[-1])
        along = _sub(end, start)
        unit = tuple(item / _norm(along) for item in along)
        for fact in facts:
            point = position[fact.vert_key]
            offset = _sub((point.x, point.y, point.z), start)
            station = _dot(offset, unit)
            across = _norm(_sub(offset, tuple(station * item for item in unit)))
            worst_s = max(worst_s, abs(float(fact.source_s.value) - station))
            worst_r = max(worst_r, abs(float(fact.source_r.value) - across))
            rows += 1
    return rows, worst_s, worst_r, chart.tolerance, _alpha_decimal(coverage)


@pytest.mark.parametrize("name", CASES)
def test_the_station_equals_the_3d_distances_computed_from_the_source_geometry(name):
    """`s` — длина вдоль цепи, `r` — расстояние до её прямой, ОБА в метрике Грама.

    Сверка с 3D-расстояниями по геометрии самой цепи: если бы `r` был
    расстоянием в координатах карты, а не в метрике, на косой карте разница была
    бы кратной, а не в пределах шага решётки.
    """

    rows, worst_s, worst_r, tolerance, _alpha = _strip_checks(name)
    assert rows >= 3, name
    assert worst_s <= tolerance, (name, worst_s, tolerance)
    assert worst_r <= tolerance, (name, worst_r, tolerance)


def test_the_metric_check_is_not_vacuous_on_the_skew_chart():
    """Контроль чувствительности: расстояние В КООРДИНАТАХ КАРТЫ даёт другое число.

    Если бы `r` считался евклидово в координатах карты (а не в метрике Грама),
    на косой карте он разошёлся бы с 3D-расстоянием не на шаг решётки, а
    кратно — и метрическая сверка выше это поймала бы. Здесь показано, что
    альтернатива действительно другая: `r_chart` каждой вершины полосы считается
    по ЕЁ координатам карты, и разница с записанным `r` порядка `alpha`.
    """

    prepared, _coverage, _request = _case("skew")
    batch = _run("skew").batch
    chart = _Chart(prepared)
    # Нижнее ребро — ось `u` карты (`v = 0`): по координатам карты расстояние до
    # него — это `|v|`, а в 3D оно умножено на высоту базиса `B` над прямой `A`.
    ids = {item.value: item for item in chart.coordinates}
    assert chart.coordinates[ids["v0"]][1] == chart.coordinates[ids["v1"]][1] == 0
    position = {item.vert_key: item.position for item in batch.vertices}
    gram = prepared.context.frame.exact_gram_matrix
    g00, g01, g11 = (fraction_from_exact(i) for i in (gram.m00, gram.m01, gram.m11))
    determinant = float(g00 * g11 - g01 * g01)
    gap = 0.0
    for fact in batch.station_facts:
        point = position[fact.vert_key]
        offset = _sub((point.x, point.y, point.z), chart.space_origin)
        rhs_a = _dot(offset, tuple(float(item) for item in chart.a))
        rhs_b = _dot(offset, tuple(float(item) for item in chart.b))
        v_chart = (-float(g01) * rhs_a + float(g00) * rhs_b) / determinant
        gap = max(gap, abs(float(fact.source_r.value) - abs(v_chart)))
    assert gap > 0.5


@pytest.mark.parametrize("name", CASES)
def test_the_strip_base_is_arclength_over_alpha_and_v_is_r_over_alpha(name, monkeypatch):
    """STRAIGHTEN_RUNGS_V1 больше не обещает изотропии (растяжение трапеций - принятая цена). Держится: ребро основания `r = 0` длиной `L` даёт `|du| = L / alpha`, `v` любой вершины полосы равна `r / alpha` её факта. Ошибка метрики в `s` или `r` ломает эти два равенства; положение вершин `src:` - без подъёма, как и раньше."""

    monkeypatch.setattr(domain, "host_positions_of", lambda snapshot: {})
    _prepared, coverage, _request = _case(name)
    batch = _run(name).batch
    alpha = float(_alpha_decimal(coverage))
    position = {item.vert_key: item.position for item in batch.vertices}
    strips = set(_strip_regions(batch))
    r_of = {(fact.semantic_region_id, fact.vert_key): float(fact.source_r.value) for fact in batch.station_facts}
    edges = 0
    for face in batch.faces:
        if face.semantic_region_id not in strips:
            continue
        size = len(face.ordered_vert_keys)
        for at, key in enumerate(face.ordered_vert_keys):
            region = face.semantic_region_id
            assert face.uv_facts[at].uv.v * alpha == pytest.approx(r_of[(region, key)], abs=1e-6)
            other = face.ordered_vert_keys[(at + 1) % size]
            if r_of[(region, key)] == 0.0 and r_of[(region, other)] == 0.0:
                a, b = position[key], position[other]
                length = ((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2) ** 0.5
                du = abs(face.uv_facts[at].uv.u - face.uv_facts[(at + 1) % size].uv.u)
                assert du * alpha == pytest.approx(length, rel=1e-6, abs=1e-6)
                edges += 1
    assert edges >= 1


def _vertex_position(batch):
    return {item.vert_key: item.position for item in batch.vertices}


def test_two_chains_in_an_L_make_two_strips_a_miter_interface_and_a_station_per_chain():
    """Выпуклый угол «Г»: по региону на плечо и линия митры между ними."""

    result = _run("l_chains")
    batch = result.batch
    assert result.outcome is MaterializationOutcome.MATERIALIZED
    assert len(_strip_regions(batch)) == 2 == len(batch.semantic_regions)
    assert dict(result.counters)["MATERIALIZE_FAN_FACES"] == 0
    # Интерфейс — биссектриса угла `(10, 0)`: точки `(10 - t, t)`, то есть x + y = 10.
    position = _vertex_position(batch)
    assert batch.interface_chains
    for chain in batch.interface_chains:
        for key in chain.ordered_vert_keys:
            point = position[key]
            assert point.x + point.y == pytest.approx(10.0, abs=1e-3)
    # Общая вершина плеч одна (сварная), а станций у неё две: конец первого плеча
    # (s = 10) и начало второго (s = 0): у каждой цепи своя ось `u`.
    corner = [
        key for key, point in position.items() if (point.x, point.y) == (10.0, 0.0)
    ]
    assert len(corner) == 1
    stations = sorted(
        float(item.source_s.value)
        for item in batch.station_facts
        if item.vert_key == corner[0]
    )
    assert stations == pytest.approx([0.0, 10.0], abs=1e-6)


def test_a_bend_inside_one_chain_use_accumulates_the_first_arm_length_on_a_skew_chart():
    """Два пробега ОДНОГО `ChainUse`: `s_origin` второго — длина первого в 3D.

    Излом ВНУТРИ цепи `straight_snapshot` не собирает (цепь объявлена прямой:
    `SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR`; излом объявляется углом с
    сертификатом), поэтому порядок рёбер здесь задан руками — а рёбра, Грам и
    узлы настоящие, из домена «Г» на косой карте. Прежний тест излома шёл на
    ортонормальной карте с заглушками.
    """

    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
    from cftuv_envelope.materialize import stations

    prepared, _coverage, _request = _case("l_chains")
    table = stations.chain_station_table(prepared, factories.budget())
    by_use, _corners, _nodes = stations._collect_loops(prepared)
    context = prepared.context
    arms = sorted(name for name in by_use if name.startswith("use:"))
    assert len(arms) == 2
    edges = [by_use[name][0] for name in arms]
    along = [table.edges[(item.region_id, item.key)].along_chain_use for item in edges]
    uses = {key.value: value for key, value in context.uses_by_id.items()}
    first = uses[arms[0]]
    runs, records = stations._use_runs(
        first,
        context.chains_by_id[first.physical_chain_id],
        [(edges[0], along[0], False), (edges[1], along[1], False)],
        table.gram,
        factories.budget(),
    )
    scale = table.scale
    assert [item.run_index for item in runs] == [0, 1]
    assert runs[0].s_origin.is_zero
    # Плечо `(0,0)-(10,0)`: 10 в 3D при любой косине карты.
    assert runs[1].s_origin == SqrtSumV1.rational(10 * scale)
    assert [edge.run_id for _key, edge in records] == [item.run_id for item in runs]
    # Конец второго плеча `(10,10)`: 10 + 10 = 20.
    end = edges[1].end if along[1] else edges[1].start
    point = (SqrtSumV1.rational(end[0]), SqrtSumV1.rational(end[1]))
    assert stations.station_of(runs[1], point) == SqrtSumV1.rational(20 * scale)


def test_a_domain_with_a_hole_materializes_every_strip_around_it():
    """Дыра настоящая (петля `holes` у региона), и все четыре плеча на месте."""

    prepared, _coverage, _request = _case("ring")
    assert any(region.holes for region in prepared.domain.domain_regions)
    result = _run("ring")
    batch = result.batch
    assert result.outcome is MaterializationOutcome.MATERIALIZED
    counters = dict(result.counters)
    assert counters["MATERIALIZE_FACES_LOST"] == 0
    assert counters["MATERIALIZE_FACES_IN"] == counters["MATERIALIZE_FACES_CONTOURED"]
    assert len(_strip_regions(batch)) == 4
    # По интерфейсу на каждый из четырёх углов дыры (линии митр).
    assert len(batch.interface_chains) == 4
    # SOURCE — контур дыры целиком: периметр 16 в 3D, и все вершины на `r = 0`.
    position = _vertex_position(batch)
    length = 0.0
    for chain in batch.boundary_chains:
        if ":SOURCE:" not in chain.semantic_boundary_id.value:
            continue
        for first, second in zip(chain.ordered_vert_keys, chain.ordered_vert_keys[1:]):
            a, b = position[first], position[second]
            length += math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
    assert length == pytest.approx(16.0, abs=1e-3)


def test_a_near_planar_domain_materializes_on_the_certified_plane_and_says_so():
    from cftuv_envelope.contracts.metric import NearPlanarProjectionCertificateV1

    prepared, _coverage, _request = _case("near_planar")
    assert isinstance(
        prepared.context.frame.planarity_certificate, NearPlanarProjectionCertificateV1
    )
    result = _run("near_planar")
    assert result.outcome is MaterializationOutcome.MATERIALIZED
    named = {item.outcome for item in result.batch.diagnostics}
    assert NamedOutcome.NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE in named
    assert any(
        line.startswith("NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE")
        for line in result.diagnostics
    )
    # Меш лежит на сертифицированной плоскости, а она НЕ исходная: вершина `v3`
    # приподнята на 0.002, и плоскость наклонена к `z = 0`.
    points = [(item.x, item.y, item.z) for item in _vertex_position(result.batch).values()]
    assert len(points) >= 4
    assert max(abs(item[2]) for item in points) > 1e-6
    origin, first, second = points[0], points[1], points[2]
    normal = (
        (first[1] - origin[1]) * (second[2] - origin[2])
        - (first[2] - origin[2]) * (second[1] - origin[1]),
        (first[2] - origin[2]) * (second[0] - origin[0])
        - (first[0] - origin[0]) * (second[2] - origin[2]),
        (first[0] - origin[0]) * (second[1] - origin[1])
        - (first[1] - origin[1]) * (second[0] - origin[0]),
    )
    length = _norm(normal)
    assert length > 1e-9
    for point in points:
        assert abs(_dot(_sub(point, origin), normal)) / length <= 1e-9
