"""ПОЛОСОВАЯ КАРТА (C2): носитель вокруг ЗАМКНУТОЙ цепи - кольцо, и оно режется по пути от вершины обода до дальней петли.

Карта у кольца одна - развёртка диска: вершины пути раздваиваются (`<вершина>|cut:R`), края разреза - стены очереди,
шов UV один. Разрез - эвристика, меняющая ответ (непокрытый клин у шва, расхождение двух копий пути), поэтому записаны и
судятся два числа: `PERIODIC_CUT_SEAM_RESIDUAL` (реестр допусков) и `PERIODIC_CUT_BISECTOR_DEVIATION` (`NOISE_DIRECTION_SINE_BOUND`).
Цилиндр даёт сдвиг и нулевой шов, конус - поворот и шов в ячейки, купол - поворот и настоящий шов кривизны (миллиметры).
"""

from __future__ import annotations

import dataclasses
import math
from decimal import Decimal
from fractions import Fraction

import pytest

import cftuv_envelope as kernel
from cftuv_envelope import _annulus_cut, _band_chart
from cftuv_envelope.contracts.metric import (
    CUT_RIGHT_COPY_MARK,
    BandBoundaryRoleV1,
    DevelopableBandChartCertificateV1,
    ExactRationalV1,
    NearPlanarLiftLawV1,
    band_is_reach_limited,
)
from cftuv_envelope.materialize.admit import materialization_request
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import BAND_TRIGGER_OUTCOMES, PlanarMetricAdmissionError
from cftuv_envelope.validation_issues import ValidationCode
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

import band_factories as factories
import developable_factories as flat

CAP = "1/2"


def _fraction(value) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def _certificate(snapshot):
    return next(iter(snapshot.surface_metric_descriptors)).planarity_certificate


def _metric(snapshot):
    return next(iter(snapshot.surface_metric_descriptors))


def _materialize(snapshot, request, alpha=None):
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    coverage = conveyor_coverage(prepared) if alpha is None else conveyor_coverage(prepared, alpha)
    assert coverage.outcome.value == "EXACT", coverage.detail
    result = materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1,
    )
    return result


@pytest.fixture(scope="module")
def column():
    """Колонна 8 x 10 квадов, обод - верхнее кольцо, досягаемость 0.5 м: носитель - 6 строк, дальше стена."""

    return factories.band_domain(factories.column_top(), reach_cap=CAP)


@pytest.fixture(scope="module")
def frustum():
    return factories.band_domain(factories.frustum_ring(segments=12, rows=8), reach_cap=CAP)


@pytest.fixture(scope="module")
def dome():
    """Полусфера: 16 сторон, 8 колец, обод - весь экватор; полоса при досягаемости 1/4 - две строки колец."""

    return factories.band_domain(factories.dome_ring(sides=16, rings=8), reach_cap="1/4")


@pytest.fixture(scope="module")
def whole_ring():
    """Колонна ниже досягаемости: носитель - весь патч, кольцо без стены досягаемости."""

    return factories.band_domain(factories.column_top(rows=3), reach_cap=CAP)


def _angle(cut) -> float:
    return math.degrees(math.atan2(float(_fraction(cut.rotation_sine)), float(_fraction(cut.rotation_cosine))))


# --------------------------------------------------------------------------
# Цилиндр, конус, купол: голономия и два числа
# --------------------------------------------------------------------------


def test_a_closed_column_is_cut_along_a_generatrix_into_a_shift_with_a_zero_seam(column):
    snapshot, request, band = column
    certificate = _certificate(snapshot)
    cut = certificate.cut
    assert type(certificate) is DevelopableBandChartCertificateV1 and cut is not None
    # Вершина разреза - начало первого по имени выбранного `ChainUse`, то есть там же, где размыкается замкнутый поток.
    assert cut.cut_vertex_id == band.rim_edges[0][0] == cut.path_vertex_ids[0]
    # Сдвиг: поворота нет, шов точно нуль, разрез точно по биссектрисе.
    assert (_fraction(cut.rotation_cosine), _fraction(cut.rotation_sine)) == (1, 0)
    assert _fraction(cut.seam_residual_squared) == 0
    assert _fraction(cut.bisector_deviation_sine_squared) < Fraction(1, 10**12)
    # Шов меряется по префиксу пути в пределах досягаемости (0.5 м при шаге строк 0.1 м): шесть вершин из семи.
    assert cut.seam_vertex_count == 6 and len(cut.path_vertex_ids) == 7
    assert float(_fraction(certificate.stretch.worst_band_squared_upper)) < 1.001
    roles = [item.role for item in certificate.strip_boundary]
    edges = len(cut.path_vertex_ids) - 1
    assert roles.count(BandBoundaryRoleV1.CUT_LEFT) == roles.count(BandBoundaryRoleV1.CUT_RIGHT) == edges == 6
    assert roles.count(BandBoundaryRoleV1.RIM) == 8 and roles.count(BandBoundaryRoleV1.REACH_WALL) == 8
    assert band_is_reach_limited(certificate)


def test_the_cut_path_runs_along_edges_between_faces_and_never_along_a_diagonal(column):
    snapshot, _request, _band = column
    cut = _certificate(snapshot).cut
    carriers: dict = {}
    for triangle in snapshot.surface_ir.surface_triangles:
        for first, second in zip(triangle.vertex_ids, triangle.vertex_ids[1:] + triangle.vertex_ids[:1]):
            carriers.setdefault(frozenset((first, second)), set()).add(triangle.source_face_id)
    for first, second in zip(cut.path_vertex_ids, cut.path_vertex_ids[1:]):
        assert len(carriers[frozenset((first, second))]) == 2, (first, second)
    position = {item.vertex_id: item.position for item in snapshot.source_vertices}
    # Колонна: образующая - вертикаль, все вершины пути на одной вертикали.
    assert len({(position[item].x, position[item].y) for item in cut.path_vertex_ids}) == 1


def test_a_frustum_cut_is_a_rotation_with_a_cell_sized_seam(frustum):
    snapshot, _request, _band = frustum
    certificate = _certificate(snapshot)
    cut = certificate.cut
    assert abs(_angle(cut)) > 20.0 and _fraction(cut.rotation_sine) != 0
    assert float(_fraction(cut.seam_residual_squared)) ** 0.5 < 2e-4
    assert float(_fraction(certificate.stretch.worst_band_squared_upper)) < 1.01
    assert float(_fraction(cut.bisector_deviation_sine_squared)) ** 0.5 < float(kernel_noise_bound())


def kernel_noise_bound() -> Fraction:
    from cftuv_envelope.reference.evaluation_binding_noise import NOISE_DIRECTION_SINE_BOUND

    return NOISE_DIRECTION_SINE_BOUND


def test_a_dome_ring_materializes_with_a_curvature_seam_inside_the_registered_bounds(dome):
    snapshot, request, _band = dome
    certificate = _certificate(snapshot)
    cut = certificate.cut
    result = _materialize(snapshot, request)
    assert result.outcome.value == "MATERIALIZED", result.detail
    # Двойная кривизна: сжатие настоящее, но в бюджете запроса, а две копии пути неконгруэнтны на доли миллиметра.
    assert 1.0 < float(_fraction(certificate.stretch.worst_band_squared_upper)) <= 1.05
    seam = float(_fraction(cut.seam_residual_squared)) ** 0.5
    assert 1e-4 < seam <= float(_annulus_cut.SEAM_RESIDUAL_BOUND) == 2e-3
    assert cut.seam_vertex_count == 2 < len(cut.path_vertex_ids)
    assert float(_fraction(cut.bisector_deviation_sine_squared)) ** 0.5 <= float(kernel_noise_bound())
    assert 60.0 < abs(_angle(cut)) < 90.0
    assert certificate.excluded_triangle_count > 0 and certificate.chart_reach_margin_squared is not None


def test_the_numbers_of_the_cut_are_named_in_the_batch_and_the_two_copies_weld_into_one_vertex(column):
    snapshot, request, _band = column
    result = _materialize(snapshot, request)
    assert result.outcome.value == "MATERIALIZED", result.detail
    names = {item.split(":", 1)[0] for item in result.diagnostics}
    assert {"PERIODIC_CUT_SEAM_RESIDUAL", "PERIODIC_CUT_BISECTOR_DEVIATION", "FACE_BEYOND_CHART_REACH"} <= names
    batch = result.batch
    vertices = {item.vert_key.value: item for item in batch.vertices}
    copies = [key for key in vertices if key.endswith(CUT_RIGHT_COPY_MARK)]
    assert copies
    for key in copies:
        left = vertices[key[: -len(CUT_RIGHT_COPY_MARK)]]
        # Одна вершина источника: общая ссылка сварки и побитово одна позиция.
        assert vertices[key].semantic_location_ref == left.semantic_location_ref
        assert vertices[key].position == left.position
    uv = {}
    for face in batch.faces:
        for fact in face.uv_facts:
            uv.setdefault(fact.vert_key.value, set()).add((float(fact.uv.u), float(fact.uv.v)))
    # Шов UV один: копии стоят на разных концах развёртки по `u`, а по `v` они совпадают.
    for key in copies:
        (left_u, left_v), (right_u, right_v) = (
            next(iter(uv[key[: -len(CUT_RIGHT_COPY_MARK)]])),
            next(iter(uv[key])),
        )
        assert left_u != right_u and left_v == pytest.approx(right_v)


def test_the_ring_band_covers_the_whole_decal_of_the_column(column):
    """Покрытие кольца полное: площадь меша равна периметр x alpha (цилиндр развёртывается изометрично)."""

    snapshot, request, _band = column
    result = _materialize(snapshot, request)
    verts = {item.vert_key.value: item.position for item in result.batch.vertices}
    total = 0.0
    for face in result.batch.faces:
        points = [verts[item.value] for item in face.ordered_vert_keys]
        for index in range(1, len(points) - 1):
            a, b, c = points[0], points[index], points[index + 1]
            u = (b.x - a.x, b.y - a.y, b.z - a.z)
            w = (c.x - a.x, c.y - a.y, c.z - a.z)
            cross = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
            total += 0.5 * math.sqrt(sum(item * item for item in cross))
    perimeter = 8 * 2 * math.sin(math.pi / 8)
    assert total == pytest.approx(perimeter * 0.25, rel=1e-4)


# --------------------------------------------------------------------------
# Носитель - весь патч: кольцо без стены досягаемости
# --------------------------------------------------------------------------


def test_a_ring_inside_the_reach_is_cut_open_without_a_reach_wall_and_alpha_is_not_capped(whole_ring):
    snapshot, request, _band = whole_ring
    certificate = _certificate(snapshot)
    assert certificate.excluded_triangle_count == 0 and certificate.first_excluded_triangle_id is None
    assert certificate.chart_reach_margin_squared is None and not band_is_reach_limited(certificate)
    assert certificate.cut is not None
    assert BandBoundaryRoleV1.REACH_WALL not in {item.role for item in certificate.strip_boundary}
    assert certificate.previous_refusals[-1] == NamedOutcome.PERIODIC_CUT_REQUIRED.value
    # Усекать фронт нечему: alpha выше досягаемости - не отказ.
    wide = dataclasses.replace(request, requested_alpha=kernel.LocalLengthV1(request.requested_alpha.value * 3))
    assert kernel.validate_snapshot_request_references(snapshot, wide) == ()
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    assert conveyor_coverage(prepared, "0.2").outcome.value == "EXACT"
    assert conveyor_coverage(prepared, "0.75").outcome.value == "EXACT"
    result = _materialize(snapshot, request)
    assert result.outcome.value == "MATERIALIZED", result.detail
    assert not [item for item in result.diagnostics if item.startswith("FACE_BEYOND_CHART_REACH")]


def test_a_ring_with_no_band_request_keeps_the_whole_patch_refusal():
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        factories.band_domain(factories.column_top(rows=3))
    assert failure.value.outcome is NamedOutcome.PERIODIC_CUT_REQUIRED
    assert NamedOutcome.PERIODIC_CUT_REQUIRED in BAND_TRIGGER_OUTCOMES


# --------------------------------------------------------------------------
# Отказы по названным числам
# --------------------------------------------------------------------------


def test_a_seam_beyond_the_registered_bound_is_a_named_refusal_with_the_number(monkeypatch):
    monkeypatch.setattr(_band_chart, "SEAM_RESIDUAL_BOUND", Fraction(1, 10**7))
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        factories.band_domain(factories.frustum_ring(segments=12, rows=8), reach_cap=CAP)
    assert failure.value.outcome is NamedOutcome.PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED
    assert "differ by" in str(failure.value) and "seam bound" in str(failure.value)


def test_a_dome_band_wider_than_the_surface_allows_is_refused_by_the_seam_number_at_the_default_reach():
    """Метровая сфера при досягаемости 1/2: полоса шириной 0.8 м - не карта, шов 7.6 мм против 2 мм (именованный отказ)."""

    with pytest.raises(PlanarMetricAdmissionError) as failure:
        factories.band_domain(factories.dome_ring(sides=16, rings=8), reach_cap=CAP)
    assert failure.value.outcome is NamedOutcome.PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED
    assert "3 of 5 path vertices within the reach" in str(failure.value)


def test_the_seam_is_measured_over_the_path_prefix_within_the_reach_and_over_the_whole_path_without_a_wall():
    nodes = {
        _annulus_cut.SourceVertexId(name): (x, 0) for name, x in (("a", 0), ("b", 10), ("c", 25), ("d", 40), ("e", 100))
    }
    path = tuple(_annulus_cut.SourceVertexId(name) for name in "abcde")
    assert _annulus_cut.seam_prefix(nodes, path, 100, Fraction(1, 4)) == path[:3]
    assert _annulus_cut.seam_prefix(nodes, path, 100, Fraction(1, 100)) == path[:2]  # не меньше двух вершин
    assert _annulus_cut.seam_prefix(nodes, path, 100, Fraction(1)) == path
    assert _annulus_cut.seam_prefix(nodes, path, 100, None) == path


def test_a_cut_far_from_the_bisector_is_a_named_refusal_with_the_number(monkeypatch):
    from cftuv_envelope.reference import evaluation_binding_noise

    monkeypatch.setattr(evaluation_binding_noise, "NOISE_DIRECTION_SINE_BOUND", Fraction(1, 10**9))
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        factories.band_domain(factories.frustum_ring(segments=12, rows=8), reach_cap=CAP)
    assert failure.value.outcome is NamedOutcome.PERIODIC_CUT_BISECTOR_DEVIATION_EXCEEDED
    assert "bisector" in str(failure.value)


def _cylinder(segments=8):
    vertices, faces, triangles = flat.closed_cylinder(segments=segments)
    positions = {
        item.vertex_id: tuple(Fraction(float(axis)) for axis in (item.position.x, item.position.y, item.position.z))
        for item in vertices
    }
    return triangles, positions


def test_a_rim_on_both_boundary_loops_has_no_cut_path():
    from cftuv_envelope._unfold import annulus_topology

    triangles, positions = _cylinder()
    topology = annulus_topology(triangles, positions)
    loops = _annulus_cut.boundary_loops(topology)
    pairs = tuple(
        (loop[index], loop[(index + 1) % len(loop)]) for loop in loops for index in range(len(loop))
    )
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _annulus_cut.ring_cut_path(topology, positions, pairs)
    assert failure.value.outcome is NamedOutcome.PERIODIC_CUT_PATH_UNAVAILABLE


def test_the_cut_of_a_plain_cylinder_is_a_deterministic_disk_with_two_copies_of_the_path():
    from cftuv_envelope._unfold import annulus_topology, owner_topology

    triangles, positions = _cylinder(segments=8)
    topology = annulus_topology(triangles, positions)
    top = next(loop for loop in _annulus_cut.boundary_loops(topology) if loop[0].value.endswith("b"))
    rim = tuple((top[index], top[(index + 1) % len(top)]) for index in range(len(top)))
    path = _annulus_cut.ring_cut_path(topology, positions, rim)
    assert path[0] == rim[0][0] and len(path) == 2  # один ряд граней: образующая из одного ребра
    assert path == _annulus_cut.ring_cut_path(topology, positions, rim)
    strip = _annulus_cut.cut_strip(topology, path)
    copies = {vertex for item in strip.triangles for vertex in item.vertex_ids if _annulus_cut.is_right_copy(vertex)}
    assert copies == strip.right_vertices()
    charted = {**positions, **{_annulus_cut.right_copy(item): positions[item] for item in path}}
    disk = owner_topology(strip.triangles, charted)
    assert disk.euler_characteristic == 1 and disk.boundary_loop_count == 1
    assert len(strip.triangles) == len(triangles)


# --------------------------------------------------------------------------
# Валидатор не верит записи о разрезе
# --------------------------------------------------------------------------


def _forged_cut(snapshot, **changes):
    metric = _metric(snapshot)
    certificate = metric.planarity_certificate
    cut = dataclasses.replace(certificate.cut, **changes)
    certificate = dataclasses.replace(certificate, cut=cut)
    return dataclasses.replace(
        snapshot,
        surface_metric_descriptors=frozenset({dataclasses.replace(metric, planarity_certificate=certificate)}),
    )


def test_a_ring_snapshot_validates_and_round_trips_through_the_codec(column):
    snapshot, request, _band = column
    assert kernel.validate_analysis_snapshot(snapshot) == ()
    assert kernel.validate_snapshot_request_references(snapshot, request) == ()
    payload = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    decoded = kernel.AnalysisSnapshotCodecV1.loads(payload)
    assert decoded == snapshot and kernel.AnalysisSnapshotCodecV1.dumps(decoded) == payload
    assert _certificate(decoded).cut == _certificate(snapshot).cut


def _forgeries(cut) -> dict:
    swapped = (cut.path_vertex_ids[0], cut.path_vertex_ids[2], cut.path_vertex_ids[1], *cut.path_vertex_ids[3:])
    return {
        "seam_residual_squared": {"seam_residual_squared": ExactRationalV1(7, 1000)},
        "bisector_deviation_sine_squared": {"bisector_deviation_sine_squared": ExactRationalV1(7, 1000)},
        "rotation_sine": {"rotation_sine": ExactRationalV1(7, 1000)},
        "translation_x": {"translation_x": ExactRationalV1(7, 1000)},
        "path": {"path_vertex_ids": swapped},
        "corners": {"right_corners": frozenset(sorted(cut.right_corners, key=lambda item: item.triangle_id.value)[1:])},
    }


@pytest.mark.parametrize(
    "field", ["seam_residual_squared", "bisector_deviation_sine_squared", "rotation_sine", "translation_x", "path", "corners"]
)
def test_the_validator_catches_a_forged_cut_record(column, field):
    snapshot, _request, _band = column
    changes = _forgeries(_certificate(snapshot).cut)[field]
    issues = kernel.validate_analysis_snapshot(_forged_cut(snapshot, **changes))
    assert any(item.code is ValidationCode.SURFACE_METRIC for item in issues), field


def test_a_forged_reach_margin_of_a_ring_band_is_caught(column):
    snapshot, _request, _band = column
    metric = _metric(snapshot)
    forged = dataclasses.replace(metric.planarity_certificate, chart_reach_margin_squared=ExactRationalV1(9, 1))
    issues = kernel.validate_analysis_snapshot(
        dataclasses.replace(
            snapshot,
            surface_metric_descriptors=frozenset({dataclasses.replace(metric, planarity_certificate=forged)}),
        )
    )
    assert any(item.code is ValidationCode.SURFACE_METRIC for item in issues)


def test_the_cut_is_a_function_of_the_selection_and_changes_when_the_selected_chains_change():
    """Вершина разреза - начало первого по имени выбранного вхождения: другой выбор - другой разрез, тот же - тот же."""

    parts = factories.column_top(rows=4)
    first, _request, band = factories.band_domain(parts, reach_cap=CAP)
    again, _request, _band = factories.band_domain(parts, reach_cap=CAP)
    fewer, _request, fewer_band = factories.band_domain(parts, reach_cap=CAP, select=range(1, 8))
    assert _certificate(first).cut == _certificate(again).cut
    assert _certificate(first).cut.cut_vertex_id == band.rim_edges[0][0]
    assert _certificate(fewer).cut.cut_vertex_id == fewer_band.rim_edges[0][0] != _certificate(first).cut.cut_vertex_id
    # Невыбранное вхождение остаётся стеной исходной границы (и дальняя петля колонны - тоже): фронт растёт только из
    # выбранных цепей, а колонна ниже досягаемости целиком в носителе - стены досягаемости нет.
    roles = [item.role for item in _certificate(fewer).strip_boundary]
    assert roles.count(BandBoundaryRoleV1.RIM) == 7 and roles.count(BandBoundaryRoleV1.ORIGINAL_BOUNDARY) == 1 + 8
    assert BandBoundaryRoleV1.REACH_WALL not in roles


def test_a_corner_relation_on_the_cut_path_of_a_ring_band_is_a_named_issue(column):
    """Вершина разреза на карте раздвоена: угловое отношение в ней описывало бы одну из двух копий, и это названо."""

    from types import SimpleNamespace

    from cftuv_envelope.validation_band import check_cut_corners

    snapshot, _request, _band = column
    metric = _metric(snapshot)
    certificate = metric.planarity_certificate
    apex = certificate.cut.cut_vertex_id
    stranger = next(item.vertex_id for item in snapshot.source_vertices if item.vertex_id not in certificate.cut.path_vertex_ids)
    sector = SimpleNamespace(owner_sector_id="sector", patch_domain_id=metric.patch_domain_id)
    other = SimpleNamespace(owner_sector_id="elsewhere", patch_domain_id=kernel.PatchDomainId("another"))
    fake = SimpleNamespace(
        angular_owner_sectors=(sector, other),
        corner_relations=(
            SimpleNamespace(owner_sector_id="sector", source_vertex_id=stranger),
            SimpleNamespace(owner_sector_id="elsewhere", source_vertex_id=apex),
            SimpleNamespace(owner_sector_id="sector", source_vertex_id=apex),
        ),
    )
    issues: list = []
    check_cut_corners(issues, ("path",), certificate, fake, metric.patch_domain_id)
    assert len(issues) == 1 and "ANGULAR_CORNERS_AT_RING_CUT" in issues[0].message
    assert kernel.validate_analysis_snapshot(snapshot) == ()


def test_every_input_of_the_cut_decision_changes_the_certificate_and_all_of_them_are_keyed_by_the_host():
    """Решение о разрезе - функция выбора цепей, досягаемости, допуска растяжения, снапшота и кода ядра.

    Все пять входят в ключ содержимого домена (`tests/test_envelope_content_key.py`: выбор домена, `band_key`, досягаемость
    запроса, подпись политики с допуском, отпечаток кода и снапшот в выгрузке воркера), поэтому результат, перенесённый на
    новую ревизию, не обходит решение о разрезе. Здесь - исполняемая половина: выбор, досягаемость и допуск двигают
    сертификат (снапшот и код двигают его очевидно: путь и числа считаются по его позициям).
    """

    parts = factories.column_top(rows=6)
    base, _request, _band = factories.band_domain(parts, reach_cap="1/4", alpha="0.25")
    wide, _request, _band = factories.band_domain(parts, reach_cap="1/2", alpha="0.25")
    strict, _request, _band = factories.band_domain(
        parts, reach_cap="1/4", alpha="0.25", developable_stretch_budget=Fraction(1, 10)
    )
    fewer, _request, _band = factories.band_domain(parts, reach_cap="1/4", alpha="0.25", select=range(1, 8))
    certificate = _certificate(base)
    assert _certificate(wide).cut.seam_vertex_count != certificate.cut.seam_vertex_count
    assert _certificate(wide).support_reach != certificate.support_reach or _certificate(wide).reach_cap != certificate.reach_cap
    assert _certificate(strict).support_reach != certificate.support_reach
    assert _certificate(fewer).cut.cut_vertex_id != certificate.cut.cut_vertex_id


# --------------------------------------------------------------------------
# Суженная досягаемость: карта под `alpha * (1 + b)` вместо запрошенной
# --------------------------------------------------------------------------


def test_the_tightened_reach_is_the_decals_own_reach_and_only_when_it_is_narrower():
    from cftuv_envelope.chart_band import BAND_TIGHTEN_OUTCOMES, tightened_reach_cap

    assert tightened_reach_cap(Fraction(1, 2), Fraction(1, 4), Fraction(1, 5)) == Fraction(3, 10)
    assert tightened_reach_cap(Fraction(1, 2), Fraction(1, 2), Fraction(1, 5)) is None  # шире запрошенной: сужать нечего
    assert tightened_reach_cap(Fraction(1, 2), Fraction(5, 12), Fraction(1, 5)) is None  # равна запрошенной
    assert tightened_reach_cap(Fraction(1, 2), None, Fraction(1, 5)) is None and tightened_reach_cap(1, 0, 0) is None
    assert {item.value for item in BAND_TIGHTEN_OUTCOMES} == {
        "PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED",
        "DEVELOPABLE_STRETCH_BUDGET_EXCEEDED",
    }


def test_a_dome_ring_that_refuses_at_the_requested_reach_is_built_under_the_tightened_one_and_validates():
    """Метровая сфера: запрошенная 1/2 отказывает швом, суженная `0.25 * 6/5 = 3/10` - карта; запрос несёт запрошенную."""

    from cftuv_envelope.chart_band import tightened_reach_cap

    with pytest.raises(PlanarMetricAdmissionError) as refused:
        factories.band_domain(factories.dome_ring(sides=16, rings=8), reach_cap=CAP)
    assert refused.value.outcome is NamedOutcome.PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED
    tight = tightened_reach_cap(Fraction(1, 2), Fraction(1, 4), Fraction(1, 5))
    snapshot, request, band = factories.band_domain(
        factories.dome_ring(sides=16, rings=8),
        reach_cap=tight,
        requested_reach_cap=CAP,
        tightened_after=refused.value.outcome.value,
    )
    certificate = _certificate(snapshot)
    assert certificate.reach_cap == ExactRationalV1(3, 10) and certificate.tightened is not None
    assert certificate.tightened.requested_reach_cap == ExactRationalV1(1, 2)
    assert certificate.tightened.refused_outcome == "PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED"
    assert request.chart_reach_cap == ExactRationalV1(1, 2) and band.requested_reach_cap == Fraction(1, 2)
    assert float(_fraction(certificate.cut.seam_residual_squared)) ** 0.5 <= float(_annulus_cut.SEAM_RESIDUAL_BOUND)
    # Валидатор пересчитывает полосу под ЗАПИСАННОЙ суженной досягаемостью и сверяет запрос с запрошенной.
    assert kernel.validate_analysis_snapshot(snapshot) == ()
    assert kernel.validate_snapshot_request_references(snapshot, request) == ()
    result = _materialize(snapshot, request)
    assert result.outcome.value == "MATERIALIZED", result.detail
    lines = [item for item in result.diagnostics if item.startswith("CHART_REACH_TIGHTENED_FOR_SEAM")]
    assert len(lines) == 1 and "requested_reach_cap_m=0.5" in lines[0] and "reach_cap_m=0.3" in lines[0]
    assert "after=PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED" in lines[0]
    payload = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    assert kernel.AnalysisSnapshotCodecV1.loads(payload) == snapshot
    # alpha выше суженной досягаемости - названный отказ запроса (карта усекла бы фронт), а не усечённое покрытие.
    wide = dataclasses.replace(request, requested_alpha=kernel.LocalLengthV1(Decimal("0.4")))
    assert any("REQUEST_ALPHA_EXCEEDS_CHART_REACH" in item.message for item in kernel.validate_snapshot_request_references(snapshot, wide))


def test_a_tightened_record_is_checked_against_the_request_and_the_refusals_that_may_tighten():
    from cftuv_envelope.contracts.metric import BandTightenedV1

    snapshot, request, _band = factories.band_domain(
        factories.dome_ring(sides=16, rings=8),
        reach_cap=Fraction(3, 10),
        requested_reach_cap=CAP,
        tightened_after="PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED",
    )
    other = dataclasses.replace(request, chart_reach_cap=ExactRationalV1(3, 4))
    assert any(item.path[-1] == "reach_cap" for item in kernel.validate_snapshot_request_references(snapshot, other))
    metric = _metric(snapshot)
    forged = dataclasses.replace(
        metric.planarity_certificate, tightened=BandTightenedV1(ExactRationalV1(1, 2), "CHART_SELF_OVERLAP")
    )
    issues = kernel.validate_analysis_snapshot(
        dataclasses.replace(
            snapshot, surface_metric_descriptors=frozenset({dataclasses.replace(metric, planarity_certificate=forged)})
        )
    )
    assert any(item.code is ValidationCode.SURFACE_METRIC for item in issues)
    with pytest.raises(ValueError):
        BandTightenedV1(ExactRationalV1(0, 1), "PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED")
    with pytest.raises(ValueError):
        factories.chart_band_request(
            snapshot.physical_chains, snapshot.chain_uses, request.selected_chain_use_ids,
            next(iter(snapshot.patch_domains)).patch_domain_id, Fraction(1, 2), Fraction(3, 10), "PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED",
        )
