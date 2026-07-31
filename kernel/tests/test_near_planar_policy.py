"""Near-planar политика: проекция объявляется, а не случается молча.

Карта N0 требует: ни один солвер не сглаживает и не проецирует по факту
собственного отказа, и каждая неточная политика записывает масштаб, метод,
бюджет невязки и ревизию источника.

Найдено при проверке: `planar_metric.py` требовал `_dot3(p - o, n) != 0`, то
есть побитово точный ноль без допуска. Поэтому ядро отвергало любой патч,
вершины которого не компланарны в точности — а это практически любой
повёрнутый, отмасштабированный или отредактированный меш. `building.002`
проходил только потому, что он осевой с перпендикулярными углами.
"""

from __future__ import annotations

from fractions import Fraction
from math import gcd

import pytest

from cftuv_envelope.contracts.analysis import SourceVertexV1
from cftuv_envelope.contracts.metric import (
    ExactSourcePlaneCertificateV1,
    GridSnappingLawV1,
    NearPlanarResidualBudgetLawV1,
    PlanarityAdmissionLawV1,
)
from cftuv_envelope.robust.grid import GridSpecV1
from cftuv_envelope.source_grid import snap_positions
from cftuv_envelope.contracts.surface import SourceFaceV1
from cftuv_envelope.ids import (
    PatchDomainId,
    PatchId,
    PhysicalChainId,
    SourceFaceId,
    SourceRevision,
    SourceVertexId,
)
from cftuv_envelope.numeric import LocalPoint3V1, canonical_primitive_normal
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import (
    PlanarMetricAdmissionError,
    build_rational_affine_planar_metric,
)


EXACT = PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1
NEAR = PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1
SOURCE_SNAP = GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1


def _unit_quad_metric(off_plane_height: float, policy: PlanarityAdmissionLawV1):
    """Единичный квад, у которого одна вершина приподнята над плоскостью."""

    vertex_ids = [SourceVertexId(f"v{index}") for index in range(4)]
    positions = (
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, off_plane_height),
    )
    vertices = [
        SourceVertexV1(vertex_id=vertex_id, position=LocalPoint3V1(*position))
        for vertex_id, position in zip(vertex_ids, positions, strict=True)
    ]
    face = SourceFaceV1(
        face_id=SourceFaceId("f0"),
        patch_id=PatchId("p0"),
        vertex_cycle=tuple(vertex_ids),
        edge_cycle=tuple(PhysicalChainId(f"e{index}") for index in range(4)),
        polygon_normal=LocalPoint3V1(0.0, 0.0, 1.0),
        triangle_ids=(),
    )
    return build_rational_affine_planar_metric(
        source_revision=SourceRevision("rev"),
        patch_domain_id=PatchDomainId("d0"),
        owner_patch_id=PatchId("p0"),
        source_vertices=vertices,
        source_faces=[face],
        planarity_policy=policy,
    )


def test_exactly_planar_source_is_unaffected_by_the_new_policy():
    """Прежнее поведение обязано остаться побитово прежним."""

    certificate = _unit_quad_metric(0.0, EXACT).planarity_certificate
    assert certificate.admission_law is EXACT
    assert certificate.exact is True


def test_near_planar_source_is_still_refused_under_the_exact_policy():
    """Никакого автоматического сглаживания по факту отказа — карта N0."""

    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _unit_quad_metric(1e-12, EXACT)
    assert (
        failure.value.outcome
        is NamedOutcome.RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED
    )


def test_requested_near_planar_policy_admits_and_records_the_projection():
    certificate = _unit_quad_metric(1e-12, NEAR).planarity_certificate
    assert certificate.admission_law is NEAR
    assert certificate.exact is False
    assert certificate.residual_budget_law is (
        NearPlanarResidualBudgetLawV1.RELATIVE_EXTENT_OR_ULP_V1
    )
    # Записан масштаб, метод, бюджет и ревизия источника.
    assert certificate.source_revision == SourceRevision("rev")
    assert certificate.planar_extent.numerator > 0
    assert certificate.residual_budget.numerator > 0
    assert certificate.coordinate_ulp_multiplier == 64
    # И поимённо — какие вершины были сдвинуты. Их больше одной, и это следствие
    # закона плоскости: плоскость патча выводится Ньюэллом по ВСЕМ его
    # полигонам, поэтому приподнятая вершина наклоняет её целиком, и от
    # наклонённой плоскости отклоняются и соседи. Прежний закон брал плоскость
    # трёх первых вершин — те три лежали в ней тождественно, и «сдвинутой»
    # оказывалась ровно одна. Перечень по-прежнему называет тех, кого
    # действительно двигали, просто плоскость теперь принадлежит патчу.
    assert SourceVertexId("v3") in certificate.projected_source_vertex_ids
    assert certificate.projected_source_vertex_ids <= {
        SourceVertexId(f"v{index}") for index in range(4)
    }


def test_source_beyond_the_budget_is_refused_by_name():
    """Это уже не шум представления, а другая геометрия."""

    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _unit_quad_metric(0.05, NEAR)
    assert (
        failure.value.outcome
        is NamedOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED
    )


def test_projected_source_reconstructs_exactly():
    """Проекция рациональная, поэтому точность ниже по конвейеру не теряется.

    Построитель проверяет `origin + u*A + v*B == position` точным равенством;
    если бы проекция вносила приближение, построение упало бы здесь.
    """

    metric = _unit_quad_metric(1e-12, NEAR)
    assert len(metric.exact_source_vertex_coordinates) == 4


@pytest.mark.parametrize("height", [1e-15, 1e-12, 1e-9, 1e-8])
def test_budget_admits_representation_noise(height: float):
    """Шум порядка ULP и мельче обязан приниматься, иначе меши не пройдут."""

    assert _unit_quad_metric(height, NEAR).planarity_certificate.exact is False


# --------------------------------------------------------------------------
# Закон плоскости: плоскость патча выводится из ПАТЧА, а не из трёх его вершин.
# --------------------------------------------------------------------------


SLOPE = -0.4525 / 0.8918
"""Наклон полевого ската building.002: нормаль (0, −0.4525, −0.8918)."""


def _slope_patch(*, bend: float = 0.0, width: float = 3.3125):
    """Скат, выдавленный вдоль X: два ряда вершин, ТОЧНО одна плоскость.

    Это форма, на которой прежний закон и ломался. Затравочная тройка вершин в
    порядке сортировки id идёт вдоль склона и почти коллинеарна, поэтому её
    векторное произведение вырождается и «плоскость кадра» схлопывается на ось
    выдавливания — на ось X.
    """

    rows = (0.0, 0.9137, 1.8271, 2.7)
    vertex_ids = []
    vertices = []
    for column, x in enumerate((0.0, width)):
        for row, y in enumerate(rows):
            index = column * len(rows) + row
            vertex_id = SourceVertexId(f"v{index:02d}")
            vertex_ids.append(vertex_id)
            height = SLOPE * y + (bend if (column and row) else 0.0)
            vertices.append(
                SourceVertexV1(
                    vertex_id=vertex_id, position=LocalPoint3V1(x, y, height)
                )
            )
    faces = []
    for row in range(len(rows) - 1):
        cycle = (
            vertex_ids[row],
            vertex_ids[row + 1],
            vertex_ids[len(rows) + row + 1],
            vertex_ids[len(rows) + row],
        )
        faces.append(
            SourceFaceV1(
                face_id=SourceFaceId(f"f{row}"),
                patch_id=PatchId("p0"),
                vertex_cycle=cycle,
                edge_cycle=tuple(
                    PhysicalChainId(f"e{row}{side}") for side in range(4)
                ),
                polygon_normal=LocalPoint3V1(0.0, 1.0, 2.0),
                triangle_ids=(),
            )
        )
    return vertices, faces


def _axial_wall():
    """Осевая стена x = const — весь зелёный корпус полевого меша."""

    vertex_ids = []
    vertices = []
    for index, (y, z) in enumerate(
        ((0.0, 0.0), (0.0, 0.6), (1.7, 0.0), (1.7, 0.6), (3.3125, 0.0), (3.3125, 0.6))
    ):
        vertex_id = SourceVertexId(f"w{index:02d}")
        vertex_ids.append(vertex_id)
        vertices.append(
            SourceVertexV1(
                vertex_id=vertex_id, position=LocalPoint3V1(1.25, y, z)
            )
        )
    faces = [
        SourceFaceV1(
            face_id=SourceFaceId(f"g{index}"),
            patch_id=PatchId("p0"),
            vertex_cycle=cycle,
            edge_cycle=tuple(PhysicalChainId(f"h{index}{side}") for side in range(4)),
            polygon_normal=LocalPoint3V1(1.0, 0.0, 0.0),
            triangle_ids=(),
        )
        for index, cycle in enumerate(
            (
                (vertex_ids[0], vertex_ids[2], vertex_ids[3], vertex_ids[1]),
                (vertex_ids[2], vertex_ids[4], vertex_ids[5], vertex_ids[3]),
            )
        )
    ]
    return vertices, faces


def _metric(vertices, faces, grid_law):
    return build_rational_affine_planar_metric(
        source_revision=SourceRevision("rev"),
        patch_domain_id=PatchDomainId("d0"),
        owner_patch_id=PatchId("p0"),
        source_vertices=vertices,
        source_faces=faces,
        planarity_policy=NEAR,
        grid_policy=grid_law,
    )


def _normal_of(metric):
    vector = metric.planarity_certificate.exact_plane_normal
    return tuple(
        Fraction(item.numerator, item.denominator)
        for item in (vector.x, vector.y, vector.z)
    )


def test_a_tilted_but_exactly_planar_patch_passes_the_metric():
    """(a) Плоский наклонный патч обязан проходить. Прежде он отвергался.

    Полевой отказ `building.002` был именно этим: патч плоский до float-точности
    (остаток к СОБСТВЕННОЙ плоскости 3.2e-18 при габарите 3.3125), а мерялся он
    против плоскости затравочной тройки, где тот же остаток равен 3.312 — всему
    габариту патча — при бюджете 3.31e-07.
    """

    vertices, faces = _slope_patch()
    metric = _metric(vertices, faces, SOURCE_SNAP)

    # Плоскость — собственная плоскость патча, а НЕ ось выдавливания. Прежний
    # закон схлопывал её ровно на X, потому что затравочная тройка шла вдоль
    # склона; здесь X — единственная координата нормали, обязанная быть нулём.
    normal = _normal_of(metric)
    assert normal[0] == 0, normal
    assert normal[1] and normal[2], normal
    # Наклон тот самый, полевой: нормаль (0, −0.4525, −0.8918) даёт отношение
    # z/y = 1.9708. Совпадение с точностью объявленной правки решётки —
    # плоскость берётся уже от привязанных координат, и точного отношения тут
    # быть не может, иначе решётка бы ничего не двигала.
    assert abs(float(normal[2] / normal[1]) - 0.8918 / 0.4525) < 1e-4
    assert len(metric.exact_source_vertex_coordinates) == 8


def test_a_genuinely_bent_patch_is_still_refused_under_its_own_name():
    """(b) Подлинная непланарность отвергается, и отвергается СВОИМ именем.

    5 см из плоскости — не шум представления и не сдвиг решётки: после привязки
    отклонение 1.87e-02 при ячейке 6.10e-05 — в 306 раз больше всего, что
    объявленная правка геометрии способна внести.
    """

    vertices, faces = _slope_patch(bend=0.05)
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _metric(vertices, faces, SOURCE_SNAP)

    assert failure.value.outcome is (
        NamedOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED
    )
    assert "beyond the declared near-planar budget" in str(failure.value)


def test_the_axial_wall_keeps_its_frame_bit_for_bit():
    """(c) Осевой патч закон плоскости не двигает ВОВСЕ.

    Весь зелёный корпус полевого меша — осевые стены. У них плоскость патча и
    прежняя плоскость затравочной тройки совпадают по направлению, поэтому
    новый вывод обязан дать тот же кадр побитово: то же начало, тот же базис,
    та же матрица Грама, те же координаты карты. Меняется одна запись — форма
    нормали, — и она каноническая.
    """

    vertices, faces = _axial_wall()
    metric = _metric(vertices, faces, SOURCE_SNAP)

    # Контроль — чистая осевая привязка, то есть ровно то, что делал прежний
    # закон. Совпадение с ней и означает «плоскость не вмешалась»: ни одна
    # вершина не спроецирована, кадр собран от тех же координат.
    scale = metric.grid_certificate.source_scale
    expected = snap_positions(
        {
            item.vertex_id: (
                Fraction(*item.position.x.as_integer_ratio()),
                Fraction(*item.position.y.as_integer_ratio()),
                Fraction(*item.position.z.as_integer_ratio()),
            )
            for item in vertices
        },
        GridSpecV1(scale=scale),
    )
    def vector(record):
        return tuple(
            Fraction(item.numerator, item.denominator)
            for item in (record.x, record.y, record.z)
        )

    origin = vector(metric.exact_origin)
    assert origin == expected[min(expected, key=lambda item: item.value)]

    # И каждая вершина: `origin + u·A + v·B` обязано дать РОВНО осевую привязку.
    # Это и есть побитовость кадра — не одно поле, а весь его геометрический
    # смысл.
    basis_a = vector(metric.exact_basis_a)
    basis_b = vector(metric.exact_basis_b)
    for record in metric.exact_source_vertex_coordinates:
        u = Fraction(
            record.domain_coordinate.x.numerator,
            record.domain_coordinate.x.denominator,
        )
        v = Fraction(
            record.domain_coordinate.y.numerator,
            record.domain_coordinate.y.denominator,
        )
        rebuilt = tuple(
            origin[axis] + u * basis_a[axis] + v * basis_b[axis]
            for axis in range(3)
        )
        assert rebuilt == expected[record.source_vertex_id], record.source_vertex_id

    # Патч точно планарен, поэтому сертификат ТОЧНЫЙ, а не «в пределах бюджета»,
    # и перечень спроецированных вершин отсутствует как класс.
    assert metric.planarity_certificate.exact is True
    assert isinstance(metric.planarity_certificate, ExactSourcePlaneCertificateV1)
    assert _normal_of(metric) == (Fraction(1), Fraction(0), Fraction(0))


def test_the_plane_normal_is_written_in_canonical_primitive_form():
    """Одна плоскость — одна запись, независимо от того, чем её вывели.

    Нормаль задаёт плоскость с точностью до ненулевого множителя. Если её
    писать «как получилось», один и тот же геометрический факт попадает в
    дайджест разными числами, и дайджест начинает различать неразличимое.
    """

    for vertices, faces in (_axial_wall(), _slope_patch()):
        normal = _normal_of(_metric(vertices, faces, SOURCE_SNAP))
        assert normal == canonical_primitive_normal(normal)
        integers = [item.numerator for item in normal]
        assert all(item.denominator == 1 for item in normal)
        assert gcd(*(abs(item) for item in integers)) == 1
        assert next(item for item in integers if item) > 0
