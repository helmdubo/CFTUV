"""Deterministic exact-affine metric construction and filtered runtime view."""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from hashlib import sha256
from math import gcd
import math
from typing import Iterable

from .contracts.analysis import SourceVertexV1
from .contracts.metric import (
    AffineChartOrientationV1,
    AffineFrameSelectionLawV1,
    AffineReconstructionLawV1,
    Binary64Matrix2V1,
    Binary64Point2V1,
    Binary64Point3V1,
    Binary64SourceVertexCoordinateV1,
    Binary64Vector3V1,
    DerivedBinary64AffineViewV1,
    ExactMatrix2V1,
    ExactPoint2V1,
    ExactPoint3V1,
    ExactRationalV1,
    ExactSourcePlaneCertificateV1,
    ExactSourceVertexCoordinateV2,
    ExactVector3V1,
    GridSnappingLawV1,
    MetricSemanticIdentityLawV1,
    NearPlanarProjectionCertificateV1,
    NearPlanarResidualBudgetLawV1,
    PlanarityAdmissionLawV1,
    RationalAffinePlanarMetricV2,
    RuntimeMetricFallbackContractV1,
    RuntimeMetricFallbackLawV1,
    RuntimePlanarMetricV1,
    RuntimePredicateFilterContractV1,
    RuntimePredicateFilterLawV1,
    RuntimePredicateResultV1,
)
from .contracts.surface import SourceFaceV1
from .ids import (
    LineageId,
    PatchDomainId,
    PatchId,
    PlanarityCertificateId,
    ReferenceMetricId,
    RuntimeMetricId,
    SourceRevision,
    SourceVertexId,
)
from .numeric import LocalPoint3V1
from .outcomes import NamedOutcome
from .source_grid import resolve_source_grid


class PlanarMetricAdmissionError(ValueError):
    def __init__(self, outcome: NamedOutcome, message: str):
        super().__init__(message)
        self.outcome = outcome


def _stable_id(kind: str, *parts: object) -> str:
    payload = "\x1f".join((kind, *(str(item) for item in parts)))
    return f"{kind}:{sha256(payload.encode('utf-8')).hexdigest()[:24]}"


def _fraction_from_float(value: float) -> Fraction:
    numerator, denominator = float(value).as_integer_ratio()
    return Fraction(numerator, denominator)


def _rational(value: Fraction | int) -> ExactRationalV1:
    item = Fraction(value)
    return ExactRationalV1(item.numerator, item.denominator)


def fraction_from_exact(value: ExactRationalV1) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def _point3(value: LocalPoint3V1) -> tuple[Fraction, Fraction, Fraction]:
    return (
        _fraction_from_float(value.x),
        _fraction_from_float(value.y),
        _fraction_from_float(value.z),
    )


def _sub3(left, right):
    return tuple(a - b for a, b in zip(left, right, strict=True))


def _add3(left, right):
    return tuple(a + b for a, b in zip(left, right, strict=True))


def _scale3(vector, scalar):
    return tuple(item * scalar for item in vector)


def _dot3(left, right):
    return sum(
        (a * b for a, b in zip(left, right, strict=True)),
        Fraction(0),
    )


def _cross3(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _point3_record(value) -> ExactPoint3V1:
    return ExactPoint3V1(*(_rational(item) for item in value))


def _vector3_record(value) -> ExactVector3V1:
    return ExactVector3V1(*(_rational(item) for item in value))


def _matrix2_record(value) -> ExactMatrix2V1:
    return ExactMatrix2V1(
        _rational(value[0][0]),
        _rational(value[0][1]),
        _rational(value[1][0]),
        _rational(value[1][1]),
    )


_RELATIVE_EXTENT_FACTOR = Fraction(1, 10**7)
_MINIMUM_EXTENT = Fraction(1, 10**3)
_COORDINATE_ULP_MULTIPLIER = 64


def _newell_normal(positions, faces):
    """Нормаль Ньюэлла по ВСЕМ полигонам патча — собственная плоскость патча.

    Почему Ньюэлл, а не векторное произведение затравочной тройки вершин:
    тройка не управляет своей обусловленностью. У ската, выдавленного вдоль
    одной оси, первые два независимых вектора в порядке сортировки id идут
    ВДОЛЬ склона и почти коллинеарны; их произведение вырождается, и нормаль
    схлопывается на ось выдавливания. Замерено на полевом скате (нормаль
    (0,−0.4525,−0.8918), габарит 3.3125, хранение вершин float32): невязка к
    такой плоскости 3.312 — то есть ВЕСЬ габарит патча — при бюджете 3.31e-07,
    тогда как к плоскости Ньюэлла та же невязка 1.84e-08. Ньюэлл суммирует
    вклад каждого ребра каждого полигона, поэтому вырождение одной тройки его
    не двигает, а на точно планарном входе он точен.
    """

    x = y = z = Fraction(0)
    for face in faces:
        cycle = face.vertex_cycle
        count = len(cycle)
        for index in range(count):
            head = positions[cycle[index]]
            tail = positions[cycle[(index + 1) % count]]
            x += (head[1] - tail[1]) * (head[2] + tail[2])
            y += (head[2] - tail[2]) * (head[0] + tail[0])
            z += (head[0] - tail[0]) * (head[1] + tail[1])
    return (x, y, z)


def _canonical_primitive(vector):
    """Каноническая примитивная форма нормали: целые, gcd = 1, знак объявлен.

    Нормаль задаёт ПЛОСКОСТЬ, а плоскость не меняется от умножения нормали на
    ненулевое число. Поэтому запись нормали обязана быть канонической, иначе
    один и тот же геометрический факт печатается разными числами в зависимости
    от того, каким выводом его получили, и дайджест начинает различать то, что
    геометрически неразличимо.

    Знак: первая ненулевая координата положительна. Ориентация карты этим не
    теряется — она живёт отдельно, в `chart_orientation`, и выводится из знака
    площади граней в координатах карты. Привязывать знак нормали ещё и к
    обходу граней значило бы хранить одну величину дважды.
    """

    denominator = 1
    for item in vector:
        denominator = denominator * item.denominator // gcd(
            denominator, item.denominator
        )
    integers = [int(item * denominator) for item in vector]
    divisor = 0
    for item in integers:
        divisor = gcd(divisor, abs(item))
    if divisor == 0:
        raise ValueError("patch polygons do not define a non-zero plane normal")
    integers = [item // divisor for item in integers]
    if next(item for item in integers if item) < 0:
        integers = [-item for item in integers]
    return tuple(Fraction(item) for item in integers)


def patch_plane_normal(positions, faces):
    """Собственная плоскость патча: Ньюэлл плюс каноническая примитивная форма."""

    return _canonical_primitive(_newell_normal(positions, faces))


def _rational_centroid(positions, required_ids):
    """Опора плоскости — рациональный центроид вершин патча.

    Опора именно центроид, а не первая вершина: невязка меряется ОТ опоры, и
    вершина смещает её на постоянную величину, то есть в худшем случае удваивает.
    Центроид от такого смещения свободен по построению, и на точно планарном
    входе он лежит в плоскости точно (среднее нулей есть ноль), поэтому
    прежнее поведение точных патчей он не двигает.
    """

    count = len(required_ids)
    return tuple(
        sum(
            (positions[vertex_id][axis] for vertex_id in required_ids),
            Fraction(0),
        )
        / count
        for axis in range(3)
    )


@dataclass(frozen=True, slots=True)
class _NearPlanarFacts:
    planar_extent: Fraction
    residual_budget: Fraction
    max_residual_squared: Fraction
    projected: tuple[SourceVertexId, ...]


def _project_onto_exact_plane(*, positions, origin, normal, off_plane):
    """Спроецировать отклонившиеся вершины на плоскость — точно, в дробях.

    `p - ((p-o)·n / (n·n)) · n` считается в рациональных числах, поэтому вниз
    по конвейеру арифметика остаётся точной. Приблизителен только выбор
    плоскости, и он записывается в сертификат.

    Невязка сравнивается с бюджетом в квадрате, чтобы не вводить корень.
    """

    normal_squared = _dot3(normal, normal)
    max_residual_squared = Fraction(0)
    for vertex_id in off_plane:
        deviation = _dot3(_sub3(positions[vertex_id], origin), normal)
        residual_squared = deviation * deviation / normal_squared
        max_residual_squared = max(max_residual_squared, residual_squared)

    planar_extent = max(
        max(point[axis] for point in positions.values())
        - min(point[axis] for point in positions.values())
        for axis in range(3)
    )
    max_ulp = Fraction(0)
    for point in positions.values():
        for value in point:
            magnitude = math.ulp(float(value)) if value else math.ulp(1.0)
            max_ulp = max(max_ulp, Fraction(*magnitude.as_integer_ratio()))

    budget = max(
        _RELATIVE_EXTENT_FACTOR * max(planar_extent, _MINIMUM_EXTENT),
        _COORDINATE_ULP_MULTIPLIER * max_ulp,
    )
    if max_residual_squared > budget * budget:
        raise PlanarMetricAdmissionError(
            NamedOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED,
            "source deviates beyond the declared near-planar budget: "
            f"vertices={[item.value for item in off_plane]}",
        )

    for vertex_id in off_plane:
        scale = _dot3(_sub3(positions[vertex_id], origin), normal) / normal_squared
        positions[vertex_id] = tuple(
            positions[vertex_id][axis] - scale * normal[axis] for axis in range(3)
        )
    return _NearPlanarFacts(
        planar_extent=planar_extent,
        residual_budget=budget,
        max_residual_squared=max_residual_squared,
        projected=off_plane,
    )


def _require_plane_preserved(
    *, positions, required_ids, origin, normal, snapping_law
):
    """Закон решётки обязан оставить источник в его собственной плоскости.

    Проверка стоит здесь, а не в законе решётки, потому что плоскость — факт
    метрики, и именно метрика на неё опирается ниже: базис A/B строится как
    разности вершин, и если вершины вышли из плоскости, то `origin + u·A + v·B`
    перестаёт восстанавливать вход. Прежде это кончалось отказом восстановления
    без причины либо — на осевой решётке и наклонном патче — отказом бюджета,
    который винил меш вместо снапа.

    Закон, объявивший `snaps_in_patch_plane`, эту проверку проходит
    тождественно: узел его решётки есть точка вида `origin + u·A + v·B`.
    """

    if not snapping_law.snaps_source:
        return
    left = tuple(
        vertex_id
        for vertex_id in required_ids
        if _dot3(_sub3(positions[vertex_id], origin), normal) != 0
    )
    if not left:
        return
    raise PlanarMetricAdmissionError(
        NamedOutcome.GRID_SNAP_LEFT_THE_PATCH_PLANE,
        f"{snapping_law.value} moved the source out of its own patch plane "
        f"(normal={tuple(int(item) for item in normal)}): "
        + ", ".join(item.value for item in left),
    )


def _planarity_certificate(
    *, source_revision, patch_domain_id, normal, required_ids, near_planar_facts
):
    """Сертификат допуска плоскости: точный либо near-planar с записью невязки."""

    if near_planar_facts is None:
        return ExactSourcePlaneCertificateV1(
            certificate_id=PlanarityCertificateId(
                _stable_id(
                    "exact-source-plane",
                    source_revision.value,
                    patch_domain_id.value,
                )
            ),
            patch_domain_id=patch_domain_id,
            admission_law=PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1,
            exact=True,
            exact_plane_normal=_vector3_record(normal),
            source_vertex_ids=frozenset(required_ids),
            reconstruction_law=AffineReconstructionLawV1.O_PLUS_U_A_PLUS_V_B_V1,
        )
    return NearPlanarProjectionCertificateV1(
        certificate_id=PlanarityCertificateId(
            _stable_id(
                "near-planar-projection",
                source_revision.value,
                patch_domain_id.value,
            )
        ),
        patch_domain_id=patch_domain_id,
        source_revision=source_revision,
        admission_law=PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        exact=False,
        exact_plane_normal=_vector3_record(normal),
        source_vertex_ids=frozenset(required_ids),
        reconstruction_law=AffineReconstructionLawV1.O_PLUS_U_A_PLUS_V_B_V1,
        residual_budget_law=(
            NearPlanarResidualBudgetLawV1.RELATIVE_EXTENT_OR_ULP_V1
        ),
        relative_extent_factor=_rational(_RELATIVE_EXTENT_FACTOR),
        minimum_extent=_rational(_MINIMUM_EXTENT),
        coordinate_ulp_multiplier=_COORDINATE_ULP_MULTIPLIER,
        planar_extent=_rational(near_planar_facts.planar_extent),
        residual_budget=_rational(near_planar_facts.residual_budget),
        max_residual=_rational(near_planar_facts.max_residual_squared),
        projected_source_vertex_ids=frozenset(near_planar_facts.projected),
    )


def build_rational_affine_planar_metric(
    *,
    source_revision: SourceRevision,
    patch_domain_id: PatchDomainId,
    owner_patch_id: PatchId,
    source_vertices: Iterable[SourceVertexV1],
    source_faces: Iterable[SourceFaceV1],
    source_lineage: frozenset[LineageId] = frozenset(),
    planarity_policy: PlanarityAdmissionLawV1 = (
        PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1
    ),
    grid_policy: GridSnappingLawV1 = GridSnappingLawV1.UNSNAPPED_EXACT_V1,
) -> RationalAffinePlanarMetricV2:
    """Build the canonical exact chart from stable source identities.

    Blender binary64 coordinates are interpreted as their exact IEEE-754
    rational values.  No fit, residual budget, snapping or normalization is
    performed.
    """

    vertex_by_id = {
        item.vertex_id: item
        for item in source_vertices
        if isinstance(item.position, LocalPoint3V1)
    }
    faces = tuple(
        sorted(
            (item for item in source_faces if item.patch_id == owner_patch_id),
            key=lambda item: item.face_id.value,
        )
    )
    required_ids = tuple(
        sorted(
            {
                vertex_id
                for face in faces
                for vertex_id in face.vertex_cycle
            },
            key=lambda item: item.value,
        )
    )
    if len(required_ids) < 3 or any(
        item not in vertex_by_id for item in required_ids
    ):
        raise ValueError(
            "exact affine metric requires at least three total source vertices"
        )
    positions = {
        vertex_id: _point3(vertex_by_id[vertex_id].position)
        for vertex_id in required_ids
    }
    # ЗАКОН ПЛОСКОСТИ. Собственная плоскость патча выводится из фактов патча —
    # полигонов — и выводится ПЕРВОЙ: и бюджет невязки, и решётка меряются от
    # неё, а не она от них. Прежний порядок был обратным (плоскость получалась
    # как побочный продукт затравочной тройки базиса уже ПОСЛЕ привязки), и
    # ровно поэтому наклонный патч мерялся против чужой плоскости.
    normal = patch_plane_normal(positions, faces)
    anchor = _rational_centroid(positions, required_ids)
    off_plane = tuple(
        vertex_id
        for vertex_id in required_ids
        if _dot3(_sub3(positions[vertex_id], anchor), normal) != 0
    )
    near_planar_facts = None
    if off_plane:
        if planarity_policy is not PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1:
            raise PlanarMetricAdmissionError(
                NamedOutcome.RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED,
                "EXACT_SOURCE_PLANE_V1 rejected source vertices: "
                + ", ".join(item.value for item in off_plane),
            )
        near_planar_facts = _project_onto_exact_plane(
            positions=positions, origin=anchor, normal=normal, off_plane=off_plane
        )
    # Бюджет судит ИСХОДНЫЙ вход, поэтому и стоит до решётки: он описывает шум
    # представления автора, а сдвиг решётки — объявленная правка геометрии, у
    # неё свой сертификат и свои границы окна. Мерить объявленный сдвиг
    # бюджетом шума значило бы сравнивать величины двух разных законов.
    grid_facts = resolve_source_grid(
        positions=positions,
        faces=faces,
        snapping_law=grid_policy,
        plane=(normal, anchor),
    )
    positions = grid_facts.positions
    origin_id = required_ids[0]
    origin = positions[origin_id]
    _require_plane_preserved(
        positions=positions,
        required_ids=required_ids,
        origin=origin,
        normal=normal,
        snapping_law=grid_policy,
    )
    candidates = tuple(
        (
            vertex_id,
            _sub3(positions[vertex_id], origin),
        )
        for vertex_id in required_ids[1:]
    )
    first = next(
        (vector for _, vector in candidates if any(vector)),
        None,
    )
    if first is None:
        raise ValueError("source vertices do not define a non-zero basis A")
    second = next(
        (
            vector
            for _, vector in candidates
            if any(_cross3(first, vector))
        ),
        None,
    )
    if second is None:
        raise ValueError(
            "source vertices do not define linearly independent A/B"
        )

    g00 = _dot3(first, first)
    g01 = _dot3(first, second)
    g11 = _dot3(second, second)
    determinant = g00 * g11 - g01 * g01
    if determinant <= 0:
        raise ValueError("exact affine Gram matrix is not positive definite")
    inverse = (
        (g11 / determinant, -g01 / determinant),
        (-g01 / determinant, g00 / determinant),
    )
    coordinates: dict[SourceVertexId, tuple[Fraction, Fraction]] = {}
    for vertex_id in required_ids:
        relative = _sub3(positions[vertex_id], origin)
        rhs_a = _dot3(first, relative)
        rhs_b = _dot3(second, relative)
        u = inverse[0][0] * rhs_a + inverse[0][1] * rhs_b
        v = inverse[1][0] * rhs_a + inverse[1][1] * rhs_b
        reconstructed = _add3(
            origin,
            _add3(_scale3(first, u), _scale3(second, v)),
        )
        if reconstructed != positions[vertex_id]:
            raise ValueError(
                f"exact affine reconstruction failed for {vertex_id}"
            )
        coordinates[vertex_id] = (u, v)

    orientation_sign = None
    for face in faces:
        cycle = tuple(coordinates[item] for item in face.vertex_cycle)
        twice_area = sum(
            (
                cycle[index][0]
                * cycle[(index + 1) % len(cycle)][1]
                - cycle[index][1]
                * cycle[(index + 1) % len(cycle)][0]
                for index in range(len(cycle))
            ),
            Fraction(0),
        )
        if twice_area:
            orientation_sign = 1 if twice_area > 0 else -1
            break
    if orientation_sign is None:
        raise ValueError("owner Patch has no non-degenerate oriented face")
    chart_orientation = (
        AffineChartOrientationV1.COORDINATE_CCW_MATCHES_OWNER_PATCH
        if orientation_sign > 0
        else AffineChartOrientationV1.COORDINATE_CW_MATCHES_OWNER_PATCH
    )
    metric_id = ReferenceMetricId(
        _stable_id(
            "reference-metric",
            source_revision.value,
            patch_domain_id.value,
            origin_id.value,
            *(item.value for item in required_ids),
        )
    )
    certificate = _planarity_certificate(
        source_revision=source_revision,
        patch_domain_id=patch_domain_id,
        normal=normal,
        required_ids=required_ids,
        near_planar_facts=near_planar_facts,
    )
    return _metric_record(
        metric_id=metric_id,
        patch_domain_id=patch_domain_id,
        source_revision=source_revision,
        origin=origin,
        first=first,
        second=second,
        gram=((g00, g01), (g01, g11)),
        inverse=inverse,
        required_ids=required_ids,
        coordinates=coordinates,
        chart_orientation=chart_orientation,
        certificate=certificate,
        source_lineage=source_lineage,
        grid_certificate=grid_facts.certificate,
    )


def _metric_record(
    *,
    metric_id,
    patch_domain_id,
    source_revision,
    origin,
    first,
    second,
    gram,
    inverse,
    required_ids,
    coordinates,
    chart_orientation,
    certificate,
    source_lineage,
    grid_certificate,
) -> RationalAffinePlanarMetricV2:
    """Собрать запись метрики. Вынесено ради бюджета длины строителя."""

    return RationalAffinePlanarMetricV2(
        reference_metric_id=metric_id,
        patch_domain_id=patch_domain_id,
        source_revision=source_revision,
        exact_origin=_point3_record(origin),
        exact_basis_a=_vector3_record(first),
        exact_basis_b=_vector3_record(second),
        exact_gram_matrix=_matrix2_record(gram),
        exact_inverse_gram_matrix=_matrix2_record(inverse),
        exact_source_vertex_coordinates=frozenset(
            ExactSourceVertexCoordinateV2(
                source_vertex_id=vertex_id,
                domain_coordinate=ExactPoint2V1(
                    _rational(coordinates[vertex_id][0]),
                    _rational(coordinates[vertex_id][1]),
                ),
            )
            for vertex_id in required_ids
        ),
        chart_orientation=chart_orientation,
        frame_selection_law=(
            AffineFrameSelectionLawV1.CANONICAL_SOURCE_VERTEX_BASIS_V1
        ),
        planarity_certificate=certificate,
        source_lineage=source_lineage,
        grid_certificate=grid_certificate,
    )


def build_runtime_planar_metric(
    reference_metric: RationalAffinePlanarMetricV2,
) -> RuntimePlanarMetricV1:
    """Derive a non-authoritative finite binary64 view."""

    def number(value: ExactRationalV1) -> float:
        result = float(fraction_from_exact(value))
        if not math.isfinite(result):
            raise OverflowError("exact metric is outside binary64 range")
        return result

    def point3(value: ExactPoint3V1) -> Binary64Point3V1:
        return Binary64Point3V1(
            number(value.x), number(value.y), number(value.z)
        )

    def vector3(value: ExactVector3V1) -> Binary64Vector3V1:
        return Binary64Vector3V1(
            number(value.x), number(value.y), number(value.z)
        )

    def matrix(value: ExactMatrix2V1) -> Binary64Matrix2V1:
        return Binary64Matrix2V1(
            number(value.m00),
            number(value.m01),
            number(value.m10),
            number(value.m11),
        )

    runtime_id = RuntimeMetricId(
        _stable_id("runtime-metric", reference_metric.reference_metric_id.value)
    )
    return RuntimePlanarMetricV1(
        runtime_metric_id=runtime_id,
        reference_metric_id=reference_metric.reference_metric_id,
        derived_binary64_view=DerivedBinary64AffineViewV1(
            origin=point3(reference_metric.exact_origin),
            basis_a=vector3(reference_metric.exact_basis_a),
            basis_b=vector3(reference_metric.exact_basis_b),
            gram_matrix=matrix(reference_metric.exact_gram_matrix),
            inverse_gram_matrix=matrix(
                reference_metric.exact_inverse_gram_matrix
            ),
            source_vertex_coordinates=frozenset(
                Binary64SourceVertexCoordinateV1(
                    source_vertex_id=item.source_vertex_id,
                    domain_coordinate=Binary64Point2V1(
                        number(item.domain_coordinate.x),
                        number(item.domain_coordinate.y),
                    ),
                )
                for item in reference_metric.exact_source_vertex_coordinates
            ),
        ),
        predicate_filter_contract=RuntimePredicateFilterContractV1(
            filter_law=(
                RuntimePredicateFilterLawV1.BINARY64_OUTWARD_INTERVAL_V1
            ),
            uncertain_result=(
                RuntimePredicateResultV1.EXACT_FALLBACK_REQUIRED
            ),
            exact_zero_requires_fallback=True,
            semantic_identity_law=(
                MetricSemanticIdentityLawV1.EXACT_CONSTRUCTION_CERTIFICATES_ONLY
            ),
        ),
        fallback_contract=RuntimeMetricFallbackContractV1(
            fallback_law=(
                RuntimeMetricFallbackLawV1.AUTHORITATIVE_REFERENCE_METRIC_V2
            ),
            authoritative_reference_metric_id=(
                reference_metric.reference_metric_id
            ),
            rounded_runtime_reconstruction_forbidden=True,
        ),
    )


@dataclass(frozen=True, slots=True)
class Binary64IntervalV1:
    lower: float
    upper: float

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.lower)
            or not math.isfinite(self.upper)
            or self.lower > self.upper
        ):
            raise ValueError("invalid finite binary64 interval")


@dataclass(frozen=True, slots=True)
class FilteredPredicateDecisionV1:
    result: RuntimePredicateResultV1
    semantic_sign: int | None
    exact_fallback_used: bool


class RuntimePredicateTelemetryV1:
    __slots__ = (
        "filtered_predicate_count",
        "fast_path_certified_count",
        "exact_fallback_count",
    )

    def __init__(self) -> None:
        self.filtered_predicate_count = 0
        self.fast_path_certified_count = 0
        self.exact_fallback_count = 0

    @property
    def fallback_fraction(self) -> str:
        if not self.filtered_predicate_count:
            return "0/0"
        return (
            f"{self.exact_fallback_count}/"
            f"{self.filtered_predicate_count}"
        )


def _outward(value: float) -> Binary64IntervalV1:
    if not math.isfinite(value):
        raise OverflowError("non-finite binary64 predicate candidate")
    return Binary64IntervalV1(
        math.nextafter(value, -math.inf),
        math.nextafter(value, math.inf),
    )


def _interval_subtract(
    left: Binary64IntervalV1,
    right: Binary64IntervalV1,
) -> Binary64IntervalV1:
    lower = left.lower - right.upper
    upper = left.upper - right.lower
    return Binary64IntervalV1(
        math.nextafter(lower, -math.inf),
        math.nextafter(upper, math.inf),
    )


def _interval_multiply(
    left: Binary64IntervalV1,
    right: Binary64IntervalV1,
) -> Binary64IntervalV1:
    values = (
        left.lower * right.lower,
        left.lower * right.upper,
        left.upper * right.lower,
        left.upper * right.upper,
    )
    if not all(math.isfinite(item) for item in values):
        raise OverflowError("binary64 interval multiplication overflow")
    return Binary64IntervalV1(
        math.nextafter(min(values), -math.inf),
        math.nextafter(max(values), math.inf),
    )


def filtered_orient2d(
    first: Binary64Point2V1,
    second: Binary64Point2V1,
    third: Binary64Point2V1,
) -> RuntimePredicateResultV1:
    """Conservative binary64 orientation filter.

    Every input and arithmetic operation is enclosed by an outward-rounded
    interval.  An interval containing zero cannot certify topology.
    """

    ax = _interval_subtract(_outward(second.x), _outward(first.x))
    ay = _interval_subtract(_outward(second.y), _outward(first.y))
    bx = _interval_subtract(_outward(third.x), _outward(first.x))
    by = _interval_subtract(_outward(third.y), _outward(first.y))
    determinant = _interval_subtract(
        _interval_multiply(ax, by),
        _interval_multiply(ay, bx),
    )
    if determinant.lower > 0:
        return RuntimePredicateResultV1.CERTIFIED_POSITIVE
    if determinant.upper < 0:
        return RuntimePredicateResultV1.CERTIFIED_NEGATIVE
    return RuntimePredicateResultV1.EXACT_FALLBACK_REQUIRED


def resolve_orient2d(
    *,
    reference_metric: RationalAffinePlanarMetricV2,
    runtime_metric: RuntimePlanarMetricV1,
    source_vertex_ids: tuple[
        SourceVertexId, SourceVertexId, SourceVertexId
    ],
    telemetry: RuntimePredicateTelemetryV1 | None = None,
) -> FilteredPredicateDecisionV1:
    """Resolve one source-vertex predicate against authoritative exact facts."""

    if runtime_metric.reference_metric_id != reference_metric.reference_metric_id:
        raise ValueError("runtime metric references another exact authority")
    exact_by_id = {
        item.source_vertex_id: (
            fraction_from_exact(item.domain_coordinate.x),
            fraction_from_exact(item.domain_coordinate.y),
        )
        for item in reference_metric.exact_source_vertex_coordinates
    }
    runtime_by_id = {
        item.source_vertex_id: item.domain_coordinate
        for item in runtime_metric.derived_binary64_view.source_vertex_coordinates
    }
    try:
        runtime_points = tuple(
            runtime_by_id[item] for item in source_vertex_ids
        )
        exact_points = tuple(exact_by_id[item] for item in source_vertex_ids)
    except KeyError as exc:
        raise ValueError("predicate source vertex is outside the metric") from exc
    candidate = filtered_orient2d(*runtime_points)
    if telemetry is not None:
        telemetry.filtered_predicate_count += 1
    if candidate is RuntimePredicateResultV1.CERTIFIED_POSITIVE:
        if telemetry is not None:
            telemetry.fast_path_certified_count += 1
        return FilteredPredicateDecisionV1(candidate, 1, False)
    if candidate is RuntimePredicateResultV1.CERTIFIED_NEGATIVE:
        if telemetry is not None:
            telemetry.fast_path_certified_count += 1
        return FilteredPredicateDecisionV1(candidate, -1, False)
    if telemetry is not None:
        telemetry.exact_fallback_count += 1
    first, second, third = exact_points
    determinant = (
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )
    sign = (determinant > 0) - (determinant < 0)
    result = {
        -1: RuntimePredicateResultV1.CERTIFIED_NEGATIVE,
        0: RuntimePredicateResultV1.CERTIFIED_ZERO,
        1: RuntimePredicateResultV1.CERTIFIED_POSITIVE,
    }[sign]
    return FilteredPredicateDecisionV1(result, sign, True)
