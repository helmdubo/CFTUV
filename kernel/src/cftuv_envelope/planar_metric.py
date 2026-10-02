"""Deterministic exact-affine metric construction and filtered runtime view."""

from __future__ import annotations

from dataclasses import dataclass, replace
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
    CurvatureLadderPolicyV1,
    DerivedBinary64AffineViewV1,
    ExactMatrix2V1,
    ExactPoint2V1,
    ExactPoint3V1,
    ExactRationalV1,
    ExactSourcePlaneCertificateV1,
    ExactSourceVertexCoordinateV2,
    ExactVector3V1,
    EmbeddingCertifiedRationalAffinePlanarMetricV1,
    GridSnappingLawV1,
    MetricSemanticIdentityLawV1,
    NearPlanarFramePolicyV1,
    PRODUCT_SKIRT_ABSOLUTE_BUDGET,
    NearPlanarLiftLawV1,
    NearPlanarProjectionCertificateV1,
    NearPlanarResidualBudgetLawV1,
    NearPlanarWidthDistortionCertificateV1,
    PlanarityAdmissionLawV1,
    RationalAffinePlanarMetricV2,
    RuntimeMetricFallbackContractV1,
    RuntimeMetricFallbackLawV1,
    RuntimePlanarMetricV1,
    RuntimePredicateFilterContractV1,
    RuntimePredicateFilterLawV1,
    RuntimePredicateResultV1,
)
from ._embedding import (
    build_projection_embedding_certificate,
    patch_plane_normal as _embedding_patch_plane_normal,
    projection_violation,
)
from ._developable import build_developable_chart
from ._plane_basis import chart_of_positions, reduced_frame
from ._width_distortion import (
    build_width_distortion_certificate,
    width_distortion_refusal_text,
    width_distortion_violation,
)
from .contracts.surface import SourceFaceV1, SurfaceTriangleV1
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
from .numeric import LocalPoint3V1, PlaneNormalUndefinedError
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

# Допуск кривизны юбки живёт РЯДОМ СО СВОИМ ЗАКОНОМ, в `contracts.metric`:
# число принадлежит закону, а не построителю, и его обязан читать ещё и
# валидатор. Положить его сюда не выйдет и технически — `validation_metric`
# импортировал бы `planar_metric` через цикл
# (`planar_metric` -> `source_grid` -> `reference` -> `validation`).
# Имя импортировано, поэтому читатель бюджетов видит его здесь наравне с
# тремя константами выше.


def patch_plane_normal(positions, faces):
    """Собственная плоскость: общий exact Newell-закон builder/validator."""

    return _embedding_patch_plane_normal(positions, faces)


def _plane_anchor(positions, required_ids):
    """Опора плоскости — первая вершина кадра, та же, что и начало координат.

    Опора задаёт СМЕЩЕНИЕ плоскости вдоль нормали, направление даёт Ньюэлл.
    Взята вершина, а не центроид, по двум причинам, и обе про записи, а не про
    точность: `projected_source_vertex_ids` обязан называть ТЕ вершины, которые
    действительно отклонились, — от центроида отклоняются все, и перечень
    перестаёт быть диагностикой; и на точно планарном входе вершина даёт ровно
    прежний ноль, поэтому прежнее поведение не двигается вовсе.
    """

    return positions[required_ids[0]]


@dataclass(frozen=True, slots=True)
class _NearPlanarFacts:
    planar_extent: Fraction
    max_coordinate_ulp: Fraction
    residual_budget: Fraction
    residual_budget_law: NearPlanarResidualBudgetLawV1
    max_residual_squared: Fraction
    projected: tuple[SourceVertexId, ...]


def _plane_extent_and_ulp(positions):
    """Габарит патча и наибольший ULP его координат — вход обоих бюджетов."""

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
    return planar_extent, max_ulp


def _representation_budget(planar_extent, max_ulp):
    """Бюджет шума ПРЕДСТАВЛЕНИЯ: сколько binary64 врёт о задуманной плоскости."""

    return max(
        _RELATIVE_EXTENT_FACTOR * max(planar_extent, _MINIMUM_EXTENT),
        _COORDINATE_ULP_MULTIPLIER * max_ulp,
    )


def _admission_budget(
    *, planar_extent, max_ulp, grid_certificate
) -> tuple[Fraction, NearPlanarResidualBudgetLawV1]:
    """Насколько источнику разрешено отклоняться от собственной плоскости.

    Власть допуска — ПРОДУКТОВАЯ, решением владельца от 2026-08-01. Величина
    едина для обоих законов решётки и равна
    `PRODUCT_SKIRT_ABSOLUTE_BUDGET` (1/80 = 1.25 см); основания — в докстроке
    константы.

    ЧТО ЭТИМ ВЫТЕСНЕНО и почему это не потеря. Прежде допуск брался у того
    закона, который источник ДВИГАЕТ: у привязывающего — ячейка выбранного
    шага, у `UNSNAPPED_EXACT_V1` — шум представления. Рассуждение было верным
    и остаётся верным: осевая решётка рвёт наклонную плоскость, каждая
    координата садится в свой узел независимо, и внесённое ею отклонение
    ограничено ячейкой (замер на полевом скате: 8.65e-06 при ячейке 6.10e-05 —
    своё; 1.87e-02 при той же ячейке — чужое). Но оба закона отвечали на
    вопрос «сколько непланарности внёс КОНВЕЙЕР», а продукту нужен ответ на
    другой: «сколько непланарности несёт САМА ПОВЕРХНОСТЬ, и сколько её юбка
    декали готова стерпеть». Кривые крыши `building.004` (0.15 мм – 1.2 см)
    отвергались обоими законами честно и бесполезно: отказ был верен закону и
    бесполезен цели.

    Прежние законы НЕ удалены. Они остаются членами перечисления и остаются
    пересчитываемыми валидатором: сертификаты, которые их объявляют, обязаны
    воспроизводить свои числа, а красные контроли на подмену закона — жить.
    Вытеснены они только из ВЫБОРА допуска, и вытеснены решением владельца, а
    не выводом кода.

    Возвращается ПАРА: число и тот закон, который его дал. Прежде возвращалось
    одно число, а сертификат безусловно писал `RELATIVE_EXTENT_OR_ULP_V1` —
    поэтому у всякого снапнутого домена объявленный закон и записанное число
    расходились. Пара делает расхождение невозможным по построению.

    `planar_extent`, `max_ulp` и `grid_certificate` остаются в подписи
    намеренно: сертификат по-прежнему ЗАПИСЫВАЕТ эти величины, и их отсутствие
    здесь означало бы, что запись перестала быть проверяемой.
    """

    return (
        PRODUCT_SKIRT_ABSOLUTE_BUDGET,
        NearPlanarResidualBudgetLawV1.PRODUCT_SKIRT_ABSOLUTE_V1,
    )


def _budget_refusal_text(
    *, max_residual_squared, budget, budget_law, grid_certificate, off_plane
) -> str:
    """Отказ бюджета обязан нести числа, а не только имена вершин.

    Прежде печатался один перечень вершин. Поле не могло по нему отличить
    честный отказ (подлинная кривизна) от неверного бюджета (сравнение с
    величиной чужого закона), и разбор полевого случая стоил ручного пересчёта
    Ньюэлла дробями. Здесь названо всё, из чего сложилось решение: обе
    сравниваемые величины, сам бюджет, ЗАКОН, который его дал, и шаг решётки,
    когда бюджет пришёл от ячейки.

    Обе сравниваемые величины — квадраты, и названы квадратами. Корень из
    рационального в общем случае не представим точно, поэтому сравнение идёт в
    квадратах; печатать «невязку» вместо квадрата невязки значило бы
    повторить ложь имени поля. Числа выводятся как binary64-приближения — это
    диагностика для чтения человеком, а не величина, на которую кто-то
    ссылается; точные величины лежат в сертификате.
    """

    step = grid_certificate.window_step
    grid_step = (
        "none"
        if step is None or not grid_certificate.snapping_law.snaps_source
        else f"{float(Fraction(step.numerator, step.denominator)):.6e}"
    )
    return (
        "source deviates beyond the declared near-planar budget: "
        f"max_residual_squared={float(max_residual_squared):.6e} "
        f"> residual_budget_squared={float(budget * budget):.6e} "
        f"(residual_budget={float(budget):.6e}, "
        f"residual_budget_law={budget_law.value}, grid_step={grid_step}); "
        f"vertices={[item.value for item in off_plane]}"
    )


def _project_onto_exact_plane(
    *, grid_certificate, positions, origin, normal, off_plane, judge_residual=True
):
    """Спроецировать отклонившиеся вершины на плоскость — точно, в дробях.

    `p - ((p-o)·n / (n·n)) · n` считается в рациональных числах, поэтому вниз
    по конвейеру арифметика остаётся точной. Приблизителен только выбор
    плоскости, и он записывается в сертификат.

    Невязка сравнивается с бюджетом в квадрате, чтобы не вводить корень. При
    укладке на треугольники источника (`judge_residual=False`) она ВСЁ РАВНО
    считается и записывается, но не судит: меш лежит на поверхности, и
    расстояние до плоскости ему безразлично. Судят искажение ширины и вложение.
    """

    normal_squared = _dot3(normal, normal)
    max_residual_squared = Fraction(0)
    for vertex_id in off_plane:
        deviation = _dot3(_sub3(positions[vertex_id], origin), normal)
        residual_squared = deviation * deviation / normal_squared
        max_residual_squared = max(max_residual_squared, residual_squared)

    planar_extent, max_ulp = _plane_extent_and_ulp(positions)
    budget, budget_law = _admission_budget(
        planar_extent=planar_extent,
        max_ulp=max_ulp,
        grid_certificate=grid_certificate,
    )
    if judge_residual and max_residual_squared > budget * budget:
        raise PlanarMetricAdmissionError(
            NamedOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED,
            _budget_refusal_text(
                max_residual_squared=max_residual_squared,
                budget=budget,
                budget_law=budget_law,
                grid_certificate=grid_certificate,
                off_plane=off_plane,
            ),
        )
    for vertex_id in off_plane:
        scale = _dot3(_sub3(positions[vertex_id], origin), normal) / normal_squared
        positions[vertex_id] = tuple(
            positions[vertex_id][axis] - scale * normal[axis] for axis in range(3)
        )
    return _NearPlanarFacts(
        planar_extent=planar_extent,
        max_coordinate_ulp=max_ulp,
        residual_budget=budget,
        residual_budget_law=budget_law,
        max_residual_squared=max_residual_squared,
        projected=off_plane,
    )


def _resolve_patch_plane(
    *,
    positions,
    faces,
    required_ids,
    planarity_policy,
    grid_certificate,
    judge_residual=True,
):
    """Плоскость патча и то, что пришлось в неё положить.

    ЗАКОН ПЛОСКОСТИ живёт здесь. Плоскость выводится из фактов патча — Ньюэлл
    по всем его полигонам — и выводится ДО базиса. Прежде порядок был обратным:
    плоскость получалась побочным продуктом затравочной тройки базиса, то есть
    выбиралась тремя вершинами, обусловленностью которых никто не управлял. У
    ската, выдавленного вдоль оси, эта тройка почти коллинеарна, нормаль
    схлопывается на ось выдавливания, и патч мерялся против чужой плоскости —
    полевой отказ на `building.002` был именно этим.

    `positions` правятся на месте: проекция кладёт отклонившиеся вершины в
    плоскость точно, и ниже базис строится уже от них.
    """

    normal = patch_plane_normal(positions, faces)
    anchor = _plane_anchor(positions, required_ids)
    off_plane = tuple(
        vertex_id
        for vertex_id in required_ids
        if _dot3(_sub3(positions[vertex_id], anchor), normal) != 0
    )
    if not off_plane:
        return normal, None
    if planarity_policy is not PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1:
        raise PlanarMetricAdmissionError(
            NamedOutcome.RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED,
            "EXACT_SOURCE_PLANE_V1 rejected source vertices: "
            + ", ".join(item.value for item in off_plane),
        )
    return normal, _project_onto_exact_plane(
        grid_certificate=grid_certificate,
        positions=positions,
        origin=anchor,
        normal=normal,
        off_plane=off_plane,
        judge_residual=judge_residual,
    )


def _planarity_certificate(
    *,
    source_revision,
    patch_domain_id,
    normal,
    required_ids,
    near_planar_facts,
    width_distortion: NearPlanarWidthDistortionCertificateV1 | None = None,
    lift_law: NearPlanarLiftLawV1 = NearPlanarLiftLawV1.CERTIFIED_PLANE_V1,
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
        # Записывается ТОТ закон, который дал число, а не один и тот же всегда.
        residual_budget_law=near_planar_facts.residual_budget_law,
        relative_extent_factor=_rational(_RELATIVE_EXTENT_FACTOR),
        minimum_extent=_rational(_MINIMUM_EXTENT),
        coordinate_ulp_multiplier=_COORDINATE_ULP_MULTIPLIER,
        max_coordinate_ulp=_rational(near_planar_facts.max_coordinate_ulp),
        planar_extent=_rational(near_planar_facts.planar_extent),
        residual_budget=_rational(near_planar_facts.residual_budget),
        max_residual_squared=_rational(near_planar_facts.max_residual_squared),
        projected_source_vertex_ids=frozenset(near_planar_facts.projected),
        width_distortion=width_distortion,
        lift_law=lift_law,
    )


@dataclass(frozen=True, slots=True)
class _AffineFrameFacts:
    origin_id: SourceVertexId
    origin: tuple
    basis_a_id: SourceVertexId
    basis_a: tuple
    basis_b_id: SourceVertexId
    basis_b: tuple
    gram: tuple
    inverse: tuple
    coordinates: dict


def _source_scope(*, owner_patch_id, source_vertices, source_faces):
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
            {vertex_id for face in faces for vertex_id in face.vertex_cycle},
            key=lambda item: item.value,
        )
    )
    if len(required_ids) < 3 or any(item not in vertex_by_id for item in required_ids):
        raise ValueError(
            "exact affine metric requires at least three total source vertices"
        )
    positions = {
        vertex_id: _point3(vertex_by_id[vertex_id].position)
        for vertex_id in required_ids
    }
    return faces, required_ids, positions


def _affine_frame(positions, required_ids) -> _AffineFrameFacts:
    origin_id = required_ids[0]
    origin = positions[origin_id]
    candidates = tuple(
        (vertex_id, _sub3(positions[vertex_id], origin))
        for vertex_id in required_ids[1:]
    )
    first_pair = next(
        ((vertex_id, vector) for vertex_id, vector in candidates if any(vector)),
        None,
    )
    if first_pair is None:
        raise ValueError("source vertices do not define a non-zero basis A")
    first_id, first = first_pair
    second_pair = next(
        (
            (vertex_id, vector)
            for vertex_id, vector in candidates
            if any(_cross3(first, vector))
        ),
        None,
    )
    if second_pair is None:
        raise ValueError("source vertices do not define linearly independent A/B")
    second_id, second = second_pair
    g00, g01, g11 = _dot3(first, first), _dot3(first, second), _dot3(second, second)
    determinant = g00 * g11 - g01 * g01
    if determinant <= 0:
        raise ValueError("exact affine Gram matrix is not positive definite")
    inverse = (
        (g11 / determinant, -g01 / determinant),
        (-g01 / determinant, g00 / determinant),
    )
    coordinates = {}
    for vertex_id in required_ids:
        relative = _sub3(positions[vertex_id], origin)
        rhs_a, rhs_b = _dot3(first, relative), _dot3(second, relative)
        u = inverse[0][0] * rhs_a + inverse[0][1] * rhs_b
        v = inverse[1][0] * rhs_a + inverse[1][1] * rhs_b
        reconstructed = _add3(origin, _add3(_scale3(first, u), _scale3(second, v)))
        if reconstructed != positions[vertex_id]:
            raise ValueError(f"exact affine reconstruction failed for {vertex_id}")
        coordinates[vertex_id] = (u, v)
    return _AffineFrameFacts(
        origin_id,
        origin,
        first_id,
        first,
        second_id,
        second,
        ((g00, g01), (g01, g11)),
        inverse,
        coordinates,
    )


def _chart_orientation(coordinates, faces):
    signs = []
    for face in faces:
        cycle = tuple(coordinates[item] for item in face.vertex_cycle)
        twice_area = sum(
            (
                cycle[index][0] * cycle[(index + 1) % len(cycle)][1]
                - cycle[index][1] * cycle[(index + 1) % len(cycle)][0]
                for index in range(len(cycle))
            ),
            Fraction(0),
        )
        signs.append((twice_area > 0) - (twice_area < 0))
    orientation_sign = next((item for item in signs if item), None)
    if orientation_sign is None:
        raise ValueError("owner Patch has no non-degenerate oriented face")
    orientation = (
        AffineChartOrientationV1.COORDINATE_CCW_MATCHES_OWNER_PATCH
        if orientation_sign > 0
        else AffineChartOrientationV1.COORDINATE_CW_MATCHES_OWNER_PATCH
    )
    return orientation_sign, orientation


def _projection_embedding(
    *, near_planar_facts, snapped, faces, frame, normal, sign, enforce_embedding
):
    if near_planar_facts is None:
        return None
    certificate = build_projection_embedding_certificate(
        before=snapped,
        projected=frame.coordinates,
        faces=faces,
        normal=normal,
        expected_orientation_sign=sign,
    )
    outcome = projection_violation(certificate)
    if enforce_embedding and outcome is not None:
        raise PlanarMetricAdmissionError(
            outcome,
            "near-planar projection did not preserve the embedding: "
            f"{certificate!r}",
        )
    return certificate


def _width_distortion_record(
    *,
    near_planar_facts,
    surface_triangles,
    source_revision,
    patch_domain_id,
    snapped,
    normal,
    faces,
    required_ids,
    judged=False,
):
    """Сертификат искажения ширины: пишется, когда есть что мерить.

    Точная плоскость искажения не имеет (проекция тождественна), поэтому у неё
    записи нет, и байты планарных доменов не двигаются. Вызвавший, не давший
    треугольников, получает `None` — «не измерялось», а не «измерено
    хорошо». При `judged` (укладка на треугольники источника) запись СУДИТ:
    вырожденный треугольник и ширина сверх бюджета — именованные отказы с
    числами, а отсутствие треугольников — дефект входа.
    """

    if near_planar_facts is None:
        return None
    if surface_triangles is None:
        if judged:
            raise PlanarMetricAdmissionError(
                NamedOutcome.NEAR_PLANAR_OWNER_SURFACE_TRIANGLES_UNAVAILABLE,
                "lifting onto source triangles needs the surface triangles of "
                "the owner Patch, and none were given",
            )
        return None
    record = build_width_distortion_certificate(
        source_revision=source_revision,
        patch_domain_id=patch_domain_id,
        snapped=snapped,
        normal=normal,
        faces=faces,
        triangles=tuple(surface_triangles),
        required_ids=required_ids,
    )
    outcome = width_distortion_violation(record) if judged else None
    if outcome is not None:
        raise PlanarMetricAdmissionError(
            outcome, width_distortion_refusal_text(record)
        )
    return record


def _near_planar_frame(
    *, near_planar_facts, frame_policy, frame, positions, normal, grid_certificate
):
    """Репер near-planar домена по политике: `(кадр, закон репера)`.

    Приведённый целочисленный базис применяется ТОЛЬКО к домену со
    спроецированными вершинами: у точной плоскости проекции нет, знаменателей она
    не наращивает, и закон остаётся прежним (байты планарных метрик не
    двигаются). Начало и позиции те же — новые только базис, Грам и координаты
    `(u, v)`, и они по-прежнему восстанавливают спроецированные позиции ТОЧНО.
    """

    canonical = AffineFrameSelectionLawV1.CANONICAL_SOURCE_VERTEX_BASIS_V1
    if (
        near_planar_facts is None
        or frame_policy
        is not NearPlanarFramePolicyV1.REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1
    ):
        return frame, canonical
    if not grid_certificate.snapping_law.snaps_source:
        raise PlanarMetricAdmissionError(
            NamedOutcome.NEAR_PLANAR_REDUCED_FRAME_REQUIRES_SOURCE_SNAP,
            "the reduced integer plane basis is measured in source-grid steps, "
            f"and the grid law {grid_certificate.snapping_law.value} did not "
            "snap the source",
        )
    basis_a, basis_b = reduced_frame(
        normal=normal, source_scale=grid_certificate.source_scale
    )
    g00 = sum((a * a for a in basis_a), Fraction(0))
    g01 = sum((a * b for a, b in zip(basis_a, basis_b)), Fraction(0))
    g11 = sum((b * b for b in basis_b), Fraction(0))
    determinant = g00 * g11 - g01 * g01
    reduced = replace(
        frame,
        basis_a_id=frame.origin_id,
        basis_a=basis_a,
        basis_b_id=frame.origin_id,
        basis_b=basis_b,
        gram=((g00, g01), (g01, g11)),
        inverse=(
            (g11 / determinant, -g01 / determinant),
            (-g01 / determinant, g00 / determinant),
        ),
        coordinates=chart_of_positions(positions, frame.origin, basis_a, basis_b),
    )
    return reduced, AffineFrameSelectionLawV1.REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1


def _build_planar_family_metric(
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
    enforce_embedding: bool = True,
    surface_triangles: Iterable[SurfaceTriangleV1] | None = None,
    near_planar_lift_law: NearPlanarLiftLawV1 = (
        NearPlanarLiftLawV1.CERTIFIED_PLANE_V1
    ),
    near_planar_frame_policy: NearPlanarFramePolicyV1 = (
        NearPlanarFramePolicyV1.CANONICAL_ONLY_V1
    ),
) -> EmbeddingCertifiedRationalAffinePlanarMetricV1:
    faces, required_ids, positions = _source_scope(
        owner_patch_id=owner_patch_id,
        source_vertices=source_vertices,
        source_faces=source_faces,
    )
    # Привязка ИСТОЧНИКА идёт здесь и только здесь: базис, матрица Грама и все
    # углы обязаны считаться уже от привязанных координат, иначе восстановленное
    # отношение живёт в позициях, а метрика по-прежнему несёт шум.
    grid_facts = resolve_source_grid(
        positions=positions,
        faces=faces,
        snapping_law=grid_policy,
        enforce_embedding=enforce_embedding,
    )
    positions = grid_facts.positions
    snapped_positions = dict(positions)
    normal, near_planar_facts = _resolve_patch_plane(
        positions=positions,
        faces=faces,
        required_ids=required_ids,
        planarity_policy=planarity_policy,
        grid_certificate=grid_facts.certificate,
        judge_residual=(
            near_planar_lift_law is NearPlanarLiftLawV1.CERTIFIED_PLANE_V1
        ),
    )
    try:
        frame = _affine_frame(positions, required_ids)
    except ValueError as error:
        if near_planar_facts is not None:
            raise PlanarMetricAdmissionError(
                NamedOutcome.NEAR_PLANAR_PROJECTION_RESOLVED_PLANE_BASIS_UNAVAILABLE,
                "resolved near-planar projection does not define a canonical "
                f"source-vertex basis: {error}",
            ) from error
        raise
    frame, frame_law = _near_planar_frame(
        near_planar_facts=near_planar_facts,
        frame_policy=near_planar_frame_policy,
        frame=frame,
        positions=positions,
        normal=normal,
        grid_certificate=grid_facts.certificate,
    )
    orientation_sign, chart_orientation = _chart_orientation(frame.coordinates, faces)
    metric_id = ReferenceMetricId(
        _stable_id(
            "reference-metric",
            source_revision.value,
            patch_domain_id.value,
            frame.origin_id.value,
            *(item.value for item in required_ids),
        )
    )
    width_distortion = _width_distortion_record(
        near_planar_facts=near_planar_facts,
        surface_triangles=surface_triangles,
        source_revision=source_revision,
        patch_domain_id=patch_domain_id,
        snapped=snapped_positions,
        normal=normal,
        faces=faces,
        required_ids=required_ids,
        judged=near_planar_lift_law.onto_surface,
    )
    certificate = _planarity_certificate(
        source_revision=source_revision,
        patch_domain_id=patch_domain_id,
        normal=normal,
        required_ids=required_ids,
        near_planar_facts=near_planar_facts,
        width_distortion=width_distortion,
        lift_law=near_planar_lift_law.judged_as,
    )
    metric = _metric_record(
        metric_id=metric_id,
        patch_domain_id=patch_domain_id,
        source_revision=source_revision,
        origin=frame.origin,
        first=frame.basis_a,
        second=frame.basis_b,
        gram=frame.gram,
        inverse=frame.inverse,
        required_ids=required_ids,
        coordinates=frame.coordinates,
        chart_orientation=chart_orientation,
        certificate=certificate,
        source_lineage=source_lineage,
        grid_certificate=grid_facts.certificate,
        frame_law=frame_law,
    )
    projection_embedding = _projection_embedding(
        near_planar_facts=near_planar_facts,
        snapped=snapped_positions,
        faces=faces,
        frame=frame,
        normal=normal,
        sign=orientation_sign,
        enforce_embedding=enforce_embedding,
    )
    return EmbeddingCertifiedRationalAffinePlanarMetricV1(
        metric=metric,
        source_snap_embedding_certificate=(
            grid_facts.source_snap_embedding_certificate
        ),
        near_planar_projection_embedding_certificate=projection_embedding,
    )


#: Отказы near-planar, после которых лестница пробует развёртку: именованные отказы
#: по ширине, перевороту треугольника источника и вложению проекции. Остальные
#: (политика, невязка плоскости, вырожденность, окно решётки) развёртка не лечит, и
#: они остаются как есть.
LADDER_TRIGGER_OUTCOMES = frozenset(
    {
        NamedOutcome.NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED,
        NamedOutcome.NEAR_PLANAR_SOURCE_TRIANGLE_FOLDED,
        *(
            item
            for item in NamedOutcome
            if item.value.startswith("NEAR_PLANAR_PROJECTION_")
        ),
    }
)


def _developable_frame(chart, required_ids):
    """Репер карты развёртки: `(origin, A, B, Gram, inverse, coordinates)`.

    Начало нуль, `A = e_x/S'`, `B = e_y/S'`; координаты — целые узлы карты.
    """

    scale = chart.chart_scale
    unit = Fraction(1, scale)
    zero = Fraction(0)
    return (
        (zero, zero, zero),
        (unit, zero, zero),
        (zero, unit, zero),
        ((unit * unit, zero), (zero, unit * unit)),
        ((Fraction(scale * scale), zero), (zero, Fraction(scale * scale))),
        {
            vertex_id: (Fraction(chart.nodes[vertex_id][0]), Fraction(chart.nodes[vertex_id][1]))
            for vertex_id in required_ids
        },
    )


#: Имя «отказа», которым лестница записывает нулевую нормаль Ньюэлла: у
#: `PlaneNormalUndefinedError` исхода ядра нет (он `ValueError`, как был), но
#: в `previous_refusals` сертификата причина попадания на ступень должна читаться.
PLANE_NORMAL_UNDEFINED_TRACE = "PATCH_PLANE_NORMAL_UNDEFINED"


def _developable_rung(
    refused: Exception,
    trace: str,
    *,
    source_revision,
    patch_domain_id,
    owner_patch_id,
    source_vertices,
    source_faces,
    source_lineage,
    grid_policy,
    enforce_embedding,
    surface_triangles,
    declared_straight_chains=(),
) -> EmbeddingCertifiedRationalAffinePlanarMetricV1:
    """Последняя ступень лестницы: развёртка после именованного отказа near-planar.

    Привязка источника та же, что на ступенях ниже (`resolve_source_grid`), позиции
    берутся ДО проекции на плоскость: развёртка плоскости не знает. Отказ ступени (и
    привязки источника, и развёртки) несёт и свой исход, и отказ near-planar, после
    которого она пробовалась.
    """

    faces, required_ids, positions = _source_scope(
        owner_patch_id=owner_patch_id,
        source_vertices=source_vertices,
        source_faces=source_faces,
    )
    face_ids = {face.face_id for face in faces}
    try:
        grid_facts = resolve_source_grid(
            positions=positions,
            faces=faces,
            snapping_law=grid_policy,
            enforce_embedding=enforce_embedding,
        )
        certificate = grid_facts.certificate
        chart = build_developable_chart(
            source_revision=source_revision,
            patch_domain_id=patch_domain_id,
            snapped=dict(grid_facts.positions),
            owner_triangles=tuple(
                item for item in surface_triangles if item.source_face_id in face_ids
            ),
            required_ids=required_ids,
            source_scale=(
                certificate.source_scale if certificate.snapping_law.snaps_source else None
            ),
            previous_refusals=(trace,),
            declared_straight_chains=declared_straight_chains,
        )
    except PlanarMetricAdmissionError as final:
        raise PlanarMetricAdmissionError(
            final.outcome, f"{final} [after near-planar {trace}: {str(refused)[:240]}]"
        ) from refused
    origin, first, second, gram, inverse, coordinates = _developable_frame(
        chart, required_ids
    )
    metric = _metric_record(
        metric_id=ReferenceMetricId(
            _stable_id(
                "reference-metric",
                source_revision.value,
                patch_domain_id.value,
                required_ids[0].value,
                *(item.value for item in required_ids),
            )
        ),
        patch_domain_id=patch_domain_id,
        source_revision=source_revision,
        origin=origin,
        first=first,
        second=second,
        gram=gram,
        inverse=inverse,
        required_ids=required_ids,
        coordinates=coordinates,
        chart_orientation=AffineChartOrientationV1.COORDINATE_CCW_MATCHES_OWNER_PATCH,
        certificate=chart.certificate,
        source_lineage=source_lineage,
        grid_certificate=certificate,
        frame_law=AffineFrameSelectionLawV1.UNFOLDED_DEVELOPMENT_FRAME_V1,
    )
    return EmbeddingCertifiedRationalAffinePlanarMetricV1(
        metric=metric,
        source_snap_embedding_certificate=grid_facts.source_snap_embedding_certificate,
        near_planar_projection_embedding_certificate=None,
    )


def _build_embedding_certified_metric(
    *,
    curvature_ladder: CurvatureLadderPolicyV1 = (
        CurvatureLadderPolicyV1.NEAR_PLANAR_ONLY_V1
    ),
    declared_straight_chains: tuple = (),
    **arguments,
) -> EmbeddingCertifiedRationalAffinePlanarMetricV1:
    """Лестница метрики: EXACT -> NEAR_PLANAR -> DEVELOPABLE.

    Без лестницы (`NEAR_PLANAR_ONLY_V1`, по умолчанию) — ровно прежний путь
    построения. С лестницей развёртка пробуется ТОЛЬКО после именованного отказа
    near-planar из `LADDER_TRIGGER_OUTCOMES`, поэтому принятый сегодня домен не
    перемаршрутизируется (его байты прежние), а отказавший получает ещё одну
    попытку, у которой есть свой сертификат и свои имена отказа. Отказ вне
    триггеров, как и отсутствие треугольников поверхности, остаётся как есть.
    """

    if curvature_ladder is CurvatureLadderPolicyV1.NEAR_PLANAR_ONLY_V1:
        return _build_planar_family_metric(**arguments)
    arguments["source_vertices"] = tuple(arguments["source_vertices"])
    arguments["source_faces"] = tuple(arguments["source_faces"])
    if arguments.get("surface_triangles") is not None:
        arguments["surface_triangles"] = tuple(arguments["surface_triangles"])
    try:
        return _build_planar_family_metric(**arguments)
    except (PlanarMetricAdmissionError, PlaneNormalUndefinedError) as refused:
        trace = (
            refused.outcome.value
            if isinstance(refused, PlanarMetricAdmissionError)
            else PLANE_NORMAL_UNDEFINED_TRACE
        )
        if (
            isinstance(refused, PlanarMetricAdmissionError)
            and refused.outcome not in LADDER_TRIGGER_OUTCOMES
        ) or arguments.get("surface_triangles") is None:
            raise
        return _developable_rung(
            refused,
            trace,
            source_revision=arguments["source_revision"],
            patch_domain_id=arguments["patch_domain_id"],
            owner_patch_id=arguments["owner_patch_id"],
            source_vertices=arguments["source_vertices"],
            source_faces=arguments["source_faces"],
            source_lineage=arguments.get("source_lineage", frozenset()),
            grid_policy=arguments.get("grid_policy", GridSnappingLawV1.UNSNAPPED_EXACT_V1),
            enforce_embedding=arguments.get("enforce_embedding", True),
            surface_triangles=arguments["surface_triangles"],
            declared_straight_chains=tuple(declared_straight_chains),
        )


def build_embedding_certified_rational_affine_planar_metric(
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
    surface_triangles: Iterable[SurfaceTriangleV1] | None = None,
    near_planar_lift_law: NearPlanarLiftLawV1 = (
        NearPlanarLiftLawV1.CERTIFIED_PLANE_V1
    ),
    near_planar_frame_policy: NearPlanarFramePolicyV1 = (
        NearPlanarFramePolicyV1.CANONICAL_ONLY_V1
    ),
    curvature_ladder: CurvatureLadderPolicyV1 = (
        CurvatureLadderPolicyV1.NEAR_PLANAR_ONLY_V1
    ),
    declared_straight_chains: tuple = (),
) -> EmbeddingCertifiedRationalAffinePlanarMetricV1:
    """Build the unchanged V2 metric together with both embedding proofs."""

    return _build_embedding_certified_metric(
        source_revision=source_revision,
        patch_domain_id=patch_domain_id,
        owner_patch_id=owner_patch_id,
        source_vertices=source_vertices,
        source_faces=source_faces,
        source_lineage=source_lineage,
        planarity_policy=planarity_policy,
        grid_policy=grid_policy,
        surface_triangles=surface_triangles,
        near_planar_lift_law=near_planar_lift_law,
        near_planar_frame_policy=near_planar_frame_policy,
        curvature_ladder=curvature_ladder,
        declared_straight_chains=declared_straight_chains,
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
    surface_triangles: Iterable[SurfaceTriangleV1] | None = None,
    near_planar_lift_law: NearPlanarLiftLawV1 = (
        NearPlanarLiftLawV1.CERTIFIED_PLANE_V1
    ),
    near_planar_frame_policy: NearPlanarFramePolicyV1 = (
        NearPlanarFramePolicyV1.CANONICAL_ONLY_V1
    ),
    curvature_ladder: CurvatureLadderPolicyV1 = (
        CurvatureLadderPolicyV1.NEAR_PLANAR_ONLY_V1
    ),
    declared_straight_chains: tuple = (),
) -> RationalAffinePlanarMetricV2:
    """Build byte-compatible V2 after both additive embedding gates pass."""

    return _build_embedding_certified_metric(
        source_revision=source_revision,
        patch_domain_id=patch_domain_id,
        owner_patch_id=owner_patch_id,
        source_vertices=source_vertices,
        source_faces=source_faces,
        source_lineage=source_lineage,
        planarity_policy=planarity_policy,
        grid_policy=grid_policy,
        enforce_embedding=True,
        surface_triangles=surface_triangles,
        near_planar_lift_law=near_planar_lift_law,
        near_planar_frame_policy=near_planar_frame_policy,
        curvature_ladder=curvature_ladder,
        declared_straight_chains=declared_straight_chains,
    ).metric


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
    frame_law=AffineFrameSelectionLawV1.CANONICAL_SOURCE_VERTEX_BASIS_V1,
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
        frame_selection_law=frame_law,
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
