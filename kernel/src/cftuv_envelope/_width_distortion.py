"""Сертификат искажения ширины near-planar: точный cos² по треугольникам источника.

Модуль внутренний, как `_embedding`: он не владеет допуском (его владеет
`contracts.metric.NEAR_PLANAR_WIDTH_BUDGET`), ничего не чинит и ничего не
округляет. Он пересчитывает точные предикаты над `Fraction` и возвращает запись.
Судит ли запись, решает вызывающий: построитель при укладке на треугольники
источника отказывает, при укладке на сертифицированную плоскость только пишет.

Математика. Ортогональная проекция плоского треугольника T на плоскость с
нормалью `n` — линейное отображение с сингулярными числами `1` и `|cos θ_T|`,
`cos θ_T = n_T·n / (|n_T||n|)`. Квадрат косинуса рационален:

    cos² θ_T = (n_T·n)² / ((n_T·n_T)(n·n)),

поэтому ни корня, ни бюджета работы здесь нет. Длина вдоль поверхности
относится к длине на карте как число из `[1, 1/cos θ_T]`; условие приёма
`cos² θ_T ≥ 1/(1+b)²` оставляет ширину декали на поверхности не больше, чем в
`1+b` раз шире заданной на карте. Меряются ПРИВЯЗАННЫЕ, не спроецированные
позиции: проекция — это ровно то отображение, чьё искажение измеряется.
"""

from __future__ import annotations

from fractions import Fraction
from hashlib import sha256

from .contracts.metric import (
    NEAR_PLANAR_WIDTH_BUDGET,
    ExactPoint3V1,
    ExactRationalV1,
    NearPlanarWidthDistortionCertificateV1,
    NearPlanarWidthDistortionLawV1,
    SnappedSourcePositionV1,
)
from .ids import PlanarityCertificateId
from .outcomes import NamedOutcome


def _rational(value: Fraction | int) -> ExactRationalV1:
    item = Fraction(value)
    return ExactRationalV1(item.numerator, item.denominator)


def _sub(left, right):
    return tuple(a - b for a, b in zip(left, right, strict=True))


def _dot(left, right) -> Fraction:
    return sum((a * b for a, b in zip(left, right, strict=True)), Fraction(0))


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def width_distortion_threshold(budget: Fraction) -> Fraction:
    """Наименьший допустимый `cos²` при относительном бюджете `b`: `1/(1+b)²`."""

    return Fraction(1) / (1 + Fraction(budget)) ** 2


def triangle_cos_squared(corners, normal) -> Fraction | None:
    """`cos²` угла между нормалью треугольника и `normal`; `None` у вырожденного.

    Знак нормали треугольника и самой `normal` не входит: наклон меряется
    квадратом, а перевёрнутый треугольник ловит сертификат вложения проекции
    (ориентация и перекрытие интерьеров), а не этот.
    """

    first, second, third = corners
    triangle_normal = _cross(_sub(second, first), _sub(third, first))
    squared = _dot(triangle_normal, triangle_normal)
    if not squared:
        return None
    along = _dot(triangle_normal, normal)
    return along * along / (squared * _dot(normal, normal))


def _unavailable(detail: str):
    # Ленивый импорт: `planar_metric` импортирует этот модуль, цикл иначе.
    from .planar_metric import PlanarMetricAdmissionError

    return PlanarMetricAdmissionError(
        NamedOutcome.NEAR_PLANAR_OWNER_SURFACE_TRIANGLES_UNAVAILABLE, detail
    )


def _owner_triangles(faces, triangles, snapped):
    """Треугольники граней владельца по имени; вход, которого не измерить, — отказ."""

    face_ids = {face.face_id for face in faces}
    owned = sorted(
        (item for item in triangles if item.source_face_id in face_ids),
        key=lambda item: item.triangle_id.value,
    )
    if not owned:
        raise _unavailable(
            "the owner Patch has no surface triangle to measure the width "
            "distortion on"
        )
    for item in owned:
        missing = [vertex.value for vertex in item.vertex_ids if vertex not in snapped]
        if missing:
            raise _unavailable(
                f"surface triangle {item.triangle_id.value} names vertices "
                f"outside the owner Patch: {missing}"
            )
    return owned


def build_width_distortion_certificate(
    *,
    source_revision,
    patch_domain_id,
    snapped,
    normal,
    faces,
    triangles,
    required_ids,
) -> NearPlanarWidthDistortionCertificateV1:
    """Сертификат искажения по треугольникам граней владельца. Не судит."""

    worst = None
    degenerate = []
    owned = _owner_triangles(faces, triangles, snapped)
    for item in owned:
        cos_squared = triangle_cos_squared(
            tuple(snapped[vertex] for vertex in item.vertex_ids), normal
        )
        if cos_squared is None:
            degenerate.append(item.triangle_id)
        elif worst is None or cos_squared < worst[0]:
            worst = (cos_squared, item)
    return NearPlanarWidthDistortionCertificateV1(
        certificate_id=PlanarityCertificateId(
            _stable_id(
                "near-planar-width-distortion",
                source_revision.value,
                patch_domain_id.value,
            )
        ),
        patch_domain_id=patch_domain_id,
        source_revision=source_revision,
        law=NearPlanarWidthDistortionLawV1.INTRINSIC_WIDTH_RELATIVE_V1,
        width_budget=_rational(NEAR_PLANAR_WIDTH_BUDGET),
        min_cos_squared=_rational(Fraction(1) if worst is None else worst[0]),
        worst_triangle_id=None if worst is None else worst[1].triangle_id,
        worst_face_id=None if worst is None else worst[1].source_face_id,
        triangles_measured=len(owned) - len(degenerate),
        degenerate_triangle_count=len(degenerate),
        first_degenerate_triangle_id=degenerate[0] if degenerate else None,
        snapped_source_positions=frozenset(
            SnappedSourcePositionV1(
                vertex_id,
                ExactPoint3V1(*(_rational(item) for item in snapped[vertex_id])),
            )
            for vertex_id in required_ids
        ),
    )


def _stable_id(kind: str, *parts: object) -> str:
    payload = "\x1f".join((kind, *(str(item) for item in parts)))
    return f"{kind}:{sha256(payload.encode('utf-8')).hexdigest()[:24]}"


def width_distortion_violations(certificate) -> tuple[NamedOutcome, ...]:
    """Именованные отказы сертификата. Порядок: вырожденность, затем бюджет.

    Вырожденный треугольник отказывает ЗАКРЫТО: его наклон не определён, и
    «пропустить и мерить остальные» было бы тихим исчезновением треугольника.
    """

    result = []
    if certificate.degenerate_triangle_count:
        result.append(NamedOutcome.NEAR_PLANAR_OWNER_TRIANGLE_DEGENERATE)
    if certificate.triangles_measured:
        threshold = width_distortion_threshold(
            Fraction(
                certificate.width_budget.numerator,
                certificate.width_budget.denominator,
            )
        )
        measured = Fraction(
            certificate.min_cos_squared.numerator,
            certificate.min_cos_squared.denominator,
        )
        if measured < threshold:
            result.append(NamedOutcome.NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED)
    return tuple(result)


def width_distortion_violation(certificate) -> NamedOutcome | None:
    failures = width_distortion_violations(certificate)
    return failures[0] if failures else None


def width_distortion_refusal_text(certificate) -> str:
    """Отказ несёт числа, из которых сложилось решение (binary64 — для чтения)."""

    budget = Fraction(
        certificate.width_budget.numerator, certificate.width_budget.denominator
    )
    measured = Fraction(
        certificate.min_cos_squared.numerator,
        certificate.min_cos_squared.denominator,
    )
    worst = (
        "none"
        if certificate.worst_triangle_id is None
        else f"{certificate.worst_triangle_id.value} "
        f"(face {certificate.worst_face_id.value})"
    )
    degenerate = (
        "none"
        if certificate.first_degenerate_triangle_id is None
        else certificate.first_degenerate_triangle_id.value
    )
    return (
        "near-planar width distortion: "
        f"min_cos_squared={float(measured):.9e} "
        f"< threshold={float(width_distortion_threshold(budget)):.9e} "
        f"(width_budget={float(budget):.6e}, "
        f"law={certificate.law.value}, "
        f"triangles_measured={certificate.triangles_measured}, "
        f"worst_triangle={worst}); "
        f"degenerate_triangles={certificate.degenerate_triangle_count} "
        f"(first={degenerate})"
    )
