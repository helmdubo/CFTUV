"""Решётка домена: окно шага, привязка ИСТОЧНИКА и доказательство привязки.

Привязывается вход, а не результат. Разница не стилистическая, она замерена:
слияние двух ВЫЧИСЛЕННЫХ точек, разнесённых на `d`, при шаге `step` происходит
с вероятностью `1 − d/step`, то есть надёжное слияние требует шага в сотню раз
крупнее шума — а деталь декали требует шага мельче. Окна нет. Привязка же
вершин источника детерминирована: если задуманное положение вершины лежит в
узле, а авторская ошибка меньше половины ячейки, вход становится ТОЧНО
удовлетворяющим задуманным отношениям, и всякое совпадение, следующее из них
алгебраически, дальше точно.

Отсюда обязательная проверка: угол, который объявленная авторская ошибка числит
задуманно прямым, после привязки обязан дать точно рациональную долю π.
`rational_intervals_over_pi` возвращает `None`, когда доля не рациональна,
поэтому проверка исполняемая, а не декларативная. Не сработало —
`SOURCE_SNAP_DID_NOT_RESTORE_RELATIONS`, а не тихое продолжение: константа
7·10⁻⁶ описывает намерение автора вообще, а проверка отвечает за ЭТОТ меш.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from math import isqrt

import sympy as sp

from .contracts.metric import (
    ExactRationalV1,
    GridSnappingLawV1,
    GridWindowOutcomeV1,
    IntegerGridCertificateV1,
)
from .reference.angle_measure import (
    CertifiedAngleUnavailable,
    angular_fraction_of_pi,
    rational_intervals_over_pi,
)
from .robust.grid import GridSpecV1, snap_value
from .robust.snapping import GridWindowOutcome, grid_window_for_patch


# Решение владельца, `DECISIONS.md` за 2026-07-25. Деталей мельче сантиметра в
# игровом пространстве нет; максимальный габарит — 100 м. Шаг берётся у НИЖНЕЙ
# границы, поэтому деталь на него не влияет вовсе — она двигает только точку,
# в которой окно закрывается.
DECAL_DETAIL = Fraction(1, 100)
AUTHOR_ANGULAR_ERROR = Fraction(7, 10**6)

_WINDOW_OUTCOMES = {
    GridWindowOutcome.WINDOW_AVAILABLE: GridWindowOutcomeV1.WINDOW_AVAILABLE,
    GridWindowOutcome.GRID_WINDOW_CLOSED: GridWindowOutcomeV1.GRID_WINDOW_CLOSED,
    GridWindowOutcome.NO_POWER_OF_TWO_STEP_IN_WINDOW: (
        GridWindowOutcomeV1.NO_POWER_OF_TWO_STEP_IN_WINDOW
    ),
}


@dataclass(frozen=True, slots=True)
class SourceGridFactsV1:
    """Что решётка сделала с позициями и чем она это подтверждает."""

    positions: dict
    certificate: IntegerGridCertificateV1


def _rational(value: Fraction | int) -> ExactRationalV1:
    item = Fraction(value)
    return ExactRationalV1(item.numerator, item.denominator)


def _sympy_rational(value: Fraction) -> sp.Rational:
    item = Fraction(value)
    return sp.Rational(item.numerator, item.denominator)


def source_extent(positions) -> Fraction:
    """Габарит патча — наибольший разброс по одной оси локальных координат.

    Именно он входит в нижнюю границу окна: авторская угловая ошибка даёт
    расхождение порядка `угол × длина`, поэтому на большом патче шум растёт,
    а деталь декали — нет.
    """

    return max(
        max(point[axis] for point in positions.values())
        - min(point[axis] for point in positions.values())
        for axis in range(3)
    )


def snap_positions(positions, grid: GridSpecV1) -> dict:
    """Привязать позиции источника к решётке, по оси за раз.

    Округление — `floor(x + 1/2)`, то же, что у `snap_value`: банковское
    округление зависит от чётности и читается в дайджесте как случайность.
    """

    return {
        vertex_id: tuple(
            Fraction(snap_value(value, grid), grid.scale) for value in point
        )
        for vertex_id, point in positions.items()
    }


def _sub(left, right):
    return tuple(a - b for a, b in zip(left, right, strict=True))


def _dot(left, right):
    return sum((a * b for a, b in zip(left, right, strict=True)), Fraction(0))


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _corners(faces):
    """Тройки `(предыдущая, вершина, следующая)` по всем циклам граней патча."""

    for face in faces:
        cycle = face.vertex_cycle
        count = len(cycle)
        for index in range(count):
            yield (
                cycle[(index - 1) % count],
                cycle[index],
                cycle[(index + 1) % count],
            )


def _corner_pair(positions, corner):
    previous, vertex, following = corner
    if any(item not in positions for item in corner):
        return None
    left = _sub(positions[previous], positions[vertex])
    right = _sub(positions[following], positions[vertex])
    normal = _cross(left, right)
    squared_sine = _dot(normal, normal)
    if squared_sine == 0:
        # Вырожденный угол: стороны коллинеарны, прямым он не бывает.
        return None
    return _dot(left, right), squared_sine


def intended_right_corners(positions, faces) -> tuple:
    """Углы, которые объявленная авторская ошибка числит задуманно прямыми.

    Это классификация угла, а не порог в геометрии: она ничего не округляет и
    ничего не решает про ответ — она задаёт множество, на котором привязка
    обязана отчитаться. Число попавших в него углов идёт в сертификат, поэтому
    классификация записана, а не спрятана.

    Уже точно прямые углы в множество не входят: восстанавливать в них нечего.
    Критерий на дробях: `|cos| ≤ угол × |sin|` — первый член разложения
    `cos(π/2 − ε) = sin ε ≤ ε` — и сравнивается в квадратах, чтобы не вводить
    корень.
    """

    result = []
    bound = AUTHOR_ANGULAR_ERROR * AUTHOR_ANGULAR_ERROR
    for corner in _corners(faces):
        pair = _corner_pair(positions, corner)
        if pair is None:
            continue
        cosine, squared_sine = pair
        if cosine == 0:
            continue
        if cosine * cosine <= bound * squared_sine:
            result.append(corner)
    return tuple(result)


def restored_right_corners(positions, corners) -> int:
    """Сколько из названных углов дают точно рациональную долю π.

    Проверка исполняемая: `rational_intervals_over_pi` возвращает `None`, если
    доля не рациональна, и поднимает именованный отказ, если угол вырожден до
    2π. Оба случая — «не восстановлено».
    """

    total = 0
    for corner in corners:
        pair = _corner_pair(positions, corner)
        if pair is None:
            continue
        cosine, squared_sine = pair
        sine = sp.sqrt(_sympy_rational(squared_sine))
        try:
            measured = rational_intervals_over_pi(
                angular_fraction_of_pi(sine, _sympy_rational(cosine))
            )
        except CertifiedAngleUnavailable:
            continue
        if measured is not None:
            total += 1
    return total


def chart_grid_for(gram, step: Fraction) -> GridSpecV1:
    """Решётка карты, у которой ячейка не крупнее ячейки источника в метрике.

    Координаты карты — кратные базисным векторам, а не метры, поэтому шаг
    источника переносить в карту напрямую нельзя. Для смещения `(du, dv)` с
    `|du|, |dv| ≤ h` метрическая длина не превосходит `h·√(g00 + 2|g01| + g11)`,
    поэтому достаточно взять `h ≤ шаг / √(g00 + 2|g01| + g11)`.

    Корень берётся сверху точно: для `p/q` годится `(isqrt(p·q) + 1)/q`. Ни
    оценки, ни порога здесь нет — только доказанная верхняя граница.
    """

    (g00, g01), (_, g11) = gram
    squared = Fraction(g00) + 2 * abs(Fraction(g01)) + Fraction(g11)
    if squared <= 0:
        raise ValueError("матрица Грама не положительно определена")
    upper = Fraction(isqrt(squared.numerator * squared.denominator) + 1, squared.denominator)
    limit = Fraction(step) / upper
    scale = 1
    while Fraction(1, scale) > limit:
        scale *= 2
    return GridSpecV1(scale=scale)


def resolve_source_grid(*, positions, faces, snapping_law: GridSnappingLawV1):
    """Окно шага, привязка источника и сертификат — или именованный отказ.

    Возвращаемые позиции при `UNSNAPPED_EXACT_V1` — те же самые объекты: закон
    описывает, что с координатами сделали, и «ничего» тоже записывается.
    """

    extent = source_extent(positions)
    if extent <= 0:
        # Нулевой габарит — все вершины совпали. Окно шага для такого патча не
        # определено, и выдумывать его нельзя: ниже он всё равно не даст
        # ненулевого базиса.
        raise ValueError("source vertices do not define a non-zero basis A")
    window = grid_window_for_patch(
        extent=extent,
        author_angular_error=AUTHOR_ANGULAR_ERROR,
        decal_detail=DECAL_DETAIL,
    )
    outcome = _WINDOW_OUTCOMES[window.outcome]
    intended = intended_right_corners(positions, faces)
    step = None if window.grid is None else Fraction(1, window.grid.scale)
    facts = {
        "window_outcome": outcome,
        "patch_extent": _rational(extent),
        "author_angular_error": _rational(AUTHOR_ANGULAR_ERROR),
        "decal_detail": _rational(DECAL_DETAIL),
        "window_lower_bound": _rational(window.lower_bound),
        "window_upper_bound": _rational(window.upper_bound),
        "window_step": None if step is None else _rational(step),
        "source_scale": None if window.grid is None else window.grid.scale,
        "magnitude_bound": (
            None if window.grid is None else window.grid.magnitude_bound
        ),
        "intended_right_corners": len(intended),
    }
    if snapping_law is GridSnappingLawV1.UNSNAPPED_EXACT_V1:
        return SourceGridFactsV1(
            positions=positions,
            certificate=IntegerGridCertificateV1(
                snapping_law=snapping_law,
                restored_right_corners=restored_right_corners(positions, intended),
                **facts,
            ),
        )
    if window.grid is None:
        raise _refusal(outcome.value, extent, window)
    snapped = snap_positions(positions, window.grid)
    restored = restored_right_corners(snapped, intended)
    if restored != len(intended):
        raise _restoration_refusal(len(intended), restored, step)
    return SourceGridFactsV1(
        positions=snapped,
        certificate=IntegerGridCertificateV1(
            snapping_law=snapping_law,
            restored_right_corners=restored,
            **facts,
        ),
    )


def _refusal(outcome_value: str, extent: Fraction, window):
    from .outcomes import NamedOutcome
    from .planar_metric import PlanarMetricAdmissionError

    return PlanarMetricAdmissionError(
        NamedOutcome(outcome_value),
        "шага решётки для этого патча не существует: "
        f"габарит={float(extent):.6g}, "
        f"нижняя={float(window.lower_bound):.6g}, "
        f"верхняя={float(window.upper_bound):.6g}",
    )


def _restoration_refusal(intended: int, restored: int, step):
    from .outcomes import NamedOutcome
    from .planar_metric import PlanarMetricAdmissionError

    return PlanarMetricAdmissionError(
        NamedOutcome.SOURCE_SNAP_DID_NOT_RESTORE_RELATIONS,
        "привязка источника не восстановила задуманные отношения: "
        f"задумано прямых={intended}, восстановлено={restored}, "
        f"шаг={float(step):.6g}",
    )
