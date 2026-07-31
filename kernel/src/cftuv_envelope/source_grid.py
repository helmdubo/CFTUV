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
поэтому проверка исполняемая, а не декларативная. Константа 7·10⁻⁶ описывает
намерение автора вообще, а проверка отвечает за ЭТОТ меш.

Она же выбирает масштаб. Ничья при привязке (`x·2^m` ровно в середине ячейки)
случается у каждой вершины ровно на одном масштабе и предсказуема заранее,
поэтому шаг у нижней границы окна — не выбор, а лотерея: на `building.002`
ничья попала ровно на него. Закон выбора — `select_grid_scale`: перебрать
степени двойки внутри окна объявленным порядком и взять ПЕРВУЮ, на которой
проверка проходит; ни одной такой нет — `NO_GRID_SCALE_RESTORES_RELATIONS`, а
не «взять ближайший». Весь перебор идёт в сертификат: иначе по картинке в
Blender не видно, почему геометрия получилась именно эта.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from math import isqrt

import sympy as sp

from .contracts.metric import (
    ExactRationalV1,
    GridScaleSearchOrderV1,
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

# Объявленный порядок перебора масштабов внутри окна. Не параметр входа и не
# настройка: одно значение на всё ядро, записываемое в каждый сертификат.
# Обоснование замером — в `select_grid_scale` и в `DECISIONS.md`.
GRID_SCALE_SEARCH_ORDER = GridScaleSearchOrderV1.FINEST_ADMISSIBLE_FIRST_V1

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


def plane_dominant_axis(normal) -> int:
    """Ось, которую решают из уравнения плоскости: наибольшая |N|, ничья — младшая.

    Наибольшая по модулю координата нормали выбрана не ради красоты: именно на
    неё делят, и она же входит в знаменатель результата, поэтому наибольшая
    даёт наименьший рост чисел и наименьшее усиление сдвига двух остальных
    координат. Ничья разрешается младшим индексом — детерминированно.
    """

    return max(range(3), key=lambda axis: (abs(normal[axis]), -axis))


def snap_positions_in_patch_plane(positions, normal, anchor, grid: GridSpecV1) -> dict:
    """Мировая решётка, ПЕРЕСЕЧЁННАЯ с собственной плоскостью патча.

    Решётка та же самая, что и у осевой привязки, — тот же шаг, те же узлы, —
    но точке разрешено садиться только в ту её часть, которая лежит в плоскости
    патча. Две координаты привязываются мировой решёткой как прежде, третья
    (доминантная ось нормали) РЕШАЕТСЯ из уравнения плоскости. Поэтому:

    - планарность сохраняется тождественно, а не с точностью до бюджета;
    - у осевой плоскости (`N` = единичный орт) решать нечего: смещение
      плоскости само лежит в узле, и результат СОВПАДАЕТ с прежней осевой
      привязкой побитово — замерено на `building.002` патч 0 на всех масштабах
      окна;
    - задуманно прямые углы восстанавливаются ровно как прежде: у осевого
      патча привязка не изменилась вовсе (3 из 3 на масштабе 2048).

    Смещение плоскости привязывается тем же `snap_value` и к той же решётке:
    иначе плоскость прошла бы мимо узлов и «пересечение с решёткой» оказалось
    бы пустым, а координата доминантной оси уехала бы на полклетки у ВСЕХ
    вершин сразу.
    """

    axis = plane_dominant_axis(normal)
    others = tuple(index for index in range(3) if index != axis)
    offset = Fraction(
        snap_value(_dot(normal, anchor), grid), grid.scale
    )
    result = {}
    for vertex_id, point in positions.items():
        coordinates = [None, None, None]
        for index in others:
            coordinates[index] = Fraction(snap_value(point[index], grid), grid.scale)
        coordinates[axis] = (
            offset - sum(normal[index] * coordinates[index] for index in others)
        ) / normal[axis]
        result[vertex_id] = tuple(coordinates)
    return result


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


def snapper_for(snapping_law: GridSnappingLawV1, plane):
    """Чем именно закон двигает источник: `(позиции, масштаб) -> позиции`.

    Один выбор на весь модуль. Перебор масштабов и итоговая привязка обязаны
    идти ОДНОЙ функцией: иначе выбранный масштаб доказан на одной привязке, а
    применён к другой, и сертификат перестаёт описывать то, что произошло.
    """

    if snapping_law.snaps_in_patch_plane:
        if plane is None:
            raise ValueError(
                f"{snapping_law.value} требует плоскость патча"
            )
        normal, anchor = plane

        def snap(positions, scale):
            return snap_positions_in_patch_plane(
                positions, normal, anchor, GridSpecV1(scale=scale)
            )

        return snap

    def snap(positions, scale):
        return snap_positions(positions, GridSpecV1(scale=scale))

    return snap


def select_grid_scale(*, positions, intended, window, search_order, snap):
    """Первый масштаб окна, на котором проверка 2 проходит, и весь перебор.

    ЭТО НЕ ПОДГОНКА, и различие принципиально. Подгонка — это свободный
    параметр, который крутят, пока результат не понравится. Здесь свободного
    параметра нет: множество кандидатов задано окном (`admissible_scales`),
    порядок обхода объявлен (`search_order`), предикат приёмки объявлен
    (все задуманно прямые углы дают точно рациональную долю π), и берётся
    ПЕРВЫЙ прошедший. Другой исполнитель с тем же входом обязан получить тот
    же масштаб, а не «похожий»; весь перебор при этом записан в сертификат,
    поэтому проверить это можно по записи, а не повторным запуском.

    Перебор нужен потому, что ничья при привязке — правило, а не случайность.
    Координата binary64 есть `p/2^k`, масштаб есть `2^m`, поэтому произведение
    равно ровно `.5` тогда и только тогда, когда `k − m = 1` при нечётном `p`.
    Каждая вершина даёт ничью ровно на ОДНОМ масштабе, и правило «половина
    вверх» уводит её в соседний узел. Выбрать масштаб «на глаз» нельзя:
    владелец не видит, где ничьи, а на `building.002` ничья попала ровно на
    тот масштаб, который прежний закон брал без перебора.

    Порядок объявлен `FINEST_ADMISSIBLE_FIRST_V1` — от самого мелкого
    допустимого шага к крупному. Обоснование замером (`DECISIONS.md` за
    2026-07-25, воспроизведение — `tools/benchmark_grid_scale.py --fixture`):
    нижняя граница окна и есть условие «авторская ошибка меньше половины
    ячейки», ради которого привязка вообще работает, поэтому любой кандидат
    окна уже достаточно крупен, и брать крупнее не покупается ничем
    объявленным, а стоит сдвигом вершин. На `building.002` крупный конец даёт
    шаг 7.81e-03 при сдвиге 3.76e-03 м, мелкий — 4.88e-04 при сдвиге 2.29e-04
    м: в 16 раз больше при том же результате проверки 2. Во столько же раз
    поднимается и минимальная допустимая alpha (3.13e-02 против 1.95e-03 м),
    то есть крупный конец сужает множество принимаемых запросов декали.
    """

    from .contracts.metric import (
        GridScaleTrialOutcomeV1,
        GridScaleTrialV1,
    )
    from .contracts.metric import GridScaleSearchOrderV1 as _Order
    from .robust.snapping import admissible_scales

    candidates = admissible_scales(window)
    if search_order is _Order.FINEST_ADMISSIBLE_FIRST_V1:
        candidates = tuple(reversed(candidates))
    trials = []
    for scale in candidates:
        grid = GridSpecV1(scale=scale)
        restored = restored_right_corners(snap(positions, scale), intended)
        passed = restored == len(intended)
        trials.append(
            GridScaleTrialV1(
                scale=scale,
                step=_rational(Fraction(1, scale)),
                restored_right_corners=restored,
                outcome=(
                    GridScaleTrialOutcomeV1.RELATIONS_RESTORED
                    if passed
                    else GridScaleTrialOutcomeV1.RELATIONS_NOT_RESTORED
                ),
            )
        )
        if passed:
            return grid, tuple(trials)
    raise _no_scale_refusal(len(intended), tuple(trials))


def resolve_source_grid(
    *,
    positions,
    faces,
    snapping_law: GridSnappingLawV1,
    search_order: GridScaleSearchOrderV1 = GRID_SCALE_SEARCH_ORDER,
    plane=None,
):
    """Окно шага, выбор масштаба законом, привязка и сертификат — или отказ.

    Возвращаемые позиции при `UNSNAPPED_EXACT_V1` — те же самые объекты: закон
    описывает, что с координатами сделали, и «ничего» тоже записывается.

    Ветка выбирается по `snaps_source`, а не по имени закона: источник
    `SOURCE_ONLY_GRID_SNAP_V1` и `INTEGER_GRID_SNAP_V1` двигают ОДИНАКОВО, и
    расходятся они ниже — на привязке конструкций, до которой этот модуль не
    доходит. Сравнение с именем здесь означало бы, что добавление третьего
    закона молча меняет поведение источника, а оно не менялось.
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
    if not snapping_law.snaps_source:
        return SourceGridFactsV1(
            positions=positions,
            certificate=IntegerGridCertificateV1(
                snapping_law=snapping_law,
                restored_right_corners=restored_right_corners(positions, intended),
                search_order=search_order,
                scale_trials=(),
                **facts,
            ),
        )
    if window.grid is None:
        raise _refusal(outcome.value, extent, window)
    snap = snapper_for(snapping_law, plane)
    grid, trials = select_grid_scale(
        positions=positions,
        intended=intended,
        window=window,
        search_order=search_order,
        snap=snap,
    )
    snapped = snap(positions, grid.scale)
    facts["window_step"] = _rational(Fraction(1, grid.scale))
    facts["source_scale"] = grid.scale
    facts["magnitude_bound"] = grid.magnitude_bound
    return SourceGridFactsV1(
        positions=snapped,
        certificate=IntegerGridCertificateV1(
            snapping_law=snapping_law,
            restored_right_corners=trials[-1].restored_right_corners,
            search_order=search_order,
            scale_trials=trials,
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


def _no_scale_refusal(intended: int, trials):
    from .outcomes import NamedOutcome
    from .planar_metric import PlanarMetricAdmissionError

    tried = ", ".join(
        f"{trial.scale}→{trial.restored_right_corners}" for trial in trials
    )
    return PlanarMetricAdmissionError(
        NamedOutcome.NO_GRID_SCALE_RESTORES_RELATIONS,
        "ни один масштаб окна не восстановил задуманные отношения: "
        f"задумано прямых={intended}, "
        f"пробы (масштаб→восстановлено): {tried}",
    )
