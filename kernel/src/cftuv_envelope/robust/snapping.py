"""Привязка точек конструкции к решётке: чистые функции, ещё никем не вызванные.

Срез R1a намеренно не трогает ни `contracts/`, ни `reference/`. Причина не в
осторожности, а в устройстве дайджеста: `to_canonical_data` обходит **все** поля
дата-класса, поэтому новое поле в контракте двигает каноническую строку даже со
значением по умолчанию. «Безопасной половины» внутри контракта не существует,
и разрез проходит по его границе.

Что здесь есть: закон привязки, построение решётки из габарита и две функции,
которые R1b подставит в `offset_support_g` и `segment_intersections`. Пока их
никто не вызывает, и это правильно — сначала они доказываются, потом
подключаются.

Почему привязка вообще нужна — две причины, вторая сильнее.

Скорость: `_canonical_expr` стоит 0.23 мкс на рациональном и 2758 мкс на
алгебраическом. На центральном патче `building.002` 21 590 алгебраических
канонизаций дают ~60 с при полных 72 с прогона.

Корректность (Huber §2.5.4): точная арифметика на неточном входе добросовестно
считает скелет шума. Контур, задуманный так, чтобы фронты сошлись в одной точке,
из-за binary64 даёт несколько близких вершин вместо одной. Привязка
восстанавливает задуманное вырождение структурно, а не допуском.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction
from typing import Iterable

from .grid import GridSpecV1, SnappedPointV1, snap_point


class GridSnappingLawV1(str, Enum):
    """Чем именно оправдана привязка данной точки.

    Закон живёт здесь, а не в `contracts/`: в контракт он попадёт срезом R1b,
    вместе с перегенерацией дайджестов, и до тех пор его присутствие тут ничего
    не двигает. Форма взята с `PlanarityAdmissionLawV1` — там политика тоже
    сначала существовала, а дайджесты сдвинулись только когда хост её включил.
    """

    # Точка построена как `point + distance * unit_normal`. Радикал приходит из
    # `unit_normal`, и это единственное место, откуда он течёт в геометрию.
    OFFSET_CONSTRUCTION_V1 = "OFFSET_CONSTRUCTION_V1"
    # Точка пересечения двух отрезков. Привязка держит вход целым на следующих
    # итерациях и сливает вырождения там, где сходятся локусы.
    SEGMENT_INTERSECTION_V1 = "SEGMENT_INTERSECTION_V1"


def grid_for_extent(
    coordinates: Iterable[tuple[Fraction | int, Fraction | int]],
    *,
    nodes_across: int,
) -> GridSpecV1:
    """Решётка из габарита координат патча. Чистая: метрику не спрашивает.

    `nodes_across` — сколько узлов должно лечь поперёк большей стороны габарита.
    Это и есть параметр, значение которого мы не знаем заранее, и подбирается он
    стендом `tools/benchmark_grid_scale.py`, а выбирается владельцем по картинке.

    Масштаб округляется вверх до степени двойки. Причина не эстетическая:
    `Fraction * 2**k` не вносит нового знаменателя, поэтому привязка не удлиняет
    числа, которые сама же и призвана укоротить.
    """

    if nodes_across <= 0:
        raise ValueError("nodes_across должен быть положительным")
    points = [(Fraction(x), Fraction(y)) for x, y in coordinates]
    if not points:
        raise ValueError("габарит пустого множества точек не определён")
    width = max(x for x, _ in points) - min(x for x, _ in points)
    height = max(y for _, y in points) - min(y for _, y in points)
    extent = max(width, height)
    if extent <= 0:
        # Вырожденный габарит — это не повод угадывать масштаб. Единичная
        # решётка честнее любого выдуманного числа, и вызывающий увидит её в
        # сертификате.
        return GridSpecV1(scale=1)
    scale = 1
    while Fraction(scale) * extent < nodes_across:
        scale *= 2
    return GridSpecV1(scale=scale)


def snap_offset(
    x: Fraction | int,
    y: Fraction | int,
    grid: GridSpecV1,
    *,
    squared_budget: Fraction | None = None,
) -> SnappedPointV1:
    """Результат смещения на alpha, привязанный к решётке.

    Здесь алгебра и заканчивается: на входе значение с радикалом, на выходе
    целый узел. Всё, что строится ниже по течению, снова рационально.
    """

    return snap_point(x, y, grid, squared_budget=squared_budget)


def snap_intersection(
    x: Fraction | int,
    y: Fraction | int,
    grid: GridSpecV1,
    *,
    squared_budget: Fraction | None = None,
) -> SnappedPointV1:
    """Точка пересечения отрезков, привязанная к решётке.

    Отдельная функция от `snap_offset` не ради разной арифметики — она
    одинаковая, — а ради разного закона в сертификате: по записи должно быть
    видно, что именно привязали, иначе сдвиг нельзя отнести к причине.
    """

    return snap_point(x, y, grid, squared_budget=squared_budget)


def law_for(function_name: str) -> GridSnappingLawV1:
    """Закон по имени места привязки. Неизвестное место — ошибка, не догадка."""

    laws = {
        "offset_support_g": GridSnappingLawV1.OFFSET_CONSTRUCTION_V1,
        "segment_intersections": GridSnappingLawV1.SEGMENT_INTERSECTION_V1,
    }
    if function_name not in laws:
        raise KeyError(f"нет объявленного закона привязки для {function_name}")
    return laws[function_name]


class GridWindowOutcome(str, Enum):
    """Есть ли шаг решётки, который чинит вход и не портит деталь декали."""

    WINDOW_AVAILABLE = "WINDOW_AVAILABLE"
    GRID_WINDOW_CLOSED = "GRID_WINDOW_CLOSED"


@dataclass(frozen=True, slots=True)
class GridWindowV1:
    """Границы допустимого шага и исход. Закрытое окно — отказ, а не крупный шаг."""

    outcome: GridWindowOutcome
    lower_bound: Fraction
    upper_bound: Fraction
    grid: GridSpecV1 | None


def grid_window_for_patch(
    *,
    extent: Fraction | int,
    author_angular_error: Fraction,
    decal_detail: Fraction,
    merge_confidence: int = 100,
) -> GridWindowV1:
    """Окно шага для привязки ИСТОЧНИКА, с обеими границами и исходом.

    Нижняя граница. Привязка вершин восстанавливает задуманное отношение, когда
    авторская ошибка меньше половины ячейки. Ошибка растёт как `угол × длина`,
    поэтому она зависит от габарита патча: на большом патче шум больше, а деталь
    декали — нет.

    Верхняя граница — деталь декали: шаг крупнее детали её и съест.

    `merge_confidence` существует потому, что слияние ВЫЧИСЛЕННОЙ точки —
    вероятность, а не свойство: `P ≈ 1 − d/шаг`, и надёжное слияние требует шага
    в сотню раз крупнее расхождения. Для привязки источника этот множитель не
    нужен — там механизм детерминированный, — но параметр оставлен явным, чтобы
    прежнее ошибочное обоснование нельзя было вернуть молча.

    Окно может не существовать вовсе: при габарите ~100 м, угле 7e-6 и детали
    1 мм нижняя граница уходит выше верхней. Это `GRID_WINDOW_CLOSED` —
    именованный отказ, а не повод взять шаг покрупнее и надеяться.
    """

    if extent <= 0 or author_angular_error < 0 or decal_detail <= 0:
        raise ValueError("габарит и деталь положительны, ошибка неотрицательна")
    lower = Fraction(2) * Fraction(author_angular_error) * Fraction(extent)
    upper = Fraction(decal_detail)
    if lower > upper:
        return GridWindowV1(
            GridWindowOutcome.GRID_WINDOW_CLOSED, lower, upper, None
        )
    # Шаг берётся у нижней границы: он чинит вход и максимально щадит деталь.
    step = lower if lower > 0 else upper
    scale = 1
    while Fraction(1, scale) > step:
        scale *= 2
    return GridWindowV1(
        GridWindowOutcome.WINDOW_AVAILABLE, lower, upper, GridSpecV1(scale=scale)
    )
