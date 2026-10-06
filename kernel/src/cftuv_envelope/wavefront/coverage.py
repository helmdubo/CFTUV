"""Покрытие к моменту alpha: грань, усечённая по ВРЕМЕНИ, и её точная площадь.

Зачем усечение. Скелет считается до конца, а покрытие декали растёт до конечной
alpha (в поле `bf6` — 1/4). Сравнивать «до конца» с «до alpha» нельзя, поэтому
грань обрезается полуплоскостью «фронт этого ребра уже дошёл»:

    a*x + b*y - c <= alpha*sqrt(q)

Это не приближение и не ещё одна модель, и оно не требует единичной скорости.
Фронт ребра в момент `t` — это прямая `a*x + b*y = c + t*sqrt(q)`; точка позади
фронта тогда и только тогда, когда `a*p_x + b*p_y - c <= t*sqrt(q)`. То есть
неравенство выше — буквально «время прихода не больше alpha», и никакой второй
геометрии для него не понадобилось. Расстояние в этот вывод не входит вовсе:
`dist(p, line) = t*sqrt(q)/|(a, b)|` зависит от скорости, а само неравенство
нет.

Резание идёт по Сазерленду—Хоџмену над `SqrtSumV1`: знак вершины решается
`sign()`, точка на ребре — делением `SqrtSumV1` на `SqrtSumV1` (оно замкнуто
через сопряжённые). Ни одного порога, ни одного float.

ОБЪЯВЛЕННЫЕ ГРАНИЦЫ (границ две, и обе проверяются тестом на собственном
результате этой функции — `test_wavefront_coverage.py`):

1. **Покрытие не убывает по alpha.** `alpha1 <= alpha2` влечёт
   `area(alpha1) <= area(alpha2)`. Это определение роста фронта, а не пожелание.
2. **Покрытие грани не превосходит самой грани, а сумма — площади
   многоугольника.** Фронт ребра не выходит за свою грань, и владелец точки
   единствен, поэтому суммирование по граням не может дать больше области.
"""

from __future__ import annotations

from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass, field
from enum import Enum
from fractions import Fraction

from .event_time import SupportLineV1
from ..exact_sqrt_sum import (
    ExactWorkBudgetV1,
    _divide_with_prime_universe,
    _prime_universe_from_q_values,
    prime_universe_remembered,
)
from .faces import (
    EdgeKey,
    FaceOutcome,
    FacePartitionV1,
    FaceV1,
    doubled_shoelace,
)
from .sqrt_sum import SqrtSumV1


Point = tuple[SqrtSumV1, SqrtSumV1]


class CoverageOutcome(str, Enum):
    """Чем кончился счёт покрытия. Тихого нуля здесь нет."""

    EXACT = "EXACT"
    PARTITION_IS_NOT_EXACT = "PARTITION_IS_NOT_EXACT"
    ALPHA_IS_NEGATIVE = "ALPHA_IS_NEGATIVE"


@dataclass(frozen=True, slots=True)
class FaceCoverageV1:
    """Часть грани, до которой фронт её ребра дошёл к моменту alpha."""

    owner: EdgeKey
    points: tuple[Point, ...]
    doubled_area: SqrtSumV1


@dataclass(frozen=True, slots=True)
class CoverageV1:
    """Покрытие области к моменту alpha, с owner'ом у каждого куска."""

    outcome: CoverageOutcome
    alpha: Fraction
    faces: tuple[FaceCoverageV1, ...]
    doubled_area: SqrtSumV1
    polygon_doubled_area: int
    detail: str = ""
    # Бюджет транзакции — по той же причине, что у `FacePartitionV1`: обе
    # объявленные границы покрытия спрашиваются СВОЙСТВАМИ, а у свойства нет
    # аргумента. Из тождества покрытия исключён: цена ответа — не ответ.
    work_budget: ExactWorkBudgetV1 | None = field(
        default=None, compare=False, repr=False
    )

    def doubled_area_of(self, owner: EdgeKey) -> SqrtSumV1:
        for face in self.faces:
            if face.owner == owner:
                return face.doubled_area
        return SqrtSumV1.zero()

    @property
    def covers_everything(self) -> bool:
        """Покрытие совпало с областью ТОЧНО: разность — пустой набор."""

        return (
            self.doubled_area - SqrtSumV1.rational(self.polygon_doubled_area)
        ).is_zero

    @property
    def does_not_exceed_the_polygon(self) -> bool:
        """Граница 2, часть про сумму: покрытие не больше области."""

        return (
            SqrtSumV1.rational(self.polygon_doubled_area) - self.doubled_area
        ).sign(budget=self.work_budget) >= 0


def _value(line: SupportLineV1, point: Point, front: SqrtSumV1) -> SqrtSumV1:
    """`a*x + b*y - c - alpha*sqrt(q)`. Ноль — на самом фронте, точно. `front` — `alpha*sqrt(q)` грани."""

    return (
        point[0].scaled(line.a)
        + point[1].scaled(line.b)
        - SqrtSumV1.rational(line.c)
        - front
    )


def clip_to_halfplane(
    points: tuple[Point, ...],
    line: SupportLineV1,
    alpha: Fraction,
    *,
    prime_universe: tuple[int, ...] | None = None,
    budget: ExactWorkBudgetV1 | None = None,
    trace: list | None = None,
) -> tuple[Point, ...]:
    """Сазерленд—Хоџмен по `a*x + b*y - c <= alpha*sqrt(q)`, точно.

    Выпуклость резака гарантирует, что результат остаётся одним контуром:
    полуплоскость выпукла, а грань скелета односвязна.

    `trace` (список) — запись для шаблона покрытия (`coverage_template`): знаки и значения вершин. Ответа не меняет.
    """

    # Фронт грани `alpha*sqrt(q)` один на все её вершины: считать радикал (разложение `q`) на каждую вершину незачем.
    front = SqrtSumV1.radical(alpha, line.q, budget)
    values = [_value(line, point, front) for point in points]
    signs = [value.sign(budget=budget) for value in values]
    if trace is not None:
        trace.append((signs, values))
    if all(sign <= 0 for sign in signs):
        return points
    if all(sign >= 0 for sign in signs):
        return ()

    result: list[Point] = []
    size = len(points)
    for index in range(size):
        current, following = index, (index + 1) % size
        if signs[current] <= 0:
            result.append(points[current])
        if signs[current] == 0 or signs[following] == 0:
            continue
        if (signs[current] > 0) == (signs[following] > 0):
            continue
        # Точка пересечения: t = v0 / (v0 - v1), деление точное.
        divisor = values[current] - values[following]
        share = (
            values[current].divided_by(divisor, budget)
            if prime_universe is None
            else _divide_with_prime_universe(
                values[current],
                divisor,
                prime_universe,
                budget,
            )
        )
        x0, y0 = points[current]
        x1, y1 = points[following]
        result.append(
            (
                x0 + (x1 - x0) * share,
                y0 + (y1 - y0) * share,
            )
        )
    return tuple(result)


def _line_of(face: FaceV1) -> SupportLineV1:
    """Несущая прямая грани. Хранится в самой грани, а не выводится заново.

    История здесь двухступенчатая, и обе ступени про одно: прямая грани не
    выводится из ключа владельца. Сначала стояло `SupportLineV1(*face.owner)` —
    ключ владельца И БЫЛ прямой, и это совмещение ролей сливало грани двух
    коллинеарных рёбер. Потом прямая собиралась из КОНЦОВ ребра плюс `q`, и это
    верно ровно пока у ребра есть длина: у скрытой опоры веера концы совпадают,
    нормаль из них не выводится, и `SupportLineV1.with_speed` честно падает
    `DegenerateEdgeError`.

    Нормаль веера задана ВХОДОМ и ниоткуда больше не следует, поэтому грань
    несёт прямую целиком. `None` бывает только у грани, собранной вручную мимо
    `build_faces`; она отвечает исходом, а не молчаливой нулевой прямой.
    """

    if face.line is None:
        raise ValueError(f"у грани {face.owner} нет несущей прямой")
    return face.line


#: Источник покрытия шага ширины (`materialize.step`): объект с методом `coverage(partition, alpha, work_budget, store)`, который
#: отдаёт покрытие либо `None` (не знает разбиения - считает `coverage_at` сам). Покрытие, воспроизведённое из шаблона внутри заверенного
#: интервала, ТО ЖЕ значение, что даёт `_coverage_at`, и ту же цену (шаблон несёт записанную цену полного счёта), поэтому и остальной
#: ответ не меняется. Контекст потока: два шага не делят источник. Памяти покрытий между вычислениями НЕТ: контуры, которых ждёт
#: материализатор, несёт то же покрытие, которое посчитала очередь в этом вычислении (`ConveyorRegionCoverageV1.contours`), а не
#: процессная память, чьё попадание не платило и делало цену свойством истории.
_SOURCE: ContextVar = ContextVar("cftuv_coverage_source", default=None)


@contextmanager
def coverage_source(source):
    """Внутри блока `coverage_at` спрашивает `source` (шаблоны покрытия шага ширины); снаружи - как прежде."""

    token = _SOURCE.set(source)
    try:
        yield source
    finally:
        _SOURCE.reset(token)


def current_coverage_source():
    """Источник покрытия шага ширины, действующий в этом потоке, либо `None`."""

    return _SOURCE.get()


def coverage_at(
    partition: FacePartitionV1,
    alpha: Fraction,
    work_budget: ExactWorkBudgetV1 | None = None,
    store: dict | None = None,
) -> CoverageV1:
    """Покрытие к моменту alpha по граням скелета, с точной площадью. `store` — память подготовки (вселенная простых)."""

    alpha = Fraction(alpha)
    source = _SOURCE.get()
    if source is not None:
        produced = source.coverage(partition, alpha, work_budget, store)
        if produced is not None:
            return produced
    return _coverage_at(partition, alpha, work_budget, store)


def _coverage_at(
    partition: FacePartitionV1,
    alpha: Fraction,
    work_budget: ExactWorkBudgetV1 | None,
    store: dict | None,
    traces: list | None = None,
) -> CoverageV1:
    if partition.outcome is not FaceOutcome.EXACT:
        return CoverageV1(
            CoverageOutcome.PARTITION_IS_NOT_EXACT,
            alpha,
            (),
            SqrtSumV1.zero(),
            partition.polygon_doubled_area,
            partition.outcome.value,
        )
    if alpha < 0:
        return CoverageV1(
            CoverageOutcome.ALPHA_IS_NEGATIVE,
            alpha,
            (),
            SqrtSumV1.zero(),
            partition.polygon_doubled_area,
            str(alpha),
        )

    face_lines = tuple(_line_of(face) for face in partition.faces)
    prime_universe = prime_universe_remembered(
        tuple(line.q for line in face_lines), work_budget, store, _prime_universe_from_q_values
    )
    covered: list[FaceCoverageV1] = []
    total = SqrtSumV1.zero()
    for face, line in zip(partition.faces, face_lines):
        entry: list | None = None if traces is None else []
        clipped = clip_to_halfplane(
            face.points,
            line,
            alpha,
            prime_universe=prime_universe,
            budget=work_budget,
            trace=entry,
        )
        if traces is not None:
            traces.append(entry[0])
        doubled = doubled_shoelace(clipped) if len(clipped) >= 3 else (
            SqrtSumV1.zero()
        )
        covered.append(FaceCoverageV1(face.owner, clipped, doubled))
        total = total + doubled
    return CoverageV1(
        CoverageOutcome.EXACT,
        alpha,
        tuple(covered),
        total,
        partition.polygon_doubled_area,
        work_budget=work_budget,
    )
