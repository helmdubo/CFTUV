"""Шаблон покрытия: точки отсечения фронта как `A + alpha * B`, чтобы внутри заверенного интервала не резать грани заново.

ЗАЧЕМ. Покрытие (`coverage.coverage_at`) режет каждую грань скелета полуплоскостью `a*x + b*y - c <= alpha*sqrt(q)`; новая вершина
рождается на ребре грани как `x0 + (x1 - x0) * v0 / (v0 - v1)` (деление `SqrtSumV1` на `SqrtSumV1` - самая дорогая операция покрытия). Но
`v0 = u0 - alpha*sqrt(q)`, `v1 = u1 - alpha*sqrt(q)`, а делитель `v0 - v1 = u0 - u1` от alpha НЕ зависит. Поэтому доля `v0 / (v0 - v1)`
- ТОЧНО аффинная функция alpha, а точка отсечения - `A + alpha * B` с `B = (x1 - x0) * (-sqrt(q) / (u0 - u1))`. Тождество, а не
приближение: коэффициенты - точные `SqrtSumV1`, и значение при любой alpha равно ровно тому, что дал бы полный счёт (каноническая форма
единственна).

ЧТО ЗАВИСИТ ОТ alpha, А ЧТО НЕТ. Комбинаторика отсечения - знаки вершин грани у её фронта - меняется только когда фронт проходит
вершину грани: корень в замкнутой форме, и это заверенный интервал `materialize.interval` (события прихода). Внутри него у каждой
грани один и тот же образец знаков: тот же перечень вершин (исходные и новые на тех же рёбрах), поэтому шаблон - перечень «исходная
вершина i» либо «точка отсечения на ребре (i, j) с коэффициентами». Площадь грани, целиком позади фронта, от alpha не зависит и
берётся готовой; площадь отсечённой грани считается тем же `doubled_shoelace`, что и в полном пути.

САМОПРОВЕРКА. Шаблон записывается только если его воспроизведение при той же alpha равно полному покрытию ТОЧНО (грань за гранью); иначе
шаблона нет, и шаг ширины идёт полным путём (причина названа выше, в `materialize.step`).
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

from ..exact_sqrt_sum import (
    ExactWorkBudgetV1,
    SqrtSumV1,
    _divide_with_prime_universe,
    _prime_universe_from_q_values,
    prime_universe_remembered,
)
from .coverage import CoverageOutcome, CoverageV1, FaceCoverageV1
from .faces import FaceOutcome, FacePartitionV1, doubled_shoelace

#: Грань целиком позади фронта (все знаки не положительны): контур - исходные точки грани, площадь не зависит от alpha.
KIND_BEHIND = "BEHIND"
#: Фронт ещё не дошёл до грани (все знаки не отрицательны): покрытия нет.
KIND_AHEAD = "AHEAD"
#: Фронт режет грань: перечень исходных вершин и точек отсечения.
KIND_CUT = "CUT"


@dataclass(frozen=True, slots=True)
class CutPointV1:
    """Точка отсечения на ребре грани: координаты `base + alpha * slope` (alpha - решётки), точные."""

    base_x: SqrtSumV1
    base_y: SqrtSumV1
    slope_x: SqrtSumV1
    slope_y: SqrtSumV1

    def at(self, alpha: Fraction) -> tuple[SqrtSumV1, SqrtSumV1]:
        return (self.base_x + self.slope_x.scaled(alpha), self.base_y + self.slope_y.scaled(alpha))


@dataclass(frozen=True, slots=True)
class FaceTemplateV1:
    """Образец отсечения одной грани: `items` - индекс исходной вершины либо `CutPointV1`, в порядке контура покрытия."""

    owner: object
    kind: str
    items: tuple = ()
    #: Площадь грани, когда она от alpha не зависит (`KIND_BEHIND`); у остальных пуста.
    area: SqrtSumV1 | None = None
    #: Точек отсечения в образце (для счёта).
    cuts: int = 0


@dataclass(frozen=True, slots=True)
class CoverageTemplateV1:
    """Шаблоны покрытия одного разбиения: по грани разбиения, в том же порядке."""

    faces: tuple
    polygon_doubled_area: int
    cuts: int


def _face_template(face, line, alpha, clipped, covered_area, signs, values, universe, budget) -> FaceTemplateV1 | None:
    """Образец одной грани по знакам и значениям её вершин либо `None`: фронт на вершине (событие) или самопроверка не сошлась."""

    # Грань с фронтом и меньше чем тремя точками в таблице событий интервала (`interval._build_table`) не числится: её образец не заверен.
    if line.q != 0 and len(face.points) < 3:
        return None
    if all(sign <= 0 for sign in signs):
        return FaceTemplateV1(face.owner, KIND_BEHIND, (), covered_area)
    if all(sign >= 0 for sign in signs):
        return FaceTemplateV1(face.owner, KIND_AHEAD)
    if line.q != 0 and any(sign == 0 for sign in signs):
        return None  # фронт на вершине - событие: окрестности нет
    root = SqrtSumV1.radical(1, line.q, budget)
    negative_root = -root
    size = len(face.points)
    items: list = []
    cuts = 0
    for index in range(size):
        current, following = index, (index + 1) % size
        if signs[current] <= 0:
            items.append(current)
        if signs[current] == 0 or signs[following] == 0:
            continue
        if (signs[current] > 0) == (signs[following] > 0):
            continue
        items.append((current, following))
        cuts += 1
    if len(items) != len(clipped):
        return None
    bound: list = []
    for item, point in zip(items, clipped):
        if type(item) is int:
            if point != face.points[item]:
                return None
            bound.append(item)
            continue
        current, following = item
        x0, y0 = face.points[current]
        x1, y1 = face.points[following]
        if root.is_zero:
            share_slope = SqrtSumV1.zero()
        else:
            divisor = values[current] - values[following]
            share_slope = _divide_with_prime_universe(negative_root, divisor, universe, budget)
        slope_x, slope_y = (x1 - x0) * share_slope, (y1 - y0) * share_slope
        bound.append(CutPointV1(point[0] - slope_x.scaled(alpha), point[1] - slope_y.scaled(alpha), slope_x, slope_y))
    template = FaceTemplateV1(face.owner, KIND_CUT, tuple(bound), None, cuts)
    return template if _points_of(template, face, alpha) == clipped else None


def _points_of(template: FaceTemplateV1, face, alpha: Fraction):
    if template.kind == KIND_BEHIND:
        return face.points
    if template.kind == KIND_AHEAD:
        return ()
    return tuple(face.points[item] if type(item) is int else item.at(alpha) for item in template.items)


def build_template(partition: FacePartitionV1, alpha: Fraction, work_budget, store, coverage: CoverageV1, traces: list):
    """Шаблон покрытия разбиения при `alpha` по ПОЛНОМУ покрытию и записи знаков (`traces`, по грани), либо `None`."""

    if partition.outcome is not FaceOutcome.EXACT or coverage.outcome is not CoverageOutcome.EXACT:
        return None
    if len(traces) != len(partition.faces) or len(coverage.faces) != len(partition.faces):
        return None
    lines = tuple(face.line for face in partition.faces)
    if any(line is None for line in lines):
        return None
    universe = prime_universe_remembered(tuple(line.q for line in lines), work_budget, store, _prime_universe_from_q_values)
    faces = []
    cuts = 0
    for face, line, covered, (signs, values) in zip(partition.faces, lines, coverage.faces, traces):
        template = _face_template(face, line, alpha, covered.points, covered.doubled_area, signs, values, universe, work_budget)
        if template is None:
            return None
        faces.append(template)
        cuts += template.cuts
    return CoverageTemplateV1(tuple(faces), partition.polygon_doubled_area, cuts)


def instantiate(template: CoverageTemplateV1, partition: FacePartitionV1, alpha: Fraction, work_budget) -> CoverageV1 | None:
    """Покрытие при `alpha` из шаблона: то же значение, что `coverage_at`, без отсечения и делений; `None` - шаблон не про это разбиение."""

    if partition.outcome is not FaceOutcome.EXACT or alpha < 0 or len(template.faces) != len(partition.faces):
        return None
    if template.polygon_doubled_area != partition.polygon_doubled_area:
        return None
    covered = []
    total = SqrtSumV1.zero()
    for face, pattern in zip(partition.faces, template.faces):
        if pattern.owner != face.owner:
            return None
        points = _points_of(pattern, face, alpha)
        if pattern.kind == KIND_BEHIND:
            doubled = pattern.area
        else:
            doubled = doubled_shoelace(points) if len(points) >= 3 else SqrtSumV1.zero()
        covered.append(FaceCoverageV1(face.owner, points, doubled))
        total = total + doubled
    return CoverageV1(
        CoverageOutcome.EXACT,
        alpha,
        tuple(covered),
        total,
        partition.polygon_doubled_area,
        work_budget=work_budget,
    )
