"""Эталон покрытия ПРЯМОУГОЛЬНОЙ фигуры с МЯГКИМ вогнутым углом. Вне кода очереди.

Митрованный эталон (`mitered_standard.py`) назвал свою точку семейства явно:
вогнутая вершина у него ОСТРАЯ, и `alpha^2` на неё — верхняя граница семейства.
Продукту нужна другая точка того же семейства — `LINEAR_REFLEX_EQUAL_V1`, — и
здесь она считается замкнутой формой, независимой от `wavefront/` целиком.

**ЧТО ИМЕННО СЧИТАЕТСЯ, и это не хорда.** Профиль при `k` скрытых опорах — не
срез угла одной хордой (это BEVEL, которого в продукте нет), а ЦЕПЬ ИЗ `k + 1`
ЗВЕНЬЕВ. Через вогнутую вершину проходят `k + 2` несущие прямые: входящая,
`k` скрытых и исходящая; каждая отодвигается на `alpha` по своей нормали;
вершины цепи — пересечения СОСЕДНИХ отодвинутых прямых. При `k = 0` цепь
вырождается в одну точку — митрованный апекс, — поэтому семейство здесь одно, а
не два разных построения.

**ВЫВОД ЗАМКНУТОЙ ФОРМЫ (руками, для прямого угла).**

Поставим вогнутую вершину в начало координат так, что материал — дополнение
открытого квадранта `{X > 0, Y > 0}`. Тогда входящее ребро лежит на `Y = 0`
(его фронт идёт в `Y = -t`), исходящее на `X = 0` (фронт в `X = -t`), избыток
`delta = phi - pi = pi/2`, и продуктовое `delta_max = pi/3` даёт
`k = max(0, ceil((pi/2)/(pi/3)) - 1) = 1`.

Единственная скрытая опора смотрит по биссектрисе: нормаль `(-1, -1)/sqrt(2)`,
прямая через вершину, фронт `X + Y = -t*sqrt(2)`.

К моменту `alpha` фронт в углу состоит из трёх кусков, и множество ПОЗАДИ него
внутри квадрата `S = [-alpha, 0]^2` есть пересечение трёх полуплоскостей:

    Y >= -alpha,   X >= -alpha,   X + Y >= -alpha*sqrt(2)

Первые две дают сам квадрат, третья срезает его угол. Срезанный треугольник
прямоуголен и равнобедрен: прямая `X + Y = -alpha*sqrt(2)` пересекает `X = -alpha`
в `Y = -alpha*(sqrt(2) - 1)` и симметрично. Катет равен

    alpha - alpha*(sqrt(2) - 1) = alpha*(2 - sqrt(2))

и площадь среза

    T(alpha) = (1/2) * alpha^2 * (2 - sqrt(2))^2 = alpha^2 * (3 - 2*sqrt(2))

Митрованный ответ накрывает ВЕСЬ квадрат `S` (апекс уходит по биссектрисе в
`(-alpha, -alpha)`), поэтому

    покрытие с веером = митрованное покрытие - T(alpha) на каждую вогнутую вершину

`3 - 2*sqrt(2) = 0.1715...`, то есть недобор меньше, чем у круглого отступа
(`1 - pi/4 = 0.2146...`), и это ровно то, чем полигональная дуга отличается от
круговой: цепь описана ВОКРУГ дуги и накрывает больше неё.

**ОБЩИЙ ВИД, который здесь не считается, но записан, потому что плотность
сегментов объявлена будущим пользовательским параметром.** Для прямого угла и
`k` скрытых опор шаг поворота `d = (pi/2)/(k + 1)`; вершины цепи стоят на
расстоянии `alpha/cos(d/2)` от угла, соседние — под углом `d`. Складывая `k`
центральных треугольников `(1/2)*r^2*sin d` и два крайних `(1/2)*alpha*r*sin(d/2)`,
получаем площадь накрытого куска квадрата

    A(k) = (k + 1) * alpha^2 * tan(pi / (4*(k + 1)))

и недобор `T_k = alpha^2 * [1 - (k + 1)*tan(pi/(4*(k+1)))]`. Проверки на концах:
`k = 0` даёт `A = alpha^2` (весь квадрат, митр), `k = 1` даёт
`2*alpha^2*(sqrt(2) - 1)`, то есть `T_1 = alpha^2*(3 - 2*sqrt(2))` — та же
величина, что выведена выше независимо; `k -> inf` даёт `pi*alpha^2/4`, то есть
круглый отступ. Три точки семейства сходятся, значит формула про семейство, а не
про один случай.

**ОТРИЦАТЕЛЬНЫЙ КОНТРОЛЬ — ХОРДА.** Если бы профиль был BEVEL (одна хорда между
двумя отодвинутыми рёбрами, `X + Y >= -alpha`), срез был бы треугольником с
катетом `alpha` и площадью `alpha^2/2`. Хорда, стало быть, накрывает МЕНЬШЕ цепи
ровно на

    alpha^2 * (1/2 - (3 - 2*sqrt(2))) = alpha^2 * (2*sqrt(2) - 5/2) = 0.3284...*alpha^2

Число записано затем, чтобы перепутать две модели было нельзя: расхождение
почти вдвое больше самого недобора цепи.

**ГРАНИЦА ПРИМЕНИМОСТИ — ТА ЖЕ, ЧТО У МИТРОВАННОГО ЭТАЛОНА, и она ПРОВЕРЯЕТСЯ,
а не предполагается.** Вычитание `T` на вершину верно ровно тогда, когда квадрат
`S` этой вершины не задет ничем чужим: иначе срезанный треугольник накрыл бы
кто-то другой, и вычитать его было бы неверно. Ровно это и означает равенство
`strip_deficit == reflex_square_budget` у `rectilinear_standard`: полосы всех
рёбер промахиваются мимо ВСЕХ квадратов целиком, значит квадраты попарно
не пересекаются, лежат внутри фигуры и накрыты только собственным углом.
Не выполнено — именованный исход, а не поправка.

Своей сетки здесь нет и быть не может: сторона среза иррациональна
(`alpha*sqrt(2)`), а точная сетка `mitered_standard` живёт в дробях. Поэтому
ответ возвращается `SqrtSumV1` — точной суммой корней, сравнимой с покрытием
очереди побитово, — а вся рациональная работа переиспользуется у митрованного
эталона, а не пересчитывается вторым способом.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction
from math import gcd

from cftuv_envelope.wavefront.polygon import (
    FanSupportV1,
    PolygonV1,
    VertexFanV1,
    with_vertex_fans,
)
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

from mitered_standard import StandardOutcome, axis_edges, rectilinear_standard


class ChamferOutcome(str, Enum):
    """Чем кончился счёт эталона мягкого угла. Пустого ответа без имени нет."""

    EXACT = "EXACT"
    # Митрованный эталон не сошёлся сам: его исход лежит в `detail`, а не
    # пересказывается своими словами.
    MITERED_STANDARD_REFUSED = "MITERED_STANDARD_REFUSED"
    # Квадраты вогнутых вершин пересекаются, вылезают из фигуры либо задеты
    # чужой полосой. Тогда «вычесть T на вершину» перестаёт быть верным, и
    # поправлять его нечем: чужое покрытие внутри квадрата эталон не считает.
    REFLEX_SQUARES_ARE_NOT_INDEPENDENT = "REFLEX_SQUARES_ARE_NOT_INDEPENDENT"
    # Профиль запрошен с `k`, для которого замкнутая форма здесь не выведена.
    # Выведена она для `k = 1` (продуктовый выбор при прямом угле); общий вид
    # записан в докстроке, но он ИРРАЦИОНАЛЕН в тангенсе и `SqrtSumV1` его не
    # выражает, поэтому молчаливого приближения тут не будет.
    HIDDEN_EDGE_COUNT_IS_NOT_ONE = "HIDDEN_EDGE_COUNT_IS_NOT_ONE"
    # У фигуры есть не осевое ребро: прямой угол в вогнутой вершине больше не
    # гарантирован, а вывод выше стоит именно на нём.
    EDGE_IS_NOT_AXIS_PARALLEL = "EDGE_IS_NOT_AXIS_PARALLEL"


@dataclass(frozen=True, slots=True)
class ChamferedStandardV1:
    """Три покрытия одной фигуры к моменту alpha: полосы, цепь, митр.

    Все три — УДВОЕННЫЕ площади, потому что с ними сравнивается `doubled_area`
    очереди, и переводить туда-сюда значило бы завести второе место, где можно
    поделить не на то.
    """

    outcome: ChamferOutcome
    alpha: Fraction
    reflex_count: int
    polygon_doubled_area: Fraction
    doubled_mitered: SqrtSumV1
    doubled_chained: SqrtSumV1
    doubled_bevelled: SqrtSumV1
    doubled_strips: SqrtSumV1
    detail: str = ""

    @property
    def doubled_corner_deficit(self) -> SqrtSumV1:
        """Насколько цепь накрыла МЕНЬШЕ митра. Ровно `r*alpha^2*(6-4*sqrt(2))`."""

        return self.doubled_mitered - self.doubled_chained

    @property
    def doubled_chain_over_bevel(self) -> SqrtSumV1:
        """Насколько цепь накрыла БОЛЬШЕ хорды. Отрицательный контроль моделей."""

        return self.doubled_chained - self.doubled_bevelled


def _primitive(vector: tuple[int, int]) -> tuple[int, int]:
    common = gcd(abs(vector[0]), abs(vector[1]))
    return (vector[0] // common, vector[1] // common)


def right_angle_fans(polygon: PolygonV1) -> tuple[VertexFanV1, ...]:
    """Веер `k = 1` в каждой вогнутой вершине ПРЯМОУГОЛЬНОЙ фигуры.

    Это ВХОД задачи, а не ответ, и пишется он здесь именно поэтому: эталон
    предсказывает, что очередь ответит на такой вход, и оба должны быть видны
    рядом. Направление скрытой опоры — сумма ПРИВЕДЁННЫХ нормалей соседних
    рёбер: приведение обязательно, иначе рёбра разной длины дали бы «биссектрису»,
    наклонённую к длинному, — то есть не биссектрису.

    У прямого угла приведённые нормали перпендикулярны и единичны, поэтому сумма
    имеет `|n|^2 = 2`, и единичная скорость записывается точно: `q = 2`. Никакого
    корня в записи не появляется — ровно то свойство, которое делает
    перпендикулярный веер РАЦИОНАЛЬНЫМ, а общий случай нет.
    """

    fans: list[VertexFanV1] = []
    for loop in polygon.loops:
        points = loop.points
        size = len(points)
        for index, is_reflex in enumerate(loop.reflex_flags()):
            if not is_reflex:
                continue
            here = points[index]
            before = points[(index - 1) % size]
            after = points[(index + 1) % size]
            incoming = _primitive((-(here[1] - before[1]), here[0] - before[0]))
            outgoing = _primitive((-(after[1] - here[1]), after[0] - here[0]))
            normal = (incoming[0] + outgoing[0], incoming[1] + outgoing[1])
            fans.append(
                VertexFanV1(
                    here,
                    (
                        FanSupportV1(
                            normal[0],
                            normal[1],
                            normal[0] * normal[0] + normal[1] * normal[1],
                        ),
                    ),
                )
            )
    return tuple(fans)


def with_right_angle_fans(polygon: PolygonV1) -> PolygonV1:
    """Та же фигура, у которой КАЖДАЯ вогнутая вершина мягкая при `k = 1`."""

    return with_vertex_fans(polygon, right_angle_fans(polygon))


def corner_deficit_doubled(alpha: Fraction) -> SqrtSumV1:
    """Удвоенный недобор цепи `k = 1` в ОДНОЙ вогнутой вершине прямого угла.

    `2 * alpha^2 * (3 - 2*sqrt(2)) = 6*alpha^2 - 4*alpha^2*sqrt(2)`.
    """

    square = alpha * alpha
    return SqrtSumV1.rational(6 * square) - SqrtSumV1.radical(4 * square, 2)


def bevel_deficit_doubled(alpha: Fraction) -> SqrtSumV1:
    """Удвоенный недобор ХОРДЫ в одной вогнутой вершине прямого угла: `alpha^2`."""

    return SqrtSumV1.rational(alpha * alpha)


def _refused(
    outcome: ChamferOutcome,
    alpha: Fraction,
    polygon: PolygonV1,
    detail: str,
) -> ChamferedStandardV1:
    return ChamferedStandardV1(
        outcome,
        alpha,
        polygon.reflex_count,
        Fraction(0),
        SqrtSumV1.zero(),
        SqrtSumV1.zero(),
        SqrtSumV1.zero(),
        SqrtSumV1.zero(),
        detail,
    )


def chamfered_standard(
    polygon: PolygonV1, alpha: Fraction, *, hidden_edge_count: int = 1
) -> ChamferedStandardV1:
    """Покрытие прямоугольной фигуры профилем `LINEAR_REFLEX_EQUAL_V1`, точно."""

    alpha = Fraction(alpha)
    if hidden_edge_count != 1:
        return _refused(
            ChamferOutcome.HIDDEN_EDGE_COUNT_IS_NOT_ONE,
            alpha,
            polygon,
            f"k = {hidden_edge_count}",
        )
    if axis_edges(polygon) is None:
        return _refused(
            ChamferOutcome.EDGE_IS_NOT_AXIS_PARALLEL,
            alpha,
            polygon,
            "у фигуры есть не осевое ребро",
        )
    mitered = rectilinear_standard(polygon, alpha)
    if mitered.outcome is not StandardOutcome.EXACT:
        return _refused(
            ChamferOutcome.MITERED_STANDARD_REFUSED,
            alpha,
            polygon,
            mitered.outcome.value,
        )
    if not mitered.deficit_is_the_full_budget:
        return _refused(
            ChamferOutcome.REFLEX_SQUARES_ARE_NOT_INDEPENDENT,
            alpha,
            polygon,
            f"недобор полос {mitered.strip_deficit} против бюджета "
            f"{mitered.reflex_square_budget}",
        )
    reflex = mitered.reflex_count
    doubled_mitered = SqrtSumV1.rational(2 * mitered.mitered_covered)
    return ChamferedStandardV1(
        outcome=ChamferOutcome.EXACT,
        alpha=alpha,
        reflex_count=reflex,
        polygon_doubled_area=2 * mitered.polygon_area,
        doubled_mitered=doubled_mitered,
        doubled_chained=doubled_mitered
        - corner_deficit_doubled(alpha).scaled(reflex),
        doubled_bevelled=doubled_mitered
        - bevel_deficit_doubled(alpha).scaled(reflex),
        doubled_strips=SqrtSumV1.rational(2 * mitered.strip_covered),
    )
