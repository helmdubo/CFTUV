"""Закон растворения прямых вершин петли домена.

ЧТО РАСТВОРЯЕТСЯ И ЗАЧЕМ. Вершина петли, у которой смежные рёбра коллинеарны и
СОНАПРАВЛЕНЫ, геометрически не существует: граница в этом месте — одна прямая,
разрезанная лишним узлом. Такой узел стоит дважды. В скелете он рождает
вершину третьего вида (`skeleton._Vertex.sliding`): у развёрнутого угла точки
пересечения биссектрис нет, и место с временами ей считают отдельным путём — то
есть лишний узел оплачивается не только счётом, но и особой веткой. В мосте он
удваивает блок класса несущей прямой: два ребра вместо одного при одном законе
дают `undetermined` — «источником в классе является не всякое ребро, а какое
именно, во входе не записано», — и фронт с этого источника не идёт вовсе.

ЗАКОН (`DISSOLVE_STRAIGHT_V3`). Вершина растворима тогда и только тогда, когда
выполнены ТРИ условия:

(а) ТОЧНЫЙ нулевой поворот. Векторное произведение смежных направлений равно
    нулю ТОЧНО, дробями, и скалярное строго положительно. Допуска здесь нет и
    быть не может: допуск растворял бы почти-прямые вершины, а «почти прямая»
    есть другая геометрия, и её потеря была бы не оптимизацией, а сглаживанием
    молча. Положительность скалярного отделяет прямой угол от РАЗВЁРНУТОГО:
    у шпильки (ход назад по той же прямой) произведение тоже нулевое, но
    растворение склеило бы две стороны нулевой площади в ничто.

(б) Класс несущей прямой несёт НЕ БОЛЕЕ ОДНОГО закона прихода. Условие
    записано счётом законов в классе, а не сравнением «закона левого ребра» с
    «законом правого», и это не упрощение — это единственная форма, которую
    вход допускает. `PlainArrivalLawV1` не имеет концов вовсе (`name`,
    `normal_x`, `normal_y`, `constant`, `speed_squared`): закон ключуется
    ПРЯМОЙ, а не ребром. Какое ребро класса чей закон — как раз то, что мост
    называет незаписанным во входе (`ambiguous`, `undetermined`). Поэтому
    «оба ребра несут один закон» проверяемо ровно тогда, когда закон в классе
    один — или когда его нет вовсе и оба ребра суть стена.

    Что гарантирует условие. При нуле законов оба ребра — стена, и слияние
    двух стен даёт стену: сопоставление не двигается. При одном законе и блоке
    из `B` рёбер сопоставлено `min(1, B) = 1` и до слияния, и после
    (`B ≥ 2`, значит `B−1 ≥ 1`), то есть число сопоставленных не меняется, а
    определённость владельца может только вырасти: блок из двух рёбер был
    `undetermined`, блок из одного становится вынужденной парой. При двух и
    более законах слияние РАЗРУШИЛО БЫ возможное однозначное сопоставление
    один-к-одному, превратив закон в `surplus`, — поэтому такая вершина
    неприкосновенна.

(в) Вершина не в стоп-листе якорей. Стоп-лист приходит подсчётом ссылок по
    входам; здесь он потребляется как множество точек. Ссылка есть — не
    растворять, без исключений: якорь веера, конец chain use, граница
    владельца и вершина junction суть ИМЕНОВАННЫЕ места, и растворить
    названное место значит потерять имя, а не узел.

Класс несущей прямой у слитого ребра тот же, что у обоих исходных, и это
доказуемо, а не наблюдение: при `cross = 0` и `dot > 0` второе направление
есть `k·d` при `k > 0`, нормаль второго ребра есть `k·n`, а его константа
`n·v = n·(p + d) = n·p`, потому что `n ⊥ d`. Значит тройка `(a, b, c)` второго
ребра пропорциональна первой с положительным множителем — ровно предикат
`bridge._proportional_positive`, то есть один и тот же `line_class`. Ключ
класса берётся у моста, а не переписывается здесь: правило класса обязано быть
одно.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

from .bridge import line_class


DISSOLVE_STRAIGHT_LAW_V3 = "DOMAIN_LOOP_DISSOLVE_STRAIGHT_V3"
"""Имя закона. Версия печатается в записях, чтобы байты были различимы ПО ИМЕНИ.

Сдвиг байтов, вызванный растворением, обязан атрибутироваться ЗАКОНУ, а не
«оптимизации»: оптимизация по определению не меняет ответ, а этот срез его
меняет — узлов скелета меньше, определённость владельца выше. Запись без
версии оставила бы будущего читателя перед расхождением дайджестов без имени
причины.
"""

RationalPoint = tuple[Fraction, Fraction]
RationalLoop = tuple[RationalPoint, ...]


@dataclass(frozen=True, slots=True)
class DissolveReportV1:
    """Что растворено и что удержано — числами, а не флагом.

    Удержанные разобраны ПО ПРИЧИНЕ. Одно общее число «не растворили» не
    отличило бы работающий стоп-лист от неработающего предиката, а это разные
    состояния с разными следствиями.
    """

    law: str
    dissolved_count: int
    #: Прямые вершины, удержанные стоп-листом якорей.
    anchored_count: int
    #: Прямые вершины, удержанные множественностью законов в классе.
    contested_count: int
    #: Вершины с нулевым поворотом и ОБРАТНЫМ ходом (шпильки). Не растворяются
    #: и считаются отдельно: это не «занятая» вершина, а другая геометрия.
    reversal_count: int
    #: Точки, растворённые фактически, в порядке обхода. Для расписки и тестов.
    dissolved_points: tuple[RationalPoint, ...]

    @property
    def changed(self) -> bool:
        return self.dissolved_count > 0


def _direction(tail: RationalPoint, head: RationalPoint):
    return (head[0] - tail[0], head[1] - tail[1])


def is_straight_turn(previous, vertex, following) -> bool:
    """Точный нулевой поворот СОНАПРАВЛЕННЫХ рёбер. Ни допуска, ни разворота."""

    incoming = _direction(previous, vertex)
    outgoing = _direction(vertex, following)
    if incoming == (0, 0) or outgoing == (0, 0):
        return False
    cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
    if cross != 0:
        return False
    return incoming[0] * outgoing[0] + incoming[1] * outgoing[1] > 0


def is_reversal(previous, vertex, following) -> bool:
    """Нулевой поворот с ОБРАТНЫМ ходом — шпилька. Растворению не подлежит."""

    incoming = _direction(previous, vertex)
    outgoing = _direction(vertex, following)
    if incoming == (0, 0) or outgoing == (0, 0):
        return False
    cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
    if cross != 0:
        return False
    return incoming[0] * outgoing[0] + incoming[1] * outgoing[1] < 0


def law_counts_by_class(laws) -> dict:
    """Сколько законов приходится на каждый класс несущей прямой.

    Ключ строит `bridge.line_class` — та же власть, что сопоставляет законы с
    рёбрами. Своя копия правила класса разошлась бы с ней ровно так же, как
    расходятся любые две копии одного закона.
    """

    counts: dict = {}
    for law in laws:
        key = line_class((law.normal_x, law.normal_y, law.constant))
        if key is None:
            continue
        counts[key] = counts.get(key, 0) + 1
    return counts


def _edge_class(tail: RationalPoint, head: RationalPoint):
    dx, dy = head[0] - tail[0], head[1] - tail[1]
    a, b = -dy, dx
    return line_class((a, b, a * tail[0] + b * tail[1]))


def dissolve_straight_vertices(
    loops: tuple[RationalLoop, ...],
    *,
    laws=(),
    protected_points: frozenset = frozenset(),
) -> tuple[tuple[RationalLoop, ...], DissolveReportV1]:
    """Убрать прямые вершины петель, слив смежные рёбра. Закон — в докстроке модуля.

    Обход ОДНОПРОХОДНЫЙ и по исходной петле: растворение одной вершины не
    открывает новых кандидатов, потому что прямизна соседа определяется его
    собственными рёбрами, а слияние сохраняет направление. Многопроходность
    добавила бы порядковую зависимость там, где её нет.

    Петля короче четырёх вершин не трогается вовсе: треугольник с прямой
    вершиной есть вырожденный отрезок, и растворять его значило бы менять не
    запись геометрии, а саму геометрию.
    """

    counts = law_counts_by_class(laws)
    result: list[RationalLoop] = []
    dissolved: list[RationalPoint] = []
    anchored = contested = reversal = 0

    for loop in loops:
        size = len(loop)
        if size < 4:
            result.append(loop)
            continue
        kept: list[RationalPoint] = []
        for index in range(size):
            previous = loop[(index - 1) % size]
            vertex = loop[index]
            following = loop[(index + 1) % size]
            if is_reversal(previous, vertex, following):
                reversal += 1
                kept.append(vertex)
                continue
            if not is_straight_turn(previous, vertex, following):
                kept.append(vertex)
                continue
            if vertex in protected_points:
                anchored += 1
                kept.append(vertex)
                continue
            key = _edge_class(previous, vertex)
            if key is not None and counts.get(key, 0) > 1:
                contested += 1
                kept.append(vertex)
                continue
            dissolved.append(vertex)
        # Растворение не имеет права оставить петлю без геометрии: если после
        # фильтрации осталось меньше трёх вершин, петля возвращается как была.
        # Такого на прямых вершинах не бывает по построению, и проверка стоит
        # здесь не «на всякий случай», а чтобы это утверждение было исполняемым.
        if len(kept) < 3:
            result.append(loop)
            for point in loop:
                if point in dissolved:
                    dissolved.remove(point)
            continue
        result.append(tuple(kept))

    return tuple(result), DissolveReportV1(
        law=DISSOLVE_STRAIGHT_LAW_V3,
        dissolved_count=len(dissolved),
        anchored_count=anchored,
        contested_count=contested,
        reversal_count=reversal,
        dissolved_points=tuple(dissolved),
    )
