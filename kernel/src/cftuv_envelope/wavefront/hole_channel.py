"""HOLE_CHANNEL_V1: дыры домена с ползущими стенами соединяются с внешним контуром узким каналом в их тени.

ЗАДАЧА. Частичное выделение швов на патче с дырами (фасад с окнами): ползущие стены (`wall_creep`) закрывают
скелет, но грань источника обходит дыру кругом — фронт единственного быстрого источника огибает окно и встречает сам
себя за ним. Грань источника тогда не односвязна, и сборка граней отказывает (`FACE_CHAIN_DOES_NOT_CLOSE`: участник в
трёх и более точках) либо скелет не разрешает компоненту (`SUPERLEVEL_COMPONENT_UNRESOLVABLE`). Измерено на
`building`/`building.005` (одиночные рёбра): каждая такая дыра ломает домен сама по себе, а все 65 отказавших
регионов закрываются, когда дыры соединены с контуром.

ЗАКОН. Для каждой дыры: якорь — ближайшая к дыре точка опорных отрезков источников; из вершины дыры, самой далёкой
от якоря, луч идёт ПРОЧЬ от якоря (тень дыры: туда фронт приходит последним, там он встретил бы сам себя) до первого
ребра петли (внешней или другой дыры). Вдоль луча вырезается канал полуширины `HOLE_CHANNEL_HALF_WIDTH_CELLS` клеток
решётки, и дыра вливается в петлю, в которую попал луч; дыра, влитая в дыру, получает свой канал следующей. Рёбра
канала — стены (ползут и выбрасываются из разбиения, как все стены). Луч, задевший ребро источника или вершину веера,
и канал, пересекающий чужое ребро, не ставятся: пробуется следующая вершина; дыра без канала считается под именем.

ЦЕНА. Канал меняет ответ только там, куда декаль доходит за тенью дыры: там вместо слияния фронтов за окном — щель
шириной канала (несколько клеток решётки). Наименьшее расстояние от источника до начала канала записано
(`..._MIN_DISTANCE_MICRO`, микроединицы карты): декаль шире него может нести щель. Все швы выделены — стен нет, и
ветка не вызывается (ответ побайтно прежний).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction

#: Полуширина канала дыры, в клетках решётки домена (без решётки — в долях `2^-16` единицы карты).
HOLE_CHANNEL_HALF_WIDTH_CELLS = 8

Point = tuple[Fraction, Fraction]


@dataclass(frozen=True, slots=True)
class HoleChannelsV1:
    loops: tuple[tuple[Point, ...], ...]
    channel_count: int
    refused_count: int
    min_distance_micro: int


def _orient(a, b, c) -> int:
    value = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return (value > 0) - (value < 0)


def _crosses(p1, p2, q1, q2) -> bool:
    """Собственное пересечение (касание концом не считается)."""

    return _orient(p1, p2, q1) * _orient(p1, p2, q2) < 0 and _orient(q1, q2, p1) * _orient(q1, q2, p2) < 0


def _area2(loop) -> Fraction:
    return sum(
        (loop[i][0] * loop[(i + 1) % len(loop)][1] - loop[i][1] * loop[(i + 1) % len(loop)][0] for i in range(len(loop))),
        Fraction(0),
    )


def _ray_hit(v, u, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = u[0] * dy - u[1] * dx
    if den == 0:
        return None
    wx, wy = a[0] - v[0], a[1] - v[1]
    t = (wx * dy - wy * dx) / den
    s = (wx * u[1] - wy * u[0]) / den
    return None if t <= 0 or s < 0 or s > 1 else (t, s)


def _toward(p, q, length: float) -> Point:
    """Точка отрезка `p -> q` на расстоянии около `length` от `p`; рациональная. Отрезок короче `3 * length` — сам
    конец `q`: точка ближе пары клеток к соседней вершине схлопнулась бы привязкой к решётке."""

    d = math.hypot(float(q[0] - p[0]), float(q[1] - p[1]))
    if d <= 3 * length:
        return q
    s = Fraction(length / d).limit_denominator(1 << 24)
    return (p[0] + (q[0] - p[0]) * s, p[1] + (q[1] - p[1]) * s)


def _nearest_on(point, span) -> Point:
    (ax, ay), (bx, by) = span
    dx, dy = bx - ax, by - ay
    length = dx * dx + dy * dy
    s = Fraction(0) if not length else min(Fraction(1), max(Fraction(0), ((point[0] - ax) * dx + (point[1] - ay) * dy) / length))
    return (ax + dx * s, ay + dy * s)


def _square(a, b) -> Fraction:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def _on_span(a, b, span) -> bool:
    p, q = span
    for x in (a, b):
        if _orient(p, q, x) != 0:
            return False
        t = (x[0] - p[0]) * (q[0] - p[0]) + (x[1] - p[1]) * (q[1] - p[1])
        if t < 0 or t > _square(p, q):
            return False
    return True


def _edges(loops):
    for which, loop in enumerate(loops):
        for i in range(len(loop)):
            yield which, i, loop[i], loop[(i + 1) % len(loop)]


def _splice(target, i, hole, k, x, width):
    """Петля `target`, в ребро `i` которой влита `hole` через канал от её вершины `k` к точке `x` ребра."""

    a, b = target[i], target[(i + 1) % len(target)]
    if (_area2(hole) > 0) == (_area2(target) > 0):  # влитая дыра идёт навстречу своей новой петле
        hole = hole[::-1]
        k = len(hole) - 1 - k
    v = hole[k]
    x1, x2 = _toward(x, a, width), _toward(x, b, width)
    walk = [_toward(v, hole[(k + 1) % len(hole)], width)]
    walk += [hole[(k + 1 + j) % len(hole)] for j in range(len(hole) - 1)]
    walk.append(_toward(v, hole[k - 1], width))
    if _crosses(x1, walk[0], walk[-1], x2) or x1 == x2 or walk[0] == walk[-1]:
        return None
    loop = target[: i + 1] + [x1] + walk + [x2] + target[i + 1 :]
    loop = [p for index, p in enumerate(loop) if p != loop[index - 1]]  # конец, совпавший с вершиной, не дублируется
    return loop, (x1, walk[0], walk[-1], x2)


def _channel_one(loops, h, spans, avoid, width):
    """`(новые петли, расстояние источника до канала)` либо `None`: дыра `h` не получила канала."""

    hole = loops[h]
    if any(_on_span(hole[j], hole[(j + 1) % len(hole)], span) for j in range(len(hole)) for span in spans):
        return None  # дыра несёт источник: её контур — фронт, а не стена, и резать его нельзя
    anchor = min((_nearest_on(p, span) for span in spans for p in hole), key=lambda q: min(_square(q, p) for p in hole))
    order = sorted(range(len(hole)), key=lambda j: _square(hole[j], anchor), reverse=True)
    for k in order:
        v = hole[k]
        if v in avoid:
            continue
        u = (v[0] - anchor[0], v[1] - anchor[1])
        hits = [(hit, which, i, a, b) for which, i, a, b in _edges(loops) if which != h for hit in (_ray_hit(v, u, a, b),) if hit]
        if not hits:
            continue
        (t, _s), which, i, a, b = min(hits, key=lambda item: item[0][0])
        if any(_on_span(a, b, span) for span in spans):
            continue
        x = (v[0] + u[0] * t, v[1] + u[1] * t)
        spliced = _splice(list(loops[which]), i, list(hole), k, x, width)
        if spliced is None:
            continue
        loop, (x1, h1, h2, x2) = spliced
        others = [(p, q) for w, _j, p, q in _edges(loops) if w not in (h, which) or (w == which and _j != i)]
        if any(_crosses(x1, h1, p, q) or _crosses(h2, x2, p, q) for p, q in others):
            continue
        rebuilt = list(loops)
        rebuilt[which] = tuple(loop)
        del rebuilt[h]  # h >= 1: внешняя петля остаётся первой
        return tuple(rebuilt), math.sqrt(float(_square(v, anchor)))
    return None


def needs_hole_channels(prepared) -> bool:
    """Третий проход домена: ползущие стены не закрыли домен с дырами — ещё раз, с каналами дыр.

    Каналы идут ПОСЛЕДНИМИ: домен, который закрылся неподвижными или ползущими стенами, остаётся прежним побайтно.
    """

    from .wall_creep import needs_creeping_walls

    report = None if prepared is None else prepared.bridge
    if report is None or report.polygon is None or not report.creeping_wall_count or report.hole_channel_count:
        return False
    if len(report.polygon.loops) < 2:
        return False
    # тот же разбор исхода, что у второго прохода, но для домена, где стены уже ползут
    return needs_creeping_walls(prepared, creeping_done=False)


def channel_holes(loops, laws, fan_points, lattice) -> HoleChannelsV1 | None:
    """Петли с дырами, влитыми в контур каналами (`None`: дыр нет или у законов нет опорных отрезков)."""

    spans = [law.span for law in laws if getattr(law, "span", None) is not None]
    if len(loops) < 2 or not spans or len(spans) != len(laws):
        return None
    width = HOLE_CHANNEL_HALF_WIDTH_CELLS / (lattice.scale if lattice is not None else 1 << 16)
    current = tuple(tuple(loop) for loop in loops)
    channels = refused = 0
    nearest = math.inf
    skipped: set = set()
    while True:
        candidates = [j for j in range(1, len(current)) if current[j] not in skipped]
        if not candidates:
            break
        # дыра, чья дальняя вершина дальше всех от источников, — первой
        h = max(candidates, key=lambda j: max(min(_square(p, _nearest_on(p, s)) for s in spans) for p in current[j]))
        made = _channel_one(current, h, spans, fan_points, width)
        if made is None:  # дыра остаётся дырой (без канала), и это названо
            refused += 1
            skipped.add(current[h])
            continue
        current, distance = made
        channels += 1
        nearest = min(nearest, distance)
    return HoleChannelsV1(
        loops=current,
        channel_count=channels,
        refused_count=refused,
        min_distance_micro=0 if math.isinf(nearest) else int(round(nearest * 1e6)),
    )
