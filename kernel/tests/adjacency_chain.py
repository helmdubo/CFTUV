"""ПРАВИЛО СМЕЖНОСТИ: измеренный кандидат на замену порядка сборки граней.

Живёт в стенде, а не в ядре, и это осознанное место, а не черновик. Правило
измерено полностью, оно чинит и полевой патч `bf6`, и весь остаток креста, — но
ПОСТАВИТЬ его нельзя, потому что на трёх фигурах корпуса частичного источника
оно превращает верный ГРОМКИЙ ОТКАЗ в число, которое противоречит независимому
митрованному эталону. Числа противоречия и его корень записаны в
`test_wavefront_faces.py` и в `DECISIONS.md`; здесь — сама реализация, чтобы
измерение было воспроизводимым, а не пересказанным.

ЧТО ЗА ПРАВИЛО. Дальняя граница грани ребра `e` состоит из ДУГ, и каждая дуга
есть след вершины фронта между `e` и каким-то одним другим ребром `f`. Значит
две точки дальней границы соседние тогда и только тогда, когда у них есть ОБЩИЙ
второй участник: именно эта вершина между ними и ехала. Концы цепочки заданы
входом — точка с участником `succ(e)` замыкает конец у `e.end`, точка с
`pred(e)` — конец у `e.start`.

ЧЕМ ОНО ОТЛИЧАЕТСЯ ОТ ПОСТАВЛЕННОГО. Поставленное (`faces.order_along_edge`)
сортирует узлы по убыванию проекции на опорное ребро и опирается на монотонность
грани — теорему straight skeleton с ГИПОТЕЗОЙ «каждое ребро движется». Частичный
источник её нарушает: дальняя граница грани идёт по НЕПОДВИЖНЫМ стенам, то есть
буквально по границе входа, а та немонотонна ровно настолько, насколько
немонотонен вход. Правило смежности гипотезы «всё движется» не содержит вовсе и
не спрашивает ни одного знака `SqrtSumV1` — порядок здесь чисто комбинаторный.

УЗЛЫ ГРУППИРУЮТСЯ ПО ТОЧКЕ, и это не оптимизация. Несколько событий в одной
точке — обычное дело в вырожденной конфигурации (у креста до пяти записей в
одной точке, у `ell` с двумя стенами шесть участников в одном узле), а вершина
контура там ОДНА. Смежность определена между точками; узлы внутри точки
выдаются подряд, и их взаимный порядок на площадь не влияет, потому что сегмент
между ними нулевой длины.
"""

from __future__ import annotations

from cftuv_envelope.wavefront.faces import (
    EdgeKey,
    doubled_shoelace,
    edge_key,
    polygon_edges,
)
from cftuv_envelope.wavefront.polygon import PolygonV1
from cftuv_envelope.wavefront.skeleton import SkeletonNodeV1, SkeletonV1
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1


def edge_neighbours(
    polygon: PolygonV1,
) -> dict[EdgeKey, tuple[EdgeKey, EdgeKey]]:
    """Для каждого ребра — сосед НАЗАД и сосед ВПЕРЁД по его собственной петле.

    Нужны затем, что цепочка грани имеет два объявленных конца, и оба заданы
    входом, а не найдены перебором. Искать концы по геометрии значило бы гадать
    там, где вход отвечает точно.
    """

    neighbours: dict[EdgeKey, tuple[EdgeKey, EdgeKey]] = {}
    for loop in polygon.loops:
        points = loop.points
        size = len(points)
        spans = [
            edge_key(points[index], points[(index + 1) % size])
            for index in range(size)
        ]
        for index, span in enumerate(spans):
            neighbours[span] = (
                spans[(index - 1) % size],
                spans[(index + 1) % size],
            )
    return neighbours


def face_chain(
    owner: EdgeKey,
    nodes: tuple[SkeletonNodeV1, ...],
    previous: EdgeKey,
    following: EdgeKey,
) -> tuple[tuple[SkeletonNodeV1, ...] | None, str]:
    """Узлы грани в порядке смежности либо `None` и ПРИЧИНА, почему не вышло.

    Причина возвращается строкой, а не проглатывается: «цепочка не сложилась»
    без числа не отличает развилку от разрыва. На обоих корпусах и на всей сетке
    крестов (3106 граней) она не понадобилась ни разу.
    """

    places: dict[tuple, list[SkeletonNodeV1]] = {}
    for node in nodes:
        places.setdefault(
            (node.point.x.terms, node.point.y.terms), []
        ).append(node)
    order_of_place = list(places)
    partners = [
        {
            participant
            for node in places[place]
            for participant in node.participants
            if participant != owner
        }
        for place in order_of_place
    ]

    shared: dict[EdgeKey, list[int]] = {}
    for index, group in enumerate(partners):
        for participant in group:
            shared.setdefault(participant, []).append(index)
    crowded = sorted(f for f, seats in shared.items() if len(seats) > 2)
    if crowded:
        return None, f"участник в трёх и более точках: {crowded[:2]}"

    links: dict[int, set[int]] = {index: set() for index in range(len(places))}
    for seats in shared.values():
        if len(seats) == 2:
            first, second = seats
            links[first].add(second)
            links[second].add(first)

    heads = [i for i, group in enumerate(partners) if following in group]
    tails = [i for i, group in enumerate(partners) if previous in group]
    if len(heads) != 1 or len(tails) != 1:
        return None, f"концов цепочки {len(heads)} и {len(tails)}, нужно по 1"

    walk = [heads[0]]
    seen = {heads[0]}
    while True:
        ahead = sorted(links[walk[-1]] - seen)
        if not ahead:
            break
        if len(ahead) > 1:
            return None, f"развилка после {len(walk)} точек из {len(places)}"
        walk.append(ahead[0])
        seen.add(ahead[0])
    if len(walk) != len(places):
        return None, f"цепочка {len(walk)} точек из {len(places)}"
    if walk[-1] != tails[0]:
        return None, "цепочка кончилась не у предшествующего ребра"
    return tuple(
        node for index in walk for node in places[order_of_place[index]]
    ), ""


def chained_contours(
    polygon: PolygonV1, skeleton: SkeletonV1
) -> tuple[dict[EdgeKey, tuple[tuple[SqrtSumV1, SqrtSumV1], ...]], str]:
    """Контуры всех граней по правилу смежности; вторым — причина отказа.

    Собирается тем же способом, что и в `faces.build_faces`: стены пропускаются
    (они не заметают площади), контур замыкается опорным ребром, и точки идут
    в том же представлении, чтобы сравнение с поставленным правилом было
    поточечным, а не «на глаз».
    """

    nodes_by_key: dict[EdgeKey, list[SkeletonNodeV1]] = {}
    for node in skeleton.nodes:
        for key in node.participants:
            nodes_by_key.setdefault(key, []).append(node)
    neighbours = edge_neighbours(polygon)

    contours: dict[EdgeKey, tuple[tuple[SqrtSumV1, SqrtSumV1], ...]] = {}
    for start, end, line in polygon_edges(polygon):
        if line.is_stationary:
            continue
        owner = edge_key(start, end)
        previous, following = neighbours[owner]
        chain, why = face_chain(
            owner, tuple(nodes_by_key.get(owner, ())), previous, following
        )
        if chain is None:
            return {}, f"ребро {start} -> {end}: {why}"
        contours[owner] = (
            (SqrtSumV1.rational(start[0]), SqrtSumV1.rational(start[1])),
            (SqrtSumV1.rational(end[0]), SqrtSumV1.rational(end[1])),
        ) + tuple((node.point.x, node.point.y) for node in chain)
    return contours, ""


def chained_doubled_area(
    polygon: PolygonV1, skeleton: SkeletonV1
) -> SqrtSumV1 | None:
    """Сумма удвоенных площадей граней, собранных по смежности."""

    contours, why = chained_contours(polygon, skeleton)
    if why:
        return None
    total = SqrtSumV1.zero()
    for points in contours.values():
        total = total + doubled_shoelace(points)
    return total
