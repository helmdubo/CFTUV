"""Копия граней бевела декали Both (BEVEL_FACES_BOTH_V3). Без Blender: чистый Python и numpy.

ЧТО ЭТО. Выделенные грани бевела (полосы и узлы) не заливаются ядром изнутри: декаль Both кладёт над ними их КОПИЮ - те же вершины, грани
и петли, что у оригинала, смещённые над поверхностью, - а ядро растит декаль только НАРУЖУ (нажатие Outward: грани вне выделения, источники -
рёбра, которые оставляет выделение). Копия от ширины не зависит; меняется только внешняя часть.

ОДИН СВАРЕННЫЙ МЕШ (BEVEL_COPY_WELD_V2). Копия - ещё один домен сборки меша (`copy_result`): вид домена с вершинами копии, у каждой ссылка
вершины исходника, которой её называет ядро (`LOCATION_FORMAT`), и позиция исходника (те же числа, что у снапшота анализа: `float` от `co`
вершины bmesh), поэтому сварка хоста (`envelope_production_weld`) сама сводит вершины рельса копии с вершинами рельса внешней декали в ОДНУ
вершину, а смещение этой вершины - митра нормалей обеих частей. Внутренние вершины копии смещены вдоль своей нормали (BEVEL_COPY_OFFSET_V2:
сумма векторов Ньюэлла её граней копии, то есть нормалей, взвешенных площадью).

T-СТЫКИ РЕЛЬСА (BEVEL_COPY_RAIL_SPLIT_V3, `split_rails`). Ядро держит на рельсе только вершины силуэта: над прямой колонкой торца юбка - одна
грань на всю высоту, а у рельса копии вершины на каждом уровне; без пары они смещались вдоль своей нормали, ребро юбки - прямо, щель. После
сварки такие вершины встают в ребро грани юбки (n-угольник, вершины на ребре по порядку рельса, UV петли - по доле, как у `WALL_EDGE_SPLIT_V1`:
модель превью ширины видит их делениями стены домена), смещение - митра нормали копии и нормали грани юбки. Где внешняя часть отказана,
вершина рельса остаётся своей: число `BEVEL_COPY_RAIL_UNPAIRED` в строке статуса (`rail_unpaired`) - не тихий разрыв; деление и его отказ -
`BEVEL_COPY_RAIL_SPLIT` и `BEVEL_COPY_RAIL_SPLIT_REFUSED` там же.

UV КОПИИ - СЕГМЕНТЫ (метры, их отображает в UV трима `bevel_wings`). Сегмент - кусок полосы (компонента граней полосы по рёбрам) по колонкам
профиля: путь по квадам полосы от ребра рельса до ребра рельса по ту сторону (`_walk`), и в нём две перекладины (цепи вершин поперёк полосы).
Стороны рельсов красятся по перекладинам (концы перекладины - разные стороны, вершины одного ребра рельса - одна). Straighten по одной оси
(BEVEL_COPY_RUNG_ONE_AXIS_V4): шаг перекладины делится на часть вдоль касательной средней линии и часть поперёк; вершина на дуге поперёк `d`
от стороны `+` (вся дуга поперёк - `W`) получает `S = W / 2 - d`, а `U` - длину дуги средней линии (ломаной через точки перекладин на половине
`W`) от первой перекладины плюс свой сдвиг вдоль касательной от точки средней линии. Перекладина поперёк полосы - одна `U` (зеркальные точки
рельсов - одна `U`), косая (митра у узла) - косая и в UV, как на меше: в прямую выпрямляются только ряды вдоль. У замкнутого кольца последняя
колонка несёт `U` полного оборота - шов UV. Одна система координат на сегмент, ничего не согласуется между сегментами.
Направление (BEVEL_SEGMENT_U_ALONG_WORLD_AXIS_V3): грани сегмента в UV против часовой, `U` растёт к плюсу мировой оси, вдоль которой сегмент
тянется больше всего (X, затем Y, затем Z при равенстве), `U = 0` - первая перекладина.

УЗЕЛ - К ОДНОМУ СЕГМЕНТУ (BEVEL_NODE_ATTACH_MOST_EDGES_LONGEST_V3). Компонента граней вне колонок (узел и грани полосы вне колонок рядом)
целиком берёт сегмент, с которым у неё больше всего общих рёбер; ничья - более длинный сегмент, затем меньший номер; другие сегменты у этого
узла получают шов UV. Вершина узла на гранях своего сегмента берёт его UV точно; остальные грани узла разворачиваются слоями от известных
вершин (у грани их две и больше): подобие без отражения из плоскости грани в UV, новая вершина берёт UV первой грани, что её положила, - `U`
сегмента продолжается за конец полосы, грань узла не выворачивается. Узел без полосы рядом - своя плоская проекция (`BEVEL_COPY_NODE_NO_STRIP`).
Владелец грани копии - `OWNER_SHIFT * сегмент + OWNER_NODE` (BEVEL_COPY_OWNER_V3: бит узла читают крылья). Каждое отступление - число с
именем (правило 4).
"""

from __future__ import annotations

import hashlib
import math
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np

BEVEL_COPY_COLUMN_OPEN = "BEVEL_COPY_COLUMN_OPEN"
BEVEL_COPY_SIDES_CONFLICT = "BEVEL_COPY_SIDES_CONFLICT"
BEVEL_COPY_COLUMNS_BRANCH = "BEVEL_COPY_COLUMNS_BRANCH"
BEVEL_COPY_RING = "BEVEL_COPY_RING"
BEVEL_COPY_STRIP_FACE_OFF_COLUMN = "BEVEL_COPY_STRIP_FACE_OFF_COLUMN"
BEVEL_COPY_NODE_CONTINUES_STRIP = "BEVEL_COPY_NODE_CONTINUES_STRIP"
BEVEL_COPY_NODE_NO_STRIP = "BEVEL_COPY_NODE_NO_STRIP"
BEVEL_COPY_NODE_FIT_DEGENERATE = "BEVEL_COPY_NODE_FIT_DEGENERATE"
BEVEL_COPY_NORMAL_DEGENERATE = "BEVEL_COPY_NORMAL_DEGENERATE"
BEVEL_COPY_UV_FLIPPED_FACES = "BEVEL_COPY_UV_FLIPPED_FACES"
BEVEL_COPY_RAIL_UNPAIRED = "BEVEL_COPY_RAIL_UNPAIRED"
BEVEL_COPY_SEGMENTS = "BEVEL_COPY_SEGMENTS"
BEVEL_COPY_RAIL_SPLIT = "BEVEL_COPY_RAIL_SPLIT"
BEVEL_COPY_RAIL_SPLIT_REFUSED = "BEVEL_COPY_RAIL_SPLIT_REFUSED"
BEVEL_COPY_RAIL_SPLIT_MITER_FALLBACK = "BEVEL_COPY_RAIL_SPLIT_MITER_FALLBACK"

#: Ссылка вершины копии: та же, что у вершины исходника в батчах ядра - `location:` (`mesh_weld.WELD_LOCATION_PREFIX` без `src:`), ключ вершины
#: батча `src:<SourceVertexId>` и идентичность вершины хоста `host-vertex:host-source:<отпечаток>:<имя>:<номер>` (`envelope_request_export._source_ids`).
#: Разойдётся формат - сварка не найдёт пар, и это число `BEVEL_COPY_RAIL_UNPAIRED` (тест Blender требует нуль).
LOCATION_FORMAT = "location:src:host-vertex:host-source:{digest}:{name}:{vertex}"
#: Значения UV двух граней копии в общей вершине, разошедшиеся больше этого (метры), - шов копии.
SEAM_UV_TOLERANCE = 1e-12
#: Владелец грани копии (BEVEL_COPY_OWNER_V3): `OWNER_SHIFT * сегмент + OWNER_NODE`, бит узла - грань вне колонок полосы (узел, грань полосы вне
#: колонок и плоская проекция узла без полосы); его читают крылья (`bevel_wings`): веер ядра у вершины узла - сегмент узла.
OWNER_SHIFT = 2
OWNER_NODE = 1
#: Сколько нажатий Both помнит реестр копий (`note_copy`): живая ширина читает копию своего нажатия.
REGISTRY_SIZE = 8


@dataclass(frozen=True, slots=True)
class BevelCopySpecV1:
    """Копия выделенных граней: топология оригинала (локальные номера вершин - по возрастанию номера исходника), смещение и UV в метрах.

    `faces` - кольца в порядке номеров граней исходника (обход оригинала); `uvs` - `(U, S)` каждого угла по граням подряд; `owners` - кадр грани
    (номер полосы); `seams` - пары локальных вершин, где UV двух граней копии расходится; `boundary` - вершины, которые копия делит с
    невыделенными гранями; `events` - числа решений по именам.
    """

    vertex_ids: tuple
    positions: tuple
    normals: tuple
    faces: tuple
    face_ids: tuple
    owners: tuple
    uvs: tuple
    seams: tuple
    boundary: frozenset
    events: tuple = ()

    @property
    def digest(self) -> str:
        text = repr((self.vertex_ids, self.positions, self.faces, self.uvs, self.owners))
        return hashlib.blake2b(text.encode("utf-8"), digest_size=10).hexdigest()


def _edges(cycle):
    count = len(cycle)
    return [frozenset((cycle[index], cycle[(index + 1) % count])) for index in range(count)]


def _newell(coords, cycle):
    nx = ny = nz = 0.0
    count = len(cycle)
    for index in range(count):
        a = coords[cycle[index]]
        b = coords[cycle[(index + 1) % count]]
        nx += (a[1] - b[1]) * (a[2] + b[2])
        ny += (a[2] - b[2]) * (a[0] + b[0])
        nz += (a[0] - b[0]) * (a[1] + b[1])
    return nx, ny, nz


def _count(events: dict, name: str, amount: int = 1) -> None:
    if amount:
        events[name] = events.get(name, 0) + int(amount)


def _components(faces: set, table: dict, rings: dict) -> list:
    """Компоненты граней `faces` по общим рёбрам (только внутри `faces`), по возрастанию меньшего номера."""

    seen: set = set()
    found = []
    for start in sorted(faces):
        if start in seen:
            continue
        seen.add(start)
        stack, part = [start], []
        while stack:
            face = stack.pop()
            part.append(face)
            for key in _edges(rings[face]):
                for other in table.get(key, ()):
                    if other in faces and other not in seen:
                        seen.add(other)
                        stack.append(other)
        found.append(frozenset(part))
    return found


def _next_pair(cycle, x, y):
    """Квад `cycle`, вход по ребру `(x, y)`: вершины противоположного ребра, смежные с `x` и с `y`."""

    count = len(cycle)
    ix, iy = cycle.index(x), cycle.index(y)
    nx = cycle[(ix - 1) % count] if cycle[(ix + 1) % count] == y else cycle[(ix + 1) % count]
    ny = cycle[(iy - 1) % count] if cycle[(iy + 1) % count] == x else cycle[(iy + 1) % count]
    return nx, ny


def _walk(rings, table, component, selected, key, face):
    """Колонка профиля от ребра рельса `key` грани `face`: `(грани, перекладина от x, перекладина от y)` либо `None` (не квад, петля, конец не на рельсе)."""

    x, y = sorted(key)
    chain_x, chain_y, path = [x], [y], []
    while True:
        cycle = rings[face]
        if len(cycle) != 4 or face in path:
            return None
        path.append(face)
        x, y = _next_pair(cycle, x, y)
        chain_x.append(x)
        chain_y.append(y)
        exit_key = frozenset((x, y))
        others = [item for item in table.get(exit_key, ()) if item != face]
        if len(others) != 1:
            return None
        beyond = others[0]
        if beyond not in selected:
            return tuple(path), tuple(chain_x), tuple(chain_y)
        if beyond not in component:
            return None
        face = beyond


def _colour(rail_edges, chains, events) -> dict:
    """Цвет вершин рельса: вершины ребра рельса - один цвет, концы перекладины - разные; ничья - цвет 0 у меньшего номера компоненты."""

    links: dict = {}
    for a, b in rail_edges:
        links.setdefault(a, []).append((b, 0))
        links.setdefault(b, []).append((a, 0))
    for chain in chains:
        links.setdefault(chain[0], []).append((chain[-1], 1))
        links.setdefault(chain[-1], []).append((chain[0], 1))
    colour: dict = {}
    conflicts = 0
    for root in sorted(links):
        if root in colour:
            continue
        colour[root] = 0
        stack = [root]
        while stack:
            vertex = stack.pop()
            for other, flip in links[vertex]:
                wanted = colour[vertex] ^ flip
                if other not in colour:
                    colour[other] = wanted
                    stack.append(other)
                elif colour[other] != wanted:
                    conflicts += 1
    _count(events, BEVEL_COPY_SIDES_CONFLICT, conflicts // 2)
    return colour


def _strip_frames(coords, rings, table, component, selected, events):
    """Кадры колонок одной полосы: список `(грани кадра, {(грань, вершина): (U, S)})` по кускам перекладин (путь или кольцо)."""

    rails = []
    for face in sorted(component):
        for key in _edges(rings[face]):
            others = [item for item in table.get(key, ()) if item != face]
            if len(others) == 1 and others[0] not in selected:
                rails.append((key, face))
    columns: dict = {}
    for key, face in rails:
        walked = _walk(rings, table, component, selected, key, face)
        if walked is None:
            _count(events, BEVEL_COPY_COLUMN_OPEN)
            continue
        path, chain_x, chain_y = walked
        columns.setdefault(frozenset(path), (path, chain_x, chain_y))
    if not columns:
        return []
    chains = [chain for _path, chain_x, chain_y in columns.values() for chain in (chain_x, chain_y)]
    colour = _colour([tuple(key) for key, _face in rails], chains, events)
    oriented: dict = {}
    graph: dict = {}
    column_of: dict = {}
    for path, chain_x, chain_y in columns.values():
        ends = []
        for chain in (chain_x, chain_y):
            chain = chain if colour.get(chain[0], 0) == 0 else tuple(reversed(chain))
            name = (chain[0], chain[-1])
            oriented[name] = chain
            ends.append(name)
        a, b = ends
        if a == b:
            continue
        graph.setdefault(a, set()).add(b)
        graph.setdefault(b, set()).add(a)
        column_of[frozenset((a, b))] = path
    if any(len(item) > 2 for item in graph.values()):
        _count(events, BEVEL_COPY_COLUMNS_BRANCH)
        return []
    frames = []
    seen: set = set()
    for start in sorted(graph, key=lambda name: (len(graph[name]) != 1, min(name))):
        if start in seen:
            continue
        order = [start]
        seen.add(start)
        while True:
            following = [item for item in sorted(graph[order[-1]]) if item not in seen]
            if not following:
                break
            order.append(following[0])
            seen.add(following[0])
        ring = len(order) > 2 and order[0] in graph[order[-1]]
        if ring:
            _count(events, BEVEL_COPY_RING)
        frames.append(_frame(coords, rings, oriented, column_of, order, ring))
    return frames


def _unit(vector):
    length = float(np.linalg.norm(vector))
    return None if length <= 0.0 else vector / length


def _tangents(centres, ring) -> list:
    """Касательная средней линии в каждой перекладине: внутри - биссектриса соседних единичных хорд; на конце пути - хорда, отражённая от
    касательной соседа (точно для окружности, для прямой - сама хорда); у одной перекладины касательной нет (`None`)."""

    count = centres.shape[0]
    if count < 2:
        return [None] * count
    chords = [_unit(centres[(at + 1) % count] - centres[at]) for at in range(count if ring else count - 1)]
    tangents = []
    for at in range(count):
        before = chords[at - 1] if (ring or at > 0) else None
        after = chords[at] if (ring or at < count - 1) else None
        if before is None or after is None:
            tangents.append(after if before is None else before)
            continue
        middle = _unit(before + after)
        tangents.append(after if middle is None else middle)
    if not ring and count > 2:
        for end, chord, near in ((0, chords[0], tangents[1]), (count - 1, chords[-1], tangents[-2])):
            if chord is not None and near is not None:
                tangents[end] = _unit(2.0 * float(near @ chord) * chord - near)
    return tangents


def _rung(coords, chain, tangent):
    """Перекладина в кадре касательной средней линии (BEVEL_COPY_RUNG_ONE_AXIS_V4): шаг перекладины делится на часть вдоль касательной
    (сдвиг `U`) и часть поперёк (её длина - дуга поперёк, `S`). `(вдоль от начала, поперёк от начала, точка средней линии, вдоль в ней)`:
    точка средней линии - на половине дуги поперёк. Без касательной - вдоль нуль, поперёк - длина дуги (перекладина поперёк полосы)."""

    points = np.asarray([coords[vertex] for vertex in chain], dtype=np.float64)
    steps = np.diff(points, axis=0)
    if tangent is None:
        along = np.zeros(steps.shape[0])
        cross = np.linalg.norm(steps, axis=1)
    else:
        along = steps @ tangent
        cross = np.linalg.norm(steps - along[:, None] * tangent[None, :], axis=1)
    a, c = np.r_[0.0, np.cumsum(along)], np.r_[0.0, np.cumsum(cross)]
    half = 0.5 * float(c[-1])
    at = min(max(int(np.searchsorted(c, half, side="left")), 1), len(chain) - 1)
    span = float(c[at] - c[at - 1])
    t = 0.0 if span <= 0.0 else (half - float(c[at - 1])) / span
    return a, c, points[at - 1] + t * (points[at] - points[at - 1]), float(a[at - 1] + t * (a[at] - a[at - 1]))


def _frame(coords, rings, oriented, column_of, order, ring):
    """Кадр куска полосы - Straighten по одной оси (BEVEL_COPY_RUNG_ONE_AXIS_V4): поперёк `S = W / 2 - d`, где `d` - дуга перекладины
    ПОПЕРЁК касательной средней линии от стороны `+`, `W` - вся такая дуга; `U` вершины - длина дуги средней линии (ломаная через точки
    перекладин на половине `W`) от первой перекладины плюс сдвиг вершины вдоль касательной от точки средней линии. Перекладина поперёк
    полосы - одна `U` (зеркальные точки рельсов - одна `U`); косая (митра у узла, наклон профиля) - косая и в UV, как на меше: в линию
    выпрямляются только ряды вдоль, перекладины - нет."""

    chains = [oriented[name] for name in order]
    first = np.asarray([_rung(coords, chain, None)[2] for chain in chains])
    rungs = [_rung(coords, chain, tangent) for chain, tangent in zip(chains, _tangents(first, ring))]
    centres = [rung[2] for rung in rungs]
    along = [0.0]
    for a, b in zip(centres, centres[1:]):
        along.append(along[-1] + float(np.linalg.norm(b - a)))
    total = along[-1] + (float(np.linalg.norm(centres[0] - centres[-1])) if ring else 0.0)

    def values(number, u):
        shift, cross, _centre, middle = rungs[number]
        width = float(cross[-1])
        return {vertex: (u + float(shift[at]) - middle, 0.5 * width - float(cross[at])) for at, vertex in enumerate(chains[number])}

    by_chain = [values(number, u) for number, u in enumerate(along)]
    corner: dict = {}
    faces: set = set()
    pairs = list(zip(range(len(order)), range(1, len(order))))
    if ring:
        pairs.append((len(order) - 1, len(order)))
        by_chain.append(values(0, total))
    for i, j in pairs:
        name_j = order[j % len(order)]
        path = column_of[frozenset((order[i], name_j))]
        merged = {**by_chain[i], **by_chain[j]}
        for face in path:
            faces.add(face)
            for vertex in rings[face]:
                corner[(face, vertex)] = merged[vertex]
    return frozenset(faces), corner


def _signed_area(points) -> float:
    total = 0.0
    for index in range(len(points)):
        a, b = points[index], points[(index + 1) % len(points)]
        total += a[0] * b[1] - b[0] * a[1]
    return 0.5 * total


def _orient(frames, rings, coords, events) -> list:
    """Направление и начало `U` каждого сегмента (BEVEL_SEGMENT_U_ALONG_WORLD_AXIS_V3): грани сегмента в UV против часовой (сумма площадей,
    иначе отражение `U -> -U`); затем `U` растёт к плюсу той мировой оси, вдоль которой сегмент тянется больше всего (ось - наибольшая по модулю
    составляющая градиента `U` по вершинам сегмента, `sum (U - mean U) (p - mean p)`; равенство - меньшая ось X, Y, Z), иначе поворот на 180
    градусов (`U, S -> -U, -S`: ориентация та же); начало `U = 0` - первая перекладина. После - вывернутые грани числом. Длины сегментов по `U`."""

    flipped = 0
    lengths = []
    for faces, corner in frames:
        total = sum(_signed_area([corner[(face, vertex)] for vertex in rings[face]]) for face in faces)
        if total < 0.0:
            for slot, (u, s) in list(corner.items()):
                corner[slot] = (-u, s)
        slots = sorted(corner)
        along = np.asarray([corner[slot][0] for slot in slots], dtype=np.float64)
        points = np.asarray([coords[slot[1]] for slot in slots], dtype=np.float64)
        gradient = (along - along.mean()) @ (points - points.mean(axis=0))
        axis = int(np.argmax(np.abs(gradient)))
        sign = -1.0 if float(gradient[axis]) < 0.0 else 1.0
        low = min(sign * u for u, _s in corner.values())
        high = max(sign * u for u, _s in corner.values())
        for slot, (u, s) in list(corner.items()):
            corner[slot] = (sign * u - low, sign * s)
        lengths.append(high - low)
        flipped += sum(1 for face in faces if _signed_area([corner[(face, vertex)] for vertex in rings[face]]) < 0.0)
    _count(events, BEVEL_COPY_UV_FLIPPED_FACES, flipped)
    return lengths


def _attach_nodes(rest, frame_of, table, rings, lengths):
    """Узел - к ОДНОМУ сегменту (BEVEL_NODE_ATTACH_MOST_EDGES_LONGEST_V3): компонента граней вне колонок (узел вместе с гранями полосы вне
    колонок рядом) целиком берёт сегмент, с гранями которого у неё больше всего общих рёбер; ничья - более длинный сегмент (длина средней
    линии), затем меньший номер. Компонента без такого соседа берёт сегмент соседней компоненты, уже присоединённой (обход слоями).
    `([(компонента, сегмент)] в порядке присоединения, грани без сегмента)`."""

    owner = dict(frame_of)
    pending = _components(set(rest), table, rings)
    attached = []
    while pending:
        chosen = []
        for component in pending:
            votes: dict = {}
            for face in sorted(component):
                for key in _edges(rings[face]):
                    for other in table.get(key, ()):
                        if other not in component and other in owner:
                            votes[owner[other]] = votes.get(owner[other], 0) + 1
            if votes:
                chosen.append((component, min(votes, key=lambda frame: (-votes[frame], -lengths[frame], frame))))
        if not chosen:
            break
        for component, frame in chosen:
            for face in component:
                owner[face] = frame
        attached.extend(chosen)
        taken = {component for component, _frame in chosen}
        pending = [component for component in pending if component not in taken]
    return attached, {face for component in pending for face in component}


def _local(coords, ring):
    """Плоские координаты грани в её плоскости (базис по нормали Ньюэлла): обход кольца против часовой; `None` - вырожденная грань."""

    n = _newell(coords, ring)
    length = math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2])
    if length <= 0.0:
        return None
    normal = np.asarray(n, dtype=np.float64) / length
    points = np.asarray([coords[vertex] for vertex in ring], dtype=np.float64)
    first = points[1] - points[0]
    first = first - normal * float(first @ normal)
    size = float(np.linalg.norm(first))
    if size <= 0.0:
        return None
    first /= size
    second = np.cross(normal, first)
    rel = points - points[0]
    return (rel @ first) + 1j * (rel @ second)


def _similarity(source, target):
    """Подобие без отражения `z -> a z + b` (комплексные числа), наименьшие квадраты по парам точек; `None` - опоры совпали."""

    cs, ct = source.mean(), target.mean()
    ds, dt = source - cs, target - ct
    norm = float(np.sum(np.abs(ds) ** 2))
    if norm <= 0.0:
        return None
    a = complex(np.sum(dt * np.conj(ds)) / norm)
    return a, ct - a * cs


def _unfold(coords, rings, cluster, known: dict, events) -> dict:
    """Узел продолжает полосу (BEVEL_COPY_NODE_CONTINUES_STRIP_V2): грани кластера разворачиваются слоями от вершин с известным UV (две и больше
    у грани) подобием без отражения из плоскости грани; новая вершина получает UV первой грани, что её положила. `{(грань, вершина): (U, S)}`."""

    values = {vertex: complex(*uv) for vertex, uv in known.items()}
    corner: dict = {}
    pending = sorted(cluster)
    while pending:
        placed = []
        for face in pending:
            ring = rings[face]
            anchors = [at for at, vertex in enumerate(ring) if vertex in values]
            if len(anchors) < 2:
                continue
            local = _local(coords, ring)
            fitted = None if local is None else _similarity(local[anchors], np.asarray([values[ring[at]] for at in anchors]))
            if fitted is None:
                continue
            a, b = fitted
            for at, vertex in enumerate(ring):
                if vertex not in values:
                    values[vertex] = a * local[at] + b
            placed.append(face)
        if not placed:
            break
        for face in placed:
            for vertex in rings[face]:
                corner[(face, vertex)] = (values[vertex].real, values[vertex].imag)
        pending = [face for face in pending if face not in placed]
    if pending:
        _count(events, BEVEL_COPY_NODE_FIT_DEGENERATE, len(pending))
        for face in pending:
            local = _local(coords, rings[face])
            anchor = next((vertex for vertex in rings[face] if vertex in values), None)
            shift = 0j if anchor is None or local is None else values[anchor] - local[rings[face].index(anchor)]
            for at, vertex in enumerate(rings[face]):
                z = values.get(vertex, (0j if local is None else local[at]) + shift)
                corner[(face, vertex)] = (z.real, z.imag)
    return corner


def _planar(coords, faces, rings):
    """Плоская проекция грани узла без полосы рядом: две главные оси разброса вершин (метры)."""

    vertices = sorted({vertex for face in faces for vertex in rings[face]})
    points = np.asarray([coords[vertex] for vertex in vertices], dtype=np.float64)
    centre = points.mean(axis=0)
    _u, _s, axes = np.linalg.svd(points - centre, full_matrices=False)
    first, second = axes[0], axes[1] if axes.shape[0] > 1 else np.array([0.0, 0.0, 0.0])
    return {vertex: (float((np.asarray(coords[vertex]) - centre) @ first), float((np.asarray(coords[vertex]) - centre) @ second)) for vertex in vertices}


def copy_spec(view, split) -> BevelCopySpecV1:
    """Копия выделенных граней вида `view` (`bevel_faces.BevelFacesViewV1`) по разбиению `split` (полосы и узлы): топология, нормали, UV в метрах."""

    events: dict = {}
    selected = frozenset(int(face) for face in view.selected)
    rings = {int(face): tuple(int(vertex) for vertex in cycle) for face, cycle in view.faces.items()}
    coords = view.coords
    table: dict = {}
    for face in sorted(rings):
        for key in _edges(rings[face]):
            table.setdefault(key, []).append(face)
    face_ids = tuple(sorted(selected))
    vertex_ids = tuple(sorted({vertex for face in face_ids for vertex in rings[face]}))
    local = {vertex: number for number, vertex in enumerate(vertex_ids)}
    sums = {vertex: [0.0, 0.0, 0.0] for vertex in vertex_ids}
    for face in face_ids:
        n = _newell(coords, rings[face])
        for vertex in rings[face]:
            total = sums[vertex]
            total[0] += n[0]
            total[1] += n[1]
            total[2] += n[2]
    normals = []
    degenerate = 0
    for vertex in vertex_ids:
        x, y, z = sums[vertex]
        length = math.sqrt(x * x + y * y + z * z)
        if length <= 0.0:
            degenerate += 1
            normals.append((0.0, 0.0, 1.0))
        else:
            normals.append((x / length, y / length, z / length))
    _count(events, BEVEL_COPY_NORMAL_DEGENERATE, degenerate)
    boundary = frozenset(
        local[vertex] for face, cycle in rings.items() if face not in selected for vertex in cycle if vertex in local
    )
    frames = []
    strip = {int(face) for face in split.strip} & selected
    for component in _components(strip, table, rings):
        frames.extend(_strip_frames(coords, rings, table, component, selected, events))
    lengths = _orient(frames, rings, coords, events)
    corner: dict = {}
    frame_of: dict = {}
    known: list = []
    for number, (faces, values) in enumerate(frames):
        corner.update(values)
        known.append({vertex: values[(face, vertex)] for face in sorted(faces) for vertex in rings[face]})
        for face in faces:
            frame_of[face] = number
    rest = selected - set(frame_of)
    _count(events, BEVEL_COPY_STRIP_FACE_OFF_COLUMN, len(rest & strip))
    attached, lonely = _attach_nodes(rest, frame_of, table, rings, lengths)
    nodes = set(lonely)
    for cluster, frame in attached:
        touching = {vertex for face in cluster for vertex in rings[face]}
        unfolded = _unfold(coords, rings, cluster, {vertex: known[frame][vertex] for vertex in touching if vertex in known[frame]}, events)
        corner.update(unfolded)
        for (_face, vertex), value in sorted(unfolded.items()):
            known[frame].setdefault(vertex, value)
        for face in cluster:
            frame_of[face] = frame
            nodes.add(face)
    _count(events, BEVEL_COPY_NODE_CONTINUES_STRIP, sum(len(cluster) for cluster, _frame in attached))
    lonely_frames = _components(set(lonely), table, rings)
    _count(events, BEVEL_COPY_NODE_NO_STRIP, len(lonely))
    for cluster in lonely_frames:
        number = len(frames)
        frames.append((cluster, {}))
        projected = _planar(coords, cluster, rings)
        if sum(_signed_area([projected[vertex] for vertex in rings[face]]) for face in cluster) < 0.0:
            projected = {vertex: (-u, s) for vertex, (u, s) in projected.items()}
        for face in cluster:
            frame_of[face] = number
            for vertex in rings[face]:
                corner[(face, vertex)] = projected[vertex]
    _count(events, BEVEL_COPY_SEGMENTS, len(frames))
    uvs = tuple(corner[(face, vertex)] for face in face_ids for vertex in rings[face])
    seams = set()
    for key, faces in table.items():
        inside = [face for face in faces if face in selected]
        if len(inside) != 2:
            continue
        first, second = inside
        if any(
            abs(corner[(first, vertex)][0] - corner[(second, vertex)][0]) > SEAM_UV_TOLERANCE
            or abs(corner[(first, vertex)][1] - corner[(second, vertex)][1]) > SEAM_UV_TOLERANCE
            for vertex in key
        ):
            a, b = sorted(local[vertex] for vertex in key)
            seams.add((a, b))
    return BevelCopySpecV1(
        vertex_ids=vertex_ids,
        positions=tuple(tuple(float(value) for value in coords[vertex]) for vertex in vertex_ids),
        normals=tuple(normals),
        faces=tuple(tuple(local[vertex] for vertex in rings[face]) for face in face_ids),
        face_ids=face_ids,
        owners=tuple(OWNER_SHIFT * int(frame_of[face]) + (OWNER_NODE if face in nodes else 0) for face in face_ids),
        uvs=tuple((float(u), float(s)) for u, s in uvs),
        seams=tuple(sorted(seams)),
        boundary=boundary,
        events=tuple(sorted(events.items())),
    )


# --------------------------------------------------------------------------
# Домен копии в сборке меша
# --------------------------------------------------------------------------


class BevelCopyResultV1:
    """Результат домена копии для сборки меша (`envelope_production_mesh.build_mesh_arrays`): вид объектом и байтами, у ядра его нет.

    Сертификата ширины и интервала у копии нет (она от ширины не зависит): модель превью ширины держит её вершины на месте
    (`DOMAIN_NO_INTERVAL`), а составной образец такта называет откат (`COMPOSITE_HELD_NOT_RECOMPUTED`) и собирает меш путём этапа A.
    """

    outcome = "MATERIALIZED"
    detail = ""
    counters = ()
    decal_topology_law = ""
    uv_policy_id = ""
    alpha_interval = None
    parametric = None
    structure_digest = ""
    is_materialized = True

    def __init__(self, patch_id: int, domain_id: str, view, view_bytes: bytes | None, source_normal: tuple, spec: BevelCopySpecV1):
        self.patch_id = int(patch_id)
        self.domain_id = str(domain_id)
        self.view = view
        self.view_bytes = view_bytes
        self.source_normal = source_normal
        self.spec = spec


def copy_result(spec: BevelCopySpecV1, patch_id: int, source_revision) -> BevelCopyResultV1:
    """Домен копии: вид (`DomainViewV1`) с вершинами копии (ссылки `location:src:<id>`, позиции исходника, нормали BEVEL_COPY_OFFSET_V2), гранями,
    UV в метрах, кадрами граней владельцами и швами копии; байты вида (VIEW_BYTES_V1) для нативной сборки, если вид их допускает. `source_revision` -
    отпечаток пакета анализа нажатия (`AnalysisBundle.source_revision`): из него ядро называет вершины исходника, и ссылки копии - те же."""

    from .envelope_production_view import DomainViewV1, structure_token
    from .envelope_production_weld import DomainVerticesV1
    from .envelope_view_bytes import ViewBytesUnsupported, encode_view

    digest, name = str(source_revision.digest).strip(), str(source_revision.source_name).strip()
    refs = tuple(LOCATION_FORMAT.format(digest=digest, name=name, vertex=vertex) for vertex in spec.vertex_ids)
    vertices = DomainVerticesV1(int(patch_id), spec.positions, spec.normals, refs, (), spec.faces)
    view = DomainViewV1(None, vertices, spec.faces, spec.uvs, spec.owners, spec.seams, (), "")
    token = structure_token(view)
    view = DomainViewV1(None, vertices, spec.faces, spec.uvs, spec.owners, spec.seams, (), "", token)
    try:
        data = encode_view(view, token)
    except ViewBytesUnsupported:
        data = None
    normal = spec.normals[0] if spec.normals else (0.0, 0.0, 1.0)
    return BevelCopyResultV1(patch_id, f"bevel-copy:{digest[:16]}:{spec.digest}", view, data, normal, spec)


_REGISTRY: OrderedDict = OrderedDict()


def note_copy(result: BevelCopyResultV1, domain_ids) -> None:
    """Нажатие Both: домен копии `result` входит в каждую сборку меша, где есть домен из `domain_ids` (кнопка, такты живой ширины, подтверждение)."""

    _REGISTRY.pop(result.domain_id, None)
    _REGISTRY[result.domain_id] = (result, frozenset(str(item) for item in domain_ids))
    while len(_REGISTRY) > REGISTRY_SIZE:
        _REGISTRY.popitem(last=False)


def copy_for(results) -> BevelCopyResultV1 | None:
    """Домен копии сборки, в которой есть домены нажатия Both (последнее нажатие с этими доменами), либо `None`."""

    if not _REGISTRY:
        return None
    present = {str(getattr(item, "domain_id", "")) for item in results}
    for result, domain_ids in reversed(_REGISTRY.values()):
        if present & domain_ids:
            return result
    return None


def _copy_number(arrays, copy):
    keys = tuple(arrays.domain_keys)
    return next((at for at, key in enumerate(keys) if key == (copy.patch_id, copy.domain_id)), None)


def _copy_index(arrays, number) -> list:
    buffers = getattr(arrays, "buffers", None)
    return buffers.domain_index(number).tolist() if buffers is not None else list(arrays.domain_vertex_index[number])


def rail_unpaired(arrays) -> int:
    """Вершины копии на границе выделения, которых нет ни в одной грани внешней части после сварки и деления рёбер рельса
    (`BEVEL_COPY_RAIL_UNPAIRED`): так остаются вершины рельса там, где внешняя часть отказана. Копии в массивах нет - нуль."""

    copy = getattr(arrays, "bevel_copy", None)
    if copy is None:
        return 0
    number = _copy_number(arrays, copy)
    if number is None:
        return 0
    mine = _copy_index(arrays, number)
    outward = {vertex for ring, domain in zip(arrays.faces, arrays.face_domain) if domain != copy.patch_id for vertex in ring}
    return sum(1 for local in copy.spec.boundary if mine[local] not in outward)


def _rail_links(spec) -> dict:
    """Рёбра рельса копии (у ребра одна грань копии) по локальным вершинам: `{вершина: [соседи]}`."""

    counts: dict = {}
    for ring in spec.faces:
        for key in _edges(ring):
            counts[key] = counts.get(key, 0) + 1
    links: dict = {}
    for key, count in sorted(counts.items(), key=lambda item: sorted(item[0])):
        if count == 1:
            a, b = sorted(key)
            links.setdefault(a, []).append(b)
            links.setdefault(b, []).append(a)
    return links


def _rail_path(links, lonely, start, end, used):
    """Путь по рёбрам рельса копии от `start` до `end` только через вершины без пары (`lonely`, ещё не взятые): внутренние вершины по порядку
    либо `None`; из двух путей - короткий."""

    found = None
    for first in sorted(links.get(start, ())):
        if first not in lonely or first in used:
            continue
        previous, current, path = start, first, [first]
        while True:
            following = [item for item in links.get(current, ()) if item != previous]
            if len(links.get(current, ())) != 2 or not following:
                break
            beyond = following[0]
            if beyond == end:
                if found is None or len(path) < len(found):
                    found = list(path)
                break
            if beyond not in lonely or beyond in used or beyond in path:
                break
            path.append(beyond)
            previous, current = current, beyond
    return found


def rail_splits(faces, face_domain, copy_patch: int, copy_index, spec) -> tuple:
    """Деления рёбер внешней части вершинами рельса копии (BEVEL_COPY_RAIL_SPLIT_V3): `([(грань, угол, ((локальная вершина копии, доля), ...))],
    отказано)`.

    Ядро держит на рельсе только вершины силуэта: ребро внешней грани от вершины рельса `a` до вершины рельса `b` (обе - вершины копии после
    сварки) идёт там, где у копии между ними ещё вершины рельса (T-стык: они без пары, смещаются вдоль своей нормали, ребро внешней грани -
    прямо, щель). Такие вершины встают в ребро внешней грани по порядку пути по рёбрам рельса копии, доля - проекция позиции исходника на
    `a b`. Путь - только через вершины без пары, ребро копии `a b` уже общее. Доли не строго по возрастанию в `(0, 1)` - ребро не делится,
    отказ числом (`BEVEL_COPY_RAIL_SPLIT_REFUSED`), щель названа."""

    inverse = {int(mesh): local for local, mesh in enumerate(copy_index)}
    links = _rail_links(spec)
    outward = {vertex for ring, domain in zip(faces, face_domain) if domain != copy_patch for vertex in ring}
    lonely = {local for local in spec.boundary if int(copy_index[local]) not in outward}
    shared = {frozenset((int(copy_index[a]), int(copy_index[b]))) for ring in spec.faces for a, b in (tuple(key) for key in _edges(ring))}
    entries, refused, used = [], 0, set()
    if not lonely:
        return entries, refused
    for number, (ring, domain) in enumerate(zip(faces, face_domain)):
        if domain == copy_patch:
            continue
        size = len(ring)
        for corner in range(size):
            a, b = int(ring[corner]), int(ring[(corner + 1) % size])
            start, end = inverse.get(a), inverse.get(b)
            if start is None or end is None or frozenset((a, b)) in shared:
                continue
            path = _rail_path(links, lonely, start, end, used)
            if path is None:
                continue
            origin, target = np.asarray(spec.positions[start]), np.asarray(spec.positions[end])
            span = target - origin
            length = float(span @ span)
            shares = [float((np.asarray(spec.positions[local]) - origin) @ span) / length if length > 0.0 else -1.0 for local in path]
            if not all(0.0 < share < 1.0 for share in shares) or any(later <= earlier for earlier, later in zip(shares, shares[1:])):
                refused += 1
                continue
            used.update(path)
            entries.append((number, corner, tuple(zip(path, shares))))
    return entries, refused


def _unit_newell(points):
    n = _newell(points, range(len(points)))
    length = math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2])
    return None if length <= 0.0 else (n[0] / length, n[1] / length, n[2] / length)


def split_rails(arrays):
    """Массивы меша Both с делениями рёбер рельса (`rail_splits`): новые углы внешних граней - вершины копии, UV новой петли - по доле между
    петлями концов ребра (как у `WALL_EDGE_SPLIT_V1`: деления домена кладутся к его `domain_wall_splits`, модель превью ширины видит их так
    же), шов ребра - швы его кусков. Вершина рельса, вставшая в ребро, - одна вершина копии и внешней грани, её смещение - митра нормали копии
    и нормали внешней грани (как у сварки), отказ митры - смещение вдоль нормали копии числом (`BEVEL_COPY_RAIL_SPLIT_MITER_FALLBACK`).
    Делить нечего - те же массивы."""

    from dataclasses import replace

    from .envelope_production_weld import miter_offset

    copy = getattr(arrays, "bevel_copy", None)
    number = None if copy is None else _copy_number(arrays, copy)
    if number is None:
        return arrays
    copy_index = _copy_index(arrays, number)
    faces, face_domain = tuple(arrays.faces), tuple(arrays.face_domain)
    entries, refused = rail_splits(faces, face_domain, copy.patch_id, copy_index, copy.spec)
    if not entries:
        if not refused:
            return arrays
        counters = ((BEVEL_COPY_RAIL_SPLIT, 0), (BEVEL_COPY_RAIL_SPLIT_REFUSED, refused), (BEVEL_COPY_RAIL_SPLIT_MITER_FALLBACK, 0))
        return replace(arrays, weld_counters=(*arrays.weld_counters, *counters))
    at_face = {(face, corner): inserted for face, corner, inserted in entries}
    ordinal = {int(patch): at for at, (patch, _domain) in enumerate(arrays.domain_keys)}
    loop_counts = [int(item) for item in arrays.domain_loop_counts]
    splits = [list(item) for item in arrays.domain_wall_splits] + [[] for _ in range(len(loop_counts) - len(arrays.domain_wall_splits))]
    domain_start = [0]
    for count in loop_counts:
        domain_start.append(domain_start[-1] + count)
    old_inserted = [{place + order for order, (place, *_rest) in enumerate(item)} for item in splits]
    positions = list(arrays.positions)
    uvs_in = arrays.uvs
    new_faces, new_uvs, added = [], [], [[] for _ in loop_counts]
    seams = {tuple(pair) for pair in arrays.seam_edges}
    fallbacks, loop_at = 0, 0
    for face, ring in enumerate(faces):
        size = len(ring)
        ring_out, uv_out = [], []
        domain = ordinal[int(face_domain[face])]
        for corner in range(size):
            ring_out.append(ring[corner])
            uv_out.append(uvs_in[loop_at + corner])
            inserted = at_face.get((face, corner))
            if inserted is None:
                continue
            here, there = loop_at + corner, loop_at + (corner + 1) % size
            local_here, local_there = here - domain_start[domain], there - domain_start[domain]
            if local_here in old_inserted[domain] or local_there in old_inserted[domain]:  # конец ребра - петля деления стены ядра: доли не к чему привязать
                refused += 1
                continue
            view_here = local_here - sum(1 for item in old_inserted[domain] if item < local_here)
            view_there = local_there - sum(1 for item in old_inserted[domain] if item < local_there)
            normal = _unit_newell([positions[item] for item in ring])
            chain = [int(ring[corner])]
            for local, share in inserted:
                vertex = int(copy_index[local])
                found = None if normal is None else miter_offset([tuple(copy.spec.normals[local]), normal])[0]
                if found is None:
                    fallbacks += 1
                else:
                    source = copy.spec.positions[local]
                    positions[vertex] = tuple(source[axis] + arrays.offset * found[axis] for axis in range(3))
                start, end = uvs_in[here], uvs_in[there]
                ring_out.append(vertex)
                uv_out.append((start[0] + (end[0] - start[0]) * share, start[1] + (end[1] - start[1]) * share))
                added[domain].append((view_here + 1, view_here, view_there, share))
                chain.append(vertex)
            chain.append(int(ring[(corner + 1) % size]))
            whole = (min(chain[0], chain[-1]), max(chain[0], chain[-1]))
            if whole in seams:
                seams.discard(whole)
                seams.update((min(x, y), max(x, y)) for x, y in zip(chain, chain[1:]))
        new_faces.append(tuple(ring_out))
        new_uvs.extend(uv_out)
        loop_at += size
    for domain, items in enumerate(added):
        if items:
            loop_counts[domain] += len(items)
            splits[domain] = sorted([*splits[domain], *items], key=lambda item: item[0])
    digest = arrays.digest
    if digest:
        digest = hashlib.sha256(f"{digest}|{BEVEL_COPY_RAIL_SPLIT}|{entries!r}".encode("utf-8")).hexdigest()
    return replace(
        arrays, positions=tuple(positions), faces=tuple(new_faces), uvs=tuple(new_uvs), seam_edges=tuple(sorted(seams)),
        domain_vertex_index=tuple(tuple(item) for item in arrays.domain_vertex_index), domain_loop_counts=tuple(loop_counts),
        domain_wall_splits=tuple(tuple(item) for item in splits), digest=digest, buffers=None,
        weld_counters=(
            *arrays.weld_counters, (BEVEL_COPY_RAIL_SPLIT, sum(len(item) for item in added)), (BEVEL_COPY_RAIL_SPLIT_REFUSED, refused),
            (BEVEL_COPY_RAIL_SPLIT_MITER_FALLBACK, fallbacks),
        ),
    )


__all__ = (
    "BEVEL_COPY_COLUMNS_BRANCH",
    "BEVEL_COPY_RAIL_SPLIT",
    "BEVEL_COPY_RAIL_SPLIT_MITER_FALLBACK",
    "BEVEL_COPY_RAIL_SPLIT_REFUSED",
    "BEVEL_COPY_SEGMENTS",
    "OWNER_NODE",
    "OWNER_SHIFT",
    "BEVEL_COPY_COLUMN_OPEN",
    "BEVEL_COPY_NODE_CONTINUES_STRIP",
    "BEVEL_COPY_NODE_FIT_DEGENERATE",
    "BEVEL_COPY_NODE_NO_STRIP",
    "BEVEL_COPY_NORMAL_DEGENERATE",
    "BEVEL_COPY_RAIL_UNPAIRED",
    "BEVEL_COPY_RING",
    "BEVEL_COPY_SIDES_CONFLICT",
    "BEVEL_COPY_STRIP_FACE_OFF_COLUMN",
    "BEVEL_COPY_UV_FLIPPED_FACES",
    "BevelCopyResultV1",
    "BevelCopySpecV1",
    "copy_for",
    "copy_result",
    "copy_spec",
    "note_copy",
    "rail_splits",
    "rail_unpaired",
    "split_rails",
)
