"""Копия граней бевела декали Both (BEVEL_FACES_BOTH_V2). Без Blender: чистый Python и numpy.

ЧТО ЭТО. Выделенные грани бевела (полосы и узлы) не заливаются ядром изнутри: декаль Both кладёт над ними их КОПИЮ - те же вершины, грани
и петли, что у оригинала, смещённые над поверхностью, - а ядро растит декаль только НАРУЖУ (нажатие Outward: грани вне выделения, источники -
рёбра, которые оставляет выделение). Копия от ширины не зависит; меняется только внешняя часть.

ОДИН СВАРЕННЫЙ МЕШ (BEVEL_COPY_WELD_V2). Копия - ещё один домен сборки меша (`copy_result`): вид домена с вершинами копии, у каждой ссылка
вершины исходника, которой её называет ядро (`LOCATION_FORMAT`), и позиция исходника (те же числа, что у снапшота анализа: `float` от `co`
вершины bmesh), поэтому
сварка хоста (`envelope_production_weld`) сама сводит вершины рельса копии с вершинами рельса внешней декали в ОДНУ вершину, а смещение этой
вершины - митра нормалей обеих частей (тот же закон смещения, что у швов между доменами ядра). Внутренние вершины копии смещены вдоль своей
нормали (BEVEL_COPY_OFFSET_V2: нормаль вершины - сумма векторов Ньюэлла её граней копии, то есть нормалей, взвешенных площадью). Где внешняя
часть отказана, у вершины рельса копии пары нет: она остаётся своей вершиной (сварка её не находит), это число `BEVEL_COPY_RAIL_UNPAIRED`
в строке статуса нажатия (`rail_unpaired`) - не тихий разрыв. Позиции рельса не берутся из видов ядра: совпадение побитово держит тест
Blender (ноль вершин без пары, ноль пар вершин в одной точке).

UV КОПИИ (метры, их отображает в крылья `bevel_wings`). Для каждой полосы (компонента граней полосы по рёбрам) - колонки профиля: путь по
квадам полосы от ребра рельса до ребра рельса по ту сторону (`_walk`), и в нём две перекладины (цепи вершин поперёк полосы). Стороны рельсов
красятся по перекладинам (концы перекладины - разные стороны, вершины одного ребра рельса - одна), знак `+` - сторона цвета 0. На перекладине
с длиной `W` вершина на расстоянии `d` от стороны `+` получает поперёк `S = W / 2 - d` (средняя линия бевела - нуль), а вся перекладина - одну
`U`: длину дуги средней линии (ломаной через середины перекладин) от первой перекладины, поэтому зеркальные точки крыльев имеют одну `U`
(перекладины идут вдоль полосы по общим колонкам; у замкнутого кольца последняя
колонка несёт `U` полного оборота - шов UV). Направление `U` выбрано так, что грани копии в UV против часовой (как карты ядра).

УЗЕЛ ПРОДОЛЖАЕТ ПОЛОСУ (BEVEL_COPY_NODE_CONTINUES_STRIP_V2). Грань узла (и грань полосы вне колонок) берёт кадр полосы, с которой у неё
больше всего общих рёбер; грань без такой полосы - кадр соседней грани узла с наибольшим числом общих рёбер (обход слоями, ничья - меньший
номер кадра). Вершина узла, лежащая на гранях своей полосы, берёт её UV точно; остальные грани узла разворачиваются слоями от уже известных
вершин (у грани их две и больше): подобие без отражения из плоскости грани в UV по известным вершинам, новая вершина берёт UV первой грани,
что её положила, поэтому грань узла не выворачивается и UV непрерывна через разрез и внутри узла. Узел без полосы рядом - своя плоская проекция
(`BEVEL_COPY_NODE_NO_STRIP`). Каждое отступление - число с именем (правило 4).
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

#: Ссылка вершины копии: та же, что у вершины исходника в батчах ядра - `location:` (`mesh_weld.WELD_LOCATION_PREFIX` без `src:`), ключ вершины
#: батча `src:<SourceVertexId>` и идентичность вершины хоста `host-vertex:host-source:<отпечаток>:<имя>:<номер>` (`envelope_request_export._source_ids`).
#: Разойдётся формат - сварка не найдёт пар, и это число `BEVEL_COPY_RAIL_UNPAIRED` (тест Blender требует нуль).
LOCATION_FORMAT = "location:src:host-vertex:host-source:{digest}:{name}:{vertex}"
#: Значения UV двух граней копии в общей вершине, разошедшиеся больше этого (метры), - шов копии.
SEAM_UV_TOLERANCE = 1e-12
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


def _dist(a, b) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


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


def _chain_steps(coords, chain) -> list:
    steps = [0.0]
    for a, b in zip(chain, chain[1:]):
        steps.append(steps[-1] + _dist(coords[a], coords[b]))
    return steps


def _centre(coords, chain, steps):
    """Точка перекладины на половине её длины (средняя линия бевела)."""

    half = 0.5 * steps[-1]
    for at in range(1, len(chain)):
        if steps[at] >= half:
            span = steps[at] - steps[at - 1]
            t = 0.0 if span <= 0.0 else (half - steps[at - 1]) / span
            a, b = coords[chain[at - 1]], coords[chain[at]]
            return tuple(a[axis] + t * (b[axis] - a[axis]) for axis in range(3))
    return tuple(coords[chain[-1]])


def _frame(coords, rings, oriented, column_of, order, ring):
    """Кадр куска полосы: перекладины `order` вдоль; у всей перекладины одна `U` - длина дуги средней линии бевела (ломаная через середины
    перекладин) от первой перекладины, поэтому `U` двух крыльев в зеркальных точках одна; `S = W / 2 - d`."""

    steps = [_chain_steps(coords, oriented[name]) for name in order]
    centres = [_centre(coords, oriented[name], chain_steps) for name, chain_steps in zip(order, steps)]
    along = [0.0]
    for first, second in zip(centres, centres[1:]):
        along.append(along[-1] + _dist(first, second))
    total = along[-1] + (_dist(centres[-1], centres[0]) if ring else 0.0)

    def values(number, u):
        chain, chain_steps = oriented[order[number]], steps[number]
        width = chain_steps[-1]
        return {vertex: (u, 0.5 * width - d) for vertex, d in zip(chain, chain_steps)}

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


def _orient(frames, rings, events) -> None:
    """Направление `U` каждого кадра: грани кадра в UV против часовой (сумма площадей); после - вывернутые грани числом."""

    flipped = 0
    for faces, corner in frames:
        total = sum(_signed_area([corner[(face, vertex)] for vertex in rings[face]]) for face in faces)
        if total < 0.0:
            for slot, (u, s) in list(corner.items()):
                corner[slot] = (-u, s)
        flipped += sum(1 for face in faces if _signed_area([corner[(face, vertex)] for vertex in rings[face]]) < 0.0)
    _count(events, BEVEL_COPY_UV_FLIPPED_FACES, flipped)


def _assign_nodes(rest, frame_of, table, rings):
    """Грани вне кадров -> кадр (BEVEL_COPY_NODE_CONTINUES_STRIP_V2): слоями, по числу общих рёбер с гранями кадра; без соседа - не назначены."""

    owner = dict(frame_of)
    pending = set(rest)
    while pending:
        chosen = {}
        for face in sorted(pending):
            votes: dict = {}
            for key in _edges(rings[face]):
                for other in table.get(key, ()):
                    if other != face and other in owner:
                        votes[owner[other]] = votes.get(owner[other], 0) + 1
            if votes:
                chosen[face] = min(votes, key=lambda frame: (-votes[frame], frame))
        if not chosen:
            break
        owner.update(chosen)
        pending -= set(chosen)
    return {face: owner[face] for face in rest if face in owner}, pending


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
    _orient(frames, rings, events)
    corner: dict = {}
    frame_of: dict = {}
    for number, (faces, values) in enumerate(frames):
        corner.update(values)
        for face in faces:
            frame_of[face] = number
    rest = selected - set(frame_of)
    _count(events, BEVEL_COPY_STRIP_FACE_OFF_COLUMN, len(rest & strip))
    assigned, lonely = _assign_nodes(rest, frame_of, table, rings)
    _count(events, BEVEL_COPY_NODE_CONTINUES_STRIP, len(assigned))
    by_frame: dict = {}
    for face, frame in assigned.items():
        by_frame.setdefault(frame, set()).add(face)
    for frame, faces in sorted(by_frame.items()):
        frame_faces, values = frames[frame]
        own = {vertex: values[(face, vertex)] for face in sorted(frame_faces) for vertex in rings[face]}
        for cluster in _components(faces, table, rings):
            touching = {vertex for face in cluster for vertex in rings[face]}
            corner.update(_unfold(coords, rings, cluster, {vertex: own[vertex] for vertex in touching if vertex in own}, events))
            for face in cluster:
                frame_of[face] = frame
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
        owners=tuple(int(frame_of[face]) for face in face_ids),
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


def rail_unpaired(arrays) -> int:
    """Вершины копии на границе выделения, которые сварка не свела ни с одной вершиной внешней части (`BEVEL_COPY_RAIL_UNPAIRED`): их номер
    вершины меша не встречается у других доменов. Копии в массивах нет - нуль."""

    copy = getattr(arrays, "bevel_copy", None)
    if copy is None:
        return 0
    keys = tuple(arrays.domain_keys)
    number = next((at for at, key in enumerate(keys) if key == (copy.patch_id, copy.domain_id)), None)
    if number is None:
        return 0
    buffers = getattr(arrays, "buffers", None)
    index = [buffers.domain_index(at).tolist() if buffers is not None else list(arrays.domain_vertex_index[at]) for at in range(len(keys))]
    others = {vertex for at, values in enumerate(index) if at != number for vertex in values}
    mine = index[number]
    return sum(1 for local in copy.spec.boundary if mine[local] not in others)


__all__ = (
    "BEVEL_COPY_COLUMNS_BRANCH",
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
    "rail_unpaired",
)
