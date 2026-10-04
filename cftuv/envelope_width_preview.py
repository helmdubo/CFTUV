"""Мгновенное превью ширины декали: граница полосы НОВОЙ ширины, чистая функция binary64.

ЗАЧЕМ. Точный пересчёт декали на новую ширину идёт в фоне и стоит секунды (`2`) и десятки секунд
(`building`); пока он считается, рука уже сдвинула ползунок. Здесь — то, что видно СРАЗУ: для каждой
выбранной цепи линия отступа на новой ширине внутри патча-владельца. Это ПРЕВЬЮ и называется так
(`PREVIEW_BINARY64_V1`): оно не ответ ядра, не пишется в меш и исчезает, когда точный результат для
последней ширины применён. Столкновения фронтов, углы-веера и закон ядра здесь не решаются
(решение владельца: превью без разрешения столкновений).

ЧТО СЧИТАЕТСЯ. Вход — `PatchSurfaceIR` последнего прогона (полигоны граней, их патчи, грани у рёбер)
и выбранные рёбра по патчам (`ProductionRunV1.selected_by_patch`): ровно те рёбра, чью полосу строит
каждый домен. Сторона = (ребро, грань патча при ребре). Стороны одного патча собираются в пути по
общим вершинам; у каждой стороны есть единичное направление внутрь грани (в её плоскости) и точка
отступа, найденная ХОДЬБОЙ по поверхности:

- луч идёт внутри грани, пока не дойдёт до расстояния ширины либо до границы грани;
- через ребро в соседнюю грань ТОГО ЖЕ патча луч переходит с поворотом на двугранный угол (развёртка
  шарниром: составляющая вдоль ребра сохраняется, перпендикулярная ложится в плоскость соседа);
- граница патча (нет соседа в патче) обрывает луч: точка названа `PREVIEW_CLIPPED_AT_PATCH_BOUNDARY`.

На плоском патче это ровно параллельный перенос на ширину (расстояние до исходной прямой равно
ширине до 1e-9); на развёртываемой кривой поверхности точки лежат НА меше, а длина пути по развёртке
равна ширине. Углы пути — митра в плоскости (расстояние до обеих сторон равно ширине), если обе
стороны компланарны и ни одна не оборвана; иначе — фаска из двух точек (`PREVIEW_BEVEL_JOIN`);
слишком острый угол режется пределом митры (`PREVIEW_MITRE_LIMITED`).

НИЧЕГО НЕ ПРОПАДАЕТ МОЛЧА: всё, что превью не смогло или обрезало, — счётчик названного исхода в
`outcomes`; строка статуса выводит их рядом с именем способа.

Модуль не знает Blender (`bpy` в него не попадает): геометрию проверяют тесты без Blender, а
обработчик отрисовки (`envelope_width_overlay`) только рисует готовые линии.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

PREVIEW_BINARY64_V1 = "PREVIEW_BINARY64_V1"

OUTCOME_CLIPPED = "PREVIEW_CLIPPED_AT_PATCH_BOUNDARY"
OUTCOME_MITRE_LIMITED = "PREVIEW_MITRE_LIMITED"
OUTCOME_BEVEL_JOIN = "PREVIEW_BEVEL_JOIN"
OUTCOME_NO_FACE = "PREVIEW_SIDE_WITHOUT_FACE"
OUTCOME_DEGENERATE = "PREVIEW_DEGENERATE_SIDE"
OUTCOME_STEP_LIMIT = "PREVIEW_MARCH_STEP_LIMIT"

#: Наибольшее число граней, через которое идёт один луч; больше — луч оборван и назван.
MARCH_STEP_LIMIT = 64
#: Предел митры: расстояние от угла до острия не больше `MITRE_LIMIT` ширин (острее — фаска).
MITRE_LIMIT = 4.0
#: Стороны компланарны, когда косинус угла между нормалями не меньше этого.
COPLANAR_COSINE = 1.0 - 1e-9
#: Направления внутрь двух соседних сторон совпадают (прямая без излома), когда косинус не меньше этого.
COLLINEAR_COSINE = 1.0 - 1e-12

Vec = tuple[float, float, float]


# --------------------------------------------------------------------------
# Векторы
# --------------------------------------------------------------------------


def _sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec, b: Vec) -> Vec:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(a: Vec, k: float) -> Vec:
    return (a[0] * k, a[1] * k, a[2] * k)


def _dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec, b: Vec) -> Vec:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _length(a: Vec) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _unit(a: Vec) -> Vec | None:
    size = _length(a)
    if size <= 0.0 or not math.isfinite(size):
        return None
    return (a[0] / size, a[1] / size, a[2] / size)


def _along(origin: Vec, direction: Vec, distance: float) -> Vec:
    return (
        origin[0] + direction[0] * distance,
        origin[1] + direction[1] * distance,
        origin[2] + direction[2] * distance,
    )


# --------------------------------------------------------------------------
# Данные входа (не зависят от ширины: считаются один раз на прогон)
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class _Face:
    patch_id: int
    points: tuple[Vec, ...]
    vertex_ids: tuple[int, ...]
    edge_ids: tuple[int, ...]
    normal: Vec
    centroid: Vec
    #: Наибольшая длина ребра: масштаб допусков.
    size: float


#: Первый выход луча из грани стороны: `(расстояние, номер ребра грани, сосед | -1)`.
_Exit = tuple[float, int, int]


@dataclass(frozen=True, slots=True)
class _Side:
    face_id: int
    start: Vec
    end: Vec
    #: Единичное направление внутрь грани, в её плоскости.
    inward: Vec
    exit_start: _Exit
    exit_end: _Exit


@dataclass(frozen=True, slots=True)
class _Run:
    patch_id: int
    sides: tuple[_Side, ...]
    closed: bool


@dataclass(frozen=True, slots=True)
class PreviewInputsV1:
    """Что превью знает о последнем прогоне: грани патчей, пути сторон и названные пропуски."""

    faces: dict
    edge_faces: dict
    runs: tuple
    outcomes: tuple
    edges: int
    build_seconds: float


@dataclass(frozen=True, slots=True)
class WidthPreviewV1:
    """Линии превью ширины: имя способа, ширина, ломаные в локальных координатах источника."""

    method: str
    width: float
    lift: float
    polylines: tuple
    outcomes: tuple
    seconds: float

    @property
    def lines(self) -> int:
        return len(self.polylines)

    @property
    def points(self) -> int:
        return sum(len(item) for item in self.polylines)

    def outcome(self, name: str) -> int:
        return dict(self.outcomes).get(name, 0)

    def status_text(self) -> str:
        """Строка статуса: способ назван ПРЕВЬЮ, а не результатом, и исходы названы."""

        found = ", ".join(f"{name} x{count}" for name, count in self.outcomes if count)
        tail = f"; {found}" if found else ""
        return (
            f"{self.method} preview, not final: {self.lines} lines, "
            f"{self.seconds * 1000.0:.1f} ms{tail}"
        )


def _make_face(face, positions) -> _Face | None:
    try:
        points = tuple(positions[vertex] for vertex in face.vertex_cycle)
    except KeyError:
        return None
    if len(points) < 3 or len(face.edge_cycle) != len(points):
        return None
    normal = _unit(tuple(float(item) for item in face.polygon_normal))
    if normal is None:
        return None
    count = len(points)
    centroid = (
        sum(item[0] for item in points) / count,
        sum(item[1] for item in points) / count,
        sum(item[2] for item in points) / count,
    )
    size = max(_length(_sub(points[(index + 1) % count], points[index])) for index in range(count))
    return _Face(
        int(face.patch_id),
        points,
        tuple(int(item) for item in face.vertex_cycle),
        tuple(int(item) for item in face.edge_cycle),
        normal,
        centroid,
        size,
    )


def _exit_of(face: _Face, point: Vec, direction: Vec, entry: int) -> tuple[float, int]:
    """Расстояние и номер ребра первого выхода луча из грани (бесконечность и `-1`: не вышел)."""

    points = face.points
    count = len(points)
    normal = face.normal
    floor = 1e-10 * face.size
    best, best_edge = math.inf, -1
    for index in range(count):
        if index == entry:
            continue
        q0 = points[index]
        edge = _sub(points[(index + 1) % count], q0)
        denominator = _dot(_cross(direction, edge), normal)
        if abs(denominator) <= 1e-12 * _length(edge):
            continue  # луч вдоль ребра (прижат к границе): не пересечение
        offset = _sub(q0, point)
        distance = _dot(_cross(offset, edge), normal) / denominator
        if distance <= floor or distance >= best:
            continue
        across = _dot(_cross(offset, direction), normal) / denominator
        if -1e-9 <= across <= 1.0 + 1e-9:
            best, best_edge = distance, index
    return best, best_edge


def _neighbour(inputs_edge_faces, faces, face_id: int, index: int) -> int:
    """Грань за ребром `index` грани в том же патче либо `-1` (граница патча, нет соседа)."""

    face = faces[face_id]
    found = [
        other
        for other in inputs_edge_faces.get(face.edge_ids[index], ())
        if other != face_id and other in faces and faces[other].patch_id == face.patch_id
    ]
    return found[0] if len(found) == 1 else -1


def _wedge_exit(face: _Face, vertex: int, entry: int, direction: Vec):
    """Номер ребра при вершине `vertex`, сквозь которое луч уходит сразу (острый угол грани), либо `-1`.

    Луч внутрь по ребру-входу лежит в клине вершины только при внутреннем угле не меньше прямого;
    иначе он выходит из грани в самой вершине через второе ребро клина.
    """

    count = len(face.points)
    other = (vertex - 1) % count if entry == vertex else vertex
    q0 = face.points[other]
    inward = _unit(_cross(face.normal, _sub(face.points[(other + 1) % count], q0)))
    if inward is None:
        return -1
    if _dot(inward, _sub(face.centroid, q0)) < 0.0:
        inward = _mul(inward, -1.0)
    return other if _dot(direction, inward) < -1e-12 else -1


def _side_of(faces, edge_faces, face_id: int, edge_id: int):
    """`(вершина начала, вершина конца, _Side)` по обходу грани либо `None` (вырожденная сторона)."""

    face = faces[face_id]
    index = face.edge_ids.index(edge_id)
    count = len(face.points)
    start, end = face.points[index], face.points[(index + 1) % count]
    inward = _unit(_cross(face.normal, _sub(end, start)))
    if inward is None:
        return None
    if _dot(inward, _sub(face.centroid, start)) < 0.0:
        inward = _mul(inward, -1.0)
    exits = []
    for origin, vertex in ((start, index), (end, (index + 1) % count)):
        leaving = _wedge_exit(face, vertex, index, inward)
        if leaving >= 0:
            distance = 0.0
        else:
            distance, leaving = _exit_of(face, origin, inward, index)
        exits.append(
            (distance, leaving, -1 if leaving < 0 else _neighbour(edge_faces, faces, face_id, leaving))
        )
    return (
        face.vertex_ids[index],
        face.vertex_ids[(index + 1) % count],
        _Side(face_id, start, end, inward, exits[0], exits[1]),
    )


def _flipped(side: _Side) -> _Side:
    return _Side(side.face_id, side.end, side.start, side.inward, side.exit_end, side.exit_start)


def _chain(patch_id: int, sides: list) -> list:
    """Пути из сторон одного патча по общим вершинам; сторона одного ребра с двух граней — отдельный путь.

    `sides` — `[(вершина начала, вершина конца, _Side, ребро)]`.
    """

    runs = []
    by_edge: dict = {}
    for item in sides:
        by_edge.setdefault(item[3], []).append(item)
    chained = []
    for items in by_edge.values():
        if len(items) > 1:
            runs.extend(_Run(patch_id, (side,), False) for _a, _b, side, _e in items)
        else:
            chained.append(items[0])
    at_vertex: dict = {}
    for index, (va, vb, _side, _edge) in enumerate(chained):
        at_vertex.setdefault(va, []).append(index)
        at_vertex.setdefault(vb, []).append(index)
    used = [False] * len(chained)

    def walk(index: int, origin) -> tuple:
        path = []
        vertex = origin
        while True:
            va, vb, side, _edge = chained[index]
            used[index] = True
            path.append(side if va == vertex else _flipped(side))
            vertex = vb if va == vertex else va
            options = [item for item in at_vertex[vertex] if not used[item]]
            if len(at_vertex[vertex]) != 2 or not options:
                return tuple(path), vertex
            index = options[0]

    for vertex, indices in sorted(at_vertex.items()):
        if len(indices) == 2:
            continue
        for index in indices:
            if not used[index]:
                path, _end = walk(index, vertex)
                runs.append(_Run(patch_id, path, False))
    for index in range(len(chained)):
        if used[index]:
            continue
        va = chained[index][0]
        path, end = walk(index, va)
        runs.append(_Run(patch_id, path, end == va and len(path) > 1))
    return runs


def build_preview_inputs(surface, selected_by_patch) -> PreviewInputsV1:
    """Входы превью по `PatchSurfaceIR` и `[(патч, рёбра)]` последнего прогона (один раз на прогон)."""

    started = time.perf_counter()
    patches = {int(patch_id) for patch_id, _edges in selected_by_patch}
    positions = {item.vertex_id: item.position for item in surface.vertices}
    edge_faces = {item.edge_id: tuple(item.source_face_ids) for item in surface.edges}
    faces = {}
    for face in surface.faces:
        if int(face.patch_id) in patches:
            built = _make_face(face, positions)
            if built is not None:
                faces[int(face.face_id)] = built
    runs, missing, degenerate, edges = [], 0, 0, 0
    for patch_id, edge_ids in selected_by_patch:
        sides = []
        for edge_id in sorted(int(item) for item in edge_ids):
            edges += 1
            owners = [
                face_id
                for face_id in edge_faces.get(edge_id, ())
                if face_id in faces and faces[face_id].patch_id == int(patch_id)
            ]
            if not owners:
                missing += 1
            for face_id in owners:
                found = _side_of(faces, edge_faces, face_id, edge_id)
                if found is None:
                    degenerate += 1
                else:
                    sides.append((found[0], found[1], found[2], edge_id))
        runs.extend(_chain(int(patch_id), sides))
    return PreviewInputsV1(
        faces,
        edge_faces,
        tuple(runs),
        ((OUTCOME_NO_FACE, missing), (OUTCOME_DEGENERATE, degenerate)),
        edges,
        time.perf_counter() - started,
    )


# --------------------------------------------------------------------------
# Счёт на ширину
# --------------------------------------------------------------------------


class _Counts:
    __slots__ = ("clipped", "mitre", "bevel", "steps")

    def __init__(self) -> None:
        self.clipped = self.mitre = self.bevel = self.steps = 0


def _march(inputs: PreviewInputsV1, face_id: int, edge: int, neighbour: int, point: Vec, direction: Vec,
           remaining: float, counts: _Counts):
    """Ходьба через рёбра: `(точка, нормаль грани, оборвано)`; сосед `neighbour` лежит за ребром `edge`."""

    faces = inputs.faces
    for _step in range(MARCH_STEP_LIMIT):
        face = faces[face_id]
        count = len(face.points)
        shared = face.edge_ids[edge]
        q0 = face.points[edge]
        hinge = _unit(_sub(face.points[(edge + 1) % count], q0))
        following = faces[neighbour]
        inward = _unit(_cross(following.normal, hinge)) if hinge is not None else None
        if inward is None:
            counts.clipped += 1
            return point, face.normal, True
        if _dot(inward, _sub(following.centroid, point)) < 0.0:
            inward = _mul(inward, -1.0)
        along = _dot(direction, hinge)
        perpendicular = math.sqrt(max(0.0, 1.0 - along * along))
        turned = _unit(_add(_mul(hinge, along), _mul(inward, perpendicular)))
        if turned is None:
            counts.clipped += 1
            return point, face.normal, True
        entry = following.edge_ids.index(shared)
        distance, leaving = _exit_of(following, point, turned, entry)
        if remaining <= distance:
            return _along(point, turned, remaining), following.normal, False
        point = _along(point, turned, distance)
        remaining -= distance
        if leaving < 0:
            counts.clipped += 1
            return point, following.normal, True
        onward = _neighbour(inputs.edge_faces, faces, neighbour, leaving)
        if onward < 0:
            counts.clipped += 1
            return point, following.normal, True
        face_id, edge, neighbour, direction = neighbour, leaving, onward, turned
    counts.steps += 1
    counts.clipped += 1
    return point, faces[face_id].normal, True


def _reach(inputs: PreviewInputsV1, side: _Side, origin: Vec, exit_info: _Exit, width: float, counts: _Counts):
    """Точка отступа от конца стороны: `(точка, нормаль, оборвано)`."""

    distance, edge, neighbour = exit_info
    face = inputs.faces[side.face_id]
    if width <= distance:
        return _along(origin, side.inward, width), face.normal, False
    point = _along(origin, side.inward, distance)
    if edge < 0 or neighbour < 0:
        counts.clipped += 1
        return point, face.normal, True
    return _march(
        inputs, side.face_id, edge, neighbour, point, side.inward, width - distance, counts
    )


def _lifted(point: Vec, normal: Vec, lift: float) -> Vec:
    return point if lift == 0.0 else _along(point, normal, lift)


def _join(inputs, previous: _Side, following: _Side, vertex: Vec, ends, width: float, lift: float, counts):
    """Точки угла между двумя сторонами: митра (компланарные, целые), иначе фаска; прямая — ничего."""

    (end_point, end_normal, end_clipped), (start_point, start_normal, start_clipped) = ends
    faces = inputs.faces
    normal_a, normal_b = faces[previous.face_id].normal, faces[following.face_id].normal
    if not (end_clipped or start_clipped) and _dot(normal_a, normal_b) >= COPLANAR_COSINE:
        cosine = _dot(previous.inward, following.inward)
        if cosine >= COLLINEAR_COSINE:
            return []
        denominator = 1.0 + cosine
        if denominator * MITRE_LIMIT * MITRE_LIMIT >= 2.0:
            corner = _along(vertex, _add(previous.inward, following.inward), width / denominator)
            return [_lifted(corner, normal_a, lift)]
        counts.mitre += 1
    counts.bevel += 1
    first = _lifted(end_point, end_normal, lift)
    second = _lifted(start_point, start_normal, lift)
    return [first] if first == second else [first, second]


def _offset_run(inputs: PreviewInputsV1, run: _Run, width: float, lift: float, counts: _Counts) -> list:
    sides = run.sides
    count = len(sides)
    reached = [
        (
            _reach(inputs, side, side.start, side.exit_start, width, counts),
            _reach(inputs, side, side.end, side.exit_end, width, counts),
        )
        for side in sides
    ]
    points: list = []
    if not run.closed:
        point, normal, _clipped = reached[0][0]
        points.append(_lifted(point, normal, lift))
    joins = range(count) if run.closed else range(1, count)
    for index in joins:
        previous = sides[index - 1]
        points.extend(
            _join(
                inputs,
                previous,
                sides[index],
                sides[index].start,
                (reached[index - 1][1], reached[index][0]),
                width,
                lift,
                counts,
            )
        )
    if run.closed:
        if points:
            points.append(points[0])
    else:
        point, normal, _clipped = reached[-1][1]
        points.append(_lifted(point, normal, lift))
    return points


def _caps(run: _Run, points: list, lift: float, inputs: PreviewInputsV1) -> list:
    """Торцы полосы у открытого пути: от исходной точки к концу линии отступа."""

    if run.closed or len(points) < 2:
        return []
    first, last = run.sides[0], run.sides[-1]
    return [
        (_lifted(first.start, inputs.faces[first.face_id].normal, lift), points[0]),
        (_lifted(last.end, inputs.faces[last.face_id].normal, lift), points[-1]),
    ]


def chain_centroid(inputs: PreviewInputsV1) -> Vec | None:
    """Среднее концов всех сторон прогона (ось модального инструмента на экране) либо `None`."""

    total = [0.0, 0.0, 0.0]
    count = 0
    for run in inputs.runs:
        for side in run.sides:
            for point in (side.start, side.end):
                total[0] += point[0]
                total[1] += point[1]
                total[2] += point[2]
                count += 1
    return None if not count else (total[0] / count, total[1] / count, total[2] / count)


def compute_width_preview(inputs: PreviewInputsV1, width: float, *, lift: float = 0.0) -> WidthPreviewV1:
    """Линии отступа на ширину `width` для всех путей прогона (чистая функция, binary64)."""

    started = time.perf_counter()
    width = float(width)
    lift = float(lift)
    counts = _Counts()
    lines: list = []
    for run in inputs.runs:
        points = _offset_run(inputs, run, width, lift, counts)
        if len(points) >= 2:
            lines.append(tuple(points))
            lines.extend(_caps(run, points, lift, inputs))
    outcomes = (
        (OUTCOME_CLIPPED, counts.clipped),
        (OUTCOME_MITRE_LIMITED, counts.mitre),
        (OUTCOME_BEVEL_JOIN, counts.bevel),
        (OUTCOME_STEP_LIMIT, counts.steps),
        *inputs.outcomes,
    )
    return WidthPreviewV1(
        PREVIEW_BINARY64_V1,
        width,
        lift,
        tuple(lines),
        tuple(item for item in outcomes if item[1]),
        time.perf_counter() - started,
    )


__all__ = (
    "COLLINEAR_COSINE",
    "COPLANAR_COSINE",
    "MARCH_STEP_LIMIT",
    "MITRE_LIMIT",
    "OUTCOME_BEVEL_JOIN",
    "OUTCOME_CLIPPED",
    "OUTCOME_DEGENERATE",
    "OUTCOME_MITRE_LIMITED",
    "OUTCOME_NO_FACE",
    "OUTCOME_STEP_LIMIT",
    "PREVIEW_BINARY64_V1",
    "PreviewInputsV1",
    "WidthPreviewV1",
    "build_preview_inputs",
    "chain_centroid",
    "compute_width_preview",
)
