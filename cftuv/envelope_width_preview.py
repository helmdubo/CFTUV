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
общим вершинам. Каждая точка линии отступа — конец ЛУЧА из вершины пути, найденный ХОДЬБОЙ по поверхности:

- в вершине считается ВЕЕР: грани патча вокруг вершины от ребра первой стороны к ребру второй (угол
  между ними — сумма внутренних углов граней веера, то есть в касательной развёртке вершины, а не в
  плоскости какой-то одной грани: на кривой стене сумма углов — внутренний угол патча, нормали граней
  при вершине разные, и направление берётся в плоскости КАЖДОЙ грани веера, а не усредняется);
- УГОЛ пути (две выбранные стороны в одной вершине): смещённые линии пересекаются на биссектрисе веера
  на расстоянии `ширина / sin(угол / 2)` — митра, при любом внутреннем угле меньше 180° (точная граница
  полосы), в том числе при остром и в развёртке кривой стены. Угол больше 180° (рефлексный) — те же
  смещённые линии, пока митра не длиннее `MITRE_LIMIT` ширин; острее — ФАСКА из двух точек отступа
  (`PREVIEW_MITRE_LIMITED`), каждая на ширине от своей стороны; исходная вершина точкой превью не бывает;
- КОНЕЦ пути (в вершине нет второй выбранной стороны): смещённая линия доходит до граничного ребра патча,
  которым веер заканчивается, и СКОЛЬЗИТ по нему: при угле веера меньше прямого точка лежит на
  граничном ребре на расстоянии `ширина / sin(угол)` от вершины, иначе — перпендикуляр к стороне;
- луч идёт внутри грани, пока не дойдёт до нужного расстояния либо до границы грани; через ребро в
  соседнюю грань ТОГО ЖЕ патча луч переходит с поворотом на двугранный угол (развёртка шарниром:
  составляющая вдоль ребра сохраняется, перпендикулярная ложится в плоскость соседа);
- граница патча (нет соседа в патче) обрывает луч: точка названа `PREVIEW_CLIPPED_AT_PATCH_BOUNDARY`;
- сторона короче вылета митры соседнего угла: смещённый отрезок вышел бы НАЗАД, и пологие точки между
  углами (прямая через много граней, излом кривой стены) убираются — настоящая граница полосы их не
  содержит; если назад идёт отрезок между двумя настоящими углами (патч уже двух ширин: смещённые
  линии противоположных сторон пересеклись), он остаётся и называется `PREVIEW_OFFSET_FOLDS_BACK`;
- две грани, касающиеся в одной вершине без общего ребра веера, веера между сторонами не дают: путь
  режется на два, каждый со своим концом, исход `PREVIEW_CORNER_WITHOUT_FAN`.

Направления лучей и их первые выходы из граней не зависят от ширины и считаются один раз на прогон
(`build_preview_inputs`); на ширину остаётся O(1) на угол и на конец пути. На плоском патче линия отступа
лежит ровно на ширине от исходной прямой (до 1e-9); на развёртываемой кривой поверхности точки лежат НА
меше, а длина пути по развёртке равна ширине.

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
OUTCOME_NO_FAN = "PREVIEW_CORNER_WITHOUT_FAN"
OUTCOME_FOLDS_BACK = "PREVIEW_OFFSET_FOLDS_BACK"
OUTCOME_NO_FACE = "PREVIEW_SIDE_WITHOUT_FACE"
OUTCOME_DEGENERATE = "PREVIEW_DEGENERATE_SIDE"
OUTCOME_STEP_LIMIT = "PREVIEW_MARCH_STEP_LIMIT"

#: Наибольшее число граней, через которое идёт один луч; больше — луч оборван и назван.
MARCH_STEP_LIMIT = 64
#: Предел митры РЕФЛЕКСНОГО угла: расстояние от вершины до угла отступа не больше `MITRE_LIMIT` ширин.
#: Выпуклый угол (меньше 180°) пределом не режется: пересечение смещённых линий там и есть граница полосы.
MITRE_LIMIT = 4.0
#: Наименьший отличимый от нуля и от полного оборота угол веера (радианы).
FAN_EPSILON = 1e-9
#: Путь не поворачивает в точке, когда синус угла между соседними отрезками не больше этого.
STRAIGHT_SINE = 1e-9
#: Угол пути, отличающийся от развёрнутого не больше этого (радианы), «пологий»: его точку отступа, когда соседний
#: угол отнял у стороны больше длины, чем она имеет, убирают, а не оставляют шипом назад.
GENTLE_TURN = 1.0

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
    #: `+1`, когда обход цикла вершин идёт против часовой стрелки вокруг нормали (внутренность слева от ребра).
    winding: float


#: Первый выход луча из грани: `(расстояние, номер ребра грани, сосед | -1)`.
_Exit = tuple[float, int, int]


@dataclass(frozen=True, slots=True)
class _Side:
    face_id: int
    edge_id: int
    start_vertex: int
    end_vertex: int
    start: Vec
    end: Vec


@dataclass(frozen=True, slots=True)
class _Step:
    """Грань веера вокруг вершины: единичное направление на ребро входа, внутрь грани, угол грани в вершине."""

    face_id: int
    toward: Vec
    across: Vec
    alpha: float


@dataclass(frozen=True, slots=True)
class _Ray:
    """Луч из вершины внутри грани веера: направление, первый выход, расстояние на единицу ширины."""

    face_id: int
    origin: Vec
    direction: Vec
    exit: _Exit
    scale: float


@dataclass(frozen=True, slots=True)
class _Corner:
    """Угол пути: один луч (митра) либо два (фаска рефлексного угла острее предела митры)."""

    rays: tuple
    limited: bool
    #: Одна митра пологого угла: точку можно убрать, если сторона поглощена соседним углом.
    gentle: bool


@dataclass(frozen=True, slots=True)
class _Run:
    patch_id: int
    sides: tuple
    closed: bool
    #: `corners[i]` — угол в начале `sides[i]` (у открытого пути `corners[0]` — `None`).
    corners: tuple
    #: У открытого пути — лучи концов `(в начале первой стороны, в конце последней)`, `None` — луча нет.
    ends: tuple


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
    area: Vec = (0.0, 0.0, 0.0)
    for index in range(count):
        area = _add(
            area, _cross(_sub(points[index], centroid), _sub(points[(index + 1) % count], centroid))
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
        1.0 if _dot(area, normal) >= 0.0 else -1.0,
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


def _patch_neighbours(edge_faces, faces, face_id: int, edge_id: int) -> list:
    """Грани того же патча за ребром, кроме самой грани."""

    patch_id = faces[face_id].patch_id
    return [
        other
        for other in edge_faces.get(edge_id, ())
        if other != face_id and other in faces and faces[other].patch_id == patch_id
    ]


def _neighbour(edge_faces, faces, face_id: int, index: int) -> int:
    """Грань за ребром `index` грани в том же патче либо `-1` (граница патча, нет соседа)."""

    found = _patch_neighbours(edge_faces, faces, face_id, faces[face_id].edge_ids[index])
    return found[0] if len(found) == 1 else -1


def _side_of(faces, face_id: int, edge_id: int) -> _Side | None:
    """Сторона (ребро, грань) по обходу грани либо `None` (вырожденное ребро)."""

    face = faces[face_id]
    index = face.edge_ids.index(edge_id)
    count = len(face.points)
    start, end = face.points[index], face.points[(index + 1) % count]
    if _length(_sub(end, start)) <= 0.0:
        return None
    return _Side(
        face_id, edge_id, face.vertex_ids[index], face.vertex_ids[(index + 1) % count], start, end
    )


def _flipped(side: _Side) -> _Side:
    return _Side(
        side.face_id, side.edge_id, side.end_vertex, side.start_vertex, side.end, side.start
    )


# --------------------------------------------------------------------------
# Веер вокруг вершины
# --------------------------------------------------------------------------


def _fan(faces, edge_faces, face_id: int, vertex: int, edge_in: int, stop_edge: int):
    """Грани патча вокруг вершины от ребра `edge_in` и исход обхода: `(шаги, исход)`.

    Обход идёт ВНУТРЬ патча: из грани через второе её ребро при вершине в соседнюю грань патча и так далее.
    Исход: `reached` (дошли до `stop_edge`), `boundary` (граничное ребро патча), `closed` (вернулись к
    `edge_in`: вершина внутри патча), `broken` (неманифольд либо вырожденная грань).
    """

    steps: list = []
    first = edge_in
    for _guard in range(len(faces) + 1):
        face = faces[face_id]
        count = len(face.points)
        index = face.vertex_ids.index(vertex)
        e_back, e_forward = face.edge_ids[index - 1], face.edge_ids[index]
        if edge_in == e_forward:
            to_in, to_out, edge_out, sign = (index + 1) % count, index - 1, e_back, face.winding
        elif edge_in == e_back:
            to_in, to_out, edge_out, sign = index - 1, (index + 1) % count, e_forward, -face.winding
        else:
            return steps, "broken"
        origin = face.points[index]
        toward = _unit(_sub(face.points[to_in], origin))
        outward = _unit(_sub(face.points[to_out], origin))
        across = None if toward is None else _unit(_mul(_cross(face.normal, toward), sign))
        if outward is None or across is None:
            return steps, "broken"
        alpha = math.atan2(sign * _dot(_cross(toward, outward), face.normal), _dot(toward, outward))
        if abs(alpha) <= FAN_EPSILON:
            return steps, "broken"
        steps.append(_Step(face_id, toward, across, alpha + 2.0 * math.pi if alpha < 0.0 else alpha))
        if edge_out == stop_edge:
            return steps, "reached"
        if edge_out == first:
            return steps, "closed"
        onward = _patch_neighbours(edge_faces, faces, face_id, edge_out)
        if not onward:
            return steps, "boundary"
        if len(onward) > 1:
            return steps, "broken"
        face_id, edge_in = onward[0], edge_out
    return steps, "closed"


def _ray_at(faces, edge_faces, steps, origin: Vec, angle: float, scale: float) -> _Ray:
    """Луч из вершины под углом `angle` от ребра входа веера, длиной `scale` ширин."""

    start = 0.0
    for step in steps:
        if angle <= start + step.alpha + 1e-12 or step is steps[-1]:
            break
        start += step.alpha
    local = min(max(angle - start, 0.0), step.alpha)
    direction = _add(_mul(step.toward, math.cos(local)), _mul(step.across, math.sin(local)))
    reach, leaving = _exit_of(faces[step.face_id], origin, direction, -1)
    neighbour = -1 if leaving < 0 else _neighbour(edge_faces, faces, step.face_id, leaving)
    return _Ray(step.face_id, origin, direction, (reach, leaving, neighbour), scale)


def _corner(faces, edge_faces, previous: _Side, following: _Side) -> _Corner | None:
    """Угол между двумя сторонами пути либо `None`, когда веер не соединяет их ребра."""

    steps, status = _fan(
        faces, edge_faces, previous.face_id, previous.end_vertex, previous.edge_id, following.edge_id
    )
    angle = sum(step.alpha for step in steps)
    if status != "reached" or not FAN_EPSILON < angle < 2.0 * math.pi - FAN_EPSILON:
        return None
    origin = following.start
    scale = 1.0 / math.sin(angle / 2.0)
    if angle < math.pi or scale <= MITRE_LIMIT:
        ray = _ray_at(faces, edge_faces, steps, origin, angle / 2.0, scale)
        return _Corner((ray,), False, abs(angle - math.pi) <= GENTLE_TURN)
    return _Corner(
        (
            _ray_at(faces, edge_faces, steps, origin, math.pi / 2.0, 1.0),
            _ray_at(faces, edge_faces, steps, origin, angle - math.pi / 2.0, 1.0),
        ),
        True,
        False,
    )


def _end(faces, edge_faces, side: _Side, at_start: bool) -> _Ray | None:
    """Луч конца открытого пути: скольжение по граничному ребру патча либо перпендикуляр к стороне."""

    vertex, origin = (side.start_vertex, side.start) if at_start else (side.end_vertex, side.end)
    steps, status = _fan(faces, edge_faces, side.face_id, vertex, side.edge_id, -1)
    if not steps:
        return None
    angle = sum(step.alpha for step in steps)
    if status == "boundary" and FAN_EPSILON < angle < math.pi / 2.0 - FAN_EPSILON:
        return _ray_at(faces, edge_faces, steps, origin, angle, 1.0 / math.sin(angle))
    return _ray_at(faces, edge_faces, steps, origin, math.pi / 2.0, 1.0)


def _dress(faces, edge_faces, patch_id: int, path: tuple, closed: bool, notes: list) -> list:
    """Путь сторон -> пути с углами и концами; угол без веера режет путь на два (названо в `notes`)."""

    count = len(path)
    corners = [None] * count
    broken = []
    for index in range(0 if closed else 1, count):
        corners[index] = _corner(faces, edge_faces, path[index - 1], path[index])
        if corners[index] is None:
            broken.append(index)
    if not broken:
        ends = ()
        if not closed:
            ends = (_end(faces, edge_faces, path[0], True), _end(faces, edge_faces, path[-1], False))
        return [_Run(patch_id, path, closed, tuple(corners), ends)]
    notes.append(len(broken))
    if closed:
        path = path[broken[0] :] + path[: broken[0]]
        cuts = [(item - broken[0]) % count for item in broken]
    else:
        cuts = broken
    bounds = sorted(set(cuts)) + [count]
    pieces = [path[: bounds[0]]] if bounds[0] > 0 else []
    pieces.extend(path[bounds[i] : bounds[i + 1]] for i in range(len(bounds) - 1))
    runs: list = []
    for piece in pieces:
        runs.extend(_dress(faces, edge_faces, patch_id, piece, False, notes))
    return runs


def _chain(faces, edge_faces, patch_id: int, sides: list, notes: list) -> list:
    """Пути из сторон одного патча по общим вершинам; сторона одного ребра с двух граней — отдельный путь."""

    paths: list = []
    by_edge: dict = {}
    for side in sides:
        by_edge.setdefault(side.edge_id, []).append(side)
    chained = []
    for items in by_edge.values():
        if len(items) > 1:
            paths.extend(((side,), False) for side in items)
        else:
            chained.append(items[0])
    at_vertex: dict = {}
    for index, side in enumerate(chained):
        at_vertex.setdefault(side.start_vertex, []).append(index)
        at_vertex.setdefault(side.end_vertex, []).append(index)
    used = [False] * len(chained)

    def walk(index: int, origin) -> tuple:
        path = []
        vertex = origin
        while True:
            side = chained[index]
            used[index] = True
            path.append(side if side.start_vertex == vertex else _flipped(side))
            vertex = path[-1].end_vertex
            options = [item for item in at_vertex[vertex] if not used[item]]
            if len(at_vertex[vertex]) != 2 or not options:
                return tuple(path), vertex
            index = options[0]

    for vertex, indices in sorted(at_vertex.items()):
        if len(indices) == 2:
            continue
        for index in indices:
            if not used[index]:
                path, _end_vertex = walk(index, vertex)
                paths.append((path, False))
    for index in range(len(chained)):
        if used[index]:
            continue
        path, end = walk(index, chained[index].start_vertex)
        paths.append((path, end == chained[index].start_vertex and len(path) > 1))
    runs: list = []
    for path, closed in paths:
        runs.extend(_dress(faces, edge_faces, patch_id, path, closed, notes))
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
    runs, missing, degenerate, edges, notes = [], 0, 0, 0, []
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
                side = _side_of(faces, face_id, edge_id)
                if side is None:
                    degenerate += 1
                else:
                    sides.append(side)
        runs.extend(_chain(faces, edge_faces, int(patch_id), sides, notes))
    degenerate += sum(1 for run in runs if not run.closed for end in run.ends if end is None)
    return PreviewInputsV1(
        faces,
        edge_faces,
        tuple(runs),
        ((OUTCOME_NO_FACE, missing), (OUTCOME_DEGENERATE, degenerate), (OUTCOME_NO_FAN, sum(notes))),
        edges,
        time.perf_counter() - started,
    )


# --------------------------------------------------------------------------
# Счёт на ширину
# --------------------------------------------------------------------------


class _Counts:
    __slots__ = ("clipped", "mitre", "steps", "folds")

    def __init__(self) -> None:
        self.clipped = self.mitre = self.steps = self.folds = 0


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
            return point, face.normal, True
        if _dot(inward, _sub(following.centroid, point)) < 0.0:
            inward = _mul(inward, -1.0)
        along = _dot(direction, hinge)
        perpendicular = math.sqrt(max(0.0, 1.0 - along * along))
        turned = _unit(_add(_mul(hinge, along), _mul(inward, perpendicular)))
        if turned is None:
            return point, face.normal, True
        entry = following.edge_ids.index(shared)
        distance, leaving = _exit_of(following, point, turned, entry)
        if remaining <= distance:
            return _along(point, turned, remaining), following.normal, False
        point = _along(point, turned, distance)
        remaining -= distance
        if leaving < 0:
            return point, following.normal, True
        onward = _neighbour(inputs.edge_faces, faces, neighbour, leaving)
        if onward < 0:
            return point, following.normal, True
        face_id, edge, neighbour, direction = neighbour, leaving, onward, turned
    counts.steps += 1
    return point, faces[face_id].normal, True


def _place(inputs: PreviewInputsV1, ray: _Ray, width: float, counts: _Counts):
    """Точка на луче в `scale * width` от вершины: `(точка, нормаль, оборвано)`."""

    distance = ray.scale * width
    reach, edge, neighbour = ray.exit
    face = inputs.faces[ray.face_id]
    if distance <= reach:
        return _along(ray.origin, ray.direction, distance), face.normal, False
    point = _along(ray.origin, ray.direction, reach)
    if edge < 0 or neighbour < 0:
        return point, face.normal, True
    return _march(inputs, ray.face_id, edge, neighbour, point, ray.direction, distance - reach, counts)


def _lifted(point: Vec, normal: Vec, lift: float) -> Vec:
    return point if lift == 0.0 else _along(point, normal, lift)


def _swallowed(first, second, sides) -> bool:
    """Отрезок отступа от точки `first` к `second` идёт против стороны, за которой первая точка ведёт путь.

    Две точки фаски одного угла — не отрезок отступа и поглощёнными не считаются.
    """

    if first[2] == second[2]:
        return False
    side = sides[first[2][-1]]
    return _dot(_sub(second[0], first[0]), _sub(side.end, side.start)) <= 0.0


def _untangled(placed: list, sides: tuple, closed: bool, counts: _Counts) -> list:
    """Убирает пологие точки, которые митра соседнего угла оставила позади: сторона короче своего вылета.

    Убранная точка — не потеря: настоящая граница полосы её не содержит. Отрезок «назад» между двумя
    настоящими углами остаётся (патч уже двух ширин: смещённые линии противоположных сторон пересеклись,
    а столкновения превью не решает) и называется: `counts.folds`.
    """

    ring = list(placed[:-1] if closed else placed)
    if closed:
        anchor = next((i for i, item in enumerate(ring) if not item[4]), None)
        if anchor is None:
            return placed
        ring = ring[anchor:] + ring[:anchor]
    index = 0
    while index < len(ring) - 1:
        first, second = ring[index], ring[index + 1]
        if _swallowed(first, second, sides):
            if first[4]:
                del ring[index]
                index = max(index - 1, 0)
                continue
            if second[4]:
                del ring[index + 1]
                continue
            counts.folds += 1
        index += 1
    while closed and len(ring) > 1 and _swallowed(ring[-1], ring[0], sides):
        if not ring[-1][4]:
            counts.folds += 1
            break
        del ring[-1]
    return ring + [ring[0]] if closed else ring


def _straightened(placed: list, closed: bool) -> list:
    """Без точек, в которых путь не поворачивает, и без повторов: прямая через много граней — одна линия."""

    ring = placed[:-1] if closed else placed
    count = len(ring)
    if count < 3:
        return placed
    kept = [ring[0]] if not closed else []
    for index in range(0 if closed else 1, count if closed else count - 1):
        before = _unit(_sub(ring[index][0], ring[index - 1][0]))
        after = _unit(_sub(ring[(index + 1) % count][0], ring[index][0]))
        if before is None or after is None:
            continue
        if _length(_cross(before, after)) <= STRAIGHT_SINE and _dot(before, after) > 0.0:
            continue
        kept.append(ring[index])
    if not closed:
        kept.append(ring[-1])
    return kept + [kept[0]] if closed and kept else kept


def _run_points(inputs: PreviewInputsV1, run: _Run, width: float, counts: _Counts) -> list:
    """Точки отступа пути: `[(точка, нормаль, стороны-источники, оборвана ли, пологая)]`, без подъёма."""

    sides = run.sides
    count = len(sides)
    placed: list = []

    def end_point(ray, side, point, which):
        if ray is None:
            return point, inputs.faces[side.face_id].normal, (which,), False, False
        found = _place(inputs, ray, width, counts)
        return found[0], found[1], (which,), found[2], False

    if not run.closed:
        placed.append(end_point(run.ends[0], sides[0], sides[0].start, 0))
    for index in range(count) if run.closed else range(1, count):
        corner = run.corners[index]
        counts.mitre += int(corner.limited)
        for ray in corner.rays:
            point, normal, clipped = _place(inputs, ray, width, counts)
            placed.append((point, normal, ((index - 1) % count, index), clipped, corner.gentle))
    if not run.closed:
        placed.append(end_point(run.ends[1], sides[-1], sides[-1].end, count - 1))
    elif placed:
        placed.append(placed[0])
    kept = _straightened(_untangled(placed, sides, run.closed, counts), run.closed)
    counts.clipped += sum(1 for item in (kept[:-1] if run.closed else kept) if item[3])
    return kept


def _offset_run(inputs: PreviewInputsV1, run: _Run, width: float, lift: float, counts: _Counts) -> list:
    return [
        _lifted(item[0], item[1], lift) for item in _run_points(inputs, run, width, counts)
    ]


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
        (OUTCOME_STEP_LIMIT, counts.steps),
        (OUTCOME_FOLDS_BACK, counts.folds),
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
    "FAN_EPSILON",
    "GENTLE_TURN",
    "MARCH_STEP_LIMIT",
    "MITRE_LIMIT",
    "OUTCOME_CLIPPED",
    "OUTCOME_DEGENERATE",
    "OUTCOME_FOLDS_BACK",
    "OUTCOME_MITRE_LIMITED",
    "OUTCOME_NO_FACE",
    "OUTCOME_NO_FAN",
    "OUTCOME_STEP_LIMIT",
    "PREVIEW_BINARY64_V1",
    "STRAIGHT_SINE",
    "PreviewInputsV1",
    "WidthPreviewV1",
    "build_preview_inputs",
    "chain_centroid",
    "compute_width_preview",
)
