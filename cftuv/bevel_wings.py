"""Крылья UV декали Both (BEVEL_WINGS_V3, BEVEL_FACES_BOTH_V3). Без Blender: numpy.

ВХОД - UV меша декали по углу грани, номер вершины меша угла, позиции вершин и грани (начала, домен, владелец). Две части меша:
* КОПИЯ граней бевела (домен `copy_domain`, `bevel_copy`): UV в метрах одной системой координат на СЕГМЕНТ (полоса между узлами вместе с
  присоединёнными к ней узлами): `U` - длина дуги средней линии полосы от первой перекладины (направление - BEVEL_SEGMENT_U_ALONG_WORLD_AXIS_V3),
  `S` поперёк от средней линии (`+W/2` и `-W/2` на двух рельсах); владелец грани копии - `2 сегмент + бит узла` (BEVEL_COPY_OWNER_V3);
* ВНЕШНЯЯ часть - UV ядра под Straight strips (`UV_DIRECT_STRIP_V1`): `v = r / alpha` (точный нуль на цепи-источнике, то есть на рельсе),
  `u` вдоль цепи в единицах alpha (параметр точки опоры на рельсе), один остров на pChain (группа граней `домен * 2^20 + владелец`).
Ядро не знает ни копии, ни сегментов: всё ниже - хост, одной функцией из всех писателей UV меша (кнопка, подтверждение, кадры живой ширины).

ЗАКОН (сегмент - ОДИН остров UV, одна система координат из геометрии сегмента; ничего не подгоняется между независимыми системами):
* грань снаружи берёт сегмент (BEVEL_WINGS_SKIRT_SEGMENT_V3): по рёбрам рельса, общим с гранями копии (больше рёбер; ничья - сегмент узла,
  затем меньший номер); без таких рёбер - по вершинам рельса: сегмент этих вершин, который есть у соседа по ребру (слоями, сначала тот же
  остров ядра), иначе сегмент граней узла в вершине (веер угла), иначе граней копии в ней (больше граней, затем меньший номер); без касания
  рельса - слоями от соседей по ребру;
* пары рельса: угол снаружи на рельсе и угол копии ТОГО ЖЕ сегмента в той же вершине (по общему ребру рельса, иначе по вершине) - узлы
  `(u, U, S)` группы «остров ядра x сегмент»;
* юбка полосы (узлы по одну сторону, рельс вдоль средней линии): `U` - кусочно-линейная функция `u` по узлам группы (за краями - `alpha` со
  знаком направления); поперёк - `V = 0.5 + s (k H(u) + v (0.5 - k H(u)))`, `H(u) = |S|` по тем же узлам (за краями - крайнее), `s` - знак `S`
  пар: на рельсе `V` равна `V` копии, на краю полной ширины (`v = 1`) - ровно 0 и 1; профиль шире полосы трима - юбка не уже
  `SKIRT_MIN_WIDTH k alpha` (край за 0 / 1, `BEVEL_WINGS_SKIRT_PAST_TRIM`); соседние узлы с `dU / du` не около `alpha` (перезапуск `u` ядра,
  чужая вершина) делят группу на куски, грань берёт свой кусок (`BEVEL_WINGS_KNOT_BREAKS`);
* параметр ядра за пределами рельса сегмента (BEVEL_WINGS_SKIRT_FOOT_ON_RAIL_V4): угол юбки, у которого `U` по параметру ядра `u`
  расходится с `U` по геометрии (проекция в 3D на прямую отрезка рельса куска, в который попадает `u`; за концом - вдоль крайнего
  отрезка) больше `FOOT_APART` alpha, берёт `(u, v)` по геометрии (`v` - расстояние до той же прямой), с `FOOT_FULL` - целиком, между -
  смесью: у короткого сегмента между узлами и у веера выпуклого угла точка опоры ядра - вне сегмента, юбка съезжала вбок
  (`BEVEL_WINGS_SKIRT_FOOT_MOVED`, углов); вывернула геометрия грань, целую у ядра, - кусок по параметру ядра (`BEVEL_WINGS_SKIRT_FOOT_REVERTED`);
* юбка края узла (рельс поперёк средней линии, BEVEL_WINGS_NODE_SKIRT_V3): точка рельса - кусочно-линейно по узлам `(U, S)`, юбка уходит
  влево от роста `u` на `v alpha` (ориентация карты ядра сохраняется), `BEVEL_WINGS_NODE_SKIRTS`;
* веер (группа с одним узлом: грань касается рельса вершиной и у ядра - своя карта) раскрывается подобием без отражения от соседней грани
  юбки своего сегмента по общему ребру (BEVEL_WINGS_FAN_UNFOLD_V3, `BEVEL_WINGS_FANS_UNFOLDED`), без соседа - сдвиг по узлу;
* угол юбки на рельсе - ровно угол копии своего сегмента (наибольшая поправка названа `BEVEL_WINGS_RAIL_SNAP_MAX_MICRO_UV`);
* копия: `(U k, 0.5 + S k)`; ОДНА плотность на нажатие: `k = 0.5 / (H_med + alpha)` UV на метр, `H_med` - медиана `|S|` пар, `alpha` - медиана
  `длина ребра рельса / |du|` по рёбрам рельса снаружи (ширина декали в метрах); юбка сегмента с `H != H_med` сжата или растянута поперёк,
  число `BEVEL_WINGS_SKIRT_STRETCH_PPM`;
* швы меша - рёбра, где UV двух граней расходится (`seam_pairs`): границы островов (сегментов) и швы ядра внутри юбок.

Каждое отступление названо числом (правило 4): грань снаружи без сегмента и группа без пар - `BEVEL_WINGS_OUTER_UNPAIRED` (граней и групп:
`u alpha k`, `0.5 + k H_med + v (0.5 - k H_med)`); пары группы разных знаков - `BEVEL_WINGS_NOT_TWO_SIDED`; направление группы против стороны
(грань в UV вывернулась бы) - `BEVEL_WINGS_OUTER_MIRRORED`; вывернутые грани юбки - `BEVEL_WINGS_FOLDS` (из них у самого ядра вывернуты -
`BEVEL_WINGS_KERNEL_FLIPPED`), копии - `BEVEL_WINGS_COPY_FLIPPED`; узлы в одной `u` с разной `U` (замкнутое кольцо) - `BEVEL_WINGS_KNOT_CONFLICTS`;
alpha не из чего оценить - `BEVEL_WINGS_ALPHA_UNKNOWN` (alpha = 1); разброс оценки alpha (90-й процентиль отклонения) -
`BEVEL_WINGS_ALPHA_SPREAD_PPM`; рёбра рельса между гранями разных сегментов (шов у узла) - `BEVEL_WINGS_SEGMENT_SEAM_RAIL_EDGES`.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass

import numpy as np

WINGS_FORMAT = "CFTUV_BEVEL_WINGS"
WINGS_VERSION = 3
#: Свойство МЕША декали: запись крыльев (`wings_to_json`), её кладёт кнопка Both в новый меш. Нет свойства - UV меша не трогаются.
MESH_PROPERTY = "cftuv_bevel_wings"
#: Атрибут граней меша декали Both: сегмент грани (`WingsPlanV3.face_segment`, -1 - грань юбки без сегмента). Его пишет каждая запись UV
#: крыльев; по нему Hotspot собирает остров сегмента (BEVEL_FACES_BOTH_HOTSPOT_V1): после отображения в метку UV ядра в меше уже нет.
SEGMENT_ATTRIBUTE = "cftuv_bevel_segment"
#: Допуск `v` угла на цепи-источнике (в единицах alpha): UV ядра у источника - точный нуль.
SEAM_V_TOLERANCE = 1e-6
#: Узлы кусочно-линейных функций одной группы с `u` ближе этого - один узел.
KNOT_U_TOLERANCE = 1e-7
#: Значения UV двух граней в общей вершине, разошедшиеся больше этого, - шов меша.
SEAM_UV_TOLERANCE = 1e-6
GROUP_SHIFT = 20
#: Рельс группы «вдоль», если `sum |dS| <= ALONG_TANGENT sum |dU|` (tan 30 градусов) по её узлам.
ALONG_TANGENT = 0.5773502691896257
#: Соседние узлы вдоль с `dU / du` вне `[RUN_RATIO_LOW, RUN_RATIO_HIGH]` от `alpha` - разрыв куска (`BEVEL_WINGS_KNOT_BREAKS`).
RUN_RATIO_LOW = 0.25
RUN_RATIO_HIGH = 4.0
#: Угол юбки берёт `(u, v)` по геометрии рельса (BEVEL_WINGS_SKIRT_FOOT_ON_RAIL_V4), когда `U` по ней расходится с `U` по параметру ядра:
#: веер выпуклого угла узла (у ядра все его вершины опираются на угловую вершину цепи) и вершина юбки за углом узла (у ядра её опора -
#: рельс вне сегмента). Расхождение до `FOOT_APART` alpha (ширины декали) - параметр ядра (точен на кривом рельсе), от `FOOT_FULL` -
#: геометрия, между - линейная смесь: соседние вершины одной грани не расходятся скачком (крышка над торцом колонки, перпендикулярная
#: рельсу, без искажения в кадр полосы не ложится, у ядра она гладкая). Углов со смесью больше нуля - `BEVEL_WINGS_SKIRT_FOOT_MOVED`;
#: кусок, где геометрия вывернула грань, целую у ядра (клин у излома рельса), целиком берёт параметр ядра -
#: `BEVEL_WINGS_SKIRT_FOOT_REVERTED`, кусков.
FOOT_APART = 0.1
FOOT_FULL = 0.3
#: Наименьшая ширина юбки поперёк в UV - эта доля `k alpha` (её ширины в той же плотности): шире полосы трима профиль - юбка всё равно наружу.
SKIRT_MIN_WIDTH = 0.5
#: Владелец грани копии: `OWNER_SHIFT * сегмент + OWNER_NODE` (те же числа, что `bevel_copy.OWNER_SHIFT` и `bevel_copy.OWNER_NODE`).
OWNER_SHIFT = 2
OWNER_NODE = 1

COUNTER_LOOPS = "BEVEL_WINGS_LOOPS"
COUNTER_COPY_LOOPS = "BEVEL_WINGS_COPY_LOOPS"
COUNTER_SEGMENTS = "BEVEL_WINGS_SEGMENTS"
COUNTER_GROUPS = "BEVEL_WINGS_GROUPS"
COUNTER_RAIL_PAIRS = "BEVEL_WINGS_RAIL_PAIRS"
COUNTER_VERTEX_PAIRED = "BEVEL_WINGS_OUTER_VERTEX_PAIRED"
COUNTER_OUTER_UNPAIRED = "BEVEL_WINGS_OUTER_UNPAIRED"
COUNTER_NOT_TWO_SIDED = "BEVEL_WINGS_NOT_TWO_SIDED"
COUNTER_OUTER_MIRRORED = "BEVEL_WINGS_OUTER_MIRRORED"
COUNTER_FOLDS = "BEVEL_WINGS_FOLDS"
COUNTER_COPY_FLIPPED = "BEVEL_WINGS_COPY_FLIPPED"
COUNTER_KNOT_BREAKS = "BEVEL_WINGS_KNOT_BREAKS"
COUNTER_NODE_SKIRTS = "BEVEL_WINGS_NODE_SKIRTS"
COUNTER_KNOT_CONFLICTS = "BEVEL_WINGS_KNOT_CONFLICTS"
COUNTER_ALPHA_UNKNOWN = "BEVEL_WINGS_ALPHA_UNKNOWN"
COUNTER_ALPHA_SPREAD = "BEVEL_WINGS_ALPHA_SPREAD_PPM"
COUNTER_ALPHA = "BEVEL_WINGS_ALPHA_MICRO_METRES"
COUNTER_HALF_BEVEL = "BEVEL_WINGS_HALF_BEVEL_MEDIAN_MICRO_METRES"
COUNTER_SKIRT_STRETCH = "BEVEL_WINGS_SKIRT_STRETCH_PPM"
COUNTER_SKIRT_PAST_TRIM = "BEVEL_WINGS_SKIRT_PAST_TRIM"
COUNTER_SKIRT_FOOT_MOVED = "BEVEL_WINGS_SKIRT_FOOT_MOVED"
COUNTER_SKIRT_FOOT_REVERTED = "BEVEL_WINGS_SKIRT_FOOT_REVERTED"
COUNTER_FANS_UNFOLDED = "BEVEL_WINGS_FANS_UNFOLDED"
COUNTER_RAIL_GAP = "BEVEL_WINGS_RAIL_SNAP_MAX_MICRO_UV"
COUNTER_KERNEL_FLIPPED = "BEVEL_WINGS_KERNEL_FLIPPED"
COUNTER_SEGMENT_SEAMS = "BEVEL_WINGS_SEGMENT_SEAM_RAIL_EDGES"
COUNTER_SEAMS = "BEVEL_WINGS_SEAM_EDGES"


@dataclass(frozen=True, slots=True)
class WingsRecordV3:
    """Запись крыльев меша декали: номер домена копии граней бевела (`bevel_copy`)."""

    copy_domain: int


def wings_to_json(record: WingsRecordV3) -> str:
    payload = {"format": WINGS_FORMAT, "version": WINGS_VERSION, "copy": int(record.copy_domain)}
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def wings_from_json(text: str) -> WingsRecordV3:
    data = json.loads(text)
    if data.get("format") != WINGS_FORMAT or data.get("version") != WINGS_VERSION:
        raise ValueError(f"not a {WINGS_FORMAT} v{WINGS_VERSION} record")
    return WingsRecordV3(int(data["copy"]))


@dataclass(frozen=True, slots=True)
class WingsPlanV3:
    """Топология крыльев (от чисел UV зависит только через маску нуля `v`): сегменты граней, группы «остров ядра x сегмент», пары рельса, рёбра."""

    copy_loops: np.ndarray
    face_segment: np.ndarray  # сегмент грани (-1 - грань снаружи без сегмента)
    loop_group: np.ndarray  # группа угла снаружи (-1 - угол копии либо грани без сегмента)
    group_count: int
    pair_outer: np.ndarray  # угол снаружи пары
    pair_copy: np.ndarray  # угол копии того же сегмента в той же вершине
    pair_group: np.ndarray
    rail_edges: np.ndarray  # (R, 2) углы концов рёбер рельса снаружи (оценка alpha)
    edge_loops: np.ndarray  # (E, 4) углы двух граней ребра у его концов: (меньшая вершина, большая) первой грани, затем второй
    edge_vertices: np.ndarray  # (E, 2) вершины меша ребра
    unassigned_loops: np.ndarray  # углы граней снаружи без сегмента
    loop_face: np.ndarray  # грань угла
    following: np.ndarray  # следующий угол той же грани
    segments: int = 0
    vertex_paired: int = 0
    unassigned_faces: int = 0
    segment_seams: int = 0


def _edge_table(loop_vertex, following):
    """`(ключи рёбер по углу, {ключ: [углы]}, углы концов рёбер двух граней (E, 4), вершины этих рёбер (E, 2))`."""

    stride = int(loop_vertex.max(initial=0)) + 1
    a, b = loop_vertex, loop_vertex[following]
    keys = np.minimum(a, b) * stride + np.maximum(a, b)
    table: dict = {}
    for loop, key in enumerate(keys.tolist()):
        table.setdefault(key, []).append(loop)
    rows, vertices = [], []
    for key, loops in sorted(table.items()):
        if len(loops) != 2:
            continue
        low = int(min(loop_vertex[loops[0]], loop_vertex[following[loops[0]]]))
        ends = []
        for loop in loops:
            ends.extend((loop, int(following[loop])) if int(loop_vertex[loop]) == low else (int(following[loop]), loop))
        rows.append(ends)
        vertices.append((low, int(max(loop_vertex[loops[0]], loop_vertex[following[loops[0]]]))))
    return keys, table, np.asarray(rows, dtype=np.int64).reshape(-1, 4), np.asarray(vertices, dtype=np.int64).reshape(-1, 2)


def _choose(votes: dict, preferred=frozenset()) -> int:
    return min(votes, key=lambda segment: (-votes[segment], segment not in preferred, segment))


def _neighbour_votes(face, mesh, assigned, allowed=None, any_island=True) -> dict:
    """Голоса сегментов соседей грани по рёбрам (уже с сегментом; `allowed` - только эти сегменты; `any_island=False` - только тот же остров ядра)."""

    face_loops, _vertex, _following, keys, table, loop_face, _copy, _segment, _node, _rail, island_of = mesh
    votes: dict = {}
    for loop in face_loops[face]:
        for other in table.get(int(keys[loop]), ()):
            partner = int(loop_face[other])
            if partner == face or partner not in assigned or (not any_island and island_of[partner] != island_of[face]):
                continue
            if allowed is None or assigned[partner] in allowed:
                votes[assigned[partner]] = votes.get(assigned[partner], 0) + 1
    return votes


def _layers(pending, mesh, assigned, allowed_of=None) -> list:
    """Слоями от соседей с сегментом: сначала соседи того же острова ядра, затем любые; `allowed_of` - сегменты, допустимые грани. Остаток."""

    any_island = False
    while pending:
        chosen = {}
        for face in pending:
            votes = _neighbour_votes(face, mesh, assigned, None if allowed_of is None else allowed_of[face], any_island)
            if votes:
                chosen[face] = _choose(votes, () if allowed_of is None else allowed_of[face][1])
        if not chosen:
            if any_island:
                break
            any_island = True
            continue
        assigned.update(chosen)
        pending = [face for face in pending if face not in chosen]
        any_island = False
    return pending


class _Allowed(tuple):
    """`(сегменты вершин рельса грани, сегменты узлов среди них)`: `in` - по первому."""

    def __contains__(self, item) -> bool:
        return item in self[0]


def _assign_outward(outer_faces, mesh) -> tuple:
    """Сегмент каждой грани снаружи (BEVEL_WINGS_SKIRT_SEGMENT_V3): `({грань: сегмент}, грани без сегмента)`.

    1. рёбра рельса, общие с гранями копии: больше рёбер, ничья - сегмент узла, затем меньший номер;
    2. касание рельса вершинами (веер, треугольник юбки): сегмент вершин, который есть у соседа по ребру (слоями, сначала тот же остров ядра) -
       грань остаётся в одном острове с юбкой рядом; нет такого - сегмент граней узла в вершине, иначе граней копии в ней (больше граней, затем
       меньший номер);
    3. без касания рельса - слоями от соседей по ребру.
    """

    face_loops, loop_vertex, following, keys, table, loop_face, is_copy, segment_of, node_of, rail, _island = mesh
    at_vertex: dict = {}
    for loop in np.flatnonzero(is_copy).tolist():
        at_vertex.setdefault(int(loop_vertex[loop]), []).append(int(loop_face[loop]))
    assigned: dict = {}
    touching: dict = {}
    for face in outer_faces:
        votes: dict = {}
        nodes: set = set()
        for loop in face_loops[face]:
            if not (rail[loop] and rail[following[loop]]):
                continue
            for other in table.get(int(keys[loop]), ()):
                if is_copy[other]:
                    partner = int(loop_face[other])
                    votes[segment_of[partner]] = votes.get(segment_of[partner], 0) + 1
                    if node_of[partner]:
                        nodes.add(segment_of[partner])
        if votes:
            assigned[face] = _choose(votes, nodes)
            continue
        found: dict = {}
        node_found: dict = {}
        for loop in face_loops[face]:
            if rail[loop]:
                copies = at_vertex.get(int(loop_vertex[loop]), ())
                for item in copies:
                    found[segment_of[item]] = found.get(segment_of[item], 0) + 1
                for item in [item for item in copies if node_of[item]]:
                    node_found[segment_of[item]] = node_found.get(segment_of[item], 0) + 1
        if found:
            touching[face] = (found, node_found)
    allowed = {face: _Allowed((frozenset(found), frozenset(node_found))) for face, (found, node_found) in touching.items()}
    left = _layers(sorted(touching), mesh, assigned, allowed)
    for face in left:
        found, node_found = touching[face]
        assigned[face] = _choose(node_found or found)
    pending = [face for face in outer_faces if face not in assigned]
    return assigned, _layers(pending, mesh, assigned)


def _pairs(rail, mesh, face_segment, preceding, face_starts) -> tuple:
    """Пары рельса `{угол снаружи: угол копии}` (по общему ребру рельса, иначе по вершине) и число пар по вершине."""

    _face_loops, loop_vertex, following, keys, table, loop_face, is_copy, segment_of, node_of, _rail, _island = mesh
    by_vertex: dict = {}
    copies = sorted(np.flatnonzero(is_copy).tolist(), key=lambda item: (segment_of[int(loop_face[item])], not node_of[int(loop_face[item])], item))
    for loop in copies:
        by_vertex.setdefault((int(loop_vertex[loop]), segment_of[int(loop_face[loop])]), loop)
    pairs: dict = {}
    vertex_paired = 0
    for loop in np.flatnonzero(rail).tolist():
        segment = int(face_segment[loop_face[loop]])
        if segment < 0:
            continue
        partner = None
        for edge_loop in (loop, int(preceding[loop])):
            if partner is None and rail[edge_loop] and rail[following[edge_loop]]:
                for other in table.get(int(keys[edge_loop]), ()):
                    if is_copy[other] and segment_of[int(loop_face[other])] == segment:
                        partner = other if int(loop_vertex[other]) == int(loop_vertex[loop]) else int(following[other])
        if partner is None:
            partner = by_vertex.get((int(loop_vertex[loop]), segment))
            vertex_paired += partner is not None
        if partner is not None:
            pairs[loop] = partner
    return pairs, vertex_paired


def wings_plan(loop_vertex, face_starts, face_domain, face_owner, rail_mask, copy_domain: int) -> WingsPlanV3:
    """Сегменты граней, группы снаружи, пары рельса и рёбра меша по топологии и маске нуля `v` (углы снаружи на рельсе)."""

    loop_vertex = np.asarray(loop_vertex, dtype=np.int64).reshape(-1)
    face_starts = np.asarray(face_starts, dtype=np.int64).reshape(-1)
    face_domain = np.asarray(face_domain, dtype=np.int64)
    face_owner = np.asarray(face_owner, dtype=np.int64)
    totals = np.diff(face_starts)
    count = int(loop_vertex.shape[0])
    loop_face = np.repeat(np.arange(totals.shape[0], dtype=np.int64), totals)
    following = np.arange(1, count + 1, dtype=np.int64)
    if totals.shape[0]:
        following[face_starts[1:] - 1] = face_starts[:-1]
    preceding = np.empty_like(following)
    preceding[following] = np.arange(count, dtype=np.int64)
    copy_face = face_domain == int(copy_domain)
    is_copy = copy_face[loop_face]
    rail = np.asarray(rail_mask, dtype=bool) & ~is_copy
    keys, table, edge_loops, edge_vertices = _edge_table(loop_vertex, following)
    segment_of = np.where(copy_face, face_owner // OWNER_SHIFT, -1).tolist()
    node_of = (copy_face & (face_owner % OWNER_SHIFT == OWNER_NODE)).tolist()
    island_of = ((face_domain << GROUP_SHIFT) + face_owner).tolist()
    face_loops = [range(int(face_starts[face]), int(face_starts[face + 1])) for face in range(totals.shape[0])]
    mesh = (face_loops, loop_vertex, following, keys, table, loop_face, is_copy, segment_of, node_of, rail, island_of)
    outer_faces = np.flatnonzero(~copy_face).tolist()
    assigned, unassigned = _assign_outward(outer_faces, mesh)
    face_segment = np.asarray([segment_of[face] if copy_face[face] else assigned.get(face, -1) for face in range(totals.shape[0])], dtype=np.int64)
    groups: dict = {}
    loop_group = np.full(count, -1, dtype=np.int64)
    for face in outer_faces:
        if face_segment[face] >= 0:
            loop_group[face_loops[face].start:face_loops[face].stop] = groups.setdefault((island_of[face], int(face_segment[face])), len(groups))
    pairs, vertex_paired = _pairs(rail, mesh, face_segment, preceding, face_starts)
    pair_outer = np.asarray(sorted(pairs), dtype=np.int64)
    pair_copy = np.asarray([pairs[item] for item in pair_outer.tolist()], dtype=np.int64)
    rail_edges = np.flatnonzero(rail & rail[following])
    face_a, face_b = loop_face[edge_loops[:, 0]], loop_face[edge_loops[:, 2]]
    segment_seams = int(np.count_nonzero((copy_face[face_a] != copy_face[face_b]) & (face_segment[face_a] != face_segment[face_b])))
    return WingsPlanV3(
        np.flatnonzero(is_copy), face_segment, loop_group, len(groups), pair_outer, pair_copy, loop_group[pair_outer],
        np.stack((rail_edges, following[rail_edges]), axis=1), edge_loops, edge_vertices,
        np.flatnonzero(~is_copy & (face_segment[loop_face] < 0)), loop_face, following, len({item for item in segment_of if item >= 0}), vertex_paired,
        len(unassigned),
        segment_seams,
    )


_PLANS: dict = {}
PLAN_CACHE_SIZE = 8


def cached_wings_plan(loop_vertex, face_starts, face_domain, face_owner, rail_mask, copy_domain: int) -> WingsPlanV3:
    """План зависит от топологии меша и маски нуля `v`: кадры живой ширины берут его из памяти. Ключ - байты массивов и домен копии."""

    parts = (
        np.ascontiguousarray(loop_vertex, dtype=np.int64).tobytes(), np.ascontiguousarray(face_starts, dtype=np.int64).tobytes(),
        np.ascontiguousarray(face_domain, dtype=np.int64).tobytes(), np.ascontiguousarray(face_owner, dtype=np.int64).tobytes(),
        np.packbits(np.asarray(rail_mask, dtype=bool)).tobytes(), str(int(copy_domain)).encode("ascii"),
    )
    key = hashlib.blake2b(b"|".join(parts), digest_size=16).digest()
    plan = _PLANS.get(key)
    if plan is None:
        plan = wings_plan(loop_vertex, face_starts, face_domain, face_owner, rail_mask, copy_domain)
        if len(_PLANS) >= PLAN_CACHE_SIZE:
            _PLANS.pop(next(iter(_PLANS)))
        _PLANS[key] = plan
    return plan


def _piecewise(xs, ts, x, slope):
    """Кусочно-линейная по узлам `(xs, ts)` (xs по возрастанию, без повторов) в точках `x`; за краями - наклон `slope` (`None` - крайнее значение)."""

    if xs.shape[0] == 1:
        return ts[0] + (0.0 if slope is None else slope) * (x - xs[0])
    mapped = np.interp(x, xs, ts)
    if slope is not None:
        mapped = np.where(x < xs[0], ts[0] + slope * (x - xs[0]), mapped)
        mapped = np.where(x > xs[-1], ts[-1] + slope * (x - xs[-1]), mapped)
    return mapped


def _knots(xs, along, height, where=None):
    """Узлы с `u` ближе `KNOT_U_TOLERANCE` сливаются (значения - средние): `(xs, U, S, конфликтов)` по возрастанию `xs`; конфликт - слитые узлы
    с разной `U`. С `where` (позиции пар, `(K, 3)`) - ещё средние позиции узлов пятым числом."""

    order = np.argsort(xs, kind="stable")
    xs, along, height = xs[order], along[order], height[order]
    starts = np.flatnonzero(np.r_[True, np.diff(xs) > KNOT_U_TOLERANCE])
    sizes = np.diff(np.r_[starts, xs.shape[0]])
    ids = np.repeat(np.arange(starts.shape[0]), sizes)
    spread = np.maximum.reduceat(along, starts) - np.minimum.reduceat(along, starts)
    conflicts = int(np.count_nonzero(spread > 1e-9 * (1.0 + np.abs(along[starts]))))
    merged = (xs[starts], np.bincount(ids, weights=along) / sizes, np.bincount(ids, weights=height) / sizes, conflicts)
    if where is None:
        return merged
    where = where[order]
    return (*merged, np.stack([np.bincount(ids, weights=where[:, axis]) / sizes for axis in range(3)], axis=1))


def _foot_u(points, rail, xs, ts, at, direction, alpha):
    """`u` угла юбки по геометрии (BEVEL_WINGS_SKIRT_FOOT_ON_RAIL_V4): проекция точки в 3D на прямую того отрезка ломаной рельса куска (узлы
    `rail` по возрастанию `xs`), в который попадает параметр ядра `at` (за краями - крайний отрезок); `u` - линейно по доле отрезка, за концом
    ломаной - сдвиг вдоль крайнего отрезка в метрах, переведённый в `u` так, что продолжение `U` за краем (`direction alpha` на единицу `u`)
    даёт `U` крайнего отрезка на метр; `v` - расстояние до прямой того же отрезка в единицах alpha (у ядра `v` веера - расстояние до угловой
    вершины, с `U` по прямой рельса оно сплющило бы веер; не больше 1 - край декали на 0 / 1 трима). Отрезок выбирает параметр ядра, а не ближайшая точка: на кривом рельсе ближайшая
    точка ломаной рвётся у её вершин, а параметр ядра там точен. `(u, v)`."""

    count = xs.shape[0] - 1
    seg = np.clip(np.searchsorted(xs, at, side="right") - 1, 0, count - 1)
    start, chord = rail[seg], rail[seg + 1] - rail[seg]
    length2 = np.einsum("ij,ij->i", chord, chord)
    good = length2 > 0.0
    t = np.where(good, np.einsum("ij,ij->i", points - start, chord) / np.where(good, length2, 1.0), 0.0)
    mapped = xs[seg] + t * (xs[seg + 1] - xs[seg])
    length = np.sqrt(length2)
    per_metre = (ts[seg + 1] - ts[seg]) / np.where(good, length, 1.0)
    before, after = good & (seg == 0) & (t < 0.0), good & (seg == count - 1) & (t > 1.0)
    mapped = np.where(before, xs[0] + t * length * per_metre / (direction * alpha), mapped)
    mapped = np.where(after, xs[-1] + (t - 1.0) * length * per_metre / (direction * alpha), mapped)
    return mapped, np.minimum(np.linalg.norm(points - start - t[:, None] * chord, axis=1) / alpha, 1.0)


def _alpha(plan, u, positions, loop_vertex, counters) -> float:
    """Ширина декали в метрах: медиана `длина ребра рельса снаружи / |du|` (на рельсе `u` - длина дуги цепи в единицах alpha)."""

    if plan.rail_edges.shape[0]:
        du = np.abs(u[plan.rail_edges[:, 0]] - u[plan.rail_edges[:, 1]])
        lengths = np.linalg.norm(positions[loop_vertex[plan.rail_edges[:, 0]]] - positions[loop_vertex[plan.rail_edges[:, 1]]], axis=1)
        keep = du > KNOT_U_TOLERANCE
        if np.any(keep):
            ratios = lengths[keep] / du[keep]
            alpha = float(np.median(ratios))
            if alpha > 0.0:
                counters[COUNTER_ALPHA_SPREAD] = int(round(1e6 * float(np.percentile(np.abs(ratios / alpha - 1.0), 90))))
                return alpha
    counters[COUNTER_ALPHA_UNKNOWN] = 1
    return 1.0


def _flat(out, loops, u, v, alpha, scale, half_median) -> None:
    """Угол снаружи без пар: полоса ядра как есть в той же плотности, у края 0 / 1 (`BEVEL_WINGS_OUTER_UNPAIRED`)."""

    out[loops, 0] = u[loops] * alpha * scale
    out[loops, 1] = 0.5 + half_median * scale + v[loops] * (0.5 - half_median * scale)


def _is_along(xs, ts, ss, side) -> bool:
    """Рельс группы идёт вдоль средней линии (юбка полосы): все узлы по одну сторону и `sum |dS| <= tan 30 sum |dU|`; иначе - поперёк (юбка
    края узла, BEVEL_WINGS_NODE_SKIRT_V3)."""

    if np.any(ss * side <= 0.0):
        return False
    return xs.shape[0] < 2 or float(np.abs(np.diff(ss)).sum()) <= ALONG_TANGENT * float(np.abs(np.diff(ts)).sum())


def _runs(xs, ts, direction, alpha):
    """Куски узлов вдоль: разрыв между соседними узлами, у которых `dU / du` не в `[1/4, 4]` от `direction alpha` (перезапуск `u` ядра на
    замкнутой цепи, чужая вершина): `[(начало, конец)]` номеров узлов."""

    if xs.shape[0] < 2:
        return [(0, xs.shape[0])]
    ratio = np.diff(ts) / np.diff(xs) / (direction * alpha)
    cuts = np.flatnonzero((ratio < RUN_RATIO_LOW) | (ratio > RUN_RATIO_HIGH)) + 1
    edges = [0, *cuts.tolist(), xs.shape[0]]
    return list(zip(edges, edges[1:]))


def _along(out, plan, group, knots, u, v, alpha, scale, side, points) -> tuple:
    """Закон юбки полосы: `U` кусочно-линейно по узлам куска (грань берёт кусок, где больше её узлов, иначе ближайший по `u`), `V` - от
    рельса до края 0 / 1; где профиль шире полосы трима (`k H > 0.5 - SKIRT_MIN_WIDTH k alpha`), юбка не выворачивается: её ширина в UV не
    меньше `SKIRT_MIN_WIDTH k alpha`, край выходит за 0 / 1 (`BEVEL_WINGS_SKIRT_PAST_TRIM`, углов). `group` - `(углы группы, номера пар,
    u пар)`. Угол, у которого параметр ядра уводит `U` от рельса сегмента, берёт `(u, v)` по геометрии (BEVEL_WINGS_SKIRT_FOOT_ON_RAIL_V4,
    `_foot_u`, смесь по `FOOT_APART` / `FOOT_FULL`); вывернула она грань, целую у ядра, - кусок по параметру ядра. `(разрывов, против стороны,
    углов за тримом, углов по геометрии, откатов)`."""

    xs, ts, hs, rail = knots
    loops, rows, pair_u = group
    slopes = np.diff(ts) / np.diff(xs) if xs.shape[0] > 1 else np.zeros(0)
    direction = side if not slopes.shape[0] else (1.0 if float(np.median(slopes)) > 0.0 else -1.0)
    runs = _runs(xs, ts, direction, alpha)
    pieces = [(loops, runs[0])] if len(runs) == 1 else _pieces(plan, loops, pair_u, rows, xs, runs, u)
    past = moved = reverted = 0
    for chosen, (a, b) in pieces:
        kernel = (u[chosen], v[chosen])
        beyond = _place(out, chosen, (xs[a:b], ts[a:b], hs[a:b]), kernel, direction, alpha, scale, side)
        if b - a < 2:
            past += beyond
            continue
        foot, off = _foot_u(points[chosen], rail[a:b], xs[a:b], ts[a:b], kernel[0], direction, alpha)
        slope = direction * alpha
        apart = np.abs(_piecewise(xs[a:b], ts[a:b], foot, slope) - _piecewise(xs[a:b], ts[a:b], kernel[0], slope)) / alpha
        weight = np.clip((apart - FOOT_APART) / (FOOT_FULL - FOOT_APART), 0.0, 1.0)
        if not np.any(weight > 0.0):
            past += beyond
            continue
        before = _face_areas(out, plan, chosen)
        held = out[chosen].copy()
        geometric = (kernel[0] + weight * (foot - kernel[0]), kernel[1] + weight * (off - kernel[1]))
        moved_beyond = _place(out, chosen, (xs[a:b], ts[a:b], hs[a:b]), geometric, direction, alpha, scale, side)
        after = _face_areas(out, plan, chosen)
        if np.any((after <= 0.0) & (before > 0.0)):
            out[chosen] = held  # геометрия вывернула грань, целую у ядра: кусок - по параметру ядра (BEVEL_WINGS_SKIRT_FOOT_REVERTED)
            reverted += 1
            past += beyond
            continue
        moved += int(np.count_nonzero(weight > 0.0))
        past += moved_beyond
    return len(runs) - 1, int(direction != side), past, moved, reverted


def _place(out, chosen, knots, at_across, direction, alpha, scale, side) -> int:
    """`U, V` углов `chosen` юбки полосы по `(u, v)` и узлам куска `(xs, U, |S|)`; углов за тримом."""

    xs, ts, hs = knots
    at, across = at_across
    height = _piecewise(xs, hs, at, None) * scale
    width = np.maximum(0.5 - height, SKIRT_MIN_WIDTH * alpha * scale)
    out[chosen, 0] = _piecewise(xs, ts, at, direction * alpha) * scale
    out[chosen, 1] = 0.5 + side * (height + across * width)
    return int(np.count_nonzero(width > 0.5 - height))


def _face_areas(out, plan, chosen):
    """Удвоенные площади со знаком граней углов `chosen` (грани куска целиком в нём) по текущим `out`, по номерам граней этих углов."""

    nxt = out[plan.following[chosen]]
    here = out[chosen]
    faces, inverse = np.unique(plan.loop_face[chosen], return_inverse=True)
    return np.bincount(inverse, weights=here[:, 0] * nxt[:, 1] - nxt[:, 0] * here[:, 1], minlength=faces.shape[0])


def _pieces(plan, loops, pair_u, rows, xs, runs, u) -> list:
    """Углы группы по кускам узлов: грань берёт кусок, где больше её узлов пар, иначе ближайший по средней `u`; `[(углы, кусок)]`."""

    faces: dict = {}
    for loop in loops.tolist():
        faces.setdefault(int(plan.loop_face[loop]), []).append(loop)
    paired: dict = {}
    for row in rows.tolist():
        paired.setdefault(int(plan.loop_face[plan.pair_outer[row]]), []).append(float(pair_u[row]))
    chosen: dict = {}
    for face, items in sorted(faces.items()):
        mine = np.asarray(paired.get(face, ()), dtype=np.float64)
        votes = [int(np.count_nonzero((mine >= xs[a] - KNOT_U_TOLERANCE) & (mine <= xs[b - 1] + KNOT_U_TOLERANCE))) for a, b in runs]
        if max(votes) > 0:
            run = int(np.argmax(votes))
        else:
            middle = float(u[items].mean())
            run = int(np.argmin([max(xs[a] - middle, middle - xs[b - 1], 0.0) for a, b in runs]))
        chosen.setdefault(run, []).extend(items)
    return [(np.asarray(items, dtype=np.int64), runs[run]) for run, items in sorted(chosen.items())]


def _across(out, loops, knots_s, u, v, alpha, scale) -> None:
    """Закон юбки края узла (рельс поперёк средней линии): точка рельса `R(u)` - кусочно-линейно по узлам `(U, S)`, юбка уходит влево от
    направления роста `u` (поворот касательной на +90 градусов, как в карте ядра: ориентация сохраняется) на `v alpha`; нормаль в узле -
    средняя нормалей соседних отрезков, между узлами - линейно."""

    xs, ts, ss = knots_s
    if xs.shape[0] < 2:
        out[loops, 0] = (ts[0] + alpha * (u[loops] - xs[0])) * scale
        out[loops, 1] = 0.5 + (ss[0] + alpha * v[loops]) * scale
        return
    chord = np.stack((np.diff(ts), np.diff(ss)), axis=1)
    tangent = chord / np.maximum(np.linalg.norm(chord, axis=1), 1e-30)[:, None]
    normal = np.stack((-tangent[:, 1], tangent[:, 0]), axis=1)
    at_knot = np.vstack((normal[:1], normal[:-1] + normal[1:], normal[-1:]))
    at_knot /= np.maximum(np.linalg.norm(at_knot, axis=1), 1e-30)[:, None]
    x = u[loops]
    first, last = chord[0] / (xs[1] - xs[0]), chord[-1] / (xs[-1] - xs[-2])
    rail = np.stack((np.interp(x, xs, ts), np.interp(x, xs, ss)), axis=1)
    rail = np.where((x < xs[0])[:, None], np.array([ts[0], ss[0]]) + (x - xs[0])[:, None] * first, rail)
    rail = np.where((x > xs[-1])[:, None], np.array([ts[-1], ss[-1]]) + (x - xs[-1])[:, None] * last, rail)
    side = np.stack((np.interp(x, xs, at_knot[:, 0]), np.interp(x, xs, at_knot[:, 1])), axis=1)
    side /= np.maximum(np.linalg.norm(side, axis=1), 1e-30)[:, None]
    point = rail + (v[loops] * alpha)[:, None] * side
    out[loops, 0] = point[:, 0] * scale
    out[loops, 1] = 0.5 + point[:, 1] * scale


def _unfold_fans(out, plan, u, v, fans) -> int:
    """Веер (группа с одним узлом: грань юбки касается рельса вершиной, у ядра она - своя карта) продолжает соседнюю юбку своего сегмента
    (BEVEL_WINGS_FAN_UNFOLD_V3): грань с общим ребром с уже отображённой гранью того же сегмента берёт подобие без отражения из своей карты
    ядра, точное в двух вершинах этого ребра (обход слоями: веер из нескольких граней раскрывается от соседа). Без такого соседа - сдвиг по
    узлу, как прежде. Раскрыто граней."""

    first, second = plan.loop_face[plan.edge_loops[:, 0]], plan.loop_face[plan.edge_loops[:, 2]]
    by_face: dict = {}
    for row, (a, b) in enumerate(zip(first.tolist(), second.tolist())):
        by_face.setdefault(a, []).append((row, 0))
        by_face.setdefault(b, []).append((row, 2))
    pending = set(fans)
    done = set(np.flatnonzero(plan.face_segment >= 0).tolist()) - pending
    unfolded = 0
    while pending:
        placed = []
        for face in sorted(pending):
            for row, at in by_face.get(face, ()):
                other_at = 2 - at
                other = int(plan.loop_face[plan.edge_loops[row, other_at]])
                if other not in done or plan.face_segment[other] != plan.face_segment[face]:
                    continue
                mine, theirs = plan.edge_loops[row, at:at + 2], plan.edge_loops[row, other_at:other_at + 2]
                source = u[mine] + 1j * v[mine]
                target = out[theirs, 0] + 1j * out[theirs, 1]
                if abs(source[1] - source[0]) <= KNOT_U_TOLERANCE:
                    continue
                factor = (target[1] - target[0]) / (source[1] - source[0])
                loops = np.flatnonzero(plan.loop_face == face)
                mapped = factor * (u[loops] + 1j * v[loops] - source[0]) + target[0]
                out[loops, 0], out[loops, 1] = mapped.real, mapped.imag
                placed.append(face)
                break
        if not placed:
            break
        done.update(placed)
        pending.difference_update(placed)
        unfolded += len(placed)
    return unfolded


def _skirt(out, plan, u, v, along, across, alpha, scale, half_median, counters, points) -> None:
    """Юбки (углы снаружи) по группам «остров ядра x сегмент»: вдоль (юбка полосы) либо поперёк (юбка края узла)."""

    pair_u = u[plan.pair_outer]
    unpaired = against = mirrored = breaks = conflicts = nodes = past = moved = reverted = 0
    stretch = 0.0
    fans: set = set()
    order = np.argsort(plan.loop_group, kind="stable")
    bounds = np.searchsorted(plan.loop_group[order], np.arange(plan.group_count + 1))
    rows_order = np.argsort(plan.pair_group, kind="stable")
    row_bounds = np.searchsorted(plan.pair_group[rows_order], np.arange(plan.group_count + 1))
    for group in range(plan.group_count):
        loops = order[bounds[group]:bounds[group + 1]]
        rows = rows_order[row_bounds[group]:row_bounds[group + 1]]
        if not rows.shape[0]:
            unpaired += 1
            _flat(out, loops, u, v, alpha, scale, half_median)
            continue
        side = 1.0 if float(across[rows].sum()) >= 0.0 else -1.0
        xs, ts, ss, found, rail = _knots(pair_u[rows], along[rows], across[rows], points[plan.pair_copy[rows]])
        conflicts += found
        if not _is_along(xs, ts, ss, side):
            nodes += 1
            _across(out, loops, (xs, ts, ss), u, v, alpha, scale)
            continue
        against += int(np.count_nonzero(np.sign(across[rows]) == -side))
        if xs.shape[0] == 1:
            fans.update(int(face) for face in np.unique(plan.loop_face[loops]).tolist())
        cut, wrong, beyond, refooted, undone = _along(out, plan, (loops, rows, pair_u), (xs, ts, np.abs(ss), rail), u, v, alpha, scale, side, points)
        moved += refooted
        reverted += undone
        breaks += cut
        mirrored += wrong
        past += beyond
        stretch = max(stretch, abs((0.5 - float(np.abs(ss).mean()) * scale) / (alpha * scale) - 1.0))
    if plan.unassigned_loops.shape[0]:
        _flat(out, plan.unassigned_loops, u, v, alpha, scale, half_median)
    counters[COUNTER_FANS_UNFOLDED] = _unfold_fans(out, plan, u, v, fans) if fans else 0
    counters.update({
        COUNTER_OUTER_UNPAIRED: unpaired + plan.unassigned_faces, COUNTER_NOT_TWO_SIDED: against, COUNTER_OUTER_MIRRORED: mirrored,
        COUNTER_KNOT_BREAKS: breaks, COUNTER_KNOT_CONFLICTS: conflicts, COUNTER_NODE_SKIRTS: nodes, COUNTER_SKIRT_STRETCH: int(round(1e6 * stretch)),
        COUNTER_SKIRT_PAST_TRIM: past, COUNTER_SKIRT_FOOT_MOVED: moved, COUNTER_SKIRT_FOOT_REVERTED: reverted,
    })


def flipped_faces(mapped, plan: WingsPlanV3) -> tuple:
    """`(граней копии, граней снаружи)`, у которых UV крыльев по часовой (площадь со знаком меньше нуля)."""

    mapped = np.asarray(mapped, dtype=np.float64).reshape(-1, 2)
    nxt = mapped[plan.following]
    twice = np.bincount(plan.loop_face, weights=mapped[:, 0] * nxt[:, 1] - nxt[:, 0] * mapped[:, 1], minlength=plan.face_segment.shape[0])
    copy = np.zeros(plan.face_segment.shape[0], dtype=bool)
    copy[plan.loop_face[plan.copy_loops]] = True
    flipped = twice < 0.0
    return int(np.count_nonzero(flipped & copy)), int(np.count_nonzero(flipped & ~copy))


def wing_uvs(uvs, loop_vertex, face_starts, face_domain, face_owner, copy_domain: int, positions) -> tuple:
    """`(UV крыльев (L, 2) float64, числа, план)` по UV меша `uvs` (копия - метры, снаружи - ядро), позициям вершин и топологии меша."""

    uvs = np.asarray(uvs, dtype=np.float64).reshape(-1, 2)
    loop_vertex = np.asarray(loop_vertex, dtype=np.int64).reshape(-1)
    positions = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    u, v = uvs[:, 0], uvs[:, 1]
    plan = cached_wings_plan(loop_vertex, face_starts, face_domain, face_owner, np.abs(v) <= SEAM_V_TOLERANCE, copy_domain)
    counters = {
        COUNTER_LOOPS: int(uvs.shape[0]), COUNTER_COPY_LOOPS: int(plan.copy_loops.shape[0]), COUNTER_SEGMENTS: plan.segments,
        COUNTER_GROUPS: plan.group_count, COUNTER_RAIL_PAIRS: int(plan.pair_outer.shape[0]), COUNTER_VERTEX_PAIRED: plan.vertex_paired,
        COUNTER_SEGMENT_SEAMS: plan.segment_seams,
    }
    alpha = _alpha(plan, u, positions, loop_vertex, counters)
    along, across = uvs[plan.pair_copy, 0], uvs[plan.pair_copy, 1]
    if across.shape[0]:
        half_median = float(np.median(np.abs(across)))
    elif plan.copy_loops.shape[0]:
        half_median = float(np.max(np.abs(v[plan.copy_loops])))
    else:
        half_median = 0.0
    scale = 0.5 / (half_median + alpha)
    counters[COUNTER_ALPHA] = int(round(1e6 * alpha))
    counters[COUNTER_HALF_BEVEL] = int(round(1e6 * half_median))
    out = np.empty_like(uvs)
    out[plan.copy_loops, 0] = u[plan.copy_loops] * scale
    out[plan.copy_loops, 1] = 0.5 + v[plan.copy_loops] * scale
    _skirt(out, plan, u, v, along, across, alpha, scale, half_median, counters, positions[loop_vertex])
    gap = float(np.max(np.abs(out[plan.pair_outer] - out[plan.pair_copy]))) if plan.pair_outer.shape[0] else 0.0
    counters[COUNTER_RAIL_GAP] = int(round(1e6 * gap))
    out[plan.pair_outer] = out[plan.pair_copy]  # точно в каждой вершине рельса: угол юбки на рельсе - угол копии своего сегмента
    counters[COUNTER_COPY_FLIPPED], counters[COUNTER_FOLDS] = flipped_faces(out, plan)
    counters[COUNTER_KERNEL_FLIPPED] = flipped_faces(uvs, plan)[1]
    return out, counters, plan


def seam_pairs(mapped, plan: WingsPlanV3) -> np.ndarray:
    """Швы меша по UV крыльев: рёбра двух граней, где UV граней в одном из концов расходится (`SEAM_UV_TOLERANCE`); `(S, 2)` пары вершин."""

    loops = plan.edge_loops
    if not loops.shape[0]:
        return np.zeros((0, 2), dtype=np.int64)
    mapped = np.asarray(mapped, dtype=np.float64).reshape(-1, 2)
    gap = np.maximum(np.abs(mapped[loops[:, 0]] - mapped[loops[:, 2]]).max(axis=1), np.abs(mapped[loops[:, 1]] - mapped[loops[:, 3]]).max(axis=1))
    return plan.edge_vertices[gap > SEAM_UV_TOLERANCE]


def parts_summary(copy_faces: int, built, skipped) -> str:
    """Строка частей нажатия Both: копия граней бевела (число граней) и внешняя часть (домены ядра в меше из всех, отказы по имени).

    Отказ одного домена не снимает другие (как у пути по рёбрам): построенное остаётся в меше, отказанное названо.
    """

    refused: dict = {}
    for _patch, _domain, outcome, _detail in skipped:
        refused[outcome] = refused.get(outcome, 0) + 1
    done = len(tuple(built))
    text = f"both: copy {int(copy_faces)} faces, outward {done}/{done + sum(refused.values())}"
    if refused:
        text += " refused (" + ", ".join(f"{name} {count}" for name, count in sorted(refused.items())) + ")"
    return text


__all__ = (
    "COUNTER_ALPHA",
    "COUNTER_ALPHA_SPREAD",
    "COUNTER_ALPHA_UNKNOWN",
    "COUNTER_COPY_FLIPPED",
    "COUNTER_COPY_LOOPS",
    "COUNTER_FANS_UNFOLDED",
    "COUNTER_FOLDS",
    "COUNTER_GROUPS",
    "COUNTER_HALF_BEVEL",
    "COUNTER_KERNEL_FLIPPED",
    "COUNTER_KNOT_BREAKS",
    "COUNTER_KNOT_CONFLICTS",
    "COUNTER_LOOPS",
    "COUNTER_NOT_TWO_SIDED",
    "COUNTER_NODE_SKIRTS",
    "COUNTER_OUTER_MIRRORED",
    "COUNTER_OUTER_UNPAIRED",
    "COUNTER_RAIL_GAP",
    "COUNTER_RAIL_PAIRS",
    "COUNTER_SEAMS",
    "COUNTER_SEGMENTS",
    "COUNTER_SEGMENT_SEAMS",
    "COUNTER_SKIRT_PAST_TRIM",
    "COUNTER_SKIRT_STRETCH",
    "COUNTER_VERTEX_PAIRED",
    "MESH_PROPERTY",
    "SEGMENT_ATTRIBUTE",
    "WingsPlanV3",
    "WingsRecordV3",
    "cached_wings_plan",
    "flipped_faces",
    "parts_summary",
    "seam_pairs",
    "wing_uvs",
    "wings_from_json",
    "wings_plan",
    "wings_to_json",
)
