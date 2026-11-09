"""Крылья UV декали Both (BEVEL_WINGS_V2, BEVEL_FACES_BOTH_V2). Без Blender: numpy.

ВХОД - UV меша декали по углу грани, номер вершины меша угла и грани (начала, домен, владелец). Две части меша:
* КОПИЯ граней бевела (домен `copy_domain`, `bevel_copy`): UV в метрах - `U` вдоль полосы, `S` поперёк от средней линии бевела (`+W/2` и
  `-W/2` на двух рельсах); от ширины не зависят;
* ВНЕШНЯЯ часть - UV ядра под Straight strips (`UV_DIRECT_STRIP_V1`): `v = r / alpha` (точный нуль на цепи-источнике, то есть на рельсе),
  `u` вдоль цепи в единицах alpha, один остров на pChain (группа граней `домен * 2^20 + владелец`).
Ядро не знает ни копии, ни средней линии: всё ниже - хост, одной функцией из всех писателей UV меша (кнопка, подтверждение, кадры живой ширины).

ЗАКОН (средняя линия бевела - средняя линия трима, крылья зеркальны около неё):
* пары рельса: ребро внешней грани с `v = 0` на обоих концах, которое есть и у грани копии (общие вершины меша после сварки), даёт пары
  `(u угла снаружи, (U, S) угла копии в той же вершине, кадр грани копии)`; остров без таких рёбер (веер у угла касается копии вершиной) -
  пара по вершине с `v = 0` (угол копии меньшего кадра), число `BEVEL_WINGS_OUTER_VERTEX_PAIRED`;
* alpha - медиана наклонов `|dU / du|` пар одного острова и одного кадра (наименьшие квадраты): копия в метрах переводится в единицы alpha;
* кадры копии согласуются по островам (BEVEL_WINGS_FRAME_ALIGN_V2): обход в ширину от кадра с наибольшим числом пар; кадр, делящий остров с
  уже положенным, поворачивается на 180 градусов (`U, S -> -U, -S`, ориентация та же), если знак `S` его пар в острове другой, и сдвигается
  вдоль так, что его пары ложатся на прямую `U(u)` пар положенного кадра в этом острове (есть общая вершина рельса двух кадров - `U` в ней
  сводится точно): остров, идущий вдоль нескольких полос, видит одну `U`; поперёк в общей вершине кадры расходятся на разность полуширин.
  Противоречие знака (нечётный цикл) - `BEVEL_WINGS_FRAME_CONFLICTS`;
* копия: `(U / alpha, S / alpha)` после согласования;
* остров снаружи: знак крыла `s` - знак большинства `S` его пар; вдоль - кусочно-линейная функция его `u` по узлам `(u, U / alpha)` (за краями -
  наклон острова), поэтому `u` совпадает с копией на вершинах рельса; поперёк - `w = H(u) + v`, `H(u) = |S| / alpha` по тем же узлам (за
  краями - крайнее значение): на рельсе `s w = S / alpha` - `v` непрерывна через рельс;
* итог: `(U k, 0.5 + S k)` у копии и `(along k, 0.5 + s w k)` снаружи, `k = 1 / (2 (1 + H_med))`, `H_med` - медиана `|S| / alpha` по парам:
  обычная полоса ложится в трим [0, 1] целиком, изотропно.

Каждое отступление названо числом (правило 4): остров без пар - `BEVEL_WINGS_OUTER_UNPAIRED` (`u k`, `0.5 + (H_med + v) k`); пары острова
разных знаков после согласования - `BEVEL_WINGS_NOT_TWO_SIDED`; направление острова против ориентации (грань в UV вывернулась бы) -
`BEVEL_WINGS_OUTER_MIRRORED`; излом кусочно-линейной функции - `BEVEL_WINGS_FOLDS`; наибольший разрыв UV на парах рельса (в микроширинах
alpha) - `BEVEL_WINGS_RAIL_GAP_MAX_MICRO_WIDTHS`; alpha не из чего оценить - `BEVEL_WINGS_ALPHA_UNKNOWN` (alpha = 1: копия в метрах);
разброс наклонов - `BEVEL_WINGS_ALPHA_SPREAD_PPM` (у кривой полосы средняя линия длиннее внутреннего рельса и короче внешнего).
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass

import numpy as np

WINGS_FORMAT = "CFTUV_BEVEL_WINGS"
WINGS_VERSION = 2
#: Свойство МЕША декали: запись крыльев (`wings_to_json`), её кладёт кнопка Both в новый меш. Нет свойства - UV меша не трогаются.
MESH_PROPERTY = "cftuv_bevel_wings"
#: Допуск `v` угла на цепи-источнике (в единицах alpha): UV ядра у источника - точный нуль.
SEAM_V_TOLERANCE = 1e-6
#: Узлы кусочно-линейных функций одного острова с `u` ближе этого - один узел.
KNOT_U_TOLERANCE = 1e-7
GROUP_SHIFT = 20

COUNTER_LOOPS = "BEVEL_WINGS_LOOPS"
COUNTER_COPY_LOOPS = "BEVEL_WINGS_COPY_LOOPS"
COUNTER_ISLANDS = "BEVEL_WINGS_ISLANDS"
COUNTER_RAIL_PAIRS = "BEVEL_WINGS_RAIL_PAIRS"
COUNTER_OUTER_UNPAIRED = "BEVEL_WINGS_OUTER_UNPAIRED"
COUNTER_NOT_TWO_SIDED = "BEVEL_WINGS_NOT_TWO_SIDED"
COUNTER_OUTER_MIRRORED = "BEVEL_WINGS_OUTER_MIRRORED"
COUNTER_FOLDS = "BEVEL_WINGS_FOLDS"
COUNTER_ALPHA_UNKNOWN = "BEVEL_WINGS_ALPHA_UNKNOWN"
COUNTER_ALPHA_SPREAD = "BEVEL_WINGS_ALPHA_SPREAD_PPM"
COUNTER_HALF_BEVEL = "BEVEL_WINGS_HALF_BEVEL_MEDIAN_MICRO_WIDTHS"
COUNTER_VERTEX_PAIRED = "BEVEL_WINGS_OUTER_VERTEX_PAIRED"
COUNTER_FRAME_CONFLICTS = "BEVEL_WINGS_FRAME_CONFLICTS"
COUNTER_RAIL_GAP = "BEVEL_WINGS_RAIL_GAP_MAX_MICRO_WIDTHS"


@dataclass(frozen=True, slots=True)
class WingsRecordV2:
    """Запись крыльев меша декали: номер домена копии граней бевела (`bevel_copy`)."""

    copy_domain: int


def wings_to_json(record: WingsRecordV2) -> str:
    payload = {"format": WINGS_FORMAT, "version": WINGS_VERSION, "copy": int(record.copy_domain)}
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def wings_from_json(text: str) -> WingsRecordV2:
    data = json.loads(text)
    if data.get("format") != WINGS_FORMAT or data.get("version") != WINGS_VERSION:
        raise ValueError(f"not a {WINGS_FORMAT} v{WINGS_VERSION} record")
    return WingsRecordV2(int(data["copy"]))


@dataclass(frozen=True, slots=True)
class WingsPlanV2:
    """Топология крыльев (не зависит от чисел UV, кроме маски нуля `v`): углы копии и их кадры, острова снаружи и пары рельса."""

    copy_loops: np.ndarray
    copy_frame: np.ndarray  # кадр (владелец грани) каждого угла копии
    island_codes: np.ndarray
    island_loops: tuple  # углы каждого острова снаружи
    pair_island: np.ndarray  # остров пары
    pair_outer: np.ndarray  # угол снаружи пары
    pair_copy: np.ndarray  # угол копии в той же вершине
    pair_frame: np.ndarray  # кадр угла копии пары
    pair_vertex: np.ndarray  # вершина меша пары
    vertex_paired: int = 0


def wings_plan(loop_vertex, face_starts, face_domain, face_owner, rail_mask, copy_domain: int) -> WingsPlanV2:
    """Углы копии, острова снаружи и пары рельса: ребро внешней грани с `v = 0` на обоих концах, которое есть и у грани копии; остров без
    таких рёбер - пары по вершинам с `v = 0`, общим с копией."""

    loop_vertex = np.asarray(loop_vertex, dtype=np.int64).reshape(-1)
    face_starts = np.asarray(face_starts, dtype=np.int64).reshape(-1)
    totals = np.diff(face_starts)
    count = int(loop_vertex.shape[0])
    loop_face = np.repeat(np.arange(totals.shape[0], dtype=np.int64), totals)
    following = np.arange(1, count + 1, dtype=np.int64)
    if totals.shape[0]:
        following[face_starts[1:] - 1] = face_starts[:-1]
    domain = np.asarray(face_domain, dtype=np.int64)[loop_face]
    owner = np.asarray(face_owner, dtype=np.int64)[loop_face]
    is_copy = domain == int(copy_domain)
    stride = int(loop_vertex.max(initial=0)) + 1
    a, b = loop_vertex, loop_vertex[following]
    keys = np.minimum(a, b) * stride + np.maximum(a, b)
    copy_loops = np.flatnonzero(is_copy)
    copy_edge = dict(zip(keys[is_copy].tolist(), copy_loops.tolist()))
    group = (domain << GROUP_SHIFT) + owner
    outer = np.flatnonzero(~is_copy)
    codes, inverse = np.unique(group[outer], return_inverse=True)
    island_of = np.full(count, -1, dtype=np.int64)
    island_of[outer] = inverse
    rail = np.asarray(rail_mask, dtype=bool)
    candidates = np.flatnonzero(~is_copy & rail & rail[following])
    seen: dict = {}
    for loop, key in zip(candidates.tolist(), keys[candidates].tolist()):
        partner = copy_edge.get(key)
        if partner is None:
            continue
        for mine in (loop, int(following[loop])):
            seen.setdefault(mine, partner if loop_vertex[partner] == loop_vertex[mine] else int(following[partner]))
    paired = {int(island_of[loop]) for loop in seen}
    at_vertex: dict = {}
    for loop in sorted(copy_loops.tolist(), key=lambda item: (int(owner[item]), item)):
        at_vertex.setdefault(int(loop_vertex[loop]), loop)
    vertex_paired = 0
    for loop in np.flatnonzero(~is_copy & rail).tolist():
        island = int(island_of[loop])
        partner = at_vertex.get(int(loop_vertex[loop]))
        if island not in paired and partner is not None:
            seen.setdefault(loop, partner)
            vertex_paired += 1
    pair_outer = np.asarray(sorted(seen), dtype=np.int64)
    pair_copy = np.asarray([seen[item] for item in pair_outer.tolist()], dtype=np.int64)
    order = np.argsort(inverse, kind="stable")
    islands = tuple(np.split(outer[order], np.cumsum(np.bincount(inverse, minlength=codes.shape[0]))[:-1])) if codes.shape[0] else ()
    return WingsPlanV2(copy_loops, owner[copy_loops], codes, islands, island_of[pair_outer], pair_outer, pair_copy, owner[pair_copy], loop_vertex[pair_outer], vertex_paired)


_PLANS: dict = {}
PLAN_CACHE_SIZE = 8


def cached_wings_plan(loop_vertex, face_starts, face_domain, face_owner, rail_mask, copy_domain: int) -> WingsPlanV2:
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


def _knots(xs, *values):
    """Узлы с `u` ближе `KNOT_U_TOLERANCE` сливаются (значения - средние): `(xs, *значения)` по возрастанию `xs`."""

    order = np.argsort(xs, kind="stable")
    xs = xs[order]
    starts = np.flatnonzero(np.r_[True, np.diff(xs) > KNOT_U_TOLERANCE])
    sizes = np.diff(np.r_[starts, xs.shape[0]])
    ids = np.repeat(np.arange(starts.shape[0]), sizes)
    return (xs[starts], *(np.bincount(ids, weights=item[order]) / sizes for item in values))


def _slope(u, target) -> float | None:
    """Наклон наименьших квадратов `target` по `u`; `None` - станции не тянутся вдоль."""

    if u.shape[0] < 2 or float(np.ptp(u)) <= KNOT_U_TOLERANCE:
        return None
    centred = u - u.mean()
    return float(centred @ (target - target.mean()) / (centred @ centred))


def _align(plan, u, along, across, alpha):
    """Согласование кадров копии по островам (BEVEL_WINGS_FRAME_ALIGN_V2): `({кадр: (поворот +1/-1, сдвиг вдоль, метры)}, противоречия)`."""

    groups: dict = {}
    for row, (island, frame) in enumerate(zip(plan.pair_island.tolist(), plan.pair_frame.tolist())):
        groups.setdefault(island, {}).setdefault(frame, []).append(row)
    islands_of: dict = {}
    for island, frames in groups.items():
        for frame in frames:
            islands_of.setdefault(frame, []).append(island)
    sizes = {frame: sum(len(groups[island][frame]) for island in islands) for frame, islands in islands_of.items()}
    state: dict = {}
    conflicts = 0
    for root in sorted(sizes, key=lambda frame: (-sizes[frame], frame)):
        if root in state:
            continue
        state[root] = (1.0, 0.0)
        queue = [root]
        while queue:
            frame = queue.pop(0)
            flip, shift = state[frame]
            for island in islands_of[frame]:
                rows = np.asarray(groups[island][frame], dtype=np.int64)
                side = 1.0 if flip * float(across[rows].sum()) >= 0.0 else -1.0
                target = flip * along[rows] + shift
                slope = _slope(u[rows], target)
                slope = side * alpha if slope is None else slope
                intercept = float(np.mean(target - slope * u[rows]))
                for other, members in sorted(groups[island].items()):
                    if other == frame:
                        continue
                    members = np.asarray(members, dtype=np.int64)
                    wanted = side * (1.0 if float(across[members].sum()) >= 0.0 else -1.0)
                    if other not in state:
                        shared = np.intersect1d(plan.pair_vertex[rows], plan.pair_vertex[members])
                        if shared.shape[0]:  # общая вершина рельса двух кадров: сдвиг сводит их U в ней точно
                            mine = {int(plan.pair_vertex[row]): float(target[at]) for at, row in enumerate(rows.tolist())}
                            theirs = {int(plan.pair_vertex[row]): float(along[row]) for row in members.tolist()}
                            shift_other = float(np.mean([mine[int(vertex)] - wanted * theirs[int(vertex)] for vertex in shared.tolist()]))
                        else:
                            shift_other = float(np.mean(slope * u[members] + intercept - wanted * along[members]))
                        state[other] = (wanted, shift_other)
                        queue.append(other)
                    elif state[other][0] != wanted:
                        conflicts += 1
    return state, conflicts


def wing_uvs(uvs, loop_vertex, face_starts, face_domain, face_owner, copy_domain: int) -> tuple:
    """`(UV крыльев (L, 2) float64, числа)` по UV меша `uvs` (копия - метры, снаружи - ядро) и топологии меша."""

    uvs = np.asarray(uvs, dtype=np.float64).reshape(-1, 2)
    u, v = uvs[:, 0], uvs[:, 1]
    plan = cached_wings_plan(loop_vertex, face_starts, face_domain, face_owner, np.abs(v) <= SEAM_V_TOLERANCE, copy_domain)
    counters = {
        COUNTER_LOOPS: int(uvs.shape[0]), COUNTER_COPY_LOOPS: int(plan.copy_loops.shape[0]), COUNTER_ISLANDS: len(plan.island_loops),
        COUNTER_RAIL_PAIRS: int(plan.pair_outer.shape[0]), COUNTER_VERTEX_PAIRED: int(plan.vertex_paired),
    }
    pair_u = u[plan.pair_outer]
    raw_along, raw_across = uvs[plan.pair_copy, 0], uvs[plan.pair_copy, 1]
    slopes = []
    for island in np.unique(plan.pair_island).tolist():
        for frame in np.unique(plan.pair_frame[plan.pair_island == island]).tolist():
            rows = np.flatnonzero((plan.pair_island == island) & (plan.pair_frame == frame))
            slope = _slope(pair_u[rows], raw_along[rows])
            if slope is not None and slope != 0.0:
                slopes.append(abs(slope))
    if slopes:
        known = np.asarray(slopes, dtype=np.float64)
        alpha = float(np.median(known))
        counters[COUNTER_ALPHA_SPREAD] = int(round(1e6 * float(np.max(np.abs(known / alpha - 1.0)))))
    else:
        alpha = 1.0
        counters[COUNTER_ALPHA_UNKNOWN] = 1
    state, conflicts = _align(plan, pair_u, raw_along, raw_across, alpha)
    counters[COUNTER_FRAME_CONFLICTS] = conflicts
    flips = np.asarray([state.get(int(frame), (1.0, 0.0))[0] for frame in plan.copy_frame.tolist()], dtype=np.float64)
    shifts = np.asarray([state.get(int(frame), (1.0, 0.0))[1] for frame in plan.copy_frame.tolist()], dtype=np.float64)
    copy_along = np.empty_like(u)
    copy_across = np.empty_like(u)
    copy_along[plan.copy_loops] = (flips * u[plan.copy_loops] + shifts) / alpha
    copy_across[plan.copy_loops] = flips * v[plan.copy_loops] / alpha
    pair_along, pair_across = copy_along[plan.pair_copy], copy_across[plan.pair_copy]
    half = np.abs(pair_across)
    if half.shape[0]:
        half_median = float(np.median(half))
    elif plan.copy_loops.shape[0]:
        half_median = float(np.max(np.abs(copy_across[plan.copy_loops])))
    else:
        half_median = 0.0
    counters[COUNTER_HALF_BEVEL] = int(round(1e6 * half_median))
    scale = 1.0 / (2.0 * (1.0 + half_median))
    out = np.empty_like(uvs)
    out[plan.copy_loops, 0] = copy_along[plan.copy_loops] * scale
    out[plan.copy_loops, 1] = 0.5 + copy_across[plan.copy_loops] * scale
    unpaired = against = mirrored = folds = 0
    for island, loops in enumerate(plan.island_loops):
        rows = np.flatnonzero(plan.pair_island == island)
        if not len(rows):
            unpaired += 1
            out[loops, 0] = u[loops] * scale
            out[loops, 1] = 0.5 + (half_median + v[loops]) * scale
            continue
        signs = np.sign(pair_across[rows])
        side = 1.0 if float(signs.sum()) >= 0.0 else -1.0
        against += int(np.count_nonzero(signs == -side))
        slope = _slope(pair_u[rows], pair_along[rows])
        direction = side if slope is None else (1.0 if slope > 0.0 else -1.0)
        if direction != side:
            mirrored += 1
        xs, ts, hs = _knots(pair_u[rows], pair_along[rows], half[rows])
        if xs.shape[0] > 1 and np.any(np.diff(ts) * direction <= 0.0):
            folds += 1
        along = _piecewise(xs, ts, u[loops], direction if slope is None else slope)
        width = _piecewise(xs, hs, u[loops], None) + v[loops]
        out[loops, 0] = along * scale
        out[loops, 1] = 0.5 + side * width * scale
    gap = float(np.max(np.abs(out[plan.pair_outer] - out[plan.pair_copy]))) / scale if plan.pair_outer.shape[0] else 0.0
    counters.update({
        COUNTER_OUTER_UNPAIRED: unpaired, COUNTER_NOT_TWO_SIDED: against, COUNTER_OUTER_MIRRORED: mirrored, COUNTER_FOLDS: folds,
        COUNTER_RAIL_GAP: int(round(1e6 * gap)),
    })
    return out, counters


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
    "COUNTER_ALPHA_SPREAD",
    "COUNTER_ALPHA_UNKNOWN",
    "COUNTER_COPY_LOOPS",
    "COUNTER_FOLDS",
    "COUNTER_HALF_BEVEL",
    "COUNTER_ISLANDS",
    "COUNTER_LOOPS",
    "COUNTER_NOT_TWO_SIDED",
    "COUNTER_OUTER_MIRRORED",
    "COUNTER_OUTER_UNPAIRED",
    "COUNTER_RAIL_GAP",
    "COUNTER_RAIL_PAIRS",
    "COUNTER_FRAME_CONFLICTS",
    "COUNTER_VERTEX_PAIRED",
    "MESH_PROPERTY",
    "WingsRecordV2",
    "cached_wings_plan",
    "parts_summary",
    "wing_uvs",
    "wings_from_json",
    "wings_plan",
    "wings_to_json",
)
