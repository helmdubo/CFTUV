"""Крылья UV декали Both (BEVEL_WINGS_V1, BEVEL_FACES_BOTH_V1). Без Blender: numpy.

ВХОД - UV ядра под Straight strips (`UV_DIRECT_STRIP_V1`) по углу грани: `v = r / alpha` (0 на цепи-источнике), `u` вдоль цепи в единицах alpha,
один остров на pChain; номер вершины меша угла; группа угла `домен * 2^20 + владелец` (сторона цепи: домен и огибающая грани); и номера
доменов ГРАНЕЙ БЕВЕЛА (полосы и узлы; остальные домены - грани снаружи). Ядро не знает ни крыльев, ни средней линии: всё ниже - хост,
одной функцией из всех писателей UV меша декали (кнопка, подтверждение, кадры живой ширины), как отображение Hotspot.

ЗАКОН (средняя линия бевела - средняя линия трима, у каждой стороны крыло, крылья зеркальны около средней линии):
* связи групп по ОБЩИМ вершинам меша (не меньше двух): рельс `R` - группа граней бевела и группа снаружи, обе с `v = 0` на общих вершинах (две
  стороны рельса); разрез `X` - две группы бевела разных доменов с `v = 0` (полоса | узел); средняя `C` - две группы одного домена бевела (где
  встречаются фронты); группа бевела с рельсом - РЕЛЬСОВАЯ, группа снаружи с рельсом - ВНЕШНЯЯ;
* знак крыла `s` (+1 / -1): обход в ширину по `R` (тот же знак) и `C` между рельсовыми одного домена (знак меняется); компонента - одна лента;
* поперёк: рельсовая `w = H(u) - v`, внешняя `w = H(u) + v`; `H(u)` рельсовой - `v` её вершин средней линии (кусочно-линейно по `u`, за краями -
  крайнее значение), внешней - `H` рельсовой в общих вершинах рельса. Поэтому на средней линии `w = 0` у обоих крыльев, на рельсе `w` внутри
  равна `w` снаружи: `v` непрерывна через рельс и средняя линия общая;
* вдоль: направление ЗАДАНО сохранением ориентации (карты ядра все против часовой, грань не выворачивается): рельсовая `-s u`, внешняя `+s u`;
  сдвиг - обход по тем же связям (среднее по общим вершинам), у каждой общей вершины цель - среднее предложений её групп в ленте, `along`
  группы - кусочно-линейная по её `u` между узлами `(u, цель)`, за краями - наклон направления: `u` совпадает на КАЖДОЙ общей вершине средней
  линии и рельса, как у метки corner (HOTSPOT_ATLAS_MAPPING_V1);
* остальные группы бевела (концы полос у разреза, узлы, свободные концы) кладутся ЖЁСТКО (поворот и сдвиг, без отражения) по целям общих
  вершин уже положенных групп: сначала рельсовые того же домена (конец полосы продолжает свою полосу), иначе группа по ту сторону разреза
  (узел продолжает полосу, к которой примыкает: BEVEL_WINGS_NODE_CONTINUES_STRIP_V1), иначе любая положенная того же домена. Где в узле
  встречаются области разных полос, UV разрывна (шов между лентами), как и между островами ядра;
* итог: `(U k, 0.5 + s w k)`, `k = 1 / (2 (1 + H_med))`, `H_med` - медиана `H` по узлам средней линии: обычная полоса ложится в трим [0, 1]
  целиком, изотропно (единица `u` и `v` одна).

Каждое отступление названо числом (правило 4): рельсовая без средней линии (фронты не встретились: alpha меньше половины полосы) -
`BEVEL_WINGS_CENTRE_UNMET`, `H` = наибольшая `v` группы; внешняя без рельса - `BEVEL_WINGS_OUTER_UNPAIRED`, `H = H_med`; конфликт знаков -
`BEVEL_WINGS_NOT_TWO_SIDED`; связь, где станции идут в одну сторону, - `BEVEL_WINGS_ALONG_AGAINST`; излом кусочно-линейной функции -
`BEVEL_WINGS_FOLDS`; укладка по соседям - `BEVEL_WINGS_FITTED` и наибольший разрыв на общих вершинах `BEVEL_WINGS_FIT_GAP_MAX_MICRO_WIDTHS`,
поворот вместо аффинного - `BEVEL_WINGS_FIT_RIGID_FALLBACK`; не положенная группа (не к чему прижаться) - `BEVEL_WINGS_UNFITTED`, её UV - UV ядра.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass

import numpy as np

WINGS_FORMAT = "CFTUV_BEVEL_WINGS"
WINGS_VERSION = 1
#: Свойство МЕША декали: запись крыльев (`wings_to_json`), её кладёт кнопка Both в новый меш. Нет свойства - UV меша не трогаются.
MESH_PROPERTY = "cftuv_bevel_wings"
#: Допуск `v` угла на цепи-источнике (в единицах alpha): UV ядра у источника - точный нуль.
SEAM_V_TOLERANCE = 1e-6
#: Узлы кусочно-линейных функций одной группы с `u` ближе этого - один узел (перекладина: вершина рельса и средней линии одной `u`).
KNOT_U_TOLERANCE = 1e-7
GROUP_SHIFT = 20
#: Направление связи судится по корреляции станций, только если общие вершины тянутся вдоль хотя бы на эту длину (в единицах alpha):
#: перекладина двух огибающих одной стороны (одна `u` у обеих, разброс - шум) направления не несёт.
ALONG_CHECK_SPAN = 0.5

COUNTER_LOOPS = "BEVEL_WINGS_LOOPS"
COUNTER_LANES = "BEVEL_WINGS_LANES"
COUNTER_RAIL_PAIRS = "BEVEL_WINGS_RAIL_PAIRS"
COUNTER_CENTRE_PAIRS = "BEVEL_WINGS_CENTRE_PAIRS"
COUNTER_CENTRE_UNMET = "BEVEL_WINGS_CENTRE_UNMET"
COUNTER_OUTER_UNPAIRED = "BEVEL_WINGS_OUTER_UNPAIRED"
COUNTER_NOT_TWO_SIDED = "BEVEL_WINGS_NOT_TWO_SIDED"
COUNTER_ALONG_AGAINST = "BEVEL_WINGS_ALONG_AGAINST"
COUNTER_FOLDS = "BEVEL_WINGS_FOLDS"
COUNTER_RIGID_PLACED = "BEVEL_WINGS_FITTED"
COUNTER_RIGID_UNPLACED = "BEVEL_WINGS_UNFITTED"
COUNTER_RIGID_GAP = "BEVEL_WINGS_FIT_GAP_MAX_MICRO_WIDTHS"
COUNTER_FIT_RIGID = "BEVEL_WINGS_FIT_RIGID_FALLBACK"
COUNTER_HALF_BEVEL = "BEVEL_WINGS_HALF_BEVEL_MEDIAN_MICRO_WIDTHS"
#: Узел примкнул к своей полосе через разрез (BEVEL_WINGS_NODE_CONTINUES_STRIP_V1).
COUNTER_NODE_CONTINUES = "BEVEL_WINGS_NODE_CONTINUES_STRIP"

RAIL, OUTER, RIGID, LONE = 0, 1, 2, 3


@dataclass(frozen=True, slots=True)
class WingsRecordV1:
    """Запись крыльев меша декали: домены граней бевела (номера патчей анализа нажатия Both)."""

    inner_domains: tuple


def wings_to_json(record: WingsRecordV1) -> str:
    payload = {"format": WINGS_FORMAT, "version": WINGS_VERSION, "inner": sorted(int(item) for item in record.inner_domains)}
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def wings_from_json(text: str) -> WingsRecordV1:
    data = json.loads(text)
    if data.get("format") != WINGS_FORMAT or data.get("version") != WINGS_VERSION:
        raise ValueError(f"not a {WINGS_FORMAT} v{WINGS_VERSION} record")
    return WingsRecordV1(tuple(int(item) for item in data["inner"]))


@dataclass(frozen=True, slots=True)
class WingsPlanV1:
    """Топология крыльев (не зависит от чисел UV, кроме масок нуля `v` на источнике): группы, слоты `(группа, вершина)`, связи, роли, знаки, обход."""

    loop_group: np.ndarray  # индекс группы угла
    loop_slot: np.ndarray  # слот угла
    slot_group: np.ndarray
    slot_vertex: np.ndarray
    role: np.ndarray  # RAIL / OUTER / RIGID / LONE по группе
    sign: np.ndarray  # знак крыла по группе (+1 / -1), у жёстких - 0
    lane: np.ndarray  # компонента ленты по группе, -1 - жёсткая
    direction: np.ndarray  # направление `along` по группе
    tree: tuple  # рёбра обхода `(родитель, ребёнок, слоты родителя, слоты ребёнка)` по порядку обхода
    along_links: tuple  # пары слотов всех связей лент (R и C между рельсовыми): `(слоты a, слоты b)`
    centre_slots: dict  # рельсовая группа -> слоты её вершин средней линии
    rail_slots: dict  # внешняя группа -> `(её слоты, слоты рельсовой)` по общим вершинам рельса
    rigid_order: tuple  # жёсткие группы: `(группа, (кандидаты по приоритету: (группа партнёра, слоты g, слоты партнёра), ...))`
    numbers: dict
    group_loops: tuple = ()  # углы каждой группы (индексы): кадр не сканирует все углы на каждую группу


def _pairs_by_vertex(slot_group, slot_vertex):
    """`{(группа a, группа b): (слоты a, слоты b)}` по общим вершинам (a < b)."""

    shared: dict = {}
    order = np.lexsort((slot_group, slot_vertex))
    vertex = slot_vertex[order]
    starts = np.flatnonzero(np.r_[True, vertex[1:] != vertex[:-1]])
    ends = np.r_[starts[1:], order.shape[0]]
    for first, last in zip(starts[ends - starts >= 2], ends[ends - starts >= 2]):
        members = order[first:last].tolist()
        for a in range(len(members)):
            for b in range(a + 1, len(members)):
                left, right = members[a], members[b]
                found = shared.setdefault((int(slot_group[left]), int(slot_group[right])), ([], []))
                found[0].append(left)
                found[1].append(right)
    return shared


def _classify(shared, inner, domain, slot_seam):
    """Связи по видам: `R` (бевел | снаружи, обе стороны на источнике), `X` (бевел | бевел разных доменов на источнике), `C` (бевел | бевел одного домена;
    пятый элемент - есть ли среди общих вершин точка на источнике у обеих: так соседствуют две огибающие ОДНОЙ стороны вдоль рельса, а средняя
    линия источника не касается)."""

    links = {"R": [], "X": [], "C": []}
    for (a, b), (slots_a, slots_b) in sorted(shared.items()):
        if len(slots_a) < 2:
            continue
        sa, sb = np.asarray(slots_a, dtype=np.int64), np.asarray(slots_b, dtype=np.int64)
        on_source = slot_seam[sa] & slot_seam[sb]
        if inner[a] != inner[b]:
            if np.count_nonzero(on_source) >= 2:
                a_in = a if inner[a] else b
                pick = on_source
                links["R"].append((a, b, sa[pick], sb[pick]) if a == a_in else (b, a, sb[pick], sa[pick]))
        elif inner[a] and domain[a] != domain[b]:
            if np.count_nonzero(on_source) >= 2:
                links["X"].append((a, b, sa[on_source], sb[on_source]))
        elif inner[a]:
            links["C"].append((a, b, sa, sb, bool(np.any(on_source))))
    return links


def _signs(count, role, links):
    """Знаки крыльев и ленты обходом в ширину; рёбра обхода; число конфликтов знака."""

    neighbours: dict = {}
    for inner_group, outer_group, slots_in, slots_out in links["R"]:
        neighbours.setdefault(inner_group, []).append((outer_group, 1, slots_in, slots_out))
        neighbours.setdefault(outer_group, []).append((inner_group, 1, slots_out, slots_in))
    for a, b, slots_a, slots_b, side in links["C"]:
        if role[a] == RAIL and role[b] == RAIL:
            flip = 1 if side else -1  # соседи вдоль одного рельса - одно крыло, встреча фронтов - два
            neighbours.setdefault(a, []).append((b, flip, slots_a, slots_b))
            neighbours.setdefault(b, []).append((a, flip, slots_b, slots_a))
    sign = np.zeros(count, dtype=np.int64)
    lane = np.full(count, -1, dtype=np.int64)
    tree, conflicts = [], 0
    for root in range(count):
        if role[root] not in (RAIL, OUTER, LONE) or lane[root] >= 0:
            continue
        sign[root], lane[root] = 1, root
        frontier = [root]
        while frontier:
            following = []
            for group in frontier:
                for other, flip, mine, theirs in neighbours.get(group, ()):
                    if lane[other] < 0:
                        sign[other], lane[other] = sign[group] * flip, root
                        tree.append((group, other, mine, theirs))
                        following.append(other)
                    elif sign[other] != sign[group] * flip and group < other:
                        conflicts += 1
            frontier = following
    return sign, lane, tuple(tree), conflicts


def _rigid_order(count, role, domain, links):
    """Кандидаты жёсткой укладки по приоритету: рельсовые того же домена (средняя `C`), группа по ту сторону разреза (`X`), любая того же домена."""

    by_group: dict = {group: ([], [], []) for group in range(count) if role[group] == RIGID}
    for a, b, slots_a, slots_b, _side in links["C"]:
        for g, h, mine, theirs in ((a, b, slots_a, slots_b), (b, a, slots_b, slots_a)):
            if g in by_group:
                by_group[g][0 if role[h] == RAIL else 2].append((h, mine, theirs))
    for a, b, slots_a, slots_b in links["X"]:
        for g, h, mine, theirs in ((a, b, slots_a, slots_b), (b, a, slots_b, slots_a)):
            if g in by_group:
                by_group[g][1].append((h, mine, theirs))
    return tuple((group, tuple(tiers)) for group, tiers in sorted(by_group.items()))


def wings_plan(loop_vertex, loop_group, seam_mask, inner_domains) -> WingsPlanV1:
    loop_vertex = np.asarray(loop_vertex, dtype=np.int64).reshape(-1)
    codes, loop_slot_group = np.unique(np.asarray(loop_group, dtype=np.int64).reshape(-1), return_inverse=True)
    count = int(codes.shape[0])
    domain = codes >> GROUP_SHIFT
    inner = np.isin(domain, np.asarray(sorted(int(item) for item in inner_domains), dtype=np.int64))
    stride = int(loop_vertex.max(initial=0)) + 1
    keys = loop_slot_group.astype(np.int64) * stride + loop_vertex
    slot_keys, loop_slot = np.unique(keys, return_inverse=True)
    slot_group, slot_vertex = slot_keys // stride, slot_keys % stride
    slot_seam = np.zeros(slot_keys.shape[0], dtype=bool)
    slot_seam[loop_slot[np.asarray(seam_mask, dtype=bool)]] = True
    links = _classify(_pairs_by_vertex(slot_group, slot_vertex), inner, domain, slot_seam)
    role = np.where(inner, RIGID, LONE).astype(np.int64)
    for inner_group, outer_group, _slots_in, _slots_out in links["R"]:
        role[inner_group], role[outer_group] = RAIL, OUTER
    sign, lane, tree, conflicts = _signs(count, role, links)
    direction = np.where(role == RAIL, -sign, sign).astype(np.float64)
    centre: dict = {}
    centre_links = [(a, b, sa, sb) for a, b, sa, sb, side in links["C"] if role[a] == RAIL and role[b] == RAIL and not side and sign[a] != sign[b]]
    side_links = [(a, b, sa, sb) for a, b, sa, sb, side in links["C"] if role[a] == RAIL and role[b] == RAIL and side and sign[a] == sign[b]]
    for a, b, slots_a, slots_b in centre_links:
        centre.setdefault(a, []).append(slots_a)
        centre.setdefault(b, []).append(slots_b)
    rails: dict = {}
    for inner_group, outer_group, slots_in, slots_out in links["R"]:
        rails.setdefault(outer_group, []).append((slots_out, slots_in))
    along_links = tuple((mine, theirs) for _g, _h, mine, theirs in tree)
    along_links += tuple((slots_a, slots_b) for _a, _b, slots_a, slots_b in (*centre_links, *side_links))
    along_links += tuple((slots_in, slots_out) for _a, _b, slots_in, slots_out in links["R"])
    numbers = {
        COUNTER_LANES: int(np.unique(lane[lane >= 0]).shape[0]),
        COUNTER_RAIL_PAIRS: len(links["R"]),
        COUNTER_CENTRE_PAIRS: len(centre_links),
        COUNTER_CENTRE_UNMET: int(sum(1 for group in range(count) if role[group] == RAIL and group not in centre)),
        COUNTER_OUTER_UNPAIRED: int(np.count_nonzero(role == LONE)),
        COUNTER_NOT_TWO_SIDED: conflicts,
    }
    return WingsPlanV1(
        loop_slot_group, loop_slot, slot_group, slot_vertex, role, sign, lane, direction, tree, along_links,
        {group: np.concatenate(items) for group, items in centre.items()},
        {group: (np.concatenate([item[0] for item in items]), np.concatenate([item[1] for item in items])) for group, items in rails.items()},
        _rigid_order(count, role, domain, links), numbers,
        tuple(np.split(np.argsort(loop_slot_group, kind="stable"), np.cumsum(np.bincount(loop_slot_group, minlength=count))[:-1])),
    )


_PLANS: dict = {}
PLAN_CACHE_SIZE = 8


def cached_wings_plan(loop_vertex, loop_group, seam_mask, inner_domains) -> WingsPlanV1:
    """План зависит от топологии меша и масок нуля `v`: кадры живой ширины берут его из памяти. Ключ - байты массивов и домены бевела."""

    parts = (
        np.ascontiguousarray(loop_vertex, dtype=np.int64).tobytes(), np.ascontiguousarray(loop_group, dtype=np.int64).tobytes(),
        np.packbits(np.asarray(seam_mask, dtype=bool)).tobytes(), repr(sorted(int(item) for item in inner_domains)).encode("ascii"),
    )
    key = hashlib.blake2b(b"|".join(parts), digest_size=16).digest()
    plan = _PLANS.get(key)
    if plan is None:
        plan = wings_plan(loop_vertex, loop_group, seam_mask, inner_domains)
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


def _knots(xs, ts):
    """Узлы с `u` ближе `KNOT_U_TOLERANCE` сливаются (цель - среднее): `(xs, ts)` по возрастанию."""

    order = np.argsort(xs, kind="stable")
    xs, ts = xs[order], ts[order]
    starts = np.flatnonzero(np.r_[True, np.diff(xs) > KNOT_U_TOLERANCE])
    sizes = np.diff(np.r_[starts, xs.shape[0]])
    ids = np.repeat(np.arange(starts.shape[0]), sizes)
    return xs[starts], np.bincount(ids, weights=ts) / sizes


def _slot_means(plan, values):
    counts = np.bincount(plan.loop_slot, minlength=plan.slot_group.shape[0]).astype(np.float64)
    return np.bincount(plan.loop_slot, weights=values, minlength=plan.slot_group.shape[0]) / np.maximum(counts, 1.0)


def _along(plan, slot_u, u, v, half):
    """`along` углов лент, число изломов и связей против направления.

    Рельсовая группа смешивает две кусочно-линейные функции своей `u`: по узлам рельса (`t = 0`) и по узлам средней линии (`t = 1`),
    `t = v / H(u)`; у кривой полосы рельсы разной длины, и одна функция с узлами обоих краёв ломалась бы зигзагом. Внешняя - функция рельса.
    """

    count = plan.role.shape[0]
    offset = np.zeros(count, dtype=np.float64)
    direction = plan.direction
    for parent, child, mine, theirs in plan.tree:
        offset[child] = float(np.mean(direction[parent] * slot_u[mine] + offset[parent] - direction[child] * slot_u[theirs]))
    against = 0
    for mine, theirs in plan.along_links:
        a, b = slot_u[mine], slot_u[theirs]
        if a.shape[0] >= 2 and np.ptp(a) > ALONG_CHECK_SPAN and np.ptp(b) > ALONG_CHECK_SPAN:
            ga, gb = int(plan.slot_group[mine[0]]), int(plan.slot_group[theirs[0]])
            if np.sum((a - a.mean()) * (b - b.mean())) * direction[ga] * direction[gb] < 0:
                against += 1
    proposal = direction[plan.slot_group] * slot_u + offset[plan.slot_group]
    linked = np.zeros(plan.slot_group.shape[0], dtype=bool)
    for mine, theirs in plan.along_links:
        linked[mine] = True
        linked[theirs] = True
    target = proposal.copy()
    chosen = linked & (plan.lane[plan.slot_group] >= 0)
    if np.any(chosen):
        place = plan.lane[plan.slot_group] * (int(plan.slot_vertex.max(initial=0)) + 1) + plan.slot_vertex
        _places, inverse = np.unique(place[chosen], return_inverse=True)
        target[chosen] = (np.bincount(inverse, weights=proposal[chosen]) / np.bincount(inverse))[inverse]
    rails: dict = {}
    for outer_group, (slots_out, slots_in) in plan.rail_slots.items():
        rails.setdefault(outer_group, []).append(slots_out)
        for group in np.unique(plan.slot_group[slots_in]).tolist():
            rails.setdefault(group, []).append(slots_in[plan.slot_group[slots_in] == group])
    along = np.zeros(u.shape[0], dtype=np.float64)
    folds = 0

    def curve(slots, at, slope):
        nonlocal folds
        xs, ts = _knots(slot_u[slots], target[slots])
        folds += int(np.count_nonzero(np.diff(ts) * slope < 0.0))
        return _piecewise(xs, ts, at, slope)

    for group in range(count):
        loops = plan.group_loops[group]
        if not loops.shape[0] or plan.role[group] == RIGID:
            continue
        slope = direction[group]
        rail = curve(np.concatenate(rails[group]), u[loops], slope) if group in rails else slope * u[loops] + offset[group]
        if plan.role[group] == RAIL and group in plan.centre_slots:
            xs, ts = half[group]
            height = _piecewise(xs, ts, u[loops], None)
            share = np.clip(np.divide(v[loops], height, out=np.zeros(loops.shape[0]), where=height > 0.0), 0.0, 1.0)
            along[loops] = (1.0 - share) * rail + share * curve(plan.centre_slots[group], u[loops], slope)
        else:
            along[loops] = rail
    return along, folds, against


def _half(plan, slot_u, slot_v, v):
    """`H` (половина полосы в единицах alpha) по группе лент: узлы `(u, H)`; и `H_med`."""

    count = plan.role.shape[0]
    half: dict = {}
    for group, slots in plan.centre_slots.items():
        half[group] = _knots(slot_u[slots], slot_v[slots])
    values = [ts for _xs, ts in half.values()]
    median = float(np.median(np.concatenate(values))) if values else 0.0
    for group in range(count):
        if plan.role[group] == RAIL and group not in half:
            loops = plan.group_loops[group]
            half[group] = (np.zeros(1), np.array([float(v[loops].max(initial=0.0))]))
    for group, pairs in plan.rail_slots.items():
        slots_out, slots_in = pairs
        heights = np.empty(slots_out.shape[0], dtype=np.float64)
        for index, theirs in enumerate(slots_in.tolist()):
            xs, ts = half[int(plan.slot_group[theirs])]
            heights[index] = float(_piecewise(xs, ts, np.array([slot_u[theirs]]), None)[0])
        half[group] = _knots(slot_u[slots_out], heights)
    for group in range(count):
        if plan.role[group] == LONE:
            half[group] = (np.zeros(1), np.array([median]))
    return half, median


def _across(plan, half, u, v):
    """`w` углов лент: расстояние от средней линии в единицах alpha (рельсовая `H - v`, внешняя `H + v`)."""

    w = np.zeros(u.shape[0], dtype=np.float64)
    for group in range(plan.role.shape[0]):
        if plan.role[group] == RIGID:
            continue
        loops = plan.group_loops[group]
        xs, ts = half[group]
        height = _piecewise(xs, ts, u[loops], None)
        w[loops] = height - v[loops] if plan.role[group] == RAIL else height + v[loops]
    return w


#: Точки кандидатов на одной прямой: второе сингулярное число центрированных точек меньше этой доли первого.
COLLINEAR_RATIO = 1e-6


def _rotation(source, target):
    """Поворот и сдвиг (без отражения) наименьших квадратов."""

    ps, qs = source - source.mean(axis=0), target - target.mean(axis=0)
    cross = float(np.sum(ps[:, 0] * qs[:, 1] - ps[:, 1] * qs[:, 0]))
    dot = float(np.sum(ps[:, 0] * qs[:, 0] + ps[:, 1] * qs[:, 1]))
    angle = np.arctan2(cross, dot)
    matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return matrix, target.mean(axis=0) - matrix @ source.mean(axis=0)


def _fit(source, target):
    """Отображение `source -> target` без отражения: `(матрица 2x2, сдвиг, наибольший разрыв, упал ли закон на поворот)`.

    Точки в общем положении - аффинное наименьших квадратов (конец кривой полосы в ленте скошен: поворот оставлял разрыв до 0.1 ширины);
    точки на прямой (край разреза) - поворот, переводящий прямую в прямую целей, с растяжением ВДОЛЬ неё (поперёк масштаб 1). Аффинное с
    отражением или вырожденное - поворот (`BEVEL_WINGS_FIT_RIGID_FALLBACK`).
    """

    centre = source.mean(axis=0)
    ps = source - centre
    singular = np.linalg.svd(ps, compute_uv=False) if ps.shape[0] >= 2 else np.zeros(2)
    fallback = False
    if singular.shape[0] >= 2 and singular[0] > 0.0 and singular[1] > COLLINEAR_RATIO * singular[0] and source.shape[0] >= 3:
        design = np.hstack((source, np.ones((source.shape[0], 1))))
        solution, *_rest = np.linalg.lstsq(design, target, rcond=None)
        matrix, shift = solution[:2].T, solution[2]
        if np.linalg.det(matrix) <= 0.0:
            matrix, shift = _rotation(source, target)
            fallback = True
    elif singular.shape[0] >= 1 and singular[0] > 0.0:
        direction = np.linalg.svd(ps)[2][0]
        along = ps @ direction
        qs = target - target.mean(axis=0)
        image = (along[:, None] * qs).sum(axis=0) / float(np.dot(along, along))
        if not np.linalg.norm(image) > 0.0:
            matrix, shift = _rotation(source, target)
            fallback = True
        else:
            unit = image / np.linalg.norm(image)
            normal, normal_image = np.array([-direction[1], direction[0]]), np.array([-unit[1], unit[0]])
            matrix = np.outer(image, direction) + np.outer(normal_image, normal)
            shift = target.mean(axis=0) - matrix @ centre
    else:
        matrix, shift = np.eye(2), target.mean(axis=0) - centre
    gap = float(np.max(np.linalg.norm(source @ matrix.T + shift - target, axis=1), initial=0.0))
    return matrix, shift, gap, fallback


def _rigid(plan, slot_u, slot_v, uvs, out, placed):
    """Жёсткие группы по приоритету кандидатов, проход за проходом, пока что-то кладётся: `(положено, не положено, разрыв, узлов к полосе)`."""

    loop_of_slot = np.full(plan.slot_group.shape[0], -1, dtype=np.int64)
    loop_of_slot[plan.loop_slot] = np.arange(plan.loop_slot.shape[0])
    pending = dict(plan.rigid_order)
    done, gap, continues, rigid = 0, 0.0, 0, 0
    while pending:
        progress = False
        for group, tiers in sorted(pending.items()):
            for tier, candidates in enumerate(tiers):
                chosen = [(mine, theirs) for other, mine, theirs in candidates if placed[other]]
                if not chosen:
                    continue
                mine = np.concatenate([item[0] for item in chosen])
                theirs = np.concatenate([item[1] for item in chosen])
                source = np.stack((slot_u[mine], slot_v[mine]), axis=1)
                target = out[loop_of_slot[theirs]]
                matrix, shift, fit_gap, fallback = _fit(source, target)
                rigid += int(fallback)
                loops = plan.group_loops[group]
                out[loops] = uvs[loops] @ matrix.T + shift
                placed[group] = True
                gap = max(gap, fit_gap)
                done += 1
                continues += int(tier == 1)
                del pending[group]
                progress = True
                break
        if not progress:
            break
    return done, len(pending), gap, continues, rigid


def wing_uvs(uvs, loop_vertex, loop_group, inner_domains) -> tuple:
    """`(UV крыльев (L, 2) float64, числа)` по UV ядра `uvs` (L, 2), вершине и группе угла и доменам граней бевела."""

    uvs = np.asarray(uvs, dtype=np.float64).reshape(-1, 2)
    u, v = uvs[:, 0].copy(), uvs[:, 1].copy()
    plan = cached_wings_plan(loop_vertex, loop_group, np.abs(v) <= SEAM_V_TOLERANCE, inner_domains)
    slot_u, slot_v = _slot_means(plan, u), _slot_means(plan, v)
    half, median = _half(plan, slot_u, slot_v, v)
    along, folds, against = _along(plan, slot_u, u, v, half)
    w = _across(plan, half, u, v)
    out = np.stack((along, plan.sign[plan.loop_group].astype(np.float64) * w), axis=1)
    placed = plan.role != RIGID
    done, left, gap, continues, rigid = _rigid(plan, slot_u, slot_v, uvs, out, placed)
    unplaced = plan.role[plan.loop_group] == RIGID
    unplaced &= ~placed[plan.loop_group]
    out[unplaced] = uvs[unplaced]
    scale = 1.0 / (2.0 * (1.0 + median))
    result = np.empty_like(out)
    result[:, 0] = out[:, 0] * scale
    result[:, 1] = 0.5 + out[:, 1] * scale
    numbers = dict(plan.numbers)
    numbers.update({
        COUNTER_LOOPS: int(uvs.shape[0]), COUNTER_FOLDS: folds, COUNTER_ALONG_AGAINST: against, COUNTER_RIGID_PLACED: done,
        COUNTER_RIGID_UNPLACED: left, COUNTER_RIGID_GAP: int(round(gap * 1e6)), COUNTER_FIT_RIGID: rigid, COUNTER_HALF_BEVEL: int(round(median * 1e6)),
        COUNTER_NODE_CONTINUES: continues,
    })
    return result, numbers


# --- Нажатие Both: закон сварки и строка частей (BEVEL_FACES_BOTH_V1) -----------------------------------------------------------
#
# СВАРКА. На рельсе у конца полосы сходятся четыре домена (грань снаружи, две полосы, узел): четыре сдвинутые плоскости в общей точке
# не встречаются, и закон митры хоста (`envelope_production_weld.WELD_LAW`, допуск невязки 1e-9) оставлял такие вершины раздельными
# (`ADAPTER_WELD_MITER_FALLBACK`, куб в 2 сегмента: 24 вершины из 48 общих). У меша Both допуск невязки - `BOTH_WELD_RESIDUAL` смещения:
# общая вершина лежит не дальше `d / 8` от сдвинутой плоскости каждого своего домена (замер: нужно 0.05-0.1 на кубе и L, 90 градусов),
# и вершина одна; больше - прежний именованный отказ митры. Отклонение граней от их плоскости после смещения меряет квитанция
# (`ADAPTER_MAX_OFF_PLANE_AFTER_OFFSET_NANOMETRES`). Закон узнаётся по доменам прогона: номер домена - хеш ревизии анализа и патча, а
# ревизия анализа Both своя (`scope_digest`), поэтому кнопка, живая ширина и подтверждение того же нажатия варят одним законом.

#: Допуск невязки системы митры у меша Both, в долях смещения.
BOTH_WELD_RESIDUAL = 0.125
_BOTH_DOMAINS: set = set()
BOTH_DOMAINS_LIMIT = 1 << 16


def note_both_domains(domain_ids) -> None:
    """Домены нажатия Both: их меш варится законом `both_weld_law` (память ограничена: переполнение - забыть прежние нажатия)."""

    found = {str(item) for item in domain_ids}
    if len(_BOTH_DOMAINS) + len(found) > BOTH_DOMAINS_LIMIT:
        _BOTH_DOMAINS.clear()
    _BOTH_DOMAINS.update(found)


def is_both_run(results) -> bool:
    return bool(_BOTH_DOMAINS) and any(str(getattr(item, "domain_id", "")) in _BOTH_DOMAINS for item in results)


def both_weld_law(law: tuple) -> tuple:
    """Закон сварки Both: тот же закон хоста, допуск невязки митры - `BOTH_WELD_RESIDUAL`."""

    return (law[0], law[1], BOTH_WELD_RESIDUAL, *law[3:])


def parts_summary(inner_patches, built, skipped) -> str:
    """Строка частей нажатия Both: сколько доменов бевела (inward) и снаружи (outward) в меше, и отказ каждой части по имени.

    Отказ одного домена не снимает другие (как у пути по рёбрам): часть, которая построилась, остаётся в меше, а отказанная названа.
    """

    inner = {int(item) for item in inner_patches}
    parts = []
    for label, mine in (("inward", True), ("outward", False)):
        done = sum(1 for patch in built if (int(patch) in inner) == mine)
        refused: dict = {}
        for patch, _domain, outcome, _detail in skipped:
            if (int(patch) in inner) == mine:
                refused[outcome] = refused.get(outcome, 0) + 1
        text = f"{label} {done}/{done + sum(refused.values())}"
        if refused:
            text += " refused (" + ", ".join(name if count == 1 else f"{name} x{count}" for name, count in sorted(refused.items())) + ")"
        parts.append(text)
    return "both: " + ", ".join(parts)


__all__ = (
    "BOTH_WELD_RESIDUAL",
    "COUNTER_ALONG_AGAINST",
    "COUNTER_CENTRE_PAIRS",
    "COUNTER_CENTRE_UNMET",
    "COUNTER_FIT_RIGID",
    "COUNTER_FOLDS",
    "COUNTER_HALF_BEVEL",
    "COUNTER_LANES",
    "COUNTER_LOOPS",
    "COUNTER_NODE_CONTINUES",
    "COUNTER_NOT_TWO_SIDED",
    "COUNTER_OUTER_UNPAIRED",
    "COUNTER_RAIL_PAIRS",
    "COUNTER_RIGID_GAP",
    "COUNTER_RIGID_PLACED",
    "COUNTER_RIGID_UNPLACED",
    "MESH_PROPERTY",
    "WingsPlanV1",
    "WingsRecordV1",
    "both_weld_law",
    "cached_wings_plan",
    "is_both_run",
    "note_both_domains",
    "parts_summary",
    "wing_uvs",
    "wings_from_json",
    "wings_plan",
    "wings_to_json",
)
