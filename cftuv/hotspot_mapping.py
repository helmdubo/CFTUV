"""Отображение UV ядра в атлас разметки Hotspot (HOTSPOT_ATLAS_MAPPING_V1). Без Blender: numpy и разметка.

ВХОД - UV ядра под Straight strips (`UV_DIRECT_STRIP_V1`) по углу грани: `v = r / alpha` (0 на шве, 1 на внешнем крае), `u` вдоль цепи в единицах
alpha (единица `u` - одна ширина декали), один остров на pChain; плюс номер вершины меша угла и номер группы (сторона шва: домен и владелец грани).

`trim`: `P = низ + v * T * вверх + u * T * вправо` в пикселях, `T` - толщина метки, `низ` - левый конец нижнего края в рамке метки (`вправо` - `вверх`,
повёрнутое на -90°). Обе стороны шва ложатся в один прямоугольник друг на друга; по длине метка повторяется через wrap (её длина - вся текстура).

`corner`: шов - средняя линия, крыло - половина толщины `T / 2` = ширина декали. Крыло `+` ложится от средней линии к `вверх`, крыло `-` - к `вниз`.
На двух сторонах шва станции ядра идут НАВСТРЕЧУ и начало расходится на миллиметры, а длина цепи в карте каждой стороны своя (полевой замер: сумма
станций двух сторон вдоль 7.8 м цепи гуляет на 1.9 ширины). Поэтому стороны выравниваются по ОБЩИМ вершинам шва, а не одной константой:

* группы с общими вершинами шва (не меньше двух) - соседи; компонента соседей раскрашивается в два цвета обходом в ширину от группы с наименьшим
  номером (наименьший домен): цвет 0 - крыло `+`, цвет 1 - крыло `-`; `swap` меняет крылья местами поворотом всей раскладки на 180° вокруг
  средней линии (`along -> -along`: по длине метка всё равно повторяется, а ориентация текстуры у обеих сторон остаётся);
* направление `along` каждой группы - знак корреляции её станций с соседом (`s = -1` у встречной стороны), сдвиг - среднее по общим вершинам;
* у каждой общей вершины шва одна цель `t` - среднее предложений её групп; `along` группы - кусочно-линейная функция её `u` по узлам `(u, t)` её
  вершин шва, за крайними узлами - наклон `s`. Поэтому крылья сходятся на КАЖДОЙ общей вершине шва точно, а перекладины острова (одна `u`)
  остаются перекладинами.

Поворот крыла `-` (`u -> -u`, `v -> -v`) - поворот на 180°, ориентация текстуры сохраняется; если данные велят отражение (станции сторон идут в одну
сторону), это названо числом `HOTSPOT_CORNER_WING_MIRRORED`. Ни один исход не молчит: числа - в ответе.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass

import numpy as np

from .hotspot_markup import KIND_CORNER, KIND_TRIM, UP_VECTORS, HotspotMarkV1

MAPPING_FORMAT = "CFTUV_HOTSPOT_MAPPING"
MAPPING_VERSION = 1
#: Свойство МЕША декали: запись отображения (mapping_to_json), её кладёт кнопка в новый меш. Нет свойства - UV меша не трогаются (режим выключен).
MESH_PROPERTY = "cftuv_hotspot"
#: Допуск `v` угла шва (в единицах ширины): UV ядра у источника - точный нуль, кадр модели его не двигает.
SEAM_V_TOLERANCE = 1e-6

# Числа отображения (имена исходов).
COUNTER_LOOPS = "HOTSPOT_LOOPS_MAPPED"
COUNTER_PAIRS = "HOTSPOT_CORNER_SIDE_PAIRS"
COUNTER_UNPAIRED = "HOTSPOT_CORNER_WING_UNPAIRED"
COUNTER_NO_SEAM = "HOTSPOT_CORNER_GROUP_WITHOUT_SEAM"
COUNTER_NOT_TWO_SIDED = "HOTSPOT_CORNER_WINGS_NOT_TWO_SIDED"
COUNTER_MIRRORED = "HOTSPOT_CORNER_WING_MIRRORED"
COUNTER_SEAM_UV_SPLIT = "HOTSPOT_CORNER_SEAM_UV_SPLIT"
COUNTER_FOLDS = "HOTSPOT_CORNER_ALIGN_FOLDS"
#: Наибольший сдвиг `along` вершины шва от собственной станции стороны, в миллионных ширины (цена выравнивания по общим вершинам).
COUNTER_SHIFT = "HOTSPOT_CORNER_ALIGN_SHIFT_MAX_MICRO_WIDTHS"


@dataclass(frozen=True, slots=True)
class HotspotMappingV1:
    """Разрешённое отображение одной метки: всё, что нужно формулам, в пикселях, и откуда оно (`markup`, `mark`) - для строк панели."""

    markup: str
    mark: HotspotMarkV1
    width: int
    height: int
    swap: bool = False

    @property
    def up(self) -> tuple:
        return UP_VECTORS[self.mark.up]

    @property
    def right(self) -> tuple:
        dx, dy = self.up
        return (dy, -dx)

    def left_end(self, across: float) -> tuple:
        """Левый (в рамке метки) конец линии на высоте `across` от середины по толщине, в пикселях."""

        mark = self.mark
        (ux, uy), (rx, ry) = self.up, self.right
        cx, cy = mark.x + mark.w / 2.0, mark.y + mark.h / 2.0
        return (cx + ux * across - rx * mark.length / 2.0, cy + uy * across - ry * mark.length / 2.0)


def mapping_to_json(mapping: HotspotMappingV1) -> str:
    mark = mapping.mark
    payload = {
        "format": MAPPING_FORMAT, "version": MAPPING_VERSION, "markup": mapping.markup, "texture": [mapping.width, mapping.height], "swap": bool(mapping.swap),
        "mark": {"id": mark.id, "name": mark.name, "kind": mark.kind, "rect": [mark.x, mark.y, mark.w, mark.h], "up": mark.up, "band": mark.band},
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def mapping_from_json(text: str) -> HotspotMappingV1:
    data = json.loads(text)
    if data.get("format") != MAPPING_FORMAT or data.get("version") != MAPPING_VERSION:
        raise ValueError(f"not a {MAPPING_FORMAT} v{MAPPING_VERSION} record")
    item = data["mark"]
    x, y, w, h = item["rect"]
    mark = HotspotMarkV1(item["id"], item["name"], item["kind"], x, y, w, h, item["up"], item["band"])
    width, height = data["texture"]
    return HotspotMappingV1(data["markup"], mark, width, height, bool(data["swap"]))


def _frame_to_atlas(mapping: HotspotMappingV1, along, across, base) -> np.ndarray:
    """Пиксели `base + along * вправо + across * вверх` (обе величины - в пикселях) в UV атласа `(x / W, y / H)`."""

    (ux, uy), (rx, ry) = mapping.up, mapping.right
    out = np.empty((along.shape[0], 2), dtype=np.float64)
    out[:, 0] = (base[0] + along * rx + across * ux) / float(mapping.width)
    out[:, 1] = (base[1] + along * ry + across * uy) / float(mapping.height)
    return out


def atlas_uvs(uvs, loop_vertex, loop_group, mapping: HotspotMappingV1) -> tuple:
    """`(UV атласа (L, 2) float64, числа)` по UV ядра `uvs` (L, 2) угла, номеру вершины угла и номеру его группы (сторона шва)."""

    uvs = np.asarray(uvs, dtype=np.float64).reshape(-1, 2)
    u, v = uvs[:, 0], uvs[:, 1]
    mark = mapping.mark
    counters = {COUNTER_LOOPS: int(uvs.shape[0])}
    if mark.kind == KIND_TRIM:
        thickness = float(mark.thickness)
        return _frame_to_atlas(mapping, u * thickness, v * thickness, mapping.left_end(-thickness / 2.0)), counters
    if mark.kind != KIND_CORNER:
        raise ValueError(f"mark kind {mark.kind!r} has no mapping")
    wing = mark.thickness / 2.0
    plan = cached_corner_plan(np.asarray(loop_vertex, dtype=np.int64).reshape(-1), np.asarray(loop_group, dtype=np.int64).reshape(-1), np.abs(v) <= SEAM_V_TOLERANCE)
    along, sign, numbers = corner_along(plan, u, mapping.swap)
    counters.update(numbers)
    return _frame_to_atlas(mapping, along * wing, sign * v * wing, mapping.left_end(0.0)), counters


@dataclass(frozen=True, slots=True)
class CornerPlanV1:
    """Топология выравнивания крыльев (не зависит от чисел UV): группы, слоты шва `(группа, вершина)`, соседи и обход по уровням."""

    groups: tuple  # номера групп по возрастанию
    loop_slot_group: np.ndarray  # индекс группы (в `groups`) каждого угла
    seam_loops: np.ndarray  # углы шва
    seam_slot: np.ndarray  # слот каждого угла шва
    slot_group: np.ndarray  # индекс группы слота
    slot_vertex: np.ndarray  # вершина слота
    colour: np.ndarray  # цвет группы: 0, 1; -1 - группа без шва
    component: np.ndarray  # корень компоненты каждой группы (индекс группы); -1 - группа без шва
    levels: tuple  # рёбра обхода по глубине: `(родители, дети, слоты родителя, слоты ребёнка, ребро каждой пары слотов)`
    pairs: int
    unpaired: int
    not_two_sided: int


def _shared_pairs(slot_group, slot_vertex) -> dict:
    """`{(группа a, группа b): (слоты a, слоты b)}` по общим вершинам шва (a < b)."""

    shared: dict = {}
    order = np.lexsort((slot_group, slot_vertex))
    sorted_vertex = slot_vertex[order]
    starts = np.flatnonzero(np.r_[True, sorted_vertex[1:] != sorted_vertex[:-1]])
    ends = np.r_[starts[1:], order.shape[0]]
    for first, last in zip(starts[ends - starts >= 2], ends[ends - starts >= 2]):
        members = order[first:last].tolist()
        for a in range(len(members)):
            for b in range(a + 1, len(members)):
                left, right = members[a], members[b]  # группы по возрастанию: порядок внутри вершины
                found = shared.setdefault((int(slot_group[left]), int(slot_group[right])), ([], []))
                found[0].append(left)
                found[1].append(right)
    return shared


def corner_plan(loop_vertex, loop_group, seam_mask) -> CornerPlanV1:
    groups, loop_slot_group = np.unique(loop_group, return_inverse=True)
    seam_loops = np.nonzero(seam_mask)[0]
    stride = int(loop_vertex.max(initial=0)) + 1
    keys = loop_slot_group[seam_loops].astype(np.int64) * stride + loop_vertex[seam_loops]
    slot_keys, seam_slot = np.unique(keys, return_inverse=True)
    slot_group, slot_vertex = slot_keys // stride, slot_keys % stride
    neighbours: dict = {}
    for (ga, gb), (slots_a, slots_b) in sorted(_shared_pairs(slot_group, slot_vertex).items()):
        if len(slots_a) >= 2:
            neighbours.setdefault(ga, []).append((gb, slots_a, slots_b))
            neighbours.setdefault(gb, []).append((ga, slots_b, slots_a))
    count = len(groups)
    colour = np.full(count, -1, dtype=np.int64)
    component = np.full(count, -1, dtype=np.int64)
    has_seam = np.zeros(count, dtype=bool)
    has_seam[slot_group] = True
    by_depth: dict = {}
    pairs = not_two_sided = unpaired = 0
    for root in range(count):
        if colour[root] >= 0 or not has_seam[root]:
            continue
        colour[root], component[root] = 0, root
        if root not in neighbours:
            unpaired += 1
        frontier, depth = [root], 0
        while frontier:
            following = []
            for group in frontier:
                for other, mine, theirs in neighbours.get(group, ()):
                    if colour[other] < 0:
                        colour[other], component[other] = 1 - colour[group], root
                        pairs += 1
                        by_depth.setdefault(depth, []).append((group, other, mine, theirs))
                        following.append(other)
                    elif colour[other] == colour[group] and group < other:
                        not_two_sided += 1
            frontier, depth = following, depth + 1
    levels = []
    for depth in sorted(by_depth):
        edges = by_depth[depth]
        sizes = [len(mine) for _parent, _child, mine, _theirs in edges]
        levels.append((
            np.array([edge[0] for edge in edges], dtype=np.int64),
            np.array([edge[1] for edge in edges], dtype=np.int64),
            np.concatenate([np.asarray(edge[2], dtype=np.int64) for edge in edges]),
            np.concatenate([np.asarray(edge[3], dtype=np.int64) for edge in edges]),
            np.repeat(np.arange(len(edges)), sizes),
        ))
    return CornerPlanV1(tuple(int(item) for item in groups), loop_slot_group, seam_loops, seam_slot, slot_group, slot_vertex, colour, component, tuple(levels), pairs, unpaired, not_two_sided)


#: План зависит только от топологии меша (углы, группы, углы шва): кадры живой ширины его не пересчитывают. Ключ - байты трёх массивов.
#: Метки по островам (HOTSPOT_ISLAND_MARKS_V1) строят план на каждую corner-метку меша и один на весь меш (пары шва выделения): 16, а не 4.
_PLANS: dict = {}
PLAN_CACHE_SIZE = 16


def cached_corner_plan(loop_vertex, loop_group, seam_mask) -> CornerPlanV1:
    key = hashlib.blake2b(b"".join((np.ascontiguousarray(loop_vertex).tobytes(), b"|", np.ascontiguousarray(loop_group).tobytes(), b"|", np.packbits(seam_mask).tobytes())), digest_size=16).digest()
    plan = _PLANS.get(key)
    if plan is None:
        plan = corner_plan(loop_vertex, loop_group, seam_mask)
        if len(_PLANS) >= PLAN_CACHE_SIZE:
            _PLANS.pop(next(iter(_PLANS)))
        _PLANS[key] = plan
    return plan


def _offsets(plan: CornerPlanV1, slot_u) -> tuple:
    """`(направление, сдвиг)` каждой группы: уровень за уровнем обхода, все рёбра уровня разом."""

    count = len(plan.groups)
    direction = np.ones(count, dtype=np.float64)
    offset = np.zeros(count, dtype=np.float64)
    for parents, children, mine, theirs, edge in plan.levels:
        sizes = np.bincount(edge, minlength=parents.shape[0]).astype(np.float64)
        a, b = slot_u[mine], slot_u[theirs]
        mean_a = np.bincount(edge, weights=a, minlength=parents.shape[0]) / sizes
        mean_b = np.bincount(edge, weights=b, minlength=parents.shape[0]) / sizes
        correlation = np.bincount(edge, weights=(a - mean_a[edge]) * (b - mean_b[edge]), minlength=parents.shape[0])
        direction[children] = direction[parents] * np.where(correlation < 0.0, -1.0, 1.0)
        proposal = direction[parents][edge] * a + offset[parents][edge] - direction[children][edge] * b
        offset[children] = np.bincount(edge, weights=proposal, minlength=parents.shape[0]) / sizes
    return direction, offset


def corner_along(plan: CornerPlanV1, u, swap: bool) -> tuple:
    """`(along (L) в единицах ширины, знак поперёк (L): +1 крыло `+`, -1 крыло `-`, числа)` по `u` углов и плану."""

    slots = plan.slot_group.shape[0]
    counts = np.bincount(plan.seam_slot, minlength=slots).astype(np.float64)
    seam_u = u[plan.seam_loops]
    slot_u = np.bincount(plan.seam_slot, weights=seam_u, minlength=slots) / np.maximum(counts, 1.0)
    split = int(np.count_nonzero(np.abs(seam_u - slot_u[plan.seam_slot]) > SEAM_V_TOLERANCE))
    count = len(plan.groups)
    direction, offset = _offsets(plan, slot_u)
    proposal = direction[plan.slot_group] * slot_u + offset[plan.slot_group]
    # Цель - среднее предложений ВНУТРИ компоненты: вершина угла, где сходятся цепи, общая и для чужих компонент, их станции к этой цепи не относятся.
    place = plan.component[plan.slot_group] * (int(plan.slot_vertex.max(initial=0)) + 1) + plan.slot_vertex
    places, slot_place = np.unique(place, return_inverse=True)
    target = (np.bincount(slot_place, weights=proposal, minlength=places.shape[0]) / np.bincount(slot_place, minlength=places.shape[0]))[slot_place]
    # Узлы `(u, t)` всех групп одним массивом, по группе и `u`; угол ищет свой отрезок в узлах своей группы, за крайними - наклон направления.
    order = np.lexsort((slot_u, plan.slot_group))
    xs, ts, gs = slot_u[order], target[order], plan.slot_group[order]
    low = np.searchsorted(gs, np.arange(count), side="left")
    high = np.searchsorted(gs, np.arange(count), side="right") - 1
    same = gs[1:] == gs[:-1]
    folds = int(np.count_nonzero(same & (np.diff(ts) * direction[gs[1:]] < 0.0)))
    along = u.astype(np.float64).copy()
    if xs.shape[0]:
        span = float(np.max(np.abs(u), initial=0.0)) * 4.0 + 4.0  # ключ `группа * span + u` упорядочен так же, как `(группа, u)`
        loop_group = plan.loop_slot_group
        has = high[loop_group] >= low[loop_group]
        first, last = np.where(has, low[loop_group], 0), np.where(has, high[loop_group], 0)
        at = np.searchsorted(gs * span + xs, loop_group * span + u)
        right = np.clip(at, np.minimum(first + 1, last), last)
        left = np.maximum(right - 1, first)
        x0, x1, t0, t1 = xs[left], xs[right], ts[left], ts[right]
        width = x1 - x0
        mapped = t0 + (t1 - t0) * np.divide(u - x0, width, out=np.zeros(u.shape[0]), where=width != 0.0)
        slope = direction[loop_group]
        mapped = np.where(u < xs[first], ts[first] + slope * (u - xs[first]), mapped)
        mapped = np.where(u > xs[last], ts[last] + slope * (u - xs[last]), mapped)
        along = np.where(has, mapped, along)
    wing = np.where(plan.colour == 1, -1.0, 1.0)
    sign = wing[plan.loop_slot_group]
    if swap:
        # Поворот всей раскладки на 180° вокруг средней линии: крылья меняются местами, ориентация текстуры у обоих остаётся.
        sign, along = -sign, -along
    mirrored = int(np.count_nonzero(((plan.colour == 0) & (direction < 0.0)) | ((plan.colour == 1) & (direction > 0.0))))
    shift = float(np.max(np.abs(target - proposal), initial=0.0))
    numbers = {
        COUNTER_PAIRS: plan.pairs, COUNTER_UNPAIRED: plan.unpaired, COUNTER_NO_SEAM: int(np.count_nonzero(plan.colour < 0)),
        COUNTER_NOT_TWO_SIDED: plan.not_two_sided, COUNTER_MIRRORED: mirrored, COUNTER_SEAM_UV_SPLIT: split, COUNTER_FOLDS: folds,
        COUNTER_SHIFT: int(round(shift * 1e6)),
    }
    return along, sign, numbers


def _segment_swapped(mapping: HotspotMappingV1) -> bool:
    return bool(mapping.swap) and mapping.mark.kind == KIND_CORNER


def segment_atlas_uvs(uvs, mapping: HotspotMappingV1) -> np.ndarray:
    """UV атласа острова СЕГМЕНТА декали Both (BEVEL_HOTSPOT_SEGMENT_FRAME_V1) по его UV крыльев `uvs` (L, 2): `U` вдоль (UV трима, та же
    плотность, что у `V`), `V` - 0 и 1 на краях юбок, 0.5 на средней линии бевела (`bevel_wings`).

    Сегмент ложится в прямоугольник метки ОДНИМ куском: `P = низ + U * T * вправо + V * T * вверх` (`T` - толщина метки, `низ` - левый конец
    нижнего края в рамке метки), то есть `V` 0..1 - вся толщина снизу вверх, `U = 0` - левый край метки, `U` растёт вправо, единица `U` -
    толщина (изотропно: `U` и `V` крыльев в одной плотности). trim и corner - одна формула: у corner средняя линия бевела (`V = 0.5`) - средняя
    линия метки, половины (полбевела и юбка) - два крыла. Только поворот (+y, -y, +x, -x); Swap у corner - поворот на 180 градусов вокруг
    ЦЕНТРА прямоугольника метки (`U -> длина / T - U`, `V -> 1 - V`): крылья меняются местами, сегмент короче метки остаётся в ней, не отражение.
    """

    uvs = np.asarray(uvs, dtype=np.float64).reshape(-1, 2)
    thickness = float(mapping.mark.thickness)
    along, across = uvs[:, 0], uvs[:, 1]
    if _segment_swapped(mapping):
        along, across = mapping.mark.length / thickness - along, 1.0 - across
    return _frame_to_atlas(mapping, along * thickness, across * thickness, mapping.left_end(-thickness / 2.0))


def segment_frame(atlas, mapping: HotspotMappingV1) -> np.ndarray:
    """UV крыльев сегмента по его UV атласа: точное обращение `segment_atlas_uvs` (оси `вправо`, `вверх` ортонормированы)."""

    atlas = np.asarray(atlas, dtype=np.float64).reshape(-1, 2)
    thickness = float(mapping.mark.thickness)
    (ux, uy), (rx, ry) = mapping.up, mapping.right
    bx, by = mapping.left_end(-thickness / 2.0)
    px, py = atlas[:, 0] * float(mapping.width) - bx, atlas[:, 1] * float(mapping.height) - by
    along, across = (px * rx + py * ry) / thickness, (px * ux + py * uy) / thickness
    if _segment_swapped(mapping):
        along, across = mapping.mark.length / thickness - along, 1.0 - across
    return np.stack((along, across), axis=1)


def mapping_for(markup_name: str, markup, mark_id: int, swap: bool = False) -> HotspotMappingV1 | None:
    """Отображение метки `mark_id` разметки; `None` - такой метки нет."""

    mark = markup.mark(mark_id)
    return None if mark is None else HotspotMappingV1(str(markup_name), mark, int(markup.width), int(markup.height), bool(swap))


__all__ = (
    "COUNTER_FOLDS",
    "COUNTER_LOOPS",
    "COUNTER_MIRRORED",
    "COUNTER_NOT_TWO_SIDED",
    "COUNTER_NO_SEAM",
    "COUNTER_PAIRS",
    "COUNTER_SEAM_UV_SPLIT",
    "COUNTER_SHIFT",
    "COUNTER_UNPAIRED",
    "CornerPlanV1",
    "HotspotMappingV1",
    "MAPPING_FORMAT",
    "MAPPING_VERSION",
    "MESH_PROPERTY",
    "atlas_uvs",
    "cached_corner_plan",
    "corner_along",
    "corner_plan",
    "mapping_for",
    "mapping_from_json",
    "mapping_to_json",
    "segment_atlas_uvs",
    "segment_frame",
)
