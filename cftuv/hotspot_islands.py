"""Метки по островам декали Hotspot (HOTSPOT_ISLAND_MARKS_V1). Без Blender: numpy, разметка и отображение.

ОСТРОВ - группа отображения `hotspot_mapping`: сторона шва одного домена и одного владельца (`cftuv_domain`, `cftuv_owner` грани), то есть одна
сторона pChain вместе с её веерами. Выделение граней расширяется до целых островов; corner-метка - ещё и до соседей по шву (компонента плана
крыльев `corner_plan` по всему мешу): крылья бывают только парой. Метка trim одной стороне corner-пары оставляет вторую одну - она ложится
крылом без пары и названа числом HOTSPOT_CORNER_WING_UNPAIRED.

ХРАНЕНИЕ. Правда покоящегося меша - атрибут граней `cftuv_hotspot_mark` (код `метка * 2 + swap`, `NO_MARK` - UV ядра как есть): его вместе с
UV возвращает и Undo в Edit Mode, которое свойств ID не трогает. Атрибут пишется только у меша с записью `cftuv_hotspot` (режим выключен -
меш тот же побитово). UV ядра отдельно НЕ хранятся (владелец): «Apply Mark» обращает текущую метку острова (`strip_from_atlas`) и отображает
в новую, `new = M_new(M_old^-1(atlas))`. trim - аффинное отображение, обращается точно (до float32). corner обращается до кадра острова
`(выровненное along, v)`: крыло `-` (поворот на 180°) узнаётся по знаку поперечной координаты острова и поворачивается обратно, поэтому
ориентация текстуры остаётся; `along` - выровненное по общим вершинам шва (не сырое `u` ядра), годная координата вдоль для любой следующей
метки, а выравнивание corner на нём - почти тождество. Ряд шва после обращения прижимается к `v = 0` в пределах разрешения float32 атласа
(`seam_snap`). Острова, код которых не изменился (и их метка та же), сохраняют UV побитово. Ошибка float32 копится только от нажатия к
нажатию; ширина и пересборка отображают заново от UV ядра и обнуляют её.

ЗАПИСЬ v2 (`cftuv_hotspot`): метка по умолчанию (код), таблица меток (снимок прямоугольников, кадры живой ширины текста разметки не читают) и
острова НЕ по умолчанию с якорями - серединами рёбер шва острова (ребро грани острова, у которого `v = 0` на обоих углах). Шов лежит на цепи
источника, его позиции от ширины не зависят, а ребро шва внутри домена принадлежит ровно одному острову. Номера владельцев от ширины
меняются (ядро нумерует огибающие по именам, имена несут ширину), поэтому ключом они быть не могут. Пересборка (точный кадр, подтверждение,
кнопка) порождает новые грани: остров получает код той записи, чьи якоря БОЛЬШЕ чем наполовину лежат на его рёбрах шва (допуск -
`ANCHOR_TOLERANCE` диагонали меша); запись без острова - HOTSPOT_ISLAND_MARK_UNMATCHED, её грани берут метку по умолчанию. Запись v1 (одна
метка) читается как v2 без островов.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, replace

import numpy as np

from .hotspot_mapping import COUNTER_SHIFT, MAPPING_FORMAT, SEAM_V_TOLERANCE, HotspotMappingV1, atlas_uvs, cached_corner_plan, mapping_from_json
from .hotspot_markup import KIND_CORNER, HotspotMarkV1

RECORD_VERSION = 2
#: Атрибут граней: код метки острова `метка * 2 + swap` (INT).
MARK_ATTRIBUTE = "cftuv_hotspot_mark"
#: Код «без метки»: UV острова - UV ядра как есть (декаль, собранная без Hotspot, до первого «Apply Mark»).
NO_MARK = -1
#: Разрешение float32 атласа (доля единицы UV с запасом): ряд шва после обращения в пределах `SEAM_SNAP * max(W, H) / толщина` - это `v = 0`.
SEAM_SNAP = 4e-7
#: Номер группы грани: `домен * GROUP_DOMAIN_SCALE + владелец` (тот же, что у сторон шва отображения).
GROUP_DOMAIN_SCALE = 1 << 20
ANCHORS_PER_ISLAND = 8
#: Допуск якоря - доля диагонали меша (не меньше `ANCHOR_TOLERANCE_FLOOR`): позиции шва от ширины не зависят, допуск - только от округления.
ANCHOR_TOLERANCE = 1e-5
ANCHOR_TOLERANCE_FLOOR = 1e-7
MATCH_SHARE = 0.5
ANCHOR_DIGITS = 8

COUNTER_ISLANDS = "HOTSPOT_ISLANDS"
COUNTER_OFF_DEFAULT = "HOTSPOT_ISLANDS_OFF_DEFAULT"
COUNTER_UNMATCHED = "HOTSPOT_ISLAND_MARK_UNMATCHED"
COUNTER_MARK_MISSING = "HOTSPOT_ISLAND_MARK_NOT_IN_RECORD"
COUNTER_MIXED = "HOTSPOT_ISLAND_MARKS_MIXED"
COUNTER_SELECTED = "HOTSPOT_ISLANDS_SELECTED"
COUNTER_EXPANDED = "HOTSPOT_CORNER_PAIR_EXPANDED"
#: Острова записи прежнего меша, которые кнопка не перенесла: другая разметка или размер текстуры.
COUNTER_DROPPED = "HOTSPOT_ISLAND_MARKS_DROPPED"


def encode(mark_id, swap) -> int:
    return NO_MARK if mark_id is None else int(mark_id) * 2 + (1 if swap else 0)


def decode(code) -> tuple:
    return (None, False) if int(code) < 0 else (int(code) >> 1, bool(int(code) & 1))


def code_for(mark: HotspotMarkV1, swap: bool) -> int:
    """Код метки: `swap` только у corner (у trim крыльев нет, иначе одна метка дала бы два отображения)."""

    return encode(mark.id, bool(swap) and mark.kind == KIND_CORNER)


@dataclass(frozen=True, slots=True)
class IslandMarkV1:
    """Остров с меткой не по умолчанию: домен, код и якоря - середины рёбер шва (позиции меша)."""

    domain: int
    code: int
    anchors: tuple


@dataclass(frozen=True, slots=True)
class HotspotRecordV2:
    markup: str
    width: int
    height: int
    default: int
    marks: tuple  # HotspotMarkV1 по возрастанию id
    islands: tuple = ()

    def mark(self, mark_id):
        return None if mark_id is None else next((mark for mark in self.marks if mark.id == int(mark_id)), None)

    def known(self, code) -> bool:
        """Код без метки (UV ядра) или с меткой из таблицы."""

        return int(code) == NO_MARK or self.mapping(code) is not None

    def mapping(self, code) -> HotspotMappingV1 | None:
        mark_id, swap = decode(code)
        mark = self.mark(mark_id)
        return None if mark is None else HotspotMappingV1(self.markup, mark, int(self.width), int(self.height), swap)

    def with_marks(self, marks) -> "HotspotRecordV2":
        """Таблица с метками `marks` поверх прежних (по id)."""

        table = {mark.id: mark for mark in self.marks}
        table.update({mark.id: mark for mark in marks})
        return replace(self, marks=tuple(table[key] for key in sorted(table)))


def record_of(mapping: HotspotMappingV1) -> HotspotRecordV2:
    """Запись кнопки: метка по умолчанию и её строка таблицы, островов нет."""

    return HotspotRecordV2(str(mapping.markup), int(mapping.width), int(mapping.height), code_for(mapping.mark, mapping.swap), (mapping.mark,))


def _mark_payload(mark: HotspotMarkV1) -> dict:
    return {"id": mark.id, "name": mark.name, "kind": mark.kind, "rect": [mark.x, mark.y, mark.w, mark.h], "up": mark.up, "band": mark.band}


def _mark_of(item) -> HotspotMarkV1:
    x, y, w, h = item["rect"]
    return HotspotMarkV1(int(item["id"]), str(item["name"]), str(item["kind"]), int(x), int(y), int(w), int(h), str(item["up"]), int(item["band"]))


def record_to_json(record: HotspotRecordV2) -> str:
    mark_id, swap = decode(record.default)
    islands = []
    for island in record.islands:
        island_mark, island_swap = decode(island.code)
        islands.append({"domain": int(island.domain), "mark": island_mark, "swap": island_swap, "anchors": [[round(float(value), ANCHOR_DIGITS) for value in point] for point in island.anchors]})
    payload = {
        "format": MAPPING_FORMAT, "version": RECORD_VERSION, "markup": record.markup, "texture": [int(record.width), int(record.height)],
        "default": {"mark": mark_id, "swap": swap}, "marks": [_mark_payload(mark) for mark in record.marks], "islands": islands,
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def record_from_json(text: str) -> HotspotRecordV2:
    """Запись v2 либо v1 (одна метка) как v2 без островов; иное - `ValueError`. Метка по умолчанию обязана быть в таблице."""

    data = json.loads(text)
    if not isinstance(data, dict) or data.get("format") != MAPPING_FORMAT:
        raise ValueError(f"not a {MAPPING_FORMAT} record")
    if data.get("version") == 1:
        return record_of(mapping_from_json(text))
    if data.get("version") != RECORD_VERSION:
        raise ValueError(f"{MAPPING_FORMAT} version {data.get('version')!r} is not 1 or {RECORD_VERSION}")
    width, height = data["texture"]
    marks = tuple(sorted((_mark_of(item) for item in data["marks"]), key=lambda mark: mark.id))
    islands = tuple(
        IslandMarkV1(int(item["domain"]), encode(item["mark"], item["swap"]), tuple(tuple(float(value) for value in point) for point in item["anchors"]))
        for item in data["islands"]
    )
    record = HotspotRecordV2(str(data["markup"]), int(width), int(height), encode(data["default"]["mark"], data["default"]["swap"]), marks, islands)
    if not record.known(record.default):
        raise ValueError(f"the default mark {decode(record.default)[0]} is not in the record")
    return record


def unmapped_record(markup_name: str, width: int, height: int) -> HotspotRecordV2:
    """Запись декали, собранной без Hotspot, при первом «Apply Mark»: по умолчанию - UV ядра (`NO_MARK`), таблица пуста."""

    return HotspotRecordV2(str(markup_name), int(width), int(height), NO_MARK, ())


# --------------------------------------------------------------------------
# Острова и якоря
# --------------------------------------------------------------------------


def face_groups(face_domain, face_owner) -> np.ndarray:
    return np.asarray(face_domain, dtype=np.int64) * GROUP_DOMAIN_SCALE + np.asarray(face_owner, dtype=np.int64)


def loop_next(totals) -> np.ndarray:
    """Следующий угол той же грани у каждого угла (за последним - первый)."""

    totals = np.asarray(totals, dtype=np.int64)
    starts = np.repeat(np.cumsum(totals) - totals, totals)
    index = np.arange(starts.shape[0], dtype=np.int64)
    following = index + 1
    last = following == starts + np.repeat(totals, totals)
    following[last] = starts[last]
    return following


def seam_points(strip_v, loop_vertex, following, loop_group, positions) -> tuple:
    """`(точки (K, 3), группа точки (K))`: середины рёбер шва каждого острова; остров без ребра шва - его вершины шва, иначе все его вершины."""

    positions = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    loop_vertex = np.asarray(loop_vertex, dtype=np.int64)
    loop_group = np.asarray(loop_group, dtype=np.int64)
    seam = np.abs(np.asarray(strip_v, dtype=np.float64)) <= SEAM_V_TOLERANCE
    edge = seam & seam[following]
    a, b = loop_vertex[edge], loop_vertex[following[edge]]
    keep = a != b
    points, groups = [(positions[a[keep]] + positions[b[keep]]) * 0.5], [loop_group[edge][keep]]
    missing = np.setdiff1d(np.unique(loop_group), groups[0])
    if missing.shape[0]:
        lonely = np.isin(loop_group, missing)
        has_seam = np.isin(loop_group, np.unique(loop_group[lonely & seam]))
        pick = lonely & (seam | ~has_seam)
        pairs = np.unique(np.stack((loop_group[pick], loop_vertex[pick]), axis=1), axis=0)
        points.append(positions[pairs[:, 1]])
        groups.append(pairs[:, 0])
    return np.concatenate(points), np.concatenate(groups)


def anchor_tolerance(positions) -> float:
    positions = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    if not positions.shape[0]:
        return ANCHOR_TOLERANCE_FLOOR
    return max(ANCHOR_TOLERANCE_FLOOR, ANCHOR_TOLERANCE * float(np.linalg.norm(positions.max(axis=0) - positions.min(axis=0))))


def island_anchors(points, groups, wanted) -> dict:
    """`{группа: якоря}` групп `wanted`: до `ANCHORS_PER_ISLAND` точек острова, равномерно по их порядку (лексикографически по позиции)."""

    order = np.lexsort((points[:, 2], points[:, 1], points[:, 0], groups))
    points, groups = points[order], groups[order]
    anchors = {}
    for group in np.asarray(wanted, dtype=np.int64).tolist():
        low, high = np.searchsorted(groups, group, side="left"), np.searchsorted(groups, group, side="right")
        count = high - low
        if not count:
            anchors[group] = ()
            continue
        picks = np.unique(np.round(np.linspace(0, count - 1, min(count, ANCHORS_PER_ISLAND))).astype(np.int64))
        anchors[group] = tuple(tuple(float(value) for value in point) for point in points[low + picks])
    return anchors


def match_islands(islands, points, groups, tolerance: float) -> tuple:
    """`({группа: код}, записей без острова)`: остров берёт код записи, чьих якорей у его точек больше `MATCH_SHARE` (лучшая доля, затем ранняя запись)."""

    domains = groups // GROUP_DOMAIN_SCALE
    by_domain: dict = {}
    best: dict = {}
    limit = float(tolerance) ** 2
    for index, island in enumerate(islands):
        anchors = np.asarray(island.anchors, dtype=np.float64).reshape(-1, 3)
        if island.domain not in by_domain:
            mine = domains == island.domain
            by_domain[island.domain] = (points[mine], groups[mine])
        candidates, owners = by_domain[island.domain]
        if not anchors.shape[0] or not candidates.shape[0]:
            continue
        near = ((anchors[:, None, :] - candidates[None, :, :]) ** 2).sum(axis=2) <= limit
        rows, columns = np.nonzero(near)
        hits = np.unique(np.stack((rows, owners[columns]), axis=1), axis=0)
        found, counts = np.unique(hits[:, 1], return_counts=True)
        for group, count in zip(found.tolist(), counts.tolist()):
            share = count / anchors.shape[0]
            if share > MATCH_SHARE and (group not in best or share > best[group][0]):
                best[group] = (share, index)
    chosen = {index for _share, index in best.values()}
    return {group: islands[index].code for group, (_share, index) in best.items()}, len(islands) - len(chosen)


def _group_majority(face_index, codes, count: int) -> tuple:
    """`(код каждой группы, групп с разными кодами)`: код большинства граней группы (при равенстве - меньший)."""

    pairs, votes = np.unique(np.stack((face_index, codes), axis=1), axis=0, return_counts=True)
    order = np.lexsort((pairs[:, 1], -votes, pairs[:, 0]))
    first = np.r_[True, pairs[order][1:, 0] != pairs[order][:-1, 0]]
    result = np.zeros(count, dtype=np.int64)
    result[pairs[order][first, 0]] = pairs[order][first, 1]
    return result, int(np.count_nonzero(np.bincount(pairs[:, 0], minlength=count) > 1))


def island_codes(record: HotspotRecordV2, face_group, face_codes=None, points=None, groups=None, tolerance: float = ANCHOR_TOLERANCE_FLOOR) -> tuple:
    """`(код каждой грани, числа)`: из атрибута граней (меш тот же - правда в нём, остров выравнивается по большинству), иначе по якорям записи."""

    face_group = np.asarray(face_group, dtype=np.int64)
    unique, face_index = np.unique(face_group, return_inverse=True)
    counters = {COUNTER_ISLANDS: int(unique.shape[0]), COUNTER_UNMATCHED: 0, COUNTER_MIXED: 0}
    if face_codes is not None:
        group_code, counters[COUNTER_MIXED] = _group_majority(face_index, np.asarray(face_codes, dtype=np.int64), unique.shape[0])
    else:
        by_group, counters[COUNTER_UNMATCHED] = match_islands(record.islands, points, groups, tolerance) if record.islands else ({}, 0)
        group_code = np.array([by_group.get(group, record.default) for group in unique.tolist()], dtype=np.int64)
    counters[COUNTER_OFF_DEFAULT] = int(np.count_nonzero(group_code != record.default))
    return group_code[face_index], counters


def _merge(total: dict, numbers: dict) -> None:
    for key, value in numbers.items():
        total[key] = max(total.get(key, value), value) if key == COUNTER_SHIFT else total.get(key, 0) + value


def map_islands(uvs, loop_vertex, loop_group, loop_code, record: HotspotRecordV2) -> tuple:
    """`(UV атласа (L, 2), числа)`: углы каждого кода - отдельным вызовом `atlas_uvs` его метки, поэтому крылья пары сходятся только у сторон с
    одной corner-меткой и одним swap; `NO_MARK` - UV ядра как есть. Код без метки в таблице - метка по умолчанию (HOTSPOT_ISLAND_MARK_NOT_IN_RECORD,
    число островов)."""

    uvs = np.asarray(uvs).reshape(-1, 2)
    loop_vertex = np.asarray(loop_vertex, dtype=np.int64)
    loop_group = np.asarray(loop_group, dtype=np.int64)
    loop_code = np.asarray(loop_code, dtype=np.int64)
    codes = np.unique(loop_code)
    lost = np.isin(loop_code, codes[~np.array([record.known(code) for code in codes.tolist()], dtype=bool)])
    counters = {COUNTER_MARK_MISSING: int(np.unique(loop_group[lost]).shape[0])}
    loop_code = np.where(lost, record.default, loop_code)
    codes = np.unique(loop_code)
    if codes.shape[0] == 1 and int(codes[0]) != NO_MARK:
        atlas, numbers = atlas_uvs(uvs, loop_vertex, loop_group, record.mapping(int(codes[0])))
        _merge(counters, numbers)
        return atlas, counters
    atlas = uvs.astype(np.float64)
    for code in codes.tolist():
        if code == NO_MARK:
            continue
        rows = loop_code == code
        part, numbers = atlas_uvs(uvs[rows], loop_vertex[rows], loop_group[rows], record.mapping(code))
        atlas[rows] = part
        _merge(counters, numbers)
    return atlas, counters


def seam_snap(mapping: HotspotMappingV1) -> float:
    """Допуск ряда шва `v` после обращения метки (в ширинах): разрешение float32 атласа в пикселях на толщину крыла или полосы."""

    scale = mapping.mark.thickness / (2.0 if mapping.mark.kind == KIND_CORNER else 1.0)
    return max(SEAM_V_TOLERANCE, SEAM_SNAP * max(mapping.width, mapping.height) / max(float(scale), 1.0))


def _invert(mapping: HotspotMappingV1, atlas) -> np.ndarray:
    """`(along, across)` в единицах ширины по UV атласа одной метки: обращение `_frame_to_atlas` (оси `вправо`, `вверх` ортонормированы)."""

    (ux, uy), (rx, ry) = mapping.up, mapping.right
    mark = mapping.mark
    if mark.kind == KIND_CORNER:
        scale, (bx, by) = mark.thickness / 2.0, mapping.left_end(0.0)
    else:
        scale, (bx, by) = float(mark.thickness), mapping.left_end(-mark.thickness / 2.0)
    px, py = atlas[:, 0] * float(mapping.width) - bx, atlas[:, 1] * float(mapping.height) - by
    return np.stack(((px * rx + py * ry) / scale, (px * ux + py * uy) / scale), axis=1)


def strip_from_atlas(atlas, loop_group, loop_code, record: HotspotRecordV2) -> np.ndarray:
    """UV ядра острова (кадр `(along, v)`) по его UV атласа и коду: `NO_MARK` - как есть; trim - точное обращение; corner - обращение, и остров в
    половине `-` (средняя поперечная координата меньше нуля: крыло `-` или Swap) поворачивается на 180° обратно. Ряд шва прижат к `v = 0`."""

    atlas = np.asarray(atlas, dtype=np.float64).reshape(-1, 2)
    loop_group = np.asarray(loop_group, dtype=np.int64)
    loop_code = np.asarray(loop_code, dtype=np.int64)
    strip = atlas.copy()
    for code in np.unique(loop_code).tolist():
        mapping = record.mapping(code)
        if mapping is None:
            continue
        rows = np.flatnonzero(loop_code == code)
        frame = _invert(mapping, atlas[rows])
        if mapping.mark.kind == KIND_CORNER:
            groups, local = np.unique(loop_group[rows], return_inverse=True)
            below = (np.bincount(local, weights=frame[:, 1], minlength=groups.shape[0]) < 0.0)[local]
            frame[below] = -frame[below]
        frame[np.abs(frame[:, 1]) <= seam_snap(mapping), 1] = 0.0
        strip[rows] = frame
    return strip


def paired_corner_groups(loop_vertex, loop_group, loop_code, strip_v, record: HotspotRecordV2) -> np.ndarray:
    """Острова corner-меток, у которых есть пара по шву в своей метке (компонента плана крыльев из двух и больше сторон): их `along` выровнен."""

    loop_vertex, loop_group, loop_code = (np.asarray(item, dtype=np.int64) for item in (loop_vertex, loop_group, loop_code))
    seam = np.abs(np.asarray(strip_v, dtype=np.float64)) <= SEAM_V_TOLERANCE
    found = []
    for code in np.unique(loop_code).tolist():
        mapping = record.mapping(code)
        if mapping is None or mapping.mark.kind != KIND_CORNER:
            continue
        rows = loop_code == code
        plan = cached_corner_plan(loop_vertex[rows], loop_group[rows], seam[rows])
        roots, sizes = np.unique(plan.component[plan.component >= 0], return_counts=True)
        paired = np.isin(plan.component, roots[sizes >= 2])
        found.append(np.asarray(plan.groups, dtype=np.int64)[paired])
    return np.concatenate(found) if found else np.zeros(0, dtype=np.int64)


def unwarp_paired(strip, loop_vertex, loop_group, groups, positions, alpha: float) -> tuple:
    """`(кадр, (островов, наибольший шаг ряда))`: `along` выровненных сторон пары -> `u` ядра. Ряд шва (`v = 0`) острова по возрастанию `along`;
    `u` ядра на нём - длина ряда от его начала / alpha (UNFOLD_ALONG_U_V1: ряд шва держит `u = станция / alpha`, начало ряда - `u = 0`);
    выравнивание кусочно-линейно по `u` с узлами на вершинах ряда, поэтому обращается по ним же, за крайними узлами - наклон 1."""

    strip = np.array(strip, dtype=np.float64).reshape(-1, 2)
    loop_vertex, loop_group = np.asarray(loop_vertex, dtype=np.int64), np.asarray(loop_group, dtype=np.int64)
    positions = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    done = 0
    for group in np.asarray(groups, dtype=np.int64).tolist():
        rows = np.flatnonzero(loop_group == group)
        row = rows[strip[rows, 1] == 0.0]
        vertices, first = np.unique(loop_vertex[row], return_index=True)
        if vertices.shape[0] < 2 or not alpha > 0.0:
            continue
        order = np.argsort(strip[row[first], 0], kind="stable")
        knots = strip[row[first], 0][order]
        points = positions[vertices[order]]
        kernel = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))] / float(alpha)
        along = strip[rows, 0]
        mapped = np.interp(along, knots, kernel)
        mapped = np.where(along < knots[0], kernel[0] + (along - knots[0]), mapped)
        mapped = np.where(along > knots[-1], kernel[-1] + (along - knots[-1]), mapped)
        strip[rows, 0] = mapped
        done += 1
    return strip, done


def remap_atlas(atlas, loop_vertex, loop_group, old_code, new_code, old: HotspotRecordV2, new: HotspotRecordV2, strip=None) -> tuple:
    """`(UV атласа, числа)` после смены кодов: `M_new(M_old^-1(atlas))` (либо от готового `strip`). Углы кода, чей состав островов и метка не
    изменились, остаются побитово (пары corner не трогаются без нужды, ошибка float32 не копится на нетронутых)."""

    atlas = np.asarray(atlas).reshape(-1, 2)
    old_code, new_code = np.asarray(old_code, dtype=np.int64), np.asarray(new_code, dtype=np.int64)
    loop_group = np.asarray(loop_group, dtype=np.int64)
    strip = strip_from_atlas(atlas, loop_group, old_code, old) if strip is None else strip
    mapped, counters = map_islands(strip, loop_vertex, loop_group, new_code, new)
    out = mapped
    for code in np.unique(new_code).tolist():
        rows = new_code == code
        same_islands = np.array_equal(np.unique(loop_group[rows]), np.unique(loop_group[old_code == code]))
        if same_islands and old.mapping(code) == new.mapping(code) and old.known(code):
            out[rows] = atlas[rows]
    return out, counters


def remember_islands(record: HotspotRecordV2, face_group, face_codes, points, groups) -> HotspotRecordV2:
    """Запись с островами НЕ по умолчанию по кодам граней: домен, код, якоря; острова по умолчанию не пишутся."""

    face_group = np.asarray(face_group, dtype=np.int64)
    unique, face_index = np.unique(face_group, return_inverse=True)
    group_code, _mixed = _group_majority(face_index, np.asarray(face_codes, dtype=np.int64), unique.shape[0])
    off = group_code != record.default
    anchors = island_anchors(points, groups, unique[off])
    islands = tuple(IslandMarkV1(int(group) // GROUP_DOMAIN_SCALE, int(code), anchors[int(group)]) for group, code in zip(unique[off].tolist(), group_code[off].tolist()))
    return replace(record, islands=islands)


def carry_islands(old: HotspotRecordV2, new: HotspotRecordV2, fresh=()) -> tuple:
    """`(запись, островов не перенесено)` кнопки поверх прежнего меша: метка по умолчанию - новая, острова прежнего - их метки (свежие из разметки
    `fresh`, иначе снимок прежней таблицы). Другая разметка или размер текстуры - острова не переносятся (HOTSPOT_ISLAND_MARKS_DROPPED)."""

    if (old.markup, old.width, old.height) != (new.markup, new.width, new.height):
        return new, len(old.islands)
    islands = tuple(island for island in old.islands if island.code != new.default)
    wanted = {decode(island.code)[0] for island in islands} - {mark.id for mark in new.marks} - {None}
    fresh_by_id = {mark.id: mark for mark in fresh}
    marks = [fresh_by_id.get(mark_id) or old.mark(mark_id) for mark_id in sorted(wanted)]
    return replace(new.with_marks([mark for mark in marks if mark is not None]), islands=islands), 0


def refresh_marks(record: HotspotRecordV2, markup) -> HotspotRecordV2:
    """Таблица записи по текущей разметке: каждая метка, что в разметке есть, - её прямоугольник; ушедшая из разметки остаётся снимком."""

    return record.with_marks([mark for mark in (markup.mark(item.id) for item in record.marks) if mark is not None])


# --------------------------------------------------------------------------
# Выделение
# --------------------------------------------------------------------------


def chosen_islands(face_group, face_selected, kind: str, loop_vertex, loop_group, strip_v) -> tuple:
    """`(группы, добавлено соседей по шву)`: острова выделенных граней; у corner-метки - и все острова их компонент плана крыльев (пара шва)."""

    face_group = np.asarray(face_group, dtype=np.int64)
    picked = np.unique(face_group[np.asarray(face_selected, dtype=bool)])
    if kind != KIND_CORNER or not picked.shape[0]:
        return picked, 0
    seam = np.abs(np.asarray(strip_v, dtype=np.float64)) <= SEAM_V_TOLERANCE
    plan = cached_corner_plan(np.asarray(loop_vertex, dtype=np.int64), np.asarray(loop_group, dtype=np.int64), seam)
    groups = np.asarray(plan.groups, dtype=np.int64)
    roots = plan.component[np.searchsorted(groups, picked)]
    partners = groups[np.isin(plan.component, roots[roots >= 0])]
    result = np.union1d(picked, partners)
    return result, int(result.shape[0] - picked.shape[0])


def assign(face_codes, face_group, groups, code: int) -> np.ndarray:
    codes = np.asarray(face_codes, dtype=np.int64).copy()
    codes[np.isin(np.asarray(face_group, dtype=np.int64), groups)] = int(code)
    return codes


def code_label(record: HotspotRecordV2, code: int) -> str:
    mark_id, swap = decode(code)
    if mark_id is None:
        return "no mark (kernel strip UV)"
    mark = record.mark(mark_id)
    if mark is None:
        return f"{mark_id}: ? ({COUNTER_MARK_MISSING})"
    return f"{mark.id}: {mark.name} ({mark.kind}{', swapped' if swap else ''})"


def selection_text(record: HotspotRecordV2, codes) -> str:
    """Строка панели: метка выделенных граней, несколько - `mixed: ...`, пусто - пустая строка."""

    return labels_text(selection_labels(record, codes))


def selection_labels(record: HotspotRecordV2, codes) -> list:
    """Подписи меток кодов граней по возрастанию кода, без повторов."""

    return [code_label(record, code) for code in sorted({int(code) for code in codes})]


def labels_text(labels) -> str:
    """Одна подпись - она сама, несколько (без повторов, порядок первых появлений) - `mixed: ...`; в Object Mode - подписи всех выделенных декалей."""

    unique = list(dict.fromkeys(labels))
    if not unique:
        return ""
    return unique[0] if len(unique) == 1 else "mixed: " + ", ".join(unique)


__all__ = (
    "ANCHORS_PER_ISLAND",
    "COUNTER_DROPPED",
    "COUNTER_EXPANDED",
    "COUNTER_ISLANDS",
    "COUNTER_MARK_MISSING",
    "COUNTER_MIXED",
    "COUNTER_OFF_DEFAULT",
    "COUNTER_SELECTED",
    "COUNTER_UNMATCHED",
    "GROUP_DOMAIN_SCALE",
    "HotspotRecordV2",
    "IslandMarkV1",
    "MARK_ATTRIBUTE",
    "NO_MARK",
    "RECORD_VERSION",
    "SEAM_SNAP",
    "anchor_tolerance",
    "assign",
    "carry_islands",
    "chosen_islands",
    "code_for",
    "code_label",
    "decode",
    "encode",
    "face_groups",
    "island_anchors",
    "island_codes",
    "loop_next",
    "map_islands",
    "match_islands",
    "record_from_json",
    "record_of",
    "record_to_json",
    "refresh_marks",
    "paired_corner_groups",
    "remap_atlas",
    "remember_islands",
    "seam_points",
    "seam_snap",
    "labels_text",
    "selection_labels",
    "selection_text",
    "strip_from_atlas",
    "unmapped_record",
    "unwarp_paired",
)
