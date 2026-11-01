"""Писатель продуктового меша: `GeometryBatchV1` доменов -> ОДИН объект Blender.

Адаптер ОТОБРАЖАЕТ контракт и не чинит геометрию (AGENTS.md): позиции, UV,
владельцы и цепи берутся из батча как есть. Хосту принадлежит ровно одно
решение — политика отображения: смещение над поверхностью вдоль нормали патча
(борьба с z-fighting; прежний декаль-режим держал 0.02), материал слота и имя
объекта. Всё остальное — запись фактов.

ФОРМА РЕЗУЛЬТАТА. Один объект `<исходный>.CFTUV_Decal`, потомок исходного с
единичным локальным преобразованием (позиции батча — в локальных координатах
источника, и родитель переводит их в мир сам), ВСЕ домены в одном меше. Грани —
многоугольники ЛЮБОЙ длины от трёх: под законом топологии `QUAD_STRIPS_V1` это
четырёхгранники лент и треугольники вееров, и писатель кладёт их как есть, без
собственной триангуляции (разрез грани на треугольники — решение ядра, названное
счётчиком, а не Blender по float). UV —
слой `UVMap`, по вершине каждой петли из `uv_facts` грани. Целочисленные
атрибуты граней: `cftuv_domain` (номер патча) и `cftuv_owner` (порядковый номер
огибающей внутри домена, `claim:N` батча). Один слот материала `material_name`:
создаётся, если нет, и НЕ перезаписывается, если есть (владелец мог его настроить).

ВЕРШИНЫ СВАРИВАЮТСЯ ТОЛЬКО ПО СЕМАНТИЧЕСКОЙ ССЫЛКЕ БАТЧА. Внутри домена ключ
`(домен, vert_key)` — одна вершина. МЕЖДУ доменами вершины с одной ссылкой
`semantic_location_ref = location:src:<id>` (она глобальна: вершина исходника) становятся
одной вершиной меша, но ТОЛЬКО при ПОБИТОВО равных позициях в батчах — это проверка, а не
ремонт (закон ядра `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` кладёт такие вершины в одну
позицию хоста); расхождение — счётчик `ADAPTER_WELD_POSITION_MISMATCH`, вершины остаются
раздельными. Смещение над поверхностью у общей вершины — МИТРА (пересечение сдвинутых
плоскостей доменов), а не нормаль одного домена: см. `envelope_production_weld`. По
расстоянию вершины не сливаются никогда. Одна вершина на ключ при разных UV в разных
регионах — шов UV, а не дубликат; UV остаётся по петле, и ребро между гранями разных доменов
с разрывным UV помечается швом.

ШВЫ. Интерфейсные цепи батча (общие полурёбра граней разных регионов — изломы
цепей и границы владения, где станции `(s, r)` разные) помечаются швами Blender
(`use_seam`): это ровно те рёбра, на которых UV разрывны. Туда же — рёбра сваренной складки
между доменами, где UV двух граней разные. Решение хоста, не ядра.

ПЕРЕСБОРКА ИДЕМПОТЕНТНА. Свой объект ищется по МАРКЕРУ, а не по имени: имя
объекта Blender режет до 63 байт и дописывает `.001` при коллизии, поэтому
`decal.name != ожидаемое` — штатный случай (длинное имя источника), и поиск по
имени создавал бы новый объект на каждом нажатии. Свой — объект с маркером
`cftuv_source_revision`, чей родитель этот источник либо чья метка
`cftuv_source_object` называет его. Он получает НОВЫЙ меш (старый удаляется, если
им никто не пользуется); объект, потомки, материал и ручные настройки остаются.
Родитель и коллекция при пересборке восстанавливаются, а уход называется. Объект
с ожидаемым именем БЕЗ маркера не трогается — именованный отказ
`DECAL_OBJECT_NAME_TAKEN`.

НИЧЕГО НЕ ПРОПАДАЕТ МОЛЧА. Домен, который не удалось записать (отказ продуктового
пути, пустой батч, нечисловая позиция или UV, отсутствующая нормаль, вершина грани
без записи), попадает в `skipped` квитанции вместе с исходом. Цена — вместо
угадывания: такой домен в меше отсутствует, а квитанция говорит какой и почему.
Домен, чья нормаль плоскости смотрит ПРОТИВ нормали исходной грани, тоже не пишется
(`ADAPTER_NORMAL_OPPOSES_SOURCE`): смещение втолкнуло бы декаль в стену. Мягкие
находки (счётчик ядра «перевёрнутые к источнику» не нулевой, шов не поставился,
общий меш, дрейф родителя) идут в `warnings` квитанции и в консоль, а не
теряются.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field

import bpy
import numpy as np

from cftuv_envelope.mesh_glue import glue_domains

from .envelope_production_weld import (
    COUNTER_SEAM_CLIP_VERTICES,
    COUNTER_SEAM_T_JUNCTIONS,
    COUNTER_WELD_SEAMS_MARKED,
    OUTCOME_SEAM_T_JUNCTIONS,
    OUTCOME_WELD_HALF_EDGE_CONFLICT,
    WELD_LAW,
    weld_of_answer,
    weld_vertices,
)
from .envelope_mesh_assembly import (
    MESH_ASSEMBLY_NATIVE,
    cross_domain_seams,
    mesh_assembly_backend,
    mesh_reports,
    native_answer,
    record_fallback,
    report_counters,
)
from .envelope_view_bytes import view_bytes_failed
from .envelope_production_view import (
    OUTCOME_EMPTY_BATCH,
    OUTCOME_NON_FINITE,
    OUTCOME_NORMAL_MISSING,
    OUTCOME_NORMAL_OPPOSES_SOURCE,
    OUTCOME_VERTEX_MISSING,
    build_domain_view,
    structure_token,
)

DECAL_OBJECT_SUFFIX = ".CFTUV_Decal"
DECAL_UV_LAYER = "UVMap"
DECAL_DOMAIN_ATTRIBUTE = "cftuv_domain"
DECAL_OWNER_ATTRIBUTE = "cftuv_owner"
DECAL_REVISION_PROPERTY = "cftuv_source_revision"
DECAL_SOURCE_PROPERTY = "cftuv_source_object"
#: Ширина (alpha), с которой записан меш: свойство МЕША, а не объекта, чтобы Undo/Redo восстанавливали её вместе с
#: геометрией (свойство объекта могло бы остаться от другого шага). Нужна живой ширине: после истории она
#: сверяется со значением ползунка (`envelope_width_live.reconcile_after_history`).
DECAL_WIDTH_PROPERTY = "cftuv_decal_width"
#: Смещение над поверхностью по умолчанию, метры: прежнее значение декаль-режима.
DEFAULT_DECAL_OFFSET = 0.02
DEFAULT_DECAL_MATERIAL = "CFTUV_Decal"

#: Имя ID Blender — не больше 63 байт UTF-8 (`MAX_ID_NAME - 2`).
ID_NAME_LIMIT_BYTES = 63

OUTCOME_NAME_TAKEN = "DECAL_OBJECT_NAME_TAKEN"
#: Перезапись меша на месте (живая ширина) не создаёт объектов: нет объекта либо он в Edit — отказ по имени.
OUTCOME_DECAL_MISSING = "DECAL_OBJECT_MISSING"
OUTCOME_DECAL_IN_EDIT_MODE = "DECAL_OBJECT_IN_EDIT_MODE"
OUTCOME_DECAL_MESH_MISSING = "DECAL_MESH_MISSING"
OUTCOME_MATERIAL_MISSING = "ADAPTER_MATERIAL_MISSING"
#: Предупреждения (домен остаётся в меше, находка названа).
OUTCOME_SOURCE_NORMAL_UNKNOWN = "ADAPTER_SOURCE_NORMAL_UNKNOWN"
OUTCOME_FLIPPED_VS_SOURCE = "MATERIALIZE_TRIANGLES_FLIPPED_VS_SOURCE"
OUTCOME_SEAM_EDGE_MISSING = "ADAPTER_SEAM_EDGE_MISSING"
OUTCOME_MESH_SHARED = "ADAPTER_MESH_DATABLOCK_SHARED"
#: Lean-массивы (`build_mesh_arrays(reports=False)`, WIDTH_REFRESH_LEAN_V1) несут геометрию, но не отчёты и дайджест: квитанцию записи по ним не выдать.
OUTCOME_LEAN_ARRAYS = "LEAN_ARRAYS_HAVE_NO_RECEIPT"
OUTCOME_PARENT_REASSERTED = "ADAPTER_DECAL_PARENT_REASSERTED"
OUTCOME_COLLECTION_REASSERTED = "ADAPTER_DECAL_COLLECTION_REASSERTED"
OUTCOME_DUPLICATE_DECALS = "ADAPTER_DUPLICATE_DECAL_OBJECTS"


class ProductionWriteError(RuntimeError):
    """Писатель отказал именованным исходом; `outcome` — его имя."""

    def __init__(self, outcome: str, message: str):
        super().__init__(f"{outcome}: {message}")
        self.outcome = outcome


@dataclass(frozen=True, slots=True)
class MeshArraysV1:
    """Что уйдёт в меш: плоские массивы без Blender, их дайджест и пропуски."""

    positions: tuple
    faces: tuple
    uvs: tuple
    face_domain: tuple
    face_owner: tuple
    seam_edges: tuple
    #: Номера патчей, чьи батчи вошли в массивы, по возрастанию.
    domains: tuple
    #: `(patch_id, domain_id, исход, деталь)` каждого домена, которого в мешe нет.
    skipped: tuple
    #: `(patch_id | None, исход, деталь)` мягких находок, домен при этом записан.
    warnings: tuple
    source_revision: str
    digest: str
    #: Закон топологии записанных доменов (`DecalTopologyLawV1.value`); при разных
    #: законах — имена через запятую по возрастанию; пусто, если ничего не записано.
    decal_topology_law: str = ""
    #: Числа сварки `((имя, значение), ...)`: общие вершины, слитые вершины доменов,
    #: расхождения позиций, отказы митры, швы складок, конфликты обхода.
    weld_counters: tuple = ()
    #: Плоскость граней от четырёх вершин ПОСЛЕ смещения: наибольшее отклонение (нм) и число граней
    #: с отклонением. Запись, а не суд (порога нет): грань куска плоская в батче, а смещение вдоль
    #: нормалей вершин развёртки её искривляет.
    offset_counters: tuple = ()
    #: Шов по цепям батчей: T-стыки между доменами и вершины `clip:` на шовных цепях (`seam_report`).
    seam_counters: tuple = ()
    #: Состав меша по ЗАПИСАННЫМ доменам в порядке записи (читает модель превью ширины, `envelope_width_preview_model`): `(патч,
    #: домен)`, номера вершин меша по локальным вершинам домена, число петель UV (петли домена лежат в `uvs` подряд) и токен
    #: локальной структуры (грани, швы, ссылки вершин; чисел в нём нет). В дайджест не входят.
    domain_keys: tuple = ()
    domain_vertex_index: tuple = ()
    domain_loop_counts: tuple = ()
    domain_tokens: tuple = ()
    #: По записанным доменам (в порядке `domain_keys`): номера доменов-соседей, чьи нормали входят в смещение узлов стены этого домена (`WeldV1.partners`). В дайджест не входят.
    domain_partners: tuple = ()
    #: `False` у lean-массивов (WIDTH_REFRESH_LEAN_V1): позиции, грани, UV, владельцы и швы те же побитово, а дайджест, счётчик обхода рёбер, отчёты о плоскости и шве
    #: не считались (`digest` пуст, `offset_counters` и `seam_counters` пусты). Писатель квитанции (`_receipt`) такие массивы не принимает.
    reports: bool = True
    #: Смещение декали над поверхностью (метры), с которым построены позиции: его читает модель превью ширины (предел ошибки кадра домена-развёртки, `PREVIEW_DEVELOPABLE_FROZEN_NORMAL_V1`). В дайджест не входит.
    offset: float = 0.0
    #: MESH_ASSEMBLY_BUFFERS_V1: плоские массивы numpy нативной сборки (`MeshBuffersV1`) либо `None` (эталон). У массивов с буферами кортежи `positions`, `faces`, `uvs`, `face_domain`,
    #: `face_owner`, `seam_edges` и `domain_vertex_index` не строятся, пока их не прочтут: первое чтение разворачивает их из буферов в те же кортежи (побитово), что строит эталон.
    buffers: object = field(default=None, compare=False, repr=False)

    def __getattr__(self, name):
        if name not in _LAZY_ARRAYS:
            raise AttributeError(f"{type(self).__name__!r} object has no attribute {name!r}")
        value = getattr(self.buffers, name)()
        object.__setattr__(self, name, value)
        return value

    @property
    def vertex_count(self) -> int:
        return len(self.positions) if self.buffers is None else self.buffers.vertex_count

    @property
    def face_count(self) -> int:
        return len(self.faces) if self.buffers is None else self.buffers.face_count

    @property
    def loop_count(self) -> int:
        return len(self.uvs) if self.buffers is None else self.buffers.loop_count

    @property
    def seam_count(self) -> int:
        return len(self.seam_edges) if self.buffers is None else self.buffers.seam_count

    def corner_counts(self) -> tuple:
        """`(четырёхгранники, треугольники, многоугольники от пяти углов)`."""

        if self.buffers is not None:
            return self.buffers.corner_counts()
        return (
            sum(1 for loop in self.faces if len(loop) == 4),
            sum(1 for loop in self.faces if len(loop) == 3),
            sum(1 for loop in self.faces if len(loop) > 4),
        )


#: Поля `MeshArraysV1`, которые у массивов с буферами разворачиваются при первом чтении.
_LAZY_ARRAYS = frozenset(("positions", "faces", "uvs", "face_domain", "face_owner", "seam_edges", "domain_vertex_index"))


class MeshBuffersV1:
    """Плоские массивы нативной сборки (`cftuv_native.mesh_assemble`), numpy поверх байтов ответа (без копии): то, что `_fill_mesh` кладёт в меш, и разворот в кортежи эталона.

    `positions` float64 `(V, 3)`, `face_starts` uint32 `(F + 1)`, `face_vertices` uint32, `uvs` float64 `(L, 2)`, `face_domain` и `face_owner` int64 `(F)`, `seams` uint32 `(S, 2)`,
    `index` uint32 — номера вершин меша для локальных вершин всех доменов подряд, `vertex_starts` — начало домена в `index`.
    """

    __slots__ = ("positions_array", "face_starts", "face_vertices", "uvs_array", "face_domain_array", "face_owner_array", "seams_array", "index_array", "vertex_starts")

    def __init__(self, positions, face_starts, face_vertices, uvs, face_domain, face_owner, seams, index, vertex_counts):
        self.positions_array = np.frombuffer(positions, dtype="<f8").reshape(-1, 3)
        self.face_starts = np.frombuffer(face_starts, dtype="<u4")
        self.face_vertices = np.frombuffer(face_vertices, dtype="<u4")
        self.uvs_array = np.frombuffer(uvs, dtype="<f8").reshape(-1, 2)
        self.face_domain_array = np.frombuffer(face_domain, dtype="<i8")
        self.face_owner_array = np.frombuffer(face_owner, dtype="<i8")
        self.seams_array = np.frombuffer(seams, dtype="<u4").reshape(-1, 2)
        self.index_array = np.frombuffer(index, dtype="<u4")
        starts = [0]
        for count in vertex_counts:
            starts.append(starts[-1] + int(count))
        self.vertex_starts = tuple(starts)

    @property
    def vertex_count(self) -> int:
        return int(self.positions_array.shape[0])

    @property
    def face_count(self) -> int:
        return int(self.face_starts.shape[0]) - 1

    @property
    def loop_count(self) -> int:
        return int(self.uvs_array.shape[0])

    @property
    def seam_count(self) -> int:
        return int(self.seams_array.shape[0])

    def face_sizes(self):
        return np.diff(self.face_starts.astype(np.int64))

    def corner_counts(self) -> tuple:
        sizes = self.face_sizes()
        return int((sizes == 4).sum()), int((sizes == 3).sum()), int((sizes > 4).sum())

    def domain_index(self, number: int):
        """Номера вершин меша локальных вершин домена `number` (uint32, вид без копии)."""

        return self.index_array[self.vertex_starts[number]:self.vertex_starts[number + 1]]

    # разворот в кортежи эталона (`MeshArraysV1.__getattr__`)
    def positions(self) -> tuple:
        return tuple(map(tuple, self.positions_array.tolist()))

    def faces(self) -> tuple:
        flat = self.face_vertices.tolist()
        starts = self.face_starts.tolist()
        return tuple(tuple(flat[a:b]) for a, b in zip(starts, starts[1:]))

    def uvs(self) -> tuple:
        return tuple(map(tuple, self.uvs_array.tolist()))

    def face_domain(self) -> tuple:
        return tuple(self.face_domain_array.tolist())

    def face_owner(self) -> tuple:
        return tuple(self.face_owner_array.tolist())

    def seam_edges(self) -> tuple:
        return tuple(map(tuple, self.seams_array.tolist()))

    def domain_vertex_index(self) -> tuple:
        flat = self.index_array.tolist()
        starts = self.vertex_starts
        return tuple(tuple(flat[a:b]) for a, b in zip(starts, starts[1:]))


@dataclass(frozen=True, slots=True)
class ProductionWriteReceiptV1:
    """Квитанция записи: что лежит в объекте и что названо пропущенным."""

    object_name: str | None
    replaced: bool
    vertices: int
    faces: int
    loops: int
    #: Швы, которые РЕАЛЬНО помечены на рёбрах меша; запрошенные — ниже.
    seam_edges: int
    seam_edges_requested: int
    uv_layer: str
    material_name: str
    material_created: bool
    offset: float
    domains: tuple
    skipped: tuple
    #: `(patch_id | None, исход, деталь)`; `None` — находка про весь меш.
    warnings: tuple
    source_revision: str
    arrays_digest: str
    mesh_name: str | None
    #: `mesh_content_digest` записанного меша; пусто у перезаписи живой ширины (MESH_DIGEST_OFF_CONFIRM_V1: его никто в продукте не читает, а на `cover.008` это 75 мс
    #: стены фиксации; кому нужен отпечаток лежащего в Blender, считает `mesh_content_digest(mesh)` сам).
    mesh_digest: str
    #: Закон топологии записанных доменов и состав граней меша по числу углов:
    #: треугольники, четырёхгранники и многоугольники (грани длиннее 4 пишет
    #: `PLANAR_POLYGONS_V1`); `faces == quads + triangles + polygons` всегда.
    decal_topology_law: str = ""
    quads: int = 0
    triangles: int = 0
    polygons: int = 0
    #: Числа сварки (см. `MeshArraysV1.weld_counters`): в квитанции и в JSON прогона.
    weld_counters: tuple = ()
    #: Плоскость граней после смещения (см. `MeshArraysV1.offset_counters`).
    offset_counters: tuple = ()
    #: Шов по цепям батчей (см. `MeshArraysV1.seam_counters`).
    seam_counters: tuple = ()


def decal_object_name(source_name: str) -> str:
    """`<исходный>.CFTUV_Decal`, укороченное справа до предела имени ID Blender.

    Укорачивается ИМЯ ИСТОЧНИКА по границе символа, суффикс сохраняется. Имя —
    только пожелание: коллизию Blender разрешит суффиксом `.001`, а сам объект
    находится по маркеру (`_find_decal`), поэтому ни усечение, ни `.001` не плодят
    объектов.
    """

    room = ID_NAME_LIMIT_BYTES - len(DECAL_OBJECT_SUFFIX.encode("utf-8"))
    stem = source_name.encode("utf-8")[:room].decode("utf-8", "ignore")
    return f"{stem}{DECAL_OBJECT_SUFFIX}"


def _domain_view(result):
    """Вид домена (`DomainViewV1`): присланный воркером вместе с результатом либо посчитанный здесь из батча тем же кодом."""

    view = getattr(result, "view", None)
    return build_domain_view(result) if view is None else view


def _domain_warnings(result) -> list:
    """Мягкие находки записанного домена: источник нормали неизвестен, ядро насчитало вывернутые."""

    found = []
    source = getattr(result, "source_normal", None)
    if source is None or not any(source):
        found.append(
            (
                result.patch_id,
                OUTCOME_SOURCE_NORMAL_UNKNOWN,
                "no source face normal: the side of the offset is not checked",
            )
        )
    flipped = dict(result.counters).get(OUTCOME_FLIPPED_VS_SOURCE, 0)
    if flipped:
        found.append(
            (
                result.patch_id,
                OUTCOME_FLIPPED_VS_SOURCE,
                f"{flipped} faces are wound against the source face",
            )
        )
    return found


def build_mesh_arrays(results, offset: float, *, reports: bool = True, digest: bool = True) -> MeshArraysV1:
    """Массивы меша по результатам домена. Не-MATERIALIZED домены — в `skipped`.

    Домены идут по номеру патча, вершины внутри домена — по ключу батча:
    порядок не зависит ни от воркера, ни от хеш-порядка множеств батча. Вершины
    соседних доменов с одной ссылкой `location:src:<id>` и побитово равными
    позициями сварены в одну (`envelope_production_weld`), смещение над
    поверхностью — митра общих вершин.

    `reports=False` (WIDTH_REFRESH_LEAN_V1) не считает то, что только сообщает: дайджест массивов, плоскость граней после смещения, отчёт о шве по цепям
    и обход рёбер. Ни одно из четырёх не читается дальше при построении `positions`, `faces`, `uvs`, `face_domain`, `face_owner` и `seams` (они
    определены ДО них, а отчёты читают их и пишут в свои поля), поэтому эти шесть массивов побитово те же (`tests/test_envelope_refresh_lean.py`).
    """

    ordered = sorted(results, key=lambda item: (item.patch_id, item.domain_id))
    found = _native_mesh_arrays(ordered, offset, reports, digest)
    return _reference_mesh_arrays(ordered, offset, reports, digest) if found is None else found


#: MESH_ASSEMBLY_BUFFERS_V1: операция порта, которая собирает массивы меша из байтов видов одним вызовом.
MESH_ASSEMBLE_OPERATION = "mesh_assemble"
OUTCOME_VIEW_NOT_BYTES = "NATIVE_PORT_UNSUPPORTED"


def _native_mesh_arrays(ordered, offset: float, reports: bool, digest: bool = True):
    """Массивы меша нативной сборкой из байтов видов (`cftuv_native.mesh_assemble`) либо `None`: заказан эталон, у домена вид объектом, а не байтами (откат назван), или порт отказал по имени.

    Отбор доменов, предупреждения доменов, закон топологии и ревизия — те же, что у эталона (`_reference_mesh_arrays`), и в том же порядке; вид-отказ узнаётся по флагу заголовка и разбирается
    (в нём только тексты), материализованный вид не разбирается вовсе.
    """

    if mesh_assembly_backend() != MESH_ASSEMBLY_NATIVE:
        return None
    domains, skipped, warnings, pending = [], [], [], []
    laws: set[str] = set()
    for result in ordered:
        if not result.is_materialized:
            skipped.append((result.patch_id, result.domain_id, result.outcome, result.detail))
            continue
        data = result.__dict__.get("view_bytes")
        if data is None:
            record_fallback(
                (OUTCOME_VIEW_NOT_BYTES, MESH_ASSEMBLE_OPERATION), 1, "a domain result carries its view as an object, not as VIEW_BYTES_V1 bytes"
            )
            return None
        if view_bytes_failed(data):
            skipped.append((result.patch_id, result.domain_id, *_domain_view(result).failure))
            continue
        pending.append(result)
        domains.append(result.patch_id)
        warnings.extend(_domain_warnings(result))
        laws.add(getattr(result, "decal_topology_law", ""))
    answer = native_answer(
        MESH_ASSEMBLE_OPERATION, [result.__dict__["view_bytes"] for result in pending], [result.patch_id for result in pending], float(offset), WELD_LAW, bool(reports),
        *(() if digest else (False,)),
    )
    if answer is None:
        return None
    (positions, face_starts, face_vertices, uvs, face_domain, face_owner, seams, index, vertex_counts, loop_counts, tokens, revisions, numbers, folds, found) = answer
    buffers = MeshBuffersV1(positions, face_starts, face_vertices, uvs, face_domain, face_owner, seams, index, vertex_counts)
    weld = weld_of_answer((None, None, *numbers))
    digest, seam, offset_counters, conflicts = report_counters(found) if reports else ("", (), (), 0)
    arrays = object.__new__(MeshArraysV1)
    for name, value in (
        ("domains", tuple(domains)),
        ("skipped", tuple(skipped)),
        ("warnings", _mesh_warnings(warnings, weld, conflicts, seam, reports)),
        ("source_revision", next((text for text in revisions if text), "")),
        ("digest", digest),
        ("decal_topology_law", ",".join(sorted(item for item in laws if item))),
        ("weld_counters", _weld_counters(weld, conflicts, folds, reports)),
        ("offset_counters", offset_counters),
        ("seam_counters", seam),
        ("domain_keys", tuple((result.patch_id, result.domain_id) for result in pending)),
        ("domain_loop_counts", tuple(loop_counts)),
        ("domain_tokens", tuple(tokens)),
        ("domain_partners", _partners_by_domain(weld.partners, len(pending))),
        ("reports", reports),
        ("offset", float(offset)),
        ("buffers", buffers),
    ):
        object.__setattr__(arrays, name, value)
    return arrays


def _weld_counters(weld, conflicts: int, folds: int, reports: bool) -> tuple:
    return (
        *weld.counters,
        *(((OUTCOME_WELD_HALF_EDGE_CONFLICT, conflicts),) if reports else ()),
        (COUNTER_WELD_SEAMS_MARKED, folds),
    )


def _mesh_warnings(warnings: list, weld, conflicts: int, seam: tuple, reports: bool) -> tuple:
    """Предупреждения доменов и находки про весь меш: сварка, полурёбра двух граней, шов по цепям."""

    warnings = list(warnings)
    warnings.extend(weld.warnings)
    if conflicts:
        warnings.append(
            (
                None,
                OUTCOME_WELD_HALF_EDGE_CONFLICT,
                f"{conflicts} directed edges lie in two faces after the weld: "
                "neighbouring domains wind against each other there",
            )
        )
    seam_found = dict(seam) if reports else {COUNTER_SEAM_T_JUNCTIONS: 0, COUNTER_SEAM_CLIP_VERTICES: 0}
    if seam_found[COUNTER_SEAM_T_JUNCTIONS] or seam_found[COUNTER_SEAM_CLIP_VERTICES]:
        warnings.append(
            (
                None,
                OUTCOME_SEAM_T_JUNCTIONS,
                f"{seam_found[COUNTER_SEAM_T_JUNCTIONS]} source-chain segments have a different number of vertices in "
                f"the two neighbour domains, {seam_found[COUNTER_SEAM_CLIP_VERTICES]} clip vertices lie on seam chains: "
                "the seam may be open",
            )
        )
    return tuple(warnings)


def _reference_mesh_arrays(ordered, offset: float, reports: bool, with_digest: bool = True) -> MeshArraysV1:
    """Эталон `build_mesh_arrays` (заказ `PYTHON` и откат нативной сборки): виды доменов разобраны в кортежи, склейка — `mesh_glue.glue_domains`."""

    domains, skipped, warnings = [], [], []
    entries = []
    revision = ""
    laws: set[str] = set()
    for result in ordered:
        if not result.is_materialized:
            skipped.append(
                (result.patch_id, result.domain_id, result.outcome, result.detail)
            )
            continue
        view = _domain_view(result)
        if view.failure is not None:
            skipped.append((result.patch_id, result.domain_id, *view.failure))
            continue
        entries.append((result, view))
        domains.append(result.patch_id)
        warnings.extend(_domain_warnings(result))
        laws.add(getattr(result, "decal_topology_law", ""))
        revision = revision or view.source_revision
    weld = weld_vertices([view.vertices for _result, view in entries], float(offset))
    faces, uvs, face_domain, face_owner, seam_pairs = glue_domains(
        [result.patch_id for result, _view in entries], [view.arrays[1:] for _result, view in entries], weld.index
    )
    domain_keys = [(result.patch_id, result.domain_id) for result, _view in entries]
    domain_vertex_index = [tuple(index) for index in weld.index]
    domain_loop_counts = [len(view.uvs) for _result, view in entries]
    domain_tokens = [structure_token(view) for _result, view in entries]  # VIEW_BYTES_V1: у вида из байтов воркера токен уже в заголовке
    folds = cross_domain_seams(faces, uvs, face_domain)
    seam_pairs.update(folds)
    seams = sorted(seam_pairs)
    positions = list(weld.positions)
    if reports:
        digest, seam, offset_counters, conflicts = mesh_reports(
            positions, faces, uvs, face_domain, face_owner, seams, [view.boundary_chains for _result, view in entries]
        )
        digest = digest if with_digest else ""  # MESH_DIGEST_OFF_CONFIRM_V1: without the arrays digest, the other reports the same
    else:
        digest, seam, offset_counters, conflicts = "", (), (), 0
    return MeshArraysV1(
        positions=tuple(positions),
        faces=tuple(faces),
        uvs=tuple(uvs),
        face_domain=tuple(face_domain),
        face_owner=tuple(face_owner),
        seam_edges=tuple(seams),
        domains=tuple(domains),
        skipped=tuple(skipped),
        warnings=_mesh_warnings(warnings, weld, conflicts, seam, reports),
        source_revision=revision,
        digest=digest,
        decal_topology_law=",".join(sorted(item for item in laws if item)),
        weld_counters=_weld_counters(weld, conflicts, len(folds), reports),
        offset_counters=offset_counters,
        seam_counters=seam,
        domain_keys=tuple(domain_keys),
        domain_vertex_index=tuple(domain_vertex_index),
        domain_loop_counts=tuple(domain_loop_counts),
        domain_tokens=tuple(domain_tokens),
        offset=float(offset),
        domain_partners=_partners_by_domain(weld.partners, len(entries)),
        reports=reports,
    )


def _partners_by_domain(partners, count: int) -> tuple:
    """`WeldV1.partners` (пары `(домен, сосед)`, отсортированы) по доменам: у каждого записанного домена кортеж его соседей."""

    found: dict = {}
    for owner, other in partners:
        found.setdefault(owner, []).append(other)
    return tuple(tuple(found.get(ordinal, ())) for ordinal in range(count))


# --------------------------------------------------------------------------
# Blender
# --------------------------------------------------------------------------


def _read(collection, name: str, dtype, size: int = 1):
    """Одно свойство каждого элемента коллекции меша в плоский массив numpy (`foreach_get`: без обхода элементов через RNA из Python)."""

    buffer = np.empty(len(collection) * size, dtype=dtype)
    collection.foreach_get(name, buffer)
    return buffer


def mesh_content_digest(mesh) -> str:
    """Отпечаток меша ПО ТОМУ, ЧТО ЛЕЖИТ В BLENDER (float32 и порядок петель).

    Позиции и UV читаются обратно, поэтому дайджест видит и округление
    хранилища: два прогона дают равные дайджесты, когда равны их меши.

    Массивы читаются `foreach_get` в numpy, а не обходом элементов (`item.co`, `item.vertices`, `item.uv`, `item.value` по одному через RNA: 13 мс на `building`, 119 мс на
    `cover.008`, то есть больше половины записи меша). Значение то же побитово: float32 приводится к тем же `float`, что давали Vector и MeshUVLoop, а JSON и хэш прежние
    (`tests/blender/test_envelope_mesh_numpy_paths.py` держит равенство с прежним обходом на настоящих мешах).
    """

    uv_layer = mesh.uv_layers.get(DECAL_UV_LAYER)
    starts = _read(mesh.polygons, "loop_start", np.int32).tolist()
    totals = _read(mesh.polygons, "loop_total", np.int32).tolist()
    corners = _read(mesh.loops, "vertex_index", np.int32).tolist()
    edge_pairs = _read(mesh.edges, "vertices", np.int32, 2).reshape(-1, 2)
    chosen = edge_pairs[_read(mesh.edges, "use_seam", np.bool_)]
    low, high = chosen.min(axis=1), chosen.max(axis=1)
    order = np.lexsort((high, low))
    payload = {
        "positions": _read(mesh.vertices, "co", np.float32, 3).astype(np.float64).reshape(-1, 3).tolist(),
        "faces": [corners[start : start + total] for start, total in zip(starts, totals)],
        "uvs": [] if uv_layer is None else _read(uv_layer.data, "uv", np.float32, 2).astype(np.float64).reshape(-1, 2).tolist(),
        "seams": np.stack((low[order], high[order]), axis=1).tolist() if len(chosen) else [],
    }
    for name in (DECAL_DOMAIN_ATTRIBUTE, DECAL_OWNER_ATTRIBUTE):
        attribute = mesh.attributes.get(name)
        payload[name] = [] if attribute is None else _read(attribute.data, "value", np.int32).tolist()
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("ascii")).hexdigest()


def _build_mesh(name: str, arrays: MeshArraysV1):
    """Меш из массивов и число РЕАЛЬНО помеченных швов: `(меш, помечено)`."""

    mesh = bpy.data.meshes.new(name)
    return mesh, _fill_mesh(mesh, arrays)


def _fill_mesh(mesh, arrays: MeshArraysV1) -> int:
    """Геометрия, UV, атрибуты и швы в ПУСТОЙ меш; число реально помеченных швов.

    Один код и для нового датаблока, и для перезаписи на месте: равенство результата кнопки и живой ширины
    держится общим кодом записи, а не копией.

    Массивы нативной сборки (`arrays.buffers`, MESH_ASSEMBLY_BUFFERS_V1) кладутся `foreach_set` из numpy, без кортежей: те же шаги, что у `from_pydata` (вершины, петли, грани,
    плоская заливка, рёбра), и те же числа (float64 -> float32 тем же округлением к ближайшему, что у `foreach_set` по питоновским `float`).
    """

    if arrays.buffers is not None:
        return _fill_mesh_buffers(mesh, arrays.buffers)
    mesh.from_pydata([tuple(item) for item in arrays.positions], [], list(arrays.faces))
    mesh.update()
    layer = mesh.uv_layers.new(name=DECAL_UV_LAYER)
    flat = [component for pair in arrays.uvs for component in pair]
    layer.data.foreach_set("uv", flat)
    for attribute_name, values in (
        (DECAL_DOMAIN_ATTRIBUTE, arrays.face_domain),
        (DECAL_OWNER_ATTRIBUTE, arrays.face_owner),
    ):
        attribute = mesh.attributes.new(
            name=attribute_name, type="INT", domain="FACE"
        )
        attribute.data.foreach_set("value", list(values))
    marked = 0
    if arrays.seam_edges:
        wanted = {tuple(pair) for pair in arrays.seam_edges}
        for edge in mesh.edges:
            if tuple(sorted(edge.vertices)) in wanted:
                edge.use_seam = True
                marked += 1
    return marked


def _fill_mesh_buffers(mesh, buffers: MeshBuffersV1) -> int:
    """`_fill_mesh` по буферам нативной сборки (шаги `Mesh.from_pydata` Blender 4.5 без рёбер, затем `update`, слой UV, атрибуты граней, швы): число помеченных швов."""

    vertex_count, face_count = buffers.vertex_count, buffers.face_count
    mesh.vertices.add(vertex_count)
    mesh.loops.add(int(buffers.face_vertices.shape[0]))
    mesh.polygons.add(face_count)
    _write_positions(mesh, buffers.positions_array.astype(np.float32).reshape(-1))
    mesh.polygons.foreach_set("loop_start", buffers.face_starts[:-1].astype(np.int32))
    _write_corners(mesh, buffers.face_vertices.astype(np.int32))
    _shade_flat(mesh)
    if face_count:
        mesh.update(calc_edges=True, calc_edges_loose=False)
    mesh.update()
    layer = mesh.uv_layers.new(name=DECAL_UV_LAYER)
    _write_uvs(mesh, layer, buffers.uvs_array.astype(np.float32).reshape(-1))
    for attribute_name, values in ((DECAL_DOMAIN_ATTRIBUTE, buffers.face_domain_array), (DECAL_OWNER_ATTRIBUTE, buffers.face_owner_array)):
        attribute = mesh.attributes.new(name=attribute_name, type="INT", domain="FACE")
        attribute.data.foreach_set("value", values.astype(np.int32))
    return _mark_seams(mesh, buffers.seams_array, vertex_count)


def _write_positions(mesh, flat) -> None:
    """Позиции вершин (плоский float32) - в атрибут `position` (MESH_WRITE_BY_ATTRIBUTE_V1): те же байты, что `vertices.foreach_set("co")`, но одним копированием (Blender 4.5: 0.005 мс
    против 0.85 мс на 9.5 тыс. вершин); нормали пересчитывает `mesh.update()`, который каждый писатель зовёт после записи. Атрибута нет - прежняя запись."""

    attribute = mesh.attributes.get("position")
    if attribute is None or attribute.data_type != "FLOAT_VECTOR" or attribute.domain != "POINT":
        mesh.vertices.foreach_set("co", flat)
    else:
        attribute.data.foreach_set("vector", flat)


def _write_corners(mesh, corners) -> None:
    """Вершины углов граней (int32, после `polygons.foreach_set("loop_start")`) - в атрибут `.corner_vert`: те же числа, что `polygons.foreach_set("vertices")` (1.1 мс против 1.75 мс на 25.7 тыс. углов).
    Атрибута нет (иная версия Blender) - прежняя запись."""

    attribute = mesh.attributes.get(".corner_vert")
    if attribute is None or attribute.data_type != "INT" or attribute.domain != "CORNER":
        mesh.polygons.foreach_set("vertices", corners)
    else:
        attribute.data.foreach_set("value", corners)


def _write_uvs(mesh, layer, flat) -> None:
    """UV петель (плоский float32) - в атрибут слоя (`FLOAT2` углов, его имя - имя слоя): те же байты, что `layer.data.foreach_set("uv")`, одним копированием (0.007 мс против 0.9 мс)."""

    attribute = mesh.attributes.get(layer.name)
    if attribute is None or attribute.data_type != "FLOAT2" or attribute.domain != "CORNER":
        layer.data.foreach_set("uv", flat)
    else:
        attribute.data.foreach_set("vector", flat)


def _shade_flat(mesh) -> None:
    """`Mesh.shade_flat` Blender 4.5 (атрибут граней `sharp_face` BOOLEAN заведён, при чужом типе - заведён заново, и истина у каждой грани) одной записью `foreach_set`, а не обходом граней в питоне."""

    attributes = mesh.attributes
    attribute = attributes.get("sharp_face")
    if attribute is not None and not (attribute.domain == "FACE" and attribute.data_type == "BOOLEAN"):
        attributes.remove(attribute)
        attribute = None
    if attribute is None:
        attribute = attributes.new("sharp_face", "BOOLEAN", "FACE")
    attribute.data.foreach_set("value", np.ones(len(attribute.data), dtype=bool))


def _clear_mesh_in_place(mesh) -> None:
    """Пустая геометрия БЕЗ смены датаблока: слои UV и атрибуты граней уходят вместе с ней.

    `clear_geometry` в Blender 4.5 снимает и их (проверено смоком); оставшиеся от иной версии
    снимаются явно, чтобы `_fill_mesh` не наткнулся на слой, не равный новым граням.
    """

    mesh.clear_geometry()
    for layer in list(mesh.uv_layers):
        mesh.uv_layers.remove(layer)
    for name in (DECAL_DOMAIN_ATTRIBUTE, DECAL_OWNER_ATTRIBUTE):
        attribute = mesh.attributes.get(name)
        if attribute is not None:
            mesh.attributes.remove(attribute)


def resolve_decal_material_name(value: str, materials) -> str:
    """Существующее имя сохраняется точно; только новое имя очищается от краевых пробелов."""

    material_name = str(value)
    if materials.get(material_name) is not None:
        return material_name
    return material_name.strip() or DEFAULT_DECAL_MATERIAL


def _material_slot(mesh, material_name: str):
    material = bpy.data.materials.get(material_name)
    created = material is None
    if created:
        material = bpy.data.materials.new(material_name)
    mesh.materials.append(material)
    return created


def _is_cftuv_decal(item) -> bool:
    return item.type == "MESH" and DECAL_REVISION_PROPERTY in item.keys()


def _find_decal(source_obj, name: str):
    """`(объект, предупреждения)`: наш объект ЭТОГО источника по МАРКЕРУ, а не по имени.

    Имя объекта Blender режет до 63 байт и дописывает `.001` при коллизии, поэтому
    `decal.name != name` — штатный случай, и поиск по имени при каждом нажатии
    плодил бы новый объект. Свой объект — тот, у кого есть маркер
    `cftuv_source_revision` и либо родитель — этот источник, либо метка
    `cftuv_source_object` с его именем (источник мог быть переименован: родитель
    переживает переименование). Из нескольких берётся тот, чьё имя совпало, иначе
    первый по имени; остальные не трогаются и называются предупреждением.
    Чужой (без маркера) объект с ожидаемым именем не перезаписывается: отказ.
    """

    mine = sorted(
        (
            item
            for item in bpy.data.objects
            if _is_cftuv_decal(item)
            and (
                item.parent == source_obj
                or (
                    DECAL_SOURCE_PROPERTY in item.keys()
                    and item[DECAL_SOURCE_PROPERTY] == source_obj.name
                )
            )
        ),
        key=lambda item: item.name,
    )
    warnings = []
    if not mine:
        taken = bpy.data.objects.get(name)
        if taken is not None and not _is_cftuv_decal(taken):
            raise ProductionWriteError(
                OUTCOME_NAME_TAKEN,
                f"object {name!r} exists and was not built by CFTUV "
                f"(no {DECAL_REVISION_PROPERTY} marker); it is left untouched",
            )
        return None, warnings
    chosen = next((item for item in mine if item.name == name), mine[0])
    if len(mine) > 1:
        others = ", ".join(item.name for item in mine if item != chosen)
        warnings.append(
            (
                None,
                OUTCOME_DUPLICATE_DECALS,
                f"{len(mine)} decal objects of {source_obj.name!r}; rebuilt "
                f"{chosen.name!r}, left untouched: {others}",
            )
        )
    return chosen, warnings


def _collections_of(item) -> tuple:
    return tuple(getattr(item, "users_collection", ()) or ())


def _link_child(source_obj, name: str, mesh):
    decal = bpy.data.objects.new(name, mesh)
    collections = _collections_of(source_obj)
    (collections[0] if collections else bpy.context.scene.collection).objects.link(
        decal
    )
    decal.parent = source_obj
    return decal


def _reassert_placement(decal, source_obj):
    """Родитель и коллекция существующего декаля: восстановлены и НАЗВАНЫ, если ушли."""

    warnings = []
    if decal.parent != source_obj:
        decal.parent = source_obj
        warnings.append(
            (None, OUTCOME_PARENT_REASSERTED, f"{decal.name!r} is a child of {source_obj.name!r} again")
        )
    wanted = _collections_of(source_obj)
    if wanted and not any(item in _collections_of(decal) for item in wanted):
        wanted[0].objects.link(decal)
        warnings.append(
            (None, OUTCOME_COLLECTION_REASSERTED, f"{decal.name!r} linked to the collection of the source")
        )
    return warnings


def _swap_mesh(existing, mesh, name: str):
    """Меш объекта заменён; старый удалён, если им никто не пользуется. Предупреждения."""

    old = existing.data
    shared = 0 if old is None else int(getattr(old, "users", 1))
    existing.data = mesh
    warnings = []
    if old is not None and getattr(old, "users", 1) == 0:
        bpy.data.meshes.remove(old)
    elif shared > 1:
        warnings.append(
            (
                None,
                OUTCOME_MESH_SHARED,
                f"mesh {old.name!r} is shared by {shared} objects and was kept; "
                f"the rebuilt mesh may be named with a suffix",
            )
        )
    # Имя датаблока одно и то же при любой пересборке: старый меш уже удалён,
    # и Blender не припишет новому `.001` (кроме названного случая общего меша).
    mesh.name = name
    return warnings


def _drop_stale_mesh(name: str) -> None:
    """Осиротевший меш с этим именем (объект удалили руками) не должен давать `.001`."""

    stale = bpy.data.meshes.get(name)
    if stale is not None and getattr(stale, "users", 1) == 0:
        bpy.data.meshes.remove(stale)


def _receipt(arrays, offset, material_name, *, object_name, replaced, mesh, marked,
             material_created, warnings, digest: bool = True):
    seam_requested = arrays.seam_count
    warnings = list(arrays.warnings) + list(warnings)
    if marked != seam_requested:
        warnings.append(
            (
                None,
                OUTCOME_SEAM_EDGE_MISSING,
                f"{seam_requested} seam edges requested by the interface chains, "
                f"{marked} marked on the mesh",
            )
        )
    quads, triangles, polygons = arrays.corner_counts()
    return ProductionWriteReceiptV1(
        object_name=object_name,
        replaced=replaced,
        vertices=arrays.vertex_count,
        faces=arrays.face_count,
        loops=arrays.loop_count,
        seam_edges=marked,
        seam_edges_requested=seam_requested,
        uv_layer=DECAL_UV_LAYER,
        material_name=material_name,
        material_created=material_created,
        offset=float(offset),
        domains=arrays.domains,
        skipped=arrays.skipped,
        warnings=tuple(warnings),
        source_revision=arrays.source_revision,
        arrays_digest=arrays.digest,
        mesh_name=None if mesh is None else mesh.name,
        mesh_digest="" if mesh is None or not digest else mesh_content_digest(mesh),
        decal_topology_law=arrays.decal_topology_law,
        quads=quads,
        triangles=triangles,
        polygons=polygons,
        weld_counters=arrays.weld_counters,
        offset_counters=arrays.offset_counters,
        seam_counters=arrays.seam_counters,
    )


def _require_reports(arrays: MeshArraysV1) -> None:
    """Зафиксированная запись (кнопка, COMMIT живой ширины) принимает только полные массивы: квитанция несёт их дайджест и отчёты."""

    if not arrays.reports:
        raise ProductionWriteError(OUTCOME_LEAN_ARRAYS, "the arrays were built without reports (WIDTH_REFRESH_LEAN_V1): a committed write needs the full arrays")


def _blank_arrays(arrays: MeshArraysV1) -> MeshArraysV1:
    """Массивы пустого результата: ничего не записано, пропуски и предупреждения те же."""

    return MeshArraysV1(
        positions=(), faces=(), uvs=(), face_domain=(), face_owner=(),
        seam_edges=(), domains=(), skipped=arrays.skipped, warnings=arrays.warnings,
        source_revision=arrays.source_revision, digest=arrays.digest,
        decal_topology_law=arrays.decal_topology_law,
        weld_counters=arrays.weld_counters,
    )


def find_decal_object(source_obj):
    """Объект декали этого источника (по маркеру, как при пересборке) либо `None`; ничего не создаёт."""

    found, _warnings = _find_decal(source_obj, decal_object_name(source_obj.name))
    return found


def write_decal_object(
    source_obj,
    results,
    *,
    offset: float = DEFAULT_DECAL_OFFSET,
    material_name: str = DEFAULT_DECAL_MATERIAL,
    width: float | None = None,
    arrays: MeshArraysV1 | None = None,
) -> ProductionWriteReceiptV1:
    """Записывает все MATERIALIZED домены одним объектом `<исходный>.CFTUV_Decal`.

    Повторный вызов заменяет меш объекта (идемпотентно: тот же вход — тот же
    дайджест, один объект, один материал); объект ищется по маркеру, а не по
    имени (имя Blender режет до 63 байт). Пустой результат НЕ создаёт объекта, а
    у существующего заменяет меш пустым: устаревший декаль не должен выдавать себя
    за свежий. Отказ имени — `ProductionWriteError`.

    `arrays` — массивы тех же `results` и `offset`, уже построенные вызывающим (`build_mesh_arrays`): кнопка берёт
    из них образец превью ширины, не строя их дважды. Без них писатель строит их сам; ответ тот же.
    """

    arrays = build_mesh_arrays(results, offset) if arrays is None else arrays
    _require_reports(arrays)
    name = decal_object_name(source_obj.name)
    existing, warnings = _find_decal(source_obj, name)
    if existing is not None:
        warnings = warnings + _reassert_placement(existing, source_obj)
    if not arrays.face_count:
        blank = _blank_arrays(arrays)
        if existing is None:
            return _receipt(blank, offset, material_name, object_name=None,
                            replaced=False, mesh=None, marked=0,
                            material_created=False, warnings=warnings)
        _drop_stale_mesh(name)
        empty = bpy.data.meshes.new(name)
        warnings = warnings + _swap_mesh(existing, empty, name)
        existing[DECAL_REVISION_PROPERTY] = arrays.source_revision
        existing[DECAL_SOURCE_PROPERTY] = source_obj.name
        _record_width(existing.data, width)
        return _receipt(blank, offset, material_name, object_name=existing.name,
                        replaced=True, mesh=existing.data, marked=0,
                        material_created=False, warnings=warnings)
    if existing is None:
        _drop_stale_mesh(name)
    mesh, marked = _build_mesh(name, arrays)
    material_created = _material_slot(mesh, material_name)
    if existing is None:
        decal = _link_child(source_obj, name, mesh)
    else:
        warnings = warnings + _swap_mesh(existing, mesh, name)
        decal = existing
    decal[DECAL_REVISION_PROPERTY] = arrays.source_revision
    decal[DECAL_SOURCE_PROPERTY] = source_obj.name
    _record_width(mesh, width)
    return _receipt(arrays, offset, material_name, object_name=decal.name,
                    replaced=existing is not None, mesh=mesh, marked=marked,
                    material_created=material_created, warnings=warnings)


def _record_width(mesh, width) -> None:
    if width is not None:
        mesh[DECAL_WIDTH_PROPERTY] = float(width)


def rewrite_decal_mesh(
    source_obj,
    results,
    *,
    offset: float = DEFAULT_DECAL_OFFSET,
    material_name: str = DEFAULT_DECAL_MATERIAL,
    width: float | None = None,
    arrays: MeshArraysV1 | None = None,
) -> ProductionWriteReceiptV1:
    """Заменяет геометрию СУЩЕСТВУЮЩЕГО меша декали НА МЕСТЕ: тот же объект, тот же датаблок меша.

    Путь живой ширины (`envelope_width_live`): его вызывает таймер, а таймер не вправе создавать и
    освобождать датаблоки (`UNDO_REQUIRED_REASON`: без шага отмены следующий Ctrl+Z падает на висячем
    указателе). Поэтому здесь нет ни `meshes.new`, ни `meshes.remove`, ни `materials.new`: объект и меш
    должны быть, материал берётся существующий. Содержимое результата — побитово то же, что даёт
    `write_decal_object` на тех же результатах (общие `build_mesh_arrays` и `_fill_mesh`; равенство держит
    смок по `mesh_content_digest`). Нет объекта, нет меша или объект в Edit-режиме — именованный отказ
    `ProductionWriteError`, а не создание нового.

    `arrays` — массивы тех же `results` и `offset`, построенные потоком точного счёта (`build_mesh_arrays` в нём): главный
    поток не строит их заново, и содержимое то же, что у кнопки (общий код массивов).
    """

    arrays = build_mesh_arrays(results, offset) if arrays is None else arrays
    _require_reports(arrays)
    existing, warnings = _find_decal(source_obj, decal_object_name(source_obj.name))
    if existing is None:
        raise ProductionWriteError(
            OUTCOME_DECAL_MISSING, "no decal object of this source: press Build Decal Mesh"
        )
    if getattr(existing, "mode", "OBJECT") == "EDIT":
        raise ProductionWriteError(
            OUTCOME_DECAL_IN_EDIT_MODE,
            f"{existing.name!r} is in Edit mode: leave it, the live width cannot rewrite its mesh",
        )
    mesh = existing.data
    if mesh is None:
        raise ProductionWriteError(OUTCOME_DECAL_MESH_MISSING, f"{existing.name!r} has no mesh")
    warnings = warnings + _reassert_placement(existing, source_obj)
    users = int(getattr(mesh, "users", 1))
    if users > 1:
        warnings.append(
            (None, OUTCOME_MESH_SHARED, f"mesh {mesh.name!r} is shared by {users} objects: all of them changed")
        )
    _clear_mesh_in_place(mesh)
    marked = _fill_mesh(mesh, arrays) if arrays.face_count else 0
    if not len(mesh.materials):
        material = bpy.data.materials.get(material_name)
        if material is None:
            warnings.append(
                (None, OUTCOME_MATERIAL_MISSING, f"material {material_name!r} is gone and is not created by the live width")
            )
        else:
            mesh.materials.append(material)
    existing[DECAL_REVISION_PROPERTY] = arrays.source_revision
    existing[DECAL_SOURCE_PROPERTY] = source_obj.name
    _record_width(mesh, width)
    return _receipt(
        arrays if arrays.face_count else _blank_arrays(arrays),
        offset,
        material_name,
        object_name=existing.name,
        replaced=True,
        mesh=mesh,
        marked=marked,
        material_created=False,
        warnings=warnings,
        digest=False,  # MESH_DIGEST_OFF_CONFIRM_V1
    )


def rebuild_mesh_from_image(mesh, positions, uvs, image) -> int:
    """Геометрия СУЩЕСТВУЮЩЕГО меша пересобрана на месте по образу точного прогона (`MeshImageV1`) и его позициям и UV: число реально помеченных швов.

    Для точного lean-кадра, чья раскладка не та, что в меше (WIDTH_REFRESH_LEAN_V1), и для отмены после него. Тот же результат, что у `_clear_mesh_in_place` + `_fill_mesh`
    по массивам этого прогона (`from_pydata`, слой UV, атрибуты граней, швы), но через `foreach_set` из numpy: без питоновских кортежей и без цикла по рёбрам меша
    (здание: 3.7 мс против 18.4; сверка с `_fill_mesh` по всем массивам меша - `tests/blender/test_envelope_mesh_numpy_paths.py`). Датаблок, объект и материал остаются теми же.
    """

    _clear_mesh_in_place(mesh)
    vertex_count = int(positions.shape[0])
    mesh.vertices.add(vertex_count)
    _write_positions(mesh, np.ascontiguousarray(positions, dtype=np.float32).reshape(-1))
    mesh.loops.add(int(image.face_vertices.shape[0]))
    mesh.polygons.add(image.face_count)
    mesh.polygons.foreach_set("loop_start", image.face_starts)
    _write_corners(mesh, image.face_vertices)
    _shade_flat(mesh)  # как `from_pydata` (его умолчание): гладкая заливка граней не нужна
    mesh.update(calc_edges=bool(image.face_count), calc_edges_loose=False)
    layer = mesh.uv_layers.new(name=DECAL_UV_LAYER)
    _write_uvs(mesh, layer, np.ascontiguousarray(uvs, dtype=np.float32).reshape(-1))
    for attribute_name, values in ((DECAL_DOMAIN_ATTRIBUTE, image.face_domain), (DECAL_OWNER_ATTRIBUTE, image.face_owner)):
        attribute = mesh.attributes.new(name=attribute_name, type="INT", domain="FACE")
        attribute.data.foreach_set("value", values)
    return _mark_seams(mesh, image.seams, vertex_count)


def _mark_seams(mesh, seams, vertex_count: int) -> int:
    """Швы меша по парам вершин (каждая пара - `(меньшая, большая)`): число помеченных рёбер. Ребро - пара вершин, поэтому ключ пары однозначен."""

    if not len(seams):
        return 0
    pairs = np.empty(len(mesh.edges) * 2, dtype=np.int32)
    mesh.edges.foreach_get("vertices", pairs)
    pairs = pairs.reshape(-1, 2).astype(np.int64)
    key = np.minimum(pairs[:, 0], pairs[:, 1]) * vertex_count + np.maximum(pairs[:, 0], pairs[:, 1])
    wanted = seams.astype(np.int64).reshape(-1, 2)
    keys = np.sort(np.minimum(wanted[:, 0], wanted[:, 1]) * vertex_count + np.maximum(wanted[:, 0], wanted[:, 1]))
    at = np.minimum(np.searchsorted(keys, key), keys.size - 1)
    mask = keys[at] == key  # то же, что `np.isin(key, keys)`, без сортировки объединения
    mesh.edges.foreach_set("use_seam", mask)
    return int(mask.sum())


def record_decal_width(mesh, width) -> None:
    """Ширина, при которой записан меш (свойство МЕША, его читает `reconcile_after_history` и проверка владения): после lean-кадра она - ширина кадра."""

    _record_width(mesh, width)


OUTCOME_PREVIEW_SHAPE_MISMATCH = "PREVIEW_MESH_SHAPE_MISMATCH"
OUTCOME_PREVIEW_NO_UV_LAYER = "PREVIEW_MESH_UV_LAYER_MISSING"
OUTCOME_PREVIEW_NO_OWNER_ATTRIBUTE = "PREVIEW_MESH_OWNER_ATTRIBUTE_MISSING"


def write_preview_geometry(mesh, positions, uvs, owners=None) -> None:
    """Позиции и UV превью в СУЩЕСТВУЮЩИЙ меш того же состава: `foreach_set`, без пересоздания геометрии (путь живой ширины).

    Единственный писатель превью. Его вызывает только `envelope_width_mesh_preview` (стена `tests/test_architecture.py`), и
    он не создаёт и не освобождает датаблоки: таймер и модальный оператор вправе его звать, не ломая шаг отмены (см.
    `rewrite_decal_mesh`). `positions` и `uvs` — плоские float32 (`vertices * 3`, петли UV `* 2`); состав меша другой —
    именованный отказ, а не запись наугад. Топологию, швы, атрибуты граней и материал писатель не трогает: они те же, что у
    точного результата, на котором держится модель.

    `owners` (int32, номер владельца каждой грани) - только у точного lean-кадра и возврата базы после него (WIDTH_REFRESH_LEAN_V1): ядро нумерует огибающие по именам, имена несут
    ширину, и номера меняются от ширины к ширине при той же структуре, а точная запись кладёт их в атрибут граней `cftuv_owner`. Кадр модели владельцев не трогает (`None`).
    """

    layer = mesh.uv_layers.get(DECAL_UV_LAYER)
    if layer is None:
        raise ProductionWriteError(OUTCOME_PREVIEW_NO_UV_LAYER, f"mesh {mesh.name!r} has no UV layer {DECAL_UV_LAYER!r}")
    if len(mesh.vertices) * 3 != len(positions) or len(layer.data) * 2 != len(uvs):
        raise ProductionWriteError(
            OUTCOME_PREVIEW_SHAPE_MISMATCH,
            f"the preview carries {len(positions) // 3} vertices and {len(uvs) // 2} loops, "
            f"mesh {mesh.name!r} has {len(mesh.vertices)} and {len(layer.data)}",
        )
    attribute = None
    if owners is not None:
        attribute = mesh.attributes.get(DECAL_OWNER_ATTRIBUTE)
        if attribute is None or len(attribute.data) != len(owners):
            raise ProductionWriteError(
                OUTCOME_PREVIEW_NO_OWNER_ATTRIBUTE, f"mesh {mesh.name!r} has no face attribute {DECAL_OWNER_ATTRIBUTE!r} of {len(owners)} faces"
            )
    _write_positions(mesh, positions)
    _write_uvs(mesh, layer, uvs)
    if attribute is not None:
        attribute.data.foreach_set("value", owners)
    mesh.update()


__all__ = (
    "DECAL_DOMAIN_ATTRIBUTE",
    "DECAL_OBJECT_SUFFIX",
    "DECAL_OWNER_ATTRIBUTE",
    "DECAL_REVISION_PROPERTY",
    "DECAL_SOURCE_PROPERTY",
    "DECAL_UV_LAYER",
    "DECAL_WIDTH_PROPERTY",
    "DEFAULT_DECAL_MATERIAL",
    "DEFAULT_DECAL_OFFSET",
    "ID_NAME_LIMIT_BYTES",
    "MeshArraysV1",
    "OUTCOME_COLLECTION_REASSERTED",
    "OUTCOME_DECAL_IN_EDIT_MODE",
    "OUTCOME_DECAL_MESH_MISSING",
    "OUTCOME_DECAL_MISSING",
    "OUTCOME_DUPLICATE_DECALS",
    "OUTCOME_EMPTY_BATCH",
    "OUTCOME_FLIPPED_VS_SOURCE",
    "OUTCOME_LEAN_ARRAYS",
    "OUTCOME_MATERIAL_MISSING",
    "OUTCOME_MESH_SHARED",
    "OUTCOME_NAME_TAKEN",
    "OUTCOME_NON_FINITE",
    "OUTCOME_NORMAL_MISSING",
    "OUTCOME_NORMAL_OPPOSES_SOURCE",
    "OUTCOME_PARENT_REASSERTED",
    "OUTCOME_PREVIEW_NO_OWNER_ATTRIBUTE",
    "OUTCOME_PREVIEW_NO_UV_LAYER",
    "OUTCOME_PREVIEW_SHAPE_MISMATCH",
    "OUTCOME_SEAM_EDGE_MISSING",
    "OUTCOME_SOURCE_NORMAL_UNKNOWN",
    "OUTCOME_VERTEX_MISSING",
    "ProductionWriteError",
    "ProductionWriteReceiptV1",
    "build_mesh_arrays",
    "decal_object_name",
    "find_decal_object",
    "mesh_content_digest",
    "rebuild_mesh_from_image",
    "record_decal_width",
    "rewrite_decal_mesh",
    "resolve_decal_material_name",
    "write_decal_object",
    "write_preview_geometry",
)
