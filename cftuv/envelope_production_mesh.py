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
from dataclasses import dataclass

import bpy

from .envelope_production_weld import (
    COUNTER_SEAM_CLIP_VERTICES,
    COUNTER_SEAM_T_JUNCTIONS,
    COUNTER_WELD_SEAMS_MARKED,
    OUTCOME_SEAM_T_JUNCTIONS,
    OUTCOME_WELD_HALF_EDGE_CONFLICT,
    cross_domain_seams,
    half_edge_conflicts,
    off_plane_after_offset,
    seam_report_of_chains,
    weld_vertices,
)
from .envelope_production_view import (
    OUTCOME_EMPTY_BATCH,
    OUTCOME_NON_FINITE,
    OUTCOME_NORMAL_MISSING,
    OUTCOME_NORMAL_OPPOSES_SOURCE,
    OUTCOME_VERTEX_MISSING,
    build_domain_view,
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
    #: Состав меша по ЗАПИСАННЫМ доменам в порядке записи (читает сертификат превью ширины, `envelope_width_certificate`): `(патч,
    #: домен)`, номера вершин меша по локальным вершинам домена, число петель UV (петли домена лежат в `uvs` подряд) и токен
    #: локальной структуры (грани, швы, ссылки вершин; чисел в нём нет). В дайджест не входят.
    domain_keys: tuple = ()
    domain_vertex_index: tuple = ()
    domain_loop_counts: tuple = ()
    domain_tokens: tuple = ()


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


def _arrays_digest(arrays: dict) -> str:
    text = json.dumps(arrays, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("ascii")).hexdigest()


def build_mesh_arrays(results, offset: float) -> MeshArraysV1:
    """Массивы меша по результатам домена. Не-MATERIALIZED домены — в `skipped`.

    Домены идут по номеру патча, вершины внутри домена — по ключу батча:
    порядок не зависит ни от воркера, ни от хеш-порядка множеств батча. Вершины
    соседних доменов с одной ссылкой `location:src:<id>` и побитово равными
    позициями сварены в одну (`envelope_production_weld`), смещение над
    поверхностью — митра общих вершин.
    """

    faces, uvs = [], []
    face_domain, face_owner = [], []
    domains, skipped, warnings = [], [], []
    entries = []
    revision = ""
    laws: set[str] = set()
    for result in sorted(results, key=lambda item: (item.patch_id, item.domain_id)):
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
    seam_pairs: set = set()
    domain_keys, domain_vertex_index, domain_loop_counts, domain_tokens = [], [], [], []
    for (result, view), index in zip(entries, weld.index):
        _vertices, d_faces, d_uvs, d_owner, d_seams = view.arrays
        faces.extend(tuple(index[item] for item in loop) for loop in d_faces)
        uvs.extend(d_uvs)
        face_domain.extend([result.patch_id] * len(d_faces))
        face_owner.extend(d_owner)
        seam_pairs.update(tuple(sorted((index[a], index[b]))) for a, b in d_seams)
        domain_keys.append((result.patch_id, result.domain_id))
        domain_vertex_index.append(tuple(index))
        domain_loop_counts.append(len(d_uvs))
        domain_tokens.append(_structure_token(view))
    folds = cross_domain_seams(faces, uvs, face_domain)
    seam_pairs.update(folds)
    seams = sorted(seam_pairs)
    positions = list(weld.positions)
    conflicts = half_edge_conflicts(faces)
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
    seam = seam_report_of_chains([view.boundary_chains for _result, view in entries])
    seam_found = dict(seam)
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
    digest = _arrays_digest(
        {
            "positions": positions,
            "faces": faces,
            "uvs": uvs,
            "face_domain": face_domain,
            "face_owner": face_owner,
            "seams": seams,
        }
    )
    return MeshArraysV1(
        positions=tuple(positions),
        faces=tuple(faces),
        uvs=tuple(uvs),
        face_domain=tuple(face_domain),
        face_owner=tuple(face_owner),
        seam_edges=tuple(seams),
        domains=tuple(domains),
        skipped=tuple(skipped),
        warnings=tuple(warnings),
        source_revision=revision,
        digest=digest,
        decal_topology_law=",".join(sorted(item for item in laws if item)),
        weld_counters=(
            *weld.counters,
            (OUTCOME_WELD_HALF_EDGE_CONFLICT, conflicts),
            (COUNTER_WELD_SEAMS_MARKED, len(folds)),
        ),
        offset_counters=off_plane_after_offset(positions, faces),
        seam_counters=seam,
        domain_keys=tuple(domain_keys),
        domain_vertex_index=tuple(domain_vertex_index),
        domain_loop_counts=tuple(domain_loop_counts),
        domain_tokens=tuple(domain_tokens),
    )


def _structure_token(view) -> str:
    """Токен локальной структуры домена: грани, швы и ссылки вершин в локальной нумерации (чисел позиций и UV в нём нет).

    Два точных прогона одного домена с равным токеном кладут вершины и петли в меш одинаково, и превью ширины вправе
    сравнивать их по номерам (`envelope_width_certificate`). Номера владельцев (`claim:N`) в токен не входят: они меняют
    лишь целочисленный атрибут граней, а не геометрию.
    """

    text = repr((view.faces, view.seams, view.vertices.refs, len(view.uvs)))
    return hashlib.blake2b(text.encode("utf-8"), digest_size=10).hexdigest()


# --------------------------------------------------------------------------
# Blender
# --------------------------------------------------------------------------


def mesh_content_digest(mesh) -> str:
    """Отпечаток меша ПО ТОМУ, ЧТО ЛЕЖИТ В BLENDER (float32 и порядок петель).

    Позиции и UV читаются обратно, поэтому дайджест видит и округление
    хранилища: два прогона дают равные дайджесты, когда равны их меши.
    """

    uv_layer = mesh.uv_layers.get(DECAL_UV_LAYER)
    payload = {
        "positions": [tuple(item.co) for item in mesh.vertices],
        "faces": [tuple(item.vertices) for item in mesh.polygons],
        "uvs": [] if uv_layer is None else [tuple(item.uv) for item in uv_layer.data],
        "seams": sorted(
            tuple(sorted(item.vertices)) for item in mesh.edges if item.use_seam
        ),
    }
    for name in (DECAL_DOMAIN_ATTRIBUTE, DECAL_OWNER_ATTRIBUTE):
        attribute = mesh.attributes.get(name)
        payload[name] = (
            [] if attribute is None else [item.value for item in attribute.data]
        )
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
    """

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
             material_created, warnings):
    seam_requested = len(arrays.seam_edges)
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
    return ProductionWriteReceiptV1(
        object_name=object_name,
        replaced=replaced,
        vertices=len(arrays.positions),
        faces=len(arrays.faces),
        loops=len(arrays.uvs),
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
        mesh_digest="" if mesh is None else mesh_content_digest(mesh),
        decal_topology_law=arrays.decal_topology_law,
        quads=sum(1 for loop in arrays.faces if len(loop) == 4),
        triangles=sum(1 for loop in arrays.faces if len(loop) == 3),
        polygons=sum(1 for loop in arrays.faces if len(loop) > 4),
        weld_counters=arrays.weld_counters,
        offset_counters=arrays.offset_counters,
        seam_counters=arrays.seam_counters,
    )


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
    name = decal_object_name(source_obj.name)
    existing, warnings = _find_decal(source_obj, name)
    if existing is not None:
        warnings = warnings + _reassert_placement(existing, source_obj)
    if not arrays.faces:
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
    marked = _fill_mesh(mesh, arrays) if arrays.faces else 0
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
        arrays if arrays.faces else _blank_arrays(arrays),
        offset,
        material_name,
        object_name=existing.name,
        replaced=True,
        mesh=mesh,
        marked=marked,
        material_created=False,
        warnings=warnings,
    )


OUTCOME_PREVIEW_SHAPE_MISMATCH = "PREVIEW_MESH_SHAPE_MISMATCH"
OUTCOME_PREVIEW_NO_UV_LAYER = "PREVIEW_MESH_UV_LAYER_MISSING"


def write_preview_geometry(mesh, positions, uvs) -> None:
    """Позиции и UV превью в СУЩЕСТВУЮЩИЙ меш того же состава: `foreach_set`, без пересоздания геометрии (путь живой ширины).

    Единственный писатель превью. Его вызывает только `envelope_width_mesh_preview` (стена `tests/test_architecture.py`), и
    он не создаёт и не освобождает датаблоки: таймер и модальный оператор вправе его звать, не ломая шаг отмены (см.
    `rewrite_decal_mesh`). `positions` и `uvs` — плоские float32 (`vertices * 3`, петли UV `* 2`); состав меша другой —
    именованный отказ, а не запись наугад. Топологию, швы, атрибуты граней и материал писатель не трогает: они те же, что у
    точного результата, на котором держится сертификат.
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
    mesh.vertices.foreach_set("co", positions)
    layer.data.foreach_set("uv", uvs)
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
    "OUTCOME_MATERIAL_MISSING",
    "OUTCOME_MESH_SHARED",
    "OUTCOME_NAME_TAKEN",
    "OUTCOME_NON_FINITE",
    "OUTCOME_NORMAL_MISSING",
    "OUTCOME_NORMAL_OPPOSES_SOURCE",
    "OUTCOME_PARENT_REASSERTED",
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
    "rewrite_decal_mesh",
    "write_decal_object",
    "write_preview_geometry",
)
