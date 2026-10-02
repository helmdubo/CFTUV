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

ВЕРШИНЫ СВАРИВАЮТСЯ ТОЛЬКО ПО СЕМАНТИЧЕСКОМУ КЛЮЧУ БАТЧА и только внутри
домена: ключ `(домен, vert_key)` — одна вершина. По расстоянию — никогда:
вершины `src:` лежат на сертифицированной плоскости и снапе решётки, а не на
исходном мешу, и склейка «по координатам» с исходником или соседним доменом была бы
ровно тем молчаливым ремонтом, который адаптеру запрещён. У соседних доменов
смещение идёт вдоль СВОИХ нормалей, поэтому общая исходная вершина у них разная.
Одна вершина на ключ при разных UV в разных регионах — шов UV, а не дубликат.

ШВЫ. Интерфейсные цепи батча (общие полурёбра граней разных регионов — изломы
цепей и границы владения, где станции `(s, r)` разные) помечаются швами Blender
(`use_seam`): это ровно те рёбра, на которых UV разрывны. Решение хоста, не ядра.

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
import math
from dataclasses import dataclass

import bpy

DECAL_OBJECT_SUFFIX = ".CFTUV_Decal"
DECAL_UV_LAYER = "UVMap"
DECAL_DOMAIN_ATTRIBUTE = "cftuv_domain"
DECAL_OWNER_ATTRIBUTE = "cftuv_owner"
DECAL_REVISION_PROPERTY = "cftuv_source_revision"
DECAL_SOURCE_PROPERTY = "cftuv_source_object"
#: Смещение над поверхностью по умолчанию, метры: прежнее значение декаль-режима.
DEFAULT_DECAL_OFFSET = 0.02
DEFAULT_DECAL_MATERIAL = "CFTUV_Decal"

#: Имя ID Blender — не больше 63 байт UTF-8 (`MAX_ID_NAME - 2`).
ID_NAME_LIMIT_BYTES = 63

OUTCOME_NAME_TAKEN = "DECAL_OBJECT_NAME_TAKEN"
OUTCOME_EMPTY_BATCH = "ADAPTER_EMPTY_BATCH"
OUTCOME_NON_FINITE = "ADAPTER_NON_FINITE_BATCH"
OUTCOME_NORMAL_MISSING = "ADAPTER_NORMAL_MISSING"
OUTCOME_VERTEX_MISSING = "ADAPTER_FACE_VERTEX_MISSING"
OUTCOME_NORMAL_OPPOSES_SOURCE = "ADAPTER_NORMAL_OPPOSES_SOURCE"
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


def _finite(values) -> bool:
    return all(math.isfinite(item) for item in values)


def _claim_ordinals(batch) -> dict[str, int]:
    """Порядковый номер каждой огибающей батча: `claim:N` -> N, иначе по сортировке."""

    names = sorted({face.ownership_claim_id.value for face in batch.faces})
    try:
        return {name: int(name.split(":", 1)[1]) for name in names}
    except (IndexError, ValueError):
        return {name: index for index, name in enumerate(names)}


def _domain_arrays(result, offset: float):
    """`(позиции, грани, uv, владельцы, швы)` одного домена либо `(None, исход, деталь)`."""

    batch = result.batch
    if not batch.faces or not batch.vertices:
        return None, OUTCOME_EMPTY_BATCH, "the batch has no faces or no vertices"
    if result.normal is None or not _finite(result.normal):
        return None, OUTCOME_NORMAL_MISSING, "the plane normal of the domain is absent"
    nx, ny, nz = result.normal
    vertex_normals = dict(getattr(result, "vertex_normals", ()) or ())
    source = getattr(result, "source_normal", None)
    # У домена-развёртки нормаль смещения своя на вершину (закон ядра), и каждая уже
    # проверена ядром против нормалей её треугольников; одной нормалью первой грани
    # на сгибе проверять нечего.
    if source is not None and any(source) and not vertex_normals:
        dot = nx * source[0] + ny * source[1] + nz * source[2]
        if not dot > 0.0:
            return (
                None,
                OUTCOME_NORMAL_OPPOSES_SOURCE,
                f"plane normal . source face normal = {dot:.6f} <= 0: the offset "
                f"would push the decal into the surface",
            )
    ordered = sorted(batch.vertices, key=lambda item: item.vert_key.value)
    index = {item.vert_key.value: number for number, item in enumerate(ordered)}
    positions = []
    for vertex in ordered:
        point = vertex.position
        if not _finite((point.x, point.y, point.z)):
            return None, OUTCOME_NON_FINITE, f"vertex {vertex.vert_key.value}"
        shift = vertex_normals.get(vertex.vert_key.value, (nx, ny, nz)) if vertex_normals else (nx, ny, nz)
        if vertex_normals and vertex.vert_key.value not in vertex_normals:
            return None, OUTCOME_NORMAL_MISSING, f"the offset normal of vertex {vertex.vert_key.value} is absent"
        positions.append(
            (point.x + offset * shift[0], point.y + offset * shift[1], point.z + offset * shift[2])
        )
    owners = _claim_ordinals(batch)
    faces, uvs, face_owner = [], [], []
    for face in batch.faces:
        try:
            loop = tuple(index[key.value] for key in face.ordered_vert_keys)
        except KeyError as exc:
            return None, OUTCOME_VERTEX_MISSING, f"face {face.face_id.value}: {exc}"
        pairs = tuple((fact.uv.u, fact.uv.v) for fact in face.uv_facts)
        if not _finite([item for pair in pairs for item in pair]):
            return None, OUTCOME_NON_FINITE, f"UV of face {face.face_id.value}"
        faces.append(loop)
        uvs.extend(pairs)
        face_owner.append(owners[face.ownership_claim_id.value])
    seams = set()
    for chain in batch.interface_chains:
        keys = [key.value for key in chain.ordered_vert_keys]
        for first, second in zip(keys, keys[1:]):
            if first in index and second in index and first != second:
                seams.add(tuple(sorted((index[first], index[second]))))
    return (positions, faces, uvs, face_owner, sorted(seams)), "", ""


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
    порядок не зависит ни от воркера, ни от хеш-порядка множеств батча.
    """

    positions, faces, uvs = [], [], []
    face_domain, face_owner, seams = [], [], []
    domains, skipped, warnings = [], [], []
    revision = ""
    laws: set[str] = set()
    for result in sorted(results, key=lambda item: (item.patch_id, item.domain_id)):
        if not result.is_materialized:
            skipped.append(
                (result.patch_id, result.domain_id, result.outcome, result.detail)
            )
            continue
        built, outcome, detail = _domain_arrays(result, float(offset))
        if built is None:
            skipped.append((result.patch_id, result.domain_id, outcome, detail))
            continue
        base = len(positions)
        d_positions, d_faces, d_uvs, d_owner, d_seams = built
        positions.extend(d_positions)
        faces.extend(tuple(base + item for item in loop) for loop in d_faces)
        uvs.extend(d_uvs)
        face_domain.extend([result.patch_id] * len(d_faces))
        face_owner.extend(d_owner)
        seams.extend((base + a, base + b) for a, b in d_seams)
        domains.append(result.patch_id)
        warnings.extend(_domain_warnings(result))
        laws.add(getattr(result, "decal_topology_law", ""))
        revision = revision or result.batch.source_revision.value
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
    )


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
    return mesh, marked


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
    )


def _blank_arrays(arrays: MeshArraysV1) -> MeshArraysV1:
    """Массивы пустого результата: ничего не записано, пропуски и предупреждения те же."""

    return MeshArraysV1(
        positions=(), faces=(), uvs=(), face_domain=(), face_owner=(),
        seam_edges=(), domains=(), skipped=arrays.skipped, warnings=arrays.warnings,
        source_revision=arrays.source_revision, digest=arrays.digest,
        decal_topology_law=arrays.decal_topology_law,
    )


def write_decal_object(
    source_obj,
    results,
    *,
    offset: float = DEFAULT_DECAL_OFFSET,
    material_name: str = DEFAULT_DECAL_MATERIAL,
) -> ProductionWriteReceiptV1:
    """Записывает все MATERIALIZED домены одним объектом `<исходный>.CFTUV_Decal`.

    Повторный вызов заменяет меш объекта (идемпотентно: тот же вход — тот же
    дайджест, один объект, один материал); объект ищется по маркеру, а не по
    имени (имя Blender режет до 63 байт). Пустой результат НЕ создаёт объекта, а
    у существующего заменяет меш пустым: устаревший декаль не должен выдавать себя
    за свежий. Отказ имени — `ProductionWriteError`.
    """

    arrays = build_mesh_arrays(results, offset)
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
    return _receipt(arrays, offset, material_name, object_name=decal.name,
                    replaced=existing is not None, mesh=mesh, marked=marked,
                    material_created=material_created, warnings=warnings)


__all__ = (
    "DECAL_DOMAIN_ATTRIBUTE",
    "DECAL_OBJECT_SUFFIX",
    "DECAL_OWNER_ATTRIBUTE",
    "DECAL_REVISION_PROPERTY",
    "DECAL_SOURCE_PROPERTY",
    "DECAL_UV_LAYER",
    "DEFAULT_DECAL_MATERIAL",
    "DEFAULT_DECAL_OFFSET",
    "ID_NAME_LIMIT_BYTES",
    "MeshArraysV1",
    "OUTCOME_COLLECTION_REASSERTED",
    "OUTCOME_DUPLICATE_DECALS",
    "OUTCOME_EMPTY_BATCH",
    "OUTCOME_FLIPPED_VS_SOURCE",
    "OUTCOME_MESH_SHARED",
    "OUTCOME_NAME_TAKEN",
    "OUTCOME_NON_FINITE",
    "OUTCOME_NORMAL_MISSING",
    "OUTCOME_NORMAL_OPPOSES_SOURCE",
    "OUTCOME_PARENT_REASSERTED",
    "OUTCOME_SEAM_EDGE_MISSING",
    "OUTCOME_SOURCE_NORMAL_UNKNOWN",
    "OUTCOME_VERTEX_MISSING",
    "ProductionWriteError",
    "ProductionWriteReceiptV1",
    "build_mesh_arrays",
    "decal_object_name",
    "mesh_content_digest",
    "write_decal_object",
)
