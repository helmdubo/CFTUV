"""Писатель продуктового меша: `GeometryBatchV1` доменов -> ОДИН объект Blender.

Адаптер ОТОБРАЖАЕТ контракт и не чинит геометрию (AGENTS.md): позиции, UV,
владельцы и цепи берутся из батча как есть. Хосту принадлежит ровно одно
решение — политика отображения: смещение над поверхностью вдоль нормали патча
(борьба с z-fighting; прежний декаль-режим держал 0.02), материал слота и имя
объекта. Всё остальное — запись фактов.

ФОРМА РЕЗУЛЬТАТА. Один объект `<исходный>.CFTUV_Decal`, потомок исходного с
единичным локальным преобразованием (позиции батча — в локальных координатах
источника, и родитель переводит их в мир сам), ВСЕ домены в одном меше. UV —
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

ПЕРЕСБОРКА ИДЕМПОТЕНТНА. Существующий объект с этим именем и маркером
`cftuv_source_revision` получает НОВЫЙ меш (старый удаляется, если им никто не
пользуется); объект, потомки, материал и ручные настройки остаются. Объект с тем
же именем БЕЗ маркера не трогается — именованный отказ `DECAL_OBJECT_NAME_TAKEN`.

НИЧЕГО НЕ ПРОПАДАЕТ МОЛЧА. Домен, который не удалось записать (отказ продуктового
пути, пустой батч, нечисловая позиция или UV, отсутствующая нормаль, вершина грани
без записи), попадает в `skipped` квитанции вместе с исходом. Цена — вместо
угадывания: такой домен в меше отсутствует, а квитанция говорит какой и почему.
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

OUTCOME_NAME_TAKEN = "DECAL_OBJECT_NAME_TAKEN"
OUTCOME_EMPTY_BATCH = "ADAPTER_EMPTY_BATCH"
OUTCOME_NON_FINITE = "ADAPTER_NON_FINITE_BATCH"
OUTCOME_NORMAL_MISSING = "ADAPTER_NORMAL_MISSING"
OUTCOME_VERTEX_MISSING = "ADAPTER_FACE_VERTEX_MISSING"


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
    source_revision: str
    digest: str


@dataclass(frozen=True, slots=True)
class ProductionWriteReceiptV1:
    """Квитанция записи: что лежит в объекте и что названо пропущенным."""

    object_name: str | None
    replaced: bool
    vertices: int
    faces: int
    loops: int
    seam_edges: int
    uv_layer: str
    material_name: str
    material_created: bool
    offset: float
    domains: tuple
    skipped: tuple
    source_revision: str
    arrays_digest: str
    mesh_digest: str


def decal_object_name(source_name: str) -> str:
    return f"{source_name}{DECAL_OBJECT_SUFFIX}"


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
    ordered = sorted(batch.vertices, key=lambda item: item.vert_key.value)
    index = {item.vert_key.value: number for number, item in enumerate(ordered)}
    positions = []
    for vertex in ordered:
        point = vertex.position
        if not _finite((point.x, point.y, point.z)):
            return None, OUTCOME_NON_FINITE, f"vertex {vertex.vert_key.value}"
        positions.append(
            (point.x + offset * nx, point.y + offset * ny, point.z + offset * nz)
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
    domains, skipped = [], []
    revision = ""
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
        source_revision=revision,
        digest=digest,
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
    if arrays.seam_edges:
        wanted = {tuple(pair) for pair in arrays.seam_edges}
        for edge in mesh.edges:
            if tuple(sorted(edge.vertices)) in wanted:
                edge.use_seam = True
    return mesh


def _material_slot(mesh, material_name: str):
    material = bpy.data.materials.get(material_name)
    created = material is None
    if created:
        material = bpy.data.materials.new(material_name)
    mesh.materials.append(material)
    return created


def _existing_object(name: str):
    existing = bpy.data.objects.get(name)
    if existing is None:
        return None
    if existing.type != "MESH" or DECAL_REVISION_PROPERTY not in existing.keys():
        raise ProductionWriteError(
            OUTCOME_NAME_TAKEN,
            f"object {name!r} exists and was not built by CFTUV "
            f"(no {DECAL_REVISION_PROPERTY} marker); it is left untouched",
        )
    return existing


def _link_child(source_obj, name: str, mesh):
    decal = bpy.data.objects.new(name, mesh)
    collections = tuple(getattr(source_obj, "users_collection", ()) or ())
    (collections[0] if collections else bpy.context.scene.collection).objects.link(
        decal
    )
    decal.parent = source_obj
    return decal


def _empty_receipt(name, replaced, arrays, offset, material_name, mesh_digest=""):
    return ProductionWriteReceiptV1(
        object_name=name,
        replaced=replaced,
        vertices=0,
        faces=0,
        loops=0,
        seam_edges=0,
        uv_layer=DECAL_UV_LAYER,
        material_name=material_name,
        material_created=False,
        offset=float(offset),
        domains=(),
        skipped=arrays.skipped,
        source_revision=arrays.source_revision,
        arrays_digest=arrays.digest,
        mesh_digest=mesh_digest,
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
    дайджест, один объект, один материал). Пустой результат НЕ создаёт объекта, а
    у существующего заменяет меш пустым: устаревший декаль не должен выдавать себя
    за свежий. Отказ имени — `ProductionWriteError`.
    """

    arrays = build_mesh_arrays(results, offset)
    name = decal_object_name(source_obj.name)
    existing = _existing_object(name)
    if not arrays.faces:
        if existing is None:
            return _empty_receipt(None, False, arrays, offset, material_name)
        old = existing.data
        empty = bpy.data.meshes.new(name)
        existing.data = empty
        if old is not None and getattr(old, "users", 1) == 0:
            bpy.data.meshes.remove(old)
        empty.name = name
        existing[DECAL_REVISION_PROPERTY] = arrays.source_revision
        return _empty_receipt(
            name, True, arrays, offset, material_name,
            mesh_content_digest(existing.data),
        )
    mesh = _build_mesh(name, arrays)
    material_created = _material_slot(mesh, material_name)
    if existing is None:
        decal = _link_child(source_obj, name, mesh)
    else:
        old = existing.data
        existing.data = mesh
        if old is not None and getattr(old, "users", 1) == 0:
            bpy.data.meshes.remove(old)
        # Имя датаблока одно и то же при любой пересборке: старый меш уже
        # удалён, и Blender не припишет новому `.001`.
        mesh.name = name
        decal = existing
    decal[DECAL_REVISION_PROPERTY] = arrays.source_revision
    decal[DECAL_SOURCE_PROPERTY] = source_obj.name
    return ProductionWriteReceiptV1(
        object_name=decal.name,
        replaced=existing is not None,
        vertices=len(arrays.positions),
        faces=len(arrays.faces),
        loops=len(arrays.uvs),
        seam_edges=len(arrays.seam_edges),
        uv_layer=DECAL_UV_LAYER,
        material_name=material_name,
        material_created=material_created,
        offset=float(offset),
        domains=arrays.domains,
        skipped=arrays.skipped,
        source_revision=arrays.source_revision,
        arrays_digest=arrays.digest,
        mesh_digest=mesh_content_digest(mesh),
    )


__all__ = (
    "DECAL_DOMAIN_ATTRIBUTE",
    "DECAL_OBJECT_SUFFIX",
    "DECAL_OWNER_ATTRIBUTE",
    "DECAL_REVISION_PROPERTY",
    "DECAL_UV_LAYER",
    "DEFAULT_DECAL_MATERIAL",
    "DEFAULT_DECAL_OFFSET",
    "MeshArraysV1",
    "OUTCOME_EMPTY_BATCH",
    "OUTCOME_NAME_TAKEN",
    "OUTCOME_NON_FINITE",
    "OUTCOME_NORMAL_MISSING",
    "OUTCOME_VERTEX_MISSING",
    "ProductionWriteError",
    "ProductionWriteReceiptV1",
    "build_mesh_arrays",
    "decal_object_name",
    "mesh_content_digest",
    "write_decal_object",
)
