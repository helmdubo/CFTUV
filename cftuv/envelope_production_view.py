"""Вид домена для писателя меша: ровно то, что `build_mesh_arrays` читает из результата домена.

Писатель меша читает из `GeometryBatchV1` домена немного: вершины (ключ, позиция, ссылка), грани (петля ключей, UV, владелец),
интерфейсные цепи (швы) и граничные цепи (отчёт шва). Происхождение, факты станций, регионы, диагностики и остальное он не
трогает. Вид — эти числа в плоском виде (кортежи чисел и строк): его считает ВОРКЕР тем же кодом (`build_domain_view`), что считает
писатель из батча, и родителю не нужно ни разворачивать граф записей батча, ни ходить по нему (на `building` 122 доменов это
~0.4 с разбора пикла в родителе на каждом шаге ширины, под GIL, внутри стены пула).

ТОТ ЖЕ КОД — ТОТ ЖЕ ОТВЕТ. Вид считается одной функцией и из присланного воркером результата, и в родителе из батча (домен, который считал
родитель; результат, перенесённый на другую ревизию), поэтому массивы меша от места счёта не зависят. Отказ домена писателя
(`ADAPTER_*`) — тоже часть вида: он называется одинаково, где бы вид ни считали.

Модуль не знает ни Blender, ни ядра на уровне импорта: он читает поля готового батча.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .envelope_production_weld import DomainVerticesV1, boundary_chain_names

OUTCOME_EMPTY_BATCH = "ADAPTER_EMPTY_BATCH"
OUTCOME_NON_FINITE = "ADAPTER_NON_FINITE_BATCH"
OUTCOME_NORMAL_MISSING = "ADAPTER_NORMAL_MISSING"
OUTCOME_VERTEX_MISSING = "ADAPTER_FACE_VERTEX_MISSING"
OUTCOME_NORMAL_OPPOSES_SOURCE = "ADAPTER_NORMAL_OPPOSES_SOURCE"


@dataclass(frozen=True, slots=True)
class DomainViewV1:
    """Вершины, грани, UV, владельцы и швы домена (индексы — в порядке ключей вершин), цепи батча и ревизия источника.

    `failure` — `(исход, деталь)`, если писатель домен не примет (`ADAPTER_*`); тогда остальные поля пусты.
    `boundary_chains` — `boundary_chain_names(батч)`: вход отчёта шва (`seam_report_of_chains`).
    """

    failure: tuple | None
    vertices: DomainVerticesV1 | None
    faces: tuple
    uvs: tuple
    owners: tuple
    seams: tuple
    boundary_chains: tuple
    source_revision: str

    @property
    def arrays(self) -> tuple:
        """`(вершины, грани, uv, владельцы, швы)` — то, что кладёт в меш `build_mesh_arrays`."""

        return self.vertices, self.faces, self.uvs, self.owners, self.seams


def _finite(values) -> bool:
    return all(math.isfinite(item) for item in values)


def _claim_ordinals(batch) -> dict[str, int]:
    """Порядковый номер каждой огибающей батча: `claim:N` -> N, иначе по сортировке."""

    names = sorted({face.ownership_claim_id.value for face in batch.faces})
    try:
        return {name: int(name.split(":", 1)[1]) for name in names}
    except (IndexError, ValueError):
        return {name: index for index, name in enumerate(names)}


def _failed(outcome: str, detail: str, source_revision: str = "") -> DomainViewV1:
    return DomainViewV1((outcome, detail), None, (), (), (), (), (), source_revision)


def build_domain_view(result) -> DomainViewV1:
    """Вид материализованного домена по его батчу, нормали и нормалям вершин; отказ писателя — `failure`.

    Вершины — `DomainVerticesV1`: позиция батча, нормаль смещения и ссылка каждой. Смещение здесь не прикладывается: у общей
    вершины оно митра нескольких доменов (сварка).
    """

    batch = result.batch
    revision = batch.source_revision.value
    if not batch.faces or not batch.vertices:
        return _failed(OUTCOME_EMPTY_BATCH, "the batch has no faces or no vertices", revision)
    if result.normal is None or not _finite(result.normal):
        return _failed(OUTCOME_NORMAL_MISSING, "the plane normal of the domain is absent", revision)
    nx, ny, nz = result.normal
    vertex_normals = dict(getattr(result, "vertex_normals", ()) or ())
    source = getattr(result, "source_normal", None)
    # У домена-развёртки нормаль смещения своя на вершину (закон ядра), и каждая уже
    # проверена ядром против нормалей её треугольников; одной нормалью первой грани
    # на сгибе проверять нечего.
    if source is not None and any(source) and not vertex_normals:
        dot = nx * source[0] + ny * source[1] + nz * source[2]
        if not dot > 0.0:
            return _failed(
                OUTCOME_NORMAL_OPPOSES_SOURCE,
                f"plane normal . source face normal = {dot:.6f} <= 0: the offset "
                f"would push the decal into the surface",
                revision,
            )
    ordered = sorted(batch.vertices, key=lambda item: item.vert_key.value)
    index = {item.vert_key.value: number for number, item in enumerate(ordered)}
    positions, normals, refs = [], [], []
    for vertex in ordered:
        point = vertex.position
        if not _finite((point.x, point.y, point.z)):
            return _failed(OUTCOME_NON_FINITE, f"vertex {vertex.vert_key.value}", revision)
        shift = vertex_normals.get(vertex.vert_key.value, (nx, ny, nz)) if vertex_normals else (nx, ny, nz)
        if vertex_normals and vertex.vert_key.value not in vertex_normals:
            return _failed(
                OUTCOME_NORMAL_MISSING, f"the offset normal of vertex {vertex.vert_key.value} is absent", revision
            )
        positions.append((point.x, point.y, point.z))
        normals.append(tuple(shift))
        location = getattr(vertex, "semantic_location_ref", None)
        refs.append(None if location is None else location.value)
    owners = _claim_ordinals(batch)
    faces, uvs, face_owner = [], [], []
    for face in batch.faces:
        try:
            loop = tuple(index[key.value] for key in face.ordered_vert_keys)
        except KeyError as exc:
            return _failed(OUTCOME_VERTEX_MISSING, f"face {face.face_id.value}: {exc}", revision)
        pairs = tuple((fact.uv.u, fact.uv.v) for fact in face.uv_facts)
        if not _finite([item for pair in pairs for item in pair]):
            return _failed(OUTCOME_NON_FINITE, f"UV of face {face.face_id.value}", revision)
        faces.append(loop)
        uvs.extend(pairs)
        face_owner.append(owners[face.ownership_claim_id.value])
    seams = set()
    for chain in sorted(batch.interface_chains, key=lambda item: tuple(key.value for key in item.ordered_vert_keys)):
        keys = [key.value for key in chain.ordered_vert_keys]
        for first, second in zip(keys, keys[1:]):
            if first in index and second in index and first != second:
                seams.add(tuple(sorted((index[first], index[second]))))
    return DomainViewV1(
        None,
        DomainVerticesV1(result.patch_id, tuple(positions), tuple(normals), tuple(refs)),
        tuple(faces),
        tuple(uvs),
        tuple(face_owner),
        tuple(sorted(seams)),
        boundary_chain_names(batch),
        revision,
    )


__all__ = (
    "DomainViewV1",
    "OUTCOME_EMPTY_BATCH",
    "OUTCOME_NON_FINITE",
    "OUTCOME_NORMAL_MISSING",
    "OUTCOME_NORMAL_OPPOSES_SOURCE",
    "OUTCOME_VERTEX_MISSING",
    "build_domain_view",
)
