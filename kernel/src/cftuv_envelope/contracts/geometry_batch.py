"""Immutable output boundary shared by future GPU and BMesh adapters."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import (
    ChainUseId,
    ContractVersionId,
    DecalRequestId,
    GeometryDiagnosticId,
    GeometryFaceId,
    GeometryStationFactId,
    LineageId,
    MaterialId,
    OwnershipClaimId,
    PatchDomainId,
    PhysicalEdgeId,
    SemanticBoundaryId,
    SemanticDigestValue,
    SemanticInterfaceId,
    SemanticLocationId,
    SemanticRegionId,
    SourceFaceId,
    SourceRevision,
    VertexKey,
)
from ..numeric import LocalCoordinateV1, LocalPoint3V1, UvPoint2V1
from ..outcomes import NamedOutcome
from .envelopes import StationModelId


GEOMETRY_BATCH_SCHEMA_V1 = "cftuv.envelope.geometry_batch.v1"


class GeometryDiagnosticSeverity(str, Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


class DecalTopologyLawV1(str, Enum):
    """Из каких граней материализатор собирает сетку декали.

    Ось закона топологии: один и тот же слитый контур допускает разные ЗАКРЫТЫЕ
    разбиения на грани, и все они делят один семантический дайджест (границу
    разбиение не меняет, новых вершин оно не вводит).

    * `TRIANGLES_V1` — только треугольники: каждый слитый контур режется
      отсечением ушей (прежний и единственный до этого закона ответ).
    * `QUAD_STRIPS_V1` — СТРОГО выпуклый четырёхугольник ленты остаётся одной
      четырёхгранью; веера, невыпуклые четырёхугольники и слитые пробеги
      (больше четырёх вершин) остаются треугольниками, и каждый такой случай
      назван счётчиком. Каноническое разбиение четырёхугольника на два
      треугольника — ровно первое ухо прежней тесселяции
      (`materialize.tessellate.split_quad`), поэтому развёртка граней закона в
      треугольники даёт прежнюю сетку.
    * `PLANAR_POLYGONS_V1` — надмножество `QUAD_STRIPS_V1` для плоских укладок. Слитый
      пробег одной цепи режется по перекладинам (бывшим перпендикулярным разделителям)
      обратно в грани своих рёбер-источников: разделитель — внутреннее ребро одного
      региона, а не шов UV (кадр и станции у частей одни). Грань ленты на ТОЧНОЙ
      плоскости остаётся ОДНИМ многоугольником любой длины, выпуклым или нет
      (допуск `PLANAR_AFFINE_UV_POLYGON_V1`), если контур прост (знаки точные) и UV в
      нём — аффинная функция положения на карте по всему контуру (проверка точная):
      тогда любая триангуляция, которую Blender выберет для показа, даёт ту же
      поверхность и ту же UV-интерполяцию, безымянной геометрии нет. Вершины на прямой
      допускаются — они несут T-стыки соседей. Не простой контур либо неаффинный UV —
      отсечение ушей под своим счётчиком. На укладке на треугольники источника
      многоугольник длиннее четырёх не
      излучается (он не плоский): четырёхгранья идут по закону
      `QUAD_IN_ONE_SOURCE_TRIANGLE_V1`, остальное — треугольники под своим счётчиком.
      Целый веер — треугольник на сектор (секторы одного угла не сливаются: у каждого
      своя опора). Грань веера, срезанная локусом соседа (контур длиннее трёх точек), на
      ТОЧНОЙ плоскости — тоже ОДНА грань при тех же условиях: контур прост, а UV аффинна
      (в веере `s` постоянна, `r` — время прихода к опорной прямой сектора — линейно по
      положению на карте; проверка всё равно точная), выпуклая или нет. Иначе (кривая
      укладка, непростой контур, неаффинная UV) она режется ОТ вершины веера
      (`FAN_FACE_TRIANGULATED_FROM_APEX_V1`: треугольники от вершины, каждый строго
      положителен), а если вершина не видна — отсечением ушей под счётчиком
      `MATERIALIZE_FAN_FACES_NOT_STAR_FROM_APEX`. Вершины, UV, цепи и семантический дайджест
      те же, что у двух других законов; сумма `n - 2` по граням — то же число.

    Закон — политика ХОСТА, как закон укладки: ядро не выбирает его за хост и
    записывает в исход материализации (`MaterializationV1.decal_topology_law`),
    а не в `contract_versions` и не в диагностики батча: те входят в
    семантический дайджест, а он тесселяции не видит.
    """

    TRIANGLES_V1 = "TRIANGLES_V1"
    QUAD_STRIPS_V1 = "QUAD_STRIPS_V1"
    PLANAR_POLYGONS_V1 = "PLANAR_POLYGONS_V1"


@dataclass(frozen=True, slots=True)
class GeometryProvenanceV1:
    source_face_ids: frozenset[SourceFaceId]
    physical_edge_ids: frozenset[PhysicalEdgeId]
    chain_use_ids: frozenset[ChainUseId]
    lineage_ids: frozenset[LineageId]


@dataclass(frozen=True, slots=True)
class GeometryVertexV1:
    vert_key: VertexKey
    position: LocalPoint3V1
    semantic_location_ref: SemanticLocationId
    provenance: GeometryProvenanceV1


@dataclass(frozen=True, slots=True)
class GeometryUvFactV1:
    vert_key: VertexKey
    uv: UvPoint2V1


@dataclass(frozen=True, slots=True)
class GeometryStationFactV1:
    station_fact_id: GeometryStationFactId
    vert_key: VertexKey
    semantic_region_id: SemanticRegionId
    ownership_claim_id: OwnershipClaimId
    source_s: LocalCoordinateV1
    source_r: LocalCoordinateV1
    station_model_id: StationModelId
    lineage_ids: frozenset[LineageId]


@dataclass(frozen=True, slots=True)
class GeometryFaceV1:
    face_id: GeometryFaceId
    ordered_vert_keys: tuple[VertexKey, ...]
    uv_facts: tuple[GeometryUvFactV1, ...]
    semantic_region_id: SemanticRegionId
    ownership_claim_id: OwnershipClaimId
    provenance: GeometryProvenanceV1
    material_id: MaterialId

    def __post_init__(self) -> None:
        if len(self.ordered_vert_keys) < 3:
            raise ValueError("GeometryFaceV1 requires at least three vertices")
        if tuple(fact.vert_key for fact in self.uv_facts) != self.ordered_vert_keys:
            raise ValueError("GeometryFaceV1 UV facts must follow the ordered vertex cycle")


@dataclass(frozen=True, slots=True)
class GeometrySemanticRegionV1:
    semantic_region_id: SemanticRegionId
    ownership_claim_id: OwnershipClaimId
    material_id: MaterialId
    provenance: GeometryProvenanceV1


@dataclass(frozen=True, slots=True)
class GeometryBoundaryChainV1:
    semantic_boundary_id: SemanticBoundaryId
    ordered_vert_keys: tuple[VertexKey, ...]


@dataclass(frozen=True, slots=True)
class GeometryInterfaceChainV1:
    semantic_interface_id: SemanticInterfaceId
    ordered_vert_keys: tuple[VertexKey, ...]


@dataclass(frozen=True, slots=True)
class GeometryDiagnosticV1:
    diagnostic_id: GeometryDiagnosticId
    severity: GeometryDiagnosticSeverity
    outcome: NamedOutcome
    provenance: frozenset[LineageId]


@dataclass(frozen=True, slots=True)
class GeometryBatchV1:
    schema_version: str
    source_revision: SourceRevision
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    vertices: frozenset[GeometryVertexV1]
    faces: tuple[GeometryFaceV1, ...]
    station_facts: frozenset[GeometryStationFactV1]
    semantic_regions: frozenset[GeometrySemanticRegionV1]
    boundary_chains: frozenset[GeometryBoundaryChainV1]
    interface_chains: frozenset[GeometryInterfaceChainV1]
    diagnostics: frozenset[GeometryDiagnosticV1]
    contract_versions: frozenset[ContractVersionId]
    semantic_digest: SemanticDigestValue
