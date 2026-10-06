"""Canonical digests separated by semantic boundary."""

from __future__ import annotations

from dataclasses import dataclass, fields, replace
from hashlib import sha256

from .codec import canonical_json_bytes
from .contracts.analysis import AnalysisSnapshotV1
from .contracts.geometry_batch import GeometryBatchV1
from .contracts.plan import CompiledPatchEvaluationPlanV1
from .ids import SemanticDigestValue


@dataclass(frozen=True, slots=True)
class SnapshotDigest:
    sha256_hex: str


@dataclass(frozen=True, slots=True)
class SemanticPlanDigest:
    sha256_hex: str


@dataclass(frozen=True, slots=True)
class GeometryBatchSemanticDigest:
    sha256_hex: str


def _digest(payload: bytes) -> str:
    return sha256(payload).hexdigest()


def snapshot_digest(snapshot: AnalysisSnapshotV1) -> SnapshotDigest:
    return SnapshotDigest(_digest(canonical_json_bytes(snapshot)))


def semantic_plan_digest(plan: CompiledPatchEvaluationPlanV1) -> SemanticPlanDigest:
    return SemanticPlanDigest(_digest(canonical_json_bytes(plan)))


#: Последний подсчитанный дайджест: `(части батча по тождеству, дайджест)`. Материализатор считает дайджест
#: при сборке, а валидатор пересчитывает его при проверке, и оба зовут эту функцию с батчами, у которых одни и те
#: же кортежи (`replace` меняет только поле дайджеста). Дайджест — чистая функция проекции, а части батча —
#: замороженные записи и кортежи, поэтому второй вызов с ТЕМИ ЖЕ объектами возвращает прежнее значение. Запись
#: держит сами объекты, и занятое тождество не может достаться другому: подмена хоть одной части (новый
#: кортеж, пусть и равный) даёт промах и честный пересчёт. Память одна запись: хвост воркера не копится.
_LAST_DIGEST: tuple[tuple, GeometryBatchSemanticDigest] | None = None


#: Ключ памяти — ВСЕ поля батча, кроме самого дайджеста (его-то и считают). Список выводится из записи, а не
#: пишется руками: новое поле батча попадает в ключ само, и устаревший ключ не даст чужого попадания.
_KEY_FIELDS = tuple(
    item.name for item in fields(GeometryBatchV1) if item.name != "semantic_digest"
)


def _projection_parts(batch: GeometryBatchV1) -> tuple:
    return tuple(getattr(batch, name) for name in _KEY_FIELDS)


def geometry_batch_semantic_digest(batch: GeometryBatchV1) -> GeometryBatchSemanticDigest:
    global _LAST_DIGEST
    parts = _projection_parts(batch)
    last = _LAST_DIGEST
    if last is not None and all(
        old is new for old, new in zip(last[0], parts)
    ):
        return last[1]
    digest = _compute_semantic_digest(batch)
    _LAST_DIGEST = (parts, digest)
    return digest


#: Значение `semantic_digest` батча, которому дайджест ещё не посчитан (сборка батча и материализация без дайджестов).
PENDING_SEMANTIC_DIGEST = "pending"


def sealed_geometry_batch(batch: GeometryBatchV1) -> GeometryBatchV1:
    """Тот же батч с настоящим `semantic_digest` (проекция та же, что считает `geometry_batch_semantic_digest`).

    Батч, собранный без дайджеста (`materialize_domain(..., digests=False)`), несёт `PENDING_SEMANTIC_DIGEST`; запечатывает его
    тот, кому дайджест нужен. Запечатанный батч побитово равен батчу eager-сборки: дайджест — чистая функция проекции.
    """

    return replace(
        batch,
        semantic_digest=SemanticDigestValue(geometry_batch_semantic_digest(batch).sha256_hex),
    )


def _compute_semantic_digest(batch: GeometryBatchV1) -> GeometryBatchSemanticDigest:
    boundary_vertex_keys = {
        key
        for chain in batch.boundary_chains
        for key in chain.ordered_vert_keys
    }
    interface_vertex_keys = {
        key
        for chain in batch.interface_chains
        for key in chain.ordered_vert_keys
    }
    semantic_vertex_keys = boundary_vertex_keys | interface_vertex_keys
    semantic_vertices = frozenset(
        vertex for vertex in batch.vertices if vertex.vert_key in semantic_vertex_keys
    )
    semantic_uv_projection = frozenset(
        (
            face.semantic_region_id,
            face.ownership_claim_id,
            fact.vert_key,
            fact.uv,
        )
        for face in batch.faces
        for fact in face.uv_facts
        if fact.vert_key in semantic_vertex_keys
    )
    semantic_station_projection = frozenset(
        fact for fact in batch.station_facts if fact.vert_key in semantic_vertex_keys
    )
    projection = {
        "schema_version": batch.schema_version,
        "source_revision": batch.source_revision,
        "decal_request_id": batch.decal_request_id,
        "patch_domain_id": batch.patch_domain_id,
        "semantic_regions": batch.semantic_regions,
        "boundary_chains": batch.boundary_chains,
        "interface_chains": batch.interface_chains,
        "semantic_vertices": semantic_vertices,
        "semantic_uv_projection": semantic_uv_projection,
        "semantic_station_projection": semantic_station_projection,
        "contract_versions": batch.contract_versions,
        "diagnostics": batch.diagnostics,
    }
    return GeometryBatchSemanticDigest(_digest(canonical_json_bytes(projection)))
