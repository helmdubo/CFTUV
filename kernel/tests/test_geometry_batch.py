from __future__ import annotations

from dataclasses import replace

from cftuv_envelope import (
    GeometryBatchCodecV1,
    SemanticBoundaryId,
    SemanticDigestValue,
    VertexKey,
    geometry_batch_semantic_digest,
    validate_geometry_batch,
)

from factories import geometry_batch


def _with_digest(batch):
    return replace(
        batch,
        semantic_digest=SemanticDigestValue(
            geometry_batch_semantic_digest(batch).sha256_hex
        ),
    )


def test_valid_internal_tessellations_share_one_semantic_digest():
    first = geometry_batch(alternate_diagonal=False)
    second = geometry_batch(alternate_diagonal=True)
    assert first.faces != second.faces
    assert geometry_batch_semantic_digest(first) == geometry_batch_semantic_digest(second)
    assert validate_geometry_batch(first) == ()
    assert validate_geometry_batch(second) == ()


def test_face_order_is_not_semantic_authority():
    batch = geometry_batch()
    reordered = replace(batch, faces=tuple(reversed(batch.faces)))
    assert geometry_batch_semantic_digest(batch) == geometry_batch_semantic_digest(reordered)


def test_exposed_boundary_change_changes_semantic_digest():
    batch = geometry_batch()
    chain = next(iter(batch.boundary_chains))
    changed_chain = replace(
        chain,
        semantic_boundary_id=SemanticBoundaryId("changed-exposed-boundary"),
        ordered_vert_keys=(VertexKey("v0"), VertexKey("v1"), VertexKey("v2"), VertexKey("v0")),
    )
    changed = replace(batch, boundary_chains=frozenset({changed_chain}))
    assert geometry_batch_semantic_digest(batch) != geometry_batch_semantic_digest(changed)


def test_geometry_batch_round_trip():
    batch = geometry_batch()
    payload = GeometryBatchCodecV1.dumps(batch)
    assert GeometryBatchCodecV1.loads(payload) == batch
    assert GeometryBatchCodecV1.dumps(GeometryBatchCodecV1.loads(payload)) == payload


def test_declared_digest_is_checked():
    batch = geometry_batch()
    invalid = replace(batch, semantic_digest=SemanticDigestValue("0" * 64))
    assert any(issue.path == ("semantic_digest",) for issue in validate_geometry_batch(invalid))
    assert validate_geometry_batch(_with_digest(invalid)) == ()

