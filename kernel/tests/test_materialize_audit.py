"""Аудит сетки ловит то, чего валидатор батча не видит: трещину, наложение, разворот."""

from __future__ import annotations

import dataclasses

from cftuv_envelope.materialize.audit import audit_batch
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.numeric import UvPoint2V1
from cftuv_envelope.contracts.geometry_batch import GeometryUvFactV1

import materialize_factories as factories

NORMAL = (0.0, 0.0, 1.0)


def _batch():
    prepared, coverage, request = factories.straight_chain_domain()
    request = dataclasses.replace(request, uv_policy_id=PolicyId("UV_DIRECT_STRIP_V1"))
    return materialize_domain(prepared, coverage, request=request).batch


def test_a_sound_batch_has_no_problems_and_a_closed_boundary():
    audit = audit_batch(_batch(), NORMAL)
    assert audit.problems() == ()
    assert audit.boundary_edges == 8 and audit.boundary_chain_mismatch == 0
    assert audit.flipped_vs_source == 0 and audit.uv_reversed == 0


def test_a_missing_triangle_is_a_crack_that_the_chains_do_not_explain():
    batch = _batch()
    cracked = dataclasses.replace(batch, faces=batch.faces[1:])
    audit = audit_batch(cracked, NORMAL)
    assert "BOUNDARY_DOES_NOT_MATCH_CHAINS" in audit.problems()


def test_a_duplicated_triangle_is_an_overlap():
    batch = _batch()
    doubled = dataclasses.replace(batch, faces=batch.faces + batch.faces[:1])
    problems = audit_batch(doubled, NORMAL).problems()
    assert "HALF_EDGE_DUPLICATED" in problems


def test_a_reversed_triangle_is_caught_twice_by_direction_and_by_normal():
    batch = _batch()
    face = batch.faces[0]
    reversed_face = dataclasses.replace(
        face,
        ordered_vert_keys=face.ordered_vert_keys[::-1],
        uv_facts=face.uv_facts[::-1],
    )
    audit = audit_batch(
        dataclasses.replace(batch, faces=(reversed_face,) + batch.faces[1:]), NORMAL
    )
    assert audit.flipped_vs_source == 1
    assert "HALF_EDGE_DUPLICATED" in audit.problems()
    assert audit.uv_reversed >= 1


def test_a_v_outside_the_unit_range_breaks_the_law():
    batch = _batch()
    face = batch.faces[0]
    first = face.uv_facts[0]
    broken = dataclasses.replace(
        face,
        uv_facts=(
            GeometryUvFactV1(first.vert_key, UvPoint2V1(first.uv.u, 1.5)),
        )
        + face.uv_facts[1:],
    )
    audit = audit_batch(
        dataclasses.replace(batch, faces=(broken,) + batch.faces[1:]), NORMAL
    )
    assert "V_OUT_OF_UNIT_RANGE" in audit.problems()
