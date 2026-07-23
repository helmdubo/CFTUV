"""Physical CapEnvelope closure construction."""

from __future__ import annotations

import sympy as sp

from ..contracts.envelopes import CapEnvelopeSpec
from ..numeric import LocalLengthV1
from .common import (
    GeometryContext,
    make_segment,
    offset_point,
    source_vertex_certificate,
    stable_id,
    support_vertex_certificate,
)
from .contracts import ReferenceEnvelopeInstanceV1
from .planar_types import ExactScalar
from .provenance import ReferenceProvenanceV1, merge_provenance


def evaluate_cap_envelope(
    context: GeometryContext,
    spec: CapEnvelopeSpec,
    alpha_value: LocalLengthV1,
    effective_alpha: sp.Expr,
) -> ReferenceEnvelopeInstanceV1:
    source_segments = context.support_segments_for_use(
        spec.incident_chain_use_id, spec.incident_strip_spec_id.value
    )
    source = source_segments[0] if spec.endpoint_role.value == "START" else source_segments[-1]
    if spec.endpoint_role.value == "START":
        source_vertex_id = source.source_vertex_start_id
        anchor = source.start
    else:
        source_vertex_id = source.source_vertex_end_id
        anchor = source.end
    if source_vertex_id != spec.physical_terminal_source_vertex_id:
        raise ValueError("CapEnvelope physical endpoint does not match its incident strip")
    moved = offset_point(anchor, source.owner_normal, effective_alpha)
    support_id = stable_id("cap-support", spec.envelope_spec_id)
    instance_id = stable_id(
        "envelope-instance",
        spec.envelope_spec_id,
        ExactScalar.from_value(effective_alpha).expression,
    )
    provenance = merge_provenance(
        context.provenance_by_spec_id[spec.envelope_spec_id.value],
        ReferenceProvenanceV1(
            envelope_instance_ids=frozenset({instance_id}),
            support_ids=frozenset({support_id}),
        ),
    )
    segment = make_segment(
        stable_id("cap-edge", instance_id),
        anchor,
        moved,
        support_ids=frozenset({support_id}),
        provenance=provenance,
        start_certificates=frozenset({source_vertex_certificate(source_vertex_id)}),
        end_certificates=frozenset(
            {support_vertex_certificate(source.support_id, support_id)}
        ),
    )
    return ReferenceEnvelopeInstanceV1(
        envelope_instance_id=instance_id,
        envelope_spec_id=spec.envelope_spec_id.value,
        envelope_variant="CapEnvelope",
        requested_alpha=alpha_value,
        effective_alpha=ExactScalar.from_value(effective_alpha),
        regions=(),
        exposed_segments=(segment,),
        provenance=provenance,
    )
