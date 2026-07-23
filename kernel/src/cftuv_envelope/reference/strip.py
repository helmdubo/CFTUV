"""Analytic StripEnvelope instance construction."""

from __future__ import annotations

import sympy as sp

from ..contracts.envelopes import CapEnvelopeSpec, StripEnvelopeSpec
from ..ids import ChainUseId
from ..numeric import LocalLengthV1
from .common import (
    GeometryContext,
    make_region,
    make_segment,
    offset_point,
    source_vertex_certificate,
    stable_id,
    support_vertex_certificate,
)
from .contracts import ReferenceEnvelopeInstanceV1
from .planar_types import ExactScalar
from .provenance import merge_provenance


def evaluate_strip_envelope(
    context: GeometryContext,
    spec: StripEnvelopeSpec,
    alpha_value: LocalLengthV1,
    effective_alpha: sp.Expr,
) -> ReferenceEnvelopeInstanceV1:
    front_seed = next(
        item
        for item in context.compilation.seeds
        if getattr(item, "seed_id", None) == spec.source_seed_id
    )
    chain_use_id: ChainUseId = front_seed.chain_use_id
    instance_id = stable_id(
        "envelope-instance",
        spec.envelope_spec_id,
        ExactScalar.from_value(effective_alpha).expression,
    )
    cap_by_role = {
        item.endpoint_role.value: item
        for item in context.compilation.envelope_specs
        if isinstance(item, CapEnvelopeSpec)
        and item.incident_strip_spec_id == spec.envelope_spec_id
    }
    regions = []
    exposed = []
    source_segments = context.support_segments_for_use(
        chain_use_id, spec.envelope_spec_id.value
    )
    for ordinal, source in enumerate(source_segments):
        instance_provenance = merge_provenance(
            source.provenance,
            type(source.provenance)(
                envelope_instance_ids=frozenset({instance_id})
            ),
        )
        moved_start = offset_point(source.start, source.owner_normal, effective_alpha)
        moved_end = offset_point(source.end, source.owner_normal, effective_alpha)
        moved_support_id = stable_id("moving-support", source.support_id)
        start_terminal_id = stable_id("terminal-support", source.support_id, "start")
        end_terminal_id = stable_id("terminal-support", source.support_id, "end")
        source_start_cert = source_vertex_certificate(source.source_vertex_start_id)
        source_end_cert = source_vertex_certificate(source.source_vertex_end_id)
        moved_start_cert = support_vertex_certificate(
            moved_support_id, start_terminal_id
        )
        moved_end_cert = support_vertex_certificate(moved_support_id, end_terminal_id)
        start_provenance = instance_provenance
        end_provenance = instance_provenance
        if ordinal == 0 and "START" in cap_by_role:
            start_provenance = merge_provenance(
                start_provenance,
                context.provenance_by_spec_id[
                    cap_by_role["START"].envelope_spec_id.value
                ],
            )
        if ordinal == len(source_segments) - 1 and "END" in cap_by_role:
            end_provenance = merge_provenance(
                end_provenance,
                context.provenance_by_spec_id[
                    cap_by_role["END"].envelope_spec_id.value
                ],
            )
        segments = (
            make_segment(
                stable_id("strip-edge", instance_id, ordinal, "source"),
                source.start,
                source.end,
                support_ids=frozenset({source.support_id}),
                provenance=instance_provenance,
                start_certificates=frozenset({source_start_cert}),
                end_certificates=frozenset({source_end_cert}),
            ),
            make_segment(
                stable_id("strip-edge", instance_id, ordinal, "end"),
                source.end,
                moved_end,
                support_ids=frozenset({end_terminal_id}),
                provenance=end_provenance,
                start_certificates=frozenset({source_end_cert}),
                end_certificates=frozenset({moved_end_cert}),
            ),
            make_segment(
                stable_id("strip-edge", instance_id, ordinal, "front"),
                moved_end,
                moved_start,
                support_ids=frozenset({moved_support_id}),
                provenance=merge_provenance(
                    instance_provenance,
                    type(source.provenance)(
                        envelope_instance_ids=frozenset({instance_id}),
                        support_ids=frozenset({moved_support_id}),
                    ),
                ),
                start_certificates=frozenset({moved_end_cert}),
                end_certificates=frozenset({moved_start_cert}),
            ),
            make_segment(
                stable_id("strip-edge", instance_id, ordinal, "start"),
                moved_start,
                source.start,
                support_ids=frozenset({start_terminal_id}),
                provenance=start_provenance,
                start_certificates=frozenset({moved_start_cert}),
                end_certificates=frozenset({source_start_cert}),
            ),
        )
        region = make_region(
            stable_id("strip-region", instance_id, ordinal),
            segments,
            instance_id=instance_id,
            spec_id=spec.envelope_spec_id.value,
        )
        if region is not None:
            regions.append(region)
            exposed.extend(region.outer.segments)
    provenance = merge_provenance(
        context.provenance_by_spec_id[spec.envelope_spec_id.value],
        type(context.provenance_by_spec_id[spec.envelope_spec_id.value])(
            envelope_instance_ids=frozenset({instance_id})
        ),
    )
    return ReferenceEnvelopeInstanceV1(
        envelope_instance_id=instance_id,
        envelope_spec_id=spec.envelope_spec_id.value,
        envelope_variant="StripEnvelope",
        requested_alpha=alpha_value,
        effective_alpha=ExactScalar.from_value(effective_alpha),
        regions=tuple(regions),
        exposed_segments=tuple(exposed),
        provenance=provenance,
    )
