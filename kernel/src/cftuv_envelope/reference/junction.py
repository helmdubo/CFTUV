"""Minimal declared T/X/Y/per-Patch JunctionEnvelope construction."""

from __future__ import annotations

from functools import cmp_to_key

import sympy as sp

from ..contracts.envelopes import JunctionEnvelopeSpec, StripEnvelopeSpec
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
from .planar_types import (
    ExactPlanarPoint,
    ExactScalar,
    compare_directions,
    cross,
    exact_sign,
    point_sub,
    points_equal,
)
from .provenance import ReferenceProvenanceV1, merge_provenance


def _convex_hull(points: tuple[ExactPlanarPoint, ...]) -> tuple[ExactPlanarPoint, ...]:
    unique = []
    for point in points:
        if not any(points_equal(point, other) for other in unique):
            unique.append(point)
    if len(unique) < 3:
        return tuple(unique)
    ordered = sorted(
        unique,
        key=cmp_to_key(
            lambda left, right: (
                exact_sign(left.x.as_expr() - right.x.as_expr())
                or exact_sign(left.y.as_expr() - right.y.as_expr())
            )
        ),
    )

    def build(sequence):
        hull = []
        for point in sequence:
            while len(hull) >= 2 and exact_sign(
                cross(point_sub(hull[-1], hull[-2]), point_sub(point, hull[-1]))
            ) <= 0:
                hull.pop()
            hull.append(point)
        return hull

    lower = build(ordered)
    upper = build(reversed(ordered))
    return tuple(lower[:-1] + upper[:-1])


def evaluate_junction_envelope(
    context: GeometryContext,
    spec: JunctionEnvelopeSpec,
    alpha_value: LocalLengthV1,
    effective_alpha: sp.Expr,
) -> ReferenceEnvelopeInstanceV1:
    relation = next(
        item
        for item in context.snapshot.junction_relations
        if item.junction_relation_id == spec.source_relation_id
    )
    anchor = context.points_by_id[relation.source_vertex_id]
    offset_records = []
    for chain_use_id in sorted(relation.incident_chain_use_ids, key=lambda item: item.value):
        if chain_use_id not in context.uses_by_id:
            continue
        chain_use = context.uses_by_id[chain_use_id]
        if chain_use.patch_domain_id != context.compilation.plan_key.patch_domain_id:
            continue
        strip = next(
            item
            for item in context.compilation.envelope_specs
            if isinstance(item, StripEnvelopeSpec)
            and next(
                seed.chain_use_id
                for seed in context.compilation.seeds
                if getattr(seed, "seed_id", None) == item.source_seed_id
            )
            == chain_use_id
        )
        source = next(
            item
            for item in context.support_segments_for_use(
                chain_use_id, strip.envelope_spec_id.value
            )
            if relation.source_vertex_id
            in (item.source_vertex_start_id, item.source_vertex_end_id)
        )
        support_id = stable_id("junction-support", spec.envelope_spec_id, chain_use_id)
        offset_records.append(
            (
                offset_point(anchor, source.owner_normal, effective_alpha),
                support_id,
            )
        )
    hull = _convex_hull((anchor, *(item[0] for item in offset_records)))
    instance_id = stable_id(
        "envelope-instance",
        spec.envelope_spec_id,
        ExactScalar.from_value(effective_alpha).expression,
    )
    all_support_ids = frozenset(item[1] for item in offset_records)
    provenance = merge_provenance(
        context.provenance_by_spec_id[spec.envelope_spec_id.value],
        ReferenceProvenanceV1(
            envelope_instance_ids=frozenset({instance_id}),
            support_ids=all_support_ids,
        ),
    )
    cert_by_point = {anchor: source_vertex_certificate(relation.source_vertex_id)}
    for point, support_id in offset_records:
        cert_by_point[point] = support_vertex_certificate(
            support_id, stable_id("junction-anchor-support", spec.envelope_spec_id)
        )
    segments = tuple(
        make_segment(
            stable_id("junction-edge", instance_id, ordinal),
            start,
            end,
            support_ids=all_support_ids,
            provenance=provenance,
            start_certificates=frozenset({cert_by_point[start]}),
            end_certificates=frozenset({cert_by_point[end]}),
        )
        for ordinal, (start, end) in enumerate(zip(hull, hull[1:] + hull[:1]))
    )
    region = make_region(
        stable_id("junction-region", instance_id),
        segments,
        instance_id=instance_id,
        spec_id=spec.envelope_spec_id.value,
    )
    return ReferenceEnvelopeInstanceV1(
        envelope_instance_id=instance_id,
        envelope_spec_id=spec.envelope_spec_id.value,
        envelope_variant="JunctionEnvelope",
        requested_alpha=alpha_value,
        effective_alpha=ExactScalar.from_value(effective_alpha),
        regions=(region,) if region is not None else (),
        exposed_segments=region.outer.segments if region is not None else (),
        provenance=provenance,
    )
