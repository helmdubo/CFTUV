from __future__ import annotations

import sympy as sp

from cftuv_envelope.reference.arrangement import ExactSegmentArrangementBackend
from cftuv_envelope.reference.common import make_region, make_segment
from cftuv_envelope.reference.contracts import ReachabilityCertificateV1
from cftuv_envelope.reference.planar_types import (
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
)
from cftuv_envelope.reference.provenance import make_reference_provenance


def _region(name: str, coordinates, *, spec_id: str, instance_id: str):
    points = tuple(ExactPlanarPoint.from_values(*item) for item in coordinates)
    certificates = tuple(
        ConstructionCertificate(
            ConstructionKind.SOURCE_VERTEX,
            source_vertex_ids=frozenset({f"vertex:{name}:{index}"}),
        )
        for index in range(len(points))
    )
    provenance = make_reference_provenance(
        envelope_spec_ids=frozenset({spec_id}) if spec_id else frozenset(),
        envelope_instance_ids=frozenset({instance_id}) if instance_id else frozenset(),
        support_ids=frozenset({f"support:{name}"}),
        physical_edge_ids=frozenset({f"edge:{name}"}),
        chain_use_ids=frozenset({f"use:{name}"}),
        patch_domain_ids=frozenset({"domain"}),
    )
    segments = tuple(
        make_segment(
            f"segment:{name}:{index}",
            points[index],
            points[(index + 1) % len(points)],
            support_ids=provenance.support_ids,
            provenance=provenance,
            start_certificates=frozenset({certificates[index]}),
            end_certificates=frozenset({certificates[(index + 1) % len(points)]}),
        )
        for index in range(len(points))
    )
    return make_region(
        f"region:{name}",
        segments,
        spec_id=spec_id,
        instance_id=instance_id,
    )


def test_internalized_angular_profile_keeps_region_claim_without_silhouette_recovery():
    covering = _region(
        "covering-strip",
        ((0, 0), (10, 0), (10, 10), (0, 10)),
        spec_id="strip-spec",
        instance_id="strip-instance",
    )
    angular = _region(
        "angular",
        ((2, 2), (8, 2), (5, 6)),
        spec_id="angular-spec",
        instance_id="angular-instance",
    )
    domain = _region(
        "domain",
        ((-1, -1), (11, -1), (11, 11), (-1, 11)),
        spec_id="",
        instance_id="",
    )
    reachability = {
        instance_id: ReachabilityCertificateV1(
            frozenset({f"front:{instance_id}"}), True, 1, 1, (), False
        )
        for instance_id in ("strip-instance", "angular-instance")
    }

    result = ExactSegmentArrangementBackend().exact_union(
        (covering, angular), (domain,), reachability
    )

    assert sp.sympify(result.exact_area_expression) == 100
    assert len(result.regions) == 1
    assert next(iter(result.regions)).contributor_envelope_spec_ids == frozenset(
        {"strip-spec", "angular-spec"}
    )
    region = next(iter(result.regions))
    assert (
        region.provenance.coverage_contributors.envelope_spec_ids
        == frozenset({"strip-spec", "angular-spec"})
    )
    assert all(
        "angular-spec"
        not in edge.provenance.coverage_contributors.envelope_spec_ids
        for edge in result.edges
    )
    assert all(
        "support:angular"
        not in edge.provenance.boundary_generator.support_ids
        for edge in result.edges
    )
