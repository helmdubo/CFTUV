from __future__ import annotations

import hashlib
import json
from functools import lru_cache
from pathlib import Path

import cftuv_envelope as kernel


KERNEL_ROOT = Path(__file__).parents[1]
SEM_CLB_ROOT = (
    KERNEL_ROOT / "fixtures" / "sem_clb_02_lost_domains_v1"
)
CASE_ROOT = SEM_CLB_ROOT / "cases"
RECEIPT_PATH = SEM_CLB_ROOT / "gate_a_triple_prime_receipt.json"
RECEIPT_SHA256 = (
    "bf1864fe80ec66ecc926ec02b82a6204a25e3b7f63032adff3fb769227ebb3b4"
)
EXPECTED_R_MIN = {
    "building_all_seams_patch_001_lost_resolved_v1": 2,
    "building_all_seams_patch_006_lost_resolved_v1": 3,
    "building_all_seams_patch_011_lost_resolved_v1": 2,
    "building_all_seams_patch_105_lost_resolved_v1": 1,
}
EXPECTED_K_SEQUENCES = {
    "building_all_seams_patch_001_lost_resolved_v1": (
        (0, 3, 8),
        (0, 1, 3, 4),
        (0, 5, 8),
    ),
    "building_all_seams_patch_006_lost_resolved_v1": (
        (0, 1, 3, 5, 7, 8),
        (0, 271129, 524288),
    ),
    "building_all_seams_patch_011_lost_resolved_v1": (
        (0, 5, 8),
        (0, 1, 3, 4),
    ),
    "building_all_seams_patch_105_lost_resolved_v1": (
        (0, 1, 2),
    ),
}


def _receipt():
    return json.loads(RECEIPT_PATH.read_text(encoding="utf-8"))


@lru_cache(maxsize=None)
def _load_case(name: str):
    root = CASE_ROOT / name
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    (domain,) = snapshot.patch_domains
    result = kernel.compile_reference_envelopes(
        snapshot,
        request,
        domain.patch_domain_id,
    )
    assert result.outcome is kernel.ReferenceOutcome.EXACT
    assert result.compilation is not None
    return snapshot, request, domain, result.compilation


def _ratio_text(value) -> str:
    if value.denominator == 1:
        return str(value.numerator)
    return f"{value.numerator}/{value.denominator}"


def _point_text(value) -> list[str]:
    return [_ratio_text(value.x), _ratio_text(value.y)]


def _node_sequence(chain) -> tuple[int, ...]:
    return (
        0,
        *(item.selected_k for item in chain.internal_assignments),
        chain.refined_endpoint_span_k,
    )


def test_accepted_a3_receipt_bytes_and_coverage_are_frozen():
    payload = RECEIPT_PATH.read_bytes()
    receipt = _receipt()
    aggregate = receipt["aggregate"]

    assert hashlib.sha256(payload).hexdigest() == RECEIPT_SHA256
    assert aggregate["domain_r_min"] == EXPECTED_R_MIN
    assert aggregate["fixture_count"] == 4
    assert aggregate["declared_straight_chain_count"] == 8
    assert aggregate["internal_vertex_count"] == 13
    assert aggregate["non_chain_vertex_count"] == 83
    assert aggregate["clamped_internal_steps_at_r_min"] == 1
    assert aggregate["unclamped_internal_steps_at_r_min"] == 12
    assert receipt["failures"] == []


def test_product_v2_matches_every_selected_a3_exact_assignment_row():
    receipt_by_case = {
        item["fixture_id"]: item for item in _receipt()["fixtures"]
    }
    internal_row_count = 0
    non_chain_count = 0
    observed_sequences = {}

    for case_name, expected_r in EXPECTED_R_MIN.items():
        _, _, _, compilation = _load_case(case_name)
        binding = compilation.evaluation_geometry_binding
        assert type(binding) is (
            kernel.ChainStraightEvaluationGeometryBindingV2
        )
        fixture = receipt_by_case[case_name]
        selected_attempt = next(
            item
            for item in fixture["attempts"]
            if item["r"] == fixture["r_min"]
        )
        expected_chains = {
            item["physical_chain_id"]: item
            for item in selected_attempt["chain_attempts"]
        }
        product_chains = {
            item.physical_chain_id.value: item
            for item in binding.straight_chain_bindings
        }
        authorities = {
            item.source_vertex_id.value: item
            for item in binding.vertex_authorities
        }

        assert binding.refinement_power == expected_r
        assert binding.base_lattice_scale == fixture["base_scale_S"]
        assert binding.lattice_scale == selected_attempt[
            "refined_scale_S_prime"
        ]
        assert product_chains.keys() == expected_chains.keys()
        observed_sequences[case_name] = tuple(
            _node_sequence(product_chains[chain_id])
            for chain_id in sorted(product_chains)
        )

        for chain_id, expected in expected_chains.items():
            chain = product_chains[chain_id]
            assignment = expected["canonical_gram_assignment"]
            assert [item.value for item in chain.ordered_source_vertex_ids] == (
                expected["ordered_source_vertex_ids"]
            )
            assert list(chain.primitive_direction) == expected[
                "primitive_direction"
            ]
            assert chain.base_endpoint_span_k == expected[
                "base_endpoint_span_K"
            ]
            assert chain.refined_endpoint_span_k == expected["K_prime"]
            assert list(_node_sequence(chain)) == assignment["k_sequence"]
            assert len(chain.internal_assignments) == len(
                assignment["steps"]
            )

            for product_row, expected_row in zip(
                chain.internal_assignments,
                assignment["steps"],
                strict=True,
            ):
                internal_row_count += 1
                authority = authorities[
                    product_row.source_vertex_id.value
                ]
                assert product_row.ordinal == expected_row["ordinal"]
                assert product_row.source_vertex_id.value == expected_row[
                    "source_vertex_id"
                ]
                assert product_row.lower_k == expected_row["lower"]
                assert product_row.upper_k == expected_row["upper"]
                assert _ratio_text(product_row.projection_k_gram) == (
                    expected_row["projection_k_gram"]
                )
                assert product_row.unconstrained_canonical_k == (
                    expected_row["unconstrained_canonical_k"]
                )
                assert product_row.selected_k == expected_row["selected_k"]
                assert product_row.clamped == expected_row["CLAMPED"]
                assert product_row.disposition.value == expected_row[
                    "boundary_disposition"
                ]
                assert list(product_row.assigned_refined_node) == (
                    expected_row["assigned_refined_node"]
                )
                assert _point_text(
                    product_row.exact_offset_from_source
                ) == expected_row["exact_displacement"]
                assert _ratio_text(
                    product_row.exact_gram_displacement_squared
                ) == expected_row["exact_gram_displacement_squared"]
                assert _ratio_text(
                    product_row.half_step_gram_squared_bound
                ) == expected_row[
                    "half_S_prime_step_gram_squared_bound"
                ]
                assert _ratio_text(
                    product_row.exact_longitudinal_displacement_k
                ) == expected_row[
                    "exact_longitudinal_displacement_k"
                ]
                assert _ratio_text(
                    product_row.longitudinal_half_step_bound_k
                ) == "1/2"
                assert _point_text(
                    authority.assigned_domain_coordinate
                ) == expected_row["assigned_coordinate"]
                assert _point_text(
                    authority.source_domain_coordinate
                ) == expected_row["source_coordinate"]

        expected_deficits = {
            item["physical_chain_id"]: (
                expected_r - 1,
                item["base_endpoint_span_K"] * (1 << (expected_r - 1)),
                item["vertex_count"] - 1,
            )
            for item in expected_chains.values()
            if item["base_endpoint_span_K"] * (1 << (expected_r - 1))
            < item["vertex_count"] - 1
        }
        assert {
            item.physical_chain_id.value: (
                item.previous_refinement_power,
                item.previous_endpoint_span_k,
                item.capacity_required,
            )
            for item in binding.previous_refinement_capacity_deficits
        } == expected_deficits
        assert all(
            item.exact_deficit
            == item.capacity_required - item.previous_endpoint_span_k
            for item in binding.previous_refinement_capacity_deficits
        )

        product_non_chain = tuple(
            item
            for item in binding.vertex_authorities
            if item.authority
            is kernel.ChainStraightVertexAuthorityV2.BASE_BOUND_NON_CHAIN_V1
        )
        non_chain_count += len(product_non_chain)
        assert len(product_non_chain) == fixture["non_chain_vertex_count"]
        assert all(
            item.assigned_domain_coordinate == item.base_bound_coordinate
            and not item.physical_chain_ids
            for item in product_non_chain
        )

    assert observed_sequences == EXPECTED_K_SEQUENCES
    assert internal_row_count == 13
    assert non_chain_count == 83


def test_patch105_has_the_sole_exact_named_clamp():
    all_clamps = []
    for case_name in EXPECTED_R_MIN:
        _, _, _, compilation = _load_case(case_name)
        binding = compilation.evaluation_geometry_binding
        all_clamps.extend(
            item
            for chain in binding.straight_chain_bindings
            for item in chain.internal_assignments
            if item.clamped
        )

    (clamp,) = all_clamps
    assert clamp.source_vertex_id.value.endswith(":building:189")
    assert _ratio_text(clamp.projection_k_gram) == (
        "578633553271514/339894311513453"
    )
    assert clamp.unconstrained_canonical_k == 2
    assert (clamp.lower_k, clamp.upper_k, clamp.selected_k) == (1, 1, 1)
    assert clamp.disposition is (
        kernel.ChainStraightAssignmentDispositionV2.CLAMPED_CONSTRAINT_EXCESS_ALLOWED
    )
    assert _point_text(clamp.exact_offset_from_source) == [
        "-27745503/451280896",
        "-130742673/451280896",
    ]
    assert _ratio_text(clamp.exact_gram_displacement_squared) == (
        "8887499590674321/288230376151711744"
    )
    assert _ratio_text(clamp.half_step_gram_squared_bound) == (
        "18014398510213009/1152921504606846976"
    )
    assert _ratio_text(clamp.exact_longitudinal_displacement_k) == (
        "238739241758061/339894311513453"
    )


def test_point_contact_v1_binding_and_raw_topology_anchor_are_unchanged():
    root = KERNEL_ROOT / "fixtures" / "building_002_point_contact_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    (domain,) = snapshot.patch_domains
    result = kernel.compile_reference_envelopes(
        snapshot,
        request,
        domain.patch_domain_id,
    )
    assert result.outcome is kernel.ReferenceOutcome.EXACT
    compilation = result.compilation
    binding = compilation.evaluation_geometry_binding

    assert type(binding) is kernel.EvaluationGeometryBindingV1
    assert hashlib.sha256(
        kernel.EvaluationGeometryBindingCodecV1.dumps(binding)
    ).hexdigest() == (
        "fe440be52dd6a4f386126a707a7528d749afd5921534abef9574b073fd1cf2d7"
    )
    evaluated = kernel.evaluate_reference_raw_coverage(
        compilation,
        request.requested_alpha,
    )
    assert evaluated.outcome is kernel.ReferenceOutcome.EXACT
    raw = evaluated.raw_coverage
    assert raw.semantic_digest == (
        "402ec97a93567aad5890786f75079b2114b1ba397819904cdb665049b31715eb"
    )
    assert (
        len(raw.point_contacts),
        len(raw.loops),
        len(raw.regions),
    ) == (0, 1, 1)
