#!/usr/bin/env python3
"""Воспроизводимая валидация текстового/JSON-корпуса EC0-P (AM11)."""

from __future__ import annotations

import json
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
CORPUS = REPO / "artifacts" / "envelope_ec0" / "corpus"
CASE_DIR = CORPUS / "cases"
SPEC_POLICY_PATH = CORPUS / "policies" / "envelope_spec_contracts.json"

CANONICAL_AUTHORITY = {
    "USER_REQUIRED",
    "FIELD_PROVEN",
    "MATHEMATICALLY_REQUIRED",
}
REVIEW_ONLY_AUTHORITY = {
    "LEGACY_COMPATIBILITY",
    "IMPLEMENTATION_ACCIDENT",
    "OPEN_RESEARCH",
}
AM3_REQUIRED_TRANSFORMS = {
    "chain_storage_reverse_preserve_semantics",
    "surface_winding_reverse_preserve_domain",
    "rigid_translation",
    "coherent_uniform_scale",
    "source_retriangulation",
    "data_chain_split_merge",
    "simultaneous_event_permutation",
    "small_perturbation_without_topology_change",
}
AM11_MAIN_TRANSFORMS = {
    "angular_profile_selection_policy_switch",
    "hidden_support_record_permutation",
    "hidden_edge_count_change",
    "angular_subdivision_policy_change",
    "downstream_tessellation_change_preserve_arrangement",
    "profile_controlled_boundary_dissolve",
    "oriented_angular_sector_record_permutation",
    "incident_support_order_swap_without_orientation_update",
    "reflex_angle_change_crossing_k_selection_boundary",
    "max_subturn_parameter_change_crossing_k_selection_boundary",
    "manual_resolved_k_override",
}
AM11_PIVOT_TRANSFORMS = AM11_MAIN_TRANSFORMS - {"profile_controlled_boundary_dissolve"}
AM11_BOUNDARY_TRANSFORMS = {"downstream_tessellation_change_preserve_arrangement"}

ENVELOPE_VARIANTS = {
    "StripEnvelopeSpec",
    "AngularEnvelopeSpec",
    "JunctionEnvelopeSpec",
    "CapEnvelopeSpec",
}
ANGULAR_PROFILE_ID = "LINEAR_REFLEX_EQUAL_V1"
ANGULAR_SELECTION_POLICY_ID = "MIN_K_FOR_MAX_SUBTURN_V1"
ANGULAR_PARAMETER_ID = "LINEAR_REFLEX_MAX_SUBTURN_V1"
ANGULAR_SECTOR_CONTRACT = "ORIENTED_OWNER_ANGULAR_SECTOR_V1"
HIDDEN_SUPPORT_GENERATION_LAW = "LINEAR_REFLEX_HIDDEN_SUPPORTS_V1"
FIXTURE_EXPECTATIONS = {
    "LINEAR_REFLEX_K0_EQUAL_FIXTURE_V1": 0,
    "LINEAR_REFLEX_K1_EQUAL_FIXTURE_V1": 1,
}
STRIP_SUPPORT_LAW = "PLANAR_LINEAR_NORMAL_OFFSET_V1"
JUNCTION_SUPPORT_LAW = "DECLARED_JUNCTION_RELATION_SUPPORTS_V1"
CAP_CLOSURE_LAW = "PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1"
OWNERSHIP_CONTRACT = "TOTAL_DISJOINT_RESOLVED_COVERAGE_V1"
EVENT_LEDGER_CONTRACT = "LAZY_DETERMINISTIC_EVENT_LEDGER_V1"
TESSELLATION_CONTRACT = "DOWNSTREAM_TESSELLATION_AFTER_SEMANTIC_COALESCING_V1"

FORBIDDEN_TOP_LEVEL = {
    "skeleton",
    "envelopes",
    "region_graph",
    "visuals",
    "pivot_digest_projection",
}
FORBIDDEN_V3_FIELDS = {
    "join_policy",
    "coverage_contributions",
    "participant_contribution_ids",
    "contribution_ids",
    "angular_profile_bindings",
    "owner_sector_angle_class",
}
FORBIDDEN_CORE_WORDS = re.compile(r"\b(?:MITER|BEVEL|ROUND|CornerEnvelope)\b")
RUNTIME_SNAPSHOT_KEYS = {
    "seeds",
    "front_components",
    "envelope_specs",
    "envelope_instances",
    "active_interval_model",
    "boundary_events",
    "capacity_states",
    "effective_alpha",
    "topology_events",
    "interactions",
    "event_ledger",
    "downstream_tessellation",
    "angular_profile_selection_certificates",
}
COORDINATE_KEYS = {
    "x",
    "y",
    "z",
    "point",
    "points",
    "vector",
    "vectors",
    "coordinate",
    "coordinates",
    "tolerance",
    "epsilon",
}
TESS_ALLOWED = {
    "INTERNAL_TRIANGULATION",
    "INTERNAL_POLYGONIZATION",
    "FACE_ORDER",
    "SEMANTICALLY_EQUIVALENT_INTERNAL_DIAGONALS",
}
TESS_FORBIDDEN = {
    "PROFILE_SELECTION",
    "HIDDEN_EDGE_COUNT_SELECTION",
    "SILHOUETTE_GENERATION_OR_REPAIR",
    "OWNER_OR_PROVENANCE_INFERENCE",
    "PROFILE_BOUNDARY_DISSOLVE",
    "FACE_COUNT_AS_SEMANTIC_AUTHORITY",
}
DIGEST_TESSELLATION_EXCLUSIONS = {
    "downstream_face_order",
    "internal_fill_diagonals",
    "valid_internal_triangulation",
    "runtime_face_count",
}


class ValidationError(RuntimeError):
    pass


def fail(case_id: str, message: str) -> None:
    raise ValidationError(f"{case_id}: {message}")


def read_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValidationError(f"{path.relative_to(REPO)}: invalid JSON: {exc}") from exc


def walk_keys(value, path=""):
    if isinstance(value, dict):
        for key, child in value.items():
            child_path = f"{path}.{key}" if path else key
            yield key, child_path
            yield from walk_keys(child, child_path)
    elif isinstance(value, list):
        for item_index, child in enumerate(value):
            yield from walk_keys(child, f"{path}[{item_index}]")


def index(records, case_id, label):
    result = {}
    for record in records:
        identifier = record.get("id")
        if not identifier:
            fail(case_id, f"{label} record has no id")
        if identifier in result:
            fail(case_id, f"duplicate {label} id {identifier}")
        result[identifier] = record
    return result


def require_exact_keys(value, expected, case_id, label):
    actual = set(value)
    expected = set(expected)
    if actual != expected:
        fail(
            case_id,
            f"{label} keys differ: missing={sorted(expected-actual)}, "
            f"extra={sorted(actual-expected)}",
        )


def validate_against_schema(value, schema, root_schema, value_path="$", case_id="schema"):
    """Проверяет используемое EC0 подмножество JSON Schema без внешней зависимости."""
    if "$ref" in schema:
        ref = schema["$ref"]
        if not ref.startswith("#/$defs/"):
            fail(case_id, f"unsupported schema reference {ref}")
        target = root_schema.get("$defs", {}).get(ref.rsplit("/", 1)[-1])
        if target is None:
            fail(case_id, f"unresolved schema reference {ref}")
        validate_against_schema(value, target, root_schema, value_path, case_id)
        return

    if "oneOf" in schema:
        matches = 0
        for alternative in schema["oneOf"]:
            try:
                validate_against_schema(value, alternative, root_schema, value_path, case_id)
            except ValidationError:
                continue
            matches += 1
        if matches != 1:
            fail(case_id, f"{value_path} matches {matches} oneOf alternatives")
        return

    expected_type = schema.get("type")
    type_matches = {
        "object": lambda item: isinstance(item, dict),
        "array": lambda item: isinstance(item, list),
        "string": lambda item: isinstance(item, str),
        "integer": lambda item: isinstance(item, int) and not isinstance(item, bool),
        "boolean": lambda item: isinstance(item, bool),
    }
    if expected_type and not type_matches[expected_type](value):
        fail(case_id, f"{value_path} must be {expected_type}")
    if "const" in schema and value != schema["const"]:
        fail(case_id, f"{value_path} differs from schema const")
    if "enum" in schema and value not in schema["enum"]:
        fail(case_id, f"{value_path} is outside schema enum")

    if isinstance(value, dict):
        required = set(schema.get("required", []))
        missing = required - set(value)
        if missing:
            fail(case_id, f"{value_path} lacks schema fields {sorted(missing)}")
        properties = schema.get("properties", {})
        if schema.get("additionalProperties") is False:
            extra = set(value) - set(properties)
            if extra:
                fail(case_id, f"{value_path} has schema-forbidden fields {sorted(extra)}")
        for key, child in value.items():
            child_schema = properties.get(key)
            if child_schema is not None:
                validate_against_schema(
                    child,
                    child_schema,
                    root_schema,
                    f"{value_path}.{key}",
                    case_id,
                )
    elif isinstance(value, list):
        if len(value) < schema.get("minItems", 0):
            fail(case_id, f"{value_path} has fewer items than schema permits")
        if schema.get("uniqueItems"):
            canonical_items = [json.dumps(item, sort_keys=True) for item in value]
            if len(set(canonical_items)) != len(canonical_items):
                fail(case_id, f"{value_path} has duplicate schema-unique items")
        item_schema = schema.get("items")
        if item_schema is not None:
            for item_index, child in enumerate(value):
                validate_against_schema(
                    child,
                    item_schema,
                    root_schema,
                    f"{value_path}[{item_index}]",
                    case_id,
                )
    elif isinstance(value, str):
        if len(value) < schema.get("minLength", 0):
            fail(case_id, f"{value_path} is shorter than schema permits")
        if "pattern" in schema and re.search(schema["pattern"], value) is None:
            fail(case_id, f"{value_path} does not match schema pattern")
    elif isinstance(value, int) and not isinstance(value, bool):
        if "minimum" in schema and value < schema["minimum"]:
            fail(case_id, f"{value_path} is below schema minimum")


def validate_spec_policy(policy):
    if policy.get("schema") != "cftuv.envelope.ec0.envelope_spec_contracts.v2":
        raise ValidationError("EnvelopeSpec policy schema id mismatch")
    if policy.get("status") != "A1_A2_ANGLE_DRIVEN_LINEAR_REFLEX_CANDIDATE":
        raise ValidationError("EnvelopeSpec policy has unexpected status")
    if set(policy.get("envelope_spec_variants", [])) != ENVELOPE_VARIANTS:
        raise ValidationError("EnvelopeSpec typed union differs from AM11")
    if policy.get("envelope_spec_union_authority") != "USER_REQUIRED":
        raise ValidationError("EnvelopeSpec typed union lacks SemanticAuthority")

    profiles = {
        item.get("angular_profile_id"): item
        for item in policy.get("angular_profile_families", [])
    }
    if set(profiles) != {ANGULAR_PROFILE_ID}:
        raise ValidationError("there must be one LinearReflex product profile family")
    profile = profiles[ANGULAR_PROFILE_ID]
    required_profile_values = {
        "authority": "USER_REQUIRED",
        "family": "LinearReflexProfile",
        "eligible_sector_angle_class": "STRICT_REFLEX_PI_TO_2PI",
        "reflex_excess_law": "DELTA_EQUALS_PHI_MINUS_PI",
        "subdivision_policy": "EQUAL_REFLEX_EXCESS",
        "subturn_count_law": "K_PLUS_ONE",
        "subturn_law": "DELTA_OVER_K_PLUS_ONE",
        "hidden_edge_count_source": "ANGLE_DRIVEN_SELECTION_POLICY_CERTIFICATE",
        "all_support_normal_speed": "UNIT",
        "vertex_speed_law": "ONE_OVER_COS_DELTA_OVER_TWO_K_PLUS_TWO",
    }
    for key, expected in required_profile_values.items():
        if profile.get(key) != expected:
            raise ValidationError(f"{ANGULAR_PROFILE_ID}: invalid {key}")
    if "MANUAL_PER_CORNER_K_SELECTOR" not in profile.get("forbidden", []):
        raise ValidationError("manual per-corner k selector is not forbidden")

    policies = {
        item.get("selection_policy_id"): item
        for item in policy.get("angular_profile_selection_policies", [])
    }
    if set(policies) != {ANGULAR_SELECTION_POLICY_ID}:
        raise ValidationError("angle-driven selection-policy catalog mismatch")
    selection = policies[ANGULAR_SELECTION_POLICY_ID]
    required_selection_values = {
        "authority": "USER_REQUIRED",
        "input_measure": "CERTIFIED_ORIENTED_REFLEX_EXCESS_DELTA",
        "parameter_ref": ANGULAR_PARAMETER_ID,
        "parameter_symbol": "DELTA_MAX",
        "parameter_constraint": "ZERO_LT_DELTA_MAX_LT_PI",
        "result_law": "K_EQUALS_MAX_ZERO_CEIL_DELTA_OVER_DELTA_MAX_MINUS_ONE",
        "minimality_lower_bound": "K_EQ_ZERO_OR_K_TIMES_DELTA_MAX_LT_DELTA",
        "admissibility_upper_bound": "DELTA_LEQ_K_PLUS_ONE_TIMES_DELTA_MAX",
        "resolved_subturn_guarantee": "DELTA_OVER_K_PLUS_ONE_LEQ_DELTA_MAX",
        "selection_certificate_form": "OPEN_K_LT_DELTA_OVER_DELTA_MAX_LEQ_K_PLUS_ONE_CLOSED",
        "selection_authority": "NAMED_ANGLE_DRIVEN_PRODUCT_POLICY",
    }
    for key, expected in required_selection_values.items():
        if selection.get(key) != expected:
            raise ValidationError(f"{ANGULAR_SELECTION_POLICY_ID}: invalid {key}")
    if not {"FIXTURE_NAME", "LEGACY_JOIN_LABEL", "TESSELLATION"} <= set(selection.get("forbidden_inputs", [])):
        raise ValidationError("angle-driven selection policy has incomplete forbidden inputs")

    parameters = {
        item.get("parameter_id"): item
        for item in policy.get("angular_profile_parameters", [])
    }
    if set(parameters) != {ANGULAR_PARAMETER_ID}:
        raise ValidationError("max-subturn product parameter catalog mismatch")
    if parameters[ANGULAR_PARAMETER_ID].get("constraint") != "ZERO_LT_DELTA_MAX_LT_PI":
        raise ValidationError("max-subturn product parameter range is incomplete")
    if (
        parameters[ANGULAR_PARAMETER_ID].get("default_status")
        != "BLOCKED_PENDING_USER_DECISION"
        or parameters[ANGULAR_PARAMETER_ID].get("approved_default_value") is not None
    ):
        raise ValidationError("max-subturn product parameter must not contain an agent-selected default")

    fixtures = {
        item.get("fixture_id"): item
        for item in policy.get("angular_profile_regression_fixtures", [])
    }
    if set(fixtures) != set(FIXTURE_EXPECTATIONS):
        raise ValidationError("K0/K1 regression fixture catalog mismatch")
    for fixture_id, expected_k in FIXTURE_EXPECTATIONS.items():
        fixture = fixtures[fixture_id]
        if fixture.get("expected_resolved_hidden_edge_count") != expected_k:
            raise ValidationError(f"{fixture_id}: resolved k mismatch")
        if fixture.get("role") != "REGRESSION_RESULT_NOT_PRODUCT_SELECTOR":
            raise ValidationError(f"{fixture_id}: fixture became a product selector")

    sectors = policy.get("oriented_angular_sector_contracts", [])
    if len(sectors) != 1 or sectors[0].get("contract_id") != ANGULAR_SECTOR_CONTRACT:
        raise ValidationError("oriented owner-sector contract is missing")
    required_sector_facts = {
        "OWNER_SECTOR_ID",
        "ORDERED_INCIDENT_CHAIN_USE_IDS",
        "INCOMING_SOURCE_SUPPORT_REF",
        "OUTGOING_SOURCE_SUPPORT_REF",
        "TURN_ORIENTATION_IN_OWNER_PATCH",
        "REFLEX_ANGLE_MEASURE_REF",
    }
    if set(sectors[0].get("required_analysis_facts", [])) != required_sector_facts:
        raise ValidationError("oriented owner-sector facts are incomplete")

    hidden_laws = policy.get("hidden_support_generation_laws", [])
    if len(hidden_laws) != 1 or hidden_laws[0].get("generation_law_id") != HIDDEN_SUPPORT_GENERATION_LAW:
        raise ValidationError("hidden-support generation law is missing")
    if hidden_laws[0].get("active_support_count_law") != "K_PLUS_TWO" or hidden_laws[0].get("local_profile_segment_count_law") != "K_PLUS_TWO_BEFORE_CLIP_UNION_INTERACTION":
        raise ValidationError("k+2 support/boundary cardinality law is missing")
    if hidden_laws[0].get("exposure_law") != "EACH_LOCAL_PROFILE_SEGMENT_IS_EXPOSED_ONLY_IF_PRESENT_ON_RESOLVED_COVERAGE_SILHOUETTE":
        raise ValidationError("local k+2 profile segments are incorrectly promoted to final silhouette count")

    strip = policy.get("strip_support_laws", [])
    if len(strip) != 1 or strip[0] != {
        "support_law_id": STRIP_SUPPORT_LAW,
        "authority": "MATHEMATICALLY_REQUIRED",
        "source_support_law": "N_DOT_POSITION_EQUALS_C",
        "moving_support_law": "N_DOT_POSITION_EQUALS_C_PLUS_ALPHA",
        "normal_speed": "UNIT",
        "orientation": "OWNER_INTERIOR",
        "longitudinal_boundaries": "PARALLEL_SUPPORT_FEATURES",
        "terminal_interfaces": "SUPPLIED_BY_REFERENCED_ANGULAR_JUNCTION_OR_CAP_SPEC",
        "station_model": "SEMANTIC_CHAIN_USE_S",
        "transverse_model": "SIGNED_OWNER_INTERIOR_R",
    }:
        raise ValidationError("Strip support law is incomplete or changed")

    cap = policy.get("cap_closure_laws", [])
    if len(cap) != 1 or cap[0].get("closure_law_id") != CAP_CLOSURE_LAW:
        raise ValidationError("Cap closure law is missing")
    if cap[0].get("authority") != "MATHEMATICALLY_REQUIRED":
        raise ValidationError("Cap closure law lacks SemanticAuthority")
    if cap[0].get("exact_two_pi_policy") != "TERMINAL_OR_JUNCTION_NEVER_ANGULAR_PROFILE":
        raise ValidationError("exact two-pi terminal policy is missing")
    if "ANGULAR_PROFILE_AT_EXACT_TWO_PI" not in cap[0].get("forbidden", []):
        raise ValidationError("Cap law permits Angular profile at exact two-pi")

    junction = policy.get("junction_support_laws", [])
    if len(junction) != 1 or junction[0].get("support_law_id") != JUNCTION_SUPPORT_LAW:
        raise ValidationError("Junction support law is missing")
    if junction[0].get("authority") != "USER_REQUIRED":
        raise ValidationError("Junction support law lacks SemanticAuthority")
    ownership = policy.get("ownership_partition_contracts", [])
    if len(ownership) != 1 or ownership[0].get("contract_id") != OWNERSHIP_CONTRACT:
        raise ValidationError("ownership partition contract is missing")
    if ownership[0].get("authority") != "MATHEMATICALLY_REQUIRED":
        raise ValidationError("ownership partition lacks SemanticAuthority")
    ledger = policy.get("event_ledger_contracts", [])
    if len(ledger) != 1 or ledger[0].get("contract_id") != EVENT_LEDGER_CONTRACT:
        raise ValidationError("lazy event-ledger contract is missing")
    if ledger[0].get("authority") != "USER_REQUIRED":
        raise ValidationError("event-ledger contract lacks SemanticAuthority")
    tessellation = policy.get("downstream_tessellation_contracts", [])
    if len(tessellation) != 1 or tessellation[0].get("contract_id") != TESSELLATION_CONTRACT:
        raise ValidationError("downstream tessellation contract is missing")
    if tessellation[0].get("authority") != "USER_REQUIRED":
        raise ValidationError("downstream tessellation contract lacks SemanticAuthority")
    if set(tessellation[0].get("allowed_variation", [])) != TESS_ALLOWED:
        raise ValidationError("downstream tessellation allowed variation mismatch")
    if set(tessellation[0].get("forbidden", [])) != TESS_FORBIDDEN:
        raise ValidationError("downstream tessellation forbidden list mismatch")
    return policy


def validate_angular_spec(
    case_id,
    spec,
    seeds,
    components,
    corners,
    angular_sectors,
    angle_certificates,
    selection_certificates,
    bindings,
):
    relation_id = spec.get("source_relation_id")
    relation = corners.get(relation_id)
    seed = seeds.get(spec.get("source_seed_id"))
    sector = angular_sectors.get(spec.get("owner_sector_id"))
    angle = angle_certificates.get(relation.get("reflex_angle_measure_ref") if relation else None)
    certificate = selection_certificates.get(spec.get("profile_selection_certificate_id"))
    binding = bindings.get(relation_id)
    if not relation or not sector or not angle or not certificate or not binding or not seed or seed.get("kind") != "CornerSeed":
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} has invalid relation/profile/seed")
    if seed.get("source_relation_id") != relation_id:
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} seed relation mismatch")
    if relation.get("relation_kind") != "ANGULAR_RELATION":
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} relation is not angular")
    if relation.get("exact_two_pi") is not False:
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} admits exact two-pi")
    if relation.get("owner_sector_id") != sector["id"] or angle.get("owner_sector_id") != sector["id"]:
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} owner-sector lineage mismatch")
    if angle.get("strict_range_certificate") != "STRICT_PI_LT_PHI_LT_2PI":
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} lacks strict reflex-angle certificate")

    resolved_count = certificate.get("resolved_hidden_edge_count")
    if not isinstance(resolved_count, int) or resolved_count < 0:
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} has invalid resolved k")
    ratio_interval = certificate.get("certified_ratio_interval", {})
    if (
        ratio_interval.get("ratio") != "DELTA_OVER_DELTA_MAX"
        or ratio_interval.get("lower_bound_kind") != "OPEN"
        or ratio_interval.get("upper_bound_kind") != "CLOSED"
        or not isinstance(ratio_interval.get("lower_bound_integer"), int)
        or ratio_interval.get("lower_bound_integer") < 0
        or ratio_interval.get("upper_bound_integer")
        != ratio_interval.get("lower_bound_integer") + 1
    ):
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} has invalid certified ratio interval")
    expected_count = ratio_interval["lower_bound_integer"]
    if resolved_count != expected_count:
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} resolved k was not derived from its angle/policy certificate")
    fixed_values = {
        "angular_profile_id": ANGULAR_PROFILE_ID,
        "owner_sector_id": sector["id"],
        "profile_selection_policy_id": ANGULAR_SELECTION_POLICY_ID,
        "profile_selection_certificate_id": certificate["id"],
        "reflex_excess_law": "DELTA_EQUALS_PHI_MINUS_PI",
        "hidden_edge_count": expected_count,
        "hidden_edge_count_policy": "ANGLE_DRIVEN_MIN_K_FOR_MAX_SUBTURN",
        "subdivision_policy": "EQUAL_REFLEX_EXCESS",
        "subturn_count_law": "K_PLUS_ONE",
        "subturn_law": "DELTA_OVER_K_PLUS_ONE",
        "active_support_count_law": "K_PLUS_TWO",
        "active_support_count": expected_count + 2,
        "local_profile_segment_count_law": "K_PLUS_TWO",
        "local_profile_segment_count": expected_count + 2,
        "hidden_support_generation_law": "ORDINALS_ONE_THROUGH_K_INSIDE_ORIENTED_OWNER_SECTOR",
        "all_support_normal_speed": "UNIT",
        "profile_provenance": "ANGLE_DRIVEN_SELECTION_CERTIFICATE",
    }
    for key, expected in fixed_values.items():
        if spec.get(key) != expected:
            fail(case_id, f"AngularEnvelopeSpec {spec['id']} invalid {key}")

    hidden = spec.get("hidden_supports", [])
    if len(hidden) != expected_count:
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} hidden-edge count mismatch")
    by_ordinal = {item.get("ordinal"): item for item in hidden}
    if len(by_ordinal) != len(hidden):
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} duplicate hidden support ordinal")
    if set(by_ordinal) != set(range(1, expected_count + 1)):
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} hidden support ordinals mismatch")
    for ordinal in range(1, expected_count + 1):
        support = by_ordinal.get(ordinal)
        if not support:
            fail(case_id, f"AngularEnvelopeSpec {spec['id']} missing hidden support")
        expected_support = {
            "ordinal": ordinal,
            "turn_fraction": f"{ordinal}/{expected_count + 1}",
            "turn_fraction_law": "ORDINAL_OVER_K_PLUS_ONE",
            "direction_law": "ROTATE_INCOMING_SUPPORT_TOWARD_OUTGOING_INSIDE_ORIENTED_OWNER_SECTOR_BY_ORDINAL_SUBTURN",
            "zero_length_at_alpha_zero": True,
            "scope": "ANGULAR_ENVELOPE_SPEC_LOCAL",
        }
        for key, expected in expected_support.items():
            if support.get(key) != expected:
                fail(case_id, f"AngularEnvelopeSpec {spec['id']} hidden support invalid {key}")
        lineage = support.get("lineage", {})
        if lineage != {
            "source_relation_id": relation_id,
            "owner_sector_id": sector["id"],
            "angular_profile_id": ANGULAR_PROFILE_ID,
            "selection_certificate_id": certificate["id"],
        }:
            fail(case_id, f"AngularEnvelopeSpec {spec['id']} hidden support lineage mismatch")

    ordered_uses = sector.get("ordered_incident_chain_use_ids", [])
    component_by_use = {component["chain_use_id"]: component_id for component_id, component in components.items()}
    expected_components = [component_by_use.get(use_id) for use_id in ordered_uses]
    if None in expected_components or spec.get("ordered_incident_front_component_ids") != expected_components:
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} incident components mismatch")
    if seed.get("ordered_incident_chain_use_ids") != ordered_uses:
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} seed support order mismatch")

    certificate_values = {
        "source_relation_id": relation_id,
        "owner_sector_id": sector["id"],
        "reflex_angle_measure_ref": angle["id"],
        "angular_profile_id": ANGULAR_PROFILE_ID,
        "selection_policy_id": ANGULAR_SELECTION_POLICY_ID,
        "max_subturn_parameter_ref": ANGULAR_PARAMETER_ID,
        "selection_law": "K_EQUALS_MAX_ZERO_CEIL_DELTA_OVER_DELTA_MAX_MINUS_ONE",
        "lower_bound_certificate": "K_EQ_ZERO_OR_K_TIMES_DELTA_MAX_LT_DELTA",
        "upper_bound_certificate": "DELTA_LEQ_K_PLUS_ONE_TIMES_DELTA_MAX",
        "certificate_authority": "EXACT_OR_CERTIFIED_ANGLE_COMPARISON",
    }
    for key, expected in certificate_values.items():
        if certificate.get(key) != expected:
            fail(case_id, f"AngularEnvelopeSpec {spec['id']} selection certificate invalid {key}")
    binding_values = {
        "angular_profile_id": ANGULAR_PROFILE_ID,
        "selection_policy_id": ANGULAR_SELECTION_POLICY_ID,
        "max_subturn_parameter_ref": ANGULAR_PARAMETER_ID,
        "selection_authority": "NAMED_ANGLE_DRIVEN_PRODUCT_POLICY",
    }
    for key, expected in binding_values.items():
        if binding.get(key) != expected:
            fail(case_id, f"AngularEnvelopeSpec {spec['id']} request binding invalid {key}")
    if seed.get("profile_selection_certificate_id") != certificate["id"] or seed.get("resolved_hidden_edge_count") != expected_count:
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} seed selection result mismatch")
    fixture_id = spec.get("regression_fixture_id")
    if fixture_id:
        if certificate.get("regression_fixture_id") != fixture_id or FIXTURE_EXPECTATIONS.get(fixture_id) != expected_count:
            fail(case_id, f"AngularEnvelopeSpec {spec['id']} regression fixture mismatch")
    if spec.get("incident_effective_alpha_policy") != "INCIDENT_EFFECTIVE_ALPHA_VECTOR":
        fail(case_id, f"AngularEnvelopeSpec {spec['id']} lacks alpha-vector policy")


def validate_case(case, manifest_contract_ids):
    case_id = case.get("case_id", "<missing-case-id>")
    required = {
        "schema",
        "case_id",
        "slug",
        "title",
        "status",
        "global_contract_ids",
        "semantic_contracts",
        "analysis_snapshot",
        "decal_request",
        "expected_compiled_plan",
        "acceptance",
        "metamorphic_expectations",
        "canonical_geometry_digest",
    }
    require_exact_keys(case, required, case_id, "top-level")
    if case["schema"] != "cftuv.envelope.ec0.case.v5":
        fail(case_id, "wrong schema")
    if case["status"] not in {
        "DEFINED",
        "UNSUPPORTED_NAMED_FAILURE",
        "BLOCKED_PENDING_USER_DECISION",
    }:
        fail(case_id, "invalid status")
    if set(case["global_contract_ids"]) != manifest_contract_ids:
        fail(case_id, "global contract references do not match manifest")
    for item in case["semantic_contracts"]:
        if item.get("authority") not in CANONICAL_AUTHORITY or not item.get("rule"):
            fail(case_id, f"invalid SemanticAuthority record {item}")
    if FORBIDDEN_TOP_LEVEL & set(case):
        fail(case_id, "parallel semantic graph is forbidden")
    serialized = json.dumps(case, ensure_ascii=False)
    if FORBIDDEN_CORE_WORDS.search(serialized):
        fail(case_id, "superseded core join vocabulary is present")

    for key, key_path in walk_keys(case):
        if key.lower() in COORDINATE_KEYS:
            fail(case_id, f"coordinate/tolerance key is forbidden: {key_path}")
        if key in {"visuals", "svg", "png", "html_presentation"}:
            fail(case_id, f"presentation visual field is forbidden: {key_path}")
        if key in FORBIDDEN_V3_FIELDS:
            fail(case_id, f"superseded v3 field is forbidden: {key_path}")

    snapshot = case["analysis_snapshot"]
    snapshot_keys = {key for key, _ in walk_keys(snapshot)}
    leak = RUNTIME_SNAPSHOT_KEYS & snapshot_keys
    if leak:
        fail(case_id, f"kernel-owned state leaked into AnalysisSnapshot: {sorted(leak)}")

    patches = index(snapshot.get("patches", []), case_id, "Patch")
    domains = index(snapshot.get("patch_domains", []), case_id, "PatchDomain")
    physical = index(snapshot.get("physical_chains", []), case_id, "PhysicalChain")
    uses = index(snapshot.get("chain_uses", []), case_id, "ChainUse")
    boundaries = index(snapshot.get("boundary_constraints", []), case_id, "BoundaryConstraint")
    vertices = index(snapshot.get("source_vertices", []), case_id, "SourceVertex")
    corners = index(snapshot.get("corner_relations", []), case_id, "CornerRelation")
    angular_sectors = index(snapshot.get("angular_owner_sectors", []), case_id, "AngularOwnerSector")
    angle_certificates = index(snapshot.get("reflex_angle_certificates", []), case_id, "ReflexAngleCertificate")
    junctions = index(snapshot.get("junction_relations", []), case_id, "JunctionRelation")

    sector_records = {}
    sectors_by_use = defaultdict(list)
    for domain in domains.values():
        if domain.get("owner_patch_id") not in patches:
            fail(case_id, f"domain {domain['id']} references missing Patch")
        for boundary_id in domain.get("boundary_constraint_ids", []):
            if boundary_id not in boundaries:
                fail(case_id, f"domain {domain['id']} references missing boundary {boundary_id}")
        for sector in domain.get("sectors", []):
            sector_id = sector.get("id")
            if not sector_id or sector_id in sector_records:
                fail(case_id, f"invalid or duplicate sector id {sector_id}")
            if not sector.get("analysis_proven") or not sector.get("multiplicity_reason"):
                fail(case_id, f"sector {sector_id} lacks analysis provenance")
            sector_records[sector_id] = sector
            sectors_by_use[sector.get("chain_use_id")].append(sector)

    for use in uses.values():
        if use.get("physical_chain_id") not in physical:
            fail(case_id, f"use {use['id']} references missing PhysicalChain")
        if use.get("owner_patch_id") not in patches:
            fail(case_id, f"use {use['id']} references missing Patch")
        if use.get("patch_domain_id") not in domains:
            fail(case_id, f"use {use['id']} references missing PatchDomain")
        launch = use.get("launch_locus", {})
        boundary = boundaries.get(launch.get("boundary_id"))
        if not boundary or boundary.get("constraint_kind") != "SOURCE_LAUNCH_BOUNDARY":
            fail(case_id, f"use {use['id']} has no SOURCE_LAUNCH_BOUNDARY")
        if not launch.get("source_support_non_blocking") or boundary.get("blocks_originating_seed") is not False:
            fail(case_id, f"use {use['id']} launch support blocks its own seed")
        if not sectors_by_use[use["id"]]:
            fail(case_id, f"use {use['id']} has no analysis-proven sector")
        if len(sectors_by_use[use["id"]]) > 1:
            reasons = {item["multiplicity_reason"] for item in sectors_by_use[use["id"]]}
            if reasons == {"SINGLE_OWNER_INTERIOR_SECTOR"} or "DEFAULT_LEFT_RIGHT" in reasons:
                fail(case_id, f"use {use['id']} has automatic sector multiplicity")

    relation_incidents = {}
    for relation in corners.values():
        sector = angular_sectors.get(relation.get("owner_sector_id"))
        angle = angle_certificates.get(relation.get("reflex_angle_measure_ref"))
        if not sector or not angle:
            fail(case_id, f"CornerRelation {relation['id']} lacks oriented sector/angle certificate")
        incident = sector.get("ordered_incident_chain_use_ids", [])
        if len(incident) != 2 or len(set(incident)) != 2 or not set(incident) <= set(uses):
            fail(case_id, f"CornerRelation {relation['id']} has invalid ordered incident uses")
        if relation.get("source_vertex_id") not in vertices:
            fail(case_id, f"CornerRelation {relation['id']} references missing vertex")
        if relation.get("relation_kind") != "ANGULAR_RELATION":
            fail(case_id, f"CornerRelation {relation['id']} is not an Angular relation")
        if relation.get("exact_two_pi") is not False:
            fail(case_id, f"CornerRelation {relation['id']} is exact two-pi")
        if sector.get("analysis_proven") is not True:
            fail(case_id, f"AngularOwnerSector {sector['id']} is not analysis-proven")
        if sector.get("patch_domain_id") not in domains or sector.get("owner_patch_id") != domains[sector["patch_domain_id"]]["owner_patch_id"]:
            fail(case_id, f"AngularOwnerSector {sector['id']} request/domain ownership mismatch")
        if any(uses[use_id]["patch_domain_id"] != sector["patch_domain_id"] for use_id in incident):
            fail(case_id, f"AngularOwnerSector {sector['id']} spans PatchDomains")
        incoming_ref = sector.get("incoming_support_ref", {})
        outgoing_ref = sector.get("outgoing_support_ref", {})
        expected_refs = (
            (incoming_ref, incident[0]),
            (outgoing_ref, incident[1]),
        )
        for support_ref, use_id in expected_refs:
            if support_ref.get("chain_use_id") != use_id:
                fail(case_id, f"AngularOwnerSector {sector['id']} support order mismatch")
            if support_ref.get("source_launch_boundary_id") != uses[use_id]["launch_locus"]["boundary_id"]:
                fail(case_id, f"AngularOwnerSector {sector['id']} support reference mismatch")
        if sector.get("turn_orientation") not in {"CCW_IN_OWNER_PATCH_ORIENTATION", "CW_IN_OWNER_PATCH_ORIENTATION"}:
            fail(case_id, f"AngularOwnerSector {sector['id']} lacks turn orientation")
        if angle.get("owner_sector_id") != sector["id"] or angle.get("measure_law") != "ORIENTED_OWNER_SECTOR_ANGLE":
            fail(case_id, f"ReflexAngleCertificate {angle['id']} owner-sector mismatch")
        if angle.get("measure_source") != "HOST_ANALYSIS_EXACT_OR_CERTIFIED" or angle.get("strict_range_certificate") != "STRICT_PI_LT_PHI_LT_2PI":
            fail(case_id, f"ReflexAngleCertificate {angle['id']} is not exact/certified strict reflex")
        relation_incidents[relation["id"]] = set(incident)
    if {item.get("owner_sector_id") for item in corners.values()} != set(angular_sectors):
        fail(case_id, "orphan or missing AngularOwnerSector")
    if {item.get("reflex_angle_measure_ref") for item in corners.values()} != set(angle_certificates):
        fail(case_id, "orphan or missing ReflexAngleCertificate")
    for relation in junctions.values():
        incident = relation.get("incident_chain_use_ids", [])
        if len(incident) < 2 or not set(incident) <= set(uses):
            fail(case_id, f"JunctionRelation {relation['id']} has invalid incident uses")
        if relation.get("source_vertex_id") not in vertices:
            fail(case_id, f"JunctionRelation {relation['id']} references missing vertex")
        relation_incidents[relation["id"]] = set(incident)

    request = case["decal_request"]
    request_id = request.get("decal_request_id")
    selected_uses = set(request.get("selected_chain_use_ids", []))
    if not request_id or not selected_uses or not selected_uses <= set(uses):
        fail(case_id, "DecalRequest has invalid selected ChainUses")
    bindings = {}
    for binding in request.get("angular_profile_selection_bindings", []):
        relation_id = binding.get("corner_relation_id")
        if relation_id in bindings:
            fail(case_id, f"duplicate Angular selection binding for {relation_id}")
        if relation_id not in corners:
            fail(case_id, "Angular selection binding references missing relation")
        expected_binding = {
            "angular_profile_id": ANGULAR_PROFILE_ID,
            "selection_policy_id": ANGULAR_SELECTION_POLICY_ID,
            "max_subturn_parameter_ref": ANGULAR_PARAMETER_ID,
            "selection_authority": "NAMED_ANGLE_DRIVEN_PRODUCT_POLICY",
        }
        if any(binding.get(key) != value for key, value in expected_binding.items()):
            fail(case_id, f"Angular selection binding for {relation_id} is not angle-driven")
        bindings[relation_id] = binding
    if set(bindings) != set(corners):
        fail(case_id, "every Angular relation must have exactly one request selection binding")

    evaluations = case["expected_compiled_plan"].get("patch_evaluations", [])
    if not evaluations:
        fail(case_id, "CompiledPatchEvaluationPlan is empty")
    plan_keys = set()
    covered_uses = set()
    all_components_by_use = defaultdict(list)

    for plan in evaluations:
        plan_key = plan.get("plan_key", {})
        key = (plan_key.get("decal_request_id"), plan_key.get("patch_domain_id"))
        if key in plan_keys:
            fail(case_id, f"duplicate evaluation key {key}")
        plan_keys.add(key)
        if key[0] != request_id or key[1] not in domains:
            fail(case_id, f"invalid evaluation key {key}")
        domain_id = key[1]
        owner_patch = domains[domain_id]["owner_patch_id"]
        if plan.get("owner_patch_id") != owner_patch:
            fail(case_id, f"evaluation {plan.get('id')} owner Patch mismatch")

        seeds = index(plan.get("seeds", []), case_id, "Seed")
        components = index(plan.get("front_components", []), case_id, "FrontComponent")
        specs = index(plan.get("envelope_specs", []), case_id, "EnvelopeSpec")
        instances = index(plan.get("envelope_instances", []), case_id, "EnvelopeInstance")
        selection_certificates = index(
            plan.get("angular_profile_selection_certificates", []),
            case_id,
            "AngularProfileSelectionCertificate",
        )
        if set(plan.get("source_seed_ids", [])) != set(seeds):
            fail(case_id, f"evaluation {plan.get('id')} source_seed_ids mismatch")
        plan_use_ids = {
            use_id
            for use_id, use in uses.items()
            if use["patch_domain_id"] == domain_id and use_id in selected_uses
        }
        covered_uses |= plan_use_ids

        for seed in seeds.values():
            kind = seed.get("kind")
            if kind == "FrontSeed":
                use_id = seed.get("chain_use_id")
                if use_id not in plan_use_ids:
                    fail(case_id, f"FrontSeed {seed['id']} is outside evaluation domain")
                expected_sectors = {item["id"] for item in sectors_by_use[use_id]}
                if set(seed.get("sector_ids", [])) != expected_sectors:
                    fail(case_id, f"FrontSeed {seed['id']} sector set mismatch")
            elif kind == "CornerSeed":
                relation_id = seed.get("source_relation_id")
                relation = corners.get(relation_id)
                sector = angular_sectors.get(relation.get("owner_sector_id") if relation else None)
                certificate = selection_certificates.get(seed.get("profile_selection_certificate_id"))
                if not relation or not sector or not certificate:
                    fail(case_id, f"CornerSeed {seed['id']} lacks relation/sector/selection certificate")
                if seed.get("ordered_incident_chain_use_ids") != sector.get("ordered_incident_chain_use_ids"):
                    fail(case_id, f"CornerSeed {seed['id']} incident support order differs from relation")
                seed_values = {
                    "owner_sector_id": sector["id"],
                    "angular_profile_id": ANGULAR_PROFILE_ID,
                    "profile_selection_policy_id": ANGULAR_SELECTION_POLICY_ID,
                    "resolved_hidden_edge_count": certificate.get("resolved_hidden_edge_count"),
                    "profile_selection_authority": "ANGLE_DRIVEN_POLICY_CERTIFICATE",
                }
                if any(seed.get(key) != value for key, value in seed_values.items()):
                    fail(case_id, f"CornerSeed {seed['id']} angle-driven selection mismatch")
            elif kind == "JunctionSeed":
                relation_id = seed.get("source_relation_id")
                incident = set(seed.get("incident_chain_use_ids", []))
                if relation_id not in junctions:
                    fail(case_id, f"JunctionSeed {seed['id']} references missing relation")
                if seed.get("projection_role") == "PER_PATCH_PROJECTION":
                    if not incident or not incident <= relation_incidents[relation_id]:
                        fail(case_id, f"Junction projection {seed['id']} has invalid incident subset")
                elif incident != relation_incidents[relation_id]:
                    fail(case_id, f"JunctionSeed {seed['id']} incident uses differ from relation")
            elif kind not in {"CapSeed", "EndpointClaimSeed"}:
                fail(case_id, f"unsupported seed kind {kind}")

        for component in components.values():
            use_id = component.get("chain_use_id")
            sector_id = component.get("sector_id")
            if use_id not in plan_use_ids or sector_id not in sector_records:
                fail(case_id, f"component {component['id']} has invalid use/sector")
            if sector_records[sector_id]["chain_use_id"] != use_id:
                fail(case_id, f"component {component['id']} sector belongs to another use")
            if component.get("front_seed_id") not in seeds:
                fail(case_id, f"component {component['id']} references missing FrontSeed")
            if component.get("branch_count_policy") != "NON_INCREASING" or component.get("initial_branch_count") != 1:
                fail(case_id, f"component {component['id']} violates branch-count policy")
            all_components_by_use[use_id].append(component["id"])

        component_ids = set(components)
        capacity_ids = {state.get("front_component_id") for state in plan.get("capacity_states", [])}
        if capacity_ids != component_ids:
            fail(case_id, f"evaluation {plan.get('id')} capacity states are incomplete")

        for spec in specs.values():
            variant = spec.get("variant")
            if variant not in ENVELOPE_VARIANTS:
                fail(case_id, f"EnvelopeSpec {spec['id']} has unknown variant {variant}")
            if spec.get("source_seed_id") not in seeds:
                fail(case_id, f"EnvelopeSpec {spec['id']} references missing seed")
            if spec.get("decal_request_id") != request_id or spec.get("patch_domain_id") != domain_id:
                fail(case_id, f"EnvelopeSpec {spec['id']} request/domain mismatch")
            if not spec.get("source_lineage_ids"):
                fail(case_id, f"EnvelopeSpec {spec['id']} has no source lineage")
            if spec.get("source_seed_id") not in spec["source_lineage_ids"]:
                fail(case_id, f"EnvelopeSpec {spec['id']} lineage omits its source seed")
            if variant == "StripEnvelopeSpec":
                front_ids = set(spec.get("front_component_ids", []))
                if len(front_ids) != 1 or not front_ids <= component_ids:
                    fail(case_id, f"StripEnvelopeSpec {spec['id']} needs one valid component")
                if spec.get("support_law_id") != STRIP_SUPPORT_LAW:
                    fail(case_id, f"StripEnvelopeSpec {spec['id']} support law mismatch")
                if spec.get("station_model_id") != "SEMANTIC_CHAIN_USE_S" or spec.get("transverse_model_id") != "SIGNED_OWNER_INTERIOR_R":
                    fail(case_id, f"StripEnvelopeSpec {spec['id']} station/support model mismatch")
                use_id = components[next(iter(front_ids))]["chain_use_id"]
                if use_id not in spec["source_lineage_ids"]:
                    fail(case_id, f"StripEnvelopeSpec {spec['id']} lineage omits its ChainUse")
            elif variant == "AngularEnvelopeSpec":
                validate_angular_spec(
                    case_id,
                    spec,
                    seeds,
                    components,
                    corners,
                    angular_sectors,
                    angle_certificates,
                    selection_certificates,
                    bindings,
                )
                if spec.get("source_relation_id") not in spec["source_lineage_ids"]:
                    fail(case_id, f"AngularEnvelopeSpec {spec['id']} lineage omits its relation")
                if spec.get("owner_sector_id") not in spec["source_lineage_ids"] or spec.get("profile_selection_certificate_id") not in spec["source_lineage_ids"]:
                    fail(case_id, f"AngularEnvelopeSpec {spec['id']} lineage omits sector/selection certificate")
            elif variant == "JunctionEnvelopeSpec":
                relation_id = spec.get("source_relation_id")
                seed = seeds.get(spec.get("source_seed_id"), {})
                if relation_id not in junctions or seed.get("kind") != "JunctionSeed":
                    fail(case_id, f"JunctionEnvelopeSpec {spec['id']} relation/seed mismatch")
                if seed.get("source_relation_id") != relation_id:
                    fail(case_id, f"JunctionEnvelopeSpec {spec['id']} seed relation mismatch")
                incident = set(spec.get("incident_front_component_ids", []))
                expected_incident = {
                    component_id
                    for component_id, component in components.items()
                    if component.get("chain_use_id") in set(seed.get("incident_chain_use_ids", []))
                }
                if not incident or incident != expected_incident:
                    fail(case_id, f"JunctionEnvelopeSpec {spec['id']} components invalid")
                if spec.get("support_law_id") != JUNCTION_SUPPORT_LAW:
                    fail(case_id, f"JunctionEnvelopeSpec {spec['id']} support law mismatch")
                if relation_id not in spec["source_lineage_ids"]:
                    fail(case_id, f"JunctionEnvelopeSpec {spec['id']} lineage omits its relation")
            elif variant == "CapEnvelopeSpec":
                seed = seeds.get(spec.get("source_seed_id"), {})
                if seed.get("kind") != "CapSeed":
                    fail(case_id, f"CapEnvelopeSpec {spec['id']} source is not CapSeed")
                if spec.get("closure_law_id") != CAP_CLOSURE_LAW:
                    fail(case_id, f"CapEnvelopeSpec {spec['id']} closure law mismatch")
                if spec.get("terminal_source_vertex_id") not in vertices:
                    fail(case_id, f"CapEnvelopeSpec {spec['id']} terminal vertex missing")
                if spec.get("incident_chain_use_id") not in plan_use_ids:
                    fail(case_id, f"CapEnvelopeSpec {spec['id']} incident use invalid")
                if spec.get("station_law") != "CONSTANT_PHYSICAL_ENDPOINT_S":
                    fail(case_id, f"CapEnvelopeSpec {spec['id']} station law mismatch")
                if spec.get("exact_two_pi_handling") != "TERMINAL_OR_JUNCTION_NEVER_ANGULAR_PROFILE":
                    fail(case_id, f"CapEnvelopeSpec {spec['id']} exact two-pi policy mismatch")
                if spec.get("incident_chain_use_id") not in spec["source_lineage_ids"]:
                    fail(case_id, f"CapEnvelopeSpec {spec['id']} lineage omits its ChainUse")

        for spec in specs.values():
            if spec["variant"] == "StripEnvelopeSpec":
                terminal_ids = set(spec.get("terminal_interface_spec_ids", []))
                if not terminal_ids <= set(specs):
                    fail(case_id, f"StripEnvelopeSpec {spec['id']} terminal reference missing")
                if any(specs[item]["variant"] == "StripEnvelopeSpec" for item in terminal_ids):
                    fail(case_id, f"StripEnvelopeSpec {spec['id']} terminates at another Strip")
            if spec["variant"] == "CapEnvelopeSpec":
                strip_id = spec.get("incident_strip_spec_id")
                if strip_id not in specs or specs[strip_id]["variant"] != "StripEnvelopeSpec":
                    fail(case_id, f"CapEnvelopeSpec {spec['id']} incident Strip missing")
                if spec["id"] not in specs[strip_id].get("terminal_interface_spec_ids", []):
                    fail(case_id, f"CapEnvelopeSpec {spec['id']} is not reciprocal Strip terminal")

        spec_to_instance = defaultdict(list)
        for instance in instances.values():
            spec_id = instance.get("spec_id")
            spec = specs.get(spec_id)
            if not spec or instance.get("spec_variant") != spec.get("variant"):
                fail(case_id, f"EnvelopeInstance {instance['id']} spec/variant mismatch")
            if instance.get("decal_request_id") != request_id or instance.get("patch_domain_id") != domain_id:
                fail(case_id, f"EnvelopeInstance {instance['id']} request/domain mismatch")
            if instance.get("boundary_resolution") != "REQUIRED_BEFORE_PATCH_UNION":
                fail(case_id, f"EnvelopeInstance {instance['id']} bypasses boundary resolution")
            alpha_binding = instance.get("effective_alpha_binding", {})
            if alpha_binding.get("kind") not in {
                "INCIDENT_FRONT_COMPONENT_VECTOR",
                "INCIDENT_STRIP_EFFECTIVE_ALPHA",
            }:
                fail(case_id, f"EnvelopeInstance {instance['id']} alpha binding kind invalid")
            binding_components = set(alpha_binding.get("front_component_ids", []))
            if not binding_components or not binding_components <= component_ids:
                fail(case_id, f"EnvelopeInstance {instance['id']} alpha components invalid")
            if spec["variant"] == "StripEnvelopeSpec":
                expected_binding = set(spec["front_component_ids"])
                expected_kind = "INCIDENT_FRONT_COMPONENT_VECTOR"
            elif spec["variant"] == "AngularEnvelopeSpec":
                ordered_binding = spec["ordered_incident_front_component_ids"]
                expected_binding = set(ordered_binding)
                expected_kind = "INCIDENT_FRONT_COMPONENT_VECTOR"
                if alpha_binding.get("front_component_ids") != ordered_binding:
                    fail(case_id, f"EnvelopeInstance {instance['id']} loses Angular support order")
            elif spec["variant"] == "JunctionEnvelopeSpec":
                expected_binding = set(spec["incident_front_component_ids"])
                expected_kind = "INCIDENT_FRONT_COMPONENT_VECTOR"
            else:
                strip_spec = specs[spec["incident_strip_spec_id"]]
                expected_binding = set(strip_spec["front_component_ids"])
                expected_kind = "INCIDENT_STRIP_EFFECTIVE_ALPHA"
            if binding_components != expected_binding or alpha_binding.get("kind") != expected_kind:
                fail(case_id, f"EnvelopeInstance {instance['id']} alpha binding differs from its spec")
            spec_to_instance[spec_id].append(instance["id"])
        if set(spec_to_instance) != set(specs) or any(len(ids) != 1 for ids in spec_to_instance.values()):
            fail(case_id, "each EnvelopeSpec must have exactly one EnvelopeInstance")
        angular_relation_ids = {
            spec["source_relation_id"]
            for spec in specs.values()
            if spec["variant"] == "AngularEnvelopeSpec"
        }
        if {item.get("source_relation_id") for item in selection_certificates.values()} != angular_relation_ids:
            fail(case_id, "Angular selection certificates do not match AngularEnvelopeSpecs exactly")

        coverage = plan.get("patch_coverage", {})
        if coverage.get("decal_request_id") != request_id or coverage.get("patch_domain_id") != domain_id:
            fail(case_id, "PatchCoverage request/domain mismatch")
        if coverage.get("operation") != "EXACT_SINGLE_COVER_UNION" or coverage.get("reachability") != "PROVEN_FROM_SOURCE_SEEDS":
            fail(case_id, "PatchCoverage does not prove exact reachable single cover")
        if set(coverage.get("envelope_instance_ids", [])) != set(instances):
            fail(case_id, "PatchCoverage EnvelopeInstance set mismatch")
        if coverage.get("raw_coverage_stage") != "AFTER_BOUNDARY_RESOLVED_ENVELOPE_INSTANCES":
            fail(case_id, "PatchCoverage union occurs before boundary resolution")

        for interaction in plan.get("interactions", []):
            if interaction.get("decal_request_id") != request_id or interaction.get("patch_domain_id") != domain_id:
                fail(case_id, f"interaction {interaction.get('id')} request/domain mismatch")
            participants = set(interaction.get("participant_envelope_instance_ids", []))
            if not participants or not participants <= set(instances):
                fail(case_id, f"interaction {interaction.get('id')} references missing instance")

        ownership = plan.get("ownership", {})
        if ownership.get("coverage_id") != coverage.get("id"):
            fail(case_id, "ownership references another coverage")
        ownership_values = {
            "partition_contract_ref": OWNERSHIP_CONTRACT,
            "total": True,
            "disjoint": True,
            "tie_boundary_owner": "NONE",
            "silhouette_effect": "NONE",
            "named_unproven_outcome": "OWNERSHIP_PARTITION_UNPROVEN",
        }
        for field, expected in ownership_values.items():
            if ownership.get(field) != expected:
                fail(case_id, f"ownership partition invalid {field}")
        claims = index(ownership.get("claims", []), case_id, "OwnershipClaim")
        claimed_specs = set()
        claimed_instances = set()
        for claim in claims.values():
            if claim.get("coverage_id") != coverage.get("id"):
                fail(case_id, f"ownership claim {claim['id']} references another coverage")
            claim_specs = set(claim.get("envelope_spec_ids", []))
            claim_instances = set(claim.get("envelope_instance_ids", []))
            if not claim_specs or not claim_specs <= set(specs):
                fail(case_id, f"ownership claim {claim['id']} spec set invalid")
            expected_instances = {spec_to_instance[item][0] for item in claim_specs}
            if claim_instances != expected_instances:
                fail(case_id, f"ownership claim {claim['id']} instance set mismatch")
            if not claim.get("claimant_kind") or not claim.get("claimant_id") or not claim.get("selection_law"):
                fail(case_id, f"ownership claim {claim['id']} is implicit")
            claimed_specs |= claim_specs
            claimed_instances |= claim_instances
        if claimed_specs != set(specs) or claimed_instances != set(instances):
            fail(case_id, "ownership claims do not cover every spec and instance")
        if ownership.get("proof_obligations") != {
            "claim_interiors": "PAIRWISE_DISJOINT",
            "claim_closure_union": "EQUALS_RESOLVED_COVERAGE",
            "equality_boundaries": "OWNERLESS",
        }:
            fail(case_id, "ownership proof obligations are incomplete")

        ledger = plan.get("event_ledger", {})
        if ledger != {
            "contract_ref": EVENT_LEDGER_CONTRACT,
            "startup_scope": "LAWS_PREDICATES_TRANSITIONS_AND_IMMEDIATE_CANDIDATES",
            "successor_scheduling": "LAZY",
            "same_alpha_policy": "ATOMIC_DETERMINISTIC_BATCH",
            "publish_policy": "COMPLETE_EXACT_STATE_ONLY",
            "pending_outcome": "PENDING_EXACT_EVALUATION",
        }:
            fail(case_id, "lazy deterministic event ledger is incomplete")

        tessellation = plan.get("downstream_tessellation", {})
        tess_values = {
            "contract_ref": TESSELLATION_CONTRACT,
            "starts_after": "SEMANTIC_COALESCING",
            "input_authority": "RESOLVED_OWNERSHIP_UV_STATION_ARRANGEMENT",
            "profile_controlled_boundary_policy": "PRESERVE",
            "semantic_digest_equivalence": "ALL_VALID_TESSELLATIONS_SHARE_ONE_SEMANTIC_DIGEST",
        }
        for field, expected in tess_values.items():
            if tessellation.get(field) != expected:
                fail(case_id, f"downstream tessellation invalid {field}")
        if set(tessellation.get("allowed_internal_variation", [])) != TESS_ALLOWED:
            fail(case_id, "downstream tessellation allowed variation mismatch")
        if set(tessellation.get("forbidden", [])) != TESS_FORBIDDEN:
            fail(case_id, "downstream tessellation forbidden list mismatch")

        provenance = plan.get("geometry_batch_provenance", {})
        if provenance.get("decal_request_id") != request_id or provenance.get("patch_domain_id") != domain_id:
            fail(case_id, "GeometryBatch provenance lacks request/domain key")
        if provenance.get("patch_coverage_id") != coverage.get("id"):
            fail(case_id, "GeometryBatch provenance references another coverage")

    if covered_uses != selected_uses:
        fail(case_id, f"selected uses are not covered by request/domain plans: {selected_uses-covered_uses}")
    for use_id in selected_uses:
        expected = len(sectors_by_use[use_id])
        actual = len(all_components_by_use[use_id])
        if actual != expected:
            fail(case_id, f"ChainUse {use_id} has {actual} components for {expected} proven sectors")

    digest = case.get("canonical_geometry_digest", {})
    if not DIGEST_TESSELLATION_EXCLUSIONS <= set(digest.get("exclude", [])):
        fail(case_id, "CanonicalGeometryDigest lacks downstream tessellation exclusions")

    # Именованные регрессии миграции Session A и предыдущих ревью.
    if case_id == "EC0-C11":
        seeds = evaluations[0]["seeds"]
        endpoint_claims = [item for item in seeds if item["kind"] == "EndpointClaimSeed"]
        if any(item["kind"] == "CornerSeed" for item in seeds) or len(endpoint_claims) != 2:
            fail(case_id, "short segment must use two endpoint claims, not an Angular seed")
    if case_id == "EC0-C12":
        plan = evaluations[0]
        angular = [item for item in plan["envelope_specs"] if item["variant"] == "AngularEnvelopeSpec"]
        claim_b = [item for item in plan["ownership"]["claims"] if item.get("id") == "claim_b_owner"]
        if len(angular) != 1 or angular[0]["id"] != "angular_spec_corner_b":
            fail(case_id, "claim B must have one AngularEnvelopeSpec identity")
        if len(claim_b) != 1 or "angular_spec_corner_b" not in claim_b[0].get("envelope_spec_ids", []):
            fail(case_id, "claim B ownership must explicitly include its AngularEnvelopeSpec")
    if case_id == "EC0-C13":
        plan = evaluations[0]
        seed = next(item for item in plan["seeds"] if item["id"] == "angular_corner_seed")
        if seed["ordered_incident_chain_use_ids"] != ["angular_in_use", "angular_out_use"]:
            fail(case_id, "other_wing_use leaked into Angular CornerSeed")
        angular = next(item for item in plan["envelope_specs"] if item["variant"] == "AngularEnvelopeSpec")
        if angular.get("angular_profile_id") != ANGULAR_PROFILE_ID or angular.get("hidden_edge_count") != 1:
            fail(case_id, "other-wing contact angle certificate must resolve a three-segment k=1 boundary")
        interactions = plan["interactions"]
        if len(interactions) != 1 or interactions[0].get("kind") != "ANGULAR_PROFILE_FRONT_CONTACT":
            fail(case_id, "other wing must meet Angular profile through an interaction")
    if case_id == "EC0-C16":
        interactions = evaluations[0]["interactions"]
        if (
            len(interactions) != 1
            or interactions[0].get("kind") != "INTRAPATCH_WING_COVERAGE_CLIP"
            or interactions[0].get("coverage_effect") != "CLIP_EXISTING_CONTRIBUTIONS_AT_EQUALITY_LOCUS"
            or interactions[0].get("creates_new_matter") is not False
        ):
            fail(case_id, "case 16 must encode selected same-request policy B")
    if case_id == "EC0-P05":
        if len(junctions) != 1 or len(evaluations) < 2:
            fail(case_id, "cross-Patch junction needs one global relation and multiple Patch plans")
        relation = next(iter(junctions.values()))
        anchor = relation.get("shared_semantic_anchor_id")
        if not anchor or relation.get("relation_kind") != "CROSS_PATCH":
            fail(case_id, "cross-Patch junction lacks shared anchor")
        for plan in evaluations:
            projections = [
                item for item in plan["envelope_specs"]
                if item["variant"] == "JunctionEnvelopeSpec"
            ]
            if len(projections) != 1 or projections[0].get("shared_semantic_anchor_id") != anchor or plan.get("interactions"):
                fail(case_id, "cross-Patch projection lost shared anchor or added collision")
    if case_id in {"EC0-P06", "EC0-P07"}:
        expected_variant = "AngularEnvelopeSpec" if case_id == "EC0-P06" else "JunctionEnvelopeSpec"
        shared = [
            item
            for plan in evaluations
            for item in plan["envelope_specs"]
            if item["variant"] == expected_variant
        ]
        if (
            len(shared) != 1
            or "incident_effective_alpha_values" not in shared[0]
            or shared[0].get("mixed_alpha_failure") != "SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN"
        ):
            fail(case_id, "mixed-alpha shared EnvelopeSpec policy is incomplete")
        if case_id == "EC0-P06" and shared[0].get("angular_profile_id") != ANGULAR_PROFILE_ID:
            fail(case_id, "mixed-alpha Angular case must use the angle-driven profile family")
    if case_id in {"EC0-C02", "EC0-C03"}:
        expected_fixture = {
            "EC0-C02": "LINEAR_REFLEX_K0_EQUAL_FIXTURE_V1",
            "EC0-C03": "LINEAR_REFLEX_K1_EQUAL_FIXTURE_V1",
        }[case_id]
        angular = [
            item for item in evaluations[0]["envelope_specs"]
            if item["variant"] == "AngularEnvelopeSpec"
        ]
        if len(angular) != 1 or angular[0].get("regression_fixture_id") != expected_fixture:
            fail(case_id, f"must remain regression fixture {expected_fixture}")
    if case_id not in {"EC0-C02", "EC0-C03"}:
        if any(
            "regression_fixture_id" in spec
            for plan in evaluations
            for spec in plan["envelope_specs"]
        ):
            fail(case_id, "a non-fixture case uses K0/K1 fixture identity as product selection")
    if case_id == "EC0-C15":
        expected_evolution = {"CURRENT_EXACT_STATE", "LAZY_SUCCESSOR_SCHEDULING", "EXACT_ATOMIC_AFTER_STATE"}
        if not expected_evolution <= set(case["acceptance"].get("alpha_evolution", [])):
            fail(case_id, "drag topology case is not lazy/atomic")


def validate_matrix(path, expected_ids, required_transforms):
    data = read_json(path)
    ids = set(data.get("cases", []))
    if ids != expected_ids:
        raise ValidationError(f"{path.relative_to(REPO)}: case set mismatch")
    rows = data.get("verdicts", [])
    transforms = {row.get("transform") for row in rows}
    missing = (AM3_REQUIRED_TRANSFORMS | required_transforms) - transforms
    if missing:
        raise ValidationError(f"{path.relative_to(REPO)}: missing transforms {sorted(missing)}")
    if len(transforms) != len(rows):
        raise ValidationError(f"{path.relative_to(REPO)}: duplicate transform")
    for row in rows:
        verdicts = row.get("by_case", {})
        if set(verdicts) != ids or not set(verdicts.values()) <= {"INVARIANT", "SEMANTIC_CHANGE"}:
            raise ValidationError(f"{path.relative_to(REPO)}: invalid row {row.get('transform')}")
    return data


def validate_matrices(main_ids, pivot_ids):
    main = validate_matrix(
        CORPUS / "matrices" / "metamorphic_matrix.json",
        main_ids,
        AM11_MAIN_TRANSFORMS,
    )
    pivot = validate_matrix(
        CORPUS / "matrices" / "pivot_metamorphic_matrix.json",
        pivot_ids,
        AM11_PIVOT_TRANSFORMS,
    )
    if main.get("schema") != "cftuv.envelope.ec0.metamorphic_matrix.v5":
        raise ValidationError("main metamorphic matrix schema id mismatch")
    if pivot.get("schema") != "cftuv.envelope.ec0.pivot_metamorphic_matrix.v4":
        raise ValidationError("pivot metamorphic matrix schema id mismatch")
    main_rows = {row["transform"]: row["by_case"] for row in main["verdicts"]}
    pivot_rows = {row["transform"]: row["by_case"] for row in pivot["verdicts"]}
    main_angular = {"EC0-C02", "EC0-C03", "EC0-C04", "EC0-C12", "EC0-C13"}
    for transform in {
        "angular_profile_selection_policy_switch",
        "hidden_edge_count_change",
        "angular_subdivision_policy_change",
        "incident_support_order_swap_without_orientation_update",
        "reflex_angle_change_crossing_k_selection_boundary",
        "max_subturn_parameter_change_crossing_k_selection_boundary",
        "manual_resolved_k_override",
    }:
        changed = {case_id for case_id, verdict in main_rows[transform].items() if verdict == "SEMANTIC_CHANGE"}
        if changed != main_angular:
            raise ValidationError(f"main matrix {transform} Angular case verdict mismatch")
        changed = {case_id for case_id, verdict in pivot_rows[transform].items() if verdict == "SEMANTIC_CHANGE"}
        if changed != {"EC0-P06"}:
            raise ValidationError(f"pivot matrix {transform} Angular case verdict mismatch")
    for rows, label in ((main_rows, "main"), (pivot_rows, "pivot")):
        if set(rows["hidden_support_record_permutation"].values()) != {"INVARIANT"}:
            raise ValidationError(f"{label} matrix hidden-support permutation must be invariant")
        if set(rows["oriented_angular_sector_record_permutation"].values()) != {"INVARIANT"}:
            raise ValidationError(f"{label} matrix oriented-sector record permutation must be invariant")
        if set(rows["chain_storage_reverse_preserve_semantics"].values()) != {"INVARIANT"}:
            raise ValidationError(f"{label} matrix coherent storage reversal must be invariant")
        if set(rows["surface_winding_reverse_preserve_domain"].values()) != {"INVARIANT"}:
            raise ValidationError(f"{label} matrix coherent winding reversal must be invariant")
        if set(rows["downstream_tessellation_change_preserve_arrangement"].values()) != {"INVARIANT"}:
            raise ValidationError(f"{label} matrix valid downstream tessellation must be invariant")
    dissolved = {
        case_id
        for case_id, verdict in main_rows["profile_controlled_boundary_dissolve"].items()
        if verdict == "SEMANTIC_CHANGE"
    }
    if dissolved != {"EC0-C03", "EC0-C13"}:
        raise ValidationError("exposed profile-controlled boundary dissolve verdict mismatch")

    boundary_policy = read_json(CORPUS / "policies" / "boundary_limited_propagation.json")
    boundary_matrix = read_json(CORPUS / "matrices" / "boundary_metamorphic_matrix.json")
    if boundary_matrix.get("schema") != "cftuv.envelope.ec0.boundary_metamorphic_matrix.v3":
        raise ValidationError("boundary metamorphic matrix schema id mismatch")
    scenario_ids = {item["id"] for item in boundary_policy.get("scenarios", [])}
    if len(scenario_ids) != 10 or scenario_ids != set(boundary_matrix.get("scenarios", [])):
        raise ValidationError("boundary policy/matrix scenario mismatch")
    by_id = {item["id"]: item for item in boundary_policy["scenarios"]}
    if by_id["BLP-04"]["contact_topology"] != "ENDPOINT_CONTACT" or by_id["BLP-05"]["contact_topology"] != "INTERIOR_CONTACT":
        raise ValidationError("hole endpoint/interior contact is not separated")
    if by_id["BLP-05"]["exact_event"] != "BARRIER_SPLIT_REQUIRED":
        raise ValidationError("interior hole contact must saturate at exact contact")
    transforms = {row.get("transform") for row in boundary_matrix.get("verdicts", [])}
    if not AM11_BOUNDARY_TRANSFORMS <= transforms:
        raise ValidationError("boundary matrix lacks downstream tessellation invariant")
    for row in boundary_matrix.get("verdicts", []):
        verdicts = row.get("by_scenario", {})
        if set(verdicts) != scenario_ids or not set(verdicts.values()) <= {"INVARIANT", "SEMANTIC_CHANGE"}:
            raise ValidationError(f"boundary matrix invalid row {row.get('transform')}")
    tessellation_row = next(
        row
        for row in boundary_matrix["verdicts"]
        if row["transform"] == "downstream_tessellation_change_preserve_arrangement"
    )
    if set(tessellation_row["by_scenario"].values()) != {"INVARIANT"}:
        raise ValidationError("boundary matrix valid downstream tessellation must be invariant")


def main() -> int:
    manifest = read_json(CORPUS / "manifest.json")
    if manifest.get("schema") != "cftuv.envelope.ec0.corpus.v5":
        raise ValidationError("manifest schema id mismatch")
    if manifest.get("status") != "A1_A2_CORRECTION_CANDIDATE_BLOCKED_PENDING_USER_DECISION":
        raise ValidationError("manifest has unexpected review status")
    if manifest.get("active_session") != "SESSION_A_EC0_LINEAR_REFLEX_REBASELINE":
        raise ValidationError("manifest active session mismatch")
    if manifest.get("ec1_gate") != "CLOSED_PENDING_MAX_SUBTURN_USER_DECISION_VALIDATOR_EXTERNAL_CI_AND_EXPLICIT_USER_ACCEPTANCE":
        raise ValidationError("EC1 gate must remain closed")
    if manifest.get("canonical_format") != "JSON":
        raise ValidationError("manifest canonical format must be JSON")
    expected_handoff = "artifacts/envelope_ec0/corpus/session_a_handoff.json"
    if manifest.get("handoff_file") != expected_handoff:
        raise ValidationError("manifest Session A handoff path mismatch")
    if manifest.get("presentation_visual_policy") != "FORBIDDEN":
        raise ValidationError("manifest must forbid presentation visuals")
    allowed_visuals = {
        "BLENDER_VIEWPORT_SCREENSHOT",
        "BLENDER_UV_EDITOR_SCREENSHOT",
        "BLENDER_DEBUG_OVERLAY_SCREENSHOT",
    }
    if set(manifest.get("allowed_visual_evidence", [])) != allowed_visuals:
        raise ValidationError("only Blender diagnostic screenshots may be visual evidence")

    expected_policy_files = {
        str((CORPUS / "policies" / "boundary_limited_propagation.json").resolve()),
        str(SPEC_POLICY_PATH.resolve()),
    }
    expected_matrix_files = {
        str((CORPUS / "matrices" / "metamorphic_matrix.json").resolve()),
        str((CORPUS / "matrices" / "pivot_metamorphic_matrix.json").resolve()),
        str((CORPUS / "matrices" / "boundary_metamorphic_matrix.json").resolve()),
    }
    policy_files = {str((REPO / item).resolve()) for item in manifest.get("policy_files", [])}
    matrix_files = {str((REPO / item).resolve()) for item in manifest.get("matrix_files", [])}
    if policy_files != expected_policy_files:
        raise ValidationError("manifest policy inventory differs from canonical policies")
    if matrix_files != expected_matrix_files:
        raise ValidationError("manifest matrix inventory differs from canonical matrices")

    schema = read_json(CORPUS / "schema" / "case.schema.json")
    if schema.get("$id") != "cftuv.envelope.ec0.case.v5":
        raise ValidationError("case JSON Schema id mismatch")
    union = schema.get("$defs", {}).get("envelopeSpec", {}).get("oneOf", [])
    union_refs = {item.get("$ref", "").rsplit("/", 1)[-1] for item in union}
    if union_refs != {
        "stripEnvelopeSpec",
        "angularEnvelopeSpec",
        "junctionEnvelopeSpec",
        "capEnvelopeSpec",
    }:
        raise ValidationError("case JSON Schema typed EnvelopeSpec union mismatch")

    validate_spec_policy(read_json(SPEC_POLICY_PATH))
    handoff = read_json(REPO / expected_handoff)
    if handoff.get("schema") != "cftuv.envelope.ec0.session_handoff.v2":
        raise ValidationError("Session A handoff schema id mismatch")
    if handoff.get("active_slice") != "SESSION_A_EC0_LINEAR_REFLEX_REBASELINE":
        raise ValidationError("Session A handoff active slice mismatch")
    if handoff.get("status") != "CANDIDATE_AWAITING_MAX_SUBTURN_USER_DECISION_EXTERNAL_CI_AND_EXPLICIT_USER_ACCEPTANCE":
        raise ValidationError("Session A handoff must not claim premature acceptance")
    if handoff.get("legacy_paths_read", {}).get("read") is not False:
        raise ValidationError("Session A handoff must record that legacy geometry was not read")
    ci = handoff.get("ci", {})
    if ci.get("required") is not True or ci.get("workflow") != ".github/workflows/envelope-ec0-corpus.yml":
        raise ValidationError("Session A handoff must delegate CI gate to the external GitHub check")
    if ci.get("status") not in {"PENDING", "PASS", "FAIL"}:
        raise ValidationError("Session A handoff CI status must be PENDING, PASS, or FAIL")
    if "UNCOMMITTED_WORKTREE" in json.dumps(handoff):
        raise ValidationError("Session A handoff contains the stale uncommitted-worktree receipt")
    global_contracts = manifest.get("global_semantic_contracts", [])
    global_ids = {item.get("id") for item in global_contracts}
    if len(global_ids) != len(global_contracts) or any(item.get("authority") not in CANONICAL_AUTHORITY for item in global_contracts):
        raise ValidationError("manifest has invalid global SemanticAuthority records")
    if not {f"EC0-G{number:02d}" for number in range(1, 19)} <= global_ids:
        raise ValidationError("manifest lacks A1/A2 global semantic contracts")

    decision_record = read_json(CORPUS / "decision_record.json")
    decisions = {item.get("id"): item for item in decision_record.get("decisions", [])}
    if decisions.get("AM4", {}).get("selected") != "B_COVERAGE_CLIP":
        raise ValidationError("decision record lost user-selected case 16 policy B")
    if decisions.get("VISUAL_POLICY", {}).get("selected") != "TEXT_JSON_VALIDATOR":
        raise ValidationError("decision record lost text/JSON artifact policy")
    if (
        decisions.get("AM11_ANGULAR_MODEL", {}).get("selected")
        != "LINEAR_REFLEX_ANGULAR_ENVELOPE_PROFILE"
    ):
        raise ValidationError("decision record lost AM11 Linear-Reflex pivot")
    a1_decision = decisions.get("A1_ANGULAR_PROFILE_SELECTION", {})
    if (
        a1_decision.get("selected") != ANGULAR_SELECTION_POLICY_ID
        or a1_decision.get("angular_profile_id") != ANGULAR_PROFILE_ID
        or a1_decision.get("selection_law")
        != "K_EQUALS_MAX_ZERO_CEIL_DELTA_OVER_DELTA_MAX_MINUS_ONE"
        or a1_decision.get("equivalent_constraint")
        != "K_IS_MINIMUM_NONNEGATIVE_INTEGER_WITH_DELTA_OVER_K_PLUS_ONE_LEQ_DELTA_MAX"
    ):
        raise ValidationError("decision record lost the A1 angle-driven selection law")
    a2_decision = decisions.get("A2_ORIENTED_ANGULAR_SECTOR", {})
    if (
        a2_decision.get("selected") != "ORDERED_SUPPORTS_INSIDE_ANALYSIS_PROVEN_OWNER_SECTOR"
        or a2_decision.get("hidden_support_turn_fraction_law")
        != "J_OVER_K_PLUS_ONE_FOR_J_ONE_THROUGH_K"
        or a2_decision.get("local_profile_cardinality_law")
        != "K_PLUS_TWO_SUPPORTS_AND_K_PLUS_TWO_PROFILE_SEGMENTS_BEFORE_CLIP_UNION_INTERACTION"
        or a2_decision.get("exposure_law")
        != "LOCAL_PROFILE_SEGMENT_IS_EXPOSED_ONLY_WHEN_ON_RESOLVED_COVERAGE_SILHOUETTE"
    ):
        raise ValidationError("decision record lost the A2 orientation/cardinality law")
    max_subturn_decision = decisions.get("A1_MAX_SUBTURN_DEFAULT", {})
    if (
        max_subturn_decision.get("status") != "BLOCKED_PENDING_USER_DECISION"
        or max_subturn_decision.get("selected_value") is not None
        or max_subturn_decision.get("forbidden_agent_action") != "DO_NOT_INVENT_OR_HIDE_A_DEFAULT_VALUE"
    ):
        raise ValidationError("max-subturn default must remain an explicit pending user decision")
    if manifest.get("blocked_pending_user_decisions") != ["A1_MAX_SUBTURN_DEFAULT"]:
        raise ValidationError("manifest must expose the pending max-subturn user decision")
    review_groups = decision_record.get("review_only_not_semantic", [])
    if {item.get("authority") for item in review_groups} != REVIEW_ONLY_AUTHORITY:
        raise ValidationError("decision record must enumerate all review-only authorities")
    if any(not item.get("items") for item in review_groups):
        raise ValidationError("decision record has an empty review-only list")

    case_paths = sorted(CASE_DIR.glob("*.json"))
    if len(case_paths) != 23:
        raise ValidationError(f"expected 23 cases, found {len(case_paths)}")
    manifest_paths = {str((REPO / item).resolve()) for item in manifest.get("case_files", [])}
    if manifest_paths != {str(path.resolve()) for path in case_paths}:
        raise ValidationError("manifest case inventory differs from filesystem")
    cases = [read_json(path) for path in case_paths]
    blocked = [case.get("case_id") for case in cases if case.get("status") == "BLOCKED_PENDING_USER_DECISION"]
    if blocked:
        raise ValidationError(f"unresolved user decisions remain: {blocked}")
    counts = Counter(case.get("case_id") for case in cases)
    duplicates = [case_id for case_id, count in counts.items() if count != 1]
    if duplicates:
        raise ValidationError(f"duplicate case ids: {duplicates}")
    for case in cases:
        validate_against_schema(
            case,
            schema,
            schema,
            case_id=case.get("case_id", "<missing-case-id>"),
        )
        validate_case(case, global_ids)

    main_ids = {case["case_id"] for case in cases if case["case_id"].startswith("EC0-C")}
    pivot_ids = {case["case_id"] for case in cases if case["case_id"].startswith("EC0-P")}
    if len(main_ids) != 16 or len(pivot_ids) != 7:
        raise ValidationError(f"expected 16 main and 7 pivot cases, found {len(main_ids)} and {len(pivot_ids)}")
    validate_matrices(main_ids, pivot_ids)

    canonical_paths = [
        *case_paths,
        CORPUS / "schema" / "case.schema.json",
        SPEC_POLICY_PATH,
        CORPUS / "matrices" / "metamorphic_matrix.json",
        CORPUS / "matrices" / "pivot_metamorphic_matrix.json",
        CORPUS / "matrices" / "boundary_metamorphic_matrix.json",
    ]
    for path in canonical_paths:
        if FORBIDDEN_CORE_WORDS.search(path.read_text(encoding="utf-8")):
            raise ValidationError(f"{path.relative_to(REPO)}: superseded core join vocabulary remains")

    presentation_files = []
    for suffix in ("*.svg", "*.png", "*.html"):
        presentation_files.extend((REPO / "artifacts" / "envelope_ec0").rglob(suffix))
    if presentation_files:
        joined = ", ".join(str(path.relative_to(REPO)) for path in presentation_files[:5])
        raise ValidationError(f"presentation visuals remain in active EC0 artifacts: {joined}")

    print("EC0 A1/A2 angle-driven corpus validation OK")
    print(f"  canonical JSON cases: {len(cases)} (main={len(main_ids)}, pivot={len(pivot_ids)})")
    print("  strict Session-A EnvelopeSpec/top-level JSON Schema conformance: OK")
    print("  typed EnvelopeSpec union and EnvelopeInstance separation: OK")
    print("  angle-driven MIN_K_FOR_MAX_SUBTURN selection certificates: OK")
    print("  oriented owner sectors, ordered supports, and k+2 cardinality: OK")
    print("  Strip support / Cap closure / exact-two-pi laws: OK")
    print("  explicit ownership partitions and single-cover union: OK")
    print("  lazy event ledger and downstream tessellation invariants: OK")
    print("  request/domain provenance and ChainUse component cardinality: OK")
    print("  C02/C03/C04/C13/P06/P07 migration regressions: OK")
    print("  legacy/implementation-accident/open-research review lists: explicit")
    print("  presentation artifacts: none; Blender screenshots remain policy-allowed")
    print("  factual receipt: corpus does not self-certify external GitHub CI")
    print("  EC1 gate: CLOSED_PENDING_MAX_SUBTURN_USER_DECISION_EXTERNAL_CI_AND_EXPLICIT_USER_ACCEPTANCE")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValidationError as exc:
        print(f"EC0 corpus validation FAILED: {exc}", file=sys.stderr)
        raise SystemExit(1)
