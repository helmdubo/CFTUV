"""Pure structural validation; геометрические predicates здесь не вычисляются."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .canonical import geometry_batch_semantic_digest
from .contracts.analysis import (
    ANALYSIS_SNAPSHOT_SCHEMA_V1,
    AnalysisCapability,
    AnalysisSnapshotV1,
)
from .contracts.coverage import CoverageEffect
from .contracts.envelopes import (
    AngularEnvelopeSpec,
    CapEnvelopeSpec,
    EnvelopeSpecVariant,
    JunctionEnvelopeSpec,
    StripEnvelopeSpec,
)
from .contracts.events import EventParticipantKind, InitialFrontFeatureKind
from .contracts.geometry_batch import GEOMETRY_BATCH_SCHEMA_V1, GeometryBatchV1
from .contracts.ownership import (
    ClaimClosureObligation,
    ClaimInteriorsObligation,
    EqualityBoundaryOwner,
    OwnershipPartitionContractId,
    SilhouetteEffect,
)
from .contracts.plan import COMPILED_PLAN_SCHEMA_V1, CompiledPatchEvaluationPlanV1
from .contracts.request import (
    DECAL_REQUEST_SCHEMA_V1,
    AngularProfileFamilyId,
    AngularProfileSelectionPolicyId,
    BoundaryPolicyId,
    CapPolicyId,
    DecalRequestV1,
    InteractionPolicyId,
    MaxSubturnParameterId,
    MaxSubturnValueId,
    OwnershipPolicyId,
)
from .contracts.seeds import (
    CapSeedV1,
    CornerSeedV1,
    EndpointClaimSeedV1,
    FrontSeedV1,
    JunctionSeedV1,
)
from .contracts.surface import SurfacePayloadMode
from .contracts.tessellation import (
    ForbiddenTessellationOperation,
    SharedVertexKeyPolicy,
    TessellationContractId,
    TessellationDigestEquivalence,
    TessellationStartStage,
)
from .ids import (
    BoundaryConstraintId,
    ChainUseId,
    EnvelopeInstanceId,
    EnvelopeSpecId,
    FrontComponentId,
    HiddenSupportId,
    OpaqueId,
    OwnerSectorId,
    SemanticDigestValue,
)
from .numeric import ExactAngleSymbol, LocalPoint3V1, MetricSpace
from .outcomes import NamedOutcome


class ValidationCode(str, Enum):
    SCHEMA_VERSION = "SCHEMA_VERSION"
    CAPABILITY = "CAPABILITY"
    DUPLICATE_ID = "DUPLICATE_ID"
    MISSING_REFERENCE = "MISSING_REFERENCE"
    CROSS_CONTRACT_MISMATCH = "CROSS_CONTRACT_MISMATCH"
    POLICY_MISMATCH = "POLICY_MISMATCH"
    ANGULAR_CERTIFICATE = "ANGULAR_CERTIFICATE"
    SEED_VARIANT_MISMATCH = "SEED_VARIANT_MISMATCH"
    PLAN_KEY_MISMATCH = "PLAN_KEY_MISMATCH"
    OWNERSHIP_DECLARATION = "OWNERSHIP_DECLARATION"
    TESSELLATION_AUTHORITY = "TESSELLATION_AUTHORITY"
    GEOMETRY_BATCH = "GEOMETRY_BATCH"
    FORBIDDEN_TOPOLOGY_IDENTITY = "FORBIDDEN_TOPOLOGY_IDENTITY"


@dataclass(frozen=True, slots=True)
class ValidationIssue:
    code: ValidationCode
    path: tuple[str, ...]
    message: str


class ContractValidationError(ValueError):
    def __init__(self, issues: tuple[ValidationIssue, ...]) -> None:
        self.issues = issues
        detail = "; ".join(
            f"{issue.code.value}@{'.'.join(issue.path)}: {issue.message}"
            for issue in issues
        )
        super().__init__(detail)


def raise_for_issues(issues: tuple[ValidationIssue, ...]) -> None:
    if issues:
        raise ContractValidationError(issues)


def _issue(
    issues: list[ValidationIssue],
    code: ValidationCode,
    path: tuple[str, ...],
    message: str,
) -> None:
    issues.append(ValidationIssue(code, path, message))


def _values(records: frozenset[object], attribute: str) -> set[OpaqueId]:
    return {getattr(record, attribute) for record in records}


def _check_unique(
    issues: list[ValidationIssue],
    records: frozenset[object],
    attribute: str,
    path: str,
) -> set[OpaqueId]:
    values = [getattr(record, attribute) for record in records]
    if len(values) != len(set(values)):
        _issue(issues, ValidationCode.DUPLICATE_ID, (path,), f"duplicate {attribute}")
    return set(values)


def _require_refs(
    issues: list[ValidationIssue],
    refs: set[OpaqueId],
    targets: set[OpaqueId],
    path: tuple[str, ...],
) -> None:
    missing = refs - targets
    if missing:
        _issue(
            issues,
            ValidationCode.MISSING_REFERENCE,
            path,
            "missing: " + ", ".join(sorted(str(value) for value in missing)),
        )


def validate_analysis_snapshot(snapshot: AnalysisSnapshotV1) -> tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []
    if snapshot.schema_version != ANALYSIS_SNAPSHOT_SCHEMA_V1:
        _issue(issues, ValidationCode.SCHEMA_VERSION, ("schema_version",), "unsupported snapshot schema")
    if snapshot.surface_ir.schema_version != "cftuv.envelope.patch_surface_ir.v1":
        _issue(issues, ValidationCode.SCHEMA_VERSION, ("surface_ir", "schema_version"), "unsupported surface schema")
    if snapshot.source_revision != snapshot.surface_ir.source_revision:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("surface_ir", "source_revision"), "surface and snapshot revisions differ")

    required = {
        AnalysisCapability.PATCH_DOMAIN_V1,
        AnalysisCapability.PHYSICAL_CHAIN_DIRECTED_USES_V1,
        AnalysisCapability.PATCH_SURFACE_IR_V1,
        AnalysisCapability.ORIENTED_OWNER_SECTOR_V1,
        AnalysisCapability.REFLEX_ANGLE_CERTIFICATE_V1,
        AnalysisCapability.RELATION_LINEAGE_V1,
    }
    missing_capabilities = required - set(snapshot.analysis_capabilities)
    if missing_capabilities:
        _issue(issues, ValidationCode.CAPABILITY, ("analysis_capabilities",), "missing required v1 capabilities")

    patch_ids = _check_unique(issues, snapshot.patches, "patch_id", "patches")
    domain_ids = _check_unique(issues, snapshot.patch_domains, "patch_domain_id", "patch_domains")
    vertex_ids = _check_unique(issues, snapshot.source_vertices, "vertex_id", "source_vertices")
    chain_ids = _check_unique(issues, snapshot.physical_chains, "physical_chain_id", "physical_chains")
    use_ids = _check_unique(issues, snapshot.chain_uses, "chain_use_id", "chain_uses")
    loop_ids = _check_unique(issues, snapshot.boundary_loops, "boundary_loop_id", "boundary_loops")
    boundary_ids = _check_unique(issues, snapshot.boundary_constraints, "boundary_constraint_id", "boundary_constraints")
    angular_sector_ids = _check_unique(issues, snapshot.angular_owner_sectors, "owner_sector_id", "angular_owner_sectors")
    angle_certificate_ids = _check_unique(issues, snapshot.reflex_angle_certificates, "certificate_id", "reflex_angle_certificates")
    corner_relation_ids = _check_unique(issues, snapshot.corner_relations, "corner_relation_id", "corner_relations")
    junction_relation_ids = _check_unique(issues, snapshot.junction_relations, "junction_relation_id", "junction_relations")
    del corner_relation_ids, junction_relation_ids

    _require_refs(issues, {domain.owner_patch_id for domain in snapshot.patch_domains}, patch_ids, ("patch_domains", "owner_patch_id"))
    _require_refs(issues, snapshot.surface_ir.source_vertex_ids, vertex_ids, ("surface_ir", "source_vertex_ids"))
    if snapshot.surface_ir.source_vertex_ids != vertex_ids:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("surface_ir", "source_vertex_ids"), "surface vertex set must equal snapshot vertex set")

    full_surface = snapshot.surface_ir.payload_mode is SurfacePayloadMode.FULL_HOST_SURFACE
    if full_surface:
        if AnalysisCapability.EC0_COORDINATE_FREE_FIXTURE_V5 in snapshot.analysis_capabilities:
            _issue(issues, ValidationCode.CAPABILITY, ("analysis_capabilities",), "fixture capability cannot label full host surface")
        for vertex in snapshot.source_vertices:
            if not isinstance(vertex.position, LocalPoint3V1):
                _issue(issues, ValidationCode.CAPABILITY, ("source_vertices", str(vertex.vertex_id), "position"), "full surface requires local coordinates")
        for chain in snapshot.physical_chains:
            if len(chain.ordered_source_vertex_ids) < 2:
                _issue(issues, ValidationCode.CAPABILITY, ("physical_chains", str(chain.physical_chain_id)), "full physical chain requires ordered vertices")
    elif AnalysisCapability.EC0_COORDINATE_FREE_FIXTURE_V5 not in snapshot.analysis_capabilities:
        _issue(issues, ValidationCode.CAPABILITY, ("analysis_capabilities",), "coordinate-free payload requires explicit fixture capability")

    face_ids = _check_unique(issues, snapshot.surface_ir.source_faces, "face_id", "surface_ir.source_faces")
    triangle_ids = _check_unique(issues, snapshot.surface_ir.surface_triangles, "triangle_id", "surface_ir.surface_triangles")
    for face in snapshot.surface_ir.source_faces:
        _require_refs(issues, {face.patch_id}, patch_ids, ("surface_ir", "source_faces", str(face.face_id), "patch_id"))
        _require_refs(issues, set(face.vertex_cycle), vertex_ids, ("surface_ir", "source_faces", str(face.face_id), "vertex_cycle"))
        _require_refs(issues, set(face.triangle_ids), triangle_ids, ("surface_ir", "source_faces", str(face.face_id), "triangle_ids"))
    for triangle in snapshot.surface_ir.surface_triangles:
        _require_refs(issues, {triangle.source_face_id}, face_ids, ("surface_ir", "surface_triangles", str(triangle.triangle_id), "source_face_id"))
        _require_refs(issues, set(triangle.vertex_ids), vertex_ids, ("surface_ir", "surface_triangles", str(triangle.triangle_id), "vertex_ids"))

    all_sector_ids: set[OpaqueId] = set(angular_sector_ids)
    for domain in snapshot.patch_domains:
        all_sector_ids.update(sector.owner_sector_id for sector in domain.sectors)
        _require_refs(issues, set(domain.boundary_constraint_ids), boundary_ids, ("patch_domains", str(domain.patch_domain_id), "boundary_constraint_ids"))
        _require_refs(issues, {sector.chain_use_id for sector in domain.sectors}, use_ids, ("patch_domains", str(domain.patch_domain_id), "sectors"))
    for chain in snapshot.physical_chains:
        _require_refs(issues, set(chain.ordered_source_vertex_ids), vertex_ids, ("physical_chains", str(chain.physical_chain_id), "ordered_source_vertex_ids"))
    for use in snapshot.chain_uses:
        _require_refs(issues, {use.physical_chain_id}, chain_ids, ("chain_uses", str(use.chain_use_id), "physical_chain_id"))
        _require_refs(issues, {use.owner_patch_id}, patch_ids, ("chain_uses", str(use.chain_use_id), "owner_patch_id"))
        _require_refs(issues, {use.patch_domain_id}, domain_ids, ("chain_uses", str(use.chain_use_id), "patch_domain_id"))
        _require_refs(issues, {use.launch_locus.boundary_constraint_id}, boundary_ids, ("chain_uses", str(use.chain_use_id), "launch_locus"))
        if use.boundary_loop_id is not None:
            _require_refs(issues, {use.boundary_loop_id}, loop_ids, ("chain_uses", str(use.chain_use_id), "boundary_loop_id"))
        elif full_surface:
            _issue(issues, ValidationCode.MISSING_REFERENCE, ("chain_uses", str(use.chain_use_id), "boundary_loop_id"), "full host ChainUse requires a boundary loop")
    for loop in snapshot.boundary_loops:
        _require_refs(issues, {loop.patch_domain_id}, domain_ids, ("boundary_loops", str(loop.boundary_loop_id), "patch_domain_id"))
        _require_refs(issues, set(loop.ordered_chain_use_ids), use_ids, ("boundary_loops", str(loop.boundary_loop_id), "ordered_chain_use_ids"))
    for constraint in snapshot.boundary_constraints:
        _require_refs(issues, {constraint.patch_domain_id}, domain_ids, ("boundary_constraints", str(constraint.boundary_constraint_id), "patch_domain_id"))
        if constraint.originating_chain_use_id is not None:
            _require_refs(issues, {constraint.originating_chain_use_id}, use_ids, ("boundary_constraints", str(constraint.boundary_constraint_id), "originating_chain_use_id"))
    for sector in snapshot.angular_owner_sectors:
        _require_refs(issues, {sector.owner_patch_id}, patch_ids, ("angular_owner_sectors", str(sector.owner_sector_id), "owner_patch_id"))
        _require_refs(issues, {sector.patch_domain_id}, domain_ids, ("angular_owner_sectors", str(sector.owner_sector_id), "patch_domain_id"))
        _require_refs(issues, set(sector.ordered_incident_chain_use_ids), use_ids, ("angular_owner_sectors", str(sector.owner_sector_id), "ordered_incident_chain_use_ids"))
        if len(sector.ordered_incident_chain_use_ids) < 2:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("angular_owner_sectors", str(sector.owner_sector_id)), "ordered incident supports are required")
        elif (
            sector.incoming_support_ref.chain_use_id != sector.ordered_incident_chain_use_ids[0]
            or sector.outgoing_support_ref.chain_use_id != sector.ordered_incident_chain_use_ids[-1]
        ):
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("angular_owner_sectors", str(sector.owner_sector_id)), "incoming/outgoing refs must match ordered supports")
        _require_refs(issues, {sector.incoming_support_ref.source_launch_boundary_id, sector.outgoing_support_ref.source_launch_boundary_id}, boundary_ids, ("angular_owner_sectors", str(sector.owner_sector_id), "support_refs"))
    for certificate in snapshot.reflex_angle_certificates:
        _require_refs(issues, {certificate.owner_sector_id}, angular_sector_ids, ("reflex_angle_certificates", str(certificate.certificate_id), "owner_sector_id"))
        if certificate.exact_two_pi:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("reflex_angle_certificates", str(certificate.certificate_id), "exact_two_pi"), "2*pi is terminal/junction, never Angular")
    for relation in snapshot.corner_relations:
        _require_refs(issues, {relation.source_vertex_id}, vertex_ids, ("corner_relations", str(relation.corner_relation_id), "source_vertex_id"))
        _require_refs(issues, {relation.owner_sector_id}, angular_sector_ids, ("corner_relations", str(relation.corner_relation_id), "owner_sector_id"))
        _require_refs(issues, {relation.reflex_angle_certificate_id}, angle_certificate_ids, ("corner_relations", str(relation.corner_relation_id), "reflex_angle_certificate_id"))
        if relation.exact_two_pi:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("corner_relations", str(relation.corner_relation_id)), "2*pi CornerRelation is forbidden")
    for relation in snapshot.junction_relations:
        _require_refs(issues, {relation.source_vertex_id}, vertex_ids, ("junction_relations", str(relation.junction_relation_id), "source_vertex_id"))
        _require_refs(issues, set(relation.incident_chain_use_ids), use_ids, ("junction_relations", str(relation.junction_relation_id), "incident_chain_use_ids"))
    return tuple(issues)


def validate_decal_request(request: DecalRequestV1) -> tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []
    expected = (
        (request.schema_version == DECAL_REQUEST_SCHEMA_V1, "schema_version"),
        (request.metric_space is MetricSpace.SOURCE_LOCAL_INTRINSIC, "metric_space"),
        (request.angular_profile_family_id is AngularProfileFamilyId.LINEAR_REFLEX_EQUAL_V1, "angular_profile_family_id"),
        (request.angular_profile_selection_policy_id is AngularProfileSelectionPolicyId.MIN_K_FOR_MAX_SUBTURN_V1, "angular_profile_selection_policy_id"),
        (request.max_subturn_parameter_id is MaxSubturnParameterId.LINEAR_REFLEX_MAX_SUBTURN_V1, "max_subturn_parameter_id"),
        (request.max_subturn_value_id is MaxSubturnValueId.LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1, "max_subturn_value_id"),
        (request.max_subturn_exact_value.symbol is ExactAngleSymbol.PI_OVER_3, "max_subturn_exact_value"),
        (request.cap_policy_id is CapPolicyId.PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1, "cap_policy_id"),
        (request.boundary_policy_id is BoundaryPolicyId.BOUNDARY_LIMITED_PROPAGATION, "boundary_policy_id"),
        (request.interaction_policy_id is InteractionPolicyId.INTRAPATCH_POLICY_B_V1, "interaction_policy_id"),
        (request.ownership_policy_id is OwnershipPolicyId.TOTAL_DISJOINT_RESOLVED_COVERAGE_V1, "ownership_policy_id"),
    )
    for valid, field in expected:
        if not valid:
            code = ValidationCode.SCHEMA_VERSION if field == "schema_version" else ValidationCode.POLICY_MISMATCH
            _issue(issues, code, (field,), "unsupported v1 value")
    if not request.selected_chain_use_ids:
        _issue(issues, ValidationCode.MISSING_REFERENCE, ("selected_chain_use_ids",), "at least one ChainUse is required")
    return tuple(issues)


def _seed_id(seed: object) -> OpaqueId:
    return seed.seed_id


def _spec_variant(spec: object) -> EnvelopeSpecVariant:
    if isinstance(spec, StripEnvelopeSpec):
        return EnvelopeSpecVariant.STRIP
    if isinstance(spec, AngularEnvelopeSpec):
        return EnvelopeSpecVariant.ANGULAR
    if isinstance(spec, JunctionEnvelopeSpec):
        return EnvelopeSpecVariant.JUNCTION
    if isinstance(spec, CapEnvelopeSpec):
        return EnvelopeSpecVariant.CAP
    raise TypeError(type(spec).__name__)


def validate_compiled_plan(plan: CompiledPatchEvaluationPlanV1) -> tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []
    if plan.schema_version != COMPILED_PLAN_SCHEMA_V1:
        _issue(issues, ValidationCode.SCHEMA_VERSION, ("schema_version",), "unsupported plan schema")
    request_id = plan.plan_key.decal_request_id
    domain_id = plan.plan_key.patch_domain_id
    seed_ids = {_seed_id(seed) for seed in plan.seeds}
    if len(seed_ids) != len(plan.seeds):
        _issue(issues, ValidationCode.DUPLICATE_ID, ("seeds",), "duplicate seed ID")
    component_ids = _check_unique(issues, plan.front_components, "front_component_id", "front_components")
    certificate_ids = _check_unique(issues, plan.angular_profile_selection_certificates, "certificate_id", "angular_profile_selection_certificates")
    spec_ids = _check_unique(issues, plan.envelope_specs, "envelope_spec_id", "envelope_specs")
    instance_ids = _check_unique(issues, plan.envelope_instances, "instance_id", "envelope_instances")
    predicate_ids = _check_unique(issues, plan.event_predicates, "predicate_id", "event_predicates")

    scoped_records = (
        list(plan.seeds)
        + list(plan.front_components)
        + list(plan.angular_profile_selection_certificates)
        + list(plan.envelope_specs)
        + list(plan.envelope_instances)
        + list(plan.event_predicates)
        + list(plan.event_transitions)
        + list(plan.interactions)
    )
    for record in scoped_records:
        if record.decal_request_id != request_id or record.patch_domain_id != domain_id:
            _issue(issues, ValidationCode.PLAN_KEY_MISMATCH, (type(record).__name__,), "record request/domain differs from plan key")

    front_seed_ids = {seed.seed_id for seed in plan.seeds if isinstance(seed, FrontSeedV1)}
    corner_seed_ids = {seed.seed_id for seed in plan.seeds if isinstance(seed, CornerSeedV1)}
    junction_seed_ids = {seed.seed_id for seed in plan.seeds if isinstance(seed, JunctionSeedV1)}
    cap_seed_ids = {seed.seed_id for seed in plan.seeds if isinstance(seed, CapSeedV1)}
    for component in plan.front_components:
        _require_refs(issues, {component.front_seed_id}, front_seed_ids, ("front_components", str(component.front_component_id), "front_seed_id"))
        if component.initial_branch_count != 1:
            _issue(issues, ValidationCode.POLICY_MISMATCH, ("front_components", str(component.front_component_id), "initial_branch_count"), "ordinary component starts with one branch")

    certificate_by_id = {certificate.certificate_id: certificate for certificate in plan.angular_profile_selection_certificates}
    for certificate in plan.angular_profile_selection_certificates:
        k = certificate.resolved_hidden_edge_count
        interval = certificate.selection_interval_certificate
        if k < 0 or certificate.resolved_subturn_count != k + 1 or certificate.local_profile_support_count != k + 2 or certificate.local_profile_segment_count != k + 2:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("angular_profile_selection_certificates", str(certificate.certificate_id)), "k+1/k+2 cardinality law violated")
        if interval.lower_bound_integer != k or interval.upper_bound_integer != k + 1:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("angular_profile_selection_certificates", str(certificate.certificate_id), "selection_interval_certificate"), "certificate must encode open k < ratio <= k+1")

    spec_by_id = {spec.envelope_spec_id: spec for spec in plan.envelope_specs}
    for spec in plan.envelope_specs:
        path = ("envelope_specs", str(spec.envelope_spec_id))
        if isinstance(spec, StripEnvelopeSpec):
            _require_refs(issues, {spec.source_seed_id}, front_seed_ids, path + ("source_seed_id",))
            _require_refs(issues, set(spec.front_component_ids), component_ids, path + ("front_component_ids",))
            _require_refs(issues, set(spec.terminal_interface_spec_ids), spec_ids, path + ("terminal_interface_spec_ids",))
        elif isinstance(spec, AngularEnvelopeSpec):
            _require_refs(issues, {spec.source_seed_id}, corner_seed_ids, path + ("source_seed_id",))
            _require_refs(issues, {spec.selection_certificate_id}, certificate_ids, path + ("selection_certificate_id",))
            _require_refs(issues, set(spec.incident_front_component_ids), component_ids, path + ("incident_front_component_ids",))
            certificate = certificate_by_id.get(spec.selection_certificate_id)
            if certificate is not None:
                k = certificate.resolved_hidden_edge_count
                if spec.resolved_hidden_edge_count != k or len(spec.hidden_supports) != k:
                    _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path, "spec k must equal selection certificate and hidden support count")
                expected_ordinals = set(range(1, k + 1))
                if {item.ordinal for item in spec.hidden_supports} != expected_ordinals:
                    _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path + ("hidden_supports",), "hidden support ordinals must be exactly 1..k")
                for support in spec.hidden_supports:
                    if support.turn_fraction.numerator != support.ordinal or support.turn_fraction.denominator != k + 1:
                        _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path + ("hidden_supports", str(support.ordinal)), "turn fraction must be ordinal/(k+1)")
            if spec.all_support_normal_speed != 1:
                _issue(issues, ValidationCode.POLICY_MISMATCH, path + ("all_support_normal_speed",), "v1 support speed is UNIT")
        elif isinstance(spec, JunctionEnvelopeSpec):
            _require_refs(issues, {spec.source_seed_id}, junction_seed_ids, path + ("source_seed_id",))
            _require_refs(issues, set(spec.incident_front_component_ids), component_ids, path + ("incident_front_component_ids",))
        elif isinstance(spec, CapEnvelopeSpec):
            _require_refs(issues, {spec.source_seed_id}, cap_seed_ids, path + ("source_seed_id",))
            _require_refs(issues, {spec.incident_strip_spec_id}, spec_ids, path + ("incident_strip_spec_id",))

    for instance in plan.envelope_instances:
        path = ("envelope_instances", str(instance.instance_id))
        _require_refs(issues, {instance.spec_id}, spec_ids, path + ("spec_id",))
        _require_refs(issues, set(instance.effective_alpha_binding.front_component_ids), component_ids, path + ("effective_alpha_binding",))
        spec = spec_by_id.get(instance.spec_id)
        if spec is not None and instance.spec_variant is not _spec_variant(spec):
            _issue(issues, ValidationCode.SEED_VARIANT_MISMATCH, path + ("spec_variant",), "instance variant differs from spec")

    if plan.initial_front_spec.decal_request_id != request_id or plan.initial_front_spec.patch_domain_id != domain_id:
        _issue(issues, ValidationCode.PLAN_KEY_MISMATCH, ("initial_front_spec",), "front spec differs from plan key")
    for feature in plan.initial_front_spec.support_features:
        source = feature.source_id
        if isinstance(source, EnvelopeSpecId):
            _require_refs(issues, {source}, spec_ids, ("initial_front_spec", "support_features"))
        elif isinstance(source, HiddenSupportId):
            hidden_ids = {
                support.hidden_support_id
                for spec in plan.envelope_specs
                if isinstance(spec, AngularEnvelopeSpec)
                for support in spec.hidden_supports
            }
            _require_refs(issues, {source}, hidden_ids, ("initial_front_spec", "support_features"))
        if feature.kind is InitialFrontFeatureKind.ANGULAR_HIDDEN_SUPPORT and not isinstance(source, HiddenSupportId):
            _issue(issues, ValidationCode.SEED_VARIANT_MISMATCH, ("initial_front_spec",), "angular hidden feature requires HiddenSupportId")

    expected_participant_types = {
        EventParticipantKind.FRONT_COMPONENT: FrontComponentId,
        EventParticipantKind.ENVELOPE_SPEC: EnvelopeSpecId,
        EventParticipantKind.ENVELOPE_INSTANCE: EnvelopeInstanceId,
        EventParticipantKind.HIDDEN_SUPPORT: HiddenSupportId,
        EventParticipantKind.BOUNDARY_CONSTRAINT: BoundaryConstraintId,
    }
    for predicate in plan.event_predicates:
        for participant in predicate.participants:
            expected_type = expected_participant_types[participant.kind]
            if not isinstance(participant.participant_id, expected_type):
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("event_predicates", str(predicate.predicate_id), "participants"), "participant kind and typed ID differ")
    for transition in plan.event_transitions:
        _require_refs(issues, {transition.predicate_id}, predicate_ids, ("event_transitions", str(transition.transition_id), "predicate_id"))

    if plan.raw_coverage.decal_request_id != request_id or plan.raw_coverage.patch_domain_id != domain_id:
        _issue(issues, ValidationCode.PLAN_KEY_MISMATCH, ("raw_coverage",), "raw coverage differs from plan key")
    _require_refs(issues, set(plan.raw_coverage.envelope_instance_ids), instance_ids, ("raw_coverage", "envelope_instance_ids"))
    if plan.resolved_coverage.raw_coverage_id != plan.raw_coverage.coverage_id:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("resolved_coverage", "raw_coverage_id"), "resolved coverage must reference raw coverage")
    interaction_ids = _values(plan.interactions, "interaction_id")
    if plan.resolved_coverage.interaction_ids != interaction_ids:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("resolved_coverage", "interaction_ids"), "resolved coverage interaction set differs")
    for interaction in plan.interactions:
        _require_refs(issues, set(interaction.participant_envelope_instance_ids), instance_ids, ("interactions", str(interaction.interaction_id), "participants"))
        _require_refs(issues, {interaction.event_gate_id}, predicate_ids, ("interactions", str(interaction.interaction_id), "event_gate_id"))
        if interaction.creates_new_matter:
            _issue(issues, ValidationCode.POLICY_MISMATCH, ("interactions", str(interaction.interaction_id), "creates_new_matter"), "policy B never creates new matter")
        if interaction.coverage_effect is CoverageEffect.NONE:
            _issue(issues, ValidationCode.POLICY_MISMATCH, ("interactions", str(interaction.interaction_id), "coverage_effect"), "declared interaction must state a clipping effect")

    ownership = plan.ownership
    if ownership.coverage_id != plan.resolved_coverage.coverage_id:
        _issue(issues, ValidationCode.OWNERSHIP_DECLARATION, ("ownership", "coverage_id"), "ownership must partition resolved coverage")
    ownership_contract_valid = (
        ownership.partition_contract_id is OwnershipPartitionContractId.TOTAL_DISJOINT_RESOLVED_COVERAGE_V1
        and ownership.total
        and ownership.disjoint
        and ownership.silhouette_effect is SilhouetteEffect.NONE
        and ownership.equality_boundary_owner is EqualityBoundaryOwner.NONE
        and ownership.named_unproven_outcome is NamedOutcome.OWNERSHIP_PARTITION_UNPROVEN
        and ownership.claim_interiors_obligation is ClaimInteriorsObligation.PAIRWISE_DISJOINT
        and ownership.claim_closure_obligation is ClaimClosureObligation.EQUALS_RESOLVED_COVERAGE
    )
    if not ownership_contract_valid:
        _issue(issues, ValidationCode.OWNERSHIP_DECLARATION, ("ownership",), "v1 total/disjoint declarations are incomplete")
    claim_ids = _check_unique(issues, ownership.claims, "ownership_claim_id", "ownership.claims")
    del claim_ids
    for claim in ownership.claims:
        if claim.coverage_id != ownership.coverage_id:
            _issue(issues, ValidationCode.OWNERSHIP_DECLARATION, ("ownership", "claims", str(claim.ownership_claim_id), "coverage_id"), "claim points to foreign coverage")
        _require_refs(issues, set(claim.envelope_spec_ids), spec_ids, ("ownership", "claims", str(claim.ownership_claim_id), "envelope_spec_ids"))
        _require_refs(issues, set(claim.envelope_instance_ids), instance_ids, ("ownership", "claims", str(claim.ownership_claim_id), "envelope_instance_ids"))

    tessellation = plan.tessellation_plan
    required_forbidden = set(ForbiddenTessellationOperation)
    tessellation_valid = (
        tessellation.contract_id is TessellationContractId.DOWNSTREAM_TESSELLATION_AFTER_SEMANTIC_COALESCING_V1
        and tessellation.starts_after is TessellationStartStage.SEMANTIC_COALESCING
        and tessellation.shared_vertex_key_policy is SharedVertexKeyPolicy.SEMANTIC_IDENTITY_NOT_COORDINATE
        and tessellation.semantic_digest_equivalence is TessellationDigestEquivalence.ALL_VALID_TESSELLATIONS_SHARE_ONE_SEMANTIC_DIGEST
        and set(tessellation.forbidden_operations) == required_forbidden
    )
    if not tessellation_valid:
        _issue(issues, ValidationCode.TESSELLATION_AUTHORITY, ("tessellation_plan",), "downstream authority boundary is incomplete")
    if plan.geometry_batch_provenance.resolved_coverage_id != plan.resolved_coverage.coverage_id:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("geometry_batch_provenance", "resolved_coverage_id"), "batch provenance must reference resolved coverage")
    return tuple(issues)


def validate_geometry_batch(batch: GeometryBatchV1) -> tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []
    if batch.schema_version != GEOMETRY_BATCH_SCHEMA_V1:
        _issue(issues, ValidationCode.SCHEMA_VERSION, ("schema_version",), "unsupported GeometryBatch schema")
    vertex_keys = _check_unique(issues, batch.vertices, "vert_key", "vertices")
    region_ids = _check_unique(issues, batch.semantic_regions, "semantic_region_id", "semantic_regions")
    face_ids = [face.face_id for face in batch.faces]
    if len(face_ids) != len(set(face_ids)):
        _issue(issues, ValidationCode.DUPLICATE_ID, ("faces",), "duplicate face ID")
    ownership_claim_ids = {region.ownership_claim_id for region in batch.semantic_regions}
    for vertex in batch.vertices:
        lowered = vertex.vert_key.value.lower()
        if lowered.startswith(("coord:", "xyz:", "rounded:")):
            _issue(issues, ValidationCode.FORBIDDEN_TOPOLOGY_IDENTITY, ("vertices", str(vertex.vert_key), "vert_key"), "VertexKey must not be coordinate-derived")
    for face in batch.faces:
        _require_refs(issues, set(face.ordered_vert_keys), vertex_keys, ("faces", str(face.face_id), "ordered_vert_keys"))
        _require_refs(issues, {face.semantic_region_id}, region_ids, ("faces", str(face.face_id), "semantic_region_id"))
        _require_refs(issues, {face.ownership_claim_id}, ownership_claim_ids, ("faces", str(face.face_id), "ownership_claim_id"))
    for chain in batch.boundary_chains:
        _require_refs(issues, set(chain.ordered_vert_keys), vertex_keys, ("boundary_chains", str(chain.semantic_boundary_id), "ordered_vert_keys"))
    for chain in batch.interface_chains:
        _require_refs(issues, set(chain.ordered_vert_keys), vertex_keys, ("interface_chains", str(chain.semantic_interface_id), "ordered_vert_keys"))
    expected_digest = geometry_batch_semantic_digest(batch).sha256_hex
    if batch.semantic_digest != SemanticDigestValue(expected_digest):
        _issue(issues, ValidationCode.GEOMETRY_BATCH, ("semantic_digest",), "declared semantic digest differs from semantic projection")
    return tuple(issues)


def validate_cross_contract_references(
    snapshot: AnalysisSnapshotV1,
    request: DecalRequestV1,
    plans: tuple[CompiledPatchEvaluationPlanV1, ...],
    geometry_batches: tuple[GeometryBatchV1, ...] = (),
) -> tuple[ValidationIssue, ...]:
    issues = list(validate_analysis_snapshot(snapshot))
    issues.extend(validate_decal_request(request))
    use_ids = _values(snapshot.chain_uses, "chain_use_id")
    domain_ids = _values(snapshot.patch_domains, "patch_domain_id")
    _require_refs(issues, set(request.selected_chain_use_ids), use_ids, ("request", "selected_chain_use_ids"))
    plan_keys: set[tuple[object, object]] = set()
    for plan in plans:
        issues.extend(validate_compiled_plan(plan))
        key = (plan.plan_key.decal_request_id, plan.plan_key.patch_domain_id)
        if key in plan_keys:
            _issue(issues, ValidationCode.DUPLICATE_ID, ("plans",), "duplicate request/domain plan key")
        plan_keys.add(key)
        if plan.source_revision != snapshot.source_revision:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("plans", str(plan.evaluation_plan_id), "source_revision"), "plan and snapshot revisions differ")
        if plan.plan_key.decal_request_id != request.decal_request_id:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("plans", str(plan.evaluation_plan_id), "decal_request_id"), "plan belongs to another request")
        _require_refs(issues, {plan.plan_key.patch_domain_id}, domain_ids, ("plans", str(plan.evaluation_plan_id), "patch_domain_id"))
    plan_by_key = {(plan.plan_key.decal_request_id, plan.plan_key.patch_domain_id): plan for plan in plans}
    for batch in geometry_batches:
        issues.extend(validate_geometry_batch(batch))
        key = (batch.decal_request_id, batch.patch_domain_id)
        if key not in plan_by_key:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("geometry_batches",), "batch has no compiled plan")
        if batch.source_revision != snapshot.source_revision:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("geometry_batches", batch.patch_domain_id.value, "source_revision"), "batch and snapshot revisions differ")
    return tuple(issues)
