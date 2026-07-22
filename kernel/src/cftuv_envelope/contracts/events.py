"""Compile-time front/event declarations; scheduler здесь отсутствует."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import (
    BoundaryConstraintId,
    DecalRequestId,
    EnvelopeInstanceId,
    EnvelopeSpecId,
    EventPredicateId,
    EventTransitionId,
    FrontComponentId,
    HiddenSupportId,
    LineageId,
    PatchDomainId,
    SameAlphaBatchKeyId,
    TopologyStateVersionId,
)
from ..outcomes import NamedOutcome


class InitialFrontFeatureKind(str, Enum):
    STRIP_SUPPORT = "STRIP_SUPPORT"
    ANGULAR_HIDDEN_SUPPORT = "ANGULAR_HIDDEN_SUPPORT"
    JUNCTION_SUPPORT = "JUNCTION_SUPPORT"
    CAP_CLOSURE = "CAP_CLOSURE"


InitialFrontSourceId = EnvelopeSpecId | HiddenSupportId


@dataclass(frozen=True, slots=True)
class InitialFrontFeatureRefV1:
    kind: InitialFrontFeatureKind
    source_id: InitialFrontSourceId


@dataclass(frozen=True, slots=True)
class InitialFrontSpec:
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    support_features: frozenset[InitialFrontFeatureRefV1]
    fixed_constraint_ids: frozenset[BoundaryConstraintId]


class EventPredicateKind(str, Enum):
    HIDDEN_EDGE_ACTIVATION = "HIDDEN_EDGE_ACTIVATION"
    FRONT_EDGE_COLLAPSE = "FRONT_EDGE_COLLAPSE"
    CAP_COLLAPSE = "CAP_COLLAPSE"
    BOUNDARY_CONTACT = "BOUNDARY_CONTACT"
    ENDPOINT_SLIDE_START = "ENDPOINT_SLIDE_START"
    BARRIER_SPLIT_REQUIRED = "BARRIER_SPLIT_REQUIRED"
    FRONT_EXHAUSTED = "FRONT_EXHAUSTED"
    EDGE_EDGE_CONTACT = "EDGE_EDGE_CONTACT"
    VERTEX_EDGE_CONTACT = "VERTEX_EDGE_CONTACT"
    COVERAGE_COMPONENT_MERGE = "COVERAGE_COMPONENT_MERGE"
    COVERAGE_COMPONENT_SPLIT = "COVERAGE_COMPONENT_SPLIT"
    SILHOUETTE_CHANGE = "SILHOUETTE_CHANGE"
    MUTUAL_ARRIVAL = "MUTUAL_ARRIVAL"
    FREEZE = "FREEZE"
    CAPACITY = "CAPACITY"
    DIVIDER_BIRTH = "DIVIDER_BIRTH"
    DIVIDER_DEATH = "DIVIDER_DEATH"
    UV_INTERFACE_CHANGE = "UV_INTERFACE_CHANGE"


class EventParticipantKind(str, Enum):
    FRONT_COMPONENT = "FRONT_COMPONENT"
    ENVELOPE_SPEC = "ENVELOPE_SPEC"
    ENVELOPE_INSTANCE = "ENVELOPE_INSTANCE"
    HIDDEN_SUPPORT = "HIDDEN_SUPPORT"
    BOUNDARY_CONSTRAINT = "BOUNDARY_CONSTRAINT"


EventParticipantId = (
    FrontComponentId
    | EnvelopeSpecId
    | EnvelopeInstanceId
    | HiddenSupportId
    | BoundaryConstraintId
)


@dataclass(frozen=True, slots=True)
class EventParticipantRefV1:
    kind: EventParticipantKind
    participant_id: EventParticipantId


@dataclass(frozen=True, slots=True)
class EventPredicateDeclarationV1:
    predicate_id: EventPredicateId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    kind: EventPredicateKind
    participants: frozenset[EventParticipantRefV1]
    provenance: frozenset[LineageId]


@dataclass(frozen=True, slots=True)
class EventTransitionDeclarationV1:
    transition_id: EventTransitionId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    predicate_id: EventPredicateId
    input_topology_state_version: TopologyStateVersionId
    output_topology_state_version: TopologyStateVersionId
    same_alpha_batch_key: SameAlphaBatchKeyId
    provenance: frozenset[LineageId]


class EventLedgerContractId(str, Enum):
    LAZY_DETERMINISTIC_EVENT_LEDGER_V1 = "LAZY_DETERMINISTIC_EVENT_LEDGER_V1"


class EventLedgerStartupScope(str, Enum):
    LAWS_PREDICATES_TRANSITIONS_AND_IMMEDIATE_CANDIDATES = (
        "LAWS_PREDICATES_TRANSITIONS_AND_IMMEDIATE_CANDIDATES"
    )


class SuccessorSchedulingPolicy(str, Enum):
    LAZY = "LAZY"


class SameAlphaPolicy(str, Enum):
    ATOMIC_DETERMINISTIC_BATCH = "ATOMIC_DETERMINISTIC_BATCH"


class EventPublishPolicy(str, Enum):
    COMPLETE_EXACT_STATE_ONLY = "COMPLETE_EXACT_STATE_ONLY"


@dataclass(frozen=True, slots=True)
class EventLedgerContractV1:
    contract_id: EventLedgerContractId
    startup_scope: EventLedgerStartupScope
    successor_scheduling: SuccessorSchedulingPolicy
    same_alpha_policy: SameAlphaPolicy
    publish_policy: EventPublishPolicy
    pending_outcome: NamedOutcome

