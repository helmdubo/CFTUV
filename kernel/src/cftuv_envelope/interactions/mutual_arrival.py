"""Exact mutual-arrival certificates; overlap alone is never an event."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations, product

from ..reference.common import stable_id
from ..reference.planar_types import CertifiedPredicateUndecidable, exact_sign
from .arrival import (
    active_domain_certificate,
    front_arrival_reading,
)
from .contracts import (
    AngularProfileArrivalModelV1,
    ArrivalModelV1,
    CapArrivalModelV1,
    InteractionCandidateKind,
    InteractionCandidateV1,
    InteractionDiagnosticSeverity,
    InteractionDiagnosticV1,
    InteractionOutcome,
    MutualArrivalCertificateV1,
    StripArrivalModelV1,
    UnsupportedJunctionArrivalModelV1,
)
from .equality_locus import (
    ClippedEqualityLocusV1,
    active_domains_share_closure,
    clip_equality_locus_to_active_domains,
    public_equality_locus,
    restrict_clipped_locus_to_active_segments,
)


SupportedArrivalModelV1 = (
    StripArrivalModelV1
    | AngularProfileArrivalModelV1
    | CapArrivalModelV1
)


@dataclass(frozen=True, slots=True)
class ProvenInteractionV1:
    candidate: InteractionCandidateV1
    left_model: SupportedArrivalModelV1
    right_model: SupportedArrivalModelV1
    mutual_arrival_certificate: MutualArrivalCertificateV1
    equality_locus: object
    clipped_locus: ClippedEqualityLocusV1


def _model_priority(model: SupportedArrivalModelV1) -> int:
    if isinstance(model, AngularProfileArrivalModelV1):
        return 0
    if isinstance(model, CapArrivalModelV1):
        return 1
    return 2


def _line_signature(clipped: ClippedEqualityLocusV1) -> tuple[str, str, str]:
    return (
        clipped.line.normal_x.expression,
        clipped.line.normal_y.expression,
        clipped.line.constant.expression,
    )


def _model_pairs(
    candidate: InteractionCandidateV1,
    models: tuple[ArrivalModelV1, ...],
) -> tuple[tuple[SupportedArrivalModelV1, SupportedArrivalModelV1], ...]:
    supported = tuple(
        item
        for item in models
        if not isinstance(item, UnsupportedJunctionArrivalModelV1)
    )
    if candidate.candidate_kind is InteractionCandidateKind.EXPLICIT_SELF_CONTACT:
        participants = tuple(
            item
            for item in supported
            if item.interaction_component_id == candidate.left_component_id
        )
        return tuple(
            (left, right)
            for left, right in combinations(participants, 2)
            if left.front_reading_id != right.front_reading_id
        )
    left = tuple(
        item
        for item in supported
        if item.interaction_component_id == candidate.left_component_id
    )
    right = tuple(
        item
        for item in supported
        if item.interaction_component_id == candidate.right_component_id
    )
    return tuple(product(left, right))


def prove_mutual_arrivals(
    candidates: tuple[InteractionCandidateV1, ...],
    arrival_models: tuple[ArrivalModelV1, ...],
) -> tuple[tuple[ProvenInteractionV1, ...], tuple[InteractionDiagnosticV1, ...]]:
    proofs = []
    diagnostics = []
    for candidate in candidates:
        pairs = _model_pairs(candidate, arrival_models)
        if (
            candidate.candidate_kind
            is InteractionCandidateKind.EXPLICIT_SELF_CONTACT
            and not pairs
        ):
            diagnostics.append(
                InteractionDiagnosticV1(
                    InteractionOutcome.SELF_CONTACT_READING_IDENTITY_UNPROVEN,
                    InteractionDiagnosticSeverity.UNSUPPORTED,
                    "explicit self-contact requires two distinct active front readings",
                    frozenset({candidate.left_component_id}),
                )
            )
            continue
        pair_proofs = []
        coincident_pair = False
        for left_model, right_model in pairs:
            try:
                clipped = clip_equality_locus_to_active_domains(
                    left_model.arrival_law,
                    right_model.arrival_law,
                    left_model.active_regions,
                    right_model.active_regions,
                    getattr(
                        left_model,
                        "component_profile_arrival_laws",
                        (),
                    ),
                    getattr(
                        right_model,
                        "component_profile_arrival_laws",
                        (),
                    ),
                )
            except ValueError as exc:
                if "coincident arrival laws" in str(exc):
                    coincident_pair = coincident_pair or active_domains_share_closure(
                        left_model.active_regions,
                        right_model.active_regions,
                    )
                    continue
                diagnostics.append(
                    InteractionDiagnosticV1(
                        InteractionOutcome.INTERACTION_INPUT_CONTRACT_INVALID,
                        InteractionDiagnosticSeverity.UNSUPPORTED,
                        str(exc),
                        frozenset(
                            {
                                candidate.left_component_id,
                                candidate.right_component_id,
                            }
                        ),
                        frozenset(
                            {
                                left_model.arrival_model_id,
                                right_model.arrival_model_id,
                            }
                        ),
                    )
                )
                continue
            except CertifiedPredicateUndecidable as exc:
                diagnostics.append(
                    InteractionDiagnosticV1(
                        InteractionOutcome.INTERACTION_EXACT_PREDICATE_UNDECIDABLE,
                        InteractionDiagnosticSeverity.UNSUPPORTED,
                        str(exc),
                        frozenset(
                            {
                                candidate.left_component_id,
                                candidate.right_component_id,
                            }
                        ),
                        frozenset(
                            {
                                left_model.arrival_model_id,
                                right_model.arrival_model_id,
                            }
                        ),
                    )
                )
                continue
            if clipped is None:
                continue
            if isinstance(left_model, CapArrivalModelV1):
                clipped = restrict_clipped_locus_to_active_segments(
                    clipped,
                    left_model.arrival_law,
                    left_model.active_segments,
                )
            if clipped is not None and isinstance(
                right_model, CapArrivalModelV1
            ):
                clipped = restrict_clipped_locus_to_active_segments(
                    clipped,
                    right_model.arrival_law,
                    right_model.active_segments,
                )
            if clipped is None:
                continue
            event_alpha = clipped.exact_first_alpha.as_expr()
            if exact_sign(event_alpha) < 0:
                continue
            if (
                exact_sign(
                    left_model.effective_alpha.as_expr() - event_alpha
                )
                < 0
                or exact_sign(
                    right_model.effective_alpha.as_expr() - event_alpha
                )
                < 0
            ):
                continue
            if (
                not left_model.reachability.source_launch_reachable
                or not right_model.reachability.source_launch_reachable
                or left_model.reachability.bypass_used
                or right_model.reachability.bypass_used
            ):
                continue
            left_reading = front_arrival_reading(left_model)
            right_reading = front_arrival_reading(right_model)
            batch_id = stable_id(
                "same-alpha-interaction-batch",
                candidate.decal_request_id,
                candidate.patch_domain_id,
                clipped.exact_first_alpha.expression,
            )
            certificate_id = stable_id(
                "mutual-arrival-certificate",
                candidate.candidate_id,
                left_reading.front_reading_id,
                right_reading.front_reading_id,
                clipped.exact_first_alpha.expression,
                *_line_signature(clipped),
            )
            certificate = MutualArrivalCertificateV1(
                certificate_id=certificate_id,
                candidate_id=candidate.candidate_id,
                left_front_reading=left_reading,
                right_front_reading=right_reading,
                exact_alpha=clipped.exact_first_alpha,
                arrival_laws=(
                    left_model.arrival_law,
                    right_model.arrival_law,
                ),
                active_domain_certificates=(
                    active_domain_certificate(left_model, True),
                    active_domain_certificate(right_model, True),
                ),
                reachability_certificates=(
                    left_model.reachability,
                    right_model.reachability,
                ),
                same_alpha_batch_identity=batch_id,
            )
            pair_proofs.append(
                ProvenInteractionV1(
                    candidate=candidate,
                    left_model=left_model,
                    right_model=right_model,
                    mutual_arrival_certificate=certificate,
                    equality_locus=public_equality_locus(
                        clipped,
                        left_reading.front_reading_id,
                        right_reading.front_reading_id,
                        certificate_id,
                    ),
                    clipped_locus=clipped,
                )
            )
        if not pair_proofs:
            if coincident_pair:
                diagnostics.append(
                    InteractionDiagnosticV1(
                        InteractionOutcome.INTERACTION_COINCIDENT_ARRIVAL_LAWS_UNPROVEN,
                        InteractionDiagnosticSeverity.UNSUPPORTED,
                        "coincident arrival laws do not define a unique Policy B side",
                        frozenset(
                            {
                                candidate.left_component_id,
                                candidate.right_component_id,
                            }
                        ),
                    )
                )
            continue
        earliest = pair_proofs[0].mutual_arrival_certificate.exact_alpha.as_expr()
        for proof in pair_proofs[1:]:
            alpha = proof.mutual_arrival_certificate.exact_alpha.as_expr()
            if exact_sign(alpha - earliest) < 0:
                earliest = alpha
        earliest_proofs = tuple(
            proof
            for proof in pair_proofs
            if exact_sign(
                proof.mutual_arrival_certificate.exact_alpha.as_expr()
                - earliest
            )
            == 0
        )
        by_signature: dict[
            tuple[str, str, str], list[ProvenInteractionV1]
        ] = {}
        for proof in earliest_proofs:
            by_signature.setdefault(
                _line_signature(proof.clipped_locus), []
            ).append(proof)
        selected = tuple(
            min(
                items,
                key=lambda proof: (
                    -len(proof.clipped_locus.segments),
                    _model_priority(proof.left_model)
                    + _model_priority(proof.right_model),
                    proof.left_model.arrival_model_id,
                    proof.right_model.arrival_model_id,
                ),
            )
            for items in by_signature.values()
        )
        if len(selected) > 1:
            left_models = {item.left_model.arrival_model_id for item in selected}
            right_models = {
                item.right_model.arrival_model_id for item in selected
            }
            left_is_profile = all(
                isinstance(item.left_model, AngularProfileArrivalModelV1)
                for item in selected
            )
            right_is_profile = all(
                isinstance(item.right_model, AngularProfileArrivalModelV1)
                for item in selected
            )
            piecewise_profile = (
                left_is_profile and len(right_models) == 1
            ) or (right_is_profile and len(left_models) == 1)
            if not piecewise_profile:
                diagnostics.append(
                    InteractionDiagnosticV1(
                        InteractionOutcome.INTERACTION_POLICY_B_PARTITION_UNPROVEN,
                        InteractionDiagnosticSeverity.UNSUPPORTED,
                        "multiple first-arrival loci are not one exposed Angular profile",
                        frozenset(
                            {
                                candidate.left_component_id,
                                candidate.right_component_id,
                            }
                        ),
                        frozenset(
                            {
                                model.arrival_model_id
                                for proof in selected
                                for model in (
                                    proof.left_model,
                                    proof.right_model,
                                )
                            }
                        ),
                    )
                )
                continue
        proofs.extend(selected)

    by_batch: dict[str, list[ProvenInteractionV1]] = {}
    for proof in proofs:
        by_batch.setdefault(
            proof.mutual_arrival_certificate.same_alpha_batch_identity, []
        ).append(proof)
    rejected = set()
    for batch, batch_proofs in by_batch.items():
        degree: dict[str, int] = {}
        unique_pairs = {
            (
                proof.candidate.left_component_id,
                proof.candidate.right_component_id,
            )
            for proof in batch_proofs
        }
        for left_component_id, right_component_id in unique_pairs:
            for component_id in {left_component_id, right_component_id}:
                degree[component_id] = degree.get(component_id, 0) + 1
        multiway_components = frozenset(
            component_id for component_id, count in degree.items() if count > 1
        )
        if not multiway_components:
            continue
        rejected.add(batch)
        diagnostics.append(
            InteractionDiagnosticV1(
                InteractionOutcome.MULTIWAY_INTERACTION_POLICY_UNPROVEN,
                InteractionDiagnosticSeverity.UNSUPPORTED,
                "same-alpha multiway meeting has no order-independent v1 rule",
                multiway_components,
            )
        )
    proofs = [
        proof
        for proof in proofs
        if proof.mutual_arrival_certificate.same_alpha_batch_identity
        not in rejected
    ]
    return tuple(
        sorted(
            proofs,
            key=lambda item: (
                item.mutual_arrival_certificate.same_alpha_batch_identity,
                item.candidate.candidate_id,
            ),
        )
    ), tuple(diagnostics)
