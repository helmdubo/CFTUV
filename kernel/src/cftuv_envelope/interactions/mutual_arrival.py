"""Exact mutual-arrival certificates; overlap alone is never an event."""

from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import product

from ..ids import (
    MutualArrivalCertificateId,
    SameAlphaInteractionBatchId,
)
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


def _line_signature(clipped: ClippedEqualityLocusV1) -> tuple[str, str, str]:
    values = clipped.line.expressions()
    divisor = next(
        (item for item in values[:2] if exact_sign(item) != 0),
        None,
    )
    if divisor is None:
        return tuple(str(item) for item in values)
    return tuple(str(item / divisor) for item in values)


def _arrival_geometry_signature(model: SupportedArrivalModelV1) -> tuple:
    law = model.arrival_law
    return (
        *law.normal.expressions(),
        law.source_constant.as_expr(),
        law.normal_speed.as_expr(),
    )


def _merge_equivalent_locus_proofs(
    proofs: tuple[ProvenInteractionV1, ...],
) -> ProvenInteractionV1 | None:
    """Merge one exact profile-support proof without selecting provenance by ID."""

    first = proofs[0]
    if all(item == first for item in proofs[1:]):
        return first
    if any(item.candidate != first.candidate for item in proofs[1:]):
        return None
    left_models = tuple(item.left_model for item in proofs)
    right_models = tuple(item.right_model for item in proofs)
    left_varies = len(set(left_models)) > 1
    right_varies = len(set(right_models)) > 1
    if left_varies == right_varies:
        return None
    varying = left_models if left_varies else right_models
    fixed = right_models if left_varies else left_models
    if any(item != fixed[0] for item in fixed[1:]):
        return None
    if len({_arrival_geometry_signature(item) for item in varying}) != 1:
        return None
    angular = tuple(
        item
        for item in varying
        if isinstance(item, AngularProfileArrivalModelV1)
    )
    if len(angular) != 1:
        return None
    angular_model = angular[0]
    if any(
        not isinstance(item, (AngularProfileArrivalModelV1, StripArrivalModelV1))
        or (
            isinstance(item, StripArrivalModelV1)
            and item.front_component_id
            not in angular_model.front_component_ids
        )
        for item in varying
    ):
        return None
    representative = next(
        item
        for item in proofs
        if (
            item.left_model is angular_model
            if left_varies
            else item.right_model is angular_model
        )
    )
    left_readings = frozenset(
        item.mutual_arrival_certificate.left_front_reading
        for item in proofs
    )
    right_readings = frozenset(
        item.mutual_arrival_certificate.right_front_reading
        for item in proofs
    )
    participant_ids = frozenset(
        item.front_reading_id for item in (*left_readings, *right_readings)
    )
    certificate_id = MutualArrivalCertificateId(
        stable_id(
            "mutual-arrival-equivalent-locus-certificate",
            representative.candidate.candidate_id,
            representative.mutual_arrival_certificate.exact_alpha.expression,
            *_line_signature(representative.clipped_locus),
            *(
                item.value
                for item in sorted(
                    participant_ids,
                    key=lambda item: item.value,
                )
            ),
        )
    )
    certificates_by_id = {
        item.certificate_id: item
        for proof in proofs
        for item in proof.mutual_arrival_certificate.active_domain_certificates
    }
    reachability = tuple(
        dict.fromkeys(
            item
            for proof in proofs
            for item in proof.mutual_arrival_certificate.reachability_certificates
        )
    )
    laws = tuple(
        dict.fromkeys(
            item
            for proof in proofs
            for item in proof.mutual_arrival_certificate.arrival_laws
        )
    )
    certificate = replace(
        representative.mutual_arrival_certificate,
        certificate_id=certificate_id,
        equivalent_left_front_readings=left_readings,
        equivalent_right_front_readings=right_readings,
        arrival_laws=laws,
        active_domain_certificates=tuple(
            certificates_by_id[item]
            for item in sorted(
                certificates_by_id,
                key=lambda item: item.value,
            )
        ),
        reachability_certificates=reachability,
    )
    return replace(
        representative,
        mutual_arrival_certificate=certificate,
        equality_locus=public_equality_locus(
            representative.clipped_locus,
            certificate.left_front_reading.front_reading_id,
            certificate.right_front_reading.front_reading_id,
            certificate_id,
            participant_ids,
        ),
    )


def _is_exposed_angular_component_side(
    proofs: tuple[ProvenInteractionV1, ...],
    model_side: str,
) -> bool:
    models = tuple(getattr(item, model_side) for item in proofs)
    angular_models = tuple(
        item
        for item in models
        if isinstance(item, AngularProfileArrivalModelV1)
    )
    if not angular_models:
        return False
    incident_front_ids = frozenset().union(
        *(item.front_component_ids for item in angular_models)
    )
    return all(
        isinstance(item, AngularProfileArrivalModelV1)
        or (
            isinstance(item, (StripArrivalModelV1, CapArrivalModelV1))
            and item.front_component_id in incident_front_ids
        )
        for item in models
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
        declared_readings = candidate.participant_front_reading_ids
        if declared_readings is None:
            return ()
        left_models = tuple(
            item
            for item in supported
            if isinstance(item, StripArrivalModelV1)
            and item.interaction_component_id
            in {
                candidate.left_component_id,
                candidate.right_component_id,
            }
            and item.front_reading_id == declared_readings[0]
        )
        right_models = tuple(
            item
            for item in supported
            if isinstance(item, StripArrivalModelV1)
            and item.interaction_component_id
            in {
                candidate.left_component_id,
                candidate.right_component_id,
            }
            and item.front_reading_id == declared_readings[1]
        )
        if len(left_models) != 1 or len(right_models) != 1:
            return ()
        return ((left_models[0], right_models[0]),)
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
                    frozenset(
                        {
                            candidate.left_component_id,
                            candidate.right_component_id,
                        }
                    ),
                )
            )
            continue
        if not pairs:
            participant_component_ids = frozenset(
                {
                    candidate.left_component_id,
                    candidate.right_component_id,
                }
            )
            unsupported_junctions = tuple(
                item
                for item in arrival_models
                if isinstance(item, UnsupportedJunctionArrivalModelV1)
                and item.interaction_component_id
                in participant_component_ids
            )
            supported_component_ids = frozenset(
                item.interaction_component_id
                for item in arrival_models
                if not isinstance(
                    item, UnsupportedJunctionArrivalModelV1
                )
            )
            if unsupported_junctions and not participant_component_ids.issubset(
                supported_component_ids
            ):
                diagnostics.append(
                    InteractionDiagnosticV1(
                        InteractionOutcome.INTERACTION_JUNCTION_ARRIVAL_UNPROVEN,
                        InteractionDiagnosticSeverity.UNSUPPORTED,
                        "candidate requires a Junction front reading with "
                        "no accepted exact support law",
                        participant_component_ids,
                        frozenset(
                            item.arrival_model_id
                            for item in unsupported_junctions
                        ),
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
            batch_id = SameAlphaInteractionBatchId(stable_id(
                "same-alpha-interaction-batch",
                candidate.decal_request_id,
                candidate.patch_domain_id,
                clipped.exact_first_alpha.expression,
            ))
            certificate_id = MutualArrivalCertificateId(stable_id(
                "mutual-arrival-certificate",
                candidate.candidate_id,
                left_reading.front_reading_id,
                right_reading.front_reading_id,
                clipped.exact_first_alpha.expression,
                *_line_signature(clipped),
            ))
            certificate = MutualArrivalCertificateV1(
                certificate_id=certificate_id,
                candidate_id=candidate.candidate_id,
                left_front_reading=left_reading,
                right_front_reading=right_reading,
                equivalent_left_front_readings=frozenset({left_reading}),
                equivalent_right_front_readings=frozenset({right_reading}),
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
        by_signature: dict[tuple[str, str, str], list[ProvenInteractionV1]] = {}
        for proof in earliest_proofs:
            by_signature.setdefault(
                _line_signature(proof.clipped_locus), []
            ).append(proof)
        selected_items = []
        equivalent_locus_unproven = False
        for signature in sorted(by_signature):
            items = tuple(by_signature[signature])
            merged = _merge_equivalent_locus_proofs(items)
            if merged is None:
                diagnostics.append(
                    InteractionDiagnosticV1(
                        InteractionOutcome.INTERACTION_EQUIVALENT_LOCUS_PROVENANCE_UNPROVEN,
                        InteractionDiagnosticSeverity.UNSUPPORTED,
                        "one canonical equality locus has distinct "
                        "reading or provenance proofs",
                        frozenset(
                            {
                                candidate.left_component_id,
                                candidate.right_component_id,
                            }
                        ),
                        frozenset(
                            model.arrival_model_id
                            for proof in items
                            for model in (
                                proof.left_model,
                                proof.right_model,
                            )
                        ),
                    )
                )
                equivalent_locus_unproven = True
                break
            selected_items.append(merged)
        if equivalent_locus_unproven:
            continue
        selected = tuple(selected_items)
        if len(selected) > 1:
            left_models = {item.left_model.arrival_model_id for item in selected}
            right_models = {
                item.right_model.arrival_model_id for item in selected
            }
            left_is_profile = _is_exposed_angular_component_side(
                selected,
                "left_model",
            )
            right_is_profile = _is_exposed_angular_component_side(
                selected,
                "right_model",
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

    by_batch: dict[
        SameAlphaInteractionBatchId, list[ProvenInteractionV1]
    ] = {}
    for proof in proofs:
        by_batch.setdefault(
            proof.mutual_arrival_certificate.same_alpha_batch_identity, []
        ).append(proof)
    rejected = set()
    for batch, batch_proofs in by_batch.items():
        degree = {}
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
                item.mutual_arrival_certificate.same_alpha_batch_identity.value,
                item.candidate.candidate_id.value,
            ),
        )
    ), tuple(diagnostics)
