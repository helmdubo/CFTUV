"""Печать доказанных Density facts в IR; геометрия и поиск сюда не входят."""

from decimal import Decimal
from fractions import Fraction

from ..contracts import envelopes as ir
from ..contracts.metric import ExactRationalV1
from ..ids import EnvelopeSpecId, SelectionCertificateId
from ..numeric import CertifiedDecimalIntervalV1, ExactRatioV1, IntervalEndpointKind
from . import contracts as ref_ir
from . import density_oracle_v1 as facts
from .common import stable_id


class DensityResultInvalid(ValueError):
    """Повреждённый провод результата, а не геометрический отказ веера."""


def _require(condition, detail):
    if not condition:
        raise DensityResultInvalid(f"DENSITY_RESULT_V1_INVALID: {detail}")


def _predicates(values):
    _require(type(values) is tuple and all(type(item) is str for item in values),
             "predicate names must be a tuple of strings")
    _require(values == tuple(sorted(set(values))), "predicate names must be sorted and unique")
    return frozenset(values)


def _vectors(values):
    _require(type(values) is tuple and all(
        type(pair) is tuple and len(pair) == 2 and all(type(item) is int for item in pair)
        for pair in values
    ), "integer vector rows required")
    return values


def _ratio(value):
    _require(type(value) is Fraction, "exact Fraction required")
    return value.numerator, value.denominator


def format_decimal(value: facts.DecimalValueFactsV1) -> Decimal:
    _require(type(value) is facts.DecimalValueFactsV1, "Decimal facts required")
    # Decimal(int) и tuple-конструктор не округляют по контексту и не печатают int.
    sign, digits, _ = Decimal(value.coefficient).as_tuple()
    return Decimal((sign, digits, value.exponent))


def format_interval(value: facts.DecimalIntervalFactsV1) -> CertifiedDecimalIntervalV1:
    _require(type(value) is facts.DecimalIntervalFactsV1, "interval facts required")
    return CertifiedDecimalIntervalV1(
        format_decimal(value.lower), format_decimal(value.upper),
        IntervalEndpointKind(value.lower_kind), IntervalEndpointKind(value.upper_kind),
        format_decimal(value.absolute_error_bound),
    )


def format_certificate(value: facts.BindingCertificateFactsV1 | None):
    if value is None:
        return None
    _require(type(value) is facts.BindingCertificateFactsV1, "binding facts required")
    fields = dict(
        bound_primitive_integer_vector=_vectors((value.bound_primitive_integer_vector,))[0],
        ideal_window_lower_slope_envelope=format_interval(value.ideal_window_lower_slope_envelope),
        ideal_window_upper_slope_envelope=format_interval(value.ideal_window_upper_slope_envelope),
        certified_window_width_lower_bound=format_decimal(value.certified_window_width_lower_bound),
        proven_predicates=_predicates(value.proven_predicates),
    )
    if value.binding_reason is None:
        return ir.DirectionBindingCertificateV1(**fields)
    return ir.EvaluationGeometryDirectionBindingCertificateV1(
        **fields, binding_reason=ir.DirectionBindingReasonV1(value.binding_reason)
    )


def _format_piece(value: facts.ChartPieceFactsV1):
    _require(type(value) is facts.ChartPieceFactsV1, "chart piece facts required")
    return ir.AdaptiveRationalFanProjectiveChartPieceV1(
        value.piece_index, value.use_x_denominator, value.denominator_sign,
        format_interval(value.lower_slope_envelope), format_interval(value.upper_slope_envelope),
        value.lower_endpoint_included, value.upper_endpoint_included,
        value.slope_increases_in_ordinal_order, ir.AdaptiveProjectivePoleOwnershipV1(value.pole_ownership),
    )


def format_window(value: facts.LocalWindowFactsV1 | facts.AtlasWindowFactsV1):
    _require(type(value) in (facts.LocalWindowFactsV1, facts.AtlasWindowFactsV1), "window facts required")
    fields = {name: _ratio(getattr(value, name)) for name in (
        "termination_lower_slope", "termination_upper_slope", "certified_termination_width",
        "admissible_lower_outward", "admissible_lower_inward",
        "admissible_upper_inward", "admissible_upper_outward",
    )}
    if type(value) is facts.LocalWindowFactsV1:
        return ir.AdaptiveRationalFanOrdinalWindowV2(
            ordinal=value.ordinal, use_x_denominator=value.use_x_denominator,
            denominator_sign=value.denominator_sign,
            full_lower_slope_envelope=format_interval(value.full_lower_slope_envelope),
            full_upper_slope_envelope=format_interval(value.full_upper_slope_envelope), **fields,
        )
    pieces = tuple(_format_piece(item) for item in value.pieces)
    _require(tuple(item.piece_index for item in pieces) == tuple(range(len(pieces))),
             "chart pieces must retain index order")
    return ir.AdaptiveRationalFanOrdinalWindowAtlasV1(
        ordinal=value.ordinal, pieces=pieces, termination_piece_index=value.termination_piece_index,
        **fields,
    )


def _check_authority_id(supplied, computed):
    _require(supplied is None or supplied == computed, "authority identity disagrees with result facts")
    return computed


def format_authority(value, selection_certificate_id: SelectionCertificateId):
    if value is None:
        return None
    _require(type(selection_certificate_id) is SelectionCertificateId, "selection declaration ID required")
    if type(value) is facts.CanonicalFanAuthorityFactsV1:
        vectors = _vectors(value.bound_primitive_integer_vectors)
        rotations = _vectors(value.ray_rotation_pairs)
        _require(len(vectors) == len(rotations) == value.hidden_edge_count,
                 "canonical ray count disagrees with hidden count")
        authority_id = stable_id(
            "canonical-rational-rotation-fan-authority-v1", selection_certificate_id,
            value.canonical_reflex_excess_over_pi, value.hidden_edge_count,
            value.max_subturn_q, rotations, vectors,
        )
        return ir.CanonicalRationalRotationFanAuthorityV1(
            _check_authority_id(value.authority_id, authority_id), ir.CanonicalFanRaysLawV1(value.ray_law),
            selection_certificate_id, ir.CanonicalReflexAngleRelationV1(value.canonical_relation),
            ExactRatioV1(*_ratio(value.canonical_reflex_excess_over_pi)), value.hidden_edge_count,
            value.max_subturn_q, rotations, vectors, _predicates(value.proven_predicates),
        )
    _require(type(value) is facts.AdaptiveFanAuthorityFactsV1, "tagged authority facts required")
    # Общий печатный digest, без повторного доказательства окон или веера.
    from .adaptive_density_fan import _authority_id

    windows = tuple(format_window(item) for item in value.ordinal_windows)
    vectors = _vectors(value.bound_primitive_integer_vectors)
    _require(len(windows) == len(vectors) == len(value.binding_reasons), "adaptive row counts disagree")
    _require(tuple(item.ordinal for item in windows) == tuple(range(1, len(windows) + 1)),
             "windows must retain ordinal order")
    identity = _authority_id(value.max_subturn_q, value.minimal_common_height, vectors, windows)
    witness = value.previous_height_witness
    _require(type(witness) is facts.FareyHeightWitnessFactsV1, "height witness facts required")
    return ir.AdaptiveMinimalRationalFanAuthorityV2(
        _check_authority_id(value.authority_id, identity), value.max_subturn_q, value.minimal_common_height,
        value.exhaustive_previous_height, value.termination_height_upper_bound, vectors,
        tuple(None if item is None else ir.DirectionBindingReasonV1(item) for item in value.binding_reasons),
        windows, ir.AdaptiveFareyHeightRangeWitnessV2(
            witness.first_height, witness.last_height, witness.primitive_candidate_counts,
        ), _predicates(value.proven_predicates),
    )


def format_count_lift(value: facts.CountLiftFactsV1 | None, selection_certificate_id: SelectionCertificateId):
    if value is None:
        return None
    _require(type(value) is facts.CountLiftFactsV1, "lift facts required")
    _require(type(selection_certificate_id) is SelectionCertificateId, "selection declaration ID required")
    return ir.EvaluationGeometrySubturnCountLiftV1(
        ir.EvaluationGeometrySubturnCountLiftLawV1(value.lift_law), selection_certificate_id,
        value.source_hidden_edge_count, value.effective_hidden_edge_count, value.max_subturn_q,
        ir.ExactTurnSignV1(value.evaluation_turn_sign), ExactRatioV1(*_ratio(value.evaluation_turn_cosine_squared)),
        value.minimality_predecessor_hidden_edge_count, _predicates(value.proven_predicates),
    )


def _declaration_ids(spec_id, selection_id):
    _require(type(spec_id) is EnvelopeSpecId and type(selection_id) is SelectionCertificateId,
             "spec and selection declaration IDs required")


def format_noise_record(value, spec_id: EnvelopeSpecId, selection_id: SelectionCertificateId):
    if value is None:
        return None
    _require(type(value) is facts.NoiseRecordFactsV1, "noise record facts required")
    _declaration_ids(spec_id, selection_id)
    return ref_ir.EvaluationBindingNoiseOnCanonicalAngleV1(
        ref_ir.EvaluationBindingNoiseLawV1(value.noise_law), ref_ir.EvaluationBindingNoiseEffectV1(value.effect),
        spec_id, selection_id, ir.CanonicalReflexAngleRelationV1(value.canonical_relation),
        ExactRatioV1(*_ratio(value.canonical_reflex_excess_over_pi)), value.source_hidden_edge_count,
        value.effective_hidden_edge_count, value.max_subturn_q, ir.ExactTurnSignV1(value.evaluation_turn_sign),
        ExactRatioV1(*_ratio(value.evaluation_turn_cosine_squared)),
        ExactRationalV1(*_ratio(value.binding_lateral_offset_gram_squared_bound)),
        ExactRationalV1(*_ratio(value.edge_direction_sine_squared_bound)),
        ExactRationalV1(*_ratio(value.direction_sine_bound)), _predicates(value.proven_predicates),
    )


def format_canonical_subturn_authority(value, spec_id: EnvelopeSpecId, selection_id: SelectionCertificateId):
    if value is None:
        return None
    _require(type(value) is facts.CanonicalSubturnAuthorityFactsV1, "canonical subturn facts required")
    _declaration_ids(spec_id, selection_id)
    return ir.CanonicalSubturnFanAuthorityV1(
        ir.SubturnGuaranteeLawV1(value.guarantee_law), spec_id, selection_id,
        ir.CanonicalReflexAngleRelationV1(value.canonical_relation),
        ExactRatioV1(*_ratio(value.canonical_reflex_excess_over_pi)), value.hidden_edge_count,
        value.max_subturn_q, ExactRatioV1(*_ratio(value.canonical_subturn_over_pi)),
        ExactRationalV1(*_ratio(value.raw_residual_upper_bound_radians)), _predicates(value.proven_predicates),
    )


def format_canonical_law_refusals(value, spec_id: EnvelopeSpecId):
    _require(type(value) is facts.CanonicalLawRefusalFactsV1 and type(spec_id) is EnvelopeSpecId,
             "canonical law refusal facts and spec ID required")
    output = []
    if value.noise_refusal is not None:
        code = ref_ir.EvaluationBindingNoiseRefusalV1(value.noise_refusal).value
        output.append(ref_ir.ReferenceEvaluationDiagnosticV1(
            outcome=ref_ir.ReferenceOutcome.EVALUATION_BINDING_NOISE_LAW_NOT_APPLIED,
            severity=ref_ir.ReferenceDiagnosticSeverity.INFO,
            message=f"{code}: the count of a canonical angle at the exact subturn limit is decided by the sign of the evaluation binding noise",
            envelope_spec_id=spec_id.value,
        ))
    if value.rays_refusal is not None:
        code = ref_ir.CanonicalFanRaysRefusalV1(value.rays_refusal).value
        output.append(ref_ir.ReferenceEvaluationDiagnosticV1(
            outcome=ref_ir.ReferenceOutcome.CANONICAL_FAN_RAYS_LAW_NOT_APPLIED,
            severity=ref_ir.ReferenceDiagnosticSeverity.INFO,
            message=f"{code}: the rays of a canonical angle are bound by the adaptive atlas or the per-ray window on the evaluation geometry, not set by the rotation table",
            envelope_spec_id=spec_id.value,
        ))
    return tuple(output)


def format_shell_events(values):
    """Перенести записанные value-outcomes в collector; значения не вычислять."""
    from .density_shell_v2 import DensityShellOutcomeBundleV2, DensityShellOutcomeCode, DensityShellRationalMetadataV2

    _require(type(values) is tuple, "shell event tuple required")
    output = []
    for value in values:
        _require(type(value) is facts.DensityShellEventFactsV1, "shell event facts required")
        _require(type(value.value_wire) is tuple and all(type(item) is int for item in value.value_wire),
                 "shell value wire must contain integers")
        _require(type(value.codes) is tuple and bool(value.codes), "shell outcome codes required")
        codes = tuple(DensityShellOutcomeCode(code) for code in value.codes)
        _require(codes[0] is DensityShellOutcomeCode.APPLIED and len(set(codes)) == len(codes),
                 "shell outcome codes must start with APPLIED and be unique")
        rational = None
        if value.rational_numerator is None:
            _require(value.rational_denominator is None and value.rational_grid_units is None,
                     "irrational shell metadata must not include rational fields")
        else:
            _require(type(value.rational_numerator) is int and type(value.rational_denominator) is int
                     and value.rational_denominator > 0, "rational shell metadata must contain integer numerator/positive denominator")
            _require(value.rational_grid_units is None or type(value.rational_grid_units) is int,
                     "shell grid units must be integers")
            rational = DensityShellRationalMetadataV2(
                value.rational_numerator, value.rational_denominator, value.rational_grid_units)
        _require(value.legacy_projected_min_width_units is None or
                 (type(value.legacy_projected_min_width_units) is int and value.legacy_projected_min_width_units >= 0),
                 "shell legacy width must be a nonnegative integer")
        output.append((value.value_wire, DensityShellOutcomeBundleV2(
            codes, rational, value.legacy_projected_min_width_units)))
    return tuple(output)


def format_density_failure(value):
    """Печатать только записанный геометрический отказ, не внутреннюю ошибку порта."""
    _require(type(value) is facts.DensityFailureFactsV1, "density failure facts required")
    known = {
        "DirectionBindingCertificateUnproven": "REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE",
        "DensityRationalAuthorityExhausted": "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED",
        "DensityWindowChartUnrepresentable": "DENSITY_WINDOW_CHART_UNREPRESENTABLE",
        "DensityTerminationBoxesExhausted": "DENSITY_FAN_BOX_REFINEMENT_EXHAUSTED",
        "AdaptiveDensityFanInvalid": "DENSITY_SEALED_FAN_INVALID",
    }
    _require(value.exception_type == "ReferenceGeometryError" or
             (value.exception_type in known and known[value.exception_type] == value.outcome),
             "internal failures cannot be printed as geometric outcomes")
    _require(type(value.message) is str, "density failure message must be text")
    outcome = ref_ir.ReferenceOutcome(value.outcome)
    diagnostic = ref_ir.ReferenceEvaluationDiagnosticV1(
        outcome=outcome, severity=ref_ir.ReferenceDiagnosticSeverity.UNSUPPORTED, message=value.message)
    return ref_ir.ReferenceCompileResultV1(outcome, None, (diagnostic,))
