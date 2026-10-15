"""One owned raw-spec transaction; no host or symbolic geometry producers."""

from dataclasses import replace
from fractions import Fraction

from .._canonical_angle import CANONICAL_SUBTURN_FAN_PREDICATES
from .._density_policy import CANONICAL_ROTATION_TABLE, CANONICAL_FAN_RAYS_PREDICATES, EVALUATION_SUBTURN_LIFT_PREDICATES
from ..contracts.analysis import TurnOrientation
from ..contracts.envelopes import DirectionBindingReasonV1, EvaluationGeometrySubturnCountLiftLawV1
from . import density_oracle_v1 as facts
from .contracts import CanonicalFanRaysRefusalV1, EvaluationBindingNoiseEffectV1, ReferenceOutcome
from .common import ReferenceGeometryError
from .density_binding_types import DirectionBindingCertificateUnproven
from .density_fan_types import (AdaptiveDensityFanInvalid, DensityRationalAuthorityExhausted,
                               DensityTerminationBoxesExhausted, DensityWindowChartUnrepresentable)
from .density_controller_v1 import bind_owned_density_fan
from .density_construct_v1 import DensityConstructArenaV1
from .density_owned_laws_v1 import OwnedDensityLawsV1
from .density_attempt_memo_v1 import DensityAttemptMemoSnapshotV1, DensityMemoBatchResultV1
from .density_raw_noise_v1 import evaluate_raw_noise
from .evaluation_binding_noise import NOISE_PREDICATES


def _decimal(value):
    sign, digits, exponent = value.as_tuple()
    coefficient = int("".join(map(str, digits)))
    return facts.DecimalValueFactsV1(-coefficient if sign else coefficient, exponent)


def _interval(value):
    return facts.DecimalIntervalFactsV1(_decimal(value.lower), _decimal(value.upper),
                                        value.lower_kind.value, value.upper_kind.value,
                                        _decimal(value.absolute_error_bound))


def _certificate(value, reason=None):
    if value is None:
        return None
    return facts.BindingCertificateFactsV1(value.bound_primitive_integer_vector,
        _interval(value.ideal_window_lower_slope_envelope), _interval(value.ideal_window_upper_slope_envelope),
        _decimal(value.certified_window_width_lower_bound), tuple(sorted(value.proven_predicates)), reason)


def _window(value):
    common = {name: Fraction(*getattr(value, name)) for name in (
        "termination_lower_slope", "termination_upper_slope", "certified_termination_width",
        "admissible_lower_outward", "admissible_lower_inward", "admissible_upper_inward", "admissible_upper_outward")}
    if hasattr(value, "pieces"):
        pieces = tuple(facts.ChartPieceFactsV1(p.piece_index, p.use_x_denominator, p.denominator_sign,
            _interval(p.lower_slope_envelope), _interval(p.upper_slope_envelope), p.lower_endpoint_included,
            p.upper_endpoint_included, p.slope_increases_in_ordinal_order, p.pole_ownership.value) for p in value.pieces)
        return facts.AtlasWindowFactsV1(value.ordinal, pieces, value.termination_piece_index, **common)
    return facts.LocalWindowFactsV1(value.ordinal, value.use_x_denominator, value.denominator_sign,
        _interval(value.full_lower_slope_envelope), _interval(value.full_upper_slope_envelope), **common)


def _authority(value):
    if value is None or isinstance(value, facts.CanonicalFanAuthorityFactsV1):
        return value
    witness = value.previous_height_witness
    return facts.AdaptiveFanAuthorityFactsV1(value.max_subturn_q, value.minimal_common_height,
        value.exhaustive_previous_height, value.termination_height_upper_bound,
        value.bound_primitive_integer_vectors, tuple(None if r is None else r.value for r in value.binding_reasons),
        tuple(_window(w) for w in value.ordinal_windows),
        facts.FareyHeightWitnessFactsV1(witness.first_height, witness.last_height, witness.primitive_candidate_counts),
        tuple(sorted(value.proven_predicates)), value.authority_id)


def _failure_result(error, canon_ids):
    if isinstance(error, DirectionBindingCertificateUnproven):
        outcome = ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE
        message = f"direction binding certificate is not proven: {error}"
    elif isinstance(error, ReferenceGeometryError):
        outcome, message = error.outcome, str(error)
    else:
        outcome = {DensityRationalAuthorityExhausted: ReferenceOutcome.DENSITY_RATIONAL_AUTHORITY_EXHAUSTED,
                   DensityWindowChartUnrepresentable: ReferenceOutcome.DENSITY_WINDOW_CHART_UNREPRESENTABLE,
                   DensityTerminationBoxesExhausted: ReferenceOutcome.DENSITY_FAN_BOX_REFINEMENT_EXHAUSTED,
                   AdaptiveDensityFanInvalid: ReferenceOutcome.DENSITY_SEALED_FAN_INVALID}[type(error)]
        message = outcome.value if isinstance(error, DensityWindowChartUnrepresentable) else str(error)
    failure = facts.DensityFailureFactsV1(type(error).__name__, outcome.value, message)
    usage = (facts.CanonUsageFactsV1(canon_ids.construction, facts.DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED),)
    return facts.RawDensitySpecResultV1(outcome.value, None, (), (), None, None, usage, (), (), (), failure=failure)


def _shell_facts(outcomes):
    return tuple(facts.DensityShellEventFactsV1(wire, tuple(code.value for code in bundle.codes),
        None if bundle.rational is None else bundle.rational.numerator,
        None if bundle.rational is None else bundle.rational.denominator,
        None if bundle.rational is None else bundle.rational.grid_units,
        bundle.legacy_projected_min_width_units) for wire, bundle in sorted(dict(outcomes).items()))


def evaluate_raw_density_batch(batch):
    """Один domain/request batch; память успешных попыток не переживает вызов."""
    if type(batch) is not facts.RawDensityBatchInputV1:
        raise TypeError("Raw Density batch requires RawDensityBatchInputV1")
    frontend = RawDensityFrontendV1(batch.canon_ids, batch.limits, window_law=batch.window_law)
    results = []
    for raw in batch.specs:
        result = frontend.evaluate(raw)
        results.append(result)
        if result.failure is not None:
            break
    return tuple(results)


def evaluate_raw_density_batch_with_memo(batch, snapshot):
    """Closed per-attempt snapshot in, ordered facts and successful delta out."""
    if type(batch) is not facts.RawDensityBatchInputV1:
        raise TypeError("Raw Density batch requires RawDensityBatchInputV1")
    laws = OwnedDensityLawsV1(DensityConstructArenaV1(), batch.limits, attempt_snapshot=snapshot)
    frontend = RawDensityFrontendV1(batch.canon_ids, batch.limits, window_law=batch.window_law, laws=laws)
    results = []
    try:
        for raw in batch.specs:
            result = frontend.evaluate(raw)
            results.append(result)
            if result.failure is not None:
                break
    except Exception as error:
        # Исходный internal failure не превращается в геометрический отказ.
        error.density_attempt_memo_delta = laws.attempt_memo.delta()
        error.density_completed_results = tuple(results)
        raise
    return DensityMemoBatchResultV1(tuple(results), laws.attempt_memo.delta())


class RawDensityFrontendV1:
    def __init__(self, canon_ids, limits, *, window_law="NARROW_ROTATION_BAND", laws=None):
        canon_ids.validate_candidate()
        if window_law not in ("NARROW_ROTATION_BAND", "VORONOI_OF_NEIGHBOUR_MIDPOINTS"):
            raise ValueError("Unknown declared Density window law")
        self.canon_ids, self.limits, self.window_law = canon_ids, limits, window_law
        self.laws = OwnedDensityLawsV1(DensityConstructArenaV1(), limits) if laws is None else laws
        if self.laws.limits != limits:
            raise ValueError("Raw frontend and owned laws must share resource limits")

    def evaluate(self, raw):
        if not isinstance(raw, facts.RawDensitySpecInput):
            raise TypeError("Raw frontend requires RawDensitySpecInput")
        first_budget = len(self.laws.budgets)
        first_outcome = len(self.laws.shell_outcomes)
        try:
            result = self._compute(raw)
        except (DirectionBindingCertificateUnproven, DensityRationalAuthorityExhausted,
                DensityWindowChartUnrepresentable, DensityTerminationBoxesExhausted,
                AdaptiveDensityFanInvalid, ReferenceGeometryError) as error:
            result = _failure_result(error, self.canon_ids)
        except Exception as error:
            error.density_partial_work_articles = tuple(facts.DensityPriceArticleV1(phase, budget.cap, budget.counters())
                for phase, budget in self.laws.budgets[first_budget:])
            error.density_partial_shell_events = _shell_facts(self.laws.shell_outcomes[first_outcome:])
            error.density_partial_canon_usage = (facts.CanonUsageFactsV1(
                self.canon_ids.construction, facts.DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED),)
            raise
        prices = tuple(facts.DensityPriceArticleV1(phase, budget.cap, budget.counters())
                       for phase, budget in self.laws.budgets[first_budget:])
        result = replace(result, ordered_work_articles=prices,
                         shell_events=_shell_facts(self.laws.shell_outcomes[first_outcome:]))
        return result

    def _compute(self, raw):
        laws, q, source_count = self.laws, raw.selection.q, raw.selection.source_hidden_count
        source_metric, si, so = laws.raw_geometry(raw.source_geometry, use_direction_declarations=True)
        binding_present = dict(raw.canonical.noise_policy_declaration)["binding_present"]
        metric, ei, eo = laws.raw_geometry(raw.evaluation_geometry, use_direction_declarations=not binding_present)
        si, so = laws.covector(source_metric, si), laws.covector(source_metric, so)
        ei, eo = laws.covector(metric, ei), laws.covector(metric, eo)
        turn = raw.turn_orientation_sign
        eval_sign, eval_squared = laws.turn_witness(metric, ei, eo, turn)
        noise = evaluate_raw_noise(raw, laws, source_metric)
        laws.verify_raw_selection(raw, noise.canonical)
        restored = raw.canonical.canonical_excess_over_pi
        canonical_used = {}

        def is_exact_irrational(m, fan, count, sign, squared):
            return (laws.turn_count_exact_limit(sign, squared, count, q)
                    and any(laws.rational_direction(ray)[1] is False for ray in fan[1:-1]))

        def make_ideal(m, incoming, outgoing, count, evaluation):
            sign, squared = laws.turn_witness(m, incoming, outgoing, turn)
            fan = laws.principal_fan(m, incoming, outgoing, turn, count)
            if laws.turn_count_feasible(sign, squared, count, q):
                return fan
            canonical = restored
            if canonical is None and evaluation and noise.applicable and noise.canonical*q == count+1:
                candidate = laws.canonical_fan(m, incoming, outgoing, turn, count, noise.canonical)
                if all(laws.rational_direction(ray)[1] is True for ray in candidate[1:-1]):
                    canonical = noise.canonical
            if canonical is not None:
                if canonical*q > count+1:
                    from .common import ReferenceGeometryError
                    from .contracts import ReferenceOutcome
                    raise ReferenceGeometryError(ReferenceOutcome.REFERENCE_CANONICAL_ANGLE_RESTORATION_INVALID,
                                                 "canonical subturn exceeds the declared maximum subturn")
                fan = laws.canonical_fan(m, incoming, outgoing, turn, count, canonical)
                canonical_used[(evaluation, count)] = True
            return fan

        def feasible(fan, count):
            if noise.applicable and noise.canonical*q == count+1:
                canonical = laws.canonical_fan(metric, ei, eo, turn, count, noise.canonical)
                sign, squared = laws.turn_witness(metric, canonical[0], canonical[1], turn)
                # Все канонические шаги, кроме последнего: остаток ограничен законом шума.
                return (all(laws.subturn(metric, a, b, q) for a, b in zip(canonical[:-2], canonical[1:-1]))
                        and not (laws.turn_count_exact_limit(sign, squared, 0, q)
                                 and any(laws.rational_direction(ray)[1] is False for ray in canonical[1:-1])))
            if canonical_used.get((True, count)):
                angular = laws.subturn(metric, fan[0], fan[1], q)
                boundary = laws.subturn_boundary(metric, fan[0], fan[1], q)
                return angular and not (boundary and any(laws.rational_direction(ray)[1] is False for ray in fan[1:-1]))
            return (laws.turn_count_feasible(eval_sign, eval_squared, count, q)
                    and not is_exact_irrational(metric, fan, count, eval_sign, eval_squared))

        ideal = make_ideal(metric, ei, eo, source_count, True)
        predecessor = ideal
        count = source_count
        lift = None
        if not feasible(ideal, count):
            for count in range(source_count+1, q):
                ideal = make_ideal(metric, ei, eo, count, True)
                if feasible(ideal, count):
                    break
                predecessor = ideal
            else:
                from .density_binding_types import DirectionBindingCertificateUnproven, BINDING_SUBTURN_LE_DELTA_MAX
                raise DirectionBindingCertificateUnproven(BINDING_SUBTURN_LE_DELTA_MAX)
            laws_enum = EvaluationGeometrySubturnCountLiftLawV1
            if (not laws.turn_count_feasible(eval_sign, eval_squared, source_count, q)
                    and not laws.turn_count_feasible(eval_sign, eval_squared, count-1, q)):
                law = laws_enum.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_V1
            elif is_exact_irrational(metric, predecessor, count-1, eval_sign, eval_squared):
                law = laws_enum.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_EXACT_LIMIT_V1
            elif noise.applicable and noise.canonical*q == count:
                canonical = laws.canonical_fan(metric, ei, eo, turn, count-1, noise.canonical)
                if not any(laws.rational_direction(ray)[1] is False for ray in canonical[1:-1]):
                    raise ValueError("DENSITY_RAW_CANONICAL_LIFT_NOT_PROVEN")
                law = laws_enum.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1
            else:
                raise ValueError("DENSITY_RAW_LIFT_NOT_PROVEN")
            lift = facts.CountLiftFactsV1(law.value, source_count, count, q,
                "POSITIVE" if eval_sign > 0 else "NEGATIVE" if eval_sign < 0 else "ZERO", eval_squared,
                count-1, tuple(sorted(EVALUATION_SUBTURN_LIFT_PREDICATES[law])))

        source_ideal = make_ideal(source_metric, si, so, count, False)
        return self._finish(raw, metric, ei, eo, source_ideal, noise, count, lift, ideal,
                            bool(canonical_used.get((True, count))))

    def _finish(self, raw, metric, ei, eo, source_ideal, noise, count, lift, ideal, canonical_used):
        laws, q, turn = self.laws, raw.selection.q, raw.turn_orientation_sign
        binding_present = dict(raw.canonical.noise_policy_declaration)["binding_present"]
        source_count, restored = raw.selection.source_hidden_count, raw.canonical.canonical_excess_over_pi
        eval_sign, eval_squared = laws.turn_witness(metric, ei, eo, turn)
        source_rational = tuple(laws.rational_direction(ray)[1] is True for ray in source_ideal[1:-1])
        needed = tuple(not source or laws.rational_direction(ray)[1] is not True
                       for source, ray in zip(source_rational, ideal[1:-1]))
        reasons = tuple("SOURCE_DIRECTION_IRRATIONAL" if not source else
                        "EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL" if need else None
                        for source, need in zip(source_rational, needed))
        rays_refusal, authority, certificates = None, None, (None,)*count
        if noise.canonical is not None:
            rotations = CANONICAL_ROTATION_TABLE.get((noise.canonical, count+1, q))
            refusals = CanonicalFanRaysRefusalV1
            if rotations is None:
                if lift is not None:
                    rays_refusal = refusals.NO_CANONICAL_ROTATION_TABLE_ENTRY.value
            elif noise.refusal is not None:
                rays_refusal = refusals.BINDING_NOISE_OUTSIDE_THE_DECLARED_BOUNDS.value
            else:
                rotation_ideal = laws.rotation_fan(metric, ei, eo, turn, rotations)
                vectors = tuple(laws.rational_direction(ray)[0] for ray in rotation_ideal[1:-1])
                if None in vectors:
                    rays_refusal = refusals.CANONICAL_RAYS_IRRATIONAL_IN_CHART.value
                elif not all(laws.subturn(metric, a, b, q) and laws.sign(laws.oriented_cross(metric, a, b)) == turn
                             for a, b in zip(rotation_ideal, rotation_ideal[1:])):
                    rays_refusal = refusals.CANONICAL_ROTATION_FAN_VIOLATES_SUBTURN_GUARANTEE.value
                elif lift is not None or any(needed):
                    authority = facts.CanonicalFanAuthorityFactsV1("CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1",
                        noise.relation, noise.canonical, count, q, rotations, vectors,
                        tuple(sorted(CANONICAL_FAN_RAYS_PREDICATES)))
        named_refusals = []
        if authority is None and (any(needed) or lift is not None):
            orientation = (TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION if turn == 1
                           else TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION)
            # Shared Python-контроллер сам строит ковекторы из primal-нормалей.
            primal_ideal = tuple(laws.primal(metric, ray) for ray in ideal)
            certificates, authority = bind_owned_density_fan(laws, metric, primal_ideal, orientation, q,
                tuple(None if reason is None else DirectionBindingReasonV1(reason) for reason in reasons),
                lifted=lift is not None, law=self.window_law, spec_id=raw.spec_id, refusals=named_refusals,
                canon_ids=self.canon_ids)
        selected = tuple(_certificate(certificate, reason if binding_present else None) if need else None
                         for certificate, reason, need in zip(certificates, reasons, needed))
        if (isinstance(authority, facts.CanonicalFanAuthorityFactsV1)
                or (authority is None and not any(selected))):
            rays_refusal = None
        noise_record = subturn = None
        effects = EvaluationBindingNoiseEffectV1
        effect = (effects.CANONICAL_COUNT_LIFT if lift is not None and lift.lift_law ==
                  EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1.value
                  else effects.CANONICAL_ROTATION_FAN if canonical_used and restored is None else None)
        if effect is not None:
            bound = Fraction(*dict(raw.canonical.noise_policy_declaration)["noise_direction_sine_bound"])
            noise_record = facts.NoiseRecordFactsV1("EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1", effect.value,
                noise.relation, noise.canonical, source_count, count, q,
                "POSITIVE" if eval_sign > 0 else "NEGATIVE" if eval_sign < 0 else "ZERO", eval_squared,
                noise.lateral_squared, noise.edge_sine_squared, bound, tuple(sorted(NOISE_PREDICATES[effect])))
        if canonical_used and restored is not None:
            declaration = dict(raw.canonical.restoration_declaration)
            subturn = facts.CanonicalSubturnAuthorityFactsV1("SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1",
                declaration["canonical_relation"], restored, count, q, restored/(count+1),
                Fraction(*declaration["deviation_upper_bound_radians"]), tuple(sorted(CANONICAL_SUBTURN_FAN_PREDICATES)))
        usage = (facts.CanonUsageFactsV1(self.canon_ids.construction, facts.DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED),)
        noise_refusal = noise.refusal if noise.canonical is not None and noise.canonical*q == source_count+1 else None
        return facts.RawDensitySpecResultV1("OK", count, reasons, selected, _authority(authority), lift,
            usage, (), (), tuple(named_refusals), noise_record=noise_record, canonical_subturn_authority=subturn,
            law_refusals=facts.CanonicalLawRefusalFactsV1(noise_refusal, rays_refusal))
