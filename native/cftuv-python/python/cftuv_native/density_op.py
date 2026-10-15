"""Типизированный raw Density batch: public pin и явные candidate-входы."""

from fractions import Fraction

OPERATION = "density"


class DensityNativeInternalErrorV1(RuntimeError):
    """Внутренний/profile gap сохраняет частичные факты; это не geometric outcome."""
    def __init__(self, detail, spec_index, completed_results, partial_effects):
        super().__init__("density_compute: internal/profile failure: " + detail)
        self.spec_index = spec_index
        self.completed_results = completed_results
        self.partial_effects = partial_effects


def _validate_report(batch, answer):
    from cftuv_envelope.reference.density_oracle_v1 import RawDensitySpecResultV1
    from cftuv_envelope.reference.density_attach_v1 import _validate_failure, _validate_success

    if type(answer) is not tuple or len(answer) != 2:
        raise RuntimeError("density_compute: malformed native batch report")
    results, internal = answer
    if type(results) is not tuple or len(results) > len(batch.specs):
        raise RuntimeError("density_compute: native results are not an input prefix")
    failed = []
    for index, value in enumerate(results):
        if type(value) is not RawDensitySpecResultV1:
            raise RuntimeError("density_compute: unknown native result type")
        if value.outcome != "OK":
            _validate_failure(value)
            failed.append(index)
        else:
            if value.failure is not None:
                raise RuntimeError("density_compute: success has failure facts")
            _validate_success(batch.specs[index], value, allow_zero=True)
    if failed and failed != [len(results)-1]:
        raise RuntimeError("density_compute: native results continue after failure")
    if internal is not None:
        if (type(internal) is not tuple or len(internal) != 3 or type(internal[0]) is not int
                or internal[0] != len(results) or not internal[0] < len(batch.specs)
                or type(internal[1]) is not str or type(internal[2]) is not RawDensitySpecResultV1 or failed):
            raise RuntimeError("density_compute: malformed partial internal failure")
        raise DensityNativeInternalErrorV1(internal[1], internal[0], results, internal[2])
    if not failed and len(results) != len(batch.specs):
        raise RuntimeError("density_compute: successful native batch is incomplete")
    return results


def density_compute(core, batch, *, candidate=False):
    from cftuv_envelope.reference.density_oracle_v1 import RawDensityBatchInputV1
    from . import pin

    if type(batch) is not RawDensityBatchInputV1:
        raise pin.NativePortUnsupported("density_compute requires an exact RawDensityBatchInputV1")
    if not candidate:
        if OPERATION not in pin.OPERATION_FILES:
            raise pin.NativePortStale("density_compute: pin is not enabled; candidate call must be explicit")
        pin.require(OPERATION)
    # Kernel record guards не производят геометрию; результат marshal не вызывает __init__.
    batch.canon_ids.validate_candidate()
    answer = core(batch, Fraction)
    try:
        result = _validate_report(batch, answer)
    except DensityNativeInternalErrorV1 as error:
        _replay_density_work((*error.completed_results, error.partial_effects))
        raise
    _replay_density_work(result)
    return result


def _validate_memo_snapshot(batch, snapshot):
    from dataclasses import fields
    from types import UnionType
    from typing import Union, get_args, get_origin, get_type_hints
    from cftuv_envelope.reference import density_attempt_memo_v1 as memo, density_oracle_v1 as facts
    hints_by_class = {}

    def typed(value, annotation):
        origin, args = get_origin(annotation), get_args(annotation)
        if origin in (UnionType, Union):
            for option in args:
                try:
                    typed(value, option)
                    return
                except (TypeError, ValueError):
                    pass
            raise TypeError("density memo fact has an invalid optional/union type")
        if annotation is type(None):
            if value is not None:
                raise TypeError("density memo requires None")
            return
        if origin is tuple or annotation is tuple:
            if type(value) is not tuple:
                raise TypeError("density memo requires immutable tuples")
            if not args:
                memo._closed_fact(value)
            elif len(args) == 2 and args[1] is Ellipsis:
                for item in value:
                    typed(item, args[0])
            else:
                if len(value) != len(args):
                    raise ValueError("density memo tuple cardinality mismatch")
                for item, expected in zip(value, args):
                    typed(item, expected)
            return
        if type(value) is not annotation:
            raise TypeError("density memo fact requires exact " + annotation.__name__)
        if annotation in (bool, int, str, Fraction):
            return
        if annotation.__module__ not in (facts.__name__, memo.__name__):
            raise TypeError("density memo contains a non-closed record")
        if annotation not in hints_by_class:
            hints_by_class[annotation] = get_type_hints(annotation)
        hints = hints_by_class[annotation]
        for field in fields(annotation):
            typed(getattr(value, field.name), hints[field.name])

    def literal(tree, depth=1):
        if depth > 512 or type(tree) is not tuple or not tree or type(tree[0]) is not str:
            raise ValueError("density memo malformed literal key")
        tag, *args = tree
        if tag == "Q":
            if len(args) != 2 or any(type(v) is not int for v in args) or args[1] <= 0:
                raise ValueError("density memo malformed literal rational")
            return
        if not ((tag == "pi" and not args) or (tag in ("Add", "Mul") and len(args) >= 2)
                or (tag in ("Pow", "atan2") and len(args) == 2)
                or (tag in ("sin", "cos", "atan", "tan") and len(args) == 1)):
            raise ValueError("density memo unsupported literal tag/arity")
        for child in args:
            literal(child, depth+1)

    typed(snapshot, memo.DensityAttemptMemoSnapshotV1)
    seen = set()
    for receipt in snapshot.entries:
        key = receipt.key
        if key.canon_ids != batch.canon_ids:
            raise ValueError("density memo canon IDs mismatch")
        if key in seen:
            raise ValueError("density memo duplicate attempt keys")
        seen.add(key)
        if (key.owner_orientation_sign not in (-1, 1) or key.orientation not in ("CCW_IN_OWNER_PATCH_ORIENTATION", "CW_IN_OWNER_PATCH_ORIENTATION")
                or not 2 <= key.q <= 6 or key.window_law not in ("NARROW_ROTATION_BAND", "VORONOI_OF_NEIGHBOUR_MIDPOINTS")
                or len(key.ideal_primal_literals) != len(key.binding_reasons)+2
                or len(receipt.binding_certificates) != len(key.binding_reasons)
                or any(reason not in (None, "SOURCE_DIRECTION_IRRATIONAL", "EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL") for reason in key.binding_reasons)):
            raise ValueError("density memo invalid attempt profile")
        for ray in key.ideal_primal_literals:
            for coordinate in ray:
                literal(coordinate)
        if any(v is not None and v.binding_reason is not None for v in receipt.binding_certificates):
            raise ValueError("density memo certificate contains a formatted binding reason")
        if receipt.authority is not None and (receipt.authority.authority_id is not None
                or receipt.authority.binding_reasons != key.binding_reasons
                or receipt.authority.max_subturn_q != key.q
                or len(receipt.authority.ordinal_windows) != len(key.binding_reasons)):
            raise ValueError("density memo authority identity/profile mismatch")
        def interval(value):
            if value.lower_kind not in ('OPEN', 'CLOSED') or value.upper_kind not in ('OPEN', 'CLOSED'):
                raise ValueError("density memo invalid interval endpoint kind")
            if any(abs(item.exponent) > 10000 for item in (value.lower,value.upper,value.absolute_error_bound)):
                raise ValueError("density memo Decimal exponent profile")
        for certificate in receipt.binding_certificates:
            if certificate is not None:
                if certificate.proven_predicates != ('BINDING_INSIDE_OWN_ORDINAL_WINDOW','BINDING_MONOTONE','BINDING_SUBTURN_LE_DELTA_MAX'):
                    raise ValueError("density memo certificate predicates")
                interval(certificate.ideal_window_lower_slope_envelope)
                interval(certificate.ideal_window_upper_slope_envelope)
                if abs(certificate.certified_window_width_lower_bound.exponent)>10000:
                    raise ValueError("density memo Decimal exponent profile")
        if receipt.authority is not None:
            authority = receipt.authority
            count = len(key.binding_reasons)
            if (len(authority.bound_primitive_integer_vectors)!=count or len(authority.previous_height_witness.primitive_candidate_counts)!=count
                    or authority.previous_height_witness.first_height!=1
                    or authority.previous_height_witness.last_height!=authority.exhaustive_previous_height
                    or any(value<0 for value in (authority.minimal_common_height,authority.exhaustive_previous_height,authority.termination_height_upper_bound,*authority.previous_height_witness.primitive_candidate_counts))):
                raise ValueError("density memo authority witness/cardinality")
            for ordinal,window in enumerate(authority.ordinal_windows,1):
                if window.ordinal != ordinal:
                    raise ValueError("density memo authority ordinal")
                if type(window) is facts.LocalWindowFactsV1:
                    if window.denominator_sign not in (-1,1):
                        raise ValueError("density memo local denominator sign")
                    interval(window.full_lower_slope_envelope)
                    interval(window.full_upper_slope_envelope)
                else:
                    if not 0<=window.termination_piece_index<len(window.pieces):
                        raise ValueError("density memo termination piece")
                    for index,piece in enumerate(window.pieces):
                        if piece.piece_index!=index or piece.denominator_sign not in (-1,1) or piece.pole_ownership not in ('NONE','X_ZERO','Y_ZERO'):
                            raise ValueError("density memo chart piece profile")
                        interval(piece.lower_slope_envelope)
                        interval(piece.upper_slope_envelope)
        for event in receipt.shell_events:
            if any(code not in ('DENSITY_DECIMAL_SHELL_V2_APPLIED','DENSITY_DECIMAL_GRID_POINT_V2','DENSITY_DECIMAL_NON_DYADIC_GRID_STRAW_V2') for code in event.codes):
                raise ValueError("density memo shell code")
            if (event.rational_numerator is None and (event.rational_denominator is not None or event.rational_grid_units is not None)
                    or event.rational_numerator is not None and (event.rational_denominator is None or event.rational_denominator<=0)
                    or event.legacy_projected_min_width_units is not None and not 0<=event.legacy_projected_min_width_units<=255):
                raise ValueError("density memo shell metadata profile")
    return snapshot


def density_compute_with_memo(core, batch, snapshot, *, candidate=False):
    from cftuv_envelope.reference.density_oracle_v1 import RawDensityBatchInputV1
    from cftuv_envelope.reference.density_attempt_memo_v1 import DensityMemoBatchResultV1
    from . import pin

    if type(batch) is not RawDensityBatchInputV1:
        raise pin.NativePortUnsupported("density_compute requires an exact RawDensityBatchInputV1")
    if not candidate:
        if OPERATION not in pin.OPERATION_FILES:
            raise pin.NativePortStale("density_compute: pin is not enabled; candidate call must be explicit")
        pin.require(OPERATION)
    batch.canon_ids.validate_candidate()
    _validate_memo_snapshot(batch, snapshot)
    answer = core(batch, snapshot, Fraction)
    if type(answer) is not tuple or len(answer) != 2:
        raise RuntimeError("density_compute_with_memo: malformed native memo report")
    report, delta = answer
    _validate_memo_snapshot(batch, delta)
    if any(entry.key in {old.key for old in snapshot.entries} for entry in delta.entries):
        raise RuntimeError("density_compute_with_memo: delta repeats a supplied successful attempt")
    try:
        results = _validate_report(batch, report)
    except DensityNativeInternalErrorV1 as error:
        error.density_attempt_memo_delta = delta
        error.density_completed_results = error.completed_results
        error.density_partial_work_articles = error.partial_effects.ordered_work_articles
        error.density_partial_shell_events = error.partial_effects.shell_events
        error.density_partial_canon_usage = error.partial_effects.canon_usage
        _replay_density_work((*error.completed_results, error.partial_effects))
        raise
    _replay_density_work(results)
    return DensityMemoBatchResultV1(results, delta)


def _replay_density_work(results):
    # Rust batch не трогает caller; boundary восстанавливает только Density articles.
    from cftuv_envelope.reference.adaptive_density_atlas import DensityExactWorkBudget
    from cftuv_envelope.reference.density_fan_types import DensityRationalAuthorityExhausted

    articles = tuple(article for result in results for article in result.ordered_work_articles)
    for article in articles:
        if (article.phase not in ("PREPARATION", "SEARCH") or type(article.cap) is not int or article.cap < 0
                or type(article.counters) is not tuple or len(article.counters) != 2
                or any(type(row) is not tuple or len(row) != 2 or type(row[0]) is not str
                       or type(row[1]) is not int or row[1] < 0 for row in article.counters)
                or tuple(name for name, _ in article.counters) != ("DENSITY_FAN_SHELL_PROBES", "DENSITY_FAN_ORDER_STEPS")
                or any(type(value) is not int or value < 0 for _, value in article.counters)):
            raise RuntimeError("density_compute: malformed recorded Density article")
    for article in articles:
        budget = DensityExactWorkBudget(DensityRationalAuthorityExhausted, article.cap)
        budget.shell_probes, budget.order_steps = tuple(value for _, value in article.counters)
