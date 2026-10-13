"""Product raw Density attachment and an explicit frozen-reference seam."""

from dataclasses import dataclass, replace

from .._density_policy import huber_density_value_contract
from ..contracts.envelopes import AngularEnvelopeSpec
from ..contracts.request import AngularProfileSelectionPolicyId
from ..contracts.surface import SurfacePayloadMode
from .common import GeometryContext
from .contracts import ReferenceCompileResultV1
from .density_attach_v1 import (attach_density_specs_v1, attach_density_results_v1,
                               _validate_success, _validate_failure)
from .density_oracle_v1 import RawDensityBatchInputV1, RawDensitySpecResultV1
from .density_attempt_memo_v1 import (
    DensityAttemptMemoSnapshotV1, DensityMemoBatchResultV1, merge_density_attempt_delta,
)
from .density_raw_v1 import extract_raw_density_spec_input


@dataclass(frozen=True, slots=True)
class _PreparedRawPrefix:
    inputs: tuple
    error: Exception | None = None
    traceback: object = None


def attach_product_density_compilation_v1(compilation, *, density_exact_memo=None, shell_event_collector=None):
    """Одна Density-операция; caps и закон окна читаются из текущего caller-профиля."""
    from ..backend import density_compute
    from . import compile as compiler, adaptive_density_atlas as atlas, adaptive_density_fan as fan
    from .density_construct_v1 import ExpandLimitsV1
    from .density_oracle_v1 import (DensityOracleCanonV1, DensityOracleLimitsV1,
        DENSITY_SOURCE_CONSTRUCTION_V1, DENSITY_SHELL_CANON_V2_CANDIDATE, DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1)

    canon_ids = DensityOracleCanonV1(DENSITY_SOURCE_CONSTRUCTION_V1, DENSITY_SHELL_CANON_V2_CANDIDATE,
                                   DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1)
    limits = DensityOracleLimitsV1(atlas.DENSITY_FAN_PREPARATION_WORK_CAP, fan._DENSITY_EXACT_WORK_CAP,
        fan._BOX_REFINEMENT_CAP, fan._RESOURCE_HEIGHT_CAP, ExpandLimitsV1(4096, 16, 65536))
    return attach_density_compilation_batch_v1(compilation, density_compute, canon_ids=canon_ids, limits=limits,
        window_law=compiler.FAN_WINDOW_LAW, density_exact_memo=density_exact_memo,
        shell_event_collector=shell_event_collector)


def _eligible_specs(compilation):
    selections = {item.certificate_id: item for item in compilation.profile_selection_certificates}
    for spec in sorted(compilation.envelope_specs, key=lambda item: item.envelope_spec_id.value):
        if not isinstance(spec, AngularEnvelopeSpec) or spec.resolved_hidden_edge_count <= 0:
            continue
        selection = selections[spec.selection_certificate_id]
        if (selection.selection_policy_id is AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
                and huber_density_value_contract(selection.max_subturn_value_id) is not None):
            yield spec, selection


def _raw_inputs(compilation, source_context, context):
    return tuple(extract_raw_density_spec_input(source_context, context, spec, selection)
                 for spec, selection in _eligible_specs(compilation))


def _raw_prefix(compilation, source_context, context):
    values = []
    try:
        for spec, selection in _eligible_specs(compilation):
            values.append(extract_raw_density_spec_input(source_context, context, spec, selection))
    except Exception as error:
        # Host extraction не исполняется за ещё не достигнутым численным отказом.
        return _PreparedRawPrefix(tuple(values), error, error.__traceback__)
    return _PreparedRawPrefix(tuple(values))


def _prepared_attachment(compilation, density_exact_memo, *, defer_raw_errors=False):
    from . import compile as compiler

    if compilation.analysis_snapshot.surface_ir.payload_mode is not SurfacePayloadMode.FULL_HOST_SURFACE:
        return compilation, None, None, ()
    if not compiler._is_explicit_density(compilation):
        raise ValueError("Candidate Density attachment requires an explicit Density request")
    frame, diagnostics = compiler.validate_reference_geometry_payload(
        compilation.analysis_snapshot, compilation.plan_key.patch_domain_id,
        density_bounded=True, density_exact_memo=density_exact_memo)
    if frame is None:
        return ReferenceCompileResultV1(diagnostics[0].outcome, None, diagnostics), None, None, ()
    context = GeometryContext.build(compilation, frame, require_certified_bound_supports=False,
                                    require_canonical_fan_authority=False, density_exact_memo=density_exact_memo)
    source = GeometryContext.build(replace(compilation, evaluation_geometry_binding=None), frame,
        require_evaluation_binding=False, require_certified_bound_supports=False,
        require_canonical_fan_authority=False, density_exact_memo=density_exact_memo)
    raw = (_raw_prefix(compilation, source, context) if defer_raw_errors
           else _raw_inputs(compilation, source, context))
    return compilation, context, source, raw


def _assembled_compilation(compilation, context, attached, ordered_work):
    from . import compile as compiler
    if ordered_work is not None:
        ordered_work.extend(attached.ordered_work_articles)
    if attached.failure is not None:
        return attached.failure, None
    updated = replace(compilation, envelope_specs=attached.envelope_specs,
        canonical_subturn_fan_authorities=attached.canonical_subturn_fan_authorities,
        evaluation_binding_noise_records=attached.evaluation_binding_noise_records,
        diagnostics=(*compilation.diagnostics, *attached.diagnostics))
    return compiler._synchronize_effective_hidden_support_records(compilation, updated), context


def attach_density_compilation_v1(compilation, raw_frontend, *, density_exact_memo=None,
                                  shell_event_collector=None, ordered_work=None):
    """Те же frame/context guards, затем shared prototype frontend на все спеки."""
    value, context, source, raw = _prepared_attachment(compilation, density_exact_memo)
    if context is None:
        return value, None
    attached = attach_density_specs_v1(compilation, context, source, raw_frontend, raw,
                                       shell_event_collector=shell_event_collector)
    return _assembled_compilation(compilation, context, attached, ordered_work)


def attach_density_compilation_batch_v1(compilation, batch_compute, *, canon_ids, limits, window_law,
                                       density_exact_memo=None, shell_event_collector=None, ordered_work=None):
    """Один injected pure batch call; caps/canon/window law заданы вызывающим явно."""
    value, context, source, prepared = _prepared_attachment(compilation, density_exact_memo, defer_raw_errors=True)
    if context is None:
        return value, None
    raw = prepared.inputs
    if prepared.error is not None and not raw:
        raise prepared.error.with_traceback(prepared.traceback)
    batch = RawDensityBatchInputV1(raw, canon_ids, limits, window_law)
    memo = {} if density_exact_memo is None else density_exact_memo.density_attempt_memo
    snapshot = DensityAttemptMemoSnapshotV1(tuple(memo.values()))
    try:
        answer = batch_compute(batch, memo_snapshot=snapshot)
    except Exception as error:
        delta = getattr(error, "density_attempt_memo_delta", DensityAttemptMemoSnapshotV1())
        completed = getattr(error, "density_completed_results", ())
        _validate_result_prefix(raw, completed, internal=True)
        merge_density_attempt_delta(memo, delta)
        _preserve_internal_prefix(error, completed, raw, canon_ids, shell_event_collector, ordered_work)
        raise
    if type(answer) is not DensityMemoBatchResultV1 or type(answer.delta) is not DensityAttemptMemoSnapshotV1:
        raise TypeError("Density memo batch compute requires DensityMemoBatchResultV1")
    results = answer.results
    _validate_result_prefix(raw, results)
    merge_density_attempt_delta(memo, answer.delta)
    pairs = tuple((raw[index].spec_id, result) for index, result in enumerate(results))
    if prepared.error is not None:
        if results[-1].failure is not None:
            from .density_attach_v1 import _attach_density_failed_prefix_v1
            attached = _attach_density_failed_prefix_v1(compilation, canon_ids, raw, pairs,
                shell_event_collector=shell_event_collector)
            return _assembled_compilation(compilation, context, attached, ordered_work)
        prepared.error.density_attempt_memo_delta = answer.delta
        prepared.error.density_completed_results = results
        _preserve_internal_prefix(prepared.error, results, raw, canon_ids, shell_event_collector, ordered_work)
        raise prepared.error.with_traceback(prepared.traceback)
    attached = attach_density_results_v1(compilation, context, source, canon_ids, raw, pairs,
                                        shell_event_collector=shell_event_collector)
    return _assembled_compilation(compilation, context, attached, ordered_work)


def _validate_result_prefix(raw, results, *, internal=False):
    if type(results) is not tuple or len(results) > len(raw):
        raise ValueError("Density batch compute must return an immutable result prefix")
    if any(type(result) is not RawDensitySpecResultV1 for result in results):
        raise TypeError("Density batch result facts have an unknown type")
    failures = tuple(index for index, result in enumerate(results) if result.outcome != "OK" or result.failure is not None)
    if failures and (internal or failures != (len(results)-1,)):
        raise ValueError("Density batch results continue after the first failure")
    if not failures and not internal and len(results) != len(raw):
        raise ValueError("Successful Density batch results are incomplete")
    for declaration, result in zip(raw, results):
        if result.failure is not None:
            _validate_failure(result)
        else:
            _validate_success(declaration, result)


def _preserve_internal_prefix(error, completed, raw, canon_ids, collector, ordered_work):
    from .density_result_v1 import format_shell_events

    for declaration, result in zip(raw, completed):
        if ordered_work is not None:
            ordered_work.extend(result.ordered_work_articles)
        if collector is not None:
            collector.replay_canon(declaration.spec_id, result.canon_usage, canon_ids)
            collector.replay(declaration.spec_id, format_shell_events(result.shell_events))
    if ordered_work is not None:
        ordered_work.extend(getattr(error, "density_partial_work_articles", ()))
    if collector is not None and len(completed) < len(raw):
        collector.replay_canon(raw[len(completed)].spec_id,
            getattr(error, "density_partial_canon_usage", ()), canon_ids)
        collector.replay(raw[len(completed)].spec_id,
            format_shell_events(getattr(error, "density_partial_shell_events", ())))
