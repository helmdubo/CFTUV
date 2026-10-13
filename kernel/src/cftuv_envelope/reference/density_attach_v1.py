"""Candidate attachment typed raw результатов; compile/default здесь не меняются."""

from __future__ import annotations

from dataclasses import dataclass, replace

from ..contracts.envelopes import AngularEnvelopeSpec
from ..contracts.request import AngularProfileSelectionPolicyId
from .._density_policy import huber_density_value_contract
from .contracts import ReferenceOutcome
from .density_oracle_v1 import (
    AdaptiveFanAuthorityFactsV1, CanonicalFanAuthorityFactsV1, DensityFailureFactsV1,
    RawDensitySpecInput, RawDensitySpecResultV1,
)
from .density_result_v1 import (
    format_authority, format_canonical_law_refusals, format_canonical_subturn_authority,
    format_certificate, format_count_lift, format_density_failure, format_noise_record, format_shell_events,
)
from .density_shell_events import DensityShellEventCollector, with_density_shell_diagnostics


@dataclass(frozen=True, slots=True)
class DensityAttachmentV1:
    # На отказе изменённая compilation отсутствует, но work/events сохраняются.
    envelope_specs: frozenset | None
    canonical_subturn_fan_authorities: frozenset
    evaluation_binding_noise_records: frozenset
    noise_diagnostics: tuple
    rays_diagnostics: tuple
    band_diagnostics: tuple
    event_diagnostics: tuple
    ordered_work_articles: tuple
    failure: object | None

    @property
    def diagnostics(self):
        return (*self.noise_diagnostics, *self.rays_diagnostics, *self.band_diagnostics, *self.event_diagnostics)


def _paired_declarations(compilation, raw_inputs):
    if type(raw_inputs) is not tuple:
        raise TypeError("Density attachment raw inputs must be an immutable tuple")
    specs = {spec.envelope_spec_id.value: spec for spec in compilation.envelope_specs
             if isinstance(spec, AngularEnvelopeSpec)}
    selections = {selection.certificate_id.value: selection for selection in compilation.profile_selection_certificates}
    eligible = {key for key, spec in specs.items() if spec.resolved_hidden_edge_count > 0
                and (selection := selections.get(spec.selection_certificate_id.value)) is not None
                and selection.selection_policy_id is AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
                and huber_density_value_contract(selection.max_subturn_value_id) is not None}
    paired, seen = [], set()
    for raw in raw_inputs:
        if type(raw) is not RawDensitySpecInput or raw.spec_id in seen:
            raise ValueError("Density attachment requires distinct typed raw spec identities")
        seen.add(raw.spec_id)
        spec = specs.get(raw.spec_id)
        if spec is None or raw.selection.selection_certificate_id != spec.selection_certificate_id.value:
            raise ValueError("Density attachment raw spec/selection identity mismatch")
        if raw.selection.selection_certificate_id not in selections:
            raise ValueError("Density attachment selection declaration is absent")
        if raw.source_relation_id != spec.source_relation_id.value or raw.owner_sector_id != spec.owner_sector_id.value:
            raise ValueError("Density attachment raw relation/sector identity mismatch")
        paired.append((spec, raw))
    return tuple(sorted(paired, key=lambda pair: pair[0].envelope_spec_id.value)), seen, eligible


def _paired_inputs(compilation, raw_inputs):
    paired, seen, eligible = _paired_declarations(compilation, raw_inputs)
    if seen != eligible:
        raise ValueError(f"Density attachment raw completeness mismatch: missing={sorted(eligible-seen)} extra={sorted(seen-eligible)}")
    return paired


def _assembled_spec(spec, result):
    # Только существующие identity/record helpers; никаких legacy predicates.
    from .compile import _adaptive_density_spec, _bound_support, _density_spec_with_hidden_count

    if not any(result.binding_reasons) and result.count_lift is None:
        if result.authority is not None:
            raise ValueError("No-binding Density result cannot contain a fan authority")
        return spec
    effective = _density_spec_with_hidden_count(spec, result.effective_hidden_count)
    lift = format_count_lift(result.count_lift, spec.selection_certificate_id)
    if result.authority is not None:
        authority = format_authority(result.authority, spec.selection_certificate_id)
        return _adaptive_density_spec(effective, authority, lift)
    certificates = tuple(format_certificate(value) for value in result.binding_certificates)
    # Late strict zip не скрывает неполный ответ frontend.
    supports = frozenset(_bound_support(support, certificate)
                         for support, certificate in zip(sorted(effective.hidden_supports, key=lambda item: item.ordinal),
                                                         certificates, strict=True))
    return replace(effective, hidden_supports=supports)


def _validate_success(raw, result, *, allow_zero=False):
    count, source_count = result.effective_hidden_count, raw.selection.source_hidden_count
    if type(count) is not int or not (0 if allow_zero else 1) <= count < raw.selection.q:
        raise ValueError("Density successful effective count is outside the declared raw/attachment range")
    if (type(result.binding_reasons) is not tuple or type(result.binding_certificates) is not tuple
            or len(result.binding_reasons) != count or len(result.binding_certificates) != count):
        raise ValueError("Density result zip cardinality does not match effective count")
    lift = result.count_lift
    if lift is None and count != source_count:
        raise ValueError("Density result changes count without a lift")
    if lift is not None and (lift.source_hidden_edge_count != source_count
            or lift.effective_hidden_edge_count != count or lift.max_subturn_q != raw.selection.q
            or count <= source_count or lift.minimality_predecessor_hidden_edge_count != count-1):
        raise ValueError("Density lift source/effective count or q mismatch")
    if not any(result.binding_reasons) and lift is None:
        if result.authority is not None or any(item is not None for item in result.binding_certificates):
            raise ValueError("No-binding Density result cannot contain authority or certificates")
    authority = result.authority
    if authority is None:
        return
    if type(authority) not in (AdaptiveFanAuthorityFactsV1, CanonicalFanAuthorityFactsV1):
        raise ValueError("Density authority facts have an unknown type")
    if authority.max_subturn_q != raw.selection.q or len(authority.bound_primitive_integer_vectors) != count:
        raise ValueError("Density authority vector count or q mismatch")
    if type(authority) is CanonicalFanAuthorityFactsV1:
        if authority.hidden_edge_count != count or len(authority.ray_rotation_pairs) != count:
            raise ValueError("Density canonical authority count mismatch")
    elif (authority.binding_reasons != result.binding_reasons or len(authority.ordinal_windows) != count
          or tuple(window.ordinal for window in authority.ordinal_windows) != tuple(range(1, count+1))):
        raise ValueError("Density adaptive authority reasons/windows mismatch")


def _validate_failure(result):
    if type(result.failure) is not DensityFailureFactsV1 or result.outcome != result.failure.outcome:
        raise ValueError("Density named failure status/facts mismatch")
    if (result.effective_hidden_count is not None or result.binding_reasons or result.binding_certificates
            or result.authority is not None or result.count_lift is not None or result.noise_record is not None
            or result.canonical_subturn_authority is not None):
        raise ValueError("Density named failure cannot contain success geometry payload")
    if result.named_refusals or result.law_refusals.noise_refusal is not None or result.law_refusals.rays_refusal is not None:
        raise ValueError("Density named failure cannot contain discarded band/noise/ray diagnostics")


def _failure_attachment(result, events, articles):
    failure = with_density_shell_diagnostics(format_density_failure(result.failure), events)
    return DensityAttachmentV1(None, frozenset(), frozenset(), (), (), (), events.diagnostics(), articles, failure)


def attach_density_specs_v1(compilation, context, source_context, raw_frontend, raw_inputs,
                            *, shell_event_collector=None) -> DensityAttachmentV1:
    """Один frontend на все sorted спеки; первый отказ завершает atomic compilation."""
    if context.snapshot is not compilation.analysis_snapshot or source_context.snapshot is not compilation.analysis_snapshot:
        raise ValueError("Density attachment contexts belong to another snapshot")
    paired = _paired_inputs(compilation, raw_inputs)
    events = DensityShellEventCollector() if shell_event_collector is None else shell_event_collector
    changed = set(compilation.envelope_specs)
    subturns, noise_records, noise_diagnostics, rays_diagnostics, band_refusals, articles = [], [], [], [], [], []
    for spec, raw in paired:
        if spec.resolved_hidden_edge_count == 0:
            continue
        result = raw_frontend.evaluate(raw)
        if type(result) is not RawDensitySpecResultV1:
            raise TypeError("Density attachment requires typed raw result facts")
        articles.extend(result.ordered_work_articles)
        events.replay_canon(spec.envelope_spec_id.value, result.canon_usage, raw_frontend.canon_ids)
        events.replay(spec.envelope_spec_id.value, format_shell_events(result.shell_events))
        if result.outcome != "OK":
            _validate_failure(result)
            return _failure_attachment(result, events, tuple(articles))
        if result.failure is not None:
            raise ValueError("Successful Density attachment contains failure facts")
        _validate_success(raw, result)
        updated = _assembled_spec(spec, result)
        changed.remove(spec)
        changed.add(updated)
        subturn = format_canonical_subturn_authority(result.canonical_subturn_authority, spec.envelope_spec_id,
                                                    spec.selection_certificate_id)
        noise = format_noise_record(result.noise_record, spec.envelope_spec_id, spec.selection_certificate_id)
        if subturn is not None:
            subturns.append(subturn)
        if noise is not None:
            noise_records.append(noise)
        for diagnostic in format_canonical_law_refusals(result.law_refusals, spec.envelope_spec_id):
            if diagnostic.outcome is ReferenceOutcome.EVALUATION_BINDING_NOISE_LAW_NOT_APPLIED:
                noise_diagnostics.append(diagnostic)
            else:
                rays_diagnostics.append(diagnostic)
        band_refusals.extend(result.named_refusals)
    from .density_fan_binding import band_refusal_diagnostics

    return DensityAttachmentV1(frozenset(changed), frozenset(subturns), frozenset(noise_records),
        tuple(noise_diagnostics), tuple(rays_diagnostics), band_refusal_diagnostics(band_refusals),
        events.diagnostics(), tuple(articles), None)


class _PrecomputedFrontend:
    # Только уже вычисленные результаты ОДНОГО batch, не новый producer/memo.
    def __init__(self, canon_ids, result_pairs):
        self.canon_ids, self.results = canon_ids, dict(result_pairs)

    def evaluate(self, raw):
        return self.results[raw.spec_id]


def attach_density_results_v1(compilation, context, source_context, canon_ids, raw_inputs, result_pairs,
                              *, shell_event_collector=None):
    """Pure batch consumer: полный ответ или точный prefix до первого отказа."""
    paired = _paired_inputs(compilation, raw_inputs)
    canon_ids.validate_candidate()
    if type(result_pairs) is not tuple or any(type(pair) is not tuple or len(pair) != 2 for pair in result_pairs):
        raise TypeError("Density batch results require immutable identity/result pairs")
    expected = tuple(raw.spec_id for _, raw in paired)
    ids = tuple(identity for identity, _ in result_pairs)
    if ids != expected[:len(ids)] or len(ids) > len(expected):
        raise ValueError("Density batch result identities are not the exact input prefix")
    values = tuple(value for _, value in result_pairs)
    if any(type(value) is not RawDensitySpecResultV1 for value in values):
        raise TypeError("Density batch result facts have an unknown type")
    failures = tuple(index for index, value in enumerate(values) if value.outcome != "OK" or value.failure is not None)
    if failures:
        if failures != (len(values)-1,):
            raise ValueError("Density batch results continue after the first failure")
    elif len(values) != len(expected):
        raise ValueError("Successful Density batch results are incomplete")
    return attach_density_specs_v1(compilation, context, source_context,
        _PrecomputedFrontend(canon_ids, result_pairs), raw_inputs, shell_event_collector=shell_event_collector)


def _attach_density_failed_prefix_v1(compilation, canon_ids, raw_inputs, result_pairs, *, shell_event_collector=None):
    """Только достигнутый отказ до отложенной host-ошибки; полный контракт не ослаблен."""
    from .density_compile_v1 import _validate_result_prefix

    paired, _, eligible = _paired_declarations(compilation, raw_inputs)
    expected = tuple(sorted(eligible))
    identities = tuple(raw.spec_id for raw in raw_inputs)
    if identities != expected[:len(identities)] or identities != tuple(raw.spec_id for _, raw in paired):
        raise ValueError("Density deferred raw inputs are not the exact eligible prefix")
    if type(result_pairs) is not tuple or any(type(pair) is not tuple or len(pair) != 2 for pair in result_pairs):
        raise TypeError("Density failed prefix requires immutable identity/result pairs")
    ids = tuple(identity for identity, _ in result_pairs)
    results = tuple(result for _, result in result_pairs)
    if ids != identities[:len(ids)] or len(ids) > len(identities):
        raise ValueError("Density failed result identities are not the exact input prefix")
    _validate_result_prefix(raw_inputs, results)
    if not results or results[-1].failure is None:
        raise ValueError("Density failed prefix requires a terminal named failure")
    canon_ids.validate_candidate()
    events = DensityShellEventCollector() if shell_event_collector is None else shell_event_collector
    articles = []
    for (identity, result) in result_pairs:
        articles.extend(result.ordered_work_articles)
        events.replay_canon(identity, result.canon_usage, canon_ids)
        events.replay(identity, format_shell_events(result.shell_events))
    return _failure_attachment(results[-1], events, tuple(articles))
