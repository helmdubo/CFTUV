"""Пассивные value-outcomes shell: capture веера и uses одной compilation.

Здесь нет выбора shell-канона, вычислителя или счётчиков времени жизни metric.
Отказ и cache hit переносят тот же immutable bundle в caller-owned collector.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, replace

from .contracts import ReferenceCompileResultV1, ReferenceDiagnosticSeverity, ReferenceEvaluationDiagnosticV1, ReferenceOutcome


SHELL_EVENT_OUTCOMES = (
    ReferenceOutcome.DENSITY_DECIMAL_SHELL_V2_APPLIED,
    ReferenceOutcome.DENSITY_DECIMAL_GRID_POINT_V2,
    ReferenceOutcome.DENSITY_DECIMAL_NON_DYADIC_GRID_STRAW_V2,
)
SOURCE_CANON_OUTCOME = ReferenceOutcome.DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED
DENSITY_CANON_EVENT_OUTCOMES = (*SHELL_EVENT_OUTCOMES, SOURCE_CANON_OUTCOME)


@dataclass(frozen=True, slots=True)
class DensityFanBindingMemoEntry:
    result: tuple
    shell_outcomes: tuple


class DensityShellEventCollector:
    """Только текущая compilation; semantic use = (spec_id, value wire)."""

    def __init__(self):
        self._uses: dict[tuple[str, tuple[int, ...]], object] = {}
        self._canon_uses: dict[tuple[str, str, str, str], None] = {}

    def reset(self):
        self._uses.clear()
        self._canon_uses.clear()

    def replay(self, spec_id, bundle):
        for value_wire, outcomes in bundle:
            self._uses[(spec_id, value_wire)] = outcomes

    def replay_canon(self, spec_id, bundle, canon_ids):
        """Вызванная owned-арифметика: один исход на spec, в том числе при отказе/memo."""

        canon_ids.validate_candidate()
        usages = tuple(bundle)
        if any(usage.canon != canon_ids.construction or usage.applied_event != SOURCE_CANON_OUTCOME.value
               for usage in usages):
            raise ValueError("DENSITY_SOURCE_CANON_USAGE_MISMATCH")
        if usages:
            self._canon_uses[(spec_id, canon_ids.construction, canon_ids.shell, canon_ids.angle)] = None

    def diagnostics(self):
        result = []
        for (spec_id, value_wire), bundle in sorted(self._uses.items()):
            message = _outcome_message(value_wire, bundle)
            for code in bundle.codes:
                result.append(ReferenceEvaluationDiagnosticV1(
                    outcome=ReferenceOutcome(code.value),
                    severity=ReferenceDiagnosticSeverity.INFO,
                    message=message,
                    envelope_spec_id=spec_id,
                ))
        result.extend(ReferenceEvaluationDiagnosticV1(
            outcome=SOURCE_CANON_OUTCOME,
            severity=ReferenceDiagnosticSeverity.INFO,
            message=f"construction={construction} shell={shell} angle={angle}",
            envelope_spec_id=spec_id,
        ) for spec_id, construction, shell, angle in sorted(self._canon_uses))
        return tuple(result)


def _outcome_message(value_wire, bundle):
    parts = [f"value_wire={value_wire}"]
    if bundle.rational is not None:
        rational = bundle.rational
        parts.extend((f"numerator={rational.numerator}", f"denominator={rational.denominator}"))
        if rational.grid_units is not None:
            parts.append(f"grid_units={rational.grid_units}")
    if bundle.legacy_projected_min_width_units is not None:
        parts.append(f"legacy_projected_min_width_units={bundle.legacy_projected_min_width_units}")
    return " ".join(parts)


@contextmanager
def capture_density_shell_candidates(metric):
    memo = metric._density_exact_memo
    previous = memo.shell_event_capture
    captured = {}
    memo.shell_event_capture = captured
    try:
        yield captured
    finally:
        memo.shell_event_capture = previous
        if previous is not None:
            previous.update(captured)


def record_density_shell_candidate(metric, candidate):
    """Hook только accepted candidate; вне build capture ничего не записывает."""

    if metric is None:
        return
    captured = metric._density_exact_memo.shell_event_capture
    if captured is not None:
        captured[candidate.value.wire()] = candidate.outcomes


def replay_density_shell_outcomes(metric, spec_id, bundle, collector):
    captured = metric._density_exact_memo.shell_event_capture
    if captured is not None:
        captured.update(bundle)
    if collector is not None:
        collector.replay(spec_id, bundle)


def with_density_shell_diagnostics(result, collector):
    diagnostics = collector.diagnostics()
    if not diagnostics:
        return result
    compilation = result.compilation
    if compilation is not None:
        compilation = replace(compilation, diagnostics=_merge_diagnostics(compilation.diagnostics, diagnostics))
    return replace(result, compilation=compilation, diagnostics=_merge_diagnostics(result.diagnostics, diagnostics))


def begin_density_shell_compilation(collector):
    if collector is None:
        return DensityShellEventCollector()
    collector.reset()
    return collector


def density_shell_compile_result(value, collector):
    if not isinstance(value, ReferenceCompileResultV1):
        value = ReferenceCompileResultV1(ReferenceOutcome.EXACT, value, ())
    return with_density_shell_diagnostics(value, collector)


def _merge_diagnostics(existing, added):
    return (*existing, *(item for item in added if item not in existing))


def density_shell_materialization_facts(diagnostics):
    """Именованные Density-каноны: event counters, не цена EXACT_WORK."""

    selected = tuple(item for item in diagnostics if item.outcome in DENSITY_CANON_EVENT_OUTCOMES)
    counters = tuple((outcome.value, sum(item.outcome is outcome for item in selected))
                     for outcome in DENSITY_CANON_EVENT_OUTCOMES if any(item.outcome is outcome for item in selected))
    lines = tuple(f"{item.outcome.value}: {item.envelope_spec_id}: {item.message}" for item in selected)
    return counters, lines
