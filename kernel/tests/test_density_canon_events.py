"""Именованный construction-канон переживает отказ, memo и перенос результата."""

from dataclasses import replace
import pickle

import pytest

from cftuv_envelope.reference.contracts import ReferenceCompileResultV1, ReferenceOutcome
from cftuv_envelope.reference.density_oracle_v1 import (
    CanonUsageFactsV1, DensityOracleCanonV1, DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1,
    DENSITY_SOURCE_CONSTRUCTION_V1, DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED,
    DENSITY_SHELL_CANON_V2_CANDIDATE,
)
from cftuv_envelope.reference.density_shell_events import (
    DensityShellEventCollector, begin_density_shell_compilation,
    density_shell_materialization_facts, with_density_shell_diagnostics,
)


CANONS = DensityOracleCanonV1(
    DENSITY_SOURCE_CONSTRUCTION_V1, DENSITY_SHELL_CANON_V2_CANDIDATE, DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1,
)
USAGE = CanonUsageFactsV1(DENSITY_SOURCE_CONSTRUCTION_V1, DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED)


def test_unused_math_emits_nothing_and_reset_is_per_compilation():
    collector = DensityShellEventCollector()
    collector.replay_canon("untouched", (), CANONS)
    assert collector.diagnostics() == ()
    collector.replay_canon("touched", (USAGE,), CANONS)
    assert len(collector.diagnostics()) == 1
    assert begin_density_shell_compilation(collector) is collector
    assert collector.diagnostics() == ()


def test_cold_warm_pickle_and_reused_values_keep_current_spec_usage():
    collector = DensityShellEventCollector()
    collector.replay_canon("spec-b", (USAGE,), CANONS)
    collector.replay_canon("spec-b", pickle.loads(pickle.dumps((USAGE,))), CANONS)
    collector.replay_canon("spec-a", (USAGE,), CANONS)
    diagnostics = collector.diagnostics()
    assert tuple(item.envelope_spec_id for item in diagnostics) == ("spec-a", "spec-b")
    assert all(item.outcome is ReferenceOutcome.DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED for item in diagnostics)
    assert all(CANONS.angle in item.message and CANONS.shell in item.message for item in diagnostics)
    assert pickle.loads(pickle.dumps(collector)).diagnostics() == diagnostics
    counters, lines = density_shell_materialization_facts(diagnostics)
    assert counters == ((DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED, 2),)
    assert len(lines) == 2
    assert all("EXACT_WORK" not in key for key, _ in counters)


def test_canon_usage_survives_an_operation_refusal_without_compilation():
    collector = DensityShellEventCollector()
    collector.replay_canon("failed-spec", (USAGE,), CANONS)
    failure = ReferenceCompileResultV1(ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE, None, ())
    result = with_density_shell_diagnostics(failure, collector)
    assert result.outcome is failure.outcome
    assert result.compilation is None
    assert result.diagnostics == collector.diagnostics()
    assert with_density_shell_diagnostics(result, collector) == result


def test_unknown_usage_does_not_record_a_partial_bundle():
    collector = DensityShellEventCollector()
    with pytest.raises(ValueError, match="USAGE_MISMATCH"):
        collector.replay_canon("spec", (USAGE, replace(USAGE, canon="unknown")), CANONS)
    assert collector.diagnostics() == ()
    with pytest.raises(ValueError, match="CANON_MISMATCH"):
        collector.replay_canon("spec", (USAGE,), replace(CANONS, angle="unknown"))
    assert collector.diagnostics() == ()
