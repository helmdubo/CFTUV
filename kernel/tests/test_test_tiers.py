"""TEST_TIERS_V1: local runs are the product tier, CI (and `CFTUV_STRICT_TESTS=1`) the strict one; the individual switches win."""

from __future__ import annotations

import os

import pytest

from cftuv_envelope.exact_sqrt_sum import canonical_audit_enabled
from kernel_test_tiers import strict_tier, tier_switches


@pytest.mark.parametrize(
    "environment, strict",
    [({}, False), ({"CI": "true"}, True), ({"CFTUV_STRICT_TESTS": "1"}, True), ({"CI": "true", "CFTUV_STRICT_TESTS": "0"}, False), ({"CI": ""}, False)],
)
def test_the_tier_of_an_environment(environment, strict):
    assert strict_tier(environment) is strict


def test_the_switches_of_each_tier_and_the_individual_overrides():
    assert tier_switches({}) == (False, None)
    assert tier_switches({"CFTUV_STRICT_TESTS": "1"}) == (True, "1")
    assert tier_switches({"CFTUV_STRICT_TESTS": "1", "CFTUV_CANONICAL_AUDIT": "0"}) == (False, "1")
    assert tier_switches({"CFTUV_CANONICAL_AUDIT": "1"}) == (True, None)
    assert tier_switches({"CFTUV_STRICT_TESTS": "1", "CFTUV_SYMBOLIC_REPLAY_CHECK": "0"}) == (True, None)


def test_a_misspelt_tier_is_refused():
    with pytest.raises(ValueError):
        strict_tier({"CFTUV_STRICT_TESTS": "yes"})


def test_this_session_runs_the_tier_its_environment_names():
    audit, _replay = tier_switches({key: value for key, value in os.environ.items() if key != "CFTUV_SYMBOLIC_REPLAY_CHECK"})
    assert canonical_audit_enabled() is audit
    if strict_tier() and "CFTUV_SYMBOLIC_REPLAY_CHECK" not in os.environ:
        pytest.fail("the strict tier left the replay check of the closure unset")
