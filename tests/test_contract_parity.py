"""CONTRACT_PARITY_V1: the gate of the differential tests of the Rust-product stages (`native_corpus.parity_differences`), without the extension.

A differential test compares the native outcome of a call with the Python reference on the same state. The gate is exact first (cheap and the usual case);
when the bytes differ, only what NATIVE_PATH_PRODUCT_CONTRACT_V1 binds can fail it: the answer by exact value and in its order, the named outcome, the
station conflict, the observations of the arguments. The representation of numbers, the container kind, the diagnostic text, the service counters (budget
articles, SIGN_COUNTS, the canonicalization memory, the result counters) and the operation at which a work budget stops (RESOURCE_POLICY) are free.

Here the gate is proven on a real answer of the reference (the skeleton of a square): every GEOMETRY_ERROR-class mutation is named, every
representation-only one passes, and `compare_outcomes` (the exact comparison the (a)-class tests keep) still names the representation-only ones.
"""

from __future__ import annotations

import dataclasses
import sys
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import native_corpus as nc  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
from cftuv_envelope.wavefront.polygon import PolygonV1  # noqa: E402
from cftuv_envelope.wavefront.skeleton import build_skeleton  # noqa: E402


@pytest.fixture(scope="module")
def answered():
    with exact.isolated_factorization_memory():
        budget = exact.exact_work_budget(stage="PREPARE", domain_id="contract-parity", superlevel="", cap=1 << 20)
        before = nc.capture_state(budget, None)
        result = build_skeleton(PolygonV1.build([(0, 0), (6, 0), (6, 2), (3, 4), (0, 2)]), work_budget=budget)
        after = nc.capture_state(budget, None)
    return before, nc.Outcome(result, None, after, {})


def test_an_identical_outcome_passes_both_gates(answered):
    before, outcome = answered
    assert nc.compare_outcomes(nc.OP_SKELETON, before, outcome, outcome) == []
    assert nc.parity_differences(nc.OP_SKELETON, before, outcome, outcome) == []


def test_a_representation_only_difference_fails_the_bytes_and_passes_the_contract(answered):
    before, outcome = answered
    other = nc.mutate_representation(nc.OP_SKELETON, outcome)
    exact_differences = {item.field for item in nc.compare_outcomes(nc.OP_SKELETON, before, outcome, other)}
    assert {"result.nodes", "budget.delta", "sign_counts.contract-parity-mutation", "memory.known_primes"} <= exact_differences, exact_differences
    assert nc.parity_differences(nc.OP_SKELETON, before, outcome, other) == []
    kinds = {kind for kind, _item in nc.contract_differences(nc.OP_SKELETON, before, outcome, other)}
    assert kinds == {nc.REPRESENTATION_ONLY}


def test_int_and_fraction_tuple_and_list_and_mapping_order_are_one_value_under_the_contract():
    assert nc.contract_canonical((1, Fraction(1, 2), 0.25)) == nc.contract_canonical([Fraction(1), Fraction(2, 4), Fraction(1, 4)])
    assert nc.contract_canonical({"a": 1, "b": 2}) == nc.contract_canonical({"b": 2, "a": 1})
    assert nc.canonical((1,)) != nc.canonical((Fraction(1),)), "the bytes still tell them apart"
    assert nc.contract_canonical((1, 2)) != nc.contract_canonical((2, 1)), "the order of a sequence is part of the answer"
    assert nc.contract_canonical(True) != nc.contract_canonical(1)
    assert nc.contract_canonical(0.1) != nc.contract_canonical(Fraction(1, 10)), "a binary64 is its exact dyadic value"


def test_the_service_counters_of_the_result_are_free_and_the_answer_is_not(answered):
    before, outcome = answered
    counters = tuple((name, value + 1) for name, value in outcome.result.counters)
    moved = dataclasses.replace(outcome, result=dataclasses.replace(outcome.result, counters=counters))
    assert nc.parity_differences(nc.OP_SKELETON, before, outcome, moved) == []
    levels = dataclasses.replace(outcome, result=dataclasses.replace(outcome.result, levels=outcome.result.levels + 1))
    assert [item.field for item in nc.parity_differences(nc.OP_SKELETON, before, outcome, levels)] == ["result.levels"]


def test_a_moved_geometry_is_a_geometry_error(answered):
    before, outcome = answered
    moved = nc.mutate_geometry(nc.OP_SKELETON, outcome)
    assert [item.field for item in nc.parity_differences(nc.OP_SKELETON, before, outcome, moved)] == ["result.nodes"]


def test_a_reordered_answer_a_missing_answer_and_another_named_outcome_are_geometry_errors(answered):
    before, outcome = answered
    nodes = outcome.result.nodes
    assert len(nodes) >= 2
    swapped = dataclasses.replace(outcome, result=dataclasses.replace(outcome.result, nodes=(nodes[1], nodes[0], *nodes[2:])))
    assert [item.field for item in nc.parity_differences(nc.OP_SKELETON, before, outcome, swapped)] == ["result.nodes"]
    raised = dataclasses.replace(outcome, result=None, exception=("ValueError", "boom"))
    assert {item.field for item in nc.parity_differences(nc.OP_SKELETON, before, outcome, raised)} == {"exception", "result"}
    other = dataclasses.replace(outcome, result=None, exception=("KeyError", "boom"))
    assert [item.field for item in nc.parity_differences(nc.OP_SKELETON, before, raised, other)] == ["exception"]
    reworded = dataclasses.replace(raised, exception=("ValueError", "another text"))
    assert nc.parity_differences(nc.OP_SKELETON, before, raised, reworded) == []
    conflict = dataclasses.replace(raised, exception=("ValueError", "boom", ((1, 2),)))
    assert [item.field for item in nc.parity_differences(nc.OP_SKELETON, before, raised, conflict)] == ["exception.station_conflict"]


def test_a_work_budget_that_stops_on_one_side_only_is_resource_policy_not_a_geometry_error(answered):
    before, outcome = answered
    exhausted = dataclasses.replace(outcome, result=None, exception=("ExactCanonicalizationWorkBudgetExhausted", "spent"))
    assert nc.parity_differences(nc.OP_SKELETON, before, outcome, exhausted) == []
    assert [kind for kind, _item in nc.contract_differences(nc.OP_SKELETON, before, outcome, exhausted)][:1] == [nc.RESOURCE_POLICY]


def test_an_observation_of_the_arguments_is_bound(answered):
    before, outcome = answered
    left = dataclasses.replace(outcome, observed={"normals": ((0, 0, 1),)})
    right = dataclasses.replace(outcome, observed={"normals": [(Fraction(0), 0, 1)]})
    assert nc.parity_differences(nc.OP_SKELETON, before, left, right) == []
    moved = dataclasses.replace(outcome, observed={"normals": ((0, 1, 0),)})
    assert [item.field for item in nc.parity_differences(nc.OP_SKELETON, before, left, moved)] == ["observed.normals"]
