"""Найденный TABLE inverse witness не подменяет реальный producer/count recipe."""
from fractions import Fraction
from pathlib import Path
import sys

import pytest

from kernel_test_paths import KERNEL_ROOT

sys.path.insert(0, str(KERNEL_ROOT / "tools"))
import density_inverse_reachability as research
from cftuv_envelope.reference.density_scalar_v2 import ScalarV2


@pytest.fixture(scope="module")
def evidence():
    return research.evidence()


def test_original_recipe_source_count_is_feasible_without_h3_lift(evidence):
    facts = evidence["choices"][0]
    assert facts["g"] == 3 and facts["outgoing"] == (-3, 1)
    assert facts["bucket"] == 3 and facts["source_count"] == 2
    assert facts["counts"][0]["feasible"] is True
    assert facts["counts"][0]["rational_hidden"] == [True, True]
    assert facts["table_primitive_covectors"] == [None, None, None]
    invalid = evidence["invalid_gram_compile"]
    assert invalid["outcome"] == "REFERENCE_INPUT_CONTRACT_INVALID" and invalid["budgets"] == []
    assert any("Gram matrix does not equal the exact A/B dot products" in row["message"] for row in invalid["diagnostics"])


def test_table_denominator_is_real_four_class_math_but_not_the_actual_regular_ideal(evidence):
    table = next(w for w in evidence["windows"] if w["g"] == 3 and w["table"] and w["law"] == research.band.WINDOW_LAW_VORONOI)
    actual = next(w for w in evidence["windows"] if w["g"] == 3 and not w["table"] and w["law"] == research.band.WINDOW_LAW_VORONOI)
    # Четыре точных члена исходного witness; тест не зависит от внешнего receipt.
    witness = (ScalarV2.rational(Fraction(6, 13))
               + ScalarV2.sqrt_rational(3).scaled(Fraction(5, 78))
               + ScalarV2.sqrt_rational(2).scaled(Fraction(1, 4))
               + ScalarV2.sqrt_rational(6).scaled(Fraction(1, 12)))
    expected = witness.scaled(3).value.wire()
    denominator = table["slopes"][1]["denominator"]
    assert denominator["value_wire"] == expected  # actual slope читает G-ковектор: y умножен на 3
    assert denominator["class_count"] == 4
    assert denominator["profile"]["code"] == "DENSITY_SCALAR_V2_INVERSE_CLASS_LIMIT"
    assert actual["slopes"][0]["denominator"]["profile"]["code"] == "DENSITY_SCALAR_V2_ROOT_REQUIRES_NONNEGATIVE_RATIONAL"
    assert table["answer"]["status"] == actual["answer"]["status"] == "returned"
    narrow = next(w for w in evidence["windows"] if w["g"] == 3 and w["table"] and w["law"] == research.band.WINDOW_LAW_NARROW_BAND)
    assert all(row["denominator"].get("class_count") == 2 for row in narrow["slopes"])


def test_realizable_g5_does_not_allow_table_ideal_to_replace_actual_pi8_ideal(evidence):
    assert evidence["producer_g5"]["gram"]["m11"]["numerator"] == 5
    unchanged, adjusted = evidence["choices"][1:]
    assert unchanged["actual_dot"] == 2  # прежний outgoing уже не задаёт quarter turn
    assert adjusted["actual_dot"] == 0
    assert [row["feasible"] for row in adjusted["counts"]] == [False, True]
    assert adjusted["table_refusal"] == "CANONICAL_RAYS_IRRATIONAL_IN_CHART"
    for window in evidence["windows"]:
        if window["g"] == 5 and not window["table"]:
            assert window["slopes"][0]["denominator"]["profile"]["code"] == "DENSITY_SCALAR_V2_ROOT_REQUIRES_NONNEGATIVE_RATIONAL"
    binds = [b for b in evidence["binds"] if b["g"] == 5]
    assert all(b["answer"]["status"] == "returned" for b in binds)
    assert [b["answer"]["result"][1]["minimal_common_height"] for b in binds] == [5, 50]
    assert [(b["budgets"][-1]["shell_probes"], b["budgets"][-1]["order_steps"]) for b in binds] == [(15, 8), (154, 41)]
