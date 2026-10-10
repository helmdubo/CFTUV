"""`EXACT_SCALAR_TEXT_CANON_UNSUPPORTED` на продуктовом пути: именованный исход домена вместо общего `PRODUCTION_DOMAIN_RAISED`.

Ядро называет отказ материализации и шага ширины само (`MaterializationOutcome.EXACT_SCALAR_TEXT_CANON_UNSUPPORTED`), хост пропускает имя как есть;
исключение, которое ядро не успело назвать (подготовка, любая ступень вне шага), хост называет тем же словом (`OUTCOME_CANON_UNSUPPORTED`).
Любое другое исключение остаётся `PRODUCTION_DOMAIN_RAISED`: имя не расползается на чужие сбои.
"""

from __future__ import annotations

import sys
from fractions import Fraction
from pathlib import Path

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv.envelope_production_export import (  # noqa: E402
    OUTCOME_CANON_UNSUPPORTED,
    OUTCOME_DOMAIN_RAISED,
    produce_domain,
)
from cftuv_envelope.materialize.admit import MaterializationOutcome  # noqa: E402
from cftuv_envelope.reference import native_exact as nx  # noqa: E402
from cftuv_envelope.reference.planar_types import ExactScalar  # noqa: E402

from envelope_fixture_bundles import quad_row_bundle  # noqa: E402
from test_envelope_production_export import ROW, _prepared_domain, _production  # noqa: E402

NAME = "EXACT_SCALAR_TEXT_CANON_UNSUPPORTED"


def _raise_for_real(*_arguments, **_keywords):
    """Исключение бросает настоящий код ядра: два класса квадратов не имеют канонической строки V2."""

    two_classes = nx.RadicalSumV1.sqrt_of_rational(Fraction(2)) + nx.RadicalSumV1.sqrt_of_rational(Fraction(3))
    ExactScalar.from_value(two_classes)
    raise AssertionError("a multi-term RadicalSumV1 must have no canonical text")


def test_the_host_mirror_is_the_kernel_name_and_not_the_generic_outcome():
    assert OUTCOME_CANON_UNSUPPORTED == NAME == nx.EXACT_SCALAR_TEXT_CANON_UNSUPPORTED
    assert MaterializationOutcome(OUTCOME_CANON_UNSUPPORTED) is MaterializationOutcome.EXACT_SCALAR_TEXT_CANON_UNSUPPORTED
    assert OUTCOME_CANON_UNSUPPORTED != OUTCOME_DOMAIN_RAISED


def test_a_refusal_the_kernel_names_in_materialization_reaches_the_host_by_that_name(monkeypatch):
    from cftuv_envelope.materialize import domain as kernel_domain

    prepared = _prepared_domain()
    monkeypatch.setattr(kernel_domain, "_build", _raise_for_real)

    refused = produce_domain(1, "domain", prepared, "0.25")

    assert refused.outcome == NAME and refused.outcome != OUTCOME_DOMAIN_RAISED
    assert refused.batch is None and refused.detail.startswith(NAME + ":")


def test_an_exception_the_kernel_did_not_name_is_named_by_the_host_with_the_same_word(monkeypatch):
    from cftuv_envelope.materialize import step as kernel_step

    prepared = _prepared_domain()
    monkeypatch.setattr(kernel_step, "step_domain", _raise_for_real)

    refused = produce_domain(2, "domain", prepared, "0.25")

    assert refused.outcome == OUTCOME_CANON_UNSUPPORTED
    assert refused.batch is None and refused.detail.startswith(NAME + ":") and refused.patch_id == 2


def test_another_exception_is_still_the_generic_outcome(monkeypatch):
    from cftuv_envelope.materialize import step as kernel_step

    prepared = _prepared_domain()

    def broken(*_arguments, **_keywords):
        raise RuntimeError("kernel bug")

    monkeypatch.setattr(kernel_step, "step_domain", broken)

    refused = produce_domain(3, "domain", prepared, "0.25")

    assert refused.outcome == OUTCOME_DOMAIN_RAISED and "kernel bug" in refused.detail


def test_a_preparation_that_raises_it_names_every_domain_of_the_run(monkeypatch):
    from cftuv import envelope_production_export as production

    monkeypatch.setattr(production, "prepare_for_production_recorded", _raise_for_real)

    run, _controller = _production(quad_row_bundle(ROW))

    assert len(run.results) == ROW
    assert {item.outcome for item in run.results} == {OUTCOME_CANON_UNSUPPORTED}
    assert all(item.batch is None and item.detail.startswith(NAME + ":") for item in run.results)
