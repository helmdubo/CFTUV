"""Реестр UV-законов запроса: хост объявляет, ядро умеет — и имена не расходятся."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv.envelope_request_policy import (  # noqa: E402
    DEFAULT_ENVELOPE_MAX_STRETCH_PERCENT,
    DEFAULT_ENVELOPE_STRETCH_BUDGET,
    ENVELOPE_MAX_STRETCH_PERCENT_RANGE,
    ENVELOPE_UV_POLICIES,
    ENVELOPE_UV_POLICY_DEBUG_NO_UV,
    ENVELOPE_UV_POLICY_DIRECT_STRIP,
    build_envelope_request_contract,
    envelope_angular_policy,
    envelope_decal_request_id_value,
    envelope_request_policy_signature,
    envelope_stretch_budget,
)


def _request(**kwargs):
    import cftuv_envelope as kernel
    from decimal import Decimal

    return build_envelope_request_contract(
        kernel,
        kernel.DecalRequestId("request"),
        frozenset(),
        Decimal("0.25"),
        envelope_angular_policy(kernel, None),
        **kwargs,
    )


def test_the_host_registry_names_exactly_the_laws_the_kernel_can_materialize():
    """Продуктовый закон хоста — тот, что умеет материализатор; отладочный — нет."""

    from cftuv_envelope.materialize.uv_law import SUPPORTED_UV_POLICIES

    assert ENVELOPE_UV_POLICY_DIRECT_STRIP in ENVELOPE_UV_POLICIES
    assert {item.value for item in SUPPORTED_UV_POLICIES} == {
        ENVELOPE_UV_POLICY_DIRECT_STRIP
    }
    assert ENVELOPE_UV_POLICY_DEBUG_NO_UV not in {
        item.value for item in SUPPORTED_UV_POLICIES
    }


def test_the_debug_request_keeps_its_policy_and_the_product_one_is_selectable():
    assert _request().uv_policy_id.value == "ENVELOPE_DEBUG_NO_UV_V1"
    chosen = _request(uv_policy_id=ENVELOPE_UV_POLICY_DIRECT_STRIP)
    assert chosen.uv_policy_id.value == "UV_DIRECT_STRIP_V1"
    # Закон — единственное, что отличает запросы.
    import dataclasses

    assert dataclasses.replace(
        chosen, uv_policy_id=_request().uv_policy_id
    ) == _request()


def test_an_unknown_uv_policy_is_an_error_and_not_a_silent_substitution():
    with pytest.raises(ValueError, match="unknown UV policy"):
        _request(uv_policy_id="SOMETHING_ELSE")


# --------------------------------------------------------------------------
# Допуск растяжения развёртки — политика запроса («Max stretch» панели)
# --------------------------------------------------------------------------


def test_the_panel_percent_is_an_exact_rational_and_the_default_is_the_kernels():
    from fractions import Fraction

    from cftuv_envelope.contracts.metric import DEFAULT_DEVELOPABLE_STRETCH_BUDGET

    assert DEFAULT_ENVELOPE_MAX_STRETCH_PERCENT == 20
    assert DEFAULT_ENVELOPE_STRETCH_BUDGET == DEFAULT_DEVELOPABLE_STRETCH_BUDGET == Fraction(1, 5)
    assert ENVELOPE_MAX_STRETCH_PERCENT_RANGE == (1, 50)
    # Умолчание панели запросу не нужно называть: запрос с умолчанием побитово прежний.
    assert envelope_stretch_budget(None) is None
    assert envelope_stretch_budget(20) is None
    assert envelope_stretch_budget(35) == Fraction(7, 20)
    assert envelope_stretch_budget(1) == Fraction(1, 100)
    assert envelope_stretch_budget(50) == Fraction(1, 2)


@pytest.mark.parametrize("percent", (0, 51, -5, True, 12.5, "20"))
def test_a_percent_outside_the_panel_or_not_an_exact_int_is_an_error(percent):
    with pytest.raises(ValueError, match="Max stretch"):
        envelope_stretch_budget(percent)


def test_a_request_without_a_budget_is_the_legacy_request_byte_for_byte():
    import cftuv_envelope as kernel

    request = _request()
    assert request.developable_stretch_budget == kernel.ExactRationalV1(1, 5)
    assert b"developable_stretch_budget" not in kernel.DecalRequestCodecV1.dumps(request)
    named_default = build_envelope_request_contract(
        kernel,
        kernel.DecalRequestId("request"),
        frozenset(),
        __import__("decimal").Decimal("0.25"),
        envelope_angular_policy(kernel, None, DEFAULT_ENVELOPE_STRETCH_BUDGET),
    )
    assert named_default == request


def test_a_non_default_budget_reaches_the_request_the_policy_signature_and_the_request_id():
    from fractions import Fraction

    import cftuv_envelope as kernel
    from decimal import Decimal

    def build(budget, density=None):
        policy = envelope_angular_policy(kernel, density, budget)
        return policy, build_envelope_request_contract(
            kernel, kernel.DecalRequestId("request"), frozenset(), Decimal("0.25"), policy
        )

    default_policy, default = build(None)
    wide_policy, wide = build(Fraction(7, 20))
    assert wide.developable_stretch_budget == kernel.ExactRationalV1(7, 20)
    assert envelope_request_policy_signature(default)[-1] == "1/5"
    assert envelope_request_policy_signature(wide)[-1] == "7/20"
    assert envelope_request_policy_signature(default)[:-1] == envelope_request_policy_signature(wide)[:-1]

    def request_id(policy):
        def typed(*parts):
            return "|".join(str(item) for item in parts)

        return envelope_decal_request_id_value(typed, "rev", (), "base", policy)

    assert request_id(default_policy) == "base"
    assert request_id(wide_policy) != "base"
    assert request_id(wide_policy) == envelope_decal_request_id_value(
        lambda *parts: "|".join(str(item) for item in parts), "rev", (), "base", wide_policy
    )
    # Плотность и допуск складываются: оба выбора видны в идентичности запроса.
    density_policy, _ = build(None, 2)
    both_policy, _ = build(Fraction(7, 20), 2)
    assert len({request_id(default_policy), request_id(density_policy), request_id(wide_policy), request_id(both_policy)}) == 4


def test_every_host_caller_that_passes_the_fan_density_also_passes_the_stretch_budget():
    """Ручки запроса идут парой: вызов без допуска вернул бы молча умолчание ядра при другом числе на панели."""

    import ast

    host = Path(__file__).resolve().parents[1] / "cftuv"
    seen = 0
    for name in ("operators.py", "envelope_production_operator.py"):
        tree = ast.parse((host / name).read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            callee = node.func.id if isinstance(node.func, ast.Name) else getattr(node.func, "attr", "")
            if callee not in {"evaluate_envelope_debug_staged", "run_production"}:
                continue
            keywords = {item.arg for item in node.keywords}
            if "density" in keywords:
                seen += 1
                assert "developable_stretch_budget" in keywords, (name, node.lineno)
    assert seen == 2


def test_the_panel_draws_max_stretch_next_to_fan_density_and_the_property_is_an_int_percent():
    import ast

    host = Path(__file__).resolve().parents[1] / "cftuv"
    panel = (host / "envelope_debug_panel.py").read_text(encoding="utf-8")
    assert panel.index('"envelope_debug_fan_density"') < panel.index('"envelope_debug_max_stretch"')
    assert panel.index('"envelope_debug_max_stretch"') < panel.index('"envelope_debug_workers"')
    tree = ast.parse((host / "operators.py").read_text(encoding="utf-8"))
    declared = {
        node.target.id: node.annotation
        for node in ast.walk(tree)
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name)
    }
    prop = declared["envelope_debug_max_stretch"]
    assert prop.func.id == "IntProperty"
    keywords = {item.arg: ast.unparse(item.value) for item in prop.keywords}
    assert keywords["default"] == "DEFAULT_ENVELOPE_MAX_STRETCH_PERCENT"
    assert keywords["update"] == "_update_envelope_debug_max_stretch"
