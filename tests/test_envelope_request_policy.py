"""Реестр UV-законов запроса: хост объявляет, ядро умеет — и имена не расходятся."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv.envelope_request_policy import (  # noqa: E402
    ENVELOPE_UV_POLICIES,
    ENVELOPE_UV_POLICY_DEBUG_NO_UV,
    ENVELOPE_UV_POLICY_DIRECT_STRIP,
    build_envelope_request_contract,
    envelope_angular_policy,
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
