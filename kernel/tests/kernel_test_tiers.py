"""TEST_TIERS_V1: which tier the kernel suite runs in.

STRICT (CI before merge, periodic full runs): the full canonicity audit of the exact numbers (`set_canonical_audit`: every quantity entering the machine of
times is checked square-free, independently of the canonicalization memory) and the second closure pass of determinism (`CFTUV_SYMBOLIC_REPLAY_CHECK=1`:
the closure of a packet is planned twice and compared). PRODUCT (local iteration): the switches as the product runs them, both off.

`CFTUV_STRICT_TESTS=1` asks for the strict tier, `CFTUV_STRICT_TESTS=0` for the product tier; without it the tier is strict exactly when `CI` is set.
The individual switches still win when set (`CFTUV_CANONICAL_AUDIT`, `CFTUV_SYMBOLIC_REPLAY_CHECK`). The CI workflows pin `CFTUV_STRICT_TESTS: "1"`
(tests/test_architecture.py checks it).
"""

from __future__ import annotations

import os

ENVIRONMENT = "CFTUV_STRICT_TESTS"


def strict_tier(environment=None) -> bool:
    environment = os.environ if environment is None else environment
    explicit = str(environment.get(ENVIRONMENT, "")).strip()
    if explicit:
        if explicit not in ("0", "1"):
            raise ValueError(f"{ENVIRONMENT}={explicit!r}: 1 (strict tier) or 0 (product tier)")
        return explicit == "1"
    return bool(str(environment.get("CI", "")).strip())


def tier_switches(environment=None) -> tuple[bool, str | None]:
    """`(canonical audit on, value to set for CFTUV_SYMBOLIC_REPLAY_CHECK or None to leave it)` of the tier, the individual switches first."""

    environment = os.environ if environment is None else environment
    strict = strict_tier(environment)
    audit = str(environment.get("CFTUV_CANONICAL_AUDIT", "1" if strict else "0")) != "0"
    replay = None if "CFTUV_SYMBOLIC_REPLAY_CHECK" in environment or not strict else "1"
    return audit, replay
