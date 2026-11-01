"""CLIP_MEMO_KEY_NATIVE_V1: the key of the clip memory (`clip_memo.clip_key`) in Rust (`memo_key.rs`), against the oracle.

Every key the cases ask for is the oracle's hexdigest; the record walks every branch of `_encode` the same way (a hand-made value with each kind of node); a value the record
does not know is declined by the port and refused by the oracle with `ClipKeyUnsupported`.
"""

from __future__ import annotations

import enum
import sys
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "clip_memo_key"), reason="расширение cftuv_native не собрано (или собрано без CLIP_MEMO_KEY_NATIVE_V1)"
)


@pytest.fixture
def approved():
    from cftuv_native import materialize_hot_op as op
    from cftuv_native import pin

    op.refresh()
    pin.refresh()
    assert op.status() == "available", op.status()
    yield
    op.refresh()


class _Colour(enum.Enum):
    RED = "r"
    BLUE = 2


@dataclass(frozen=True)
class _Record:
    first: object
    second: object = None


def test_the_clip_memory_routes_its_key_through_the_dispatcher():
    text = (ROOT / "kernel/src/cftuv_envelope/materialize/clip_memo.py").read_text(encoding="utf-8")
    assert "key = backend.clip_key_compute(clip_key, inputs, plane.triangles, policy)" in text


@needs_native
def test_every_key_of_the_cases_is_the_oracles(approved):
    from cftuv_envelope import backend
    from cftuv_envelope.materialize import clip_memo
    from test_native_surface_lift import _cases

    calls = []
    saved = backend.clip_key_compute

    def capture(oracle, inputs, triangles, policy):
        calls.append((dict(inputs), triangles, policy))
        return saved(oracle, inputs, triangles, policy)

    backend.clip_key_compute = capture
    try:
        for prepare, laws, alpha in _cases():
            clip_memo.MEMO.clear()
            _keys_of(prepare(), laws, alpha)
    finally:
        backend.clip_key_compute = saved
        clip_memo.MEMO.clear()
    assert len(calls) >= 6, "the cases asked for no key: the test would prove nothing"
    for inputs, triangles, policy in calls:
        assert cftuv_native.clip_memo_key(inputs, triangles, policy) == clip_memo.clip_key(inputs, triangles, policy)


def _keys_of(prepared, laws, alpha):
    """One width of `prepared` with the clip memory ON (the key is asked only then), under the NATIVE W1 stage."""

    from cftuv_envelope import backend
    from cftuv_envelope.materialize.admit import materialization_request
    from cftuv_envelope.materialize.step import step_domain

    text = str(prepared.requested_alpha.value) if alpha is None else alpha
    with backend.use_backend("PYTHON", materialize_backend="NATIVE"):
        step_domain(
            prepared, text, request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"), near_planar_lift_law=laws[0], decal_topology_law=laws[1],
            digests=False, parametric=True,
        )


@needs_native
def test_every_branch_of_the_record_is_the_oracles(approved):
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
    from cftuv_envelope.materialize import clip_memo

    value = {
        "ints": (0, -1, 7, 2**70, -(3**50)),
        "fractions": [Fraction(1, 3), Fraction(-22, 7), Fraction(2**80, 3)],
        "texts": ("", "ascii", "кириллица", "emoji \U0001F600", "a;b=c"),
        "flags": (True, False, None),
        "floats": (0.0, -0.0, 1.5, 1e-300, float("inf")),
        "sets": (frozenset({"b", "a", "c"}), {3, 1, 2}, frozenset({frozenset({1, 2}), frozenset({3})})),
        "nested": {("k", 1): [(), [], {}, frozenset()]},
        "enums": (_Colour.RED, _Colour.BLUE),
        "records": (_Record(1), _Record(SqrtSumV1(((2, Fraction(1, 2)), (3, Fraction(-1)))), _Record((1, 2), "x"))),
    }
    policy = (("bits", 64), ("chord", Fraction(1, 1024)))
    triangles = (_Record("t0", (SqrtSumV1.rational(1), SqrtSumV1.zero())),)
    assert cftuv_native.clip_memo_key(value, triangles, policy) == clip_memo.clip_key(value, triangles, policy)


@needs_native
def test_a_value_the_record_does_not_know_is_declined_and_the_oracle_refuses(approved):
    from cftuv_envelope.materialize import clip_memo

    class _Plain:
        pass

    for odd in (_Plain(), _Record, b"bytes", (1, [2, {3: object()}])):
        with pytest.raises(cftuv_native.NATIVE_REFUSALS):
            cftuv_native.clip_memo_key({"odd": odd}, (), ())
        with pytest.raises(clip_memo.ClipKeyUnsupported):
            clip_memo.clip_key({"odd": odd}, (), ())
