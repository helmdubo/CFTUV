"""The native `codec.canonical_json_bytes` (MATERIALIZE_DIGEST_NATIVE_V1, `canonical_json.rs`) against the oracle: the same bytes, or a named decline where the
oracle raises.

The values: the geometry batches of the materialize seed (sealed, so the semantic digest is in them) and their semantic projections, plus synthetic trees
with every branch of `to_canonical_data` (ids, enums, Decimals of every shape, wire-default fields, escapes, big ints, floats at the repr boundaries).
"""

from __future__ import annotations

import dataclasses
import decimal
import enum
import lzma
import pickle
import random
import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "kernel" / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "kernel" / "src"))
if str(ROOT / "tests") not in sys.path:
    sys.path.insert(0, str(ROOT / "tests"))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "canonical_json_bytes"), reason="расширение cftuv_native не собрано (или собрано без canonical_json_bytes)"
)

SEED = ROOT / "tests" / "data" / "native_materialize_hot_seed.recs.xz"


@pytest.fixture
def approved():
    from cftuv_native import materialize_hot_op as op

    op.refresh()
    assert op.status() == "available", op.status()
    yield
    op.refresh()


def _same(value):
    """`"answered"` when both give the same bytes, `"declined"` when the oracle raises and the port declines by name."""

    from cftuv_envelope.codec import canonical_json_bytes

    try:
        expected = canonical_json_bytes(value)
    except Exception:  # noqa: BLE001 - the oracle's raise is an outcome
        with pytest.raises(cftuv_native.NATIVE_REFUSALS):
            cftuv_native.canonical_json_bytes(value)
        return "declined"
    assert cftuv_native.canonical_json_bytes(value) == expected
    return "answered"


class Colour(str, enum.Enum):
    RED = "RED"
    BLUE = "B\"lue\\"


class Rank(enum.IntEnum):
    LOW = 1
    HIGH = 10**30


@dataclasses.dataclass(frozen=True)
class Plain:
    zeta: object
    alpha: object
    middle_name: object = None


def _wire_record():
    from cftuv_envelope.schema import wire_default_field

    @dataclasses.dataclass(frozen=True)
    class Wired:
        kept: object
        omitted: frozenset = wire_default_field(frozenset())
        count: int = wire_default_field(0)

    return Wired


class Text(str):
    pass


class Number(int):
    pass


class Real(float):
    pass


@needs_native
def test_the_seed_batches_and_their_semantic_projections_are_the_same_bytes(approved):
    import test_native_materialize_hot as hot
    from cftuv_envelope import canonical
    from cftuv_envelope.materialize.assemble import assemble_batch

    with lzma.open(SEED, "rb") as handle:
        rows = [row for row in pickle.load(handle) if row["kind"] == "assemble"]
    from cftuv_envelope import backend

    projections = []

    def capture(oracle, value):
        projections.append(value)
        return oracle(value)

    saved = backend.canonical_bytes_compute
    backend.canonical_bytes_compute = capture
    try:
        batches = [canonical.sealed_geometry_batch(assemble_batch(**hot._assemble_kwargs(row))) for row in rows]
    finally:
        backend.canonical_bytes_compute = saved
    assert len(batches) >= 90 and len(projections) == len(batches)
    assert [_same(item) for item in batches] == ["answered"] * len(batches)
    assert [_same(item) for item in projections] == ["answered"] * len(projections)


@needs_native
def test_every_branch_of_the_canonical_tree(approved):
    from cftuv_envelope.ids import LineageId, VertexKey
    from cftuv_envelope.numeric import LocalCoordinateV1

    wired = _wire_record()
    values = [
        None, True, False, 0, -1, 2**70, -(2**200), "", "plain", "quote\" back\\slash", "\b\f\n\r\t\x00\x1f\x7f", "unit\x1fseparator only", "ünïcödé ✓ 𝄞", Text("sub"), Number(5),
        0.0, -0.0, 1.0, 0.1, 1e16, 1e-5, 5e-324, 1.7976931348623157e308, 963818134831184.2, Real(2.5),
        (), (1, (2, (3,))), frozenset(), frozenset({"b", "a", "ab", "", "é"}), frozenset({(1, 2), (1,), ()}),
        {}, {"b": 1, "a": {"z": (), "": None}, "é": "x", "$type": "a key, not a record"},
        VertexKey("node:1"), frozenset({LineageId("b"), LineageId("a")}), Colour.RED, Colour.BLUE, Rank.LOW, Rank.HIGH,
        decimal.Decimal("0"), decimal.Decimal("-0"), decimal.Decimal("0E-8"), decimal.Decimal("1.500"), decimal.Decimal("-0.00012"), decimal.Decimal("1.2E+5"),
        decimal.Decimal("1E-7"), decimal.Decimal("123456789012345678901234567.8"), LocalCoordinateV1(decimal.Decimal(1) / 7),
        Plain("z", ("a",), None), Plain(frozenset({Plain(1, 2), Plain(2, 1)}), {"k": Colour.RED}, middle_name="m"),
        wired("x"), wired("x", frozenset({"y"}), 3), wired("x", count=0), wired("x", count=Number(0)), wired("x", frozenset({"y"}) - {"y"}),
    ]
    assert [_same(item) for item in values] == ["answered"] * len(values)
    assert _same(tuple(values)) == "answered" and _same(frozenset(Plain(index, str(index)) for index in range(50))) == "answered"


@needs_native
def test_what_the_oracle_does_not_encode_is_declined_by_name(approved):
    declined = [
        [1, 2], {1, 2}, float("nan"), float("inf"), {1: "x"}, decimal.Decimal("NaN"), decimal.Decimal("Infinity"), "lone \ud800 surrogate",
        object(), Plain, bytes(b"x"), (1, [2]),
    ]
    assert [_same(item) for item in declined] == ["declined"] * len(declined)


@needs_native
def test_the_decimal_precision_is_the_callers_context(approved):
    """`_canonical_decimal` normalizes under the CALLER's context: 30 digits answer under prec 30 and are declined under 28 (where the oracle rounds)."""

    value = decimal.Decimal("123456789012345678901234567.891")
    with decimal.localcontext() as context:
        context.prec = 30
        assert _same(value) == "answered"
    from cftuv_envelope.codec import canonical_json_bytes

    assert canonical_json_bytes(value) != b'"123456789012345678901234567.891"'
    with pytest.raises(cftuv_native.NATIVE_REFUSALS):
        cftuv_native.canonical_json_bytes(value)


@needs_native
def test_random_floats_are_written_as_repr_writes_them(approved):
    rng = random.Random(20261020)
    values = [struct.unpack("<d", rng.getrandbits(64).to_bytes(8, "little"))[0] for _ in range(20000)]
    values = [value for value in values if value == value and abs(value) != float("inf")]
    values += [float(rng.randint(-(2**53), 2**53)) * 2.0 ** rng.randint(-60, 10) for _ in range(20000)]
    assert _same(tuple(values)) == "answered"


@needs_native
def test_the_dispatcher_runs_the_port_under_the_w1_switch(approved, monkeypatch):
    from cftuv_envelope import backend
    from cftuv_envelope.codec import canonical_json_bytes

    calls = []
    real = cftuv_native.canonical_json_bytes
    monkeypatch.setattr(cftuv_native, "canonical_json_bytes", lambda value: calls.append(value) or real(value))
    with backend.use_backend("PYTHON", materialize_backend="NATIVE") as ledger:
        assert backend.canonical_bytes_compute(canonical_json_bytes, ("a", 1)) == b'["a",1]'
        assert backend.canonical_bytes_compute(canonical_json_bytes, ({"k": "v"},)) == b'[{"k":"v"}]'
        with pytest.raises(Exception):
            backend.canonical_bytes_compute(canonical_json_bytes, [1])
    assert len(calls) == 3 and ledger.record().materialize_native_calls >= 2
    assert any(item[0] == "NATIVE_PORT_UNSUPPORTED" for item in ledger.record().materialize_fallbacks)
    with backend.use_backend("PYTHON", materialize_backend="PYTHON"):
        assert backend.canonical_bytes_compute(canonical_json_bytes, ("a",)) == b'["a"]'
    assert len(calls) == 3


def test_the_domain_and_the_semantic_digest_route_their_bytes_through_the_dispatcher():
    """MATERIALIZE_DIGEST_NATIVE_V1: the content digest of the domain and the semantic digest of the batch take their bytes from `backend.canonical_bytes_compute`."""

    domain = (ROOT / "kernel/src/cftuv_envelope/materialize/domain.py").read_text(encoding="utf-8")
    canonical = (ROOT / "kernel/src/cftuv_envelope/canonical.py").read_text(encoding="utf-8")
    assert "sha256(backend.canonical_bytes_compute(canonical_json_bytes, batch))" in domain
    assert "backend.canonical_bytes_compute(canonical_json_bytes, projection)" in canonical
