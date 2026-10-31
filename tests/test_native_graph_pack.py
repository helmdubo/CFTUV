"""COMMIT_BATCH_PACK_V1: батч отложенного результата пакует Rust (`cftuv_native.graph_pack`), и родитель получает ТОТ ЖЕ граф, что дал бы пикл.

Эталон - сам pickle: `graph_unpack(graph_pack(x))` против `pickle.loads(pickle.dumps(x))`. Равенство значений мало (оно слепо к порядку `frozenset`,
к разделению объектов, к знаку нуля), поэтому сравнивается ПИКЛ двух графов: его байты пишут каждый класс, каждое значение (числа побитово), порядок
обхода каждого множества и каждую ссылку на уже записанный объект. Эталон формата без расширения (`envelope_batch_pack.reference_unpack`) держится тем же.
"""

from __future__ import annotations

import dataclasses
import decimal
import enum
import functools
import pickle
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
KERNEL_SRC = ROOT / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

try:
    import cftuv_native
except ImportError:  # pragma: no cover - the extension is not built here
    cftuv_native = None

from cftuv import envelope_batch_pack as batch_pack  # noqa: E402

pytestmark = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "graph_pack"), reason="расширение cftuv_native не собрано (или собрано без graph_pack)"
)


class Colour(str, enum.Enum):
    RED = "red"
    BLUE = "blue"


class Level(enum.IntEnum):
    LOW = 1
    HIGH = 2


@dataclasses.dataclass(frozen=True, slots=True)
class Leaf:
    name: str
    amount: decimal.Decimal
    colour: Colour


@dataclasses.dataclass(frozen=True, slots=True)
class Node:
    leaves: frozenset
    order: tuple
    weight: float
    level: Level
    extra: object = None


@dataclasses.dataclass(frozen=True)
class NoSlots:
    value: int


@dataclasses.dataclass(slots=True)
class NotFrozen:
    value: int


@dataclasses.dataclass(frozen=True, slots=True)
class OwnReduce:
    value: int

    def __reduce__(self):
        return (OwnReduce, (self.value,))


def _pickle_round(value):
    return pickle.loads(pickle.dumps(value, protocol=5))


def _same_graph(found, wanted) -> None:
    """Пикл двух графов побитово один (классы, значения, порядок множеств, разделение)."""

    assert pickle.dumps(found, protocol=5) == pickle.dumps(wanted, protocol=5)


def _round_trips(value):
    packed = cftuv_native.graph_pack(value)
    assert isinstance(packed, bytes) and packed.startswith(batch_pack.MAGIC)
    wanted = _pickle_round(value)
    native = cftuv_native.graph_unpack(packed)
    reference = batch_pack.reference_unpack(packed)
    _same_graph(native, wanted)
    _same_graph(reference, wanted)
    return native, reference, packed


def _synthetic():
    shared = Leaf("shared", decimal.Decimal("-0.000E+5"), Colour.RED)
    leaves = frozenset({shared, Leaf("a", decimal.Decimal("1.5"), Colour.BLUE), Leaf("б-юникод", decimal.Decimal("123456789012345678901234567.89"), Colour.RED)})
    text = "повтор"
    node = Node(leaves, (shared, shared, text, text, (), (1,), b"\x00raw"), -0.0, Level.HIGH)
    floats = (0.0, -0.0, 5e-324, 1.7976931348623157e308, float("inf"), float("-inf"), float("nan"), 0.1 + 0.2)
    ints = (0, -1, 1, 2**63 - 1, -(2**63), 255, 256)
    return (node, Node(frozenset(), (node,), 1.0, Level.LOW, extra=(None, True, False)), floats, ints, frozenset({text, 7, 1.25}), shared)


def test_every_carried_kind_comes_back_as_pickle_brings_it():
    native, reference, _packed = _round_trips(_synthetic())

    for found in (native, reference):
        node = found[0]
        assert node.order[0] is node.order[1] and node.order[2] is node.order[3]  # разделение, которое пикл сохраняет
        assert found[1].order[0] is node and found[5] in node.leaves
        assert next(item for item in node.leaves if item.name == "shared") is found[5]
        assert node.level is Level.HIGH and found[5].colour is Colour.RED  # член перечисления - тот же объект
        assert str(found[5].amount) == "-0E+2" and repr(found[2][1]) == "-0.0"
        assert type(node.order[4]) is tuple and node.order[4] == () and node.order[6] == b"\x00raw"


def test_the_order_of_every_frozenset_is_the_order_pickle_gives():
    items = frozenset(Leaf(f"leaf-{index}", decimal.Decimal(index), Colour.BLUE) for index in range(300))
    native, _reference, _packed = _round_trips((items, Node(items, (), 0.5, Level.LOW)))

    assert list(native[0]) == list(_pickle_round(items))
    assert native[1].leaves is native[0]


@pytest.mark.parametrize(
    "value, reason",
    [
        (2**64, "an int beyond 64 bits"),
        ([1, 2], "unsupported value: list"),
        ({"a": 1}, "unsupported value: dict"),
        ("\ud800", "a str with a lone surrogate"),
        (NoSlots(1), "is not a slots dataclass"),
        (NotFrozen(1), "does not pickle by _dataclass_getstate"),
        (OwnReduce(1), "has its own __new__ or __reduce_ex__ or __reduce__"),
        (functools.reduce(lambda inner, _: (inner,), range(250), ()), "nesting deeper than the packer's limit"),
    ],
)
def test_a_graph_the_packer_does_not_carry_is_declined_by_name(value, reason):
    with pytest.raises(cftuv_native.NativePortUnsupported, match="graph_pack: .*" + reason.replace("(", r"\(").replace(")", r"\)")):
        cftuv_native.graph_pack(value)


def _malformed(packed: bytes):
    yield b"CGP0" + packed[4:], "not a COMMIT_BATCH_PACK_V1 buffer"
    yield packed[:-1], "ends early"
    yield packed + b"\x00", "bytes after the value"
    yield packed[:5] + b"\x63", None  # the class table cut short or garbled: any named error


def test_a_buffer_the_packer_did_not_write_is_a_value_error():
    packed = cftuv_native.graph_pack(_synthetic())
    for spoiled, text in _malformed(packed):
        for unpack in (cftuv_native.graph_unpack, batch_pack.reference_unpack):
            with pytest.raises((ValueError, IndexError, UnicodeDecodeError, ModuleNotFoundError, AttributeError)) as caught:
                unpack(spoiled)
            if text is not None:
                assert text in str(caught.value)
    reference = bytes([*batch_pack.MAGIC, 0, 12, 5])  # a reference beyond the memo
    with pytest.raises(ValueError, match="beyond the memo"):
        cftuv_native.graph_unpack(reference)
    with pytest.raises(ValueError, match="unknown tag"):
        cftuv_native.graph_unpack(bytes([*batch_pack.MAGIC, 0, 99]))


def test_a_class_whose_fields_changed_since_the_packing_is_refused():
    packed = cftuv_native.graph_pack(Leaf("x", decimal.Decimal(1), Colour.RED))
    spoiled = packed.replace(b"\x06amount", b"\x06amoun2")
    for unpack in (cftuv_native.graph_unpack, batch_pack.reference_unpack):
        with pytest.raises(ValueError, match="fields of"):
            unpack(spoiled)


# --------------------------------------------------------------------------
# настоящие батчи продуктового пути
# --------------------------------------------------------------------------

ROW = 5
CASES = [(0, 0.0), (2, 0.0), (ROW - 1, 1.0), (2, 1.0)]


@functools.lru_cache(maxsize=None)
def _eager(patch: int, lifted: float):
    from content_fixtures import cold_domain, with_revision
    from envelope_fixture_bundles import quad_row_bundle

    from cftuv.envelope_production_export import produce_domain

    source = with_revision(quad_row_bundle(ROW, lifted_corner=lifted), "row")
    deferred, prepared, _revision, _request = cold_domain(source, patch, frozenset(range(ROW)), alpha=0.25)
    return produce_domain(patch, deferred.domain_id, prepared, "0.25")


@pytest.mark.parametrize("patch, lifted", CASES)
def test_a_domain_batch_comes_back_as_pickle_brings_it(patch, lifted):
    from cftuv_envelope.codec import canonical_json_bytes

    eager = _eager(patch, lifted)
    value = (eager.batch, eager.vertex_normals)
    native, reference, packed = _round_trips(value)

    for found in (native, reference):
        assert found == value and found[0].semantic_digest == eager.batch.semantic_digest
        assert canonical_json_bytes(found[0]) == canonical_json_bytes(eager.batch)
    assert len(zlib.compress(packed, 1)) < len(zlib.compress(pickle.dumps(value, protocol=5), 1)) * 2


@pytest.mark.parametrize("order", [batch_pack.BATCH_PACK_NATIVE, batch_pack.BATCH_PACK_PICKLE])
def test_a_deferred_result_thaws_to_the_eager_answer_under_either_order(order, monkeypatch):
    from cftuv.envelope_production_export import deferred_result

    monkeypatch.setenv(batch_pack.BATCH_PACK_ENVIRONMENT, order)
    eager = _eager(ROW - 1, 1.0)
    sent = pickle.loads(pickle.dumps(deferred_result(eager)))
    head = zlib.decompress(sent.heavy)[:4]

    assert head == (batch_pack.MAGIC if order == batch_pack.BATCH_PACK_NATIVE else b"\x80\x05" + head[2:])
    assert sent == eager and sent.batch == eager.batch and sent.vertex_normals == eager.vertex_normals
    assert sent.content_digest == eager.content_digest


def test_a_declined_graph_is_pickled_and_the_fallback_is_named(monkeypatch, capsys):
    monkeypatch.setenv(batch_pack.BATCH_PACK_ENVIRONMENT, batch_pack.BATCH_PACK_NATIVE)
    monkeypatch.setattr(batch_pack, "FALLBACKS", {})
    value = ([1, 2], "list")

    packed = batch_pack.pack(value)

    assert packed == pickle.dumps(value, protocol=5) and batch_pack.unpack(packed) == value
    ((outcome, operation), (count, detail)), = batch_pack.FALLBACKS.items()
    assert (outcome, operation, count) == ("NATIVE_PORT_UNSUPPORTED", "graph_pack", 1) and "list" in detail
    assert "BATCH PACK: graph_pack by pickle" in capsys.readouterr().out


def test_an_unknown_order_is_a_value_error(monkeypatch):
    monkeypatch.setenv(batch_pack.BATCH_PACK_ENVIRONMENT, "FAST")
    with pytest.raises(ValueError, match="CFTUV_BATCH_PACK"):
        batch_pack.pack(())
