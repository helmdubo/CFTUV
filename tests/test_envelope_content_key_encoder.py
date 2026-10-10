"""Кодировщик ключа содержимого пишет ТЕ ЖЕ байты, что прежний рекурсивный (`tests/content_key_legacy.py`).

Ключ - адрес результата домена в хранилище по содержимому. Кодировщик выбирает запись по точному типу таблицей, пишет однородные кортежи целых и
вещественных одним `join` и ведёт датакласс по плану класса; байты при этом обязаны остаться теми же, что давал прямой обход с цепочкой проверок
типа: ключ, изменившийся без изменения входа, молча обнулил бы хранилище, а ключ, свёвший два входа к одному, отдал бы чужой результат. Сверка:

1. ТОЧНЫЕ ТИПЫ. Каждый тип по отдельности, в том числе края: `bool` не `int`, подкласс `int`/`float`/`str`/`tuple` не кодируется (отказ с тем же
   текстом), `nan`, бесконечности, `-0.0`, числа вне 64 бит, пустые контейнеры, смешанные кортежи, множества и словари со смешанными ключами.
2. РЕВИЗИЯ И НОМЕРА ПАТЧЕЙ: строки с ревизией внутри метятся, пустая ревизия ведёт себя как прежде, поля `patch_id`/`neighbor_patch_id` пишутся рангом,
   поле `nodes` со словарём - по рангам ключей, неверный номер - тот же отказ.
3. СЛУЧАЙНЫЕ ДЕРЕВЬЯ из всех кодируемых и некодируемых значений: равны либо строка, либо тип и текст исключения.
4. НАСТОЯЩИЕ ВХОДЫ: ключ каждого домена всех фикстур (политики, плотности, допуски, полосы, бэкенды) равен ключу прежнего кодировщика.
5. План записи принадлежит ТИПУ: два класса с одним именем не делят план.
"""

from __future__ import annotations

import dataclasses
import itertools
import math
import random
import sys
from enum import Enum, IntEnum
from fractions import Fraction
from pathlib import Path
from typing import NamedTuple

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv.envelope_content_key import ContentKeyUnsupported, _Encoder, domain_content_key  # noqa: E402
from cftuv.envelope_export_input import build_host_export_input  # noqa: E402
from cftuv.envelope_request_export import EnvelopeHostAdapterError  # noqa: E402
from cftuv.envelope_metric_export import band_key_of  # noqa: E402
from cftuv.envelope_topology_export import build_envelope_topology_export, stage_domain_inputs  # noqa: E402
from cftuv.surface_ir import SourceRevision  # noqa: E402
from content_key_legacy import LegacyEncoder, legacy_domain_content_key  # noqa: E402
from envelope_fixture_bundles import (  # noqa: E402
    bundle_from_exported_snapshot,
    host_exported_snapshot_paths,
    planar_quad_bundle,
    quad_row_bundle,
    square_hole_bundle,
    u_route_bundle,
)
from surface_adjacency_field_corpus import load_snapshot  # noqa: E402

REVISION = "host-source:abc123:building.002"
PATCHES = {5: 0, 7: 1, 9: 2}


class Color(Enum):
    RED = "red"
    BLUE = 2


class Level(IntEnum):
    LOW = 1
    HIGH = 2


class Tag(str, Enum):
    ONE = "one"


@dataclasses.dataclass(frozen=True, slots=True)
class Leaf:
    a: int
    b: float


@dataclasses.dataclass(frozen=True, slots=True)
class Holder:
    patch_id: int
    nodes: object
    items: tuple
    label: str = ""


@dataclasses.dataclass(frozen=True, slots=True)
class Neighbour:
    neighbor_patch_id: int
    face: tuple


@dataclasses.dataclass(frozen=True)
class Empty:
    pass


class FloatLike(float):
    pass


class IntLike(int):
    pass


class StrLike(str):
    pass


class TupleLike(tuple):
    pass


class Pair(NamedTuple):
    x: int
    y: int


def encoders(revision=REVISION, patches=None):
    patches = PATCHES if patches is None else patches
    return LegacyEncoder(revision, dict(patches)), _Encoder(revision, dict(patches))


def outcome(encoder, value):
    try:
        return ("ok", encoder.encode(value))
    except Exception as exc:  # noqa: BLE001 - сравнивается и исключение
        return ("raised", type(exc).__name__, str(exc))


def assert_same(value, revision=REVISION, patches=None):
    legacy, fast = encoders(revision, patches)
    assert outcome(fast, value) == outcome(legacy, value), repr(value)[:200]
    return outcome(fast, value)


# --------------------------------------------------------------------------
# 1. Точные типы и края
# --------------------------------------------------------------------------

FLOATS = (0.0, -0.0, 1.0, -1.5, 0.1, 5e-324, -5e-324, 1.7976931348623157e308, math.inf, -math.inf, math.nan, 2.0**-1022, 1e-7, 123456789.123456789)
INTS = (0, 1, -1, 7, 10**18, -(10**18), 2**63, 2**64 + 5, -(2**100), 10**40)

SCALARS = (
    *INTS,
    *FLOATS,
    True,
    False,
    None,
    "",
    "x",
    "строка",
    "s3:abc;",
    "i1;",
    f"id:{REVISION}:tail",
    REVISION,
    Fraction(1, 3),
    Fraction(-7, 2),
    Fraction(0),
    Color.RED,
    Color.BLUE,
    Level.LOW,
    Tag.ONE,
)

SEQUENCES = (
    (),
    [],
    (1,),
    (1, 2, 3),
    [1, 2, 3],
    (-1, 2**70, 0),
    (1.0,),
    (0.5, -0.0, math.nan, math.inf),
    [0.5, 1.5],
    (1, 1.0),
    (1.0, 1),
    (True, 1),
    (1, True),
    (True, False),
    (None, None),
    ("a", "b"),
    (1, "a", 2.5, None, True),
    ((1, 2), (3.0, 4.0)),
    (((),),),
    ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)),
    tuple(range(100)),
    tuple(float(item) / 7 for item in range(100)),
    ((), (1,), (1.0,), ("x",)),
)


@pytest.mark.parametrize("value", SCALARS, ids=lambda v: f"{type(v).__name__}:{str(v)[:20]}")
def test_every_scalar_is_written_with_the_same_bytes(value):
    assert_same(value)


@pytest.mark.parametrize("value", SEQUENCES, ids=lambda v: f"{type(v).__name__}:{str(v)[:30]}")
def test_every_sequence_is_written_with_the_same_bytes(value):
    assert assert_same(value)[0] == "ok"


def test_unordered_and_keyed_containers_are_written_with_the_same_bytes():
    for value in (
        frozenset(),
        set(),
        frozenset({1, 2, 3}),
        {3, 1, 2},
        frozenset({"b", "a", REVISION}),
        frozenset({(1, 2), (2, 1), ()}),
        frozenset({1.5, -0.0, 0.0}),
        {},
        {1: "a", 2: "b"},
        {"b": 1, "a": 2},
        {(1, 2): [1.0], (): None},
        {1: 1, "1": 2, 1.0: 3, None: 4, True: 5},
        {Color.RED: Leaf(1, 2.0), Color.BLUE: (1, 2)},
        {"k": {"nested": {"deeper": (1, 2.0)}}},
    ):
        assert_same(value)


def test_the_type_decides_and_not_the_value_a_bool_is_not_an_int_and_a_subclass_is_not_its_base():
    assert assert_same(True)[1] == "T;" and assert_same(1)[1] == "i1;"
    for value in (FloatLike(1.5), IntLike(3), StrLike("x"), TupleLike((1, 2)), Pair(1, 2), (1, IntLike(3)), (FloatLike(1.0), FloatLike(2.0)), [StrLike("a")]):
        result = assert_same(value)
        assert result[0] == "raised" and result[1] == "ContentKeyUnsupported", repr(value)
    for value in (object(), b"bytes", 1 + 2j, int, Leaf, (1, object()), {1: object()}, frozenset({object()}), {object(): 1}):
        assert assert_same(value)[0] == "raised"


def test_a_big_homogeneous_tuple_of_ints_and_floats_keeps_every_element():
    ints = tuple(range(-5000, 5000))
    floats = tuple(math.sin(item) * 10**item % 7 for item in range(-300, 300))
    assert assert_same(ints)[0] == "ok" and assert_same(floats)[0] == "ok"


# --------------------------------------------------------------------------
# 2. Ревизия и номера патчей
# --------------------------------------------------------------------------


def test_a_revision_inside_a_string_is_marked_and_an_empty_revision_behaves_as_before():
    for text in ("", "plain", REVISION, f"a{REVISION}b", f"{REVISION}{REVISION}", f"host-source:{REVISION}"):
        assert_same(text)
        assert_same((text, text))
        assert_same(Leaf(1, 2.0), revision=REVISION)
    for revision in ("", "a", "x:y"):
        assert_same("banana", revision=revision)
        assert_same(("banana", 1, ("a",)), revision=revision)


def test_the_source_revision_record_is_a_mark_and_never_its_fields():
    first, second = SourceRevision("name", "digest-a"), SourceRevision("other", "digest-b")
    assert assert_same(first)[1] == assert_same(second)[1] == "\x00SRC;"
    assert_same(Holder(5, {5: first, 7: second}, (first, second)))


def test_patch_numbers_are_ranks_negative_numbers_stay_and_a_wrong_number_is_refused_the_same():
    assert assert_same(Holder(5, {}, ()))[1].startswith("DHolder[patch_id=p0;")
    for patch in (5, 7, 9, -1, -5, 0, 6, 10**6, True, None, 1.5, "5", (5,)):
        assert_same(Holder(patch, {}, ()))
        assert_same(Neighbour(patch, ()))
    for nodes in ({}, {5: Leaf(1, 1.0)}, {9: (1, 2), 5: (3, 4), 7: ()}, {5: 1, 6: 2}, {True: 1}, {"5": 1}, [1, 2], (5,), None, 7):
        assert_same(Holder(5, nodes, ()))
    assert_same(Holder(5, {7: Holder(7, {}, (Neighbour(9, ()),))}, (Neighbour(-1, (1,)),)))
    assert_same(Holder(5, {5: 1}, ()), patches={})
    assert_same(Holder(5, {5: 1}, ()), patches={5: 0, 6: 7})


def test_a_patch_field_is_found_by_name_in_any_dataclass_and_a_plain_field_is_not():
    @dataclasses.dataclass(frozen=True)
    class Carrier:
        patch_id: int
        patch: int
        neighbor_patch_id: int
        patch_ids: tuple

    value = Carrier(5, 99, 7, (5, 7))
    result = assert_same(value)
    assert result[0] == "ok"
    # `patch` и `patch_ids` - обычные поля: их целые пишутся как есть, а не рангом; только `patch_id` и `neighbor_patch_id` - номера патчей
    assert "patch=i99;" in result[1] and "patch_id=p0;" in result[1] and "neighbor_patch_id=p1;" in result[1] and "patch_ids=(i5;i7;)" in result[1]
    assert assert_same(Carrier(6, 99, 7, (5, 7)))[0] == "raised"


# --------------------------------------------------------------------------
# 3. Датаклассы и планы
# --------------------------------------------------------------------------


def test_dataclasses_without_fields_with_slots_and_nested_are_written_the_same():
    assert assert_same(Empty())[1] == "DEmpty[]"
    assert_same(Leaf(1, 2.0))
    assert_same((Leaf(1, 2.0), Leaf(2, math.nan), Leaf(-3, -0.0)))
    assert_same(Holder(5, {5: Leaf(1, 2.0)}, (Leaf(1, 2.0), Holder(7, {}, ())), "label"))
    assert_same(Leaf("not an int", None))


def test_the_plan_of_a_record_belongs_to_the_type_and_not_to_its_name():
    def make(extra):
        @dataclasses.dataclass(frozen=True)
        class Same:
            first: int

        if extra:

            @dataclasses.dataclass(frozen=True)
            class Same:  # noqa: F811
                first: int
                second: int

        return Same

    one, two = make(False), make(True)
    assert one.__qualname__ == two.__qualname__ and one is not two
    assert assert_same(one(1))[1] != assert_same(two(1, 2))[1]
    assert assert_same(one(1))[1].count("=") == 1 and assert_same(two(1, 2))[1].count("=") == 2


def test_an_instance_of_a_dataclass_subclass_is_written_with_its_own_fields():
    @dataclasses.dataclass(frozen=True)
    class Base:
        a: int

    @dataclasses.dataclass(frozen=True)
    class Derived(Base):
        b: float

    assert_same(Base(1))
    assert_same(Derived(1, 2.0))
    assert assert_same(Derived(1, 2.0))[1] != assert_same(Base(1))[1]


# --------------------------------------------------------------------------
# 4. Случайные деревья
# --------------------------------------------------------------------------


def random_value(rng, depth):
    kinds = ["int", "float", "str", "bool", "none", "frac", "enum"]
    if depth > 0:
        kinds += ["tuple", "tuple", "ints", "floats", "list", "set", "dict", "leaf", "holder", "neighbour", "revision", "bad"]
    kind = rng.choice(kinds)
    if kind == "int":
        return rng.choice([0, 1, -1, rng.randint(-10**6, 10**6), rng.randint(-(2**70), 2**70)])
    if kind == "float":
        return rng.choice([0.0, -0.0, math.nan, math.inf, -math.inf, rng.random(), rng.uniform(-1e9, 1e9), 5e-324])
    if kind == "str":
        return rng.choice(["", "a", REVISION, f"x{REVISION}y", "s1:a;", "(", "\x00", "строка"] + [str(rng.random())])
    if kind == "bool":
        return rng.random() < 0.5
    if kind == "none":
        return None
    if kind == "frac":
        return Fraction(rng.randint(-9, 9), rng.randint(1, 9))
    if kind == "enum":
        return rng.choice([Color.RED, Color.BLUE, Level.HIGH, Tag.ONE])
    if kind == "tuple":
        return tuple(random_value(rng, depth - 1) for _ in range(rng.randint(0, 4)))
    if kind == "ints":
        return tuple(rng.randint(-100, 100) for _ in range(rng.randint(0, 6)))
    if kind == "floats":
        return tuple(rng.choice([rng.random(), -0.0, math.nan]) for _ in range(rng.randint(0, 6)))
    if kind == "list":
        return [random_value(rng, depth - 1) for _ in range(rng.randint(0, 4))]
    if kind == "set":
        return frozenset(rng.choice([rng.randint(0, 9), str(rng.randint(0, 9)), (rng.randint(0, 3),)]) for _ in range(rng.randint(0, 5)))
    if kind == "dict":
        return {rng.choice([rng.randint(0, 9), str(rng.randint(0, 9)), (rng.randint(0, 3),)]): random_value(rng, depth - 1) for _ in range(rng.randint(0, 4))}
    if kind == "leaf":
        return Leaf(random_value(rng, 0), random_value(rng, 0))
    if kind == "holder":
        nodes = rng.choice([{}, {rng.choice([5, 7, 9, 6, -1]): random_value(rng, depth - 1)}, random_value(rng, 0)])
        return Holder(rng.choice([5, 7, 9, -1, 6]), nodes, random_value(rng, depth - 1))
    if kind == "neighbour":
        return Neighbour(rng.choice([5, 7, 9, -1, 6]), random_value(rng, depth - 1))
    if kind == "revision":
        return SourceRevision(str(rng.random()), str(rng.random()))
    return rng.choice([object(), FloatLike(1.0), IntLike(1), b"x", TupleLike((1,))])


def test_random_trees_are_written_with_the_same_bytes_or_refused_the_same():
    rng = random.Random(20261011)
    answered = refused = 0
    for _ in range(4000):
        value = random_value(rng, 4)
        result = assert_same(value)
        answered += result[0] == "ok"
        refused += result[0] == "raised"
    assert answered > 1000 and refused > 100, (answered, refused)


# --------------------------------------------------------------------------
# 5. Настоящие входы
# --------------------------------------------------------------------------


def fixture_bundles():
    found = [
        quad_row_bundle(5),
        quad_row_bundle(3, lifted_corner=0.3),
        planar_quad_bundle(),
        planar_quad_bundle((2.0, 0.4, 0.0)),
        u_route_bundle(),
        u_route_bundle(split_route=True),
        square_hole_bundle(),
    ]
    for path in host_exported_snapshot_paths():
        found.append(bundle_from_exported_snapshot(load_snapshot(path.parent))[0])
    return found


def domains(bundle):
    topology = build_envelope_topology_export(bundle)
    edges = frozenset(int(edge) for record in topology.host_chains for edge in record.canonical_edge_ids)
    _scene, _revision, patch_ids, request_id, by_domain = stage_domain_inputs(bundle, edges, topology_export=topology)
    return topology, patch_ids, request_id, by_domain


def test_the_key_of_every_domain_of_every_fixture_equals_the_legacy_key_under_every_policy():
    compared = 0
    for bundle in fixture_bundles():
        try:
            topology, patch_ids, request_id, by_domain = domains(bundle)
        except EnvelopeHostAdapterError:
            continue  # фикстура выпущенного снапшота: сторона шва без пары в срезе, выделение всех рёбер не разрешается
        for density, budget, backend in itertools.product(("0", "1", "2", "3", "4", None), (None, Fraction(1, 4), Fraction(1, 5)), ("PYTHON", "NATIVE")):
            narrowed = topology.with_developable_stretch_budget(budget).with_chart_band(None, frozenset())
            for patch in patch_ids:
                export = build_host_export_input(narrowed, patch, alpha=0.25, request_id=request_id, density=density)
                selected = frozenset(by_domain[narrowed.patch_domain_id_by_patch[patch]])
                band = band_key_of(narrowed, patch)
                try:
                    expected = legacy_domain_content_key(export, selected, band, backend)
                except Exception as exc:  # noqa: BLE001
                    with pytest.raises(type(exc)) as raised:
                        domain_content_key(export, selected, band, backend)
                    assert str(raised.value) == str(exc)
                    continue
                assert domain_content_key(export, selected, band, backend) == expected
                compared += 1
    assert compared > 500


def test_a_poisoned_export_is_refused_with_the_same_message():
    bundle = quad_row_bundle(5)
    topology, patch_ids, request_id, by_domain = domains(bundle)
    patch = patch_ids[0]
    export = build_host_export_input(topology, patch, alpha=0.25, request_id=request_id, density="1")
    selected = frozenset(by_domain[topology.patch_domain_id_by_patch[patch]])
    for poisoned in (
        dataclasses.replace(export, host_chains=(*export.host_chains, object())),
        dataclasses.replace(export, developable_stretch_budget=object()),
        dataclasses.replace(export, density=object()),
    ):
        with pytest.raises(ContentKeyUnsupported) as legacy:
            legacy_domain_content_key(poisoned, selected)
        with pytest.raises(ContentKeyUnsupported) as new:
            domain_content_key(poisoned, selected)
        assert str(new.value) == str(legacy.value)


# --------------------------------------------------------------------------
# 6. Память общих записей одного прогона
# --------------------------------------------------------------------------


class CountingMemo(dict):
    def __init__(self):
        super().__init__()
        self.hits = 0
        self.misses = 0

    def get(self, key, default=None):
        found = super().get(key, default)
        if found is None:
            self.misses += 1
        else:
            self.hits += 1
        return found


def keyed_domains(bundle):
    topology, patch_ids, request_id, by_domain = domains(bundle)
    narrowed = topology.with_chart_band(None, frozenset())
    for patch in patch_ids:
        export = build_host_export_input(narrowed, patch, alpha=0.25, request_id=request_id, density="1")
        yield export, frozenset(by_domain[narrowed.patch_domain_id_by_patch[patch]]), band_key_of(narrowed, patch)


def test_a_memo_shared_by_the_domains_of_a_run_changes_no_key_and_is_hit():
    hits = compared = 0
    for bundle in fixture_bundles():
        try:
            batch = list(keyed_domains(bundle))
        except EnvelopeHostAdapterError:
            continue
        memo = CountingMemo()
        for export, selected, band in batch:
            assert domain_content_key(export, selected, band, memo=memo) == legacy_domain_content_key(export, selected, band)
            compared += 1
        # другой порядок доменов и та же память: ключи те же
        for export, selected, band in reversed(batch):
            assert domain_content_key(export, selected, band, memo=memo) == domain_content_key(export, selected, band)
        hits += memo.hits
    assert compared >= 10 and hits > 100, (compared, hits)


def test_the_shared_records_of_a_surface_are_written_once_for_all_the_domains_that_hold_them():
    bundle = quad_row_bundle(5)
    batch = list(keyed_domains(bundle))
    assert len(batch) == 5
    memo = CountingMemo()
    for export, selected, band in batch:
        domain_content_key(export, selected, band, memo=memo)
    held = sum(
        len(export.bundle.patch_surface.vertices) + len(export.bundle.patch_surface.edges) + len(export.bundle.patch_surface.neighbour_faces)
        for export, _selected, _band in batch
    )
    assert memo.hits > 0 and len(memo) < held, "neighbouring domains hold the same vertices and edges, which are written once"
    assert memo.hits + len(memo) >= held, "every shared record went through the memo"


def test_a_ranked_record_is_remembered_under_the_rank_of_its_patch_in_the_domain_that_writes_it():
    from cftuv.envelope_topology_export import NeighbourFaceV1

    face = NeighbourFaceV1(7, 5, (1, 2, 3), ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)))
    memo = {}
    first = _Encoder(REVISION, {5: 0, 7: 1}, memo)
    second = _Encoder(REVISION, {9: 0, 5: 1}, memo)
    third = _Encoder(REVISION, {9: 0, 11: 1}, memo)
    legacy_first, legacy_second = LegacyEncoder(REVISION, {5: 0, 7: 1}), LegacyEncoder(REVISION, {9: 0, 5: 1})
    assert first.encode(face) == legacy_first.encode(face) and second.encode(face) == legacy_second.encode(face)
    assert first.encode(face) != second.encode(face), "the same record reads differently in a domain where its patch has another rank"
    assert len(memo) == 2
    with pytest.raises(ContentKeyUnsupported):
        third.encode(face)  # патч грани не домен и не сосед: отказ, и в память он не ложится
    assert len(memo) == 2
    odd = NeighbourFaceV1(7, -1, (1, 2, 3), ((0.0, 0.0, 0.0),))
    assert third.encode(odd) == LegacyEncoder(REVISION, {9: 0, 11: 1}).encode(odd) and first.encode(odd) == third.encode(odd)


def test_a_record_with_a_patch_number_that_is_not_an_int_is_refused_as_before_and_never_remembered():
    from cftuv.envelope_topology_export import NeighbourFaceV1

    memo = {}
    for number in (None, "5", 5.0, True, (5,), [5]):
        face = NeighbourFaceV1(7, number, (1,), ((0.0, 0.0, 0.0),))
        legacy = outcome(LegacyEncoder(REVISION, dict(PATCHES)), face)
        assert outcome(_Encoder(REVISION, dict(PATCHES), memo), face) == legacy, number
    assert not memo


def test_a_memo_cannot_serve_a_new_object_that_took_the_address_of_a_freed_one():
    import gc

    from cftuv.surface_ir import SourceVertex

    memo = {}
    encoder, legacy = _Encoder(REVISION, dict(PATCHES), memo), LegacyEncoder(REVISION, dict(PATCHES))
    for number in range(3000):
        vertex = SourceVertex(number, (float(number), -float(number), 0.5))
        assert encoder.encode(vertex) == legacy.encode(vertex)
        del vertex
        if number % 500 == 0:
            gc.collect()
    assert len(memo) == 3000, "every record is held by the memo, so its address cannot be reused"


def test_one_memo_never_mixes_two_revisions():
    from cftuv.surface_ir import SourceEdge

    memo = {}
    edge = SourceEdge(3, (1, 2), (4, 5))
    for revision in ("rev-a", "rev-b"):
        assert _Encoder(revision, {}, memo).encode(edge) == LegacyEncoder(revision, {}).encode(edge)
    assert len(memo) == 2


def test_the_records_the_memo_serves_hold_nothing_but_numbers_and_a_single_patch_number_at_most():
    """Перечень `_shared_kinds` держит текст записи функцией ТОЛЬКО записи, ревизии и ранга патча: полей-датаклассов и строк у них нет."""

    from cftuv.envelope_content_key import _PATCH_FIELDS, _PATCH_MAP_FIELD, _shared_kinds
    from cftuv.envelope_topology_export import NeighbourFaceV1

    surface = quad_row_bundle(2).patch_surface
    samples = {
        "SourceVertex": surface.vertices[0],
        "SourceEdge": surface.edges[0],
        "NeighbourFaceV1": NeighbourFaceV1(1, 2, (1, 2, 3), ((0.0, 0.0, 0.0),)),
    }

    def only_numbers(value):
        if type(value) in (int, float) or value is None:
            return True
        return type(value) is tuple and all(only_numbers(item) for item in value)

    kinds = _shared_kinds()
    assert {kind.__name__ for kind in kinds} == set(samples)
    for kind, ranked in kinds.items():
        names = {item.name for item in dataclasses.fields(kind)}
        assert names & (_PATCH_FIELDS | {_PATCH_MAP_FIELD}) == ({"patch_id"} if ranked else set()), kind
        sample = samples[kind.__name__]
        assert type(sample) is kind and all(only_numbers(getattr(sample, name)) for name in names), kind


# --------------------------------------------------------------------------
# 7. Проводка в кнопке: одна память на прогон, ключи равны прежним
# --------------------------------------------------------------------------


def test_a_run_passes_one_memo_to_the_keys_of_all_its_domains_and_a_new_memo_to_the_next_run(monkeypatch):
    from cftuv import envelope_content_key as key_module
    from cftuv.envelope_debug_session import EnvelopeDebugSessionController
    from cftuv.envelope_domain_pool import shutdown_domain_pool
    from cftuv.envelope_kernel_backend import DEFAULT_KERNEL_BACKEND
    from cftuv.envelope_production_export import run_production

    seen = []
    real = key_module.domain_content_key

    def spy(export, selected, band_key=None, backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=None, embedding_backend=None, memo=None):
        key = real(export, selected, band_key, backend, skeleton_backend, embedding_backend, memo)
        seen.append((export, selected, band_key, backend, skeleton_backend, embedding_backend, memo, key))
        return key

    monkeypatch.setattr(key_module, "domain_content_key", spy)
    bundle = quad_row_bundle(5)
    for _ in range(2):
        run_production(
            EnvelopeDebugSessionController(),
            bundle,
            frozenset(range(5)),
            0.25,
            source_object_key="object",
            source_data_key="mesh",
            density=None,
            workers=0,
        )
    shutdown_domain_pool()
    assert len(seen) == 10
    first, second = seen[:5], seen[5:]
    assert all(item[6] is first[0][6] for item in first) and all(item[6] is second[0][6] for item in second)
    assert first[0][6] is not second[0][6] and isinstance(first[0][6], dict) and first[0][6]
    for export, selected, band_key, backend, skeleton_backend, embedding_backend, _memo, key in seen:
        assert key == legacy_domain_content_key(export, selected, band_key, backend, skeleton_backend, embedding_backend)
