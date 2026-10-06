"""Граница Python <-> Rust нативных целых операций: быстрые пути разбора и сборки объектов равны медленным, а ключ поиска в `store` — обычному ключу.

Накладные границы (`native/cftuv-python/src/pyobj.rs`, `coverage.rs`, шим `cftuv_native.cost`) сокращены, ответ и цена — те же:

* целые за 64 бита идут словами (`PyLong_AsUnsignedLongLongMask`, `>> 64`), в обратную сторону — шестнадцатеричным текстом по таблице пар цифр;
* слоты `Fraction`, `SqrtSumV1`, `FaceCoverageV1`, `LocalPoint3V1` читаются и пишутся по смещению, найденному ПРОБОЙ настоящего экземпляра (`Raw::probe`); не подтверждённая
  раскладка — отказ от быстрого пути (протокол атрибутов), а не догадка. Эти тесты держат: быстрый путь включён на 3.11 и 3.13, и без него (протокол атрибутов) ответ тот же;
* `Fraction` в наименьших членах проверяется двоичным gcd на слове; ручная несократимая дробь по-прежнему сокращается;
* ключ поиска в `store` (`cost.StoreKey`) считает `hash` один раз на разбиение (`Fraction.__hash__` — питон-код, на 3.11 80 мкс на 54 грани) и отвечает `==` как обычный ключ,
  а `store` никогда не получает этот класс;
* `_general_diff` не строит индекс, когда в таблице осталось меньше половины зеркала (ответ тот же, что у полного разбора);
* таблицы памяти канонизации узнаются неизменными по ТОЖДЕСТВУ записей (`view.rs`: сеанс держит сами объекты таблиц после прошлого вызова), любое иное различие идёт через полное
  сравнение, как раньше, и зеркало остаётся равным настоящим таблицам.

Модуль пропускается с названной причиной, пока расширение не собрано либо нативные операции не сверены с этим деревом ядра (`native_gate`).
"""

from __future__ import annotations

import math
import random
import sys
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка границы Python <-> Rust пропущена)",
        allow_module_level=True,
    )

from native_gate import skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "coverage")
skip_unless_available(cftuv_native, "clip")

import native_clip_geometry as geometry  # noqa: E402
import native_corpus as nc  # noqa: E402
from cftuv_native import cost as native_cost  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1  # noqa: E402


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    with exact.isolated_factorization_memory():
        yield
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(nc._ARTICLES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)


# --------------------------------------------------------------------------
# целые
# --------------------------------------------------------------------------


def _integers() -> list:
    values = [0, 1, 2, 255, 256, 65535, 65536]
    for bits in (31, 32, 33, 62, 63, 64, 65, 95, 96, 127, 128, 129, 191, 192, 193, 255, 256, 257, 511, 512, 1000):
        values += [(1 << bits) - 1, 1 << bits, (1 << bits) + 1, (1 << bits) - (1 << (bits // 2)), ((1 << bits) // 3)]
    values += [0x0100000000000000, 0x00FF00FF00FF00FF00FF, 0x0102030405060708090A0B0C0D0E0F10, int("1" + "0" * 40), 10**77 + 12345]
    rng = random.Random(20261006)
    for _ in range(3000):
        values.append(rng.getrandbits(rng.randint(1, 600)))
    return values + [-value for value in values]


def test_integers_cross_the_boundary_unchanged_in_both_directions():
    checked = 0
    for value in _integers():
        back = cftuv_native.int_round_trip(value)
        assert type(back) is int and back == value, value
        checked += 1
    assert checked > 6000
    with pytest.raises(TypeError):
        cftuv_native.int_round_trip(1.5)


# --------------------------------------------------------------------------
# слоты по смещению
# --------------------------------------------------------------------------


def test_raw_slot_access_is_engaged_on_the_supported_interpreters():
    layouts = cftuv_native.new_mirror().raw_layouts()
    assert layouts and all(layouts.values()), f"the probe did not confirm a raw slot layout (the attribute protocol is used instead, slower): {layouts}"
    assert set(layouts) == {"coverage Fraction", "coverage SqrtSumV1", "FaceCoverageV1", "clip Fraction", "clip SqrtSumV1", "LocalPoint3V1"}


def _unreduced(numerator: int, denominator: int) -> Fraction:
    """Дробь, собранная вручную НЕ в наименьших членах (так её не получить публичным API): питон 3.11 — `_normalize=False`, позже — `_from_coprime_ints`."""

    builder = getattr(Fraction, "_from_coprime_ints", None)
    if builder is not None:
        return builder(numerator, denominator)
    return Fraction(numerator, denominator, _normalize=False)


def _fractions() -> list:
    rng = random.Random(7)
    values = [Fraction(0), Fraction(1), Fraction(-1), Fraction(1, 2), Fraction(-3, 7), Fraction(5), Fraction(2**63 - 1, 2**63), Fraction(-(2**63), 2**64 - 1), Fraction(2**64 - 1, 3), Fraction(10**40, 7**25)]
    for _ in range(400):
        bits = rng.choice((8, 20, 40, 62, 63, 64, 65, 100, 200))
        values.append(Fraction(rng.getrandbits(bits) * rng.choice((1, -1)), rng.getrandbits(bits) + 1))
    return values


@pytest.mark.parametrize("raw", [True, False], ids=["raw slots", "attribute protocol"])
def test_fractions_and_sums_round_trip_through_the_boundary(raw):
    mirror = cftuv_native.new_mirror()
    mirror.raw_layouts()
    if not raw:
        mirror.disable_raw()
    for value in _fractions():
        back = mirror.round_trip(value)
        assert type(back) is Fraction and back == value and back.numerator == value.numerator and back.denominator == value.denominator, value
    rng = random.Random(11)
    radicands = [1, 2, 3, 5, 6, 7, 10, 11, 2**40 + 15, (1 << 70) + 25, 3**50]
    for _ in range(300):
        chosen = sorted(rng.sample(radicands, rng.randint(0, 4)))
        terms = tuple((radicand, rng.choice(_fractions()) if rng.random() < 0.7 else rng.choice((1, -2, 5, 2**70, -(2**65)))) for radicand in chosen)
        terms = tuple((radicand, coefficient) for radicand, coefficient in terms if coefficient != 0)
        value = SqrtSumV1(terms)
        back = mirror.round_trip(value, sum=True)
        assert type(back) is SqrtSumV1 and back == value and type(back.terms) is tuple and len(back.terms) == len(value.terms)
        for (radicand, coefficient), (back_radicand, back_coefficient) in zip(value.terms, back.terms):
            assert back_radicand == radicand and type(back_radicand) is int
            assert type(back_coefficient) is type(coefficient) and back_coefficient == coefficient, (value, back)


def test_a_fraction_in_lowest_terms_is_kept_and_a_hand_made_one_is_reduced():
    """`gcd(numerator, denominator) == 1` decided on machine words agrees with `math.gcd` on every pair, including 63- and 64-bit edges and a zero numerator."""

    mirror = cftuv_native.new_mirror()
    mirror.raw_layouts()
    rng = random.Random(3)
    pairs = [(n, d) for n in range(-24, 25) for d in range(1, 41)]
    edges = [0, 1, 2, 2**31, 2**32 + 1, 2**62, 2**63 - 1, 2**63, 2**63 + 1, 2**64 - 1, 2**64, 2**64 + 1]
    pairs += [(sign * n, d) for n in edges for d in edges[1:] for sign in (1, -1)]
    pairs += [(rng.getrandbits(rng.randint(1, 70)) * rng.choice((1, -1)), rng.getrandbits(rng.randint(1, 70)) + 1) for _ in range(2000)]
    pairs += [(a * g, b * g) for a, b, g in ((rng.getrandbits(30), rng.getrandbits(30) + 1, rng.getrandbits(12) + 2) for _ in range(500))]
    reduced = 0
    for numerator, denominator in pairs:
        made = _unreduced(numerator, denominator)
        assert (made.numerator, made.denominator) == (numerator, denominator)
        back = mirror.round_trip(made)
        want = Fraction(numerator, denominator)
        assert type(back) is Fraction and (back.numerator, back.denominator) == (want.numerator, want.denominator), (numerator, denominator)
        reduced += math.gcd(numerator, denominator) != 1
    assert reduced > 500


def _compare_coverage(mirror, record):
    before = record.before()
    oracle = nc.execute(nc.prepare_call(nc.OP_COVERAGE, record.call_blob, before))
    call = nc.prepare_call(nc.OP_COVERAGE, record.call_blob, before)
    try:
        result, error = mirror.coverage_at(call.args[0], call.args[1], call.budget, call.store), None
    except Exception as exc:  # noqa: BLE001 - исключение операции — часть её исхода
        result, error = None, (type(exc).__qualname__, str(exc))
    native = nc.Outcome(result, error, nc.capture_state(call.budget, call.store), nc.observe(call), 0.0)
    return nc.compare_outcomes(nc.OP_COVERAGE, before, oracle, native)


def _field_rows(operation: str) -> list:
    directory = nc.matching_corpus()
    if directory is None:
        return []
    return [(directory, row) for row in nc.load_index(directory)["records"] if row["op"] == operation and not row.get("derived") and not row["exception"]]


def test_answers_through_the_attribute_protocol_equal_the_oracle():
    """Без быстрого пути слотов результат тот же (запасная дорога не гниёт): выборка полевых вызовов обеих операций."""

    coverage_rows = _field_rows(nc.OP_COVERAGE)[::9]
    clip_paths = geometry.field_paths()[::6]
    if not coverage_rows or not clip_paths:
        pytest.skip(nc.describe_missing_corpus() + ": настоящие вызовы не сверены")
    mirror = cftuv_native.new_mirror()
    mirror.raw_layouts()
    mirror.disable_raw()
    for directory, row in coverage_rows:
        found = _compare_coverage(mirror, nc.read_record(directory / row["path"]))
        assert not found, f"{row['id']}: " + "; ".join(str(item) for item in found[:4])
    runner = geometry.DropinRunner(mirror)
    runs = [(path.name, runner.compare(nc.read_record(path))) for path in clip_paths]
    failed = [(name, run) for name, run in runs if not run.equal]
    assert not failed, geometry.explain(failed)
    assert len(coverage_rows) >= 20 and len(runs) >= 10


# --------------------------------------------------------------------------
# ключ поиска в store
# --------------------------------------------------------------------------


def _key_of(fractions) -> tuple:
    return ("prime-universe", tuple(fractions))


def _store_key(plain: tuple):
    return native_cost.StoreKey((plain[0], plain[1], hash(plain), plain, []))


def test_the_store_lookup_key_hashes_and_compares_like_the_plain_key():
    plain = _key_of(Fraction(n, d) for n, d in ((1, 3), (-5, 7), (2, 9), (11, 2)))
    lookup = _store_key(plain)
    assert hash(lookup) == hash(plain)
    other = _key_of(Fraction(n, d) for n, d in ((1, 3), (-5, 7), (2, 9), (11, 2)))
    assert other is not plain and other == plain
    different = _key_of(Fraction(n, d) for n, d in ((1, 3), (-5, 7), (2, 9), (11, 3)))

    assert {plain: "mine"}.get(lookup) == "mine", "the key the miss wrote is found by identity"
    assert lookup[4] == [], "identity needs no memory"
    assert {other: "oracle"}.get(lookup) == "oracle", "an equal key made by someone else is found"
    assert lookup[4] == [other] and lookup[4][0] is other, "...and remembered"
    assert {other: "oracle"}.get(lookup) == "oracle", "...and found again from the memory"
    assert lookup == plain and lookup == other and not lookup != plain
    assert lookup != different and not lookup == different and lookup.__eq__(different) is False
    assert {different: "x"}.get(lookup) is None
    assert {lookup: 1}.get(plain) == 1 and {lookup: 1}.get(other) == 1, "the other direction: the plain key finds the entry too"
    assert lookup != "prime-universe" and lookup.__eq__(()) is False


def _coverage_rows_with_lines():
    rows = _field_rows(nc.OP_COVERAGE)
    return [(directory, row) for directory, row in rows if row["mesh"] != "building" or row["faces"] >= 4][::13]


def test_a_store_holding_the_oracles_key_is_hit_and_stays_as_the_oracle_wrote_it():
    """Попадание в `store`, заполненный эталоном (ключ — ДРУГОЙ объект): ответ и цена как у эталона, ключ в `store` остаётся обычным кортежем и тем же объектом."""

    rows = _coverage_rows_with_lines()
    if not rows:
        pytest.skip(nc.describe_missing_corpus() + ": настоящие вызовы не сверены")
    checked = 0
    for directory, row in rows[:12]:
        record = nc.read_record(directory / row["path"])
        before = record.before()
        # the oracle fills the store; the two states below then hold the same store content, the key an object the native side did not make
        fill = nc.prepare_call(nc.OP_COVERAGE, record.call_blob, before)
        nc.ORACLE[nc.OP_COVERAGE](fill.args[0], fill.args[1], fill.budget, fill.store)
        if not fill.store:
            continue
        keys = list(fill.store)
        recorded = nc.capture_state(fill.budget, fill.store)
        mirror = cftuv_native.new_mirror()
        for step, factor in enumerate((Fraction(1), Fraction(3, 4), Fraction(3, 4), Fraction(5, 4))):
            budget_o, store_o = nc.restore_state(recorded)
            partition, alpha = nc.decode_call(nc.OP_COVERAGE, record.call_blob, None, None).args
            oracle = nc.execute(nc.Call(nc.OP_COVERAGE, (partition, alpha * factor), {}, budget_o, store_o))
            budget_n, store_n = nc.restore_state(recorded)  # the process back in the state before the call: the native side starts where the oracle did
            partition_n, alpha_n = nc.decode_call(nc.OP_COVERAGE, record.call_blob, None, None).args
            call_n = nc.Call(nc.OP_COVERAGE, (partition_n, alpha_n * factor), {}, budget_n, store_n)
            started_keys = list(store_n)
            try:
                result, error = mirror.coverage_at(call_n.args[0], call_n.args[1], call_n.budget, call_n.store), None
            except Exception as exc:  # noqa: BLE001
                result, error = None, (type(exc).__qualname__, str(exc))
            native = nc.Outcome(result, error, nc.capture_state(call_n.budget, call_n.store), {}, 0.0)
            found = nc.compare_outcomes(nc.OP_COVERAGE, recorded, oracle, native)
            assert not found, f"{row['id']} step {step}: " + "; ".join(str(item) for item in found[:3])
            assert [type(key) for key in store_n] == [tuple] * len(store_n), "the store never holds the lookup key class"
            assert [id(key) for key in store_n] == [id(key) for key in started_keys], "a hit does not replace the key"
            checked += 1
        assert keys
    assert checked >= 8


def test_a_miss_writes_the_plain_key_and_the_next_call_hits_it():
    rows = _coverage_rows_with_lines()
    if not rows:
        pytest.skip(nc.describe_missing_corpus() + ": настоящие вызовы не сверены")
    directory, row = next((directory, row) for directory, row in rows if not nc.read_record(directory / row["path"]).before().store)
    record = nc.read_record(directory / row["path"])
    call = nc.prepare_call(nc.OP_COVERAGE, record.call_blob, record.before())
    store: dict = {}
    mirror = cftuv_native.new_mirror()
    mirror.coverage_at(call.args[0], call.args[1], call.budget, store)
    assert len(store) == 1 and [type(key) for key in store] == [tuple]
    key = next(iter(store))
    assert key[0] == "prime-universe" and all(type(value) is Fraction for value in key[1])
    spent = call.budget.spent_by_article() if call.budget is not None else None
    mirror.coverage_at(call.args[0], call.args[1], call.budget, store)
    assert len(store) == 1 and next(iter(store)) is key
    if spent is not None:
        assert call.budget.spent_by_article() == spent, "the second call is a store hit on a warm memory: it costs nothing"


# --------------------------------------------------------------------------
# разбор таблиц памяти
# --------------------------------------------------------------------------


def _general_diff_reference(mirror_keys: list, mirror_values: list, keys: list, values: list) -> tuple:
    """`native_cost._general_diff` до сокращения: полный разбор без раннего выхода."""

    index = {key: position for position, key in enumerate(mirror_keys)}
    last = -1
    cut = len(keys)
    for position, key in enumerate(keys):
        found = index.get(key)
        if found is None or found <= last or mirror_values[found] != values[position]:
            cut = position
            break
        last = found
    kept = set(keys[:cut])
    deleted = [key for key in mirror_keys if key not in kept]
    if len(deleted) > len(kept):
        return True, [], 0
    return False, deleted, cut


def test_the_general_diff_of_a_table_answers_as_the_full_comparison_does():
    rng = random.Random(5)
    early = 0
    for _ in range(4000):
        size = rng.randint(0, 40)
        mirror_keys = rng.sample(range(100), size)
        mirror_values = [rng.randint(0, 3) for _ in mirror_keys]
        shape = rng.choice(("subset", "shuffled", "fresh", "changed", "tail"))
        if shape == "subset":
            picked = [position for position in range(size) if rng.random() < rng.choice((0.1, 0.4, 0.8))]
        elif shape == "shuffled":
            picked = rng.sample(range(size), rng.randint(0, size))
        elif shape == "changed":
            picked = list(range(size))
        else:
            picked = list(range(size))
        keys = [mirror_keys[position] for position in picked]
        values = [mirror_values[position] for position in picked]
        if shape == "changed" and values:
            values[rng.randrange(len(values))] += 1
        if shape in ("fresh", "tail"):
            extra = [key for key in range(100, 140) if key not in keys]
            keys += rng.sample(extra, rng.randint(0, 20))
            values += [rng.randint(0, 3) for _ in range(len(keys) - len(values))]
        want = _general_diff_reference(mirror_keys, mirror_values, keys, values)
        got = native_cost._general_diff(mirror_keys, mirror_values, keys, values)
        assert got == want, (mirror_keys, mirror_values, keys, values)
        early += 2 * len(keys) < len(mirror_keys)
    assert early > 300


# --------------------------------------------------------------------------
# вид сеанса на таблицы памяти
# --------------------------------------------------------------------------


def _lengths_equal_the_real_tables(mirror) -> bool:
    return mirror.lengths() == (len(exact._KNOWN_PRIMES), len(exact._FACTORIZATION_MEMO), len(exact._SQUAREFREE_MEMO), len(exact._PRIME_SUPPORT_MEMO))


def test_unchanged_tables_are_recognised_by_identity_and_every_other_change_takes_the_full_comparison():
    mirror = cftuv_native.new_mirror()
    assert mirror.prime_support(30 * 49 * 11) == tuple(oracle_support(30 * 49 * 11))
    assert _lengths_equal_the_real_tables(mirror)
    slow = mirror.slow_syncs
    for _ in range(5):
        mirror.prime_support(30 * 49 * 11)
    assert mirror.slow_syncs == slow, "nothing changed between the calls: no full comparison"

    def changed(mutation, label):
        before = mirror.slow_syncs
        mutation()
        mirror.prime_support(30 * 49 * 11)  # a call whose sync must have brought the mirror to the tables
        assert mirror.slow_syncs == before + 1, label
        assert _lengths_equal_the_real_tables(mirror), label
        mirror.prime_support(30 * 49 * 11)
        assert mirror.slow_syncs == before + 1, label + ": the view is taken anew, the next call is fast again"

    changed(lambda: exact._FACTORIZATION_MEMO.__setitem__(99991, ((99991, 1),)), "an entry appended")
    changed(lambda: exact._SQUAREFREE_MEMO.__setitem__(778, (1, 778)), "a squarefree split appended")
    key = next(iter(exact._PRIME_SUPPORT_MEMO))
    changed(lambda: exact._PRIME_SUPPORT_MEMO.__setitem__(key, tuple(list(exact._PRIME_SUPPORT_MEMO[key]))), "a value replaced by an EQUAL object")
    changed(lambda: exact._PRIME_SUPPORT_MEMO.__setitem__(key, (key + 1,)), "a value replaced by another")

    def touch_oldest():
        assert len(exact._FACTORIZATION_MEMO) >= 2
        oldest = next(iter(exact._FACTORIZATION_MEMO))
        exact._FACTORIZATION_MEMO[oldest] = exact._FACTORIZATION_MEMO.pop(oldest)

    changed(touch_oldest, "the oldest entry touched (the order changed, the objects did not)")
    changed(lambda: exact._KNOWN_PRIMES.remove(exact._KNOWN_PRIMES[1]) or exact._KNOWN_PRIME_SET.discard(3), "a registry entry dropped")
    changed(exact.reset_factorization_memory, "everything reset")
    mirror.invalidate()
    before = mirror.slow_syncs
    exact._FACTORIZATION_MEMO[5] = ((5, 1),)
    mirror.prime_support(30)
    assert mirror.slow_syncs == before + 1 and _lengths_equal_the_real_tables(mirror), "an invalidated mirror reloads the tables whole"


def oracle_support(radicand: int) -> tuple:
    """`prime_support` of the Python oracle on a copy of the tables (the process state is not touched)."""

    with exact.isolated_factorization_memory():
        return tuple(exact.prime_support(radicand))
