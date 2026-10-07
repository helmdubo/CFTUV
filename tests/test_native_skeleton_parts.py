"""Нативный слой времён событий и закон кандидата скелета (`native/cftuv-skeleton`, WP-S0 и WP-S2) равны эталону на Python ПО ЧАСТЯМ.

Эталон — ядро `kernel/src/cftuv_envelope/wavefront`: `event_time.py` (`SupportLineV1`, `EventTimeV1`, `compare_times`, `times_are_equal`, `concurrency_time`,
`sliding_time`, `sliding_point`, `_event_point` с базисом простых и без), `events.py` (`EventQueueV1`: `heapq` со сравнением `compare_times`, `pop_level`,
`_count_at_time`) и закон кандидата (`candidate_law.evaluate_split_candidate` с `exact_candidate_view`: `position`, `span_containment`, память мест и времён
superlevel'а, ключи которой по `id()`), плюс `repr` как порядок (ключи сортировок ядра).

Метод — пошаговая сверка на ОДНОМ состоянии (`tools/native_leaf_gate.py`): состояние ДО вызова копируется, эталон исполняется на живом состоянии (его эффекты остаются:
прогон идёт, как без обёртки), нативный шов исполняется на копии и сравнивается ТОЧНО: результат канонически (`int` и `Fraction` различны), исключение `(класс, текст)`,
дельта `SIGN_COUNTS`, шесть статей бюджета (либо неоплаченная работа), журнал памяти канонизации против живых таблиц ПОСЛЕ (с порядком), рост памяти superlevel'а.
Источники вызовов: реальный `build_skeleton` на корпусах ядра (`wavefront_cases`: именованные, частичные источники, взвешенные; со всеми бюджетами: полный, урезанный до
исчерпания, `None`; плотный режим без памяти мест), случайные прямые и времена, случайные виды закона (все ветки: скольжение, замороженные места, стационарные прямые,
трасса, отказ по стыку), записанные полевые полигоны, если они лежат в корпусе.

Модуль пропускается с названной причиной, пока расширение не собрано (`python tools/native_build.py`) либо дерево ядра ушло от закреплённых файлов листа.
"""

from __future__ import annotations

import dataclasses
import math
import os
import random
import sys
from collections import Counter
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
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка слоя времён событий скелета с эталоном пропущена)",
        allow_module_level=True,
    )

from cftuv_native import skeleton_seams as wire  # noqa: E402

_STALE = wire.stale_leaf_files()
if _STALE:
    pytest.skip(
        f"файлы эталона, которые зеркалит нативный лист скелета, ушли от закрепления: {', '.join(_STALE)} (перенос дельты и новое закрепление — отдельный шаг)",
        allow_module_level=True,
    )

from native_gate import field_tier  # noqa: E402

import native_leaf_gate as gate  # noqa: E402
import native_corpus as nc  # noqa: E402
import wavefront_cases as cases  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.wavefront.candidate_law as candidate_law  # noqa: E402
import cftuv_envelope.wavefront.event_time as et  # noqa: E402
import cftuv_envelope.wavefront.events as events  # noqa: E402
import cftuv_envelope.wavefront.exact_candidate_view as candidate_view  # noqa: E402
import cftuv_envelope.wavefront.exact_identity as exact_identity  # noqa: E402
import cftuv_envelope.wavefront.motorcycle as motorcycle  # noqa: E402
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1  # noqa: E402

FIELD_POLYGONS = Path(os.environ.get("CFTUV_NATIVE_CORPUS", "E:/cftuv_native_corpus")) / "skeleton_polygons" / "field_polygons.pkl"
#: The polygons of the owner's FIELD scene (`tools/native_leaf_gate.py fetch`): they exist only on the owner's drive, so the tests that need them are the field tier (`native_gate.field_tier`: CI
#: deselects the tier by name and reports it; a skip would be a failure in strict mode).
needs_field_polygons = field_tier(FIELD_POLYGONS.exists(), f"нет записанных полевых полигонов ({FIELD_POLYGONS}): их кладёт `python tools/native_leaf_gate.py fetch`")
#: Каких полевых полигонов (подстроки имён) не брать в быстрый прогон: их вызовов десятки тысяч, их считает G1 (`tools/native_leaf_gate.py gate`).
FIELD_FAST = ("patch_105", "point_contact", "weighted_normals", "patch4", "patch3", "mesh2_patch2", "building004")

CHECKED: Counter = Counter()


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    """Эталон пишет в процессные счётчики и память ядра; тест их не оставляет."""

    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    audit = exact.set_canonical_audit(False)
    with exact.isolated_factorization_memory():
        yield
    exact.set_canonical_audit(audit)
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(nc._ARTICLES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line("native skeleton leaf compared with the Python oracle: " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items())))


def explain(verifier) -> str:
    found = verifier.mismatches
    return "\n".join(str(item) for item in found[:12]) + (f"\n... and {len(found) - 12} more" if len(found) > 12 else "")


def settle(verifier) -> None:
    """Ничего не разошлось, ничего не отказано портом; учёт сверенных вызовов в итог модуля."""

    assert not verifier.mismatches, explain(verifier)
    assert not verifier.unsupported, f"the port refused calls it should carry: {dict(verifier.unsupported)}"
    for seam, count in verifier.checked.items():
        CHECKED[seam] += count


def everything() -> "gate.Sampling":
    return gate.Sampling(head=10**9, stride=1, cap=10**9)


# --------------------------------------------------------------------------
# the table, the encoders
# --------------------------------------------------------------------------


def test_the_seam_table_of_the_shim_is_the_extensions():
    assert wire.SEAMS == cftuv_native.skeleton_seam_table()


def test_a_line_outside_the_machine_range_is_unsupported_not_wrong():
    with pytest.raises(wire.SeamUnsupported):
        wire.enc_line(et.SupportLineV1(1 << 62, 1, 0, 1))
    with pytest.raises(wire.SeamUnsupported):
        wire.enc_line(et.SupportLineV1(1, 1, 1 << 124, 1))
    big = et.SupportLineV1((1 << 62) - 1, -((1 << 62) - 1), (1 << 124) - 1, Fraction(1, 2))
    assert wire.enc_line(big)[:4] == [(1 << 62) - 1, -((1 << 62) - 1), (1 << 124) - 1, Fraction(1, 2)]
    with pytest.raises(wire.SeamUnsupported):
        wire.enc_line(et.SupportLineV1(1, 1, 0, 1.5))


def test_the_extreme_lines_of_the_range_give_the_oracles_answers():
    """The determinants of lines at the edge of the range (`a*b - a*b` near 2^125, offsets near 2^124) neither wrap nor round."""

    verifier = gate.LeafVerifier(everything())
    top, offset = (1 << 62) - 1, (1 << 124) - 1
    lines = [et.SupportLineV1(top, -top + 3, offset, 4), et.SupportLineV1(-top + 5, top - 7, -offset, Fraction(9, 4)), et.SupportLineV1(top - 11, top - 13, offset - 17, 0)]
    for first, second, third in ((lines[0], lines[1], lines[2]), (lines[2], lines[0], lines[1])):
        attempt(verifier, "CONCURRENCY_TIME", lambda: et.concurrency_time(first, second, third), lambda: [wire.enc_line(item) for item in (first, second, third)], wire.dec_time_entry, None)
        attempt(verifier, "SLIDING_TIME", lambda: et.sliding_time(first, SqrtSumV1.rational(5), second), lambda: [wire.enc_line(first), SqrtSumV1.rational(5), wire.enc_line(second)], wire.dec_time_entry, None)
        attempt(verifier, "SLIDING_POINT", lambda: et.sliding_point(first, SqrtSumV1.rational(5), et.ZERO_TIME), lambda: [wire.enc_line(first), SqrtSumV1.rational(5), wire.enc_time(et.ZERO_TIME)], wire.dec_point, None)
    settle(verifier)
    assert verifier.checked["CONCURRENCY_TIME"] == 2 and verifier.checked["SLIDING_POINT"] == 2


# --------------------------------------------------------------------------
# repr: the order key of ~65 sites of the kernel
# --------------------------------------------------------------------------

_ALPHABET = "abcXYZ019 _-'\"\\\n\t,.()[]{}"


def random_text(rng) -> str:
    return "".join(rng.choice(_ALPHABET) for _ in range(rng.randrange(0, 9)))


def random_number(rng):
    kind = rng.randrange(6)
    if kind == 0:
        return rng.randrange(-10, 11)
    if kind == 1:
        return rng.randrange(-(10**30), 10**30)
    if kind == 2:
        return Fraction(rng.randrange(-50, 51), rng.randrange(1, 40))
    if kind == 3:
        return Fraction(rng.randrange(-(10**25), 10**25), rng.randrange(1, 10**12))
    return rng.choice((0, 1, -1, 2**64, -(2**64), 2**200))


def random_sum(rng, *, int_coefficients: bool = False):
    if int_coefficients and rng.random() < 0.3:
        radicands = sorted(rng.sample((1, 2, 3, 5, 6, 7, 10, 11), rng.randrange(0, 4)))
        return SqrtSumV1(tuple((radicand, rng.choice((1, -3, 7)) if rng.random() < 0.5 else Fraction(rng.randrange(1, 9), rng.randrange(1, 9))) for radicand in radicands))
    total = SqrtSumV1.zero()
    for _ in range(rng.randrange(0, 4)):
        total = total + SqrtSumV1.radical(Fraction(rng.randrange(-9, 10), rng.randrange(1, 10)), Fraction(rng.randrange(1, 60), rng.choice((1, 1, 2, 3, 7))))
    return total


def random_time(rng):
    divisor = SqrtSumV1.rational(Fraction(rng.randrange(1, 9), rng.randrange(1, 9)))
    if rng.random() < 0.5:
        divisor = divisor + SqrtSumV1.radical(Fraction(rng.randrange(1, 9), rng.randrange(1, 9)), rng.choice((2, 3, 5, 6, 7, 10)))
    return et.EventTimeV1.normalized(Fraction(rng.randrange(-30, 31), rng.randrange(1, 12)), divisor)


def random_value(rng, depth: int = 0):
    """Anything the kernel puts into a sort key, nested."""

    kind = rng.randrange(14 if depth < 3 else 6)
    if kind == 0:
        return None
    if kind == 1:
        return rng.random() < 0.5
    if kind in (2, 3):
        return random_number(rng) if kind == 3 or rng.random() < 0.5 else int(random_number(rng))
    if kind == 4:
        return random_text(rng)
    if kind == 5:
        return Fraction(rng.randrange(-9, 10), rng.randrange(1, 9))
    if kind in (6, 7):
        return tuple(random_value(rng, depth + 1) for _ in range(rng.randrange(0, 5)))
    if kind == 8:
        return [random_value(rng, depth + 1) for _ in range(rng.randrange(0, 4))]
    if kind == 9:
        return random_sum(rng, int_coefficients=True)
    if kind == 10:
        return random_time(rng)
    if kind == 11:
        return et.EventPointV1(random_sum(rng), random_sum(rng))
    if kind == 12:
        return rng.choice(list(events.EventKind))
    return exact_identity.ExactIdentityKeyV1(tuple(random_value(rng, depth + 1) for _ in range(rng.randrange(0, 4))))


def test_py_repr_equals_python_repr_on_random_values():
    rng = random.Random(20261007)
    runner = wire.SeamRunner()
    checked = 0
    for _ in range(3000):
        value = random_value(rng)
        assert runner.value("PY_REPR", [wire.enc_repr(value)]) == repr(value), repr(value)
        checked += 1
    CHECKED["PY_REPR random"] += checked


def kernel_keys(rng) -> list:
    """The key classes the order sorts by (`superlevel_closure`, `symbolic_*`, `superlevel`), with the values they really hold."""

    from cftuv_envelope.wavefront import superlevel, superlevel_closure, superlevel_fixed_point, symbolic_edge_closure, symbolic_overlay, symbolic_split_endpoint

    def point_key():
        x, y = random_sum(rng), random_sum(rng)
        return exact_identity.exact_point_key(et.EventPointV1(x, y))

    def time_key():
        time = random_time(rng)
        return exact_identity.identity_key((time.dividend, time.divisor.terms))

    def occurrence():
        return tuple(rng.randrange(-5, 40) for _ in range(rng.choice((4, 5))))

    def family():
        return superlevel_closure.SpanFamilyRefV1(occurrence(), tuple(occurrence() for _ in range(rng.randrange(0, 3))))

    def junction():
        return symbolic_overlay.JunctionRefV1(rng.choice(("EXISTING", "BIRTH", "VIRTUAL_BOUNDARY")), (rng.randrange(0, 90),) if rng.random() < 0.5 else (point_key(), occurrence()))

    def contact():
        return superlevel_closure.SplitContactKeyV1(time_key(), point_key(), (point_key(),), family(), (occurrence(),))

    found = [junction(), family(), contact()]
    found.append(superlevel_closure.SegmentRefV1(family(), contact() if rng.random() < 0.7 else None, contact() if rng.random() < 0.7 else None, occurrence()))
    found.append(superlevel_fixed_point.SymbolicSplitContactKeyV1(time_key(), point_key(), junction(), family(), (occurrence(), occurrence())))
    found.append(symbolic_edge_closure.SymbolicEdgeContactKeyV1(time_key(), point_key(), junction(), junction(), family(), family(), family()))
    found.append(symbolic_split_endpoint.EndpointContactKeyV1(time_key(), point_key(), junction(), junction(), family(), (occurrence(),)))
    found.append(superlevel.BoundaryBirthV1(point_key(), occurrence(), occurrence(), (rng.randrange(9),), (rng.randrange(9),)))
    return found


def test_py_repr_equals_python_repr_on_the_real_key_classes_of_the_kernel():
    rng = random.Random(7)
    runner = wire.SeamRunner()
    checked = 0
    for _ in range(150):
        keys = kernel_keys(rng)
        for key in [*keys, tuple(keys[:2]), [keys[2]], (keys[3],)]:
            assert runner.value("PY_REPR", [wire.enc_repr(key)]) == repr(key), repr(key)[:300]
            checked += 1
    CHECKED["PY_REPR kernel keys"] += checked


def test_repr_orders_by_text_not_by_number():
    """The reason the module exists: `'10' < '9'`, `Fraction(1, 10)` before `Fraction(1, 9)`: the order key is the TEXT."""

    runner = wire.SeamRunner()
    texts = [runner.value("PY_REPR", [wire.enc_repr(value)]) for value in (9, 10, Fraction(1, 9), Fraction(1, 10))]
    assert texts == ["9", "10", "Fraction(1, 9)", "Fraction(1, 10)"]
    assert sorted(texts[:2]) == ["10", "9"] and sorted(texts[2:]) == ["Fraction(1, 10)", "Fraction(1, 9)"]


def test_repr_refuses_by_name_a_character_it_cannot_print_like_cpython():
    runner = wire.SeamRunner()
    answer = runner.call("PY_REPR", [wire.enc_repr("é")])
    assert answer.status == wire.STATUS_UNSUPPORTED and "Unicode" in wire.dec_str(answer.detail[0])


# --------------------------------------------------------------------------
# lines and times
# --------------------------------------------------------------------------


def random_speed(rng):
    kind = rng.randrange(6)
    if kind == 0:
        return 0
    if kind == 1:
        return rng.choice((1, 2, 3, 4, 8, 9, 12, 18, 50, 72, 100, 121))
    if kind == 2:
        return Fraction(rng.randrange(1, 40), rng.choice((2, 3, 5, 7, 12)))
    if kind == 3:
        return Fraction(rng.choice((4, 9, 16, 25)), 1)
    return rng.randrange(1, 300)


def random_line(rng, *, span: int = 40, speed=None):
    while True:
        start = (rng.randrange(-span, span + 1), rng.randrange(-span, span + 1))
        end = (rng.randrange(-span, span + 1), rng.randrange(-span, span + 1))
        if start != end:
            break
    q = random_speed(rng) if speed is None else speed
    return et.SupportLineV1.with_speed(start, end, q)


def random_budget(rng):
    kind = rng.randrange(5)
    if kind == 0:
        return None
    if kind == 1:
        return exact.unlimited_reference_budget(stage="LEAF")
    if kind == 2:
        return exact.exact_work_budget(stage="LEAF", domain_id="d", cap=rng.randrange(0, 40))
    return exact.exact_work_budget(stage="LEAF", domain_id="leaf")


def attempt(verifier, seam, oracle, encode, decode, budget, **options):
    """One lockstep call; the oracle's own exception is part of the outcome (compared inside) and is not the test's."""

    try:
        return verifier.lockstep(seam, oracle, encode, decode, budget=budget, **options)
    except (exact.ExactCanonicalizationWorkBudgetExhausted, et.ZeroDivisorTimeError, et.ParallelSupportLinesError, ZeroDivisionError, exact.ZeroSqrtSumDivisorError, ValueError):
        return None


def test_support_line_algebra_equals_the_oracle_including_its_refusals():
    rng = random.Random(3)
    runner = wire.SeamRunner()
    checked = Counter()
    for _ in range(600):
        start = (rng.randrange(-30, 31), rng.randrange(-30, 31))
        end = start if rng.random() < 0.1 else (rng.randrange(-30, 31), rng.randrange(-30, 31))
        q = random_speed(rng)
        if rng.random() < 0.1:
            q = -Fraction(rng.randrange(1, 9), rng.choice((1, 2, 3))) if rng.random() < 0.5 else -rng.randrange(1, 9)
        for mode, build in ((0, lambda: et.SupportLineV1.with_speed(start, end, q)), (1, lambda: et.SupportLineV1.through(start, end))):
            answer = runner.call("SUPPORT_LINE", [start[0], start[1], end[0], end[1], q, mode])
            try:
                line = build()
            except (et.DegenerateEdgeError, et.NegativeSpeedError) as error:
                assert not answer.ok and wire.exception_of(answer, None) == (type(error).__qualname__, str(error)), (start, end, q, mode)
                checked["refused"] += 1
                continue
            assert answer.ok and nc.canonical(tuple(answer.value)) == nc.canonical((line.a, line.b, line.c, line.q)), (start, end, q, mode)
            assert line.normal_squared == line.a * line.a + line.b * line.b
            checked["built"] += 1
    assert checked["refused"] > 50 and checked["built"] > 600
    CHECKED["SUPPORT_LINE"] += sum(checked.values())


def time_pool(rng, size: int = 24) -> list:
    """Times of every kind the layer meets: rational, irrational divisors, zero, negative, equal values in different forms."""

    pool = [et.ZERO_TIME]
    while len(pool) < size:
        lines = [random_line(rng, speed=rng.choice((1, 2, 3, 4, 9, 16, Fraction(5, 2), 7))) for _ in range(3)]
        time, outcome = et.concurrency_time(*lines)
        if time is not None:
            pool.append(time)
            if rng.random() < 0.3:
                pool.append(time.canonical())
                pool.append(et.EventTimeV1(time.dividend * 3, time.divisor.scaled(3)))
        pool.append(random_time(rng))
    return pool


def test_compare_times_and_the_time_operations_equal_the_oracle_on_random_times():
    rng = random.Random(11)
    verifier = gate.LeafVerifier(everything())
    pool = time_pool(rng)
    for _ in range(900):
        left, right = rng.choice(pool), rng.choice(pool)
        budget = random_budget(rng)
        attempt(verifier, "COMPARE_TIMES", lambda: et.compare_times(left, right, budget), lambda: [wire.enc_time(left), wire.enc_time(right)], lambda value: value, budget)
        attempt(verifier, "TIMES_ARE_EQUAL", lambda: et.times_are_equal(left, right), lambda: [wire.enc_time(left), wire.enc_time(right)], lambda value: value, None)
    for time in pool:
        attempt(verifier, "TIME_CANONICAL", lambda: time.canonical(), lambda: [wire.enc_time(time)], wire.dec_time, None)
    for _ in range(200):
        dividend, divisor = Fraction(rng.randrange(-20, 21), rng.randrange(1, 9)), random_sum(rng)
        budget = random_budget(rng)
        attempt(verifier, "TIME_NORMALIZED", lambda: et.EventTimeV1.normalized(dividend, divisor, budget), lambda: [dividend, divisor], wire.dec_time, budget)
    settle(verifier)
    assert verifier.checked["COMPARE_TIMES"] >= 900 and verifier.checked["TIME_NORMALIZED"] >= 200


def near_cancelling_sums() -> list:
    """`1 +- (a*sqrt(2) - b*sqrt(3))` for the convergents `a/b` of `sqrt(3/2)`: within about 1/b of one, so the 64-bit enclosure cannot decide the sign of the difference with one once `b` is
    beyond 2^64, and the question goes to the conjugation."""

    ratio = Fraction(math.isqrt(3 * 10**80 // 2), 10**40)
    found = []
    for limit in (10**9, 10**17, 10**20, 10**22, 10**24):
        fraction = ratio.limit_denominator(limit)
        difference = SqrtSumV1.radical(fraction.numerator, 2) - SqrtSumV1.radical(fraction.denominator, 3)
        found.append(SqrtSumV1.rational(1) + difference)
        found.append(SqrtSumV1.rational(1) - difference)
    return found


def test_compare_times_goes_to_the_conjugation_when_the_enclosure_cannot_decide():
    verifier = gate.LeafVerifier(everything())
    one = et.EventTimeV1(Fraction(1), SqrtSumV1.rational(1))
    reached = 0
    for divisor in near_cancelling_sums():
        if divisor.sign() <= 0:
            continue
        time = et.EventTimeV1.normalized(Fraction(1), divisor)
        for budget in (None, exact.exact_work_budget(stage="LEAF", domain_id="conj"), *(exact.exact_work_budget(stage="LEAF", domain_id="starved", cap=cap) for cap in (0, 1))):
            before = exact.SIGN_COUNTS["closed_by_conjugation"]
            with exact.isolated_factorization_memory():  # a cold memory: a warm one answers before it pays, and a starved budget would never run out
                for left, right in ((time, one), (one, time)):
                    attempt(verifier, "COMPARE_TIMES", lambda: et.compare_times(left, right, budget), lambda: [wire.enc_time(left), wire.enc_time(right)], lambda value: value, budget)
            reached += exact.SIGN_COUNTS["closed_by_conjugation"] - before
    settle(verifier)
    assert reached >= 6, "the crafted times never reached the conjugation: the test does not test what it says"
    assert verifier.raised["COMPARE_TIMES"] >= 2, "the starved budget did not run out inside the conjugation"


def test_the_queue_runs_out_of_budget_at_the_comparison_the_oracle_does():
    """Pushes whose order only the conjugation can tell, on budgets that run out: the exhaustion falls inside `heappush`, at the same comparison, with the same articles."""

    verifier = gate.LeafVerifier(everything())
    sums = [divisor for divisor in near_cancelling_sums() if divisor.sign() > 0][:6]
    times = [et.EventTimeV1.normalized(Fraction(1), divisor) for divisor in sums] + [et.EventTimeV1(Fraction(1), SqrtSumV1.rational(1))]
    for cap in (None, 0, 1, 2, 8, 40):
        budget = None if cap is None else exact.exact_work_budget(stage="LEAF", domain_id="queue", cap=cap)
        script = [("push", time, index) for index, time in enumerate(times)] + [("pop",), ("pop",), ("count", times[0])]
        with exact.isolated_factorization_memory():
            attempt(verifier, "QUEUE_SCRIPT", lambda: run_python_script(script, budget), lambda: [script_wire(script)], decode_script, budget)
    settle(verifier)
    assert verifier.raised["QUEUE_SCRIPT"] >= 2


def test_the_event_time_of_three_lines_and_of_a_sliding_point_equal_the_oracle():
    rng = random.Random(23)
    verifier = gate.LeafVerifier(everything())
    outcomes = Counter()
    for _ in range(700):
        budget = random_budget(rng)
        first, second, third = (random_line(rng, span=rng.choice((6, 40, 3000))) for _ in range(3))
        if rng.random() < 0.15:
            third = et.SupportLineV1(first.a, first.b, first.c + rng.randrange(-3, 4), first.q)
        got = attempt(verifier, "CONCURRENCY_TIME", lambda: et.concurrency_time(first, second, third, budget), lambda: [wire.enc_line(item) for item in (first, second, third)], wire.dec_time_entry, budget)
        if got is not None:
            outcomes[got[1].value] += 1
        along = random_sum(rng)
        got = attempt(verifier, "SLIDING_TIME", lambda: et.sliding_time(first, along, second, budget), lambda: [wire.enc_line(first), along, wire.enc_line(second)], wire.dec_time_entry, budget)
        if got is not None:
            outcomes["sliding " + got[1].value] += 1
    settle(verifier)
    assert outcomes["EXACT"] > 100 and outcomes["WAVEFRONT_TRIPLE_NEVER_CONCURRENT"] > 5 and outcomes["sliding EXACT"] > 100
    assert verifier.raised["CONCURRENCY_TIME"] + verifier.raised["SLIDING_TIME"] > 5, "no starved budget reached an exhaustion inside the time laws"


def meeting_lines(rng, count: int = 3):
    """Lines through one point at one integer time (squared speeds, so `sqrt(q)` is whole): their concurrency time is that time and the event gets a place."""

    point, moment = (rng.randrange(-20, 21), rng.randrange(-20, 21)), rng.randrange(1, 9)
    lines = []
    while len(lines) < count:
        a, b = rng.randrange(-9, 10), rng.randrange(-9, 10)
        if a == 0 and b == 0:
            continue
        root = rng.randrange(0, 6)
        lines.append(et.SupportLineV1(a, b, a * point[0] + b * point[1] - moment * root, root * root))
    return lines


def test_the_places_of_an_event_equal_the_oracle_with_and_without_a_prime_universe():
    rng = random.Random(31)
    verifier = gate.LeafVerifier(everything())
    universe_pool = ((), (2,), (2, 3), (2, 3, 5, 7, 11, 13), (3, 5, 7))
    shown = Counter()
    for round_number in range(500):
        budget = random_budget(rng)
        lines = meeting_lines(rng, 3) if round_number % 2 else [random_line(rng) for _ in range(3)]
        time, _outcome = et.concurrency_time(*lines)
        if time is None:
            time = rng.choice(time_pool(rng, 4))
        first, second = lines[0], lines[1]
        universe = rng.choice(universe_pool) if rng.random() < 0.7 else None
        got = attempt(
            verifier,
            "EVENT_POINT",
            lambda: et._event_point(first, second, time, prime_universe=universe, budget=budget),
            lambda: [wire.enc_line(first), wire.enc_line(second), wire.enc_time(time), None if universe is None else list(universe)],
            wire.dec_point,
            budget,
        )
        shown["parallel" if got is None and first.a * second.b - second.a * first.b == 0 else "other"] += 1
        along = random_sum(rng)
        attempt(verifier, "SLIDING_POINT", lambda: et.sliding_point(first, along, time, budget), lambda: [wire.enc_line(first), along, wire.enc_time(time)], wire.dec_point, budget)
    settle(verifier)
    assert verifier.raised["EVENT_POINT"] > 5 and shown["parallel"] > 3, dict(shown)


def test_a_zero_divisor_and_parallel_lines_are_refused_with_the_oracles_text():
    verifier = gate.LeafVerifier(everything())
    line = et.SupportLineV1.through((0, 0), (4, 0))
    parallel = et.SupportLineV1.through((0, 3), (4, 3))
    for oracle_call in (
        lambda: et._event_point(line, parallel, et.ZERO_TIME, prime_universe=None, budget=None),
        lambda: et._event_point(line, parallel, et.ZERO_TIME, prime_universe=(2, 3), budget=None),
    ):
        with pytest.raises(et.ParallelSupportLinesError):
            verifier.lockstep("EVENT_POINT", oracle_call, lambda: [wire.enc_line(line), wire.enc_line(parallel), wire.enc_time(et.ZERO_TIME), None], wire.dec_point, budget=None)
    with pytest.raises(et.ZeroDivisorTimeError):
        verifier.lockstep("TIME_NORMALIZED", lambda: et.EventTimeV1.normalized(Fraction(1), SqrtSumV1.zero()), lambda: [Fraction(1), SqrtSumV1.zero()], wire.dec_time, budget=None)
    settle(verifier)
    assert verifier.raised["EVENT_POINT"] == 2 and verifier.raised["TIME_NORMALIZED"] == 1


# --------------------------------------------------------------------------
# the queue
# --------------------------------------------------------------------------


def candidate_event(time, tag: int):
    return events.CandidateEventV1(events.EventKind.SPLIT, time, et.EventPointV1(SqrtSumV1.zero(), SqrtSumV1.zero()), tag, -1, -1)


def run_python_script(script, budget) -> list:
    queue = events.EventQueueV1(work_budget=budget)
    results = []
    for op in script:
        if op[0] == "push":
            queue.push(candidate_event(op[1], op[2]))
            results.append(None)
        elif op[0] == "pop":
            results.append([event.vertex for event in queue.pop_level()])
        elif op[0] == "peek":
            results.append(queue.peek_time())
        else:
            results.append(queue._count_at_time(op[1]))
    return [results, [entry.sequence for entry in queue._heap], queue.pushed, queue.popped]


def script_wire(script) -> list:
    out = []
    for op in script:
        if op[0] == "push":
            out.append([0, wire.enc_time(op[1]), op[2]])
        elif op[0] == "pop":
            out.append([1])
        elif op[0] == "peek":
            out.append([2])
        else:
            out.append([3, wire.enc_time(op[1])])
    return out


def decode_script(answer_value):
    results, arrangement, pushed, popped = answer_value
    decoded = []
    for item in results:
        if isinstance(item, list) and len(item) == 2 and isinstance(item[1], SqrtSumV1):
            decoded.append(wire.dec_time(item))
        else:
            decoded.append(item)
    return [decoded, list(arrangement), pushed, popped]


def test_the_event_queue_equals_heapq_with_compare_times_on_random_scripts_with_ties():
    rng = random.Random(41)
    verifier = gate.LeafVerifier(everything())
    pops = 0
    for round_number in range(120):
        pool = time_pool(rng, 12) if round_number % 3 else [et.EventTimeV1(Fraction(rng.randrange(0, 4)), SqrtSumV1.rational(1)) for _ in range(6)]
        script = []
        for _ in range(rng.randrange(4, 80)):
            roll = rng.random()
            if roll < 0.55:
                script.append(("push", rng.choice(pool), rng.randrange(1000)))
            elif roll < 0.8:
                script.append(("pop",))
            elif roll < 0.9:
                script.append(("peek",))
            else:
                script.append(("count", rng.choice(pool)))
        budget = random_budget(rng)
        pops += sum(1 for op in script if op[0] == "pop")
        attempt(verifier, "QUEUE_SCRIPT", lambda: run_python_script(script, budget), lambda: [script_wire(script)], decode_script, budget)
    settle(verifier)
    assert pops > 500 and verifier.checked["QUEUE_SCRIPT"] == 120


# --------------------------------------------------------------------------
# the candidate law on real skeletons
# --------------------------------------------------------------------------


def run_with_leaf(verifier, polygon, *, budget_kind: str = "full", dense: bool = False):
    """`build_skeleton` on a cold state with every `evaluate_split_candidate` call checked against the native seam; `budget_kind`: full, none, or a fraction of the spent."""

    from cftuv_envelope.wavefront.skeleton import build_skeleton

    spent = None
    if budget_kind not in ("full", "none"):
        gate.fresh_process_state()
        probe = exact.exact_work_budget(stage="PREPARE", domain_id="probe")
        build_skeleton(polygon, work_budget=probe, dense_hydration=dense)
        spent = probe.spent
    budget = gate.fresh_process_state()
    if budget_kind == "none":
        budget = None
    elif spent is not None:
        budget = exact.exact_work_budget(stage="PREPARE", domain_id="starved", cap=max(1, int(spent * float(budget_kind))))
    with verifier.installed():
        try:
            return build_skeleton(polygon, work_budget=budget, dense_hydration=dense)
        except exact.ExactCanonicalizationWorkBudgetExhausted as error:
            return error


def named_polygons() -> list:
    return list(cases.named_corpus()) + list(cases.partial_source_corpus()) + list(cases._weighted_entries())


def test_evaluate_split_candidate_equals_the_oracle_on_every_call_of_the_named_polygons():
    verifier = gate.LeafVerifier(everything())
    polygons = named_polygons()
    for name, polygon in polygons:
        run_with_leaf(verifier, polygon)
    settle(verifier)
    assert verifier.checked["EVALUATE_SPLIT_CANDIDATE"] > 1000
    for reason in ("CANDIDATE", "FILTER_EVENT_IN_THE_PAST", "FILTER_POINT_OUTSIDE_FRONT", "FILTER_BEYOND_TRACE", "FILTER_TRIPLE_NEVER_CONCURRENT"):
        assert verifier.outcomes[reason] > 0, f"no call of the corpus ended in {reason}: {dict(verifier.outcomes)}"
    CHECKED["named polygons"] += len(polygons)


def chosen_polygons() -> list:
    return [(name, polygon) for name, polygon in named_polygons() if name.startswith(("comb", "ell", "star", "staircase", "holes", "cross", "double_notch", "rect_12x8", "field"))][:14]


@pytest.mark.parametrize("kind", ("none", "dense"))
def test_evaluate_split_candidate_equals_the_oracle_unbudgeted_and_without_the_memory_of_places(kind):
    """`None` is the unbudgeted telemetry (`UNBUDGETED_WORK`); `dense` is the reference mode without the memory of places (`position_memo is None`)."""

    verifier = gate.LeafVerifier(everything())
    for _name, polygon in chosen_polygons():
        run_with_leaf(verifier, polygon, budget_kind="none" if kind == "none" else "full", dense=kind == "dense")
    settle(verifier)
    assert verifier.checked["EVALUATE_SPLIT_CANDIDATE"] > 200


def test_evaluate_split_candidate_equals_the_oracle_when_the_budget_runs_out_inside_the_law():
    """A cap just under what the run spends: the exhaustion falls in the event loop, at the operation the oracle's falls at (inside a time, a place, a sign), with the articles and
    memory it leaves; calls before it are the ordinary ones."""

    verifier = gate.LeafVerifier(everything())
    for fraction in ("0.7", "0.9", "0.98", "0.995", "0.9995"):
        for _name, polygon in chosen_polygons():
            run_with_leaf(verifier, polygon, budget_kind=fraction)
    settle(verifier)
    assert verifier.checked["EVALUATE_SPLIT_CANDIDATE"] > 200
    raised = sum(verifier.raised[name] for name in verifier.raised)
    assert raised >= 3, f"the starved runs never ran out inside a checked call: {dict(verifier.raised)}"


@needs_field_polygons
def test_the_polygons_of_the_field_equal_the_oracle_when_they_are_in_the_corpus():
    verifier = gate.LeafVerifier(everything())
    polygons = gate.load_polygons(FIELD_POLYGONS, only=FIELD_FAST)
    assert polygons
    for _name, polygon in polygons:
        run_with_leaf(verifier, polygon)
    settle(verifier)
    CHECKED["field polygons"] += len(polygons)


# --------------------------------------------------------------------------
# the law on random views: every branch, `position` and `span_containment` on their own
# --------------------------------------------------------------------------


def random_trace(rng, crash_time):
    return motorcycle.TraceV1(
        0,
        next(iter(motorcycle.TraceOutcome)),
        et.SupportLineV1(1, 0, 0, 1),
        et.SupportLineV1(0, 1, 0, 1),
        et.ZERO_TIME,
        et.EventPointV1(SqrtSumV1.zero(), SqrtSumV1.zero()),
        (SqrtSumV1.zero(), SqrtSumV1.zero()),
        crash_time,
        None,
        next(iter(motorcycle.CrashKind)),
        0,
        None,
    )


def small_time(rng):
    return et.EventTimeV1(Fraction(rng.randrange(0, 9), rng.randrange(1, 4)), SqrtSumV1.rational(1))


class RandomView:
    """A view made of dictionaries: vertices, spans, traces. References are plain ints."""

    def __init__(self, rng, *, budget, memo: bool) -> None:
        meeting = rng.random() < 0.6
        lines = meeting_lines(rng, 4) if meeting else [random_line(rng, span=12) for _ in range(4)]
        if rng.random() < 0.2:
            lines[1] = lines[0]
        if rng.random() < 0.15:
            lines[3] = et.SupportLineV1(lines[0].a, lines[0].b, lines[0].c, 0)
        vertex_count, span_count = rng.randrange(2, 5), rng.randrange(3, 6)
        sliding = lambda: random_sum(rng) if rng.random() < 0.3 else None  # noqa: E731
        self.vertices = {
            number: candidate_view.CandidateVertexStateV1(rng.randrange(span_count), rng.randrange(span_count), small_time(rng) if rng.random() < 0.5 else et.ZERO_TIME, sliding())
            for number in range(vertex_count)
        }
        maybe_vertex = lambda: rng.randrange(vertex_count) if rng.random() < 0.85 else None  # noqa: E731
        frozen = rng.random() < 0.4
        self.spans = {
            number: candidate_view.CandidateSpanStateV1(
                rng.choice(lines),
                tuple(rng.randrange(-9, 10) for _ in range(5 if rng.random() < 0.08 else 4)),
                maybe_vertex(),
                maybe_vertex(),
                small_time(rng) if frozen else None,
                et.EventPointV1(random_sum(rng), random_sum(rng)) if frozen and rng.random() < 0.7 else None,
                et.EventPointV1(random_sum(rng), random_sum(rng)) if frozen and rng.random() < 0.7 else None,
            )
            for number in range(span_count)
        }
        self.traces = {number: random_trace(rng, small_time(rng) if rng.random() < 0.6 else None) for number in range(vertex_count) if rng.random() < 0.5}
        universe = rng.choice(((), (2, 3), (2, 3, 5, 7, 11, 13), (3, 5, 7)))
        self.budget, self.universe = budget, universe
        # a memory that does not serve the view's basis (`admits` is false) is read by nobody and written by nobody
        self.memo = candidate_view.PositionMemoV1(universe if rng.random() < 0.9 else (13, 17)) if memo else None
        self.span_count, self.vertex_count = span_count, vertex_count
        # the spans of the vertices that stand on `lines[0]`, `lines[1]`: the law reads three different lines
        for number in range(vertex_count):
            state = self.vertices[number]
            if rng.random() < 0.6:
                self.vertices[number] = candidate_view.CandidateVertexStateV1(0, 1, state.birth, state.sliding)
        for number, line in ((0, lines[0]), (1, lines[1]), (2, lines[2])):
            if number < span_count:
                old = self.spans[number]
                self.spans[number] = dataclasses.replace(old, line=line)

    def view(self):
        def trace_bounds(ref, time):
            trace = self.traces.get(ref)
            return None if trace is None else trace.bounds_time(time, self.budget)

        return candidate_view.ExactCandidateViewV1(self.universe, self.vertices.__getitem__, self.spans.__getitem__, trace_bounds, self.budget, self.memo)


def test_the_law_equals_the_oracle_on_random_views_in_every_branch():
    rng = random.Random(59)
    verifier = gate.LeafVerifier(everything())
    with verifier.installed():
        for round_number in range(1500):
            budget = random_budget(rng)
            built = RandomView(rng, budget=budget, memo=rng.random() < 0.8)
            view = built.view()
            now = small_time(rng) if rng.random() < 0.5 else et.ZERO_TIME
            vertex, target = rng.randrange(built.vertex_count), rng.randrange(built.span_count)
            factory = (lambda: ("proof", vertex)) if rng.random() < 0.5 else None
            try:
                for _repeat in range(2):  # the second call meets the memory the first one filled
                    candidate_law.evaluate_split_candidate(view, vertex, target, now=now, proof_identity_factory=factory)
            except (exact.ExactCanonicalizationWorkBudgetExhausted, et.ParallelSupportLinesError, ValueError, ZeroDivisionError, exact.ZeroSqrtSumDivisorError):
                continue
    settle(verifier)
    assert verifier.checked["EVALUATE_SPLIT_CANDIDATE"] > 2000
    for reason in ("CANDIDATE", "FILTER_POINT_OUTSIDE_FRONT", "FILTER_BEYOND_TRACE", "FILTER_EVENT_IN_THE_PAST", "FILTER_TRIPLE_NEVER_CONCURRENT", "NO_RULE_TRIPLE_ALWAYS_CONCURRENT"):
        assert verifier.outcomes[reason] > 0, f"no random view ended in {reason}: {dict(verifier.outcomes)}"
    assert verifier.raised["EVALUATE_SPLIT_CANDIDATE"] > 0, "no random view exhausted a budget or raised inside the law"


def test_position_and_span_containment_on_their_own_equal_the_oracle():
    rng = random.Random(61)
    verifier = gate.LeafVerifier(everything())
    with verifier.installed():
        for _ in range(900):
            budget = random_budget(rng)
            built = RandomView(rng, budget=budget, memo=rng.random() < 0.7)
            view, time = built.view(), rng.choice(time_pool(rng, 3))
            vertex, span = rng.randrange(built.vertex_count), rng.randrange(built.span_count)
            point = et.EventPointV1(random_sum(rng), random_sum(rng))
            for call in (lambda: verifier.check_position(view, vertex, time), lambda: verifier.check_containment(view, span, point, time)):
                try:
                    call()
                    call()
                except (exact.ExactCanonicalizationWorkBudgetExhausted, et.ParallelSupportLinesError, ValueError, ZeroDivisionError, exact.ZeroSqrtSumDivisorError):
                    pass
    settle(verifier)
    assert verifier.checked["POSITION"] > 500 and verifier.checked["SPAN_CONTAINMENT"] > 500


def test_a_source_span_of_five_numbers_raises_the_interpreters_own_text_on_a_stationary_line():
    """`x0, y0, x1, y1 = span.source_span` of a fan span (five numbers) on a wall: the oracle's ValueError, word for word."""

    verifier = gate.LeafVerifier(everything())
    wall = et.SupportLineV1(1, 0, 0, 0)
    mover = et.SupportLineV1(0, 1, 0, 1)
    state = candidate_view.CandidateVertexStateV1(0, 1, et.ZERO_TIME, None)
    for source in ((1, 2, 3, 4, 5), (1, 2, 3)):
        spans = {0: candidate_view.CandidateSpanStateV1(mover, (0, 0, 1, 1), None, None), 1: candidate_view.CandidateSpanStateV1(mover, (0, 0, 1, 1), None, None), 2: candidate_view.CandidateSpanStateV1(wall, source, 0, 0)}
        view = candidate_view.ExactCandidateViewV1((), lambda ref: state, spans.__getitem__, lambda ref, time: None, None, None)
        with verifier.installed(), pytest.raises(ValueError, match="values to unpack"):
            verifier.check_containment(view, 2, et.EventPointV1(SqrtSumV1.zero(), SqrtSumV1.zero()), et.ZERO_TIME)
    settle(verifier)
    assert verifier.raised["SPAN_CONTAINMENT"] == 2


# --------------------------------------------------------------------------
# the harness is not vacuous
# --------------------------------------------------------------------------


def test_the_verifier_reports_a_one_sign_difference_and_a_wrong_answer():
    """A comparison that cannot fail proves nothing: a sign counted that the native side does not count, and an answer that is not the oracle's, are both reported."""

    verifier = gate.LeafVerifier(everything())
    left, right = et.ZERO_TIME, et.EventTimeV1(Fraction(1), SqrtSumV1.rational(1))

    def one_sign_too_many():
        result = et.compare_times(left, right)
        exact.SIGN_COUNTS["total"] += 1
        return result

    verifier.lockstep("COMPARE_TIMES", one_sign_too_many, lambda: [wire.enc_time(left), wire.enc_time(right)], lambda value: value, budget=None)
    assert [item.field for item in verifier.mismatches] == ["sign_counts"]
    verifier.mismatches.clear()
    verifier.lockstep("COMPARE_TIMES", lambda: et.compare_times(left, right), lambda: [wire.enc_time(right), wire.enc_time(left)], lambda value: value, budget=None)
    assert [item.field for item in verifier.mismatches] == ["result"]
    verifier.mismatches.clear()

    def memory_the_native_side_does_not_make():
        exact._SQUAREFREE_MEMO[7919 * 7919 * 3] = (7919, 3)
        return et.compare_times(left, right)

    verifier.lockstep("COMPARE_TIMES", memory_the_native_side_does_not_make, lambda: [wire.enc_time(left), wire.enc_time(right)], lambda value: value, budget=None)
    assert [item.field for item in verifier.mismatches] == ["memory.squarefree"]
