"""Первая треть нативной `clip_geometry` (`native/cftuv-clip`) равна эталону на Python ПО ЧАСТЯМ: числовой фасад, ячейки, привязка, тесселяция.

Эталон — ядро `kernel/src/cftuv_envelope` (WP-A: `line_value`, `window`, `values_in`, `stretch_square`, `lift_known`, `nanometres`,
`milli_cells`, `faces.orientation/shoelace_sign/doubled_shoelace`, `clip_snap.within_edge_gap`, `clip_cells.chord_of/hinge_depth_square`,
`ClipStageV1._cheap_sign/_edge_constants/_rational_pair`; WP-B: `build_cells`, `snap_source_vertices`; WP-C: `triangulate_exact`,
`convex_quad_ring`, `has_right_turn`; и `_ordered` — единственное место, где результат зависит от минорной версии CPython).

Метод: пока эталон воспроизводит запись корпуса (`tools/native_clip_seams.py`), записывающие обёртки на швах снимают каждый вызов (аргументы
в проводе, результат или исключение, состояние цены ДО и ПОСЛЕ); потом тот же вызов идёт в нативный шов на записанном состоянии
(`SeamRunner`, заголовок загружает таблицы памяти целиком) и сравнивается ТОЧНО: результат каноническим кодом (`int` и `Fraction`
различны, `float` по `hex`), исключение `(класс, текст)`, дельта `SIGN_COUNTS`, статьи бюджета, журнал памяти против таблиц ПОСЛЕ.

Источники вызовов: 1. ПОЛЕВОЙ корпус ЭТОГО ядра (`nc.matching_corpus`, `E:/cftuv_native_corpus/<HEAD>`: вызовы `clip_geometry` из Blender и производные с урезанным бюджетом);
2. СИНТЕТИЧЕСКИЙ корпус (`synthetic_clip`: вызовы `clip_geometry` и швов из тестов ядра `kernel/tests/test_clip_*.py`, `test_materialize_*`, ...;
строит `tools/native_clip_synthetic.py`); 3. ЦЕЛЕВЫЕ случаи этого файла для веток, которых нет ни там, ни там (сопряжение и исчерпание бюджета на
каждой границе, переполнения float, нецелая карта, отказы ячеек, подавления привязки, `doubled_shoelace`).

Эмуляция CPython: порядок вопросов `list.sort` (`count_run` 3.11 и 3.13) и float `sum()` сверены с НАСТОЯЩИМ интерпретатором (журнал сравнений на
случайных входах с равными и убывающими пробегами); отрицательный контроль — чужая версия даёт другой журнал.

Модуль пропускается с названной причиной, пока расширение не собрано (`python tools/native_build.py`) либо нет корпуса.
"""

from __future__ import annotations

import functools
import math
import os
import random
import struct
import sys
import time
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
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка первой трети clip_geometry с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "clip")

from cftuv_native import clip_seams as wire  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.materialize.clip as clip  # noqa: E402
import cftuv_envelope.materialize.clip_cells as clip_cells  # noqa: E402
import cftuv_envelope.materialize.clip_snap as clip_snap  # noqa: E402
import cftuv_envelope.materialize.tessellate as tessellate  # noqa: E402
import cftuv_envelope.wavefront.faces as faces  # noqa: E402
import native_clip_geometry as geometry  # noqa: E402
import native_clip_seams as seams  # noqa: E402
import native_corpus as nc  # noqa: E402
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1  # noqa: E402
from cftuv_envelope.materialize.lift_surface import LiftTriangleV1, SurfaceLiftV1  # noqa: E402

CORPUS_BASE = geometry.corpus_base()
SYNTHETIC_BASE = CORPUS_BASE / "synthetic_clip"
PYTHON_VERSION = (sys.version_info.major, sys.version_info.minor)
#: Сколько вызовов каждого шва из тестов ядра проверяется (остальные прорежены шагом): весь корпус — сотни тысяч вызовов.
KERNEL_SEAM_BUDGET = 3000
#: Каждый какой по счёту полевой вызов проверяется (1 — все); для быстрого прогона на машине без времени.
RECORD_STRIDE = int(os.environ.get("CFTUV_CLIP_SEAM_STRIDE", "1"))

#: Сколько швов проверено по каждому источнику: итог печатается в конце модуля.
CHECKED: Counter = Counter()
TIMINGS: dict = {}


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    """Эталон пишет в процессные счётчики и память ядра; тест их не оставляет."""

    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    with exact.isolated_factorization_memory():
        yield
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(nc._ARTICLES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line("native clip seams compared with the Python oracle: " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items())))
        for source, rows in TIMINGS.items():
            for seam, row in sorted(rows.items()):
                reporter.write_line(f"  {source:9s} {seam:22s} n={row['checked']:6d} oracle {row['oracle_us']:9.2f} us  native compute {row['compute_us']:9.2f} us  x{row['compute_speedup']:.1f}")


def settle(verifier: "seams.Verifier", calls, source: str) -> list:
    found = verifier.verify(calls)
    CHECKED[source] += len(calls) - sum(verifier.unsupported.values())
    return found


def explain(found) -> str:
    return "\n".join(str(item) for item in found[:12]) + (f"\n... and {len(found) - 12} more" if len(found) > 12 else "")


# --------------------------------------------------------------------------
# Эмуляция CPython: сортировка и сумма
# --------------------------------------------------------------------------


def real_sort_log(keys):
    log = []

    def compare(left, right):
        log.append((left, right))
        return (keys[left] > keys[right]) - (keys[left] < keys[right])

    return sorted(range(len(keys)), key=functools.cmp_to_key(compare)), log


def tie_heavy_keys(rng: random.Random, size: int, alphabet: int) -> list:
    mode = rng.random()
    keys = [rng.randrange(alphabet) for _ in range(size)]
    if mode < 0.3:
        keys.sort(reverse=True)
    elif mode < 0.4:
        keys.sort()
    elif mode < 0.55:
        keys = sorted(keys[: size // 2], reverse=True) + sorted(keys[size // 2 :])
    return keys


def test_the_sort_comparison_sequence_equals_the_running_interpreter():
    runner = wire.SeamRunner()
    rng = random.Random(20261006)
    checked = 0
    for size in range(0, 64):
        for alphabet in (1, 2, 3, 5, 1000):
            for _ in range(24):
                keys = tie_heavy_keys(rng, size, alphabet)
                order, log = real_sort_log(keys)
                got = runner.value("SORT_SEQUENCE", [*PYTHON_VERSION, keys])
                assert got == (order, log), f"python {PYTHON_VERSION} keys {keys}: the native comparison sequence differs"
                checked += 1
    CHECKED["sort"] += checked


def test_the_sort_emulation_of_the_other_minor_version_is_a_different_sequence():
    """Отрицательный контроль: чужая версия на входах с равными и убывающими пробегами даёт другой журнал (иначе проверка пуста)."""

    runner = wire.SeamRunner()
    other = (3, 13) if PYTHON_VERSION == (3, 11) else (3, 11)
    rng = random.Random(7)
    different = 0
    for _ in range(400):
        keys = tie_heavy_keys(rng, rng.randrange(4, 30), 3)
        order, log = real_sort_log(keys)
        got = runner.value("SORT_SEQUENCE", [*other, keys])
        assert got[0] == order, "both versions sort the same way; only the questions differ"
        different += got[1] != log
    assert different > 40


def test_a_sort_the_port_does_not_cover_and_an_unknown_interpreter_are_named_refusals():
    runner = wire.SeamRunner()
    long = runner.call("SORT_SEQUENCE", [*PYTHON_VERSION, list(range(64))])
    assert long.unsupported, "a sort of 64 elements needs the merge machinery of list.sort: named refusal, never a guess"
    assert runner.call("SORT_SEQUENCE", [*PYTHON_VERSION, list(range(63))]).ok
    assert runner.call("SORT_SEQUENCE", [3, 12, [3, 1, 2]]).unsupported
    assert runner.call("SORT_SEQUENCE", [2, 7, [3, 1, 2]]).unsupported
    assert runner.call("FLOAT_SUM", [3, 14, [1.0, 2.0]]).unsupported


FLOAT_SPECIALS = (0.0, -0.0, 1.0, -1.0, 1e100, -1e100, 1e-300, 5e-324, math.inf, -math.inf, math.nan, 0.1, 0.2, 0.3, 1e308, -1e308, 1.7976931348623157e308, 3.0, -3.0)


def test_the_float_sum_equals_the_running_interpreter_bit_for_bit():
    runner = wire.SeamRunner()
    rng = random.Random(99)
    for _ in range(12000):
        terms = [
            rng.choice(FLOAT_SPECIALS) if rng.random() < 0.5 else rng.uniform(-10, 10) * 10 ** rng.randrange(-6, 6)
            for _ in range(rng.choice((1, 2, 3, 3, 3, 4, 5)))
        ]
        expected = sum(terms)
        got = runner.value("FLOAT_SUM", [*PYTHON_VERSION, terms])
        assert struct.pack("<d", expected) == struct.pack("<d", got) or (expected != expected and got != got), f"python {PYTHON_VERSION} sum of {terms}"
    CHECKED["float sum"] += 12000


def test_the_float_sum_of_the_other_version_differs_where_python_changed_it():
    runner = wire.SeamRunner()
    other = (3, 13) if PYTHON_VERSION == (3, 11) else (3, 11)
    terms = [0.1, 0.2, 0.3]
    assert struct.pack("<d", sum(terms)) == struct.pack("<d", runner.value("FLOAT_SUM", [*PYTHON_VERSION, terms]))
    assert runner.value("FLOAT_SUM", [*other, terms]) != sum(terms)


def test_the_seam_table_of_the_extension_is_the_table_of_the_harness():
    assert tuple(cftuv_native.clip_seam_table()) == wire.SEAMS


# --------------------------------------------------------------------------
# Корпуса
# --------------------------------------------------------------------------


def field_paths() -> list:
    paths = sorted((CORPUS_BASE / "records").glob("*/*clip_geometry*.rec"))
    return paths[::RECORD_STRIDE]


def synthetic_paths() -> list:
    return sorted((SYNTHETIC_BASE / "records").glob("*/*clip_geometry*.rec"))


def replay_with_seams(path: Path, sampling: "seams.Sampling") -> "seams.SeamRecorder":
    record = nc.read_record(path)
    call = nc.prepare_call(nc.OP_CLIP, record.call_blob, record.before())
    recorder = seams.SeamRecorder(sampling)
    with recorder.installed():
        nc.execute(call)
    return recorder


def verify_records(paths, sampling, source: str) -> "seams.Verifier":
    verifier = seams.Verifier()
    problems = []
    for path in paths:
        recorder = replay_with_seams(path, sampling)
        assert not recorder.errors, f"{path.name}: the seam cannot carry an input of the oracle: {dict(recorder.errors)}"
        found = settle(verifier, recorder.calls, source)
        problems.extend(f"{path.parent.name}/{path.name}: {item}" for item in found)
    assert not problems, explain(problems)
    assert not verifier.unsupported, f"native refusals on corpus inputs: {dict(verifier.unsupported)}"
    return verifier


#: Швы, которые обязаны быть проверены на любом корпусе кусков (иначе «зелёный» пуст).
CORPUS_SEAMS = (
    "BUILD_CELLS", "SNAP_SOURCE_VERTICES", "CHEAP_SIGN", "EDGE_CONSTANTS", "RATIONAL_PAIR", "LINE_VALUE", "WINDOW", "VALUES_IN", "STRETCH_SQUARE",
    "LIFT_KNOWN", "SHOELACE_SIGN", "HAS_RIGHT_TURN", "TRIANGULATE_EXACT", "CHORD_OF", "HINGE_DEPTH_SQUARE", "NANOMETRES", "MILLI_CELLS", "ORDERED",
)


def record_timings(source: str, verifier: "seams.Verifier") -> None:
    TIMINGS[source] = verifier.report()


@pytest.mark.skipif(not CORPUS_BASE.exists(), reason=f"нет полевого корпуса {CORPUS_BASE}: `tools/native_corpus_export.py`")
def test_every_seam_equals_the_oracle_on_the_field_corpus():
    paths = field_paths()
    assert len(paths) >= 100 // RECORD_STRIDE
    verifier = verify_records(paths, seams.Sampling(head=40, stride=80, cap=500), "field")
    record_timings("field", verifier)
    for seam in CORPUS_SEAMS:
        assert verifier.checked[seam] > 0, f"the field corpus never reached {seam}"


@pytest.mark.skipif(not SYNTHETIC_BASE.exists(), reason=f"нет синтетического корпуса {SYNTHETIC_BASE}: `python tools/native_clip_synthetic.py build`")
def test_every_seam_equals_the_oracle_on_the_synthetic_clip_records():
    paths = synthetic_paths()
    assert len(paths) >= 100
    verifier = verify_records(paths, seams.Sampling(head=60, stride=60, cap=400), "synthetic")
    record_timings("synthetic", verifier)
    for seam in ("BUILD_CELLS", "SNAP_SOURCE_VERTICES", "CHEAP_SIGN", "LINE_VALUE", "SHOELACE_SIGN", "TRIANGULATE_EXACT", "DOUBLED_SHOELACE", "HAS_RIGHT_TURN"):
        assert verifier.checked[seam] > 0, f"the synthetic corpus never reached {seam}"


@pytest.mark.skipif(not SYNTHETIC_BASE.exists(), reason=f"нет синтетического корпуса {SYNTHETIC_BASE}: `python tools/native_clip_synthetic.py build`")
def test_every_seam_call_recorded_from_the_kernel_tests_equals_the_oracle():
    import native_clip_synthetic as synthetic

    index = synthetic.load_index(SYNTHETIC_BASE)
    strides = {seam: max(1, -(-total // KERNEL_SEAM_BUDGET)) for seam, total in index["seam_calls"].items()}
    kept: Counter = Counter()
    verifier = seams.Verifier()
    problems = []
    for row in index["seam_files"]:
        calls = []
        for call in synthetic.read_seams(SYNTHETIC_BASE / row["path"]):
            kept[call.seam] += 1
            # хвост горячих швов прорежен шагом; вызов с исключением или ценой проверяется всегда
            if kept[call.seam] % strides[call.seam] == 0 or call.error is not None or (call.pre is not None and seams._spent(call.pre, call.post)):
                calls.append(call)
        for item in settle(verifier, calls, "kernel tests"):
            problems.append(f"{row['test']}: {item}")
    assert not problems, explain(problems)
    assert not verifier.unsupported, f"native refusals on kernel test inputs: {dict(verifier.unsupported)}"
    record_timings("kernel", verifier)
    for seam in ("CONVEX_QUAD_RING", "DOUBLED_SHOELACE", "TRIANGULATE_EXACT", "WITHIN_EDGE_GAP", "BUILD_CELLS", "SNAP_SOURCE_VERTICES", "LIFT_KNOWN", "CHORD_OF"):
        assert verifier.checked[seam] > 0, f"the kernel tests never reached {seam}"


# --------------------------------------------------------------------------
# Целевые случаи: ветки, которых нет ни в одном корпусе
# --------------------------------------------------------------------------

ALL = seams.Sampling(head=10**9, stride=1, cap=10**9)


def hard_sum(shift: int = 90) -> SqrtSumV1:
    """`sqrt(2) + sqrt(3) - r`, `r` — приближение с точностью `2^-shift`: оболочка в 64 бита знака не решает, решает сопряжение."""

    def scaled(radicand: int) -> int:
        return math.isqrt(radicand << (2 * shift))

    approximation = Fraction(scaled(2) + scaled(3) + 1, 1 << shift)
    return SqrtSumV1(((1, -approximation), (2, Fraction(1)), (3, Fraction(1))))


def sum_of(**terms) -> SqrtSumV1:
    """`sum_of(r1=3, r2=Fraction(1, 2))` — `3 + sqrt(2)/2`."""

    return SqrtSumV1(tuple(sorted((int(key[1:]), value if isinstance(value, int) else Fraction(value)) for key, value in terms.items() if value)))


def rational_point(x, y):
    return SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y))


def fresh_budget(cap=None):
    return exact.exact_work_budget(stage="CLIP_PARTS", domain_id="parts", superlevel="L1", cap=cap)


def observe(body, *, only=None):
    """Исполняет `body` под записывающими обёртками, сверяет все записанные вызовы; возвращает `(рекордер, сверщик)`."""

    recorder = seams.SeamRecorder(ALL, only=only)
    with recorder.installed():
        body()
    assert not recorder.errors, f"the seam cannot carry an input: {dict(recorder.errors)}"
    verifier = seams.Verifier()
    found = settle(verifier, recorder.calls, "targeted")
    assert not found, explain(found)
    return recorder, verifier


def conjugations(recorder) -> int:
    return sum(call.post.sign_counts["closed_by_conjugation"] - call.pre.sign_counts["closed_by_conjugation"] for call in recorder.calls if call.pre is not None)


def exhaustions(verifier) -> int:
    return sum(verifier.refused.values())


def attempt_budgeted(call, budget):
    try:
        return call(budget)
    except exact.ExactCanonicalizationWorkBudgetExhausted:
        return None


CAPS = (None, 0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 14, 20)


def collinear_polygon(rng: random.Random, offset: SqrtSumV1 | None):
    """Многоугольник решётки, у которого одна вершина лежит точно на прямой соседей и сдвинута на `offset` (микроскопически)."""

    count = rng.randrange(3, 8)
    points = [(rng.randrange(0, 7), rng.randrange(0, 7)) for _ in range(count)]
    middle = rng.randrange(count)
    before, after = points[middle - 1], points[(middle + 1) % count]
    x, y = before[0] + after[0], before[1] + after[1]
    if x % 2 == 0 and y % 2 == 0:
        points[middle] = (x // 2, y // 2)
    out = [rational_point(*item) for item in points]
    if offset is not None:
        out[middle] = (out[middle][0], out[middle][1] + offset)
    return out


def test_the_orientation_family_asks_the_same_exact_signs_with_a_budget_a_sweep_of_caps_and_none():
    hard = hard_sum()
    rng = random.Random(5)
    polygons = [collinear_polygon(rng, hard if index % 3 else None) for index in range(45)]
    near = [rational_point(0, 0), rational_point(1, 0), (SqrtSumV1.zero(), hard)]
    quad = [rational_point(0, 0), rational_point(2, 0), (SqrtSumV1.rational(2), SqrtSumV1.rational(2) + hard), rational_point(0, 2)]
    short = [rational_point(0, 0), rational_point(1, 1)]
    far = [rational_point(0, 0), rational_point(1, 0), (SqrtSumV1(((1, Fraction(10**400)),)), SqrtSumV1.rational(1)), rational_point(0, 1)]

    def body():
        for cap in CAPS:
            for polygon in polygons + [near, quad, short, far]:
                with exact.isolated_factorization_memory():
                    budget = fresh_budget(cap)
                    count = len(polygon)
                    attempt_budgeted(lambda b: faces.shoelace_sign(polygon, b), budget)
                    attempt_budgeted(lambda b: tessellate.has_right_turn(polygon, range(count), b), budget)
                    attempt_budgeted(lambda b: tessellate.triangulate_exact(polygon, b), budget)
                    attempt_budgeted(lambda b: tessellate.convex_quad_ring(polygon, b), budget)
                    if count >= 3:
                        attempt_budgeted(lambda b: faces.orientation(polygon[0], polygon[1], polygon[2], b), budget)
        for polygon in polygons[:12] + [near, quad]:
            faces.orientation(polygon[0], polygon[1], polygon[2], None)
            tessellate.triangulate_exact(polygon, None)

    recorder, verifier = observe(body)
    assert conjugations(recorder) > 0, "no sign needed a conjugation: the exact path is untested"
    assert exhaustions(verifier) > 0, "no call ran out of budget: the partial state is untested"
    assert verifier.checked["ORIENTATION"] > 100 and verifier.checked["TRIANGULATE_EXACT"] > 100
    assert verifier.checked["CONVEX_QUAD_RING"] > 30
    assert any(call.pre.budget is None for call in recorder.calls if call.pre is not None), "the UNBUDGETED path"


def random_sum(rng: random.Random, py_int: bool = False) -> SqrtSumV1:
    radicands = rng.sample((1, 2, 3, 5, 6, 7, 10, 11), rng.randrange(0, 4))
    terms = []
    for radicand in sorted(radicands):
        value = rng.randrange(-9, 10) or 1
        coefficient = value if py_int and rng.random() < 0.6 else Fraction(value, rng.randrange(1, 6))
        terms.append((radicand, coefficient))
    return SqrtSumV1(tuple(terms))


def test_doubled_shoelace_equals_the_oracle_for_every_size_with_int_and_fraction_coefficients():
    rng = random.Random(11)

    def body():
        for size in (0, 1, 2, 3, 3, 4, 5, 6, 9):
            for _ in range(12):
                polygon = [(random_sum(rng, py_int=True), random_sum(rng, py_int=True)) for _ in range(size)]
                faces.doubled_shoelace(polygon)
        for _ in range(10):
            polygon = [rational_point(rng.randrange(-5, 6), rng.randrange(-5, 6)) for _ in range(rng.randrange(3, 8))]
            with exact.isolated_factorization_memory():
                attempt_budgeted(lambda b: faces.shoelace_sign(polygon, b), fresh_budget(None))

    _recorder, verifier = observe(body)
    assert verifier.checked["DOUBLED_SHOELACE"] >= 100


def test_the_float_window_and_the_lift_name_their_overflows_and_zero_divisions():
    huge = SqrtSumV1(((1, Fraction(10**400)),))
    near_max = SqrtSumV1(((1, Fraction(2**1024 - 2**970)),))
    tiny = SqrtSumV1(((1, Fraction(1, 10**400)),))
    plane = SurfaceLiftV1.from_triangles(
        [("t0", ((0, 0), (4, 0), (4, 4)), ((0, 0, 0), (1, 0, 0), (1, 1, 1)))], scale=4
    ).bind(fresh_budget())
    triangle = plane.triangles[0]
    sloped = LiftTriangleV1("slope", triangle.chart, triangle.corners, Fraction(1, 10**400), triangle.box, normals=((0.0, 0.0, 1.0),) * 3, face="")

    def body():
        for x, y in ((huge, tiny), (tiny, huge), (near_max, near_max), (tiny, tiny), (SqrtSumV1.zero(), SqrtSumV1.zero()), (hard_sum(), sum_of(r2=1, r3=1))):
            try:
                plane.window((x, y))
            except OverflowError:
                pass
        values = plane.values_in(triangle, rational_point(1, 1))
        for bad in (huge, tiny):
            try:
                plane.lift_known(triangle, [bad, values[1], values[2]])
            except OverflowError:
                pass
        try:
            plane.lift_known(sloped, values)
        except (ZeroDivisionError, OverflowError):
            pass

    _recorder, verifier = observe(body)
    assert verifier.refused["WINDOW"] >= 3, "the oracle must have raised OverflowError from the window"
    assert verifier.refused["LIFT_KNOWN"] >= 1


def test_lift_known_equals_the_oracle_with_normals_and_names_a_zero_blend():
    normals_ok = ((0.0, 0.0, 1.0), (0.0, 0.6, 0.8), (0.6, 0.0, 0.8))
    normals_flat = ((0.0, 0.0, 1.0), (0.0, 0.0, -1.0), (0.0, 0.0, 0.0))
    items = [
        ("a", ((0, 0), (8, 0), (8, 8)), ((0, 0, 0), (1, 0, 0.1), (1, 1, 0.3)), normals_ok, "F"),
        ("b", ((0, 0), (8, 8), (0, 8)), ((0, 0, 0), (1, 1, 0.3), (0, 1, 0.2)), normals_flat, "F"),
        ("c", ((0, 0), (8, 8), (-8, 4)), ((0, 0, 0), (1, 1, 0.3), (-1, 0.5, 0.1)), (), ""),
    ]
    plane = SurfaceLiftV1.from_triangles(items, scale=8).bind(fresh_budget())
    rng = random.Random(3)

    def body():
        for triangle in plane.triangles:
            for _ in range(40):
                point = (
                    sum_of(r1=Fraction(rng.randrange(0, 8)), r2=Fraction(rng.randrange(-2, 3), 7)),
                    sum_of(r1=Fraction(rng.randrange(0, 8, 1), 2), r3=Fraction(rng.randrange(-2, 3), 5)),
                )
                values = plane.values_in(triangle, point)
                try:
                    plane.lift_known(triangle, values)
                except Exception:  # noqa: BLE001 - отказ эталона (нулевая смесь) тоже исход
                    pass
        # середина ребра первых двух углов: веса `0.5, 0.5, 0` и нормали `(0,0,1), (0,0,-1)` смешиваются в нуль
        weights_zero = plane.values_in(plane.triangles[1], rational_point(4, 4))
        try:
            plane.lift_known(plane.triangles[1], weights_zero)
        except Exception:  # noqa: BLE001
            pass

    recorder, verifier = observe(body)
    assert verifier.checked["LIFT_KNOWN"] >= 100
    assert any(call.seam == "LIFT_KNOWN" and call.error is not None and call.error[0] == "MaterializationRefusal" for call in recorder.calls), "the zero blend was not reached"
    assert any(call.seam == "LIFT_KNOWN" and call.error is None and call.result[1][1] is not None for call in recorder.calls), "no lift carried an offset normal"


def test_charts_with_fractions_negative_and_large_coordinates_equal_the_oracle():
    charts = [
        ((Fraction(1, 2), Fraction(0)), (Fraction(7, 3), Fraction(1, 5)), (Fraction(-1, 4), Fraction(9, 2))),
        ((0, 0), (4, 0), (0, 4)),
        ((-3, -2), (5, -7), (2, 9)),
        ((2**45, 0), (2**45 + 3, 2**41), (2**45 - 7, 5)),
    ]
    items = [(f"t{index}", chart, ((0, 0, 0), (1, 0, 0), (0, 1, 0.5))) for index, chart in enumerate(charts)]
    plane = SurfaceLiftV1.from_triangles(items, scale=1).bind(fresh_budget())
    rng = random.Random(8)

    def body():
        for triangle in plane.triangles:
            for _ in range(30):
                point = (random_sum(rng), random_sum(rng))
                for index in range(3):
                    plane.line_value(triangle, index, point)
                plane.values_in(triangle, point)
                plane.window(point)
            plane.stretch_square(triangle)
            plane.stretch_square(triangle)

    _recorder, verifier = observe(body)
    assert verifier.checked["LINE_VALUE"] >= 300 and verifier.checked["STRETCH_SQUARE"] >= 8


def test_edge_constants_and_the_cheap_sign_equal_the_oracle_at_every_branch():
    charts = [
        ((0, 0), (4, 0), (0, 4)),
        ((-5, 3), (11, -2), (0, 7)),
        ((2**39, 0), (2**39 + 5, 2**39), (3, 3)),
        ((2**40, 0), (2**40 + 5, 3), (3, 3)),
        ((Fraction(1, 2), 0), (4, 0), (0, 4)),
    ]
    items = [(f"t{index}", chart, ((0, 0, 0), (1, 0, 0), (0, 1, 0))) for index, chart in enumerate(charts)]
    plane = SurfaceLiftV1.from_triangles(items, scale=1).bind(fresh_budget())
    rng = random.Random(21)
    huge_rational = (SqrtSumV1.rational(Fraction(10**400, 3)), SqrtSumV1.rational(Fraction(7, 10**390)))
    points = [
        rational_point(0, 0),
        rational_point(1, 1),
        rational_point(Fraction(1, 3), Fraction(-5, 7)),
        rational_point(2**39, 5),
        rational_point(-9, 2**45),
        huge_rational,
        (sum_of(r2=1), sum_of(r3=1)),
        (hard_sum(), SqrtSumV1.zero()),
        (SqrtSumV1.zero(), hard_sum()),
        (SqrtSumV1(((1, Fraction(10**400)), (2, Fraction(1)))), sum_of(r1=1)),
        (sum_of(r1=2, r2=Fraction(1, 1000)), sum_of(r1=2, r2=Fraction(1, 1000))),
    ] + [(random_sum(rng), random_sum(rng)) for _ in range(40)]

    def body():
        stage = clip.ClipStageV1(plane, fresh_budget(), {})
        for ti, item in enumerate(stage.regions):
            for index in range(len(item.chart)):
                constants = stage._edge_constants(ti, index)
                stage._edge_constants(ti, index)
                if constants is None:
                    continue
                for point in points:
                    node = stage._node(point)
                    for watch in (False, True):
                        stage._cheap_sign(node, constants, watch)

    recorder, verifier = observe(body)
    assert verifier.checked["EDGE_CONSTANTS"] >= 10 and verifier.checked["CHEAP_SIGN"] >= 200
    constants = [call.result for call in recorder.calls if call.seam == "EDGE_CONSTANTS"]
    assert any(item is None for item in constants) and any(item is not None for item in constants)
    outcomes = {call.result for call in recorder.calls if call.seam == "CHEAP_SIGN"}
    assert any(sign is None for sign, _far in outcomes) and any(far for _sign, far in outcomes) and any(sign == 0 for sign, _far in outcomes)


def test_the_integer_roots_of_the_records_equal_the_oracle_and_a_negative_square_is_a_value_error():
    values = [Fraction(0), Fraction(1), Fraction(1, 3), Fraction(2), Fraction(10**7, 3), Fraction(10**30), Fraction(1, 10**20), Fraction(-1, 3), Fraction(-4)]

    def body():
        for value in values:
            for call in (clip_cells.nanometres, clip_snap.milli_cells):
                try:
                    call(value)
                except ValueError:
                    pass
        for point in (rational_point(1, 2), (sum_of(r2=1), sum_of(r2=1)), (SqrtSumV1.zero(), SqrtSumV1.zero())):
            clip._rational_pair(point)
        clip._rational_pair((SqrtSumV1(((1, 3),)), SqrtSumV1(((1, 5),))))

    _recorder, verifier = observe(body)
    assert verifier.refused["NANOMETRES"] == 2 and verifier.refused["MILLI_CELLS"] == 2
    assert verifier.checked["RATIONAL_PAIR"] == 4


def lattice_items(prefix: str, offset: int, face: str, triangles, height=lambda x, y: 0):
    """`(имя, карта, 3D)` треугольников грани: 3D — плоскость решётки в сотых долях метра плюс высота `height(x, y)`."""

    items = []
    for number, chart in enumerate(triangles):
        corners = tuple((Fraction(x + offset, 100), Fraction(y, 100), Fraction(height(x, y))) for x, y in chart)
        items.append((f"{prefix}{number}", tuple((x + offset, y) for x, y in chart), corners, (), face))
    return items


FACE_CONFIGURATIONS = {
    # четырёхгранье из двух треугольников: излом по диагонали (высота непланарна)
    "hinge": ([((0, 0), (4, 0), (4, 4)), ((0, 0), (4, 4), (0, 4))], lambda x, y: Fraction(x * y, 700)),
    # тот же квадрат, точно планарный: излома нет, `jump_square = 0`
    "plane": ([((0, 0), (4, 0), (4, 4)), ((0, 0), (4, 4), (0, 4))], lambda x, y: 0),
    # три треугольника с прямой вершиной (4, 0) на нижнем ребре прямоугольника
    "straight": ([((0, 0), (4, 0), (0, 6)), ((4, 0), (8, 0), (8, 6)), ((4, 0), (8, 6), (0, 6))], lambda x, y: Fraction(x + y, 900)),
    # Г-образная грань: веер от (0, 0), поворот против обхода в (4, 4)
    "ell": ([((0, 0), (8, 0), (8, 4)), ((0, 0), (8, 4), (4, 4)), ((0, 0), (4, 4), (4, 8)), ((0, 0), (4, 8), (0, 8))], lambda x, y: Fraction(x * x, 2000)),
    # два треугольника одной грани с общим ребром в ОДНОМ направлении (наложение): `_loop_of` отказывает на повторном полуребре
    "overlap": ([((0, 0), (4, 0), (0, 4)), ((0, 0), (4, 0), (2, 2))], lambda x, y: 0),
    # два треугольника одной грани, касающиеся в вершине: из неё выходят два непарных ребра
    "touch": ([((0, 0), (4, 0), (0, 4)), ((0, 0), (0, -4), (4, -4))], lambda x, y: 0),
    # противоположные обходы одной грани (второй треугольник по часовой стрелке)
    "mixed": ([((0, 0), (4, 0), (0, 4)), ((9, 9), (9, 13), (13, 9))], lambda x, y: 0),
    # два далёких треугольника одной грани: не одна петля
    "apart": ([((0, 0), (4, 0), (0, 4)), ((9, 9), (13, 9), (9, 13))], lambda x, y: 0),
    # треугольник без грани и треугольник со своей гранью
    "single": ([((0, 0), (4, 0), (0, 4))], lambda x, y: 0),
}


def face_plane(selection) -> "SurfaceLiftV1":
    items = []
    for number, name in enumerate(selection):
        triangles, height = FACE_CONFIGURATIONS[name]
        face = "" if name == "single" and number % 2 == 0 else f"face-{name}-{number}"
        items.extend(lattice_items(f"{number:02d}{name}-", 40 * number, face, triangles, height))
    return SurfaceLiftV1.from_triangles(items, scale=100)


def test_build_cells_equals_the_oracle_for_every_cell_kind_split_and_memo():
    selections = [
        ("hinge",), ("plane",), ("straight",), ("ell",), ("mixed",), ("apart",), ("single", "single"),
        ("overlap",), ("touch",),
        ("hinge", "straight", "ell", "mixed", "apart", "single", "plane", "overlap", "touch"),
        ("ell", "ell", "hinge"),
    ]
    results = []

    def body():
        for selection in selections:
            lift = face_plane(selection)
            memo: dict = {}
            plan = clip_cells.build_cells(lift.triangles, memo=memo)
            results.append(plan)
            keys = [cell.key for cell in plan.cells if cell.key[0] in ("f", "g")] + [cell.group for cell in plan.cells if cell.group is not None]
            for size in range(len(keys) + 1):
                clip_cells.build_cells(lift.triangles, frozenset(keys[:size]), memo)
                clip_cells.build_cells(lift.triangles, frozenset(keys[size:]))
            clip_cells.build_cells(lift.triangles, frozenset(keys), {})
            clip_cells.build_cells(lift.triangles)

    _recorder, verifier = observe(body)
    reasons = {reason for plan in results for _face, reason in plan.unmergeable}
    assert {"MIXED_WINDING", "NOT_ONE_LOOP"} <= reasons, reasons
    cells = [cell for plan in results for cell in plan.cells]
    assert any(cell.hinge is not None for cell in cells), "no two-triangle cell with a hinge"
    assert any(cell.flat_square is not None and len(cell.members) > 2 for cell in cells), "no merged cell of three triangles"
    assert any(cell.straight for cell in cells), "no cell with a straight vertex"
    assert any(cell.group is not None for cell in cells), "no group of a non-convex face"
    assert any(cell.hinge is not None and cell.hinge.jump_square == 0 for cell in cells), "no planar two-triangle cell"
    assert verifier.checked["BUILD_CELLS"] >= 80


def plan_pair_sets(lift, rng: random.Random) -> list:
    """Наборы пар плана станций цепей на гранях подъёма: пары любых двух граней (соседних и нет), цепочки, чужая грань, пара не из двух имён."""

    faces = sorted({item.face for item in lift.triangles if item.face})
    sets = [frozenset()]
    if len(faces) >= 2:
        sets.append(frozenset({frozenset(faces[:2])}))
        sets.append(frozenset(frozenset(pair) for pair in zip(faces, faces[1:])))
        sets.extend(frozenset(frozenset(rng.sample(faces, 2)) for _ in range(rng.choice((1, 2, 3, 5)))) for _ in range(6))
    if faces:
        sets.append(frozenset({frozenset((faces[0], "elsewhere"))}))
        sets.append(frozenset({frozenset((faces[-1],)), frozenset((faces[0], faces[-1], "elsewhere"))}))
    return sets


def test_build_cells_with_the_pairs_of_the_chain_station_plan_equals_the_oracle():
    """`inert` (`CHAIN_STATION_PLAN_V1`): группы `("p", имя)`, оценка группы, порядок записей памяти, расщепление группы, пары чужих и негодных граней."""

    selections = [
        ("plane", "plane"), ("hinge", "plane", "straight"), ("ell", "plane"), ("ell", "ell", "hinge"), ("mixed", "plane", "apart", "overlap"),
        ("single", "single", "plane"), ("touch", "plane", "hinge"),
        ("hinge", "straight", "ell", "mixed", "apart", "single", "plane", "overlap", "touch"),
    ]
    rng = random.Random(20261006)
    results = []

    def body():
        for selection in selections:
            lift = face_plane(selection)
            for inert in plan_pair_sets(lift, rng):
                memo: dict = {}
                plan = clip_cells.build_cells(lift.triangles, memo=memo, inert=inert)
                results.append(plan)
                keys = list(dict.fromkeys([cell.key for cell in plan.cells if cell.key[0] in ("f", "g")] + [cell.group for cell in plan.cells if cell.group is not None]))
                for size in range(len(keys) + 1):
                    clip_cells.build_cells(lift.triangles, frozenset(keys[:size]), memo, inert)
                    clip_cells.build_cells(lift.triangles, frozenset(keys[size:]), None, inert)
                clip_cells.build_cells(lift.triangles, frozenset(keys), {}, inert)

    _recorder, verifier = observe(body)
    planned = [plan for plan in results if plan.plan_pairs]
    assert planned, "no pair of the plan glued two faces"
    groups = {cell.group for plan in planned for cell in plan.cells if cell.group is not None and cell.group[0] == "p"}
    assert groups, "no group of the plan"
    flats = {cell.flat_square for plan in planned for cell in plan.cells if cell.group is not None and cell.group[0] == "p"}
    assert Fraction(0) in flats and any(flat > 0 for flat in flats), "the estimate of a plan group is zero or the largest of its non-convex faces"
    assert verifier.checked["BUILD_CELLS"] >= 400


def square_lift():
    """Квадрат 400x400 ячеек, диагональ `(0,0)-(400,400)`, 3D `z = 0`, ячейка = 0.01 м (как у `test_clip_snap`)."""

    def top(x, y):
        return (Fraction(x, 100), Fraction(y, 100), Fraction(0))

    return SurfaceLiftV1.from_triangles(
        [
            ("t0", ((0, 0), (400, 0), (400, 400)), (top(0, 0), top(400, 0), top(400, 400))),
            ("t1", ((0, 0), (400, 400), (0, 400)), (top(0, 0), top(400, 400), top(0, 400))),
        ],
        scale=100,
    )


def test_snap_source_vertices_equals_the_oracle_at_every_branch_with_exact_signs_and_a_budget_sweep():
    hard = hard_sum()
    close = SurfaceLiftV1.from_triangles(
        [
            ("a", ((0, 0), (6, 0), (0, 6)), tuple((Fraction(x, 100), Fraction(y, 100), Fraction(0)) for x, y in ((0, 0), (6, 0), (0, 6)))),
            ("b", ((6, 0), (6, 6), (0, 6)), tuple((Fraction(x, 100), Fraction(y, 100), Fraction(0)) for x, y in ((6, 0), (6, 6), (0, 6)))),
        ],
        scale=100,
    )
    cases = [
        {"src:a": rational_point(397, 1), "node:b": rational_point(100, 300)},
        {"src:a": rational_point(395, 0), "src:b": rational_point(404, -3)},
        {"src:edge": rational_point(396, 0), "src:corner": rational_point(0, 400)},
        {"src:a": rational_point(397, 1), "src:b": rational_point(399, 2)},
        {"src:a": rational_point(397, 1), "node:b": rational_point(400, 0)},
        {"src:a": (SqrtSumV1.rational(396) + hard, SqrtSumV1.zero())},
        {"src:a": (SqrtSumV1.rational(396) - hard, SqrtSumV1.zero()), "src:b": (SqrtSumV1.rational(3) + hard, SqrtSumV1.rational(399))},
        {"src:a": (sum_of(r1=Fraction(397), r2=Fraction(1, 100)), sum_of(r1=1, r3=Fraction(1, 50)))},
        {"node:n": rational_point(5, 5)},
        {},
    ]
    close_cases = [{"src:m": rational_point(3, 1)}, {"src:m": (sum_of(r1=3, r2=Fraction(1, 7)), sum_of(r1=1))}, {"src:m": rational_point(30, 30)}]

    def body():
        for plane_lift, group in ((square_lift(), cases), (close, close_cases)):
            for points in group:
                for cap in (None, 0, 1, 2, 3, 5, 9):
                    with exact.isolated_factorization_memory():
                        budget = fresh_budget(cap)
                        plane = plane_lift.bind(budget)
                        attempt_budgeted(lambda b: clip_snap.snap_source_vertices(plane, b, points), budget)
                with exact.isolated_factorization_memory():
                    clip_snap.snap_source_vertices(plane_lift.bind(fresh_budget()), None, points)

    recorder, verifier = observe(body)
    snaps = [call for call in recorder.calls if call.seam == "SNAP_SOURCE_VERTICES" and call.error is None]
    counters = [dict(call.result.counters) for call in snaps]
    assert any(item[clip_snap.SOURCE_VERTICES_SNAPPED] for item in counters), "no vertex snapped"
    assert any(item[clip_snap.SOURCE_VERTEX_SNAP_REFUSED_TAKEN] for item in counters), "no snap was refused: corner taken"
    assert any(item[clip_snap.SOURCE_VERTEX_SNAP_REFUSED_AMBIGUOUS] for item in counters), "no snap was refused: two corners in the gap"
    assert conjugations(recorder) > 0 and exhaustions(verifier) > 0


def test_within_edge_gap_chord_of_and_the_hinge_depth_equal_the_oracle_with_conjugation_and_budget_caps():
    hard = hard_sum()
    values = [SqrtSumV1.zero(), SqrtSumV1.rational(Fraction(1, 2)), SqrtSumV1.rational(-5), sum_of(r2=1), hard, -hard, sum_of(r1=3, r2=Fraction(-1, 5)), hard + SqrtSumV1.rational(Fraction(1, 2**80))]
    squares = [Fraction(1), Fraction(2), Fraction(1, 2**100), Fraction(1, 2**170), Fraction(25), Fraction(1, 3), Fraction(10**6)]
    plane = face_plane(("hinge", "straight", "ell"))
    cells = clip_cells.build_cells(plane.triangles).cells
    with_hinge = [cell for cell in cells if cell.hinge is not None]
    flat = [cell for cell in cells if cell.hinge is None and cell.flat_square is not None]
    singles = [cell for cell in cells if len(cell.members) == 1]
    columns = [[hard, -hard, SqrtSumV1.rational(1)], [SqrtSumV1.rational(2), sum_of(r2=1)], [-hard, SqrtSumV1.rational(-3)], [SqrtSumV1.zero(), SqrtSumV1.zero()]]

    def body():
        for cap in CAPS:
            for value in values:
                for square in squares:
                    with exact.isolated_factorization_memory():
                        budget = fresh_budget(cap)
                        attempt_budgeted(lambda b: clip_snap.within_edge_gap(value, square, b), budget)
            for cell in with_hinge + flat + singles:
                diagonals = max(len(cell.diagonals), 1)
                for number in range(4):
                    pattern = [columns[(number + step) % 4] for step in range(diagonals)]
                    with exact.isolated_factorization_memory():
                        budget = fresh_budget(cap)
                        attempt_budgeted(lambda b: clip_cells.chord_of(cell, pattern, b), budget)
        for jump in (Fraction(0), Fraction(3, 7)):
            for column in columns + [[hard], [SqrtSumV1.rational(1)], [SqrtSumV1.rational(-1)]]:
                clip_cells.hinge_depth_square(jump, column)

    recorder, verifier = observe(body)
    assert conjugations(recorder) > 0 and exhaustions(verifier) > 0
    assert verifier.checked["WITHIN_EDGE_GAP"] > 400 and verifier.checked["CHORD_OF"] > 100 and verifier.checked["HINGE_DEPTH_SQUARE"] > 50
    assert any(call.seam == "WITHIN_EDGE_GAP" and call.error is None and call.result[0] for call in recorder.calls)
    assert any(call.seam == "WITHIN_EDGE_GAP" and call.error is None and not call.result[0] for call in recorder.calls)


def test_ordered_equals_the_oracle_for_every_tie_axis_budget_and_size_and_names_the_sort_it_does_not_cover():
    rng = random.Random(17)
    plane = square_lift().bind(fresh_budget())
    hard = hard_sum()
    abscissas = [SqrtSumV1.rational(0), SqrtSumV1.rational(1), sum_of(r2=1), sum_of(r2=1, r3=Fraction(1, 3)), hard, SqrtSumV1.rational(Fraction(1, 2)), SqrtSumV1.rational(2)]
    ordinates = [SqrtSumV1.rational(value) for value in range(12)] + [sum_of(r3=1), hard]

    def body():
        stage = clip.ClipStageV1(plane, fresh_budget(), {})
        for size in list(range(0, 24)) + [40, 62, 63, 64, 70]:
            for repeat in range(6):
                along_x = repeat % 2 == 0
                first = stage._node((SqrtSumV1.rational(-1), SqrtSumV1.rational(-1)))
                second = stage._node((SqrtSumV1.rational(9), SqrtSumV1.rational(-1)) if along_x else (SqrtSumV1.rational(-1), SqrtSumV1.rational(9)))
                nodes = []
                while len(nodes) < size:
                    x, y = rng.choice(abscissas), rng.choice(ordinates)
                    node = stage._node((x, y) if along_x else (y, x))
                    if node not in nodes:
                        nodes.append(node)
                for cap in (None, None, 2, 5):
                    with exact.isolated_factorization_memory():
                        stage.budget = fresh_budget(cap)
                        try:
                            stage._ordered(first, second, list(nodes))
                        except exact.ExactCanonicalizationWorkBudgetExhausted:
                            pass

    recorder, verifier = observe(body)
    assert verifier.checked["ORDERED"] > 300
    assert verifier.unsupported["ORDERED"] > 0, "a sort of 64 or more elements must have been refused by name"
    permutations = [call.result for call in recorder.calls if call.seam == "ORDERED" and call.error is None and len(call.result) > 3]
    assert any(item != sorted(item) for item in permutations), "every sort was already in order: the test checks nothing"
