"""Отказ ПОРТА целой операции оставляет ВСЁ видимое из Python состояние, как оно было до вызова: продукт после него идёт в эталон на тех же бюджете, плоскости и таблицах.

Продукт (`backend.py` Python-сессии) зовёт `cftuv_native.clip_geometry` / `coverage_at` и на именованный отказ порта (`cftuv_native.NATIVE_REFUSALS`) откатывается на
эталон `clip.clip_geometry` / `coverage._coverage_at` с ТЕМИ ЖЕ бюджетом, плоскостью и таблицами памяти. Поздний отказ (многоугольник без вершин, ограда
сканирования углов, внутреннее состояние) возникает после того, как работа посчитана; если бы его эффекты уже лежали в объектах хоста, эталон платил бы второй раз и мог
упереться в потолок. Исход самого ЭТАЛОНА (`MaterializationRefusal`, `ExactCanonicalizationWorkBudgetExhausted`, `OverflowError`, `ValueError`, ...) оставляет
частичные эффекты как раз так, как их оставляет его исключение — это проверяют `test_native_clip_dropin.py` и `test_native_coverage.py`, здесь он лишь не теряется.

Для каждого вида отказа порта: снимок ВСЕГО состояния до вызова (статьи и потолок бюджета, четыре таблицы памяти с порядком и множество простых, `SIGN_COUNTS`,
`UNBUDGETED_WORK`, `plane._normal_by_position`, запись `store`, список `traces`), отказ, снимок после — равен до (`native_corpus.compare_outcomes` плюс каноническая запись
всего состояния), затем (а) эталон на ЭТОМ состоянии: исход и состояние после равны чистому прогону эталона от снимка; (б) тот же нативный вызов без отказа на ТОМ ЖЕ состоянии:
равен чистому прогону эталона (зеркало памяти сессии после отказа согласовано с таблицами).

Отказы: настоящий вход — многоугольник без вершин (порт отказывается `NativePortUnsupported`, а эталон поднимает `ValueError` с частичными эффектами: исход эталона
не отказ порта) на бюджете, на бюджете с конечным потолком и без бюджета; и ручка `force_refusal` (нативная сессия досчитывает операцию ВЦЕЛО — статьи, счётчики, журнал
памяти, запись нормалей, запись `store`, `traces` — и лишь потом отказывается): `unsupported`, `invalid_input`, `internal`, `diverged`, `panic`. Ранние отказы
(`NativePortStale`, `NativeUnsupportedPython`, плоскость без таблицы нормалей, вход, который расширение не несёт) — то же состояние до и после. Таблица статусов-исходов
эталона у шима и у расширения одна.

Модуль пропускается с названной причиной, пока расширение не собрано либо нативные `clip` и `coverage` не сверены с этим деревом ядра (`native_gate`).
"""

from __future__ import annotations

import dataclasses
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
    import cftuv_native  # noqa: F401
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (проверка отказов порта пропущена)",
        allow_module_level=True,
    )

from native_gate import skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "clip")
skip_unless_available(cftuv_native, "coverage")

import native_clip_generated as gen  # noqa: E402
import native_clip_geometry as geometry  # noqa: E402
import native_corpus as nc  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.materialize.clip as clip  # noqa: E402
import cftuv_envelope.wavefront.coverage as coverage_module  # noqa: E402
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1  # noqa: E402
from cftuv_envelope.wavefront import build_skeleton  # noqa: E402
from cftuv_envelope.wavefront.faces import build_faces  # noqa: E402
from cftuv_native import cost  # noqa: E402


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    """Эталон и нативная сторона пишут в процессные счётчики и память ядра; тест их не оставляет."""

    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    with exact.isolated_factorization_memory():
        yield
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(nc._ARTICLES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)


@pytest.fixture(scope="module")
def mirror():
    """Одна сессия на весь модуль: после каждого отказа зеркало обязано быть согласовано с таблицами, какими бы они ни стали."""

    return cftuv_native.new_mirror()


# --------------------------------------------------------------------------
# Что значит «состояние не тронуто»
# --------------------------------------------------------------------------


def assert_untouched(op: str, before: "nc.StateV1", observed_before: dict, call, label: str, traces=None) -> None:
    """Всё видимое из Python — как до вызова: статьи и потолок бюджета, четыре таблицы с порядком, простые, счётчики, неоплаченное, нормали плоскости, `store`, `traces`."""

    after = nc.capture_state(call.budget, call.store)  # also refuses a prime list that disagrees with the prime set
    observed = nc.observe(call)
    found = nc.compare_outcomes(op, before, nc.Outcome(None, None, before, observed_before), nc.Outcome(None, None, after, observed))
    assert not found, f"{label}: " + "; ".join(str(item) for item in found[:4])
    assert nc.canonical(after.as_payload()) == nc.canonical(before.as_payload()), f"{label}: the exact canonical record of the state differs"
    assert observed == observed_before, f"{label}: the offset normals of the plane differ"
    assert exact._KNOWN_PRIME_SET == set(before.known_primes), f"{label}: the set of known primes differs"
    if traces is not None:
        assert traces == [], f"{label}: the traces were written"


def assert_state_moved(before: "nc.StateV1", outcome: "nc.Outcome", label: str) -> None:
    """Не пустая проверка: чистый прогон эталона на этом вызове СДВИГАЕТ состояние (есть что не пускать в хост)."""

    assert nc.canonical(outcome.after.as_payload()) != nc.canonical(before.as_payload()), f"{label}: the oracle run changes nothing, so a refusal has nothing to leave untouched"


# --------------------------------------------------------------------------
# Вызовы резки: собранные в коде, без полевого корпуса
# --------------------------------------------------------------------------


def _root(radicand: int, coefficient: Fraction) -> SqrtSumV1:
    return SqrtSumV1(((radicand, Fraction(coefficient)),))


def _rational(value) -> SqrtSumV1:
    return SqrtSumV1.rational(Fraction(value))


def diagonal_call(variant: int, *, by_faces: bool, law: int, empty_polygon: bool = False):
    """`(подъём, kwargs)`: треугольник с вершинами `q * sqrt(m)` через диагональ квадрата с нормалями смещения.

    Вершины среза на диагонали лежат на иррациональных координатах (настоящая работа точного слоя: статьи, факторизации, расщепления) и несут нормали смещения
    (запись в `plane._normal_by_position`). `empty_polygon` дописывает многоугольник без вершин ПОСЛЕ этого треугольника."""

    first, second, third = ((2, 3, 5), (3, 5, 6), (2, 7, 3), (5, 2, 7))[variant % 4]
    shift = Fraction(1 + variant % 3, 7 + variant)
    vertices = [(_rational(1) + _root(first, shift), _rational(5)), (_rational(7), _rational(2) + _root(second, shift)), (_rational(2), _rational(6) + _root(third, shift))]
    keys = [f"node:{index}" for index in range(len(vertices))]
    points = dict(zip(keys, vertices))
    polygons = [(tuple(keys),)]
    if empty_polygon:
        polygons.append(((),))
    lift = gen.lift_square(height=lambda x, y: Fraction(x + y, 80), normals=True)
    kwargs = {
        "points": points,
        "cycles": [[(key, points[key]) for key in keys]],
        "polygons": polygons,
        "law": gen.LAWS[law],
        "seam": frozenset(),
        "fans": None,
        "flows": None,
        "by_faces": by_faces,
    }
    return lift, kwargs


#: `(variant, by_faces, law index)`: four calls that the oracle cuts to the end, writing two offset normals, paying exact work and growing the memory.
SPECS = ((0, False, 0), (1, True, 2), (2, False, 2), (3, True, 0))


def generated_call(seed: int):
    """Сгенерированный вызов на других подъёмах (для сквозной сверки сессии после отказов)."""

    rng = random.Random(seed)
    names = sorted(gen.PLANES)
    lift = gen.PLANES[names[seed % len(names)]]()
    kwargs = gen.random_call(rng, lift, seed, noise=0.9, int_coefficients=False, outside=0.0)
    return None if kwargs is None else (lift, gen.with_plan(f"refusal-{seed}", kwargs, lift))


@dataclasses.dataclass
class ClipCase:
    label: str
    blob: bytes
    before: "nc.StateV1"

    def fresh(self):
        """Состояние процесса — снимок «до», вход собран заново."""

        return nc.prepare_call(nc.OP_CLIP, self.blob, self.before)

    def pure_oracle(self) -> "nc.Outcome":
        return nc.execute(self.fresh())


def make_clip_case(spec: tuple, *, empty_polygon: bool = False, cap=None, budgeted: bool = True) -> ClipCase:
    variant, by_faces, law = spec
    lift, kwargs = diagonal_call(variant, by_faces=by_faces, law=law, empty_polygon=empty_polygon)
    budget = exact.exact_work_budget(stage="MATERIALIZE", domain_id=f"refusal-{variant}", superlevel="", cap=cap)
    plane = lift.bind(budget)
    with exact.isolated_factorization_memory():
        # a memory that is not empty: the refusal must leave ITS contents and order, not those of a cold process
        warm_lift, warm_kwargs = diagonal_call(variant + 1, by_faces=False, law=0)
        warm_budget = exact.exact_work_budget(stage="MATERIALIZE", domain_id="warm", superlevel="", cap=None)
        clip.clip_geometry(warm_lift.bind(warm_budget), warm_budget, **warm_kwargs)
        before = nc.capture_state(budget, None)
        assert before.squarefree and before.factorization, "the warm-up filled the memory"
        blob = nc.encode_call(nc.Call(nc.OP_CLIP, (plane,), kwargs, budget, None))
    if not budgeted:
        before = dataclasses.replace(before, budget=None)
    return ClipCase(f"diagonal {spec}{' + a polygon without vertices' if empty_polygon else ''}{'' if budgeted else ', no budget'}", blob, before)


def run_native_clip(mirror, call):
    return mirror.clip_geometry(call.args[0], call.budget, **call.kwargs)


def check_clip_refusal(mirror, case: ClipCase, arm, expect, label: str, retry: ClipCase | None = None) -> "nc.Outcome":
    """Один отказ порта на `case`: состояние не тронуто; эталон на нём = чистый эталон; нативный вызов на нём же после отказа = чистый эталон. Возвращает чистый прогон `case`.

    `arm(mirror)` включает ручку (или ничего), `expect()` — фабрика `pytest.raises(...)` (контекст используется дважды). Нативный вызов после отказа — тот же самый
    (ручка сработала один раз) или, если отказ детерминирован входом, `retry`: другой вызов, который порт отвечает (тот же снимок «до»)."""

    pure = case.pure_oracle()
    assert_state_moved(case.before, pure, label)
    if retry is None:
        wanted = pure
    else:
        # computed BEFORE the refusal: a pure oracle run restores the snapshot and would wipe the live state the retry must run on
        wanted = ClipCase(retry.label, retry.blob, case.before).pure_oracle()
        assert wanted.exception is None, f"{label}: the call to retry is one the port answers"
    # (а) the refusal, then the ORACLE on the very same state
    call = case.fresh()
    observed_before = nc.observe(call)
    arm(mirror)
    with expect():
        run_native_clip(mirror, call)
    mirror.force_refusal(None)
    assert_untouched(nc.OP_CLIP, case.before, observed_before, call, label)
    continued = nc.execute(call)
    found = nc.compare_outcomes(nc.OP_CLIP, case.before, pure, continued)
    assert not found, f"{label}: the oracle after the refusal differs from a pure oracle run: " + "; ".join(str(item) for item in found[:4])
    # (б) the refusal, then the SAME native call on the very same state
    call = case.fresh()
    observed_before = nc.observe(call)
    arm(mirror)
    with expect():
        run_native_clip(mirror, call)
    mirror.force_refusal(None)
    assert_untouched(nc.OP_CLIP, case.before, observed_before, call, label)
    retried = call if retry is None else nc.decode_call(nc.OP_CLIP, retry.blob, call.budget, call.store)
    answered = nc.execute(retried, function=mirror.clip_geometry)
    found = nc.compare_outcomes(nc.OP_CLIP, case.before, wanted, answered)
    assert not found, f"{label}: the native call after the refusal differs from a pure oracle run: " + "; ".join(str(item) for item in found[:4])
    return pure


# --------------------------------------------------------------------------
# Настоящий вход: многоугольник без вершин
# --------------------------------------------------------------------------

REAL_INPUT_VARIANTS = (
    ("a budget without a cap", {"cap": None, "budgeted": True}),
    ("a budget with a cap that holds the work", {"cap": 5_000, "budgeted": True}),
    ("no budget (the unbudgeted telemetry)", {"cap": None, "budgeted": False}),
)


@pytest.mark.parametrize("name,options", REAL_INPUT_VARIANTS, ids=[name for name, _ in REAL_INPUT_VARIANTS])
def test_a_zero_vertex_polygon_is_a_late_port_refusal_that_leaves_the_state_as_it_was(mirror, name, options):
    """Порт отказывается `NativePortUnsupported` ПОСЛЕ работы над предыдущим многоугольником; эталон поднимает `ValueError` — тоже с частичными эффектами (это его исход)."""

    for spec in SPECS:
        case = make_clip_case(spec, empty_polygon=True, **options)
        retry = make_clip_case(spec, **options)
        pure = check_clip_refusal(mirror, case, lambda _mirror: None, lambda: pytest.raises(cftuv_native.NativePortUnsupported, match="without vertices"), f"{name}, {case.label}", retry)
        assert pure.exception is not None and pure.exception[0] == "ValueError", f"the oracle's own outcome on a polygon without vertices: {pure.exception}"
        if case.before.budget is not None:
            spent = sum(pure.after.budget["articles"]) - sum(case.before.budget["articles"])
        else:
            spent = sum(pure.after.unbudgeted) - sum(case.before.unbudgeted)
        assert spent > 0, f"{case.label}: no exact work happened before the refusal"


# --------------------------------------------------------------------------
# Ручка: отказ после ПОЛНОГО вычисления (есть нормали, журнал памяти, статьи, счётчики)
# --------------------------------------------------------------------------

CLIP_FORCED = (
    ("unsupported", cftuv_native.NativePortUnsupported, "forced by the test knob"),
    ("invalid_input", cftuv_native.NativePortUnsupported, "invalid mirror input"),
    ("internal", cftuv_native.NativePortUnsupported, "internal state"),
    ("diverged", cftuv_native.NativeDivisionDiverged, "did not finish"),
    ("panic", RuntimeError, "native panic: forced by the test knob"),
)


@pytest.mark.parametrize("kind,error,text", CLIP_FORCED, ids=[kind for kind, _error, _text in CLIP_FORCED])
@pytest.mark.parametrize("budgeted", (True, False), ids=("budget", "no budget"))
def test_a_forced_late_refusal_of_a_clip_that_would_have_written_normals_leaves_the_state_as_it_was(mirror, kind, error, text, budgeted):
    for spec in SPECS:
        case = make_clip_case(spec, budgeted=budgeted)
        pure = check_clip_refusal(mirror, case, lambda session, kind=kind: session.force_refusal(kind), lambda: pytest.raises(error, match=text), f"{kind}, {case.label}")
        assert pure.exception is None and pure.observed["plane_normals"] != "[]", "a call that completes and writes offset normals: the writes are what the refusal must not let through"


def test_the_forced_refusals_are_the_named_family_a_caller_catches():
    caught = {kind: error for kind, error, _text in CLIP_FORCED}
    for kind in ("unsupported", "invalid_input", "internal", "diverged"):
        assert issubclass(caught[kind], cftuv_native.NATIVE_REFUSALS), kind
    assert not issubclass(RuntimeError, cftuv_native.NATIVE_REFUSALS), "a panic is no member of the family: a plain RuntimeError"
    assert set(cftuv_native.NATIVE_REFUSALS) == {cftuv_native.NativePortStale, cftuv_native.NativeUnsupportedPython, cftuv_native.NativePortUnsupported, cftuv_native.NativeDivisionDiverged}
    import cftuv_envelope.materialize.frames as frames

    for oracle_error in (frames.MaterializationRefusal, exact.ExactCanonicalizationWorkBudgetExhausted, OverflowError, ValueError, ZeroDivisionError, KeyError):
        assert not issubclass(oracle_error, cftuv_native.NATIVE_REFUSALS), f"{oracle_error.__name__} is an outcome of the oracle, not a refusal of the port"


# --------------------------------------------------------------------------
# Покрытие: ручка (настоящего позднего отказа покрытия достичь нечем)
# --------------------------------------------------------------------------

COVERAGE_FORCED = tuple(item for item in CLIP_FORCED if item[0] != "unsupported")


def coverage_partition():
    import wavefront_cases

    figure = wavefront_cases.right_triangle(12)
    return build_faces(figure, build_skeleton(figure))


@dataclasses.dataclass
class CoverageCase:
    partition: object
    alpha: Fraction
    before: "nc.StateV1"

    def fresh(self):
        budget, store = nc.restore_state(self.before)
        return nc.Call(nc.OP_COVERAGE, (self.partition, self.alpha), {}, budget, store)


def make_coverage_case(*, budgeted: bool, cap=None, warm: bool = True) -> CoverageCase:
    partition = coverage_partition()
    with exact.isolated_factorization_memory():
        if warm:
            # a memory that is not empty: the refusal must leave its contents and order
            coverage_module._coverage_at(partition, Fraction(2), None, None)
        budget = exact.exact_work_budget(stage="COVERAGE", domain_id="refusal", superlevel="L1", cap=cap)
        before = nc.capture_state(budget, {})
    if not budgeted:
        before = dataclasses.replace(before, budget=None)
    return CoverageCase(partition, Fraction(5, 2), before)


def run_coverage(function, call, traces):
    """Вызов с пятым аргументом `traces`; исход — `Outcome` со снимком процесса, исключение — часть исхода (тип и текст)."""

    try:
        result, error = function(call.args[0], call.args[1], call.budget, call.store, traces), None
    except Exception as exc:  # noqa: BLE001 - the exception of the operation is part of its outcome
        result, error = None, (type(exc).__qualname__, str(exc))
    return nc.Outcome(result, error, nc.capture_state(call.budget, call.store), nc.observe(call), 0.0)


@pytest.mark.parametrize("kind,error,text", COVERAGE_FORCED, ids=[kind for kind, _error, _text in COVERAGE_FORCED])
@pytest.mark.parametrize("budgeted", (True, False), ids=("budget", "no budget"))
def test_a_forced_late_refusal_of_a_coverage_leaves_the_state_the_store_and_the_traces_as_they_were(mirror, kind, error, text, budgeted):
    case = make_coverage_case(budgeted=budgeted)
    pure_traces: list = []
    pure = run_coverage(coverage_module._coverage_at, case.fresh(), pure_traces)
    assert pure.exception is None and pure_traces and pure.after.store, "a coverage that completes, records traces and writes a store record"
    assert_state_moved(case.before, pure, kind)
    # (а) the refusal, then the ORACLE on the very same state
    call = case.fresh()
    traces: list = []
    mirror.force_refusal(kind)
    with pytest.raises(error, match=text):
        mirror.coverage_at(call.args[0], call.args[1], call.budget, call.store, traces)
    mirror.force_refusal(None)
    assert_untouched(nc.OP_COVERAGE, case.before, {}, call, kind, traces)
    traces_continued: list = []
    continued = run_coverage(coverage_module._coverage_at, call, traces_continued)
    found = nc.compare_outcomes(nc.OP_COVERAGE, case.before, pure, continued)
    assert not found and nc.canonical(traces_continued) == nc.canonical(pure_traces), f"{kind}: the oracle after the refusal differs from a pure oracle run: " + "; ".join(str(item) for item in found[:4])
    # (б) the refusal, then the SAME native call on the very same state
    call = case.fresh()
    traces = []
    mirror.force_refusal(kind)
    with pytest.raises(error, match=text):
        mirror.coverage_at(call.args[0], call.args[1], call.budget, call.store, traces)
    mirror.force_refusal(None)
    assert_untouched(nc.OP_COVERAGE, case.before, {}, call, kind, traces)
    answered = run_coverage(mirror.coverage_at, call, traces)
    found = nc.compare_outcomes(nc.OP_COVERAGE, case.before, pure, answered)
    assert not found and nc.canonical(traces) == nc.canonical(pure_traces), f"{kind}: the native call after the refusal differs from a pure oracle run: " + "; ".join(str(item) for item in found[:4])


def test_a_clip_only_knob_is_an_error_for_a_coverage_not_a_silent_no_op(mirror):
    case = make_coverage_case(budgeted=True)
    call = case.fresh()
    mirror.force_refusal("unsupported")
    with pytest.raises(ValueError, match="clip's"):
        mirror.coverage_at(call.args[0], call.args[1], call.budget, call.store)
    assert mirror.coverage_at(call.args[0], call.args[1], call.budget, call.store) is not None, "the knob was consumed by the call that refused it"
    with pytest.raises(ValueError, match="unknown forced refusal"):
        mirror.force_refusal("no such refusal")


# --------------------------------------------------------------------------
# Ранние отказы и отказ шима по чужому коду статуса
# --------------------------------------------------------------------------


def test_the_early_refusals_leave_the_state_as_it_was(mirror, monkeypatch):
    case = make_clip_case(SPECS[0])
    pin = cftuv_native.pin
    # a stale port, an interpreter below the floor
    for verdict, error in ((("stale", ("materialize/clip.py",)), cftuv_native.NativePortStale), (("unsupported_python",), cftuv_native.NativeUnsupportedPython)):
        call = case.fresh()
        observed = nc.observe(call)
        monkeypatch.setitem(pin._VERDICTS, "clip", verdict)
        with pytest.raises(error):
            run_native_clip(mirror, call)
        monkeypatch.undo()
        assert_untouched(nc.OP_CLIP, case.before, observed, call, error.__name__)
    # a plane without the table the normals are written into
    call = case.fresh()

    class Bare:
        triangles = call.args[0].triangles

    with pytest.raises(cftuv_native.NativePortUnsupported, match="_normal_by_position"):
        mirror.clip_geometry(Bare(), call.budget, **call.kwargs)
    assert_untouched(nc.OP_CLIP, case.before, nc.observe(call), call, "a plane without a normal table")
    # an input the extension cannot carry
    call = case.fresh()
    broken = {**call.kwargs, "points": {**call.kwargs["points"], "node:broken": (1, 2)}}
    with pytest.raises(TypeError, match="cftuv_native"):
        mirror.clip_geometry(call.args[0], call.budget, **broken)
    assert_untouched(nc.OP_CLIP, case.before, nc.observe(call), call, "an input the extension cannot carry")
    # and the session answers right after all of them
    after = nc.execute(case.fresh(), function=mirror.clip_geometry)
    assert not nc.compare_outcomes(nc.OP_CLIP, case.before, case.pure_oracle(), after)


def test_a_stale_coverage_port_leaves_the_state_as_it_was(mirror, monkeypatch):
    case = make_coverage_case(budgeted=True)
    call = case.fresh()
    traces: list = []
    monkeypatch.setitem(cftuv_native.pin._VERDICTS, "coverage", ("stale", ("wavefront/coverage.py",)))
    with pytest.raises(cftuv_native.NativePortStale):
        mirror.coverage_at(call.args[0], call.args[1], call.budget, call.store, traces)
    monkeypatch.undo()
    assert_untouched(nc.OP_COVERAGE, case.before, {}, call, "a stale coverage port", traces)


class _FakeSession:
    """Сессия расширения, которая отвечает заданным кодом и ненулевой ценой, будто вычисление ушло вперёд: цену отказа порта шим не вправе применять."""

    def __init__(self, answer):
        self.answer = answer
        self.cleared = 0

    def clip_geometry(self, *_arguments):
        return self.answer

    def coverage_at(self, *_arguments):
        return self.answer

    def clear(self):
        self.cleared += 1

    def view_matches(self, *_tables):
        return True


@pytest.mark.parametrize("status,detail", ((99, None), (5, ("a",)), (6, None), (7, ("b",)), (12, ("c",))))
def test_a_status_the_shim_does_not_know_is_a_port_refusal_nothing_of_which_is_applied(status, detail):
    """Таблица замкнута с другой стороны: всё, чего нет в `ORACLE_STATUSES` (и неизвестный код тоже), — отказ порта, а его цена (даже ненулевая) не применяется."""

    case = make_clip_case(SPECS[0])
    real = cftuv_native.new_mirror()
    real._bind_clip()
    call = case.fresh()
    observed = nc.observe(call)
    fake = _FakeSession((None, status, detail, [3, 1, 1, 1, 0], [9, 9, 9, 9, 9, 9], 15, [0, 0, 0, 0, 0]))
    real._session = fake
    with pytest.raises(cftuv_native.NATIVE_REFUSALS):
        real.clip_geometry(call.args[0], call.budget, **call.kwargs)
    assert fake.cleared == 1, "the shim forgets the mirror too"
    assert_untouched(nc.OP_CLIP, case.before, observed, call, f"clip status {status}")
    coverage = make_coverage_case(budgeted=True)
    call = coverage.fresh()
    traces: list = []
    real = cftuv_native.new_mirror()
    real._bind_coverage()
    fake = _FakeSession((None, status, detail, [3, 1, 1, 1, 0], [9, 9, 9, 9, 9, 9], 15, [0, 0, 0, 0, 0]))
    real._session = fake
    with pytest.raises(cftuv_native.NATIVE_REFUSALS):
        real.coverage_at(call.args[0], call.args[1], call.budget, call.store, traces)
    assert fake.cleared == 1
    assert_untouched(nc.OP_COVERAGE, coverage.before, {}, call, f"coverage status {status}", traces)


def test_the_oracle_statuses_are_one_table_for_the_shim_and_the_extension():
    assert tuple(sorted(cost.ORACLE_STATUSES)) == tuple(sorted(cftuv_native.oracle_statuses()))
    refusals = {cost.STATUS_INVALID_INPUT, cost.STATUS_DIVERGED, cost.STATUS_INTERNAL, cost.CLIP_STATUS_UNSUPPORTED}
    assert not refusals & cost.ORACLE_STATUSES
    assert cost.ORACLE_STATUSES == {
        cost.STATUS_OK, cost.STATUS_EXHAUSTED, cost.STATUS_NEGATIVE_RADICAND, cost.STATUS_ZERO_DIVISOR, cost.STATUS_RECONSTRUCTION,
        cost.STATUS_MISSING_LINE, cost.CLIP_STATUS_OVERFLOW, cost.CLIP_STATUS_ZERO_DIVISION, cost.CLIP_STATUS_VALUE, cost.CLIP_STATUS_REFUSAL, cost.CLIP_STATUS_MISSING_KEY,
    }
    for status in (*refusals, 99):
        assert isinstance(cost.CostMirror._port_refusal(status, ("detail",)), cftuv_native.NATIVE_REFUSALS)


# --------------------------------------------------------------------------
# Отрицательный контроль: проверка «не тронуто» видит каждый эффект, который пропустил бы протекающий отказ
# --------------------------------------------------------------------------


def _spoilers() -> dict:
    import bisect

    def article(call):
        call.budget.radical_materializations += 1

    def counts(_call):
        exact.SIGN_COUNTS["total"] += 1

    def unbudgeted(_call):
        exact.UNBUDGETED_WORK.radical_materializations += 1

    def squarefree(_call):
        exact._SQUAREFREE_MEMO[10**9 + 7] = (1, 10**9 + 7)

    def support_order(_call):
        first = next(iter(exact._PRIME_SUPPORT_MEMO))
        exact._PRIME_SUPPORT_MEMO[first] = exact._PRIME_SUPPORT_MEMO.pop(first)

    def factorization_order(_call):
        first = next(iter(exact._FACTORIZATION_MEMO))
        exact._FACTORIZATION_MEMO[first] = exact._FACTORIZATION_MEMO.pop(first)

    def prime(_call):
        bisect.insort(exact._KNOWN_PRIMES, 10**9 + 7)
        exact._KNOWN_PRIME_SET.add(10**9 + 7)

    def prime_in_the_set_only(_call):
        exact._KNOWN_PRIME_SET.add(10**9 + 9)

    def normal(call):
        call.args[0]._normal_by_position[(0.5, 0.5, 0.5)] = (0.0, 0.0, 1.0)

    return {
        "an article is paid": article,
        "a sign counter moves": counts,
        "the unbudgeted telemetry moves": unbudgeted,
        "a squarefree entry is added": squarefree,
        "a support entry is reordered": support_order,
        "a factorization is touched": factorization_order,
        "a prime is registered": prime,
        "a prime is in the set only": prime_in_the_set_only,
        "a normal is written": normal,
    }


@pytest.mark.parametrize("name", sorted(_spoilers()))
def test_the_untouched_check_names_every_effect_a_leaky_refusal_would_let_through(name):
    case = make_clip_case(SPECS[0])
    call = case.fresh()
    observed = nc.observe(call)
    assert_untouched(nc.OP_CLIP, case.before, observed, call, "control")
    _spoilers()[name](call)
    with pytest.raises((AssertionError, nc.CorpusError)):
        assert_untouched(nc.OP_CLIP, case.before, observed, call, name)


def test_the_untouched_check_names_a_store_record_and_a_trace_a_leaky_coverage_refusal_would_let_through():
    case = make_coverage_case(budgeted=True)
    call = case.fresh()
    traces: list = []
    assert_untouched(nc.OP_COVERAGE, case.before, {}, call, "control", traces)
    call.store[("prime-universe", ())] = (1,)
    with pytest.raises(AssertionError):
        assert_untouched(nc.OP_COVERAGE, case.before, {}, call, "a store record", traces)
    call = case.fresh()
    with pytest.raises(AssertionError):
        assert_untouched(nc.OP_COVERAGE, case.before, {}, call, "a trace", [([1], [])])


# --------------------------------------------------------------------------
# Исходы эталона по-прежнему применяют частичные эффекты (отказ порта их не отнимает)
# --------------------------------------------------------------------------


def test_an_oracle_outcome_still_applies_its_partial_effects_exactly_as_the_oracle_leaves_them(mirror):
    """Исчерпание бюджета посреди резки и посреди покрытия — исходы эталона: частичное состояние равно состоянию чистого прогона эталона, а не «как до вызова»."""

    done = make_clip_case(SPECS[0]).pure_oracle()
    spent = sum(done.after.budget["articles"])
    exhausted = 0
    for cap in (1, 3, spent // 2, spent - 1):
        case = make_clip_case(SPECS[0], cap=cap)
        expected = case.pure_oracle()
        actual = nc.execute(case.fresh(), function=mirror.clip_geometry)
        assert not nc.compare_outcomes(nc.OP_CLIP, case.before, expected, actual), f"clip cap {cap}"
        if expected.exception is not None and expected.exception[0] == "ExactCanonicalizationWorkBudgetExhausted":
            exhausted += 1
            assert nc.canonical(expected.after.as_payload()) != nc.canonical(case.before.as_payload()), "the oracle's exhaustion leaves partial effects"
    assert exhausted >= 1, "the clip sweep reaches an exhaustion"
    reached = 0
    for cap in (0, 2, 5, 11):
        case = make_coverage_case(budgeted=True, cap=cap, warm=False)
        wanted_traces, got_traces = [], []
        expected = run_coverage(coverage_module._coverage_at, case.fresh(), wanted_traces)
        actual = run_coverage(mirror.coverage_at, case.fresh(), got_traces)
        assert not nc.compare_outcomes(nc.OP_COVERAGE, case.before, expected, actual) and nc.canonical(wanted_traces) == nc.canonical(got_traces), f"coverage cap {cap}"
        if expected.exception is not None and expected.exception[0] == "ExactCanonicalizationWorkBudgetExhausted":
            reached += 1
            assert nc.canonical(expected.after.as_payload()) != nc.canonical(case.before.as_payload()), "the oracle's exhaustion leaves partial effects"
    assert reached >= 1, "the coverage sweep reaches an exhaustion"


def test_a_follow_up_native_call_on_a_clean_state_is_the_oracles_after_every_refusal(mirror):
    """Одна и та же сессия модуля пережила все отказы выше: общий прогон вызовов на ней равен эталону."""

    runner = geometry.DropinRunner(mirror)
    compared = 0
    for spec in SPECS:
        lift, kwargs = diagonal_call(spec[0], by_faces=spec[1], law=spec[2])
        run = geometry.compare_generated(runner, f"after-refusals-{spec}", lift, kwargs, None)
        assert run.equal, geometry.explain([(f"after-refusals-{spec}", run)])
        compared += 1
    for seed in range(12):
        made = generated_call(seed)
        if made is None:
            continue
        run = geometry.compare_generated(runner, f"after-refusals-{seed}", made[0], made[1], None)
        assert run.equal, geometry.explain([(f"after-refusals-{seed}", run)])
        compared += 1
    assert compared >= 12
