"""Нативные вызовы идут под ОДНОЙ процессной блокировкой (`cftuv_native.cost.NATIVE_LOCK`): два потока дают те же ответы, что один.

Продукт зовёт `coverage_at` из потока предпросмотра alpha и из главного потока; таблицы памяти канонизации, счётчики знаков и нативная сессия —
состояние ПРОЦЕССА, а расширение отпускает GIL, пока считает. Без блокировки два вызова читали бы и писали одни таблицы и одну сессию.

1. Каждый публичный метод `CostMirror` обёрнут (проверка по самому классу: добавленный метод без блокировки краснит тест).
2. На границе сессии, в каждом вызове расширения, блокировка удерживается ЭТИМ потоком (шпион вместо сессии), для покрытия, резки, стоимостных операций и служебных методов.
3. Вызов другого потока ждёт блокировку, пока её держит первый; блокировка повторно входимая (вызов внутри вызова не виснет).
4. Два-четыре потока вперемешку гонят покрытие и резку на общем зеркале: ответ каждого вызова побитово равен последовательному (канонический код: `int` и
   `Fraction` различны, `float` по `hex`), итоговые счётчики знаков равны сумме последовательных, а после гонки зеркало равно эталону на настоящем вызове.

Модуль пропускается с названной причиной, пока расширение не собрано либо нативные операции не сверены с этим деревом ядра (`native_gate`).
"""

from __future__ import annotations

import os
import sys
import threading
import types
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import mpmath  # noqa: F401
except ModuleNotFoundError:  # питон Blender: сторонние пакеты ядра лежат среди пользовательских модулей Blender
    _modules = Path(os.environ.get("APPDATA", "")) / "Blender Foundation" / "Blender" / "4.5" / "scripts" / "modules"
    if _modules.is_dir():
        sys.path.append(str(_modules))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (проверка блокировки нативных вызовов пропущена)",
        allow_module_level=True,
    )

from native_gate import skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "coverage")
skip_unless_available(cftuv_native, "clip")

import native_clip_geometry as geometry  # noqa: E402
import native_corpus as nc  # noqa: E402
import wavefront_cases  # noqa: E402
from cftuv_native import cost  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1  # noqa: E402
from cftuv_envelope.wavefront import build_skeleton  # noqa: E402
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces  # noqa: E402


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    """Вызовы пишут в процессные счётчики и память ядра; тест их не оставляет."""

    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    with exact.isolated_factorization_memory():
        yield
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(nc._ARTICLES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)


# --------------------------------------------------------------------------
# Работа: покрытие и резка, ответ каждого вызова — канонический код
# --------------------------------------------------------------------------


def _partitions() -> list:
    found = []
    named = dict(wavefront_cases.named_corpus())
    for name in ("axis_square", "right_triangle", "diamond", "ell", "comb_2", "cross", "staircase", "u_shape"):
        polygon = named.get(name)
        if polygon is None:
            continue
        partition = build_faces(polygon, build_skeleton(polygon))
        if partition.outcome is FaceOutcome.EXACT:
            found.append((name, partition))
    return found


def _coverage_jobs() -> list:
    """`(метка, функция)`: покрытие каждой фигуры на нескольких alpha без бюджета и без `store` (ответ не зависит от памяти, цена зависит)."""

    jobs = []
    for name, partition in _partitions():
        for alpha in (Fraction(1), Fraction(5, 2), Fraction(-1, 3), Fraction(13, 3)):
            jobs.append((f"coverage {name} {alpha}", lambda partition=partition, alpha=alpha: cftuv_native.coverage_at(partition, alpha)))
    return jobs


def _clip_jobs(count: int) -> list:
    """`(метка, функция)`: сгенерированные вызовы резки (`geometry.fresh_calls`) без потолка бюджета: исход — резка либо названный отказ."""

    jobs = []
    for label, lift, kwargs, _cap in geometry.fresh_calls(20261008, count):
        def run(lift=lift, kwargs=kwargs, label=label):
            budget = exact.exact_work_budget(stage="MATERIALIZE", domain_id=f"lock-{label}", superlevel="", cap=None)
            return cftuv_native.clip_geometry(lift.bind(budget), budget, **kwargs)

        jobs.append((f"clip {label}", run))
    return jobs


def _signs_job() -> list:
    value = SqrtSumV1(((2, Fraction(1)), (3, Fraction(-1)), (1, Fraction(1, 3))))
    return [("sign", lambda: cftuv_native.sign(value, filter_bits=64))]


def _outcome_of(label: str, run) -> tuple:
    """Исход вызова для сравнения: канонический ответ либо `(класс, текст)` исключения."""

    try:
        result = run()
    except Exception as exc:  # noqa: BLE001 - исключение операции — часть её исхода
        return (type(exc).__qualname__, str(exc))
    if label.startswith("coverage"):
        return ("ok", nc.answer_digest(nc.OP_COVERAGE, result))
    if label.startswith("clip"):
        return ("ok", nc.answer_digest(nc.OP_CLIP, result))
    return ("ok", nc.canonical(result))


def _jobs() -> list:
    return _coverage_jobs() + _clip_jobs(36) + _signs_job()


# --------------------------------------------------------------------------
# 1-3. Блокировка стоит там, где должна
# --------------------------------------------------------------------------


def test_every_public_method_of_the_mirror_is_serialized():
    public = {name: member for name, member in vars(cost.CostMirror).items() if isinstance(member, types.FunctionType) and not name.startswith("_")}
    assert {"coverage_at", "clip_geometry", "execute", "sign", "invalidate", "forget_clip", "prime_universe_remembered", "reset_memory"} <= set(public)
    unwrapped = sorted(name for name, member in public.items() if not hasattr(member, "__wrapped__"))
    assert not unwrapped, f"public methods of CostMirror outside NATIVE_LOCK: {unwrapped}"
    assert not hasattr(cost.CostMirror.__init__, "__wrapped__"), "the constructor shares no state: it takes no lock"


class _SessionSpy:
    """Вместо нативной сессии: каждый вызов расширения отмечает, держит ли блокировку ЭТОТ поток."""

    def __init__(self, session) -> None:
        self._session = session
        self.calls: list = []
        self.unguarded: list = []

    def __getattr__(self, name):
        member = getattr(self._session, name)
        if not callable(member):
            return member

        def spied(*arguments, **keywords):
            self.calls.append(name)
            if not cost.NATIVE_LOCK._is_owned():
                self.unguarded.append(name)
            return member(*arguments, **keywords)

        return spied


def test_every_call_of_the_extension_is_made_with_the_lock_held():
    mirror = cftuv_native.new_mirror()
    spy = _SessionSpy(mirror._session)
    mirror._session = spy
    name, partition = _partitions()[0]
    mirror.coverage_at(partition, Fraction(1), exact.exact_work_budget(stage="COVERAGE", cap=None), {})
    for label, lift, kwargs, _cap in geometry.fresh_calls(20261008, 6):
        budget = exact.exact_work_budget(stage="MATERIALIZE", domain_id=f"spy-{label}", superlevel="", cap=None)
        try:
            mirror.clip_geometry(lift.bind(budget), budget, **kwargs)
        except Exception:  # noqa: BLE001 - исход вызова неважен, важна блокировка на границе
            pass
    mirror.sign(SqrtSumV1(((2, Fraction(1)), (3, Fraction(1)))))
    mirror.prime_support(2 * 3 * 5 * 7 * 11)
    mirror.squarefree_split(2 * 2 * 3 * 5)
    mirror.lengths()
    mirror.clip_cache_size()
    mirror.set_clip_warm_enabled(True)
    mirror.clear_clip_warm()
    mirror.clip_warm_stats()
    mirror.forget_clip()
    mirror.reset_memory()
    mirror.invalidate()
    assert not spy.unguarded, f"extension calls made without NATIVE_LOCK: {sorted(set(spy.unguarded))}"
    assert {"coverage_at", "clip_geometry", "run", "lengths", "clip_cache", "forget_clip", "clear"} <= set(spy.calls), sorted(set(spy.calls))


def test_a_call_from_another_thread_waits_while_the_lock_is_held_and_the_lock_is_reentrant():
    mirror = cftuv_native.new_mirror()
    value = SqrtSumV1(((2, Fraction(1)), (3, Fraction(-1))))
    expected = mirror.sign(value)
    finished = threading.Event()
    answers: list = []

    def other():
        answers.append(mirror.sign(value))
        finished.set()

    thread = threading.Thread(target=other)
    with cost.NATIVE_LOCK:
        assert mirror.sign(value) == expected, "a call inside a call must not deadlock"
        thread.start()
        assert not finished.wait(0.3), "the other thread ran a native call while this one held the lock"
    assert finished.wait(30), "the other thread never got the lock back"
    thread.join()
    assert answers == [expected]


# --------------------------------------------------------------------------
# 4. Потоки дают те же ответы, что один
# --------------------------------------------------------------------------


def _run_in_threads(jobs: list, threads: int, rounds: int) -> tuple:
    """Каждый поток гонит весь список по кругу со своим сдвигом; `(исходы {метка: {исход}}, ошибки потоков)`."""

    outcomes: dict = {}
    errors: list = []
    guard = threading.Lock()
    start = threading.Barrier(threads)

    def worker(number: int) -> None:
        try:
            start.wait(60)
            for round_number in range(rounds):
                for index in range(len(jobs)):
                    label, run = jobs[(index + number * 7 + round_number * 3) % len(jobs)]
                    found = _outcome_of(label, run)
                    with guard:
                        outcomes.setdefault(label, set()).add(found)
        except BaseException as exc:  # noqa: BLE001 - любая ошибка потока — провал теста, с именем
            with guard:
                errors.append(f"thread {number}: {type(exc).__name__}: {exc}")

    pool = [threading.Thread(target=worker, args=(number,)) for number in range(threads)]
    for thread in pool:
        thread.start()
    for thread in pool:
        thread.join()
    return outcomes, errors


def _sign_total(counts: dict) -> int:
    return sum(counts.values())


@pytest.mark.parametrize("threads", [2, 4])
def test_calls_from_several_threads_are_bit_equal_to_sequential_calls(threads):
    jobs = _jobs()
    assert len(jobs) > 40
    reference = {}
    before = dict(exact.SIGN_COUNTS)
    for label, run in jobs:
        reference[label] = _outcome_of(label, run)
    sequential = {key: exact.SIGN_COUNTS[key] - before[key] for key in before}
    assert any(found[0] == "ok" for found in reference.values()) and any(found[0] != "ok" for found in reference.values()), "the jobs must include a named refusal"
    rounds = 2
    before = dict(exact.SIGN_COUNTS)
    outcomes, errors = _run_in_threads(jobs, threads, rounds)
    concurrent = {key: exact.SIGN_COUNTS[key] - before[key] for key in before}
    assert not errors, errors
    differing = [label for label, found in outcomes.items() if found != {reference[label]}]
    assert not differing, f"answers of concurrent calls differ from the sequential ones: {differing[:5]}"
    assert set(outcomes) == set(reference)
    assert concurrent == {key: value * threads * rounds for key, value in sequential.items()}, (sequential, concurrent)


def test_after_concurrent_calls_the_mirror_still_equals_the_oracle_on_a_real_call():
    jobs = _jobs()
    outcomes, errors = _run_in_threads(jobs, 3, 1)
    assert not errors, errors
    assert outcomes
    runner = geometry.DropinRunner(mirror=cftuv_native.default_mirror())
    runs = []
    for label, lift, kwargs, cap in geometry.fresh_calls(20261009, 24):
        runs.append((label, geometry.compare_generated(runner, label, lift, kwargs, cap)))
    failed = [(name, run) for name, run in runs if not run.equal]
    assert not failed, geometry.explain(failed)


def test_negative_control_without_the_lock_the_same_threads_collide(monkeypatch):
    """Отрицательный контроль: с пустой блокировкой те же потоки получают `Already borrowed` от сессии (расширение отпускает GIL): значит, держит именно замок."""

    class NoLock:
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def _is_owned(self):
            return True

    jobs = _jobs()
    collisions = 0
    monkeypatch.setattr(cost, "NATIVE_LOCK", NoLock())
    for _attempt in range(3):
        outcomes, errors = _run_in_threads(jobs, 4, 2)
        collisions += sum(1 for found in outcomes.values() for outcome in found if outcome[0] == "RuntimeError" and "borrowed" in outcome[1]) + len(errors)
        if collisions:
            break
    cftuv_native.default_mirror().invalidate()
    assert collisions, "four threads without the lock never collided: the test no longer proves the lock does the holding"
