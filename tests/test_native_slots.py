"""Матрица доступа к слотам границы Python <-> Rust (`native/cftuv-python/src/pyobj.rs`): принудительный `raw` и принудительный `attr`, каждый доказан, а не предположен.

Расширение читает и пишет слоты `Fraction`, `SqrtSumV1`, `FaceCoverageV1`, `LocalPoint3V1` по СМЕЩЕНИЮ, найденному пробой настоящего экземпляра (`Raw::probe`), либо по протоколу атрибутов. Раскладка экземпляра — не часть
стабильного ABI (`abi3` её не обещает), поэтому одной сборки под `abi3` мало: проба обязана подтвердить раскладку на КАЖДОМ интерпретаторе, а путь без пробы — оставаться верным. Режим (`CFTUV_NATIVE_SLOTS`) читается
ОДИН раз при импорте расширения: `auto` (умолчание: смещение там, где проба подтвердила, иначе атрибуты), `raw` (только смещение; раскладка, которую проба не подтвердила, — названный отказ, не откат), `attr` (только атрибуты).

Что держат тесты. 1. Режим процесса равен тому, что просили средой, и читается один раз (подпроцессы: `raw`, `attr`, `auto`, пусто, лишние пробелы и регистр; неизвестное значение — названный отказ импорта; смена
среды ПОСЛЕ импорта ничего не меняет). 2. Целые операции (покрытие на фигурах ядра, резка на сгенерированных вызовах) под КАЖДЫМ режимом сессии равны эталону, а счётчики путей (`slot_counters`) показывают, что принудительный путь прошёл
и другой не тронут. 3. Сессии процесса по умолчанию (`new_mirror()`, `coverage_at`, `clip_geometry` шима) идут путём режима процесса: в CI режим процесса — ножка матрицы (`raw`/`attr`/`auto` на 3.11 и 3.13). 4. `raw`, у которого проба не подтвердила
раскладку, — `TypeError` с именем класса и причиной при привязке; `auto` в том же случае молча идёт по атрибутам (его решение), `attr` пробу не делает.
"""

from __future__ import annotations

import os
import subprocess
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
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (матрица доступа к слотам не проверена)",
        allow_module_level=True,
    )

from native_gate import skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "coverage")
skip_unless_available(cftuv_native, "clip")

import native_clip_geometry as geometry  # noqa: E402
import native_corpus as nc  # noqa: E402
import wavefront_cases  # noqa: E402
from cftuv_native import cost as native_cost  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
from cftuv_envelope.wavefront import build_skeleton  # noqa: E402
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces  # noqa: E402

MODES = ("raw", "attr", "auto")
#: The slot classes the extension probes, as `raw_layouts()` names them.
SLOT_CLASSES = {"coverage Fraction", "coverage SqrtSumV1", "FaceCoverageV1", "clip Fraction", "clip SqrtSumV1", "LocalPoint3V1"}
FIGURES = ("axis_square", "right_triangle", "diamond", "ell", "comb_2", "cross", "staircase", "u_shape", "star_9_seed_0", "star_9_seed_3")


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    with exact.isolated_factorization_memory():
        yield
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(nc._ARTICLES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)


def _expected_process_mode() -> str:
    return (os.environ.get(cftuv_native.SLOT_MODE_ENVIRONMENT) or "auto").strip().lower() or "auto"


# --------------------------------------------------------------------------
# работа: целые операции без корпуса
# --------------------------------------------------------------------------


def _coverage_cases() -> list:
    """`(метка, состояние до, пикл входа)` покрытия на фигурах ядра: лесенка alpha с бюджетом и `store`, без них, отказ отрицательной alpha."""

    named = dict(wavefront_cases.named_corpus())
    cases = []
    for name in FIGURES:
        polygon = named[name]
        partition = build_faces(polygon, build_skeleton(polygon))
        if partition.outcome is not FaceOutcome.EXACT:
            continue
        xs = [x for loop in polygon.loops for x, _ in loop.points]
        ys = [y for loop in polygon.loops for _, y in loop.points]
        span = max(max(xs) - min(xs), max(ys) - min(ys))
        for step, alpha in enumerate((Fraction(span, 8), Fraction(span, 2), Fraction(span * 5, 8), Fraction(-1, 3))):
            with exact.isolated_factorization_memory():
                budget = exact.exact_work_budget(stage="COVERAGE", domain_id=f"slots-{name}", superlevel="L1", cap=None)
                store: dict = {}
                before = nc.capture_state(budget, store)
                blob = nc.encode_call(nc.Call(nc.OP_COVERAGE, (partition, alpha), {}, budget, store))
            cases.append((f"{name} alpha#{step}", before, blob))
    return cases


def _compare_coverage(mirror, cases) -> Counter:
    answered = Counter()
    for label, before, blob in cases:
        oracle = nc.execute(nc.prepare_call(nc.OP_COVERAGE, blob, before))
        native = nc.execute(nc.prepare_call(nc.OP_COVERAGE, blob, before), function=mirror.coverage_at)
        found = nc.compare_outcomes(nc.OP_COVERAGE, before, oracle, native)
        assert not found, f"{label}: " + "; ".join(str(item) for item in found[:4])
        answered["raised" if oracle.exception else oracle.result.outcome.value] += 1
    return answered


def _compare_clip(mirror, count: int = 60) -> Counter:
    runner = geometry.DropinRunner(mirror)
    outcomes = Counter()
    for label, lift, kwargs, cap in geometry.fresh_calls(20261007, count):
        run = geometry.compare_generated(runner, label, lift, kwargs, cap)
        assert run.equal, geometry.explain([(label, run)])
        outcomes[run.outcome_label] += 1
    return outcomes


def _work(mirror) -> tuple:
    """Покрытие и резка на `mirror`: счёт исходов, ответ равен эталону на каждом вызове."""

    covered = _compare_coverage(mirror, _coverage_cases())
    clipped = _compare_clip(mirror)
    assert covered["EXACT"] >= 20 and covered["ALPHA_IS_NEGATIVE"] >= 5, covered
    assert clipped["CLIPPED"] >= 20, clipped
    return covered, clipped


def _path_taken(before: dict, after: dict) -> dict:
    return {name: after[name] - before[name] for name in after}


# --------------------------------------------------------------------------
# 1. режим процесса: читается один раз при импорте
# --------------------------------------------------------------------------


def test_the_process_mode_is_what_the_environment_asked_for_and_a_new_session_follows_it():
    expected = _expected_process_mode()
    assert expected in cftuv_native.SLOT_MODES, f"{cftuv_native.SLOT_MODE_ENVIRONMENT}={expected!r} is not a slot mode"
    assert cftuv_native.slot_mode() == expected, "the extension did not honour the mode the process was started with"
    assert cftuv_native.new_mirror().slot_mode() == expected
    assert cftuv_native.default_mirror().slot_mode() == expected


def _import_in_a_new_process(value: str | None, code: str = "") -> subprocess.CompletedProcess:
    environment = {key: item for key, item in os.environ.items() if key != cftuv_native.SLOT_MODE_ENVIRONMENT}
    if value is not None:
        environment[cftuv_native.SLOT_MODE_ENVIRONMENT] = value
    script = "import cftuv_native; print(cftuv_native.slot_mode()); " + code
    return subprocess.run([sys.executable, "-c", script], env=environment, capture_output=True, text=True, timeout=120, check=False)


@pytest.mark.parametrize(
    "value, mode",
    [("raw", "raw"), ("attr", "attr"), ("auto", "auto"), (None, "auto"), ("", "auto"), (" RAW ", "raw"), ("Attr", "attr")],
    ids=["raw", "attr", "auto", "unset", "empty", "blanks and case", "case"],
)
def test_a_new_process_reads_the_mode_from_the_environment_when_it_imports_the_extension(value, mode):
    done = _import_in_a_new_process(value)
    assert done.returncode == 0, done.stderr[-600:]
    assert done.stdout.split() == [mode]


def test_an_unknown_mode_is_a_named_refusal_of_the_import_not_a_quiet_auto():
    done = _import_in_a_new_process("bogus")
    assert done.returncode != 0
    assert "ValueError" in done.stderr and cftuv_native.SLOT_MODE_ENVIRONMENT in done.stderr and "bogus" in done.stderr, done.stderr[-600:]


def test_the_mode_is_read_once_a_later_change_of_the_environment_changes_nothing():
    code = f"import os; os.environ[{cftuv_native.SLOT_MODE_ENVIRONMENT!r}] = 'attr'; print(cftuv_native.new_mirror().slot_mode(), cftuv_native.slot_mode())"
    done = _import_in_a_new_process("raw", code)
    assert done.returncode == 0, done.stderr[-600:]
    assert done.stdout.split() == ["raw", "raw", "raw"]


def test_a_session_can_name_its_own_mode_and_an_unknown_one_is_refused():
    for mode in MODES:
        assert cftuv_native.new_mirror(mode).slot_mode() == mode
    with pytest.raises(TypeError, match="Session"):
        cftuv_native.new_mirror("fast")


# --------------------------------------------------------------------------
# 2. каждый режим сессии: ответ равен эталону, путь — принудительный
# --------------------------------------------------------------------------


@pytest.mark.parametrize("mode", MODES)
def test_a_session_of_each_mode_answers_like_the_oracle_and_takes_exactly_the_path_it_was_given(mode):
    mirror = cftuv_native.new_mirror(mode)
    assert mirror.slot_mode() == mode
    layouts = mirror.raw_layouts()
    assert set(layouts) == SLOT_CLASSES
    assert all(layouts.values()) is (mode != "attr"), f"{mode}: the layouts the probe confirmed: {layouts}"
    cftuv_native.reset_slot_counters()
    _work(mirror)
    taken = cftuv_native.slot_counters()
    raw_used = taken["raw_reads"] + taken["raw_builds"]
    attr_used = taken["attr_reads"] + taken["attr_builds"]
    if mode == "attr":
        assert taken["attr_reads"] > 500 and taken["attr_builds"] > 500 and raw_used == 0, f"attr was forced but the raw path ran: {taken}"
    else:
        assert taken["raw_reads"] > 500 and taken["raw_builds"] > 500 and attr_used == 0, f"{mode} must take the raw path on this interpreter ({sys.version.split()[0]}), got {taken}"


def test_the_three_modes_give_the_same_answers_not_just_each_one_the_oracles():
    """Тот же вход, три режима: канонический ответ побайтно один (равенство эталону у каждого — выше; здесь — равенство режимов между собой)."""

    cases = _coverage_cases()[:12]
    answers = {}
    for mode in MODES:
        mirror = cftuv_native.new_mirror(mode)
        digests = []
        for _label, before, blob in cases:
            outcome = nc.execute(nc.prepare_call(nc.OP_COVERAGE, blob, before), function=mirror.coverage_at)
            digests.append(nc.outcome_digest(nc.OP_COVERAGE, before, outcome))
        answers[mode] = digests
    assert answers["raw"] == answers["attr"] == answers["auto"]
    assert len(set(answers["raw"])) > 3, "the cases must differ from each other, or the comparison proves nothing"


# --------------------------------------------------------------------------
# 3. сессии процесса по умолчанию идут путём режима процесса (ножка матрицы CI)
# --------------------------------------------------------------------------


def test_the_default_sessions_take_the_path_of_the_process_mode():
    mode = cftuv_native.slot_mode()
    cftuv_native.reset_slot_counters()
    # the public drop-ins on the process-wide mirror, the way the product calls them
    answered = 0
    for label, before, blob in _coverage_cases()[:16]:
        oracle = nc.execute(nc.prepare_call(nc.OP_COVERAGE, blob, before))
        native = nc.execute(nc.prepare_call(nc.OP_COVERAGE, blob, before), function=cftuv_native.coverage_at)
        assert not nc.compare_outcomes(nc.OP_COVERAGE, before, oracle, native), label
        answered += 1
    assert answered >= 10
    _compare_clip(cftuv_native.new_mirror(), 40)
    taken = cftuv_native.slot_counters()
    if mode == "attr":
        assert taken["attr_reads"] > 0 and taken["attr_builds"] > 0 and taken["raw_reads"] == taken["raw_builds"] == 0, taken
    else:
        assert taken["raw_reads"] > 0 and taken["raw_builds"] > 0 and taken["attr_reads"] == taken["attr_builds"] == 0, taken


# --------------------------------------------------------------------------
# 4. `raw` не откатывается молча
# --------------------------------------------------------------------------


class _WideFraction:
    """Класс со слотами, у которого раскладка не та, что у `Fraction` (три слота вместо двух): проба не может её подтвердить."""

    __slots__ = ("_numerator", "_denominator", "_extra")


def test_raw_refuses_by_name_a_layout_the_probe_does_not_confirm(monkeypatch):
    monkeypatch.setattr(native_cost, "Fraction", _WideFraction)
    mirror = cftuv_native.new_mirror("raw")
    with pytest.raises(TypeError) as caught:
        mirror.raw_layouts()
    message = str(caught.value)
    assert "raw slot access was forced" in message and cftuv_native.SLOT_MODE_ENVIRONMENT + "=raw" in message, message
    assert "Fraction" in message and "__basicsize__" in message, "the refusal names the class and the reason"
    assert not mirror._coverage_bound, "a refused binding leaves the session unbound"


def test_auto_falls_back_to_the_attribute_protocol_and_attr_never_probes_for_the_same_class(monkeypatch):
    monkeypatch.setattr(native_cost, "Fraction", _WideFraction)
    auto = cftuv_native.new_mirror("auto").raw_layouts()
    assert auto["coverage Fraction"] is False and auto["clip Fraction"] is False, "auto does not fake a layout the probe refused"
    assert auto["coverage SqrtSumV1"] and auto["FaceCoverageV1"] and auto["LocalPoint3V1"], "the classes the probe did confirm stay raw"
    assert not any(cftuv_native.new_mirror("attr").raw_layouts().values())
