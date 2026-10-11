"""Круг кодека над снапшотом идёт на выборке кнопки и целиком в воротах (SNAPSHOT_BYTE_CHECK_SAMPLED_V1).

Утверждения:

1. Политика: переключатель `CFTUV_SNAPSHOT_BYTE_CHECK` (`all` | `sample` | `off`); выборка детерминирована (дайджест идентичности домена) и ~1/16; первое холодное
   нажатие новой идентичности кода проверяется целиком и помечается проверенным лишь после того, как отработало; неразборчивое значение - `all`, названное счётчиком.
2. Каждый пропуск считается: решение по домену записано счётчиком под доменом, итоги без домена сложены в конце нажатия.
3. Ответ не зависит от политики (результаты нажатия равны при `all`, `sample`, `off`), а круг кодека реально не вызывается у пропущенных доменов.
4. Вердикт воркера (`SnapshotCleanV1`) не зависит от проверки байтов: он выведен из проверки самого объекта, декодированный объект в него не входит.
5. Ворота (тесты, CI, полевые инструменты) ставят `all` и не выбирают, что проверять.
"""

from __future__ import annotations

import ast
import pickle
import re
import sys
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

import cftuv_envelope as kernel  # noqa: E402
from cftuv import envelope_domain_pool as pool_module  # noqa: E402
from cftuv import envelope_snapshot_byte_check as byte_check  # noqa: E402
from cftuv import envelope_snapshot_check as check_module  # noqa: E402
from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import DomainPoolRunV1, order_by_cost, shutdown_domain_pool, solve_task  # noqa: E402
from cftuv.envelope_production_export import ColdProductionInputV1, run_production  # noqa: E402
from cftuv.envelope_snapshot_check import SNAPSHOT_CHECK_ADOPTED  # noqa: E402
from cftuv.envelope_snapshot_byte_check import (  # noqa: E402
    BYTE_CHECK_ALL,
    BYTE_CHECK_ENV_INVALID,
    BYTE_CHECK_FIRST_PRESS,
    BYTE_CHECK_OFF,
    BYTE_CHECK_SAMPLED,
    BYTE_CHECK_SKIPPED,
    MODE_ALL,
    MODE_OFF,
    MODE_SAMPLE,
    SnapshotByteCheckPlanV1,
    byte_check_decision,
    finish_press,
    in_sample,
    mode_of_environment,
)
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
ALPHA = 0.25
ROOT = Path(__file__).resolve().parents[1]
ENVIRONMENT = byte_check.BYTE_CHECK_ENVIRONMENT


@pytest.fixture(autouse=True)
def _a_session_that_has_checked_nothing():
    """Память «эта идентичность кода уже проверена» живёт в процессе: каждый тест начинает как первое нажатие сеанса и оставляет её пустой."""

    byte_check.forget_checked_identities()
    yield
    byte_check.forget_checked_identities()


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


# --------------------------------------------------------------------------
# 1. Политика
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("environ", "expected"),
    [
        ({}, (MODE_SAMPLE, "")),
        ({ENVIRONMENT: ""}, (MODE_SAMPLE, "")),
        ({ENVIRONMENT: "   "}, (MODE_SAMPLE, "")),
        ({ENVIRONMENT: "all"}, (MODE_ALL, "")),
        ({ENVIRONMENT: " ALL "}, (MODE_ALL, "")),
        ({ENVIRONMENT: "sample"}, (MODE_SAMPLE, "")),
        ({ENVIRONMENT: "Off"}, (MODE_OFF, "")),
        ({ENVIRONMENT: "sampel"}, (MODE_ALL, "sampel")),
        ({ENVIRONMENT: "1"}, (MODE_ALL, "1")),
    ],
)
def test_the_switch_reads_all_sample_off_and_an_unreadable_value_means_the_full_check(environ, expected):
    assert mode_of_environment(environ) == expected


def test_the_product_default_is_the_sample():
    assert byte_check.DEFAULT_PRODUCT_MODE == MODE_SAMPLE
    assert mode_of_environment({}) == (MODE_SAMPLE, "")
    assert byte_check.SAMPLE_MODULUS == 16


def test_the_sample_is_deterministic_and_about_one_domain_in_sixteen():
    ids = [f"patch-domain:revision-{revision}:{number}" for revision in range(64) for number in range(64)]
    chosen = [item for item in ids if in_sample(item)]
    assert chosen == [item for item in ids if in_sample(item)]  # то же решение при повторе
    assert len(ids) / 32 < len(chosen) < len(ids) / 8, len(chosen)
    # Дайджест SHA-256, а не `hash()`: решение не зависит от процесса (PYTHONHASHSEED). Закреплены конкретные значения.
    assert [number for number in range(40) if in_sample(f"patch-domain:pinned:{number}")] == [2, 19, 21]


def test_the_decision_table():
    sampled = next(item for item in (f"patch-domain:t:{n}" for n in range(1000)) if in_sample(item))
    skipped = next(item for item in (f"patch-domain:t:{n}" for n in range(1000)) if not in_sample(item))
    assert SnapshotByteCheckPlanV1(MODE_ALL).decision(skipped) == BYTE_CHECK_ALL
    assert SnapshotByteCheckPlanV1(MODE_OFF).decision(sampled) == BYTE_CHECK_OFF
    assert SnapshotByteCheckPlanV1(MODE_SAMPLE, first_press=True).decision(skipped) == BYTE_CHECK_FIRST_PRESS
    assert SnapshotByteCheckPlanV1(MODE_SAMPLE).decision(sampled) == BYTE_CHECK_SAMPLED
    assert SnapshotByteCheckPlanV1(MODE_SAMPLE).decision(skipped) == BYTE_CHECK_SKIPPED
    # Снапшот не одного домена выборке не подлежит: проверяется целиком.
    assert SnapshotByteCheckPlanV1(MODE_SAMPLE).decision(None) == BYTE_CHECK_ALL
    assert {name for name, runs in byte_check.DECISIONS.items() if not runs} == {BYTE_CHECK_SKIPPED, BYTE_CHECK_OFF}


def test_no_plan_means_the_check_runs_and_nothing_is_recorded():
    profile = EnvelopeDebugProfileBuilderV1("t", "t")
    assert byte_check_decision(None, "patch-domain:x:1", profile) is True
    assert profile.snapshot().counters == ()


def test_every_decision_is_counted_under_its_domain():
    profile = EnvelopeDebugProfileBuilderV1("t", "t")
    sampled = next(item for item in (f"patch-domain:c:{n}" for n in range(1000)) if in_sample(item))
    skipped = next(item for item in (f"patch-domain:c:{n}" for n in range(1000)) if not in_sample(item))
    plan = SnapshotByteCheckPlanV1(MODE_SAMPLE)
    assert byte_check_decision(plan, sampled, profile) is True
    assert byte_check_decision(plan, skipped, profile) is False
    assert byte_check_decision(SnapshotByteCheckPlanV1(MODE_OFF), skipped, profile) is False
    counters = {(item.name, item.patch_domain_id): item.value for item in profile.snapshot().counters}
    assert counters == {(BYTE_CHECK_SAMPLED, sampled): 1, (BYTE_CHECK_SKIPPED, skipped): 1, (BYTE_CHECK_OFF, skipped): 1}


def test_the_first_press_of_an_identity_is_checked_fully_and_marked_only_after_it_ran(monkeypatch):
    identity = ("kernel", "host", "NATIVE:build-1")
    monkeypatch.setattr(byte_check, "execution_identity_of", lambda *args: identity)

    first = byte_check.resolve_plan("NATIVE", environ={})
    assert (first.mode, first.first_press, first.identity) == (MODE_SAMPLE, True, identity)

    # Нажатие, где FIRST_PRESS не отработал (всё из кэша), идентичность проверенной не делает.
    warm = EnvelopeDebugProfileBuilderV1("t", "t")
    finish_press(warm, first)
    assert byte_check.resolve_plan("NATIVE", environ={}).first_press is True

    ran = EnvelopeDebugProfileBuilderV1("t", "t")
    assert byte_check_decision(first, "patch-domain:r:1", ran) is True
    finish_press(ran, first)
    second = byte_check.resolve_plan("NATIVE", environ={})
    assert (second.mode, second.first_press) == (MODE_SAMPLE, False)

    # Другая идентичность (новый код либо другая сборка нативного ядра) - снова первое нажатие.
    monkeypatch.setattr(byte_check, "execution_identity_of", lambda *args: ("kernel", "host", "NATIVE:build-2"))
    assert byte_check.resolve_plan("NATIVE", environ={}).first_press is True
    # Идентичность не определить - нажатие считается первым всегда.
    monkeypatch.setattr(byte_check, "execution_identity_of", lambda *args: None)
    unknown = byte_check.resolve_plan("NATIVE", environ={})
    assert unknown.first_press is True
    profile = EnvelopeDebugProfileBuilderV1("t", "t")
    byte_check_decision(unknown, "patch-domain:r:1", profile)
    finish_press(profile, unknown)
    assert byte_check.resolve_plan("NATIVE", environ={}).first_press is True


def test_all_and_off_never_ask_for_the_identity(monkeypatch):
    monkeypatch.setattr(byte_check, "execution_identity_of", lambda *args: pytest.fail("identity must not be read"))
    for value in (MODE_ALL, MODE_OFF):
        plan = byte_check.resolve_plan("NATIVE", environ={ENVIRONMENT: value})
        assert (plan.mode, plan.first_press, plan.identity) == (value, False, None)


def test_the_real_identity_is_a_stable_tuple_of_the_code_and_the_backends():
    identity = byte_check.execution_identity_of("PYTHON")
    assert identity is not None and identity == byte_check.execution_identity_of("PYTHON")
    assert len(identity) == 3 and all(isinstance(item, str) for item in identity)
    hash(identity)


def test_an_unreadable_value_runs_the_full_check_and_is_named(capsys):
    plan = byte_check.resolve_plan("NATIVE", environ={ENVIRONMENT: "sampel"})
    assert (plan.mode, plan.first_press, plan.invalid) == (MODE_ALL, False, "sampel")
    assert BYTE_CHECK_ENV_INVALID in capsys.readouterr().out
    profile = EnvelopeDebugProfileBuilderV1("t", "t")
    finish_press(profile, plan)
    assert {(item.name, item.patch_domain_id): item.value for item in profile.snapshot().counters} == {(BYTE_CHECK_ENV_INVALID, None): 1}


def test_the_end_of_a_press_folds_the_decisions_of_all_domains_into_totals():
    profile = EnvelopeDebugProfileBuilderV1("t", "t")
    for domain, name in (("d1", BYTE_CHECK_SKIPPED), ("d2", BYTE_CHECK_SKIPPED), ("d3", BYTE_CHECK_SAMPLED)):
        profile.add_counter(name, 1, domain)
    profile.add_counter("PRODUCTION_SOMETHING_ELSE", 7, "d1")
    finish_press(profile, SnapshotByteCheckPlanV1(MODE_SAMPLE))
    totals = {item.name: item.value for item in profile.snapshot().counters if item.patch_domain_id is None}
    assert totals == {BYTE_CHECK_SKIPPED: 2, BYTE_CHECK_SAMPLED: 1}
    finish_press(profile, None)  # вне продуктового пути итогов нет, и это не ошибка


# --------------------------------------------------------------------------
# 3. Продуктовый путь: ответ не зависит от политики, пропущенный домен не гонит кодек
# --------------------------------------------------------------------------


class _Room:
    where = "parent"


class _InProcessPool:
    """Пул «в процессе»: задача проходит пиклом туда и обратно, как труба, и считается как в воркере."""

    requested = 2

    def run(self, tasks):
        results = {}
        _Room.where = "worker"
        try:
            for task, _frame in order_by_cost(tasks):
                results[task.task_id] = pickle.loads(pickle.dumps(solve_task(task)))
        finally:
            _Room.where = "parent"
        return DomainPoolRunV1(results, self.requested)


@pytest.fixture
def _pool_always(monkeypatch):
    from cftuv import envelope_queue_pool

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)
    pool = _InProcessPool()
    monkeypatch.setattr(pool_module, "get_domain_pool", lambda workers, external_python="": pool if int(workers) >= pool_module.MIN_POOL_WORKERS else None)
    return pool


def _press(monkeypatch, mode, *, workers=2, bundle=None, controller=None):
    """Нажатие под переключателем `mode`; при `sample` идентичность считается уже проверенной, если не задано иное (выборка, а не первое нажатие)."""

    monkeypatch.setenv(ENVIRONMENT, mode)
    controller = EnvelopeDebugSessionController() if controller is None else controller
    run = run_production(
        controller,
        quad_row_bundle(ROW) if bundle is None else bundle,
        frozenset(range(ROW)),
        ALPHA,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=workers,
    )
    return run


def _counts(run):
    return {item.name: item.value for item in run.profile.counters if item.patch_domain_id is None and item.name in byte_check.BYTE_CHECK_COUNTERS}


def _per_domain(run):
    return {(item.name, item.patch_domain_id) for item in run.profile.counters if item.patch_domain_id is not None and item.name in byte_check.DECISIONS}


def _count_codec(monkeypatch):
    calls = {"dumps": 0, "loads": 0}
    codec = kernel.AnalysisSnapshotCodecV1
    for name in calls:
        real = getattr(codec, name)

        def counted(*args, _real=real, _name=name):
            calls[_name] += 1
            return _real(*args)

        monkeypatch.setattr(codec, name, staticmethod(counted))
    return calls


@pytest.fixture
def _half_of_the_domains_are_sampled(monkeypatch):
    """Выборка 1 из 2 на пяти доменах фикстуры: в нажатии есть и выбранные, и пропущенные (при 1/16 пять доменов, скорее всего, не выбрал бы никто)."""

    monkeypatch.setattr(byte_check, "SAMPLE_MODULUS", 2)


@pytest.mark.parametrize("workers", [0, 2], ids=["in the parent", "in the workers"])
@pytest.mark.parametrize("lifted_corner", [0.0, 1.0], ids=["planar", "with an unfolded domain"])
def test_the_answer_does_not_depend_on_the_policy_and_a_skipped_domain_runs_no_codec(
    monkeypatch, _pool_always, _half_of_the_domains_are_sampled, workers, lifted_corner
):
    bundle = quad_row_bundle(ROW, lifted_corner=lifted_corner)
    calls = _count_codec(monkeypatch)

    full = _press(monkeypatch, MODE_ALL, workers=workers, bundle=bundle)
    full_codec = dict(calls)
    assert _counts(full) == {BYTE_CHECK_ALL: ROW}
    assert full_codec["dumps"] >= 2 * ROW and full_codec["loads"] >= ROW

    # Выборка при уже проверенном коде: первое нажатие бы проверило всё.
    identity = byte_check.execution_identity_of(full.kernel_backend)
    byte_check.mark_identity_checked(identity)
    for key in calls:
        calls[key] = 0
    sampled = _press(monkeypatch, MODE_SAMPLE, workers=workers, bundle=bundle)
    counts = _counts(sampled)
    assert set(counts) == {BYTE_CHECK_SAMPLED, BYTE_CHECK_SKIPPED}, counts
    assert counts[BYTE_CHECK_SAMPLED] + counts[BYTE_CHECK_SKIPPED] == ROW
    assert calls["dumps"] == full_codec["dumps"] - 2 * counts[BYTE_CHECK_SKIPPED]
    assert calls["loads"] == full_codec["loads"] - counts[BYTE_CHECK_SKIPPED]
    assert tuple(sampled.results) == tuple(full.results)

    for key in calls:
        calls[key] = 0
    off = _press(monkeypatch, MODE_OFF, workers=workers, bundle=bundle)
    assert _counts(off) == {BYTE_CHECK_OFF: ROW}
    assert calls == {"dumps": full_codec["dumps"] - 2 * ROW, "loads": full_codec["loads"] - ROW}
    assert tuple(off.results) == tuple(full.results)
    # Каждое решение записано и под доменом, а не только итогом.
    assert len(_per_domain(off)) == ROW == len(_per_domain(full))


def test_the_first_press_of_a_session_checks_every_domain_and_the_next_one_samples(monkeypatch, _pool_always, _half_of_the_domains_are_sampled):
    bundle = quad_row_bundle(ROW)
    first = _press(monkeypatch, MODE_SAMPLE, bundle=bundle)
    assert _counts(first) == {BYTE_CHECK_FIRST_PRESS: ROW}
    # Другая выгрузка (другая ревизия -> другие домены), тот же код: теперь выборка.
    second = _press(monkeypatch, MODE_SAMPLE, bundle=quad_row_bundle(ROW, lifted_corner=1.0))
    counts = _counts(second)
    assert BYTE_CHECK_FIRST_PRESS not in counts and counts[BYTE_CHECK_SAMPLED] + counts[BYTE_CHECK_SKIPPED] == ROW


def test_a_domain_whose_bytes_fail_is_refused_with_the_name_it_always_had_when_checked_and_proceeds_when_skipped(monkeypatch, _pool_always):
    """Выбранный домен, чьи байты не прошли, отказан как раньше (то же имя исхода хоста); пропущенный идёт дальше - объявленное изменение поведения."""

    real = kernel.AnalysisSnapshotCodecV1.dumps
    calls = {"dumps": 0}

    def not_a_fixed_point(snapshot):
        # Второй `dumps` проверки (над декодированным объектом) даёт другие байты: «канонические байты не неподвижная точка кодека».
        calls["dumps"] += 1
        payload = real(snapshot)
        return payload + b" " if calls["dumps"] % 2 == 0 else payload

    monkeypatch.setattr(kernel.AnalysisSnapshotCodecV1, "dumps", staticmethod(not_a_fixed_point))

    refused = _press(monkeypatch, MODE_ALL, workers=0)
    assert _counts(refused) == {BYTE_CHECK_ALL: ROW}
    assert not any(item.is_materialized for item in refused.results)
    assert {item.outcome for item in refused.results} == {"ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID"}
    assert all("fixed point of the codec" in item.detail for item in refused.results)

    # Выборка, в которую не попал ни один домен, при проверенном коде: те же байты не проверены, и домены идут дальше.
    monkeypatch.setattr(byte_check, "SAMPLE_MODULUS", 10**18)
    byte_check.mark_identity_checked(byte_check.execution_identity_of(refused.kernel_backend))
    proceeded = _press(monkeypatch, MODE_SAMPLE, workers=0)
    assert _counts(proceeded) == {BYTE_CHECK_SKIPPED: ROW}
    assert all(item.is_materialized for item in proceeded.results)


# --------------------------------------------------------------------------
# 4. Вердикт воркера не зависит от проверки байтов
# --------------------------------------------------------------------------


def test_the_clean_verdict_of_the_worker_is_the_check_of_the_object_whatever_the_byte_check_did(monkeypatch, _pool_always):
    import cftuv_envelope.validation as kernel_validation

    seen = []
    real = kernel_validation.validate_analysis_snapshot

    def spy(snapshot, **kwargs):
        seen.append((snapshot, kwargs.get("developable_stretch_budget")))
        return real(snapshot, **kwargs)

    monkeypatch.setattr(kernel, "validate_analysis_snapshot", spy)
    monkeypatch.setattr(kernel_validation, "validate_analysis_snapshot", spy)
    verdicts = {}
    for mode in (MODE_ALL, MODE_OFF):
        seen.clear()
        run = _press(monkeypatch, mode)
        assert all(item.is_materialized for item in run.results)
        budgeted = [snapshot for snapshot, budget in seen if budget is not None]
        # Вердикт воркера выведен из объекта под допуском запроса: тот же объект проверен и без допуска, декодированный объект - только при `all`.
        assert all(any(snapshot is other and budget is None for other, budget in seen) for snapshot in budgeted)
        verdicts[mode] = len(seen)
        # Родитель принимает чистые записи воркера одинаково, как бы ни шла проверка байтов: запись выведена из объекта, а не из декодированного.
        assert run.counter(SNAPSHOT_CHECK_ADOPTED) == ROW, mode
    assert verdicts[MODE_ALL] - verdicts[MODE_OFF] == ROW  # ровно одна проверка (декодированного объекта) на домен


def test_the_snapshot_check_module_does_not_know_the_byte_check():
    """Модуль вердикта воркера не читает ни политику, ни кодек: пропуск круга кодека не может попасть в `SnapshotCleanV1`."""

    source = (ROOT / "cftuv" / "envelope_snapshot_check.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)} | {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    imported = {alias.name for node in ast.walk(tree) if isinstance(node, (ast.Import, ast.ImportFrom)) for alias in node.names} | {
        node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) and node.module
    }
    assert "envelope_snapshot_byte_check" not in " ".join(sorted(imported))
    assert not {"AnalysisSnapshotCodecV1", "dumps", "loads", "byte_check", "snapshot_byte_check", "validate_exported_snapshot"} & names
    assert hasattr(check_module, "WorkerSnapshotCheck")


def test_the_plan_travels_in_the_cold_task_and_is_picklable():
    plan = SnapshotByteCheckPlanV1(MODE_SAMPLE, True, ("kernel", "host", "NATIVE:build"), "")
    assert ColdProductionInputV1().snapshot_byte_check is None  # без плана (отладочный путь, фикстуры) проверка идёт на каждом домене
    task_input = ColdProductionInputV1(snapshot_byte_check=plan)
    assert pickle.loads(pickle.dumps(task_input)) == task_input


# --------------------------------------------------------------------------
# 5. Ворота ставят `all`
# --------------------------------------------------------------------------

#: Инструменты, которые считают продуктовым путём (кнопка, `run_production` или отладочный оператор) и потому обязаны задавать политику ворот сами.
_FIELD_DRIVERS = ("run_production", "build_envelope_decal_mesh", "build_exact_reference_envelope_debug")
_PINS_ALL = re.compile(r"""CFTUV_SNAPSHOT_BYTE_CHECK["']?\s*(?:,|\]\s*=|=)\s*["']?all""")


def test_every_field_tool_that_drives_the_product_path_pins_the_full_check():
    drivers = [
        path
        for path in sorted((ROOT / "tools").glob("*.py"))
        if any(token in path.read_text(encoding="utf-8") for token in _FIELD_DRIVERS)
    ]
    assert len(drivers) >= 10, "the tools directory has its field drivers"
    unpinned = [path.name for path in drivers if not _PINS_ALL.search(path.read_text(encoding="utf-8"))]
    assert unpinned == [], f"field tools that must pin CFTUV_SNAPSHOT_BYTE_CHECK=all: {unpinned}"


def test_every_blender_test_that_drives_the_product_path_pins_the_full_check_through_the_shared_module():
    """Blender-тесты импортируют помощников из `test_envelope_debug_bridge`; он и ставит `all` (сам тест запускается как отдельный скрипт)."""

    folder = ROOT / "tests" / "blender"
    shared = folder / "test_envelope_debug_bridge.py"
    assert _PINS_ALL.search(shared.read_text(encoding="utf-8"))
    drivers = [
        path
        for path in sorted(folder.glob("test_*.py"))
        if path != shared and any(token in path.read_text(encoding="utf-8") for token in _FIELD_DRIVERS)
    ]
    assert drivers
    unpinned = [path.name for path in drivers if "from test_envelope_debug_bridge import" not in path.read_text(encoding="utf-8")]
    assert unpinned == [], f"Blender tests that run the button without the shared pin: {unpinned}"


@pytest.mark.parametrize("workflow", ["host-suite.yml", "native.yml"])
def test_ci_runs_the_host_tests_with_the_full_check(workflow):
    text = (ROOT / ".github" / "workflows" / workflow).read_text(encoding="utf-8")
    assert re.search(r"^\s+CFTUV_SNAPSHOT_BYTE_CHECK:\s*all\s*$", text, re.MULTILINE), workflow


def test_the_test_suite_pins_the_full_check_unless_told_otherwise():
    text = (ROOT / "tests" / "conftest.py").read_text(encoding="utf-8")
    assert 'os.environ.setdefault("CFTUV_SNAPSHOT_BYTE_CHECK", "all")' in text
