"""Продуктовый путь Envelope (хост без Blender): сессия, пул, названные исходы.

Утверждений четыре, и каждое стоит на числе:

1. ПОДГОТОВКА НЕ ПЕРЕСТРАИВАЕТСЯ. Закон UV — параметр материализации, а не
   часть ключа кэша подготовок: нажатие продукта после отладочной кнопки берёт
   те же подготовки (счётчики сборок контроллера не растут), а холодное
   нажатие наполняет кэши тем же вычислителем, что и отладка.
2. РАЗМЕЩЕНИЕ НЕ МЕНЯЕТ ОТВЕТ. Домен считает воркер либо родитель одним и тем
   же `produce_domain`: результаты (батч, исход, счётчики, дайджест) равны на
   нуле воркеров, на двух настоящих воркерах и в пуле «в процессе».
3. ОТКАЗ НАЗВАН. Пул, который не стартовал, и задача, которая упала, — счётчик,
   строка консоли и пометка `placement`; домен считает родитель, ответ тот же.
4. ВОРКЕР БЕЗ BLENDER. Цепочка импортов продуктового пути не тянет `bpy`.
"""

from __future__ import annotations

import dataclasses
import os
import pickle
import subprocess
import sys
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_domain_pool as pool_module  # noqa: E402
from cftuv import envelope_production_export as production  # noqa: E402
from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1  # noqa: E402
from cftuv.envelope_debug_session import (  # noqa: E402
    EnvelopeDebugSessionController,
    evaluate_envelope_debug_staged,
)
from cftuv.envelope_domain_pool import (  # noqa: E402
    DomainPool,
    DomainPoolRunV1,
    DomainTaskResultV1,
    DomainTaskV1,
    order_by_cost,
    shutdown_domain_pool,
    solve_task,
)
from cftuv.envelope_production_export import (  # noqa: E402
    MATERIALIZED,
    OUTCOME_DOMAIN_RAISED,
    PLACEMENT_FALLBACK,
    PLACEMENT_UNAVAILABLE,
    PLACEMENT_WORKER,
    PRODUCTION_UV_POLICY,
    ProductionInputV1,
    produce_domain,
    production_console_lines,
    production_status_text,
    run_production,
    solve_production_task,
)
from cftuv.envelope_queue_export import (  # noqa: E402
    POOL_DISPATCHED,
    POOL_TASK_FALLBACK,
    POOL_UNAVAILABLE,
    POOL_WORKERS,
)
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
ALPHA = 0.25


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


@pytest.fixture
def _pool_always(monkeypatch):
    """Малая партия остаётся в родителе (порог); тесту нужен именно пул."""

    from cftuv import envelope_queue_pool

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)


class _InProcessPool:
    """Пул без подпроцессов: та же `solve_task`, ответ через pickle, как по трубе."""

    requested = 2

    def __init__(self, *, fail=(), drop=()):
        self.fail = frozenset(fail)
        self.drop = frozenset(drop)
        self.kinds: list[str] = []

    def run(self, tasks):
        results = {}
        for task, _frame in order_by_cost(tasks):
            self.kinds.append("production" if task.production is not None else "other")
            if task.task_id in self.drop:
                continue
            if task.task_id in self.fail:
                results[task.task_id] = DomainTaskResultV1(
                    task.task_id,
                    error="Traceback (most recent call last):\n"
                    "InjectedTaskFailure: boom",
                )
                continue
            results[task.task_id] = pickle.loads(pickle.dumps(solve_task(task)))
        return DomainPoolRunV1(results, self.requested)


def _in_process_session(monkeypatch, pool):
    monkeypatch.setattr(
        pool_module, "get_domain_pool", lambda workers, external_python="": pool
    )


def _debug_build(bundle, controller, *, workers=0, alpha=ALPHA, density=None):
    """Кнопка отладки движка QUEUE на сессии: то, что нажимает владелец первым."""

    return evaluate_envelope_debug_staged(
        bundle,
        frozenset(range(ROW)),
        alpha,
        profile=EnvelopeDebugProfileBuilderV1("row", "QUEUE"),
        controller=controller,
        source_object_key="object",
        source_data_key="mesh",
        engine="QUEUE",
        density=density,
        workers=workers,
    )


def _production(bundle, controller=None, *, workers=0, alpha=ALPHA, density=None, **kwargs):
    controller = controller or EnvelopeDebugSessionController()
    run = run_production(
        controller,
        bundle,
        frozenset(range(ROW)),
        alpha,
        source_object_key="object",
        source_data_key="mesh",
        density=density,
        workers=workers,
        **kwargs,
    )
    return run, controller


def _answers(run):
    """Ответ прогона: результаты (равенство без секунд и размещения) целиком."""

    return tuple(run.results)


def _counter(run, name):
    return run.counter(name)


# --------------------------------------------------------------------------
# 1. Подготовка не перестраивается
# --------------------------------------------------------------------------


def test_the_cache_key_of_a_preparation_ignores_the_uv_law_and_alpha():
    """Ключ — ревизия, домен, рёбра и подпись угловой политики; закон UV не в нём."""

    import cftuv_envelope as kernel
    from decimal import Decimal

    from cftuv.envelope_request_policy import (
        ENVELOPE_UV_POLICY_DEBUG_NO_UV,
        ENVELOPE_UV_POLICY_DIRECT_STRIP,
        build_envelope_request_contract,
        envelope_angular_policy,
    )

    def request(uv, alpha):
        return build_envelope_request_contract(
            kernel,
            kernel.DecalRequestId("request"),
            frozenset(),
            Decimal(alpha),
            envelope_angular_policy(kernel, 2),
            uv_policy_id=uv,
        )

    key = EnvelopeDebugSessionController._preparation_key
    debug = key("revision", "domain", frozenset({1, 2}), request(ENVELOPE_UV_POLICY_DEBUG_NO_UV, "0.25"))
    strip = key("revision", "domain", frozenset({1, 2}), request(ENVELOPE_UV_POLICY_DIRECT_STRIP, "0.5"))
    assert debug == strip
    other_density = key(
        "revision",
        "domain",
        frozenset({1, 2}),
        build_envelope_request_contract(
            kernel,
            kernel.DecalRequestId("request"),
            frozenset(),
            Decimal("0.25"),
            envelope_angular_policy(kernel, 4),
        ),
    )
    assert other_density != debug


def test_a_production_press_after_a_debug_build_reuses_every_preparation():
    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    _debug_build(bundle, controller)
    builds = controller.build_counts
    keys = sorted(map(repr, controller._conveyor_preparation_cache))
    assert builds["CONVEYOR_PREPARATION"] == ROW

    run, _ = _production(bundle, controller)

    assert not run.cold
    assert [item.outcome for item in run.results] == [MATERIALIZED] * ROW
    # Доказательство повторного использования — счётчики сборок, не секунды.
    assert controller.build_counts == builds
    assert _counter(run, production.PRODUCTION_PREPARATION_BUILDS) == 0
    assert _counter(run, production.PRODUCTION_PATCH_METRIC_BUILDS) == 0
    assert _counter(run, production.PRODUCTION_DOMAIN_GEOMETRY_BUILDS) == 0
    assert _counter(run, production.PRODUCTION_PREPARATION_REUSED) == ROW
    assert _counter(run, production.PRODUCTION_COLD_FILL) == 0
    assert sorted(map(repr, controller._conveyor_preparation_cache)) == keys


def test_a_cold_press_fills_the_session_like_the_debug_build_and_the_next_one_is_warm():
    bundle = quad_row_bundle(ROW)
    reference = EnvelopeDebugSessionController()
    _debug_build(bundle, reference)

    cold, controller = _production(bundle)

    assert cold.cold
    assert _counter(cold, production.PRODUCTION_COLD_FILL) == 1
    assert _counter(cold, production.PRODUCTION_PREPARATION_BUILDS) == ROW
    # Те же кэши, что у отладочной кнопки: ключи и счёт сборок.
    assert controller.build_counts == reference.build_counts
    assert sorted(map(repr, controller._conveyor_preparation_cache)) == sorted(
        map(repr, reference._conveyor_preparation_cache)
    )

    warm, _ = _production(bundle, controller)

    assert not warm.cold
    assert _counter(warm, production.PRODUCTION_PREPARATION_BUILDS) == 0
    assert _answers(warm) == _answers(cold)


def test_the_production_press_leaves_the_debug_session_state_alone():
    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    _debug_build(bundle, controller)
    queue_session = controller.queue_session

    _production(bundle, controller)

    assert controller.queue_session is queue_session
    assert controller.invalidation_count == 0


def test_the_slider_alpha_reaches_the_coverage_and_the_preparation_is_still_reused():
    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    _debug_build(bundle, controller, alpha=0.25)
    near, _ = _production(bundle, controller, alpha=0.25)

    far, _ = _production(bundle, controller, alpha=0.5)

    assert _counter(far, production.PRODUCTION_PREPARATION_BUILDS) == 0
    areas = lambda run: [len(item.batch.faces) for item in run.results]  # noqa: E731
    assert all(item.is_materialized for item in far.results)
    # Широкая полоса — другое покрытие: содержательные дайджесты разошлись.
    assert [item.content_digest for item in near.results] != [
        item.content_digest for item in far.results
    ]
    assert areas(far)


# --------------------------------------------------------------------------
# 2. Размещение не меняет ответ
# --------------------------------------------------------------------------


def test_the_in_process_pool_reproduces_the_sequential_answer(monkeypatch, _pool_always):
    bundle = quad_row_bundle(ROW)
    expected, _ = _production(bundle, workers=0)
    pool = _InProcessPool()
    _in_process_session(monkeypatch, pool)

    run, controller = _production(bundle, workers=2)

    assert _answers(run) == _answers(expected)
    assert [item.placement for item in run.results] == [PLACEMENT_WORKER] * ROW
    assert {item.placement for item in expected.results} == {"parent"}
    assert _counter(run, POOL_WORKERS) == 2
    assert _counter(run, POOL_DISPATCHED) == ROW
    assert _counter(run, POOL_TASK_FALLBACK) == 0
    assert _counter(run, POOL_UNAVAILABLE) == 0
    assert "production" in pool.kinds
    assert _counter(expected, POOL_WORKERS) is None


def test_a_small_batch_stays_in_the_parent_and_the_pool_is_not_asked(monkeypatch):
    from cftuv import envelope_queue_pool

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 10**9)
    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    _debug_build(bundle, controller)
    pool = _InProcessPool()
    _in_process_session(monkeypatch, pool)

    run, _ = _production(bundle, controller, workers=2)

    assert "production" not in pool.kinds
    assert _counter(run, POOL_DISPATCHED) == 0
    assert {item.placement for item in run.results} == {"parent"}


def test_real_workers_reproduce_the_sequential_answer_bit_for_bit(_pool_always):
    bundle = quad_row_bundle(ROW)
    expected, _ = _production(bundle, workers=0)

    run, controller = _production(bundle, workers=2)

    assert _answers(run) == _answers(expected)
    assert [item.content_digest for item in run.results] == [
        item.content_digest for item in expected.results
    ]
    assert [item.placement for item in run.results] == [PLACEMENT_WORKER] * ROW
    assert _counter(run, POOL_WORKERS) == 2
    assert _counter(run, POOL_TASK_FALLBACK) == 0
    # Тёплое нажатие на живых воркерах: подготовки взяты из кэша сессии.
    warm, _ = _production(bundle, controller, workers=2)
    assert _counter(warm, production.PRODUCTION_PREPARATION_BUILDS) == 0
    assert [item.placement for item in warm.results] == [PLACEMENT_WORKER] * ROW
    assert _answers(warm) == _answers(expected)


# --------------------------------------------------------------------------
# 3. Отказ назван
# --------------------------------------------------------------------------


@pytest.mark.parametrize("injection", ("fail", "drop"))
def test_a_failed_task_is_named_counted_and_computed_in_the_parent(
    injection, monkeypatch, capsys, _pool_always
):
    bundle = quad_row_bundle(ROW)
    expected, _ = _production(bundle, workers=0)
    controller = EnvelopeDebugSessionController()
    _debug_build(bundle, controller)
    _in_process_session(monkeypatch, _InProcessPool(**{injection: (2,)}))
    capsys.readouterr()

    run, _ = _production(bundle, controller, workers=2)

    assert _counter(run, POOL_TASK_FALLBACK) == 1
    assert _counter(run, POOL_DISPATCHED) == ROW
    assert _counter(run, POOL_UNAVAILABLE) == 0
    placements = [item.placement for item in run.results]
    assert placements.count(PLACEMENT_FALLBACK) == 1
    assert placements.count(PLACEMENT_WORKER) == ROW - 1
    console = capsys.readouterr().out
    assert POOL_TASK_FALLBACK in console
    assert (
        "InjectedTaskFailure: boom" in console
        if injection == "fail"
        else "every worker died" in console
    )
    assert _answers(run) == _answers(expected)


def test_an_unavailable_pool_is_named_and_the_parent_computes(
    monkeypatch, capsys, _pool_always
):
    bundle = quad_row_bundle(ROW)
    expected, _ = _production(bundle, workers=0)
    controller = EnvelopeDebugSessionController()
    _debug_build(bundle, controller)
    _in_process_session(
        monkeypatch, DomainPool(2, python_executable="C:/nowhere/python.exe")
    )
    capsys.readouterr()

    run, _ = _production(bundle, controller, workers=2)

    assert _counter(run, POOL_UNAVAILABLE) == 1
    assert _counter(run, POOL_DISPATCHED) == 0
    assert _counter(run, POOL_WORKERS) == 0
    assert {item.placement for item in run.results} == {PLACEMENT_UNAVAILABLE}
    out = capsys.readouterr().out
    assert POOL_UNAVAILABLE in out and "sequentially" in out
    assert _answers(run) == _answers(expected)


def test_a_preparation_that_cannot_be_shipped_is_a_named_fallback(
    monkeypatch, capsys, _pool_always
):
    from cftuv import envelope_queue_pool

    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    _debug_build(bundle, controller)
    expected, _ = _production(bundle, controller, workers=0)
    blobs = controller.preparation_blobs
    original = blobs.blob_of
    victim = next(iter(controller._conveyor_preparation_cache.values()))

    def refusing(prepared):
        if prepared is victim:
            raise pickle.PicklingError("injected")
        return original(prepared)

    monkeypatch.setattr(blobs, "blob_of", refusing)
    _in_process_session(monkeypatch, _InProcessPool())
    capsys.readouterr()

    run, _ = _production(bundle, controller, workers=2)

    assert _counter(run, POOL_TASK_FALLBACK) == 1
    assert [item.placement for item in run.results].count(PLACEMENT_FALLBACK) == 1
    assert "cannot be shipped" in capsys.readouterr().out
    assert _answers(run) == _answers(expected)
    assert envelope_queue_pool.COVERAGE_POOL_MIN_BYTES == 0


def test_a_domain_refused_on_the_metric_is_named_with_its_host_outcome():
    bundle = quad_row_bundle(ROW, lifted_corner=0.05)

    run, _ = _production(bundle)

    refused = [item for item in run.results if not item.is_materialized]
    assert [item.patch_id for item in refused] == [ROW - 1]
    assert refused[0].outcome == "NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED"
    assert refused[0].batch is None and refused[0].detail
    assert production_status_text(run.results) == (
        f"MATERIALIZED {ROW - 1} / refused 1 (NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED)"
    )
    lines = production_console_lines(run.results)
    assert len(lines) == 2
    assert f"patch {ROW - 1}" in lines[0]
    assert "NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED" in lines[0]
    assert lines[1].endswith(production_status_text(run.results))
    assert _counter(run, production.PRODUCTION_REFUSED) == 1
    assert _counter(run, production.PRODUCTION_MATERIALIZED) == ROW - 1


def test_the_status_text_counts_repeated_outcomes():
    def result(patch, outcome, batch=None):
        return production.ProductionDomainResultV1(
            patch, f"d{patch}", outcome, batch
        )

    items = [
        result(0, MATERIALIZED, object()),
        result(1, "B_OUTCOME"),
        result(2, "A_OUTCOME"),
        result(3, "B_OUTCOME"),
        # Исход без батча — не материализованный домен, как бы он ни назывался.
        result(4, MATERIALIZED),
    ]
    assert production_status_text(items) == (
        "MATERIALIZED 1 / refused 4 (A_OUTCOME, B_OUTCOME x2, MATERIALIZED)"
    )


def _prepared_domain():
    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    _debug_build(bundle, controller)
    return next(iter(controller._conveyor_preparation_cache.values()))


def test_the_debug_law_and_an_unknown_law_are_refused_differently():
    prepared = _prepared_domain()

    named = produce_domain(0, "d", prepared, "0.25", uv_policy_id="ENVELOPE_DEBUG_NO_UV_V1")

    assert named.outcome == "UV_POLICY_UNSUPPORTED"
    assert named.detail == "ENVELOPE_DEBUG_NO_UV_V1" and named.batch is None
    with pytest.raises(ValueError, match="unknown UV policy"):
        produce_domain(0, "d", prepared, "0.25", uv_policy_id="SOMETHING_ELSE")


def test_a_preparation_that_is_not_exact_is_named_by_the_kernel_and_never_covered():
    from cftuv_envelope.wavefront.conveyor import ConveyorOutcome

    prepared = dataclasses.replace(
        _prepared_domain(),
        outcome=ConveyorOutcome.PREPARATION_IS_NOT_EXACT,
        compilation=None,
    )

    refused = produce_domain(3, "domain", prepared, "0.25")

    assert refused.outcome == "COVERAGE_IS_NOT_EXACT"
    assert refused.detail.startswith("preparation:")
    assert refused.patch_id == 3 and refused.batch is None


def test_an_exception_inside_a_domain_is_named_and_does_not_escape(monkeypatch):
    from cftuv_envelope.materialize import domain as kernel_domain

    prepared = _prepared_domain()

    def broken(*_args, **_kwargs):
        raise RuntimeError("kernel bug")

    monkeypatch.setattr(kernel_domain, "materialize_domain", broken)

    refused = produce_domain(1, "domain", prepared, "0.25")

    assert refused.outcome == OUTCOME_DOMAIN_RAISED
    assert "kernel bug" in refused.detail


def test_equality_ignores_the_seconds_and_the_placement_but_nothing_else():
    prepared = _prepared_domain()
    first = produce_domain(1, "d", prepared, "0.25")
    second = produce_domain(1, "d", prepared, "0.25")
    assert first.is_materialized and first == second
    assert dataclasses.replace(second, placement="worker", seconds=9.0) == first
    assert dataclasses.replace(second, content_digest="x") != first
    assert dataclasses.replace(second, counters=()) != first


# --------------------------------------------------------------------------
# Сам вид задачи пула
# --------------------------------------------------------------------------


def _task(prepared, task_id=0, uv=PRODUCTION_UV_POLICY):
    return DomainTaskV1(
        task_id,
        4,
        "domain",
        None,
        None,
        "0.25",
        frozenset(),
        production=ProductionInputV1(pickle.dumps(prepared, protocol=5), uv),
    )


def test_the_task_kind_answers_in_process_and_the_answer_is_the_parents():
    prepared = _prepared_domain()

    reply = solve_task(_task(prepared))

    assert reply.ok and reply.error == "" and reply.queue_domain is None
    assert reply.production.placement == PLACEMENT_WORKER
    assert reply.production == produce_domain(4, "domain", prepared, "0.25")
    assert solve_production_task(_task(prepared)).production == reply.production
    # Ответ едет по трубе целиком.
    assert pickle.loads(pickle.dumps(reply)).production == reply.production


def test_a_refusal_of_the_kernel_is_an_answer_and_not_a_failed_task():
    reply = solve_task(_task(_prepared_domain(), uv="ENVELOPE_DEBUG_NO_UV_V1"))

    assert reply.ok and reply.production.outcome == "UV_POLICY_UNSUPPORTED"


def test_a_bad_input_returns_the_trace_instead_of_raising():
    reply = solve_task(_task(_prepared_domain(), uv="SOMETHING_ELSE"))

    assert not reply.ok and "unknown UV policy" in reply.error


def test_production_tasks_are_ranked_like_coverage_tasks():
    prepared = _prepared_domain()
    light = _task(prepared, 0)
    heavy = DomainTaskV1(
        1, 1, "x", "y" * 500_000, None, "0.25", frozenset()
    )

    ordered = order_by_cost([light, heavy])

    assert [task.task_id for task, _frame in ordered] == [1, 0]
    frame_length = len(ordered[1][1])
    assert pool_module._frame_cost(light, ordered[1][1]) == frame_length / (
        pool_module.COVERAGE_FRAME_COST_DIVISOR
    )


# --------------------------------------------------------------------------
# Свидетельства и стена Blender
# --------------------------------------------------------------------------


def test_the_json_export_round_trips_every_batch_through_the_kernel_codec(tmp_path):
    from cftuv_envelope import GeometryBatchCodecV1

    bundle = quad_row_bundle(ROW, lifted_corner=0.05)
    run, _ = _production(bundle)

    summary = production.export_production_json(run.results, tmp_path, label="row")

    import json

    data = json.loads(summary.read_text(encoding="utf-8"))
    assert data["status"] == production_status_text(run.results)
    assert len(data["domains"]) == ROW
    written = {item["patch_id"]: item for item in data["domains"] if "batch_file" in item}
    assert sorted(written) == [0, 1, 2, 3]
    for result in run.results:
        if not result.is_materialized:
            continue
        batch = GeometryBatchCodecV1.loads(
            (tmp_path / written[result.patch_id]["batch_file"]).read_bytes()
        )
        assert batch == result.batch
        assert written[result.patch_id]["content_digest"] == result.content_digest
    refused = next(item for item in data["domains"] if "batch_file" not in item)
    assert refused["outcome"] == "NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED"


def test_the_production_chain_imports_without_blender():
    """Воркер — обычный интерпретатор: продуктовый путь не тянет `bpy`."""

    script = "\n".join(
        (
            "import sys",
            "class Block:",
            "    def find_spec(self, name, path=None, target=None):",
            "        if name.split('.')[0] in {'bpy', 'bmesh', 'mathutils'}:",
            "            raise ImportError('blocked: ' + name)",
            "sys.meta_path.insert(0, Block())",
            "import cftuv.envelope_production_export",
            "import cftuv.envelope_domain_pool",
            "assert not {'bpy', 'bmesh', 'mathutils'} & set(sys.modules)",
            "print('NO_BLENDER_IN_PRODUCTION_CHAIN')",
        )
    )
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(KERNEL_SRC.parents[1]), str(KERNEL_SRC))
    )
    completed = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        env=environment,
        timeout=120,
    )
    assert completed.returncode == 0, completed.stderr.decode(errors="replace")
    assert b"NO_BLENDER_IN_PRODUCTION_CHAIN" in completed.stdout
