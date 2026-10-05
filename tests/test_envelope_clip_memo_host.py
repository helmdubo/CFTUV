"""Память стадии резки на стороне хоста: привязка первого круга пула, метка в результатах, счётчики, отпечаток кода.

1. ПРИВЯЗКА ПЕРВОГО КРУГА. Память стадии живёт в воркере (`cftuv_envelope.materialize.clip_memo`), и попадание возможно, только
   если домен снова попал к тому же воркеру. `plan_first_round` раздаёт первые `N` задач (тяжёлые первыми) их прошлым воркерам;
   остальное — прежнее (воркеры берут по очереди). Раздача — подсказка размещения, ответ от неё не зависит.
2. МЕТКА И СЧЁТЧИКИ. Результат продуктового пути несёт `clip_memo` (не ответ: в равенство не входит), профиль прогона считает
   попадания и промахи ПОСЧИТАННЫХ доменов, строка владельца их называет.
3. ОТПЕЧАТОК КОДА ядра равен отпечатку, который хост кладёт в ключ содержимого (`package_fingerprint`, закон установщика).
"""

from __future__ import annotations

import dataclasses
import os
import sys
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_production_export as production  # noqa: E402
from cftuv.envelope_content_key import package_fingerprint  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import (  # noqa: E402
    DomainPool,
    DomainTaskV1,
    order_by_cost,
    plan_first_round,
    shutdown_domain_pool,
)
from cftuv.envelope_production_export import run_production  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


def _task(task_id, weight, affinity=""):
    return DomainTaskV1(
        task_id, task_id, f"d{task_id}", "x" * weight, None, "0.25", frozenset(), affinity=affinity
    )


def _ordered(*weights, affinity=True):
    return order_by_cost([_task(index, weight, f"k{index}" if affinity else "") for index, weight in enumerate(weights)])


# --------------------------------------------------------------------------
# 1. Привязка первого круга
# --------------------------------------------------------------------------


def test_without_a_history_the_heaviest_tasks_go_to_the_workers_in_order():
    ordered = _ordered(10, 5000, 900, 50)  # по цене: задачи 1, 2, 3, 0
    assert plan_first_round(ordered, [4, 7], {}) == {4: 0, 7: 1}


def test_a_task_goes_back_to_the_worker_that_computed_it_last_time():
    ordered = _ordered(10, 5000, 900, 50)
    # Прошлый раз задача 1 (самая тяжёлая) была у воркера 7, задача 2 — у воркера 4.
    plan = plan_first_round(ordered, [4, 7], {"k1": 7, "k2": 4})
    assert plan == {7: 0, 4: 1}


def test_a_task_whose_worker_is_gone_or_already_taken_goes_to_a_free_one():
    ordered = _ordered(10, 5000, 900, 50)
    # Прежний воркер задачи 1 умер (его нет среди участников); задача 2 тоже просит воркера 4, но он занят задачей 1? Нет: 1 не
    # получила своего, поэтому свободны оба, и задача 2 берёт воркера 4.
    assert plan_first_round(ordered, [4, 7], {"k1": 99, "k2": 4}) == {4: 1, 7: 0}
    # Две задачи просят одного воркера: достаётся более тяжёлой, вторая — свободному.
    assert plan_first_round(ordered, [4, 7], {"k1": 4, "k2": 4}) == {4: 0, 7: 1}


def test_the_plan_covers_every_worker_that_has_a_task_and_nobody_twice():
    ordered = _ordered(10, 5000, 900, 50, 60)
    plan = plan_first_round(ordered, [1, 2, 3], {"k3": 2})
    assert sorted(plan) == [1, 2, 3] and sorted(plan.values()) == [0, 1, 2]
    assert plan_first_round(ordered, [1, 2, 3, 4, 5, 6, 7], {}) == {1: 0, 2: 1, 3: 2, 4: 3, 5: 4}
    assert plan_first_round([], [1, 2], {}) == {}


def test_a_task_without_an_affinity_key_has_no_preference():
    ordered = _ordered(10, 5000, affinity=False)
    assert plan_first_round(ordered, [4, 7], {"": 7, "k0": 7}) == {4: 0, 7: 1}


def test_a_real_pool_records_who_computed_a_task_and_starts_it_there_next_time():
    """Настоящие воркеры: пул запоминает воркера задачи и подсказка выполняется. Задачи падают сразу — размещение видно по записи."""

    pool = DomainPool(2)
    try:
        pool.ensure_started()
        first, second = (worker.index for worker in pool._workers)
        tasks = [_task(0, 5000, "heavy"), _task(1, 900, "light")]
        pool.run(tasks)
        recorded = dict(pool._last_worker)
        assert set(recorded) == {"heavy", "light"} and set(recorded.values()) == {first, second}
        # Против записанного: каждая задача к воркеру ДРУГОЙ задачи; пул обязан выполнить подсказку, а не повторить прошлое.
        swapped = {"heavy": recorded["light"], "light": recorded["heavy"]}
        pool._last_worker.update(swapped)
        pool.run(tasks)
        assert pool._last_worker == swapped
        pool._last_worker.update(recorded)
        pool.run(tasks)
        assert pool._last_worker == recorded
    finally:
        pool.close()


# --------------------------------------------------------------------------
# 2. Метка и счётчики
# --------------------------------------------------------------------------


def _press(bundle, controller, alpha):
    return run_production(
        controller,
        bundle,
        frozenset(range(ROW)),
        alpha,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=0,
    )


def test_the_label_and_the_counters_name_a_clip_memo_hit_and_the_equality_ignores_the_label():
    from cftuv_envelope.materialize.clip_memo import MEMO

    MEMO.clear()
    MEMO.reset_stats()
    bundle = quad_row_bundle(ROW, lifted_corner=0.05)
    controller = EnvelopeDebugSessionController()
    # alpha 8 и 16: покрытие насыщено, резка домена с приподнятым углом та же (проверено в `kernel/tests/test_clip_memo.py`).
    first = _press(bundle, controller, 8.0)
    second = _press(bundle, controller, 16.0)
    lifted_first, lifted_second = first.results[ROW - 1], second.results[ROW - 1]
    assert (lifted_first.clip_memo, lifted_second.clip_memo) == ("MISS", "HIT")
    assert (first.counter(production.PRODUCTION_CLIP_MEMO_HITS), first.counter(production.PRODUCTION_CLIP_MEMO_MISSES)) == (0, 1)
    assert (second.counter(production.PRODUCTION_CLIP_MEMO_HITS), second.counter(production.PRODUCTION_CLIP_MEMO_MISSES)) == (1, 0)
    assert "clip memo 1 hits, 0 misses" in production.production_timing_text(second)
    assert "clip memo" in production.production_timing_text(first)
    # Метка запуска, а не ответ: ни равенство, ни дайджесты её не видят.
    assert dataclasses.replace(lifted_second, clip_memo="") == lifted_second
    assert dataclasses.replace(lifted_second, clip_memo="MISS").content_digest == lifted_second.content_digest
    # Домены без резки метки не несут.
    assert all(item.clip_memo == "" for item in second.results[: ROW - 1])


def test_a_result_from_the_session_cache_carries_no_label_of_a_stage_it_did_not_run():
    from cftuv_envelope.materialize.clip_memo import MEMO

    MEMO.clear()
    bundle = quad_row_bundle(ROW, lifted_corner=0.05)
    controller = EnvelopeDebugSessionController()
    _press(bundle, controller, 8.0)
    again = _press(bundle, controller, 8.0)
    assert all(item.placement == production.PLACEMENT_CACHED and item.clip_memo == "" for item in again.results)
    assert (again.counter(production.PRODUCTION_CLIP_MEMO_HITS), again.counter(production.PRODUCTION_CLIP_MEMO_MISSES)) == (0, 0)
    assert "clip memo" not in production.production_timing_text(again)


# --------------------------------------------------------------------------
# 3. Отпечаток кода
# --------------------------------------------------------------------------


def test_the_kernel_code_identity_is_the_fingerprint_the_host_and_the_installer_use():
    import cftuv_envelope
    from cftuv_envelope.materialize.clip_memo import kernel_code_identity

    assert kernel_code_identity() == package_fingerprint(os.path.dirname(cftuv_envelope.__file__))
