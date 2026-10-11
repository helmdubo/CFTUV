"""Умолчание числа воркеров (POOL_DEFAULT_12_V1) и название размещения задач пула относительно прошлого прогона. Хост без Blender.

1. Умолчание настройки `envelope_debug_workers` - 12 (число физических ядер машины владельца), но не больше числа логических ядер; настройка сцены
   остаётся за владельцем (умолчание только там, где она не записана).
2. `PoolStatsV1.tasks_to_owner / tasks_unowned / tasks_stolen` называют, куда легла каждая задача с ключом привязки: воркеру прошлого прогона, без
   живого владельца либо чужому (кража в хвосте прогона). Это ИЗМЕРЕНИЕ, а не политика: очередь «владелец первым» (POOL_STICKY_OWNER_FIRST_V1) измерена на
   настоящем пуле и отвергнута (`NO_CERTIFICATE` не снизился), очередь осталась прежней (`test_envelope_worker_store`: воркер берёт задачу, чью подготовку держит).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv.envelope_domain_pool import DEFAULT_POOL_WORKERS, DomainPool, DomainTaskV1, shutdown_domain_pool  # noqa: E402


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


def _task(task_id, weight, affinity=None):
    return DomainTaskV1(
        task_id, task_id, f"d{task_id}", "x" * weight, None, "0.25", frozenset(), affinity=f"k{task_id}" if affinity is None else affinity
    )


def test_the_default_worker_count_is_twelve_but_never_above_the_logical_cores():
    assert DEFAULT_POOL_WORKERS == min(12, os.cpu_count() or 1)
    assert 1 <= DEFAULT_POOL_WORKERS <= 12


def test_a_real_pool_names_where_every_keyed_task_went_relative_to_the_previous_run():
    """Задачи падают сразу (снапшот - строка): размещение видно по статистике прогона, а не по ответам."""

    pool = DomainPool(2)
    try:
        pool.ensure_started()
        tasks = [_task(index, 100 + 400 * index) for index in range(6)]
        first = pool.run(tasks).stats
        assert (first.tasks_to_owner, first.tasks_unowned, first.tasks_stolen) == (0, 6, 0)  # истории нет: все ничьи
        second = pool.run(tasks).stats
        assert second.tasks_unowned == 0 and second.tasks_to_owner + second.tasks_stolen == 6
        assert second.tasks_to_owner >= 2  # первый круг у владельцев всегда (`plan_first_round`)
        keyless = pool.run([_task(index, 100, affinity="") for index in range(3)]).stats
        assert keyless.tasks_to_owner + keyless.tasks_unowned + keyless.tasks_stolen == 0  # без ключа привязки размещение не называется
    finally:
        pool.close()
