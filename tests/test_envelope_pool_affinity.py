"""Воркеры пула на первых потоках физических ядер (POOL_WORKERS_ON_PRIMARY_THREADS_V1): когда маска ставится, когда нет и что об этом остаётся.

Утверждение одно: привязка решает, КАКОЙ процессор исполнит тот же код, и больше ничего. Поэтому здесь не ускорение (замер лежит в `DECISIONS.md`), а ЗАКОН выбора: маска ставится только на машине, где она что-то даёт
и ничего не отнимает (SMT-пары, один класс ядер, воркеров не больше ядер, родитель не ограничен уже), а на любой другой воркеры идут без неё, и причина - один числовой код прогона, без строк и отказов.
Топологию подставляет тест; одна проверка идёт на настоящей Windows и ставит маску настоящему процессу.
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_domain_pool as pool_module  # noqa: E402
from cftuv import envelope_pool_affinity as affinity  # noqa: E402
from cftuv.envelope_domain_pool import DomainPool, DomainPoolRunV1, DomainTaskV1, shutdown_domain_pool  # noqa: E402
from cftuv.envelope_pool_affinity import (  # noqa: E402
    AFFINITY_APPLIED,
    AFFINITY_APPLY_FAILED,
    AFFINITY_DISABLED,
    AFFINITY_MORE_WORKERS_THAN_CORES,
    AFFINITY_NOT_EVALUATED,
    AFFINITY_NOT_SMT_PAIRS,
    AFFINITY_NOT_WINDOWS,
    AFFINITY_PARENT_MASK_TOO_NARROW,
    AFFINITY_TOPOLOGY_UNKNOWN,
    apply_worker_affinity,
    plan_worker_mask,
)


@pytest.fixture(autouse=True)
def _no_pool_outlives_the_test():
    yield
    shutdown_domain_pool()


@pytest.fixture
def _on_windows(monkeypatch):
    """Закон выбора - чистая функция: платформу и окружение подставляет тест, а не машина, на которой он идёт."""

    monkeypatch.setattr(affinity, "_windows", lambda: True)
    monkeypatch.delenv(affinity.ENVIRONMENT_VARIABLE, raising=False)


def _smt_cores(count, efficiency=0, group=0):
    """`count` физических ядер по два логических процессора: (0,1), (2,3), ..."""

    return [(0b11 << (2 * index), efficiency, (group,)) for index in range(count)]


ALL = (1 << 24) - 1
PRIMARY_12 = 0x555555


# --------------------------------------------------------------------------
# 1. Закон выбора
# --------------------------------------------------------------------------


def test_smt_pairs_and_enough_cores_give_the_first_thread_of_every_core(_on_windows):
    assert plan_worker_mask(_smt_cores(12), ALL, 12) == (PRIMARY_12, AFFINITY_APPLIED)
    # меньше воркеров - та же маска: выбор процессора внутри неё остаётся планировщику (закрепление по одному на ядро замером хуже)
    assert plan_worker_mask(_smt_cores(12), ALL, 8) == (PRIMARY_12, AFFINITY_APPLIED)


@pytest.mark.parametrize(
    "cores, parent, workers, code",
    (
        pytest.param(None, ALL, 12, AFFINITY_TOPOLOGY_UNKNOWN, id="topology-unreadable"),
        pytest.param(_smt_cores(12), None, 12, AFFINITY_TOPOLOGY_UNKNOWN, id="parent-mask-unreadable"),
        pytest.param([(1 << index, 0, (0,)) for index in range(12)], (1 << 12) - 1, 12, AFFINITY_NOT_SMT_PAIRS, id="smt-off"),
        pytest.param(_smt_cores(6) + [(0b11 << (12 + 2 * index), 1, (0,)) for index in range(6)], ALL, 12, AFFINITY_NOT_SMT_PAIRS, id="hybrid-classes"),
        pytest.param([(0b11, 0, (0,)), (0b111, 0, (0,))], 0b11111, 2, AFFINITY_NOT_SMT_PAIRS, id="a-core-of-three"),
        pytest.param(_smt_cores(4) + [(0b11, 0, (1,))], ALL, 4, AFFINITY_NOT_SMT_PAIRS, id="second-processor-group"),
        pytest.param(_smt_cores(12), ALL, 16, AFFINITY_MORE_WORKERS_THAN_CORES, id="sixteen-workers-on-twelve-cores"),
        pytest.param(_smt_cores(12), 0x0000FF, 12, AFFINITY_PARENT_MASK_TOO_NARROW, id="parent-already-restricted"),
    ),
)
def test_any_other_machine_gets_no_mask_and_a_named_code(_on_windows, cores, parent, workers, code):
    assert plan_worker_mask(cores, parent, workers) == (0, code)


def test_a_restricted_parent_passes_on_only_what_it_may_use(_on_windows):
    # родителю разрешены процессоры 0..11 (шесть ядер): воркерам - первые потоки этих шести
    assert plan_worker_mask(_smt_cores(12), 0xFFF, 6) == (0b010101010101, AFFINITY_APPLIED)
    assert plan_worker_mask(_smt_cores(12), 0xFFF, 7) == (0, AFFINITY_PARENT_MASK_TOO_NARROW)


def test_the_owner_switches_it_off_by_the_environment_and_other_platforms_are_named(monkeypatch, _on_windows):
    monkeypatch.setenv(affinity.ENVIRONMENT_VARIABLE, " off ")
    assert plan_worker_mask(_smt_cores(12), ALL, 12) == (0, AFFINITY_DISABLED)
    monkeypatch.setenv(affinity.ENVIRONMENT_VARIABLE, "ON")
    assert plan_worker_mask(_smt_cores(12), ALL, 12) == (PRIMARY_12, AFFINITY_APPLIED)
    monkeypatch.setattr(affinity, "_windows", lambda: False)
    assert plan_worker_mask(_smt_cores(12), ALL, 12) == (0, AFFINITY_NOT_WINDOWS)


# --------------------------------------------------------------------------
# 2. Применение: каждому процессу ТА ЖЕ маска, отказ назван, без маски никому ничего
# --------------------------------------------------------------------------


def test_the_same_mask_goes_to_every_worker_and_a_refusal_names_the_code_without_undoing_the_others(monkeypatch, _on_windows):
    monkeypatch.setattr(affinity, "physical_core_masks", lambda: _smt_cores(12))
    monkeypatch.setattr(affinity, "parent_affinity_mask", lambda: ALL)
    given = []
    monkeypatch.setattr(affinity, "set_process_affinity", lambda pid, mask: (given.append((pid, mask)), pid != 30)[1])

    assert apply_worker_affinity([10, 20], 12) == AFFINITY_APPLIED
    assert given == [(10, PRIMARY_12), (20, PRIMARY_12)]
    given.clear()
    assert apply_worker_affinity([10, 30, 40], 12) == AFFINITY_APPLY_FAILED
    assert given == [(10, PRIMARY_12), (30, PRIMARY_12), (40, PRIMARY_12)]  # отказ одному не отнимает маску у остальных
    given.clear()
    assert apply_worker_affinity([10, 20], 16) == AFFINITY_MORE_WORKERS_THAN_CORES
    assert given == []


# --------------------------------------------------------------------------
# 3. Пул: код привязки у пула, в итоге прогона и в счётчике профиля
# --------------------------------------------------------------------------

STUB_WORKER = """
pool = importlib.import_module(spec["package"] + ".envelope_domain_pool")
channel = os.fdopen(os.dup(1), "wb")
os.dup2(2, 1)
sys.stdout = sys.stderr
source = sys.stdin.buffer
pool.write_frame(channel, ("ready", os.getpid(), {}))
while True:
    task = pool.read_frame(source)
    if task is None:
        break
    pool.write_frame(channel, pool.DomainTaskResultV1(task.task_id, snapshot="ok"))
"""
LAST_LINE = 'importlib.import_module(spec["package"] + ".envelope_domain_pool").worker_main()'


def test_the_pool_asks_for_the_mask_once_per_start_with_the_ordered_worker_count_and_reports_the_code(monkeypatch):
    monkeypatch.setattr(pool_module, "_BOOTSTRAP", pool_module._BOOTSTRAP.replace(LAST_LINE, STUB_WORKER))
    calls = []
    monkeypatch.setattr(pool_module, "apply_worker_affinity", lambda pids, workers: (calls.append((len(pids), workers)), AFFINITY_APPLIED)[1])
    pool = DomainPool(2)
    try:
        assert pool.affinity == AFFINITY_NOT_EVALUATED
        pool.ensure_started()
        assert calls == [(2, 2)] and pool.affinity == AFFINITY_APPLIED
        pool.ensure_started()
        assert calls == [(2, 2)]  # воркеры живы: привязку заново не ставят
        run = pool.run([DomainTaskV1(0, 0, "d0", None, None, "0", frozenset(), cold="stub")])
        assert run.affinity == AFFINITY_APPLIED and run.results[0].snapshot == "ok"
        pool._workers[0].process.kill()
        pool._workers[0].process.wait()
        pool._workers[0].dead = True
        monkeypatch.setattr(pool_module, "apply_worker_affinity", lambda pids, workers: (calls.append((len(pids), workers)), AFFINITY_MORE_WORKERS_THAN_CORES)[1])
        pool.ensure_started()
        assert calls[-1] == (1, 2) and pool.affinity == AFFINITY_MORE_WORKERS_THAN_CORES  # докомплектация: маска тому, кто стартовал
    finally:
        pool.close()


def test_the_run_counter_carries_the_code_and_a_pool_less_run_carries_zero():
    from cftuv.envelope_queue_export import POOL_WORKER_AFFINITY
    from cftuv.envelope_queue_pool import _record_pool_counters

    class Profile:
        def __init__(self):
            self.counters = {}

        def set_counter(self, name, value):
            self.counters[name] = value

    profile = Profile()
    _record_pool_counters(profile, DomainPoolRunV1({}, 3, None, None, AFFINITY_NOT_SMT_PAIRS), "", 0, 0, 0)
    assert profile.counters[POOL_WORKER_AFFINITY] == AFFINITY_NOT_SMT_PAIRS
    profile = Profile()
    _record_pool_counters(profile, None, "unavailable", 0, 0, 0)
    assert profile.counters[POOL_WORKER_AFFINITY] == AFFINITY_NOT_EVALUATED


# --------------------------------------------------------------------------
# 4. Настоящая Windows
# --------------------------------------------------------------------------


@pytest.mark.skipif(sys.platform != "win32", reason="affinity is a Windows API")
def test_a_real_process_gets_the_planned_mask_or_keeps_its_own_and_the_topology_is_sane():
    cores = affinity.physical_core_masks()
    if cores is not None:
        assert cores and all(mask and bin(mask).count("1") >= 1 for mask, _efficiency, _groups in cores)
        assert len({mask for mask, _efficiency, _groups in cores}) == len(cores)  # ядра не пересекаются
        union = 0
        for mask, _efficiency, _groups in cores:
            assert union & mask == 0
            union |= mask
    parent = affinity.parent_affinity_mask()
    assert parent
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    try:
        before = affinity.process_affinity_mask(child.pid)
        assert before == parent  # ребёнок наследует маску родителя
        code = apply_worker_affinity([child.pid], 1)
        after = affinity.process_affinity_mask(child.pid)
        if code == AFFINITY_APPLIED:
            mask, planned = plan_worker_mask(cores, parent, 1)
            assert planned == AFFINITY_APPLIED and after == mask and after & ~parent == 0
            assert affinity.process_affinity_mask(os.getpid()) == parent  # родитель маски не получил
        else:
            assert after == before and code != AFFINITY_NOT_EVALUATED
    finally:
        child.kill()
        child.wait()
