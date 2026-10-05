"""Продуктовый путь Envelope (хост без Blender): сессия, пул, названные исходы.

Утверждений четыре, и каждое стоит на числе:

1. ПОДГОТОВКА НЕ ПЕРЕСТРАИВАЕТСЯ. Закон UV — параметр материализации, а не
   часть ключа кэша подготовок: нажатие продукта после отладочной кнопки берёт
   те же подготовки (счётчики сборок контроллера не растут), а холодное
   нажатие наполняет кэши так же, как отладка (ключи и счёт сборок те же), но
   ОДНОЙ задачей на домен: подготовка и материализация, без отладочного прогона.
   Результаты доменов кэшируются в сессии: то же нажатие не считает ничего, а
   правка выделения пересчитывает только затронутые домены.
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
    PLACEMENT_CACHED,
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
from envelope_fixture_bundles import pin_near_planar_only, quad_row_bundle  # noqa: E402

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
            self.kinds.append(
                "production"
                if task.production is not None
                else "cold"
                if task.cold is not None
                else "other"
            )
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


def test_the_snapshot_is_validated_once_per_session_not_on_every_press(monkeypatch):
    """Снапшот домена от alpha не зависит: замечания к нему — память сессии, а не работа каждого нажатия."""

    import cftuv_envelope as kernel
    import cftuv_envelope.validation as validation

    explicit, internal = [], []
    real_explicit, real_internal = kernel.validate_analysis_snapshot, validation.validate_analysis_snapshot
    monkeypatch.setattr(
        kernel,
        "validate_analysis_snapshot",
        lambda item, **kwargs: explicit.append(item) or real_explicit(item, **kwargs),
    )
    monkeypatch.setattr(
        validation,
        "validate_analysis_snapshot",
        lambda item, **kwargs: internal.append(item) or real_internal(item, **kwargs),
    )
    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    first, _ = _production(bundle, controller, alpha=0.25)
    # Холодное нажатие проверяет снапшоты (в том числе в выгрузке и подготовке); тёплое с другой alpha — ни одного.
    after_first = (len(explicit), len(internal))
    assert after_first[0] >= ROW
    second, _ = _production(bundle, controller, alpha=0.5)
    assert (len(explicit), len(internal)) == after_first
    assert [item.outcome for item in second.results] == [item.outcome for item in first.results]
    # Память живёт с сессией: после `clear()` снапшот проверяется заново.
    snapshot = explicit[0]
    assert controller.snapshot_issues(snapshot) is controller.snapshot_issues(snapshot)
    controller.clear()
    assert not controller._snapshot_issues


def test_the_snapshot_issues_memory_is_keyed_by_the_requests_stretch_budget(monkeypatch):
    """Замечания к снапшоту зависят от допуска растяжения запроса: один снапшот под другим допуском не берётся из памяти."""

    from fractions import Fraction

    import cftuv_envelope as kernel

    controller = EnvelopeDebugSessionController()
    _production(quad_row_bundle(ROW), controller)
    snapshot = next(iter(controller._snapshot_issues.values()))[0]
    seen = []
    real = kernel.validate_analysis_snapshot
    monkeypatch.setattr(
        kernel,
        "validate_analysis_snapshot",
        lambda item, **kwargs: seen.append(kwargs.get("developable_stretch_budget")) or real(item, **kwargs),
    )
    wide = Fraction(7, 20)
    first = controller.snapshot_issues(snapshot, wide)
    assert seen == [wide]
    assert controller.snapshot_issues(snapshot, wide) is first and seen == [wide]
    controller.snapshot_issues(snapshot, Fraction(1, 10))
    assert seen == [wide, Fraction(1, 10)]


def test_the_snapshot_issues_memory_is_bounded_and_evicts_the_oldest(monkeypatch):
    from cftuv import envelope_debug_session as session_module

    monkeypatch.setattr(session_module, "SNAPSHOT_ISSUES_CACHE_LIMIT", 3)
    controller = EnvelopeDebugSessionController()
    bundle = quad_row_bundle(ROW)
    _production(bundle, controller)
    assert len(controller._snapshot_issues) <= 3
    held = [item[0] for item in controller._snapshot_issues.values()]
    assert len(held) == len({id(item) for item in held})


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
    # Холодная сессия: каждый домен — одна холодная задача (подготовка и материализация),
    # а отладочного прогона и пикла готовой подготовки нет вовсе.
    assert pool.kinds == ["cold"] * ROW
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
    # Тот же alpha и те же рёбра: результаты лежат в кэше сессии, воркеры не нужны.
    same, _ = _production(bundle, controller, workers=2)
    assert [item.placement for item in same.results] == [PLACEMENT_CACHED] * ROW
    assert _answers(same) == _answers(expected)
    # Другой alpha на живых воркерах: подготовки взяты из кэша сессии (пиклом), покрытие
    # и материализация посчитаны заново.
    far, _ = _production(bundle, controller, workers=2, alpha=0.5)
    assert _counter(far, production.PRODUCTION_PREPARATION_BUILDS) == 0
    assert [item.placement for item in far.results] == [PLACEMENT_WORKER] * ROW
    assert _answers(far) == _answers(_production(bundle, workers=0, alpha=0.5)[0])


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
    # Другой alpha: результаты прежнего лежали бы в кэше, и до пула дело не дошло бы.
    expected, _ = _production(bundle, workers=0, alpha=0.5)
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

    run, _ = _production(bundle, controller, workers=2, alpha=0.5)

    assert _counter(run, POOL_TASK_FALLBACK) == 1
    assert [item.placement for item in run.results].count(PLACEMENT_FALLBACK) == 1
    assert "cannot be shipped" in capsys.readouterr().out
    assert _answers(run) == _answers(expected)
    assert envelope_queue_pool.COVERAGE_POOL_MIN_BYTES == 0


def test_a_domain_refused_on_the_metric_is_named_with_its_host_outcome(monkeypatch):
    pin_near_planar_only(monkeypatch)
    bundle = quad_row_bundle(ROW, lifted_corner=1.0)

    run, _ = _production(bundle)

    refused = [item for item in run.results if not item.is_materialized]
    assert [item.patch_id for item in refused] == [ROW - 1]
    assert refused[0].outcome == "NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED"
    assert refused[0].batch is None and refused[0].detail
    assert production_status_text(run.results) == (
        f"MATERIALIZED {ROW - 1} / refused 1 (NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED)"
    )
    lines = production_console_lines(run.results)
    assert len(lines) == 2
    assert f"patch {ROW - 1}" in lines[0]
    assert "NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED" in lines[0]
    assert lines[1].endswith(production_status_text(run.results))
    assert _counter(run, production.PRODUCTION_REFUSED) == 1
    assert _counter(run, production.PRODUCTION_MATERIALIZED) == ROW - 1


def test_a_near_planar_domain_is_materialized_onto_the_source_triangles():
    """Хост просит укладку на треугольники источника (NEAR_PLANAR V2).

    Приподнятый на 5 см угол последнего квадрата — за прежним абсолютным
    бюджетом юбки (1.25 см), но ширина в бюджете: домен строится, а меш лежит
    на поверхности. Планарные соседи укладку не затрагивают: ни счётчиков
    подъёма, ни диагностики near-planar у них нет.
    """

    bundle = quad_row_bundle(ROW, lifted_corner=0.05)
    run, _ = _production(bundle)

    assert all(item.is_materialized for item in run.results)
    lifted = run.results[ROW - 1]
    assert any(
        line.startswith("NEAR_PLANAR_LIFT_ONTO_SOURCE_TRIANGLES")
        for line in lifted.diagnostics
    ), lifted.diagnostics
    counters = dict(lifted.counters)
    # Кнопка просит укладку с РЕЗКОЙ граней: вершины `clip:` поднимает сама резка (в треугольнике, где они
    # родились), поэтому нахождений столько, сколько прочих вершин батча.
    assert (
        counters["MATERIALIZE_SURFACE_LIFT_LOCATIONS"] + counters["MATERIALIZE_CLIP_VERTICES_INSERTED"]
        == len(lifted.batch.vertices)
    )
    assert counters["MATERIALIZE_CLIP_VERTICES_INSERTED"] >= 1
    assert any(line.startswith("SOURCE_EDGES_LIFTED_ONTO_SURFACE") for line in lifted.diagnostics)
    assert counters["MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES"] == 0
    assert counters["MATERIALIZE_SURFACE_LIFT_TRIANGLES"] > 0
    for item in run.results[:-1]:
        assert not any("SURFACE_LIFT" in name for name, _ in item.counters)
        assert not any("NEAR_PLANAR" in line for line in item.diagnostics)


def test_an_unfolded_domain_is_materialized_with_a_vertex_offset_normal_law():
    """Изогнутый квад (лестница S1) материализуется; смещение — по нормали ВЕРШИНЫ.

    У развёртки нет плоскости источника: `normal` результата — лишь сводка (среднее нормалей
    вершин), а писатель меша кладёт смещение по нормали каждой вершины батча.
    """

    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    run, _ = _production(bundle)

    assert all(item.is_materialized for item in run.results)
    unfolded = run.results[ROW - 1]
    assert unfolded.offset_normal_law == "SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1"
    keys = {item.vert_key.value for item in unfolded.batch.vertices}
    assert {name for name, _ in unfolded.vertex_normals} == keys
    assert any(line.startswith("DEVELOPABLE_LIFT_ONTO_UNFOLDED") for line in unfolded.diagnostics)
    assert any(line.startswith("DEVELOPABLE_OFFSET_MIN_GAP_COSINE") for line in unfolded.diagnostics)
    for item in run.results[:-1]:
        assert item.vertex_normals == () and item.offset_normal_law == ""
        assert item.offset_normals_digest == ""
    length = sum(axis * axis for axis in unfolded.normal) ** 0.5
    assert abs(length - 1.0) < 1e-12


def test_the_console_shows_how_much_of_the_stretch_budget_an_unfolded_domain_used():
    """Решение владельца «растяжения до 20 %»: артист видит, сколько растяжения на деле израсходовала развёртка.

    Домен берётся целиком из настоящего пути (изогнутый квад лестницы S1), число читается из его диагностики
    независимо от разбора хоста и округляется вверх до десятой: строка «не больше» не врёт.
    """

    import math
    import re
    from types import SimpleNamespace

    from cftuv_envelope.contracts.metric import DEFAULT_DEVELOPABLE_STRETCH_BUDGET

    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    run, _ = _production(bundle)
    unfolded = run.results[ROW - 1]
    line = next(item for item in unfolded.diagnostics if item.startswith("DEVELOPABLE_LIFT_ONTO_UNFOLDED"))
    band = float(re.search(r"worst_band_squared<=([0-9.e+-]+)", line).group(1))
    budget = float(DEFAULT_DEVELOPABLE_STRETCH_BUDGET) * 100.0
    expected = math.ceil((math.sqrt(band) - 1.0) * 1000.0 - 1e-9) / 10.0
    assert budget == pytest.approx(20.0)

    lines = production.developable_stretch_lines(run.results)
    assert lines == [
        f"[CFTUV][Production] STRETCH patch {unfolded.patch_id} (domain ...{unfolded.domain_id[-6:]}): "
        f"stretch <= {expected:.1f} % (budget 20 %, chart HINGE)"
    ]
    receipt = SimpleNamespace(skipped=(), warnings=(), domains=(), weld_counters=(), offset_counters=())
    assert lines[0] in production.receipt_console_lines(receipt, run.results)
    # Планарные соседи растяжения не имеют: строки у них нет.
    assert not production.developable_stretch_lines(run.results[:-1])


def test_the_stretch_line_rounds_up_and_names_the_largest_of_several_domains():
    from types import SimpleNamespace

    def domain(patch, band, budget="0.2", chart="HINGE"):
        text = (
            f"DEVELOPABLE_LIFT_ONTO_UNFOLDED_SOURCE_TRIANGLES: worst_band_squared<={band} "
            f"stretch_budget={budget} triangles_measured=3 proposal={chart} "
            "proposal_selection=BEST_ARAP_WON_V1 hinge_band_squared<=1.2 arap_band_squared<=1.0001 arap_refusal=none"
        )
        return SimpleNamespace(patch_id=patch, domain_id=f"domain-{patch:06d}", diagnostics=(text, "OTHER: x"))

    results = [
        domain(7, "1.44"),
        domain(3, "1.0001", chart="ARAP"),
        SimpleNamespace(patch_id=1, domain_id="d1", diagnostics=()),
    ]
    assert production.developable_stretch_lines(results) == [
        "[CFTUV][Production] STRETCH patch 3 (domain ...000003): stretch <= 0.1 % (budget 20 %, chart ARAP)",
        "[CFTUV][Production] STRETCH patch 7 (domain ...000007): stretch <= 20.0 % (budget 20 %, chart HINGE)",
        "[CFTUV][Production] STRETCH: 2 developable domains, the largest stretch <= 20.0 % "
        "(patch 7, budget 20 %, chart HINGE)",
    ]
    assert production.developable_stretch_lines([]) == []
    # Бюджет читается из записи ядра, а не из константы хоста: допуск запроса 35 % виден в строке.
    assert production.developable_stretch_lines([domain(5, "1.69", budget="0.35")]) == [
        "[CFTUV][Production] STRETCH patch 5 (domain ...000005): stretch <= 30.0 % (budget 35 %, chart HINGE)"
    ]


def test_the_offset_normals_are_visible_to_the_equality_and_to_the_json_line(tmp_path):
    """Нормали смещают вершины меша, но в дайджест батча не входят: у них свой дайджест."""

    import dataclasses
    import json

    from cftuv_envelope.materialize.offset_normal import offset_normals_digest

    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    run, _ = _production(bundle)
    unfolded = run.results[ROW - 1]
    assert unfolded.offset_normals_digest == offset_normals_digest(unfolded.vertex_normals)
    assert len(unfolded.offset_normals_digest) == 64
    # Результат с другим дайджестом нормалей — другой результат (равенство прогонов его видит).
    assert dataclasses.replace(unfolded, offset_normals_digest="x") != unfolded
    summary = production.export_production_json(run.results, tmp_path, label="row")
    rows = {item["patch_id"]: item for item in json.loads(summary.read_text(encoding="utf-8"))["domains"]}
    assert rows[unfolded.patch_id]["offset_normals_digest"] == unfolded.offset_normals_digest
    assert rows[unfolded.patch_id]["offset_normal_law"] == "SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1"
    assert all(
        row["offset_normals_digest"] == ""
        for patch, row in rows.items()
        if patch != unfolded.patch_id
    )


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
        detail="INNER_REASON: the predicate that was not proven",
    )

    refused = produce_domain(3, "domain", prepared, "0.25")

    assert refused.outcome == "COVERAGE_IS_NOT_EXACT"
    assert refused.detail.startswith("preparation:")
    # Строка отказа в консоли несёт внутреннюю причину подготовки, а не только её имя.
    assert refused.detail.endswith("INNER_REASON: the predicate that was not proven")
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
# 4. Холодный домен — одна задача; результаты доменов лежат в сессии
# --------------------------------------------------------------------------


def _edges(*extra):
    return frozenset(range(ROW)) | frozenset(extra)


def _press(bundle, controller, edges, *, workers=0, alpha=ALPHA, **kwargs):
    return run_production(
        controller,
        bundle,
        edges,
        alpha,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=workers,
        **kwargs,
    )


def _hits(run):
    return _counter(run, production.PRODUCTION_RESULT_CACHE_HIT)


def _misses(run):
    return _counter(run, production.PRODUCTION_RESULT_CACHE_MISS)


def test_a_cold_press_never_runs_the_debug_evaluator(monkeypatch):
    """Холодный домен — подготовка и материализация одной задачей, без отладочного прогона.

    Прежний холодный путь гнал отладочный вычислитель по всем доменам (подготовка, покрытие,
    контур) и считал покрытие второй раз. Вычислитель, который падает при вызове, это
    доказывает: прогон проходит, а кэши наполнены так же, как после кнопки отладки.
    """

    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()

    def forbidden(*_args, **_kwargs):
        raise AssertionError("the product press must not run the debug evaluator")

    monkeypatch.setattr(controller, "evaluate_staged", forbidden)

    run, _ = _production(bundle, controller)

    assert run.cold and all(item.is_materialized for item in run.results)
    assert controller.build_counts["CONVEYOR_PREPARATION"] == ROW
    assert controller.build_counts["PATCH_METRIC"] == ROW
    assert controller.build_counts["DOMAIN_GEOMETRY"] == ROW


def test_a_cold_answer_is_the_answer_of_a_domain_on_a_cached_preparation():
    bundle = quad_row_bundle(ROW)
    cold, _ = _production(bundle)
    controller = EnvelopeDebugSessionController()
    _debug_build(bundle, controller)

    warm, _ = _production(bundle, controller)

    assert cold.cold and not warm.cold
    assert _answers(cold) == _answers(warm)


def test_the_same_press_is_served_from_the_result_cache_and_computes_nothing(monkeypatch):
    bundle = quad_row_bundle(ROW)
    first, controller = _production(bundle)
    builds = controller.build_counts
    assert (_hits(first), _misses(first)) == (0, ROW)
    pool = _InProcessPool()
    _in_process_session(monkeypatch, pool)

    again, _ = _production(bundle, controller, workers=2)

    assert _answers(again) == _answers(first)
    assert [item.placement for item in again.results] == [PLACEMENT_CACHED] * ROW
    assert (_hits(again), _misses(again)) == (ROW, 0)
    assert not again.cold
    assert controller.build_counts == builds
    # Пул не спрашивали вовсе: считать нечего, и в профиле нет ни одной задачи.
    assert pool.kinds == []
    assert _counter(again, POOL_DISPATCHED) == 0
    assert all(item.seconds == 0.0 for item in again.results)
    assert "results cached 5, computed 0" in production.production_timing_text(again)
    assert "results cached 0, computed 5" in production.production_timing_text(first)


def test_another_alpha_or_another_law_is_another_key_and_the_preparations_stay():
    bundle = quad_row_bundle(ROW)
    first, controller = _production(bundle)
    laws = sorted(production.PRODUCTION_TOPOLOGY_LAWS - {production.PRODUCTION_TOPOLOGY_LAW})
    assert laws

    wider, _ = _production(bundle, controller, alpha=0.5)
    other_law, _ = _production(bundle, controller, topology_law=laws[0])

    for run in (wider, other_law):
        assert (_hits(run), _misses(run)) == (0, ROW)
        assert _counter(run, production.PRODUCTION_PREPARATION_BUILDS) == 0
        assert not run.cold
    assert [item.content_digest for item in wider.results] != [
        item.content_digest for item in first.results
    ]
    assert {item.decal_topology_law for item in other_law.results} == {laws[0]}
    # Прежний ключ жив: тот же alpha и закон — снова из кэша.
    back, _ = _production(bundle, controller)
    assert (_hits(back), _misses(back)) == (ROW, 0)
    assert _answers(back) == _answers(first)


def test_a_selection_edit_recomputes_only_the_domains_it_touches():
    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    _press(bundle, controller, _edges())

    top = _press(bundle, controller, _edges(ROW + 2))
    seam = _press(bundle, controller, _edges(2 * ROW + 1))
    both = _press(bundle, controller, _edges(ROW + 2, 2 * ROW + 1))

    # Верхнее ребро патча 2 меняет выделение ОДНОГО домена: он считается заново, остальные — из кэша.
    assert (_hits(top), _misses(top)) == (ROW - 1, 1)
    assert _counter(top, production.PRODUCTION_PREPARATION_BUILDS) == 1
    assert [item.placement for item in top.results].count(PLACEMENT_CACHED) == ROW - 1
    assert top.results[2].placement == "parent"
    # Шов между патчами 0 и 1 касается двух доменов.
    assert (_hits(seam), _misses(seam)) == (ROW - 2, 2)
    assert _counter(seam, production.PRODUCTION_PREPARATION_BUILDS) == 2
    # Оба правки вместе: каждый домен уже считался под своим выделением, и считать нечего.
    assert (_hits(both), _misses(both)) == (ROW, 0)
    assert _counter(both, production.PRODUCTION_PREPARATION_BUILDS) == 0


def test_the_result_cache_is_bounded_and_forgets_the_least_recently_used(monkeypatch):
    from cftuv import envelope_debug_session

    monkeypatch.setattr(envelope_debug_session, "PRODUCTION_RESULT_CACHE_LIMIT", 3)
    bundle = quad_row_bundle(ROW)
    first, controller = _production(bundle)

    assert controller.production_result_count == 3
    # Остались три последних домена; первые два вытеснены и считаются заново.
    again, _ = _production(bundle, controller)

    assert (_hits(again), _misses(again)) == (3, 2)
    assert _answers(again) == _answers(first)
    assert [item.placement for item in again.results] == [
        "parent",
        "parent",
        PLACEMENT_CACHED,
        PLACEMENT_CACHED,
        PLACEMENT_CACHED,
    ]
    assert controller.production_result_count == 3


def test_a_session_reset_drops_the_cached_results():
    bundle = quad_row_bundle(ROW)
    _first, controller = _production(bundle)
    assert controller.production_result_count == ROW

    controller.clear()

    assert controller.production_result_count == 0
    again, _ = _production(bundle, controller)
    assert again.cold and (_hits(again), _misses(again)) == (0, ROW)


def test_an_exception_inside_a_domain_is_named_and_never_cached(monkeypatch):
    from cftuv_envelope.materialize import domain as kernel_domain

    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    original = kernel_domain.materialize_domain

    def broken(*_args, **_kwargs):
        raise RuntimeError("kernel bug")

    monkeypatch.setattr(kernel_domain, "materialize_domain", broken)
    raised, _ = _production(bundle, controller)

    assert {item.outcome for item in raised.results} == {OUTCOME_DOMAIN_RAISED}
    assert controller.production_result_count == 0
    # Исключение — не ответ: после починки домены считаются заново, а не берутся из кэша.
    monkeypatch.setattr(kernel_domain, "materialize_domain", original)
    fixed, _ = _production(bundle, controller)

    assert (_hits(fixed), _misses(fixed)) == (0, ROW)
    assert all(item.is_materialized for item in fixed.results)
    # Подготовки при этом уже были собраны исключённой прогонкой и взяты из кэша.
    assert _counter(fixed, production.PRODUCTION_PREPARATION_BUILDS) == 0


@pytest.mark.parametrize("injection", ("fail", "drop"))
def test_a_failed_cold_task_is_named_counted_and_computed_in_the_parent(
    injection, monkeypatch, capsys, _pool_always
):
    bundle = quad_row_bundle(ROW)
    expected, _ = _production(bundle, workers=0)
    _in_process_session(monkeypatch, _InProcessPool(**{injection: (2,)}))
    capsys.readouterr()

    run, controller = _production(bundle, workers=2)

    assert _counter(run, POOL_TASK_FALLBACK) == 1
    assert _counter(run, POOL_DISPATCHED) == ROW
    placements = [item.placement for item in run.results]
    assert placements.count(PLACEMENT_FALLBACK) == 1
    assert placements.count(PLACEMENT_WORKER) == ROW - 1
    console = capsys.readouterr().out
    assert POOL_TASK_FALLBACK in console
    assert _answers(run) == _answers(expected)
    # Подготовка домена, который считал родитель, в кэше сессии так же, как у воркерских.
    assert controller.build_counts["CONVEYOR_PREPARATION"] == ROW
    assert controller.production_result_count == ROW


def test_an_unavailable_pool_is_named_and_the_parent_computes_the_cold_domains(
    monkeypatch, capsys, _pool_always
):
    bundle = quad_row_bundle(ROW)
    expected, _ = _production(bundle, workers=0)
    _in_process_session(
        monkeypatch, DomainPool(2, python_executable="C:/nowhere/python.exe")
    )
    capsys.readouterr()

    run, _ = _production(bundle, workers=2)

    assert _counter(run, POOL_UNAVAILABLE) == 1
    assert _counter(run, POOL_DISPATCHED) == 0
    assert {item.placement for item in run.results} == {PLACEMENT_UNAVAILABLE}
    out = capsys.readouterr().out
    assert POOL_UNAVAILABLE in out
    assert _answers(run) == _answers(expected)


def test_a_metric_refusal_in_a_cold_worker_task_is_named_like_the_parents(
    monkeypatch, _pool_always
):
    pin_near_planar_only(monkeypatch)
    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    expected, _ = _production(bundle, workers=0)
    pool = _InProcessPool()
    _in_process_session(monkeypatch, pool)

    run, controller = _production(bundle, workers=2)

    assert _answers(run) == _answers(expected)
    refused = [item for item in run.results if not item.is_materialized]
    assert [item.outcome for item in refused] == ["NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED"]
    # Домен, чью выгрузку воркер отказал, воркеру «не уходил»: он не отправлен и не упал.
    assert _counter(run, POOL_DISPATCHED) == ROW - 1
    assert _counter(run, POOL_TASK_FALLBACK) == 0
    assert pool.kinds == ["cold"] * ROW
    # Отказ запомнен кэшем метрики сессии, и следующее нажатие называет его тем же исходом.
    again, _ = _production(bundle, controller, workers=2)
    assert _answers(again) == _answers(expected)
    assert (_hits(again), _misses(again)) == (ROW - 1, 0)


def test_the_cold_task_answers_in_process_with_the_preparation_and_the_snapshot(
    monkeypatch, _pool_always
):
    """Задача «холодный домен» целиком: ответ несёт подготовку, батч и (при выгрузке) снапшот."""

    bundle = quad_row_bundle(ROW)
    kinds: list = []

    class Recorder(_InProcessPool):
        def run(self, tasks):
            kinds.extend(task for task in tasks)
            return super().run(tasks)

    _in_process_session(monkeypatch, Recorder())

    run, _ = _production(bundle, workers=2)

    exported = [task for task in kinds if task.export is not None]
    assert len(exported) == ROW and all(task.cold is not None for task in kinds)
    reply = solve_task(exported[0])
    assert reply.ok and reply.error == "" and reply.queue_domain is None
    assert reply.prepared is not None and reply.snapshot is not None
    assert reply.production == run.results[exported[0].patch_id]
    assert reply.production.placement == PLACEMENT_WORKER
    assert any(item.stage == "QUEUE_PREPARE" for item in reply.export_timings)
    assert pickle.loads(pickle.dumps(reply)).production == reply.production


def test_cold_tasks_are_ranked_like_full_domains_and_not_like_coverage_only_tasks():
    from cftuv.envelope_production_export import ColdProductionInputV1

    cold = DomainTaskV1(
        0, 1, "x", "y" * 500_000, None, "0.25", frozenset(), cold=ColdProductionInputV1()
    )
    warm = DomainTaskV1(
        1, 2, "x", None, None, "0.25", frozenset(),
        production=ProductionInputV1(b"z" * 500_000),
    )

    ordered = order_by_cost([warm, cold])

    assert [task.task_id for task, _frame in ordered] == [0, 1]
    frame = dict((task.task_id, frame) for task, frame in ordered)
    assert pool_module._frame_cost(cold, frame[0]) == float(len(frame[0]))
    assert pool_module._frame_cost(warm, frame[1]) == len(frame[1]) / (
        pool_module.COVERAGE_FRAME_COST_DIVISOR
    )


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


def test_the_json_export_round_trips_every_batch_through_the_kernel_codec(tmp_path, monkeypatch):
    pin_near_planar_only(monkeypatch)
    from cftuv_envelope import GeometryBatchCodecV1

    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
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
    assert refused["outcome"] == "NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED"


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


# --------------------------------------------------------------------------
# Аудит среза 4: строка и консоль по квитанции, диагностики, свидетельства
# --------------------------------------------------------------------------


def test_a_produced_domain_carries_the_source_normal_and_the_chart_orientation():
    prepared = _prepared_domain()

    result = produce_domain(2, "domain", prepared, "0.25")

    assert result.is_materialized
    assert result.chart_orientation == prepared.context.frame.chart_orientation.value
    assert result.source_normal is not None
    dot = sum(a * b for a, b in zip(result.normal, result.source_normal))
    assert dot > 0.99


def test_the_batch_diagnostics_are_summarised_by_name_in_the_console():
    def result(patch, *lines):
        return production.ProductionDomainResultV1(
            patch, f"d{patch}", MATERIALIZED, object(), diagnostics=lines
        )

    lines = production.diagnostic_summary_lines(
        [
            result(3, "NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE: residual_budget=1"),
            result(1, "NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE: residual_budget=1"),
            result(2, "U_RESTARTS_AT_DOMAIN_BORDER: chain:a", "U_RESTARTS_AT_DOMAIN_BORDER: chain:b"),
            result(4),
        ]
    )

    assert lines == [
        "[CFTUV][Production] DIAGNOSTIC NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE: "
        "2 in 2 domains (patch 1, 3)",
        "[CFTUV][Production] DIAGNOSTIC U_RESTARTS_AT_DOMAIN_BORDER: "
        "2 in 1 domains (patch 2)",
    ]


#: Операторы, которые создают либо удаляют датаблоки, пока источник в EDIT-режиме.
#: Без шага отмены следующий Ctrl+Z переиспользует сцену как есть и освобождает
#: созданные объекты — висячий указатель, падение Blender (воспроизведено в UI 4.5).
EDIT_MODE_DATABLOCK_WRITERS = frozenset(
    {
        "hotspotuv.build_envelope_decal_mesh",
        "hotspotuv.build_envelope_topology_debug",
        "hotspotuv.build_exact_reference_envelope_debug",
        "hotspotuv.build_envelope_debug",
        "hotspotuv.clear_envelope_debug",
    }
)


def _operator_options_by_idname(*paths):
    """`bl_idname -> bl_options` по AST, с учётом примесей (`_EnvelopeDebugBuildBase`)."""

    import ast

    classes = {}
    for path in paths:
        for node in ast.parse(path.read_text(encoding="utf-8")).body:
            if isinstance(node, ast.ClassDef):
                classes[node.name] = node

    def attribute(node, name):
        for item in node.body:
            if isinstance(item, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id == name
                for target in item.targets
            ):
                try:
                    return ast.literal_eval(item.value)
                except ValueError:  # не литерал (`bl_idname = ADDON_PACKAGE`)
                    return None
        for base in node.bases:
            parent = classes.get(getattr(base, "id", None))
            found = None if parent is None else attribute(parent, name)
            if found is not None:
                return found
        return None

    found = {}
    for node in classes.values():
        idname = attribute(node, "bl_idname")
        if idname is not None:
            found[idname] = attribute(node, "bl_options") or set()
    return found


def test_operators_that_write_datablocks_from_edit_mode_declare_undo():
    """Шаг отмены после оператора заставляет Blender записать memfile с созданными ID.

    Иначе Ctrl+Z декодирует предыдущий memfile с переиспользованием «неизменившейся»
    сцены, которая всё ещё ссылается на освобождённый объект (см. `UNDO_REQUIRED_REASON`
    и запись DECISIONS от 2026-10-03).
    """

    from pathlib import Path

    package = Path(__file__).resolve().parents[1] / "cftuv"
    options = _operator_options_by_idname(
        package / "envelope_production_operator.py", package / "operators.py"
    )
    missing = sorted(EDIT_MODE_DATABLOCK_WRITERS - set(options))
    assert not missing, f"операторы не найдены: {missing}"
    without_undo = sorted(
        name for name in EDIT_MODE_DATABLOCK_WRITERS if "UNDO" not in options[name]
    )
    assert not without_undo, f"без флага UNDO: {without_undo}"
    source = (package / "envelope_production_operator.py").read_text(encoding="utf-8")
    assert "UNDO_REQUIRED_REASON" in source and "memfile" in source
    assert "UNDO_DROPPED_REASON" not in source


# --------------------------------------------------------------------------
# Закон топологии декали: политика хоста, пул, строка JSON
# --------------------------------------------------------------------------


def test_the_host_asks_for_the_silhouette_topology_and_its_names_are_the_kernels():
    from cftuv.surface_ir import HOST_DECAL_TOPOLOGY_POLICY, HostDecalTopologyPolicy
    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1

    assert HOST_DECAL_TOPOLOGY_POLICY is HostDecalTopologyPolicy.SILHOUETTE_TOPOLOGY_V1
    assert production.PRODUCTION_TOPOLOGY_LAW == "SILHOUETTE_TOPOLOGY_V1"
    assert {item.value for item in HostDecalTopologyPolicy} == {
        item.value for item in DecalTopologyLawV1
    }
    assert production.PRODUCTION_TOPOLOGY_LAWS == {
        item.value for item in DecalTopologyLawV1
    }


def _arities(result):
    return {len(face.ordered_vert_keys) for face in result.batch.faces}


def test_a_produced_domain_is_built_under_the_host_topology_law_and_names_it():
    prepared = _prepared_domain()

    silhouette = produce_domain(2, "domain", prepared, "0.25")
    polygons = produce_domain(2, "domain", prepared, "0.25", topology_law="PLANAR_POLYGONS_V1")
    quads = produce_domain(2, "domain", prepared, "0.25", topology_law="QUAD_STRIPS_V1")
    triangles = produce_domain(2, "domain", prepared, "0.25", topology_law="TRIANGLES_V1")

    assert silhouette.decal_topology_law == "SILHOUETTE_TOPOLOGY_V1" and len(silhouette.batch.faces) <= len(polygons.batch.faces)
    assert polygons.decal_topology_law == "PLANAR_POLYGONS_V1" and 4 in _arities(polygons)
    assert quads.decal_topology_law == "QUAD_STRIPS_V1" and 4 in _arities(quads)
    assert triangles.decal_topology_law == "TRIANGLES_V1" and _arities(triangles) == {3}
    # Закон меняет сборку граней и больше ничего: вершины, смысл, число треугольников.
    for other in (polygons, quads):
        assert other.batch.vertices == triangles.batch.vertices
        assert other.batch.semantic_digest == triangles.batch.semantic_digest
        assert other.content_digest != triangles.content_digest
        assert dict(other.counters)["MATERIALIZE_TRIANGLES"] == dict(triangles.counters)[
            "MATERIALIZE_TRIANGLES"
        ]
        assert dict(other.counters)["MATERIALIZE_QUADS"] > 0
        assert other.normal == triangles.normal
        assert other != triangles
    with pytest.raises(ValueError, match="unknown decal topology law"):
        produce_domain(2, "domain", prepared, "0.25", topology_law="SOMETHING_ELSE")


def test_a_refusal_has_no_topology_law_to_name():
    refused = produce_domain(
        0, "d", _prepared_domain(), "0.25", uv_policy_id="ENVELOPE_DEBUG_NO_UV_V1"
    )

    assert refused.batch is None and refused.decal_topology_law == ""


def test_the_pool_task_carries_the_law_and_a_task_of_the_old_shape_still_reads():
    prepared = _prepared_domain()
    blob = pickle.dumps(prepared, protocol=5)

    default = ProductionInputV1(blob, PRODUCTION_UV_POLICY)
    assert default.topology_law == production.PRODUCTION_TOPOLOGY_LAW
    task = DomainTaskV1(
        4, 4, "domain", None, None, "0.25", frozenset(), production=default
    )
    assert pickle.loads(pickle.dumps(task)) == task

    named = dataclasses.replace(
        task, production=ProductionInputV1(blob, PRODUCTION_UV_POLICY, "TRIANGLES_V1")
    )
    reply = solve_task(named)
    assert reply.ok and reply.production.decal_topology_law == "TRIANGLES_V1"
    assert reply.production == produce_domain(
        4, "domain", prepared, "0.25", topology_law="TRIANGLES_V1"
    )
    assert solve_task(task).production.decal_topology_law == "SILHOUETTE_TOPOLOGY_V1"
    bad = dataclasses.replace(
        task, production=ProductionInputV1(blob, PRODUCTION_UV_POLICY, "SOMETHING_ELSE")
    )
    assert not solve_task(bad).ok


@pytest.mark.parametrize("law", ("SILHOUETTE_TOPOLOGY_V1", "PLANAR_POLYGONS_V1", "QUAD_STRIPS_V1", "TRIANGLES_V1"))
def test_the_law_reaches_the_parent_and_the_pool_workers_alike(
    law, monkeypatch, _pool_always
):
    bundle = quad_row_bundle(ROW)
    expected, _ = _production(bundle, workers=0, topology_law=law)
    _in_process_session(monkeypatch, _InProcessPool())

    run, _ = _production(bundle, workers=2, topology_law=law)

    assert [item.placement for item in run.results] == [PLACEMENT_WORKER] * ROW
    assert _answers(run) == _answers(expected)
    assert {item.decal_topology_law for item in run.results} == {law}
    assert {item.decal_topology_law for item in expected.results} == {law}
    assert (4 in {size for item in run.results for size in _arities(item)}) == (
        law != "TRIANGLES_V1"
    )


def test_a_press_uses_the_host_law_by_default_and_refuses_an_unknown_one():
    bundle = quad_row_bundle(ROW)

    run, _ = _production(bundle)

    assert {item.decal_topology_law for item in run.results} == {"SILHOUETTE_TOPOLOGY_V1"}
    with pytest.raises(ValueError, match="unknown decal topology law"):
        _production(bundle, topology_law="SOMETHING_ELSE")


def test_the_json_row_names_the_law_and_its_counters(tmp_path):
    run, _ = _production(quad_row_bundle(ROW))

    summary = production.export_production_json(run.results, tmp_path, label="row")

    import json

    rows = json.loads(summary.read_text(encoding="utf-8"))["domains"]
    materialized = [item for item in rows if "batch_file" in item]
    assert materialized
    assert {item["decal_topology_law"] for item in materialized} == {"SILHOUETTE_TOPOLOGY_V1"}
    assert all(item["counters"]["MATERIALIZE_QUADS"] > 0 for item in materialized)
    # Числа закона названы поимённо (причина каждого оставшегося треугольника видна в строке).
    assert all(
        {
            "MATERIALIZE_POLYGON_FACES_EMITTED",
            "MATERIALIZE_POLYGON_FACES_CONCAVE_EMITTED",
            "MATERIALIZE_POLYGON_FACES_TRIANGULATED_NOT_SIMPLE",
            "MATERIALIZE_POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE",
            "MATERIALIZE_CURVED_STRIP_FACES_TRIANGULATED",
            "MATERIALIZE_MERGED_RUNS_SPLIT_AT_RUNGS",
            "MATERIALIZE_MERGED_RUNS_KEPT_WHOLE",
            "MATERIALIZE_FAN_FACES_CUT_BY_NEIGHBOUR",
            "MATERIALIZE_FAN_POLYGON_FACES_EMITTED",
            "MATERIALIZE_FAN_POLYGON_FACES_CONCAVE_EMITTED",
            "MATERIALIZE_FAN_FACES_TRIANGULATED_FROM_APEX",
            "MATERIALIZE_FAN_FACES_NOT_STAR_FROM_APEX",
        }
        <= set(item["counters"])
        and "MATERIALIZE_MERGED_RUN_FACES_TRIANGULATED" not in item["counters"]
        for item in materialized
    )
    # Грани закона — треугольники, четырёхгранья и многоугольники: сумма `n - 2` не меньше
    # числа граней плюс четырёхгранья (каждое стоит двух треугольников, многоугольник — больше).
    assert all(
        item["counters"]["MATERIALIZE_TRIANGLES"]
        >= item["counters"]["MATERIALIZE_FACES_EMITTED"] + item["counters"]["MATERIALIZE_QUADS"]
        for item in materialized
    )


# --------------------------------------------------------------------------
# Допуск растяжения запроса: метрика, воркеры, кэши сессии, консоль
# --------------------------------------------------------------------------


def _developable_line(run) -> str:
    unfolded = run.results[ROW - 1]
    return next(item for item in unfolded.diagnostics if item.startswith("DEVELOPABLE_LIFT_ONTO_UNFOLDED"))


def test_the_requests_stretch_budget_reaches_the_unfolded_domain_the_workers_and_the_console(
    monkeypatch, _pool_always
):
    from fractions import Fraction

    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    budget = Fraction(7, 20)
    default, _ = _production(bundle, workers=0)
    sequential, _ = _production(bundle, workers=0, developable_stretch_budget=budget)
    assert "stretch_budget=0.2 " in _developable_line(default)
    assert "stretch_budget=0.35 " in _developable_line(sequential)
    assert "(budget 20 %, chart HINGE)" in production.developable_stretch_lines(default.results)[0]
    assert "(budget 35 %, chart HINGE)" in production.developable_stretch_lines(sequential.results)[0]
    # Метрику холодного домена строит воркер: допуск едет в его задаче, ответ тот же, что у родителя.
    _in_process_session(monkeypatch, _InProcessPool())
    pooled, _ = _production(bundle, workers=2, developable_stretch_budget=budget)
    assert _answers(pooled) == _answers(sequential)
    assert "stretch_budget=0.35 " in _developable_line(pooled)
    assert _answers(pooled) != _answers(default)


def test_the_session_keeps_one_metric_and_one_result_per_budget_and_reuses_each():
    from fractions import Fraction

    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    controller = EnvelopeDebugSessionController()
    first, _ = _production(bundle, controller)
    assert controller.build_counts["PATCH_METRIC"] == ROW
    wide, _ = _production(bundle, controller, developable_stretch_budget=Fraction(7, 20))
    assert controller.build_counts["PATCH_METRIC"] == 2 * ROW
    assert controller.build_counts["CONVEYOR_PREPARATION"] == 2 * ROW
    again, _ = _production(bundle, controller)
    again_wide, _ = _production(bundle, controller, developable_stretch_budget=Fraction(7, 20))
    assert controller.build_counts["PATCH_METRIC"] == 2 * ROW
    assert controller.build_counts["CONVEYOR_PREPARATION"] == 2 * ROW
    assert (_hits(again), _misses(again)) == (ROW, 0)
    assert (_hits(again_wide), _misses(again_wide)) == (ROW, 0)
    assert "stretch_budget=0.2 " in _developable_line(again)
    assert "stretch_budget=0.35 " in _developable_line(again_wide)
    assert _answers(again) == _answers(first) and _answers(again_wide) == _answers(wide)
