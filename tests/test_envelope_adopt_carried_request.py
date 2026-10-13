"""Родитель принимает запрос, который воркер уже собрал и проверил (REQUEST_CARRIED_IN_REPLY_V1, хост без Blender).

Утверждения:

1. РОДИТЕЛЬ НЕ СОБИРАЕТ ВТОРОЙ РАЗ: на холодном пуле «в процессе» (ответ через pickle, как по трубе) вызов сборки запроса, шедший из приёма ответа, не происходит ни разу; воркер собрал по одному запросу на домен;
2. ЗАПРОС ТОТ ЖЕ: принятый из ответа запрос РАВЕН (значением) запросу, который родитель собрал бы на том же снапшоте, и ключ его подписи тот же;
3. ОТВЕТ ПРЕЖНИЙ: результаты, ключи кэша подготовок, ключи кэша результатов, ключи хранилища по содержимому, записи сборки для первого шага ширины и счётчики прогона равны прогону с выключателем (`REQUEST_CARRIED = False`);
4. ЗАПАСНОЙ ПУТЬ: ответ без запроса (отказ сборки, снапшот родителя, воркер старше) - родитель собирает сам, и ответ тот же; непустые замечания снапшота в памяти сессии - сборка запроса как прежде, и она отказывает домен
   тем же исходом хоста.
"""

from __future__ import annotations

import dataclasses
import pickle
import sys
import traceback
from fractions import Fraction
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_domain_pool as pool_module  # noqa: E402
from cftuv import envelope_production_export as production  # noqa: E402
from cftuv import envelope_request_export as request_export  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import DomainPoolRunV1, order_by_cost, shutdown_domain_pool, solve_task  # noqa: E402
from cftuv.envelope_production_export import run_production  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
ALPHA = 0.25


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


class _Pool:
    """Пул без подпроцессов: та же `solve_task`, ответ через pickle; `strip` убирает запрос из ответов."""

    requested = 2

    def __init__(self, *, strip=False):
        self.strip = strip

    def run(self, tasks, cancel=None):
        results = {}
        for task, _frame in order_by_cost(tasks):
            reply = pickle.loads(pickle.dumps(solve_task(task)))
            results[task.task_id] = dataclasses.replace(reply, request=None) if self.strip else reply
        return DomainPoolRunV1(results, self.requested)


def _press(monkeypatch, *, strip=False, carried=True, controller=None):
    from cftuv import envelope_queue_pool

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)
    pool = _Pool(strip=strip)
    monkeypatch.setattr(pool_module, "get_domain_pool", lambda workers, external_python="": pool)
    monkeypatch.setattr(production, "REQUEST_CARRIED", carried)
    controller = EnvelopeDebugSessionController() if controller is None else controller
    run = run_production(
        controller,
        quad_row_bundle(ROW),
        frozenset(range(ROW)),
        ALPHA,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=2,
    )
    return run, controller


def _spy_builds(monkeypatch):
    """Вызовы сборки запроса: `{"parent": сколько шло из приёма ответа (`_queue_snapshot_and_request`), "worker": сколько шло из выгрузки воркера (`task_inputs`)}`."""

    real = request_export.build_envelope_decal_request
    calls = {"parent": 0, "worker": 0}

    def counted(*args, **kwargs):
        frames = {frame.name for frame in traceback.extract_stack()}
        calls["parent" if "_queue_snapshot_and_request" in frames else "worker"] += 1
        return real(*args, **kwargs)

    monkeypatch.setattr(request_export, "build_envelope_decal_request", counted)
    return calls


def _state(run, controller):
    """Всё видимое после нажатия: ответы, ключи кэшей В ПОРЯДКЕ ЗАПИСИ (порядок приёма и вытеснения), записи сборки для шага ширины, счётчики прогона (кроме байтов и времени)."""

    counters = {
        (item.name, item.patch_domain_id): item.value
        for item in run.profile.counters
        if "BYTES" not in item.name and not item.name.endswith(("_US", "_SECONDS", "_MS"))
    }
    records = {
        tuple(map(repr, key)): {patch: (record.selected, record.prep_key, record.binding, record.request, record.snapshot) for patch, record in sorted(found.items())}
        for key, found in controller.scan_memo._runs.items()  # noqa: SLF001
    }
    return {
        "answer": [(item.patch_id, item.outcome, item.detail, item.content_digest if item.is_materialized else "") for item in run.results],
        "preparation_keys": list(map(repr, controller._conveyor_preparation_cache)),  # noqa: SLF001
        "result_keys": list(map(repr, controller._production_result_cache)),  # noqa: SLF001
        "bindings": list(map(repr, controller._content_bindings)),  # noqa: SLF001
        "store": list(controller.content_store._entries),  # noqa: SLF001
        "records": records,
        "counters": counters,
    }


def test_the_parent_builds_no_request_for_a_domain_a_worker_answered_and_the_answer_is_the_old_one(monkeypatch):
    calls = _spy_builds(monkeypatch)
    carried_run, carried_controller = _press(monkeypatch, carried=True)
    carried_calls = dict(calls)
    calls.update(parent=0, worker=0)
    plain_run, plain_controller = _press(monkeypatch, carried=False)
    plain_calls = dict(calls)

    assert carried_calls == {"parent": 0, "worker": ROW}  # воркер собрал по запросу на домен, родитель - ни одного
    assert plain_calls == {"parent": ROW, "worker": ROW}  # выключатель возвращает прежнюю вторую сборку
    assert all(item.is_materialized for item in carried_run.results)
    assert _state(carried_run, carried_controller) == _state(plain_run, plain_controller)


def test_the_carried_request_equals_the_one_the_parent_would_have_built_on_the_same_snapshot(monkeypatch):
    seen = []
    real = production._carried_inputs

    def recording(run, entry_key, provider, request):
        inputs = real(run, entry_key, provider, request)
        seen.append((run, entry_key, inputs))
        return inputs

    monkeypatch.setattr(production, "_carried_inputs", recording)
    run, controller = _press(monkeypatch, carried=True)

    assert len(seen) == ROW
    for tracked, (patch_id, domain_id, selected), (snapshot, request) in seen:
        rebuilt = request_export.build_envelope_decal_request(
            snapshot,
            selected,
            tracked.alpha,
            decal_request_id_value=tracked.request_id,
            density=tracked.density,
            developable_stretch_budget=tracked.topology_export.developable_stretch_budget,
            snapshot_issues_of=controller.snapshot_issues,
            chart_reach_cap=request_export.topology_chart_reach_cap(tracked.topology_export) if hasattr(request_export, "topology_chart_reach_cap") else None,
            silhouette_uv_slide=tracked.topology_export.silhouette_uv_slide,
        )
        assert request == rebuilt, (patch_id, domain_id)
        from cftuv.envelope_request_policy import envelope_request_policy_signature

        assert envelope_request_policy_signature(request) == envelope_request_policy_signature(rebuilt)


def test_a_reply_without_a_request_makes_the_parent_build_its_own_with_the_same_answer(monkeypatch):
    calls = _spy_builds(monkeypatch)
    stripped_run, stripped_controller = _press(monkeypatch, strip=True, carried=True)
    stripped_calls = dict(calls)
    calls.update(parent=0, worker=0)
    carried_run, carried_controller = _press(monkeypatch, carried=True)

    assert stripped_calls["parent"] == ROW
    assert _state(stripped_run, stripped_controller) == _state(carried_run, carried_controller)


def test_remembered_issues_of_the_snapshot_send_the_domain_through_the_old_build_and_its_named_refusal(monkeypatch):
    calls = _spy_builds(monkeypatch)
    issue = type("Issue", (), {"code": type("Code", (), {"value": "INJECTED_ISSUE"})(), "path": ("snapshot",), "message": "injected"})()
    real = EnvelopeDebugSessionController.snapshot_issues

    def with_issue(self, snapshot, stretch_budget=None):
        return (*real(self, snapshot, stretch_budget), issue)

    monkeypatch.setattr(EnvelopeDebugSessionController, "snapshot_issues", with_issue)
    run, _controller = _press(monkeypatch, carried=True)

    assert calls["parent"] == ROW  # заменить запрос воркера нельзя: сборка идёт как прежде
    assert not any(item.is_materialized for item in run.results)
    assert {str(item.outcome) for item in run.results} == {"ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID"}
    assert all("INJECTED_ISSUE" in str(item.detail) for item in run.results)


# --------------------------------------------------------------------------
# Вид патча строится один раз на экспорт (VIEW_PER_PATCH_ONCE_V1)
# --------------------------------------------------------------------------


def test_the_patch_view_is_built_once_per_patch_and_export_and_equals_a_fresh_one(monkeypatch):
    from cftuv import envelope_topology_export as topology_module
    from cftuv.envelope_topology_export import build_envelope_topology_export

    bundle = quad_row_bundle(ROW)
    export = build_envelope_topology_export(bundle)
    real = topology_module.build_analysis_bundle_id_view
    built = []
    monkeypatch.setattr(topology_module, "build_analysis_bundle_id_view", lambda *args: built.append(args[1]) or real(*args))

    first = export.patch_view(2)

    assert export.patch_view(2) is first and built == [frozenset({2})]
    assert first == real(bundle, frozenset({2}))
    assert export.patch_view(3) is not first and len(built) == 2
    copy = export.with_developable_stretch_budget(Fraction(1, 3))
    assert copy is not export and copy.patch_view(2) is not first  # копия экспорта строит свой вид: запись не переживает своего объекта


def test_a_cold_press_builds_one_view_per_patch_for_the_scan_and_the_adoption_together(monkeypatch):
    from cftuv import envelope_topology_export as topology_module

    real = topology_module.build_analysis_bundle_id_view
    built = []
    monkeypatch.setattr(topology_module, "build_analysis_bundle_id_view", lambda *args: built.append(sorted(args[1])) or real(*args))

    run, _controller = _press(monkeypatch, carried=True)

    assert all(item.is_materialized for item in run.results)
    assert sorted(built) == sorted([[patch] for patch in range(ROW)])  # по одному на патч: раньше сканирование строило вид, и приём ответа строил его снова


# --------------------------------------------------------------------------
# Слепок снапшота: имена полей считаются один раз на тип
# --------------------------------------------------------------------------


def test_the_witness_sizes_equal_the_walk_over_dataclasses_fields_on_real_snapshots():
    from dataclasses import fields

    from cftuv import envelope_snapshot_check as check
    from cftuv.envelope_metric_export import build_envelope_patch_metric_export
    from cftuv.envelope_topology_export import build_envelope_topology_export

    def reference(record):
        return tuple(len(value) for item in fields(record) if isinstance(value := getattr(record, item.name), frozenset))

    bundle = quad_row_bundle(ROW)
    topology = build_envelope_topology_export(bundle)
    for patch_id in sorted(bundle.patch_graph.nodes):
        snapshot = build_envelope_patch_metric_export(topology, patch_id).snapshot
        for record in (snapshot, snapshot.surface_ir):
            assert check._sizes(record) == reference(record) and check._sizes(record), type(record)  # noqa: SLF001
        assert check.snapshot_witness(snapshot)[2:] == (reference(snapshot), reference(snapshot.surface_ir))
