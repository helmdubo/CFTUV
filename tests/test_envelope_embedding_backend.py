"""B1: экспорт покрыт scope до подготовки, запись не теряется/не удваивается, память — прежняя."""
from __future__ import annotations

from types import SimpleNamespace

import pytest

from cftuv import envelope_domain_pool as pool_module
from cftuv import envelope_export_input as export_input
from cftuv import envelope_kernel_backend as host
from cftuv import envelope_production_export as production
from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1
from cftuv.envelope_debug_session import EnvelopeDebugSessionController
from cftuv.envelope_production_mesh import DEFAULT_DECAL_OFFSET, build_mesh_arrays
from cftuv.envelope_request_export import EnvelopeHostAdapterError
from test_envelope_kernel_backend import _backend_state, _fake_native  # noqa: F401 - фикстура состояния
from test_envelope_production_content import ROW, row, pool  # noqa: F401 - настоящий путь с внутрипроцессным пулом
from cftuv_envelope import backend
from cftuv_envelope import _embedding

ARGS = ({}, {}, (), (), (), "law")


def _native_embedding(*, real=False):
    calls = []
    def leaf(*args):
        calls.append((backend.active_embedding_backend().value, args))
        return _embedding._compute_source_snap_embedding_certificate(*args) if real else object()
    module = _fake_native(snap_embedding_certificate=leaf)
    return module, calls


def _task():
    return pool_module.DomainTaskV1(0, 7, "domain", None, None, ".25", frozenset(),
        cold=production.ColdProductionInputV1(), backend="PYTHON", skeleton_backend="PYTHON", embedding_backend="NATIVE")


def _record(count=1):
    return backend.BackendRecordV1("PYTHON", 0, 0, embedding_requested="NATIVE", embedding_native_calls=count)


def _produced(record=None):
    return production._refusal(7, "domain", "MATERIALIZED", "").with_changes(backend_record=record)


def test_embedding_follows_the_master_switch_unless_the_api_orders_it_apart_and_preparation_identity_is_unchanged():
    assert not hasattr(host, "DEFAULT_EMBEDDING_BACKEND")  # единственная константа - главный переключатель (EMBEDDING_NATIVE_DEFAULT_V1 теперь через него)
    task = pool_module.DomainTaskV1(0, 7, "d", None, None, ".25", frozenset())
    assert task.embedding_backend == "NATIVE"
    assert pool_module.DomainTaskV1(0, 7, "d", None, None, ".25", frozenset(), backend="PYTHON").embedding_backend == "PYTHON"
    assert host.backend_identity_of("PYTHON", "PYTHON", "PYTHON") == "PYTHON"
    assert host.backend_identity_of("PYTHON", "PYTHON") == host.backend_identity_of("PYTHON") == "PYTHON"  # стадия без слова - как главный переключатель
    assert host.backend_identity_of("PYTHON", "PYTHON", "NATIVE").startswith("PYTHON|snap_embedding=NATIVE:")
    assert host.skeleton_identity_of("PYTHON") == "PYTHON"


def test_worker_cold_export_preparation_and_materialization_are_recorded_once(monkeypatch):
    _module, calls = _native_embedding()
    profile = EnvelopeDebugProfileBuilderV1("worker", "QUEUE")
    def inputs(task):
        backend.embedding_compute(*ARGS)
        return export_input.TaskInputsV1(object(), object(), profile, task.task_id, True)
    def prepare(snapshot, request, **kwargs):
        return host.prepared_under_backend(lambda: backend.embedding_compute(*ARGS), kwargs["backend"],
            kwargs["skeleton_backend"], kwargs["embedding_backend"], borrowed_ledger=kwargs["borrowed_ledger"])
    @host.with_kernel_backend
    def produce(*args, **kwargs):
        backend.embedding_compute(*ARGS)
        return _produced()
    monkeypatch.setattr(export_input, "task_inputs", inputs)
    monkeypatch.setattr(production, "prepare_for_production_recorded", prepare)
    monkeypatch.setattr(production, "produce_domain", produce)
    reply = production.solve_cold_production_task(_task())
    assert reply.ok and not reply.backend_record
    assert reply.production.backend_record.embedding_native_calls == len(calls) == 3
    assert not reply.production.backend_record.native_calls and not reply.production.backend_record.skeleton_native_calls
    assert all(choice == "NATIVE" for choice, _ in calls)


@pytest.mark.parametrize("error", [False, True])
def test_worker_export_refusal_or_error_keeps_the_b1_record(monkeypatch, error):
    _native_embedding()
    def inputs(task):
        backend.embedding_compute(*ARGS)
        if error:
            raise KeyError("source vertex")
        return pool_module.DomainTaskResultV1(task.task_id, refusal=export_input.ExportRefusalV1("WHOLE_PATCH", "no chart", "domain"))
    monkeypatch.setattr(export_input, "task_inputs", inputs)
    reply = production.solve_cold_production_task(_task())
    assert reply.backend_record.embedding_native_calls == 1
    assert bool(reply.error) is error and reply.refused is (not error)
    assert not reply.production


def _run_stub(**parts):
    return SimpleNamespace(backend="PYTHON", skeleton_backend="PYTHON", embedding_backend="NATIVE", cancel=None,
        revision="r", alpha=.25, alpha_text=".25", request_id="q", density=None, topology_law=production.PRODUCTION_TOPOLOGY_LAW,
        uv_policy_id=production.PRODUCTION_UV_POLICY, profile=EnvelopeDebugProfileBuilderV1("parent", "QUEUE"), **parts)


def test_parent_scan_metric_and_inputs_share_one_record(monkeypatch):
    from cftuv import envelope_scan_memo
    from cftuv.envelope_request_export import _typed_value
    _module, calls = _native_embedding()
    snapshot = object()
    def metric(*args):
        backend.embedding_compute(*ARGS)
        return snapshot
    controller = SimpleNamespace(get_patch_metric=metric, get_domain_geometry=lambda metric: SimpleNamespace(snapshot=metric))
    run = _run_stub(controller=controller, topology_export=object(), patch_ids=(7,),
        selected_by_domain={_typed_value("patch-domain", "r", 7): frozenset()},
        hooks=SimpleNamespace(export_provider=lambda *a: None))
    monkeypatch.setattr(envelope_scan_memo, "scan_key", lambda run: None)
    monkeypatch.setattr(production, "_bound_entry", lambda *a: None)
    def inputs(run, key, provider):
        assert provider(7, "domain") is snapshot
        backend.embedding_compute(*ARGS)
        return snapshot, object()
    monkeypatch.setattr(production, "_inputs_of", inputs)
    monkeypatch.setattr(production, "_entry_with_inputs", lambda run, patch, domain, selected, inputs:
        production._DomainEntryV1(patch, domain, selected, inputs=inputs))
    entries = production._scan(run)
    assert entries[0].export_record.embedding_native_calls == len(calls) == 2


@pytest.mark.parametrize("refuse_parent", [False, True])
def test_worker_whole_patch_refusal_then_parent_band_keeps_both_export_records_once(monkeypatch, refuse_parent):
    _native_embedding()
    def band(*args):
        backend.embedding_compute(*ARGS)
        if refuse_parent:
            raise EnvelopeHostAdapterError("BAND_REFUSED", "no band")
        return object()
    run = _run_stub(controller=object(), hooks=SimpleNamespace(export_adopter=lambda *a: None, snapshot_provider=band))
    entry = production._DomainEntryV1(7, "domain", frozenset(), export=object())
    reply = pool_module.DomainTaskResultV1(0, refusal=export_input.ExportRefusalV1("WHOLE_PATCH", "no chart", "domain"), backend_record=_record())
    monkeypatch.setattr(production, "_inputs_of", lambda run, key, provider: (provider(*key[:2]), object()))
    monkeypatch.setattr(production, "_produce_cold_in_parent", lambda *a: _produced(_record(2)))
    monkeypatch.setattr(production, "_labeled_by_parent", lambda run, patch, result, log: result)
    monkeypatch.setattr(production, "_remember", lambda *a: None)
    monkeypatch.setattr(production, "_result_key", lambda *a: None)
    monkeypatch.setattr(production, "_register_content", lambda *a: None)
    refused, result = production._adopt_cold(run, entry, reply, "parent")
    results = production._domain_results([entry], {} if refused else {"domain": result}, {"domain": refused} if refused else {})
    assert results[0].backend_record.embedding_native_calls == (2 if refuse_parent else 4)
    assert results[0].outcome == ("BAND_REFUSED" if refuse_parent else "MATERIALIZED")


def test_cached_result_does_not_claim_old_native_calls():
    entry = production._DomainEntryV1(7, "domain", frozenset(), cached=_produced(_record(99)))
    cached, = production._domain_results([entry], {}, {})
    assert cached.placement == production.PLACEMENT_CACHED and cached.backend_record is None


def test_cached_result_keeps_only_the_export_calls_of_this_run():
    # `_domain_results` кладёт в результат из кэша запись экспорта ЭТОГО прогона (она может нести вызовы: запрос строится внутри области экспорта), а не запись, с которой он лёг в кэш
    entry = production._DomainEntryV1(7, "domain", frozenset(), cached=_produced(_record(99)), export_records=[_record(2)])
    cached, = production._domain_results([entry], {}, {})
    assert cached.placement == production.PLACEMENT_CACHED and cached.backend_record.embedding_native_calls == 2


def _press(bundle, controller, choice, workers, alpha=.25):
    return production.run_production(controller, bundle, frozenset(range(ROW)), alpha,
        source_object_key="obj", source_data_key="mesh", density=None, workers=workers,
        kernel_backend="NATIVE", skeleton_backend="NATIVE", embedding_backend=choice)


@pytest.mark.parametrize("workers", [0, 2])
def test_real_host_paths_keep_certificates_ordered_mesh_uv_and_price(row, pool, workers):
    from cftuv_envelope.wavefront.coverage import _coverage_at
    from cftuv_envelope.wavefront.skeleton import build_skeleton
    from cftuv_envelope.materialize.clip import clip_geometry
    module, calls = _native_embedding(real=True)
    module.coverage_at, module.clip_geometry, module.build_skeleton = _coverage_at, clip_geometry, build_skeleton
    # Шим скелета принимает split_search=None; эталону этот default не передают.
    module.build_skeleton = lambda polygon, split_search=None, **kw: build_skeleton(polygon, **kw)
    with _embedding.embedding_memo_limit(0):
        reference = _press(row, EnvelopeDebugSessionController(), "PYTHON", workers)
        controller = EnvelopeDebugSessionController()
        native = _press(row, controller, "NATIVE", workers)
    assert calls and all(choice == "NATIVE" for choice, _ in calls)
    assert sum(item.backend_record.embedding_native_calls for item in native.results) == len(calls)
    for one, other in zip(reference.results, native.results):
        assert (one.outcome, one.content_digest, one.batch) == (other.outcome, other.content_digest, other.batch)
        assert {k: v for k, v in one.counters if k.startswith("EXACT_WORK_")} == {k: v for k, v in other.counters if k.startswith("EXACT_WORK_")}
        assert not other.backend_record.embedding_fallbacks
    a, b = build_mesh_arrays(reference.results, DEFAULT_DECAL_OFFSET), build_mesh_arrays(native.results, DEFAULT_DECAL_OFFSET)
    assert (a.positions, a.faces, a.uvs, a.face_domain, a.face_owner, a.seam_edges) == (b.positions, b.faces, b.uvs, b.face_domain, b.face_owner, b.seam_edges)
    count = len(calls)
    warm = _press(row, controller, "NATIVE", workers, .3)
    assert len(calls) == count
    assert sum(getattr(item.backend_record, "embedding_native_calls", 0) for item in warm.results) == 0
    cached = _press(row, controller, "NATIVE", workers, .3)
    assert all(item.placement == production.PLACEMENT_CACHED for item in cached.results)
    assert len(calls) == count, "a press that serves every domain from the result cache computes no certificate"
    # Результат из кэша не приписывает прогону прежние вызовы. Запись у него есть: журнал экспорта домена ЭТОГО прогона (область экспорта открыта до поиска в кэше результатов,
    # потому что запрос, из которого берётся ключ, строится внутри неё), но в этом прогоне она пуста - ни одного вызова ни одной стадии.
    assert all(
        item.backend_record is None
        or not (
            item.backend_record.native_calls + item.backend_record.python_calls + item.backend_record.skeleton_native_calls + item.backend_record.skeleton_python_calls
            + item.backend_record.embedding_native_calls + item.backend_record.embedding_python_calls
        )
        for item in cached.results
    )
