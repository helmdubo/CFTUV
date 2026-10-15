"""Density следует главному переключателю и сохраняет выбор в очереди, кэшах и заимствованных журналах."""

import dataclasses
import builtins
import inspect
import pickle

import pytest

from cftuv import envelope_content_key as keys
from cftuv import envelope_kernel_backend as host
from cftuv import envelope_production_export as production
from cftuv import envelope_production_task as tasks
from cftuv import envelope_snapshot_byte_check as byte_check
from cftuv import envelope_snapshot_check as snapshot_check
from cftuv.envelope_domain_pool import DomainTaskV1, shutdown_domain_pool
from cftuv.envelope_production_scan import _RunInputsV1
from cftuv.envelope_width_live import LastProductionBuildV1, remember_build
from cftuv_envelope import backend
from envelope_fixture_bundles import quad_row_bundle
from content_equivalence import result_projection
from test_envelope_production_export import _production
from native_gate import strict


@pytest.fixture(autouse=True)
def isolated_orders(monkeypatch):
    for name in host.STAGE_ENVIRONMENT.values():
        monkeypatch.setenv(name, 'PYTHON')
    monkeypatch.delenv('CFTUV_DENSITY_BACKEND', raising=False)
    backend.refresh_native()
    yield
    shutdown_domain_pool()
    backend.refresh_native()


def require_native():
    status = backend.native_status()
    if not status.density_available:
        reason = f'Density propagation native smoke: {status.density}; {status.detail}'
        if strict():
            pytest.fail(reason)
        pytest.skip(reason)
    assert status.density_available, status
    assert status.build_id
    return status.build_id


def bare_record(cls, master, choice=None):
    values = {f.name: None for f in dataclasses.fields(cls)
              if f.default is dataclasses.MISSING and f.default_factory is dataclasses.MISSING}
    values[master] = 'NATIVE'
    values['density_backend'] = choice
    return cls(**values)


@pytest.mark.parametrize('cls,master', [
    (DomainTaskV1, 'backend'), (_RunInputsV1, 'backend'),
    (production.ProductionRunV1, 'kernel_backend'), (LastProductionBuildV1, 'kernel_backend'),
])
def test_frozen_choice_survives_environment_change_and_pickle(monkeypatch, cls, master):
    monkeypatch.setenv('CFTUV_DENSITY_BACKEND', 'NATIVE')
    record = bare_record(cls, master)
    monkeypatch.setenv('CFTUV_DENSITY_BACKEND', 'PYTHON')
    restored = pickle.loads(pickle.dumps(record))
    assert record.density_backend == restored.density_backend == 'NATIVE'
    assert bare_record(cls, master).density_backend == 'PYTHON'
    assert inspect.signature(cls).parameters['density_backend'].kind is inspect.Parameter.KEYWORD_ONLY


def test_density_and_frame_are_public_stages_with_six_positional_arguments():
    assert host.STAGES[-1] == ('density', 'density')
    assert len(host.STAGES) == 8
    assert host.stage_orders().as_arguments() == ('NATIVE',) * 3 + ('PYTHON',) * 3
    assert host.stage_orders().as_keywords() == {'frame_backend': 'PYTHON', 'density_backend': 'NATIVE'}
    with host.entered_backend('NATIVE') as ledger:
        assert backend.active_density_backend() is backend.KernelBackendV1.NATIVE
        assert ledger.density_requested == 'NATIVE'


@pytest.mark.parametrize('master,explicit,ambient,wanted', [
    ('NATIVE', None, None, 'NATIVE'), ('PYTHON', None, 'NATIVE', 'PYTHON'),
    ('NATIVE', None, 'PYTHON', 'PYTHON'), ('NATIVE', 'NATIVE', 'PYTHON', 'NATIVE'),
    ('NATIVE', 'PYTHON', 'NATIVE', 'PYTHON'), ('PYTHON', 'NATIVE', 'PYTHON', 'NATIVE'),
    ('NATIVE', 'NATIVE', 'INVALID', 'NATIVE'), ('PYTHON', None, 'INVALID', 'PYTHON'),
])
def test_density_uses_the_public_master_environment_and_explicit_order_law(monkeypatch, master, explicit, ambient, wanted):
    if ambient is None:
        monkeypatch.delenv('CFTUV_DENSITY_BACKEND', raising=False)
    else:
        monkeypatch.setenv('CFTUV_DENSITY_BACKEND', ambient)
    assert host.stage_orders(master, density_backend=explicit).density == wanted
    assert host.resolve_density_backend(master, explicit) == wanted


def test_mixed_python_master_can_borrow_the_explicit_native_density_ledger(monkeypatch):
    require_native()
    with host.entered_backend('PYTHON', density_backend='NATIVE') as ledger:
        assert ledger.requested == 'PYTHON' and ledger.density_requested == 'NATIVE'
        assert backend.active_density_backend() is backend.KernelBackendV1.NATIVE
        prepared, record = host.prepared_under_backend(
            backend.active_density_backend, 'PYTHON', density_backend='NATIVE', borrowed_ledger=ledger)
        assert prepared is backend.KernelBackendV1.NATIVE and record is None
        with pytest.raises(ValueError, match='BACKEND_BORROW_SCOPE_MISMATCH'):
            host.prepared_under_backend(lambda: None, 'PYTHON', borrowed_ledger=ledger)
    monkeypatch.setenv('CFTUV_DENSITY_BACKEND', 'NATIVE')
    with host.entered_backend('PYTHON') as ledger:
        assert ledger is None and backend.active_density_backend() is backend.KernelBackendV1.PYTHON


def test_preparation_identity_validates_before_the_optional_kernel_import(monkeypatch):
    original = builtins.__import__
    def without_backend(name, *args, **kwargs):
        if name == 'cftuv_envelope.backend':
            raise ImportError('kernel unavailable')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', without_backend)
    assert host.skeleton_identity_of('PYTHON', density_backend='python') == 'PYTHON'
    with pytest.raises(ValueError):
        host.skeleton_identity_of('PYTHON', density_backend='unknown')


@pytest.mark.parametrize('skeleton', ['PYTHON', 'NATIVE'])
def test_preparation_partition_uses_the_same_native_build_identity(skeleton):
    build = require_native()
    old = host.skeleton_identity_of(skeleton, density_backend='PYTHON')
    assert host.skeleton_identity_of(skeleton, density_backend='PYTHON') == old
    assert host.skeleton_identity_of(skeleton, density_backend='python') == old
    with pytest.raises(ValueError):
        host.skeleton_identity_of(skeleton, density_backend=backend.KernelBackendV1.PYTHON)
    with pytest.raises(ValueError):
        host.skeleton_identity_of(skeleton, density_backend='unknown')
    native = host.skeleton_identity_of(skeleton, density_backend='NATIVE')
    assert native == old + '|density=NATIVE:' + build
    assert host.skeleton_identity_of(skeleton) == native


def test_direct_preparation_preserves_explicit_borrowed_density(monkeypatch):
    from cftuv_envelope import wavefront
    monkeypatch.setattr(wavefront, 'prepare_conveyor', lambda *a, **kw: backend.active_density_backend())
    with host.entered_backend('NATIVE', 'PYTHON', 'PYTHON', density_backend='NATIVE') as ledger:
        prepared, record = tasks.prepare_for_production_recorded(
            None, None, backend='NATIVE', skeleton_backend='PYTHON', embedding_backend='PYTHON',
            density_backend='NATIVE', borrowed_ledger=ledger)
        assert prepared is backend.KernelBackendV1.NATIVE and record is None
        with pytest.raises(ValueError, match='BACKEND_BORROW_SCOPE_MISMATCH'):
            tasks.prepare_for_production_recorded(
                None, None, backend='NATIVE', skeleton_backend='PYTHON', embedding_backend='PYTHON',
                density_backend='PYTHON', borrowed_ledger=ledger)
    assert tasks.prepare_for_production(None, None, backend='NATIVE', density_backend='NATIVE') is backend.KernelBackendV1.NATIVE


def test_active_and_explicit_snapshot_identity_use_the_resolved_choice():
    require_native()
    identities = []
    for choice in ('PYTHON', 'NATIVE', 'PYTHON'):
        with host.entered_backend('NATIVE', 'PYTHON', 'PYTHON', density_backend=choice):
            explicit = keys.execution_identity('NATIVE', 'PYTHON', 'PYTHON', density_backend=choice)
            assert snapshot_check.active_execution() == explicit
            assert byte_check.execution_identity_of('NATIVE', 'PYTHON', 'PYTHON', density_backend=choice) == explicit
            identities.append(explicit)
    assert identities[0] == identities[2] != identities[1]


@pytest.mark.parametrize('workers', [0, 2], ids=['parent', 'real-pool'])
def test_real_product_switch_partitions_preparation_and_returns_to_warm_python(monkeypatch, workers):
    require_native()
    from cftuv import envelope_queue_pool
    monkeypatch.setattr(envelope_queue_pool, 'COVERAGE_POOL_MIN_BYTES', 0)
    bundle = quad_row_bundle(5)
    first, controller = _production(bundle, workers=workers, density=2,
                                    kernel_backend='NATIVE', skeleton_backend='PYTHON', embedding_backend='PYTHON', density_backend='PYTHON')
    monkeypatch.setenv('CFTUV_DENSITY_BACKEND', 'NATIVE')
    captured = remember_build(controller, 'source', bundle, first, source_object_key='object', source_data_key='mesh',
                              selected=frozenset(range(5)), density=2, stretch_percent=0, width=0.25,
                              kernel_backend='NATIVE', skeleton_backend='PYTHON', embedding_backend='PYTHON')
    assert captured.density_backend == first.density_backend == 'PYTHON'
    builds = controller.build_counts['CONVEYOR_PREPARATION']
    native, _ = _production(bundle, controller, workers=workers, density=2,
                            kernel_backend='NATIVE', skeleton_backend='PYTHON', embedding_backend='PYTHON', density_backend='NATIVE')
    assert controller.build_counts['CONVEYOR_PREPARATION'] == builds * 2
    assert all(r.backend_record.density_requested == 'NATIVE' for r in native.results)
    assert sum(r.backend_record.density_native_calls for r in native.results) > 0
    assert sum(r.backend_record.density_python_calls for r in native.results) == 0
    assert [result_projection(r) for r in native.results] == [result_projection(r) for r in first.results]
    monkeypatch.setenv('CFTUV_DENSITY_BACKEND', 'NATIVE')
    warm, _ = _production(bundle, controller, workers=workers, density=2,
                          kernel_backend='NATIVE', skeleton_backend='PYTHON', embedding_backend='PYTHON', density_backend='PYTHON')
    assert controller.build_counts['CONVEYOR_PREPARATION'] == builds * 2
    assert all(r.placement == production.PLACEMENT_CACHED for r in warm.results)
    assert warm.density_backend == 'PYTHON'
    assert [result_projection(r) for r in warm.results] == [result_projection(r) for r in first.results]


@pytest.mark.parametrize('workers', [0, 2], ids=['parent', 'real-pool'])
def test_real_default_density_runs_natively_and_warm_cache_preserves_the_reference(monkeypatch, workers):
    require_native()
    from cftuv import envelope_queue_pool
    monkeypatch.setattr(envelope_queue_pool, 'COVERAGE_POOL_MIN_BYTES', 0)
    for name in host.STAGE_ENVIRONMENT.values():
        monkeypatch.delenv(name, raising=False)
    bundle = quad_row_bundle(5)
    native, controller = _production(bundle, workers=workers, density=2)
    assert native.kernel_backend == native.density_backend == 'NATIVE'
    assert all(r.backend_record.density_requested == 'NATIVE' for r in native.results)
    assert sum(r.backend_record.density_native_calls for r in native.results) > 0
    assert sum(r.backend_record.density_python_calls for r in native.results) == 0
    assert all(not r.backend_record.density_outcomes for r in native.results)
    builds = controller.build_counts['CONVEYOR_PREPARATION']
    warm, _ = _production(bundle, controller, workers=workers, density=2)
    assert controller.build_counts['CONVEYOR_PREPARATION'] == builds
    assert all(r.placement == production.PLACEMENT_CACHED for r in warm.results)
    reference, _ = _production(bundle, workers=workers, density=2, kernel_backend='PYTHON')
    assert reference.density_backend == 'PYTHON'
    assert all(r.backend_record is None for r in reference.results)
    expected = [result_projection(r) for r in reference.results]
    assert [result_projection(r) for r in native.results] == [result_projection(r) for r in warm.results] == expected
    assert host.backend_identity_of(native.kernel_backend, density_backend=native.density_backend) != host.backend_identity_of(reference.kernel_backend, density_backend=reference.density_backend)
