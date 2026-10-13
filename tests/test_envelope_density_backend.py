"""Кандидат density: явный process hook, без смены публичного умолчания."""
import pytest
from cftuv import envelope_kernel_backend as host
from cftuv import envelope_production_export as production
from cftuv_envelope import backend
from test_envelope_kernel_backend import _backend_state, _fake_native, BUILD_ID


@pytest.fixture(autouse=True)
def clean_density_environment(monkeypatch):
    monkeypatch.delenv('CFTUV_DENSITY_BACKEND',raising=False)


def test_density_is_python_until_explicitly_ordered_and_master_python_wins(monkeypatch):
    assert host._density_order('NATIVE')=='PYTHON'
    assert 'density' not in dict(host.STAGES)
    assert 'density' not in host._STATUS_FLAGS
    monkeypatch.setenv('CFTUV_DENSITY_BACKEND','NATIVE')
    assert host._density_order('NATIVE')=='NATIVE'
    assert host._density_order('PYTHON')=='PYTHON'
    assert host._density_order('PYTHON','NATIVE')=='PYTHON'
    assert host._density_order('NATIVE','PYTHON')=='PYTHON'
    monkeypatch.setenv('CFTUV_DENSITY_BACKEND','RUST')
    with pytest.raises(ValueError,match='unknown kernel backend'):host._density_order('NATIVE')
    assert host._density_order('PYTHON')=='PYTHON'


def test_candidate_identity_changes_only_when_requested(monkeypatch):
    _fake_native()
    old=host.backend_identity_of('NATIVE')
    assert '|density=' not in old
    monkeypatch.setenv('CFTUV_DENSITY_BACKEND','NATIVE')
    assert host.backend_identity_of('NATIVE')==old+'|density=NATIVE:'+BUILD_ID
    assert host.backend_identity_of('PYTHON')=='PYTHON'
    assert host.backend_identity_of('NATIVE',density_backend='PYTHON')==old


def test_preparation_keyword_and_borrowed_owner_ledger_are_preserved(monkeypatch):
    monkeypatch.setenv('CFTUV_DENSITY_BACKEND','NATIVE')
    def build():
        assert backend.active_density_backend() is backend.KernelBackendV1.NATIVE
        return object()
    prepared,record=host.prepared_under_backend(build,'NATIVE','PYTHON','PYTHON')
    assert record.density_requested=='NATIVE'
    with host.entered_backend('NATIVE','PYTHON','PYTHON') as ledger:
        answer,borrowed=host.prepared_under_backend(build,'NATIVE','PYTHON','PYTHON',borrowed_ledger=ledger)
        assert borrowed is None
        with pytest.raises(ValueError,match='BACKEND_BORROW_SCOPE_MISMATCH'):
            host.prepared_under_backend(build,'NATIVE','PYTHON','PYTHON',density_backend='PYTHON',borrowed_ledger=ledger)
    assert backend.active_density_backend() is backend.KernelBackendV1.PYTHON


def test_production_wrapper_records_native_density_and_summary(monkeypatch):
    answer=object();_fake_native(density_compute=lambda batch:answer)
    monkeypatch.setenv('CFTUV_DENSITY_BACKEND','NATIVE')
    @host.with_kernel_backend
    def produce(batch):
        assert backend.density_compute(batch) is answer
        return production._refusal(7,'domain','MATERIALIZED','')
    result=produce(object(),backend='NATIVE',skeleton_backend='PYTHON',embedding_backend='PYTHON')
    assert result.backend_record.density_native_calls==1
    monkeypatch.setenv('CFTUV_DENSITY_BACKEND','PYTHON')
    summary=host.backend_summary([result],'NATIVE')
    assert summary.stage('density').requested=='NATIVE'
    assert summary.stage('density').native==1
    assert 'density candidate 1' in host.backend_text(summary)
    cached=result.with_changes(placement=production.PLACEMENT_CACHED)
    assert host.backend_summary([cached],'NATIVE').stage('density').native==0
    assert 'density' not in [x.stage for x in host.backend_summary([],'NATIVE').stages]


def test_explicit_python_candidate_does_not_leak_to_producer(monkeypatch):
    monkeypatch.setenv('CFTUV_DENSITY_BACKEND','NATIVE')
    @host.with_kernel_backend
    def produce():
        assert backend.active_density_backend() is backend.KernelBackendV1.PYTHON
        return production._refusal(7,'domain','MATERIALIZED','')
    assert produce(backend='PYTHON').backend_record is None
    result=produce(backend='NATIVE',density_backend='PYTHON')
    assert result.backend_record.density_requested=='PYTHON'
