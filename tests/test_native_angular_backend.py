"""Portable ANGULAR dispatcher tests: no real extension, Blender or approved pins."""
from types import SimpleNamespace
import inspect
import pytest
from cftuv_envelope import backend as b

class NativePortStale(Exception):pass
class NativeUnsupportedPython(Exception):pass
class NativePortUnsupported(Exception):pass
class NativeDivisionDiverged(Exception):pass
REFUSALS=(NativePortStale,NativeUnsupportedPython,NativePortUnsupported,NativeDivisionDiverged)
ARGS=dict(binding_inputs={'host':'roots'},dependencies={'host':'callables'},raw={},symbolic_mode='NATIVE_EXACT')

@pytest.fixture
def native(monkeypatch):
    module=SimpleNamespace(**{x.__name__:x for x in REFUSALS},NATIVE_REFUSALS=REFUSALS,
        native_status=lambda:{'coverage':'available','clip':'available'},native_build_id=lambda:'test-build')
    monkeypatch.setattr(b,'_native',lambda:(module,''))
    return module

def test_disabled_by_default_even_with_native_master(native,monkeypatch):
    monkeypatch.setenv('CFTUV_ANGULAR_BACKEND','NATIVE') # kernel scope never invents host env policy
    native.angular_compute=lambda **kw:pytest.fail('disabled operation reached')
    assert b.angular_compute(**ARGS)==(False,None)
    with b.use_backend('NATIVE') as ledger:
        assert b.active_angular_backend() is b.KernelBackendV1.PYTHON
        assert b.angular_compute(**ARGS)==(False,None)
        assert not ledger.native and not ledger.python

@pytest.mark.parametrize('error',[NativePortStale('stale'),NativeUnsupportedPython('runtime'),NativePortUnsupported('shape')])
def test_named_pre_effect_decline_records_fallback(native,error):
    def compute(**kw):raise error
    native.angular_compute=compute
    with b.use_backend('PYTHON',angular_backend='NATIVE') as ledger:
        assert b.angular_compute(**ARGS)==(False,None)
        record=ledger.record()
        assert record.angular_python_calls==1 and record.angular_native_calls==0
        assert record.angular_ran=='python' and record.angular_fallbacks[0][1]=='angular'

@pytest.mark.parametrize('error',[NativePortStale('late stale'),NativeUnsupportedPython('late runtime'),NativePortUnsupported('late shape'),NativeDivisionDiverged('late divergence'),b._OperationAbsent('late absent'),b._TracesUnsupported('late traces'),ValueError('host measure'),RuntimeError('native panic')])
def test_every_after_phase_exception_preserves_identity_and_no_fallback(native,error):
    effects=[]
    def compute(**kw):
        kw['before_assemble']();effects.append('canonical');effects.append('measure');raise error
    native.angular_compute=compute
    with b.use_backend('PYTHON',angular_backend='NATIVE') as ledger:
        with pytest.raises(type(error)) as caught:b.angular_compute(**ARGS)
        assert caught.value is error
        assert effects==['canonical','measure']
        assert not ledger.native and not ledger.python and not ledger.fallbacks and ledger.refusal is None

@pytest.mark.parametrize('error',[RuntimeError('native panic'),TypeError('cftuv_native: bad wire'),ValueError('input defect')])
def test_pre_effect_defects_not_silently_retried(native,error):
    def compute(**kw):raise error
    native.angular_compute=compute
    with b.use_backend('PYTHON',angular_backend='NATIVE') as ledger:
        with pytest.raises(type(error)) as caught:b.angular_compute(**ARGS)
        assert caught.value is error and not ledger.python

def test_empty_success_and_opaque_arguments(native):
    answer=(frozenset(),frozenset(),frozenset());calls=[]
    def compute(**kw):
        calls.append(kw);kw['before_assemble']();return answer
    native.angular_compute=compute
    with b.use_backend('PYTHON',angular_backend='NATIVE') as ledger:
        assert b.angular_compute(**ARGS)==(True,answer)
        assert len(calls)==1 and calls[0]['binding_inputs'] is ARGS['binding_inputs']
        assert ledger.record().angular_native_calls==1
        assert b.active_angular_backend() is b.KernelBackendV1.NATIVE
    assert b.active_angular_backend() is b.KernelBackendV1.PYTHON

def test_old_wheel_unavailable_and_status(native):
    assert not b.native_status().angular_available
    assert b.native_status().as_record()['angular']=='unavailable'
    with b.use_backend('PYTHON',angular_backend='NATIVE') as ledger:
        assert b.angular_compute(**ARGS)==(False,None)
        assert ledger.record().angular_outcomes==('NATIVE_UNAVAILABLE',)
    native.native_status=lambda:{'angular':'available'}
    assert b.native_status().angular_available

def test_record_merge_identity_scope_and_positional_compatibility(native):
    for fn in (b.use_backend,b.backend_identity):
        signature=inspect.signature(fn)
        assert signature.parameters['angular_backend'].kind is inspect.Parameter.KEYWORD_ONLY
        assert len([p for p in signature.parameters.values() if p.kind is inspect.Parameter.POSITIONAL_OR_KEYWORD])==6
    assert b.backend_identity('PYTHON')=='PYTHON'
    assert b.backend_identity('PYTHON',angular_backend='NATIVE')=='PYTHON|angular=NATIVE:test-build'
    with b.use_backend('PYTHON',angular_backend='NATIVE') as ledger:
        ledger.note_native(b.ANGULAR);first=ledger.record()
        with b.use_backend('PYTHON',angular_backend='NATIVE',borrowed_ledger=ledger) as borrowed:assert borrowed is ledger
        with pytest.raises(ValueError,match='BACKEND_BORROW_SCOPE_MISMATCH'):
            with b.use_backend('PYTHON',borrowed_ledger=ledger):pass
    combined=first.merged(first)
    assert combined.angular_native_calls==2 and combined.angular_ran=='native'
    assert combined.as_record()['angular_requested']=='NATIVE'
    assert 'ANGULAR' in b.__all__ and 'angular_compute' in b.__all__

def test_reserved_phase_callback(native):
    native.angular_compute=lambda **kw:pytest.fail('invalid caller reached')
    with b.use_backend('PYTHON',angular_backend='NATIVE'):
        with pytest.raises(TypeError,match='reserved'):b.angular_compute(before_assemble=lambda:None)