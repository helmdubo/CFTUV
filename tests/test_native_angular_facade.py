"""Facade protocol tests with TEST-ONLY pins and fake reader; no production approval."""
import importlib
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace as N

import pytest

SHIM = Path(__file__).resolve().parents[1] / 'native/cftuv-python/python/cftuv_native'


@pytest.fixture
def facade(monkeypatch):
    name = '_angular_facade_test'
    assert not any(key == name or key.startswith(name + '.') for key in sys.modules)
    package = ModuleType(name)
    package.__path__ = [str(SHIM)]
    sys.modules[name] = package
    # Публичные отказы моделируются без загрузки расширения и pin.py.
    pin = ModuleType(name + '.pin')
    pin.NativePortStale = type('NativePortStale', (RuntimeError,), {})
    pin.NativePortUnsupported = type('NativePortUnsupported', (RuntimeError,), {})
    # Файловый пин операции (`pin.OPERATION_FILES['angular']`): по умолчанию совпадает.
    pin.verdict = ['available']
    def require(operation):
        assert operation == 'angular'
        if pin.verdict[0] != 'available':
            raise pin.NativePortStale('the native `angular` port was compared with another version of the Python oracle; changed since the pin: ' + pin.verdict[0])
    pin.require = require
    pin.status = lambda operation: pin.verdict[0] if pin.verdict[0] == 'available' else f'stale({pin.verdict[0]})'
    sys.modules[pin.__name__] = pin
    try:
        op = importlib.import_module(name + '.angular_op')
        core = N(angular_patch=lambda patch: (0, ()))
        deps = N(**{key:object() for key in op._DEPENDENCIES})
        owner = N(helper=lambda:None)
        inputs = dict(profile='TEST_ONLY',kernel=object(),classes={},callables={},anchors=((owner,'helper',owner.helper),))
        pins = {'hashes':{'TEST_ONLY':True},'runtime':{'TEST_ONLY':True},'wire_metadata':{'TEST_ONLY':True}}
        log = []
        class Bound:
            def __init__(self, wire):self.wire_binding=wire
            def validate(self,*args):
                log.append('validate')
                for root,key,value in self.wire_binding.anchors:
                    if getattr(root,key)is not value:raise op.bindings.NativePortStale('owner changed')
        def bind(**values):
            log.append('bind')
            return N(metadata=pins['wire_metadata'],anchors=values['anchors'])
        def freeze(dependencies,**kwargs):
            assert kwargs['checked_hashes']is pins['hashes']
            log.append('freeze')
            return Bound(kwargs['wire_binding'])
        monkeypatch.setattr(op.wire,'bind',bind)
        monkeypatch.setattr(op.bindings,'freeze',freeze)
        monkeypatch.setattr(op,'APPROVED_PINS',{'TEST_ONLY':pins})
        options=dict(binding_inputs=inputs,dependencies=deps,raw={},symbolic_mode='NATIVE_EXACT',before_assemble=lambda:log.append('boundary'))
        def success(**kwargs):
            kwargs['assembly_binding'].validate()
            assert kwargs['native_patch']is core.angular_patch
            kwargs['before_assemble']()
            log.append('assemble')
            return (frozenset(),)*3
        monkeypatch.setattr(op.operation,'compute',success)
        yield N(op=op,pin=pin,core=core,deps=deps,inputs=inputs,owner=owner,log=log,options=options)
    finally:
        for key in tuple(sys.modules):
            if key == name or key.startswith(name + '.'):
                del sys.modules[key]


def test_success_and_binding_only_memo(facade):
    f=facade
    assert f.op.angular_compute(f.core,**f.options)==(frozenset(),)*3
    assert f.op.angular_compute(f.core,**f.options)==(frozenset(),)*3
    assert f.log==['bind','freeze','validate','boundary','assemble','validate','boundary','assemble']


def test_empty_production_table_is_unavailable_before_binding(facade,monkeypatch):
    f=facade
    monkeypatch.setattr(f.op,'APPROVED_PINS',{})
    assert f.op.status()=='unavailable'
    with pytest.raises(f.pin.NativePortStale,match='APPROVED_PROFILE_MISSING'):
        f.op.angular_compute(f.core,**f.options)
    assert f.log==[]


def test_empty_table_stays_unavailable_whatever_the_file_pin_says(facade,monkeypatch):
    f=facade
    monkeypatch.setattr(f.op,'APPROVED_PINS',{})
    for verdict in ('available','reference/metric.py'):
        f.pin.verdict[0]=verdict
        assert f.op.status()=='unavailable'


def test_approved_table_status_is_the_file_pin_verdict(facade):
    f=facade
    assert f.op.status()=='available'
    f.pin.verdict[0]='reference/angle_measure.py'
    assert f.op.status()=='stale(reference/angle_measure.py)'


def test_stale_file_pin_declines_before_binding_or_effects(facade):
    f=facade
    f.pin.verdict[0]='contracts/analysis.py'
    with pytest.raises(f.pin.NativePortStale,match='contracts/analysis.py'):
        f.op.angular_compute(f.core,**f.options)
    with pytest.raises(f.pin.NativePortStale,match='contracts/analysis.py'):
        f.op.warm(f.core,binding_inputs=f.inputs,dependencies=f.deps)
    assert f.log==[] and not f.op._BINDINGS


def test_pre_effect_decline_becomes_public_unsupported(facade,monkeypatch):
    f=facade
    monkeypatch.setattr(f.op.operation,'compute',lambda **kwargs:f.op.operation.OperationDecline('fixture'))
    with pytest.raises(f.pin.NativePortUnsupported,match='fixture'):
        f.op.angular_compute(f.core,**f.options)
    assert f.log==['bind','freeze']


def test_stale_owner_after_warm_is_not_rebound_or_executed(facade):
    f=facade
    assert f.op.warm(f.core,binding_inputs=f.inputs,dependencies=f.deps)
    f.owner.helper=lambda:None
    with pytest.raises(f.pin.NativePortStale,match='owner changed'):
        f.op.angular_compute(f.core,**f.options)
    assert f.log==['bind','freeze','validate','validate']


def test_changed_incoming_dependencies_do_not_silently_rebind(facade):
    f=facade
    f.op.warm(f.core,binding_inputs=f.inputs,dependencies=f.deps)
    f.deps.measure=object()
    with pytest.raises(f.pin.NativePortStale,match='INPUTS_CHANGED'):
        f.op.angular_compute(f.core,**f.options)
    assert f.log==['bind','freeze','validate']


@pytest.mark.parametrize('kind',['ordinary','custom_stale','public_stale','unsupported'])
def test_post_boundary_errors_preserve_original_identity(facade,monkeypatch,kind):
    f=facade
    error={'ordinary':ValueError,'custom_stale':f.op.bindings.NativePortStale,
           'public_stale':f.pin.NativePortStale,'unsupported':f.pin.NativePortUnsupported}[kind]('original')
    def fail(**kwargs):
        kwargs['before_assemble']()
        raise error
    monkeypatch.setattr(f.op.operation,'compute',fail)
    with pytest.raises(type(error))as found:
        f.op.angular_compute(f.core,**f.options)
    assert found.value is error and f.log==['bind','freeze','boundary']


def test_binding_owner_anchor_checks_dictionary_identity_not_contents(facade):
    f=facade
    dictionary={'canonical':1};owner=N(SYMBOLIC_FALLBACK_COUNTS=dictionary)
    state=f.op.bindings._function_state(dictionary)
    anchors=(('counter',owner,'SYMBOLIC_FALLBACK_COUNTS',dictionary,None,state),)
    core=object()
    binding=f.op.bindings.BoundAssembly(N(),anchors,{},'{}',N(metadata={}),N(stage_names=()),(),N(core=core),core)
    maps={name:{} for name in f.op.bindings.IDS if name not in ('domain','metric')}
    dictionary['canonical']+=1
    assert binding.validate(N(binding_metadata={},site_metadata=()),maps)==maps
    owner.SYMBOLIC_FALLBACK_COUNTS={'canonical':1}
    with pytest.raises(f.op.bindings.NativePortStale,match='CHANGED:counter'):
        binding.validate(N(),{})


def test_actual_oracle_imported_alias_mutation_is_stale_after_warm(facade,monkeypatch):
    from cftuv import envelope_angular_binding as host, envelope_angular_relations as angular
    import cftuv_envelope as kernel
    f=facade
    f.inputs.update(host.build_binding_inputs(kernel))
    f.inputs['profile']='TEST_ONLY'
    f.op.warm(f.core,binding_inputs=f.inputs,dependencies=f.deps)
    monkeypatch.setattr(angular,'chart_points',lambda *args:pytest.fail('stale geometry ran'))
    with pytest.raises(f.pin.NativePortStale,match='owner changed'):
        f.op.angular_compute(f.core,**f.options)
    assert f.log==['bind','freeze','validate','validate']


def test_rebuilt_actual_host_inputs_keep_warm_binding(facade):
    from cftuv import envelope_angular_binding as host
    import cftuv_envelope as kernel
    import sympy
    f=facade
    first=host.build_binding_inputs(kernel)
    second=host.build_binding_inputs(kernel)
    # Два обращения к classmethod создают разные объекты, но ту же привязку.
    for key in ('scalar_from','metric.from_descriptor'):
        assert first['callables'][key] is not second['callables'][key]
        assert first['callables'][key].__func__ is second['callables'][key].__func__
        assert first['callables'][key].__self__ is second['callables'][key].__self__
    first['profile']=second['profile']='TEST_ONLY'
    deps1=host.build_dependencies(kernel,sympy)
    deps2=host.build_dependencies(kernel,sympy)
    assert f.op.warm(f.core,binding_inputs=first,dependencies=deps1)
    assert f.op.warm(f.core,binding_inputs=second,dependencies=deps2)
    assert f.log==['bind','freeze','validate','validate']


@pytest.mark.parametrize('change',['receiver','function'])
def test_bound_method_identity_rejects_changed_receiver_or_function(facade,change):
    from types import MethodType
    f=facade
    class Owner:
        @classmethod
        def method(cls):return None
    class Other(Owner):pass
    first=Owner.method
    assert f.op._same(first,Owner.method)
    def replacement(cls):return None
    changed=Other.method if change=='receiver'else MethodType(replacement,Owner)
    assert not f.op._same(first,changed)
    f.inputs['callables']['bound']=first
    f.op.warm(f.core,binding_inputs=f.inputs,dependencies=f.deps)
    f.inputs['callables']['bound']=changed
    with pytest.raises(f.pin.NativePortStale,match='INPUTS_CHANGED'):
        f.op.warm(f.core,binding_inputs=f.inputs,dependencies=f.deps)
    assert f.log==['bind','freeze','validate']
