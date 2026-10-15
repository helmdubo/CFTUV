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
    f.op.warm(f.core,binding_inputs=f.inputs,dependencies=f.deps)
    monkeypatch.setattr(angular,'chart_points',lambda *args:pytest.fail('stale geometry ran'))
    with pytest.raises(f.pin.NativePortStale,match='owner changed'):
        f.op.angular_compute(f.core,**f.options)
    assert f.log==['bind','freeze','validate','validate']
