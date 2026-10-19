"""Whole ANGULAR facade; only gate-approved offline pins open the port (an empty table keeps it unavailable)."""
from types import MappingProxyType, SimpleNamespace, MethodType
from . import angular_wire as wire, angular_bindings as bindings, angular_operation as operation

# Только отдельно утверждённые константы, никогда live observe(): r4 утверждён (ANGULAR_PINS_APPROVED_V1; утверждение по гейтам — PIN_APPROVAL_BY_GATES_V1).
from .angular_approved_pins import APPROVED as _APPROVED
APPROVED_PINS = MappingProxyType(_APPROVED)
_BINDINGS = {}
_DEPENDENCIES = ('kernel','sympy','angle_measure','measure','replay_event','host_error',
                 'outcomes','typed_value','stage_names')


def status():
    # Пустая таблица: стадия недоступна при любом пине файлов; утверждённый профиль - вердикт файлового пина (pin.OPERATION_FILES['angular']).
    if not APPROVED_PINS:
        return 'unavailable'
    from . import pin
    return pin.status('angular')


def _identity_inputs(inputs, dependencies):
    return (inputs['kernel'], tuple((name,tuple(kinds)) for name,kinds in sorted(inputs['classes'].items())),
            tuple(sorted(inputs['callables'].items())), tuple(inputs['anchors']),
            tuple((name,getattr(dependencies,name)) for name in _DEPENDENCIES))


def _same(left, right):
    if type(left)is MethodType and type(right)is MethodType:
        return left.__func__ is right.__func__ and left.__self__ is right.__self__
    if type(left)is tuple and type(right)is tuple:
        return len(left)==len(right) and all(_same(a,b) for a,b in zip(left,right))
    if type(left)is str and type(right)is str:
        return left==right
    return left is right


def _prepared(core, binding_inputs, dependencies):
    from .pin import NativePortStale
    if type(binding_inputs)is not dict:
        raise NativePortStale('ANGULAR_BINDING_INPUTS_REQUIRED')
    profile=binding_inputs.get('profile')
    if type(profile)is not str or profile not in APPROVED_PINS:
        raise NativePortStale('ANGULAR_APPROVED_PROFILE_MISSING')
    pins=APPROVED_PINS[profile]
    from .pin import require
    require('angular')
    callback=getattr(core,'angular_patch',None)
    if not callable(callback):
        raise NativePortStale('ANGULAR_PATCH_READER_MISSING')
    values={name:binding_inputs[name] for name in ('kernel','classes','callables','anchors')}
    identity=_identity_inputs(values,dependencies)
    if profile in _BINDINGS:
        old_identity,old_pins,old_callback,bound=_BINDINGS[profile]
        if old_pins is not pins or old_callback is not callback or not _same(old_identity,identity):
            raise NativePortStale('ANGULAR_BINDING_INPUTS_CHANGED')
        return bound,callback
    with wire.source_scope():
        wb=wire.bind(**values)
        bound=bindings.freeze(dependencies,checked_hashes=pins['hashes'],checked_runtime=pins['runtime'],
                              wire_binding=wb,checked_wire_metadata=pins['wire_metadata'])
    _BINDINGS[profile]=(identity,pins,callback,bound)
    return bound,callback


def warm(core, *, binding_inputs, dependencies):
    from .pin import NativePortStale
    try:
        bound,_=_prepared(core,binding_inputs,dependencies)
        maps={name:{} for name in bindings.IDS if name not in ('domain','metric')}
        result=bound.validate(SimpleNamespace(binding_metadata=bound.wire_binding.metadata,site_metadata=()),maps)
        if type(result)is bindings.BindingDecline:
            raise bindings.NativePortStale('ANGULAR_BOUND_PREFLIGHT:'+result.reason)
    except bindings.NativePortStale as error:
        raise NativePortStale(str(error)) from error
    return True


def angular_compute(core, *, binding_inputs, dependencies, raw, symbolic_mode, before_assemble):
    from .pin import NativePortStale, NativePortUnsupported
    started=False
    def begin():
        nonlocal started
        # Исключение callback уже принадлежит фазе assembly.
        started=True
        before_assemble()
    try:
        bound,callback=_prepared(core,binding_inputs,dependencies)
        result=operation.compute(wire_binding=bound.wire_binding,assembly_binding=bound,
                                 native_patch=callback,raw=raw,symbolic_mode=symbolic_mode,
                                 before_assemble=begin)
        if type(result)is operation.OperationDecline:
            if started:
                raise RuntimeError('ANGULAR_DECLINE_AFTER_ASSEMBLY_BOUNDARY')
            raise NativePortUnsupported('angular declined: '+result.reason)
        return result
    except bindings.NativePortStale as error:
        if started:
            raise
        raise NativePortStale(str(error)) from error
