"""Unapplied whole-operation binding gate. No cftuv imports or approved pin generation.
checked_hashes/checked_wire_metadata MUST come from separately reviewed constants.
observe_hashes is diagnostic TEST/authoring input only, never authorization.
prepare_checked is the integration entry; direct angular_assemble.prepare is unbound.
"""
from dataclasses import dataclass
from hashlib import sha256
from inspect import getsource
from enum import Enum
import json
from types import MappingProxyType
from . import angular_assemble as assembly
from . import angular_runtime_pins as runtime_pins

class NativePortStale(RuntimeError):
    """Binding/pin drift: propagate; never silently fall back to the oracle."""

@dataclass(frozen=True)
class BindingDecline:
    reason: str

@dataclass(frozen=True)
class Facade:
    fields: object
    def __getattr__(self, name):
        try: return self.fields[name]
        except KeyError: raise AttributeError(name) from None

def _facade(values): return Facade(MappingProxyType(dict(values)))
def _identity(value): return getattr(value, '__func__', value)
def _hash(value):
    text = getsource(type(value)) + repr(value.value) if isinstance(value,Enum) else getsource(value)
    return sha256(text.replace('\r\n', '\n').encode()).hexdigest()
def _function_state(value):
    fn=_identity(value)
    return (getattr(fn,'__code__',None), id(getattr(fn,'__defaults__',None)),
            tuple((key,id(item)) for key,item in (getattr(fn,'__kwdefaults__',None) or {}).items()))

def _class_members(kind):
    # Detect mutable class attributes/descriptors, including inherited ID methods.
    return tuple((base, tuple((name, id(value), _function_state(value)) for name, value in vars(base).items()))
                 for base in kind.__mro__ if base is not object)

KERNEL = ('CertifiedDecimalIntervalV1','CertifiedReflexAngleMeasureV1','AngleNormalizationLaw','ExactRationalV1','ExactVector2V1','CertifiedAffineSupportDirectionV2',
 'SourceSupportRefV1','OrientedOwnerSectorV1','AngleCertificateId','ReflexAngleCertificateV1',
 'CornerRelationId','CornerRelationV1','TurnOrientation','InteriorSelectionLaw','AngleMeasureLaw',
 'AngleMeasureSource','StrictAngleRangeCertificate','ReflexExcessLaw')
IDS = {'patch_ids':'PatchId','use_id_by_ref':'ChainUseId','sector_id_by_ref':'OwnerSectorId',
       'launch_id_by_ref':'BoundaryConstraintId','vertex_ids':'SourceVertexId',
       'domain':'PatchDomainId','metric':'ReferenceMetricId'}
OUTCOMES = ('ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE',
            'ENVELOPE_DEBUG_MULTIPLE_ANGULAR_RELATIONS_PER_CHAIN_UNSUPPORTED')
DIRECT = ('measure','replay_event','host_error','typed_value')

def _slots(deps, wire_binding):
    slots = [(name, deps, name) for name in DIRECT]
    slots += [('measure.'+name,deps.measure.__globals__,name) for name in
              ('canonical_significant_digits','_rounded_outward')]
    slots += [('ids.'+name,deps.typed_value.__globals__,name) for name in ('_stable_token','typed_id')]
    token=deps.typed_value.__globals__['_stable_token']
    slots += [('ids.stable_token',token.__globals__,'stable_token')]
    slots += [('kernel.'+name,deps.kernel,name) for name in (*KERNEL,*dict.fromkeys(IDS.values()))]
    slots += [('sympy.'+name,deps.sympy,name) for name in ('Rational','sqrt','factor')]
    slots += [('angle_measure.'+name,deps.angle_measure,name) for name in
              ('CertifiedAngleUnavailable','reflex_angle_intervals_over_pi')]
    slots += [('outcomes.'+name,deps.outcomes,name) for name in OUTCOMES]
    # Wire closure metadata is observed previously; also bind its live callable objects.
    slots += [('wire.'+name,wire_binding.callables,name) for name in wire_binding.callables]
    return slots

def _metadata(value): return json.dumps(value,sort_keys=True,separators=(',',':'))

def _get(owner, name):
    return owner[name] if type(owner) in (dict,MappingProxyType) else getattr(owner,name)

def observe_hashes(dependencies, wire_binding):
    """Diagnostics only; caller must NOT treat this result as checked approval."""
    return {**{name:_hash(_get(owner,key)) for name,owner,key in _slots(dependencies,wire_binding)},
            'stage_names':sha256(_metadata(dependencies.stage_names).encode()).hexdigest()}

@dataclass(frozen=True)
class BoundAssembly:
    dependencies: object
    anchors: tuple
    classes: object
    wire_metadata: object
    wire_binding: object
    stage_owner: object
    stage_names: tuple
    sympy_owner: object
    sympy_core: object

    def validate(self, wire, maps):
        """Once per wholeop, BEFORE scalar staging. Stale is not ordinary decline."""
        for name,owner,key,value,members,state in self.anchors:
            current = _get(owner,key)
            if _identity(current) is not _identity(value): raise NativePortStale('CALLABLE_OR_CLASS_CHANGED:'+name)
            if _function_state(current)!=state: raise NativePortStale('FUNCTION_STATE_CHANGED:'+name)
            if members is not None and _class_members(current)!=members: raise NativePortStale('CLASS_MEMBERS_CHANGED:'+name)
        if self.stage_owner.stage_names != self.stage_names: raise NativePortStale('STAGE_NAMES_CHANGED')
        if self.sympy_owner.core is not self.sympy_core: raise NativePortStale('SYMPY_CORE_CHANGED')
        if _metadata(self.wire_binding.metadata) != self.wire_metadata or _metadata(wire.binding_metadata) != self.wire_metadata:
            raise NativePortStale('WIRE_BINDING_METADATA_CHANGED')
        try:
            if type(maps) is not dict or set(maps)!=set(IDS)-{'domain','metric'}: return BindingDecline('MAP_SET')
            copied={}
            for name, source in maps.items():
                if type(source) is not dict: return BindingDecline('MAP_TYPE')
                target={}
                for key,value in source.items():
                    if name=='patch_ids' or name=='vertex_ids':
                        if type(key) is not int: return BindingDecline('MAP_INT_KEY')
                    elif type(key) is not tuple or len(key)!=3 or any(type(x)is not int for x in key):
                        return BindingDecline('MAP_REF_KEY')
                    if type(value) is not self.classes[name] or type(value.value) is not str:
                        return BindingDecline('EXACT_ID_CLASS:'+name)
                    target[key]=value
                copied[name]=target
            for side in wire.site_metadata:
                domain=side['domain_id'];metric=side['frame'].reference_metric_id
                if type(domain) is not self.classes['domain'] or type(domain.value)is not str: return BindingDecline('EXACT_DOMAIN_ID')
                if type(metric) is not self.classes['metric'] or type(metric.value)is not str: return BindingDecline('EXACT_METRIC_ID')
            return copied
        except Exception as error: return BindingDecline('MAP_ADMISSION:'+type(error).__name__)

def freeze(dependencies, *, checked_hashes, checked_runtime, wire_binding, checked_wire_metadata):
    """Build a sealed facade from actual host-supplied objects and checked pins."""
    slots=_slots(dependencies,wire_binding)
    required={name for name,_,_ in slots} | {'stage_names'}
    if type(checked_hashes)is not dict or set(checked_hashes)!=required:
        raise NativePortStale('CHECKED_PINS_REQUIRED')
    if sha256(_metadata(dependencies.stage_names).encode()).hexdigest()!=checked_hashes['stage_names']:
        raise NativePortStale('STAGE_NAMES_PIN_MISMATCH')
    if wire_binding.metadata!=checked_wire_metadata: raise NativePortStale('WIRE_CHECKED_METADATA_MISMATCH')
    try:
        runtime_pins.verify({name:_get(owner,key) for name,owner,key in slots},checked_runtime)
    except ValueError as error: raise NativePortStale(str(error)) from error
    anchors=[]; captured={}
    # Прямые aliases oracle и фактический владелец цены replay, не снимок значений.
    for ordinal,(owner,key,value) in enumerate(wire_binding.anchors):
        anchors.append(('wire.owner.'+str(ordinal),owner,key,value,
                        _class_members(value) if isinstance(value,type) else None,_function_state(value)))
    try:
        replay_globals=dependencies.replay_event.__globals__
        metric_types=replay_globals['_metric_types']
        counter_owner=metric_types.SYMBOLIC_FALLBACK_COUNTS
    except (AttributeError,KeyError) as error:
        raise NativePortStale('REPLAY_COUNTER_OWNER_REQUIRED') from error
    if type(counter_owner)is not dict:
        raise NativePortStale('REPLAY_COUNTER_DICT_REQUIRED')
    anchors += [('replay._metric_types',replay_globals,'_metric_types',metric_types,None,_function_state(metric_types)),
                ('replay.SYMBOLIC_FALLBACK_COUNTS',metric_types,'SYMBOLIC_FALLBACK_COUNTS',counter_owner,None,_function_state(counter_owner))]
    anchors.append(('wire.callables',wire_binding,'callables',wire_binding.callables,None,_function_state(wire_binding.callables)))
    for name in ('kernel','sympy','angle_measure','outcomes'):
        anchors.append(('dependency.'+name,dependencies,name,getattr(dependencies,name),None,_function_state(getattr(dependencies,name))))
    anchors += [('sympy.core.parameters',dependencies.sympy.core,'parameters',dependencies.sympy.core.parameters,None,_function_state(dependencies.sympy.core.parameters)),
                ('sympy.global_parameters',dependencies.sympy.core.parameters,'global_parameters',dependencies.sympy.core.parameters.global_parameters,None,_function_state(dependencies.sympy.core.parameters.global_parameters))]
    for name,owner,key in slots:
        value=_get(owner,key)
        if name in required and _hash(value)!=checked_hashes[name]: raise NativePortStale('SOURCE_PIN_MISMATCH:'+name)
        anchors.append((name,owner,key,value,_class_members(value) if isinstance(value,type) else None,_function_state(value)))
        captured[name]=value
    names=dependencies.stage_names
    if type(names)is not tuple or any(type(x)is not str for x in names) or len(set(names))!=len(names):
        raise NativePortStale('STAGE_NAMES_SHAPE')
    # Explicit nested facades: no generic recursive freezer or live module fallback.
    k=_facade({name:captured['kernel.'+name] for name in KERNEL})
    sp=_facade({**{name:captured['sympy.'+name] for name in ('Rational','sqrt','factor')},'core':dependencies.sympy.core})
    am=_facade({name:captured['angle_measure.'+name] for name in ('CertifiedAngleUnavailable','reflex_angle_intervals_over_pi')})
    out=_facade({name:captured['outcomes.'+name] for name in OUTCOMES})
    frozen=_facade({**{name:captured[name] for name in DIRECT},'kernel':k,'sympy':sp,
                    'angle_measure':am,'outcomes':out,'stage_names':names})
    # Copy metadata via JSON-compatible values, not an alias to the mutable observation.
    metadata=_metadata(checked_wire_metadata)
    classes=MappingProxyType({name:captured['kernel.'+kind] for name,kind in IDS.items()})
    return BoundAssembly(frozen,tuple(anchors),classes,metadata,wire_binding,dependencies,names,
                         dependencies.sympy,dependencies.sympy.core)

def prepare_checked(binding, wire, native_plans, *, maps, revision):
    if type(binding)is not BoundAssembly: raise NativePortStale('EXACT_BOUND_ASSEMBLY_REQUIRED')
    admitted=binding.validate(wire,maps)
    if type(admitted)is BindingDecline: return assembly.AssemblyDecline(admitted.reason)
    return assembly.prepare(wire,native_plans,maps=admitted,revision=revision,dependencies=binding.dependencies)