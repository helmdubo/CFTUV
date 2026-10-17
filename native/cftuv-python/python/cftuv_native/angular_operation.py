"""External whole-op draft; NOT installed or native accepted.
Caller supplies separately checked frozen bindings and pinned pure native callback.
OperationDecline alone permits caller-owned independent-oracle fallback. Stale,
native exceptions and all assembly/measure exceptions propagate, never reexecute.
No cftuv_core import, no native extension discovery or dynamic pin approval.
"""
from dataclasses import dataclass
from types import SimpleNamespace
from . import angular_wire as wire
from . import angular_bindings as bindings
from . import angular_assemble as assembly

@dataclass(frozen=True)
class OperationDecline:
    reason: str
    patch: int | None = None

def compute(*, wire_binding, assembly_binding, native_patch, raw, symbolic_mode, before_assemble):
    """Prepare ALL patches before effects; success is the original 3-frozenset tuple.
    raw is the original host keyword dict, including revision and optional profile.
    native_patch must be the caller's checked pure single-patch callback. Its named
    decline tuple is (2, code, detail); errors/panics are NOT ordinary fallback.
    Encoder and prepare use existing pre-effect admission; no geometry is retried.
    Future backend adapter maps NativePortStale to its existing native error model.
    """
    if type(assembly_binding) is not bindings.BoundAssembly:
        raise bindings.NativePortStale('EXACT_BOUND_ASSEMBLY_REQUIRED')
    if type(raw) is not dict or type(raw.get('revision')) is not str:
        return OperationDecline('RAW_KEYWORDS_OR_REVISION')
    try:
        maps={name:raw[name] for name in bindings.IDS if name not in ('domain','metric')}
    except KeyError:
        return OperationDecline('MISSING_HOST_ID_MAP')
    # Gate stale host callables BEFORE encode can call angular_sites. Full sidecar
    # admission is repeated by prepare_checked after encoding; no unsafe stale fallback.
    early=assembly_binding.validate(SimpleNamespace(binding_metadata=wire_binding.metadata,site_metadata=()),maps)
    if type(early)is bindings.BindingDecline:
        return OperationDecline('BINDING:'+early.reason)
    encoded = wire.encode(wire_binding, **raw, symbolic_mode=symbolic_mode)
    if type(encoded) is wire.WireDecline:
        return OperationDecline('WIRE:'+encoded.reason)
    if type(encoded) is not wire.AngularWire:
        raise bindings.NativePortStale('ENCODER_RETURN_PROTOCOL')
    plans=[]
    for patch in encoded.patches:
        plan=native_patch(patch)
        if type(plan) is tuple and len(plan)==3 and type(plan[0])is int and plan[0]==2:
            if type(plan[1])is not str or type(plan[2])is not str:
                raise bindings.NativePortStale('NATIVE_DECLINE_PROTOCOL')
            return OperationDecline('NATIVE:'+plan[1]+':'+plan[2],patch['patch_id'])
        if type(plan)is not tuple or len(plan)!=2 or type(plan[0])is not int or plan[0]!=0:
            raise bindings.NativePortStale('NATIVE_ACCEPT_PROTOCOL')
        plans.append(plan)
    # Anchors were checked by the early gate of THIS call; prepare_checked repeats only the sidecar/map admission.
    prepared=bindings.prepare_checked(assembly_binding,encoded,tuple(plans),maps=maps,revision=raw['revision'],anchors_checked=True)
    if type(prepared)is assembly.AssemblyDecline:
        return OperationDecline('ASSEMBLY:'+prepared.reason)
    # No catch beyond this boundary: reached effects belong to the real operation.
    before_assemble()
    return assembly.assemble(prepared,profile=raw.get('profile'))