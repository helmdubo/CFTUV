"""External code-pin capsule, not a general monkeypatch/security verifier.
Expected fingerprints are separately approved OFFLINE constants per CPython minor.
No automatic approval: observe() is test/authoring diagnostics only.
Code covers registered functions and Python methods of registered class MROs,
including generated dataclass methods. Only filenames are normalized recursively.
Defaults/closure cell contents/globals and C-extension machine code are NOT covered
by these code fingerprints. Live identities/default-tuple identity are separately
checked by the binder; approved callable closure + runtime/library artifact/version
pins must establish the remaining trust. This is not arbitrary mutation coverage.
"""
import hashlib,marshal,sys,types
SUPPORTED=frozenset({'CPython-3.11','CPython-3.13'})

def runtime_tag():return ('CPython' if sys.implementation.name=='cpython' else sys.implementation.name)+'-'+str(sys.version_info.major)+'.'+str(sys.version_info.minor)
def _normalized(code):
    return code.replace(co_filename='<angular-approved>',co_consts=tuple(
        _normalized(x) if isinstance(x,types.CodeType) else x for x in code.co_consts))
def _code(value):
    value=getattr(value,'__func__',value)
    code=getattr(value,'__code__',None)
    if code is None:return ('C_OR_NONCODE',getattr(value,'__module__',type(value).__module__),getattr(value,'__qualname__',type(value).__qualname__))
    return ('PYTHON_CODE',hashlib.sha256(marshal.dumps(_normalized(code))).hexdigest())
def fingerprint(value):
    if isinstance(value,type):
        data=('CLASS',tuple((base.__module__,base.__qualname__,tuple(
            (name,_code(item)) for name,item in sorted(vars(base).items())
            if callable(item) or isinstance(item,(classmethod,staticmethod))))
            for base in value.__mro__ if base is not object))
    else:data=_code(value)
    return hashlib.sha256(marshal.dumps(data)).hexdigest()
def observe(values):
    """Diagnostic data only; NEVER substitute for checked production constants."""
    return {'runtime':runtime_tag(),'fingerprints':{name:fingerprint(value) for name,value in values.items()}}
def verify(values,checked):
    tag=runtime_tag()
    if tag not in SUPPORTED:raise ValueError('RUNTIME_UNSUPPORTED:'+tag)
    if type(checked)is not dict or set(checked)!={'runtime','fingerprints'} or checked['runtime']!=tag:
        raise ValueError('CHECKED_RUNTIME_PINS_REQUIRED:'+tag)
    pins=checked['fingerprints']
    if type(pins)is not dict or set(pins)!=set(values):raise ValueError('RUNTIME_PIN_SET')
    for name,value in values.items():
        if fingerprint(value)!=pins[name]:raise ValueError('RUNTIME_CODE_PIN_MISMATCH:'+name)