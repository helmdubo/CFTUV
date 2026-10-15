"""Offline ANGULAR pin AUTHORING; emits candidates, never approves runtime pins.

Light worker profile uses the established host-stub loader. Full-model candidates
must run explicitly under an actual Blender Python session, never synthetic stubs.
No extension or installed cftuv_native package is imported.
"""
from __future__ import annotations
import argparse
import ctypes
import importlib
import importlib.metadata
import inspect
import json
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import types

ROOT=Path(__file__).resolve().parents[1]
SHIM=ROOT/'native/cftuv-python/python/cftuv_native'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_files(objects):
    files={}
    for value in objects:
        try:
            filename=inspect.getsourcefile(value)
        except TypeError:
            filename=None
        if filename and Path(filename).is_file():
            path=Path(filename).resolve()
            key=path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT) else str(path)
            files[key]=sha(path)
    return dict(sorted(files.items()))


def git_record(*arguments):
    argv=['git','-C',str(ROOT),*arguments]
    proc=subprocess.Popen(argv,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    creation=None
    if os.name=='nt':
        from ctypes import wintypes
        values=[wintypes.FILETIME() for _ in range(4)]
        api=ctypes.WinDLL('kernel32',use_last_error=True)
        api.GetProcessTimes.argtypes=[wintypes.HANDLE]+[ctypes.POINTER(wintypes.FILETIME)]*4
        if not api.GetProcessTimes(int(proc._handle),*(ctypes.byref(value)for value in values)):
            proc.wait()
            raise ctypes.WinError(ctypes.get_last_error())
        creation=(values[0].dwHighDateTime<<32)|values[0].dwLowDateTime
    out,error=proc.communicate()
    if proc.returncode:raise RuntimeError(error.decode(errors='replace'))
    return out.decode().strip(),dict(pid=proc.pid,creation_filetime=creation,argv=argv,exit=proc.returncode)


def artifact(module,distribution):
    dist=importlib.metadata.distribution(distribution)
    records=[Path(dist.locate_file(p))for p in (dist.files or ())if str(p).endswith('.dist-info/RECORD')]
    return dict(version=module.__version__,module_file=str(Path(module.__file__).resolve()),
                module_sha256=sha(module.__file__),record_files={str(p):sha(p)for p in records},
                scope='module entry and distribution RECORD hashed; not a full installed-file integrity audit')


def candidate(profile_kind):
    if sys.implementation.name!='cpython' or sys.version_info[:2]not in ((3,11),(3,13)):
        raise RuntimeError('UNSUPPORTED_ANGULAR_AUTHORING_RUNTIME')
    sys.path[:0]=[str(ROOT),str(ROOT/'kernel/src'),str(ROOT/'tools')]
    if profile_kind=='light':
        if sys.modules.get('cftuv.model')is not None:raise RuntimeError('LIGHT_PROFILE_ALREADY_HAS_FULL_MODEL')
        from native_snapshot_check_corpus import install_host_stubs
        install_host_stubs()
    else:
        import bpy
        if not hasattr(bpy,'app'):raise RuntimeError('FULL_PROFILE_REQUIRES_ACTUAL_BLENDER')
        importlib.import_module('cftuv.model')
    import sympy,mpmath,cftuv_envelope as kernel
    from cftuv import envelope_angular_binding as host
    package_name='_angular_offline_pin_authoring'
    if any(key==package_name or key.startswith(package_name+'.')for key in sys.modules):
        raise RuntimeError('AUTHORING_NAMESPACE_ALREADY_EXISTS')
    package=types.ModuleType(package_name);package.__path__=[str(SHIM)];sys.modules[package_name]=package
    try:
        wire=importlib.import_module(package_name+'.angular_wire')
        binding=importlib.import_module(package_name+'.angular_bindings')
        runtime=importlib.import_module(package_name+'.angular_runtime_pins')
        inputs=host.build_binding_inputs(kernel);profile=inputs.pop('profile')
        if not profile.endswith(':'+profile_kind):raise RuntimeError('ACTUAL_HOST_PROFILE_MISMATCH')
        wb=wire.bind(**inputs);deps=host.build_dependencies(kernel,sympy)
        slots=binding._slots(deps,wb)
        objects={name:binding._get(owner,key)for name,owner,key in slots}
        hashes=binding.observe_hashes(deps,wb);rt=runtime.observe(objects)
        # Убедиться, что candidate полностью пригоден для проверки, не утверждая его.
        binding.freeze(deps,checked_hashes=hashes,checked_runtime=rt,wire_binding=wb,checked_wire_metadata=wb.metadata)
        head,head_process=git_record('rev-parse','HEAD');dirty,status_process=git_record('status','--porcelain')
        files=source_files([*objects.values(),*(kind for kinds in inputs['classes'].values() for kind in kinds),host,wire,binding,runtime])
        for path in (*SHIM.glob('angular_*.py'),Path(__file__)):
            files[path.relative_to(ROOT).as_posix()]=sha(path)
        return dict(schema='cftuv.angular.offline-candidate.v1',approved=False,profile=profile,
                    pins=dict(hashes=hashes,runtime=rt,wire_metadata=wb.metadata),
                    actual_oracle_identity=wb.callables['oracle'].__module__+'.'+wb.callables['oracle'].__qualname__,
                    source_head=head,source_status=dirty,source_files=dict(sorted(files.items())),git_processes=[head_process,status_process],
                    runtime=dict(version=sys.version,cache_tag=sys.implementation.cache_tag,executable=sys.executable,
                                 executable_sha256=sha(sys.executable),platform=sys.platform),
                    dependencies=dict(sympy=artifact(sympy,'sympy'),mpmath=artifact(mpmath,'mpmath')),
                    runtime_fingerprint_encoding='Tuple trees encoded as JSON arrays; restore recursively before static Python table authoring',
                    observed_only='No APPROVED_PINS mutation; independent review and mutation gates required')
    finally:
        for key in tuple(sys.modules):
            if key==package_name or key.startswith(package_name+'.'):del sys.modules[key]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',choices=('light','full'),required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.out.exists():raise FileExistsError(args.out)
    result=candidate(args.profile)
    with args.out.open('x',encoding='utf8')as stream:json.dump(result,stream,indent=2,sort_keys=True)
    print(json.dumps(dict(profile=result['profile'],approved=False,out=str(args.out))))


if __name__=='__main__':main()
