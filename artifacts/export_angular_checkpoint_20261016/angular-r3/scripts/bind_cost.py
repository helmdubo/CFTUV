"""Diagnostic: cost of the first ANGULAR binding (wire.bind + freeze) and of a warm re-validation, light profile, injected TEST-ONLY pins."""
import json, sys, time
from pathlib import Path
from types import MappingProxyType
S = Path('D:/cftuv-angular-gates-20261017')
ROOT = Path('D:/CFTUV-export-angular-20261016')
sys.path[:0] = [str(S / 'wheel-final2/site'), str(ROOT / 'kernel/src'), str(ROOT), str(ROOT / 'tools'), 'C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages']
from native_snapshot_check_corpus import install_host_stubs
install_host_stubs()
t0 = time.perf_counter()
import cftuv_native, sympy, cftuv_envelope as kernel
from cftuv_native import angular_op as op
from cftuv import envelope_angular_binding as host
t_import = time.perf_counter() - t0
sys.path.insert(0, str(S / 'abhook'))
import os
os.environ['ANGULAR_HARNESS_PROFILES'] = str(S / 'cand/profile-test-CPython-3.11-light-82bff418.json')
import angular_pin_hook
assert op.APPROVED_PINS
times = []
for i in range(4):
    t = time.perf_counter()
    inputs = host.build_binding_inputs(kernel)
    t_inputs = time.perf_counter() - t
    deps = host.build_dependencies(kernel, sympy)
    t_deps = time.perf_counter() - t - t_inputs
    cftuv_native.warm_angular(binding_inputs=inputs, dependencies=deps)
    times.append(dict(total=time.perf_counter() - t, inputs=t_inputs, deps=t_deps))
from cftuv_native import angular_wire as wire, angular_bindings as bindings
inputs = host.build_binding_inputs(kernel); profile = inputs.pop('profile')
t = time.perf_counter(); wb = wire.bind(**inputs); t_bind = time.perf_counter() - t
deps = host.build_dependencies(kernel, sympy)
pins = op.APPROVED_PINS[profile]
t = time.perf_counter(); bindings.freeze(deps, checked_hashes=pins['hashes'], checked_runtime=pins['runtime'], wire_binding=wb, checked_wire_metadata=pins['wire_metadata']); t_freeze = time.perf_counter() - t
print(json.dumps(dict(import_s=t_import, warm_calls=times, wire_bind_s=t_bind, freeze_s=t_freeze), indent=1))
