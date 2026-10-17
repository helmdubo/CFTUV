from pathlib import Path
p = Path('D:/CFTUV-export-angular-20261016/cftuv/envelope_domain_pool.py')
raw = p.read_bytes().decode(); crlf = '\r\n' in raw
s = raw.replace('\r\n', '\n')
old = """            import cftuv_envelope as kernel
            import sympy
            from cftuv_envelope.backend import warm_angular
            from .envelope_angular_binding import build_binding_inputs, build_dependencies
"""
new = """            from cftuv_envelope.backend import warm_angular
            from .envelope_angular_binding import build_binding_inputs, build_dependencies
            from .envelope_request_export import _load_kernel

            kernel, sympy = _load_kernel()  # те же модули, что получит ANGULAR в экспорте домена
"""
assert s.count(old) == 1
s = s.replace(old, new)
p.write_bytes((s.replace('\n', '\r\n') if crlf else s).encode())
print(crlf)
