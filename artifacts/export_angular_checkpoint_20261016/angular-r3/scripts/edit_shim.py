from pathlib import Path
p = Path('D:/CFTUV-export-angular-20261016/native/cftuv-python/python/cftuv_native/__init__.py')
raw = p.read_bytes().decode('utf-8')
crlf = '\r\n' in raw
s = raw.replace('\r\n', '\n')
old = '    "angular_compute",\n    "warm_angular",\n'
assert s.count(old) == 1
s = s.replace(old, '    "angular_compute",\n    "angular_patch",\n    "warm_angular",\n')
anchor = 'def warm_angular('
assert s.count(anchor) == 1
fn = ('def angular_patch(patch):\n'
      '    """Test-only: the raw single-patch ANGULAR evaluator on an encoded wire dict (`angular_wire.encode` shape): `(0, plans)` or `(2, tag, detail)`.\n\n'
      '    No pins, no binding, no assembly, no effects: the whole operation is `angular_compute`, which declines while no profile is approved."""\n'
      '    return _core.angular_patch(patch)\n\n\n')
s = s.replace(anchor, fn + anchor)
if crlf:
    s = s.replace('\n', '\r\n')
p.write_bytes(s.encode('utf-8'))
print('ok', crlf)
