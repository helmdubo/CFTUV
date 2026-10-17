from pathlib import Path
import hashlib
REPO = Path('D:/CFTUV-export-angular-20261016')
p = REPO / 'native/cftuv-python/python/cftuv_native/pin.py'
raw = p.read_bytes().decode('utf-8')
CRLF = '\r\n' in raw
s = raw.replace('\r\n', '\n')
anchor = 'OPERATION_FILES = {\n'
block = '''#: ANGULAR: the kernel files whose objects the binding of the frozen host oracle `_build_angular_relations_reference` reads (the kernel part of the source files of
#: the offline pin candidate, `tools/native_angular_pins.py`). A file-level guard like the lists above; the function-level pins are `angular_op.APPROVED_PINS`, and
#: while that table is empty the stage stays `unavailable` whatever these digests say (`angular_op.status`).
ANGULAR_FILES = ("contracts/analysis.py", "contracts/metric.py", "ids.py", "numeric.py", "reference/angle_measure.py", "reference/metric.py", "reference/planar_types.py")

'''
assert s.count(anchor) == 1
s = s.replace(anchor, block + anchor)
old = '    "density": DENSITY_FILES,\n}'
assert s.count(old) == 1
s = s.replace(old, '    "density": DENSITY_FILES,\n    "angular": ANGULAR_FILES,\n}')
root = REPO / 'kernel/src/cftuv_envelope'


def d(n):
    return hashlib.sha256((root / n).read_bytes().replace(b'\r\n', b'\n')).hexdigest()


new = {}
for n in ("contracts/analysis.py", "contracts/metric.py", "ids.py", "reference/angle_measure.py", "reference/metric.py", "reference/planar_types.py"):
    assert f'"{n}":' not in s
    new[n] = d(n)
assert f'"numeric.py": "{d("numeric.py")}"' in s
end = '    "reference/density_attempt_memo_v1.py":'
i = s.index(end)
j = s.index('\n', i) + 1
s = s[:j] + ''.join(f'    "{n}": "{v}",\n' for n, v in new.items()) + s[j:]
if CRLF:
    s = s.replace('\n', '\r\n')
p.write_bytes(s.encode('utf-8'))
print(CRLF, new)
