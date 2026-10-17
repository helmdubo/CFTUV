from pathlib import Path
p = Path('D:/CFTUV-export-angular-20261016/native/cftuv-python/python/cftuv_native/angular_op.py')
raw = p.read_bytes().decode()
crlf = '\r\n' in raw
s = raw.replace('\r\n', '\n')
old = "def status():\n    return 'unavailable'\n"
new = ("def status():\n"
       "    # Пустая таблица: стадия недоступна при любом пине файлов; утверждённый профиль - вердикт файлового пина (pin.OPERATION_FILES['angular']).\n"
       "    if not APPROVED_PINS:\n        return 'unavailable'\n"
       "    from . import pin\n    return pin.status('angular')\n")
assert s.count(old) == 1
s = s.replace(old, new)
old2 = "    pins=APPROVED_PINS[profile]\n"
new2 = "    pins=APPROVED_PINS[profile]\n    from .pin import require\n    require('angular')\n"
assert s.count(old2) == 1
s = s.replace(old2, new2)
if crlf:
    s = s.replace('\n', '\r\n')
p.write_bytes(s.encode())
