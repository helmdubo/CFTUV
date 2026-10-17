"""The owner's own Blender may be open (idle): a sample is rejected only when a FOREIGN blender.exe used CPU during it (never stopped)."""
from pathlib import Path
c = Path('D:/cftuv-angular-gates-20261017/operator/operator_controller.py')
s = c.read_text(encoding='utf-8')


def rep(old, new):
    global s
    assert s.count(old) == 1, old
    s = s.replace(old, new)


rep("def disk_guard():",
    "def cpu_seconds(pid):\n"
    "    k = ctypes.WinDLL('kernel32', use_last_error=True)\n"
    "    k.OpenProcess.restype = wintypes.HANDLE\n"
    "    k.GetProcessTimes.argtypes = [wintypes.HANDLE] + [ctypes.POINTER(wintypes.FILETIME)] * 4\n"
    "    k.CloseHandle.argtypes = [wintypes.HANDLE]\n"
    "    h = k.OpenProcess(0x1000, False, int(pid))\n"
    "    if not h:\n        return None\n"
    "    v = [wintypes.FILETIME() for _ in range(4)]\n"
    "    try:\n"
    "        if not k.GetProcessTimes(h, *(ctypes.byref(x) for x in v)):\n            return None\n"
    "        return (((v[2].dwHighDateTime << 32) | v[2].dwLowDateTime) + ((v[3].dwHighDateTime << 32) | v[3].dwLowDateTime)) / 1e7\n"
    "    finally:\n        k.CloseHandle(h)\n\n\n"
    "def foreign_cpu(foreign):\n"
    "    return {(f['pid'], f['creation']): cpu_seconds(f['pid']) for f in foreign}\n\n\n"
    "def disk_guard():")
rep("    assert not blender_processes(), 'FOREIGN_BLENDER: retry after owner closes Blender'\n",
    "    state_foreign = blender_processes()\n")
rep("                    assert not blender_processes(), 'FOREIGN_BLENDER before next sample'\n",
    "                    foreign = [f for f in blender_processes()]\n"
    "                    cpu_before = foreign_cpu(foreign)\n")
rep("                    assert not row['foreign_seen'], 'foreign Blender overlapped; sample rejected'\n",
    "                    cpu_after = foreign_cpu(foreign)\n"
    "                    row['foreign_idle'] = [dict(pid=k[0], cpu_before=cpu_before[k], cpu_after=cpu_after[k]) for k in cpu_before]\n"
    "                    busy = [r for r in row['foreign_idle'] if r['cpu_before'] is not None and r['cpu_after'] is not None and r['cpu_after'] - r['cpu_before'] > 0.75]\n"
    "                    appeared = [f for f in row['foreign_seen'] if (f['pid'], f['creation']) not in cpu_before]\n"
    "                    save()\n"
    "                    assert not busy and not appeared, 'foreign Blender busy or started during the sample; sample rejected'\n")
# foreign_seen collects every non-own blender; with the owner's idle one present that is expected
c.write_text(s, encoding='utf-8')
print('ok')
