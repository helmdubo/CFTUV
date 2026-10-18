"""R3+ S2 copy of the ANGULAR timing controller (TEST-ONLY chain_station pin hook on the branch side, --branch-env, --reps). Парные холодные кнопки. Только отдельные background Blender; никаких kill/save/install."""
import argparse
import ctypes
from ctypes import wintypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from station_timing_runner import creation


SCENES = {
    'building': (Path('C:/Users/helmd/AppData/Local/Temp/cftuv_field_parity_20261008/frozen/testscene.blend'),
                 '750baefc50ddbccad2ff090dae135c55e7b0d1584ca62697261a32b249e15fb4'),
    'cover008': (Path('C:/Users/helmd/AppData/Local/Temp/cftuv_cover008_frozen_v2/buildings2_2.blend'),
                 'a044b5aa8d5156bcf64167d7181557fb563b2d1472d8f837b4c6a51ce414ab32'),
}


def blender_processes():
    class Entry(ctypes.Structure):
        _fields_ = [('size', wintypes.DWORD), ('usage', wintypes.DWORD), ('pid', wintypes.DWORD),
                    ('heap', ctypes.c_size_t), ('module', wintypes.DWORD), ('threads', wintypes.DWORD),
                    ('parent', wintypes.DWORD), ('priority', wintypes.LONG), ('flags', wintypes.DWORD),
                    ('exe', wintypes.WCHAR * 260)]
    k = ctypes.WinDLL('kernel32', use_last_error=True)
    k.CreateToolhelp32Snapshot.restype = wintypes.HANDLE
    k.Process32FirstW.argtypes = [wintypes.HANDLE, ctypes.POINTER(Entry)]
    k.Process32NextW.argtypes = [wintypes.HANDLE, ctypes.POINTER(Entry)]
    k.CloseHandle.argtypes = [wintypes.HANDLE]
    handle = k.CreateToolhelp32Snapshot(2, 0)
    assert handle != ctypes.c_void_p(-1).value, 'process inventory unavailable'
    entry = Entry()
    entry.size = ctypes.sizeof(entry)
    rows = []
    try:
        more = k.Process32FirstW(handle, ctypes.byref(entry))
        while more:
            if entry.exe.lower() == 'blender.exe':
                rows.append({'pid': entry.pid, 'creation': creation(entry.pid)})
            more = k.Process32NextW(handle, ctypes.byref(entry))
    finally:
        k.CloseHandle(handle)
    return rows


def cpu_seconds(pid):
    k = ctypes.WinDLL('kernel32', use_last_error=True)
    k.OpenProcess.restype = wintypes.HANDLE
    k.GetProcessTimes.argtypes = [wintypes.HANDLE] + [ctypes.POINTER(wintypes.FILETIME)] * 4
    k.CloseHandle.argtypes = [wintypes.HANDLE]
    h = k.OpenProcess(0x1000, False, int(pid))
    if not h:
        return None
    v = [wintypes.FILETIME() for _ in range(4)]
    try:
        if not k.GetProcessTimes(h, *(ctypes.byref(x) for x in v)):
            return None
        return (((v[2].dwHighDateTime << 32) | v[2].dwLowDateTime) + ((v[3].dwHighDateTime << 32) | v[3].dwLowDateTime)) / 1e7
    finally:
        k.CloseHandle(h)


def foreign_cpu(foreign):
    return {(f['pid'], f['creation']): cpu_seconds(f['pid']) for f in foreign}


def disk_guard():
    for drive in ('C:/', 'E:/'):
        assert shutil.disk_usage(drive).free >= 1.5 * 1024**3, 'DISK_STOP ' + drive


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024**2), b''):
            h.update(block)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser()
    for side in ('base', 'branch'):
        for key in ('tree', 'native-site', 'build-id'):
            p.add_argument('--' + side + '-' + key, required=True)
    p.add_argument('--case', choices=('building', 'cover008', 'both'), default='both')
    p.add_argument('--output', required=True, help='NEW directory on D:')
    p.add_argument('--branch-env', action='append', default=[])
    p.add_argument('--branch-hook', action='store_true')
    p.add_argument('--reps', type=int, default=3)
    p.add_argument('--blender', default='C:/Program Files/Blender Foundation/Blender 4.5/blender.exe')
    a = p.parse_args()
    out = Path(a.output).resolve()
    assert out.drive.lower() == 'd:', 'evidence must be on D:'
    assert not out.exists(), 'output directory already exists'
    disk_guard()
    state_foreign = blender_processes()
    out.mkdir(parents=True)
    temporary = out / 'tmp'
    temporary.mkdir()
    state = {'schema': 'cftuv.real.operator.pairs.v1', 'controller_pid': os.getpid(),
             'controller_creation': creation(os.getpid()), 'controller_argv': sys.argv, 'runs': []}

    def save():
        (out / 'controller.json').write_text(json.dumps(state, indent=2), encoding='utf8')

    cases = ('building', 'cover008') if a.case == 'both' else (a.case,)
    try:
        for case in cases:
            scene, expected = SCENES[case]
            for repeat in range(1, a.reps + 1):
                for side in (('base', 'branch') if repeat % 2 else ('branch', 'base')):
                    disk_guard()
                    foreign = [f for f in blender_processes()]
                    cpu_before = foreign_cpu(foreign)
                    before = sha(scene)
                    assert before == expected, 'FROZEN_SCENE_CHANGED before sample'
                    label = f'{case}_{repeat}_{side}'
                    result = out / (label + '.json')
                    hook = ['--python', str(Path(__file__).resolve().parent / 'abhook/station_pin_hook.py')] if side == 'branch' and a.branch_hook else []
                    argv = [a.blender, '-b', str(scene), '--python-exit-code', '1', *hook, '--python',
                            str(Path(__file__).with_name('station_timing_runner.py')), '--', '--tree',
                            getattr(a, side + '_tree'), '--native-site', getattr(a, side + '_native_site'),
                            '--build-id', getattr(a, side + '_build_id'), '--case', case, '--output', str(result)]
                    env = dict(os.environ, PYTHONSAFEPATH='1', PYTHONDONTWRITEBYTECODE='1', PYTHONHASHSEED='0',
                               TMP=str(temporary), TEMP=str(temporary))
                    # Сравнивается публичное умолчание каждого дерева; никакой записи корпусов.
                    for key in tuple(env):
                        if key.startswith('CFTUV_'):
                            del env[key]
                    if side == 'branch':
                        env.update(item.split('=', 1) for item in a.branch_env)
                        if a.branch_hook:
                            env.update(PYTHONPATH=str(Path(__file__).resolve().parent / 'abhook'), STATION_HARNESS_APPROVE='1',
                                       STATION_HARNESS_LOG=str(out / (label + '.hook.jsonl')))
                    row = {'label': label, 'argv': argv, 'sha_before': before, 'foreign_seen': [], 'exit': None}
                    state['runs'].append(row)
                    with (out / (label + '.stdout.txt')).open('x', encoding='utf8') as stdout, \
                         (out / (label + '.stderr.txt')).open('x', encoding='utf8') as stderr:
                        process_started = time.perf_counter()
                        process = subprocess.Popen(argv, stdout=stdout, stderr=stderr, env=env)
                        row.update(pid=process.pid, creation=creation(process.pid))
                        assert row['creation'] is not None, 'cannot record Blender creation time'
                        save()
                        low_disk = False
                        while process.poll() is None:
                            for found in blender_processes():
                                if found['pid'] != process.pid or found['creation'] != row['creation']:
                                    if found not in row['foreign_seen']:
                                        row['foreign_seen'].append(found)
                            try:
                                disk_guard()
                            except AssertionError:
                                low_disk = True
                            save()
                            # Текущий собственный процесс не убивается; новые sample при проблемах не стартуют.
                            time.sleep(1)
                        row['exit'] = process.returncode
                        row['process_wall_seconds'] = time.perf_counter() - process_started
                    row['sha_after'] = sha(scene)
                    row['low_disk'] = low_disk
                    save()
                    assert row['sha_after'] == expected, 'FROZEN_SCENE_CHANGED after sample'
                    cpu_after = foreign_cpu(foreign)
                    row['foreign_idle'] = [dict(pid=k[0], cpu_before=cpu_before[k], cpu_after=cpu_after[k]) for k in cpu_before]
                    busy = [r for r in row['foreign_idle'] if r['cpu_before'] is not None and r['cpu_after'] is not None and r['cpu_after'] - r['cpu_before'] > 0.75]
                    appeared = [f for f in row['foreign_seen'] if (f['pid'], f['creation']) not in cpu_before]
                    save()
                    assert not busy and not appeared, 'foreign Blender busy or started during the sample; sample rejected'
                    assert not low_disk, 'DISK_STOP; no further samples'
                    assert row['exit'] == 0 and result.is_file(), 'operator sample failed'
                    print(label, 'complete', flush=True)
        import statistics
        state['summary'] = {}
        for case in cases:
            rows = {side: [json.loads((out / f'{case}_{n}_{side}.json').read_text()) for n in range(1, a.reps + 1)]
                    for side in ('base', 'branch')}
            assert len({r['selection_sha'] for group in rows.values() for r in group}) == 1
            assert len({r['decal']['digest'] for group in rows.values() for r in group}) == 1
            state['summary'][case] = {side: {'samples': [r['button_wall'] for r in group],
                'median': statistics.median(r['button_wall'] for r in group)} for side, group in rows.items()}
        state['accepted'] = True
        save()
        return 0
    except Exception as error:
        state.update(accepted=False, failure=f'{type(error).__name__}: {error}')
        save()
        raise


if __name__ == '__main__':
    raise SystemExit(main())
