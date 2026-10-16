"""Windows owned-child guard; forced OS termination of controller is outside guarantee."""
import argparse
import ctypes
from ctypes import wintypes
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

K = ctypes.WinDLL('kernel32', use_last_error=True)
K.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
K.OpenProcess.restype = wintypes.HANDLE
K.GetProcessTimes.argtypes = [wintypes.HANDLE] + [ctypes.POINTER(wintypes.FILETIME)] * 4
K.TerminateProcess.argtypes = [wintypes.HANDLE, wintypes.UINT]
K.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
K.CloseHandle.argtypes = [wintypes.HANDLE]

def save(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding='utf8')
    deadline = time.monotonic() + 0.25
    while True:
        try:
            os.replace(tmp, path)
            break
        except OSError as error:
            if os.name != 'nt' or getattr(error, 'winerror', None) not in (5, 32, 33) or time.monotonic() >= deadline:
                raise
            time.sleep(min(0.01, max(0, deadline - time.monotonic())))

def creation_filetime(handle):
    values = [wintypes.FILETIME() for _ in range(4)]
    if not K.GetProcessTimes(handle, *(ctypes.byref(v) for v in values)):
        raise ctypes.WinError(ctypes.get_last_error())
    return (values[0].dwHighDateTime << 32) | values[0].dwLowDateTime

class Owned:
    """Read-only identity helper for short child receipts; guard cleanup uses Popen HANDLE."""
    def __init__(self, pid):
        self.handle = K.OpenProcess(0x1000, False, pid)
        if not self.handle:
            raise ctypes.WinError(ctypes.get_last_error())
        try:
            self.filetime = creation_filetime(self.handle)
        except BaseException:
            self.close()
            raise
    def close(self):
        if self.handle:
            K.CloseHandle(self.handle)
            self.handle = None

def identity(pid, argv):
    owned = Owned(pid)
    return owned, dict(pid=pid, creation_filetime=owned.filetime, argv=argv, exit=None)

def disks(drives, floor):
    values = {d: shutil.disk_usage(d).free for d in drives}
    if any(v < floor for v in values.values()):
        raise RuntimeError('DISK_FLOOR_CROSSED ' + json.dumps(values))
    return values

def stop_held(proc):
    # This retained creation HANDLE cannot accidentally target a recycled PID.
    state = K.WaitForSingleObject(int(proc._handle), 0)
    if state == 0:
        return
    # WAIT_FAILED is an error, never an exit; cleanup still attempts owned termination.
    if not K.TerminateProcess(int(proc._handle), 91):
        raise ctypes.WinError(ctypes.get_last_error())

def run(args):
    out = Path(args.receipt)
    out.mkdir(parents=True, exist_ok=False)
    held, controller = identity(os.getpid(), sys.argv)
    held.close()
    receipt = dict(status='STARTING', started=time.time(), controller=controller, processes=[],
                   limitation='Forced OS termination of controller not survived; no descendant tree ownership claimed')
    proc = None
    try:
        floor = int(args.floor_gib * 1024**3)
        receipt['disk_before'] = disks(args.drives, floor)
        env = os.environ.copy()
        for value in args.env:
            key, val = value.split('=', 1)
            env[key] = val
        command = args.command[1:] if args.command[:1] == ['--'] else args.command
        if not command:
            raise ValueError('EMPTY_COMMAND')
        with (out / 'stdout.txt').open('wb') as stdout, (out / 'stderr.txt').open('wb') as stderr:
            proc = subprocess.Popen(command, cwd=args.cwd, env=env, stdout=stdout, stderr=stderr)
            row = dict(pid=proc.pid, argv=command, creation_filetime=None, exit=None)
            receipt['processes'].append(row)
            # Even identity read failure leaves the original Popen HANDLE for finally cleanup.
            row['creation_filetime'] = creation_filetime(int(proc._handle))
            expected = row['creation_filetime'] + 1 if args.test_fault == 'wrong-filetime' else args.expect_filetime
            row['expected_creation_filetime'] = expected
            if expected is not None and expected != row['creation_filetime']:
                raise RuntimeError('OWNED_FILETIME_MISMATCH_REFUSE')
            receipt['status'] = 'RUNNING'
            save(out / 'process.json', receipt)
            started = time.monotonic()
            while True:
                if args.timeout is not None and time.monotonic() - started >= args.timeout:
                    raise RuntimeError('OWNED_JOB_TIMEOUT')
                if args.test_fault == 'lowdisk' and time.monotonic() - started >= .2:
                    raise RuntimeError('DISK_FLOOR_CROSSED_INJECTED')
                if args.test_fault == 'crash' and time.monotonic() - started >= .2:
                    raise RuntimeError('INJECTED_GUARD_READ_FAILURE')
                receipt['disk_last'] = disks(args.drives, floor)
                receipt['heartbeat'] = time.time()
                save(out / 'process.json', receipt)
                try:
                    proc.wait(timeout=args.poll)
                    break
                except subprocess.TimeoutExpired:
                    pass
            receipt['disk_after'] = disks(args.drives, floor)
            row['exit'] = proc.returncode
            if proc.returncode:
                raise RuntimeError('OWNED_JOB_FAILED')
            receipt.update(status='COMPLETE', exit=0)
    except BaseException:
        receipt.update(status='FAILED_CLOSED', failure=traceback.format_exc(), exit=1)
    finally:
        if proc is not None:
            try:
                stop_held(proc)
                proc.wait(timeout=15)
                receipt['processes'][0]['exit'] = proc.returncode
                receipt['owned_child_reaped'] = True
            except BaseException:
                receipt.update(status='CLEANUP_FAILED_NOT_ACCEPTED', exit=1, cleanup_failure=traceback.format_exc())
        receipt.update(ended=time.time())
        receipt['controller']['exit'] = receipt.get('exit', 1)
        save(out / 'process.json', receipt)
    return receipt['exit']

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['run'])
    parser.add_argument('--receipt', required=True)
    parser.add_argument('--cwd', required=True)
    parser.add_argument('--env', action='append', default=[])
    parser.add_argument('--floor-gib', type=float, default=1.5)
    parser.add_argument('--drives', nargs='+', default=['C:/', 'D:/', 'E:/'])
    parser.add_argument('--poll', type=float, default=1)
    parser.add_argument('--expect-filetime', type=int)
    parser.add_argument('--timeout', type=float)
    parser.add_argument('--test-fault', choices=['lowdisk', 'crash', 'wrong-filetime'], help=argparse.SUPPRESS)
    if '--' not in sys.argv[1:]:
        parser.error('explicit -- command separator required')
    split = sys.argv.index('--')
    args = parser.parse_args(sys.argv[1:split])
    args.command = sys.argv[split + 1:]
    if not 0 < args.poll <= 1 or args.floor_gib < 1.5:
        parser.error('poll must be (0,1], floor must be >=1.5 GiB')
    if args.timeout is not None and args.timeout <= 0:
        parser.error('timeout must be positive')
    if args.test_fault:
        command = args.command[1:] if args.command[:1] == ['--'] else args.command
        if command != [sys.executable, '-c', 'import time; time.sleep(2)']:
            parser.error('fault injection requires exact harmless sleep child')
    return run(args)

if __name__ == '__main__':
    raise SystemExit(main())

