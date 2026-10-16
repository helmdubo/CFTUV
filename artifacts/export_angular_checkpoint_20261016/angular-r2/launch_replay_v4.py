"""Bounded/whole ANGULAR replay launcher v4. Continuation depends on acceptance, never on exit code alone.

v2 continued from rounded to building after exit 0 although replay.json said accepted=false.
`continuation` is the only gate between scopes and is unit-tested (test_launch_replay_v4.py).
Usage: launch_replay_v4.py --tag T --profile P --profile-sha S --site SITE --scope rounded:1 building:1 ...
       a scope is ARCHIVE[:MAX_CHUNKS]; ARCHIVE '*' with no bound is the whole corpus.
"""
import argparse, hashlib, json, os, subprocess, sys
from pathlib import Path

ROOT = Path('D:/CFTUV-export-20261015-evidence')
A = ROOT / 'angular'
OUT = ROOT / 'angular-r2'
PY = 'C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe'
SOURCE = 'D:/CFTUV-export-angular-20261016'
RUNNER = A / 'replay_chunks_v2.py'
RUNNER_SHA = '670115859fda70fb0220cc6d4b2c49808f932b6f094ee9313e871380d0470b5c'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def continuation(exit_code, state):
    """(continue, reason). Only exit 0 AND a complete scope AND accepted is True continues."""
    if type(exit_code) is not int or exit_code != 0:
        return False, 'STOP_AFTER_FAILED_SCOPE'
    if type(state) is not dict or state.get('status') != 'COMPLETE_REPLAY_SCOPE':
        return False, 'STOP_AFTER_INCOMPLETE_SCOPE'
    if state.get('accepted') is not True:
        return False, 'STOP_AFTER_NONACCEPTED_SCOPE'
    return True, 'CONTINUE'


def run_scope(args, archive, bound):
    sys.path.insert(0, str(ROOT / 'corpus'))
    import owned_job_guard_v2 as guard
    assert sha(RUNNER) == RUNNER_SHA, 'RUNNER_CHANGED'
    name = 'replay-' + ('all' if archive == '*' else archive) + ('' if bound is None else '-c' + str(bound)) + '-' + args.tag
    out = OUT / name; log = OUT / (name + '-launch')
    log.mkdir(exist_ok=False); (log / 'tmp').mkdir()
    env = {k: v for k, v in os.environ.items() if not k.startswith('CFTUV_')}
    env.update(PYTHONPATH=';'.join([args.site, 'C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages',
                                    SOURCE + '/kernel/src', SOURCE + '/tools', str(A)]),
               PYTHONSAFEPATH='1', PYTHONDONTWRITEBYTECODE='1', PYTHONHASHSEED='0', TEMP=str(log / 'tmp'), TMP=str(log / 'tmp'))
    command = [PY, '-B', str(RUNNER), '--pair-root', str(A / 'paired-capture-01'), '--archive', archive,
               '--source', SOURCE, '--site', args.site, '--approved-profile', args.profile,
               '--approved-profile-sha256', args.profile_sha, '--guard', str(ROOT / 'corpus/owned_job_guard_v2.py'), '--out', str(out)]
    if bound is not None:
        command[command.index('--source'):command.index('--source')] = ['--max-chunks', str(bound)]
    guard.disks(['C:/', 'D:/', 'E:/'], int(1.5 * 1024 ** 3))
    with (log / 'stdout.txt').open('w') as stdout, (log / 'stderr.txt').open('w') as stderr:
        proc = subprocess.Popen(command, env=env, cwd=SOURCE, stdout=stdout, stderr=stderr)
        receipt = dict(pid=proc.pid, creation_filetime=guard.creation_filetime(proc._handle), argv=command, exit=None, runner_sha_before=RUNNER_SHA)
        (log / 'process.json').write_text(json.dumps(receipt, indent=2))
        receipt['exit'] = proc.wait()
    receipt['runner_sha_after'] = sha(RUNNER)
    (log / 'process.json').write_text(json.dumps(receipt, indent=2))
    state = None
    if (out / 'replay.json').exists():
        state = json.loads((out / 'replay.json').read_text())
    if receipt['runner_sha_after'] != RUNNER_SHA:
        return False, 'STOP_RUNNER_CHANGED', state
    go, reason = continuation(receipt['exit'], state)
    return go, reason, state


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('tag', 'profile', 'profile-sha', 'site'):
        p.add_argument('--' + key, required=True)
    p.add_argument('--scope', nargs='+', required=True)
    args = p.parse_args()
    for scope in args.scope:
        archive, _, bound = scope.partition(':')
        go, reason, state = run_scope(args, archive, int(bound) if bound else None)
        summary = None if state is None else {k: state.get(k) for k in ('status', 'accepted', 'tasks', 'calls')}
        if state is not None:
            summary['unsupported'] = sum(c.get('unsupported', 0) for c in state.get('chunks', []))
        print(json.dumps(dict(scope=scope, reason=reason, state=summary)), flush=True)
        if not go:
            raise SystemExit(reason)


if __name__ == '__main__':
    main()
