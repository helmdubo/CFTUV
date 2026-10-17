"""Whole-corpus ANGULAR replay (frozen replay_chunks_v2 runner, guarded per chunk) into a D: output dir.

Usage: replay.py --site SITE --profile PROFILE --out OUTDIR [--archive A] [--max-chunks N]
Exit 0 only when status COMPLETE_REPLAY_SCOPE and accepted is True.
"""
import argparse, hashlib, json, os, subprocess, sys
from pathlib import Path

A = Path('D:/CFTUV-export-20261015-evidence/angular')
PY = 'C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe'
SOURCE = 'D:/CFTUV-export-angular-20261016'
RUNNER = A / 'replay_chunks_v2.py'
RUNNER_SHA = '670115859fda70fb0220cc6d4b2c49808f932b6f094ee9313e871380d0470b5c'
GUARD = 'D:/CFTUV-export-20261015-evidence/corpus/owned_job_guard_v2.py'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--site', required=True); p.add_argument('--profile', required=True); p.add_argument('--out', required=True)
    p.add_argument('--archive', default='*'); p.add_argument('--max-chunks', type=int)
    a = p.parse_args()
    assert sha(RUNNER) == RUNNER_SHA, 'RUNNER_CHANGED'
    out = Path(a.out); log = Path(str(out) + '-launch'); log.mkdir(parents=True, exist_ok=False); (log / 'tmp').mkdir()
    env = {k: v for k, v in os.environ.items() if not k.startswith('CFTUV_')}
    env.update(PYTHONPATH=';'.join([a.site, 'C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages', SOURCE + '/kernel/src', SOURCE + '/tools', str(A)]),
               PYTHONSAFEPATH='1', PYTHONDONTWRITEBYTECODE='1', PYTHONHASHSEED='0', TEMP=str(log / 'tmp'), TMP=str(log / 'tmp'))
    command = [PY, '-B', str(RUNNER), '--pair-root', str(A / 'paired-capture-01'), '--archive', a.archive]
    if a.max_chunks:
        command += ['--max-chunks', str(a.max_chunks)]
    command += ['--source', SOURCE, '--site', a.site, '--approved-profile', a.profile, '--approved-profile-sha256', sha(a.profile), '--guard', GUARD, '--out', str(out)]
    with (log / 'stdout.txt').open('w') as so, (log / 'stderr.txt').open('w') as se:
        proc = subprocess.Popen(command, env=env, cwd=SOURCE, stdout=so, stderr=se)
        (log / 'process.json').write_text(json.dumps(dict(pid=proc.pid, argv=command), indent=2))
        code = proc.wait()
    state = json.loads((out / 'replay.json').read_text()) if (out / 'replay.json').exists() else {}
    calls = []
    for chunk in sorted(out.glob('*-pair')):
        c = out / chunk.name / 'chunk.json'
        if c.exists():
            calls += [v['status'] for v in json.loads(c.read_text()).get('calls', [])]
    from collections import Counter
    summary = dict(exit=code, status=state.get('status'), accepted=state.get('accepted'), tasks=state.get('tasks'), calls=state.get('calls'),
                   chunks_done=len(state.get('chunks', [])), call_status=dict(Counter(calls)),
                   unsupported=sum(c.get('unsupported', 0) for c in state.get('chunks', [])))
    (log / 'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary))
    raise SystemExit(0 if code == 0 and summary['status'] == 'COMPLETE_REPLAY_SCOPE' and summary['accepted'] is True else 1)


if __name__ == '__main__':
    main()
