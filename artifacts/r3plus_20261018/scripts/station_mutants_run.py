"""Runs the single-site mutants of station_mutants.py: a copy of native/ per run (D:), a fast wheel (same sources, LTO off), then tests/test_native_chain_station.py with that wheel.

python station_mutants_run.py --repo D:/CFTUV-r3plus --work D:/r3plus-scratch/mut --out results.json [--only ID,ID]
A mutant is KILLED when the suite fails; the baseline (no mutant) must pass first.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from station_mutants import MUTANTS  # noqa: E402

BUILD = Path('D:/r3plus-scratch/scripts/build_wheel.ps1')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', required=True)
    parser.add_argument('--work', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--only', default='')
    args = parser.parse_args()
    repo, work = Path(args.repo), Path(args.work)
    assert work.drive.lower() == 'd:'
    pristine = repo / 'native'
    tree = work / 'native'
    if tree.exists():
        shutil.rmtree(tree)
    # copyfile, not copy2: fresh mtimes, so cargo never takes the pristine sources for older than the last mutant's build in the shared target
    shutil.copytree(pristine, tree, ignore=shutil.ignore_patterns('target'), copy_function=shutil.copyfile)
    only = set(filter(None, args.only.split(',')))
    runs = [('BASELINE', None, None, None)] + [m for m in MUTANTS if not only or m[0] in only]
    results = []
    for ident, file, old, new in runs:
        started = time.time()
        if file is not None:
            text = (pristine / file).read_text(encoding='utf-8')
            assert text.count(old) == 1, (ident, text.count(old))
            (tree / file).write_text(text.replace(old, new), encoding='utf-8')
        out = work / 'wheel'
        if out.exists():
            shutil.rmtree(out)
        build = subprocess.run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', str(BUILD), '-Native', str(tree), '-Out', str(out), '-Target', str(work / 'target'), '-Fast'],
                               capture_output=True, text=True, encoding='utf-8', errors='replace')
        if build.returncode != 0 or not (out / 'site' / 'cftuv_native').is_dir():
            row = dict(id=ident, outcome='build_failed', seconds=round(time.time() - started, 1), tail=build.stdout[-1500:] + build.stderr[-1500:])
        else:
            env = dict(os.environ, PYTHONPATH=str(out / 'site'), TMP=str(work / 'tmp'), TEMP=str(work / 'tmp'))
            (work / 'tmp').mkdir(exist_ok=True)
            test = subprocess.run([sys.executable, '-m', 'pytest', 'tests/test_native_chain_station.py', '-x', '-q', '-p', 'no:cacheprovider', '--basetemp', str(work / 'pt')],
                                  cwd=repo, env=env, capture_output=True, text=True, encoding='utf-8', errors='replace')
            cargo = subprocess.run(['cargo', 'test', '-p', 'cftuv-station', '--offline', '--quiet'], cwd=tree, env=dict(os.environ, CARGO_TARGET_DIR=str(work / 'target-test')),
                                   capture_output=True, text=True, encoding='utf-8', errors='replace')
            failed = test.returncode != 0 or cargo.returncode != 0
            outcome = ('BASELINE_FAILED' if ident == 'BASELINE' else 'killed') if failed else ('passed' if ident == 'BASELINE' else 'SURVIVED')
            first = next((line for line in test.stdout.splitlines() if line.startswith('FAILED') or line.startswith('ERROR')), '') or ('cargo test: ' + cargo.stdout[-300:] if cargo.returncode else '')
            row = dict(id=ident, outcome=outcome, seconds=round(time.time() - started, 1), first_failure=first[:300])
        results.append(row)
        print(json.dumps(row), flush=True)
        if file is not None:
            shutil.copyfile(pristine / file, tree / file)
        if ident == 'BASELINE' and row['outcome'] != 'passed':
            break
        Path(args.out).write_text(json.dumps(results, indent=1), encoding='utf-8')
    Path(args.out).write_text(json.dumps(results, indent=1), encoding='utf-8')
    survivors = [row['id'] for row in results if row['outcome'] not in ('passed', 'killed')]
    print('survivors/failures:', survivors)
    return 0 if not survivors else 1


if __name__ == '__main__':
    raise SystemExit(main())
