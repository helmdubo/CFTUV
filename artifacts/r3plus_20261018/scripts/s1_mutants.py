"""Single-site mutants of DENSITY_SHAPES_BOUND_ONCE_V1 (R3+ S1): the bind-once class check of cftuv_native/density_pin.py and the unbind in pin.refresh.

python s1_mutants.py --repo D:/CFTUV-r3plus --work D:/r3plus-scratch/s1mut --out results.json
A copy of the tree (tests/, kernel/src, the cftuv_native package; no wheel: tests/test_native_density_pin.py loads the package from source) per run;
a mutant is KILLED when tests/test_native_density_pin.py fails; the baseline must pass first.
Equivalent sites (not listed): the order of the two lookups in `_live_module` (sys.modules first is what import_module returns for a loaded module); the method OBJECT
identity next to its `__code__` identity (another function with the same code object has the same source, so the per-call reference gives the same digest too).
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

D = 'native/cftuv-python/python/cftuv_native/density_pin.py'
P = 'native/cftuv-python/python/cftuv_native/pin.py'

MUTANTS = [
    ('B01_bound_identity_not_checked', D, 'if bound is not None and bound[0] is cls and all(', 'if bound is not None and all('),
    ('B02_method_identity_not_checked', D, 'and all(getattr(cls, name, None) is method and getattr(method, "__code__", None) is code for name, method, code in bound[1]):', ':'),
    ('B03_method_code_not_checked', D, 'getattr(cls, name, None) is method and getattr(method, "__code__", None) is code for', 'getattr(cls, name, None) is method for'),
    ('B05_never_bound', D, '    _BOUND[(file, qualified)] = (cls, _methods(cls), digest)\n', ''),
    ('B06_unbind_keeps_the_binding', D, '    _BOUND.clear()\n', '    pass\n'),
    ('B07_refresh_does_not_unbind', P, '    unbind()\n', '    pass\n'),
    ('B08_bound_digest_ignored_on_rebind', D, '    digest = _class_digest(cls)\n    _BOUND', '    digest = next(row[2] for row in CLASSES if row[:2] == (file, qualified))\n    _BOUND'),
    ('B09_bound_without_methods', D, '    _BOUND[(file, qualified)] = (cls, _methods(cls), digest)', '    _BOUND[(file, qualified)] = (cls, (), digest)'),
    ('B10_one_method_name_only', D, '_METHOD_NAMES = ("__post_init__", "validate_candidate")', '_METHOD_NAMES = ("__post_init__",)'),
    ('B11_class_failure_not_named', D, '                failures.append(file + ":" + qualified + " (shape)")', '                pass'),
    ('B12_missing_class_not_named', D, '            failures.append(file + ":" + qualified + " (shape missing/invalid)")', '            pass'),
    ('B13_first_bind_key_shared', D, '    bound = _BOUND.get((file, qualified))', '    bound = _BOUND.get(file)'),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', required=True)
    parser.add_argument('--work', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    repo, work = Path(args.repo), Path(args.work)
    assert work.drive.lower() == 'd:'
    tree = work / 'tree'
    if tree.exists():
        shutil.rmtree(tree)
    for part in ('tests', 'kernel/src', 'native/cftuv-python/python/cftuv_native'):
        shutil.copytree(repo / part, tree / part, ignore=shutil.ignore_patterns('__pycache__', 'data'), copy_function=shutil.copyfile)
    results = []
    for ident, file, old, new in [('BASELINE', None, None, None)] + MUTANTS:
        started = time.time()
        if file is not None:
            text = (repo / file).read_text(encoding='utf-8')
            assert text.count(old) == 1, (ident, text.count(old))
            (tree / file).write_text(text.replace(old, new), encoding='utf-8')
        env = dict(os.environ, PYTHONPATH='', PYTHONDONTWRITEBYTECODE='1', TMP=str(work / 'tmp'), TEMP=str(work / 'tmp'))
        (work / 'tmp').mkdir(exist_ok=True)
        test = subprocess.run([sys.executable, '-m', 'pytest', 'tests/test_native_density_pin.py', '-x', '-q', '-p', 'no:cacheprovider', '--basetemp', str(work / 'pt')],
                              cwd=tree, env=env, capture_output=True, text=True, encoding='utf-8', errors='replace')
        failed = test.returncode != 0
        outcome = ('BASELINE_FAILED' if ident == 'BASELINE' else 'killed') if failed else ('passed' if ident == 'BASELINE' else 'SURVIVED')
        first = next((line for line in test.stdout.splitlines() if line.startswith('FAILED') or line.startswith('ERROR')), '')
        row = dict(id=ident, outcome=outcome, seconds=round(time.time() - started, 1), first_failure=first[:300])
        results.append(row)
        print(json.dumps(row), flush=True)
        if file is not None:
            shutil.copyfile(repo / file, tree / file)
        if ident == 'BASELINE' and outcome != 'passed':
            print(test.stdout[-3000:])
            break
    Path(args.out).write_text(json.dumps(results, indent=1), encoding='utf-8')
    survivors = [row['id'] for row in results if row['outcome'] not in ('passed', 'killed')]
    print('survivors/failures:', survivors)
    return 0 if not survivors else 1


if __name__ == '__main__':
    raise SystemExit(main())
