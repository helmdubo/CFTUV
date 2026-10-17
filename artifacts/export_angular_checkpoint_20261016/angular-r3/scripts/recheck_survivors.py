"""Re-run the survivors of run_mutants.py against the NEW tests (crate tests copied from the checkout; the PyO3 wire test file).

Crate mutants: cargo test -p cftuv-angular. PyO3 mutants: fast wheel + pytest tests/test_native_angular_patch_wire.py on the mutant site.
Results to recheck.jsonl.
"""
import json, shutil, subprocess, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import run_mutants as rm
from mutants import MUTANTS

W = rm.W
REPO = Path('D:/CFTUV-export-angular-20261016')
OUT = W / 'mut/recheck.jsonl'


def main():
    for name in ('admission.rs', 'stages.rs', 'reference.rs', 'decimal_text.rs'):
        shutil.copyfile(REPO / 'native/cftuv-angular/tests' / name, rm.MUT / 'cftuv-angular/tests' / name)
    survivors = [json.loads(l)['id'] for l in rm.RESULTS.read_text().splitlines() if l.strip() and json.loads(l)['verdict'] == 'SURVIVED']
    survivors = sys.argv[1:] or survivors
    for mid, rel, old, new in MUTANTS:
        if mid not in survivors:
            continue
        path = rm.MUT / rel
        original = path.read_bytes()
        text = original.decode('utf-8')
        assert text.count(old) == 1
        logs = W / 'mut/logs' / mid
        logs.mkdir(parents=True, exist_ok=True)
        record = dict(id=mid, started=time.time())
        try:
            path.write_bytes(text.replace(old, new).encode('utf-8'))
            if rel.startswith('cftuv-angular/'):
                e = rm.env(); e['CARGO_TARGET_DIR'] = str(W / 'target-test')
                code = rm.run([rm.TC + '/cargo.exe', 'test', '--offline', '--locked', '-p', 'cftuv-angular', '--no-fail-fast'], logs / 'recheck-cargo.log', cwd=rm.MUT, env=e)
                log = (logs / 'recheck-cargo.log').read_text(encoding='utf-8', errors='replace')
                record['failing'] = sorted({line.split()[0] for line in log.splitlines() if line.startswith('    ') and '::' not in line and line.strip() and not line.strip().startswith('left') and not line.strip().startswith('right')} ) if code else []
                record['verdict'] = 'KILLED_BY_NEW_RUST_TESTS' if code else 'SURVIVED'
            else:
                site_root = W / 'mut/site' / mid
                if site_root.exists():
                    shutil.rmtree(site_root)
                e = rm.env(); e['CARGO_TARGET_DIR'] = str(W / 'target-fast')
                rm.run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', str(W / 'scripts/build_wheel.ps1'), '-Native', str(rm.MUT), '-Out', str(site_root),
                        '-Target', str(W / 'target-fast'), '-Fast'], logs / 'recheck-wheel.log', env=e)
                assert (site_root / 'site/cftuv_native').exists(), 'BUILD'
                e = rm.env(); e['PYTHONPATH'] = str(site_root / 'site') + ';C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages'
                code = rm.run([rm.BP, '-B', '-m', 'pytest', '-q', '-p', 'no:cacheprovider', '--basetemp', str(W / 'mut/pt' / mid), str(REPO / 'tests/test_native_angular_patch_wire.py')],
                              logs / 'recheck-pytest.log', cwd=str(REPO), env=e)
                log = (logs / 'recheck-pytest.log').read_text(encoding='utf-8', errors='replace')
                record['summary'] = log.strip().splitlines()[-1] if log.strip() else ''
                record['verdict'] = 'KILLED_BY_WIRE_TESTS' if code else 'SURVIVED'
                shutil.rmtree(site_root, ignore_errors=True)
        except Exception as error:
            record['verdict'] = 'HARNESS_ERROR'; record['error'] = repr(error)
        finally:
            path.write_bytes(original)
            record['seconds'] = round(time.time() - record['started'], 1)
            with OUT.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(record) + '\n')
            print(mid, record['verdict'], record.get('summary', ''), flush=True)
    print('DONE', flush=True)


if __name__ == '__main__':
    main()
