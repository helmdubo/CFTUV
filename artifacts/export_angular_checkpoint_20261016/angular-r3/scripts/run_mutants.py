"""Mutation gate driver for the Rust ANGULAR code. Never touches the checkout: mutants live in a copy of native/ on D:.

Stage 1 (crate mutants): cargo test -p cftuv-angular. Stage 2 (survivors and PyO3 mutants): fast wheel + whole-corpus replay.
Results appended to results.jsonl; a mutant id already in results.jsonl is skipped (resumable).
"""
import json, os, shutil, subprocess, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from mutants import MUTANTS

W = Path('D:/cftuv-angular-gates-20261017')
SRC_NATIVE = Path('D:/CFTUV-export-angular-20261016/native')
MUT = W / 'mut/native'
RESULTS = W / 'mut/results.jsonl'
BP = 'C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe'
TC = 'C:/Users/helmd/.rustup/toolchains/stable-x86_64-pc-windows-msvc/bin'
PROFILE = W / 'cand/profile-test-711a-0eee.json'
CAND = W / 'cand/light-711a-pre.json'
MAKE_PROFILE = Path('D:/CFTUV-export-angular-20261016/artifacts/export_angular_checkpoint_20261016/angular-r2/make_test_profile.py')
KERNEL_IDENTITY = json.loads(PROFILE.read_text())['kernel_identity']


def env():
    e = {k: v for k, v in os.environ.items() if not k.startswith('CFTUV_')}
    e.update(TEMP=str(W / 'tmp'), TMP=str(W / 'tmp'), PATH=TC + ';' + e.get('PATH', ''), RUSTC=TC + '/rustc.exe', CARGO=TC + '/cargo.exe',
             PYO3_PYTHON=BP, CARGO_BUILD_JOBS='6', PYTHONDONTWRITEBYTECODE='1')
    return e


def run(cmd, log, **kw):
    with open(log, 'w', encoding='utf-8', errors='replace') as stream:
        return subprocess.call(cmd, stdout=stream, stderr=subprocess.STDOUT, **kw)


def fresh_copy():
    if MUT.exists():
        return
    shutil.copytree(SRC_NATIVE, MUT, ignore=shutil.ignore_patterns('target', '__pycache__'))


def main():
    fresh_copy()
    done = set()
    if RESULTS.exists():
        done = {json.loads(line)['id'] for line in RESULTS.read_text().splitlines() if line.strip()}
    only = set(sys.argv[1:])
    for mid, rel, old, new in MUTANTS:
        if mid in done or (only and mid not in only):
            continue
        path = MUT / rel
        original = path.read_bytes()
        text = original.decode('utf-8')
        assert text.count(old) == 1, (mid, text.count(old))
        logs = W / 'mut/logs' / mid
        logs.mkdir(parents=True, exist_ok=True)
        record = dict(id=mid, file=rel, old=old, new=new, started=time.time())
        try:
            path.write_bytes(text.replace(old, new).encode('utf-8'))
            if rel.startswith('cftuv-angular/'):
                e = env(); e['CARGO_TARGET_DIR'] = str(W / 'target-test')
                code = run([TC + '/cargo.exe', 'test', '--offline', '--locked', '-p', 'cftuv-angular'], logs / 'cargo-test.log', cwd=MUT, env=e)
                log = (logs / 'cargo-test.log').read_text(encoding='utf-8', errors='replace')
                record['cargo_test_exit'] = code
                record['cargo_compile_error'] = code != 0 and 'error[' in log and 'test result' not in log
                if code != 0:
                    record['verdict'] = 'STILLBORN' if record['cargo_compile_error'] else 'KILLED_BY_RUST_TESTS'
                    continue
            # stage 2: fast wheel + corpus replay
            site_root = W / 'mut/site' / mid
            if site_root.exists():
                shutil.rmtree(site_root)
            e = env(); e['CARGO_TARGET_DIR'] = str(W / 'target-fast')
            code = run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', str(W / 'scripts/build_wheel.ps1'), '-Native', str(MUT), '-Out', str(site_root),
                        '-Target', str(W / 'target-fast'), '-Fast'], logs / 'wheel.log', env=e)
            log = (logs / 'wheel.log').read_text(encoding='utf-8', errors='replace')
            if not (site_root / 'site/cftuv_native').exists():
                record['verdict'] = 'STILLBORN' if 'error[' in log else 'BUILD_FAILED'
                continue
            ident = subprocess.check_output([BP, '-B', '-c', 'import cftuv_native as n;print(n.native_build_id())'], env={**env(), 'PYTHONPATH': str(site_root / 'site')}, text=True).strip()
            record['native_build_id'] = ident
            prof = site_root / 'profile.json'
            subprocess.check_call([sys.executable, str(MAKE_PROFILE), str(PROFILE), str(CAND), ident, KERNEL_IDENTITY, str(prof)])
            out = W / 'mut/replay' / mid
            code = run([BP, '-B', str(W / 'scripts/replay.py'), '--site', str(site_root / 'site'), '--profile', str(prof), '--out', str(out)], logs / 'replay.log', env=env())
            summary = json.loads((Path(str(out) + '-launch') / 'summary.json').read_text())
            record['replay'] = summary
            record['verdict'] = 'SURVIVED' if code == 0 else 'KILLED_BY_CORPUS_REPLAY'
        except Exception as error:
            record['verdict'] = 'HARNESS_ERROR'
            record['error'] = repr(error)
        finally:
            path.write_bytes(original)
            record['seconds'] = round(time.time() - record['started'], 1)
            with RESULTS.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(record) + '\n')
            print(mid, record.get('verdict'), record['seconds'], flush=True)
            site_root = W / 'mut/site' / mid
            if site_root.exists():
                shutil.rmtree(site_root, ignore_errors=True)
    print('DONE', flush=True)


if __name__ == '__main__':
    main()
