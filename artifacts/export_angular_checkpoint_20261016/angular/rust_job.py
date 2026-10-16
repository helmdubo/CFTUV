"""Owned ANGULAR Rust gates; preparation only until parent releases heavy slot.
Uses the reviewed Job Object helper so killing the held Python root closes the
sole Job HANDLE and terminates only its Cargo/rustc/link descendants.
"""
import argparse,hashlib,json,os,subprocess,sys,time
from pathlib import Path

SOURCE=Path('D:/CFTUV-export-angular-20261016')
WORK=Path(__file__).resolve().parent
CORPUS=WORK.parent/'corpus'
TARGET=Path('C:/Users/helmd/AppData/Local/Temp/cftuv-r3-density-target')
TOOLCHAIN=Path('C:/Users/helmd/.rustup/toolchains/stable-x86_64-pc-windows-msvc/bin')
BP=Path('C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe')
for name,digest in (('strict_process_job.py','c765659f1173faf069aac747c2a8462c69c00722cc8b76140902fc426ab5f2f9'),('owned_job_guard_v2.py','90fd91af228d696bd181f1177b3dd1f5e1c6745e79b0707e4383fd4af378eb6b'),('owned_job_guard.py','9cd1384d53d27fe3523023a9341af1abdc9932503689d252de3376ac874fe05b')):
    assert hashlib.sha256((CORPUS/name).read_bytes()).hexdigest()==digest,'REVIEWED_HELPER_CHANGED '+name
sys.path.insert(0,str(CORPUS))
from strict_process_job import setup
from owned_job_guard_v2 import save

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def git(*args):return subprocess.check_output(['git','-C',str(SOURCE),*args],text=True).strip()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--step',choices=('angular','pyo3'),required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--expected-head',required=True)
    args=p.parse_args();out=args.out.resolve()
    assert out.is_relative_to(WORK)and out!=WORK,'OWN_EVIDENCE_OUTPUT_REQUIRED'
    assert Path(os.environ.get('CARGO_TARGET_DIR','')).resolve()==TARGET.resolve(),'MANDATED_C_TARGET_REQUIRED'
    assert Path(os.environ.get('RUSTC','')).resolve()==(TOOLCHAIN/'rustc.exe').resolve(),'DIRECT_RUSTC_REQUIRED'
    assert Path(os.environ.get('PYO3_PYTHON','')).resolve()==BP.resolve(),'PYO3_PY311_REQUIRED'
    for name in ('TEMP','TMP'):
        assert Path(os.environ[name]).resolve().is_relative_to(WORK),'D_TEMP_REQUIRED'
    out.mkdir(parents=True,exist_ok=False)
    finish=setup(out/'job.json')
    command=[str(TOOLCHAIN/'cargo.exe'), 'test'if args.step=='angular'else'check',
             '--locked','--offline','-p','cftuv-angular'if args.step=='angular'else'cftuv-python',
             '--tests'if args.step=='angular'else'--all-targets','--jobs','1']
    record=dict(status='STARTING',step=args.step,argv=command,source=str(SOURCE),expected_head=args.expected_head,
                target=str(TARGET),bootstrap_sha256=sha(__file__),job_helper_sha256=sha(CORPUS/'strict_process_job.py'),
                cargo_sha256=sha(TOOLCHAIN/'cargo.exe'),rustc_sha256=sha(TOOLCHAIN/'rustc.exe'),started=time.time())
    code=1
    try:
        assert git('rev-parse','HEAD')==args.expected_head,'SOURCE_HEAD_CHANGED'
        assert not git('status','--porcelain'),'SOURCE_NOT_CLEAN'
        save(out/'result.json',record)
        child=subprocess.Popen(command,cwd=SOURCE/'native')
        record['cargo_pid']=child.pid;save(out/'result.json',record)
        code=child.wait()
        assert git('rev-parse','HEAD')==args.expected_head and not git('status','--porcelain'),'SOURCE_CHANGED_DURING_RUST_GATE'
        record.update(status='COMPLETE'if code==0 else'FAILED',exit=code,ended=time.time())
        save(out/'result.json',record)
    except BaseException as error:
        code=1;record.update(status='FAILED_NOT_ACCEPTED',failure=repr(error),exit=1,ended=time.time())
        save(out/'result.json',record)
        raise
    finally:
        finish(code)
    return code

if __name__=='__main__':raise SystemExit(main())
