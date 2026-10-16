"""Owned isolated ANGULAR release wheel: prepare only until parent GO.
Build and install are explicit separate phases; no develop/foreign installation.
"""
import argparse,hashlib,json,os,subprocess,sys,time,zipfile
from pathlib import Path

WORK=Path(__file__).resolve().parent
assert hashlib.sha256((WORK/'rust_job.py').read_bytes()).hexdigest()=='3ae624a9246e95daf3a44b434f751a614996eb744e6dc943a56adbb66b0fada4','RUST_JOB_HELPER_CHANGED'
import rust_job as gate
ROOT=gate.SOURCE
HEAD='4f3a00f8ec6be68c9e547e0e4a7a9f4fc7c99d66'
SOURCE_SHA='04641fcda171e5655a9dfae1cc16a7deac9d1428c038fb6ef277f5bfedc32a3d'
PARTS={'id':'e5480f28953cafd9e8bb67d53241d7ee0dd561408a8fa4b3b57bd34eb30dde56','rust':'329069f4074ed52cd3840cb383c6cb6e42cc98f682951c93d76c5bfebb75431d','shim':'b09719465f66a08053cce645b42bd4288976949f7c1da3f7b40ddb32b5b8974c'}
KERNEL='05a3ad9115455e76'
MATURIN=WORK/'tool-bin/maturin.exe'
MATURIN_SHA='787779fa7f453d3444ac732689ad2bb8f1399b4d4e2af1ad9e00f84bb6460f07'
DEPS=Path('C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages')


def source_guard():
    assert gate.git('rev-parse','HEAD')==HEAD and not gate.git('status','--porcelain'),'SOURCE_HEAD_OR_STATUS_CHANGED'
    files=sorted(name for name in gate.git('ls-files','cftuv','kernel/src','native').splitlines()
                 if Path(name).suffix in ('.py','.rs','.toml')or Path(name).name=='Cargo.lock')
    h=hashlib.sha256()
    for name in files:h.update(name.encode()+b'\0'+(ROOT/name).read_bytes()+b'\n')
    assert h.hexdigest()==SOURCE_SHA,'SOURCE_BYTES_CHANGED'
    sys.path.insert(0,str(ROOT/'tools'))
    import native_build_id,native_ci_corpus
    assert native_build_id.tree_parts(ROOT/'native')==PARTS,'NATIVE_SOURCE_BUILD_CHANGED'
    assert native_ci_corpus.kernel_identity(ROOT)==KERNEL,'KERNEL_IDENTITY_CHANGED'
    assert gate.sha(MATURIN)==MATURIN_SHA,'OWNED_MATURIN_CHANGED'


def wheel_guard(out):
    wheels=list((out/'wheels').glob('*.whl'))
    assert len(wheels)==1 and wheels[0].name=='cftuv_native-0.1.0-cp311-abi3-win_amd64.whl','EXACT_ONE_EXPECTED_WHEEL_REQUIRED'
    wheel=wheels[0];source=ROOT/'native/cftuv-python/python/cftuv_native'
    with zipfile.ZipFile(wheel)as archive:
        names=archive.namelist();assert len(names)==len(set(names)),'DUPLICATE_WHEEL_ENTRIES'
        pyfiles={p.name:p for p in source.glob('*.py')}
        packaged={name.split('/')[-1]:name for name in names if name.startswith('cftuv_native/')and name.count('/')==1 and name.endswith('.py')}
        assert set(packaged)==set(pyfiles),'WHEEL_SHIM_FILESET_CHANGED'
        for name,path in pyfiles.items():assert archive.read(packaged[name])==path.read_bytes(),'WHEEL_SHIM_BYTES_DIFFER '+name
        extensions=[name for name in names if name.startswith('cftuv_native/_core')and name.endswith('.pyd')]
        assert len(extensions)==1,'EXACT_ONE_CORE_EXTENSION_REQUIRED'
    return wheel


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--phase',choices=('build','install'),required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();out=args.out.resolve()
    assert out.is_relative_to(WORK)and out!=WORK,'OWN_D_OUTPUT_REQUIRED'
    assert Path(os.environ.get('CARGO_TARGET_DIR','')).resolve()==gate.TARGET.resolve(),'MANDATED_C_TARGET_REQUIRED'
    out.mkdir(parents=True,exist_ok=args.phase=='install');phase=out/args.phase;phase.mkdir()
    finish=gate.setup(phase/'job.json');code=1
    record=dict(status='STARTING',phase=args.phase,head=HEAD,source_sha256=SOURCE_SHA,parts=PARTS,kernel_identity=KERNEL,
                script_sha256=gate.sha(__file__),maturin_sha256=MATURIN_SHA,started=time.time())
    try:
        source_guard()
        env={key:value for key,value in os.environ.items()if not key.startswith('CFTUV_')}
        (out/'tmp').mkdir(exist_ok=True)
        env.update(TEMP=str(out/'tmp'),TMP=str(out/'tmp'),PYTHONDONTWRITEBYTECODE='1',PYO3_PYTHON=str(gate.BP),
                   RUSTC=str(gate.TOOLCHAIN/'rustc.exe'),CARGO=str(gate.TOOLCHAIN/'cargo.exe'),CARGO_BUILD_JOBS='1',
                   PATH=str(gate.TOOLCHAIN)+os.pathsep+env.get('PATH',''),PIP_DISABLE_PIP_VERSION_CHECK='1',PIP_NO_INDEX='1')
        if args.phase=='build':
            (out/'wheels').mkdir()
            command=[str(MATURIN),'build','--release','--locked','--offline','--interpreter',str(gate.BP),'--out',str(out/'wheels')]
        else:
            previous=json.loads((out/'build/result.json').read_text());assert previous['status']=='COMPLETE'and previous['head']==HEAD and previous['source_sha256']==SOURCE_SHA and previous['parts']==PARTS,'MATCHING_BUILD_RECEIPT_REQUIRED'
            wheel=wheel_guard(out);assert gate.sha(wheel)==previous['wheel_sha256'],'BUILT_WHEEL_CHANGED'
            site=out/'site';assert not site.exists(),'NEW_OWN_SITE_REQUIRED'
            metadata=DEPS/'pip-24.3.1.dist-info/METADATA';assert 'Version: 24.3.1' in metadata.read_text(),'READONLY_PIP_24_3_1_REQUIRED'
            env['PYTHONPATH']=str(DEPS)
            command=[str(gate.BP),'-B','-m','pip','install','--no-deps','--no-index','--no-cache-dir','--target',str(site),str(wheel)]
            record['pip_metadata_sha256']=gate.sha(metadata)
        record['argv']=command;gate.save(phase/'result.json',record)
        subprocess.run(command,cwd=ROOT/'native/cftuv-python',env=env,check=True)
        wheel=wheel_guard(out)
        if args.phase=='install':
            env['PYTHONPATH']=os.pathsep.join(map(str,(out/'site',ROOT/'kernel/src',ROOT/'tools',DEPS)))
            verify="import pathlib,cftuv_native as n; assert pathlib.Path(n.__file__).resolve().parent==pathlib.Path("+repr(str(out/'site/cftuv_native'))+").resolve(); p=n.native_build_parts(); assert all(p[k]==v for k,v in "+repr(PARTS)+".items()); s=n.native_status(); assert s['angular']=='unavailable'; assert all(v=='available' for k,v in s.items() if k!='angular'); print(p); print(s)"
            subprocess.run([str(gate.BP),'-B','-c',verify],cwd=ROOT,env=env,check=True)
            subprocess.run([str(gate.BP),'-B',str(ROOT/'tools/native_build_id.py'),'--json','--require'],cwd=ROOT,env=env,check=True)
        source_guard();code=0
        record.update(status='COMPLETE',exit=0,wheel=str(wheel),wheel_sha256=gate.sha(wheel),ended=time.time())
        gate.save(phase/'result.json',record)
    except BaseException as error:
        record.update(status='FAILED_NOT_ACCEPTED',exit=1,error=repr(error),ended=time.time());gate.save(phase/'result.json',record)
        raise
    finally:finish(code)
    return code

if __name__=='__main__':raise SystemExit(main())
