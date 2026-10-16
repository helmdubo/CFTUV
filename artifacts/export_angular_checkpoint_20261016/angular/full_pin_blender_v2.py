"""Actual Blender full pin AUTHORING only; no save, geometry, native port or approval."""
import json,os,runpy,subprocess,sys,time,traceback
from pathlib import Path
WORK=Path(__file__).resolve().parent
ROOT=Path('D:/CFTUV-export-angular-20261016')
SCENE=Path('C:/Users/helmd/AppData/Local/Temp/cftuv_field_parity_20261008/frozen/testscene.blend')
SCENE_SHA='750baefc50ddbccad2ff090dae135c55e7b0d1584ca62697261a32b249e15fb4'
TOOL=ROOT/'tools/native_angular_pins.py'
TOOL_SHA='4d48ce3a27c6cd3b03b951e4b300d782d67fd42817fc80197a005ecf90e5bf46'
CANDIDATE=WORK/'profile-candidate-311-full-4f3-v2.json'
sys.path.insert(0,str(WORK))
import wheel_job
from owned_job_guard_v2 import creation_filetime
import _winapi


def main():
    out=WORK/'full-pin-311-v2';out.mkdir(exist_ok=False)
    finish=wheel_job.gate.setup(out/'job.json');code=1
    report=dict(status='INCOMPLETE',scope='Offline full-profile observation only; no product pin approval',
                pid=os.getpid(),creation_filetime=creation_filetime(_winapi.GetCurrentProcess()),argv=list(sys.argv),
                script_sha256=wheel_job.gate.sha(__file__),tool_sha256=TOOL_SHA,started=time.time())
    try:
        isolation={'BLENDER_USER_CONFIG':WORK/'full-pin-311-v2-config','BLENDER_USER_SCRIPTS':WORK/'full-pin-311-v2-scripts'}
        report['child_environment_isolation']={key:str(value)for key,value in isolation.items()}
        for key,path in isolation.items():
            assert Path(os.environ.get(key,'')).resolve()==path.resolve()and path.is_dir(),'ISOLATED_CHILD_ENV_REQUIRED '+key
        report['isolation_files_after_startup']={key:[str(item.relative_to(path))for item in path.rglob('*')if item.is_file()]for key,path in isolation.items()}
        import bpy
        assert bpy.app.background and Path(bpy.data.filepath).resolve()==SCENE.resolve(),'EXACT_FROZEN_BACKGROUND_REQUIRED'
        assert '--factory-startup'not in sys.argv and '-b'in sys.argv and any(value.replace('\\','/')==SCENE.as_posix()for value in sys.argv),'SAFE_ARGV_REQUIRED'
        query="@(Get-CimInstance Win32_Process -Filter \"Name = 'blender.exe'\" | Select-Object -ExpandProperty ProcessId) | ConvertTo-Json -Compress"
        ids=json.loads(subprocess.check_output(['powershell.exe','-NoProfile','-NonInteractive','-Command',query],text=True))
        ids=ids if type(ids)is list else[ids];report['blender_inventory']=ids
        assert ids==[os.getpid()],'FOREIGN_BLENDER_PRESENT'
        assert wheel_job.gate.sha(SCENE)==SCENE_SHA and wheel_job.gate.sha(TOOL)==TOOL_SHA,'FROZEN_OR_TOOL_CHANGED'
        report['cached_host_modules']={name:getattr(module,'__file__',None)for name,module in tuple(sys.modules.items())if name=='cftuv'or name.startswith('cftuv.')}
        for name,filename in report['cached_host_modules'].items():
            assert type(filename)is str and Path(filename).resolve().is_relative_to(ROOT/'cftuv'),'WRONG_CACHED_HOST '+name+':'+repr(filename)
        assert 'cftuv_native'not in sys.modules,'UNEXPECTED_NATIVE_IMPORT'
        sys.path[:0]=list(map(str,(ROOT,ROOT/'kernel/src',ROOT/'tools',wheel_job.DEPS)))
        wheel_job.source_guard()
        original=list(sys.argv)
        try:
            sys.argv=[str(TOOL),'--profile','full','--out',str(CANDIDATE)]
            try:runpy.run_path(str(TOOL),run_name='__main__')
            except SystemExit as done:
                if done.code not in (None,0):raise
        finally:sys.argv=original
        value=json.loads(CANDIDATE.read_text())
        assert value['profile']=='CPython-3.11:full'and value['source_head']==wheel_job.HEAD and value['source_status']==''and value['approved']is False,'FULL_CANDIDATE_IDENTITY'
        assert 'cftuv.model'in sys.modules and Path(sys.modules['cftuv.model'].__file__).resolve()==ROOT/'cftuv/model.py'
        assert 'cftuv_native'not in sys.modules,'NATIVE_PORT_IMPORTED'
        code=0;report.update(status='FULL_PROFILE_CANDIDATE_COMPLETE_NOT_APPROVED',candidate=str(CANDIDATE),
                            candidate_sha256=wheel_job.gate.sha(CANDIDATE),profile=value['profile'],
                            hashes=len(value['pins']['hashes']),runtime_fingerprints=len(value['pins']['runtime']['fingerprints']),
                            class_shapes=len(value['pins']['wire_metadata']['class_shapes']),source_files=len(value['source_files']))
    except BaseException:
        report.update(status='FAILED_NOT_ACCEPTED',failure=traceback.format_exc())
    finally:
        try:
            report['frozen_sha256_after']=wheel_job.gate.sha(SCENE)
            assert report['frozen_sha256_after']==SCENE_SHA,'FROZEN_CHANGED'
            wheel_job.source_guard()
        except BaseException:
            code=1;report.update(status='FAILED_NOT_ACCEPTED',postguard_failure=traceback.format_exc())
        report.update(exit=code,ended=time.time());wheel_job.gate.save(out/'receipt.json',report);finish(code)
    return code

if __name__=='__main__':raise SystemExit(main())
