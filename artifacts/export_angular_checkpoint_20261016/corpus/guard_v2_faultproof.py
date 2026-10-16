import ctypes,json,sys,time,os
from pathlib import Path
from types import SimpleNamespace
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import owned_job_guard_v2 as g
OUT=HERE/'guard-v2-faultproof-01';OUT.mkdir(exist_ok=False)
original_replace=g.os.replace;original_popen=g.subprocess.Popen
results=[]
for mode,number in [('normal',0),('transient',5),('transient',32),('transient',33),('permanent',5),('permanent',32),('permanent',33),('unrelated',2)]:
    label=mode+'-'+str(number);folder=OUT/label;owned=[];attempts=[]
    def popen(*args,**kwargs):
        proc=original_popen(*args,**kwargs);owned.append(proc);return proc
    def replace(source,target):
        if Path(target)==folder/'process.json':
            attempts.append(time.monotonic())
            if mode=='permanent' or mode=='unrelated' or (mode=='transient' and len(attempts)<=3):
                error=OSError('INJECTED_REPLACE_'+label);error.winerror=number;raise error
        return original_replace(source,target)
    g.subprocess.Popen=popen;g.os.replace=replace
    args=SimpleNamespace(receipt=str(folder),cwd=str(HERE),floor_gib=1.5,drives=['C:/','D:/','E:/'],env=[],command=[sys.executable,'-c','import time; time.sleep(.3)'],test_fault=None,expect_filetime=None,timeout=5,poll=.02)
    started=time.monotonic();exception=None;code=None
    try:code=g.run(args)
    except OSError as error:exception=dict(type=type(error).__name__,winerror=error.winerror,message=str(error))
    finally:g.os.replace=original_replace;g.subprocess.Popen=original_popen
    elapsed=time.monotonic()-started
    assert len(owned)==1;proc=owned[0];assert proc.returncode is not None and g.K.WaitForSingleObject(int(proc._handle),0)==0
    if mode in ('normal','transient'):
        assert code==0 and proc.returncode==0 and exception is None
        receipt=json.loads((folder/'process.json').read_text());assert receipt['status']=='COMPLETE' and receipt['owned_child_reaped']
    else:
        assert exception and exception['winerror']==number and proc.returncode==91
        if mode=='unrelated':assert len(attempts)==2,attempts
        else:assert .45<=elapsed<1.5,(elapsed,len(attempts))
    results.append(dict(name=label,pass_=True,elapsed=elapsed,replace_attempts=len(attempts),exception=exception,child_pid=proc.pid,child_filetime=g.creation_filetime(int(proc._handle)),child_argv=args.command,child_exit=proc.returncode,reaped=True))
proof=dict(status='ALL_8_PASS',guard_sha256=__import__('hashlib').sha256(Path(g.__file__).read_bytes()).hexdigest(),tests=results,scope='Harmless owned sleep children only; replace errors injected, permanent error cleanup verified')
(OUT/'receipt.json').write_text(json.dumps(proof,indent=2));print(json.dumps(proof),flush=True)
