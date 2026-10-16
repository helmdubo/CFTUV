import os,sys,json,subprocess,hashlib
from pathlib import Path
ROOT=Path('D:/CFTUV-export-20261015-evidence');A=ROOT/'angular';sys.path.insert(0,str(ROOT/'corpus'));import owned_job_guard_v2 as guard
PY='C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe'
runner=A/'replay_chunks_v2.py';expected='670115859fda70fb0220cc6d4b2c49808f932b6f094ee9313e871380d0470b5c'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for archive in ('rounded','building'):
 assert sha(runner)==expected
 log=A/('replay-'+archive+'-chunk02-launch');log.mkdir(exist_ok=False);(log/'tmp').mkdir()
 env={k:v for k,v in os.environ.items()if not k.startswith('CFTUV_')}
 env.update(PYTHONPATH=';'.join([str(A/'wheel-build-01/site'),'C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages','D:/CFTUV-export-angular-20261016/kernel/src','D:/CFTUV-export-angular-20261016/tools',str(A)]),PYTHONSAFEPATH='1',PYTHONDONTWRITEBYTECODE='1',PYTHONHASHSEED='0',TEMP=str(log/'tmp'),TMP=str(log/'tmp'))
 command=[PY,'-B',str(runner),'--pair-root',str(A/'paired-capture-01'),'--archive',archive,'--max-chunks','1','--source','D:/CFTUV-export-angular-20261016','--site',str(A/'wheel-build-01/site'),'--approved-profile',str(A/'profile-311-light-4f3-isolated-parity-01.json'),'--approved-profile-sha256','ee7b8f013d8dcb7c8b98ccab2fb5d13044007583934d22bcc683c63df3f72b0c','--guard',str(ROOT/'corpus/owned_job_guard_v2.py'),'--out',str(A/('replay-'+archive+'-chunk02'))]
 guard.disks(['C:/','D:/','E:/'],1.5)
 with (log/'stdout.txt').open('w')as stdout,(log/'stderr.txt').open('w')as stderr:
  proc=subprocess.Popen(command,env=env,cwd='D:/CFTUV-export-angular-20261016',stdout=stdout,stderr=stderr)
  receipt=dict(pid=proc.pid,creation_filetime=guard.creation_filetime(proc._handle),argv=command,exit=None,runner_sha_before=expected)
  (log/'process.json').write_text(json.dumps(receipt,indent=2));receipt['exit']=proc.wait()
 receipt['runner_sha_after']=sha(runner);(log/'process.json').write_text(json.dumps(receipt,indent=2))
 print(archive,receipt['exit'],flush=True)
 assert receipt['exit']==0 and receipt['runner_sha_after']==expected,'STOP_AFTER_FAILED_SCOPE'
 state=json.loads((A/('replay-'+archive+'-chunk02')/'replay.json').read_text());print(json.dumps(state),flush=True)
