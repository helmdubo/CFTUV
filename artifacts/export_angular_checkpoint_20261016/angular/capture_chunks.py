"""Последовательные chunks16: свежий процесс, immutable receipts, resume COMPLETE chunks."""
import argparse
import hashlib
import json
import lzma
import os
from pathlib import Path
import pickle
import subprocess
import sys
import time

import record_angular as recorder

WORK = Path(__file__).resolve().parent
ARCHIVES = ('building.tasks.xz','cover008.tasks.xz','half_sphere.tasks.xz','rounded.tasks.xz',
            'sagging02239.tasks.xz','sagging0987.tasks.xz','two.tasks.xz')
SITE = WORK.parent / 'frame-build/site'
BP = Path('C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe')
VENV_SITE = Path('C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages')
BUILD = '2a74a8b573f322821c0da35ca5ef8dde77ddd076edfc9b95a27cc712397cb8d4'


def inventory(tasks):
    archives, chunks = {}, []
    for name in ARCHIVES:
        path = tasks / name
        archives[name] = recorder.sha(path)
        rows = []
        with lzma.open(path,'rb') as stream:
            while True:
                try: case, blob = pickle.load(stream)
                except EOFError:break
                if not isinstance(blob,bytes):raise TypeError('TASK_BLOB_SCHEMA_CHANGED')
                rows.append({'case':case,'input_sha256':hashlib.sha256(blob).hexdigest()})
        for start in range(0,len(rows),16):
            selected = rows[start:start+16]
            chunks.append({'id':name[:-9]+'-'+str(start).zfill(6),'archive':name,'start':start,
                           'count':len(selected),'inputs':selected,'attempts':[],'completed':None})
    return archives,chunks


def verify_chunk(out, chunk, complete, identity, archive_hashes):
    directory = out / complete['path']
    manifest_path = directory / 'manifest.json'
    assert recorder.sha(manifest_path) == complete['manifest_sha256'], 'COMPLETED_MANIFEST_CHANGED'
    manifest = json.loads(manifest_path.read_text())
    assert manifest['head']==identity['head'] and manifest['kernel_identity']==identity['kernel'], 'COMPLETED_SOURCE_IDENTITY_DIFFERS'
    assert manifest['native_source_build']['id']==identity['build'], 'COMPLETED_NATIVE_BUILD_DIFFERS'
    assert manifest['runner_sha256']==identity['recorder_sha256'], 'COMPLETED_RECORDER_DIFFERS'
    assert manifest['raw_pickle_helper_sha256']==identity['helper_sha256'], 'COMPLETED_HELPER_DIFFERS'
    assert {str(Path(k).resolve()):v for k,v in manifest['sources'].items()}=={str(Path(identity['tasks_root'])/chunk['archive']):archive_hashes[chunk['archive']]}, 'COMPLETED_SOURCE_ARCHIVE_DIFFERS'
    assert manifest['phase']=='COMPLETE' and manifest['capture_mode']=='angular_wrappers'
    assert manifest['task_count']==len(manifest['tasks'])==chunk['count'], 'CHUNK_NOT_COMPLETE'
    assert manifest['archive']==chunk['archive'] and manifest['start']==chunk['start'] and manifest['count']==chunk['count']
    assert [row['source_task_index'] for row in manifest['tasks']]==list(range(chunk['start'],chunk['start']+chunk['count']))
    assert [row['source_task_blob_sha256'] for row in manifest['tasks']]==[row['input_sha256'] for row in chunk['inputs']]
    for row in manifest['tasks']:
        assert row['observer_errors']==0, 'OBSERVER_ERROR'
        path=(directory/row['path']).resolve()
        assert path.is_relative_to(directory.resolve()), 'ARTIFACT_ESCAPES'
        assert recorder.sha(path)==row['sha256'], 'COMPLETED_TASK_CHANGED'
    return manifest


def environment(out):
    env={k:v for k,v in os.environ.items() if not k.startswith('CFTUV_')}
    env.update(PYTHONSAFEPATH='1',PYTHONDONTWRITEBYTECODE='1',PYTHONHASHSEED='0',
               TMP=str(out/'tmp'),TEMP=str(out/'tmp'),
               PYTHONPATH=os.pathsep.join(map(str,(SITE,VENV_SITE,recorder.ROOT/'kernel/src',recorder.ROOT/'tools',WORK))))
    return env


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--tasks',type=Path,default=recorder.TASKS)
    parser.add_argument('--guard',type=Path,required=True,help='Reviewed numeric FILETIME owned_job_guard.py')
    parser.add_argument('--observer-proof',type=Path,required=True,help='Bounded baseline-vs-wrappers receipt for THIS recorder')
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--max-chunks',type=int,help='Explicit bounded subset; remains INCOMPLETE')
    args=parser.parse_args()
    args.tasks=args.tasks.resolve()
    if args.max_chunks is not None and args.max_chunks < 1:parser.error('--max-chunks must be positive')
    out=args.out.resolve()
    assert out.is_relative_to(WORK.resolve()) and out!=WORK.resolve(), 'OWN_OUTPUT_REQUIRED'
    assert not (out.is_relative_to(args.tasks.resolve()) or args.tasks.resolve().is_relative_to(out)), 'INPUT_OUTPUT_OVERLAP'
    proof=json.loads(args.observer_proof.read_text())
    code_sha=recorder.sha(WORK/'record_angular.py')
    assert proof['status']=='BOUNDED_OBSERVER_EQUAL' and proof['recorder_sha256']==code_sha, 'OBSERVER_PROOF_REQUIRED'
    assert proof['head']==recorder.HEAD and proof['kernel_identity']==recorder.KERNEL
    assert proof['helper_sha256']==recorder.sha(WORK/'angular_raw_pickle.py'), 'OBSERVER_HELPER_DIFFERS'
    for role in ('baseline','recorded'):
        assert recorder.sha(Path(proof[role])/'manifest.json')==proof[role+'_manifest_sha256'], 'OBSERVER_PROOF_MANIFEST_CHANGED '+role
    identity={'head':recorder.HEAD,'kernel':recorder.KERNEL,'build':BUILD,'recorder_sha256':code_sha,
              'helper_sha256':recorder.sha(WORK/'angular_raw_pickle.py'),'guard_sha256':recorder.sha(args.guard),
              'observer_proof_sha256':recorder.sha(args.observer_proof),'chunk_size':16,
              'coordinator_sha256':recorder.sha(__file__),'compare_baseline_sha256':recorder.sha(WORK/'compare_baseline.py'),
              'tasks_root':str(args.tasks)}
    archive_hashes,chunks=inventory(args.tasks)
    state_path=out/'capture.json'
    if args.resume:
        state=json.loads(state_path.read_text())
        assert state['identity']==identity and state['archives']==archive_hashes, 'RESUME_PROVENANCE_CHANGED'
        current=[{k:v for k,v in c.items() if k not in ('attempts','completed')} for c in chunks]
        saved=[{k:v for k,v in c.items() if k not in ('attempts','completed')} for c in state['chunks']]
        assert current==saved,'ORDERED_CHUNK_INPUTS_CHANGED'
    else:
        out.mkdir(parents=True,exist_ok=False)
        state={'schema':'cftuv.angular.chunk-capture.v1','status':'INCOMPLETE','identity':identity,
               'archives':archive_hashes,'chunks':chunks,'phase':'READY','not_native_parity':True}
        recorder.save(state_path,state)
    lock=out/'capture.lock'
    with lock.open('x',encoding='utf8') as stream:stream.write(json.dumps({'pid':os.getpid(),'creation_filetime':recorder.process_creation()}))
    try:
        (out/'tmp').mkdir(exist_ok=True)
        launched=0
        for chunk in state['chunks']:
            if chunk['completed'] is not None:
                verify_chunk(out,chunk,chunk['completed'],identity,archive_hashes);continue
            if args.max_chunks is not None and launched>=args.max_chunks:break
            assert recorder.sha(WORK/'record_angular.py')==identity['recorder_sha256'], 'RECORDER_CHANGED'
            assert recorder.sha(WORK/'angular_raw_pickle.py')==identity['helper_sha256'], 'HELPER_CHANGED'
            assert recorder.sha(args.guard)==identity['guard_sha256'], 'GUARD_CHANGED'
            for name,digest in archive_hashes.items():assert recorder.sha(args.tasks/name)==digest,'SOURCE_ARCHIVE_CHANGED'
            attempt_name=chunk['id']+'-attempt-'+str(len(chunk['attempts'])+1).zfill(3)
            target=out/attempt_name
            guard_receipt=out/(attempt_name+'-guard')
            assert not target.exists() and not guard_receipt.exists(),'FRESH_ATTEMPT_REQUIRED'
            bootstrap="import sys,pathlib,runpy; import cftuv_native; assert pathlib.Path(cftuv_native.__file__).resolve().parent==pathlib.Path("+repr(str(SITE/'cftuv_native'))+").resolve(); assert cftuv_native.native_build_id()=="+repr(BUILD)+"; sys.argv=sys.argv[1:]; runpy.run_path(sys.argv[0],run_name='__main__')"
            command=[str(BP),'-B','-c',bootstrap,str(WORK/'record_angular.py'),'--tasks',str(args.tasks),
                     '--archive',chunk['archive'],'--start',str(chunk['start']),'--count',str(chunk['count']),
                     '--limit',str(chunk['count']),'--out',str(target)]
            argv=[sys.executable,'-B',str(args.guard),'run','--receipt',str(guard_receipt),'--cwd',str(recorder.ROOT),
                  '--floor-gib','1.5','--drives','C:/','D:/','E:/','--',*command]
            entry={'path':attempt_name,'guard_receipt':str(guard_receipt),'argv':argv,'started':time.time(),'exit':None}
            chunk['attempts'].append(entry)
            state.update(phase='CHUNK_RUNNING',current_chunk=chunk['id'])
            recorder.save(state_path,state)
            proc=subprocess.Popen(argv,cwd=recorder.ROOT,env=environment(out))
            entry.update(guard_pid=proc.pid,guard_creation_filetime=creation_for_pid(proc))
            recorder.save(state_path,state)
            entry['exit']=proc.wait();entry['ended']=time.time();launched+=1
            if entry['exit']!=0:
                state.update(status='INCOMPLETE',phase='FAILED_CHUNK_RETAINED')
                recorder.save(state_path,state)
                raise RuntimeError('CHUNK_FAILED_NO_AUTOMATIC_RETRY '+chunk['id'])
            complete={'path':attempt_name,'manifest_sha256':recorder.sha(target/'manifest.json')}
            manifest=verify_chunk(out,chunk,complete,identity,archive_hashes)
            chunk['completed']=complete
            state['phase']='CHUNK_DURABLE'
            recorder.save(state_path,state)
        state.update(status='RECORDED_NOT_PARITY_ACCEPTED' if all(c['completed'] is not None for c in state['chunks']) else 'INCOMPLETE',phase='STOPPED',ended=time.time())
        recorder.save(state_path,state)
    finally:lock.unlink()


def creation_for_pid(proc):
    import ctypes
    from ctypes import wintypes
    fields=[wintypes.FILETIME() for _ in range(4)]
    dll=ctypes.WinDLL('kernel32',use_last_error=True)
    dll.GetProcessTimes.argtypes=[wintypes.HANDLE]+[ctypes.POINTER(wintypes.FILETIME)]*4
    if not dll.GetProcessTimes(wintypes.HANDLE(int(proc._handle)),*(ctypes.byref(f) for f in fields)):raise ctypes.WinError(ctypes.get_last_error())
    return (fields[0].dwHighDateTime<<32)|fields[0].dwLowDateTime

if __name__=='__main__':main()
