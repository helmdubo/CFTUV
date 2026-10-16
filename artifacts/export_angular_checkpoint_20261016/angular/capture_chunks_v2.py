"""Paired chunks16, two fresh CPython3.11 processes; explicit launch only.
Resume accepts only verified complete pairs. Failed/interrupted pairs require a
new output, never an automatic retry. This records observer parity, not native parity.
"""
import argparse,json,os,subprocess,sys,time
from pathlib import Path
import capture_chunks as old
import compare_chunks as comparator

WORK=old.WORK
PROFILE_PATH=WORK/'observer-baseline-rounded-01/manifest.json'
PROFILE_SHA='5939e57151054ace47d6423045235107bd82c4edcc76f589eff131d73512587b'
GUARD_SHA='90fd91af228d696bd181f1177b3dd1f5e1c6745e79b0707e4383fd4af378eb6b'

def guards(identity,args,archives):
    for path,key in ((WORK/'record_angular.py','recorder_sha256'),(WORK/'angular_raw_pickle.py','helper_sha256'),
                     (Path(args.guard),'guard_sha256'),(Path(__file__),'coordinator_sha256'),
                     (Path(comparator.__file__),'comparator_sha256'),(Path(old.__file__),'previous_coordinator_sha256')):
        assert old.recorder.sha(path)==identity[key],'CODE_CHANGED '+key
    for name,digest in archives.items():assert old.recorder.sha(args.tasks/name)==digest,'SOURCE_ARCHIVE_CHANGED'
    for name,digest in identity['native_site_files'].items():assert old.recorder.sha(old.SITE/'cftuv_native'/name)==digest,'NATIVE_SITE_CHANGED'


def run_side(args,out,chunk,role,identity,state,state_path):
    target=out/(chunk['id']+'-'+role);receipt=out/(chunk['id']+'-'+role+'-guard')
    assert not target.exists()and not receipt.exists(),'FRESH_SIDE_REQUIRED'
    bootstrap="import sys,pathlib,runpy; import cftuv_native; assert pathlib.Path(cftuv_native.__file__).resolve().parent==pathlib.Path("+repr(str(old.SITE/'cftuv_native'))+").resolve(); assert cftuv_native.native_build_id()=="+repr(old.BUILD)+"; sys.argv=sys.argv[1:]; runpy.run_path(sys.argv[0],run_name='__main__')"
    command=[str(old.BP),'-B','-c',bootstrap,str(WORK/'record_angular.py'),'--tasks',str(args.tasks),
             '--archive',chunk['archive'],'--start',str(chunk['start']),'--count',str(chunk['count']),
             '--limit',str(chunk['count']),'--out',str(target)]
    if role=='baseline':command.append('--baseline')
    argv=[str(old.BP),'-B',str(args.guard),'run','--receipt',str(receipt),'--cwd',str(old.recorder.ROOT),
          '--floor-gib','1.5','--drives','C:/','D:/','E:/','--',*command]
    entry=dict(role=role,path=target.name,guard=receipt.name,argv=argv,started=time.time(),exit=None)
    chunk['attempts'].append(entry);state.update(phase='PAIR_'+role.upper()+'_RUNNING',current_chunk=chunk['id'])
    old.recorder.save(state_path,state)
    proc=subprocess.Popen(argv,cwd=old.recorder.ROOT,env=old.environment(out))
    entry['pid']=proc.pid
    entry['creation_filetime']=old.creation_for_pid(proc);old.recorder.save(state_path,state)
    entry['exit']=proc.wait();entry['ended']=time.time();old.recorder.save(state_path,state)
    guard=json.loads((receipt/'process.json').read_text())
    assert entry['exit']==0 and guard['status']=='COMPLETE'and guard['owned_child_reaped'],'OWNED_SIDE_FAILED'
    return target


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--tasks',type=Path,default=old.recorder.TASKS)
    p.add_argument('--guard',type=Path,required=True);p.add_argument('--resume',action='store_true')
    p.add_argument('--max-chunks',type=int)
    args=p.parse_args();args.tasks=args.tasks.resolve();args.guard=args.guard.resolve();out=args.out.resolve()
    assert args.max_chunks is None or args.max_chunks>0,'POSITIVE_MAX_CHUNKS'
    assert out.is_relative_to(WORK.resolve())and out!=WORK.resolve(),'OWN_OUTPUT_REQUIRED'
    assert not(out.is_relative_to(args.tasks)or args.tasks.is_relative_to(out)),'INPUT_OUTPUT_OVERLAP'
    assert old.recorder.sha(args.guard)==GUARD_SHA,'REVIEWED_GUARD_V2_REQUIRED'
    assert old.recorder.sha(PROFILE_PATH)==PROFILE_SHA,'EXPECTED_PROFILE_CHANGED'
    profile=json.loads(PROFILE_PATH.read_text())
    assert (profile['sympy'],profile['mpmath'],profile['symbolic_mode'])==('1.14.0','1.3.0','NATIVE_EXACT'),'REVIEWED_PROFILE_REQUIRED'
    identity=dict(expected_environment={key:profile[key] for key in comparator.ENV},expected_profile=dict(path=str(PROFILE_PATH),sha256=PROFILE_SHA),head=old.recorder.HEAD,kernel=old.recorder.KERNEL,build=old.BUILD,
                  tasks_root=str(args.tasks),chunk_size=16,guard_sha256=GUARD_SHA,
                  recorder_sha256=old.recorder.sha(WORK/'record_angular.py'),helper_sha256=old.recorder.sha(WORK/'angular_raw_pickle.py'),
                  coordinator_sha256=old.recorder.sha(__file__),comparator_sha256=old.recorder.sha(comparator.__file__),
                  previous_coordinator_sha256=old.recorder.sha(old.__file__),
                  native_site_files={p.name:old.recorder.sha(p)for p in sorted((old.SITE/'cftuv_native').iterdir())if p.is_file()})
    archives,chunks=old.inventory(args.tasks);state_path=out/'pairs.json'
    if args.resume:
        state=json.loads(state_path.read_text());assert state['identity']==identity and state['archives']==archives,'RESUME_IDENTITY_CHANGED'
        strip=lambda rows:[{k:v for k,v in row.items()if k not in('attempts','completed')}for row in rows]
        assert strip(chunks)==strip(state['chunks']),'ORDERED_INPUTS_CHANGED'
    else:
        out.mkdir(parents=True,exist_ok=False)
        state=dict(schema='cftuv.angular.paired-chunks.v1',identity=identity,archives=archives,chunks=chunks,status='INCOMPLETE',phase='READY',not_native_parity=True)
        old.recorder.save(state_path,state)
    lock=out/'paired.lock'
    with lock.open('x')as f:json.dump(dict(pid=os.getpid(),creation_filetime=old.recorder.process_creation()),f)
    retain_lock=False
    try:
        (out/'tmp').mkdir(exist_ok=True);launched=0
        for chunk in state['chunks']:
            clean={k:v for k,v in chunk.items()if k not in('attempts','completed')}
            complete=chunk['completed']
            if complete is not None:
                receipt=comparator.contained(out,complete['receipt'])
                assert old.recorder.sha(receipt)==complete['sha256'],'PAIR_RECEIPT_CHANGED'
                comparator.verify(receipt,comparator.contained(out,complete['baseline']),comparator.contained(out,complete['recorded']),clean,identity,archives)
                continue
            assert not chunk['attempts'],'INCOMPLETE_PAIR_RETAINED_NEW_OUTPUT_REQUIRED'
            if args.max_chunks is not None and launched>=args.max_chunks:break
            guards(identity,args,archives)
            baseline=run_side(args,out,chunk,'baseline',identity,state,state_path)
            guards(identity,args,archives)
            recorded=run_side(args,out,chunk,'recorded',identity,state,state_path)
            guards(identity,args,archives)
            result=comparator.compare(baseline,recorded,clean,identity,archives)
            receipt=out/(chunk['id']+'-pair.json')
            with receipt.open('x')as f:json.dump(result,f,indent=2)
            assert result['status']=='PAIRED_CHUNK_OBSERVER_EQUAL','OBSERVER_MISMATCH_RETAINED'
            comparator.verify(receipt,baseline,recorded,clean,identity,archives)
            chunk['completed']=dict(receipt=receipt.name,sha256=old.recorder.sha(receipt),baseline=baseline.name,recorded=recorded.name)
            state['phase']='PAIR_DURABLE';old.recorder.save(state_path,state);launched+=1
        state.update(status='PAIRED_ALL_OBSERVER_EQUAL_NOT_NATIVE_PARITY'if all(c['completed']for c in state['chunks'])else'INCOMPLETE',phase='STOPPED')
        old.recorder.save(state_path,state)
    except BaseException as error:
        # При прерывании родителя не убиваем guard: только он владеет HANDLE oracle.
        retain_lock=any(entry.get('pid')and entry['exit']is None for c in state['chunks']for entry in c['attempts'])
        state.update(status='INCOMPLETE',phase='FAILED_PAIR_RETAINED',failure=repr(error),interrupted_guard_may_still_run=retain_lock)
        old.recorder.save(state_path,state)
        raise
    finally:
        if not retain_lock:lock.unlink()

if __name__=='__main__':main()
