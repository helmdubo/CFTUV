"""Explicit chunked ANGULAR differential replay; isolated test approval only.
One fresh guarded worker per verified capture pair. No automatic retry of partials.
Accepted chunk resume rehashes ALL durable outputs and guards. No production approval.
"""
import argparse,hashlib,json,os,pickle,subprocess,sys
from pathlib import Path
from types import MappingProxyType
import replay_pair as pair
import replay_operation as schema

PAIR_SHA='c3e5c5a8fd8ae80f9b8765cdf3cd7688b5220cd61dc9b4bf23768c3f5caa87b9'
GUARD_SHA='90fd91af228d696bd181f1177b3dd1f5e1c6745e79b0707e4383fd4af378eb6b'

def save(path,value):
    temp=path.with_suffix(path.suffix+'.tmp')
    with temp.open('x')as stream:json.dump(value,stream,indent=2);stream.flush();os.fsync(stream.fileno())
    os.replace(temp,path)

def historical(call,result):
    expected={key:call['telemetry'][key]for key in ('fallback_delta','backend_delta')}
    if 'exception'in call:expected['exception']=call['exception']
    else:expected.update(exception=None,output_bytes=call['output_codec_bytes'].hex())
    writes={}
    for name,value,domain in call['profile_writes']:writes[(name,domain)]=[name,value,domain]
    expected['profile_counters']=[writes[k]for k in sorted(writes,key=lambda k:(k[0],k[1]or''))]
    differences={side:[key for key,value in expected.items()if not schema.exact(result[side].get(key),value)]for side in ('reference','native')}
    result['historical_delta_output_profile_differences']=differences
    if differences['reference'] or (result['native_accepted'] and differences['native']):
        result['status']='HISTORICAL_OPERATION_OBSERVATION_MISMATCH'
    elif not result['native_accepted']:
        observation=result['native'];error=observation['exception']
        # A named refusal permits fallback only with an empty observable prefix.
        if error and error['type']=='cftuv_native.pin.NativePortUnsupported':
            safe=(observation['phase']==[] and observation['profile_counters']==[]
                and all(type(value)is int and value==0 for key,value in observation['fallback_delta']+observation['backend_delta'])
                and schema.exact(observation['decimal_before'],observation['decimal_after'])
                and schema.exact(observation['iv_before'],observation['iv_after']))
            result['status']='NATIVE_UNSUPPORTED_NOT_PARITY'if safe else'UNSAFE_PRE_EFFECT_DECLINE'
    return result

def files(root):
    return {str(p.relative_to(root)):schema.sha(p)for p in sorted(root.rglob('*'))if p.is_file()and '__pycache__'not in p.parts}

def sources(root,profile):
    return {name:schema.sha(Path(name)if Path(name).is_absolute()else root/name)for name in profile['source_files']}

def worker(args):
    root=Path(args.source).resolve();site=Path(args.site).resolve();out=Path(args.out).resolve();out.mkdir(exist_ok=False)
    profile=schema.approval(args.approved_profile,args.approved_profile_sha256);pins=pair.load_pins(profile)
    before=sources(root,profile);assert before==profile['source_files'],'SOURCE_PROFILE'
    wheel=files(site/'cftuv_native');helpers={str(Path(m.__file__).resolve()):schema.sha(m.__file__)for m in (pair,schema)}
    assert schema.sha(pair.__file__)==PAIR_SHA,'ACCEPTED_PAIR_HELPER_CHANGED'
    sys.path[:0]=[str(site),str(root/'tools'),str(root/'kernel/src'),str(root)]
    import native_snapshot_check_corpus as corpus
    corpus.install_host_stubs()
    import cftuv_native as native,sympy as sp,mpmath
    assert Path(native.__file__).resolve().parent==site/'cftuv_native','OWN_SITE'
    assert native.native_build_id()==profile['native_build_id'],'NATIVE_BUILD'
    assert (sys.version,sp.__version__,mpmath.__version__)==(profile['python'],profile['sympy'],profile['mpmath']),'RUNTIME_PROFILE'
    import cftuv_envelope as kernel
    from cftuv import envelope_angular_relations as angular,envelope_angular_binding as builder
    from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1 as Profile
    from cftuv_envelope.reference import planar_types,symbolic_backend
    from cftuv_envelope import codec
    from cftuv_native import angular_op
    assert symbolic_backend.backend_mode().value=='NATIVE_EXACT','CAPTURE_MODE'
    profile={**profile,'symbolic_mode':'NATIVE_EXACT'}
    assert not angular_op.APPROVED_PINS and not angular_op._BINDINGS,'ISOLATED_FACADE'
    angular_op.APPROVED_PINS=MappingProxyType({profile['profile']:pins})
    state=dict(status='INCOMPLETE',accepted=False,calls=[],tasks=[],pair_sha256=schema.sha(args.pair),profile_sha256=args.approved_profile_sha256,runner_sha256=schema.sha(__file__),helpers=helpers,wheel_sha_before=wheel,source_sha_before=before)
    path=out/'chunk.json';save(path,state)
    try:
        _,calls=schema.prepare(args.pair,args.approved_profile,args.approved_profile_sha256)
        spec=json.loads(Path(args.pair).read_text());root_capture=Path(spec['recorded']);manifest=json.loads((root_capture/'manifest.json').read_text())
        for item in manifest['tasks']:
            selected=[call for call in calls if call['task_index']==item['source_task_index']]
            task=dict(source_task_index=item['source_task_index'],case=item['case'],angular_calls=len(selected),status='ZERO_ANGULAR_CALLS'if not selected else'RECORDED_CALLS')
            state['tasks'].append(task);save(path,state)
            for call in selected:
                result=pair.run_pair(call['expected']['raw_input_pickle'],kernel=kernel,sp=sp,native=native,profile=profile,Profile=Profile,
                    reference=angular._build_angular_relations_reference,builder=builder,codec=codec,planar=planar_types,symbolic=symbolic_backend,iv=mpmath.iv)
                result=historical(call['expected'],result)
                target=out/('task-'+str(call['task_index'])+'-call-'+str(call['ordinal'])+'.json');save(target,result)
                state['calls'].append(dict(task_index=call['task_index'],ordinal=call['ordinal'],path=target.name,sha256=schema.sha(target),status=result['status']));save(path,state)
                assert result['status']in ('FRESH_OPERATION_PAIR_EQUAL','NATIVE_UNSUPPORTED_NOT_PARITY'),'CALL_FAILURE_RETAINED'
        assert len(state['tasks'])==spec['chunk']['count'] and len(state['calls'])==len(calls),'COMPLETE_COVERAGE'
        state['source_sha_after']=sources(root,profile);state['wheel_sha_after']=files(site/'cftuv_native')
        assert state['source_sha_after']==before and state['wheel_sha_after']==wheel,'SOURCE_OR_WHEEL_CHANGED'
        assert schema.sha(args.approved_profile)==args.approved_profile_sha256,'PROFILE_CHANGED'
        assert all(schema.sha(p)==digest for p,digest in helpers.items()),'HELPER_CHANGED'
        state.update(status='COMPLETE_CHUNK_OBSERVATIONS',accepted=all(v['status']=='FRESH_OPERATION_PAIR_EQUAL'for v in state['calls']),task_count=len(state['tasks']),call_count=len(calls),unsupported=sum(v['status']=='NATIVE_UNSUPPORTED_NOT_PARITY'for v in state['calls']))
        save(path,state)
    except BaseException as error:
        state.update(status='FAILED_RETAINED',failure=repr(error));save(path,state);raise
    finally:angular_op._BINDINGS.clear();angular_op.APPROVED_PINS=MappingProxyType({})

def coordinate(args):
    out=Path(args.out).resolve();profile=schema.approval(args.approved_profile,args.approved_profile_sha256)
    assert schema.sha(args.guard)==GUARD_SHA and schema.sha(pair.__file__)==PAIR_SHA,'FROZEN_HELPERS'
    pairs=sorted(Path(args.pair_root).glob(args.archive+'-*-pair.json'));assert pairs,'NO_PAIRS'
    assert args.max_chunks is None or args.max_chunks>0,'POSITIVE_BOUND'
    if args.archive=='*' and not args.max_chunks:
        inventory=json.loads((Path(args.pair_root)/'pairs.json').read_text())
        assert inventory['status']=='PAIRED_ALL_OBSERVER_EQUAL_NOT_NATIVE_PARITY','FULL_CAPTURE_REQUIRED'
        assert sum(c['count']for c in inventory['chunks'])==1221 and len(pairs)==len(inventory['chunks']),'FULL_1221_TASK_COVERAGE'
    if args.max_chunks:pairs=pairs[:args.max_chunks]
    binding=dict(source_sha=sources(Path(args.source).resolve(),profile),wheel_sha=files(Path(args.site).resolve()/'cftuv_native'),pair_helper_sha=PAIR_SHA,schema_helper_sha=schema.sha(schema.__file__),pairs={str(p.resolve()):schema.sha(p)for p in pairs},profile_sha256=args.approved_profile_sha256,runner_sha256=schema.sha(__file__),source=str(Path(args.source).resolve()),site=str(Path(args.site).resolve()))
    statepath=out/'replay.json'
    if args.resume:
        state=json.loads(statepath.read_text());assert state['binding']==binding,'RESUME_BINDING'
    else:out.mkdir(exist_ok=False);state=dict(status='INCOMPLETE',binding=binding,chunks=[]);save(statepath,state)
    lock=out/'running.lock'
    with lock.open('x')as stream:json.dump(dict(pid=os.getpid()),stream)
    active=False
    try:
        for index,capture in enumerate(pairs):
            target=out/capture.stem;guard=out/(capture.stem+'-guard')
            if index<len(state['chunks']):
                previous=state['chunks'][index]
                assert files(target)==previous['files'] and schema.sha(guard/'process.json')==previous['guard_sha256'],'RESUME_OUTPUT_CHANGED'
                assert previous['accepted']is True,'ONLY_ACCEPTED_CHUNKS_RESUME'
                continue
            assert not target.exists()and not guard.exists(),'PARTIAL_REQUIRES_NEW_OUTPUT'
            command=[sys.executable,'-B',str(Path(__file__).resolve()),'--worker','--pair',str(capture),'--source',args.source,'--site',args.site,'--approved-profile',args.approved_profile,'--approved-profile-sha256',args.approved_profile_sha256,'--out',str(target)]
            argv=[sys.executable,'-B',args.guard,'run','--receipt',str(guard),'--cwd',args.source,'--floor-gib','1.5','--drives','C:/','D:/','E:/','--',*command]
            active=True;process=subprocess.Popen(argv,cwd=args.source);code=process.wait();active=False
            report=json.loads((guard/'process.json').read_text());assert code==0 and report['status']=='COMPLETE'and report['owned_child_reaped'],'GUARDED_CHUNK_FAILED'
            chunk=json.loads((target/'chunk.json').read_text());assert chunk['status']=='COMPLETE_CHUNK_OBSERVATIONS'
            state['chunks'].append(dict(pair=str(capture),files=files(target),guard_sha256=schema.sha(guard/'process.json'),accepted=chunk['accepted'],tasks=chunk['task_count'],calls=chunk['call_count'],unsupported=chunk['unsupported']));save(statepath,state)
        state.update(status='COMPLETE_REPLAY_SCOPE',accepted=all(v['accepted']for v in state['chunks']),tasks=sum(v['tasks']for v in state['chunks']),calls=sum(v['calls']for v in state['chunks']));save(statepath,state)
    finally:
        if not active:lock.unlink()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('source','site','approved-profile','approved-profile-sha256','out'):p.add_argument('--'+key,required=True)
    p.add_argument('--worker',action='store_true');p.add_argument('--pair');p.add_argument('--pair-root');p.add_argument('--archive',default='*');p.add_argument('--guard');p.add_argument('--resume',action='store_true');p.add_argument('--max-chunks',type=int)
    args=p.parse_args()
    if args.worker:worker(args)
    else:coordinate(args)
if __name__=='__main__':main()
