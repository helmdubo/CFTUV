"""Fresh unpatched reference/native ANGULAR pair. Isolated test approval ONLY.
No task export replay, observer wrappers, production approval or fallback retries.
Run under owned_job_guard; explicit source/site/profile paths are mandatory.
"""
import argparse,copy,decimal,hashlib,json,pickle,sys,time
from pathlib import Path
from types import MappingProxyType
import replay_operation as schema
import angular_raw_pickle

PROFILE_SCOPE='isolated parity evaluation only; not production acceptance'

def exception(error):
    result={'type':type(error).__module__+'.'+type(error).__qualname__,'message':str(error)}
    for name in ('outcome','patch_domain_id'):
        value=getattr(error,name,None);result[name]=getattr(value,'value',value)
    return result

def context():
    c=decimal.getcontext()
    return dict(prec=c.prec,rounding=c.rounding,Emin=c.Emin,Emax=c.Emax,capitals=c.capitals,clamp=c.clamp,
                flags=[[k.__name__,v]for k,v in c.flags.items()],traps=[[k.__name__,v]for k,v in c.traps.items()])

def load_pins(profile):
    assert profile['approval_scope']==PROFILE_SCOPE and profile['production_approved']is False,'TEST_ONLY_PROFILE_REQUIRED'
    pins={key:copy.deepcopy(profile[key])for key in ('hashes','runtime','wire_metadata')}
    shapes=pins['wire_metadata']['class_shapes']
    assert type(shapes)is dict,'CLASS_SHAPES'
    for name,value in shapes.items():
        assert type(name)is str and type(value)is list and all(type(v)is str for v in value),'CLASS_SHAPES'
        shapes[name]=tuple(value)
    return pins

def delta(before,after):
    return [[k,after.get(k,0)-before.get(k,0)]for k in list(before)+[k for k in after if k not in before]]

def run_pair(raw_bytes, *, kernel,sp,native,profile,Profile,reference,builder,codec,planar,symbolic,iv):
    # Seed BOTH sides from the same actual entry state; absolute historical capture
    # counters are deliberately not asserted because task prefix was not replayed.
    dc=decimal.getcontext().copy();precision=iv.prec
    stores=(planar.SYMBOLIC_FALLBACK_COUNTS,symbolic.BACKEND_COUNTS)
    seeds=tuple(dict(v)for v in stores)
    def restore():
        decimal.setcontext(dc.copy());iv.prec=precision
        for target,seed in zip(stores,seeds):target.clear();target.update(seed)
    records=[]
    try:
        for kind in ('reference','native'):
            restore();raw=pickle.loads(raw_bytes)
            assert type(raw)is dict and 'profile'not in raw,'RAW_CALL_SCHEMA'
            collector=Profile('angular-replay','isolated-parity');raw['profile']=collector
            before=tuple(dict(v)for v in stores);record=dict(decimal_before=context(),iv_before=iv.prec)
            phase=[];result=None;error=None
            started=time.perf_counter()
            try:
                if kind=='reference':result=reference(kernel,sp,**raw)
                else:
                    result=native.angular_compute(binding_inputs=builder.build_binding_inputs(kernel),
                        dependencies=builder.build_dependencies(kernel,sp),raw=raw,
                        symbolic_mode=profile['symbolic_mode'],before_assemble=lambda:phase.append('assembly'))
            except Exception as caught:error=exception(caught)
            record.update(elapsed=time.perf_counter()-started,decimal_after=context(),iv_after=iv.prec,
                fallback_delta=delta(before[0],stores[0]),backend_delta=delta(before[1],stores[1]),
                profile_counters=[[v.name,v.value,v.patch_domain_id]for v in collector.snapshot().counters],
                exception=error,phase=phase)
            # Serialization must not contaminate captured computation flags/traps.
            if error is None:
                assert type(result)is tuple and len(result)==3 and all(type(v)is frozenset for v in result),'RESULT_CONTRACT'
                with decimal.localcontext(decimal.getcontext().copy()):record['output_bytes']=codec.canonical_json_bytes(result).hex()
            records.append(record)
        fields=('decimal_before','decimal_after','iv_before','iv_after','fallback_delta','backend_delta','profile_counters','exception','output_bytes')
        different=[key for key in fields if (key in records[0])!=(key in records[1]) or not schema.exact(records[0].get(key),records[1].get(key))]
        accepted=records[1]['phase']==['assembly']
        return dict(status=('NATIVE_NOT_ACCEPTED' if not accepted else ('FRESH_OPERATION_PAIR_EQUAL'if not different else'FRESH_OPERATION_PAIR_MISMATCH')),native_accepted=accepted,
            different_fields=different,reference=records[0],native=records[1],historical_absolute_counter_parity=False,
            per_site_ast_or_error_prefix_trace=False,production_acceptance=False)
    finally:restore()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('pair','approved-profile','approved-profile-sha256','source','site','out'):parser.add_argument('--'+key,required=True)
    parser.add_argument('--task-index',type=int,required=True);parser.add_argument('--call-ordinal',type=int,default=0)
    args=parser.parse_args();out=Path(args.out);assert not out.exists(),'NEW_OUTPUT_REQUIRED'
    root=Path(args.source).resolve();site=Path(args.site).resolve()
    profile=schema.approval(args.approved_profile,args.approved_profile_sha256);pins=load_pins(profile)
    for name,digest in profile['source_files'].items():
        source=Path(name) if Path(name).is_absolute() else root/name
        assert schema.sha(source)==digest,'APPROVED_SOURCE_CHANGED '+name
    sys.path[:0]=[str(site),str(root/'tools'),str(root/'kernel/src'),str(root)]
    import native_snapshot_check_corpus as corpus
    corpus.install_host_stubs()
    import cftuv_native as native
    assert Path(native.__file__).resolve().parent==site/'cftuv_native','OWN_SITE_REQUIRED'
    assert native.native_build_id()==profile['native_build_id'],'NATIVE_BUILD_REQUIRED'
    import sympy as sp,mpmath
    assert (sys.version,sp.__version__,mpmath.__version__)==(profile['python'],profile['sympy'],profile['mpmath']),'RUNTIME_PROFILE'
    import cftuv_envelope as kernel
    from cftuv import envelope_angular_relations as angular,envelope_angular_binding as builder
    from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1 as Profile
    from cftuv_envelope.reference import planar_types,symbolic_backend as symbolic
    from cftuv_envelope import codec
    from cftuv_native import angular_op
    assert symbolic.backend_mode().value=='NATIVE_EXACT','CAPTURE_SYMBOLIC_MODE_REQUIRED'
    profile={**profile,'symbolic_mode':'NATIVE_EXACT'}
    assert not angular_op.APPROVED_PINS and not angular_op._BINDINGS,'FRESH_ISOLATED_FACADE_REQUIRED'
    # No live fingerprint generation: this table is exclusively supplied offline.
    angular_op.APPROVED_PINS=MappingProxyType({profile['profile']:pins})
    try:
        _,calls=schema.prepare(args.pair,args.approved_profile,args.approved_profile_sha256)
        selected=[v for v in calls if v['task_index']==args.task_index and v['ordinal']==args.call_ordinal]
        assert len(selected)==1,'EXACT_ONE_RECORDED_CALL_REQUIRED'
        call=selected[0]['expected']
        result=run_pair(call['raw_input_pickle'],kernel=kernel,sp=sp,native=native,profile=profile,Profile=Profile,
            reference=angular._build_angular_relations_reference,builder=builder,codec=codec,
            planar=planar_types,symbolic=symbolic,iv=mpmath.iv)
        # Historical observations are checked separately from the fresh context pair.
        historical={key:call['telemetry'][key] for key in ('fallback_delta','backend_delta')}
        if 'exception'in call:historical['exception']=call['exception']
        else:historical.update(exception=None,output_bytes=call['output_codec_bytes'].hex())
        writes={}
        for name,value,domain in call['profile_writes']:writes[(name,domain)]=[name,value,domain]
        historical['profile_counters']=[writes[k] for k in sorted(writes,key=lambda k:(k[0],k[1]or''))]
        history_diff={side:[key for key,value in historical.items() if not schema.exact(result[side].get(key),value)] for side in ('reference','native')}
        result['historical_delta_output_profile_differences']=history_diff
        if any(history_diff.values()) and result['native_accepted']:result['status']='HISTORICAL_OPERATION_OBSERVATION_MISMATCH'
        post_changed=[]
        for name,digest in profile['source_files'].items():
            source=Path(name) if Path(name).is_absolute() else root/name
            if schema.sha(source)!=digest:post_changed.append(name)
        result['source_sha_after_unchanged']=not post_changed
        result['source_sha_changed']=post_changed
        if post_changed:result['status']='SOURCE_CHANGED_DURING_PAIR'
        result.update(runner_sha256=schema.sha(__file__),pair_sha256=schema.sha(args.pair),
            approved_profile_sha256=args.approved_profile_sha256,task_index=args.task_index,call_ordinal=args.call_ordinal)
        with out.open('x')as stream:json.dump(result,stream,indent=2)
        raise SystemExit(result['status']!='FRESH_OPERATION_PAIR_EQUAL')
    finally:
        angular_op._BINDINGS.clear();angular_op.APPROVED_PINS=MappingProxyType({})

if __name__=='__main__':main()
