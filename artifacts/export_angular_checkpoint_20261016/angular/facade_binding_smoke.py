"""Actual installed facade binding smoke; static test-only profile, no geometry."""
import hashlib,json,os,sys,time
from pathlib import Path
from types import MappingProxyType

WORK=Path(__file__).resolve().parent
ROOT=Path('D:/CFTUV-export-angular-20261016')
SITE=WORK/'wheel-build-01/site'
DEPS=Path('C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages')
PROFILE=WORK/'profile-311-light-4f3-isolated-parity-01.json'
PROFILE_SHA='ee7b8f013d8dcb7c8b98ccab2fb5d13044007583934d22bcc683c63df3f72b0c'
sys.path[:0]=list(map(str,(SITE,ROOT/'kernel/src',ROOT,ROOT/'tools',DEPS)))
import wheel_job
from owned_job_guard_v2 import creation_filetime
import _winapi


def main():
    out=WORK/'installed-binding-smoke-01';out.mkdir(exist_ok=False)
    finish=wheel_job.gate.setup(out/'job.json');code=1
    report=dict(status='INCOMPLETE',scope='Actual installed binding only; no geometry/native evaluator/oracle',
                pid=os.getpid(),creation_filetime=creation_filetime(_winapi.GetCurrentProcess()),argv=sys.argv,
                profile_sha256=PROFILE_SHA,script_sha256=wheel_job.gate.sha(__file__),checks=[],started=time.time())
    old_pins=None;op=None
    try:
        wheel_job.source_guard()
        assert wheel_job.gate.sha(PROFILE)==PROFILE_SHA
        profile=json.loads(PROFILE.read_text());assert profile['approved']is True and profile['production_approved']is False
        assert profile['approval_scope']=='isolated parity evaluation only; not production acceptance'
        assert profile['source_head']==wheel_job.HEAD and profile['kernel_identity']==wheel_job.KERNEL and profile['native_build_id']==wheel_job.PARTS['id']
        assert profile['python']==sys.version
        pins={key:profile[key]for key in ('hashes','runtime','wire_metadata')}
        shapes=pins['wire_metadata']['class_shapes']
        for name,fields in shapes.items():
            assert type(name)is str and type(fields)is list and all(type(value)is str for value in fields)
            shapes[name]=tuple(fields)
        assert all(type(value)is str for value in pins['runtime']['fingerprints'].values())
        from native_snapshot_check_corpus import install_host_stubs
        install_host_stubs()
        import cftuv_native,sympy,mpmath,cftuv_envelope as kernel
        from cftuv_native import angular_op as installed_op
        from cftuv import envelope_angular_binding as host,envelope_angular_relations as angular
        from cftuv_envelope.reference import planar_types as pt
        op=installed_op
        assert Path(cftuv_native.__file__).resolve().parent==(SITE/'cftuv_native').resolve()
        assert {key:cftuv_native.native_build_parts()[key]for key in wheel_job.PARTS}==wheel_job.PARTS
        assert sympy.__version__==profile['sympy']and mpmath.__version__==profile['mpmath']
        assert not op.APPROVED_PINS
        old_pins=op.APPROVED_PINS;op.APPROVED_PINS=MappingProxyType({profile['profile']:pins})
        counts=dict(pt.SYMBOLIC_FALLBACK_COUNTS)
        def warm():
            inputs=host.build_binding_inputs(kernel);assert inputs['profile']==profile['profile']
            return cftuv_native.warm_angular(binding_inputs=inputs,dependencies=host.build_dependencies(kernel,sympy))
        assert warm()and warm();report['checks'].append(dict(name='actual_fresh_inputs_warm_twice',status='PASS'))
        def stale(name,change,restore):
            change()
            try:
                try:warm()
                except cftuv_native.NativePortStale as error:
                    report['checks'].append(dict(name=name,status='PASS',exception=dict(type=type(error).__module__+'.'+type(error).__qualname__,message=str(error))))
                else:raise AssertionError('STALE_NOT_REJECTED '+name)
            finally:restore()
            assert warm()
        original=angular.chart_points
        stale('actual_oracle_chart_points_alias',lambda:setattr(angular,'chart_points',lambda *args:(_ for _ in ()).throw(AssertionError('GEOMETRY_FORBIDDEN'))),lambda:setattr(angular,'chart_points',original))
        original_counts=pt.SYMBOLIC_FALLBACK_COUNTS
        stale('replay_counter_dictionary_identity',lambda:setattr(pt,'SYMBOLIC_FALLBACK_COUNTS',dict(original_counts)),lambda:setattr(pt,'SYMBOLIC_FALLBACK_COUNTS',original_counts))
        value=original_counts['canonical']
        try:
            original_counts['canonical']=value+1
            assert warm();report['checks'].append(dict(name='counter_value_change_not_stale',status='PASS'))
        finally:original_counts['canonical']=value
        assert dict(pt.SYMBOLIC_FALLBACK_COUNTS)==counts
        op.APPROVED_PINS=old_pins;assert not op.APPROVED_PINS
        wheel_job.source_guard();code=0
        report.update(status='ACTUAL_INSTALLED_BINDING_SMOKE_PASS',native_status=cftuv_native.native_status(),
                      head=wheel_job.HEAD,native_parts=wheel_job.PARTS,site=str(SITE),no_native_geometry=True,
                      counters_restored=True,pins_restored_empty=True,ended=time.time())
    except BaseException as error:
        report.update(status='FAILED_NOT_ACCEPTED',error=dict(type=type(error).__module__+'.'+type(error).__qualname__,message=str(error)),ended=time.time())
        raise
    finally:
        if op is not None and old_pins is not None:op.APPROVED_PINS=old_pins
        wheel_job.gate.save(out/'receipt.json',report);finish(code)
    print(json.dumps(report))
    return code

if __name__=='__main__':raise SystemExit(main())
