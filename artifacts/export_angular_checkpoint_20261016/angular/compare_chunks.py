"""Exact paired-chunk observer comparison; trusted local pickle inputs only."""
import argparse,hashlib,json,lzma,pickle
from pathlib import Path

FIELDS=('outcome','snapshot_bytes','request_bytes','refusal','exception','telemetry','profile_counters','decimal_before','decimal_after','iv_prec_before','iv_prec_after')
ENV=('head','kernel_identity','python','sympy','mpmath','symbolic_mode','runner_sha256','raw_pickle_helper_sha256','native_source_build','oracle_source_sha256','decimal_context','raw_pickle_policy','schema','scope')
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def contained(root,path):
    result=(root/path).resolve()
    assert result.is_relative_to(root.resolve())and result!=root.resolve(),'ARTIFACT_ESCAPES'
    return result


def project(row):
    assert type(row) is dict,'ROW_SCHEMA'
    common=('telemetry','decimal_before','decimal_after','iv_prec_before','iv_prec_after')
    assert all(k in row for k in common),'MISSING_COMMON_FIELD'
    assert type(row['telemetry']) is dict,'TELEMETRY_SCHEMA'
    assert all(type(row[k]) is dict for k in ('decimal_before','decimal_after')),'CONTEXT_SCHEMA'
    assert all(type(row[k]) is int for k in ('iv_prec_before','iv_prec_after')),'PRECISION_SCHEMA'
    if 'exception' in row:
        required={'exception'};optional=set()
        assert type(row['exception']) is dict and all(k in row['exception'] for k in ('type','message','outcome','patch_domain_id')),'EXCEPTION_SCHEMA'
    else:
        assert row.get('outcome') in ('inputs','refusal'),'OUTCOME_DISCRIMINATOR'
        required={'outcome','profile_counters'}
        assert type(row.get('profile_counters')) is list,'COUNTERS_SCHEMA'
        if row['outcome']=='inputs':required|={'snapshot_bytes','request_bytes'};optional=set()
        else:
            required.add('refusal');optional={'snapshot_bytes'}
            assert type(row.get('refusal')) is dict and set(row['refusal'])=={'outcome','message','domain'},'REFUSAL_SCHEMA'
    applicable=set(common)|required|optional
    assert required<=row.keys(),'MISSING_APPLICABLE_FIELD'
    assert not ((set(FIELDS)-applicable)&row.keys()),'INAPPLICABLE_FIELD_PRESENT'
    for key in ('snapshot_bytes','request_bytes'):
        if key in row:assert type(row[key]) is bytes,'CODEC_BYTES_SCHEMA'
    # Explicit None represents only schema-proven inapplicability, never missing data.
    return tuple(row[k] if k in row else None for k in FIELDS)


def read(directory,mode,chunk,identity,archives):
    directory=Path(directory).resolve();manifest_path=directory/'manifest.json'
    m=json.loads(manifest_path.read_text());assert m['phase']=='COMPLETE'and m['status']in('INCOMPLETE_LIMIT_CAPTURE','RECORDED_NOT_PARITY_ACCEPTED'),'INCOMPLETE_CAPTURE'
    assert m['capture_mode']==mode and m['task_count']==len(m['tasks'])==chunk['count'],'MODE_OR_COUNT'
    assert (m['archive'],m['start'],m['count'])==(chunk['archive'],chunk['start'],chunk['count']),'CHUNK_RANGE'
    for key,target in (('head','head'),('kernel_identity','kernel'),('runner_sha256','recorder_sha256'),('raw_pickle_helper_sha256','helper_sha256')):
        assert m[key]==identity[target],'IDENTITY_DIFFERS '+key
    assert m['native_source_build']['id']==identity['build'],'BUILD_DIFFERS'
    source=str((Path(identity['tasks_root'])/chunk['archive']).resolve())
    assert {str(Path(k).resolve()):v for k,v in m['sources'].items()}=={source:archives[chunk['archive']]},'SOURCE_ARCHIVE_DIFFERS'
    paths=set();indices=set();rows=[];audit={}
    for offset,item in enumerate(m['tasks']):
        index=chunk['start']+offset;expected=chunk['inputs'][offset]
        assert item['source_task_index']==index and index not in indices,'TASK_ORDER_OR_DUPLICATE'
        indices.add(index);assert item['source_task_blob_sha256']==expected['input_sha256']and item['case']==expected['case'],'ORDERED_INPUT_DIFFERS'
        path=contained(directory,item['path']);assert path not in paths,'DUPLICATE_PATH'
        paths.add(path);assert sha(path)==item['sha256'],'ARTIFACT_CHANGED'
        row=pickle.loads(lzma.decompress(path.read_bytes()))
        assert not item['observer_errors']and not row['observer_errors'],'OBSERVER_ERROR'
        assert row['source_task_index']==index and row['ordinal']==offset,'ROW_ORDER'
        assert row['source_task_blob_sha256']==expected['input_sha256']and row['case']==expected['case'],'ROW_INPUT_DIFFERS'
        assert str(Path(row['source']).resolve())==source,'ROW_ARCHIVE_DIFFERS'
        rows.append(row);audit[item['path']]=item['sha256']
    assert {p.resolve()for p in directory.iterdir()if p.is_file()}==paths|{manifest_path},'UNTRACKED_ARTIFACT'
    return m,rows,dict(manifest_sha256=sha(manifest_path),files=audit)


def compare(baseline,recorded,chunk,identity,archives):
    a,x,aa=read(baseline,'baseline_without_wrappers',chunk,identity,archives)
    b,y,bb=read(recorded,'angular_wrappers',chunk,identity,archives)
    for manifest in (a,b):
        for name in ENV:assert manifest[name]==identity['expected_environment'][name],'EXPECTED_ENVIRONMENT_DIFFERS '+name
    differences=[{'offset':n,'fields':[key for key,l,r in zip(FIELDS,project(left),project(right)) if l!=r]}for n,(left,right)in enumerate(zip(x,y))]
    differences=[row for row in differences if row['fields']]
    return dict(status='PAIRED_CHUNK_OBSERVER_EQUAL'if not differences else'OBSERVER_PERTURBATION',
                differences=differences,task_count=len(x),chunk=chunk,identity=identity,
                baseline=str(Path(baseline).resolve()),recorded=str(Path(recorded).resolve()),
                audits={'baseline':aa,'recorded':bb},comparator_sha256=sha(__file__),not_native_parity=True)


def verify(receipt,baseline,recorded,chunk,identity,archives):
    saved=json.loads(Path(receipt).read_text())
    assert saved['comparator_sha256']==sha(__file__),'COMPARATOR_CHANGED'
    fresh=compare(baseline,recorded,chunk,identity,archives)
    assert fresh==saved and saved['status']=='PAIRED_CHUNK_OBSERVER_EQUAL','PAIR_RECEIPT_CHANGED_OR_MISMATCH'
    return saved


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('baseline','recorded','spec','out'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();spec=json.loads(a.spec.read_text());assert not a.out.exists(),'NEW_RECEIPT_REQUIRED'
    result=compare(a.baseline,a.recorded,spec['chunk'],spec['identity'],spec['archives'])
    with a.out.open('x')as f:json.dump(result,f,indent=2)
    raise SystemExit(result['status']!='PAIRED_CHUNK_OBSERVER_EQUAL')
if __name__=='__main__':main()
