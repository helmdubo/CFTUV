"""Offline ANGULAR replay preparation/comparison. No execution or pin approval.
Trusted local pickle only. Candidate MUST run the real facade without Recorder.installed.
A separate approved profile is mandatory; live observations never approve pins.
"""
import argparse
import hashlib
import json
import lzma
import pickle
from pathlib import Path
import angular_raw_pickle
import compare_chunks

SCHEMA='cftuv.angular.operation-observation.v1'
PROFILE_SCHEMA='cftuv.angular.offline-approved-profile.v1'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def approval(path, expected_sha):
    assert type(expected_sha)is str and len(expected_sha)==64,'EXPLICIT_PROFILE_SHA_REQUIRED'
    assert sha(path)==expected_sha,'APPROVED_PROFILE_CHANGED'
    value=json.loads(Path(path).read_text())
    assert value['schema']==PROFILE_SCHEMA and value['approved']is True,'OFFLINE_APPROVAL_REQUIRED'
    for key in ('profile','source_head','kernel_identity','native_build_id','python','sympy','mpmath'):
        assert type(value[key])is str and value[key],'PROFILE_IDENTITY_REQUIRED '+key
    for key in ('hashes','runtime','wire_metadata'):
        assert type(value[key])is dict and value[key],'APPROVED_PIN_CLOSURE_REQUIRED '+key
    return value

def expected(call):
    assert type(call)is dict,'CALL_SCHEMA'
    required=('ordinal','raw_input_pickle','telemetry','profile_writes','measures','angle_g_calls','geometry_tail')
    assert all(k in call for k in required),'MISSING_CALL_FIELD'
    assert type(call['raw_input_pickle'])is bytes,'RAW_INPUT_BYTES'
    assert type(call['telemetry'])is dict and type(call['geometry_tail'])is dict,'PRICE_SCHEMA'
    for prices in (call['telemetry'],call['geometry_tail']):
        assert set(prices)=={'fallback_before','fallback_after','fallback_delta','backend_before','backend_after','backend_delta'},'PRICE_FIELDS'
        assert all(type(v)is list for v in prices.values()),'PRICE_VALUES'
    assert all(type(call[k])is list for k in ('profile_writes','measures','angle_g_calls')),'ORDERED_FIELDS'
    if 'exception' in call:
        assert type(call['exception'])is dict and set(call['exception'])=={'type','message','outcome','patch_domain_id'},'EXCEPTION_SCHEMA'
        assert 'output_codec_bytes'not in call and 'output_pickle'not in call,'MIXED_OUTCOME'
        outcome={'exception':call['exception']}
    else:
        assert type(call['output_codec_bytes'])is bytes and type(call['output_pickle'])is bytes,'RESULT_FIELDS'
        result=pickle.loads(call['output_pickle'])
        assert type(result)is tuple and len(result)==3 and all(type(v)is frozenset for v in result),'RESULT_CONTRACT'
        outcome={'output_codec_bytes':call['output_codec_bytes'],'result':result}
    for measure in call['measures']:
        assert all(k in measure for k in ('orientation','sine','cosine','decimal_before','decimal_after','iv_prec_before','iv_prec_after','telemetry','geometry_since_previous_measure')),'MEASURE_PREFIX_FIELDS'
        assert ('exception'in measure)!=('output_codec_bytes'in measure),'MEASURE_OUTCOME'
    return dict(schema=SCHEMA,outcome=outcome,telemetry=call['telemetry'],profile_writes=call['profile_writes'],
                measures=call['measures'],angle_g_calls=call['angle_g_calls'],geometry_tail=call['geometry_tail'])

def exact(left,right):
    if type(left)is not type(right):return False
    if type(left)is dict:
        return left.keys()==right.keys() and all(exact(left[k],right[k]) for k in left)
    if type(left)in (list,tuple):
        return len(left)==len(right) and all(exact(a,b) for a,b in zip(left,right))
    return left==right

def compare(call,candidate):
    target=expected(call)
    assert type(candidate)is dict and set(candidate)==set(target),'CANDIDATE_SCHEMA_OR_MISSING_FIELDS'
    assert candidate['schema']==SCHEMA,'CANDIDATE_SCHEMA'
    # Strict ordered typed data; no price subtraction, AST normalization or error rewriting.
    different=[key for key in target if not exact(target[key],candidate[key])]
    return dict(status='OPERATION_OBSERVATION_EQUAL'if not different else'OPERATION_MISMATCH',different_fields=different,
                full_acceptance=False,limitation='Recorder lacks operation-entry/exit Decimal/iv snapshots; measure snapshots alone do not prove whole-op context parity')

def prepare(pair_receipt,profile_path,profile_sha):
    profile=approval(profile_path,profile_sha)
    receipt=json.loads(Path(pair_receipt).read_text())
    compare_chunks.verify(pair_receipt,receipt['baseline'],receipt['recorded'],receipt['chunk'],receipt['identity'],receipt['identity_archives'] if 'identity_archives'in receipt else _archives(receipt))
    root=Path(receipt['recorded']);manifest=json.loads((root/'manifest.json').read_text())
    # Baseline is historical FRAME; candidate source/native are separately approved.
    assert (profile['python'],profile['sympy'],profile['mpmath'])==(manifest['python'],manifest['sympy'],manifest['mpmath']),'REPLAY_RUNTIME_PROFILE_DIFFERS'
    calls=[]
    for item in manifest['tasks']:
        path=compare_chunks.contained(root,item['path']);assert sha(path)==item['sha256']
        task=pickle.loads(lzma.decompress(path.read_bytes()))
        for call in task['angular_calls']:
            expected(call)
            raw=pickle.loads(call['raw_input_pickle'])  # stable helper imported above restores mappingproxy
            assert type(raw)is dict and 'profile'not in raw,'RAW_CALL_KEYWORDS'
            calls.append(dict(task_index=item['source_task_index'],ordinal=call['ordinal'],raw=raw,expected=call))
    return profile,calls

def _archives(receipt):
    source=Path(receipt['baseline'])/'manifest.json'
    manifest=json.loads(source.read_text())
    return {Path(key).name:value for key,value in manifest['sources'].items()}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pair',type=Path,required=True)
    parser.add_argument('--approved-profile',type=Path,required=True)
    parser.add_argument('--approved-profile-sha256',required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();assert not args.out.exists(),'NEW_OUTPUT_REQUIRED'
    profile,calls=prepare(args.pair,args.approved_profile,args.approved_profile_sha256)
    result=dict(status='INCOMPLETE_PREPARED_NO_NATIVE_EXECUTION',accepted=False,calls=len(calls),profile=profile['profile'],
                pair_sha256=sha(args.pair),approved_profile_sha256=args.approved_profile_sha256,runner_sha256=sha(__file__),
                context_gap='No whole-operation Decimal/iv entry/exit capture; cannot claim complete parity')
    with args.out.open('x')as stream:json.dump(result,stream,indent=2)

if __name__=='__main__':main()
