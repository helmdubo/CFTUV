"""Tiny offline schema checks; no oracle, native import or approval authoring."""
import copy,json
from pathlib import Path
import replay_operation as r
prices={key:[] for key in ('fallback_before','fallback_after','fallback_delta','backend_before','backend_after','backend_delta')}
call=dict(ordinal=0,raw_input_pickle=b'not decoded here',telemetry=prices,geometry_tail=prices,profile_writes=[],measures=[],angle_g_calls=[],exception=dict(type='builtins.ValueError',message='x',outcome=None,patch_domain_id=None))
checks=[]
assert r.compare(call,r.expected(call))['status']=='OPERATION_OBSERVATION_EQUAL';checks.append('equal')
for key in ('telemetry','profile_writes','geometry_tail'):
 candidate=r.expected(call);del candidate[key]
 try:r.compare(call,candidate)
 except AssertionError:checks.append('missing-'+key)
 else:raise AssertionError('MISSING_ACCEPTED')
candidate=copy.deepcopy(r.expected(call));candidate['telemetry']['fallback_delta']=[['canonical',1]]
assert r.compare(call,candidate)['status']=='OPERATION_MISMATCH';checks.append('different-prices')
broken=copy.deepcopy(call);del broken['telemetry']
try:r.expected(broken)
except AssertionError:checks.append('missing-reference-field')
else:raise AssertionError('MISSING_REFERENCE_ACCEPTED')
try:r.approval('does-not-exist',None)
except AssertionError:checks.append('missing-approved-sha')
else:raise AssertionError('APPROVAL_NOT_REQUIRED')
assert r.compare(call,r.expected(call))['full_acceptance']is False;checks.append('context-gap-not-accepted')
assert not r.exact([1],[True]);checks.append('nested-bool-int-distinct')
result=dict(status='PASS',checks=checks,runner_sha256=r.sha(r.__file__),scope='synthetic schema only; no approved artifact/native execution',context_followup='Fresh unpatched reference/native same-input pairs required for whole-operation Decimal/iv entry/exit proof')
out=Path(__file__).with_name('replay-operation-schema-final-receipt.json');assert not out.exists();out.write_text(json.dumps(result,indent=2));print(json.dumps(result))
