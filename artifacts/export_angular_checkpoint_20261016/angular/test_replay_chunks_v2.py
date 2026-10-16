"""Tiny historical result/classification checks, no native or oracle."""
import copy,json
from pathlib import Path
import replay_chunks_v2 as r
call={'telemetry':{'fallback_delta':[['canonical',2]],'backend_delta':[]},'profile_writes':[['z',1,'d'],['a',2,None],['z',3,'d']],'output_codec_bytes':b'[]'}
side={'fallback_delta':[['canonical',2]],'backend_delta':[],'profile_counters':[['a',2,None],['z',3,'d']],'exception':None,'output_bytes':b'[]'.hex()}
for key,value in {'fallback_before':[('canonical',40)],'fallback_after':[('canonical',42)],'backend_before':[],'backend_after':[]}.items():
 call['telemetry'][key]=value;side[key]=copy.deepcopy(value)
def result():return dict(native_accepted=True,status='FRESH_OPERATION_PAIR_EQUAL',reference=copy.deepcopy(side),native=copy.deepcopy(side))
assert r.historical(call,result())['status']=='FRESH_OPERATION_PAIR_EQUAL'
v=result();v['native']['fallback_delta']=[['canonical',3]];assert r.historical(call,v)['status']=='HISTORICAL_OPERATION_OBSERVATION_MISMATCH'
v=result();v.update(native_accepted=False,status='NATIVE_NOT_ACCEPTED');v['native'].update(exception={'type':'cftuv_native.pin.NativePortUnsupported','message':'x'},phase=[],profile_counters=[],fallback_delta=[['canonical',0]],backend_delta=[],decimal_before={},decimal_after={},iv_before=53,iv_after=53);assert r.historical(call,v)['status']=='NATIVE_UNSUPPORTED_NOT_PARITY'
v['native']['exception']['type']='cftuv_native.pin.NativePortStale';v['status']='NATIVE_NOT_ACCEPTED';assert r.historical(call,v)['status']=='NATIVE_NOT_ACCEPTED'
v['native']['exception']['type']='cftuv_native.pin.NativePortUnsupported'
v['reference']['fallback_delta']=[['canonical',99]]
assert r.historical(call,v)['status']=='HISTORICAL_OPERATION_OBSERVATION_MISMATCH'
v['reference']=copy.deepcopy(side);v['native']['fallback_delta']=[['canonical',1]]
assert r.historical(call,v)['status']=='UNSAFE_PRE_EFFECT_DECLINE'
v['native']['fallback_delta']=[['canonical',0]];v['native']['decimal_after']={'changed':True}
assert r.historical(call,v)['status']=='UNSAFE_PRE_EFFECT_DECLINE'
out=Path(__file__).with_name('replay-chunks-v2-tuple-receipt.json');assert not out.exists()
receipt=dict(status='PASS',checks=['historical-equal-lastwrite-profile','price-mismatch','named-unsupported-distinct','stale-not-decline','reference-drift-not-hidden-by-decline','counter-effects-unsafe','context-effects-unsafe'],runner_sha256=r.schema.sha(r.__file__),actual_native_executed=False)
out.write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
