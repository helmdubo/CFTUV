"""Harmless executor mechanics only: no host/native execution."""
import decimal,json,pickle
from types import SimpleNamespace as NS
import replay_pair_v2 as r
class Profile:
 def __init__(self,*args):self.rows=[]
 def snapshot(self):return NS(counters=self.rows)
planar=NS(SYMBOLIC_FALLBACK_COUNTS={'canonical':7});symbolic=NS(BACKEND_COUNTS={'x':3});iv=NS(prec=53)
entry=r.context();calls=[]
def body(**kwargs):
 calls.append('body');planar.SYMBOLIC_FALLBACK_COUNTS['canonical']+=2
 decimal.getcontext().prec=19;iv.prec=61
 kwargs['raw']['profile'].rows.append(NS(name='stage',value=2,patch_domain_id='d'))
 return (frozenset(),frozenset(),frozenset())
def reference(k,s,**raw):return body(raw=raw)
def native(**kwargs):kwargs['before_assemble']();return body(raw=kwargs['raw'])
kwargs=dict(recorded_before={'fallback_before':[('canonical',40)],'backend_before':[('angle_certificate_sign.certified',20)]},kernel=NS(),sp=NS(),native=NS(angular_compute=native),profile={'symbolic_mode':'NATIVE_EXACT'},Profile=Profile,reference=reference,
 builder=NS(build_binding_inputs=lambda k:{},build_dependencies=lambda k,s:{}),codec=NS(canonical_json_bytes=lambda v:b'[]'),planar=planar,symbolic=symbolic,iv=iv)
result=r.run_pair(pickle.dumps({'revision':'r'}),**kwargs)
assert result['status']=='FRESH_OPERATION_PAIR_EQUAL' and calls==['body','body']
assert r.context()==entry and iv.prec==53 and planar.SYMBOLIC_FALLBACK_COUNTS=={'canonical':7}
assert result['native']['backend_delta']==[['angle_certificate_sign.certified',0]]
assert result['native']['backend_before']==result['native']['backend_after']==[('angle_certificate_sign.certified',20)]
checks=['zero-key-seed-preserved','same-state-pair','real-shaped-profile-snapshot','finally-state-restore']
calls.clear()
def badnative(**kw):kw['before_assemble']();calls.append('native-error');raise ValueError('measure')
kwargs['native']=NS(angular_compute=badnative)
result=r.run_pair(pickle.dumps({'revision':'r'}),**kwargs)
assert result['status']=='FRESH_OPERATION_PAIR_MISMATCH' and calls==['body','native-error'];checks.append('postphase-no-retry')
assert r.context()==entry and iv.prec==53;checks.append('error-restores-state')
calls.clear()
class NativePortStale(Exception):pass
def stale(**kw):calls.append('stale');raise NativePortStale('EXACT_STALE_DETAIL')
kwargs['native']=NS(angular_compute=stale)
result=r.run_pair(pickle.dumps({'revision':'r'}),**kwargs)
assert result['status']=='NATIVE_NOT_ACCEPTED' and not result['native_accepted']
assert result['native']['exception']['message']=='EXACT_STALE_DETAIL' and result['native']['phase']==[]
assert calls==['body','stale'];checks.append('prephase-stale-preserved-no-retry')
for bad in ([('x',True)],[('x',1),('x',2)],[('x','1')],[['x',1]],(('x',1),)):
 try:r.checked_seed(bad)
 except AssertionError:pass
 else:raise AssertionError('BAD_SEED_ACCEPTED')
checks.append('typed-unique-seed-validation')
def drift(**kw):
 kw['before_assemble']();value=body(raw=kw['raw']);planar.SYMBOLIC_FALLBACK_COUNTS['canonical']+=1;return value
kwargs['native']=NS(angular_compute=drift)
result=r.run_pair(pickle.dumps({'revision':'r'}),**kwargs)
assert result['status']=='FRESH_OPERATION_PAIR_MISMATCH' and 'fallback_after'in result['different_fields'];checks.append('nonzero-price-drift')
assert planar.SYMBOLIC_FALLBACK_COUNTS=={'canonical':7} and symbolic.BACKEND_COUNTS=={'x':3};checks.append('outer-state-distinct-from-recorded-restored')
root=__import__('pathlib').Path(__file__).resolve().parent
capture=__import__('json').loads((root/'paired-capture-01/rounded-000000-pair.json').read_text())
recorded=__import__('pathlib').Path(capture['recorded']);manifest=__import__('json').loads((recorded/'manifest.json').read_text())
row=pickle.loads(__import__('lzma').decompress((recorded/manifest['tasks'][1]['path']).read_bytes()))
telemetry=row['angular_calls'][0]['telemetry']
for key in ('fallback_before','fallback_after','backend_before','backend_after'):
 assert type(telemetry[key])is list and all(type(v)is tuple for v in telemetry[key])
 assert list(r.checked_seed(telemetry[key]).items())==telemetry[key]
checks.append('actual-saved-rounded-tuple-rows')
out=__import__('pathlib').Path(__file__).with_name('replay-pair-v2-tuple-receipt.json');assert not out.exists()
receipt=dict(status='PASS',checks=checks,runner_sha256=r.schema.sha(r.__file__),actual_native_executed=False)
out.write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
