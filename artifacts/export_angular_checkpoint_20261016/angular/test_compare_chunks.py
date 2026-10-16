"""Tiny synthetic comparator gates. No host imports, archives, replay or Blender."""
import copy,json,lzma,pickle,tempfile
from pathlib import Path
import compare_chunks as c

ROOT=Path(__file__).resolve().parent
out=ROOT/'paired-comparator-unit-02';out.mkdir(exist_ok=False)
identity=dict(head='h',kernel='k',build='b',recorder_sha256='r',helper_sha256='p',tasks_root=str(out/'archives'))
chunk=dict(id='tiny-000000',archive='tiny.tasks.xz',start=5,count=2,inputs=[dict(case='case',input_sha256='one'),dict(case='case',input_sha256='two')])
archives={'tiny.tasks.xz':'archive-sha'}
def make(root,mode):
 root.mkdir();m={key:'same'for key in c.ENV};m.update(head='h',kernel_identity='k',runner_sha256='r',raw_pickle_helper_sha256='p',native_source_build={'id':'b'},sources={str((out/'archives/tiny.tasks.xz').resolve()):'archive-sha'},phase='COMPLETE',status='INCOMPLETE_LIMIT_CAPTURE',capture_mode=mode,task_count=2,archive=chunk['archive'],start=5,count=2,tasks=[])
 for offset,sha in enumerate(('one','two')):
  row=dict(outcome='inputs',snapshot_bytes=b'snapshot',request_bytes=b'request',telemetry={},profile_counters=[],decimal_before={},decimal_after={},iv_prec_before=53,iv_prec_after=53);row.update(source_task_index=5+offset,ordinal=offset,source_task_blob_sha256=sha,case='case',source=str((out/'archives/tiny.tasks.xz').resolve()),observer_errors=[])
  name='task-'+str(offset)+'.xz';(root/name).write_bytes(lzma.compress(pickle.dumps(row)))
  m['tasks'].append(dict(path=name,sha256=c.sha(root/name),source_task_index=5+offset,source_task_blob_sha256=sha,case='case',observer_errors=0))
 (root/'manifest.json').write_text(json.dumps(m));return m
identity['expected_environment']={key:'same' for key in c.ENV}
identity['expected_environment'].update(head='h',kernel_identity='k',runner_sha256='r',raw_pickle_helper_sha256='p',native_source_build={'id':'b'})
checks=[]
def pair(name):
 base=out/(name+'-baseline');wrapped=out/(name+'-wrapped');make(base,'baseline_without_wrappers');make(wrapped,'angular_wrappers');return base,wrapped

def rejected(name,change):
 a,b=pair(name);change(b)
 try:c.compare(a,b,chunk,identity,archives)
 except (AssertionError,KeyError):checks.append(name);return
 raise AssertionError('FALSE_PASS '+name)

def manifest_change(root,fn):
 p=root/'manifest.json';m=json.loads(p.read_text());fn(m);p.write_text(json.dumps(m))

def row_change(root,fn,rehash=True):
 p=root/'task-0.xz';row=pickle.loads(lzma.decompress(p.read_bytes()));fn(row);p.write_bytes(lzma.compress(pickle.dumps(row)))
 if rehash:manifest_change(root,lambda m:m['tasks'][0].update(sha256=c.sha(p)))
a,b=pair('equal');result=c.compare(a,b,chunk,identity,archives);assert result['status']=='PAIRED_CHUNK_OBSERVER_EQUAL'
receipt=out/'equal.json';receipt.write_text(json.dumps(result));c.verify(receipt,a,b,chunk,identity,archives);checks.append('equal-and-resume')
a,b=pair('difference');row_change(b,lambda row:row.update(snapshot_bytes=b'changed'));assert c.compare(a,b,chunk,identity,archives)['status']=='OBSERVER_PERTURBATION';checks.append('difference')
rejected('order',lambda b:manifest_change(b,lambda m:m['tasks'].reverse()))
rejected('observererror',lambda b:row_change(b,lambda row:row.update(observer_errors=['broken'])))
rejected('tamper',lambda b:row_change(b,lambda row:row.update(request_bytes=b'changed'),False))
rejected('environment',lambda b:manifest_change(b,lambda m:m.update(sympy='different')))
rejected('duplicate-path',lambda b:manifest_change(b,lambda m:m['tasks'][1].update(path=m['tasks'][0]['path'])))
rejected('escape',lambda b:manifest_change(b,lambda m:m['tasks'][0].update(path='../outside.xz')))
rejected('archive',lambda b:manifest_change(b,lambda m:m.update(sources={'other':'wrong'})))
rejected('incomplete',lambda b:manifest_change(b,lambda m:m.update(phase='TASK_ORACLE_RUNNING')))
for field in ('telemetry','request_bytes','outcome'):
 a,b=pair('missing-both-'+field)
 for root in (a,b):row_change(root,lambda row:row.pop(field))
 try:c.compare(a,b,chunk,identity,archives)
 except AssertionError:checks.append('missing-both-'+field)
 else:raise AssertionError('missing applicable field accepted')
a,b=pair('profile-drift-both')
for root in (a,b):manifest_change(root,lambda m:m.update(sympy='both-changed'))
try:c.compare(a,b,chunk,identity,archives)
except AssertionError:checks.append('profile-drift-both')
else:raise AssertionError('both changed profile accepted')
for branch in ('exception','refusal'):
 a,b=pair('legitimate-'+branch)
 def branch_row(row):
  for key in ('outcome','profile_counters','snapshot_bytes','request_bytes'):row.pop(key)
  if branch=='exception':row['exception']=dict(type='builtins.ValueError',message='x',outcome=None,patch_domain_id=None)
  else:row.update(outcome='refusal',profile_counters=[],refusal=dict(outcome='UNSUPPORTED',message='x',domain=None))
 for root in (a,b):row_change(root,branch_row)
 assert c.compare(a,b,chunk,identity,archives)['status']=='PAIRED_CHUNK_OBSERVER_EQUAL'
 checks.append('legitimate-'+branch)
saved=json.loads(receipt.read_text());saved['comparator_sha256']='changed';receipt.write_text(json.dumps(saved))
try:c.verify(receipt,out/'equal-baseline',out/'equal-wrapped',chunk,identity,archives)
except AssertionError:checks.append('comparator-pin')
else:raise AssertionError('changed comparator accepted')
(out/'receipt.json').write_text(json.dumps(dict(status='PASS',checks=checks,comparator_sha256=c.sha(c.__file__)),indent=2));print(json.dumps(dict(status='PASS',checks=len(checks))))
