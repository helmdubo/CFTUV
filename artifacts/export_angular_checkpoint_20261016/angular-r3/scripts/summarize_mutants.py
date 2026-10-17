import json
from pathlib import Path
W = Path('D:/cftuv-angular-gates-20261017/mut')
first = [json.loads(l) for l in (W / 'results.jsonl').read_text().splitlines() if l.strip()]
again = {}
for l in (W / 'recheck.jsonl').read_text().splitlines():
    if l.strip():
        r = json.loads(l); again[r['id']] = r['verdict']
EQUIVALENT = {
    'R06_price_cross_zero_paid': 'c = owner_sign*det[n,m] with n,m = +-J*G*tangent, so c = +-det(G)*det[v,w] and c==0 iff turn==0, which the reflex branch excludes: the dropped c!=0 guard is unreachable',
    'O08_same_direction_nonstrict': 'direction is the face edge with the chain edge id, i.e. +-tangent on a consistent wire, so dot_G(tangent,direction)=+-|tangent|_G^2 != 0 (degenerate tangent declines first): >0 and >=0 agree; only a wire whose face edge id joins other vertices than the chain edge could tell them apart',
}
rows = []
for r in first:
    final = r['verdict']
    if final == 'SURVIVED':
        final = again.get(r['id'], 'SURVIVED')
        if final == 'SURVIVED' and r['id'] in EQUIVALENT:
            final = 'EQUIVALENT_SURVIVOR'
    rows.append(dict(id=r['id'], file=r['file'], old=r['old'], new=r['new'], first_gate=r['verdict'], final=final,
                     replay=(r.get('replay') or {}).get('call_status'), native_build_id=r.get('native_build_id'), equivalence=EQUIVALENT.get(r['id'])))
from collections import Counter
summary = dict(schema='cftuv.angular.mutation-gates.v1', mutants=len(rows), final=dict(Counter(x['final'] for x in rows)), first_gate=dict(Counter(x['first_gate'] for x in rows)),
               gates=['cargo test -p cftuv-angular (crate mutants)', 'fast wheel + whole-corpus replay 1218 calls (replay_chunks_v2, stops at first non-equal chunk)',
                      'after new tests: cargo test (admission.rs, stages.rs) / tests/test_native_angular_patch_wire.py on the mutant wheel'],
               baseline=dict(native_build_id='0eee95cffdc9a9220220c79cda4b2c95217c408b3e73c10fd7cdaf05bb9ea2d0', replay='1218/1218 FRESH_OPERATION_PAIR_EQUAL, unsupported 0'),
               note='mutants built in a copy of native/ on D:, never committed; fast wheel = same sources with LTO off (same build id as the release build of the unmutated tree)', rows=rows)
out = Path('D:/CFTUV-export-angular-20261016/artifacts/export_angular_checkpoint_20261016/angular-r3/mutation-gates.json')
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')
print(summary['final'], summary['first_gate'])
