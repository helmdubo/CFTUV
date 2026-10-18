import json, sys, pstats
from collections import defaultdict
r = json.load(open(sys.argv[1], encoding='utf8'))
print('button', round(r['button_wall'], 3), 'status', r.get('status'), 'missing', r.get('lap_missing'))
laps = r['laps']
for k in sorted(laps, key=lambda k: -laps[k]['s']):
    if laps[k]['s'] >= 0.02 and (k.startswith(('stage.', 'PC.', 'EXPORT', 'PREPARE', 'PRODUCE', 'leaf.', 'native.', 'DB.', 'W.'))):
        print(f"  {k:70s} n={laps[k]['n']:6d} s={laps[k]['s']:8.3f}")
if len(sys.argv) > 2:
    st = pstats.Stats(sys.argv[2])
    total = sum(v[2] for v in st.stats.values())
    print('PLAN_COMPILE profiled tottime', round(total, 3))
    buckets = defaultdict(float)
    files = defaultdict(float)
    for (path, line, name), (cc, nc, tt, ct, callers) in st.stats.items():
        p = path.replace('\\', '/')
        b = 'builtin' if p.startswith(('~', '<')) else 'sympy' if '/sympy/' in p else 'mpmath' if '/mpmath/' in p else 'fractions' if p.endswith('/fractions.py') else 'decimal' if p.endswith('/decimal.py') or '_pydecimal' in p else 'kernel' if '/cftuv_envelope/' in p else 'native' if 'cftuv_native' in p else 'other'
        buckets[b] += tt
        if '/cftuv_envelope/' in p:
            files[p.split('/cftuv_envelope/')[1]] += tt
        elif b != 'builtin':
            files[b + ':' + p.rsplit('/', 1)[-1]] += tt
        else:
            files['builtin:' + name] += tt
    print({k: round(v, 3) for k, v in sorted(buckets.items(), key=lambda kv: -kv[1])})
    print('--- self time by file')
    for k, v in sorted(files.items(), key=lambda kv: -kv[1])[:45]:
        print(f'  {v:8.3f} {k}')
    print('--- cumulative (kernel functions)')
    rows = [(ct, nc, path.replace('\\', '/').split('/cftuv_envelope/')[-1], line, name) for (path, line, name), (cc, nc, tt, ct, callers) in st.stats.items() if '/cftuv_envelope/' in path.replace('\\', '/')]
    for ct, nc, f, line, name in sorted(rows, reverse=True)[:int(sys.argv[3]) if len(sys.argv) > 3 else 70]:
        print(f'  {ct:8.3f} {nc:8d} {f}:{line} {name}')
