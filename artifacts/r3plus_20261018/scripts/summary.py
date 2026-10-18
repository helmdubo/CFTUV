"""Phase A summary: worker-second laps (pool w12) and serial laps of the base runs into one JSON."""
import glob, json, os, sys
from collections import defaultdict

OUT = sys.argv[1]
RUNS = {'building_pool': 'building_w12_p3', 'cover008_pool': 'cover008_w12_p2', 'building_pool_p2': 'building_w12_p2',
        'building_serial': 'building_w0_l1', 'cover008_serial': 'cover008_w0_l1'}
base = 'D:/r3plus-scratch/prof/out'
summary = {'tree': '61e46e89260aa06ed3b0fae0d5c88fc9515762cb', 'native_build_id': 'a14d5084e0363e9a3f8693e95e5b1ac48c98045f77d242bae25ff86b52db60d7',
           'note': 'laps are wall seconds of the outermost call per lap name inside the process that ran it (pool: summed over the 12 workers = worker-seconds); '
                   'laps add wrapper overhead; button_wall is the real operator under laps (not a timing sample)', 'runs': {}}
for label, name in RUNS.items():
    run = json.load(open(f'{base}/{name}.json', encoding='utf8'))
    entry = {'button_wall_with_laps': round(run['button_wall'], 3), 'status': run.get('status'), 'decal': run.get('decal'), 'selection_sha': run.get('selection_sha')}
    if run.get('worker_dump_dir'):
        laps = defaultdict(lambda: [0, 0.0])
        dom = {}
        tasks = []
        for path in glob.glob(os.path.join(run['worker_dump_dir'], '*.json')):
            data = json.load(open(path, encoding='utf8'))
            for k, (n, s) in data['laps'].items():
                laps[k][0] += n
                laps[k][1] += s
            dom.update(data['domain'])
            tasks += data['tasks']
        entry['worker_seconds'] = {k: [v[0], round(v[1], 4)] for k, v in sorted(laps.items(), key=lambda kv: -kv[1][1]) if v[1] >= 0.005}
        entry['task_cpu_sum'] = round(sum(t['cpu'] for t in tasks), 3)
        walls = defaultdict(float)
        for t in tasks:
            walls[str(t['patch'])] += t['t1'] - t['t0']
        top = sorted(walls, key=lambda p: -walls[p])[:5]
        entry['heaviest_domains'] = {p: {'wall': round(walls[p], 4), 'laps': {k: round(v, 4) for k, v in sorted(dom.get(p, {}).items(), key=lambda kv: -kv[1]) if v >= 0.004}} for p in top}
    else:
        entry['serial_seconds'] = {k: [v['n'], v['s']] for k, v in sorted(run['laps'].items(), key=lambda kv: -kv[1]['s']) if v['s'] >= 0.005}
    summary['runs'][label] = entry
json.dump(summary, open(OUT, 'w', encoding='utf8'), indent=1, sort_keys=False)
print('wrote', OUT)
