import json, sys, glob, os
from collections import defaultdict
run = json.load(open(sys.argv[1], encoding='utf8'))
d = run.get('worker_dump_dir')
laps = defaultdict(lambda: [0, 0.0])
tasks = []
pids = 0
for path in glob.glob(os.path.join(d, '*.json')):
    data = json.load(open(path, encoding='utf8'))
    pids += 1
    for k, (n, s) in data['laps'].items():
        laps[k][0] += n
        laps[k][1] += s
    tasks += data['tasks']
cpu = sum(t['cpu'] for t in tasks)
print('button', round(run['button_wall'], 3), 'status', run.get('status'), 'workers', pids, 'tasks', len(tasks), 'sum task cpu', round(cpu, 2), 'sum serve wall', round(laps['W.serve_task'][1], 2))
prefixes = tuple(sys.argv[2].split(',')) if len(sys.argv) > 2 else ('stage.', 'PC.', 'EXPORT', 'PREPARE', 'PRODUCE', 'native.', 'DB.', 'W.', 'leaf.')
for k in sorted(laps, key=lambda k: -laps[k][1]):
    if laps[k][1] >= 0.02 and k.startswith(prefixes):
        print(f"  {k:70s} n={laps[k][0]:6d} s={laps[k][1]:8.3f}")
