import json, statistics, sys
from pathlib import Path
ref, nat = [], []
for f in Path(sys.argv[1]).glob('*-pair/task-*-call-*.json'):
    r = json.loads(f.read_text())
    ref.append(r['reference']['elapsed']); nat.append(r['native']['elapsed'])
print('calls', len(ref), 'reference sum', round(sum(ref), 3), 'median ms', round(statistics.median(ref) * 1e3, 3),
      '| native sum', round(sum(nat), 3), 'median ms', round(statistics.median(nat) * 1e3, 3), 'max ms', round(max(nat) * 1e3, 1))
