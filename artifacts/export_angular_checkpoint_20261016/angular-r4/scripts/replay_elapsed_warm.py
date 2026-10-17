"""Per-call elapsed native vs oracle, splitting the first call of each chunk process (it pays the one-time bind) from warm calls."""
import json, re, statistics, sys
from pathlib import Path
first_ref, first_nat, ref, nat = [], [], [], []
for chunk in sorted(Path(sys.argv[1]).glob('*-pair')):
    calls = sorted(chunk.glob('task-*-call-*.json'), key=lambda f: tuple(map(int, re.findall(r'\d+', f.name))))
    for i, f in enumerate(calls):
        r = json.loads(f.read_text())
        (first_ref if i == 0 else ref).append(r['reference']['elapsed'])
        (first_nat if i == 0 else nat).append(r['native']['elapsed'])
ms = lambda v: round(v * 1e3, 3)
print('first calls', len(first_nat), 'native median ms', ms(statistics.median(first_nat)), 'oracle median ms', ms(statistics.median(first_ref)))
print('warm calls', len(nat), 'native sum s', round(sum(nat), 3), 'median ms', ms(statistics.median(nat)),
      '| oracle sum s', round(sum(ref), 3), 'median ms', ms(statistics.median(ref)),
      '| native<=oracle', sum(n <= o for n, o in zip(nat, ref)), 'of', len(nat))
