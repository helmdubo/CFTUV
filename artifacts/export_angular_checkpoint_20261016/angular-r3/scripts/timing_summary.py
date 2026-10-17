import json, statistics, sys
from pathlib import Path
d = Path(sys.argv[1])
c = json.loads((d / 'controller.json').read_text())
print('accepted', c.get('accepted'), 'failure', c.get('failure'))
for case, sides in c.get('summary', {}).items():
    b, r = sides['base']['median'], sides['branch']['median']
    print(case, 'base', [round(x, 3) for x in sides['base']['samples']], 'med', round(b, 4), '| branch', [round(x, 3) for x in sides['branch']['samples']], 'med', round(r, 4), f'{(r - b) / b * 100:+.2f}%')
for f in sorted(d.glob('*_branch.json')):
    j = json.loads(f.read_text())
    hook = d / (f.stem + '.hook.jsonl')
    n = sum(1 for l in hook.read_text().splitlines() if l.strip()) if hook.exists() else 0
    status = j.get('status', '')
    print(f.stem, 'angular_env', j.get('angular_env'), 'angular_status', j.get('angular_status'), 'injections', n, 'build', j['native']['build_id'][:8], 'digest', j['decal']['digest'][:12])
    print('   ', status[:400].replace('\n', ' | '))
for f in sorted(d.glob('*_base.json')):
    j = json.loads(f.read_text())
    print(f.stem, 'build', j['native']['build_id'][:8], 'head', j['tree_head'][:8], 'digest', j['decal']['digest'][:12])
print('foreign', [r.get('foreign_idle') for r in c['runs']][:2])
