"""Per case/workers/side: ANGULAR native/python/fallback calls summed over the domain rows of every press, and the injection log."""
import json, sys
from collections import Counter, defaultdict
report = json.load(open(sys.argv[1]))
out = {}
for result in report['results']:
    key = f"{result['case']} w{result.get('workers')}"
    row = {}
    for side, rep in result['reports'].items():
        totals = defaultdict(lambda: [0, 0, 0]); ran = Counter()
        for press, data in rep['presses'].items():
            for r in data.get('rows', []):
                calls = r.get('angular_calls')
                if calls:
                    totals[press] = [a + b for a, b in zip(totals[press], calls)]
                ran[(press, r.get('angular_ran'))] += 1
        row[side] = dict(calls={p: v for p, v in totals.items()}, ran={f'{p}:{v}': n for (p, v), n in ran.items()})
    row['differences'] = len(result.get('differences', []))
    out[key] = row
hook = [json.loads(l) for l in open(sys.argv[2]) if l.strip()] if len(sys.argv) > 2 else []
summary = dict(verdict_lines=None, cases=out, hook_injections=len(hook), hook_processes=dict(Counter('blender' if r['argv0'].endswith('blender.exe') else 'pool-worker' for r in hook)),
               hook_profiles=sorted({tuple(r['profiles']) for r in hook}), hook_modules=sorted({r['module'] for r in hook}))
print(json.dumps(summary, indent=1))
