import json, sys, glob, os
from collections import defaultdict
run = json.load(open(sys.argv[1], encoding='utf8'))
d = run['worker_dump_dir']
dom = {}
walls = {}
for path in glob.glob(os.path.join(d, '*.json')):
    data = json.load(open(path, encoding='utf8'))
    for k, v in data['domain'].items():
        dom[k] = v
    for t in data['tasks']:
        walls[str(t['patch'])] = walls.get(str(t['patch']), 0) + (t['t1'] - t['t0'])
keys = sys.argv[3].split(',') if len(sys.argv) > 3 else ['PREPARE.total', 'PC.compile_reference_envelopes', 'PCD.native_op_density_compute_with_memo', 'PCD.density_pin_runtime_stale', 'PCD.prepared_attachment', 'PC.attach_front_reading_declarations', 'PC.chain_station_plans@PLAN_COMPILE', 'PC.validate_reference_geometry_certificates', 'PCE.verify_evaluation_geometry_binding', 'PC.verify_evaluation_direction_binding_reasons@PREPARE', 'native.skeleton_compute', 'PRODUCE.total', 'EXPORT.task_inputs']
top = sorted(walls, key=lambda p: -walls[p])[:int(sys.argv[2]) if len(sys.argv) > 2 else 6]
print('patch wall ' + ' '.join(k.split('.')[-1][:18] for k in keys))
for p in top:
    print(p, round(walls[p], 3), ' '.join(f"{dom.get(p, {}).get(k, 0):.3f}" for k in keys))
