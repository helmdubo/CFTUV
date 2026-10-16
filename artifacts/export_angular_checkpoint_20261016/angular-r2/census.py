"""Census of planarity certificate classes across ANGULAR paired corpus (read-only)."""
import json, lzma, pickle, sys, collections
from pathlib import Path
sys.path[:0] = ['D:/CFTUV-export-angular-20261016/tools', 'D:/CFTUV-export-angular-20261016/kernel/src',
                'D:/CFTUV-export-angular-20261016', 'D:/CFTUV-export-20261015-evidence/angular']
import native_snapshot_check_corpus as corpus
corpus.install_host_stubs()
import angular_raw_pickle  # noqa: F401  (mappingproxy restore)
root = Path('D:/CFTUV-export-20261015-evidence/angular/paired-capture-01')
calls = collections.Counter(); patches = collections.Counter(); per_case = collections.defaultdict(collections.Counter)
examples = {}
for rec in sorted(root.glob('*-recorded')):
    case = rec.name.split('-')[0]
    manifest = json.loads((rec / 'manifest.json').read_text())
    for item in manifest['tasks']:
        task = pickle.loads(lzma.decompress((rec / item['path']).read_bytes()))
        for call in task['angular_calls']:
            raw = pickle.loads(call['raw_input_pickle'])
            kinds = set()
            for pid, frame in raw['frames'].items():
                cert = frame.planarity_certificate
                cut = getattr(cert, 'cut', None) is not None
                sup = getattr(cert, 'support_triangle_ids', None) is not None
                key = (type(frame).__name__, type(cert).__name__, 'cut' if cut else '-', 'support' if sup else '-')
                patches[key] += 1; kinds.add(key)
            exact_only = all(k[1] == 'ExactSourcePlaneCertificateV1' for k in kinds)
            ck = 'EXACT_ONLY' if exact_only else 'NON_EXACT:' + '+'.join(sorted(k[1] + '/' + k[2] + '/' + k[3] for k in kinds if k[1] != 'ExactSourcePlaneCertificateV1'))
            calls[ck] += 1; per_case[case][ck] += 1
            examples.setdefault(ck, (rec.name, item['source_task_index'], call['ordinal'], 'exception' in call))
print(json.dumps({'calls': calls, 'patches': {'|'.join(k): v for k, v in patches.items()},
                  'per_case': per_case, 'examples': examples}, indent=1, default=str))
