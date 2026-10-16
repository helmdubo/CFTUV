"""Чтение двух малых собственных captures; host oracle не выполняется."""
import argparse
import hashlib
import json
import lzma
from pathlib import Path
import pickle


def read(directory):
    manifest = json.loads((directory / 'manifest.json').read_text())
    assert manifest['phase'] == 'COMPLETE' and manifest['task_count'] == len(manifest['tasks']) == 1
    record = manifest['tasks'][0]
    payload = (directory / record['path']).read_bytes()
    assert hashlib.sha256(payload).hexdigest() == record['sha256']
    row = pickle.loads(lzma.decompress(payload))
    assert not row['observer_errors'], 'OBSERVER_ERROR'
    return manifest, row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--recorded', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    assert not args.out.exists(), 'NEW_RECEIPT_REQUIRED'
    a, before = read(args.baseline)
    b, after = read(args.recorded)
    assert a['capture_mode'] == 'baseline_without_wrappers' and b['capture_mode'] == 'angular_wrappers'
    for name in ('head','kernel_identity','python','sympy','mpmath','symbolic_mode','runner_sha256','raw_pickle_helper_sha256','native_source_build','oracle_source_sha256','decimal_context'):
        assert a[name] == b[name], 'INPUT_ENVIRONMENT_DIFFERS ' + name
    assert before['source_task_blob_sha256'] == after['source_task_blob_sha256']
    fields = ('outcome','snapshot_bytes','request_bytes','refusal','exception','telemetry','profile_counters','decimal_before','decimal_after','iv_prec_before','iv_prec_after')
    differences = [name for name in fields if before.get(name) != after.get(name)]
    result = {'status':'BOUNDED_OBSERVER_EQUAL' if not differences else 'OBSERVER_PERTURBATION_OR_ENVIRONMENT_DRIFT',
              'differences':differences,'input_blob_sha256':before['source_task_blob_sha256'],
              'baseline_manifest_sha256':hashlib.sha256((args.baseline/'manifest.json').read_bytes()).hexdigest(),
              'recorded_manifest_sha256':hashlib.sha256((args.recorded/'manifest.json').read_bytes()).hexdigest(),
              'scope':'ONE_TASK_ONLY; no native parity or general observer purity claim',
              'recorder_sha256':a['runner_sha256'], 'kernel_identity':a['kernel_identity'], 'head':a['head'],
              'helper_sha256':a['raw_pickle_helper_sha256'],
              'baseline':str(args.baseline),'recorded':str(args.recorded)}
    with args.out.open('x',encoding='utf8') as handle:json.dump(result,handle,indent=2)
    print(json.dumps(result))
    raise SystemExit(bool(differences))

if __name__ == '__main__':main()
