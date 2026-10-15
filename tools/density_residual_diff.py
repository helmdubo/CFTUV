"""Rust-фильтр Density против actual source, с записью вызовов границы bind."""
import argparse
from collections import Counter
import hashlib
import json
import lzma
from pathlib import Path
import random
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'kernel' / 'src'))
sys.path.insert(0, str(ROOT / 'tools'))
sys.set_int_max_str_digits(0)

import sympy as sp
from cftuv_envelope.reference import density_residual
from native_density_corpus import read_document, run_record
from native_density_wire import Wire, json_key


class Metric:
    def __init__(self):
        self._density_exact_memo = type('Memo', (), {'intervals': {}})()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, action='append', default=[])
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--reuse-trace', action='store_true')
    parser.add_argument('--exe', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.reuse_trace:
        doc = json.loads(lzma.decompress(args.trace.read_bytes()))
    else:
        wire, rows, counts, mismatches, sources = Wire(), [], Counter(), [], []
        original = density_residual._filtered_sign

        def traced(metric, dot, left, right, q):
            result = original(metric, dot, left, right, q)
            row = [wire.expression(dot), wire.expression(left), wire.expression(right), q, result]
            key = json_key(row)
            counts[key] += 1
            if counts[key] == 1:
                rows.append(row)
            return result

        density_residual._filtered_sign = traced
        try:
            for path in args.file:
                source = read_document(path)
                for index, row in enumerate(source['records']):
                    actual = run_record(source, row)
                    if actual != row['outcome']:
                        mismatches.append({'file': str(path), 'record': index, 'expected': row['outcome'], 'actual': actual})
                    if (index + 1) % 25 == 0:
                        print(json.dumps({'file': str(path), 'done': index + 1, 'filter_calls': sum(counts.values())}), flush=True)
                sources.append({'file': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'records': len(source['records'])})
            # Отступы от границы по обе стороны запаса и opaque source failures.
            rng = random.Random(0xD3517)
            for q in range(1, 8):
                for bits in (20, 80, 99, 100, 101, 110, 160, 220):
                    for sign in (-1, 0, 1):
                        dot = sp.Add(sp.Rational(1, 2), sign * sp.Rational(1, 2**bits), evaluate=False)
                        traced(Metric(), dot, sp.S.One, sp.S.One, q)
                for _ in range(150):
                    dot = sp.Add(sp.Rational(rng.randrange(-20, 21), rng.randrange(1, 33)),
                                 sp.sin(sp.Rational(rng.randrange(-20, 21), 17)), evaluate=False)
                    traced(Metric(), dot, sp.Rational(rng.randrange(1, 40), 13), sp.sqrt(2), q)
                for dot in (sp.S.One, sp.sqrt(2), sp.Add(sp.S.One, sp.tan(1), evaluate=False),
                            sp.Add(sp.S.One, sp.Pow(-1, sp.S.Half, evaluate=False), evaluate=False)):
                    traced(Metric(), dot, sp.S.One, sp.S.One, q)
        finally:
            density_residual._filtered_sign = original
        doc = {'schema': 'cftuv.density-residual-trace.v1', 'sources': sources, 'expressions': wire.nodes,
               'records': rows, 'occurrences': [counts[json_key(row)] for row in rows], 'bind_mismatches': mismatches}
        args.trace.parent.mkdir(parents=True, exist_ok=True)
        args.trace.write_bytes(lzma.compress(json_key(doc).encode('ascii')))
    program = ['clear']
    expected = ['CLEARED']
    for index, node in enumerate(doc['expressions']):
        program.append('\t'.join(map(str, node)))
        expected.append(f'NODE\t{index}')
    for dot, left, right, q, answer in doc['records']:
        program.append(f'residual_filter\t{dot}\t{left}\t{right}\t{q}')
        expected.append('FILTER\t' + ('NONE' if answer is None else str(answer)))
    process = subprocess.Popen([str(args.exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    args.out.with_suffix('.pid').write_text(str(process.pid), encoding='utf-8')
    output, error = process.communicate('\n'.join(program) + '\n')
    actual = output.splitlines()
    failures = [{'command': program[i], 'expected': a, 'actual': b} for i, (a, b) in enumerate(zip(expected, actual)) if a != b]
    report = {'sources': doc['sources'], 'records': len(doc['records']), 'calls': sum(doc['occurrences']),
              'outcomes': dict(Counter(str(row[-1]) for row in doc['records'])), 'failures': failures,
              'bind_mismatches': doc['bind_mismatches'], 'commands': len(program), 'rows': len(actual),
              'trace_sha256': hashlib.sha256(args.trace.read_bytes()).hexdigest(),
              'binary_sha256': hashlib.sha256(args.exe.read_bytes()).hexdigest(), 'pid': process.pid,
              'exit_code': process.returncode, 'stderr': error}
    args.out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('records', 'calls', 'outcomes', 'exit_code')} | {'failures': len(failures), 'bind_mismatches': len(doc['bind_mismatches'])}), flush=True)
    return bool(failures or doc['bind_mismatches'] or process.returncode or len(program) != len(actual))


if __name__ == '__main__':
    raise SystemExit(main())
