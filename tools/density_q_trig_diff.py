"""Проверка всей закрытой q=2..6 source-таблицы по деревьям выражений."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'kernel' / 'src'))
import sympy as sp
from cftuv_envelope.reference.density_projective import _exact_q_trig
from cftuv_envelope.reference.density_fan_types import AdaptiveDensityFanInvalid


def tree(value):
    if isinstance(value, sp.Rational):
        return f'Q({value.p},{value.q})'
    if value is sp.pi:
        return 'pi'
    return f'{value.func.__name__}({",".join(map(tree, value.args))})'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exe', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    values = list(range(15))
    expected = []
    for q in values:
        try:
            expected.append('Q_TRIG\t' + '\t'.join(map(tree, _exact_q_trig(q))))
        except AdaptiveDensityFanInvalid:
            expected.append('Q_TRIG_UNSUPPORTED')
    proc = subprocess.Popen([str(args.exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    args.out.with_suffix('.pid').write_text(str(proc.pid), encoding='utf-8')
    output, error = proc.communicate('\n'.join(f'q_trig\t{q}' for q in values) + '\n')
    rows = output.splitlines()
    failures = [{'q': q, 'expected': a, 'actual': b} for q, a, b in zip(values, expected, rows) if a != b]
    report = {'cases': len(values), 'rows': len(rows), 'accepted': 5, 'source_unsupported': 10,
              'failures': failures, 'exit_code': proc.returncode, 'stderr': error, 'pid': proc.pid,
              'binary_sha256': hashlib.sha256(args.exe.read_bytes()).hexdigest()}
    args.out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)
    return bool(failures or proc.returncode or len(rows) != len(values))


if __name__ == '__main__':
    raise SystemExit(main())
