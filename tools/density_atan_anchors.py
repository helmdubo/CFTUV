"""Конечная калибровка legacy atan cache; независимая рациональная граница."""

from collections import Counter
import argparse
from fractions import Fraction as F
import hashlib
import json
from pathlib import Path
import sys

from mpmath.libmp import libelefun
from density_atan_seed_probe import atan_bounds

sys.set_int_max_str_digits(0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    precisions = (28, 36, 52, 84, 148, 276, 532)
    terms = 352
    a, b = atan_bounds(F(1, 5), terms), atan_bounds(F(1, 239), terms)
    pi = (16*a[0]-4*b[1], 16*a[1]-4*b[0])
    records = []
    for n in range(256):
        if n <= 64:
            lower, upper = atan_bounds(F(n, 128), terms)
        else:
            delta = atan_bounds(F(n-128, n+128), terms)
            lower, upper = pi[0]/4+delta[0], pi[1]/4+delta[1]
        for precision in precisions:
            low = (lower.numerator << precision) // lower.denominator
            high = (upper.numerator << precision) // upper.denominator
            if low != high:
                raise AssertionError(('uncertified floor', n, precision))
            value = int(libelefun.atan_newton(n << (precision-7), precision))
            records.append({'n': n, 'precision': precision, 'legacy': str(value),
                            'certified_floor': str(low), 'delta': value-low})
    reverse_differences = []
    libelefun.cos_sin_cache.clear()
    # Очищается только память собственного процесса калибровки.
    for row in reversed(records):
        value = int(libelefun.atan_newton(row['n'] << (row['precision']-7), row['precision']))
        if value != int(row['legacy']):
            reverse_differences.append([row['n'], row['precision'], str(value)])
    result = {'scope': 'finite n=0..255 and seven cache precisions; canonical source cache only',
              'source_sha256': hashlib.sha256(Path(libelefun.__file__).read_bytes()).hexdigest(),
              'series_terms': terms, 'records': records,
              'delta_histogram': dict(Counter(row['delta'] for row in records)),
              'reverse_differences': reverse_differences}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({'cases': len(records), 'delta_histogram': result['delta_histogram'],
                      'reverse_differences': len(reverse_differences)}), flush=True)
    return bool(reverse_differences)


if __name__ == '__main__':
    raise SystemExit(main())
