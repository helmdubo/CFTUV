"""Независимая целочисленная проекция recorded MPI в исходные Decimal строки."""

import argparse
from fractions import Fraction
import hashlib
import json
import lzma
from pathlib import Path
import sys

sys.set_int_max_str_digits(0)


def scaled_text(coefficient, exponent):
    """Decimal(coefficient * 10**exponent) с сохранением quantum, без Decimal."""
    sign = '-' if coefficient < 0 else ''
    digits = str(abs(coefficient))
    adjusted = len(digits)-1+exponent
    if exponent <= 0 and adjusted >= -6:
        if exponent == 0:
            return sign+digits
        point = len(digits)+exponent
        if point > 0:
            return sign+digits[:point]+'.'+digits[point:]
        return sign+'0.'+'0'*(-point)+digits
    mantissa = digits[0]+('.'+digits[1:] if len(digits) > 1 else '')
    return sign+mantissa+'E'+('+' if adjusted >= 0 else '')+str(adjusted)


def mpf_fraction(raw):
    sign, mantissa, exponent, bits = raw
    if mantissa == 0 and (exponent != 0 or bits != 0):
        raise ValueError('nonfinite endpoint')
    numerator = (-1 if sign else 1)*mantissa
    return Fraction(numerator << exponent) if exponent >= 0 else Fraction(numerator, 1 << -exponent)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, action='append', required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    reports, failures = [], []
    for path in args.file:
        data = json.loads(lzma.decompress(path.read_bytes()))
        assert data['schema'] == 'cftuv.density-decimal-trace.v2'
        compared, raised = 0, 0
        for index, (_, outcome, intervals) in enumerate(data['records']):
            if outcome[0] != 'ok':
                raised += 1
                continue
            if len(intervals) != 1:
                raise AssertionError(('expected one source enclosure', path, index, intervals))
            lower, upper = [mpf_fraction(raw)*10**27 for raw in intervals[0]]
            lo = lower.numerator // lower.denominator
            hi = -((-upper.numerator) // upper.denominator)
            actual = [scaled_text(value, -27) for value in (lo, hi, hi-lo)]
            if actual != outcome[1:]:
                failures.append({'file': str(path), 'record': index, 'expected': outcome[1:], 'actual': actual})
            compared += 1
        reports.append({'file': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                        'compared': compared, 'raised_not_projection_inputs': raised})
    result = {'scope': 'integer floor/ceil and quantum-preserving text from legacy MPI; not new V2 enclosure',
              'sources': reports, 'failures': failures}
    args.out.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({'compared': sum(row['compared'] for row in reports),
                      'failures': len(failures)}), flush=True)
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(main())
