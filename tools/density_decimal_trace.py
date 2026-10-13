"""Записывает decimal authority envelope без потери Decimal scale и grid-соломинок."""

import argparse
from collections import Counter
from contextlib import contextmanager
from fractions import Fraction
import hashlib
import json
import lzma
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'kernel' / 'src'))
sys.set_int_max_str_digits(0)

from cftuv_envelope.reference import density_binding_values
from native_density_corpus import read_document, run_record
from native_density_wire import Wire, json_key


@contextmanager
def capture(wire, records, counts):
    original = density_binding_values._density_decimal_envelope
    original_interval = density_binding_values.density_interval_enclosure
    saved = []
    active_intervals = []

    def traced_interval(*args, **kwargs):
        result = original_interval(*args, **kwargs)
        if active_intervals:
            active_intervals[-1].append([[int(part) for part in endpoint] for endpoint in result._mpi_])
        return result

    def traced(expression, metric=None):
        active_intervals.append([])
        try:
            try:
                envelope = original(expression, metric)
            except Exception as error:
                outcome = ['raised', type(error).__name__, str(error)]
                remember(expression, outcome, active_intervals[-1])
                raise
            outcome = ['ok', str(envelope.lower), str(envelope.upper),
                       str(envelope.absolute_error_bound)]
            remember(expression, outcome, active_intervals[-1])
            return envelope
        finally:
            active_intervals.pop()

    def remember(expression, outcome, intervals):
        row = [wire.expression(expression), outcome, intervals]
        key = json_key(row)
        counts[key] += 1
        if counts[key] == 1:
            records.append(row)

    for name, module in list(sys.modules.items()):
        if name.startswith('cftuv_envelope') and module is not None:
            for attribute, value in list(vars(module).items()):
                if value is original:
                    saved.append((module, attribute, value))
                    setattr(module, attribute, traced)
    density_binding_values.density_interval_enclosure = traced_interval
    try:
        yield
    finally:
        density_binding_values.density_interval_enclosure = original_interval
        for module, attribute, value in saved:
            setattr(module, attribute, value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, action='append', required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    wire, records, counts, sources, mismatches = Wire(), [], Counter(), [], []
    with capture(wire, records, counts):
        for path in args.file:
            document = read_document(path)
            for index, row in enumerate(document['records']):
                if run_record(document, row) != row['outcome']:
                    mismatches.append([str(path), index])
            sources.append({'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                            'records': len(document['records'])})
            print(json.dumps(sources[-1]), flush=True)
    straws, rounded_widths = [], []
    for index, (node_id, outcome, intervals) in enumerate(records):
        if outcome[0] != 'ok':
            continue
        low, high, width = map(Fraction, outcome[1:])
        if high-low != width:
            rounded_widths.append(index)
        node = wire.nodes[node_id]
        if node[0] == 'Q':
            scaled = Fraction(node[1]*10**27, node[2])
            exact_low = scaled.numerator // scaled.denominator
            exact_high = -((-scaled.numerator) // scaled.denominator)
            if (low*10**27, high*10**27) != (exact_low, exact_high):
                straws.append({'record': index, 'rational': node[1:],
                               'exact_units': [exact_low, exact_high],
                               'legacy': outcome[1:], 'occurrences': counts[json_key(records[index])]})
    result = {'schema': 'cftuv.density-decimal-trace.v2', 'sources': sources,
              'scope': 'actual envelope calls in warm-input bind replay; not all full-compile calls',
              'expressions': wire.nodes, 'records': records,
              'occurrences': [counts[json_key(row)] for row in records],
              'mismatches': mismatches, 'rational_grid_straws': straws,
              'decimal_rounded_width_records': rounded_widths}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(lzma.compress(json_key(result).encode('ascii')))
    summary = {'scope': result['scope'], 'sources': sources, 'unique_records': len(records),
               'calls': sum(counts.values()), 'mismatches': len(mismatches),
               'rational_grid_straws': straws, 'decimal_rounded_width_records': rounded_widths,
               'sha256': hashlib.sha256(args.out.read_bytes()).hexdigest()}
    args.out.with_suffix('.receipt.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps(summary), flush=True)
    return bool(mismatches)


if __name__ == '__main__':
    raise SystemExit(main())
