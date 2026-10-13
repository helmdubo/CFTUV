"""Admission grammar exact_quadratic_value, без implicit expand, против Rust V2 reader."""
import argparse
import hashlib
import json
import lzma
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'kernel' / 'src'))
sys.path.insert(0, str(ROOT / 'tools'))
sys.set_int_max_str_digits(0)
from native_density_wire import Wire
from cftuv_envelope.reference.planar_types import exact_quadratic_value, ExactQuadraticFieldUnsupported
from cftuv_envelope.reference.radical_v2 import from_v1

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, action='append', required=True)
    parser.add_argument('--exe', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    program, expected, sources = [], [], []
    for path in args.file:
        doc = json.loads(lzma.decompress(path.read_bytes()))
        wire = Wire(doc['expressions'])
        program.append('clear'); expected.append(('CLEARED', None))
        for index, node in enumerate(doc['expressions']):
            program.append('\t'.join(map(str, node))); expected.append((f'NODE\t{index}', None))
        accepted, refused = 0, 0
        for index in range(len(wire.nodes)):
            expression = wire.unexpression(index)
            try:
                value, _ = from_v1(exact_quadratic_value(expression))
            except ExactQuadraticFieldUnsupported:
                wanted = 'UNSUPPORTED'; refused += 1
            else:
                wanted = 'RADICAL\t' + ','.join(map(str, value.wire())); accepted += 1
            program.append(f'exact\t{index}')
            expected.append((wanted, {'file': str(path), 'node': index}))
            if (index + 1) % 5000 == 0:
                print(json.dumps({'file': str(path), 'done': index + 1, 'accepted': accepted, 'refused': refused}), flush=True)
        sources.append({'file': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                        'nodes': len(wire.nodes), 'accepted': accepted, 'refused': refused})
    proc = subprocess.Popen([str(args.exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    args.out.with_suffix('.pid').write_text(str(proc.pid), encoding='utf-8')
    output, error = proc.communicate('\n'.join(program) + '\n')
    rows = output.splitlines()
    failures = [{'index': i, 'input': program[i], 'location': location, 'expected': wanted, 'actual': actual}
                for i, ((wanted, location), actual) in enumerate(zip(expected, rows)) if wanted != actual]
    report = {'scope': 'Raw reader grammar; not expand normalization or whole sign operation',
              'sources': sources, 'commands': len(program), 'rows': len(rows), 'failures': failures,
              'exit_code': proc.returncode, 'stderr': error, 'pid': proc.pid,
              'binary_sha256': hashlib.sha256(args.exe.read_bytes()).hexdigest()}
    args.out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'nodes': sum(item['nodes'] for item in sources), 'rows': len(rows),
                      'failures': len(failures), 'exit_code': proc.returncode}), flush=True)
    return bool(failures or proc.returncode or len(program) != len(rows))

if __name__ == '__main__':
    raise SystemExit(main())
