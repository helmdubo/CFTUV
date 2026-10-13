"""Полный Rust interval whitelist против записанных промежуточных Density-выражений."""
import argparse
import hashlib
import json
import lzma
from pathlib import Path
import subprocess
import sys

sys.set_int_max_str_digits(0)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, action='append', required=True)
    parser.add_argument('--exe', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    program, expected, sources = [], [], []
    for path in args.file:
        doc = json.loads(lzma.decompress(path.read_bytes()))
        program.append('clear')
        expected.append(('CLEARED', None))
        for index, node in enumerate(doc['expressions']):
            program.append('\t'.join(map(str, node)))
            expected.append((f'NODE\t{index}', None))
        for index, (node, precision, result) in enumerate(doc['records']):
            program.append(f'eval\t{precision}\t{node}')
            wanted = 'INTERVAL\t' + '\t'.join(','.join(map(str, endpoint)) for endpoint in result[1]) if result[0] == 'ok' else None
            expected.append((wanted, {'file': str(path), 'record': index, 'node': node, 'outcome': result}))
        sources.append({'file': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                        'nodes': len(doc['expressions']), 'records': len(doc['records'])})
    process = subprocess.Popen([str(args.exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    args.out.with_suffix('.pid').write_text(str(process.pid), encoding='utf-8')
    output, error = process.communicate('\n'.join(program) + '\n')
    rows = output.splitlines()
    failures, refusals = [], []
    for i, ((wanted, location), actual) in enumerate(zip(expected, rows)):
        if wanted is None:
            # Отказ не считается совпавшим по одному лишь ERROR: нужен явный будущий mapper.
            refusals.append({'location': location, 'actual': actual})
        elif actual != wanted:
            failures.append({'index': i, 'input': program[i], 'location': location, 'expected': wanted, 'actual': actual})
    report = {'sources': sources, 'commands': len(program), 'rows': len(rows), 'failures': failures,
              'unmapped_source_refusals': refusals, 'exit_code': process.returncode, 'stderr': error,
              'binary_sha256': hashlib.sha256(args.exe.read_bytes()).hexdigest(), 'pid': process.pid}
    args.out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'records': sum(item['records'] for item in sources), 'commands': len(program),
                      'rows': len(rows), 'failures': len(failures), 'unmapped_source_refusals': len(refusals),
                      'exit_code': process.returncode}), flush=True)
    return bool(failures or refusals or process.returncode or len(program) != len(rows))

if __name__ == '__main__':
    raise SystemExit(main())
