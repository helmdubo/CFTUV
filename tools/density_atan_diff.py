"""Сравнение integer-only atan/atan2 с сырыми результатами Python-эталона."""
import argparse
import hashlib
import json
import lzma
from pathlib import Path
import random
import subprocess
import sys
from mpmath.libmp import libelefun as ef, libmpi as iv, libmpf as mp

sys.set_int_max_str_digits(0)

def raw(value):
    return ','.join(map(str, value))

def scalar(name, values):
    if name == 'mpf_atan':
        x, p, *rounding = values
        r = rounding[0] if rounding else 'd'
        return f'atan\t{p}\t{r}\t{raw(x)}'
    y, x, p, *rounding = values
    r = rounding[0] if rounding else 'd'
    return f'atan2\t{p}\t{r}\t{raw(y)}\t{raw(x)}'

def interval(name, values):
    if name == 'mpi_atan':
        (a, b), p = values
        return f'iatan\t{p}\t{raw(a)}\t{raw(b)}'
    (ya, yb), (xa, xb), p = values
    return f'iatan2\t{p}\t{raw(ya)}\t{raw(yb)}\t{raw(xa)}\t{raw(xb)}'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, action='append', default=[])
    parser.add_argument('--exe', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--random', type=int, default=3000)
    args = parser.parse_args()
    cases, sources = [], []
    def add(name, values, result):
        if name in ('mpf_atan', 'mpf_atan2'):
            request, wanted = scalar(name, values), 'MPF\t' + raw(result)
        elif name in ('mpi_atan', 'mpi_atan2'):
            request, wanted = interval(name, values), 'INTERVAL\t' + '\t'.join(map(raw, result))
        else:
            n, p = values
            request, wanted = f'cache\t{p}\t{n}', 'FIXED_PAIR\t' + '\t'.join(map(str, result))
        cases.append((request, wanted))
    names = ('mpf_atan', 'mpf_atan2', 'mpi_atan', 'mpi_atan2', 'atan_taylor_get_cached')
    for path in args.file:
        data = json.loads(lzma.decompress(path.read_bytes()))
        count = 0
        for name, values, keywords, result in data['records']:
            if name not in names:
                continue
            assert not keywords and result[0] == 'ok', (path, name, keywords, result)
            add(name, values, result[1])
            count += 1
        sources.append({'file': str(path), 'cases': count, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    for p in (8, 16, 32, 64, 128, 256, 512):
        for n in range(256):
            add('atan_taylor_get_cached', (n, p), ef.atan_taylor_get_cached(n, p))
    rng = random.Random(20261012)
    for i in range(args.random):
        p = rng.choice((2, 3, 7, 16, 53, 80, 160, 200, 256))
        values = [mp.from_man_exp(rng.randint(-2**30, 2**30), rng.randint(-90, 30)) for _ in range(4)]
        x, y = values[:2]
        r = 'fcdun'[i % 5]
        add('mpf_atan', (x, p, r), ef.mpf_atan(x, p, r))
        add('mpf_atan2', (y, x, p, r), ef.mpf_atan2(y, x, p, r))
        sort = lambda a, b: (a, b) if mp.mpf_le(a, b) else (b, a)
        xi, yi = sort(x, y), sort(*values[2:])
        add('mpi_atan', (xi, p), iv.mpi_atan(xi, p))
        add('mpi_atan2', (yi, xi, p), iv.mpi_atan2(yi, xi, p))
    special = (mp.fzero, mp.finf, mp.fninf, mp.fnan, mp.fone, mp.fnone,
               mp.from_man_exp(1, -1000), mp.from_man_exp(-1, -1000),
               mp.from_man_exp(1, 1000), mp.from_man_exp(-1, 1000))
    for p in (2, 53, 160, 350):
        for r in 'fcdun':
            for x in special:
                add('mpf_atan', (x, p, r), ef.mpf_atan(x, p, r))
                for y in special:
                    add('mpf_atan2', (y, x, p, r), ef.mpf_atan2(y, x, p, r))
        spans = ((mp.fzero, mp.fzero), (mp.fninf, mp.finf), (mp.fzero, mp.finf),
                 (mp.fninf, mp.fzero), (mp.fnone, mp.fone), (mp.fone, mp.fone))
        for xi in spans:
            add('mpi_atan', (xi, p), iv.mpi_atan(xi, p))
            for yi in spans:
                add('mpi_atan2', (yi, xi, p), iv.mpi_atan2(yi, xi, p))
    proc = subprocess.Popen([str(args.exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    args.out.with_suffix('.pid').write_text(str(proc.pid), encoding='utf-8')
    output, error = proc.communicate('\n'.join(request for request, _ in cases) + '\n')
    rows = output.splitlines()
    failures = [{'index': i, 'input': request, 'expected': wanted, 'actual': actual}
                for i, ((request, wanted), actual) in enumerate(zip(cases, rows)) if wanted != actual]
    report = {'sources': sources, 'cases': len(cases), 'rows': len(rows), 'pid': proc.pid,
              'exit_code': proc.returncode, 'failures': failures, 'stderr': error,
              'binary_sha256': hashlib.sha256(args.exe.read_bytes()).hexdigest()}
    args.out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'cases': len(cases), 'rows': len(rows), 'failures': len(failures), 'exit_code': proc.returncode}), flush=True)
    return bool(failures or proc.returncode or len(rows) != len(cases))

if __name__ == '__main__':
    raise SystemExit(main())
