"""Source Gram/dual dot и cross: фактический AST после native construction."""
import argparse
import hashlib
import json
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'kernel' / 'src'))
sys.path.insert(0, str(ROOT / 'tools'))
import sympy as sp
from cftuv_envelope.reference.density_projective import _dual_dot, _oriented_cross
from cftuv_envelope.reference.adaptive_density_fan import _objective
from native_density_wire import ExpressionVector, Wire
from density_q_trig_diff import tree


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exe', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    rng = random.Random(0xD0743)
    wire = Wire()
    cases = []

    def scalar():
        rational = sp.Rational(rng.randrange(-7, 8), rng.randrange(1, 8))
        root = sp.sqrt(rng.choice((2, 3, 5, 6)))
        trig = sp.sin(sp.Rational(rng.randrange(1, 12), 17))
        return rng.choice((rational, root, trig, rational + root, rational * trig, root * trig))

    for index in range(600):
        matrix = [[sp.Rational(rng.randrange(-4, 5), rng.randrange(1, 8)) for _ in range(2)] for _ in range(2)]
        matrix[1][0] = matrix[0][1]
        left = ExpressionVector(scalar(), scalar())
        right = ExpressionVector(scalar(), scalar())
        orientation = rng.choice((-1, 1))
        metric = SimpleNamespace(inverse_gram=matrix, owner_orientation_sign=orientation,
                                 density_expressions=lambda vector: vector.expressions(),
                                 _density_exact_memo=SimpleNamespace(dual_dots={}, crosses={}))
        roots = [wire.expression(value) for value in (left.x, left.y, right.x, right.y)]
        matrix_nodes = [wire.expression(value) for row in matrix for value in row]
        cases.append(('vector_dot\t' + '\t'.join(map(str, matrix_nodes + roots)), 'VECTOR_AST\t' + tree(_dual_dot(metric, left, right))))
        cases.append(('vector_cross\t' + '\t'.join(map(str, roots + [orientation])), 'VECTOR_AST\t' + tree(_oriented_cross(metric, left, right))))
        covector = [matrix[j][0] * left.x + matrix[j][1] * left.y for j in range(2)]
        cases.append(('matrix_vector\t' + '\t'.join(map(str, matrix_nodes + roots[:2])), 'VECTOR_AST\t' + '\t'.join(map(tree,covector))))
        # Отдельный admitted objective: ненулевой integer candidate, positive Gram.
        matrix = [[sp.S(2), sp.S.One], [sp.S.One, sp.S(3)]]
        metric.inverse_gram = matrix
        metric._density_exact_memo.dual_dots.clear()
        matrix_nodes = [wire.expression(value) for row in matrix for value in row]
        candidate = (rng.randrange(1, 9), rng.randrange(-9, 10))
        cases.append(('vector_objective\t' + '\t'.join(map(str, matrix_nodes + roots[:2] + list(candidate))),
                      'VECTOR_AST\t' + tree(_objective(metric, (left,), 0, candidate))))
    program = ['clear'] + ['\t'.join(map(str, node)) for node in wire.nodes] + [a for a, _ in cases]
    expected = ['CLEARED'] + [f'NODE\t{i}' for i in range(len(wire.nodes))] + [b for _, b in cases]
    proc = subprocess.Popen([str(args.exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    args.out.with_suffix('.pid').write_text(str(proc.pid), encoding='utf-8')
    output, error = proc.communicate('\n'.join(program) + '\n')
    rows = output.splitlines()
    failures = [{'command': program[i], 'expected': a, 'actual': b} for i, (a, b) in enumerate(zip(expected, rows)) if a != b]
    report = {'scope': 'Actual source cache-miss expression trees; fixed-root provenance2,3,5,6; no full fan',
              'cases': len(cases), 'nodes': len(wire.nodes), 'commands': len(program), 'rows': len(rows),
              'failures': failures, 'exit_code': proc.returncode, 'stderr': error, 'pid': proc.pid,
              'binary_sha256': hashlib.sha256(args.exe.read_bytes()).hexdigest()}
    args.out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'cases': len(cases), 'failures': len(failures), 'exit_code': proc.returncode}), flush=True)
    return bool(failures or proc.returncode or len(program) != len(rows))


if __name__ == '__main__':
    raise SystemExit(main())
