"""Рекурсивная структура native Add/Mul против actual SymPy; отказ — отдельный счётчик."""
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
from native_density_wire import Wire


def tree(value):
    if isinstance(value, sp.Rational):
        return ['Q', str(value.p), str(value.q)]
    if value is sp.pi:
        return ['pi']
    return [value.func.__name__, *map(tree, value.args)]


def root_contract_counterexamples():
    # Это source regression evidence известных staging gaps, НЕ допуск generic inputs.
    p,q,r=sp.Integer(2)**89-1,sp.Integer(2)**61-1,sp.Integer(2)**31-1
    hidden=p*p*q
    a,b=q*r,p*q
    product=sp.sqrt(a)*sp.sqrt(b)
    gcd_value=sp.Mul(q,sp.Pow(p*r,sp.Rational(1,2),evaluate=False),evaluate=False)
    raw_hidden=sp.Pow(hidden,sp.Rational(1,2),evaluate=False)
    bare_product=sp.Pow(a*b,sp.Rational(1,2),evaluate=False)
    assert sp.sqrt(hidden)==raw_hidden and hidden%(p*p)==0
    assert product==bare_product and product!=gcd_value
    return {'normalized_root_not_squarefree':{'radicand':str(hidden),'known_square_divisor':str(p*p),'actual':tree(sp.sqrt(hidden))},
            'squarefree_gcd_not_source_mul_ast':{'operands':[str(a),str(b)],'actual_source':tree(product),'native_gcd_value_ast':tree(gcd_value)},
            'conclusion':'canonical value closure only; generic source AST closure is disproved even for squarefree operands'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', action='append', type=Path, default=[])
    parser.add_argument('--exe', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--limit', type=int, default=1000)
    parser.add_argument('--mode',choices=('raw','certified','both'),default='both')
    args = parser.parse_args()
    program, expected, queries, sources = [], [], [], []
    rng = random.Random(20261013)

    def append_source(wire, operations, name, certified=False):
        program.append('clear'); expected.append('CLEARED')
        for i, node in enumerate(wire.nodes):
            program.append('\t'.join(map(str, node))); expected.append(f'NODE\t{i}')
        declarations=0
        if certified:
            for i,node in enumerate(wire.nodes):
                value=wire.unexpression(i)
                if (node[0]=='Pow' and value.exp==sp.Rational(1,2) and
                    value.base.is_Integer and value.base>1 and sp.sqrt(value.base)==value):
                    program.append(f'source_sqrt\t{i}\t{value.base}')
                    expected.append(f'DECLARED\t{i}');declarations+=1
        sources.append({'source':name,'canonical_root_declarations':declarations,
                        'provenance':'actual SymPy-normalized source root in recorder only' if certified else 'raw no root provenance'})
        for operation, ids in operations:
            operands = [wire.unexpression(i) for i in ids]
            actual = (sp.Add if operation == 'add' else sp.Mul)(*operands)
            command = '\t'.join([operation, *map(str, ids)])
            queries.append({'row': len(program), 'source': name, 'command': command,
                            'operands': [sp.srepr(v) for v in operands], 'expected': tree(actual)})
            program.append(command); expected.append(None)

    for path in args.file:
        document = json.loads(lzma.decompress(path.read_bytes()))
        wire = Wire(document['expressions'])
        operations = [(node[0].lower(), node[1:]) for node in wire.nodes if node[0] in ('Add', 'Mul')]
        if args.limit:
            operations=operations[:args.limit]
        pool = [i for i, node in enumerate(wire.nodes) if node[0] in ('Q', 'pi', 'sin', 'cos', 'atan', 'atan2', 'Pow')][:100]
        for _ in range(min(args.limit or 200, 200)):
            ids = [rng.choice(pool), rng.choice(pool)]
            operations.extend([('add', ids), ('mul', ids)])
        if args.mode in ('raw','both'):
            append_source(wire, operations, str(path)+':raw')
        if args.mode in ('certified','both'):
            append_source(wire, operations, str(path)+':declared-source-roots',certified=True)
        sources.append({'file': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                        'operations': len(operations)})

    wire = Wire()
    seeds = [sp.Rational(0), sp.Rational(1), sp.Rational(-1), sp.Rational(1,2), sp.Rational(2,3),
             sp.pi, sp.sin(1), sp.cos(2), sp.atan(3), sp.atan2(2,3), sp.sqrt(2), sp.sqrt(3)]
    operations = []
    for _ in range(400):
        operands = rng.choices(seeds[:40], k=rng.randrange(0,5))
        operation = rng.choice(('add','mul'))
        operations.append((operation, [wire.expression(x) for x in operands]))
        result = (sp.Add if operation == 'add' else sp.Mul)(*operands)
        if len(sp.srepr(result)) < 500 and result not in seeds:
            seeds.append(result)
    a, b = sp.pi, sp.sin(1)
    sum_ = sp.Add(a,b)
    # Источник two-arg hack и точные cancellations, включая canonical numeric radicals.
    extra = [('mul',[sp.Rational(2),sum_]),
             ('mul',[sp.Rational(2),sp.Mul(3,sum_,evaluate=False)]),
             ('add',[sp.sqrt(2),-sp.sqrt(2)]),
             ('mul',[sp.sin(1)**2,sp.sin(1)**-2]),
             ('add',[sp.Rational(1,2)*sp.pi,sp.Rational(2,3)*sp.pi]),
             ('mul',[sp.pi,sp.pi,sp.pi])]
    operations.extend((op,[wire.expression(x) for x in values]) for op,values in extra)
    raw8=sp.Pow(8,sp.Rational(1,2),evaluate=False)
    raw12=sp.Pow(12,sp.Rational(1,2),evaluate=False)
    nested=sp.Pow(sp.sqrt(2),sp.Rational(1,2),evaluate=False)
    operations.extend((op,[wire.expression(x) for x in values]) for op,values in [
        ('mul',[sp.Integer(2),raw8]),('mul',[sp.Integer(3),raw12]),('mul',[sp.Integer(2),nested]),
        ('add',[raw8,raw8]),('add',[raw12,raw12])])
    if args.mode in ('raw','both'):
        append_source(wire, operations, 'synthetic-closure:raw')
    if args.mode in ('certified','both'):
        append_source(wire, operations, 'synthetic-closure:declared-source-roots',certified=True)

    proc = subprocess.Popen([str(args.exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.with_suffix('.pid').write_text(str(proc.pid), encoding='utf-8')
    stdout, stderr = proc.communicate('\n'.join(program)+'\n')
    rows = stdout.splitlines()
    failures = []
    for i,(want,actual) in enumerate(zip(expected,rows)):
        if want is not None and actual != want:
            failures.append({'row':i,'command':program[i],'expected':want,'actual':actual})
    refused = Counter(); coverage = {}; examples = []
    for query in queries:
        actual = rows[query['row']] if query['row'] < len(rows) else ''
        count = coverage.setdefault(query['source'], {'success':0,'refused':0,'mismatches':0})
        if actual.startswith('ERROR\tUnsupportedProfile('):
            reason = actual.split('UnsupportedProfile(',1)[1].split('(',1)[0]
            refused[reason] += 1; count['refused'] += 1
            if len(examples) < 3 and query['source'] != 'synthetic-closure':
                examples.append({**query,'actual':actual})
        elif actual.startswith('TREE\t'):
            value = json.loads(actual.split('\t',1)[1])
            count['success'] += 1
            if value != query['expected']:
                count['mismatches'] += 1
                failures.append({**query,'actual':value})
        else:
            failures.append({**query,'actual':actual})
    receipt = {'schema':'cftuv.density-construct-diff.v1','sympy_version':sp.__version__,
               'root_contract_evidence':'Recorder normalized source AST is an EMPIRICAL assumption; even true squarefree does NOT prove generic source MulAST',
               'known_root_compatibility_gaps':root_contract_counterexamples(),
               'sources':sources,'coverage':coverage,'refusal_reasons':dict(refused),
               'field_refusal_examples':examples,'queries':len(queries),'failures':failures,
               'exit_code':proc.returncode,'pid':proc.pid,'stderr':stderr,
               'binary_sha256':hashlib.sha256(args.exe.read_bytes()).hexdigest(),
               'row_count_matches':len(rows)==len(program)}
    args.out.write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'queries':len(queries),'coverage':coverage,'refusals':dict(refused),
                      'failures':len(failures),'exit_code':proc.returncode}),flush=True)
    return bool(failures or proc.returncode or len(rows)!=len(program))


if __name__ == '__main__':
    raise SystemExit(main())
