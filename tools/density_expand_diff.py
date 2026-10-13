"""Source expand inventory/AST/sign gate; unsupported отражается отдельно."""
import argparse
from collections import Counter
import hashlib
import json
import lzma
from pathlib import Path
import sys
import subprocess
import random

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'kernel'/'src'));sys.path.insert(0,str(ROOT/'tools'))
sys.set_int_max_str_digits(0)
import sympy as sp
from native_density_wire import Wire
from density_construct_diff import root_contract_counterexamples
from cftuv_envelope.reference import density_sign
from cftuv_envelope.reference.common import ReferenceGeometryError

def tree(value):
    if isinstance(value,sp.Rational):return ['Q',str(value.p),str(value.q)]
    if value is sp.pi:return ['pi']
    return [value.func.__name__,*[tree(arg) for arg in value.args]]

def actual_sign(value):
    fallback=[];quadratic=density_sign.exact_quadratic_value
    def capture(expression):
        fallback.append(expression);return quadratic(expression)
    density_sign.exact_quadratic_value=capture
    try:
        try:outcome='SIGN\t'+str(density_sign._density_exact_sign(value))
        except ReferenceGeometryError:outcome='UNDECIDABLE'
    finally:density_sign.exact_quadratic_value=quadratic
    assert len(fallback)<=1
    return outcome,bool(fallback)

def gate(args):
    program=[];expected=[];queries=[];sources=[]
    def load(wire,name,requests,sign_ids):
        program.append('clear');expected.append('CLEARED')
        for i,node in enumerate(wire.nodes):program.append('\t'.join(map(str,node)));expected.append(f'NODE\t{i}')
        roots=0
        for i,node in enumerate(wire.nodes):
            value=wire.unexpression(i)
            if node[0]=='Pow' and value.exp==sp.Rational(1,2) and value.base.is_Integer and value.base>1 and sp.sqrt(value.base)==value:
                program.append(f'source_sqrt\t{i}\t{value.base}');expected.append(f'DECLARED\t{i}');roots+=1
        for node in requests:
            value=wire.unexpression(node)
            # Adversarial raw polynomial проверяет отказ до huge expansion и в Python.
            profile_refusal=(value.is_Pow and value.exp.is_Integer and value.exp>64 and value.base.is_Add)
            expanded=None if profile_refusal else tree(sp.expand(value))
            queries.append({'row':len(program),'source':name,'op':'expand','node':node,'expected':expanded,
                            'required_refusal':'PolynomialExponentLimit' if profile_refusal else None})
            program.append(f'expand\t{node}');expected.append(None)
        fallbacks=0;old_refusals=0
        for node in sign_ids:
            outcome,fallback=actual_sign(wire.unexpression(node));fallbacks+=fallback;old_refusals+=outcome=='UNDECIDABLE'
            queries.append({'row':len(program),'source':name,'op':'sign','node':node,'expected':outcome,'source_used_fallback':fallback})
            program.append(f'sign\t{node}');expected.append(None)
        sources.append({'source':name,'nodes':len(wire.nodes),'expansions':len(requests),'signs':len(sign_ids),'normalized_root_assumptions':roots,
                        'actual_source_fallbacks':fallbacks,'actual_source_refusals':old_refusals,
                        'root_proof':'empirical caller assumption only; normalized and even squarefree do NOT prove generic source MulAST'})
    for path in args.file:
        doc=json.loads(lzma.decompress(path.read_bytes()));wire=Wire(doc['expressions'])
        load(wire,str(path),list(range(len(wire.nodes))),sorted(set(row[0] for row in doc['records'])))
    wire=Wire();values=[];rng=random.Random(20261014)
    atoms=[sp.pi,sp.sin(1),sp.cos(2),sp.sqrt(2),sp.sqrt(3),sp.sqrt(5),sp.sqrt(6),sp.tan(1)]
    for _ in range(150):
        a,b=rng.sample(atoms,2);degree=rng.randrange(2,5)
        values.append(sp.Pow(sp.Add(a,b,evaluate=False),degree,evaluate=False))
    t=sp.tan(1)
    poly=sp.Add(sp.Mul(sp.Add(t,1,evaluate=False),sp.Add(t,-1,evaluate=False),evaluate=False),sp.Mul(-1,sp.Pow(t,2,evaluate=False),evaluate=False),1,evaluate=False)
    values.extend([poly,t,sp.Add(1,1,evaluate=False),sp.Pow(2,2,evaluate=False),
                   sp.sin(sp.Mul(2,sp.Add(sp.pi,1,evaluate=False),evaluate=False),evaluate=False),
                   sp.Pow(sp.Add(sp.pi,1,evaluate=False),-1,evaluate=False),
                   sp.Pow(sp.Add(sp.pi,-sp.pi,evaluate=False),2**32-1,evaluate=False)])
    roots=[wire.expression(v) for v in values]
    load(wire,'synthetic-polynomial-and-negative',roots,roots[:-1])
    proc=subprocess.Popen([str(args.exe)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    args.out.with_suffix('.pid').write_text(str(proc.pid),encoding='utf-8')
    output,error=proc.communicate('\n'.join(program)+'\n');rows=output.splitlines();failures=[];coverage={};refused=Counter();examples={}
    for i,(want,actual) in enumerate(zip(expected,rows)):
        if want is not None and want!=actual:failures.append({'row':i,'expected':want,'actual':actual})
    for q in queries:
        result=rows[q['row']] if q['row']<len(rows) else ''
        stats=coverage.setdefault(q['source']+':'+q['op'],{'success':0,'unsupported':0,'mismatches':0})
        if result.startswith('ERROR\t'):
            stats['unsupported']+=1
            if 'UnsupportedProfile(' in result:
                reason=result.split('UnsupportedProfile(',1)[1].split('(',1)[0].split(' {',1)[0]
            else:
                reason='UnexpectedInternal';failures.append({**q,'actual':result})
            if q.get('required_refusal') and q['required_refusal']!=reason:failures.append({**q,'actual':result})
            refused[reason]+=1
            if len(examples.setdefault(reason,[]))<3:examples[reason].append({**q,'actual':result})
        else:
            actual=json.loads(result.split('\t',1)[1]) if result.startswith('TREE\t') else '\t'.join(result.split('\t')[:2]) if result.startswith('SIGN\t') else result.split('\t')[0]
            if q['op']=='sign':
                path=result.split('\t')[-1]
                stats[path]=stats.get(path,0)+1
            stats['success']+=1
            if actual!=q['expected']:stats['mismatches']+=1;failures.append({**q,'actual':actual})
    report={'schema':'cftuv.density-expand-diff.v1','sympy_version':sp.__version__,'sources':sources,'coverage':coverage,'unsupported':dict(refused),'examples':examples,
            'failures':failures,'pid':proc.pid,'exit_code':proc.returncode,'stderr':error,'row_count_matches':len(rows)==len(program),'binary_sha256':hashlib.sha256(args.exe.read_bytes()).hexdigest(),
            'known_root_compatibility_gaps':root_contract_counterexamples(),
            'scope':'explicit partial expand staging; root rewrites only fixed2/3/5/6-generated family; sign probes source first; UnsupportedProfile is internal only'}
    args.out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'coverage':coverage,'failures':len(failures),'exit_code':proc.returncode}),flush=True)
    return bool(failures or proc.returncode or len(rows)!=len(program))

def inventory(path):
    doc=json.loads(lzma.decompress(path.read_bytes()));wire=Wire(doc['expressions'])
    roots={i for i,node in enumerate(wire.nodes) if node[0]=='Pow' and
           wire.unexpression(i).exp==sp.Rational(1,2) and
           wire.unexpression(i).base.is_Integer and wire.unexpression(i).base>1 and
           sp.sqrt(wire.unexpression(i).base)==wire.unexpression(i)}
    expanded={};profiles={};reasons=Counter();examples={};powers=Counter()
    def expand(i):
        if i not in expanded:expanded[i]=sp.expand(wire.unexpression(i))
        return expanded[i]
    def profile(i):
        if i in profiles:return profiles[i]
        node=wire.nodes[i];kind=node[0]
        if kind in ('Q','pi') or i in roots:reason=None
        elif kind in ('Add','Mul'):
            reason=next((profile(c) for c in node[1:] if profile(c)),None)
        elif kind=='Pow':
            base,e=node[1:];exponent=wire.unexpression(e)
            powers[str(exponent)]+=1
            if not exponent.is_Integer or exponent<0:reason='NonPolynomialPower'
            else:reason=profile(base)
        elif kind in ('sin','cos','atan','atan2'):
            reason=next((profile(c) for c in node[1:] if profile(c)),None)
            if reason is None and any(expand(c)!=wire.unexpression(c) for c in node[1:]):reason='OpaqueChildChanges'
        else:reason='NodeOutsideProfile'
        profiles[i]=reason;return reason
    changed=0;max_args=0
    for i in range(len(wire.nodes)):
        value=expand(i);changed+=value!=wire.unexpression(i);max_args=max(max_args,len(value.args))
        reason=profile(i)
        if reason:
            reasons[reason]+=1
            if len(examples.setdefault(reason,[]))<3:
                examples[reason].append({'node':i,'source':sp.srepr(wire.unexpression(i)),'expanded':sp.srepr(value)})
    requested=[row[0] for row in doc.get('records',[])]
    request_coverage=Counter(profile(i) or 'Supported' for i in requested)
    return {'file':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'nodes':len(wire.nodes),'changed_nodes':changed,'largest_expanded_top_arity':max_args,
            'source_normalized_root_assumptions':len(roots),
            'root_evidence':'Empirical normalized AST only; NOT mathematical squarefree proof',
            'all_node_coverage':dict(Counter(profile(i) or 'Supported' for i in range(len(wire.nodes)))),
            'recorded_request_coverage':dict(request_coverage),'integer_power_shapes':dict(powers),'examples':examples}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file',type=Path,action='append',required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--exe',type=Path)
    args=parser.parse_args()
    if args.exe:
        raise SystemExit(gate(args))
    receipt={'schema':'cftuv.density-expand-inventory.v1','sympy_version':sp.__version__,
             'sources':[inventory(path) for path in args.file]}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'sources':[{k:s[k] for k in ('file','nodes','changed_nodes','all_node_coverage','recorded_request_coverage','integer_power_shapes')} for s in receipt['sources']]}),flush=True)

if __name__=='__main__':main()
