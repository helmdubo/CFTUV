"""Отдельный whole-AST V2 candidate против Rust; никаких legacy fallback/product calls."""
import argparse
from collections import Counter
from fractions import Fraction
import hashlib
import json
import lzma
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'kernel'/'src'));sys.path.insert(0,str(ROOT/'tools'))
sys.set_int_max_str_digits(0)
import sympy as sp
from native_density_wire import Wire
from cftuv_envelope.reference.density_shell_v2 import density_shell_v2,DensityShellV2NotApplicable,_shell
from cftuv_envelope.reference.planar_types import exact_quadratic_value
from cftuv_envelope.reference.radical_v2 import from_v1

def pair(value):return [str(value.numerator),str(value.denominator)]
def expectation(expression):
    try:result=density_shell_v2(expression)
    except ZeroDivisionError:return {'kind':'internal','code':'ZERO_DIVISION'}
    work=list(result.work.wire())
    if isinstance(result,DensityShellV2NotApplicable):
        return {'kind':'not_applicable','canon':result.canon,'code':result.code.value,'detail':result.detail,'work':work}
    rational=result.outcomes.rational
    return {'kind':'applied','canon':result.canon,'value':list(map(str,result.value.wire())),
            'bounds':[pair(v) for v in result.enclosure160],
            'units':list(map(str,(result.shell.lower_units,result.shell.upper_units,result.shell.upper_units-result.shell.lower_units))),
            'quantum':-27,'codes':[c.value for c in result.outcomes.codes],
            'rational':None if rational is None else {'numerator':str(rational.numerator),'denominator':str(rational.denominator),
                'grid_units':None if rational.grid_units is None else str(rational.grid_units)},
            'legacy_projected_min_width_units':result.outcomes.legacy_projected_min_width_units,'work':work}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file',type=Path,action='append',required=True)
    parser.add_argument('--exe',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();program=[];expected=[];locations=[];sources=[]
    def append(line,wanted,location=None):program.append(line);expected.append(wanted);locations.append(location)
    def load(wire,name,records):
        append('clear','CLEARED')
        for i,node in enumerate(wire.nodes):append('\t'.join(map(str,node)),f'NODE\t{i}')
        counts=Counter();calls=Counter()
        for index,node,occurrences in records:
            result=expectation(wire.unexpression(node));counts[result['kind']]+=1;calls[result['kind']]+=occurrences
            append(f'shell\t{node}',result,{'source':name,'record':index,'node':node,'occurrences':occurrences})
        sources.append({'source':name,'nodes':len(wire.nodes),'records':len(records),'counts':dict(counts),'calls':dict(calls)})
    for path in args.file:
        document=json.loads(lzma.decompress(path.read_bytes()))
        assert document['schema']=='cftuv.density-decimal-trace.v2' and not document['mismatches']
        load(Wire(document['expressions']),str(path),[(i,row[0],document['occurrences'][i]) for i,row in enumerate(document['records'])])
        sources[-1]['sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    p,q,r=sp.Integer(2)**89-1,sp.Integer(2)**61-1,sp.Integer(2)**31-1
    a,b=q*r,p*q
    half=sp.Rational(1,2)
    sqrt=lambda n:sp.Pow(n,half,evaluate=False)
    sum3=sp.Add(sqrt(2),sqrt(3),sqrt(5),evaluate=False)
    sum4=sp.Add(sqrt(2),sqrt(3),sqrt(5),sqrt(7),evaluate=False)
    witness=sp.Pow(sum3,-2,evaluate=False)
    values=[sp.Integer(0),sp.Integer(1),sp.Integer(-1),sp.Rational(1657,5120),sp.Rational(-1657,5120),sp.Rational(3647,5120),
            sp.Rational(1,3),sp.Rational(1,8),sp.Rational(1,10**27),sp.Rational(-1,10**27),sp.Integer(10)**100,
            sqrt(2),sp.Mul(half,sqrt(8),evaluate=False),sqrt(3),sp.Mul(half,sqrt(12),evaluate=False),
            sqrt(p*p*q),sp.Mul(p,sqrt(q),evaluate=False),sqrt(a*b),sp.Mul(q,sqrt(p*r),evaluate=False),
            sp.Mul(sqrt(a),sqrt(b),evaluate=False),sp.Pow(sp.Rational(9,16),sp.Rational(-3,2),evaluate=False),
            witness,sp.Pow(sp.Pow(sum3,2,evaluate=False),-1,evaluate=False),sum4,sp.Pow(sum4,-1,evaluate=False),sp.Pow(sum4,-2,evaluate=False),
            sp.pi,sp.sin(1),sp.cos(2),sp.tan(1),sp.atan(1),sp.atan2(2,3),
            sp.Pow(sp.Add(2,sqrt(2),evaluate=False),half,evaluate=False),sp.Mul(0,sp.pi,evaluate=False),
            sp.Pow(sp.Pow(sp.Add(2,sqrt(2),evaluate=False),half,evaluate=False),0,evaluate=False),
            sp.Pow(-1,half,evaluate=False),sp.Pow(0,-1,evaluate=False),sp.Pow(0,-half,evaluate=False)]
    wire=Wire();ids=[wire.expression(v) for v in values]
    load(wire,'synthetic-value-scope',[(i,node,1) for i,node in enumerate(ids)])
    # Отдельный V1 source witness проверяет actual inverse-base-first, не только новый candidate.
    value,_=from_v1(exact_quadratic_value(witness))
    append(f'exact\t{ids[values.index(witness)]}','RADICAL\t'+','.join(map(str,value.wire())),{'source':'source-v1-negative-power-witness'})
    for exponent in [1048577,2**70]:
        # База0 позволяет source candidate безопасно вычислить большую степень без огромного value.
        wire=Wire();id_=wire.expression(sp.Pow(0,exponent,evaluate=False));append('clear','CLEARED')
        for i,node in enumerate(wire.nodes):append('\t'.join(map(str,node)),f'NODE\t{i}')
        append(f'shell\t{id_}',{'kind':'internal','code':'EXPONENT_PROFILE','work':[0,0,0]}, {'source':'declared-native-exponent-profile','exponent':str(exponent)})
    append('shell\t18446744073709551615',{'kind':'internal','code':'INVALID_REFERENCE','work':[0,0,0]},{'source':'invalid-reference'})
    for lo,hi in [(Fraction(-10**100),Fraction(10**100)),(Fraction(-1,3),Fraction(-1,3)),(Fraction(1657,5120),Fraction(3647,5120))]:
        result=_shell((lo,hi));width=Fraction(result.absolute_error_bound)*10**27
        assert width.denominator==1
        line='\t'.join(['bounds',str(lo.numerator),str(lo.denominator),str(hi.numerator),str(hi.denominator)])
        append(line,'\t'.join(['UNITS',str(result.lower_units),str(result.upper_units),str(width.numerator),'-27']),{'source':'integer-projection-huge-width'})
    args.out.parent.mkdir(parents=True,exist_ok=True)
    proc=subprocess.Popen([str(args.exe)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    args.out.with_suffix('.pid').write_text(str(proc.pid),encoding='utf-8')
    stdout,stderr=proc.communicate('\n'.join(program)+'\n');rows=stdout.splitlines();failures=[];internal_cases=[]
    for i,(wanted,actual) in enumerate(zip(expected,rows)):
        if isinstance(wanted,dict):
            if actual.startswith('SHELL\t'):
                actual=json.loads(actual.split('\t',1)[1])
                # Python zero division has no diagnostic work on its exception.
                if wanted['kind']=='internal' and 'work' not in wanted:
                    internal_cases.append({'location':locations[i],'native_work':actual.get('work')});actual.pop('work',None)
            if actual!=wanted:failures.append({'row':i,'location':locations[i],'expected':wanted,'actual':actual})
        elif wanted!=actual:failures.append({'row':i,'location':locations[i],'expected':wanted,'actual':actual})
    report={'schema':'cftuv.density-shell-v2-native-diff.v1','sources':sources,'failures':failures,'internal_exception_work':internal_cases,
            'commands':len(program),'rows':len(rows),'pid':proc.pid,'exit_code':proc.returncode,'stderr':stderr,
            'binary_sha256':hashlib.sha256(args.exe.read_bytes()).hexdigest(),'python_source_sha256':hashlib.sha256((ROOT/'kernel/src/cftuv_envelope/reference/density_shell_v2.py').read_bytes()).hexdigest(),
            'scope':'Separate candidate whole-AST value/enclosure/units/quantum/outcomes/diagnostic work; no legacy replacement or DensityWork parity',
            'native_profile':'Reader integer numerator abs<=2^20 and i64; excess is explicit internal EXPONENT_PROFILE, not NotApplicable'}
    args.out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'sources':sources,'failures':len(failures),'rows':len(rows),'exit_code':proc.returncode}),flush=True)
    return bool(failures or proc.returncode or len(program)!=len(rows))
if __name__=='__main__':raise SystemExit(main())
