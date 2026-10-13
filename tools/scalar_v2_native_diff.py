"""Небольшой fresh-operation/literal/cached-state fixture gate; не source-AST parity."""
import argparse
from collections import Counter
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import random
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'kernel'/'src'));sys.set_int_max_str_digits(0)
from cftuv_envelope.reference.density_scalar_v2 import ScalarV2,DensityScalarProfileUnsupported

def record(value):
    literal=value.literal_ast()
    return {'canon':literal.canon,'v2_wire':list(value.value.wire()),
            'nodes':[list(row) for row in literal.nodes],'root':literal.root}
def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--exe',type=Path,required=True)
    parser.add_argument('--cache-witness',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();program=[];expected=[];registers=[];counts=Counter()
    def emit(command,operation):
        program.append(command)
        try:value=operation()
        except DensityScalarProfileUnsupported as error:
            expected.append(f'PROFILE\t{error.code.value}\t{error.detail}');counts['profile']+=1;return None
        except ZeroDivisionError:
            expected.append('INTERNAL\tZERO_DIVISION');counts['exact_zero']+=1;return None
        id_=len(registers);registers.append(value)
        expected.append(f'SCALAR\t{id_}\t'+json.dumps(record(value),separators=(',',':')));counts['fresh_numeric_operations']+=1
        return id_
    def q(n,d=1):return emit(f'q\t{n}\t{d}',lambda:ScalarV2.rational(Fraction(n,d)))
    def unary(op,id_,function):return emit(f'{op}\t{id_}',lambda:function(registers[id_]))
    def binary(op,a,b,function):return emit(f'{op}\t{a}\t{b}',lambda:function(registers[a],registers[b]))
    rng=random.Random(20261013);zero=q(0);one=q(1);roots=[]
    for n in [2,3,5,6,7,8,12,18,30]:roots.append(unary('sqrt',q(n),lambda s:s.sqrt()))
    for _ in range(80):
        a=q(rng.randrange(-50,51),rng.randrange(1,51));b=q(rng.randrange(1,100),rng.randrange(1,41))
        root=unary('sqrt',b,lambda s:s.sqrt());product=binary('mul',a,root,lambda a,b:a*b)
        unary('sqrt',product,lambda s:s.sqrt())
        binary('div',product,b,lambda a,b:a/b)
    scalar=binary('add',one,roots[0],lambda a,b:a+b)
    for exponent in [-7,-2,-1,0,1,2,9]:emit(f'pow\t{scalar}\t{exponent}',lambda exponent=exponent:registers[scalar]**exponent)
    three=binary('add',binary('add',roots[0],roots[1],lambda a,b:a+b),roots[2],lambda a,b:a+b)
    emit(f'pow\t{three}\t-2',lambda:registers[three]**-2)
    four=binary('add',three,roots[4],lambda a,b:a+b);emit(f'pow\t{four}\t-2',lambda:registers[four]**-2)
    unary('inverse',zero,lambda s:s.inverse());unary('sqrt',q(-1),lambda s:s.sqrt());unary('sqrt',roots[0],lambda s:s.sqrt())
    unary('neg',roots[0],lambda s:-s)
    scaled=emit(f'scaled\t{roots[0]}\t3\t2',lambda:registers[roots[0]].scaled(Fraction(3,2)))
    binary('sub',scaled,scaled,lambda a,b:a-b)
    for exponent in [1048577,-9223372036854775808]:
        program.append(f'pow\t{zero}\t{exponent}');expected.append('INTERNAL\tEXPONENT_PROFILE');counts['explicit_native_exponent_profile']+=1
    cache=json.loads(args.cache_witness.read_text())
    assert cache['candidate_value_and_literal_identical'] and len(cache['states'])==2
    for state in cache['states']:
        n=int(state['candidate']['v2_wire'][0]);root=unary('sqrt',q(n),lambda s:s.sqrt())
        assert record(registers[root])==state['candidate'];counts['historical_cache_state_candidates']+=1
    args.out.parent.mkdir(parents=True,exist_ok=True)
    proc=subprocess.Popen([str(args.exe)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    args.out.with_suffix('.pid').write_text(str(proc.pid),encoding='utf-8')
    stdout,stderr=proc.communicate('\n'.join(program)+'\n');rows=stdout.splitlines()
    failures=[{'row':i,'command':program[i],'expected':want,'actual':actual}
              for i,(want,actual) in enumerate(zip(expected,rows)) if want!=actual]
    report={'schema':'cftuv.scalar-v2-native-synthetic-diff.v1','counts':dict(counts),'failures':failures,
            'commands':len(program),'rows':len(rows),'pid':proc.pid,'exit_code':proc.returncode,'stderr':stderr,
            'binary_sha256':hashlib.sha256(args.exe.read_bytes()).hexdigest(),
            'cache_witness_sha256':hashlib.sha256(args.cache_witness.read_bytes()).hexdigest(),
            'scope':'Exact V2 values and deterministic literal JSON bytes; historical cold/warm old SymPy shapes are NOT equality expectations or product admission'}
    args.out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'counts':dict(counts),'failures':len(failures),'rows':len(rows),'exit_code':proc.returncode}),flush=True)
    return bool(failures or proc.returncode or len(program)!=len(rows))
if __name__=='__main__':raise SystemExit(main())
