"""Rust integer projection: recorded authority strings плюс широкие Decimal widths."""
import argparse
from decimal import Decimal, localcontext, ROUND_HALF_EVEN
import hashlib
import json
import lzma
from pathlib import Path
import random
import subprocess
import sys
from mpmath.libmp import libmpf as mp
from density_decimal_projection import mpf_fraction

sys.set_int_max_str_digits(0)

def raw(value): return ','.join(map(str, value))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file',type=Path,action='append',required=True)
    parser.add_argument('--exe',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    cases,sources=[],[]
    for path in args.file:
        doc=json.loads(lzma.decompress(path.read_bytes()))
        count=0
        for _,outcome,intervals in doc['records']:
            assert outcome[0]=='ok' and len(intervals)==1
            a,b=intervals[0]
            cases.append((f'decimal\t28\t{raw(a)}\t{raw(b)}','DECIMAL\t'+'\t'.join(outcome[1:])))
            count+=1
        sources.append({'file':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'cases':count})
    rng=random.Random(20261012)
    rounded=0
    for i in range(5000):
        p=(1,2,3,7,28,50,100)[i%7]
        a=mp.from_man_exp(rng.randrange(-2**160,2**160),rng.randrange(-250,250))
        b=a if i%10==0 else mp.from_man_exp(rng.randrange(-2**160,2**160),rng.randrange(-250,250))
        if mp.mpf_gt(a,b): a,b=b,a
        low,high=mpf_fraction(a)*10**27,mpf_fraction(b)*10**27
        lo=low.numerator//low.denominator; hi=-((-high.numerator)//high.denominator)
        left,right=Decimal(f'{lo}E-27'),Decimal(f'{hi}E-27')
        with localcontext() as ctx:
            ctx.prec=p;ctx.rounding=ROUND_HALF_EVEN
            width=right-left
        exact=Decimal(f'{hi-lo}E-27')
        rounded+=width!=exact
        cases.append((f'decimal\t{p}\t{raw(a)}\t{raw(b)}',f'DECIMAL\t{left}\t{right}\t{width}'))
    proc=subprocess.Popen([str(args.exe)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    args.out.with_suffix('.pid').write_text(str(proc.pid),encoding='utf-8')
    output,error=proc.communicate('\n'.join(request for request,_ in cases)+'\n')
    rows=output.splitlines()
    failures=[{'index':i,'input':request,'expected':wanted,'actual':actual}
              for i,((request,wanted),actual) in enumerate(zip(cases,rows)) if wanted!=actual]
    report={'sources':sources,'cases':len(cases),'rows':len(rows),'synthetic_rounded_widths':rounded,
            'failures':failures,'exit_code':proc.returncode,'stderr':error,'pid':proc.pid,
            'binary_sha256':hashlib.sha256(args.exe.read_bytes()).hexdigest(),
            'scope':'Original legacy MPI bounds and quantum; explicit ROUND_HALF_EVEN width precision; no canon change'}
    args.out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'cases':len(cases),'rows':len(rows),'failures':len(failures),'rounded_widths':rounded,'exit_code':proc.returncode}),flush=True)
    return bool(failures or proc.returncode or len(cases)!=len(rows))

if __name__=='__main__': raise SystemExit(main())
