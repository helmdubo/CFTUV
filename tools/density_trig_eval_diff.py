"""Actual MPF/MPI sin/cos: записанные вызовы плюс направленное случайное сравнение."""
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

def expected_pair(pair, interval=False):
    if interval:
        return 'INTERVAL_PAIR\t'+'\t'.join(raw(value) for item in pair for value in item)
    return 'PAIR\t'+'\t'.join(map(raw, pair))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, action='append', default=[])
    parser.add_argument('--exe', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--random', type=int, default=0)
    args=parser.parse_args()
    cases, sources=[],[]
    for path in args.file:
        data=json.loads(lzma.decompress(path.read_bytes()))
        count=0
        for name, values, keywords, result in data['records']:
            if name not in ('mpf_cos_sin','mpi_cos_sin','mod_pi2'):
                continue
            assert not keywords and result[0]=='ok', (path,name,keywords,result)
            if name=='mpf_cos_sin':
                assert len(values) in (2,3), values
                x,p=values[:2]
                rnd=values[2] if len(values)==3 else 'd'
                request=f'scalar\t{p}\t{rnd}\t{raw(x)}'
                wanted=expected_pair(result[1])
            elif name=='mpi_cos_sin':
                (a,b),p=values
                request=f'interval\t{p}\t{raw(a)}\t{raw(b)}'
                wanted=expected_pair(result[1],True)
            else:
                man,exp,mag,p=values
                request=f'reduce\t{p}\t{raw((0,man,exp,mag-exp))}'
                wanted='REDUCED\t'+'\t'.join(map(str,result[1]))
            cases.append((request,wanted))
            count+=1
        sources.append({'file':str(path),'cases':count,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    rng=random.Random(20261012)
    for i in range(args.random):
        p=rng.choice((2,3,7,16,53,80,160,200,256))
        x=mp.from_man_exp(rng.randint(-2**30,2**30),rng.randint(-90,-27))
        rnd='fcdun'[i%5]
        cases.append((f'scalar\t{p}\t{rnd}\t{raw(x)}',expected_pair(ef.mpf_cos_sin(x,p,rnd))))
        y=mp.from_man_exp(rng.randint(-2**30,2**30),rng.randint(-90,-27))
        a,b=(x,y) if mp.mpf_le(x,y) else (y,x)
        cases.append((f'interval\t{p}\t{raw(a)}\t{raw(b)}',expected_pair(iv.mpi_cos_sin((a,b),p),True)))
    for p in (2,53,160,350):
        for x in (mp.fzero,mp.finf,mp.fninf,mp.fnan,mp.from_man_exp(1,-1000),mp.from_man_exp(-1,-1000)):
            for rnd in 'fcdun':
                cases.append((f'scalar\t{p}\t{rnd}\t{raw(x)}',expected_pair(ef.mpf_cos_sin(x,p,rnd))))
        for pair in ((mp.fzero,mp.fzero),(mp.fninf,mp.finf),(mp.fzero,mp.finf)):
            cases.append((f'interval\t{p}\t{raw(pair[0])}\t{raw(pair[1])}',expected_pair(iv.mpi_cos_sin(pair,p),True)))
    proc=subprocess.Popen([str(args.exe)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    args.out.with_suffix('.pid').write_text(str(proc.pid),encoding='utf-8')
    output,error=proc.communicate('\n'.join(item[0] for item in cases)+'\n')
    rows=output.splitlines()
    failures=[{'index':i,'input':request,'expected':want,'actual':actual}
              for i,((request,want),actual) in enumerate(zip(cases,rows)) if want!=actual]
    report={'sources':sources,'cases':len(cases),'rows':len(rows),'pid':proc.pid,'exit_code':proc.returncode,
            'failures':failures,'stderr':error,'binary_sha256':hashlib.sha256(args.exe.read_bytes()).hexdigest()}
    args.out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'cases':len(cases),'rows':len(rows),'failures':len(failures),'exit_code':proc.returncode}),flush=True)
    return bool(failures or proc.returncode or len(rows)!=len(cases))

if __name__=='__main__':
    raise SystemExit(main())
