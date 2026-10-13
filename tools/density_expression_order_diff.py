"""Native Density source-order против реального SymPy Basic.compare на корпусе."""
import argparse
import hashlib
import json
import lzma
from pathlib import Path
import random
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'kernel'/'src'));sys.path.insert(0,str(ROOT/'tools'))
sys.set_int_max_str_digits(0)
from native_density_wire import Wire

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file',action='append',type=Path,required=True)
    parser.add_argument('--exe',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();program=[];expected=[];sources=[];rng=random.Random(20261012)
    for path in args.file:
        doc=json.loads(lzma.decompress(path.read_bytes()));wire=Wire(doc['expressions'])
        program.append('clear');expected.append('CLEARED')
        for i,node in enumerate(wire.nodes):
            program.append('\t'.join(map(str,node)));expected.append(f'NODE\t{i}')
        pairs=[(rng.randrange(len(wire.nodes)),rng.randrange(len(wire.nodes))) for _ in range(10000)]
        for node in wire.nodes:
            if node[0] in ('Add','Mul'):
                pairs.extend(zip(node[1:],node[2:]))
        for a,b in pairs:
            program.append(f'compare\t{a}\t{b}');expected.append(f'CMP\t{wire.unexpression(a).compare(wire.unexpression(b))}')
        sources.append({'file':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'comparisons':len(pairs)})
    proc=subprocess.Popen([str(args.exe)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    args.out.with_suffix('.pid').write_text(str(proc.pid),encoding='utf-8')
    output,error=proc.communicate('\n'.join(program)+'\n');rows=output.splitlines()
    failures=[{'index':i,'input':program[i],'expected':want,'actual':actual} for i,(want,actual) in enumerate(zip(expected,rows)) if want!=actual]
    args.out.write_text(json.dumps({'sources':sources,'rows':len(rows),'commands':len(program),'failures':failures,
        'stderr':error,'exit_code':proc.returncode,'pid':proc.pid,'binary_sha256':hashlib.sha256(args.exe.read_bytes()).hexdigest()},indent=2),encoding='utf-8')
    print(json.dumps({'comparisons':sum(s['comparisons'] for s in sources),'failures':len(failures),'exit_code':proc.returncode}),flush=True)
    return bool(failures or proc.returncode or len(rows)!=len(program))

if __name__=='__main__':raise SystemExit(main())
