"""Fresh source sign, native ordered iv160 и явно записанный expanded fallback."""
import argparse
import hashlib
import json
import lzma
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'kernel'/'src'));sys.path.insert(0,str(ROOT/'tools'))
sys.set_int_max_str_digits(0)
from native_density_wire import Wire
from cftuv_envelope.reference import density_sign
from cftuv_envelope.reference.common import ReferenceGeometryError
from cftuv_envelope.reference.contracts import ReferenceOutcome

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file',type=Path,action='append',required=True)
    parser.add_argument('--exe',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    program,expected,sources=[],[],[]
    for path in args.file:
        doc=json.loads(lzma.decompress(path.read_bytes()));wire=Wire(doc['expressions'])
        requests=[];fallback=[];quadratic=density_sign.exact_quadratic_value
        def capture(expression):
            fallback.append(wire.expression(expression));return quadratic(expression)
        density_sign.exact_quadratic_value=capture
        changes=[];fallback_count=0;expanded_different=0;refusals=0
        try:
            for index,(node,cached,oldfallback,oldoutcome) in enumerate(doc['records']):
                fallback.clear()
                try:
                    sign=density_sign._density_exact_sign(wire.unexpression(node))
                    wanted=f'SIGN\t{sign}';outcome=['ok',sign]
                except ReferenceGeometryError as error:
                    assert error.outcome is ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE
                    wanted='UNDECIDABLE';outcome=['raised',type(error).__name__,str(error)];refusals+=1
                expanded=fallback[0] if fallback else node
                assert len(fallback)<=1
                fallback_count+=bool(fallback);expanded_different+=expanded!=node
                if outcome!=oldoutcome:changes.append({'record':index,'previous':oldoutcome,'fresh':outcome})
                requests.append((f'sign\t{node}\t{expanded}',wanted,{'file':str(path),'record':index,'fallback':bool(fallback)}))
                if (index+1)%2000==0: print(json.dumps({'file':str(path),'done':index+1,'fallback':fallback_count}),flush=True)
        finally:
            density_sign.exact_quadratic_value=quadratic
        program.append('clear');expected.append(('CLEARED',None))
        for index,node in enumerate(wire.nodes):
            program.append('\t'.join(map(str,node)));expected.append((f'NODE\t{index}',None))
        for request,wanted,location in requests:
            program.append(request);expected.append((wanted,location))
        sources.append({'file':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'records':len(requests),
                        'fallback_count':fallback_count,'expanded_different':expanded_different,'refusals':refusals,'fresh_vs_recorded_changes':changes})
    proc=subprocess.Popen([str(args.exe)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    args.out.with_suffix('.pid').write_text(str(proc.pid),encoding='utf-8')
    output,error=proc.communicate('\n'.join(program)+'\n');rows=output.splitlines()
    failures=[{'index':i,'input':program[i],'location':loc,'expected':want,'actual':actual}
              for i,((want,loc),actual) in enumerate(zip(expected,rows)) if want!=actual]
    report={'scope':'Controller receives source and Python-expanded fallback nodes; native normalization not covered',
            'sources':sources,'commands':len(program),'rows':len(rows),'failures':failures,'pid':proc.pid,
            'exit_code':proc.returncode,'stderr':error,'binary_sha256':hashlib.sha256(args.exe.read_bytes()).hexdigest()}
    args.out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'records':sum(s['records'] for s in sources),'failures':len(failures),'exit_code':proc.returncode}),flush=True)
    return bool(failures or proc.returncode or len(program)!=len(rows) or any(s['fresh_vs_recorded_changes'] for s in sources))

if __name__=='__main__':raise SystemExit(main())
