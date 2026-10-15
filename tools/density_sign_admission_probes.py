"""Контрпримеры sign admission: exact fallback и именованные старые отказы."""
import argparse
import json
import lzma
from math import isqrt
from pathlib import Path
import sys
import sympy as sp

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'kernel'/'src'));sys.path.insert(0,str(ROOT/'tools'))
sys.set_int_max_str_digits(0)
from native_density_wire import Wire, json_key
from cftuv_envelope.reference.density_sign import _density_exact_sign
from cftuv_envelope.reference.common import ReferenceGeometryError

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    d=1<<256;p=isqrt(2*d*d)
    nested=sp.sqrt(2+sp.sqrt(2));near=213955882546153770510803628816683935098316969411928952863250352738335112326797
    phi=sp.atan2(5*sp.sqrt(7)/16,-sp.Rational(9,16))/3
    sine=sp.sin(sp.Rational(365,8192));a=1041546121700086938441153703724603043422243247291
    d400=1<<400;floor_sum=isqrt(2*d400*d400)+isqrt(3*d400*d400)
    examples=[sp.Integer(0),sp.Integer(1),sp.Integer(-1),nested,-nested,
              nested-sp.Rational(near,d),nested-sp.Rational(near+1,d),
              sp.sqrt(2)-sp.Rational(p,d),sp.sqrt(2)-sp.Rational(p+1,d),
              sp.sqrt(2)+sp.sqrt(3)-sp.Rational(floor_sum,d400),
              sp.sqrt(2)+sp.sqrt(3)-sp.Rational(floor_sum+2,d400),
              sp.cos(phi)-sp.Rational(3,4),sp.cos(phi)-1+sp.sin(phi)/sp.sqrt(7),
              sine-sp.Rational(a,2**164),
              sp.Add(sine,sp.Mul(-1,sine,evaluate=False),evaluate=False),
              sp.Pow(sp.Pow(-1,sp.Rational(1,2),evaluate=False),0,evaluate=False),
              sp.tan(0,evaluate=False),
              (sine+1)*(sine-1)-sine**2+1,
              (sp.tan(1)+1)*(sp.tan(1)-1)-sp.tan(1)**2+1]
    wire=Wire();records=[]
    for expression in examples:
        try: outcome=['ok',_density_exact_sign(expression)]
        except ReferenceGeometryError as error: outcome=['raised',type(error).__name__,str(error)]
        records.append([wire.expression(expression),False,[],outcome])
    doc={'schema':'cftuv.density-sign-trace.v1','sources':['deliberate admission counterexamples'],
         'expressions':wire.nodes,'records':records,'occurrences':[1]*len(records),'mismatches':[]}
    args.out.write_bytes(lzma.compress(json_key(doc).encode('ascii')))
    print(json.dumps({'records':len(records),'refusals':sum(row[3][0]=='raised' for row in records)}),flush=True)

if __name__=='__main__':main()
