"""Точный real-arithmetic корень UV-fit: полином над двумя радикалами + рациональный изолятор.

Это не сертификат binary64-предиката продукта. Его отдельные значения есть в independent_focus.json.
SymPy нужен только этому исследовательскому скрипту, не продукту.
"""
import hashlib
import json
import math
from fractions import Fraction as F
from pathlib import Path
import sympy as sp
from sympy.polys.rings import ring

INPUT = Path('D:/lazydrag-scratch/out/independent_focus.json')
OUTPUT = Path('D:/lazydrag-scratch/out/exact_event.json')
data = json.loads(INPUT.read_text(encoding='utf-8'))
radicands = sorted({q for part in data['coefficients'].values() for pt in part for axis in pt
                    for name in ('intercept', 'slope') for q, _ in axis[name] if q != 1})
K = sp.QQ.algebraic_field(*(sp.sqrt(q) for q in radicands))
R, w = ring('w', K)


def const(terms):
    return R(K.from_sympy(sum((sp.Rational(c) * sp.sqrt(q) for q, c in terms), sp.Integer(0))))


def poly(axis):
    return const(axis['intercept']) + w * const(axis['slope'])


points = [[poly(v) for v in p] for p in data['coefficients']['points']]
facts = [[poly(v) for v in p] for p in data['coefficients']['facts']]
n = len(points)
centre = [sum(p[j] for p in points) / n for j in (0, 1)]
xs = [p[0] - centre[0] for p in points]
ys = [p[1] - centre[1] for p in points]
sxx, sxy, syy = sum(x*x for x in xs), sum(x*y for x,y in zip(xs,ys)), sum(y*y for y in ys)
det = sxx*syy - sxy*sxy
res = []
for j in (0, 1):
    mean = sum(f[j] for f in facts) / n
    fs = [f[j] - mean for f in facts]
    rx, ry = sum(x*f for x,f in zip(xs,fs)), sum(y*f for y,f in zip(ys,fs))
    nx, ny = rx*syy-ry*sxy, sxx*ry-sxy*rx
    res.append([f*det-nx*x-ny*y for f,x,y in zip(fs,xs,ys)])
scale = sp.Rational(data['lattice_scale'])
denom = det*w*int(scale)
polys = [a*a+b*b-denom*denom/(256**2) for a,b in zip(*res)]


def pair_add(a,b):
    return a[0]+b[0], a[1]+b[1]


def pair_mul(a,b):
    v = [x*y for x in a for y in b]
    return min(v),max(v)


BITS = 320
def exact_interval(expr):
    if expr.is_Rational:
        v = F(int(expr.p),int(expr.q))
        return v,v
    if expr.is_Add:
        out = F(0),F(0)
        for term in expr.args:
            out = pair_add(out,exact_interval(term))
        return out
    if expr.is_Mul:
        out = F(1),F(1)
        for term in expr.args:
            out = pair_mul(out,exact_interval(term))
        return out
    if expr.is_Pow and expr.exp == sp.Rational(1,2) and expr.base.is_Integer and expr.base > 0:
        q = int(expr.base)
        low = math.isqrt(q << (2*BITS))
        return F(low,2**BITS),F(low+1,2**BITS)
    raise ValueError(str(expr))


def coeffs(p):
    return [K.to_sympy(p.get((i,),K.zero)) for i in range(p.degree()+1)]


def interval_coeffs(p):
    return [exact_interval(c) for c in coeffs(p)]


def horner(cs,lo,hi):
    out = F(0),F(0)
    for coefficient in reversed(cs):
        out = pair_add(pair_mul(out,(lo,hi)),coefficient)
    return out


def sign(cs,x):
    lo,hi = horner(cs,x,x)
    if lo > 0: return 1
    if hi < 0: return -1
    raise ValueError('SIGN_NOT_SEPARATED')


all_coeffs = [interval_coeffs(p) for p in polys]
lo,hi = F('0.0485189140073'),F('0.0485189140074')
active = [i for i,cs in enumerate(all_coeffs) if sign(cs,lo) != sign(cs,hi)]
assert len(active) == 1, active
i = active[0]
cs = all_coeffs[i]
for _ in range(120):
    mid = (lo+hi)/2
    if sign(cs,lo) == sign(cs,mid): lo=mid
    else: hi=mid
assert sign(cs,lo) == 1 and sign(cs,hi) == -1
derivative = horner(interval_coeffs(polys[i].diff(w)),lo,hi)
assert derivative[1] < 0, derivative
det_range = horner(interval_coeffs(det),lo,hi)
assert det_range[0] > 0
inactive = [horner(other,lo,hi) for j,other in enumerate(all_coeffs) if j != i]
assert all(v[1] < 0 for v in inactive)

z = sp.Symbol('w')
samples = {}
for txt in ('0.04214220866560936','0.048518914007','0.048518914008'):
    # Цифры только для чтения; доказательство выше использует дробные интервалы.
    val = sp.Rational(txt)
    d = sum(c*val**j for j,c in enumerate(coeffs(denom)))
    all_res = [sp.N((sum(c*val**j for j,c in enumerate(coeffs(p))))/(d*d),25) for p in polys]
    samples[txt] = [str(v) for v in all_res]

out = {
    'input_sha256':hashlib.sha256(INPUT.read_bytes()).hexdigest(),
    'radicands':radicands,'coefficient_field_degree':K.mod.degree(),
    'polynomial_definition':'P_i=R_ui^2+R_vi^2-(65536*w*D/256)^2; D=Sxx*Syy-Sxy^2; R_ji=Fc_ji*D-Nx_j*xc_i-Ny_j*yc_i',
    'active_vertex_index':i,'active_vertex':data['union'][i],
    'polynomial_degree':polys[i].degree(),
    'polynomial_coefficients_low_to_high':[str(c) for c in coeffs(polys[i])],
    'isolating_rationals':[str(lo),str(hi)],
    'root_decimal':str(sp.N(sp.Rational((lo+hi).numerator,(lo+hi).denominator)/2,45)),
    'exact_checks':{'left_sign':1,'right_sign':-1,'derivative_negative_on_isolator':True,
                    'determinant_positive_on_isolator':True,'other_vertex_margins_negative_on_isolator':True},
    'margin_real_samples':samples,
    'scope':'Exact isolated real-arithmetic LSQ root conditional on the recorded affine operands; NOT an exact binary64 switching threshold, NOT a proof of affine laws over the whole interval.',
    'field_note':'Two independent radicals, field degree 4. No claim that this root belongs to Q or a quadratic field; general algebraic representation required unless an additional reduction is proved.'
}
OUTPUT.write_text(json.dumps(out,indent=2),encoding='utf-8', newline='\n')
print('EXACT_EVENT_OK',out['root_decimal'],'degree',out['polynomial_degree'],'field',out['coefficient_field_degree'])
