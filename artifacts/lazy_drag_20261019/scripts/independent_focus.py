"""Узкий повтор 1146c49e: без cold press, без записи blend, точные операнды.

Запуск только Blender -b frozen_scene --python-exit-code 1 --python this.py.
Артефакты предыдущего capture читаются как локальные доверенные данные владельца.
"""
import hashlib
import json
import pickle
import shutil
import sys
from fractions import Fraction as F
from pathlib import Path

sys.dont_write_bytecode = True
ROOT = Path('D:/CFTUV-lazydrag')
OUT = Path('D:/lazydrag-scratch/out/independent_focus.json')
FINE = '--fine' in sys.argv
if FINE:
    OUT = Path('D:/lazydrag-scratch/out/independent_focus_fine.json')
PICKLE = Path('D:/lazydrag-scratch/cap_cover008/00624_1146c49e3661.pkl')
SCENE = Path('C:/Users/helmd/AppData/Local/Temp/cftuv_cover008_frozen_v2/buildings2_2.blend')
EXPECTED = 'a044b5aa8d5156bcf64167d7181557fb563b2d1472d8f837b4c6a51ce414ab32'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_space():
    for drive in ('C:/', 'D:/', 'E:/', 'F:/', 'G:/', 'H:/'):
        if Path(drive).exists():
            assert shutil.disk_usage(drive).free >= 1.5 * 1024**3, 'DISK_LOW:' + drive


def encode(value):
    return [[int(rad), str(coef)] for rad, coef in value.terms]


check_space()
assert digest(SCENE) == EXPECTED, 'SCENE_HASH_MISMATCH'
sys.path.insert(0, str(ROOT / 'tools'))
import blender_tree_ab as T
T.load_tree(ROOT, 'D:/lazydrag-scratch/site')
native = T.verify_native('D:/lazydrag-scratch/site', 'a3efcde8')
import cftuv_envelope.materialize.domain as D
import cftuv_envelope.materialize.silhouette as S
from cftuv_envelope import wavefront
from cftuv_envelope.float_filter import centre_and_bound

with PICKLE.open('rb') as stream:
    blob = pickle.load(stream)
prepared, kw = blob['prepared'], dict(blob['kw'])
real_merge, real_dissolve, real_apply = S._Mesh._try_merge, S._Mesh._try_dissolve, D.apply_silhouette
rows, grabs = {}, {}
current, captured = {}, {}


def apply(source, budget):
    current['before_silhouette'] = {
        'faces': sum(len(p) for p in source.polygons),
        'vertices': len({v for p in source.polygons for r in p for v in r}),
        'rings': sorted(tuple(r) for p in source.polygons for r in p),
    }
    return real_apply(source, budget)


def merge(self, first, second):
    if first != 'node:4' or not second.endswith(':2265'):
        return real_merge(self, first, second)
    one, two = self.faces[self.half[first, second]], self.faces[self.half[second, first]]
    ring = self._union(one, two, first, second)
    points = [self.source.points[v] for v in ring]
    facts = [self.source.facts[one.region, v] for v in ring]
    charts = [tuple(centre_and_bound(x)[0] for x in p) for p in points]
    uvs = [self.uv(one.region, v) for v in ring]
    residual = S.uv_fit_residual(charts, uvs)
    current['merge'] = {
        'edge': [first, second], 'ring': list(ring),
        'residual_hex': residual.hex(), 'residual': residual,
        'tau': str(self.source.uv_slide), 'accepted_by_uv': S._within(residual, self.source.uv_slide),
        'chord_m': self._merge_depth(one, two, ring),
        'affine_uv_in_chart': bool(self._affine_union(one, two)),
    }
    captured.update(points=points, facts=facts, lattice=F(self.source.lattice_alpha), ring=list(ring))
    before = dict(self.tally)
    result = real_merge(self, first, second)
    current['merge']['tally_delta'] = {k: v - before.get(k, 0) for k, v in self.tally.items() if v != before.get(k, 0)}
    return result


def dissolve(self, key, incident, covered, gone):
    result = real_dissolve(self, key, incident, covered, gone)
    if key == 'node:4':
        current['node4_dissolved'] = result
    return result


D.apply_silhouette = apply
S._Mesh._try_merge = merge
S._Mesh._try_dissolve = dissolve
widths = ('0.04214220866560936', '0.045', '0.047', '0.048518914007', '0.048518914008',
          '0.0485189140073579', '0.0485189140074579')
if FINE:
    from decimal import Decimal
    widths += tuple(str(Decimal('0.0485189140073579') + Decimal(k) * Decimal('0.0000000000000001')) for k in range(-40, 41))
for text in widths:
    check_space()
    current, captured = {}, {}
    coverage = wavefront.conveyor_coverage(prepared, text)
    result = D.materialize_domain(prepared, coverage, request=kw['request'],
        near_planar_lift_law=kw['near_planar_lift_law'], decal_topology_law=kw['decal_topology_law'], digests=False, certify=True)
    assert result.is_materialized, str(result.outcome)
    current.update(faces=len(result.batch.faces), vertices=len(result.batch.vertices),
                   structure=dict(result.structure.parts), interval=result.interval.as_record())
    rows[text] = current
    grabs[text] = captured
    print('INDEPENDENT', text, current['faces'], current['vertices'], current['merge']['accepted_by_uv'], flush=True)

a, b, c = [F(v) for v in widths[:3]]
ga, gb, gc = [grabs[v] for v in widths[:3]]
assert ga['ring'] == gb['ring'] == gc['ring']
coefficients = {}
exact_mismatches = []
for name in ('points', 'facts'):
    pairs = []
    for i, (va, vb, vc) in enumerate(zip(ga[name], gb[name], gc[name])):
        row = []
        for j, (x, y, z) in enumerate(zip(va, vb, vc)):
            slope = (y - x).scaled(1 / (b - a))
            intercept = x - slope.scaled(a)
            if not (intercept + slope.scaled(c) - z).is_zero:
                exact_mismatches.append([name, i, j])
            row.append({'intercept': encode(intercept), 'slope': encode(slope)})
        pairs.append(row)
    coefficients[name] = pairs

clean_mismatches = []
if FINE:
    D.apply_silhouette = real_apply
    S._Mesh._try_merge = real_merge
    S._Mesh._try_dissolve = real_dissolve
    for text, observed in rows.items():
        coverage = wavefront.conveyor_coverage(prepared, text)
        clean = D.materialize_domain(prepared, coverage, request=kw['request'],
            near_planar_lift_law=kw['near_planar_lift_law'], decal_topology_law=kw['decal_topology_law'], digests=False, certify=True)
        if not clean.is_materialized or (len(clean.batch.faces), len(clean.batch.vertices)) != (observed['faces'], observed['vertices']):
            clean_mismatches.append(text)
    assert not clean_mismatches

report = dict(base='f8f52ed75dce0ecb650d8f1f45dcb586597b1de0',
    previous_design='1d89542a9fcf62007d23c6335b986e112a1ed20b',
    scene_sha256_before=EXPECTED, scene_sha256_after=digest(SCENE),
    native=native, pickle_sha256=digest(PICKLE), patch=blob['patch'],
    widths=rows, exact_affine_third_width_mismatches=exact_mismatches,
    clean_no_hooks_checked=len(rows) if FINE else 0, clean_no_hooks_mismatches=clean_mismatches,
    coefficients=coefficients, lattice_scale=str(ga['lattice']/a), union=ga['ring'],
    evidence_limit='Third-point equality is a cross-check, not an interval proof or a certified float event band.')
assert report['scene_sha256_after'] == EXPECTED
assert not exact_mismatches
OUT.write_text(json.dumps(report, indent=2, sort_keys=True), encoding='utf-8', newline='\n')
print('INDEPENDENT_OK', flush=True)
