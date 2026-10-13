use cftuv_density::farey::{Endpoint, FareyPredicates, PrimitiveVector, WorkBudget, best_fan};
use cftuv_density::geometry::{GeometryAdapter, GeometryFailureKind, WindowLaw};
use cftuv_density::order::CanonicalOrdering;
use cftuv_density::{DensityTransaction, IBig, Metric2, Radical, Rational, UBig, Vector2};

fn r(n: i64, d: u64) -> Radical {
    Radical::rational(IBig::from(n), UBig::from(d))
}
fn vector(x: i64, y: i64) -> Vector2 {
    Vector2::new(r(x, 1), r(y, 1))
}
fn primitive(x: i64, y: i64) -> PrimitiveVector {
    PrimitiveVector(IBig::from(x), IBig::from(y))
}
fn metric() -> Metric2 {
    let one = Rational::integer(IBig::ONE);
    let zero = Rational::integer(IBig::ZERO);
    Metric2::from_gram([[one.clone(), zero.clone()], [zero, one]], 1)
        .unwrap()
        .0
}
fn adapter(ideal: Vec<Vector2>, q: u8, law: WindowLaw) -> GeometryAdapter {
    GeometryAdapter::new(DensityTransaction::new(metric()), ideal, q, 1, law).unwrap()
}
fn ordinary(q: u8) -> GeometryAdapter {
    adapter(
        vec![vector(1, 0), Vector2::new(r(4, 5), r(3, 5)), vector(0, 1)],
        q,
        WindowLaw::Voronoi,
    )
}

#[test]
fn local_checks_the_window_then_the_real_incoming_and_outgoing_subturns() {
    let mut geometry = ordinary(3);
    assert!(geometry.local_candidate(1, &primitive(3, 2)).unwrap());
    assert!(geometry.inside_ordinal(1, &primitive(2, 1)).unwrap());
    assert!(!geometry.local_candidate(1, &primitive(2, 1)).unwrap());
    assert!(!geometry.local_candidate(1, &primitive(-1, 1)).unwrap());
    assert!(!ordinary(6).local_candidate(1, &primitive(3, 2)).unwrap());
}

#[test]
fn strict_voronoi_boundaries_are_not_admitted() {
    let mut geometry = ordinary(2);
    let previous = vector(1, 0);
    let center = vector(1, 1);
    let following = vector(0, 1);
    assert!(
        !geometry
            .inside_window(&vector(1, 0), &previous, &center, &following)
            .unwrap()
    );
    assert!(
        geometry
            .inside_window(&vector(1, 1), &previous, &center, &following)
            .unwrap()
    );
    assert!(
        !geometry
            .inside_window(&vector(0, 1), &previous, &center, &following)
            .unwrap()
    );
}

#[test]
fn narrow_band_threshold_equality_keeps_the_real_neighbour_and_cache() {
    let previous = Vector2::new(r(3248, 3250), r(-114, 3250));
    let center = vector(1, 0);
    let following = vector(0, 1);
    let mut geometry = adapter(
        vec![previous.clone(), center.clone(), following],
        2,
        WindowLaw::NarrowBand,
    );
    assert!(
        geometry
            .neighbour_is_inside_double_band(&center, &previous)
            .unwrap()
    );
    let (actual_previous, actual_following) = geometry.window_neighbours(1).unwrap();
    assert_eq!(actual_previous, previous);
    assert_eq!(actual_following, Vector2::new(r(3248, 3250), r(114, 3250)));
    let work = geometry.work();
    assert_eq!(
        geometry.window_neighbours(1).unwrap(),
        (actual_previous, actual_following)
    );
    assert_eq!(geometry.work(), work);
}

#[test]
fn phantom_windows_never_replace_real_neighbours_of_the_subturn_predicate() {
    let root = Radical::sqrt_of_rational(IBig::ONE, UBig::from(2u8))
        .unwrap()
        .0;
    let ideal = vec![vector(1, 0), Vector2::new(root.clone(), root), vector(0, 1)];
    let mut geometry = adapter(ideal.clone(), 6, WindowLaw::NarrowBand);
    assert!(geometry.inside_ordinal(1, &primitive(1, 1)).unwrap());
    assert!(!geometry.local_candidate(1, &primitive(1, 1)).unwrap());
    assert!(
        adapter(ideal, 4, WindowLaw::NarrowBand)
            .local_candidate(1, &primitive(1, 1))
            .unwrap()
    );
}

#[test]
fn objective_is_exact_and_left_dominant_ties_keep_even_the_legacy_nonantisymmetry() {
    let mut geometry = adapter(
        vec![vector(1, 0), vector(1, 1), vector(0, 1)],
        2,
        WindowLaw::Voronoi,
    );
    assert_eq!(geometry.objective(1, &primitive(2, 1)).unwrap(), r(9, 5));
    assert_eq!(
        geometry
            .candidate_compare(1, &primitive(2, 1), &primitive(1, 2))
            .unwrap(),
        1
    );
    assert_eq!(
        geometry
            .candidate_compare(1, &primitive(1, 2), &primitive(2, 1))
            .unwrap(),
        1
    );
    assert_eq!(
        geometry
            .candidate_compare(1, &primitive(1, 2), &primitive(-1, -2))
            .unwrap(),
        0
    );
    let error = geometry
        .candidate_compare(1, &primitive(1, 0), &primitive(0, 1))
        .unwrap_err();
    assert_eq!(error.kind, GeometryFailureKind::ZeroSlopeDenominator);
    assert_eq!(error.work, geometry.work());
}

#[test]
fn edge_tests_cross_first_and_the_adapter_is_an_actual_farey_geometry_owner() {
    let mut geometry = ordinary(3);
    assert!(
        !geometry
            .edge_valid(1, Endpoint::Incoming, Endpoint::Primitive(&primitive(1, 0)))
            .unwrap()
    );
    assert!(
        !geometry
            .edge_valid(
                1,
                Endpoint::Incoming,
                Endpoint::Primitive(&primitive(-1, -1))
            )
            .unwrap()
    );
    let mut budget = WorkBudget::search();
    let fan = best_fan(
        &[vec![primitive(2, 1), primitive(3, 2)]],
        &mut budget,
        &mut geometry,
        &mut CanonicalOrdering,
    )
    .unwrap();
    assert_eq!(fan, Some(vec![primitive(3, 2)]));
    assert_eq!(budget.shell_probes, UBig::ZERO);
    assert!(budget.order_steps > UBig::ZERO);
}

#[test]
fn malformed_inputs_and_nonpositive_candidate_norm_are_internal_errors_with_work() {
    assert!(
        matches!(GeometryAdapter::new(DensityTransaction::new(metric()), vec![], 1, 0, WindowLaw::Voronoi), Err(error) if error.kind == GeometryFailureKind::InvalidQ(1))
    );
    let mut geometry = ordinary(5);
    let error = geometry.objective(1, &primitive(0, 0)).unwrap_err();
    assert_eq!(error.kind, GeometryFailureKind::NonPositiveCandidateNorm);
    assert_eq!(error.work, geometry.work());
    assert_eq!(
        geometry.objective(0, &primitive(1, 1)).unwrap_err().kind,
        GeometryFailureKind::InvalidOrdinal(0)
    );
}

#[test]
#[ignore = "Needs CFTUV_GEOMETRY_ORACLE_PYTHON and CFTUV_GEOMETRY_REPLAY"]
fn live_python_geometry_and_best_fan_match_actual_v2_adapter_and_canonical_ordering() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let python = std::env::var("CFTUV_GEOMETRY_ORACLE_PYTHON").expect("oracle Python");
    let replay = std::env::var("CFTUV_GEOMETRY_REPLAY").expect("built geometry_replay");
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let oracle = Command::new(python)
        .args(["-c", ORACLE])
        .env("PYTHONPATH", repo.join("kernel/src"))
        .output()
        .unwrap();
    assert!(
        oracle.status.success(),
        "{}",
        String::from_utf8_lossy(&oracle.stderr)
    );
    let text = String::from_utf8(oracle.stdout).unwrap();
    let mut input = String::new();
    let mut expected = Vec::new();
    for line in text.lines() {
        if let Some(line) = line.strip_prefix("IN\t") {
            input.push_str(line);
            input.push('\n');
        } else if let Some(line) = line.strip_prefix("OUT\t") {
            expected.push(line.to_owned());
        } else {
            panic!("invalid oracle line");
        }
    }
    assert!(expected.len() > 1000);
    let mut native = Command::new(replay)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = native.stdin.take().unwrap();
    let writer = std::thread::spawn(move || stdin.write_all(input.as_bytes()));
    let output = native.wait_with_output().unwrap();
    writer.join().unwrap().unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let actual = String::from_utf8(output.stdout).unwrap();
    let actual: Vec<_> = actual
        .lines()
        .map(|line| line.split("\tV2_WORK:").next().unwrap())
        .collect();
    assert_eq!(actual.len(), expected.len());
    for (row, (actual, expected)) in actual.iter().zip(&expected).enumerate() {
        assert!(
            actual == expected,
            "geometry case{row}:actual{actual} expected{expected}"
        );
    }
    println!(
        "{} actual Python geometry/best_fan cases:0mismatch;V2 Work diagnostic is separately emitted",
        expected.len()
    );
}

const ORACLE: &str = r#"
import sympy as sp
from fractions import Fraction as F
from cftuv_envelope.reference import adaptive_density_fan as fan, adaptive_density_band as band
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.planar_types import exact_quadratic_value
from cftuv_envelope.reference.radical_v2 import RadicalSumV2, canonical_text
from cftuv_envelope.contracts.analysis import TurnOrientation

def value(expr): return canonical_text(RadicalSumV2.from_terms(exact_quadratic_value(sp.expand(expr)).terms)[0])
def vtext(vec): return '~'.join(value(expr) for expr in vec.expressions())
def primitive_text(vec): return ','.join(map(str,vec))
def emit(op,case,metric,ideal,q,orientation,law,args,compute,ordinal=1):
    gram=metric.gram
    rationals=[gram[0][0],gram[0][1],gram[1][1]]
    wire=[op,case,*[f'{v.p}/{v.q}' for v in rationals],metric.owner_orientation_sign,orientation,q,law,'|'.join(map(vtext,ideal)),ordinal,args]
    try: result=compute()
    except ZeroDivisionError: result='ERROR:ZeroSlopeDenominator'
    print('IN\t'+'\t'.join(map(str,wire)))
    print('OUT\t'+op+'\t'+case+'\t'+result)

points=[(1,0),(0,1),(1,1),(2,1),(1,2),(-1,1),(1,-1),(-1,-1),(3,2),(2,3)]
coefficient_fans=[
 [(1,0),(sp.Rational(4,5),sp.Rational(3,5)),(0,1)],
 [(1,0),(sp.sqrt(2)/2,sp.sqrt(2)/2),(0,1)],
 [(1,0),((sp.sqrt(6)+sp.sqrt(2))/4,(sp.sqrt(6)-sp.sqrt(2))/4),(0,1)],
 [(1,0),(sp.sqrt(3)/2,sp.Rational(1,2)),(sp.Rational(1,2),sp.sqrt(3)/2),(0,1)],
]
grams=[(1,0,1),(2,sp.Rational(1,3),3),(sp.Rational(9,7),sp.Rational(-2,5),sp.Rational(5,3))]
case_id=0
for gram in grams:
 for owner in [-1,1]:
  for orientation in [-1,1]:
   matrix=sp.Matrix([[gram[0],gram[1]],[gram[1],gram[2]]])
   inv=matrix.inv()
   metric=ExactPlanarMetric(tuple(tuple(sp.Rational(matrix[i,j]) for j in range(2)) for i in range(2)),tuple(tuple(sp.Rational(inv[i,j]) for j in range(2)) for i in range(2)),owner)
   first=fan._vector(1/sp.sqrt(inv[0,0]),0)
   perpendicular=fan._quarter_turn(metric,first,orientation)
   fx,fy=first.expressions();px,py=perpendicular.expressions()
   for coefficients in coefficient_fans:
    base=[fan._vector(c*fx+s*px,c*fy+s*py) for c,s in coefficients]
    for law in ['voronoi','narrow']:
     q=case_id%5+2
     ideal=fan._IdealTuple(base)
     ideal.metric=metric
     ideal.window_law=band.WINDOW_LAW_VORONOI if law=='voronoi' else band.WINDOW_LAW_NARROW_BAND
     ideal.band_orientation=orientation
     ideal.band_cache={}
     orient_enum=TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION if orientation==1 else TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION
     prefix=f'variant-{case_id}'
     for candidate in points:
      vec=fan._vector(*candidate)
      emit('inside',prefix+primitive_text(candidate),metric,ideal,q,orientation,law,primitive_text(candidate),lambda vec=vec:'BOOL:'+str(int(fan._inside_ordinal(metric,vec,*[band.window_neighbours(ideal,1)[0],ideal[1],band.window_neighbours(ideal,1)[1]]))))
      emit('local',prefix+primitive_text(candidate),metric,ideal,q,orientation,law,primitive_text(candidate),lambda candidate=candidate:'BOOL:'+str(int(fan._local_candidate(metric,ideal,1,candidate,q))))
     for candidate in points[:5]:
      emit('objective',prefix+primitive_text(candidate),metric,ideal,q,orientation,law,primitive_text(candidate),lambda candidate=candidate:'VALUE:'+value(fan._objective(metric,ideal,1,candidate)))
     for left,right in zip(points,points[1:]+points[:1]):
      emit('compare',prefix+primitive_text(left)+'-'+primitive_text(right),metric,ideal,q,orientation,law,primitive_text(left)+'|'+primitive_text(right),lambda left=left,right=right:'SIGN:'+str(fan._candidate_compare(metric,ideal,1)(left,right)))
     for left,right in [('IN',points[2]),(points[2],'OUT'),('IN',points[0]),(points[1],'OUT'),(points[5],points[6])]:
      lv=ideal[0] if left=='IN' else fan._vector(*left)
      rv=ideal[-1] if right=='OUT' else fan._vector(*right)
      emit('edge',prefix+str(left)+str(right),metric,ideal,q,orientation,law,(left if type(left)is str else primitive_text(left))+'|'+(right if type(right)is str else primitive_text(right)),lambda lv=lv,rv=rv:'BOOL:'+str(int(fan._sign(fan._oriented_cross(metric,lv,rv),metric)==orientation and fan._subturn(metric,lv,rv,q))))
     emit('neighbours',prefix,metric,ideal,q,orientation,law,'-',lambda:'VECTORS:'+'|'.join(map(vtext,band.window_neighbours(ideal,1))))
     for side in [-1,1]:
      emit('rotate',prefix+str(side),metric,ideal,q,orientation,law,str(side),lambda side=side:'VECTOR:'+vtext(band._rotated_by_double_band(metric,ideal[1],side,orientation)))
     for neighbour in [0,2]:
      emit('bandinside',prefix+str(neighbour),metric,ideal,q,orientation,law,str(neighbour),lambda neighbour=neighbour:'BOOL:'+str(int(band._neighbour_is_inside_the_double_band(metric,ideal[1],ideal[neighbour]))))
     rows=[points[:5]]*(len(ideal)-2)
     for cap in [0,131072]:
      def best(cap=cap):
       candidates=[]
       for row in rows:
        candidates.append(set())
        candidates[-1].update(tuple(row))
       budget=fan._work_budget(cap)
       try:
        answer=fan._best_fan(metric,ideal,orient_enum,q,candidates,budget)
        result='FAN:NONE' if answer is None else 'FAN:'+(';'.join(map(primitive_text,answer)) or '-')
       except ZeroDivisionError: result='ERROR:ZeroSlopeDenominator'
       except fan.DensityRationalAuthorityExhausted as error: result='BUDGET:'+str(error)
       return result+'\tSEARCH_WORK:'+str(budget.shell_probes)+','+str(budget.order_steps)
      emit('best',prefix+str(cap),metric,ideal,q,orientation,law,str(cap)+':'+ '/'.join(';'.join(map(primitive_text,row)) for row in rows),best)
     case_id+=1
# Точная граница полосы и рациональные подходы с обеих сторон.
metric=ExactPlanarMetric(((sp.Integer(1),sp.Integer(0)),(sp.Integer(0),sp.Integer(1))),((sp.Integer(1),sp.Integer(0)),(sp.Integer(0),sp.Integer(1))),1)
for offset in [F(-1,10**8),F(0),F(1,10**8)]:
 t=sp.Rational(F(1,57)+offset)
 c=(1-t*t)/(1+t*t);s=2*t/(1+t*t)
 ideal=fan._IdealTuple([fan._vector(c,-s),fan._vector(1,0),fan._vector(c,s)])
 ideal.metric=metric;ideal.window_law=band.WINDOW_LAW_NARROW_BAND;ideal.band_orientation=1;ideal.band_cache={}
 for q in range(2,7):
  name=f'band-boundary-{offset}-{q}'
  emit('neighbours',name,metric,ideal,q,1,'narrow','-',lambda:'VECTORS:'+'|'.join(map(vtext,band.window_neighbours(ideal,1))))
  for neighbour in [0,2]: emit('bandinside',name+str(neighbour),metric,ideal,q,1,'narrow',str(neighbour),lambda neighbour=neighbour:'BOOL:'+str(int(band._neighbour_is_inside_the_double_band(metric,ideal[1],ideal[neighbour]))))
"#;
