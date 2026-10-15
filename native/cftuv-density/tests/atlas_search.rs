use cftuv_density::atlas_search::{
    GlobalHeightSegment, cell_segments, numerator_bounds, search_global_height, segment_candidates,
};
use cftuv_density::farey::{Endpoint, FareyFailure, FareyPredicates, PrimitiveVector, WorkBudget};
use cftuv_density::order::CanonicalOrdering;
use cftuv_density::{IBig, Rational, UBig};

fn r(n: i64, d: u64) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
fn segment(lo: (i64, u64), hi: (i64, u64)) -> GlobalHeightSegment {
    GlobalHeightSegment {
        use_x_denominator: true,
        denominator_sign: 1,
        outer_lower: r(lo.0, lo.1),
        inner_lower: r(0, 1),
        inner_upper: r(0, 1),
        outer_upper: r(hi.0, hi.1),
    }
}
#[derive(Default)]
struct Predicates {
    local: usize,
    compare: usize,
    edge: usize,
    valid_edge: bool,
}
impl FareyPredicates for Predicates {
    type Error = ();
    fn local_candidate(&mut self, _: usize, _: &PrimitiveVector) -> Result<bool, ()> {
        self.local += 1;
        Ok(true)
    }
    fn candidate_compare(
        &mut self,
        _: usize,
        a: &PrimitiveVector,
        b: &PrimitiveVector,
    ) -> Result<i8, ()> {
        self.compare += 1;
        Ok(if a.1 < b.1 {
            -1
        } else if a.1 > b.1 {
            1
        } else {
            0
        })
    }
    fn edge_valid(&mut self, _: usize, _: Endpoint<'_>, _: Endpoint<'_>) -> Result<bool, ()> {
        self.edge += 1;
        Ok(self.valid_edge)
    }
}

#[test]
fn cell_split_keeps_piece_order_reciprocal_bounds_and_signed_chart_switches() {
    let pieces = cell_segments(true, 1, &[r(-2, 1), r(-3, 2), r(3, 2), r(2, 1)]);
    assert_eq!(pieces.len(), 3);
    assert!(!pieces[0].use_x_denominator);
    assert_eq!(pieces[0].denominator_sign, -1);
    assert_eq!(
        (&pieces[0].outer_lower, &pieces[0].outer_upper),
        (&r(-1, 1), &r(-1, 2))
    );
    assert_eq!(
        (&pieces[0].inner_lower, &pieces[0].inner_upper),
        (&r(-1, 1), &r(-2, 3))
    );
    assert!(pieces[1].use_x_denominator);
    assert_eq!(
        (&pieces[1].outer_lower, &pieces[1].outer_upper),
        (&r(-1, 1), &r(1, 1))
    );
    assert!(!pieces[2].use_x_denominator);
    assert_eq!(pieces[2].denominator_sign, 1);
    assert_eq!(
        (&pieces[2].outer_lower, &pieces[2].outer_upper),
        (&r(1, 2), &r(1, 1))
    );
    assert_eq!(
        (&pieces[2].inner_lower, &pieces[2].inner_upper),
        (&r(2, 3), &r(1, 1))
    );
    assert!(cell_segments(false, -1, &[r(2, 1), r(0, 1), r(0, 1), r(1, 1)]).is_empty());
    let singleton = cell_segments(true, 1, &[r(1, 1), r(1, 1), r(1, 1), r(1, 1)]);
    assert_eq!(singleton.len(), 1);
    assert!(!singleton[0].use_x_denominator);
    assert_eq!(singleton[0].inner_lower, r(0, 1));
}

#[test]
fn numerator_bounds_include_exact_diagonals_and_clamp_to_the_max_norm_cell() {
    assert_eq!(
        numerator_bounds(&r(-1, 1), &r(1, 1), &UBig::ONE),
        (IBig::from(-1), IBig::ONE)
    );
    assert_eq!(
        numerator_bounds(&r(-100, 1), &r(100, 1), &UBig::from(4u8)),
        (IBig::from(-4), IBig::from(4))
    );
    assert_eq!(
        numerator_bounds(&r(-1, 3), &r(-1, 7), &UBig::from(8u8)),
        (IBig::from(-2), IBig::from(-2))
    );
    assert_eq!(
        numerator_bounds(&r(2, 1), &r(3, 1), &UBig::ONE),
        (IBig::from(2), IBig::ONE)
    );
    assert_eq!(
        numerator_bounds(&r(-1, 1), &r(1, 1), &UBig::ZERO),
        (IBig::ZERO, IBig::ZERO)
    );
}

#[test]
fn shell_pays_for_the_integer_span_and_uses_strict_inner_before_local_checks() {
    let mut item = segment((-1, 1), (1, 1));
    item.inner_lower = r(-1, 1);
    item.inner_upper = r(1, 1);
    let mut predicates = Predicates::default();
    let mut budget = WorkBudget::search();
    let result = segment_candidates(&item, 1, &UBig::ONE, &mut budget, &mut predicates).unwrap();
    assert_eq!(
        result,
        vec![
            PrimitiveVector(IBig::ONE, IBig::from(-1)),
            PrimitiveVector(IBig::ONE, IBig::ZERO),
            PrimitiveVector(IBig::ONE, IBig::ONE)
        ]
    );
    assert_eq!(predicates.local, 2);
    assert_eq!(budget.shell_probes, UBig::from(3u8));
    let mut exhausted = WorkBudget::new(UBig::from(2u8));
    predicates.local = 0;
    assert!(matches!(
        segment_candidates(&item, 1, &UBig::ONE, &mut exhausted, &mut predicates),
        Err(FareyFailure::Work(_))
    ));
    assert_eq!(exhausted.shell_probes, UBig::from(3u8));
    assert_eq!(predicates.local, 0);
    assert!(
        segment_candidates(&item, 1, &UBig::ZERO, &mut budget, &mut predicates)
            .unwrap()
            .is_empty()
    );
    assert_eq!(budget.shell_probes, UBig::from(4u8));
}

#[test]
fn overlapping_pieces_pay_and_call_local_before_union_removes_duplicate_vectors() {
    let item = segment((-1, 1), (1, 1));
    let mut predicates = Predicates::default();
    let mut budget = WorkBudget::search();
    let result = search_global_height(
        &[vec![item.clone(), item.clone(), item]],
        &UBig::ONE,
        &mut budget,
        &mut predicates,
        &mut CanonicalOrdering,
    )
    .unwrap();
    assert_eq!(result.height, None);
    assert_eq!(result.fan, None);
    assert_eq!(result.previous_counts, [3]);
    assert_eq!(result.counts, [3]);
    assert_eq!(predicates.local, 9);
    assert_eq!(predicates.edge, 3);
    assert_eq!(budget.shell_probes, UBig::from(9u8));
    assert_eq!(budget.order_steps, UBig::from(6u8));
}

#[test]
fn an_empty_ordinal_plan_returns_before_any_spend_but_zero_ordinals_visit_the_final_edge() {
    let mut predicates = Predicates {
        valid_edge: true,
        ..Default::default()
    };
    let mut budget = WorkBudget::new(UBig::ZERO);
    let result = search_global_height(
        &[vec![segment((-1, 1), (1, 1))], vec![]],
        &UBig::from(30u8),
        &mut budget,
        &mut predicates,
        &mut CanonicalOrdering,
    )
    .unwrap();
    assert_eq!(result.previous_counts, [0, 0]);
    assert_eq!(result.counts, [0, 0]);
    assert_eq!(budget.spent(), UBig::ZERO);
    assert_eq!(predicates.edge, 0);
    assert_eq!(predicates.local, 0);
    let result = search_global_height(
        &[],
        &UBig::ONE,
        &mut budget,
        &mut predicates,
        &mut CanonicalOrdering,
    )
    .unwrap();
    assert_eq!(result.height, Some(UBig::ONE));
    assert_eq!(result.fan, Some(vec![]));
    assert_eq!(predicates.edge, 1);
    assert_eq!(budget.spent(), UBig::ZERO);
}

#[test]
#[ignore = "Needs CFTUV_ATLAS_ORACLE_PYTHON and CFTUV_ATLAS_REPLAY"]
fn live_python_prepared_atlas_search_matches_budgets_enumeration_and_full_callbacks() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let python = std::env::var("CFTUV_ATLAS_ORACLE_PYTHON").expect("oraclePython");
    let replay = std::env::var("CFTUV_ATLAS_REPLAY").expect("atlasReplay");
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
            panic!("invalidoracle")
        }
    }
    assert!(expected.len() > 500);
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
    let actual: Vec<_> = actual.lines().collect();
    assert_eq!(actual.len(), expected.len());
    for (row, (actual, expected)) in actual.iter().zip(&expected).enumerate() {
        assert!(
            actual == expected,
            "atlascase{row}:actual{actual} expected{expected}"
        )
    }
    println!(
        "{} actual Python atlas cases:0mismatch in cell/bounds/candidates/height/fan/previous_counts/budgets/every L-C-E callback",
        expected.len()
    );
}

const ORACLE: &str = r#"
import sys,random
from fractions import Fraction as F
import sympy as sp
from cftuv_envelope.reference import adaptive_density_atlas as atlas,adaptive_density_fan as fan,adaptive_density_band as band
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.planar_types import exact_quadratic_value
from cftuv_envelope.reference.radical_v2 import RadicalSumV2,canonical_text
from cftuv_envelope.contracts.analysis import TurnOrientation

rng=random.Random(20261012)
old_local,old_compare,old_cross,old_subturn=fan._local_candidate,fan._candidate_compare,fan._oriented_cross,fan._subturn
identity=((sp.Integer(1),sp.Integer(0)),(sp.Integer(0),sp.Integer(1)))
def value(expr):return canonical_text(RadicalSumV2.from_terms(exact_quadratic_value(sp.expand(expr)).terms)[0])
def primitive(v):return ','.join(map(str,v))
def candidates(v):return ';'.join(map(primitive,v)) or '-'
def fraction(v):return f'{v.numerator}/{v.denominator}'
def segtext(s):return ','.join([str(int(s.use_x_denominator)),str(s.denominator_sign),*[fraction(v) for v in [s.outer_lower,s.inner_lower,s.inner_upper,s.outer_upper]]])
def plantext(plans):return '#'.join('@'.join(map(segtext,plan))or'_' for plan in plans)or'-'
def emit(command,answer):print('IN\t'+command);print('OUT\t'+command.split('\t')[0]+'\t'+command.split('\t')[1]+'\t'+answer)
def make(values,use_x=True,sign=1):return atlas.GlobalHeightSegment(use_x,sign,*map(F,values))
def cell(name,use_x,sign,bracket):
 answer=atlas.cell_segments(use_x,sign,bracket)
 emit('\t'.join(['CELL',name,str(int(use_x)),str(sign),*map(fraction,bracket)]),'@'.join(map(segtext,answer))or'-')
 return answer
for use_x in [False,True]:
 for sign in [-1,1]:
  for n,b in enumerate([(-2,-F(3,2),F(3,2),2),(-1,-1,1,1),(1,1,1,1),(-1,-1,-1,-1),(0,0,0,0),(2,0,0,1),(-5,-4,-2,-1),(1,2,3,5),(-F(3,2),0,0,F(5,4))]):cell(f'cell-{use_x}-{sign}-{n}',use_x,sign,list(map(F,b)))
for _ in range(150):
 b=[F(rng.randrange(-30,31),rng.randrange(1,15))for i in range(4)]
 cell(f'cell-random-{_}',bool(_%2),[-1,1][_//2%2],b)
for height in [0,1,2,3,8,65,10**50]:
 for lo,hi in [(-1,1),(-F(1,3),-F(1,7)),(F(1,8),F(1,8)),(2,3),(-10**100,10**100),(F(3,7),F(1,3))]:
  lo,hi=F(lo),F(hi)
  answer=atlas.numerator_bounds(lo,hi,height)
  emit(f'BOUNDS\tbounds-{height}-{lo}-{hi}\t{fraction(lo)}\t{fraction(hi)}\t{height}',primitive(answer))

def run(name,plans,cap,stop,mode='none',geometry=None,shell=None,height=None,ordinal=1):
 events=[];counts={'L':0,'C':0,'E':0}
 failure=mode.split(':')if':'in mode else None
 if geometry is None:
  metric=ExactPlanarMetric(identity,identity,1);q=2;orientation=1
  ideals=[fan._vector(1,0),*[fan._vector(1,1)for _ in plans],fan._vector(0,1)]
  law='voronoi'
 else:metric,ideals,q,orientation,law=geometry
 ideal=fan._IdealTuple(ideals);ideal.metric=metric;ideal.window_law=band.WINDOW_LAW_NARROW_BAND if law=='narrow'else band.WINDOW_LAW_VORONOI;ideal.band_orientation=orientation;ideal.band_cache={}
 def vt(v):
  if v is ideal[0]:return'IN'
  if v is ideal[-1]:return'OUT'
  if hasattr(v,'expressions'):v=v.expressions()
  return primitive(map(int,v))
 def record(kind,ordinal,args,compute):
  counts[kind]+=1
  if failure and failure==[kind,str(counts[kind])]:
   msg='callback_'+{'L':'local','C':'compare','E':'edge'}[kind]+'_refused';events.append(f'{kind}:{ordinal}:{args}:!{msg}');raise ValueError(msg)
  try:result=compute()
  except ZeroDivisionError:events.append(f'{kind}:{ordinal}:{args}:!ZeroSlopeDenominator');raise
  events.append(f'{kind}:{ordinal}:{args}:{int(result)}');return result
 def local(metric,ideal,ordinal,vector,q):return record('L',ordinal,vt(vector),lambda:old_local(metric,ideal,ordinal,vector,q)if geometry else(sum(map(abs,vector))+ordinal)%3!=0)
 def compare_factory(metric,ideal,ordinal):
  original=old_compare(metric,ideal,ordinal)
  return lambda left,right:record('C',ordinal,vt(left)+'|'+vt(right),lambda:original(left,right)if geometry else((sum(left)%7)>(sum(right)%7))-((sum(left)%7)<(sum(right)%7)))
 def cross(metric,left,right):
  result=old_cross(metric,left,right);caller=sys._getframe(1)
  if caller.f_code.co_name=='visit'and fan._sign(result,metric)!=orientation:record('E',caller.f_locals['ordinal']+1,vt(left)+'|'+vt(right),lambda:False)
  return result
 def subturn(metric,left,right,q):
  caller=sys._getframe(1)
  return record('E',caller.f_locals['ordinal']+1,vt(left)+'|'+vt(right),lambda:old_subturn(metric,left,right,q))if caller.f_code.co_name=='visit'else old_subturn(metric,left,right,q)
 fan._candidate_compare=compare_factory;fan._oriented_cross=cross;fan._subturn=subturn
 budget=fan._work_budget(cap)
 try:
  if shell is not None:
   answer=atlas.segment_candidates(metric,ideal,ordinal,q,shell,height,budget,local_candidate=local,vector_from_slope=fan._vector_from_slope)
   result='CANDIDATES:'+candidates(answer)
  else:
   orient=TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION if orientation==1 else TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION
   h,answer,previous,total=atlas.search_global_height(metric,ideal,orient,q,stop,segments=plans,local_candidate=local,vector_from_slope=fan._vector_from_slope,best_fan=fan._best_fan,budget=budget)
   ct=lambda values:','.join(map(str,values))or'-'
   result='\t'.join(['OK',str(h)if h is not None else'-','NONE'if answer is None else candidates(answer),ct(previous),ct(total)])
 except ZeroDivisionError:result='ERROR:ZeroSlopeDenominator'
 except (fan.DensityRationalAuthorityExhausted,ValueError)as error:result='ERROR:'+str(error)
 result+='\tWORK:'+str(budget.shell_probes)+','+str(budget.order_steps)+'\tTRACE:'+'@'.join(events)
 if shell is not None:command=f'SHELL\t{name}\t{segtext(shell)}\t{height}\t{ordinal}\t{cap}\t{mode}'
 elif geometry is None:command=f'SEARCH\t{name}\t{cap}\t{stop}\t{plantext(plans)}\t{mode}'
 else:
  a,b,c=metric.gram[0][0],metric.gram[0][1],metric.gram[1][1]
  command='\t'.join(['GEOMETRY',name,str(cap),str(stop),plantext(plans),'-',*[f'{v.p}/{v.q}'for v in [a,b,c]],str(metric.owner_orientation_sign),str(orientation),str(q),law,'|'.join('~'.join(value(e)for e in v.expressions())for v in ideal)])
 emit(command,result)

segments=[make((-1,-1,1,1)),make((-1,0,0,1)),make((F(1,8),0,0,F(1,4))),make((-F(3,4),-F(2,3),-F(1,3),-F(1,4)),False,-1)]
for index,s in enumerate(segments):
 for h in [0,1,2,3,4,8,15]:
  for cap in [0,1,2,131072]:run(f'shell-{index}-{h}-{cap}',[],cap,0,shell=s,height=h,ordinal=index%3+1)
plans_cases=[[],[[]],[[segments[0]],[]],[[segments[0]]],[[segments[1],segments[1]]],[[segments[2]],[segments[0]]],[[segments[0],segments[3]],[segments[1],segments[2]]]]
for n,plans in enumerate(plans_cases):
 for stop in [0,1,2,3,7]:
  for cap in [0,1,2,5,11,32,131072]:run(f'search-{n}-{stop}-{cap}',plans,cap,stop)
for kind in ['L','C','E']:
 for at in range(1,10):run(f'refusal-{kind}-{at}',[[segments[1],segments[3]],[segments[0]]],131072,9,mode=f'{kind}:{at}')
for round in range(100):
 plans=[]
 for ordinal in range(rng.randrange(1,4)):
  plan=[]
  for piece in range(rng.randrange(1,4)):
   bounds=[F(rng.randrange(-5,6),rng.randrange(1,8))for _ in range(4)];bounds[0],bounds[3]=min(bounds[0],bounds[3]),max(bounds[0],bounds[3]);plan.extend(atlas.cell_segments(bool(rng.randrange(2)),rng.choice([-1,1]),bounds))
  plans.append(plan)
 run(f'random-search-{round}',plans,rng.choice([0,20,100,131072]),rng.randrange(10))
# Реальные V2-геометрические callbacks, с доказанным исходным V2-входом.
for owner in [-1,1]:
 for orientation in [-1,1]:
  metric=ExactPlanarMetric(identity,identity,owner)
  sign=owner*orientation
  for q in range(2,7):
   for law in ['voronoi','narrow']:
    ideals=[fan._vector(1,0),fan._vector(sp.sqrt(2)/2,sign*sp.sqrt(2)/2),fan._vector(0,sign)]
    interval=(0,0,0,1)if sign>0 else(-1,0,0,0)
    plan=[make(interval),make(interval,False,sign)]
    run(f'geometry-{owner}-{orientation}-{q}-{law}',[plan],131072,10,geometry=(metric,ideals,q,orientation,law))
"#;
