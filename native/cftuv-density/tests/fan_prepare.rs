use cftuv_density::atlas_plan::{DecimalEnvelope, EndpointKind};
use cftuv_density::fan_prepare::*;
use cftuv_density::farey::WorkBudget;
use cftuv_density::{IBig, Rational, UBig};
fn r(n: i64, d: u64) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
fn env(v: Rational) -> DecimalEnvelope {
    DecimalEnvelope {
        lower: v.clone(),
        upper: v,
        lower_kind: EndpointKind::Closed,
        upper_kind: EndpointKind::Closed,
        absolute_error_bound: r(0, 1),
        quantum: None,
    }
}
fn record() -> PrepareRecord {
    PrepareRecord {
        use_x_denominator: true,
        denominator_sign: 1,
        lower_slope_envelope: env(r(-1, 1)),
        upper_slope_envelope: env(r(1, 1)),
    }
}
fn bx(lo: Rational, hi: Rational) -> FanTerminationBox {
    FanTerminationBox {
        lower: lo,
        upper: hi,
        use_x_denominator: true,
        denominator_sign: 1,
    }
}
#[derive(Clone, Debug, PartialEq, Eq)]
enum Error {
    Reference,
    Other,
}
struct Laws {
    trace: Vec<&'static str>,
    local: bool,
    inside: bool,
    cross: i8,
    turn: bool,
    limit: Result<bool, Error>,
    boundaries: Vec<Rational>,
    throw_candidate: bool,
}
impl Default for Laws {
    fn default() -> Self {
        Self {
            trace: vec![],
            local: true,
            inside: true,
            cross: 1,
            turn: true,
            limit: Ok(false),
            boundaries: vec![],
            throw_candidate: false,
        }
    }
}
impl FanPrepareLaws for Laws {
    type Vector = Rational;
    type Scalar = Rational;
    type Error = Error;
    fn subturn_boundary_vectors(
        &mut self,
        _: usize,
        _: i8,
        _: usize,
    ) -> Result<Vec<Rational>, Error> {
        self.trace.push("boundaries");
        Ok(self.boundaries.clone())
    }
    fn denominator(&mut self, _: &Rational, _: bool) -> Result<Rational, Error> {
        self.trace.push("denominator");
        Ok(r(1, 1))
    }
    fn sign(&mut self, _: &Rational) -> Result<i8, Error> {
        self.trace.push("sign");
        Ok(1)
    }
    fn slope(&mut self, v: &Rational, _: bool) -> Result<Rational, Error> {
        self.trace.push("slope");
        Ok(v.clone())
    }
    fn decimal_envelope(&mut self, v: &Rational) -> Result<DecimalEnvelope, Error> {
        self.trace.push("envelope");
        Ok(env(v.clone()))
    }
    fn vector_from_slope(&mut self, v: &Rational, _: bool, _: i8) -> Result<Rational, Error> {
        self.trace.push("vector");
        Ok(v.clone())
    }
    fn candidate_vector(&mut self, v: &Rational, _: bool, _: i8) -> Result<Rational, Error> {
        self.trace.push("candidate");
        if self.throw_candidate {
            Err(Error::Other)
        } else {
            Ok(v.clone())
        }
    }
    fn local_candidate(&mut self, _: usize, _: &Rational, _: usize) -> Result<bool, Error> {
        self.trace.push("local");
        Ok(self.local)
    }
    fn window_neighbours(&mut self, _: usize) -> Result<(Rational, Rational), Error> {
        self.trace.push("neighbours");
        Ok((r(-1, 1), r(1, 1)))
    }
    fn inside_ordinal(
        &mut self,
        _: &Rational,
        _: &Rational,
        _: &Rational,
        _: &Rational,
    ) -> Result<bool, Error> {
        self.trace.push("inside");
        Ok(self.inside)
    }
    fn oriented_cross_sign(&mut self, _: &Rational, _: &Rational) -> Result<i8, Error> {
        self.trace.push("cross");
        Ok(self.cross)
    }
    fn subturn(&mut self, _: &Rational, _: &Rational, _: usize) -> Result<bool, Error> {
        self.trace.push("subturn");
        Ok(self.turn)
    }
    fn subturn_boundary(&mut self, _: &Rational, _: &Rational, _: usize) -> Result<bool, Error> {
        self.trace.push("limit");
        self.limit.clone()
    }
    fn is_reference_geometry_error(error: &Error) -> bool {
        *error == Error::Reference
    }
}

#[test]
fn sealed_collects_all_boundaries_before_center_overlap_and_shortcircuits_inner_proof() {
    let ideal = [r(-1, 1), r(0, 1), r(1, 1)];
    let mut laws = Laws {
        boundaries: vec![r(0, 1), r(1, 2)],
        ..Default::default()
    };
    assert_eq!(
        sealed_local_interval(&ideal, 1, 1, 5, &record(), &mut laws),
        Err(FanPrepareFailure::Invalid(
            PrepareInvalid::BoundaryOverlapsIdeal
        ))
    );
    assert_eq!(
        laws.trace,
        [
            "boundaries",
            "denominator",
            "sign",
            "slope",
            "envelope",
            "denominator",
            "sign",
            "slope",
            "envelope",
            "slope",
            "envelope"
        ]
    );
    let mut laws = Laws {
        local: false,
        ..Default::default()
    };
    assert_eq!(
        sealed_local_interval(&ideal, 1, 1, 5, &record(), &mut laws),
        Err(FanPrepareFailure::Invalid(
            PrepareInvalid::InnerEnvelopeNotAdmissible
        ))
    );
    assert_eq!(
        laws.trace,
        ["boundaries", "slope", "envelope", "vector", "local"]
    );
    let mut laws = Laws::default();
    assert_eq!(
        sealed_local_interval(&ideal, 1, 1, 5, &record(), &mut laws).unwrap(),
        [r(-1, 1), r(-1, 2), r(1, 2), r(1, 1)]
    );
}

#[test]
fn feasibility_has_lower_upper_neighbour_inside_then_cross_scan_before_subturn_scan() {
    let ideal = [r(-1, 1), r(0, 1), r(1, 1)];
    let boxes = [bx(r(-1, 2), r(1, 2))];
    let mut laws = Laws::default();
    assert!(box_is_feasible(&ideal, 1, 5, &boxes, &mut laws).unwrap());
    assert_eq!(
        laws.trace,
        [
            "candidate",
            "candidate",
            "neighbours",
            "inside",
            "inside",
            "cross",
            "cross",
            "subturn",
            "subturn",
            "cross",
            "cross",
            "subturn",
            "subturn"
        ]
    );
    let mut laws = Laws {
        inside: false,
        ..Default::default()
    };
    assert!(!box_is_feasible(&ideal, 1, 5, &boxes, &mut laws).unwrap());
    assert_eq!(
        laws.trace,
        ["candidate", "candidate", "neighbours", "inside"]
    );
    let mut laws = Laws {
        cross: 0,
        ..Default::default()
    };
    assert!(!box_is_feasible(&ideal, 1, 5, &boxes, &mut laws).unwrap());
    assert!(!laws.trace.contains(&"subturn"));
}

#[test]
fn preparation_spends_before_candidate_failure_and_before_budget_throw() {
    let ideal = [r(-1, 1), r(0, 1), r(1, 1)];
    let records = [record()];
    let mut budget = WorkBudget::new(UBig::ONE);
    let mut laws = Laws::default();
    assert!(matches!(
        termination_boxes(&ideal, 1, 5, &records, &mut budget, &mut laws),
        Err(FanPrepareFailure::Work(_))
    ));
    assert_eq!(budget.order_steps, UBig::from(2u8));
    assert_eq!(laws.trace, ["slope", "envelope"]);
    let mut budget = WorkBudget::preparation();
    let mut laws = Laws {
        throw_candidate: true,
        ..Default::default()
    };
    assert_eq!(
        termination_boxes(&ideal, 1, 5, &records, &mut budget, &mut laws),
        Err(FanPrepareFailure::Callback(Error::Other))
    );
    assert_eq!(budget.order_steps, UBig::from(2u8));
    assert_eq!(laws.trace, ["slope", "envelope", "candidate"]);
    let mut bad = record();
    bad.lower_slope_envelope = env(r(0, 1));
    let mut budget = WorkBudget::preparation();
    let mut laws = Laws::default();
    assert_eq!(
        termination_boxes(&ideal, 1, 5, &[bad], &mut budget, &mut laws),
        Err(FanPrepareFailure::Invalid(
            PrepareInvalid::IdealOutsideWindow
        ))
    );
    assert_eq!(budget.spent(), UBig::ZERO);
}

#[test]
fn first_radius_is_exact_dyadic_and_final_diagnostic_only_catches_reference_geometry_error() {
    let ideal = [r(-1, 1), r(0, 1), r(1, 1)];
    let mut budget = WorkBudget::preparation();
    let mut laws = Laws::default();
    let boxes = termination_boxes(&ideal, 1, 5, &[record()], &mut budget, &mut laws).unwrap();
    assert_eq!(boxes[0].lower, r(-1, 1 << 24));
    assert_eq!(boxes[0].upper, r(1, 1 << 24));
    assert_eq!(budget.order_steps, UBig::from(2u8));
    for (limit, diagnostic) in [
        (Ok(true), Some(LimitDiagnostic::True)),
        (Ok(false), Some(LimitDiagnostic::False)),
        (Err(Error::Reference), Some(LimitDiagnostic::Undecided)),
        (Err(Error::Other), None),
    ] {
        let mut budget = WorkBudget::preparation();
        let mut laws = Laws {
            inside: false,
            limit,
            ..Default::default()
        };
        let result = termination_boxes(&ideal, 1, 5, &[record()], &mut budget, &mut laws);
        assert_eq!(budget.order_steps, UBig::from(192u16));
        assert_eq!(laws.trace.iter().filter(|&&s| s == "limit").count(), 1);
        if let Some(at_limit) = diagnostic {
            assert_eq!(
                result,
                Err(FanPrepareFailure::TerminationBoxesExhausted {
                    hidden_rays: 1,
                    at_limit,
                    halvings: BOX_REFINEMENT_CAP,
                })
            );
        } else {
            assert_eq!(result, Err(FanPrepareFailure::Callback(Error::Other)));
        }
    }
}

#[test]
fn termination_height_is_strict_width_floor_plus_two_with_empty_zero() {
    assert_eq!(termination_height(&[]), Ok(UBig::ZERO));
    assert_eq!(
        termination_height(&[bx(r(1, 4), r(1, 2)), bx(r(-1, 3), r(1, 3))]),
        Ok(UBig::from(6u8))
    );
    assert_eq!(
        termination_height(&[bx(r(1, 1), r(1, 1))]),
        Err(PrepareInvalid::NonPositiveTerminationWidth)
    );
    assert_eq!(
        termination_height(&[bx(r(2, 1), r(1, 1))]),
        Err(PrepareInvalid::NonPositiveTerminationWidth)
    );
}

#[test]
#[ignore = "Needs CFTUV_FAN_PREPARE_ORACLE_PYTHON and CFTUV_FAN_PREPARE_REPLAY"]
fn actual_python_fan_prepare_outcomes_full_callbacks_prices_and_final_diagnostic() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let oracle = Command::new(std::env::var("CFTUV_FAN_PREPARE_ORACLE_PYTHON").unwrap())
        .args(["-c", ORACLE])
        .env("PYTHONPATH", repo.join("kernel/src"))
        .output()
        .unwrap();
    assert!(
        oracle.status.success(),
        "{}",
        String::from_utf8_lossy(&oracle.stderr)
    );
    let oracle = String::from_utf8(oracle.stdout).unwrap();
    let mut input = String::new();
    let mut expected = Vec::new();
    let mut names = Vec::new();
    for line in oracle.lines() {
        if let Some(line) = line.strip_prefix("IN\t") {
            names.push(line.split('\t').nth(1).unwrap().to_owned());
            input.push_str(line);
            input.push('\n');
        } else if let Some(line) = line.strip_prefix("OUT\t") {
            expected.push(line.to_owned());
        } else {
            panic!("oracle output")
        }
    }
    assert!(expected.len() > 1000);
    let mut native = Command::new(std::env::var("CFTUV_FAN_PREPARE_REPLAY").unwrap())
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = native.stdin.take().unwrap();
    let writer = std::thread::spawn(move || stdin.write_all(input.as_bytes()));
    let result = native.wait_with_output().unwrap();
    writer.join().unwrap().unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let actual = String::from_utf8(result.stdout).unwrap();
    let rows: Vec<_> = actual.lines().collect();
    assert_eq!(rows.len(), expected.len());
    for (row, (actual, expected)) in rows.iter().zip(&expected).enumerate() {
        assert_eq!(*actual, expected, "case {row} {}", names[row]);
    }
    println!(
        "{} actual Python fan preparation cases: 0 mismatch in outcomes, callback/failure order, budget prices/spend-before-throw, caps 0/1/2/96 and final diagnostic distinction",
        expected.len()
    );
}
const ORACLE: &str = r#"
from decimal import Decimal
from fractions import Fraction as F
from types import SimpleNamespace as NS
import random,sys
from cftuv_envelope.reference import adaptive_density_fan as fan,adaptive_density_band as band,density_binding_values as values
from cftuv_envelope.reference.common import ReferenceGeometryError
from cftuv_envelope.reference.contracts import ReferenceOutcome
from cftuv_envelope.contracts.analysis import TurnOrientation
from cftuv_envelope.numeric import CertifiedDecimalIntervalV1 as Env,IntervalEndpointKind as Kind
old_preparation=fan._preparation_budget;old_vector=fan._vector_from_slope
class CallbackError(ValueError):pass
def f(v):v=F(v);return f'{v.numerator}/{v.denominator}'
def vt(v):return','.join(map(f,v))
def vectors(vs):return';'.join(map(vt,vs))or'_'
def env(lo,hi=None):
 e=object.__new__(Env)
 for field,value in [('lower',F(lo)),('upper',F(lo if hi is None else hi)),('lower_kind',Kind.CLOSED),('upper_kind',Kind.CLOSED),('absolute_error_bound',F(0))]:object.__setattr__(e,field,value)
 return e
def et(e):return'~'.join([f(e.lower),f(e.upper),'C'if e.lower_kind is Kind.CLOSED else'O','C'if e.upper_kind is Kind.CLOSED else'O',f(e.absolute_error_bound)])
def rec(t):return','.join([str(int(t[0])),str(t[1]),et(t[2]),et(t[3])])
def bx(b):return','.join([f(b[0]),f(b[1]),str(int(b[2])),str(b[3])])
def boxes(bs):return';'.join(map(bx,bs))or'_'
def ideal_for(records,center=F(0)):
 centers=[(F(t[1]),center*t[1])if t[0]else(center*t[1],F(t[1]))for t in records]
 return [(F(1),F(-2)),*centers,(F(1),F(2))]
def payload(op,arg):return rec(arg)if op=='SEALED'else';'.join(map(rec,arg))or'_'if op=='BOXES'else boxes(arg)

def run(op,name,arg,*,ideal=None,ordinal=1,orientation=1,q=5,cap=65536,box_cap=96,L='1',I='1',C=None,U='1',limit='false',bs=None,width=F(0),failure=0):
 fan._BOX_REFINEMENT_CAP=box_cap
 if C is None:C=[orientation]
 if ideal is None:ideal=ideal_for([arg]if op=='SEALED'else arg if op=='BOXES'else[(True,1,env(-1),env(1))]*len(arg))
 if bs is None:bs=[(F(1),-F(3,4)),(F(1),F(3,4))]
 events=[];counts=[0]*4;patterns=[L,I,C,U];budget=None
 orient=TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION if orientation==1 else TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION
 def next(kind):
  value=patterns[kind][counts[kind]%len(patterns[kind])];counts[kind]+=1;return int(value)
 def call(prefix,compute,format=str):
  if failure and len(events)+1==failure:events.append(prefix+':!CallbackError:refused');raise CallbackError('refused')
  try:value=compute()
  except (ValueError,ArithmeticError)as e:events.append(prefix+':!'+type(e).__name__+':'+str(e));raise
  events.append(prefix+':'+format(value));return value
 class Metric:
  def density_expressions(self,v):
   x=sys._getframe(1).f_locals['use_x'];call(f'D:{vt(v)}:{int(x)}',lambda:v[0 if x else 1],f);return v
 metric=Metric()
 def boundary(metric,ideal,ordinal,orientation,q):return call(f'B:{ordinal}:{fan._expected_orientation(orientation)}:{q}',lambda:bs,vectors)
 def sign(v,metric):return call('S:'+f(v),lambda:int(v>0)-int(v<0))
 def slope(v,x,metric):return call(f'T:{vt(v)}:{int(x)}',lambda:F(v[1 if x else 0],v[0 if x else 1]),f)
 def enclosure(v,metric):
  def value():
   e=env(v-width,v+width);object.__setattr__(e,'absolute_error_bound',width);return e
  return call('E:'+f(v),value,et)
 def vector(v,x,s):return call(f'V:{f(v)}:{int(x)}:{s}',lambda:old_vector(v,x,s),vt)
 def candidate(v,x,s,metric):return call(f'Q:{f(v)}:{int(x)}:{s}',lambda:old_vector(v,x,s),vt)
 def local(metric,ideal,ordinal,v,q):return call(f'L:{ordinal}:{q}:{vt(v)}',lambda:bool(next(0)),lambda v:str(int(v)))
 def neighbours(ideal,ordinal):return call(f'N:{ordinal}',lambda:(ideal[ordinal-1],ideal[ordinal+1]),lambda v:vt(v[0])+'|'+vt(v[1]))
 def inside(metric,v,p,c,following):return call(f'I:{vt(v)}|{vt(p)}|{vt(c)}|{vt(following)}',lambda:bool(next(1)),lambda v:str(int(v)))
 def cross(metric,l,r):return call(f'C:{vt(l)}|{vt(r)}',lambda:F(next(2)),f)
 def subturn(metric,l,r,q):return call(f'U:{q}:{vt(l)}|{vt(r)}',lambda:bool(next(3)),lambda v:str(int(v)))
 def final(metric,l,r,q):
  def value():
   if limit=='ref':raise ReferenceGeometryError(ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,'diagnostic_unproven')
   if limit=='other':raise CallbackError('diagnostic_failed')
   return limit=='true'
  return call(f'F:{q}:{vt(l)}|{vt(r)}',value,lambda v:str(int(v)))
 def preparation():
  nonlocal budget
  budget=old_preparation(cap);return budget
 fan._subturn_boundary_vectors=boundary;fan._sign=sign;fan._slope=slope;values._density_decimal_envelope=enclosure;fan._vector_from_slope=vector;fan._candidate_vector=candidate;fan._local_candidate=local;band.window_neighbours=neighbours;fan._inside_ordinal=inside;fan._oriented_cross=cross;fan._subturn=subturn;fan._subturn_boundary=final;fan._preparation_budget=preparation
 try:
  if op=='SEALED':result='SEALED:'+','.join(map(f,fan._sealed_local_interval(metric,ideal,ordinal,orient,q,arg)))
  elif op=='FEASIBLE':result='FEASIBLE:'+str(int(fan._box_is_feasible(metric,ideal,orient,q,arg)))
  elif op=='BOXES':result='BOXES:'+boxes(fan._termination_boxes(metric,ideal,orient,q,arg))
  elif op=='HEIGHT':result='HEIGHT:'+str(fan._termination_height(arg))
  else:raise AssertionError(op)
 except fan.DensityRationalAuthorityExhausted as e:result='ERROR:WORK:'+str(e)
 except fan.DensityTerminationBoxesExhausted as e:result='ERROR:TERMINATION:'+str(e)
 except fan.AdaptiveDensityFanInvalid as e:result='ERROR:INVALID:'+str(e)
 except (ValueError,ArithmeticError)as e:result='ERROR:CALLBACK:'+type(e).__name__+':'+str(e)
 shell,order=(budget.shell_probes,budget.order_steps)if budget else(0,0)
 fields=[op,name,str(failure),L,I,','.join(map(str,C)),U,limit,vectors(bs),f(width),vectors(ideal),payload(op,arg),str(ordinal),str(orientation),str(q),str(cap),str(box_cap)]
 print('IN\t'+'\t'.join(fields));print('OUT\t'+result+f'\tWORK:{shell},{order}\tTRACE:'+'@'.join(events))
 return len(events)

# Sealed: zero denominator skip, OPPOSITE sign boundary retained, center overlap,
# nonpoint outer endpoints, reversed/empty bounds, point-midpoint inward adjustment.
envelopes=[(env(-1),env(1)),(env(-1,-F(3,4)),env(F(3,4),1)),(env(0),env(0)),(env(1),env(-1)),(env(-2,-1),env(1,2))]
boundary_cases=[[],[(F(1),-F(1,2)),(F(1),F(1,2))],[(F(-1),F(1,2)),(F(-1),-F(1,2))],[(F(0),F(1)),(F(1),F(0))]]
for chart in [False,True]:
 for signum in [-1,1]:
  for center in [-F(1,2),F(0),F(1,2)]:
   for ei,(lower,upper)in enumerate(envelopes):
    t=(chart,signum,lower,upper);ideal=ideal_for([t],center)
    for bi,bs in enumerate(boundary_cases):
     for width in [F(0),F(1,100)]:
      for pattern in ['1','0','10']:run('SEALED',f'sealed-{chart}-{signum}-{center}-{ei}-{bi}-{width}-{pattern}',t,ideal=ideal,L=pattern,bs=bs,width=width)
for chart in [False,True]:
 for signum in [-1,1]:
  t=(chart,signum,env(-1),env(1));bs=[(F(1),-F(1,2)),(F(1),F(1,2))]if chart else[(-F(1,2),F(1)),(F(1,2),F(1))]
  count=run('SEALED',f'sealed-full-{chart}-{signum}',t,bs=bs)
  for failure in range(1,count+1):run('SEALED',f'sealed-throw-{chart}-{signum}-{failure}',t,bs=bs,failure=failure)

# Full Cartesian order, lower/upper candidates built before neighbour/inside,
# cross scan separated from subturn scan; all actual _box_is_feasible code.
for hidden in range(4):
 boxes_arg=[(-F(i+1,5),F(i+1,5),bool(i%2),[-1,1][i%2])for i in range(hidden)]
 ideal=ideal_for([(b[2],b[3],env(-1),env(1))for b in boxes_arg])
 for orientation in [-1,1]:
  for inside in ['1','0','10','10111']:
   for cross in [[orientation],[0],[orientation,orientation,0],[orientation,-orientation]]:
    for turn in ['1','0','10']:run('FEASIBLE',f'feasible-{hidden}-{orientation}-{inside}-{cross}-{turn}',boxes_arg,ideal=ideal,orientation=orientation,I=inside,C=cross,U=turn)
for hidden in [0,1,2]:
 boxes_arg=[(-F(1,2),F(1,2),True,1)]*hidden
 count=run('FEASIBLE',f'feasible-full-{hidden}',boxes_arg)
 for failure in range(1,count+1):run('FEASIBLE',f'feasible-throw-{hidden}-{failure}',boxes_arg,failure=failure)

# Exact prep fees including empty records one sequence and >=17 upfront refusal.
for hidden in [0,1,2,3,4,10,17]:
 records=[(True,1,env(-1),env(1))]*hidden
 for cap in [0,1,2,16,65536]:
  for inside in ['1','0']:
   for cross in [[1],[0]]:run('BOXES',f'boxes-{hidden}-{cap}-{inside}-{cross}',records,cap=cap,I=inside,C=cross)
for hidden in [1,2,3]:
 records=[(bool(i%2),[-1,1][i%2],env(-1),env(1))for i in range(hidden)]
 for orientation in [-1,1]:
  count=run('BOXES',f'boxes-full-{hidden}-{orientation}',records,orientation=orientation)
  for failure in range(1,count+1):run('BOXES',f'boxes-throw-{hidden}-{orientation}-{failure}',records,orientation=orientation,failure=failure)
# Несколько уточнений перед успешным proof, exact radii divisor 2^24→2^25→2^26.
for hidden in [1,2]:
 records=[(True,1,env(-1),env(1))]*hidden
 for inside,cross,turn in [('00111111111111111111111111',[1],'1'),('1',[0,0,*([1]*100)],'1'),('1',[1],'00111111111111111111111111')]:run('BOXES',f'refine-then-success-{hidden}-{inside}-{cross}-{turn}',records,I=inside,C=cross,U=turn)
# Full 96 exhaustion; ReferenceGeometryError alone becomes diagnostic undecided.
for box_cap in [0,1,2,96]:
 for hidden in [0,1,3]:
  records=[(True,1,env(-1),env(1))]*hidden
  for limit in ['true','false','ref','other']:
   for inside in ['0','1','01']:
    run('BOXES',f'custom-cap-{box_cap}-{hidden}-{limit}-{inside}',records,box_cap=box_cap,I=inside,C=[0],limit=limit)
for hidden in [0,1,3]:
 records=[(True,1,env(-1),env(1))]*hidden
 for limit in ['true','false','ref','other']:
  count=run('BOXES',f'exhaust-{hidden}-{limit}',records,C=[0],limit=limit)
  for failure in [1,2,max(1,count-1),count]:run('BOXES',f'exhaust-throw-{hidden}-{limit}-{failure}',records,C=[0],limit=limit,failure=failure)
# First invalid center stops before creating prep budget / later centers / geometry.
for index in range(3):
 records=[(True,1,env(-1),env(1))]*3;records[index]=(True,1,env(0),env(1))
 run('BOXES',f'invalid-center-{index}',records)
 for failure in range(1,2*(index+1)+1):run('BOXES',f'invalid-center-throw-{index}-{failure}',records,failure=failure)

for chart in [False,True]:
 for signum in [-1,1]:
  for lower,upper in [(F(-1),F(1)),(F(1,4),F(1,2)),(F(1),F(1)),(F(2),F(1)),(F(0),F(1,10**100))]:run('HEIGHT',f'height-{chart}-{signum}-{lower}-{upper}',[(lower,upper,chart,signum)])
run('HEIGHT','height-empty',[])
run('HEIGHT','height-valid-before-invalid',[(F(0),F(1),True,1),(F(1),F(1),False,-1)])
rng=random.Random(783546)
for number in range(100):
 hidden=rng.randrange(4);records=[]
 for i in range(hidden):
  lower,upper=sorted([F(rng.randrange(-8,9),4),F(rng.randrange(-8,9),4)])
  records.append((bool(rng.randrange(2)),rng.choice([-1,1]),env(lower),env(upper)))
 run('BOXES',f'random-{number}',records,ideal=ideal_for(records,rng.choice([-F(1,2),F(0),F(1,2)])),orientation=rng.choice([-1,1]),q=rng.randrange(2,7),cap=rng.choice([0,1,2,16,65536]),I=rng.choice(['1','0','101']),C=[rng.choice([-1,0,1])],U=rng.choice(['1','0','10']),limit=rng.choice(['true','false','ref','other']),width=F(rng.randrange(3),100),failure=rng.choice([0,0,1,3,10]))
"#;
