use cftuv_density::atlas_plan::*;
use cftuv_density::atlas_search::GlobalHeightSegment;
use cftuv_density::farey::PrimitiveVector;
use cftuv_density::{IBig, Rational, UBig};

fn r(n: i64, d: u64) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
fn envelope(value: Rational) -> DecimalEnvelope {
    DecimalEnvelope {
        lower: value.clone(),
        upper: value,
        lower_kind: EndpointKind::Closed,
        upper_kind: EndpointKind::Closed,
        absolute_error_bound: r(0, 1),
        quantum: None,
    }
}
fn piece(index: usize, cell: usize) -> ChartPiece {
    ChartPiece {
        is_piece_v1: true,
        piece_index: NumericField::Int(IBig::from(index)),
        use_x_denominator: BoolField::Bool(cell % 2 == 0),
        denominator_sign: NumericField::Int(IBig::from(if cell < 2 { 1 } else { -1 })),
        lower_slope_envelope: envelope(r(-1, 1)),
        upper_slope_envelope: envelope(r(1, 1)),
        lower_endpoint_included: BoolField::Bool(false),
        upper_endpoint_included: BoolField::Bool(false),
        slope_increases_in_ordinal_order: BoolField::Bool(cell % 2 == 0),
        pole_ownership: PoleField::Enum(if cell % 2 == 0 {
            PoleOwnership::YZero
        } else {
            PoleOwnership::XZero
        }),
    }
}
fn window() -> AtlasWindow {
    let mut a = piece(0, 0);
    a.upper_endpoint_included = BoolField::Bool(true);
    AtlasWindow {
        kind: WindowRecordKind::AtlasV1,
        ordinal: NumericField::Int(IBig::ONE),
        pieces: vec![a, piece(1, 1)],
        termination_piece_index: NumericField::Int(IBig::ZERO),
        termination_lower_slope: RatioField::Other,
        termination_upper_slope: RatioField::Other,
        certified_termination_width: RatioField::Other,
        admissible_lower_outward: RatioField::Other,
        admissible_lower_inward: RatioField::Other,
        admissible_upper_inward: RatioField::Other,
        admissible_upper_outward: RatioField::Other,
    }
}
fn record() -> LocalRecord {
    LocalRecord {
        use_x_denominator: BoolField::Bool(true),
        denominator_sign: NumericField::Int(IBig::ONE),
        lower_slope_envelope: envelope(r(-1, 1)),
        upper_slope_envelope: envelope(r(1, 1)),
    }
}

#[test]
fn structural_type_guards_keep_numeric_equality_and_late_identity() {
    let mut w = window();
    assert_eq!(validate_atlas_structure(&w), Ok(()));
    w.pieces[0].piece_index = NumericField::OtherNumber(r(0, 1));
    w.pieces[1].denominator_sign = NumericField::OtherNumber(r(1, 1));
    assert_eq!(validate_atlas_structure(&w), Ok(()));
    w.termination_piece_index = NumericField::OtherNumber(r(0, 1));
    assert_eq!(validate_atlas_structure(&w), Err(AtlasInvalid::Identity));
    w.pieces[0].lower_endpoint_included = BoolField::Bool(true);
    assert_eq!(
        validate_atlas_structure(&w),
        Err(AtlasInvalid::OuterBoundaryClosed)
    );
    w.pieces[0].lower_endpoint_included = BoolField::OtherTruthiness(false);
    assert_eq!(validate_atlas_structure(&w), Err(AtlasInvalid::Piece));
    w.kind = WindowRecordKind::Prepared;
    assert_eq!(validate_atlas_structure(&w), Err(AtlasInvalid::Cardinality));
}

#[test]
fn contains_keeps_included_point_boundary_and_truthiness_for_direct_helpers() {
    let mut p = piece(0, 0);
    let v = |x, y| PrimitiveVector(IBig::from(x), IBig::from(y));
    assert!(!piece_contains_vector(&p, &v(1, -1)));
    p.lower_endpoint_included = BoolField::OtherTruthiness(true);
    assert!(piece_contains_vector(&p, &v(1, -1)));
    p.lower_slope_envelope.lower = r(-2, 1);
    assert!(!piece_contains_vector(&p, &v(1, -1)));
    assert!(piece_contains_vector(&p, &v(2, 1)));
    assert!(!piece_contains_vector(&p, &v(0, 1)));
    assert!(!piece_contains_vector(&p, &v(-2, -1)));
    p.denominator_sign = NumericField::NonNumeric;
    assert_eq!(piece_cell_index(&p), 2);
    assert!(!piece_contains_vector(&p, &v(2, 1)));
}

#[test]
fn reachable_walks_negative_direction_first_and_stops_one_direction_only() {
    let mut w = window();
    w.pieces = (0..4).map(|i| piece(i, i)).collect();
    w.termination_piece_index = NumericField::Int(IBig::from(2));
    let mut trace = Vec::new();
    let reached = reachable_pieces(&w, |v| {
        trace.push(v.clone());
        Ok::<_, ()>(trace.len() != 2)
    })
    .unwrap();
    assert_eq!(reached, vec![1, 2, 3]);
    assert_eq!(
        trace,
        vec![
            PrimitiveVector(IBig::from(-1), IBig::ONE),
            PrimitiveVector(IBig::ONE, IBig::ONE),
            PrimitiveVector(IBig::from(-1), IBig::from(-1))
        ]
    );
    w.pieces[1].use_x_denominator = BoolField::Bool(true);
    w.pieces[1].denominator_sign = NumericField::Int(IBig::ONE);
    let failed = reachable_pieces(&w, |_| Ok::<_, ()>(true));
    assert_eq!(
        failed,
        Err(AtlasPlanFailure::Invalid(
            AtlasInvalid::ChartUnrepresentable
        ))
    );
}

#[test]
fn height_recharts_each_local_call_and_keeps_signed_ceiling_and_empty_default() {
    let boxes = vec![TerminationBox {
        lower: r(-3, 2),
        upper: r(1, 4),
        use_x_denominator: BoolField::OtherTruthiness(false),
        denominator_sign: NumericField::NonNumeric,
    }];
    let mut seen = 0;
    let value = global_termination_height(&boxes, |b| {
        seen += 1;
        assert_eq!(b.use_x_denominator, BoolField::Bool(true));
        assert_eq!(b.denominator_sign, NumericField::Int(IBig::ONE));
        Ok::<_, ()>(IBig::from(3))
    })
    .unwrap();
    assert_eq!(value, IBig::from(5));
    assert_eq!(seen, 1);
    assert_eq!(
        global_termination_height(&boxes, |_| Ok::<_, ()>(IBig::from(-3))).unwrap(),
        IBig::from(-3)
    );
    assert_eq!(
        global_termination_height(&[], |_| Err::<IBig, _>(())).unwrap(),
        IBig::ZERO
    );
}

#[derive(Default)]
struct Laws {
    trace: Vec<&'static str>,
    accept: bool,
    fail_boundary: bool,
}
impl SegmentPlanLaws for Laws {
    type Vector = Rational;
    type Scalar = Rational;
    type Error = &'static str;
    fn denominator(&mut self, v: &Rational, _: bool) -> Result<Rational, Self::Error> {
        self.trace.push("denominator");
        Ok(v.clone())
    }
    fn sign(&mut self, v: &Rational) -> Result<i8, Self::Error> {
        self.trace.push("sign");
        Ok(if v.parts().0 > &IBig::ZERO { 1 } else { -1 })
    }
    fn slope(&mut self, v: &Rational, _: bool) -> Result<Rational, Self::Error> {
        self.trace.push("slope");
        Ok(v.clone())
    }
    fn decimal_envelope(&mut self, v: &Rational) -> Result<DecimalEnvelope, Self::Error> {
        self.trace.push("envelope");
        Ok(envelope(v.clone()))
    }
    fn local_candidate(&mut self, _: usize, _: &Rational, _: usize) -> Result<bool, Self::Error> {
        self.trace.push("local");
        Ok(self.accept)
    }
    fn vector_from_slope(&mut self, v: &Rational, _: bool, _: i8) -> Result<Rational, Self::Error> {
        self.trace.push("vector");
        Ok(v.clone())
    }
    fn local_transition(
        &mut self,
        _: usize,
        _: &PrimitiveVector,
        _: usize,
    ) -> Result<bool, Self::Error> {
        self.trace.push("transition");
        Ok(self.accept)
    }
    fn subturn_boundary_vectors(
        &mut self,
        _: usize,
        _: i8,
        _: usize,
    ) -> Result<Vec<Rational>, Self::Error> {
        self.trace.push("boundary");
        if self.fail_boundary {
            Err("boundary_refused")
        } else {
            Ok(vec![])
        }
    }
}

#[test]
fn proof_shortcircuits_and_boundary_callback_precedes_identity() {
    let segment = GlobalHeightSegment {
        use_x_denominator: true,
        denominator_sign: 1,
        outer_lower: r(-1, 1),
        inner_lower: r(-1, 2),
        inner_upper: r(1, 2),
        outer_upper: r(1, 1),
    };
    let mut laws = Laws::default();
    let out = proven_segment(2, 5, &segment, &mut laws).unwrap();
    assert_eq!(laws.trace, ["vector", "local"]);
    assert_eq!(out.inner_lower, r(0, 1));
    laws.trace.clear();
    proven_segment(2, 5, &out, &mut laws).unwrap();
    assert!(laws.trace.is_empty());
    let mut w = window();
    w.termination_piece_index = NumericField::NonNumeric;
    let failure = ordinal_segments(
        1,
        -1,
        5,
        &Window::Atlas(w.clone()),
        &record(),
        &[r(-1, 1), r(-1, 2), r(1, 2), r(1, 1)],
        &mut laws,
    );
    assert_eq!(
        failure,
        Err(AtlasPlanFailure::Invalid(AtlasInvalid::Identity))
    );
    assert_eq!(laws.trace, ["boundary"]);
    laws.fail_boundary = true;
    assert_eq!(
        ordinal_segments(
            1,
            1,
            2,
            &Window::Atlas(w),
            &record(),
            &[r(0, 1), r(0, 1), r(0, 1), r(0, 1)],
            &mut laws
        ),
        Err(AtlasPlanFailure::Callback("boundary_refused"))
    );
}

#[test]
fn strict_zip_is_late_and_earlier_callback_failure_wins() {
    let sealed = [r(-1, 1), r(-1, 2), r(1, 2), r(1, 1)];
    let mut laws = Laws {
        accept: true,
        ..Default::default()
    };
    assert_eq!(
        search_segments(
            1,
            2,
            &[Window::SingleChart],
            &[record(), record()],
            &[sealed.clone()],
            &mut laws
        ),
        Err(AtlasPlanFailure::StrictZip {
            argument: 2,
            relation: ZipRelation::Longer
        })
    );
    assert_eq!(laws.trace, ["vector", "local", "vector", "local"]);
    laws.trace.clear();
    laws.fail_boundary = true;
    assert_eq!(
        search_segments(
            1,
            2,
            &[Window::Atlas(window())],
            &[record(), record()],
            &[sealed],
            &mut laws
        ),
        Err(AtlasPlanFailure::Callback("boundary_refused"))
    );
    assert_eq!(laws.trace, ["boundary"]);
}

#[test]
fn malformed_typed_chart_is_internal_and_checked_only_after_reached_callbacks() {
    let sealed = [r(-1, 1), r(-1, 2), r(1, 2), r(1, 1)];
    let mut laws = Laws::default();
    let mut input = record();
    input.use_x_denominator = BoolField::OtherTruthiness(true);
    assert_eq!(
        ordinal_segments(1, 1, 2, &Window::SingleChart, &input, &sealed, &mut laws),
        Err(AtlasPlanFailure::UnsupportedChartDescriptor)
    );
    assert!(laws.trace.is_empty());
    let mut w = window();
    w.pieces[0].use_x_denominator = BoolField::OtherTruthiness(true);
    assert_eq!(
        ordinal_segments(1, 1, 2, &Window::Atlas(w), &record(), &sealed, &mut laws),
        Err(AtlasPlanFailure::UnsupportedChartDescriptor)
    );
    assert_eq!(laws.trace, ["boundary", "transition"]);
    let mut input = record();
    input.denominator_sign = NumericField::OtherNumber(r(3, 2));
    assert_eq!(
        ordinal_segments(1, 1, 2, &Window::SingleChart, &input, &sealed, &mut laws),
        Err(AtlasPlanFailure::UnsupportedDenominatorDescriptor)
    );
}

#[test]
#[ignore = "Needs CFTUV_ATLAS_PLAN_ORACLE_PYTHON and CFTUV_ATLAS_PLAN_REPLAY"]
fn actual_python_atlas_plan_all_fields_callbacks_errors_and_strict_zip() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let oracle = Command::new(std::env::var("CFTUV_ATLAS_PLAN_ORACLE_PYTHON").unwrap())
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
    assert!(
        expected.len() > 1000,
        "insufficient actual oracle cases: {}",
        expected.len()
    );
    let mut native = Command::new(std::env::var("CFTUV_ATLAS_PLAN_REPLAY").unwrap())
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
        "{} actual Python Atlas preparation cases: 0 mismatch in all fields, outputs, invalid contracts, full callback traces and throw-at-k/late zip ordering",
        expected.len()
    );
}

const ORACLE: &str = r#"
from dataclasses import replace
from decimal import Decimal
from fractions import Fraction as F
from types import SimpleNamespace as NS
import random,sys
from cftuv_envelope.reference import adaptive_density_atlas as a
from cftuv_envelope.reference import adaptive_density_fan as fan
from cftuv_envelope.contracts.envelopes import AdaptiveRationalFanOrdinalWindowAtlasV1 as W, AdaptiveRationalFanProjectiveChartPieceV1 as P, AdaptiveProjectivePoleOwnershipV1 as Pole
from cftuv_envelope.numeric import CertifiedDecimalIntervalV1 as Env,IntervalEndpointKind as Kind

def f(value):
 value=F(value);return f'{value.numerator}/{value.denominator}'
def num(value):
 if type(value)is int:return'i'+str(value)
 if isinstance(value,(bool,F,Decimal)):return'n'+f(value)
 return'x'
def boolean(value):return('b'if type(value)is bool else't')+str(int(bool(value)))
def rawenv(lo,hi=None,lk=Kind.CLOSED,uk=Kind.CLOSED,error=0):
 result=object.__new__(Env)
 for name,value in [('lower',F(lo)),('upper',F(lo if hi is None else hi)),('lower_kind',lk),('upper_kind',uk),('absolute_error_bound',F(error))]:object.__setattr__(result,name,value)
 return result
def ev(value):
 kind=lambda k:'C'if k is Kind.CLOSED else'O'if k is Kind.OPEN else'X'
 return'~'.join([f(value.lower),f(value.upper),kind(value.lower_kind),kind(value.upper_kind),f(value.absolute_error_bound)])
def pair(value):return'*'.join(num(v)for v in value)if type(value)is tuple and len(value)==2 else'_'
def pc(p):
 pole={Pole.NONE:'N',Pole.X_ZERO:'X',Pole.Y_ZERO:'Y'}.get(p.pole_ownership,'O')if type(p.pole_ownership)is Pole else'O'
 return','.join([str(int(type(p)is P)),num(p.piece_index),boolean(p.use_x_denominator),num(p.denominator_sign),ev(p.lower_slope_envelope),ev(p.upper_slope_envelope),boolean(p.lower_endpoint_included),boolean(p.upper_endpoint_included),boolean(p.slope_increases_in_ordinal_order),pole])
metadata=['termination_lower_slope','termination_upper_slope','certified_termination_width','admissible_lower_outward','admissible_lower_inward','admissible_upper_inward','admissible_upper_outward']
def win(w):
 if w is None:return'-'
 kind='A'if type(w)is W else'P'if type(w)is a.AtlasWindowPrepared else'O'
 return'|'.join([kind,num(getattr(w,'ordinal',0)),num(w.termination_piece_index),*[pair(getattr(w,name,None))for name in metadata],';'.join(pc(p)for p in w.pieces)or'_'])
def rec(t):return','.join([boolean(t[0]),num(t[1]),ev(t[2]),ev(t[3])])
def vt(v):return','.join(f(x)for x in v)
def vectors(vs):return';'.join(vt(v)for v in vs)or'_'
def bt(b):return','.join(map(f,b))
def seg(s):return','.join([str(int(s.use_x_denominator)),str(s.denominator_sign),*map(f,[s.outer_lower,s.inner_lower,s.inner_upper,s.outer_upper])])
def segs(items):return';'.join(map(seg,items))or'_'
def make(start=0,step=1,count=3,termination=1,ownership=0):
 pieces=[]
 for index in range(count):
  cell=(start+index*step)%4;use_x=cell%2==0
  pieces.append(P(index,use_x,1 if cell<2 else-1,rawenv(-1),rawenv(1),False,False,use_x if step==1 else not use_x,Pole.Y_ZERO if use_x else Pole.X_ZERO))
 for index in range(count-1):
  left,right=pieces[index:index+2]
  pieces[index]=replace(left,**{('upper_endpoint_included'if left.slope_increases_in_ordinal_order else'lower_endpoint_included'):bool((index+ownership)%2)})
  pieces[index+1]=replace(right,**{('lower_endpoint_included'if right.slope_increases_in_ordinal_order else'upper_endpoint_included'):not bool((index+ownership)%2)})
 return W(1,tuple(pieces),termination,*[(0,1)]*7)
base=make();record=(True,1,rawenv(-1),rawenv(1));sealed=(F(-1),-F(1,2),F(1,2),F(1))
boundaries=[(F(1),F(-1,2)),(F(1),F(1,2)),(F(-1),F(-1,2)),(F(0),F(1)),(F(1),F(0))]

def payload(op,args):
 if op in ['VALIDATE','TERM','REACH']:return[win(args[0])]
 if op in ['END','CELL','BRACKET']:return[pc(args[0])]
 if op=='CONTAINS':return[pc(args[0]),vt(args[1])]
 if op=='HEIGHT':return[';'.join(','.join([f(lo),f(hi),boolean(x),num(sign)])for lo,hi,x,sign in args[0])or'_']
 if op=='PROVEN':return[seg(args[0]),str(args[1]),str(args[2])]
 if op=='ORDINAL':return[win(args[0]),rec(args[1]),bt(args[2]),*map(str,args[3:])]
 if op=='SEARCH':return['@'.join(map(win,args[0]))or'_','@'.join(map(rec,args[1]))or'_','@'.join(map(bt,args[2]))or'_',str(args[3]),str(args[4])]
 raise AssertionError(op)

def run(op,name,*args,pattern='1',failure=0,bs=None,width=F(0),height=3):
 events=[];locals_count=0;bs=boundaries if bs is None else bs
 def call(prefix,compute,format=str):
  if failure and len(events)+1==failure:events.append(prefix+':!refused');raise ValueError('refused')
  value=compute();events.append(prefix+':'+format(value));return value
 class Metric:
  def density_expressions(self,v):
   use_x=sys._getframe(1).f_locals['use_x']
   call(f'D:{vt(v)}:{int(bool(use_x))}',lambda:v[0 if use_x else 1],f)
   return v
 metric=Metric()
 def sign(value,metric):return call('S:'+f(value),lambda:(value>0)-(value<0))
 def slope(v,use_x,metric):return call(f'T:{vt(v)}:{int(bool(use_x))}',lambda:F(v[1 if use_x else 0],v[0 if use_x else 1]),f)
 def envelope(value,metric):return call('E:'+f(value),lambda:rawenv(value-width,value+width,error=width),ev)
 def local(metric,ideal,ordinal,v,q):
  nonlocal locals_count
  result=pattern[locals_count%len(pattern)]=='1';locals_count+=1
  return call(f'L:{ordinal}:{q}:{vt(v)}',lambda:result,lambda x:str(int(x)))
 def vector(value,use_x,denominator_sign):return call(f'V:{f(value)}:{int(bool(use_x))}:{denominator_sign}',lambda:fan._vector_from_slope(value,use_x,denominator_sign),vt)
 def boundary(metric,ideal,ordinal,orientation,q):return call(f'B:{ordinal}:{orientation}:{q}',lambda:bs,vectors)
 def localheight(boxes):
  (lo,hi,x,sign),=boxes
  return call(f'H:{f(lo)},{f(hi)}:{int(x)}:{sign}',lambda:height)
 laws=a.SegmentPlanLaws(sign,slope,envelope,local,vector,boundary,ValueError)
 try:
  if op=='VALIDATE':a.validate_atlas_structure(args[0],invalid=ValueError);result='VALID'
  elif op=='TERM':p=a.termination_piece(args[0],invalid=ValueError);result='TERM:'+str(next(i for i,x in enumerate(args[0].pieces)if x is p))
  elif op=='END':result='END:'+';'.join(map(ev,a.piece_ordered_endpoints(args[0])))
  elif op=='CELL':result='CELL:'+str(a._piece_cell_index(args[0]))
  elif op=='CONTAINS':result='CONTAINS:'+str(int(a.piece_contains_vector(*args)))
  elif op=='HEIGHT':result='HEIGHT:'+str(a.global_termination_height(args[0],local_height=localheight))
  elif op=='REACH':result='REACH:'+','.join(map(str,a.reachable_pieces(args[0],admissible=lambda v:local(metric,None,1,v,2),invalid=ValueError)))
  elif op=='BRACKET':result='BRACKET:'+bt(a.piece_bracket(metric,bs,args[0],laws))
  elif op=='PROVEN':result='SEGMENTS:'+segs([a.proven_segment(metric,None,args[1],args[2],args[0],laws)])
  elif op=='ORDINAL':w,t,b,ordinal,orientation,q=args;result='SEGMENTS:'+segs(a.ordinal_segments(metric,None,ordinal,orientation,q,w,t,b,laws))
  elif op=='SEARCH':ws,ts,ss,orientation,q=args;plans=a.search_segments(metric,None,orientation,q,ws,ts,ss,laws);result='PLANS:'+('#'.join(map(segs,plans))or'-')
  else:raise AssertionError(op)
 except (ValueError,ZeroDivisionError)as error:result='ERROR:'+str(error)
 fields=[op,name,str(failure),pattern,vectors(bs),f(width),str(height),*payload(op,args)]
 print('IN\t'+'\t'.join(fields));print('OUT\t'+result+'\tTRACE:'+'@'.join(events))
 return len(events)

# Каждая комбинация cardinality/cell/direction/start; фактические helpers oracle.
for start in range(4):
 for step in [-1,1]:
  for count in [2,3,4]:
   for termination in range(count):
    for ownership in [0,1]:
     w=make(start,step,count,termination,ownership);name=f'{start}-{step}-{count}-{termination}-{ownership}'
     run('VALIDATE','valid-'+name,w);run('TERM','term-'+name,w)
     for pattern in ['1','0','10','110']:
      run('REACH','reach-'+name+'-'+pattern,w,pattern=pattern)
      run('ORDINAL','ordinal-'+name+'-'+pattern,w,record,sealed,2,step,5,pattern=pattern,width=F(1,100))
     for p in w.pieces:
      run('END','end-'+name+'-'+str(p.piece_index),p);run('CELL','cell-'+name+'-'+str(p.piece_index),p)

# Все 10 source window fields: семь opaque metadata + ordinal намеренно ignored.
for field in ['ordinal',*metadata]:
 for value in [object(),(-2,0),(True,F(3,2))]:
  w=replace(base,**{field:value});run('VALIDATE','ignored-window-'+field+str(type(value)),w);run('TERM','ignored-term-'+field+str(type(value)),w)
for kind in ['Prepared','Foreign']:
 w=a.AtlasWindowPrepared(base.pieces,base.termination_piece_index,None)if kind=='Prepared'else NS(**{name:getattr(base,name)for name in ['ordinal','pieces','termination_piece_index',*metadata]})
 run('VALIDATE',kind,w);run('TERM','direct-'+kind,w);run('ORDINAL','ordinal-'+kind,w,record,sealed,1,1,2)
for pieces in [(),base.pieces[:1],base.pieces*3]:run('VALIDATE','cardinality-'+str(len(pieces)),replace(base,pieces=pieces))
for index in [-1,3,10**100,True,F(0),Decimal(0),object()]:
 w=replace(base,termination_piece_index=index)
 for op in ['VALIDATE','TERM','REACH']:run(op,'identity-'+op+'-'+num(index),w)
 run('ORDINAL','boundary-before-identity-'+num(index),w,record,sealed,1,1,2)
 run('ORDINAL','boundary-error-before-identity-'+num(index),w,record,sealed,1,1,2,failure=1)

# Все 9 piece fields + каждый endpoint envelope field, включая отсутствие extra guards.
piece_fields={
 'piece_index':[1,True,F(0),Decimal(0),object()],
 'use_x_denominator':[False,0,2,object()],
 'denominator_sign':[0,2,-1,True,F(1),Decimal(1),object()],
 'lower_endpoint_included':[True,0,2],
 'upper_endpoint_included':[True,0,2],
 'slope_increases_in_ordinal_order':[False,0,2],
 'pole_ownership':[Pole.NONE,Pole.X_ZERO,'Y_ZERO',object()],
 'lower_slope_envelope':[rawenv(-1,0),rawenv(0),rawenv(1),rawenv(-2,-1),rawenv(0,1)],
 'upper_slope_envelope':[rawenv(0),rawenv(-1),rawenv(1,2),rawenv(-1,0)],
}
for field,values in piece_fields.items():
 for number,value in enumerate(values):
  p=replace(base.pieces[0],**{field:value});w=replace(base,pieces=(p,*base.pieces[1:]));name=field+'-'+str(number)
  run('VALIDATE','piece-'+name,w);run('END','direct-end-'+name,p);run('CELL','direct-cell-'+name,p)
  for v in [(0,1),(1,0),(1,-1),(1,1),(-1,1),(2,1)]:run('CONTAINS','contains-'+name+'-'+str(v),p,v)
p=NS(**{field:getattr(base.pieces[0],field)for field in piece_fields})
run('VALIDATE','wrong-piece-type',replace(base,pieces=(p,*base.pieces[1:])))
for field in ['lower_slope_envelope','upper_slope_envelope']:
 for endpoint in ['lower','upper','lower_kind','upper_kind','absolute_error_bound']:
  values=[F(-2),F(0),F(1),F(2)]if endpoint in ['lower','upper']else[Kind.OPEN,object()]if'kind'in endpoint else[F(-1),F(100)]
  for number,value in enumerate(values):
   old=getattr(base.pieces[0],field);e=rawenv(old.lower,old.upper,old.lower_kind,old.upper_kind,old.absolute_error_bound);object.__setattr__(e,endpoint,value)
   p=replace(base.pieces[0],**{field:e});w=replace(base,pieces=(p,*base.pieces[1:]));name=field+'-'+endpoint+'-'+str(number)
   run('VALIDATE','envelope-'+name,w);run('END','envelope-end-'+name,p)

# guard precedence: exact source refusal, не pre-validation другой операции.
for badindex in [-1,True]:
 for p in [replace(base.pieces[0],piece_index=1),replace(base.pieces[0],pole_ownership=Pole.NONE),replace(base.pieces[0],lower_endpoint_included=True)]:
  run('VALIDATE','precedence-before-identity-'+str(badindex)+pc(p),replace(base,termination_piece_index=badindex,pieces=(p,*base.pieces[1:])))
for cell in [0,2,3]:
 p=replace(base.pieces[1],use_x_denominator=cell%2==0,denominator_sign=1 if cell<2 else-1,pole_ownership=Pole.Y_ZERO if cell%2==0 else Pole.X_ZERO)
 w=replace(base,pieces=(base.pieces[0],p,*base.pieces[2:]));run('VALIDATE','bad-order-'+str(cell),w);run('REACH','invalid-transition-'+str(cell),w)
for inc in [False,True]:
 p=base.pieces[1];field='upper_endpoint_included'if p.slope_increases_in_ordinal_order else'lower_endpoint_included'
 run('VALIDATE','pole-owner-'+str(inc),replace(base,pieces=(base.pieces[0],replace(p,**{field:inc}),*base.pieces[2:])))
for point in [F(1,2),F(-1),F(1)]:
 first=replace(base.pieces[0],upper_slope_envelope=rawenv(point));second=replace(base.pieces[1],upper_slope_envelope=rawenv(point))
 run('VALIDATE','shared-pole-point-'+f(point),replace(base,pieces=(first,second,*base.pieces[2:])))

# Bracket: mismatch/zero denom skip, middle equality, stable FIRST tie and interval overlap.
for cell in range(4):
 p=make(cell,1,2,0).pieces[0]
 for bs in [[],boundaries,list(reversed(boundaries)),[(1,1),(2,2)],[(1,0),(0,1)],[(1,-F(1,2)),(2,-1),(1,F(1,2)),(2,1)],[(1,F(1,100)),(1,-F(1,100))]]:
  for width in [F(0),F(1,100),F(1,2)]:run('BRACKET',f'bracket-{cell}-{bs}-{width}',p,bs=bs,width=width)
# Проверка FIRST при равном key, но разных outer endpoints.
tie_piece=replace(base.pieces[0],lower_slope_envelope=rawenv(-F(3,4),-F(1,2)),upper_slope_envelope=rawenv(F(1,2),F(3,4)))
run('BRACKET','stable-first-tie',tie_piece,bs=[(1,-F(1,2)),(1,F(1,2))])

# Height: original chart/sign ignored; каждый local callback отдельно; signed ceil, большие числа.
height_boxes=[[],[(F(-3,2),F(1,4),False,-1)],[(F(-3),F(2),2,object()),(F(1,4),F(1,2),False,True)],[(F(-10**100),F(1),False,0)],[(F(1),F(1),True,1)]]
for boxes in height_boxes:
 for height in [-5,-1,0,1,3,10**80]:
  count=run('HEIGHT','height-'+str(boxes)+'-'+str(height),boxes,height=height)
  for failure in range(1,count+1):run('HEIGHT','height-refusal-'+str(boxes)+'-'+str(height)+'-'+str(failure),boxes,height=height,failure=failure)

# Inner proof: оба ends in order, первый false/refusal останавливает второй.
for chart in [False,True]:
 for signum in [-1,1]:
  for lo,hi in [(F(-1,2),F(1,2)),(F(0),F(0)),(F(1),F(-1)),(F(-1),F(1)),(F(1,4),F(1,2))]:
   s=a.GlobalHeightSegment(chart,signum,F(-1),lo,hi,F(1))
   for pattern in ['1','0','10','01']:
    count=run('PROVEN',f'proven-{chart}-{signum}-{lo}-{hi}-{pattern}',s,3,5,pattern=pattern)
    for failure in range(1,count+1):run('PROVEN',f'proven-throw-{chart}-{signum}-{lo}-{hi}-{pattern}-{failure}',s,3,5,pattern=pattern,failure=failure)

# Каждый callback position полноценного ordinal, включая late bracket до любого proof.
for w in [base,make(3,-1,4,2),None]:
 for pattern in ['1','101','011','0']:
  count=run('ORDINAL','full-order-'+str(w)+'-'+pattern,w,record,sealed,4,-1,5,pattern=pattern,width=F(1,100))
  for failure in range(1,count+1):run('ORDINAL','full-order-throw-'+str(w)+'-'+pattern+'-'+str(failure),w,record,sealed,4,-1,5,pattern=pattern,width=F(1,100),failure=failure)
# Single chart читает только record[0:2]; остальные поля не обрабатываются.
for t in [(True,1,rawenv(999),rawenv(-999)),(False,-1,rawenv(0),rawenv(0)),(True,Decimal(1),rawenv(-1),rawenv(1))]:run('ORDINAL','single-record-fields-'+rec(t),None,t,sealed,1,1,2)

# zip(strict=True) читает ordinals до первой mismatch, не предварительно lengths.
for nw in range(4):
 for nr in range(4):
  for ns in range(4):
   ws=[base,None,make(2,-1,2,1)][:nw];ts=[record]*nr;ss=[sealed]*ns
   count=run('SEARCH',f'zip-{nw}-{nr}-{ns}',ws,ts,ss,-1,5,pattern='101')
   for failure in [1,2,3,count]:
    if failure>0:run('SEARCH',f'zip-throw-{nw}-{nr}-{ns}-{failure}',ws,ts,ss,-1,5,pattern='101',failure=failure)

rng=random.Random(513948)
for number in range(150):
 w=make(rng.randrange(4),rng.choice([-1,1]),rng.randrange(2,5),0);w=replace(w,termination_piece_index=rng.randrange(len(w.pieces)))
 bs=[(F(rng.choice([-4,-2,-1,0,1,2,4])),F(rng.randrange(-4,5),rng.choice([1,2,4])))for _ in range(rng.randrange(7))]
 bounds=[F(rng.randrange(-8,9),4)for _ in range(4)];bounds[0],bounds[3]=min(bounds[0],bounds[3]),max(bounds[0],bounds[3])
 run('ORDINAL',f'random-{number}',w,record,bounds,number%4+1,rng.choice([-1,1]),rng.randrange(2,7),pattern=rng.choice(['1','0','10','10111']),bs=bs,width=F(rng.randrange(3),100),failure=rng.choice([0,0,1,5,10,20]))

# Фактический Python Atlas builder, не копия формул port: exact bool/±1 на выходе.
# Его построение остаётся за пределами Rust task; generated output идёт в plan replay.
class BuilderMetric:
 def density_expressions(self,v):return v
bm=BuilderMetric()
def rotate(v,k,orientation):
 x,y=v[0],v[1]*orientation
 for _ in range(k):x,y=-y,x
 return x,y
for k in range(4):
 for orientation in [-1,1]:
  for left,right in [((2,-1),(-1,2)),((1,-2),(-1,2)),((-1,-2),(-2,1))]:
   center=rotate((1,0),k,orientation);left=rotate(left,k,orientation);right=rotate(right,k,orientation)
   prepared=a.build_atlas_window_record(bm,center,left,right,sign=lambda v,m:int(v>0)-int(v<0),slope=lambda v,x,m:F(v[1 if x else 0],v[0 if x else 1]),vector=lambda x,y,m:(x,y),decimal_envelope=lambda v,m:rawenv(v),invalid=ValueError)
   assert type(prepared.local_record[0])is bool and type(prepared.local_record[1])is int and prepared.local_record[1]in(-1,1)
   for p in prepared.pieces:
    assert type(p.use_x_denominator)is bool and type(p.denominator_sign)is int and p.denominator_sign in(-1,1)
    assert all(type(getattr(p,field))is bool for field in ['lower_endpoint_included','upper_endpoint_included','slope_increases_in_ordinal_order'])
   w=W(1,prepared.pieces,prepared.termination_piece_index,*[(0,1)]*7)
   run('VALIDATE',f'actual-atlas-builder-{k}-{orientation}-{left}-{right}',w)
   run('ORDINAL',f'actual-atlas-builder-plan-{k}-{orientation}-{left}-{right}',prepared,prepared.local_record,sealed,1,orientation,5)

# Фактический product record builder _window_envelopes и _chart_for_window.
import sympy as sp
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference import adaptive_density_band as band
identity=((sp.Integer(1),sp.Integer(0)),(sp.Integer(0),sp.Integer(1)))
for k in range(4):
 for orientation in [-1,1]:
  for owner in [-1,1]:
   for law in [band.WINDOW_LAW_VORONOI,band.WINDOW_LAW_NARROW_BAND]:
    metric=ExactPlanarMetric(identity,identity,owner)
    vs=[rotate((F(3,5),-F(4,5)),k,orientation),rotate((1,0),k,orientation),rotate((F(3,5),F(4,5)),k,orientation)]
    ideal=fan._IdealTuple([fan._vector(*v,metric)for v in vs]);ideal.metric=metric;ideal.window_law=law;ideal.band_orientation=orientation*owner;ideal.band_cache={}
    ws,rs=fan._window_envelopes(ideal);assert len(rs)==1
    local=fan._local_record(rs[0]);assert type(local[0])is bool and type(local[1])is int and local[1]in(-1,1)
    run('ORDINAL',f'actual-product-record-{k}-{orientation}-{owner}-{law}',None,local,sealed,1,orientation,5)
"#;
