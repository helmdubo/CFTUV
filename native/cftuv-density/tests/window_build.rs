use cftuv_density::atlas_plan::DecimalEnvelope;
use cftuv_density::atlas_plan::{
    AtlasInvalid, BoolField, NumericField, Window, WindowRecordKind, validate_atlas_structure,
};
use cftuv_density::window_build::*;
use cftuv_density::{IBig, Rational, UBig};
fn r(n: i64, d: u64) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
type Vector = (Rational, Rational);
fn v(x: i64, y: i64) -> Vector {
    (r(x, 1), r(y, 1))
}
fn add(a: &Rational, b: &Rational) -> Rational {
    let (an, ad) = a.parts();
    let (bn, bd) = b.parts();
    Rational::new(an * bd + bn * ad, ad * bd).unwrap()
}
fn neg(a: &Rational) -> Rational {
    let (n, d) = a.parts();
    Rational::new(-n, d.clone()).unwrap()
}
fn mul(a: &Rational, b: &Rational) -> Rational {
    let (an, ad) = a.parts();
    let (bn, bd) = b.parts();
    Rational::new(an * bn, ad * bd).unwrap()
}
fn cmp(a: &Rational, b: &Rational) -> std::cmp::Ordering {
    let (an, ad) = a.parts();
    let (bn, bd) = b.parts();
    (an * bd).cmp(&(bn * ad))
}
#[derive(Clone, Debug, PartialEq, Eq)]
enum Error {
    Other,
    Chart,
}
struct Laws {
    trace: Vec<&'static str>,
    fail_at: usize,
    class: Error,
    width: Rational,
    ideal: Vec<Vector>,
}
impl Laws {
    fn new(ideal: Vec<Vector>) -> Self {
        Self {
            trace: vec![],
            fail_at: 0,
            class: Error::Other,
            width: r(0, 1),
            ideal,
        }
    }
    fn emit<T>(&mut self, name: &'static str, value: T) -> Result<T, Error> {
        self.trace.push(name);
        if self.fail_at == self.trace.len() {
            Err(self.class.clone())
        } else {
            Ok(value)
        }
    }
}
impl WindowBuildLaws for Laws {
    type Vector = Vector;
    type Scalar = Rational;
    type Error = Error;
    fn window_neighbours(&mut self, o: usize) -> Result<(Vector, Vector), Error> {
        self.emit(
            "neighbours",
            (self.ideal[o - 1].clone(), self.ideal[o + 1].clone()),
        )
    }
    fn coordinates(&mut self, v: &Vector) -> Result<Vector, Error> {
        self.emit("coordinates", v.clone())
    }
    fn add(&mut self, a: &Rational, b: &Rational) -> Result<Rational, Error> {
        self.emit("add", add(a, b))
    }
    fn sub(&mut self, a: &Rational, b: &Rational) -> Result<Rational, Error> {
        self.emit("sub", add(a, &neg(b)))
    }
    fn mul(&mut self, a: &Rational, b: &Rational) -> Result<Rational, Error> {
        self.emit("mul", mul(a, b))
    }
    fn sign(&mut self, a: &Rational) -> Result<i8, Error> {
        self.emit(
            "sign",
            if a.parts().0 < &IBig::ZERO {
                -1
            } else if a.parts().0 > &IBig::ZERO {
                1
            } else {
                0
            },
        )
    }
    fn vector(&mut self, x: Rational, y: Rational) -> Result<Vector, Error> {
        self.emit("vector", (x, y))
    }
    fn integer_vector(&mut self, x: &IBig, y: &IBig) -> Result<Vector, Error> {
        self.emit(
            "integer_vector",
            (Rational::integer(x.clone()), Rational::integer(y.clone())),
        )
    }
    fn slope(&mut self, v: &Vector, x: bool) -> Result<Rational, Error> {
        let (a, b) = if x { (&v.1, &v.0) } else { (&v.0, &v.1) };
        let (an, ad) = a.parts();
        let (bn, bd) = b.parts();
        let numerator = an * bd;
        self.emit(
            "slope",
            Rational::new(
                if bn < &IBig::ZERO {
                    -numerator
                } else {
                    numerator
                },
                ad * cftuv_radical::arith::magnitude(bn),
            )
            .unwrap(),
        )
    }
    fn decimal_envelope(&mut self, a: &Rational) -> Result<DecimalEnvelope, Error> {
        let mut e = exact_decimal_interval(IBig::ZERO);
        e.lower = add(a, &neg(&self.width));
        e.upper = add(a, &self.width);
        e.absolute_error_bound = self.width.clone();
        self.emit("envelope", e)
    }
    fn equals_integer(&mut self, a: &Rational, i: i8) -> Result<bool, Error> {
        self.emit("equals", a == &r(i.into(), 1))
    }
    fn integer_value(&mut self, a: &Rational) -> Result<IBig, Error> {
        let (n, d) = a.parts();
        self.emit("int", n / d)
    }
    fn rational_lt_scalar(&mut self, r: &Rational, s: &Rational) -> Result<bool, Error> {
        self.emit("lower_compare", cmp(r, s).is_lt())
    }
    fn scalar_lt_rational(&mut self, s: &Rational, r: &Rational) -> Result<bool, Error> {
        self.emit("upper_compare", cmp(s, r).is_lt())
    }
    fn is_chart_unrepresentable(e: &Error) -> bool {
        *e == Error::Chart
    }
}

#[test]
fn diagonal_cell_uses_x_and_zero_turn_still_evaluates_following_turn() {
    let mut laws = Laws::new(vec![]);
    assert_eq!(cell(&v(1, 1), &mut laws).unwrap(), (0, true, 1));
    assert_eq!(
        laws.trace,
        ["coordinates", "mul", "mul", "sub", "sign", "sign"]
    );
    let mut laws = Laws::new(vec![]);
    assert_eq!(
        cell_path(&v(1, 0), &v(1, 0), &v(0, 1), &mut laws),
        Err(WindowBuildFailure::ChartUnrepresentable)
    );
    assert_eq!(
        laws.trace,
        [
            "coordinates",
            "coordinates",
            "coordinates",
            "mul",
            "mul",
            "sub",
            "sign",
            "mul",
            "mul",
            "sub",
            "sign"
        ]
    );
}

#[test]
fn chart_reads_all_three_values_before_all_three_signs_even_when_one_is_zero() {
    let mut laws = Laws::new(vec![]);
    assert_eq!(
        chart_for_window(&v(1, 0), &v(0, -1), &v(1, 1), &mut laws),
        Err(WindowBuildFailure::ChartUnrepresentable)
    );
    assert_eq!(
        laws.trace,
        [
            "coordinates",
            "mul",
            "mul",
            "sub",
            "sign",
            "coordinates",
            "coordinates",
            "coordinates",
            "sign",
            "sign",
            "sign",
            "coordinates",
            "coordinates",
            "coordinates",
            "sign",
            "sign",
            "sign"
        ]
    );
}

#[test]
fn class_specific_chart_fallback_excludes_midpoint_and_later_envelope_errors() {
    let ideal = vec![v(1, -1), v(1, 0), v(1, 1)];
    for (failure, fallback) in [(1, false), (7, false), (11, true), (19, true), (26, false)] {
        let mut laws = Laws::new(ideal.clone());
        laws.fail_at = failure;
        laws.class = Error::Chart;
        let result = window_envelopes(&ideal, &mut laws);
        if fallback {
            assert!(matches!(
                result.unwrap().records[0],
                PreparedWindowRecord::Atlas(_)
            ));
        } else {
            assert_eq!(result, Err(WindowBuildFailure::Callback(Error::Chart)));
        }
    }
    let mut laws = Laws::new(ideal.clone());
    laws.width = r(1, 2);
    assert_eq!(
        window_envelopes(&ideal, &mut laws),
        Err(WindowBuildFailure::OrdinalEnvelopesOverlap)
    );
}

#[test]
fn direct_one_cell_prepared_is_not_prematurely_validated_as_wire_atlas() {
    let mut laws = Laws::new(vec![]);
    let a = build_atlas_window_record(&v(1, 0), &v(2, -1), &v(2, 1), &mut laws).unwrap();
    assert_eq!(a.pieces.len(), 1);
    let record = PreparedWindowRecord::Atlas(a);
    assert!(record.local_record().use_x_denominator);
    let Window::Atlas(plan) = record.plan_window(1) else {
        panic!("atlas")
    };
    assert_eq!(plan.kind, WindowRecordKind::Prepared);
    assert_eq!(
        validate_atlas_structure(&plan),
        Err(AtlasInvalid::Cardinality)
    );
    for piece in &plan.pieces {
        assert!(piece.is_piece_v1);
        assert!(matches!(piece.use_x_denominator, BoolField::Bool(_)));
        assert!(matches!(piece.denominator_sign, NumericField::Int(_)));
    }
}

#[test]
fn atlas_reuses_shared_transitions_and_emits_exact_unit_points_and_piece_fields() {
    let mut laws = Laws::new(vec![]);
    let a = build_atlas_window_record(&v(1, 0), &v(-1, -2), &v(-2, 1), &mut laws).unwrap();
    assert_eq!(a.pieces.len(), 4);
    assert_eq!(a.termination_piece_index, 1);
    assert_eq!(
        laws.trace
            .iter()
            .filter(|&&s| s == "integer_vector")
            .count(),
        6
    );
    assert_eq!(a.local_record.denominator_sign, 1);
    assert_eq!(
        a.pieces[1].lower_slope_envelope,
        exact_decimal_interval(IBig::from(-1))
    );
    assert_eq!(
        a.pieces[1].upper_slope_envelope,
        exact_decimal_interval(IBig::ONE)
    );
    assert_eq!(a.pieces[1].lower_endpoint_included, BoolField::Bool(true));
    assert_eq!(a.pieces[1].upper_endpoint_included, BoolField::Bool(true));
    assert_eq!(a.pieces[0].lower_endpoint_included, BoolField::Bool(false));
    assert_eq!(a.pieces[3].upper_endpoint_included, BoolField::Bool(false));
}

#[test]
fn short_ideal_has_no_callbacks_and_exact_integer_envelopes_support_big_values() {
    for ideal in [vec![], vec![v(1, 0)], vec![v(1, 0), v(0, 1)]] {
        let mut laws = Laws::new(ideal.clone());
        let built = window_envelopes(&ideal, &mut laws).unwrap();
        assert!(built.windows.is_empty());
        assert!(built.records.is_empty());
        assert!(laws.trace.is_empty());
    }
    let number = IBig::ONE << 500usize;
    let e = exact_decimal_interval(number.clone());
    assert_eq!(e.lower, Rational::integer(number));
    assert_eq!(e.lower, e.upper);
    assert_eq!(e.absolute_error_bound, r(0, 1));
}

#[test]
#[ignore = "Needs CFTUV_WINDOW_BUILD_ORACLE_PYTHON and CFTUV_WINDOW_BUILD_REPLAY"]
fn actual_python_window_build_all_fields_full_callbacks_and_exact_fallback_scope() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let oracle = Command::new(std::env::var("CFTUV_WINDOW_BUILD_ORACLE_PYTHON").unwrap())
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
    let mut native = Command::new(std::env::var("CFTUV_WINDOW_BUILD_REPLAY").unwrap())
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
        "{} actual Python window/Atlas builder cases: 0 mismatch in every record/envelope/piece field, full fixed-operation callback traces, throw-at-k and class-specific chart-only fallback",
        expected.len()
    );
}
const ORACLE: &str = r#"
from fractions import Fraction as F
from types import SimpleNamespace as NS
import random
from cftuv_envelope.reference import adaptive_density_atlas as atlas,adaptive_density_fan as fan,adaptive_density_band as band,density_binding_values as values,density_projective as projective
from cftuv_envelope.numeric import CertifiedDecimalIntervalV1 as Env,IntervalEndpointKind as Kind
class CallbackError(ValueError):pass
def unwrap(value):return value.value if type(value)is Scalar else F(value)
def f(v):v=unwrap(v);return f'{v.numerator}/{v.denominator}'
def vt(v):return','.join(map(f,v))
def vectors(vs):return';'.join(map(vt,vs))or'_'
def et(e):return'~'.join([f(e.lower),f(e.upper),'C'if e.lower_kind is Kind.CLOSED else'O','C'if e.upper_kind is Kind.CLOSED else'O',f(e.absolute_error_bound)])
def rec(t):return','.join([str(int(t[0])),str(t[1]),et(t[2]),et(t[3])])
def piece(p):return','.join([str(p.piece_index),str(int(p.use_x_denominator)),str(p.denominator_sign),et(p.lower_slope_envelope),et(p.upper_slope_envelope),str(int(p.lower_endpoint_included)),str(int(p.upper_endpoint_included)),str(int(p.slope_increases_in_ordinal_order)),{'NONE':'N','X_ZERO':'X','Y_ZERO':'Y'}[p.pole_ownership.value]])
def record(r):return f'A!{r.termination_piece_index}!'+'@'.join(map(piece,r.pieces))+'!'+rec(r.local_record)if type(r)is atlas.AtlasWindowPrepared else'S!'+rec(r)
events=[];failure=0;failure_class='normal'
def call(prefix,compute,format=str):
 if failure and len(events)+1==failure:
  error=fan.DensityWindowChartUnrepresentable('refused')if failure_class=='chart'else CallbackError('refused')
  events.append(prefix+':!'+type(error).__name__+':'+str(error));raise error
 try:value=compute()
 except (ValueError,ArithmeticError)as error:events.append(prefix+':!'+type(error).__name__+':'+str(error));raise
 events.append(prefix+':'+format(value));return value
class Scalar:
 def __init__(self,v):self.value=F(v)
 def __add__(self,other):return call(f'A:{f(self)}|{f(other)}',lambda:Scalar(self.value+unwrap(other)),f)
 def __sub__(self,other):return call(f'R:{f(self)}|{f(other)}',lambda:Scalar(self.value-unwrap(other)),f)
 def __mul__(self,other):return call(f'M:{f(self)}|{f(other)}',lambda:Scalar(self.value*unwrap(other)),f)
 def __eq__(self,other):return call(f'Q:{f(self)}:{other}',lambda:self.value==other,lambda r:str(int(r)))
 def __int__(self):return call('J:'+f(self),lambda:int(self.value))
 def __gt__(self,other):return call(f'RL:{f(other)}|{f(self)}',lambda:unwrap(other)<self.value,lambda r:str(int(r)))
 def __lt__(self,other):return call(f'SL:{f(self)}|{f(other)}',lambda:self.value<unwrap(other),lambda r:str(int(r)))
class Metric:
 def density_expressions(self,v):return call('D:'+vt(v),lambda:(Scalar(v[0]),Scalar(v[1])),vt)
def run(op,name,input,*,width=F(0),throw=0,kind='normal'):
 global events,failure,failure_class
 events=[];failure=throw;failure_class=kind;metric=Metric()
 def sign(v,m):return call('S:'+f(v),lambda:int(unwrap(v)>0)-int(unwrap(v)<0))
 def vector(x,y,m):return call('VI:'+str(x)+','+str(y)if type(x)is int and type(y)is int else'V:'+vt((x,y)),lambda:(unwrap(x),unwrap(y)),vt)
 def slope(v,x,m):
  def value():
   if v[0 if x else 1]==0:raise fan.AdaptiveDensityFanInvalid('fan chart denominator is zero')
   return Scalar(F(v[1 if x else 0],v[0 if x else 1]))
  return call(f'T:{vt(v)}:{int(x)}',value,f)
 def envelope(v,m):
  def value():
   e=object.__new__(Env)
   for key,item in [('lower',unwrap(v)-width),('upper',unwrap(v)+width),('lower_kind',Kind.CLOSED),('upper_kind',Kind.CLOSED),('absolute_error_bound',width)]:object.__setattr__(e,key,item)
   return e
  return call('E:'+f(v),value,et)
 def neighbours(ideal,ordinal):return call(f'N:{ordinal}',lambda:(ideal[ordinal-1],ideal[ordinal+1]),lambda v:vt(v[0])+'|'+vt(v[1]))
 fan._sign=sign;projective._sign=sign;fan._vector=vector;fan._slope=slope;values._density_decimal_envelope=envelope;band.window_neighbours=neighbours
 try:
  if op=='POINT':result='POINT:'+et(atlas._exact_decimal_interval(input))
  elif op=='CELL':c,x,s=atlas._cell(metric,input[0],sign=sign,invalid=fan.DensityWindowChartUnrepresentable);result=f'CELL:{c},{int(x)},{s}'
  elif op=='CHART':x,s=projective._chart_for_window(metric,input[1],input[0],input[2]);result=f'CHART:{int(x)},{s}'
  elif op=='PATH':result='PATH:'+','.join(map(str,atlas._cell_path(metric,*input,sign=sign,invalid=fan.DensityWindowChartUnrepresentable)))
  elif op=='ATLAS':result='ATLAS:'+record(atlas.build_atlas_window_record(metric,input[1],input[0],input[2],sign=sign,slope=slope,vector=vector,decimal_envelope=envelope,invalid=fan.DensityWindowChartUnrepresentable))
  elif op=='WINDOWS':
   ideal=fan._IdealTuple(input);ideal.metric=metric;ideal.window_law=band.WINDOW_LAW_VORONOI;ideal.band_cache={};ideal.band_orientation=1
   ws,rs=fan._window_envelopes(ideal);result='WINDOWS:'+(';'.join(vt(a)+'|'+vt(b)for a,b in ws)or'_')+'\tRECORDS:'+('#'.join(map(record,rs))or'_')
  else:raise AssertionError(op)
 except fan.DensityWindowChartUnrepresentable as error:result='ERROR:CHART:'+str(error)if str(error)=='DENSITY_WINDOW_CHART_UNREPRESENTABLE'else'ERROR:CALLBACK:'+type(error).__name__+':'+str(error)
 except fan.AdaptiveDensityFanInvalid as error:result='ERROR:INVALID:'+str(error)
 except (ValueError,ArithmeticError)as error:result='ERROR:CALLBACK:'+type(error).__name__+':'+str(error)
 wire=str(input)if op=='POINT'else vectors(input)
 print('IN\t'+'\t'.join([op,name,str(throw),kind,f(width),wire]));print('OUT\t'+result+'\tTRACE:'+'@'.join(events))
 return len(events)

for i in [0,-1,1,-10**100,10**100,2**500]:run('POINT','point-'+str(i),i)
grid=[(F(x),F(y))for x in range(-4,5)for y in range(-4,5)]
for number,v in enumerate(grid):
 count=run('CELL','cell-'+str(number),[v])
 if v in [(F(0),F(0)),(F(1),F(1)),(F(1),F(2)),(F(-1),F(-2))]:
  for k in range(1,count+1):run('CELL',f'cell-throw-{number}-{k}',[v],throw=k)
triples=[[(F(2),F(-1)),(F(1),F(0)),(F(2),F(1))],[(F(2),F(-1)),(F(1),F(0)),(F(-1),F(2))],[(F(1),F(-2)),(F(1),F(0)),(F(-1),F(2))],[(F(-1),F(-2)),(F(1),F(0)),(F(-2),F(1))],[(F(1),F(0)),(F(1),F(0)),(F(0),F(1))],[(F(1),F(-1)),(F(1),F(1)),(F(-1),F(1))],[(F(-1),F(1)),(F(0),F(0)),(F(1),F(1))],[(F(0),F(-1)),(F(1),F(0)),(F(0),F(1))]]
def rotate(v,k,orientation):
 x,y=v[0],v[1]*orientation
 for _ in range(k):x,y=-y,x
 return x,y
for k in range(4):
 for orientation in [-1,1]:
  for ti,t in enumerate(triples):
   t=[rotate(v,k,orientation)for v in t]
   for op in ['CHART','PATH','ATLAS']:
    for width in [F(0),F(1,100),F(1,2)]:run(op,f'{op}-{k}-{orientation}-{ti}-{width}',t,width=width)
for ti,t in enumerate(triples[:4]):
 for op in ['CHART','PATH','ATLAS']:
  count=run(op,f'full-{op}-{ti}',t)
  for failure in range(1,count+1):run(op,f'throw-{op}-{ti}-{failure}',t,throw=failure)

window_cases=[[],[(F(1),F(0))],[(F(1),F(0)),(F(0),F(1))],[(F(1),F(-1)),(F(1),F(0)),(F(1),F(1))],[(F(-2),F(-2)),(F(1),F(0)),(F(-3),F(1))],[(F(1),F(-2)),(F(1),F(0)),(F(1),F(2)),(F(-1),F(3))],[(F(-1),F(0)),(F(1),F(0)),(F(0),F(1))]]
for k in range(4):
 for orientation in [-1,1]:
  for wi,ideal in enumerate(window_cases):
   ideal=[rotate(v,k,orientation)for v in ideal]
   for width in [F(0),F(1,100),F(1,2)]:run('WINDOWS',f'windows-{k}-{orientation}-{wi}-{width}',ideal,width=width)
# Failure class identical but phase differs: only failures IN chart_for_window retry Atlas.
for wi,ideal in enumerate(window_cases[3:]):
 count=run('WINDOWS','windows-full-'+str(wi),ideal)
 for kind in ['normal','chart']:
  for failure in range(1,count+1):run('WINDOWS',f'windows-{kind}-throw-{wi}-{failure}',ideal,throw=failure,kind=kind)

rng=random.Random(678023)
for number in range(200):
 t=[rng.choice(grid)for _ in range(3)]
 op=rng.choice(['CELL','CHART','PATH','ATLAS','WINDOWS']);t=t[:1]if op=='CELL'else t
 run(op,f'random-{number}',t,width=rng.choice([F(0),F(1,100),F(1,2)]),throw=rng.choice([0,0,1,5,10,20]),kind=rng.choice(['normal','chart']))
"#;
