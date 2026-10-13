//! TAB replay: SEALED/FEASIBLE/BOXES/HEIGHT, outcomes + budgets + полный trace.
//! Fields: op,name,throwAt,L,I,Csigns,subturn,limit,boundaries,width,ideal,payload,
//! ordinal,orientation,q,cap. Record: chart,sign,lowerEnvelope,upperEnvelope.

use cftuv_density::atlas_plan::{DecimalEnvelope, EndpointKind};
use cftuv_density::fan_prepare::*;
use cftuv_density::farey::WorkBudget;
use cftuv_density::{IBig, Rational, UBig};
use std::io::{self, BufRead};

fn r(text: &str) -> Rational {
    let (n, d) = text.split_once('/').unwrap();
    Rational::new(n.parse().unwrap(), d.parse().unwrap()).unwrap()
}
fn rt(value: &Rational) -> String {
    let (n, d) = value.parts();
    format!("{n}/{d}")
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
fn div(a: &Rational, b: &Rational) -> Rational {
    let (an, ad) = a.parts();
    let (bn, bd) = b.parts();
    let n = an * bd;
    Rational::new(
        if bn < &IBig::ZERO { -n } else { n },
        ad * cftuv_radical::arith::magnitude(bn),
    )
    .unwrap()
}
fn envelope(text: &str) -> DecimalEnvelope {
    let f: Vec<_> = text.split('~').collect();
    let k = |t| match t {
        "O" => EndpointKind::Open,
        "C" => EndpointKind::Closed,
        _ => EndpointKind::Other,
    };
    DecimalEnvelope {
        lower: r(f[0]),
        upper: r(f[1]),
        lower_kind: k(f[2]),
        upper_kind: k(f[3]),
        absolute_error_bound: r(f[4]),
        quantum: None,
    }
}
fn et(e: &DecimalEnvelope) -> String {
    let k = |t| match t {
        EndpointKind::Open => "O",
        EndpointKind::Closed => "C",
        EndpointKind::Other => "X",
    };
    format!(
        "{}~{}~{}~{}~{}",
        rt(&e.lower),
        rt(&e.upper),
        k(e.lower_kind),
        k(e.upper_kind),
        rt(&e.absolute_error_bound)
    )
}
fn record(text: &str) -> PrepareRecord {
    let f: Vec<_> = text.split(',').collect();
    PrepareRecord {
        use_x_denominator: f[0] == "1",
        denominator_sign: f[1].parse().unwrap(),
        lower_slope_envelope: envelope(f[2]),
        upper_slope_envelope: envelope(f[3]),
    }
}
fn bx(text: &str) -> FanTerminationBox {
    let f: Vec<_> = text.split(',').collect();
    FanTerminationBox {
        lower: r(f[0]),
        upper: r(f[1]),
        use_x_denominator: f[2] == "1",
        denominator_sign: f[3].parse().unwrap(),
    }
}
fn bxt(b: &FanTerminationBox) -> String {
    format!(
        "{},{},{},{}",
        rt(&b.lower),
        rt(&b.upper),
        u8::from(b.use_x_denominator),
        b.denominator_sign
    )
}
fn boxes(text: &str) -> Vec<FanTerminationBox> {
    if text == "_" {
        vec![]
    } else {
        text.split(';').map(bx).collect()
    }
}
fn boxt(values: &[FanTerminationBox]) -> String {
    if values.is_empty() {
        "_".into()
    } else {
        values.iter().map(bxt).collect::<Vec<_>>().join(";")
    }
}
type Vector = (Rational, Rational);
fn vector(text: &str) -> Vector {
    let (a, b) = text.split_once(',').unwrap();
    (r(a), r(b))
}
fn vt(v: &Vector) -> String {
    format!("{},{}", rt(&v.0), rt(&v.1))
}
fn vectors(text: &str) -> Vec<Vector> {
    if text == "_" {
        vec![]
    } else {
        text.split(';').map(vector).collect()
    }
}
fn vlist(vs: &[Vector]) -> String {
    if vs.is_empty() {
        "_".into()
    } else {
        vs.iter().map(vt).collect::<Vec<_>>().join(";")
    }
}
fn failure(error: FanPrepareFailure<String>) -> String {
    match error {
        FanPrepareFailure::Invalid(e) => format!("INVALID:{}", e.message()),
        FanPrepareFailure::Callback(e) => format!("CALLBACK:{e}"),
        FanPrepareFailure::Work(e) => format!("WORK:{e}"),
        FanPrepareFailure::TerminationBoxesExhausted {
            hidden_rays,
            at_limit,
            halvings,
        } => format!(
            "TERMINATION:termination box refinement exhausted after {} halvings for {hidden_rays} hidden rays: no feasible box was witnessed (ideal fan exactly at the subturn limit: {})",
            halvings,
            at_limit.text()
        ),
        FanPrepareFailure::MissingIdealVector { index } => {
            format!("INTERNAL:MissingIdealVector:{index}")
        }
    }
}

struct Laws {
    trace: Vec<String>,
    throw_at: usize,
    patterns: [Vec<i8>; 4],
    counts: [usize; 4],
    limit: String,
    boundaries: Vec<Vector>,
    width: Rational,
    ideal: Vec<Vector>,
}
impl Laws {
    fn begin(&mut self, prefix: &str) -> Result<(), String> {
        if self.throw_at != 0 && self.trace.len() + 1 == self.throw_at {
            self.trace.push(format!("{prefix}:!CallbackError:refused"));
            Err("CallbackError:refused".into())
        } else {
            Ok(())
        }
    }
    fn emit<T>(&mut self, prefix: String, result: String, value: T) -> Result<T, String> {
        self.begin(&prefix)?;
        self.trace.push(format!("{prefix}:{result}"));
        Ok(value)
    }
    fn next(&mut self, kind: usize) -> i8 {
        let i = self.counts[kind];
        self.counts[kind] += 1;
        self.patterns[kind][i % self.patterns[kind].len()]
    }
    fn slope_vector(value: &Rational, x: bool, sign: i8) -> Vector {
        let (n, d) = value.parts();
        let a = Rational::integer(n * IBig::from(sign));
        let b = Rational::integer(cftuv_radical::arith::signed(d) * IBig::from(sign));
        if x {
            (b, a)
        } else {
            (a, b)
        }
    }
}
impl FanPrepareLaws for Laws {
    type Vector = Vector;
    type Scalar = Rational;
    type Error = String;
    fn subturn_boundary_vectors(
        &mut self,
        o: usize,
        s: i8,
        q: usize,
    ) -> Result<Vec<Vector>, String> {
        self.emit(
            format!("B:{o}:{s}:{q}"),
            vlist(&self.boundaries),
            self.boundaries.clone(),
        )
    }
    fn denominator(&mut self, v: &Vector, x: bool) -> Result<Rational, String> {
        let a = if x { v.0.clone() } else { v.1.clone() };
        self.emit(format!("D:{}:{}", vt(v), u8::from(x)), rt(&a), a)
    }
    fn sign(&mut self, a: &Rational) -> Result<i8, String> {
        let n = a.parts().0;
        let s = if n < &IBig::ZERO {
            -1
        } else if n > &IBig::ZERO {
            1
        } else {
            0
        };
        self.emit(format!("S:{}", rt(a)), s.to_string(), s)
    }
    fn slope(&mut self, v: &Vector, x: bool) -> Result<Rational, String> {
        let prefix = format!("T:{}:{}", vt(v), u8::from(x));
        self.begin(&prefix)?;
        let (a, b) = if x { (&v.1, &v.0) } else { (&v.0, &v.1) };
        if b.parts().0.is_zero() {
            let error = format!("ZeroDivisionError:Fraction({}, 0)", a.parts().0);
            self.trace.push(format!("{prefix}:!{error}"));
            return Err(error);
        }
        let result = div(a, b);
        self.trace.push(format!("{prefix}:{}", rt(&result)));
        Ok(result)
    }
    fn decimal_envelope(&mut self, v: &Rational) -> Result<DecimalEnvelope, String> {
        let e = DecimalEnvelope {
            lower: add(v, &neg(&self.width)),
            upper: add(v, &self.width),
            lower_kind: EndpointKind::Closed,
            upper_kind: EndpointKind::Closed,
            absolute_error_bound: self.width.clone(),
            quantum: None,
        };
        self.emit(format!("E:{}", rt(v)), et(&e), e)
    }
    fn vector_from_slope(&mut self, v: &Rational, x: bool, s: i8) -> Result<Vector, String> {
        let result = Self::slope_vector(v, x, s);
        self.emit(
            format!("V:{}:{}:{s}", rt(v), u8::from(x)),
            vt(&result),
            result,
        )
    }
    fn candidate_vector(&mut self, v: &Rational, x: bool, s: i8) -> Result<Vector, String> {
        let result = Self::slope_vector(v, x, s);
        self.emit(
            format!("Q:{}:{}:{s}", rt(v), u8::from(x)),
            vt(&result),
            result,
        )
    }
    fn local_candidate(&mut self, o: usize, v: &Vector, q: usize) -> Result<bool, String> {
        let result = self.next(0) != 0;
        self.emit(
            format!("L:{o}:{q}:{}", vt(v)),
            u8::from(result).to_string(),
            result,
        )
    }
    fn window_neighbours(&mut self, o: usize) -> Result<(Vector, Vector), String> {
        let result = (self.ideal[o - 1].clone(), self.ideal[o + 1].clone());
        self.emit(
            format!("N:{o}"),
            format!("{}|{}", vt(&result.0), vt(&result.1)),
            result,
        )
    }
    fn inside_ordinal(
        &mut self,
        v: &Vector,
        p: &Vector,
        c: &Vector,
        f: &Vector,
    ) -> Result<bool, String> {
        let result = self.next(1) != 0;
        self.emit(
            format!("I:{}|{}|{}|{}", vt(v), vt(p), vt(c), vt(f)),
            u8::from(result).to_string(),
            result,
        )
    }
    fn oriented_cross_sign(&mut self, l: &Vector, r: &Vector) -> Result<i8, String> {
        let result = self.next(2);
        let scalar = Rational::integer(IBig::from(result));
        let scalar = self.emit(format!("C:{}|{}", vt(l), vt(r)), rt(&scalar), scalar)?;
        self.sign(&scalar)
    }
    fn subturn(&mut self, l: &Vector, r: &Vector, q: usize) -> Result<bool, String> {
        let result = self.next(3) != 0;
        self.emit(
            format!("U:{q}:{}|{}", vt(l), vt(r)),
            u8::from(result).to_string(),
            result,
        )
    }
    fn subturn_boundary(&mut self, l: &Vector, r: &Vector, q: usize) -> Result<bool, String> {
        let prefix = format!("F:{q}:{}|{}", vt(l), vt(r));
        self.begin(&prefix)?;
        let (result, text) = match self.limit.as_str() {
            "true" => (Ok(true), "1"),
            "false" => (Ok(false), "0"),
            "ref" => (
                Err("ReferenceGeometryError:diagnostic_unproven".into()),
                "!ReferenceGeometryError:diagnostic_unproven",
            ),
            _ => (
                Err("CallbackError:diagnostic_failed".into()),
                "!CallbackError:diagnostic_failed",
            ),
        };
        self.trace.push(format!("{prefix}:{text}"));
        result
    }
    fn is_reference_geometry_error(error: &String) -> bool {
        error.starts_with("ReferenceGeometryError:")
    }
}

fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        let ideal = vectors(f[10]);
        let mut laws = Laws {
            trace: vec![],
            throw_at: f[2].parse().unwrap(),
            patterns: [
                f[3].chars().map(|c| i8::from(c == '1')).collect(),
                f[4].chars().map(|c| i8::from(c == '1')).collect(),
                f[5].split(',').map(|s| s.parse().unwrap()).collect(),
                f[6].chars().map(|c| i8::from(c == '1')).collect(),
            ],
            counts: [0; 4],
            limit: f[7].into(),
            boundaries: vectors(f[8]),
            width: r(f[9]),
            ideal: ideal.clone(),
        };
        let mut budget = WorkBudget::new(f[15].parse::<UBig>().unwrap());
        let o = f[12].parse().unwrap();
        let s = f[13].parse().unwrap();
        let q = f[14].parse().unwrap();
        let result = match f[0] {
            "SEALED" => sealed_local_interval(&ideal, o, s, q, &record(f[11]), &mut laws)
                .map(|b| format!("SEALED:{}", b.iter().map(rt).collect::<Vec<_>>().join(","))),
            "FEASIBLE" => box_is_feasible(&ideal, s, q, &boxes(f[11]), &mut laws)
                .map(|v| format!("FEASIBLE:{}", u8::from(v))),
            "BOXES" => {
                let records = if f[11] == "_" {
                    vec![]
                } else {
                    f[11].split(';').map(record).collect()
                };
                termination_boxes_with_limits(
                    &ideal,
                    s,
                    q,
                    &records,
                    &mut budget,
                    &mut laws,
                    PreparationLimits {
                        box_refinement_cap: f
                            .get(16)
                            .map(|v| v.parse().unwrap())
                            .unwrap_or(BOX_REFINEMENT_CAP),
                    },
                )
                .map(|b| format!("BOXES:{}", boxt(&b)))
            }
            "HEIGHT" => termination_height(&boxes(f[11]))
                .map(|h| format!("HEIGHT:{h}"))
                .map_err(FanPrepareFailure::Invalid),
            _ => panic!("operation"),
        };
        let result = result.unwrap_or_else(|e| format!("ERROR:{}", failure(e)));
        println!(
            "{result}\tWORK:{},{}\tTRACE:{}",
            budget.shell_probes,
            budget.order_steps,
            laws.trace.join("@")
        );
    }
}
