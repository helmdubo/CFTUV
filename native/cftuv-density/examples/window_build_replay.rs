//! TAB replay: POINT/CELL/CHART/PATH/ATLAS/WINDOWS. Full fixed-operation callback
//! trace, включая membership/int/rational comparisons и class-specific fallback.
//! Fields: op,name,throwAt,failureClass,width,vectors-or-integer.

use cftuv_density::atlas_plan::{
    BoolField, ChartPiece, DecimalEnvelope, EndpointKind, NumericField, PoleField, PoleOwnership,
};
use cftuv_density::fan_prepare::PrepareRecord;
use cftuv_density::window_build::*;
use cftuv_density::{IBig, Rational};
use std::io::{self, BufRead};
fn r(t: &str) -> Rational {
    let (n, d) = t.split_once('/').unwrap();
    Rational::new(n.parse().unwrap(), d.parse().unwrap()).unwrap()
}
fn rt(v: &Rational) -> String {
    let (n, d) = v.parts();
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
fn mul(a: &Rational, b: &Rational) -> Rational {
    let (an, ad) = a.parts();
    let (bn, bd) = b.parts();
    Rational::new(an * bn, ad * bd).unwrap()
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
fn cmp(a: &Rational, b: &Rational) -> std::cmp::Ordering {
    let (an, ad) = a.parts();
    let (bn, bd) = b.parts();
    (an * bd).cmp(&(bn * ad))
}
type Vector = (Rational, Rational);
fn vector(t: &str) -> Vector {
    let (a, b) = t.split_once(',').unwrap();
    (r(a), r(b))
}
fn vt(v: &Vector) -> String {
    format!("{},{}", rt(&v.0), rt(&v.1))
}
fn vs(t: &str) -> Vec<Vector> {
    if t == "_" {
        vec![]
    } else {
        t.split(';').map(vector).collect()
    }
}
fn et(e: &DecimalEnvelope) -> String {
    let k = |v| match v {
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
fn record(r: &PrepareRecord) -> String {
    format!(
        "{},{},{},{}",
        u8::from(r.use_x_denominator),
        r.denominator_sign,
        et(&r.lower_slope_envelope),
        et(&r.upper_slope_envelope)
    )
}
fn piece(p: &ChartPiece) -> String {
    let number = |f: &NumericField| {
        if let NumericField::Int(i) = f {
            i.to_string()
        } else {
            panic!("builder exact int")
        }
    };
    let boolean = |f: &BoolField| {
        if let BoolField::Bool(v) = f {
            u8::from(*v)
        } else {
            panic!("builder exact bool")
        }
    };
    let pole = match p.pole_ownership {
        PoleField::Enum(PoleOwnership::None) => "N",
        PoleField::Enum(PoleOwnership::XZero) => "X",
        PoleField::Enum(PoleOwnership::YZero) => "Y",
        _ => panic!("builder enum"),
    };
    format!(
        "{},{},{},{},{},{},{},{},{}",
        number(&p.piece_index),
        boolean(&p.use_x_denominator),
        number(&p.denominator_sign),
        et(&p.lower_slope_envelope),
        et(&p.upper_slope_envelope),
        boolean(&p.lower_endpoint_included),
        boolean(&p.upper_endpoint_included),
        boolean(&p.slope_increases_in_ordinal_order),
        pole
    )
}
fn atlas(a: &PreparedAtlasWindow) -> String {
    format!(
        "A!{}!{}!{}",
        a.termination_piece_index,
        a.pieces.iter().map(piece).collect::<Vec<_>>().join("@"),
        record(&a.local_record)
    )
}
fn records(rs: &[PreparedWindowRecord]) -> String {
    if rs.is_empty() {
        "_".into()
    } else {
        rs.iter()
            .map(|r| match r {
                PreparedWindowRecord::SingleChart(r) => format!("S!{}", record(r)),
                PreparedWindowRecord::Atlas(a) => atlas(a),
            })
            .collect::<Vec<_>>()
            .join("#")
    }
}
fn error(e: WindowBuildFailure<String>) -> String {
    match e {
        WindowBuildFailure::ChartUnrepresentable => {
            "CHART:DENSITY_WINDOW_CHART_UNREPRESENTABLE".into()
        }
        WindowBuildFailure::OrdinalEnvelopesOverlap => {
            "INVALID:ordinal rational envelopes overlap".into()
        }
        WindowBuildFailure::Callback(e) => format!("CALLBACK:{e}"),
        WindowBuildFailure::MissingIdealCell(c) => format!("INTERNAL:MissingIdealCell:{c}"),
    }
}
struct Laws {
    trace: Vec<String>,
    throw_at: usize,
    class: String,
    width: Rational,
    ideal: Vec<Vector>,
}
impl Laws {
    fn emit<T>(&mut self, prefix: String, result: String, value: T) -> Result<T, String> {
        if self.throw_at != 0 && self.trace.len() + 1 == self.throw_at {
            let failure = if self.class == "chart" {
                "DensityWindowChartUnrepresentable:refused"
            } else {
                "CallbackError:refused"
            };
            self.trace.push(format!("{prefix}:!{failure}"));
            return Err(failure.into());
        }
        self.trace.push(format!("{prefix}:{result}"));
        Ok(value)
    }
}
impl WindowBuildLaws for Laws {
    type Vector = Vector;
    type Scalar = Rational;
    type Error = String;
    fn window_neighbours(&mut self, o: usize) -> Result<(Vector, Vector), String> {
        let result = (self.ideal[o - 1].clone(), self.ideal[o + 1].clone());
        self.emit(
            format!("N:{o}"),
            format!("{}|{}", vt(&result.0), vt(&result.1)),
            result,
        )
    }
    fn coordinates(&mut self, v: &Vector) -> Result<Vector, String> {
        self.emit(format!("D:{}", vt(v)), vt(v), v.clone())
    }
    fn add(&mut self, a: &Rational, b: &Rational) -> Result<Rational, String> {
        let result = add(a, b);
        self.emit(format!("A:{}|{}", rt(a), rt(b)), rt(&result), result)
    }
    fn sub(&mut self, a: &Rational, b: &Rational) -> Result<Rational, String> {
        let result = add(a, &neg(b));
        self.emit(format!("R:{}|{}", rt(a), rt(b)), rt(&result), result)
    }
    fn mul(&mut self, a: &Rational, b: &Rational) -> Result<Rational, String> {
        let result = mul(a, b);
        self.emit(format!("M:{}|{}", rt(a), rt(b)), rt(&result), result)
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
    fn vector(&mut self, x: Rational, y: Rational) -> Result<Vector, String> {
        let result = (x, y);
        self.emit(format!("V:{}", vt(&result)), vt(&result), result)
    }
    fn integer_vector(&mut self, x: &IBig, y: &IBig) -> Result<Vector, String> {
        let result = (Rational::integer(x.clone()), Rational::integer(y.clone()));
        self.emit(format!("VI:{x},{y}"), vt(&result), result)
    }
    fn slope(&mut self, v: &Vector, x: bool) -> Result<Rational, String> {
        let prefix = format!("T:{}:{}", vt(v), u8::from(x));
        let (a, b) = if x { (&v.1, &v.0) } else { (&v.0, &v.1) };
        if b.parts().0.is_zero() {
            if self.throw_at != 0 && self.trace.len() + 1 == self.throw_at {
                return self.emit(prefix, "_".into(), Rational::integer(IBig::ZERO));
            }
            let e = "AdaptiveDensityFanInvalid:fan chart denominator is zero";
            self.trace.push(format!("{prefix}:!{e}"));
            return Err(e.into());
        }
        let result = div(a, b);
        self.emit(prefix, rt(&result), result)
    }
    fn decimal_envelope(&mut self, a: &Rational) -> Result<DecimalEnvelope, String> {
        let e = DecimalEnvelope {
            lower: add(a, &neg(&self.width)),
            upper: add(a, &self.width),
            lower_kind: EndpointKind::Closed,
            upper_kind: EndpointKind::Closed,
            absolute_error_bound: self.width.clone(),
            quantum: None,
        };
        self.emit(format!("E:{}", rt(a)), et(&e), e)
    }
    fn equals_integer(&mut self, a: &Rational, i: i8) -> Result<bool, String> {
        let result = a == &Rational::integer(IBig::from(i));
        self.emit(
            format!("Q:{}:{i}", rt(a)),
            u8::from(result).to_string(),
            result,
        )
    }
    fn integer_value(&mut self, a: &Rational) -> Result<IBig, String> {
        let (n, d) = a.parts();
        let i = n / d;
        self.emit(format!("J:{}", rt(a)), i.to_string(), i)
    }
    fn rational_lt_scalar(&mut self, r: &Rational, s: &Rational) -> Result<bool, String> {
        let result = cmp(r, s).is_lt();
        self.emit(
            format!("RL:{}|{}", rt(r), rt(s)),
            u8::from(result).to_string(),
            result,
        )
    }
    fn scalar_lt_rational(&mut self, s: &Rational, r: &Rational) -> Result<bool, String> {
        let result = cmp(s, r).is_lt();
        self.emit(
            format!("SL:{}|{}", rt(s), rt(r)),
            u8::from(result).to_string(),
            result,
        )
    }
    fn is_chart_unrepresentable(e: &String) -> bool {
        e.starts_with("DensityWindowChartUnrepresentable:")
    }
}
fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        let input = if f[0] == "POINT" { vec![] } else { vs(f[5]) };
        let mut laws = Laws {
            trace: vec![],
            throw_at: f[2].parse().unwrap(),
            class: f[3].into(),
            width: r(f[4]),
            ideal: input.clone(),
        };
        let result = match f[0] {
            "POINT" => Ok(format!(
                "POINT:{}",
                et(&exact_decimal_interval(f[5].parse().unwrap()))
            )),
            "CELL" => {
                cell(&input[0], &mut laws).map(|(c, x, s)| format!("CELL:{c},{},{s}", u8::from(x)))
            }
            "CHART" => chart_for_window(&input[1], &input[0], &input[2], &mut laws)
                .map(|(x, s)| format!("CHART:{},{s}", u8::from(x))),
            "PATH" => cell_path(&input[0], &input[1], &input[2], &mut laws).map(|p| {
                format!(
                    "PATH:{}",
                    p.iter().map(usize::to_string).collect::<Vec<_>>().join(",")
                )
            }),
            "ATLAS" => build_atlas_window_record(&input[1], &input[0], &input[2], &mut laws)
                .map(|a| format!("ATLAS:{}", atlas(&a))),
            "WINDOWS" => window_envelopes(&input, &mut laws).map(|built| {
                format!(
                    "WINDOWS:{}\tRECORDS:{}",
                    if built.windows.is_empty() {
                        "_".into()
                    } else {
                        built
                            .windows
                            .iter()
                            .map(|(a, b)| format!("{}|{}", vt(a), vt(b)))
                            .collect::<Vec<_>>()
                            .join(";")
                    },
                    records(&built.records)
                )
            }),
            _ => panic!("operation"),
        };
        let result = result.unwrap_or_else(|e| format!("ERROR:{}", error(e)));
        println!("{result}\tTRACE:{}", laws.trace.join("@"));
    }
}
