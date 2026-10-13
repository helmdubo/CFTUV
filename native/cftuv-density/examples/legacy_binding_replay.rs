//! Prepared legacy controller replay; scalar laws — явные fixture callbacks.

use std::io::{self, BufRead};

use cftuv_density::atlas_plan::{DecimalEnvelope, EndpointKind};
use cftuv_density::fan_prepare::PrepareRecord;
use cftuv_density::farey::{PrimitiveVector, WorkBudget};
use cftuv_density::legacy_binding::{
    LegacyBindingFailure, LegacyBindingLaws, LegacyCertificate, ProvenPredicates,
    legacy_authority_budget, legacy_density_certificate, legacy_full_fan_valid,
};
use cftuv_density::window_build::{PreparedAtlasWindow, PreparedWindowRecord};
use cftuv_density::{IBig, Rational, UBig};
use cftuv_radical::arith::{magnitude, signed, signum};

type Vector = (Rational, Rational);

fn rational(text: &str) -> Rational {
    let (n, d) = text.split_once('/').unwrap();
    Rational::new(n.parse().unwrap(), d.parse().unwrap()).unwrap()
}
fn fraction(value: &Rational) -> String {
    let (n, d) = value.parts();
    format!("{n}/{d}")
}
fn vector_text(value: &Vector) -> String {
    format!("{},{}", fraction(&value.0), fraction(&value.1))
}
fn primitive_text(value: &PrimitiveVector) -> String {
    format!("{},{}", value.0, value.1)
}
fn parse_vector(text: &str) -> Vector {
    let (x, y) = text.split_once(',').unwrap();
    (rational(x), rational(y))
}
fn vectors(text: &str) -> Vec<Vector> {
    if text == "_" {
        Vec::new()
    } else {
        text.split(';').map(parse_vector).collect()
    }
}
fn kind(text: &str) -> EndpointKind {
    match text {
        "C" => EndpointKind::Closed,
        "O" => EndpointKind::Open,
        _ => EndpointKind::Other,
    }
}
fn kind_text(value: EndpointKind) -> &'static str {
    match value {
        EndpointKind::Closed => "C",
        EndpointKind::Open => "O",
        EndpointKind::Other => "X",
    }
}
fn envelope(text: &str) -> DecimalEnvelope {
    let f: Vec<_> = text.split('~').collect();
    DecimalEnvelope {
        lower: rational(f[0]),
        upper: rational(f[1]),
        lower_kind: kind(f[2]),
        upper_kind: kind(f[3]),
        absolute_error_bound: rational(f[4]),
        quantum: None,
    }
}
fn envelope_text(e: &DecimalEnvelope) -> String {
    format!(
        "{}~{}~{}~{}~{}",
        fraction(&e.lower),
        fraction(&e.upper),
        kind_text(e.lower_kind),
        kind_text(e.upper_kind),
        fraction(&e.absolute_error_bound)
    )
}
fn point(value: i8) -> DecimalEnvelope {
    envelope(&format!("{value}/1~{value}/1~C~C~0/1"))
}
fn record(text: &str) -> PreparedWindowRecord {
    if text == "A" {
        PreparedWindowRecord::Atlas(PreparedAtlasWindow {
            pieces: Vec::new(),
            termination_piece_index: 0,
            local_record: PrepareRecord {
                use_x_denominator: true,
                denominator_sign: 1,
                lower_slope_envelope: point(-1),
                upper_slope_envelope: point(1),
            },
        })
    } else {
        let f: Vec<_> = text.strip_prefix("S!").unwrap().split(',').collect();
        PreparedWindowRecord::SingleChart(PrepareRecord {
            use_x_denominator: f[0] == "1",
            denominator_sign: f[1].parse().unwrap(),
            lower_slope_envelope: envelope(f[2]),
            upper_slope_envelope: envelope(f[3]),
        })
    }
}
fn records(text: &str) -> Vec<PreparedWindowRecord> {
    if text == "_" {
        Vec::new()
    } else {
        text.split('#').map(record).collect()
    }
}
fn certificates(text: &str) -> Vec<Option<LegacyCertificate>> {
    if text == "_" {
        return Vec::new();
    }
    text.split('#')
        .map(|text| {
            if text == "N" {
                return None;
            }
            let f: Vec<_> = text.split(',').collect();
            Some(LegacyCertificate {
                bound_primitive_integer_vector: PrimitiveVector(
                    f[0].parse().unwrap(),
                    f[1].parse().unwrap(),
                ),
                ideal_window_lower_slope_envelope: point(-1),
                ideal_window_upper_slope_envelope: point(1),
                certified_window_width_lower_bound: Rational::integer(IBig::from(2u8)),
                proven_predicates: if f[2] == "1" {
                    ProvenPredicates::Canonical
                } else {
                    ProvenPredicates::NonCanonical
                },
            })
        })
        .collect()
}
fn difference(left: &Rational, right: &Rational) -> Rational {
    let (ln, ld) = left.parts();
    let (rn, rd) = right.parts();
    Rational::new(ln * rd - rn * ld, ld * rd).unwrap()
}
fn add(left: &Rational, right: &Rational) -> Rational {
    let (ln, ld) = left.parts();
    let (rn, rd) = right.parts();
    Rational::new(ln * rd + rn * ld, ld * rd).unwrap()
}
fn mul(left: &Rational, right: &Rational) -> Rational {
    let (ln, ld) = left.parts();
    let (rn, rd) = right.parts();
    Rational::new(ln * rn, ld * rd).unwrap()
}
fn divide(left: &Rational, right: &Rational) -> Result<Rational, String> {
    let (ln, ld) = left.parts();
    let (rn, rd) = right.parts();
    if rn.is_zero() {
        return Err(format!("ZeroDivisionError:Fraction({}, 0)", ln * rd));
    }
    let numerator = ln * rd * IBig::from(signum(rn));
    Ok(Rational::new(numerator, ld * magnitude(rn)).unwrap())
}

struct Adapter {
    trace: Vec<String>,
    throw_at: usize,
    ideals: Vec<Vector>,
    u: Vec<i8>,
    x: Vec<i8>,
    inside: Vec<i8>,
    signs: Option<Vec<i8>>,
    counters: [usize; 4],
    score: String,
    certified_width: Rational,
}
impl Adapter {
    fn call<T>(
        &mut self,
        prefix: String,
        value: Result<T, String>,
        show: impl FnOnce(&T) -> String,
    ) -> Result<T, String> {
        if self.throw_at != 0 && self.trace.len() + 1 == self.throw_at {
            self.trace.push(prefix + ":!CallbackError:refused");
            return Err("CallbackError:refused".into());
        }
        match value {
            Ok(value) => {
                self.trace.push(format!("{prefix}:{}", show(&value)));
                Ok(value)
            }
            Err(error) => {
                self.trace.push(format!("{prefix}:!{error}"));
                Err(error)
            }
        }
    }
    fn next(&mut self, slot: usize) -> i8 {
        let pattern = match slot {
            0 => &self.u,
            1 => &self.x,
            2 => &self.inside,
            _ => self.signs.as_ref().unwrap(),
        };
        let value = pattern[self.counters[slot] % pattern.len()];
        self.counters[slot] += 1;
        value
    }
    fn sign(&mut self, value: &Rational) -> Result<i8, String> {
        let answer = if self.signs.is_some() {
            self.next(3)
        } else {
            signum(value.parts().0)
        };
        self.call("S:".to_owned() + &fraction(value), Ok(answer), |v| {
            v.to_string()
        })
    }
    fn score(&self, ordinal: usize, p: &PrimitiveVector) -> Rational {
        match self.score.as_str() {
            "asc" => Rational::integer(p.1.clone()),
            "desc" => Rational::integer(-&p.1),
            "mod" => Rational::integer(signed(
                &((magnitude(&p.1) + magnitude(&p.0) * UBig::from(2u8)) % UBig::from(3u8)),
            )),
            "gram" => {
                let v = (
                    Rational::integer(p.0.clone()),
                    Rational::integer(p.1.clone()),
                );
                let center = &self.ideals[ordinal];
                let dot = add(&mul(&v.0, &center.0), &mul(&v.1, &center.1));
                let norm = add(&mul(&v.0, &v.0), &mul(&v.1, &v.1));
                divide(&mul(&dot, &dot), &norm).unwrap()
            }
            _ => Rational::integer(IBig::ZERO),
        }
    }
}
impl LegacyBindingLaws for Adapter {
    type Vector = Vector;
    type Score = Rational;
    type Error = String;
    fn vector(&mut self, p: &PrimitiveVector) -> Result<Vector, String> {
        self.call(
            "V:".to_owned() + &primitive_text(p),
            Ok((
                Rational::integer(p.0.clone()),
                Rational::integer(p.1.clone()),
            )),
            vector_text,
        )
    }
    fn subturn(&mut self, left: &Vector, right: &Vector, q: usize) -> Result<bool, String> {
        let answer = self.next(0) != 0;
        self.call(
            format!("U:{q}:{}|{}", vector_text(left), vector_text(right)),
            Ok(answer),
            |v| i8::from(*v).to_string(),
        )
    }
    fn objective(&mut self, ordinal: usize, p: &PrimitiveVector) -> Result<Rational, String> {
        let answer = self.score(ordinal, p);
        self.call(
            format!("O:{ordinal}:{}", primitive_text(p)),
            Ok(answer),
            fraction,
        )
    }
    fn score_difference_sign(&mut self, left: &Rational, right: &Rational) -> Result<i8, String> {
        self.sign(&difference(left, right))
    }
    fn oriented_cross_sign(&mut self, left: &Vector, right: &Vector) -> Result<i8, String> {
        let answer = Rational::integer(IBig::from(self.next(1)));
        let cross = self.call(
            format!("X:{}|{}", vector_text(left), vector_text(right)),
            Ok(answer),
            fraction,
        )?;
        self.sign(&cross)
    }
    fn window_triple(&mut self, ordinal: usize) -> Result<(Vector, Vector, Vector), String> {
        let values = (
            self.ideals[ordinal - 1].clone(),
            self.ideals[ordinal].clone(),
            self.ideals[ordinal + 1].clone(),
        );
        self.call(format!("N:{ordinal}"), Ok(values), |v| {
            format!(
                "{}|{}|{}",
                vector_text(&v.0),
                vector_text(&v.1),
                vector_text(&v.2)
            )
        })
    }
    fn inside_ordinal(
        &mut self,
        candidate: &Vector,
        triple: &(Vector, Vector, Vector),
    ) -> Result<bool, String> {
        let answer = self.next(2) != 0;
        self.call(
            format!(
                "I:{}|{}|{}|{}",
                vector_text(candidate),
                vector_text(&triple.0),
                vector_text(&triple.1),
                vector_text(&triple.2)
            ),
            Ok(answer),
            |v| i8::from(*v).to_string(),
        )
    }
    fn slope(&mut self, value: &Vector, use_x: bool) -> Result<Rational, String> {
        let result = if use_x {
            divide(&value.1, &value.0)
        } else {
            divide(&value.0, &value.1)
        };
        self.call(
            format!("T:{}:{}", vector_text(value), i8::from(use_x)),
            result,
            fraction,
        )
    }
    fn certified_width(
        &mut self,
        lower: &DecimalEnvelope,
        upper: &DecimalEnvelope,
    ) -> Result<Rational, String> {
        // Frozen ответ actual Decimal subtraction; числовой law у wire adapter.
        let answer = self.certified_width.clone();
        self.call(
            format!("W:{}|{}", fraction(&upper.lower), fraction(&lower.upper)),
            Ok(answer),
            fraction,
        )
    }
}

#[derive(Debug)]
pub enum ReplayResult {
    Budget(UBig),
    Certificate(Option<LegacyCertificate>),
    Valid(bool),
}
fn digits(text: &str) -> Vec<i8> {
    text.chars()
        .map(|c| c.to_digit(10).unwrap() as i8)
        .collect()
}
pub fn evaluate_line(
    line: &str,
) -> (
    Result<ReplayResult, LegacyBindingFailure<String>>,
    Option<WorkBudget>,
    Vec<String>,
) {
    let f: Vec<_> = line.split('\t').collect();
    assert_eq!(f.len(), 17);
    let ideal = vectors(f[12]);
    let mut adapter = Adapter {
        trace: Vec::new(),
        throw_at: f[6].parse().unwrap(),
        ideals: ideal.clone(),
        u: digits(f[7]),
        x: f[8].split(',').map(|s| s.parse().unwrap()).collect(),
        inside: digits(f[9]),
        signs: if f[11] == "actual" {
            None
        } else {
            Some(f[11].split(',').map(|s| s.parse().unwrap()).collect())
        },
        counters: [0; 4],
        score: f[10].to_owned(),
        certified_width: rational(f[16]),
    };
    let mut budget = None;
    let result = match f[0] {
        "B" => legacy_authority_budget::<String>(&rational(f[15])).map(ReplayResult::Budget),
        "CERT" => {
            let records = records(f[13]);
            let report = legacy_density_certificate(
                &ideal,
                f[3].parse().unwrap(),
                f[5].parse().unwrap(),
                &records[0],
                f[2].parse().unwrap(),
                &mut adapter,
            );
            budget = report.budget;
            report.result.map(ReplayResult::Certificate)
        }
        "FULL" => legacy_full_fan_valid(
            &ideal,
            f[4].parse().unwrap(),
            f[5].parse().unwrap(),
            &records(f[13]),
            &certificates(f[14]),
            &mut adapter,
        )
        .map(ReplayResult::Valid),
        _ => panic!("fixture operation"),
    };
    (result, budget, adapter.trace)
}
fn failure(error: &LegacyBindingFailure<String>) -> String {
    match error {
        LegacyBindingFailure::EmptyWindow => {
            "AdaptiveDensityFanInvalid:legacy Density window is empty".into()
        }
        LegacyBindingFailure::Work(e) => format!("DensityRationalAuthorityExhausted:{e}"),
        LegacyBindingFailure::Callback(e) => e.clone(),
        LegacyBindingFailure::MissingIdealVector { .. } => {
            "IndexError:tuple index out of range".into()
        }
        LegacyBindingFailure::InvalidOrdinal => {
            "InternalInvalidOrdinal:ordinal must be positive".into()
        }
        LegacyBindingFailure::RecordsShorter => {
            "ValueError:zip() argument 3 is shorter than arguments 1-2".into()
        }
        LegacyBindingFailure::RecordsLonger => {
            "ValueError:zip() argument 3 is longer than arguments 1-2".into()
        }
    }
}
pub fn replay_line(line: &str) -> String {
    let (result, budget, trace) = evaluate_line(line);
    let result = match result {
        Ok(ReplayResult::Budget(b)) => format!("BOUND:{b}"),
        Ok(ReplayResult::Valid(value)) => format!("VALID:{}", i8::from(value)),
        Ok(ReplayResult::Certificate(None)) => "CERT:N".into(),
        Ok(ReplayResult::Certificate(Some(c))) => format!(
            "CERT:{};{};{};{};P={}",
            primitive_text(&c.bound_primitive_integer_vector),
            envelope_text(&c.ideal_window_lower_slope_envelope),
            envelope_text(&c.ideal_window_upper_slope_envelope),
            fraction(&c.certified_window_width_lower_bound),
            i8::from(c.proven_predicates == ProvenPredicates::Canonical)
        ),
        Err(error) => "ERROR:".to_owned() + &failure(&error),
    };
    let budget = budget
        .map(|b| format!("{},{},{}", b.cap, b.shell_probes, b.order_steps))
        .unwrap_or_else(|| "_".into());
    format!("{result}\tBUDGET:{budget}\tTRACE:{}", trace.join("@"))
}
fn main() {
    for line in io::stdin().lock().lines() {
        println!("{}", replay_line(&line.unwrap()));
    }
}
