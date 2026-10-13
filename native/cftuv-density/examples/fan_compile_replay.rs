//! Controller replay: TAB name,prepCap,searchCap,boxCap,heightCap,q,orientation,
//! throwAt,L,I,Xsigns,U,limit,boundaries,width,ideal,records.
//! Record S!chart,sign,lowerEnv,upperEnv либо A!index!pieces!localRecord.
//! Только подготовленные value descriptors; authority ID/Decimal quantum не кодируются.

use cftuv_density::atlas_plan::{
    AtlasPlanFailure, BoolField, ChartPiece, DecimalEnvelope, EndpointKind, NumericField,
    PoleField, PoleOwnership, SegmentPlanLaws,
};
use cftuv_density::fan_compile::*;
use cftuv_density::fan_prepare::{FanPrepareFailure, FanPrepareLaws};
use cftuv_density::farey::{Endpoint, FareyFailure, FareyPredicates, PrimitiveVector, WorkBudget};
use cftuv_density::window_build::{PreparedAtlasWindow, PreparedWindowRecord};
use cftuv_density::{IBig, Rational, UBig};
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
pub type Vector = (Rational, Rational);
fn vector(t: &str) -> Vector {
    let (x, y) = t.split_once(',').unwrap();
    (r(x), r(y))
}
fn vt(v: &Vector) -> String {
    format!("{},{}", rt(&v.0), rt(&v.1))
}
fn vectors(t: &str) -> Vec<Vector> {
    if t == "_" {
        vec![]
    } else {
        t.split(';').map(vector).collect()
    }
}
fn vlist(v: &[Vector]) -> String {
    if v.is_empty() {
        "_".into()
    } else {
        v.iter().map(vt).collect::<Vec<_>>().join(";")
    }
}
fn env(t: &str) -> DecimalEnvelope {
    let f: Vec<_> = t.split('~').collect();
    let k = |v| match v {
        "C" => EndpointKind::Closed,
        "O" => EndpointKind::Open,
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
    let k = |v| match v {
        EndpointKind::Closed => "C",
        EndpointKind::Open => "O",
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
fn local(t: &str) -> cftuv_density::fan_prepare::PrepareRecord {
    let f: Vec<_> = t.split(',').collect();
    cftuv_density::fan_prepare::PrepareRecord {
        use_x_denominator: f[0] == "1",
        denominator_sign: f[1].parse().unwrap(),
        lower_slope_envelope: env(f[2]),
        upper_slope_envelope: env(f[3]),
    }
}
fn record(t: &str) -> PreparedWindowRecord {
    let f: Vec<_> = t.split('!').collect();
    if f[0] == "S" {
        PreparedWindowRecord::SingleChart(local(f[1]))
    } else {
        PreparedWindowRecord::Atlas(PreparedAtlasWindow {
            termination_piece_index: f[1].parse().unwrap(),
            pieces: f[2].split('@').map(piece).collect(),
            local_record: local(f[3]),
        })
    }
}
fn piece(t: &str) -> ChartPiece {
    let f: Vec<_> = t.split(',').collect();
    ChartPiece {
        is_piece_v1: true,
        piece_index: NumericField::Int(f[0].parse().unwrap()),
        use_x_denominator: BoolField::Bool(f[1] == "1"),
        denominator_sign: NumericField::Int(f[2].parse().unwrap()),
        lower_slope_envelope: env(f[3]),
        upper_slope_envelope: env(f[4]),
        lower_endpoint_included: BoolField::Bool(f[5] == "1"),
        upper_endpoint_included: BoolField::Bool(f[6] == "1"),
        slope_increases_in_ordinal_order: BoolField::Bool(f[7] == "1"),
        pole_ownership: PoleField::Enum(match f[8] {
            "X" => PoleOwnership::XZero,
            "Y" => PoleOwnership::YZero,
            _ => PoleOwnership::None,
        }),
    }
}
fn piece_text(p: &ChartPiece) -> String {
    let n = |v: &NumericField| match v {
        NumericField::Int(v) => v.to_string(),
        _ => panic!("int"),
    };
    let b = |v: BoolField| u8::from(v.truth());
    let pole = match p.pole_ownership {
        PoleField::Enum(PoleOwnership::XZero) => "X",
        PoleField::Enum(PoleOwnership::YZero) => "Y",
        _ => "N",
    };
    format!(
        "{},{},{},{},{},{},{},{},{}",
        n(&p.piece_index),
        b(p.use_x_denominator),
        n(&p.denominator_sign),
        et(&p.lower_slope_envelope),
        et(&p.upper_slope_envelope),
        b(p.lower_endpoint_included),
        b(p.upper_endpoint_included),
        b(p.slope_increases_in_ordinal_order),
        pole
    )
}
fn bounds_text(b: &AuthorityWindowBounds) -> String {
    format!(
        "{},{},{},{},{}",
        b.ordinal,
        rt(&b.termination_lower_slope),
        rt(&b.termination_upper_slope),
        rt(&b.certified_termination_width),
        b.admissible.iter().map(rt).collect::<Vec<_>>().join(",")
    )
}
fn windows_text(ws: &[PreparedAuthorityWindow]) -> String {
    if ws.is_empty() {
        return "_".into();
    }
    ws.iter()
        .map(|w| match w {
            PreparedAuthorityWindow::SingleChart { bounds, record } => format!(
                "{}!S!{},{},{},{}",
                bounds_text(bounds),
                u8::from(record.use_x_denominator),
                record.denominator_sign,
                et(&record.lower_slope_envelope),
                et(&record.upper_slope_envelope)
            ),
            PreparedAuthorityWindow::Atlas {
                bounds,
                pieces,
                termination_piece_index,
            } => format!(
                "{}!A!{}!{}",
                bounds_text(bounds),
                termination_piece_index,
                pieces.iter().map(piece_text).collect::<Vec<_>>().join("@")
            ),
        })
        .collect::<Vec<_>>()
        .join("#")
}
fn counts(v: &[usize]) -> String {
    if v.is_empty() {
        "_".into()
    } else {
        v.iter().map(usize::to_string).collect::<Vec<_>>().join(",")
    }
}
fn budget_text(b: &Option<WorkBudget>) -> String {
    b.as_ref().map_or_else(
        || "_".into(),
        |b| format!("{},{},{}", b.cap, b.shell_probes, b.order_steps),
    )
}
fn preparation_error(e: FanPrepareFailure<String>) -> String {
    match e {
        FanPrepareFailure::Invalid(e) => format!("AdaptiveDensityFanInvalid:{}", e.message()),
        FanPrepareFailure::Callback(e) => e,
        FanPrepareFailure::Work(e) => format!("DensityRationalAuthorityExhausted:{e}"),
        FanPrepareFailure::TerminationBoxesExhausted {
            hidden_rays,
            at_limit,
            halvings,
        } => format!(
            "DensityTerminationBoxesExhausted:termination box refinement exhausted after {halvings} halvings for {hidden_rays} hidden rays: no feasible box was witnessed (ideal fan exactly at the subturn limit: {})",
            at_limit.text()
        ),
        FanPrepareFailure::MissingIdealVector { index } => {
            format!("INTERNAL:MissingIdealVector:{index}")
        }
    }
}
fn error(e: FanCompileFailure<String>) -> String {
    match e {
        FanCompileFailure::Preparation(e) => preparation_error(e),
        FanCompileFailure::Height(e) => format!("AdaptiveDensityFanInvalid:{}", e.message()),
        FanCompileFailure::Segments(e) => match e {
            AtlasPlanFailure::Callback(e) => e,
            AtlasPlanFailure::Invalid(e) => format!("AdaptiveDensityFanInvalid:{}", e.message()),
            e => format!("INTERNAL:{e:?}"),
        },
        FanCompileFailure::Search(e) => match e {
            FareyFailure::Work(e) => format!("DensityRationalAuthorityExhausted:{e}"),
            FareyFailure::Predicate(e) => e,
            e => format!("INTERNAL:{e:?}"),
        },
        FanCompileFailure::RationalAuthorityExhausted => {
            "DensityRationalAuthorityExhausted:DENSITY_RATIONAL_AUTHORITY_EXHAUSTED".into()
        }
        FanCompileFailure::MissingSearchHeight => "INTERNAL:MissingSearchHeight".into(),
    }
}

pub struct Adapter {
    pub trace: Vec<String>,
    throw_at: usize,
    patterns: [Vec<i8>; 4],
    counts: [usize; 4],
    limit: String,
    boundaries: Vec<Vector>,
    width: Rational,
    ideal: Vec<Vector>,
    q: usize,
    orientation: i8,
}
impl Adapter {
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
    fn next(&mut self, k: usize) -> i8 {
        let i = self.counts[k];
        self.counts[k] += 1;
        self.patterns[k][i % self.patterns[k].len()]
    }
    fn local(&mut self, o: usize, v: &Vector, q: usize) -> Result<bool, String> {
        let result = self.next(0) != 0;
        self.emit(
            format!("L:{o}:{q}:{}", vt(v)),
            u8::from(result).to_string(),
            result,
        )
    }
    fn primitive(v: &PrimitiveVector) -> Vector {
        (
            Rational::integer(v.0.clone()),
            Rational::integer(v.1.clone()),
        )
    }
    fn endpoint(&self, e: Endpoint<'_>) -> Vector {
        match e {
            Endpoint::Incoming => self.ideal[0].clone(),
            Endpoint::Outgoing => self.ideal.last().unwrap().clone(),
            Endpoint::Primitive(v) => Self::primitive(v),
        }
    }
}
impl FanPrepareLaws for Adapter {
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
        let d = if x { v.0.clone() } else { v.1.clone() };
        self.emit(format!("D:{}:{}", vt(v), u8::from(x)), rt(&d), d)
    }
    fn sign(&mut self, v: &Rational) -> Result<i8, String> {
        let n = v.parts().0;
        let s = if n < &IBig::ZERO {
            -1
        } else if n > &IBig::ZERO {
            1
        } else {
            0
        };
        self.emit(format!("S:{}", rt(v)), s.to_string(), s)
    }
    fn slope(&mut self, v: &Vector, x: bool) -> Result<Rational, String> {
        let prefix = format!("T:{}:{}", vt(v), u8::from(x));
        self.begin(&prefix)?;
        let (a, b) = if x { (&v.1, &v.0) } else { (&v.0, &v.1) };
        if b.parts().0.is_zero() {
            let e = format!("ZeroDivisionError:Fraction({}, 0)", a.parts().0);
            self.trace.push(format!("{prefix}:!{e}"));
            return Err(e);
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
        let (n, d) = v.parts();
        let a = Rational::integer(n * IBig::from(s));
        let b = Rational::integer(cftuv_radical::arith::signed(d) * IBig::from(s));
        let result = if x { (b, a) } else { (a, b) };
        self.emit(
            format!("V:{}:{}:{s}", rt(v), u8::from(x)),
            vt(&result),
            result,
        )
    }
    fn candidate_vector(&mut self, v: &Rational, x: bool, s: i8) -> Result<Vector, String> {
        let (n, d) = v.parts();
        let a = Rational::integer(n * IBig::from(s));
        let b = Rational::integer(cftuv_radical::arith::signed(d) * IBig::from(s));
        let result = if x { (b, a) } else { (a, b) };
        self.emit(
            format!("Q:{}:{}:{s}", rt(v), u8::from(x)),
            vt(&result),
            result,
        )
    }
    fn local_candidate(&mut self, o: usize, v: &Vector, q: usize) -> Result<bool, String> {
        self.local(o, v, q)
    }
    fn window_neighbours(&mut self, o: usize) -> Result<(Vector, Vector), String> {
        let pair = (self.ideal[o - 1].clone(), self.ideal[o + 1].clone());
        self.emit(
            format!("N:{o}"),
            format!("{}|{}", vt(&pair.0), vt(&pair.1)),
            pair,
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
        let x = Rational::integer(IBig::from(self.next(2)));
        let x = self.emit(format!("X:{}|{}", vt(l), vt(r)), rt(&x), x)?;
        FanPrepareLaws::sign(self, &x)
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
    fn is_reference_geometry_error(e: &String) -> bool {
        e.starts_with("ReferenceGeometryError:")
    }
}
impl SegmentPlanLaws for Adapter {
    type Vector = Vector;
    type Scalar = Rational;
    type Error = String;
    fn denominator(&mut self, v: &Vector, x: bool) -> Result<Rational, String> {
        FanPrepareLaws::denominator(self, v, x)
    }
    fn sign(&mut self, v: &Rational) -> Result<i8, String> {
        FanPrepareLaws::sign(self, v)
    }
    fn slope(&mut self, v: &Vector, x: bool) -> Result<Rational, String> {
        FanPrepareLaws::slope(self, v, x)
    }
    fn decimal_envelope(&mut self, v: &Rational) -> Result<DecimalEnvelope, String> {
        FanPrepareLaws::decimal_envelope(self, v)
    }
    fn local_candidate(&mut self, o: usize, v: &Vector, q: usize) -> Result<bool, String> {
        self.local(o, v, q)
    }
    fn vector_from_slope(&mut self, v: &Rational, x: bool, s: i8) -> Result<Vector, String> {
        FanPrepareLaws::vector_from_slope(self, v, x, s)
    }
    fn local_transition(
        &mut self,
        o: usize,
        v: &PrimitiveVector,
        q: usize,
    ) -> Result<bool, String> {
        self.local(o, &Self::primitive(v), q)
    }
    fn subturn_boundary_vectors(
        &mut self,
        o: usize,
        s: i8,
        q: usize,
    ) -> Result<Vec<Vector>, String> {
        FanPrepareLaws::subturn_boundary_vectors(self, o, s, q)
    }
}
impl FareyPredicates for Adapter {
    type Error = String;
    fn local_candidate(&mut self, o: usize, v: &PrimitiveVector) -> Result<bool, String> {
        self.local(o, &Self::primitive(v), self.q)
    }
    fn candidate_compare(
        &mut self,
        o: usize,
        l: &PrimitiveVector,
        r: &PrimitiveVector,
    ) -> Result<i8, String> {
        let remainder = |v: IBig| {
            let r = v % IBig::from(7);
            if r < IBig::ZERO { r + IBig::from(7) } else { r }
        };
        let a = remainder(&l.0 + &l.1);
        let b = remainder(&r.0 + &r.1);
        let c = if a < b {
            -1
        } else if a > b {
            1
        } else {
            0
        };
        self.emit(
            format!(
                "K:{o}:{}|{}",
                vt(&Self::primitive(l)),
                vt(&Self::primitive(r))
            ),
            c.to_string(),
            c,
        )
    }
    fn edge_valid(&mut self, _: usize, l: Endpoint<'_>, r: Endpoint<'_>) -> Result<bool, String> {
        let l = self.endpoint(l);
        let r = self.endpoint(r);
        if FanPrepareLaws::oriented_cross_sign(self, &l, &r)? != self.orientation {
            return Ok(false);
        }
        FanPrepareLaws::subturn(self, &l, &r, self.q)
    }
}

pub fn evaluate_line(line: &str) -> (CompileReport<String>, Vec<String>) {
    let f: Vec<_> = line.split('\t').collect();
    let ideal = vectors(f[15]);
    let q = f[5].parse().unwrap();
    let orientation = f[6].parse().unwrap();
    let records = if f[16] == "_" {
        vec![]
    } else {
        f[16].split('#').map(record).collect()
    };
    let mut adapter = Adapter {
        trace: vec![],
        throw_at: f[7].parse().unwrap(),
        patterns: [
            f[8].chars().map(|c| i8::from(c == '1')).collect(),
            f[9].chars().map(|c| i8::from(c == '1')).collect(),
            f[10].split(',').map(|v| v.parse().unwrap()).collect(),
            f[11].chars().map(|c| i8::from(c == '1')).collect(),
        ],
        counts: [0; 4],
        limit: f[12].into(),
        boundaries: vectors(f[13]),
        width: r(f[14]),
        ideal: ideal.clone(),
        q,
        orientation,
    };
    let report = compile_prepared(
        &ideal,
        orientation,
        q,
        &records,
        &CompileLimits {
            preparation_work_cap: f[1].parse::<UBig>().unwrap(),
            search_work_cap: f[2].parse().unwrap(),
            box_refinement_cap: f[3].parse().unwrap(),
            resource_height_cap: f[4].parse().unwrap(),
        },
        &mut adapter,
    );
    (report, adapter.trace)
}
pub fn replay_line(line: &str) -> String {
    let (report, trace) = evaluate_line(line);
    let result = match report.result {
        Ok(a) => format!(
            "OK\tQ:{}\tHEIGHT:{}\tPREVIOUS_HEIGHT:{}\tTERMINATION:{}\tFAN:{}\tPREVIOUS_COUNTS:{}\tCOUNTS:{}\tWINDOWS:{}",
            a.max_subturn_q,
            a.minimal_common_height,
            a.exhaustive_previous_height,
            a.termination_height_upper_bound,
            a.bound_primitive_integer_vectors
                .iter()
                .map(|v| format!("{},{}", v.0, v.1))
                .collect::<Vec<_>>()
                .join(";"),
            counts(&a.previous_counts),
            counts(&a.counts),
            windows_text(&a.ordinal_windows)
        ),
        Err(e) => format!("ERROR:{}", error(e)),
    };
    format!(
        "{result}\tBUDGETS:{}#{}\tTRACE:{}",
        budget_text(&report.budgets.preparation),
        budget_text(&report.budgets.search),
        trace.join("@")
    )
}
fn main() {
    for line in io::stdin().lock().lines() {
        println!("{}", replay_line(&line.unwrap()));
    }
}
