//! Prepared atlas replay: CELL/BOUNDS/SHELL/SEARCH/GEOMETRY, TAB-fields.
//! Segment: chart,sign,outerlow,innerlow,innerhigh,outerhigh (rationals n/d).
//! Plans: segments separated @, ordinals separated #; _ emptyordinal, - noplans.
//! Full L/C/E callback journal; actual CanonicalOrdering, no ordering stub.

use std::cmp::Ordering;
use std::io::{self, BufRead};

use cftuv_density::atlas_search::{
    GlobalHeightSegment, cell_segments, numerator_bounds, search_global_height, segment_candidates,
};
use cftuv_density::farey::{
    Endpoint, FareyFailure, FareyPredicates, PrimitiveVector, SearchResult, WorkBudget,
};
use cftuv_density::geometry::{GeometryAdapter, GeometryFailure, GeometryFailureKind, WindowLaw};
use cftuv_density::order::CanonicalOrdering;
use cftuv_density::{DensityTransaction, IBig, Metric2, Radical, Rational, UBig, Vector2};
use cftuv_radical::arith::magnitude;

fn rational(text: &str) -> Rational {
    let (n, d) = text.split_once('/').expect("n/d");
    Rational::new(n.parse::<IBig>().unwrap(), d.parse::<UBig>().unwrap()).unwrap()
}
fn rational_text(value: &Rational) -> String {
    let (n, d) = value.parts();
    format!("{n}/{d}")
}
fn segment(text: &str) -> GlobalHeightSegment {
    let f: Vec<_> = text.split(',').collect();
    GlobalHeightSegment {
        use_x_denominator: f[0] == "1",
        denominator_sign: f[1].parse().unwrap(),
        outer_lower: rational(f[2]),
        inner_lower: rational(f[3]),
        inner_upper: rational(f[4]),
        outer_upper: rational(f[5]),
    }
}
fn segment_text(value: &GlobalHeightSegment) -> String {
    format!(
        "{},{},{},{},{},{}",
        u8::from(value.use_x_denominator),
        value.denominator_sign,
        rational_text(&value.outer_lower),
        rational_text(&value.inner_lower),
        rational_text(&value.inner_upper),
        rational_text(&value.outer_upper)
    )
}
fn plans(text: &str) -> Vec<Vec<GlobalHeightSegment>> {
    if text == "-" {
        Vec::new()
    } else {
        text.split('#')
            .map(|plan| {
                if plan == "_" {
                    Vec::new()
                } else {
                    plan.split('@').map(segment).collect()
                }
            })
            .collect()
    }
}
fn vector_text(value: &PrimitiveVector) -> String {
    format!("{},{}", value.0, value.1)
}
fn vectors_text(values: &[PrimitiveVector]) -> String {
    if values.is_empty() {
        "-".into()
    } else {
        values.iter().map(vector_text).collect::<Vec<_>>().join(";")
    }
}
fn counts(values: &[usize]) -> String {
    if values.is_empty() {
        "-".into()
    } else {
        values
            .iter()
            .map(usize::to_string)
            .collect::<Vec<_>>()
            .join(",")
    }
}
fn endpoint_text(value: Endpoint<'_>) -> String {
    match value {
        Endpoint::Incoming => "IN".into(),
        Endpoint::Outgoing => "OUT".into(),
        Endpoint::Primitive(value) => vector_text(value),
    }
}
fn sign(value: Ordering) -> i8 {
    match value {
        Ordering::Less => -1,
        Ordering::Equal => 0,
        Ordering::Greater => 1,
    }
}
fn geometry_failure(error: GeometryFailure) -> String {
    match error.kind {
        GeometryFailureKind::ZeroSlopeDenominator => "ZeroSlopeDenominator".into(),
        kind => format!("INTERNAL:{kind:?}"),
    }
}

struct Stub {
    failure: Option<(String, usize)>,
    counts: [usize; 3],
}
impl Stub {
    fn new(mode: &str) -> Self {
        Self {
            failure: mode
                .split_once(':')
                .map(|(kind, n)| (kind.into(), n.parse().unwrap())),
            counts: [0; 3],
        }
    }
    fn step(&mut self, at: usize, kind: &str) -> Result<(), String> {
        self.counts[at] += 1;
        if self
            .failure
            .as_ref()
            .is_some_and(|(name, n)| name == kind && *n == self.counts[at])
        {
            Err(format!(
                "callback_{}_refused",
                match kind {
                    "L" => "local",
                    "C" => "compare",
                    _ => "edge",
                }
            ))
        } else {
            Ok(())
        }
    }
}
impl FareyPredicates for Stub {
    type Error = String;
    fn local_candidate(
        &mut self,
        ordinal: usize,
        candidate: &PrimitiveVector,
    ) -> Result<bool, String> {
        self.step(0, "L")?;
        Ok(
            (magnitude(&candidate.0) + magnitude(&candidate.1) + UBig::from(ordinal))
                % UBig::from(3u8)
                != UBig::ZERO,
        )
    }
    fn candidate_compare(
        &mut self,
        _ordinal: usize,
        left: &PrimitiveVector,
        right: &PrimitiveVector,
    ) -> Result<i8, String> {
        self.step(1, "C")?;
        let rank = |v: &PrimitiveVector| {
            let n = (&v.0 + &v.1) % UBig::from(7u8);
            if n < IBig::ZERO { n + 7u8 } else { n }
        };
        Ok(sign(rank(left).cmp(&rank(right))))
    }
    fn edge_valid(
        &mut self,
        _ordinal: usize,
        left: Endpoint<'_>,
        right: Endpoint<'_>,
    ) -> Result<bool, String> {
        self.step(2, "E")?;
        let value = |endpoint| match endpoint {
            Endpoint::Incoming => PrimitiveVector(IBig::ONE, IBig::ZERO),
            Endpoint::Outgoing => PrimitiveVector(IBig::ZERO, IBig::ONE),
            Endpoint::Primitive(value) => value.clone(),
        };
        let (a, b) = (value(left), value(right));
        Ok(&a.0 * &b.1 - &a.1 * &b.0 > IBig::ZERO && &a.0 * &b.0 + &a.1 * &b.1 >= IBig::ZERO)
    }
}

enum Backend {
    Stub(Stub),
    Geometry(GeometryAdapter),
}
struct Trace {
    backend: Backend,
    events: Vec<String>,
}
impl Trace {
    fn log(
        &mut self,
        kind: &str,
        ordinal: usize,
        args: String,
        result: Result<i8, String>,
    ) -> Result<i8, String> {
        self.events.push(format!(
            "{kind}:{ordinal}:{args}:{}",
            match &result {
                Ok(value) => value.to_string(),
                Err(error) => format!("!{error}"),
            }
        ));
        result
    }
}
impl FareyPredicates for Trace {
    type Error = String;
    fn local_candidate(
        &mut self,
        ordinal: usize,
        candidate: &PrimitiveVector,
    ) -> Result<bool, String> {
        let result = match &mut self.backend {
            Backend::Stub(p) => p.local_candidate(ordinal, candidate),
            Backend::Geometry(p) => p
                .local_candidate(ordinal, candidate)
                .map_err(geometry_failure),
        };
        self.log("L", ordinal, vector_text(candidate), result.map(i8::from))
            .map(|value| value != 0)
    }
    fn candidate_compare(
        &mut self,
        ordinal: usize,
        left: &PrimitiveVector,
        right: &PrimitiveVector,
    ) -> Result<i8, String> {
        let result = match &mut self.backend {
            Backend::Stub(p) => p.candidate_compare(ordinal, left, right),
            Backend::Geometry(p) => p
                .candidate_compare(ordinal, left, right)
                .map_err(geometry_failure),
        };
        self.log(
            "C",
            ordinal,
            format!("{}|{}", vector_text(left), vector_text(right)),
            result,
        )
    }
    fn edge_valid(
        &mut self,
        ordinal: usize,
        left: Endpoint<'_>,
        right: Endpoint<'_>,
    ) -> Result<bool, String> {
        let result = match &mut self.backend {
            Backend::Stub(p) => p.edge_valid(ordinal, left, right),
            Backend::Geometry(p) => p.edge_valid(ordinal, left, right).map_err(geometry_failure),
        };
        self.log(
            "E",
            ordinal,
            format!("{}|{}", endpoint_text(left), endpoint_text(right)),
            result.map(i8::from),
        )
        .map(|value| value != 0)
    }
}

fn error(error: FareyFailure<String>) -> String {
    match error {
        FareyFailure::Work(value) => value.to_string(),
        FareyFailure::Predicate(value) => value,
        value => format!("INTERNAL:{value:?}"),
    }
}
fn answer(result: Result<SearchResult, FareyFailure<String>>) -> String {
    match result {
        Ok(result) => format!(
            "OK\t{}\t{}\t{}\t{}",
            result.height.map_or_else(|| "-".into(), |h| h.to_string()),
            result
                .fan
                .as_deref()
                .map_or_else(|| "NONE".into(), vectors_text),
            counts(&result.previous_counts),
            counts(&result.counts)
        ),
        Err(value) => format!("ERROR:{}", error(value)),
    }
}
fn geometry(fields: &[&str]) -> GeometryAdapter {
    let (a, b, c) = (
        rational(fields[6]),
        rational(fields[7]),
        rational(fields[8]),
    );
    let (metric, _) =
        Metric2::from_gram([[a, b.clone()], [b, c]], fields[9].parse().unwrap()).unwrap();
    let vectors = fields[13]
        .split('|')
        .map(|item| {
            let (x, y) = item.split_once('~').unwrap();
            Vector2::new(
                Radical::from_text(x).unwrap().0,
                Radical::from_text(y).unwrap().0,
            )
        })
        .collect();
    GeometryAdapter::new(
        DensityTransaction::new(metric),
        vectors,
        fields[11].parse().unwrap(),
        fields[10].parse().unwrap(),
        if fields[12] == "narrow" {
            WindowLaw::NarrowBand
        } else {
            WindowLaw::Voronoi
        },
    )
    .unwrap()
}
fn run(f: &[&str]) -> String {
    match f[0] {
        "CELL" => {
            let result = cell_segments(
                f[2] == "1",
                f[3].parse().unwrap(),
                &[
                    rational(f[4]),
                    rational(f[5]),
                    rational(f[6]),
                    rational(f[7]),
                ],
            );
            if result.is_empty() {
                "-".into()
            } else {
                result
                    .iter()
                    .map(segment_text)
                    .collect::<Vec<_>>()
                    .join("@")
            }
        }
        "BOUNDS" => {
            let (lo, hi) =
                numerator_bounds(&rational(f[2]), &rational(f[3]), &f[4].parse().unwrap());
            format!("{lo},{hi}")
        }
        "SHELL" => {
            let mut trace = Trace {
                backend: Backend::Stub(Stub::new(f[6])),
                events: Vec::new(),
            };
            let mut budget = WorkBudget::new(f[5].parse().unwrap());
            let result = segment_candidates(
                &segment(f[2]),
                f[4].parse().unwrap(),
                &f[3].parse().unwrap(),
                &mut budget,
                &mut trace,
            );
            let result = match result {
                Ok(candidates) => format!("CANDIDATES:{}", vectors_text(&candidates)),
                Err(value) => format!("ERROR:{}", error(value)),
            };
            format!(
                "{result}\tWORK:{},{}\tTRACE:{}",
                budget.shell_probes,
                budget.order_steps,
                trace.events.join("@")
            )
        }
        "SEARCH" | "GEOMETRY" => {
            let backend = if f[0] == "SEARCH" {
                Backend::Stub(Stub::new(f[5]))
            } else {
                Backend::Geometry(geometry(f))
            };
            let mut trace = Trace {
                backend,
                events: Vec::new(),
            };
            let mut budget = WorkBudget::new(f[2].parse().unwrap());
            let result = search_global_height(
                &plans(f[4]),
                &f[3].parse().unwrap(),
                &mut budget,
                &mut trace,
                &mut CanonicalOrdering,
            );
            format!(
                "{}\tWORK:{},{}\tTRACE:{}",
                answer(result),
                budget.shell_probes,
                budget.order_steps,
                trace.events.join("@")
            )
        }
        op => panic!("unknown {op}"),
    }
}
fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        if line.is_empty() {
            continue;
        }
        let f: Vec<_> = line.split('\t').collect();
        println!("{}\t{}\t{}", f[0], f[1], run(&f));
    }
}
