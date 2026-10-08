//! `wavefront.coverage._coverage_at`: the coverage of a face partition at time `alpha`, whole.
//!
//! The operation follows the oracle's sequence of cost-bearing calls exactly, because the cost IS part of the answer
//! (six budget articles, the sign counters, the four memory tables): both refusals first, the face lines, ONE
//! `prime_universe_remembered` (store absent / miss / hit), then per face `clip_to_halfplane` (one `radical` for the
//! front, one `sign` per vertex in vertex order, one `_divide_with_prime_universe` per crossing edge in edge order)
//! and the doubled shoelace area; the running total adds every face in order. Everything between those calls is free
//! to be done any cheaper way that yields the same canonical values:
//!
//! * `value = a*x + b*y - c - front` equals `base - front` with `base = a*x + b*y - c` (every coefficient of all of
//!   them is a `Fraction`, the canonical value is unique), and `base` does not depend on `alpha`: it is computed once
//!   per face and kept with the partition;
//! * the sign of `base - front` is read off the enclosure of `base` (cached per point) and the one term of the front
//!   when the front's radicand is not among the point's: the termwise enclosure of the difference IS the sum of the
//!   two enclosures then, and it is scaled by a positive number, which decides nothing differently; otherwise (and
//!   whenever the enclosure does not decide) the merged integer items go through `exact::sign_items`;
//! * the doubled area of a face that is not clipped is a pure function of its original points (no budget, no memory,
//!   no sign counter): computed once per face as well.
//!
//! * the cut point of an edge is AFFINE in `alpha`: the front `alpha*kappa*sqrt(m)` of a face is linear in `alpha`
//!   (`kappa` and `m` come from the face's `q` alone), the divisor `base(cur) - base(next)` does not depend on `alpha`
//!   at all, so `x0 + (x1 - x0) * (value(cur) / divisor)` is `constant - alpha * slope` with two canonical sums per
//!   coordinate, found ONCE per edge from the conjugation plan of the divisor ([`exact::conjugation_plan`]). Every call
//!   still pays the memory and budget the oracle's division pays (`squarefree_split` of each prime of the plan, in the
//!   plan's order, the oracle's own fallback when the memory answers anything but `(1, p)`); only the products and
//!   gcds of the division are gone. The plans are kept per face and re-validated on every call against the `kappa`,
//!   the radicand and the prime universe they were built under (a memory poisoned between calls rebuilds them).
//!
//! The answer names its vertices instead of copying them: a clipped polygon is a list of [`Vertex`], a vertex being
//! either an index into the face's ORIGINAL points (the oracle keeps those very objects) or a new cut point. A face
//! that is not cut at all is [`Clipped::Unchanged`] (the oracle returns the original tuple object itself).
//!
//! An exhaustion (or any refusal of the arithmetic) is a `Result::Err` in [`Run::outcome`]; the universe record a
//! store miss produced is in [`Run::record`] whatever happened afterwards, because the oracle writes it into the
//! store before the faces are touched.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::Instant;

use cftuv_canon::{QValue, UniverseRecord};

use crate::exact::{self, ExactCtx, ExactError, UniverseStore};
use crate::fused::sum_of_products;
use crate::num::{IBig, UBig};
use crate::products::{Items, ProductMemo};
use crate::rat::{Coef, Rat};
use crate::sqrt_sum::{integer_enclosure, SqrtSum, Term, SIGN_FILTER_BITS};

/// `(x, y)` of a face vertex.
pub type Point = (SqrtSum, SqrtSum);

/// `SupportLineV1`: `a*x + b*y = c + t*sqrt(q)`. Integers in the kernel; any rational is carried.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Line {
    pub a: Rat,
    pub b: Rat,
    pub c: Rat,
    pub q: Rat,
}

/// Diagnostic phase timers in nanoseconds (`front`, `signs`, `cuts`, `areas`, `total`), off unless [`set_profiling`]
/// turns them on; `examples/coverage_profile.rs` reads them. Off, a phase boundary costs one relaxed load.
pub static PHASE_NANOS: [AtomicU64; 5] = [const { AtomicU64::new(0) }; 5];
pub const PHASES: [&str; 5] = ["front", "signs", "cuts", "areas", "total"];
static PROFILING: AtomicBool = AtomicBool::new(false);

pub fn set_profiling(on: bool) {
    PROFILING.store(on, Ordering::Relaxed);
}

fn started() -> Option<Instant> {
    PROFILING.load(Ordering::Relaxed).then(Instant::now)
}

fn lap(phase: usize, since: Option<Instant>) {
    if let Some(since) = since {
        PHASE_NANOS[phase].fetch_add(u64::try_from(since.elapsed().as_nanos()).unwrap_or(u64::MAX), Ordering::Relaxed);
    }
}

fn locked<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

/// One face of the partition: its contour, its supporting line (`None` only for a face assembled by hand) and the
/// pure per-face caches.
#[derive(Debug)]
pub struct Face {
    points: Vec<Point>,
    line: Option<Line>,
    base: OnceLock<Vec<SqrtSum>>,
    bounds: OnceLock<Vec<Bounds>>,
    area: OnceLock<SqrtSum>,
    plans: Mutex<FacePlans>,
}

/// The integer form of a base value and its enclosure at the filter width: `base = (sum a_m sqrt(m)) / common`,
/// `[low, high]` bound `2^bits * sum a_m sqrt(m)`.
#[derive(Debug)]
struct Bounds {
    common: UBig,
    low: IBig,
    high: IBig,
}

/// `T(alpha) = constant - alpha * slope`: the cut of an edge as a function of `alpha`, per coordinate.
#[derive(Debug)]
struct Affine {
    constant: SqrtSum,
    slope: SqrtSum,
}

#[derive(Debug)]
struct EdgePlan {
    /// The primes the oracle's conjugation loop asks `squarefree_split` about, in order.
    primes: Vec<UBig>,
    x: Affine,
    y: Affine,
}

#[derive(Debug, Default)]
enum EdgeState {
    #[default]
    Unplanned,
    /// The divisor needs more than the universe proves: every call runs the oracle's division.
    Slow,
    Fast(Box<EdgePlan>),
}

/// The plans of a face and what they were built under.
#[derive(Debug, Default)]
struct FacePlans {
    epoch: u64,
    /// `(kappa, m)` with `front = alpha * kappa * sqrt(m)`.
    front: Option<(Rat, UBig)>,
    edges: Vec<EdgeState>,
}

impl FacePlans {
    /// Brings the plans to this call's universe and front; `false` when the front is zero (no alpha-linear form: the
    /// division runs as the oracle runs it).
    fn sync(&mut self, epoch: u64, front: &SqrtSum, alpha: &Rat, edges: usize) -> bool {
        let key = match front.terms() {
            [term] => match term.coef.value().div(alpha) {
                Ok(kappa) => Some((kappa, term.radicand.clone())),
                Err(_) => None,
            },
            _ => None,
        };
        let Some(key) = key else {
            return false;
        };
        if self.epoch != epoch || self.front.as_ref() != Some(&key) || self.edges.len() != edges {
            self.epoch = epoch;
            self.front = Some(key);
            self.edges.clear();
            self.edges.resize_with(edges, EdgeState::default);
        }
        true
    }
}

impl Face {
    pub fn new(points: Vec<Point>, line: Option<Line>) -> Face {
        Face { points, line, base: OnceLock::new(), bounds: OnceLock::new(), area: OnceLock::new(), plans: Mutex::new(FacePlans::default()) }
    }

    pub fn points(&self) -> &[Point] {
        &self.points
    }

    pub fn line(&self) -> Option<&Line> {
        self.line.as_ref()
    }

    /// `a*x + b*y - c` per point: the part of `_value` that does not depend on `alpha`.
    pub fn base_values(&self, line: &Line) -> &[SqrtSum] {
        self.base.get_or_init(|| {
            let offset = SqrtSum::rational(&line.c);
            self.points.iter().map(|(x, y)| x.scaled(&line.a).add(&y.scaled(&line.b)).sub(&offset)).collect()
        })
    }

    fn bounds(&self, line: &Line) -> &[Bounds] {
        self.bounds.get_or_init(|| {
            self.base_values(line)
                .iter()
                .map(|value| {
                    let form = value.int_form();
                    let (low, high) = integer_enclosure(&form.items, SIGN_FILTER_BITS);
                    Bounds { common: form.common.clone(), low, high }
                })
                .collect()
        })
    }

    /// The unclipped area if a call has computed it already.
    pub fn cached_area(&self) -> Option<&SqrtSum> {
        self.area.get()
    }

    /// `doubled_shoelace(points)` of the unclipped contour (zero below three points, as the oracle's guard has it).
    pub fn original_area(&self, memo: &mut ProductMemo) -> &SqrtSum {
        self.area.get_or_init(|| {
            if self.points.len() >= 3 {
                let refs: Vec<&Point> = self.points.iter().collect();
                doubled_shoelace(&refs, memo)
            } else {
                SqrtSum::zero()
            }
        })
    }
}

/// A face partition ready for the native operation: built once, used for every `alpha`.
#[derive(Debug)]
pub struct Partition {
    exact: bool,
    faces: Vec<Face>,
    q_values: Vec<QValue>,
    missing_line: Option<usize>,
    planned: bool,
    /// The prime universe the face plans were built under, and a counter that moves when it changes.
    universe: Mutex<(u64, Vec<UBig>)>,
}

impl Partition {
    /// A partition whose outcome is `FaceOutcome.EXACT` (`exact`) or anything else (the faces are never read).
    pub fn new(exact: bool, faces: Vec<Face>) -> Partition {
        let missing_line = faces.iter().position(|face| face.line.is_none());
        let q_values = if missing_line.is_some() {
            Vec::new()
        } else {
            faces.iter().filter_map(|face| face.line.as_ref()).map(|line| q_value(&line.q)).collect()
        };
        Partition { exact, faces, q_values, missing_line, planned: true, universe: Mutex::new((0, Vec::new())) }
    }

    /// The same partition with the per-edge plans switched off: every division runs as the oracle runs it (the
    /// differential tests and the profile compare both roads).
    pub fn without_plans(mut self) -> Partition {
        self.planned = false;
        self
    }

    fn epoch_of(&self, universe: &[UBig]) -> u64 {
        let mut known = locked(&self.universe);
        if known.1 != universe {
            known.0 += 1;
            known.1 = universe.to_vec();
        }
        known.0
    }

    pub fn is_exact(&self) -> bool {
        self.exact
    }

    pub fn faces(&self) -> &[Face] {
        &self.faces
    }

    /// The `q` of every face line in face order: the key material of `prime_universe_remembered`.
    pub fn q_values(&self) -> &[QValue] {
        &self.q_values
    }
}

fn q_value(q: &Rat) -> QValue {
    QValue { numerator: q.numerator().clone(), denominator: q.denominator().clone() }
}

/// The two refusals of the oracle that carry no faces.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal {
    /// `partition.outcome is not FaceOutcome.EXACT` (`CoverageOutcome.PARTITION_IS_NOT_EXACT`).
    PartitionNotExact,
    /// `alpha < 0` (`CoverageOutcome.ALPHA_IS_NEGATIVE`).
    AlphaNegative,
}

/// A vertex of a clipped polygon.
#[derive(Debug, Clone, PartialEq)]
pub enum Vertex {
    /// The original point with this index (the oracle appends the very same object).
    Kept(usize),
    /// A new point where the front crosses an edge: `x0 + (x1 - x0) * share`.
    Cut(Box<Point>),
}

/// What `clip_to_halfplane` made of a face.
#[derive(Debug, Clone, PartialEq)]
pub enum Clipped {
    /// Every vertex is behind the front: the original points tuple itself.
    Unchanged,
    /// Every vertex is ahead of it: `()`.
    Empty,
    Polygon(Vec<Vertex>),
}

/// The doubled area of a face's coverage.
#[derive(Debug, Clone, PartialEq)]
pub enum Area {
    /// The cached shoelace of the unclipped face ([`Face::original_area`]).
    Original,
    Fresh(SqrtSum),
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceOut {
    pub clipped: Clipped,
    pub area: Area,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Answer {
    Refused(Refusal),
    Exact { faces: Vec<FaceOut>, total: SqrtSum },
}

/// Why a run produced no answer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CoverageError {
    /// `_line_of` raised `ValueError`: this face has no supporting line (nothing was spent before).
    MissingLine { face: usize },
    /// A refusal of the exact arithmetic: budget exhaustion, negative radicand, ...
    Exact(ExactError),
}

impl From<ExactError> for CoverageError {
    fn from(error: ExactError) -> CoverageError {
        CoverageError::Exact(error)
    }
}

/// What `clip_to_halfplane` records into its `trace` for one face (the sign trace `wavefront.coverage_template` reads): the signs of the points
/// against the front and the values `a*x + b*y - c - front` they are the signs of.
#[derive(Debug, Clone, PartialEq)]
pub struct Trace {
    pub signs: Vec<i8>,
    pub values: Vec<SqrtSum>,
}

/// The result of one call: the answer or the refusal, the store record a miss produced (whatever happened after it: the oracle stores it
/// first) and, when the call asked for them, the traces of the faces whose clipping completed (also when a later face refused, as the
/// oracle's `traces` list holds them).
#[derive(Debug)]
pub struct Run {
    pub record: Option<UniverseRecord>,
    pub outcome: Result<Answer, CoverageError>,
    pub traces: Vec<Trace>,
}

/// `_coverage_at(partition, alpha, budget, store)` on the native memory, budget and counters in `ctx`; `budgeted` is `budget is not None`
/// (the price of a store hit is replayed into a budget and into nothing else, see `exact::prime_universe`); `traced` is `traces is not None`.
pub fn coverage_at(ctx: &mut ExactCtx<'_>, partition: &Partition, alpha: &Rat, store: UniverseStore<'_>, budgeted: bool, traced: bool) -> Run {
    let refused = |outcome| Run { record: None, outcome, traces: Vec::new() };
    if !partition.exact {
        return refused(Ok(Answer::Refused(Refusal::PartitionNotExact)));
    }
    if alpha.signum() < 0 {
        return refused(Ok(Answer::Refused(Refusal::AlphaNegative)));
    }
    if let Some(face) = partition.missing_line {
        return refused(Err(CoverageError::MissingLine { face }));
    }
    let (universe, record) = match exact::prime_universe(ctx, partition.q_values(), store, budgeted) {
        Ok(found) => found,
        Err(error) => return refused(Err(error.into())),
    };
    let mut traces = Vec::new();
    let outcome = clip_all(ctx, partition, alpha, &universe, traced.then_some(&mut traces));
    Run { record, outcome, traces }
}

fn clip_all(ctx: &mut ExactCtx<'_>, partition: &Partition, alpha: &Rat, universe: &[UBig], mut traces: Option<&mut Vec<Trace>>) -> Result<Answer, CoverageError> {
    let epoch = partition.epoch_of(universe);
    let mut faces = Vec::with_capacity(partition.faces.len());
    let mut total = SqrtSum::zero();
    for face in &partition.faces {
        let Some(line) = face.line.as_ref() else {
            return Err(CoverageError::MissingLine { face: faces.len() });
        };
        // Эталон публикует локальную трассу только после успешного усечения всей грани.
        let mut face_traces = Vec::new();
        let clipped = clip_to_halfplane(ctx, face, line, alpha, universe, partition.planned.then_some(epoch), traces.as_ref().map(|_| &mut face_traces))?;
        if let Some(traces) = traces.as_deref_mut() {
            traces.extend(face_traces);
        }
        let timer = started();
        let area = match &clipped {
            Clipped::Unchanged => Area::Original,
            Clipped::Empty => Area::Fresh(SqrtSum::zero()),
            Clipped::Polygon(vertices) if vertices.len() >= 3 => {
                let refs: Vec<&Point> = vertices
                    .iter()
                    .map(|vertex| match vertex {
                        Vertex::Kept(index) => &face.points[*index],
                        Vertex::Cut(point) => &**point,
                    })
                    .collect();
                Area::Fresh(doubled_shoelace(&refs, ctx.products))
            }
            Clipped::Polygon(_) => Area::Fresh(SqrtSum::zero()),
        };
        lap(3, timer);
        let timer = started();
        let doubled = match &area {
            // below three points the oracle skips the shoelace and adds a zero
            Area::Original => face.original_area(ctx.products),
            Area::Fresh(value) => value,
        };
        total = total.add(doubled);
        lap(4, timer);
        faces.push(FaceOut { clipped, area });
    }
    Ok(Answer::Exact { faces, total })
}

/// The front `c * sqrt(m)` of a face for one call, with the floor root the enclosure needs.
struct Front<'a> {
    numerator: &'a IBig,
    denominator: IBig,
    floor_root: IBig,
    ceiling_root: IBig,
}

impl<'a> Front<'a> {
    fn of(front: &'a SqrtSum, products: &mut ProductMemo) -> Option<Front<'a>> {
        let term = front.terms().first()?;
        if term.radicand.is_one() {
            return None;
        }
        let floor_root = IBig::from(products.floor_root(&term.radicand, 2 * SIGN_FILTER_BITS));
        let ceiling_root = &floor_root + IBig::ONE;
        let value = term.coef.value();
        Some(Front { numerator: value.numerator(), denominator: IBig::from(value.denominator().clone()), floor_root, ceiling_root })
    }
}

/// `(base - front).sign(budget=...)` with the counters, the memory and the budget the oracle moves.
fn sign_of_difference(ctx: &mut ExactCtx<'_>, face: &Face, line: &Line, index: usize, front: &SqrtSum, split: Option<&Front<'_>>) -> Result<i8, ExactError> {
    let base = &face.base_values(line)[index];
    let Some(front_term) = front.terms().first() else {
        // a zero front: the value is the base itself
        return exact::sign(ctx, base, SIGN_FILTER_BITS);
    };
    let form = base.int_form();
    let merged = base.terms().binary_search_by(|term| term.radicand.cmp(&front_term.radicand));
    if let (Some(split), Err(_), false) = (split, merged, base.is_zero()) {
        // The radicand of the front is not among the point's: the difference's items are the point's items (scaled by
        // the front's denominator) and one more, so its enclosure is the sum of the two parts.
        let bounds = &face.bounds(line)[index];
        let weight = IBig::from(bounds.common.clone()) * split.numerator;
        let low = &split.denominator * &bounds.low - &weight * &split.ceiling_root;
        let high = &split.denominator * &bounds.high - &weight * &split.floor_root;
        ctx.counts.total += 1;
        if let Some(certified) = exact::certify(&low, &high) {
            ctx.counts.closed_by_enclosure += 1;
            return Ok(certified);
        }
        ctx.counts.closed_by_conjugation += 1;
        let items = difference_items(&form.items, &form.common, front_term, &merged);
        return exact::exact_sign_after_enclosure(ctx, &items, SIGN_FILTER_BITS);
    }
    let items = difference_items(&form.items, &form.common, front_term, &merged);
    exact::sign_items(ctx, &items, SIGN_FILTER_BITS)
}

/// `denominator * common * (base - front)` as integer items: the base items times the front's denominator, the
/// front's term subtracted (merged when the radicand is there, which may cancel it).
fn difference_items(items: &Items, common: &UBig, front: &Term, merged: &Result<usize, usize>) -> Items {
    let value = front.coef.value();
    let scale = IBig::from(value.denominator().clone());
    let weight = IBig::from(common.clone()) * value.numerator();
    let mut out: Items = items.iter().map(|(radicand, numerator)| (radicand.clone(), numerator * &scale)).collect();
    match merged {
        Ok(position) => {
            out[*position].1 -= &weight;
            if out[*position].1.is_zero() {
                out.remove(*position);
            }
        }
        Err(position) => out.insert(*position, (front.radicand.clone(), -weight)),
    }
    out
}

/// `clip_to_halfplane(points, line, alpha, prime_universe=universe, budget=budget)`. `planned`: the epoch of the
/// universe when the per-edge plans are on.
fn clip_to_halfplane(ctx: &mut ExactCtx<'_>, face: &Face, line: &Line, alpha: &Rat, universe: &[UBig], planned: Option<u64>, trace: Option<&mut Vec<Trace>>) -> Result<Clipped, ExactError> {
    let timer = started();
    let front = exact::radical(ctx, alpha, &line.q)?;
    let split = Front::of(&front, ctx.products);
    lap(0, timer);
    let timer = started();
    let mut signs = Vec::with_capacity(face.points.len());
    for index in 0..face.points.len() {
        signs.push(sign_of_difference(ctx, face, line, index, &front, split.as_ref())?);
    }
    lap(1, timer);
    if let Some(trace) = trace {
        // Локальная запись после знаков ещё не опубликована вызывающему: усечение может исчерпать бюджет.
        trace.push(Trace { signs: signs.clone(), values: face.base_values(line).iter().map(|base| base.sub(&front)).collect() });
    }
    if signs.iter().all(|sign| *sign <= 0) {
        return Ok(Clipped::Unchanged);
    }
    if signs.iter().all(|sign| *sign >= 0) {
        return Ok(Clipped::Empty);
    }
    let size = face.points.len();
    let mut plans = planned.map(|_| locked(&face.plans));
    let usable = match (&mut plans, planned) {
        (Some(plans), Some(epoch)) => plans.sync(epoch, &front, alpha, size),
        _ => false,
    };
    let mut result = Vec::with_capacity(size + 2);
    for current in 0..size {
        let following = (current + 1) % size;
        if signs[current] <= 0 {
            result.push(Vertex::Kept(current));
        }
        if signs[current] == 0 || signs[following] == 0 {
            continue;
        }
        if (signs[current] > 0) == (signs[following] > 0) {
            continue;
        }
        // The crossing: t = v0 / (v0 - v1), an exact division; the point is `x0 + (x1 - x0) * t`.
        let timer = started();
        let cut = match plans.as_mut().filter(|_| usable) {
            Some(plans) => cut_by_plan(ctx, face, line, plans, alpha, &front, (current, following), universe)?,
            None => cut_by_division(ctx, face, line, &front, (current, following), universe)?,
        };
        lap(2, timer);
        result.push(Vertex::Cut(Box::new(cut)));
    }
    Ok(Clipped::Polygon(result))
}

/// The oracle's own road: the division, then the two coordinates.
fn cut_by_division(ctx: &mut ExactCtx<'_>, face: &Face, line: &Line, front: &SqrtSum, edge: (usize, usize), universe: &[UBig]) -> Result<Point, ExactError> {
    let (current, following) = edge;
    let base = face.base_values(line);
    let (value, next) = (base[current].sub(front), base[following].sub(front));
    let divisor = value.sub(&next);
    let share = exact::divide_with_prime_universe(ctx, &value, &divisor, universe)?;
    let ((x0, y0), (x1, y1)) = (&face.points[current], &face.points[following]);
    let x = x0.add(&x1.sub(x0).mul(&share, ctx.products));
    let y = y0.add(&y1.sub(y0).mul(&share, ctx.products));
    Ok((x, y))
}

/// The plan's road: the same cost as the division (the memory and the budget are asked about the same primes in the
/// same order), the point from the two affine forms of the edge.
#[allow(clippy::too_many_arguments)]
fn cut_by_plan(
    ctx: &mut ExactCtx<'_>,
    face: &Face,
    line: &Line,
    plans: &mut FacePlans,
    alpha: &Rat,
    front: &SqrtSum,
    edge: (usize, usize),
    universe: &[UBig],
) -> Result<Point, ExactError> {
    let (current, following) = edge;
    if matches!(plans.edges[current], EdgeState::Unplanned) {
        let state = match &plans.front {
            Some((kappa, radicand)) => build_edge_plan(ctx, face, line, (kappa, radicand), edge, universe),
            None => EdgeState::Slow,
        };
        plans.edges[current] = state;
    }
    if let EdgeState::Fast(plan) = &plans.edges[current] {
        let mut agrees = true;
        for prime in &plan.primes {
            let (outside, inside) = ctx.memory.squarefree_split_unsigned(prime, ctx.budget)?;
            if outside != UBig::ONE || inside != *prime {
                agrees = false;
                break;
            }
        }
        if agrees {
            let (x0, y0) = &face.points[current];
            let x = x0.add(&plan.x.constant.sub(&plan.x.slope.scaled(alpha)));
            let y = y0.add(&plan.y.constant.sub(&plan.y.slope.scaled(alpha)));
            return Ok((x, y));
        }
    }
    cut_by_division(ctx, face, line, front, (current, following), universe)
}

/// `T(alpha) = (x1 - x0) * (v0 / d)` with `v0 = base(cur) - alpha*kappa*sqrt(m)` and `d = base(cur) - base(next)`:
/// `d` is the same for every `alpha`, so `v0 / d = base(cur)*C/N - alpha * (kappa*sqrt(m))*C/N` for the conjugation
/// plan `(C, N)` of `d`.
fn build_edge_plan(ctx: &mut ExactCtx<'_>, face: &Face, line: &Line, front: (&Rat, &UBig), edge: (usize, usize), universe: &[UBig]) -> EdgeState {
    let (current, following) = edge;
    let base = face.base_values(line);
    let divisor = base[current].sub(&base[following]);
    let Some(plan) = exact::conjugation_plan(&divisor, universe, ctx.products) else {
        return EdgeState::Slow;
    };
    let Ok(inverse) = Rat::one().div(&plan.rational) else {
        return EdgeState::Slow;
    };
    let unit = SqrtSum::from_terms_unchecked(vec![Term { radicand: front.1.clone(), coef: Coef::fraction(front.0.clone()) }]);
    let constant = base[current].mul(&plan.conjugate, ctx.products).scaled(&inverse);
    let slope = unit.mul(&plan.conjugate, ctx.products).scaled(&inverse);
    let ((x0, y0), (x1, y1)) = (&face.points[current], &face.points[following]);
    let (dx, dy) = (x1.sub(x0), y1.sub(y0));
    let affine = |difference: &SqrtSum, ctx: &mut ExactCtx<'_>| Affine { constant: difference.mul(&constant, ctx.products), slope: difference.mul(&slope, ctx.products) };
    let x = affine(&dx, ctx);
    let y = affine(&dy, ctx);
    EdgeState::Fast(Box::new(EdgePlan { primes: plan.primes, x, y }))
}

/// `doubled_shoelace(points)` for three or more points: the fan from the first point, one `sum_of_products`.
pub fn doubled_shoelace(points: &[&Point], memo: &mut ProductMemo) -> SqrtSum {
    let (origin_x, origin_y) = points[0];
    let steps: Vec<Point> = points[1..].iter().map(|(x, y)| (x.sub(origin_x), y.sub(origin_y))).collect();
    let mut products: Vec<(&SqrtSum, &SqrtSum, IBig)> = Vec::with_capacity(2 * steps.len());
    for pair in steps.windows(2) {
        let ((previous_x, previous_y), (next_x, next_y)) = (&pair[0], &pair[1]);
        products.push((previous_x, next_y, IBig::ONE));
        products.push((previous_y, next_x, -IBig::ONE));
    }
    sum_of_products(&products, memo)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rat::Coef;
    use crate::session::Session;
    use crate::sqrt_sum::{SignCounts, Term};
    use cftuv_canon::WorkBudget;

    fn rat(n: i64, d: i64) -> Rat {
        Rat::new(IBig::from(n), IBig::from(d)).unwrap()
    }

    fn rational(n: i64) -> SqrtSum {
        SqrtSum::rational(&rat(n, 1))
    }

    fn point(x: i64, y: i64) -> Point {
        (rational(x), rational(y))
    }

    /// The unit square `[0, 2]^2` and the front of its bottom edge (`y = 0`, normal (0, 1), `q = 1`).
    fn square_face() -> Face {
        let points = vec![point(0, 0), point(2, 0), point(2, 2), point(0, 2)];
        // a = 0, b = 1, c = 0: `y = t` (the normal of the bottom edge looks up)
        Face::new(points, Some(Line { a: rat(0, 1), b: rat(1, 1), c: rat(0, 1), q: rat(1, 1) }))
    }

    struct World {
        session: Session,
        budget: WorkBudget,
        counts: SignCounts,
    }

    impl World {
        fn new() -> World {
            World { session: Session::new(), budget: WorkBudget::unlimited(), counts: SignCounts::default() }
        }

        fn run(&mut self, partition: &Partition, alpha: &Rat) -> Run {
            let mut ctx = ExactCtx { memory: &mut self.session.memory, budget: &mut self.budget, counts: &mut self.counts, products: &mut self.session.products };
            coverage_at(&mut ctx, partition, alpha, UniverseStore::Absent, false, false)
        }
    }

    fn area_of(run: Run) -> (Vec<FaceOut>, SqrtSum) {
        match run.outcome.unwrap() {
            Answer::Exact { faces, total } => (faces, total),
            other => panic!("expected an exact answer, got {other:?}"),
        }
    }

    #[test]
    fn a_front_cuts_the_face_and_the_area_is_exact() {
        let partition = Partition::new(true, vec![square_face()]);
        let mut world = World::new();
        // alpha = 1: the half-plane `y <= 1` keeps the lower half of the square: doubled area 2 * (2 * 1) = 4
        let (faces, total) = area_of(world.run(&partition, &rat(1, 1)));
        assert_eq!(faces.len(), 1);
        let Clipped::Polygon(vertices) = &faces[0].clipped else { panic!("{:?}", faces[0].clipped) };
        assert_eq!(vertices.len(), 4);
        assert!(matches!(vertices[0], Vertex::Kept(0)) && matches!(vertices[1], Vertex::Kept(1)));
        assert!(matches!(vertices[2], Vertex::Cut(_)) && matches!(vertices[3], Vertex::Cut(_)));
        assert_eq!(total, rational(4));
        assert_eq!(world.counts.total, 4, "one sign per vertex");
        // alpha = 0: only the edge on the front itself is behind: the area is zero and the polygon degenerate
        let (_, total) = area_of(world.run(&partition, &rat(0, 1)));
        assert!(total.is_zero());
    }

    #[test]
    fn a_front_past_the_face_leaves_it_unchanged_and_the_cached_area_is_the_total() {
        let partition = Partition::new(true, vec![square_face()]);
        let mut world = World::new();
        let (faces, total) = area_of(world.run(&partition, &rat(5, 1)));
        assert_eq!(faces[0].clipped, Clipped::Unchanged);
        assert_eq!(faces[0].area, Area::Original);
        assert_eq!(total, rational(8));
        assert_eq!(*partition.faces()[0].original_area(&mut world.session.products), rational(8));
    }

    #[test]
    fn irrational_fronts_cut_with_a_square_root() {
        // q = 2: the front sits at `y = alpha * sqrt(2)`; alpha = 1 puts it at y = sqrt(2) inside [0, 2]
        let mut face = square_face();
        face.line.as_mut().unwrap().q = rat(2, 1);
        let partition = Partition::new(true, vec![face]);
        let mut world = World::new();
        let (faces, total) = area_of(world.run(&partition, &rat(1, 1)));
        let Clipped::Polygon(vertices) = &faces[0].clipped else { panic!() };
        let Vertex::Cut(cut) = &vertices[2] else { panic!() };
        // the cut point (2, sqrt(2)): its y has the single term sqrt(2)
        assert_eq!(cut.1.terms(), &[Term { radicand: UBig::from(2u8), coef: Coef::fraction(rat(1, 1)) }]);
        // doubled area 2 * 2 * sqrt(2) = 4 sqrt(2)
        assert_eq!(total.terms(), &[Term { radicand: UBig::from(2u8), coef: Coef::fraction(rat(4, 1)) }]);
        assert_eq!(world.counts.closed_by_conjugation, 0);
    }

    #[test]
    fn the_refusals_come_before_any_cost() {
        let partition = Partition::new(true, vec![square_face()]);
        let mut world = World::new();
        assert!(matches!(world.run(&partition, &rat(-1, 3)).outcome, Ok(Answer::Refused(Refusal::AlphaNegative))));
        let not_exact = Partition::new(false, Vec::new());
        assert!(matches!(world.run(&not_exact, &rat(1, 1)).outcome, Ok(Answer::Refused(Refusal::PartitionNotExact))));
        let without_line = Partition::new(true, vec![square_face(), Face::new(vec![point(0, 0)], None)]);
        assert!(matches!(world.run(&without_line, &rat(1, 1)).outcome, Err(CoverageError::MissingLine { face: 1 })));
        assert_eq!(world.budget.articles(), [0; 6]);
        assert_eq!(world.counts, SignCounts::default());
    }

    /// y-coordinates `0`, `1` and `sqrt(2) + sqrt(3)` against a front `alpha * sqrt(6)`: the divisors need two
    /// conjugations (primes 2 and 3).
    fn two_prime_face() -> Face {
        let root = |radicand: u64, numerator: i64| {
            SqrtSum::from_terms(vec![Term { radicand: UBig::from(radicand), coef: Coef::fraction(rat(numerator, 1)) }]).unwrap()
        };
        let y2 = root(2, 1).add(&root(3, 1));
        let points = vec![point(0, 0), point(4, 1), (rational(1), y2)];
        Face::new(points, Some(Line { a: rat(0, 1), b: rat(1, 1), c: rat(0, 1), q: rat(6, 1) }))
    }

    fn run_sequence(planned: bool, alphas: &[Rat]) -> (Vec<Answer>, [u64; 6], SignCounts, cftuv_canon::MemoryState) {
        let partition = Partition::new(true, vec![two_prime_face(), square_face()]);
        let partition = if planned { partition } else { partition.without_plans() };
        let mut world = World::new();
        let mut answers = Vec::new();
        for (index, alpha) in alphas.iter().enumerate() {
            if index == 2 {
                world.session.memory.reset();
            }
            answers.push(world.run(&partition, alpha).outcome.unwrap());
        }
        (answers, world.budget.articles(), world.counts, world.session.memory.export_state())
    }

    #[test]
    fn the_planned_cut_is_the_division_answer_cost_and_memory_included() {
        let alphas = [rat(1, 1), rat(3, 4), rat(1, 1), rat(5, 4), rat(1, 2), rat(9, 8)];
        let (planned, planned_cost, planned_counts, planned_memory) = run_sequence(true, &alphas);
        let (plain, plain_cost, plain_counts, plain_memory) = run_sequence(false, &alphas);
        assert_eq!(planned, plain);
        assert_eq!(planned_cost, plain_cost);
        assert_eq!(planned_counts, plain_counts);
        assert_eq!(planned_memory, plain_memory);
        let crossings = planned.iter().filter(|answer| matches!(answer, Answer::Exact { faces, .. } if matches!(faces[0].clipped, Clipped::Polygon(_)))).count();
        assert!(crossings >= 3, "the alphas must cut the two-prime face");
    }

    #[test]
    fn a_universe_that_changes_between_calls_rebuilds_the_plans_and_falls_back_like_the_oracle() {
        // an incomplete universe (no 3) cannot prove the radicals of the divisors: the oracle's division falls back
        // to the full `divided_by`; the planned road must give the same answer, the same cost and the same memory
        let incomplete = UniverseRecord { universe: vec![UBig::from(2u8)], delta: Vec::new(), price: None, memory: Default::default() };
        let sequence = |planned: bool| {
            let partition = Partition::new(true, vec![two_prime_face()]);
            let partition = if planned { partition } else { partition.without_plans() };
            let mut world = World::new();
            let mut answers = Vec::new();
            for hit in [false, true, false, true] {
                let mut ctx = ExactCtx { memory: &mut world.session.memory, budget: &mut world.budget, counts: &mut world.counts, products: &mut world.session.products };
                let store = if hit { UniverseStore::Hit(&incomplete) } else { UniverseStore::Absent };
                answers.push(coverage_at(&mut ctx, &partition, &rat(1, 1), store, false, false).outcome.unwrap());
            }
            let fast = locked(&partition.faces()[0].plans).edges.iter().filter(|edge| matches!(edge, EdgeState::Fast(_))).count();
            (answers, world.budget.articles(), world.counts, world.session.memory.export_state(), fast)
        };
        let (planned, planned_cost, planned_counts, planned_memory, fast) = sequence(true);
        let (plain, plain_cost, plain_counts, plain_memory, _) = sequence(false);
        assert_eq!(planned, plain);
        assert_eq!((planned_cost, planned_counts), (plain_cost, plain_counts));
        assert_eq!(planned_memory, plain_memory);
        assert_eq!(planned[0], planned[2], "the complete universe gives the same cut every time");
        assert_eq!(fast, 0, "the last universe could not prove the divisors, so no plan is left standing");
    }

    struct Rng(u64);

    impl Rng {
        fn next(&mut self, bound: u64) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0 % bound
        }
    }

    const RADICANDS: [u64; 8] = [1, 2, 3, 5, 6, 10, 15, 30];

    fn random_sum(rng: &mut Rng, terms: usize) -> SqrtSum {
        let mut radicands: Vec<u64> = (0..terms).map(|_| RADICANDS[rng.next(8) as usize]).collect();
        radicands.sort_unstable();
        radicands.dedup();
        let terms = radicands
            .into_iter()
            .filter_map(|radicand| {
                let numerator = rng.next(2001) as i64 - 1000;
                (numerator != 0).then(|| Term { radicand: UBig::from(radicand), coef: Coef::fraction(rat(numerator, 1 + rng.next(40) as i64)) })
            })
            .collect();
        SqrtSum::from_terms(terms).unwrap()
    }

    /// `a * sqrt(2)` against `c * sqrt(3)` with `c` the 2^-80 approximation of `a * sqrt(2/3)`: the 64-bit enclosure cannot decide.
    fn hard_pair(a: i64, shift: i64) -> (SqrtSum, SqrtSum) {
        let scaled = (UBig::from((2 * a * a) as u64) << 160usize) / UBig::from(3u8);
        let approx = Rat::new(IBig::from(crate::num::isqrt(&scaled)) + IBig::from(shift), IBig::ONE << 80usize).unwrap();
        let base = SqrtSum::from_terms(vec![Term { radicand: UBig::from(2u8), coef: Coef::fraction(rat(a, 1)) }]).unwrap();
        let front = SqrtSum::from_terms(vec![Term { radicand: UBig::from(3u8), coef: Coef::fraction(approx) }]).unwrap();
        (base, front)
    }

    /// `(base - front).sign()` the oracle's way, and the way the face code reads it off the cached enclosure and
    /// the front's one term: same sign, same counters, same memory and budget.
    #[test]
    fn the_sign_of_a_difference_read_off_the_cached_enclosure_is_the_oracles_sign_and_cost() {
        let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
        let mut cases: Vec<(SqrtSum, SqrtSum)> = Vec::new();
        for _ in 0..600 {
            let terms = 1 + rng.next(4) as usize;
            let base = random_sum(&mut rng, terms);
            let radicand = RADICANDS[rng.next(8) as usize];
            let coefficient = rat(1 + rng.next(900) as i64, 1 + rng.next(60) as i64);
            let front = if rng.next(10) == 0 { SqrtSum::zero() } else { SqrtSum::from_terms(vec![Term { radicand: UBig::from(radicand), coef: Coef::fraction(coefficient) }]).unwrap() };
            cases.push((base, front));
        }
        for (a, shift) in [(1, 1), (7, -3), (30, 2), (123, 0), (5, 1)] {
            cases.push(hard_pair(a, shift));
        }
        let mut conjugated = 0;
        for (index, (base, front)) in cases.iter().enumerate() {
            // the face's base values are `a*x + b*y - c` of its points: with a = 1, b = c = 0 they are the x coordinates
            let line = Line { a: rat(1, 1), b: rat(0, 1), c: rat(0, 1), q: rat(1, 1) };
            let face = Face::new(vec![(base.clone(), SqrtSum::zero())], Some(line.clone()));
            let mut fresh = World::new();
            let mut ctx = ExactCtx { memory: &mut fresh.session.memory, budget: &mut fresh.budget, counts: &mut fresh.counts, products: &mut fresh.session.products };
            let want = exact::sign(&mut ctx, &base.sub(front), SIGN_FILTER_BITS).unwrap();
            let (want_counts, want_budget, want_memory) = (fresh.counts, fresh.budget.articles(), fresh.session.memory.export_state());
            let mut other = World::new();
            let mut ctx = ExactCtx { memory: &mut other.session.memory, budget: &mut other.budget, counts: &mut other.counts, products: &mut other.session.products };
            let split = Front::of(front, ctx.products);
            let got = sign_of_difference(&mut ctx, &face, &line, 0, front, split.as_ref()).unwrap();
            assert_eq!(got, want, "case {index}");
            assert_eq!(other.counts, want_counts, "case {index}: counters");
            assert_eq!(other.budget.articles(), want_budget, "case {index}: budget");
            assert_eq!(other.session.memory.export_state(), want_memory, "case {index}: memory");
            conjugated += want_counts.closed_by_conjugation;
        }
        assert!(conjugated >= 5, "the near-equal pairs must reach the conjugation: {conjugated}");
    }

    #[test]
    fn a_traced_call_records_the_signs_and_the_values_of_every_face_whose_clipping_completed() {
        let face = square_face();
        let partition = Partition::new(true, vec![face, square_face()]);
        let mut session = Session::new();
        let mut budget = WorkBudget::unlimited();
        let mut counts = SignCounts::default();
        let run = {
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: &mut budget, counts: &mut counts, products: &mut session.products };
            coverage_at(&mut ctx, &partition, &rat(1, 1), UniverseStore::Absent, true, true)
        };
        assert!(run.outcome.is_ok());
        assert_eq!(run.traces.len(), 2);
        // the front of the bottom edge at alpha = 1 is `y = 1`: the values of (0,0) (2,0) (2,2) (0,2) are `y - 1`
        assert_eq!(run.traces[0].signs, vec![-1, -1, 1, 1]);
        assert_eq!(run.traces[0].values, vec![rational(-1), rational(-1), rational(1), rational(1)]);
        // an untraced call records nothing, and a call that refuses before the faces records nothing either
        let untraced = {
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: &mut budget, counts: &mut counts, products: &mut session.products };
            coverage_at(&mut ctx, &partition, &rat(1, 1), UniverseStore::Absent, true, false)
        };
        assert!(untraced.traces.is_empty());
        let negative = {
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: &mut budget, counts: &mut counts, products: &mut session.products };
            coverage_at(&mut ctx, &partition, &rat(-1, 1), UniverseStore::Absent, true, true)
        };
        assert!(negative.traces.is_empty());
    }

    #[test]
    fn an_exhausted_budget_returns_the_miss_record_and_the_partial_cost() {
        // q = 6 has a real factorization to pay for
        let mut face = square_face();
        face.line.as_mut().unwrap().q = rat(6, 1);
        let partition = Partition::new(true, vec![face]);
        let mut session = Session::new();
        let mut budget = WorkBudget::bounded(0);
        let mut counts = SignCounts::default();
        let run = {
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: &mut budget, counts: &mut counts, products: &mut session.products };
            coverage_at(&mut ctx, &partition, &rat(1, 1), UniverseStore::Miss, true, false)
        };
        assert!(matches!(run.outcome, Err(CoverageError::Exact(ExactError::Canon(_)))), "{:?}", run.outcome);
        // 6 = 2 * 3 factors without any paid work, so the universe is built and recorded; the first `radical` then
        // pays a materialization the cap of zero refuses: the oracle has stored the record by then
        assert!(run.record.is_some(), "the miss record survives the later exhaustion");
        assert_eq!(budget.articles()[4], 1, "the failing spend stays incremented");
        // a generous budget: the record comes with the answer
        let mut budget = WorkBudget::unlimited();
        let run = {
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: &mut budget, counts: &mut counts, products: &mut session.products };
            coverage_at(&mut ctx, &partition, &rat(1, 1), UniverseStore::Miss, true, false)
        };
        assert!(run.outcome.is_ok());
        assert!(run.record.is_some());
    }
}
