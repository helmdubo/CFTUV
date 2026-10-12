//! The time and the place of a wavefront event, exactly and without extracting a root (`wavefront/event_time.py`).
//!
//! A time is a pair `(dividend, divisor)`: a rational and a positive `SqrtSum`, never evaluated. Two times compare by the sign of
//! `d1*S2 - d2*S1` (no division, no root) and are EQUAL exactly when that difference has no terms. Every function here asks the exact layer its
//! questions in the oracle's order (which signs, which radicals, which divisions, which hydration spends), because that order is the
//! cost: the six budget articles, the memory tables and the sign counters are compared with the oracle's after the call.

use std::cell::{OnceCell, RefCell};
use std::collections::HashMap;
use std::rc::Rc;

use cftuv_canon::fxhash::FxBuild;
use cftuv_canon::Operation;
use cftuv_core::exact::{self, ExactCtx, ExactError};
use cftuv_core::num::{IBig, UBig};
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{filter_decision, scaled_difference_parts, Closure, SqrtSum, SIGN_FILTER_BITS};

use crate::error::{SkelError, SkelResult};
use crate::line::SupportLine;
use crate::profile::{timed, Phase};
use crate::pyval::Val;

/// `EventTimeOutcome`: why a triple of lines gives no event (there is no silent return).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimeOutcome {
    Exact,
    NeverConcurrent,
    AlwaysConcurrent,
}

/// `EventTimeV1`: `t = dividend / divisor`, the divisor positive (made so by [`EventTime::normalized`]).
#[derive(Debug, Clone)]
pub struct EventTime {
    pub dividend: Rat,
    pub divisor: SqrtSum,
    /// The conjugation of the divisor (what every division by `divisor * determinant` of the places at this time starts from), made on the first division and kept.
    plan: RefCell<Option<Rc<DivisorPlan>>>,
    /// A number that identifies the value of this object for the memories keyed by pairs of times (a clone keeps it: the two are the same value).
    pub id: u64,
    /// The value of the time as a key (dividend, divisor in lowest terms, and a hash of both), made on first use and shared by every key that holds this time.
    value: OnceCell<Rc<TimeValue>>,
}

/// The value of a time as the key of a place: the dividend and the divisor in lowest terms (equal values, equal keys), with their hash made once.
#[derive(Debug)]
pub struct TimeValue {
    hash: u64,
    dividend: Rat,
    divisor: cftuv_core::sqrt_sum::IntForm,
}

impl PartialEq for TimeValue {
    fn eq(&self, other: &TimeValue) -> bool {
        self.hash == other.hash && self.dividend == other.dividend && self.divisor == other.divisor
    }
}

impl Eq for TimeValue {}

impl std::hash::Hash for TimeValue {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        state.write_u64(self.hash);
    }
}

/// The rounds of the conjugation of a divisor against one prime universe (`None`: the universe cannot make them), computed once for a time and shared by the
/// divisions of all its places: the rounds depend on the radicands of the divisor alone, and a rational factor (the determinant of two lines) changes none of them.
#[derive(Debug)]
struct DivisorPlan {
    universe: Vec<UBig>,
    plan: Option<exact::ConjugationPlan>,
}

/// `EventPointV1`: two canonical sums.
#[derive(Debug, Clone)]
pub struct EventPoint {
    pub x: SqrtSum,
    pub y: SqrtSum,
    /// `(x.terms, y.terms)` as the oracle's keys hold them, made on first use (a point is immutable, and the same point object is keyed again and again).
    key: OnceCell<Val>,
}

impl EventPoint {
    pub fn new(x: SqrtSum, y: SqrtSum) -> EventPoint {
        EventPoint { x, y, key: OnceCell::new() }
    }

    /// `exact_point_key(point)`: the key of the point, made once.
    pub fn key(&self) -> &Val {
        self.key.get_or_init(|| Val::tuple_of([Val::terms_of(&self.x), Val::terms_of(&self.y)]))
    }
}

pub type TimeRef = Rc<EventTime>;
pub type PointRef = Rc<EventPoint>;

/// The ids of the times, for the whole process (a time built on one thread and compared on another must not meet an id of that thread's own).
static TIME_IDS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);

fn next_time_id() -> u64 {
    TIME_IDS.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
}

fn rat_of(value: i128) -> Rat {
    Rat::from_int(IBig::from(value))
}

/// `first*a - second*b` as [`SqrtSum::scaled_difference`] answers it, but NOT taken to lowest terms: the integer form of the combination, its radicands put in
/// order and its zeros dropped, and nothing more. The reduction is a gcd chain over every numerator, paid so that the value has its canonical terms at hand;
/// a value that is only asked its sign (the projections of a span, the numerators a division takes the conjugates of) never needs them, and one that is
/// asked them later (`terms`, `canonical_form`) takes them then, from the same integers.
pub fn combined(first: &SqrtSum, a: &Rat, second: &SqrtSum, b: &Rat) -> SqrtSum {
    let cftuv_core::sqrt_sum::IntForm { common, mut items } = scaled_difference_parts(first, a, second, b);
    if !items.windows(2).all(|pair| pair[0].0 < pair[1].0) {
        items.sort_by(|left, right| left.0.cmp(&right.0));
    }
    SqrtSum::from_sorted_form(common, items, false)
}

/// `coefficient * cofactor` as a big integer without an overflow of the machine product.
fn product(coefficient: i128, cofactor: i128) -> IBig {
    match coefficient.checked_mul(cofactor) {
        Some(found) => IBig::from(found),
        None => IBig::from(coefficient) * IBig::from(cofactor),
    }
}

impl EventTime {
    pub fn new(dividend: Rat, divisor: SqrtSum) -> EventTime {
        EventTime { dividend, divisor, plan: RefCell::new(None), id: next_time_id(), value: OnceCell::new() }
    }

    /// The value of this time as a key of a place (see [`TimeValue`]).
    pub fn value_key(&self) -> Rc<TimeValue> {
        Rc::clone(self.value.get_or_init(|| {
            use std::hash::{Hash, Hasher};
            crate::profile::count(crate::profile::Stat::TimeKeys, 1);
            let divisor = self.divisor.canonical_form().clone();
            let mut hasher = cftuv_canon::fxhash::FxHasher::default();
            self.dividend.hash(&mut hasher);
            divisor.hash(&mut hasher);
            Rc::new(TimeValue { hash: hasher.finish(), dividend: self.dividend.clone(), divisor })
        }))
    }

    /// `ZERO_TIME`: `0 / 1`.
    pub fn zero() -> EventTime {
        EventTime::new(Rat::zero(), SqrtSum::rational(&Rat::one()))
    }

    /// The conjugation plan of the divisor against `universe`, made on the first ask for a universe.
    fn divisor_plan(&self, universe: &[UBig], products: &mut ProductMemo) -> Rc<DivisorPlan> {
        if let Some(found) = self.plan.borrow().as_ref() {
            if found.universe == universe {
                return Rc::clone(found);
            }
        }
        let made = Rc::new(DivisorPlan { universe: universe.to_vec(), plan: exact::conjugation_plan(&self.divisor, universe, products) });
        *self.plan.borrow_mut() = Some(Rc::clone(&made));
        made
    }

    /// `EventTimeV1.normalized`: the sign of the divisor is asked (and paid for), zero is `ZeroDivisorTimeError`, a negative divisor flips both.
    pub fn normalized(ctx: &mut ExactCtx<'_>, dividend: Rat, divisor: SqrtSum) -> SkelResult<EventTime> {
        let sign = exact::sign(ctx, &divisor, SIGN_FILTER_BITS)?;
        if sign == 0 {
            return Err(SkelError::ZeroDivisorTime);
        }
        if sign < 0 {
            return Ok(EventTime::new(dividend.neg(), divisor.neg()));
        }
        Ok(EventTime::new(dividend, divisor))
    }

    /// `EventTimeV1.sign`: the sign of the dividend.
    pub fn sign(&self) -> i8 {
        self.dividend.signum()
    }

    /// `EventTimeV1.canonical`: the one representative of the value, the divisor's first coefficient made `+-1`.
    pub fn canonical(&self) -> SkelResult<EventTime> {
        if self.dividend.is_zero() {
            return Ok(EventTime::zero());
        }
        let first = self.divisor.terms().first().ok_or(ExactError::Internal("the canonical time of a zero divisor (IndexError in the oracle)"))?;
        let scale = first.coef.value();
        let scale = if scale.signum() < 0 { scale.neg() } else { scale.clone() };
        let factor = Rat::one().div(&scale).map_err(|_| ExactError::Internal("a zero scale"))?;
        Ok(EventTime::new(self.dividend.mul(&factor), self.divisor.scaled(&factor)))
    }
}

/// `compare_times(left, right, budget)`: the exact sign of `left - right`. The 64-bit enclosure filter decides what it can on the unreduced integer
/// forms (counters bumped as `_filtered_sign` bumps them); when it gives up the question goes the whole way as `scaled_difference(...).sign(budget)`.
///
/// A pair of times the filter decided once is decided the same way every time (the question is a pure function of the two values, and a time keeps its `id` for as long as it
/// is the same value), so the answer is remembered with the counter it moved: a repeat bumps the same counters and builds nothing. 55 % of the comparisons of a skeleton are repeats.
pub fn compare_times(ctx: &mut ExactCtx<'_>, left: &EventTime, right: &EventTime) -> SkelResult<i8> {
    timed(Phase::CompareTimes, || {
        crate::profile::count_seen(crate::profile::Stat::CompareTimes, crate::profile::Stat::CompareTimesRepeated, || left.id.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ right.id);
        let key = (left.id, right.id);
        if let Some((sign, closure)) = COMPARED.with(|memo| memo.borrow().get(&key).copied()) {
            ctx.counts.close(closure);
            return Ok(sign);
        }
        let form = scaled_difference_parts(&right.divisor, &left.dividend, &left.divisor, &right.dividend);
        if let Some((sign, closure)) = filter_decision(&form.items, SIGN_FILTER_BITS) {
            crate::profile::count(crate::profile::Stat::CompareTimesDecidedByFilter, 1);
            ctx.counts.close(closure);
            COMPARED.with(|memo| memo.borrow_mut().insert(key, (sign, closure)));
            return Ok(sign);
        }
        let difference = right.divisor.scaled_difference(&left.dividend, &left.divisor, &right.dividend);
        Ok(exact::sign(ctx, &difference, SIGN_FILTER_BITS)?)
    })
}

/// `(left.id, right.id)` of the comparisons the filter decided: the sign and how it closed.
type Compared = HashMap<(u64, u64), (i8, Closure), FxBuild>;

thread_local! {
    /// The comparisons the filter decided. Emptied when a skeleton starts and when it ends ([`forget_compared`]).
    static COMPARED: RefCell<Compared> = RefCell::new(HashMap::default());
}

/// Empties the memory of compared pairs (the ids are never reused, so a stale entry is harmless, only memory).
pub fn forget_compared() {
    COMPARED.with(|memo| *memo.borrow_mut() = HashMap::default());
}

/// `times_are_equal`: the difference has no terms (a pure emptiness test: no counter moves).
pub fn times_are_equal(left: &EventTime, right: &EventTime) -> bool {
    scaled_difference_parts(&right.divisor, &left.dividend, &left.divisor, &right.dividend).items.is_empty()
}

/// `concurrency_time(first, second, third, budget)`: when three moving lines meet in one point. The answer may be negative: the past is an answer too,
/// dropped by the caller.
pub fn concurrency_time(ctx: &mut ExactCtx<'_>, first: &SupportLine, second: &SupportLine, third: &SupportLine) -> SkelResult<(Option<EventTime>, TimeOutcome)> {
    timed(Phase::ConcurrencyTime, || concurrency_time_body(ctx, first, second, third))
}

fn concurrency_time_body(ctx: &mut ExactCtx<'_>, first: &SupportLine, second: &SupportLine, third: &SupportLine) -> SkelResult<(Option<EventTime>, TimeOutcome)> {
    let cofactor_first = second.determinant(third);
    let cofactor_second = third.determinant(first);
    let cofactor_third = first.determinant(second);
    let offset = product(first.c, cofactor_first) + product(second.c, cofactor_second) + product(third.c, cofactor_third);
    let speed = exact::radical_sum(ctx, &[(rat_of(cofactor_first), first.q.clone()), (rat_of(cofactor_second), second.q.clone()), (rat_of(cofactor_third), third.q.clone())])?;
    if speed.is_zero() {
        return Ok((None, if offset.is_zero() { TimeOutcome::AlwaysConcurrent } else { TimeOutcome::NeverConcurrent }));
    }
    let time = EventTime::normalized(ctx, Rat::from_int(-offset), speed)?;
    Ok((Some(time), TimeOutcome::Exact))
}

/// `sliding_time(line, along, other, budget)`: when the point sliding on `line` with the projection `along` reaches the line `other`. The numerator is
/// irrational (it holds `along`), so the time is kept as `1 / (S / N)`.
pub fn sliding_time(ctx: &mut ExactCtx<'_>, line: &SupportLine, along: &SqrtSum, other: &SupportLine) -> SkelResult<(Option<EventTime>, TimeOutcome)> {
    timed(Phase::SlidingTime, || sliding_time_body(ctx, line, along, other))
}

fn sliding_time_body(ctx: &mut ExactCtx<'_>, line: &SupportLine, along: &SqrtSum, other: &SupportLine) -> SkelResult<(Option<EventTime>, TimeOutcome)> {
    let weight = i128::from(line.a) * i128::from(other.a) + i128::from(line.b) * i128::from(other.b);
    let cross = line.determinant(other);
    let norm = line.normal_squared();
    let numerator = SqrtSum::rational(&Rat::from_int(product(other.c, norm)))
        .add(&along.scaled(&rat_of(cross)))
        .sub(&SqrtSum::rational(&Rat::from_int(product(line.c, weight))));
    let speed = exact::radical_sum(ctx, &[(rat_of(weight), line.q.clone()), (rat_of(-norm), other.q.clone())])?;
    if speed.is_zero() {
        return Ok((None, if numerator.is_zero() { TimeOutcome::AlwaysConcurrent } else { TimeOutcome::NeverConcurrent }));
    }
    if numerator.is_zero() {
        return Ok((Some(EventTime::zero()), TimeOutcome::Exact));
    }
    let quotient = exact::divided_by(ctx, &speed, &numerator)?;
    Ok((Some(EventTime::normalized(ctx, Rat::one(), quotient)?), TimeOutcome::Exact))
}

/// One hydration of an exact position is a declared unit of work (`exact_position_hydrations`); `line.q` truncated is the radicand a refusal names.
fn spend_hydration(ctx: &mut ExactCtx<'_>, line: &SupportLine) -> SkelResult<()> {
    Ok(ctx.budget.spend_exact_position_hydrations(1, Operation::ExactPosition, &line.q_int)?)
}

/// `sliding_point(line, along, time, budget)`: the point on a moving line with the projection along it pinned (`a*x + b*y = c + t*sqrt(q)` and
/// `b*x - a*y = along`; the determinant of the system is `a^2 + b^2`).
pub fn sliding_point(ctx: &mut ExactCtx<'_>, line: &SupportLine, along: &SqrtSum, time: &EventTime) -> SkelResult<EventPoint> {
    timed(Phase::SlidingPoint, || sliding_point_body(ctx, line, along, time))
}

fn sliding_point_body(ctx: &mut ExactCtx<'_>, line: &SupportLine, along: &SqrtSum, time: &EventTime) -> SkelResult<EventPoint> {
    spend_hydration(ctx, line)?;
    let radical = exact::radical(ctx, &time.dividend, &line.q)?;
    let moving = SqrtSum::rational(&rat_of(line.c)).add(&exact::divided_by(ctx, &radical, &time.divisor)?);
    let scale = Rat::new(IBig::ONE, IBig::from(line.normal_squared())).map_err(|_| ExactError::Internal("a degenerate line has a zero normal"))?;
    let (a, b) = (rat_of(i128::from(line.a)), rat_of(i128::from(line.b)));
    let x = moving.scaled(&a).add(&along.scaled(&b)).scaled(&scale);
    let y = moving.scaled(&b).sub(&along.scaled(&a)).scaled(&scale);
    Ok(EventPoint::new(x, y))
}

/// `_event_point(first, second, time, prime_universe=..., budget)`: the crossing of two moving lines at `time`. With a universe the divisions take their
/// conjugation primes from it (`_divide_with_prime_universe`), otherwise from the factorization memory (`divided_by`); `event_point` is the second.
pub fn event_point(ctx: &mut ExactCtx<'_>, first: &SupportLine, second: &SupportLine, time: &EventTime, universe: Option<&[UBig]>) -> SkelResult<EventPoint> {
    let determinant = first.determinant(second);
    if determinant == 0 {
        return Err(SkelError::ParallelSupportLines);
    }
    crate::profile::count(crate::profile::Stat::EventPoints, 1);
    crate::profile::count_seen(crate::profile::Stat::PlanDivisors, crate::profile::Stat::PlanDivisorsRepeated, || time as *const EventTime as usize as u64);
    spend_hydration(ctx, first)?;
    let first_radical = timed(Phase::EventPointRadical, || exact::radical(ctx, &time.dividend, &first.q))?;
    let second_radical = timed(Phase::EventPointRadical, || exact::radical(ctx, &time.dividend, &second.q))?;
    let (x_numerator, y_numerator) = timed(Phase::EventPointArithmetic, || {
        let right_first = time.divisor.scaled(&rat_of(first.c)).add(&first_radical);
        let right_second = time.divisor.scaled(&rat_of(second.c)).add(&second_radical);
        let x_numerator = combined(&right_first, &rat_of(i128::from(second.b)), &right_second, &rat_of(i128::from(first.b)));
        let y_numerator = combined(&right_second, &rat_of(i128::from(first.a)), &right_first, &rat_of(i128::from(second.a)));
        (x_numerator, y_numerator)
    });
    let (x, y) = timed(Phase::EventPointDivide, || -> SkelResult<(SqrtSum, SqrtSum)> {
        Ok(match universe {
            None => {
                let scale = time.divisor.scaled(&rat_of(determinant));
                (exact::divided_by(ctx, &x_numerator, &scale)?, exact::divided_by(ctx, &y_numerator, &scale)?)
            }
            Some(universe) => divide_pair_by_time(ctx, time, determinant, &x_numerator, &y_numerator, universe)?,
        })
    })?;
    Ok(EventPoint::new(x, y))
}

/// `(x_numerator / scale, y_numerator / scale)` for `scale = time.divisor * determinant`, as the oracle computes them (one `_divide_with_prime_universe` after the other), with the
/// conjugation of the denominator made ONCE per time: the primes of the rounds of that loop depend on the radicands of the denominator alone, and a rational factor (the
/// determinant of two lines) changes none of them, so both coordinates of every place at this time take them (and the product of the conjugates) from the plan the time keeps.
/// What the oracle's loops ask of the memory is replayed in their order: per division, `squarefree_split(prime)` for each prime of the plan, until one of them is not
/// `(1, prime)` (then that division is `divided_by` on its original operands, as in the oracle). A plan the universe cannot make leaves both divisions to the oracle's loop
/// itself, which knows how far it got before it left. `scale` is built only for those two fallbacks; the conjugate is the divisor's and the rational it ends with is the
/// divisor's times the determinant. The quotient is the same value, whichever way its integers were reached.
fn divide_pair_by_time(ctx: &mut ExactCtx<'_>, time: &EventTime, determinant: i128, x_numerator: &SqrtSum, y_numerator: &SqrtSum, universe: &[UBig]) -> SkelResult<(SqrtSum, SqrtSum)> {
    let scale = || time.divisor.scaled(&rat_of(determinant));
    let base = time.divisor_plan(universe, ctx.products);
    let Some(plan) = base.plan.as_ref() else {
        let scale = scale();
        return Ok((exact::divide_with_prime_universe(ctx, x_numerator, &scale, universe)?, exact::divide_with_prime_universe(ctx, y_numerator, &scale, universe)?));
    };
    let inverse = Rat::one().div(&plan.rational.mul(&rat_of(determinant))).map_err(|_| ExactError::Internal("a zero rational denominator"))?;
    let mut quotients = Vec::with_capacity(2);
    for numerator in [x_numerator, y_numerator] {
        let mut agrees = true;
        for prime in &plan.primes {
            let (outside, inside) = ctx.memory.squarefree_split_unsigned(prime, ctx.budget).map_err(ExactError::from)?;
            if outside != UBig::ONE || inside != *prime {
                agrees = false;
                break;
            }
        }
        quotients.push(if agrees { numerator.mul(&plan.conjugate, ctx.products).scaled(&inverse) } else { exact::divided_by(ctx, numerator, &scale())? });
    }
    let y = quotients.pop().ok_or(ExactError::Internal("two quotients"))?;
    let x = quotients.pop().ok_or(ExactError::Internal("two quotients"))?;
    Ok((x, y))
}
