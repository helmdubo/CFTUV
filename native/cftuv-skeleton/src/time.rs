//! The time and the place of a wavefront event, exactly and without extracting a root (`wavefront/event_time.py`).
//!
//! A time is a pair `(dividend, divisor)`: a rational and a positive `SqrtSum`, never evaluated. Two times compare by the sign of
//! `d1*S2 - d2*S1` (no division, no root) and are EQUAL exactly when that difference has no terms. Every function here asks the exact layer its
//! questions in the oracle's order (which signs, which radicals, which divisions, which hydration spends), because that order is the
//! cost: the six budget articles, the memory tables and the sign counters are compared with the oracle's after the call.

use std::rc::Rc;

use cftuv_canon::Operation;
use cftuv_core::exact::{self, ExactCtx, ExactError};
use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{filtered_sign, scaled_difference_parts, SqrtSum, SIGN_FILTER_BITS};

use crate::error::{SkelError, SkelResult};
use crate::line::SupportLine;
use crate::profile::{timed, Phase};

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
}

/// `EventPointV1`: two canonical sums.
#[derive(Debug, Clone)]
pub struct EventPoint {
    pub x: SqrtSum,
    pub y: SqrtSum,
}

pub type TimeRef = Rc<EventTime>;
pub type PointRef = Rc<EventPoint>;

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
        EventTime { dividend, divisor }
    }

    /// `ZERO_TIME`: `0 / 1`.
    pub fn zero() -> EventTime {
        EventTime { dividend: Rat::zero(), divisor: SqrtSum::rational(&Rat::one()) }
    }

    /// `EventTimeV1.normalized`: the sign of the divisor is asked (and paid for), zero is `ZeroDivisorTimeError`, a negative divisor flips both.
    pub fn normalized(ctx: &mut ExactCtx<'_>, dividend: Rat, divisor: SqrtSum) -> SkelResult<EventTime> {
        let sign = exact::sign(ctx, &divisor, SIGN_FILTER_BITS)?;
        if sign == 0 {
            return Err(SkelError::ZeroDivisorTime);
        }
        if sign < 0 {
            return Ok(EventTime { dividend: dividend.neg(), divisor: divisor.neg() });
        }
        Ok(EventTime { dividend, divisor })
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
        Ok(EventTime { dividend: self.dividend.mul(&factor), divisor: self.divisor.scaled(&factor) })
    }
}

/// `compare_times(left, right, budget)`: the exact sign of `left - right`. The 64-bit enclosure filter decides what it can on the unreduced integer
/// forms (counters bumped as `_filtered_sign` bumps them); when it gives up the question goes the whole way as `scaled_difference(...).sign(budget)`.
pub fn compare_times(ctx: &mut ExactCtx<'_>, left: &EventTime, right: &EventTime) -> SkelResult<i8> {
    timed(Phase::CompareTimes, || {
        let form = scaled_difference_parts(&right.divisor, &left.dividend, &left.divisor, &right.dividend);
        if let Some(decided) = filtered_sign(&form.items, SIGN_FILTER_BITS, ctx.counts) {
            return Ok(decided);
        }
        let difference = right.divisor.scaled_difference(&left.dividend, &left.divisor, &right.dividend);
        Ok(exact::sign(ctx, &difference, SIGN_FILTER_BITS)?)
    })
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
    Ok(EventPoint { x, y })
}

/// `_event_point(first, second, time, prime_universe=..., budget)`: the crossing of two moving lines at `time`. With a universe the divisions take their
/// conjugation primes from it (`_divide_with_prime_universe`), otherwise from the factorization memory (`divided_by`); `event_point` is the second.
pub fn event_point(ctx: &mut ExactCtx<'_>, first: &SupportLine, second: &SupportLine, time: &EventTime, universe: Option<&[UBig]>) -> SkelResult<EventPoint> {
    let determinant = first.determinant(second);
    if determinant == 0 {
        return Err(SkelError::ParallelSupportLines);
    }
    spend_hydration(ctx, first)?;
    let first_radical = timed(Phase::EventPointRadical, || exact::radical(ctx, &time.dividend, &first.q))?;
    let second_radical = timed(Phase::EventPointRadical, || exact::radical(ctx, &time.dividend, &second.q))?;
    let (scale, x_numerator, y_numerator) = timed(Phase::EventPointArithmetic, || {
        let right_first = time.divisor.scaled(&rat_of(first.c)).add(&first_radical);
        let right_second = time.divisor.scaled(&rat_of(second.c)).add(&second_radical);
        let scale = time.divisor.scaled(&rat_of(determinant));
        let x_numerator = combined(&right_first, &rat_of(i128::from(second.b)), &right_second, &rat_of(i128::from(first.b)));
        let y_numerator = combined(&right_second, &rat_of(i128::from(first.a)), &right_first, &rat_of(i128::from(second.a)));
        (scale, x_numerator, y_numerator)
    });
    let (x, y) = timed(Phase::EventPointDivide, || -> SkelResult<(SqrtSum, SqrtSum)> {
        Ok(match universe {
            None => (exact::divided_by(ctx, &x_numerator, &scale)?, exact::divided_by(ctx, &y_numerator, &scale)?),
            Some(universe) => divide_pair_with_prime_universe(ctx, &x_numerator, &y_numerator, &scale, universe)?,
        })
    })?;
    Ok(EventPoint { x, y })
}

/// `(x_numerator / scale, y_numerator / scale)` as the oracle computes them, one `_divide_with_prime_universe` after the other, with the conjugation of `scale`
/// made ONCE: the primes of the rounds of that loop depend on the denominator alone, so both coordinates take them (and the product of the conjugates) from one
/// plan. What the oracle's loops ask of the memory is replayed in their order: per division, `squarefree_split(prime)` for each prime of the plan, until one
/// of them is not `(1, prime)` (then that division is `divided_by` on its original operands, as in the oracle). A plan the universe cannot make leaves both
/// divisions to the oracle's loop itself, which knows how far it got before it left.
fn divide_pair_with_prime_universe(ctx: &mut ExactCtx<'_>, x_numerator: &SqrtSum, y_numerator: &SqrtSum, scale: &SqrtSum, universe: &[UBig]) -> SkelResult<(SqrtSum, SqrtSum)> {
    let plan = if scale.is_zero() { None } else { exact::conjugation_plan(scale, universe, ctx.products) };
    let Some(plan) = plan else {
        return Ok((exact::divide_with_prime_universe(ctx, x_numerator, scale, universe)?, exact::divide_with_prime_universe(ctx, y_numerator, scale, universe)?));
    };
    let inverse = Rat::one().div(&plan.rational).map_err(|_| ExactError::Internal("a zero rational denominator"))?;
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
        quotients.push(if agrees { numerator.mul(&plan.conjugate, ctx.products).scaled(&inverse) } else { exact::divided_by(ctx, numerator, scale)? });
    }
    let y = quotients.pop().ok_or(ExactError::Internal("two quotients"))?;
    let x = quotients.pop().ok_or(ExactError::Internal("two quotients"))?;
    Ok((x, y))
}
