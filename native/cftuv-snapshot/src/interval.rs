//! The checks of a certified angle measure in `validate_analysis_snapshot`: `phi / pi` lies strictly inside `(1, 2)`, and the reflex excess `delta` is `phi - 1`, end by end, with the same kinds.
//!
//! The one subtlety is `phi.lower - Decimal(1)`: the oracle computes it in the AMBIENT decimal context (28 digits by default), which rounds a difference that needs more digits. The port compares
//! with the exact difference and says so when the context would not have rounded it (`decimal::minus_one_exact`); a difference that needs more digits than the context holds is a decline
//! (`Reason::Mode`), because what Python compares then is a rounded number the port does not imitate.

use std::cmp::Ordering;

use crate::decimal::{compare, minus_one_exact};
use crate::model::{Dec, Endpoint, Interval};
use crate::verdict::{decline, issue, Check, Context, Reason};

fn dec_of(value: i64) -> Dec {
    Dec { coefficient: cftuv_core::num::IBig::from(value), exponent: 0 }
}

fn order(left: &Dec, right: &Dec) -> Check<Ordering> {
    match compare(left, right) {
        Some(order) => Ok(order),
        None => decline(Reason::Shape, "decimals whose exponents are too far apart to compare"),
    }
}

/// `_interval_strictly_above(interval, value)`.
fn strictly_above(interval: &Interval, value: &Dec) -> Check<bool> {
    let order = order(&interval.lower, value)?;
    Ok(order == Ordering::Greater || (order == Ordering::Equal && interval.lower_kind == Endpoint::Open))
}

/// `_interval_strictly_below(interval, value)`.
fn strictly_below(interval: &Interval, value: &Dec) -> Check<bool> {
    let order = order(&interval.upper, value)?;
    Ok(order == Ordering::Less || (order == Ordering::Equal && interval.upper_kind == Endpoint::Open))
}

/// The two interval checks of a `CertifiedReflexAngleMeasureV1`.
pub fn check_measure(phi: &Interval, delta: &Interval, context: &Context) -> Check<()> {
    if !strictly_above(phi, &dec_of(1))? || !strictly_below(phi, &dec_of(2))? {
        return issue("certified phi interval must be strictly inside (1, 2)");
    }
    let lower = minus_one_exact(&phi.lower, context.decimal_precision);
    let upper = minus_one_exact(&phi.upper, context.decimal_precision);
    let (Some(lower), Some(upper)) = (lower, upper) else {
        return decline(Reason::Mode, "phi - 1 needs more digits than the decimal context holds: Python compares a rounded number");
    };
    if order(&delta.lower, &lower)? != Ordering::Equal || order(&delta.upper, &upper)? != Ordering::Equal || delta.lower_kind != phi.lower_kind || delta.upper_kind != phi.upper_kind {
        return issue("delta interval must equal phi_over_pi minus one exactly");
    }
    Ok(())
}
