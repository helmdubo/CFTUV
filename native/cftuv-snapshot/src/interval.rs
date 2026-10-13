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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decimal::parse;
    use crate::model::Endpoint::{Closed, Open};
    use crate::verdict::Scope;

    fn dec(text: &str) -> Dec {
        parse(text).unwrap()
    }

    fn interval(lower: &str, upper: &str, lower_kind: Endpoint, upper_kind: Endpoint) -> Interval {
        Interval { lower: dec(lower), upper: dec(upper), lower_kind, upper_kind }
    }

    fn context(precision: u32) -> Context {
        Context { scope: Scope::Analysis, native_exact: true, decimal_precision: precision, analysis_schema: Vec::new() }
    }

    #[test]
    fn the_exact_270_degree_corner_is_inside_with_closed_ends() {
        let phi = interval("1.5", "1.5", Closed, Closed);
        let delta = interval("0.5", "0.5", Closed, Closed);
        assert!(check_measure(&phi, &delta, &context(28)).is_ok());
        // trailing zeros are the same number
        let wide = interval("1.50", "1.500", Closed, Closed);
        assert!(check_measure(&wide, &delta, &context(28)).is_ok());
    }

    #[test]
    fn the_bounds_of_the_strict_range_are_open_only() {
        for (lower, lower_kind, upper, upper_kind, inside) in [
            ("1", Closed, "1.5", Closed, false),
            ("1", Open, "1.5", Closed, true),
            ("1.5", Closed, "2", Closed, false),
            ("1.5", Closed, "2", Open, true),
            ("0.9", Closed, "1.5", Closed, false),
            ("1.5", Closed, "2.1", Closed, false),
        ] {
            let phi = interval(lower, upper, lower_kind, upper_kind);
            let lower_delta = decimal_minus_one(lower);
            let upper_delta = decimal_minus_one(upper);
            let delta = interval(&lower_delta, &upper_delta, lower_kind, upper_kind);
            let verdict = check_measure(&phi, &delta, &context(28));
            assert_eq!(verdict.is_ok(), inside, "{lower}..{upper}: {verdict:?}");
            if !inside {
                assert_eq!(verdict.unwrap_err().reason, Reason::Issue);
            }
        }
    }

    fn decimal_minus_one(text: &str) -> String {
        let value = dec(text);
        let reduced = minus_one_exact(&value, 28).unwrap();
        format!("{}E{}", reduced.coefficient, reduced.exponent)
    }

    #[test]
    fn the_excess_must_be_phi_minus_one_end_by_end_with_the_same_kinds() {
        let phi = interval("1.25", "1.75", Closed, Open);
        assert!(check_measure(&phi, &interval("0.25", "0.75", Closed, Open), &context(28)).is_ok());
        for delta in [
            interval("0.26", "0.75", Closed, Open),
            interval("0.25", "0.76", Closed, Open),
            interval("0.25", "0.75", Open, Open),
            interval("0.25", "0.75", Closed, Closed),
        ] {
            assert_eq!(check_measure(&phi, &delta, &context(28)).unwrap_err().reason, Reason::Issue);
        }
    }

    #[test]
    fn a_difference_the_context_would_round_is_a_named_mode_decline_never_a_verdict() {
        // 1.5000000000000000000000000001 has 29 digits: `phi - 1` needs 28 digits for 0.5000000000000000000000000001, which fits; 29 digits of phi after the point do not
        let phi = interval("1.50000000000000000000000000001", "1.7", Closed, Closed);
        let delta = interval("0.50000000000000000000000000001", "0.7", Closed, Closed);
        let verdict = check_measure(&phi, &delta, &context(28));
        assert_eq!(verdict.unwrap_err().reason, Reason::Mode);
        // the same numbers in a context wide enough to hold them are compared exactly
        assert!(check_measure(&phi, &delta, &context(60)).is_ok());
        // and a context narrower than the usual digits of the bounds declines too
        let plain = interval("1.2345678", "1.3", Closed, Closed);
        let plain_delta = interval("0.2345678", "0.3", Closed, Closed);
        assert_eq!(check_measure(&plain, &plain_delta, &context(3)).unwrap_err().reason, Reason::Mode);
        assert!(check_measure(&plain, &plain_delta, &context(28)).is_ok());
    }
}
