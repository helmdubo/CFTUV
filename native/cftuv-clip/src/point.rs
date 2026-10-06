//! Point facts the stage reads: the identity key of a point (`coalesce.point_key`), its integer record when both
//! coordinates are rational (`clip._rational_pair`), and the owned point type.

use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

/// A point of the plane that owns its coordinates (the values of the `points` dictionaries).
pub type Point = (SqrtSum, SqrtSum);

/// `point_key(point)`: `(x.terms, y.terms)`, equality by value (`int 3 == Fraction(3, 1)`, so the Python type of a
/// coefficient is not part of the identity; the first object interned stays the representative, see the stage).
pub type PointKey = (Vec<(UBig, Rat)>, Vec<(UBig, Rat)>);

fn terms_of(value: &SqrtSum) -> Vec<(UBig, Rat)> {
    value.terms().iter().map(|term| (term.radicand.clone(), term.coef.value().clone())).collect()
}

pub fn point_key(point: &Point) -> PointKey {
    (terms_of(&point.0), terms_of(&point.1))
}

/// `(x numerator, x denominator, y numerator, y denominator)` of a rational point: a coordinate is rational when it
/// has no term or one term at radicand 1.
pub type RationalPair = (IBig, UBig, IBig, UBig);

/// `clip._rational_pair(point)`.
pub fn rational_pair(point: &Point) -> Option<RationalPair> {
    let mut parts: Vec<(IBig, UBig)> = Vec::with_capacity(2);
    for coordinate in [&point.0, &point.1] {
        let terms = coordinate.terms();
        match terms {
            [] => parts.push((IBig::ZERO, UBig::ONE)),
            [only] if only.radicand.is_one() => {
                let value = only.coef.value();
                parts.push((value.numerator().clone(), value.denominator().clone()));
            }
            _ => return None,
        }
    }
    let (second, first) = (parts.pop()?, parts.pop()?);
    Some((first.0, first.1, second.0, second.1))
}
