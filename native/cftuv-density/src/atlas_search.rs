//! Поиск chart-invariant Farey height по УЖЕ подготовленным cell segments.
//! Oracle: reference/adaptive_density_atlas.py::GlobalHeightSegment,
//! cell_segments, numerator_bounds, segment_candidates, search_global_height.
//! Подготовка окон/reachable pieces/validation/proof inner здесь не выполняется.

use std::cmp::Ordering;
use std::collections::HashSet;

use cftuv_radical::arith::{gcd_signed, magnitude, signed};

use crate::farey::{
    FareyFailure, FareyPredicates, PrimitiveVector, SearchResult, WorkBudget, best_fan,
    strict_numerator_bounds,
};
use crate::order::CanonicalOrdering;
use crate::{IBig, Rational, UBig};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GlobalHeightSegment {
    pub use_x_denominator: bool,
    pub denominator_sign: i8,
    pub outer_lower: Rational,
    pub inner_lower: Rational,
    pub inner_upper: Rational,
    pub outer_upper: Rational,
}

fn compare(left: &Rational, right: &Rational) -> Ordering {
    let (ln, ld) = left.parts();
    let (rn, rd) = right.parts();
    (ln * rd).cmp(&(rn * ld))
}

fn inverse(value: &Rational) -> Rational {
    let (n, d) = value.parts();
    let numerator = if n < &IBig::ZERO {
        -signed(d)
    } else {
        signed(d)
    };
    Rational::new(numerator, magnitude(n)).expect("outer cell reciprocal never crosses zero")
}

/// Исходный low→high порядок кусков; внешние части меняют карту slope'=1/slope.
pub fn cell_segments(
    use_x: bool,
    denominator_sign: i8,
    bracket: &[Rational; 4],
) -> Vec<GlobalHeightSegment> {
    let [outer_lower, inner_lower, inner_upper, outer_upper] = bracket;
    if compare(outer_lower, outer_upper).is_gt() {
        return Vec::new();
    }
    let negative = Rational::integer(IBig::from(-1));
    let positive = Rational::integer(IBig::ONE);
    let mut cuts = vec![outer_lower.clone()];
    for value in [&negative, &positive] {
        if compare(outer_lower, value).is_lt() && compare(value, outer_upper).is_lt() {
            cuts.push(value.clone());
        }
    }
    cuts.push(outer_upper.clone());
    let mut result = Vec::new();
    for pair in cuts.windows(2) {
        let (low, high) = (&pair[0], &pair[1]);
        let lower = if compare(inner_lower, low).is_gt() {
            inner_lower
        } else {
            low
        };
        let upper = if compare(inner_upper, high).is_lt() {
            inner_upper
        } else {
            high
        };
        let mut inner = if compare(lower, upper).is_lt() {
            Some((lower.clone(), upper.clone()))
        } else {
            None
        };
        let outside_positive = !compare(low, &positive).is_lt();
        let outside_negative = !compare(high, &negative).is_gt();
        let (chart, sign, bounds) = if outside_positive || outside_negative {
            let sign = if outside_positive {
                denominator_sign
            } else {
                -denominator_sign
            };
            inner = inner.map(|(low, high)| (inverse(&high), inverse(&low)));
            (!use_x, sign, (inverse(high), inverse(low)))
        } else {
            (use_x, denominator_sign, (low.clone(), high.clone()))
        };
        let (inner_lower, inner_upper) =
            inner.unwrap_or_else(|| (Rational::integer(IBig::ZERO), Rational::integer(IBig::ZERO)));
        result.push(GlobalHeightSegment {
            use_x_denominator: chart,
            denominator_sign: sign,
            outer_lower: bounds.0,
            inner_lower,
            inner_upper,
            outer_upper: bounds.1,
        });
    }
    result
}

/// Closed ceil(lower*h)..floor(upper*h), затем clamp |numerator|<=height.
pub fn numerator_bounds(lower: &Rational, upper: &Rational, height: &UBig) -> (IBig, IBig) {
    let (mut first, mut last) = strict_numerator_bounds(lower, upper, height);
    let (ln, ld) = lower.parts();
    let (un, ud) = upper.parts();
    if ((ln * height) % ld).is_zero() {
        first -= IBig::ONE;
    }
    if ((un * height) % ud).is_zero() {
        last += IBig::ONE;
    }
    let maximum = signed(height);
    (first.max(-&maximum), last.min(maximum))
}

fn slope_is_inside(numerator: &IBig, height: &UBig, lower: &Rational, upper: &Rational) -> bool {
    let (ln, ld) = lower.parts();
    let (un, ud) = upper.parts();
    numerator * ld > ln * height && numerator * ud < un * height
}

fn vector(numerator: &IBig, height: &UBig, segment: &GlobalHeightSegment) -> PrimitiveVector {
    let sign = IBig::from(segment.denominator_sign);
    let numerator = numerator * &sign;
    let denominator = signed(height) * sign;
    if segment.use_x_denominator {
        PrimitiveVector(denominator, numerator)
    } else {
        PrimitiveVector(numerator, denominator)
    }
}

/// Один probe на каждый integer input numerator, даже gcd skip; empty span =1.
pub fn segment_candidates<P: FareyPredicates>(
    segment: &GlobalHeightSegment,
    ordinal: usize,
    height: &UBig,
    budget: &mut WorkBudget,
    predicates: &mut P,
) -> Result<Vec<PrimitiveVector>, FareyFailure<P::Error>> {
    let (first, last) = numerator_bounds(&segment.outer_lower, &segment.outer_upper, height);
    let probes = if first > last {
        UBig::ONE
    } else {
        (&last - &first + IBig::ONE)
            .try_into()
            .expect("positive span")
    };
    budget
        .spend_shell_probes(probes)
        .map_err(FareyFailure::Work)?;
    let mut result = Vec::new();
    let mut numerator = first;
    while numerator <= last {
        if gcd_signed(&numerator, height).is_one() {
            let candidate = vector(&numerator, height, segment);
            if slope_is_inside(
                &numerator,
                height,
                &segment.inner_lower,
                &segment.inner_upper,
            ) || predicates
                .local_candidate(ordinal, &candidate)
                .map_err(FareyFailure::Predicate)?
            {
                result.push(candidate);
            }
        }
        numerator += IBig::ONE;
    }
    Ok(result)
}

/// Высота→ordinal→segment→numerator. Shared endpoints оплачены до set dedup.
pub fn search_global_height<P: FareyPredicates>(
    segments: &[Vec<GlobalHeightSegment>],
    stop_height: &UBig,
    budget: &mut WorkBudget,
    predicates: &mut P,
    ordering: &mut CanonicalOrdering,
) -> Result<SearchResult, FareyFailure<P::Error>> {
    let mut candidates = vec![Vec::new(); segments.len()];
    // Membership не задаёт порядка; Vec сохраняет first-insertion log oracle.
    let mut membership = vec![HashSet::new(); segments.len()];
    let mut previous_counts = vec![0; segments.len()];
    if segments.iter().any(Vec::is_empty) {
        return Ok(SearchResult {
            height: None,
            fan: None,
            counts: previous_counts.clone(),
            previous_counts,
        });
    }
    let mut height = UBig::ONE;
    while &height <= stop_height {
        for (index, plan) in segments.iter().enumerate() {
            for segment in plan {
                for candidate in
                    segment_candidates(segment, index + 1, &height, budget, predicates)?
                {
                    if membership[index].insert(candidate.clone()) {
                        candidates[index].push(candidate);
                    }
                }
            }
        }
        if candidates.iter().all(|items| !items.is_empty()) {
            if let Some(fan) = best_fan(&candidates, budget, predicates, ordering)? {
                return Ok(SearchResult {
                    height: Some(height),
                    fan: Some(fan),
                    previous_counts,
                    counts: candidates.iter().map(Vec::len).collect(),
                });
            }
        }
        previous_counts = candidates.iter().map(Vec::len).collect();
        height += UBig::ONE;
    }
    Ok(SearchResult {
        height: None,
        fan: None,
        previous_counts,
        counts: candidates.iter().map(Vec::len).collect(),
    })
}
