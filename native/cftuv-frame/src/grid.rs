//! The grid of the domain: the step window, the intended right corners, the scale search and the snap of the source (`source_grid.resolve_source_grid` for a law that snaps the source, and
//! `robust.snapping.grid_window_for_patch` / `admissible_scales` under it).
//!
//! Everything is exact and runs on integers. The positions are binary64 values, that is dyadic rationals; a whole map of them is lifted to integers over the common `2^-e_min`
//! (`dyadic.rs`), the snapped positions are integers over the scale `2^s` (`k / 2^s`), and every predicate is a zero test or a comparison of polynomials in those integers.
//!
//! One piece of the oracle is NOT mirrored by an algebra but by a proof (below): `restored_right_corners` counts the intended corners whose angle `atan2(|sin|, cos) / pi` sympy evaluates to a
//! rational number. For an INTENDED corner (`cos^2 <= AUTHOR_ANGULAR_ERROR^2 * sin^2`, so `|tan| >= 1 / 7e-6 > 142857`) that happens exactly when `cos = 0`: sympy knows `atan` of a number only
//! at 0, +-1 and the table of the algebraic tangents of the multiples of `pi/30` (all smaller than 4), and `atan2(y, 0)` of a positive `y` is `pi/2`. The two sides are checked by the
//! differential corpora (every trial of every scale search of every domain) and the 270-degree and near-right mutants.

use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;

use crate::dyadic::{minimum_exponent, Dyadic};
use crate::model::{decline, FaceIn, GridFacts, Outcome, Reason, Trial};

/// `_authoring_intent.AUTHOR_ANGULAR_ERROR = Fraction(7, 10**6)`.
pub const AUTHOR_ANGULAR_ERROR: (u64, u64) = (7, 1_000_000);
/// `_authoring_intent.DECAL_DETAIL = Fraction(1, 100)`.
pub const DECAL_DETAIL: (u64, u64) = (1, 100);
/// `GridSpecV1.magnitude_bound` (`robust.grid.DOUBLE_EXACT_MAGNITUDE_BOUND`), recorded in the certificate.
pub const MAGNITUDE_BOUND: u32 = 25;
/// The finest step the port runs: a scale above this exponent is a named decline (the fast paths and the writer of the answer are far from it; a patch this small is not a domain).
const MAXIMUM_EXPONENT: u32 = 100;
/// The smallest scale exponent of a window: the coarsest admissible step is the largest `1/2^e` that does not exceed the upper bound `1/100`, `2^7 = 128`.
const COARSEST_EXPONENT: u32 = 7;

/// A corner as the oracle enumerates it: ranks of `(previous, vertex, following)`.
pub type Corner = [u32; 3];

/// The corners of the faces and how the oracle classifies them on the unsnapped positions.
#[derive(Clone, Debug, Default)]
pub struct Corners {
    pub intended: Vec<Corner>,
    pub unclassifiable: Vec<Corner>,
}

/// Everything the scale search produced.
pub struct Resolved {
    pub facts: GridFacts,
    /// The snapped positions, integers over `2^facts.exponent`, by rank.
    pub snapped: Vec<[IBig; 3]>,
    pub corners: Corners,
    /// The dyadic positions and the common exponent they are lifted over, for the embedding adapter.
    pub dyadic: Vec<[Dyadic; 3]>,
    pub e_min: i32,
    pub lifted: Vec<[IBig; 3]>,
}

fn rational_of(numerator: u64, denominator: u64) -> Rat {
    Rat::reduced(IBig::from(numerator), UBig::from(denominator))
}

/// `value * 2^exponent` as a canonical rational.
pub fn scaled_rational(value: IBig, exponent: i32) -> Rat {
    if exponent >= 0 {
        Rat::from_int(value << exponent as usize)
    } else {
        Rat::reduced(value, UBig::ONE << (-exponent) as usize)
    }
}

fn max_min_extent(lifted: &[[IBig; 3]]) -> IBig {
    let mut extent = IBig::ZERO;
    for axis in 0..3 {
        let mut low = &lifted[0][axis];
        let mut high = &lifted[0][axis];
        for point in lifted {
            if point[axis] < *low {
                low = &point[axis];
            }
            if point[axis] > *high {
                high = &point[axis];
            }
        }
        let span = high - low;
        if span > extent {
            extent = span;
        }
    }
    extent
}

/// `intended_right_corner_facts`: the corners of every face cycle in order, classified on the unsnapped positions by `cos^2 <= AUTHOR_ANGULAR_ERROR^2 * |left x right|^2`.
pub fn classify_corners(lifted: &[[IBig; 3]], faces: &[FaceIn]) -> Corners {
    let mut corners = Corners::default();
    let (num, den) = AUTHOR_ANGULAR_ERROR;
    // cos^2 * den^2 <= num^2 * sin^2 (the squared bound, as integers)
    let numerator_squared = IBig::from(num) * IBig::from(num);
    let denominator_squared = IBig::from(den) * IBig::from(den);
    for face in faces {
        let cycle = &face.vertices;
        let count = cycle.len();
        for index in 0..count {
            let corner = [cycle[(index + count - 1) % count], cycle[index], cycle[(index + 1) % count]];
            let (previous, vertex, following) = (&lifted[corner[0] as usize], &lifted[corner[1] as usize], &lifted[corner[2] as usize]);
            let left = [&previous[0] - &vertex[0], &previous[1] - &vertex[1], &previous[2] - &vertex[2]];
            let right = [&following[0] - &vertex[0], &following[1] - &vertex[1], &following[2] - &vertex[2]];
            let normal = cross(&left, &right);
            let squared_sine = &normal[0] * &normal[0] + &normal[1] * &normal[1] + &normal[2] * &normal[2];
            if squared_sine == IBig::ZERO {
                // a degenerate corner: the sides are collinear, it is never a right angle
                corners.unclassifiable.push(corner);
                continue;
            }
            let cosine = &left[0] * &right[0] + &left[1] * &right[1] + &left[2] * &right[2];
            if cosine == IBig::ZERO {
                continue;
            }
            if &cosine * &cosine * &denominator_squared <= &numerator_squared * &squared_sine {
                corners.intended.push(corner);
            }
        }
    }
    corners
}

fn cross(left: &[IBig; 3], right: &[IBig; 3]) -> [IBig; 3] {
    [
        &left[1] * &right[2] - &left[2] * &right[1],
        &left[2] * &right[0] - &left[0] * &right[2],
        &left[0] * &right[1] - &left[1] * &right[0],
    ]
}

/// The integer a count of restored corners runs on.
trait Int: Clone {
    fn minus(&self, other: &Self) -> Self;
    fn times(&self, other: &Self) -> Self;
    fn plus(&self, other: &Self) -> Self;
    fn nonzero(&self) -> bool;
}

impl Int for i128 {
    #[inline]
    fn minus(&self, other: &i128) -> i128 {
        self - other
    }
    #[inline]
    fn times(&self, other: &i128) -> i128 {
        self * other
    }
    #[inline]
    fn plus(&self, other: &i128) -> i128 {
        self + other
    }
    #[inline]
    fn nonzero(&self) -> bool {
        *self != 0
    }
}

impl Int for IBig {
    fn minus(&self, other: &IBig) -> IBig {
        self - other
    }
    fn times(&self, other: &IBig) -> IBig {
        self * other
    }
    fn plus(&self, other: &IBig) -> IBig {
        self + other
    }
    fn nonzero(&self) -> bool {
        *self != IBig::ZERO
    }
}

/// `restored_right_corners(snapped, intended)`: the intended corners that are not degenerate on the snapped positions and have `cos = 0` (see the note of the module).
fn restored<N: Int>(points: &[[N; 3]], intended: &[Corner]) -> u32 {
    let mut total = 0;
    for corner in intended {
        let (previous, vertex, following) = (&points[corner[0] as usize], &points[corner[1] as usize], &points[corner[2] as usize]);
        let left = [previous[0].minus(&vertex[0]), previous[1].minus(&vertex[1]), previous[2].minus(&vertex[2])];
        let right = [following[0].minus(&vertex[0]), following[1].minus(&vertex[1]), following[2].minus(&vertex[2])];
        let normal = [
            left[1].times(&right[2]).minus(&left[2].times(&right[1])),
            left[2].times(&right[0]).minus(&left[0].times(&right[2])),
            left[0].times(&right[1]).minus(&left[1].times(&right[0])),
        ];
        if !(normal[0].nonzero() || normal[1].nonzero() || normal[2].nonzero()) {
            continue;
        }
        let cosine = left[0].times(&right[0]).plus(&left[1].times(&right[1])).plus(&left[2].times(&right[2]));
        if !cosine.nonzero() {
            total += 1;
        }
    }
    total
}

/// The widest magnitude (in bits) the `i128` path of [`restored`] runs on: a difference is below `2^(B+1)`, a cross-product component below `2^(2B+3)`, a dot product below `2^(2B+3)`.
const FIXED_BITS: u32 = 60;

enum Snapped {
    Fixed(Vec<[i128; 3]>),
    Big(Vec<[IBig; 3]>),
}

fn snap(dyadic: &[[Dyadic; 3]], exponent: u32) -> Snapped {
    let mut fixed: Vec<[i128; 3]> = Vec::with_capacity(dyadic.len());
    let mut fits = true;
    'points: for point in dyadic {
        let mut out = [0i128; 3];
        for axis in 0..3 {
            match point[axis].snap_fixed(exponent) {
                Some(value) if value.unsigned_abs() < (1u128 << FIXED_BITS) => out[axis] = value,
                _ => {
                    fits = false;
                    break 'points;
                }
            }
        }
        fixed.push(out);
    }
    if fits {
        return Snapped::Fixed(fixed);
    }
    Snapped::Big(dyadic.iter().map(|point| [point[0].snap_big(exponent), point[1].snap_big(exponent), point[2].snap_big(exponent)]).collect())
}

fn restored_count(snapped: &Snapped, intended: &[Corner]) -> u32 {
    match snapped {
        Snapped::Fixed(points) => restored(points, intended),
        Snapped::Big(points) => restored(points, intended),
    }
}

fn into_big(snapped: Snapped) -> Vec<[IBig; 3]> {
    match snapped {
        Snapped::Fixed(points) => points.into_iter().map(|point| [IBig::from(point[0]), IBig::from(point[1]), IBig::from(point[2])]).collect(),
        Snapped::Big(points) => points,
    }
}

/// The window, the corners, the search over the scales finest-first (`FINEST_ADMISSIBLE_FIRST_V1`) and the snap, for a law that snaps the source.
///
/// Declines (the oracle refuses or raises there): a patch of zero extent, a closed window, a window without a power of two, a search without a scale that restores the intended corners.
pub fn resolve(positions: &[[f64; 3]], faces: &[FaceIn]) -> Outcome<Resolved> {
    let mut dyadic: Vec<[Dyadic; 3]> = Vec::with_capacity(positions.len());
    for point in positions {
        let (Some(x), Some(y), Some(z)) = (Dyadic::of(point[0]), Dyadic::of(point[1]), Dyadic::of(point[2])) else {
            return decline(Reason::Shape, "a position is not finite");
        };
        dyadic.push([x, y, z]);
    }
    let Some(e_min) = minimum_exponent(&dyadic) else {
        return decline(Reason::Frame, "the source vertices do not define a non-zero basis A: all positions are zero");
    };
    let lifted: Vec<[IBig; 3]> = dyadic.iter().map(|point| [point[0].lifted(e_min), point[1].lifted(e_min), point[2].lifted(e_min)]).collect();
    let extent_units = max_min_extent(&lifted);
    if extent_units == IBig::ZERO {
        return decline(Reason::Frame, "the source vertices do not define a non-zero basis A: zero extent");
    }
    let extent = scaled_rational(extent_units, e_min);
    // lower = 2 * AUTHOR_ANGULAR_ERROR * extent; upper = DECAL_DETAIL
    let lower = Rat::from_i64(2).mul(&rational_of(AUTHOR_ANGULAR_ERROR.0, AUTHOR_ANGULAR_ERROR.1)).mul(&extent);
    let upper = rational_of(DECAL_DETAIL.0, DECAL_DETAIL.1);
    if lower > upper {
        return decline(Reason::Grid, "GRID_WINDOW_CLOSED");
    }
    // the last scale 2^e whose step 1/2^e is not below the lower bound: 1/2^e >= lower  <=>  lower.num * 2^e <= lower.den
    let numerator = lower.numerator().clone();
    let denominator = IBig::from(lower.denominator().clone());
    let mut exponent: u32 = 0;
    while (&numerator << exponent as usize) <= denominator {
        exponent += 1;
        if exponent > MAXIMUM_EXPONENT + 1 {
            return decline(Reason::Range, "the finest admissible step is below 2^-100");
        }
    }
    exponent -= 1;
    if exponent < COARSEST_EXPONENT {
        // the step 2^-exponent is above the upper bound 1/100: no power of two in the window
        return decline(Reason::Grid, "NO_POWER_OF_TWO_STEP_IN_WINDOW");
    }
    let corners = classify_corners(&lifted, faces);
    let wanted = corners.intended.len() as u32;
    let mut trials: Vec<Trial> = Vec::new();
    let mut chosen: Option<(u32, Snapped)> = None;
    let mut trial_exponent = exponent;
    loop {
        let snapped = snap(&dyadic, trial_exponent);
        let count = restored_count(&snapped, &corners.intended);
        let passed = count == wanted;
        trials.push(Trial { exponent: trial_exponent, restored: count, passed });
        if passed {
            chosen = Some((trial_exponent, snapped));
            break;
        }
        if trial_exponent == COARSEST_EXPONENT {
            break;
        }
        trial_exponent -= 1;
    }
    let Some((exponent, snapped)) = chosen else {
        return decline(Reason::Grid, "NO_GRID_SCALE_RESTORES_RELATIONS");
    };
    let restored_total = trials.last().map_or(0, |trial| trial.restored);
    debug_assert_eq!(restored_total, wanted);
    let facts = GridFacts { extent, window_lower: lower, window_upper: upper, exponent, intended_right_corners: wanted, trials };
    Ok(Resolved { facts, snapped: into_big(snapped), corners, dyadic, e_min, lifted })
}
