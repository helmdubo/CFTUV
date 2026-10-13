//! `validate_rational_affine_planar_metric` and the source comparison of `validate_metric_against_source`, for the metric the port carries: a `RationalAffinePlanarMetricV2` with an
//! `ExactSourcePlaneCertificateV1` and the canonical source-vertex basis (`validation_metric.py`, mirrored here step by step).
//!
//! Every place where the Python function adds an issue is a decline. A metric with another certificate (near-planar projection, developable unfolding, band chart) or another frame law never reaches this
//! module: the reader gives it as `Metric::Other` and the structural pass declines it by name.

use std::collections::HashSet;

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::rat::Rat;

use crate::decimal::rat_of_f64;
use crate::model::{Affine, Id, Snapshot};
use crate::verdict::{issue, Check};

type Vec3 = [Rat; 3];

fn dot3(left: &Vec3, right: &Vec3) -> Rat {
    // `fraction_dot3`: `sum((a * b ...), Fraction(0))`
    let mut total = Rat::zero();
    for axis in 0..3 {
        total = total.add(&left[axis].mul(&right[axis]));
    }
    total
}

fn cross3(left: &Vec3, right: &Vec3) -> Vec3 {
    [
        left[1].mul(&right[2]).sub(&left[2].mul(&right[1])),
        left[2].mul(&right[0]).sub(&left[0].mul(&right[2])),
        left[0].mul(&right[1]).sub(&left[1].mul(&right[0])),
    ]
}

fn is_zero_vector(vector: &Vec3) -> bool {
    vector.iter().all(Rat::is_zero)
}

/// `numeric.canonical_primitive_normal`: integers, gcd 1, the first non-zero coordinate positive. The caller guarantees a non-zero vector (`PlaneNormalUndefinedError` otherwise).
fn canonical_primitive_normal(vector: &Vec3) -> Vec3 {
    let mut denominator = UBig::ONE;
    for item in vector {
        denominator = num::lcm(&denominator, item.denominator());
    }
    let denominator_signed = IBig::from(denominator);
    // `int(item * denominator)`: the product is an integer, so the truncation is exact
    let mut integers: Vec<IBig> = vector.iter().map(|item| item.numerator() * &denominator_signed / IBig::from(item.denominator().clone())).collect();
    let mut divisor = UBig::ZERO;
    for item in &integers {
        divisor = num::gcd(&divisor, &num::magnitude(item));
    }
    let divisor_signed = IBig::from(divisor);
    for item in &mut integers {
        *item = floor_div(item, &divisor_signed);
    }
    let first_negative = integers.iter().find(|item| !item.is_zero()).map(num::is_negative).unwrap_or(false);
    if first_negative {
        for item in &mut integers {
            *item = -item.clone();
        }
    }
    [Rat::from_int(integers[0].clone()), Rat::from_int(integers[1].clone()), Rat::from_int(integers[2].clone())]
}

/// Python's `a // b` for a positive `b`.
pub fn floor_div(a: &IBig, b: &IBig) -> IBig {
    let quotient = a / b;
    let remainder = a - &quotient * b;
    if num::is_negative(&remainder) {
        quotient - IBig::ONE
    } else {
        quotient
    }
}

/// `classify_metric_normal_wire(declared, basis_normal)` with no compatibility policy: accepted only when the declared normal IS the canonical primitive one.
fn normal_is_accepted(declared: &Vec3, basis_normal: &Vec3) -> bool {
    if is_zero_vector(basis_normal) || is_zero_vector(declared) {
        return false;
    }
    let canonical = canonical_primitive_normal(basis_normal);
    if !is_zero_vector(&cross3(declared, &canonical)) {
        return false;
    }
    let pivot = canonical.iter().position(|item| !item.is_zero()).expect("a non-zero vector has a pivot");
    let Ok(shared_scale) = declared[pivot].div(&canonical[pivot]) else { return false };
    if (0..3).any(|index| declared[index] != shared_scale.mul(&canonical[index])) {
        return false;
    }
    if shared_scale.signum() < 0 {
        return false;
    }
    // CANONICAL_PRIMITIVE is `declared == canonical`; a positive multiple is POSITIVE_SCALED_COMPATIBILITY_EXHAUSTED under the empty policy
    declared == &canonical
}

/// `_check_gram`: the Gram matrix is the dot products of the basis, positive definite, and the inverse is exact.
fn gram_is_exact(metric: &Affine) -> Check<()> {
    let expected = [
        [dot3(&metric.basis_a, &metric.basis_a), dot3(&metric.basis_a, &metric.basis_b)],
        [dot3(&metric.basis_b, &metric.basis_a), dot3(&metric.basis_b, &metric.basis_b)],
    ];
    if metric.gram != expected {
        return issue("exact_gram_matrix differs from the dot products of the basis");
    }
    let gram = &metric.gram;
    let determinant = gram[0][0].mul(&gram[1][1]).sub(&gram[0][1].mul(&gram[1][0]));
    if determinant.signum() <= 0 {
        return issue("Gram matrix is not positive definite");
    }
    let divide = |value: Rat| value.div(&determinant).expect("the determinant is positive");
    let expected_inverse = [[divide(gram[1][1].clone()), divide(gram[0][1].neg())], [divide(gram[1][0].neg()), divide(gram[0][0].clone())]];
    if metric.inverse_gram != expected_inverse {
        return issue("inverse Gram matrix is not exact");
    }
    Ok(())
}

/// `validate_rational_affine_planar_metric(metric)` for the metric the port carries; any issue it would add is a decline.
pub fn validate_affine(metric: &Affine) -> Check<()> {
    let certificate = &metric.certificate;
    if certificate.domain != metric.domain || !certificate.exact_law || !certificate.reconstruction_ok || !certificate.exact {
        return issue("metric requires an exact same-domain source-plane certificate");
    }
    let normal = cross3(&metric.basis_a, &metric.basis_b);
    if is_zero_vector(&normal) {
        return issue("affine basis vectors must be linearly independent");
    }
    if !normal_is_accepted(&certificate.normal, &normal) {
        return issue("plane normal is not the canonical primitive normal of the basis");
    }
    gram_is_exact(metric)?;
    let mut seen: HashSet<Id, FxBuild> = HashSet::with_capacity_and_hasher(metric.coordinates.len(), FxBuild::default());
    for (id, _) in &metric.coordinates {
        seen.insert(*id);
    }
    if seen.len() != metric.coordinates.len() {
        return issue("duplicate source vertex coordinate");
    }
    let certified: HashSet<Id, FxBuild> = certificate.vertex_ids.iter().copied().collect();
    if seen != certified {
        return issue("coordinate and planarity-certificate vertex sets differ");
    }
    Ok(())
}

/// `position_under_grid_law` for one coordinate: the exact binary64, or its grid node when the law snaps the source and the certificate names a scale.
fn node_coordinate(value: f64, metric: &Affine) -> Check<Option<Rat>> {
    let Some(exact) = rat_of_f64(value) else { return crate::verdict::decline(crate::verdict::Reason::Shape, "a non-finite source coordinate") };
    let grid = &metric.grid;
    let Some(scale) = grid.source_scale.as_ref().filter(|_| grid.snaps_source) else { return Ok(Some(exact)) };
    if !scale.is_positive_value() {
        return crate::verdict::decline(crate::verdict::Reason::Shape, "a grid scale that is not positive (GridSpecV1 raises)");
    }
    // snap_value: floor((2 numerator + denominator) / (2 denominator)) of value * scale, then / scale
    let scaled = exact.mul(&Rat::from_int(scale.clone()));
    let denominator = IBig::from(scaled.denominator().clone());
    let twice = &denominator * IBig::from(2);
    let snapped = floor_div(&(scaled.numerator() * IBig::from(2) + &denominator), &twice);
    Ok(Some(Rat::new(snapped, scale.clone()).expect("the scale is positive")))
}

trait Positive {
    fn is_positive_value(&self) -> bool;
}

impl Positive for IBig {
    fn is_positive_value(&self) -> bool {
        !self.is_zero() && !num::is_negative(self)
    }
}

/// `validate_metric_against_source` for the exact-plane metric: `origin + u A + v B` of every coordinate is the position the declared laws give the vertex.
pub fn against_source(snapshot: &Snapshot, metric: &Affine) -> Check<()> {
    let positions: std::collections::HashMap<Id, [f64; 3], FxBuild> = snapshot.vertices.iter().filter_map(|vertex| vertex.position.map(|position| (vertex.id, position))).collect();
    // `source_by_id.get(...)` of a vertex that exists but has no local position is skipped; a vertex that does not exist is skipped too
    for (id, coordinate) in &metric.coordinates {
        let Some(position) = positions.get(id) else { continue };
        let mut node: Vec<Rat> = Vec::with_capacity(3);
        for axis in 0..3 {
            node.push(node_coordinate(position[axis], metric)?.expect("a node"));
        }
        for axis in 0..3 {
            let reconstructed = metric.origin[axis].add(&coordinate[0].mul(&metric.basis_a[axis])).add(&coordinate[1].mul(&metric.basis_b[axis]));
            if reconstructed != node[axis] {
                return issue("exact affine reconstruction disagrees with the source position the declared laws produce");
            }
        }
    }
    Ok(())
}
