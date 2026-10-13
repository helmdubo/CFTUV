//! The plane of the snapped patch and its canonical source-vertex frame (`planar_metric._resolve_patch_plane`, `_affine_frame`, `_chart_orientation`, the exact-plane branch).
//!
//! The positions are integers `k` over the scale `S = 2^s` (the snapped map). Every quantity of the oracle is then a rational whose denominator is a power of `S` times a determinant, and
//! the whole computation is on the integers:
//!
//! * Newell's normal is the integer sum itself (the oracle's fractions carry the common factor `S^2`, which the canonical primitive form divides out);
//! * with `a`, `b` the integer basis vectors (`S` times the oracle's), `A00 = a.a`, `A01 = a.b`, `A11 = b.b`, `D = A00*A11 - A01^2`: the Gram matrix is `A / S^2`, its inverse
//!   `S^2 * [[A11, -A01], [-A01, A00]] / D`, and the coordinates of a vertex with `q = p - origin` are `u = (A11*Ra - A01*Rb) / D`, `v = (A00*Rb - A01*Ra) / D` with `Ra = a.q`, `Rb = b.q`
//!   (`S` cancels): `Rat::reduced` takes the one gcd that each of them needs;
//! * the exact reconstruction `origin + u*a + v*b = p` the oracle checks is `D*q = uN*a + vN*b` on the integers;
//! * the orientation sign of a face in the chart is the sign of `sum_i (q_i x q_{i+1})` along `a x b`, which is `twice_area_uv` times `a x b` (every `q` lies in the span of `a` and `b`): no division.

use cftuv_core::num::{gcd_signed, magnitude, IBig, UBig};
use cftuv_core::rat::Rat;

use crate::model::{decline, FaceIn, Outcome, Reason};

type V3 = [IBig; 3];

fn sub(left: &V3, right: &V3) -> V3 {
    [&left[0] - &right[0], &left[1] - &right[1], &left[2] - &right[2]]
}

fn dot(left: &V3, right: &V3) -> IBig {
    &left[0] * &right[0] + &left[1] * &right[1] + &left[2] * &right[2]
}

fn cross(left: &V3, right: &V3) -> V3 {
    [
        &left[1] * &right[2] - &left[2] * &right[1],
        &left[2] * &right[0] - &left[0] * &right[2],
        &left[0] * &right[1] - &left[1] * &right[0],
    ]
}

fn any_nonzero(vector: &V3) -> bool {
    vector.iter().any(|item| *item != IBig::ZERO)
}

/// `canonical_primitive_normal(Newell sum)`: integers with gcd 1, the first non-zero coordinate positive. `PLANE_NORMAL_UNDEFINED`: the faces define no plane (the oracle unfolds).
pub fn plane_normal(points: &[V3], faces: &[FaceIn]) -> Outcome<V3> {
    let mut sum = [IBig::ZERO, IBig::ZERO, IBig::ZERO];
    for face in faces {
        let cycle = &face.vertices;
        let count = cycle.len();
        for index in 0..count {
            let head = &points[cycle[index] as usize];
            let tail = &points[cycle[(index + 1) % count] as usize];
            sum[0] += (&head[1] - &tail[1]) * (&head[2] + &tail[2]);
            sum[1] += (&head[2] - &tail[2]) * (&head[0] + &tail[0]);
            sum[2] += (&head[0] - &tail[0]) * (&head[1] + &tail[1]);
        }
    }
    let divisor = gcd_signed(&gcd_signed_of(&sum[0], &sum[1]), &sum[2]);
    if divisor == UBig::ZERO {
        return decline(Reason::Plane, "PLANE_NORMAL_UNDEFINED");
    }
    let divisor = IBig::from(divisor);
    let mut normal = [&sum[0] / &divisor, &sum[1] / &divisor, &sum[2] / &divisor];
    let first = normal.iter().find(|item| **item != IBig::ZERO).expect("the gcd is not zero");
    if *first < IBig::ZERO {
        normal = [-&normal[0], -&normal[1], -&normal[2]];
    }
    Ok(normal)
}

fn gcd_signed_of(left: &IBig, right: &IBig) -> IBig {
    IBig::from(gcd_signed(left, right))
}

/// Whether every vertex lies in the plane through the first one with the given normal (`off_plane` of `_resolve_patch_plane` is empty).
pub fn is_exact_plane(points: &[V3], normal: &V3) -> bool {
    let anchor = &points[0];
    points.iter().all(|point| dot(&sub(point, anchor), normal) == IBig::ZERO)
}

fn reduced(numerator: IBig, denominator: &UBig) -> Rat {
    Rat::reduced(numerator, denominator.clone())
}

fn sign_of(value: &IBig) -> i8 {
    if *value == IBig::ZERO {
        0
    } else if *value < IBig::ZERO {
        -1
    } else {
        1
    }
}

/// The frame of the exactly planar patch.
pub struct Affine {
    pub origin: [Rat; 3],
    pub basis_a: [Rat; 3],
    pub basis_b: [Rat; 3],
    pub gram: [Rat; 3],
    pub inverse: [Rat; 3],
    pub coordinates: Vec<[Rat; 2]>,
    pub orientation_sign: i8,
}

/// `_affine_frame` (basis from the first two independent vertex vectors in rank order), `_chart_orientation`. Declines where the oracle raises `ValueError`.
pub fn frame(points: &[V3], exponent: u32, faces: &[FaceIn]) -> Outcome<Affine> {
    let scale = UBig::ONE << exponent as usize;
    let scale_squared = UBig::ONE << (2 * exponent as usize);
    let origin = &points[0];
    let relative: Vec<V3> = points.iter().map(|point| sub(point, origin)).collect();
    let Some(first) = relative.iter().skip(1).find(|vector| any_nonzero(vector)) else {
        return decline(Reason::Frame, "source vertices do not define a non-zero basis A");
    };
    let Some(second) = relative.iter().skip(1).find(|vector| any_nonzero(&cross(first, vector))) else {
        return decline(Reason::Frame, "source vertices do not define linearly independent A/B");
    };
    let (a00, a01, a11) = (dot(first, first), dot(first, second), dot(second, second));
    let determinant = &a00 * &a11 - &a01 * &a01;
    if determinant <= IBig::ZERO {
        return decline(Reason::Frame, "exact affine Gram matrix is not positive definite");
    }
    let determinant_unsigned = magnitude(&determinant);
    let scale_squared_signed = IBig::from(scale_squared.clone());
    let mut coordinates = Vec::with_capacity(points.len());
    for q in &relative {
        let ra = dot(first, q);
        let rb = dot(second, q);
        let u_numerator = &a11 * &ra - &a01 * &rb;
        let v_numerator = &a00 * &rb - &a01 * &ra;
        for axis in 0..3 {
            if &determinant * &q[axis] != &u_numerator * &first[axis] + &v_numerator * &second[axis] {
                return decline(Reason::Frame, "exact affine reconstruction failed");
            }
        }
        coordinates.push([reduced(u_numerator, &determinant_unsigned), reduced(v_numerator, &determinant_unsigned)]);
    }
    // orientation: the first face whose area vector along a x b is non-zero
    let normal_ab = cross(first, second);
    let axis = (0..3).find(|axis| normal_ab[*axis] != IBig::ZERO).expect("a x b is not zero");
    let (next, last) = ((axis + 1) % 3, (axis + 2) % 3);
    let mut orientation_sign = 0i8;
    for face in faces {
        let cycle = &face.vertices;
        let count = cycle.len();
        let mut area = IBig::ZERO;
        for index in 0..count {
            let head = &relative[cycle[index] as usize];
            let tail = &relative[cycle[(index + 1) % count] as usize];
            area += &head[next] * &tail[last] - &head[last] * &tail[next];
        }
        let sign = sign_of(&area) * sign_of(&normal_ab[axis]);
        if sign != 0 {
            orientation_sign = sign;
            break;
        }
    }
    if orientation_sign == 0 {
        return decline(Reason::Frame, "owner Patch has no non-degenerate oriented face");
    }
    let vector = |v: &V3| [reduced(v[0].clone(), &scale), reduced(v[1].clone(), &scale), reduced(v[2].clone(), &scale)];
    Ok(Affine {
        origin: vector(origin),
        basis_a: vector(first),
        basis_b: vector(second),
        gram: [reduced(a00.clone(), &scale_squared), reduced(a01.clone(), &scale_squared), reduced(a11.clone(), &scale_squared)],
        inverse: [
            reduced(&a11 * &scale_squared_signed, &determinant_unsigned),
            reduced(-(&a01 * &scale_squared_signed), &determinant_unsigned),
            reduced(&a00 * &scale_squared_signed, &determinant_unsigned),
        ],
        coordinates,
        orientation_sign,
    })
}
