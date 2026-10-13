//! Конечная таблица owned source canon; Numeric только для multiquadratic islands.
//! Не меняет legacy admission/sign. Неизвестная компонента остаётся raw trig;
//! exact_q sin(pi/5) повторяет density_trig_nodes как explicit opaque nested root.
use crate::density_construct_v1::{ConstructErrorV1, ConstructorV1, OwnedExprV1};
use crate::scalar_v2::{ScalarError, ScalarV2};
use crate::{IBig, Rational, UBig};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TrigErrorV1 {
    RatioOutsideClosedUnitInterval,
    InconsistentDotSign,
    InvalidSubdivision,
    InvalidOrdinal,
    UnsupportedDensityQ,
    Construction(ConstructErrorV1),
}
impl From<ConstructErrorV1> for TrigErrorV1 {
    fn from(value: ConstructErrorV1) -> Self {
        Self::Construction(value)
    }
}
impl From<ScalarError> for TrigErrorV1 {
    fn from(value: ScalarError) -> Self {
        Self::Construction(ConstructErrorV1::Scalar(value))
    }
}
pub type TrigResultV1<T> = Result<T, TrigErrorV1>;

fn rational(n: i32, d: u32) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).expect("positive table denominator")
}

/// Только пять объявленных cos(theta)^2; valid unknown ratio не является отказом.
pub fn principal_pi_multiple(ratio: &Rational, sign: i8) -> TrigResultV1<Option<Rational>> {
    let (p, q) = ratio.parts();
    if p < &IBig::ZERO || p > &IBig::from(q.clone()) {
        return Err(TrigErrorV1::RatioOutsideClosedUnitInterval);
    }
    if !((p.is_zero() && sign == 0) || (p > &IBig::ZERO && matches!(sign, -1 | 1))) {
        return Err(TrigErrorV1::InconsistentDotSign);
    }
    if p.is_zero() {
        return Ok(Some(rational(1, 2)));
    }
    let positive = if ratio == &rational(1, 4) {
        rational(1, 3)
    } else if ratio == &rational(1, 2) {
        rational(1, 4)
    } else if ratio == &rational(3, 4) {
        rational(1, 6)
    } else if ratio == &rational(1, 1) {
        rational(0, 1)
    } else {
        return Ok(None);
    };
    Ok(Some(if sign > 0 {
        positive
    } else {
        let (n, d) = positive.parts();
        Rational::new(IBig::from(d.clone()) - n, d.clone()).expect("positive denominator")
    }))
}

type Recipe = &'static [(i32, i32, u32)];
fn quadrant_recipe(n: i32, d: u32) -> Option<(Option<Recipe>, Option<Recipe>)> {
    Some(match (n, d) {
        (0, 1) => (Some(&[(1, 1, 1)]), Some(&[])),
        (1, 2) => (Some(&[]), Some(&[(1, 1, 1)])),
        (1, 3) => (Some(&[(1, 1, 2)]), Some(&[(3, 1, 2)])),
        (1, 4) => (Some(&[(2, 1, 2)]), Some(&[(2, 1, 2)])),
        (1, 6) => (Some(&[(3, 1, 2)]), Some(&[(1, 1, 2)])),
        (1, 12) => (
            Some(&[(6, 1, 4), (2, 1, 4)]),
            Some(&[(6, 1, 4), (2, -1, 4)]),
        ),
        (5, 12) => (
            Some(&[(6, 1, 4), (2, -1, 4)]),
            Some(&[(6, 1, 4), (2, 1, 4)]),
        ),
        (1, 5) => (Some(&[(1, 1, 4), (5, 1, 4)]), None),
        (2, 5) => (Some(&[(1, -1, 4), (5, 1, 4)]), None),
        (1, 10) => (None, Some(&[(1, -1, 4), (5, 1, 4)])),
        (3, 10) => (None, Some(&[(1, 1, 4), (5, 1, 4)])),
        _ => return None,
    })
}
fn scalar(recipe: Recipe, sign: i32) -> TrigResultV1<ScalarV2> {
    let mut value = ScalarV2::rational(&rational(0, 1));
    for &(radicand, numerator, denominator) in recipe {
        let root = if radicand == 1 {
            ScalarV2::rational(&rational(1, 1))
        } else {
            ScalarV2::sqrt_rational(&rational(radicand, 1))?
        };
        value = value.add(&root.scaled(&rational(sign * numerator, denominator)));
    }
    Ok(value)
}
fn numeric_components(multiple: &Rational) -> TrigResultV1<[Option<ScalarV2>; 2]> {
    let (phase, signs) = quadrant(multiple);
    let (n, d) = phase.parts();
    let key = (i32::try_from(n.clone()).ok(), u32::try_from(d.clone()).ok());
    let (Some(n), Some(d)) = key else {
        return Ok([None, None]);
    };
    let Some((cosine, sine)) = quadrant_recipe(n, d) else {
        return Ok([None, None]);
    };
    Ok([
        cosine.map(|v| scalar(v, signs[0])).transpose()?,
        sine.map(|v| scalar(v, signs[1])).transpose()?,
    ])
}
fn quadrant(multiple: &Rational) -> (Rational, [i32; 2]) {
    let (n, d) = multiple.parts();
    let denominator = IBig::from(d.clone());
    let modulus = &denominator * IBig::from(2);
    let mut phase = n % &modulus;
    if phase < IBig::ZERO {
        phase += &modulus;
    }
    let mut signs = [1, 1];
    if phase > denominator {
        phase = &modulus - phase;
        signs[1] = -1;
    }
    if &phase * IBig::from(2) > denominator {
        phase = &denominator - phase;
        signs[0] = -1;
    }
    (
        Rational::new(phase, d.clone()).expect("positive denominator"),
        signs,
    )
}
fn fifth_root(c: &mut ConstructorV1<'_>, positive: bool) -> TrigResultV1<OwnedExprV1> {
    let five = c.numeric(ScalarV2::sqrt_rational(&rational(5, 1))?)?;
    let term = c.scaled(&five, &rational(if positive { 2 } else { -2 }, 1))?;
    let ten = c.rational(&rational(10, 1))?;
    let radicand = c.add(&[ten, term])?;
    let root = c.sqrt(&radicand)?;
    Ok(c.scaled(&root, &rational(1, 4))?)
}
fn eighth_pair(c: &mut ConstructorV1<'_>) -> TrigResultV1<[OwnedExprV1; 2]> {
    let two = c.rational(&rational(2, 1))?;
    let root_two = c.numeric(ScalarV2::sqrt_rational(&rational(2, 1))?)?;
    let plus = c.add(&[two.clone(), root_two.clone()])?;
    let minus = c.sub(&two, &root_two)?;
    let cosine = c.sqrt(&plus)?;
    let sine = c.sqrt(&minus)?;
    Ok([
        c.scaled(&cosine, &rational(1, 2))?,
        c.scaled(&sine, &rational(1, 2))?,
    ])
}
fn combine_pair(
    c: &mut ConstructorV1<'_>,
    [ca, sa]: [OwnedExprV1; 2],
    [cb, sb]: [OwnedExprV1; 2],
    sign: i32,
) -> TrigResultV1<[OwnedExprV1; 2]> {
    let cc = c.mul(&[ca.clone(), cb.clone()])?;
    let ss = c.mul(&[sa.clone(), sb.clone()])?;
    let ss = c.scaled(&ss, &rational(sign, 1))?;
    let cosine = c.sub(&cc, &ss)?;
    let sc = c.mul(&[sa, cb])?;
    let cs = c.mul(&[ca, sb])?;
    let cs = c.scaled(&cs, &rational(sign, 1))?;
    let sine = c.add(&[sc, cs])?;
    Ok([cosine, sine])
}
/// Только actual old-special whitelist, НЕ generic half-angle classification.
fn finite_quadrant_pair(
    c: &mut ConstructorV1<'_>,
    phase: &Rational,
) -> TrigResultV1<Option<[OwnedExprV1; 2]>> {
    let [cosine, sine] = numeric_components(phase)?;
    if let (Some(a), Some(b)) = (&cosine, &sine) {
        return Ok(Some([c.numeric(a.clone())?, c.numeric(b.clone())?]));
    }
    let (n, d) = phase.parts();
    let key = (i32::try_from(n.clone()).ok(), u32::try_from(d.clone()).ok());
    let (Some(n), Some(d)) = key else {
        return Ok(None);
    };
    match (n, d) {
        (1, 8) | (3, 8) => {
            let [a, b] = eighth_pair(c)?;
            return Ok(Some(if n == 1 { [a, b] } else { [b, a] }));
        }
        (1, 5) | (2, 5) => {
            return Ok(Some([
                c.numeric(cosine.expect("known cosine"))?,
                fifth_root(c, n == 2)?,
            ]))
        }
        (1, 10) | (3, 10) => {
            return Ok(Some([
                fifth_root(c, n == 1)?,
                c.numeric(sine.expect("known sine"))?,
            ]))
        }
        _ => {}
    }
    // Только шесть явных дополнений прежних ключей, без recursive closure.
    let complement = match (n, d) {
        (7, 30) => Some(rational(4, 15)),
        (7, 24) => Some(rational(5, 24)),
        (7, 20) => Some(rational(3, 20)),
        (11, 30) => Some(rational(2, 15)),
        (13, 30) => Some(rational(1, 15)),
        (11, 24) => Some(rational(1, 24)),
        _ => None,
    };
    if let Some(original) = complement {
        let [a, b] = finite_quadrant_pair(c, &original)?.expect("closed original key");
        return Ok(Some([b, a]));
    }
    let (left, right, sign) = match (n, d) {
        (1, 30) => (rational(1, 5), rational(1, 6), -1),
        (1, 24) => (rational(1, 6), rational(1, 8), -1),
        (1, 20) => (rational(1, 4), rational(1, 5), -1),
        (1, 15) => (rational(1, 6), rational(1, 10), -1),
        (2, 15) => (rational(1, 3), rational(1, 5), -1),
        (3, 20) => (rational(1, 4), rational(1, 10), -1),
        (5, 24) => (rational(1, 3), rational(1, 8), -1),
        (4, 15) => (rational(1, 6), rational(1, 10), 1),
        (7, 15) => (rational(1, 6), rational(3, 10), 1),
        (9, 20) => (rational(1, 4), rational(1, 5), 1),
        _ => return Ok(None),
    };
    let left = finite_quadrant_pair(c, &left)?.expect("closed recipe child");
    let right = finite_quadrant_pair(c, &right)?.expect("closed recipe child");
    combine_pair(c, left, right, sign).map(Some)
}

/// Отсутствующая компонента использует ORIGINAL multiple*pi, без opaque identities.
pub fn owned_rational_pi_trig(
    c: &mut ConstructorV1<'_>,
    multiple: &Rational,
) -> TrigResultV1<[OwnedExprV1; 2]> {
    let [cosine, sine] = numeric_components(multiple)?;
    if cosine.is_none() || sine.is_none() {
        let (phase, signs) = quadrant(multiple);
        if let Some([a, b]) = finite_quadrant_pair(c, &phase)? {
            return Ok([
                c.scaled(&a, &rational(signs[0], 1))?,
                c.scaled(&b, &rational(signs[1], 1))?,
            ]);
        }
    }
    let angle = if cosine.is_none() || sine.is_none() {
        let pi = c.pi()?;
        Some(c.scaled(&pi, multiple)?)
    } else {
        None
    };
    let cosine = match cosine {
        Some(v) => c.numeric(v)?,
        None => c.raw_cos(angle.as_ref().unwrap())?,
    };
    let sine = match sine {
        Some(v) => c.numeric(v)?,
        None => c.raw_sin(angle.as_ref().unwrap())?,
    };
    Ok([cosine, sine])
}

pub fn owned_principal_ray(
    c: &mut ConstructorV1<'_>,
    ratio: &Rational,
    sign: i8,
    subdivision: u8,
    ordinal: u8,
) -> TrigResultV1<Option<[OwnedExprV1; 2]>> {
    if !(2..=6).contains(&subdivision) {
        return Err(TrigErrorV1::InvalidSubdivision);
    }
    if ordinal > subdivision {
        return Err(TrigErrorV1::InvalidOrdinal);
    }
    let Some(angle) = principal_pi_multiple(ratio, sign)? else {
        return Ok(None);
    };
    let (n, d) = angle.parts();
    let multiple = Rational::new(n * IBig::from(ordinal), d * UBig::from(subdivision))
        .expect("positive subdivision");
    owned_rational_pi_trig(c, &multiple).map(Some)
}

pub fn owned_exact_q_trig(c: &mut ConstructorV1<'_>, q: u8) -> TrigResultV1<[OwnedExprV1; 2]> {
    if !(2..=6).contains(&q) {
        return Err(TrigErrorV1::UnsupportedDensityQ);
    }
    owned_rational_pi_trig(c, &rational(1, u32::from(q)))
}
