//! Чистые Stage B примитивы: certified rational facts, angle descriptors и table rays.
//! Входные Rational факты уже сертифицированы прежним admission; канон V2 его не доказывает.
//! Здесь нет legacy DAG/text, fan orchestration, mutable memo или бюджета Density-поиска.
//! Principal дескриптор не вычисляет atan2/sin/cos; V2 Work остаётся диагностикой.

use cftuv_radical::arith::{magnitude, signum};
use cftuv_radical::Failure;

use crate::{IBig, Priced, Radical, Rational, UBig, Vector2, Work};

/// Доказанные вызывающим adapter нормы и `expand(raw_dot²).is_Rational` факт.
/// Тип поля ограничивает wire, но не является новым доказательством old-syntax admission.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CertifiedGramSquares {
    pub incoming_squared: Rational,
    pub outgoing_squared: Rational,
    pub dot_squared: Rational,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RayProductReason {
    StrictPrincipalTurnRequired,
}

impl RayProductReason {
    pub fn outcome(&self) -> &'static str {
        "PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED"
    }
    pub fn message(&self) -> &'static str {
        "Density A requires a strict principal turn in (0, pi)"
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RayInvalid {
    NonPositiveCertifiedNorm,
    InconsistentDotSign(i8),
    InvalidSubdivision(u8),
    InvalidOrdinal { ordinal: u8, subdivision: u8 },
    WrongTableLength { expected: usize, actual: usize },
    ZeroRotationRow { ordinal: usize },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RayFailureKind {
    Product(RayProductReason),
    InternalInvalid(RayInvalid),
    InternalAlgebra(Failure),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RayFailure {
    pub kind: RayFailureKind,
    pub work: Work,
}

pub type RayResult<T> = Result<Priced<T>, RayFailure>;

impl RayFailure {
    fn invalid(reason: RayInvalid, work: Work) -> Self {
        Self {
            kind: RayFailureKind::InternalInvalid(reason),
            work,
        }
    }
    fn after(mut self, work: Work) -> Self {
        self.work = work + self.work;
        self
    }
}

/// Гейт :145–151 density_turn.py: отрицательный c² проверяется раньше c²>=1.
fn validate_cosine_squared(ratio: &Rational) -> Result<(), RayFailure> {
    let (n, d) = ratio.parts();
    if signum(n) < 0 || *n >= IBig::from(d.clone()) {
        return Err(RayFailure {
            kind: RayFailureKind::Product(RayProductReason::StrictPrincipalTurnRequired),
            work: Work::NONE,
        });
    }
    Ok(())
}

pub fn cosine_squared(facts: &CertifiedGramSquares) -> RayResult<Rational> {
    let (left_n, left_d) = facts.incoming_squared.parts();
    let (right_n, right_d) = facts.outgoing_squared.parts();
    if signum(left_n) <= 0 || signum(right_n) <= 0 {
        return Err(RayFailure::invalid(
            RayInvalid::NonPositiveCertifiedNorm,
            Work::NONE,
        ));
    }
    let (dot_n, dot_d) = facts.dot_squared.parts();
    let n = dot_n * IBig::from(left_d.clone()) * IBig::from(right_d.clone());
    let d = dot_d * magnitude(left_n) * magnitude(right_n);
    let ratio = Rational::new(n, d)
        .expect("сертифицированные положительные нормы дают ненулевой знаменатель");
    validate_cosine_squared(&ratio)?;
    Ok((ratio, Work::NONE))
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SignedCosRatio {
    cosine_squared: Rational,
    dot_sign: i8,
}

impl SignedCosRatio {
    pub fn cosine_squared(&self) -> &Rational {
        &self.cosine_squared
    }
    pub fn dot_sign(&self) -> i8 {
        self.dot_sign
    }
}

/// Означает principal atan2(sqrt(1-r), sign*sqrt(r)); не expression tree.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PrincipalTurn {
    source: SignedCosRatio,
}

impl PrincipalTurn {
    pub fn source(&self) -> &SignedCosRatio {
        &self.source
    }
}

/// Adapter сам решает, когда вызвать эту ступень; table mode её не требует.
pub fn principal_turn(cosine_squared: &Rational, raw_dot: &Radical) -> RayResult<PrincipalTurn> {
    validate_cosine_squared(cosine_squared)?;
    let (dot_sign, work) = raw_dot
        .signum(None)
        .map_err(|failure| algebra(failure, Work::NONE))?;
    let ratio_zero = cosine_squared.parts().0.is_zero();
    if (dot_sign == 0) != ratio_zero {
        return Err(RayFailure::invalid(
            RayInvalid::InconsistentDotSign(dot_sign),
            work,
        ));
    }
    Ok((
        PrincipalTurn {
            source: SignedCosRatio {
                cosine_squared: cosine_squared.clone(),
                dot_sign,
            },
        },
        work,
    ))
}

/// Канонический угол луча: точная сокращённая доля pi, без trig materialization.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PiMultiple {
    coefficient: Rational,
}

impl PiMultiple {
    pub fn coefficient(&self) -> &Rational {
        &self.coefficient
    }

    pub fn ray_angle(canonical: &Rational, ordinal: u8, subdivision: u8) -> RayResult<Self> {
        if !(2..=6).contains(&subdivision) {
            return Err(RayFailure::invalid(
                RayInvalid::InvalidSubdivision(subdivision),
                Work::NONE,
            ));
        }
        if ordinal == 0 || ordinal >= subdivision {
            return Err(RayFailure::invalid(
                RayInvalid::InvalidOrdinal {
                    ordinal,
                    subdivision,
                },
                Work::NONE,
            ));
        }
        let (n, d) = canonical.parts();
        let coefficient = Rational::new(n * IBig::from(ordinal), d * UBig::from(subdivision))
            .expect("ненулевой subdivision сохраняет знаменатель");
        Ok((Self { coefficient }, Work::NONE))
    }
}

/// Source :181–187: нормализованный integer(a,b) в Gram basis StageA.
/// Orientation — числовой множитель, нового раннего ±1 валидатора здесь нет.
pub fn table_ray(
    incoming: &Vector2,
    left: &Vector2,
    orientation: i8,
    rotation: &(IBig, IBig),
) -> RayResult<Vector2> {
    if rotation.0.is_zero() && rotation.1.is_zero() {
        return Err(RayFailure::invalid(
            RayInvalid::ZeroRotationRow { ordinal: 1 },
            Work::NONE,
        ));
    }
    let length_squared = &rotation.0 * &rotation.0 + &rotation.1 * &rotation.1;
    let (length, mut work) = Radical::sqrt_of_rational(length_squared, UBig::ONE)
        .map_err(|refusal| algebra(refusal.into(), Work::NONE))?;
    let scale_left = &rotation.1 * IBig::from(orientation);
    let (x, priced) = incoming
        .x
        .scaled(&rotation.0, &UBig::ONE)
        .add(&left.x.scaled(&scale_left, &UBig::ONE));
    work = work + priced;
    let (x, priced) = x.div(&length).map_err(|failure| algebra(failure, work))?;
    work = work + priced;
    let (y, priced) = incoming
        .y
        .scaled(&rotation.0, &UBig::ONE)
        .add(&left.y.scaled(&scale_left, &UBig::ONE));
    work = work + priced;
    let (y, priced) = y.div(&length).map_err(|failure| algebra(failure, work))?;
    Ok((Vector2::new(x, y), work + priced))
}

pub fn table_rays(
    incoming: &Vector2,
    left: &Vector2,
    orientation: i8,
    expected_ordinals: usize,
    rotations: &[(IBig, IBig)],
) -> RayResult<Vec<Vector2>> {
    if rotations.len() != expected_ordinals {
        return Err(RayFailure::invalid(
            RayInvalid::WrongTableLength {
                expected: expected_ordinals,
                actual: rotations.len(),
            },
            Work::NONE,
        ));
    }
    let mut rays = Vec::with_capacity(rotations.len());
    let mut work = Work::NONE;
    for (index, rotation) in rotations.iter().enumerate() {
        let (ray, priced) =
            table_ray(incoming, left, orientation, rotation).map_err(|mut failure| {
                if matches!(
                    failure.kind,
                    RayFailureKind::InternalInvalid(RayInvalid::ZeroRotationRow { .. })
                ) {
                    failure.kind = RayFailureKind::InternalInvalid(RayInvalid::ZeroRotationRow {
                        ordinal: index + 1,
                    });
                }
                failure.after(work)
            })?;
        rays.push(ray);
        work = work + priced;
    }
    Ok((rays, work))
}

fn algebra(failure: Failure, work_before: Work) -> RayFailure {
    let failed_work = match &failure {
        Failure::Refused(refusal) => refusal.work,
        _ => Work::NONE,
    };
    RayFailure {
        kind: RayFailureKind::InternalAlgebra(failure),
        work: work_before + failed_work,
    }
}
