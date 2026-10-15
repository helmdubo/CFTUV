use cftuv_radical::arith::{reduced, signum};
use cftuv_radical::{IBig, Radical, UBig, Work};

use crate::{DensityFailure, DensityResult, InvalidInput};

/// Каноническая рациональная запись; нулевой знаменатель не попадает в algebra.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Rational {
    numerator: IBig,
    denominator: UBig,
}

impl Rational {
    pub fn new(numerator: IBig, denominator: UBig) -> Result<Self, DensityFailure> {
        if denominator.is_zero() {
            return Err(DensityFailure::invalid(InvalidInput::ZeroDenominator));
        }
        let (numerator, denominator) = reduced(numerator, denominator);
        Ok(Self {
            numerator,
            denominator,
        })
    }

    pub fn integer(value: IBig) -> Self {
        Self {
            numerator: value,
            denominator: UBig::ONE,
        }
    }

    pub fn parts(&self) -> (&IBig, &UBig) {
        (&self.numerator, &self.denominator)
    }

    pub fn radical(&self) -> Radical {
        Radical::rational(self.numerator.clone(), self.denominator.clone())
    }

    fn from_rational_value(value: Radical) -> Self {
        let (numerator, denominator) = value
            .as_rational()
            .expect("операции над рациональными сохраняют рациональность");
        Self {
            numerator,
            denominator,
        }
    }

    pub(crate) fn scale(&self, value: &Radical) -> Radical {
        value.scaled(&self.numerator, &self.denominator)
    }
}

/// Covector или chart vector: Gram/dual Gram выбирает вызывающая операция.
/// Нулевой вектор допустим: это сохраняет поведение predicates oracle.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Vector2 {
    pub x: Radical,
    pub y: Radical,
}

impl Vector2 {
    pub fn new(x: Radical, y: Radical) -> Self {
        Self { x, y }
    }
}

/// Симметричная positive-definite рациональная метрика owner patch.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Metric2 {
    gram: [[Rational; 2]; 2],
    inverse_gram: [[Rational; 2]; 2],
    determinant: Rational,
    owner_orientation_sign: i8,
}

impl Metric2 {
    /// Inverse выводится из Gram; несогласованный второй matrix не принимается.
    pub fn from_gram(gram: [[Rational; 2]; 2], owner_orientation_sign: i8) -> DensityResult<Self> {
        if !matches!(owner_orientation_sign, -1 | 1) {
            return Err(DensityFailure::invalid(
                InvalidInput::InvalidOwnerOrientation(owner_orientation_sign),
            ));
        }
        if gram[0][1] != gram[1][0] {
            return Err(DensityFailure::invalid(InvalidInput::AsymmetricGram));
        }
        let (diagonal, first) = gram[0][0].radical().mul(&gram[1][1].radical());
        let (off_diagonal, second) = gram[0][1].radical().mul(&gram[1][0].radical());
        let (determinant_value, third) = diagonal.sub(&off_diagonal);
        let work = first + second + third;
        let determinant = Rational::from_rational_value(determinant_value.clone());
        if signum(&gram[0][0].numerator) <= 0 || signum(&determinant.numerator) <= 0 {
            return Err(DensityFailure::invalid(InvalidInput::NonPositiveDefiniteGram).after(work));
        }
        let divide = |entry: Radical| {
            entry
                .div(&determinant_value)
                .map(|(value, work)| (Rational::from_rational_value(value), work))
                .map_err(|failure| DensityFailure::from_algebra(failure, Work::NONE))
        };
        let (i00, fourth) = divide(gram[1][1].radical()).map_err(|failure| failure.after(work))?;
        let work = work + fourth;
        let (i01, fifth) =
            divide(gram[0][1].radical().neg()).map_err(|failure| failure.after(work))?;
        let work = work + fifth;
        let (i11, sixth) = divide(gram[0][0].radical()).map_err(|failure| failure.after(work))?;
        Ok((
            Self {
                gram,
                inverse_gram: [[i00, i01.clone()], [i01, i11]],
                determinant,
                owner_orientation_sign,
            },
            work + sixth,
        ))
    }

    pub fn gram(&self) -> &[[Rational; 2]; 2] {
        &self.gram
    }

    pub fn inverse_gram(&self) -> &[[Rational; 2]; 2] {
        &self.inverse_gram
    }

    pub fn determinant(&self) -> &Rational {
        &self.determinant
    }

    pub fn owner_orientation_sign(&self) -> i8 {
        self.owner_orientation_sign
    }
}
