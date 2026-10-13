//! Exact rational-coefficient arithmetic for the bounded AngleAtom prototype.
//! Полиномы нормализуются только ПОЛОЖИТЕЛЬНЫМ множителем.

use cftuv_radical::arith::{gcd, lcm, magnitude, signed};

pub(crate) use crate::interval::{
    rational_add as add, rational_cmp as compare, rational_mul as mul, rational_neg as neg,
    rational_sign as sign, rational_sub as sub,
};

use crate::{IBig, Rational, UBig};

pub(crate) fn integer(n: i64) -> Rational {
    Rational::integer(IBig::from(n))
}

fn divide(left: &Rational, right: &Rational) -> Rational {
    crate::interval::rational_div(left, right).expect("nonzero leading coefficient")
}

pub(crate) fn midpoint(left: &Rational, right: &Rational) -> Rational {
    mul(
        &add(left, right),
        &Rational::new(IBig::ONE, UBig::from(2u8)).expect("positive denominator"),
    )
}

/// Коэффициенты по возрастанию степени; нулевой polynomial — пустой список.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Polynomial(pub(crate) Vec<Rational>);

impl Polynomial {
    fn trim(mut self) -> Self {
        while self.0.last().is_some_and(|last| sign(last) == 0) {
            self.0.pop();
        }
        self
    }

    /// Убирает denominators/content, но НИКОГДА не меняет знак polynomial.
    fn normalized(self) -> Self {
        let mut polynomial = self.trim();
        if polynomial.0.is_empty() {
            return polynomial;
        }
        let denominator = polynomial
            .0
            .iter()
            .fold(UBig::ONE, |current, entry| lcm(&current, entry.parts().1));
        let integers: Vec<_> = polynomial
            .0
            .iter()
            .map(|entry| entry.parts().0 * signed(&(&denominator / entry.parts().1)))
            .collect();
        let content = integers.iter().fold(UBig::ZERO, |current, entry| {
            gcd(&current, &magnitude(entry))
        });
        debug_assert!(content > UBig::ZERO);
        polynomial.0 = integers
            .into_iter()
            .map(|entry| Rational::integer(entry / &content))
            .collect();
        polynomial
    }

    pub(crate) fn degree(&self) -> usize {
        self.0.len() - 1
    }

    fn derivative(&self) -> Self {
        Self(
            self.0
                .iter()
                .enumerate()
                .skip(1)
                .map(|(power, coefficient)| mul(coefficient, &Rational::integer(IBig::from(power))))
                .collect(),
        )
        .trim()
    }

    fn remainder(&self, divisor: &Self) -> Self {
        let mut remainder = self.clone().trim();
        while !remainder.0.is_empty() && remainder.degree() >= divisor.degree() {
            let shift = remainder.degree() - divisor.degree();
            let factor = divide(remainder.0.last().unwrap(), divisor.0.last().unwrap());
            for (index, coefficient) in divisor.0.iter().enumerate() {
                remainder.0[index + shift] =
                    sub(&remainder.0[index + shift], &mul(&factor, coefficient));
            }
            remainder = remainder.trim();
        }
        remainder
    }

    fn neg(&self) -> Self {
        Self(self.0.iter().map(neg).collect())
    }

    fn scale(&self, factor: &Rational) -> Self {
        Self(
            self.0
                .iter()
                .map(|coefficient| mul(coefficient, factor))
                .collect(),
        )
        .trim()
    }

    fn subtract(&self, other: &Self) -> Self {
        let length = self.0.len().max(other.0.len());
        let mut coefficients = vec![integer(0); length];
        for (index, coefficient) in coefficients.iter_mut().enumerate() {
            *coefficient = sub(
                self.0.get(index).unwrap_or(&integer(0)),
                other.0.get(index).unwrap_or(&integer(0)),
            );
        }
        Self(coefficients).trim()
    }

    fn square(&self) -> Self {
        let mut coefficients = vec![integer(0); 2 * self.degree() + 1];
        for (left_power, left) in self.0.iter().enumerate() {
            for (right_power, right) in self.0.iter().enumerate() {
                let power = left_power + right_power;
                coefficients[power] = add(&coefficients[power], &mul(left, right));
            }
        }
        Self(coefficients).trim()
    }

    pub(crate) fn evaluate(&self, x: &Rational) -> Rational {
        self.0.iter().rev().fold(integer(0), |value, coefficient| {
            add(&mul(&value, x), coefficient)
        })
    }
}

pub(crate) fn chebyshev(n: u8) -> Polynomial {
    let mut previous = Polynomial(vec![integer(1)]);
    if n == 0 {
        return previous;
    }
    let mut current = Polynomial(vec![integer(0), integer(1)]);
    for _ in 1..n {
        let mut shifted = current.scale(&integer(2));
        shifted.0.insert(0, integer(0));
        let next = shifted.subtract(&previous);
        previous = current;
        current = next;
    }
    current
}

pub(crate) fn angle_polynomial(n: u8, r: &Rational) -> Polynomial {
    let t = chebyshev(n);
    if sign(r) == 0 {
        return t;
    }
    let (p, d) = r.parts();
    t.square()
        .scale(&Rational::integer(signed(d)))
        .subtract(&Polynomial(vec![Rational::integer(p.clone())]))
        .normalized()
}

pub(crate) fn sturm(polynomial: &Polynomial) -> Vec<Polynomial> {
    let mut sequence = vec![
        polynomial.clone().normalized(),
        polynomial.derivative().normalized(),
    ];
    while !sequence.last().unwrap().0.is_empty() {
        let length = sequence.len();
        let next = sequence[length - 2]
            .remainder(&sequence[length - 1])
            .neg()
            .normalized();
        if next.0.is_empty() {
            break;
        }
        sequence.push(next);
    }
    sequence
}

/// При нуле polynomial его знак пропускается; at-root V — правая граница.
pub(crate) fn variations(sequence: &[Polynomial], x: &Rational) -> (usize, bool) {
    let mut previous = 0;
    let mut count = 0;
    let mut is_root = false;
    for (index, polynomial) in sequence.iter().enumerate() {
        let next = sign(&polynomial.evaluate(x));
        if index == 0 {
            is_root = next == 0;
        }
        if next == 0 {
            continue;
        }
        if previous != 0 && previous != next {
            count += 1;
        }
        previous = next;
    }
    (count, is_root)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalisation_is_positive_even_when_the_leading_coefficient_is_negative() {
        let polynomial = Polynomial(vec![
            Rational::new(IBig::from(3), UBig::from(2u8)).unwrap(),
            Rational::new(IBig::from(-6), UBig::from(5u8)).unwrap(),
        ]);
        let normalized = polynomial.clone().normalized();
        assert_eq!(normalized.0, vec![integer(5), integer(-4)]);
        for x in [integer(-1), integer(0), integer(1)] {
            assert_eq!(
                sign(&polynomial.evaluate(&x)),
                sign(&normalized.evaluate(&x))
            );
        }
    }

    #[test]
    fn exact_rational_remainders_keep_the_standard_sturm_signs() {
        let polynomial = Polynomial(vec![integer(-1), integer(0), integer(4)]);
        let sequence = sturm(&polynomial);
        assert_eq!(
            sequence,
            vec![
                polynomial,
                Polynomial(vec![integer(0), integer(1)]),
                Polynomial(vec![integer(1)])
            ]
        );
        assert_eq!(variations(&sequence, &integer(-1)), (2, false));
        assert_eq!(variations(&sequence, &integer(1)), (0, false));
        assert_eq!(
            variations(
                &sequence,
                &Rational::new(IBig::ONE, UBig::from(2u8)).unwrap()
            ),
            (0, true)
        );
        assert_eq!(
            variations(
                &sequence,
                &Rational::new(IBig::NEG_ONE, UBig::from(2u8)).unwrap()
            ),
            (1, true)
        );
    }
}
