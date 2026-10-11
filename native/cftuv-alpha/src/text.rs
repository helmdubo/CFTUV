//! The text of a rational `ExactScalar` (`native_exact._rational_text` and the two rational shapes of `native_of_text`).
//!
//! `parse_rational_text` reads exactly what `_INTEGER_TEXT` (`\AInteger\((-?\d+)\)\Z`) and `_RATIONAL_TEXT` (`\ARational\((-?\d+), (-?\d+)\)\Z`) read, and nothing else: any other text (a
//! root, a legacy sympy form, a `Float`, a zero denominator, non-ASCII digits that `\d` would also accept) is `None`, and the pair is the oracle's.

use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::rat::Rat;

fn parse_integer(text: &str) -> Option<IBig> {
    let (negative, digits) = match text.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, text),
    };
    if digits.is_empty() || !digits.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }
    let magnitude = UBig::from_str_radix(digits, 10).ok()?;
    Some(num::from_sign_magnitude(negative, magnitude))
}

/// `Fraction` of an `Integer(n)` / `Rational(p, q)` text; `None` for every other text.
pub fn parse_rational_text(text: &str) -> Option<Rat> {
    if let Some(inner) = text.strip_prefix("Integer(").and_then(|rest| rest.strip_suffix(')')) {
        return Some(Rat::from_int(parse_integer(inner)?));
    }
    let inner = text.strip_prefix("Rational(").and_then(|rest| rest.strip_suffix(')'))?;
    let (numerator, denominator) = inner.split_once(", ")?;
    Rat::new(parse_integer(numerator)?, parse_integer(denominator)?).ok()
}

/// `_rational_text(value)`: `Integer(n)` for a whole number, `Rational(p, q)` otherwise.
pub fn rational_text(value: &Rat) -> String {
    if value.denominator().is_one() {
        format!("Integer({})", value.numerator())
    } else {
        format!("Rational({}, {})", value.numerator(), value.denominator())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_two_rational_shapes_and_only_they_parse() {
        assert_eq!(parse_rational_text("Integer(-12)"), Some(Rat::from_i64(-12)));
        assert_eq!(parse_rational_text("Integer(0)"), Some(Rat::zero()));
        assert_eq!(parse_rational_text("Integer(-0)"), Some(Rat::zero()));
        assert_eq!(parse_rational_text("Rational(-1, 3)"), Some(Rat::new(IBig::from(-1), IBig::from(3)).unwrap()));
        // `Fraction(p, q)` normalises sign and common factors
        assert_eq!(parse_rational_text("Rational(4, -6)"), Some(Rat::new(IBig::from(-2), IBig::from(3)).unwrap()));
        for text in [
            "",
            "Integer()",
            "Integer(+3)",
            "Integer( 3)",
            "Integer(3) ",
            "Integer(3)\n",
            "Rational(1,3)",
            "Rational(1, 0)",
            "Rational(1, 3, 4)",
            "Sqrt(Rational(2, 1))",
            "Pow(Integer(2), Rational(1, 2))",
            "Float('1.5', precision=53)",
            "Integer(\u{0663})",
        ] {
            assert_eq!(parse_rational_text(text), None, "{text:?}");
        }
    }

    #[test]
    fn the_text_of_a_rational() {
        assert_eq!(rational_text(&Rat::from_i64(7)), "Integer(7)");
        assert_eq!(rational_text(&Rat::zero()), "Integer(0)");
        assert_eq!(rational_text(&Rat::new(IBig::from(-3), IBig::from(6)).unwrap()), "Rational(-1, 2)");
    }

    #[test]
    fn a_big_integer_round_trips() {
        let text = "Integer(-123456789012345678901234567890123456789012345678901234567890)";
        assert_eq!(rational_text(&parse_rational_text(text).unwrap()), text);
    }
}
