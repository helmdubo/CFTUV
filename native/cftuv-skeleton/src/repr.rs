//! Byte-exact Python `repr` of the values the kernel sorts and groups by TEXT.
//!
//! About 65 live sites of the superlevel and symbolic layers order things by `repr()` of nested tuples and dataclasses (`key=repr`,
//! `sorted(..., key=repr)`, the text of a signature row): the order defines vertex ids, twin ids and the push order of the queue, so a port that
//! orders by a "natural" numeric order is wrong (`'10' < '9'`). This module prints exactly what CPython prints for the shapes that occur:
//! `None`, `bool`, `int`, `str` (ASCII), `Fraction`, tuples (with the `(x,)` of a one-tuple), lists, dataclasses (`Name(field=value, ...)`),
//! `str`-mixin enum members (`<Kind.EDGE: 'EDGE'>`), and the three value types `SqrtSumV1`, `EventTimeV1`, `EventPointV1`.
//!
//! A string with a character outside ASCII is a named [`ReprError`]: whether CPython prints such a character or escapes it depends on the Unicode
//! database (`str.isprintable`), which this port does not carry, and the kernel's texts are ASCII.

use cftuv_core::num::IBig;
use cftuv_core::rat::{Coef, Rat};
use cftuv_core::sqrt_sum::SqrtSum;

use crate::time::{EventPoint, EventTime};

/// A value to print.
#[derive(Debug, Clone)]
pub enum Repr<'a> {
    None,
    Bool(bool),
    Int(IBig),
    Str(&'a str),
    Frac(&'a Rat),
    /// A tuple: `()`, `(x,)`, `(x, y)`.
    Tuple(Vec<Repr<'a>>),
    /// A list: `[]`, `[x]`, `[x, y]`.
    List(Vec<Repr<'a>>),
    /// A dataclass instance: the class name and its fields in declaration order (those with `repr=False` left out by the caller).
    Data(&'a str, Vec<(&'a str, Repr<'a>)>),
    /// A member of a `str`-mixin enum: class name, member name, the value.
    Member(&'a str, &'a str, Box<Repr<'a>>),
    Sum(&'a SqrtSum),
    Time(&'a EventTime),
    Point(&'a EventPoint),
}

/// The text contains something the port does not print (see the module note).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReprError(pub String);

/// `repr(value)`.
pub fn py_repr(value: &Repr<'_>) -> Result<String, ReprError> {
    let mut out = String::new();
    write(&mut out, value)?;
    Ok(out)
}

fn write_items<'a>(out: &mut String, open: char, close: char, items: &[Repr<'a>], one_tuple: bool) -> Result<(), ReprError> {
    out.push(open);
    for (index, item) in items.iter().enumerate() {
        if index > 0 {
            out.push_str(", ");
        }
        write(out, item)?;
    }
    if one_tuple && items.len() == 1 {
        out.push(',');
    }
    out.push(close);
    Ok(())
}

/// `repr(str)`: single quotes unless the text holds a single quote and no double quote; `\\`, the quote, `\n`, `\r`, `\t` and the other control
/// characters escaped like CPython (`\xNN`).
pub fn repr_str(text: &str, out: &mut String) -> Result<(), ReprError> {
    if let Some(found) = text.chars().find(|character| !character.is_ascii()) {
        return Err(ReprError(format!("the string {text:?} holds {found:?}, whose repr depends on the Unicode database")));
    }
    let quote = if text.contains('\'') && !text.contains('"') { '"' } else { '\'' };
    out.push(quote);
    for character in text.chars() {
        match character {
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            other if other == quote => {
                out.push('\\');
                out.push(other);
            }
            other if (other as u32) < 0x20 || other as u32 == 0x7f => out.push_str(&format!("\\x{:02x}", other as u32)),
            other => out.push(other),
        }
    }
    out.push(quote);
    Ok(())
}

/// `repr(Fraction(n, d))`.
pub fn repr_frac(value: &Rat, out: &mut String) {
    out.push_str(&format!("Fraction({}, {})", value.numerator(), value.denominator()));
}

fn repr_coef(coef: &Coef, out: &mut String) {
    if coef.is_py_int() {
        out.push_str(&coef.value().numerator().to_string());
    } else {
        repr_frac(coef.value(), out);
    }
}

/// `repr(SqrtSumV1)`: `SqrtSumV1(terms=((1, Fraction(1, 2)), (2, Fraction(3, 1))))`.
pub fn repr_sum(value: &SqrtSum, out: &mut String) {
    out.push_str("SqrtSumV1(terms=(");
    let terms = value.terms();
    for (index, term) in terms.iter().enumerate() {
        if index > 0 {
            out.push_str(", ");
        }
        out.push('(');
        out.push_str(&term.radicand.to_string());
        out.push_str(", ");
        repr_coef(&term.coef, out);
        out.push(')');
    }
    if terms.len() == 1 {
        out.push(',');
    }
    out.push_str("))");
}

pub fn repr_time(value: &EventTime, out: &mut String) {
    out.push_str("EventTimeV1(dividend=");
    repr_frac(&value.dividend, out);
    out.push_str(", divisor=");
    repr_sum(&value.divisor, out);
    out.push(')');
}

pub fn repr_point(value: &EventPoint, out: &mut String) {
    out.push_str("EventPointV1(x=");
    repr_sum(&value.x, out);
    out.push_str(", y=");
    repr_sum(&value.y, out);
    out.push(')');
}

fn write<'a>(out: &mut String, value: &Repr<'a>) -> Result<(), ReprError> {
    match value {
        Repr::None => out.push_str("None"),
        Repr::Bool(flag) => out.push_str(if *flag { "True" } else { "False" }),
        Repr::Int(number) => out.push_str(&number.to_string()),
        Repr::Str(text) => repr_str(text, out)?,
        Repr::Frac(number) => repr_frac(number, out),
        Repr::Tuple(items) => write_items(out, '(', ')', items, true)?,
        Repr::List(items) => write_items(out, '[', ']', items, false)?,
        Repr::Data(name, fields) => {
            out.push_str(name);
            out.push('(');
            for (index, (field, item)) in fields.iter().enumerate() {
                if index > 0 {
                    out.push_str(", ");
                }
                out.push_str(field);
                out.push('=');
                write(out, item)?;
            }
            out.push(')');
        }
        Repr::Member(class, member, inner) => {
            out.push_str(&format!("<{class}.{member}: "));
            write(out, inner)?;
            out.push('>');
        }
        Repr::Sum(sum) => repr_sum(sum, out),
        Repr::Time(time) => repr_time(time, out),
        Repr::Point(point) => repr_point(point, out),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cftuv_core::num::UBig;
    use cftuv_core::sqrt_sum::Term;

    fn text(value: &Repr<'_>) -> String {
        py_repr(value).unwrap()
    }

    #[test]
    fn scalars_and_tuples_print_like_cpython() {
        assert_eq!(text(&Repr::None), "None");
        assert_eq!(text(&Repr::Bool(true)), "True");
        assert_eq!(text(&Repr::Int(IBig::from(-12))), "-12");
        assert_eq!(text(&Repr::Tuple(vec![])), "()");
        assert_eq!(text(&Repr::Tuple(vec![Repr::Int(IBig::from(1))])), "(1,)");
        assert_eq!(text(&Repr::Tuple(vec![Repr::Int(IBig::from(1)), Repr::None])), "(1, None)");
        assert_eq!(text(&Repr::List(vec![Repr::Int(IBig::from(1))])), "[1]");
    }

    #[test]
    fn strings_choose_their_quote_like_cpython() {
        assert_eq!(text(&Repr::Str("EXISTING")), "'EXISTING'");
        assert_eq!(text(&Repr::Str("it's")), "\"it's\"");
        assert_eq!(text(&Repr::Str("a'b\"c")), "'a\\'b\"c'");
        assert_eq!(text(&Repr::Str("a\\b\n")), "'a\\\\b\\n'");
        assert!(py_repr(&Repr::Str("é")).is_err());
    }

    #[test]
    fn fractions_sums_and_times_print_like_the_dataclasses() {
        let half = Rat::new(IBig::from(1), IBig::from(2)).unwrap();
        assert_eq!(text(&Repr::Frac(&Rat::from_int(IBig::from(5)))), "Fraction(5, 1)");
        assert_eq!(text(&Repr::Frac(&Rat::new(IBig::from(-2), IBig::from(14)).unwrap())), "Fraction(-1, 7)");
        let sum = SqrtSum::from_terms(vec![Term { radicand: UBig::from(1u8), coef: Coef::fraction(half.clone()) }, Term { radicand: UBig::from(2u8), coef: Coef::int(IBig::from(3)) }]).unwrap();
        assert_eq!(text(&Repr::Sum(&sum)), "SqrtSumV1(terms=((1, Fraction(1, 2)), (2, 3)))");
        assert_eq!(text(&Repr::Sum(&SqrtSum::zero())), "SqrtSumV1(terms=())");
        let one = SqrtSum::rational(&Rat::one());
        assert_eq!(text(&Repr::Sum(&one)), "SqrtSumV1(terms=((1, Fraction(1, 1)),))");
        let time = EventTime::new(half, one);
        assert_eq!(text(&Repr::Time(&time)), "EventTimeV1(dividend=Fraction(1, 2), divisor=SqrtSumV1(terms=((1, Fraction(1, 1)),)))");
    }

    #[test]
    fn dataclasses_and_enum_members() {
        let key = Repr::Data("JunctionRefV1", vec![("kind", Repr::Str("EXISTING")), ("key", Repr::Tuple(vec![Repr::Int(IBig::from(3))]))]);
        assert_eq!(text(&key), "JunctionRefV1(kind='EXISTING', key=(3,))");
        let member = Repr::Member("EventKind", "EDGE", Box::new(Repr::Str("EDGE")));
        assert_eq!(text(&member), "<EventKind.EDGE: 'EDGE'>");
    }
}
