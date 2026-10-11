//! The algebra of `RadicalSumV1` alone, replayed against the Python class: `tools/native_effective_alpha_vectors.py algebra` writes random sums over one to four square classes (representatives that are not
//! squarefree, different between the two operands, the order of the operations that made them deciding which one stands for a class) and what `+ - * / neg inverse signum as_rational` give in Python.
//!
//! The field never leaves one square class, so the corpora of the pairs prove none of the multi-class paths -- class merging, the conjugation of an inverse, the sign ladder -- and these vectors do:
//! the TERMS of every result must be the Python tuple, representative and order included, or the refusal must be the same (`E:outside` for more than three classes to invert, `E:zero`).

use cftuv_alpha::{terms_from_wire, Radical, Refused};
use cftuv_core::num::{IBig, UBig};

const VECTORS: &str = include_str!("algebra.txt");

fn wire(text: &str) -> Radical {
    if text == "-" {
        return Radical::zero();
    }
    let numbers: Vec<&str> = text.split(',').collect();
    assert_eq!(numbers.len() % 3, 0, "a whole number of triples: {text}");
    let triples: Vec<(UBig, IBig, UBig)> = numbers
        .chunks(3)
        .map(|triple| (UBig::from_str_radix(triple[0], 10).unwrap(), IBig::from_str_radix(triple[1], 10).unwrap(), UBig::from_str_radix(triple[2], 10).unwrap()))
        .collect();
    Radical::from_terms(terms_from_wire(&triples).unwrap())
}

fn text(value: &Radical) -> String {
    if value.terms().is_empty() {
        return "-".to_string();
    }
    value.terms().iter().map(|(radicand, coefficient)| format!("{radicand},{},{}", coefficient.numerator(), coefficient.denominator())).collect::<Vec<_>>().join(",")
}

fn outcome(result: Result<Radical, Refused>) -> String {
    match result {
        Ok(value) => text(&value),
        Err(Refused::OutsideField) => "E:outside".to_string(),
        Err(Refused::ZeroDivision) => "E:zero".to_string(),
        Err(Refused::SignUndecided) => "E:undecided".to_string(),
    }
}

#[test]
fn every_operation_gives_the_python_terms() {
    let mut counts = std::collections::BTreeMap::<String, usize>::new();
    let (mut multi, mut refused) = (0usize, 0usize);
    for (number, line) in VECTORS.lines().enumerate() {
        let fields: Vec<&str> = line.split('\t').collect();
        assert_eq!(fields.len(), 4, "line {}", number + 1);
        let (name, left, right, expected) = (fields[0], wire(fields[1]), wire(fields[2]), fields[3]);
        let got = match name {
            "add" => text(&left.add(&right)),
            "sub" => text(&left.sub(&right)),
            "mul" => text(&left.mul(&right)),
            "div" => outcome(left.div(&right)),
            "neg" => text(&left.neg()),
            "inverse" => outcome(left.inverse()),
            "signum" => match left.signum() {
                Ok(sign) => sign.to_string(),
                Err(_) => "E:undecided".to_string(),
            },
            "as_rational" => left.as_rational().map_or("-".to_string(), |value| format!("Q:{}/{}", value.numerator(), value.denominator())),
            other => panic!("line {}: unknown operation {other:?}", number + 1),
        };
        assert_eq!(got, expected, "line {}: {name} of {} and {}", number + 1, fields[1], fields[2]);
        *counts.entry(name.to_string()).or_default() += 1;
        multi += (expected.matches(',').count() / 3 >= 2) as usize;
        refused += expected.starts_with("E:") as usize;
    }
    assert!(counts.values().all(|&count| count >= 100), "every operation is asked many times: {counts:?}");
    assert!(multi > 200, "results over several square classes: {multi}");
    assert!(refused >= 10, "the refusals are there too: {refused}");
}
