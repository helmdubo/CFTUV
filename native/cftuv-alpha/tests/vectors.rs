//! The vectors of `tools/native_effective_alpha_vectors.py`: pairs the Python oracle answered, replayed with the Rust arithmetic alone.
//!
//! Every pair of the file must come out as the oracle's: the counter delta, the contacts in order, the terms of alpha and station (the square-class representatives and their order are part of the
//! answer), the two texts of the point. A frame is the data `effective_alpha_op` hands the extension; the pairs asked against it follow it in the file.

use cftuv_alpha::{terms_from_wire, Answer, Entry, Frame, Radical};
use cftuv_core::num::{IBig, UBig};

const VECTORS: &str = include_str!("vectors.txt");

fn wire(text: &str) -> Vec<(UBig, IBig, UBig)> {
    if text == "-" {
        return Vec::new();
    }
    let numbers: Vec<&str> = text.split(',').collect();
    assert_eq!(numbers.len() % 3, 0, "a whole number of triples: {text}");
    numbers
        .chunks(3)
        .map(|triple| (UBig::from_str_radix(triple[0], 10).unwrap(), IBig::from_str_radix(triple[1], 10).unwrap(), UBig::from_str_radix(triple[2], 10).unwrap()))
        .collect()
}

fn terms(text: &str) -> cftuv_alpha::Terms {
    terms_from_wire(&wire(text)).unwrap()
}

fn floats(text: &str) -> Option<[Entry; 7]> {
    if text == "-" {
        return None;
    }
    let bits: Vec<f64> = text.split(',').map(|item| f64::from_bits(u64::from_str_radix(item, 16).unwrap())).collect();
    assert_eq!(bits.len(), 14);
    let mut entries = [(0.0, 0.0); 7];
    for (index, entry) in entries.iter_mut().enumerate() {
        *entry = (bits[2 * index], bits[2 * index + 1]);
    }
    Some(entries)
}

/// The oracle's wire of a value: `r,n,d,...` with `-` for the empty sum.
fn wire_of(value: &Radical) -> String {
    if value.terms().is_empty() {
        return "-".to_string();
    }
    value.terms().iter().map(|(radicand, coefficient)| format!("{radicand},{},{}", coefficient.numerator(), coefficient.denominator())).collect::<Vec<_>>().join(",")
}

#[test]
fn every_vector_equals_the_oracle() {
    let mut frames: Vec<Option<Frame>> = Vec::new();
    let (mut pairs, mut contacts_seen, mut rejected_seen, mut several, mut irrational) = (0usize, 0usize, 0usize, 0usize, 0usize);
    for (number, line) in VECTORS.lines().enumerate() {
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let fields: Vec<&str> = line.split('\t').collect();
        match fields[0] {
            "F" => {
                assert_eq!(fields.len(), 10, "line {}", number + 1);
                let index: usize = fields[1].parse().unwrap();
                assert_eq!(index, frames.len(), "frames are numbered in order");
                frames.push(Some(Frame::new(Some((fields[2], fields[3])), terms(fields[5]), (terms(fields[6]), terms(fields[7])), (terms(fields[8]), terms(fields[9])), floats(fields[4]))));
            }
            "P" => {
                assert_eq!(fields.len(), 9, "line {}", number + 1);
                let frame = frames[fields[1].parse::<usize>().unwrap()].as_ref().unwrap();
                let (rejected, passed): (u32, u32) = (fields[6].parse().unwrap(), fields[7].parse().unwrap());
                let expected: Vec<Vec<&str>> = if fields[8] == "-" { Vec::new() } else { fields[8].split(" | ").map(|contact| contact.split(';').collect()).collect() };
                match frame.pair(fields[2], fields[3], fields[4], fields[5]) {
                    Answer::Contacts { rejected: got_rejected, passed: got_passed, contacts } => {
                        assert_eq!((got_rejected, got_passed), (rejected, passed), "line {}: the counter delta", number + 1);
                        assert_eq!(contacts.len(), expected.len(), "line {}: the number of contacts", number + 1);
                        for (contact, want) in contacts.iter().zip(&expected) {
                            assert_eq!(wire_of(&contact.alpha), want[0], "line {}: alpha", number + 1);
                            assert_eq!(wire_of(&contact.station), want[1], "line {}: station", number + 1);
                            assert_eq!((contact.x.as_str(), contact.y.as_str()), (want[2], want[3]), "line {}: the point", number + 1);
                            irrational += contact.alpha.terms().iter().any(|(radicand, _)| !radicand.is_one()) as usize;
                        }
                        contacts_seen += contacts.len();
                        rejected_seen += rejected as usize;
                        several += (contacts.len() > 1) as usize;
                    }
                    Answer::Declined(reason) => panic!("line {}: the oracle answered the pair and the crate declined it: {reason}", number + 1),
                }
                pairs += 1;
            }
            other => panic!("line {}: unknown record {other:?}", number + 1),
        }
    }
    // the file must reach every corner of the answer, or a green run says little
    assert!(pairs >= 100 && contacts_seen > 100 && rejected_seen > 5 && several > 10 && irrational > 10, "{pairs} pairs, {contacts_seen} contacts, {rejected_seen} rejected, {several} with several, {irrational} irrational");
}
