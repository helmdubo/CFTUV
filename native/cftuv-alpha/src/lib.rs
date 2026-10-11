//! The contact leaf of the effective-alpha resolver (`kernel/src/cftuv_envelope/reference/boundary.py::_native_contacts`: the prefilter `SourceContactFrame.excludes` and
//! `boundary_native.contact_candidates_native`), bit-equal to the Python oracle, including the counts it moves.
//!
//! The unit is ONE pair: a source segment (its frame, built once per source) against one barrier segment. The answer is the ordered contacts `(alpha, station, point)` with the terms of
//! `alpha` and `station` exactly as the oracle's `RadicalSumV1` holds them (square-class representatives and term order included: see `radical.rs`) and the point as the two texts the oracle
//! writes, plus the delta of `contact_candidates.prefilter_rejected` / `prefilter_passed`.
//!
//! What the port does NOT answer it DECLINES, by name, touching nothing (the oracle computes the pair, with its own counts and its own sympy fallback): a point that is not an
//! `Integer(n)` / `Rational(p, q)` text (an irrational end of the segment, the start of the source), an operation of the algebra that refuses (`NativeExactError`), a contact whose parameter
//! is irrational (its point is written through sympy), a frame whose own numbers are not valid. A refusal of the oracle's native path is therefore never produced here: it happens in Python
//! when the oracle replays the pair.

mod contact;
mod prefilter;
pub mod radical;
mod text;

use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;

pub use contact::{Contact, Decline};
pub use prefilter::{excludes, rational_entry, Entry};
pub use radical::{Radical, Refused, Terms};
pub use text::{parse_rational_text, rational_text};

/// The data of a source segment that the leaf reads, converted once per source: the start as a rational (when it is one), the metric length of the segment, the covectors `G·t` and `G·n`
/// (as the oracle holds them) and the seven binary64 `(centre, bound)` pairs of the prefilter (`None`: the oracle's `_float_parts()` is `False`, there is no prefilter).
#[derive(Debug, Clone)]
pub struct Frame {
    origin: Option<(Rat, Rat)>,
    length: Radical,
    tangent: (Radical, Radical),
    normal: (Radical, Radical),
    floats: Option<[Entry; 7]>,
}

/// What the leaf answers for one pair.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Answer {
    /// `counts.rejected += rejected`, `counts.passed += passed`, the contacts in the order of the oracle.
    Contacts { rejected: u32, passed: u32, contacts: Vec<Contact> },
    /// The oracle computes this pair.
    Declined(&'static str),
}

/// A frame the port does not carry (a radicand that is not positive, a denominator that is not positive).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InvalidFrame(pub &'static str);

/// The flat wire of a `RadicalSumV1`: `(radicand, numerator, denominator)` triples, `terms_wire`'s format, decoded to the Rust integers.
pub fn terms_from_wire(wire: &[(UBig, IBig, UBig)]) -> Result<Terms, InvalidFrame> {
    let mut terms = Vec::with_capacity(wire.len());
    for (radicand, numerator, denominator) in wire {
        if radicand.is_zero() {
            return Err(InvalidFrame("a term with the radicand zero"));
        }
        if denominator.is_zero() {
            return Err(InvalidFrame("a coefficient with the denominator zero"));
        }
        terms.push((radicand.clone(), Rat::reduced(numerator.clone(), denominator.clone())));
    }
    Ok(terms)
}

impl Frame {
    /// `start`: the two texts of the source start (`None`: the start is not a rational text, every pair of the frame is the oracle's).
    pub fn new(start: Option<(&str, &str)>, length: Terms, tangent: (Terms, Terms), normal: (Terms, Terms), floats: Option<[Entry; 7]>) -> Frame {
        let origin = start.and_then(|(x, y)| Some((parse_rational_text(x)?, parse_rational_text(y)?)));
        Frame {
            origin,
            length: Radical::from_terms(length),
            tangent: (Radical::from_terms(tangent.0), Radical::from_terms(tangent.1)),
            normal: (Radical::from_terms(normal.0), Radical::from_terms(normal.1)),
            floats,
        }
    }

    /// The pair `boundary_start -> boundary_end` (the four texts of the segment's points).
    pub fn pair(&self, start_x: &str, start_y: &str, end_x: &str, end_y: &str) -> Answer {
        let Some(origin) = &self.origin else {
            return Answer::Declined("the source start is not a rational text");
        };
        let points = (
            parse_rational_text(start_x).zip(parse_rational_text(start_y)),
            parse_rational_text(end_x).zip(parse_rational_text(end_y)),
        );
        let (Some(start), Some(end)) = points else {
            return Answer::Declined("a barrier point is not a rational text");
        };
        if let Some(floats) = &self.floats {
            if excludes(floats, [&start.0, &start.1], [&end.0, &end.1]) {
                return Answer::Contacts { rejected: 1, passed: 0, contacts: Vec::new() };
            }
        }
        let frame = contact::LeafFrame { origin, length: &self.length, tangent: &self.tangent, normal: &self.normal };
        match contact::contacts(&frame, &start, &end) {
            Ok(contacts) => Answer::Contacts { rejected: 0, passed: 1, contacts },
            Err(decline) => Answer::Declined(decline.reason()),
        }
    }
}
