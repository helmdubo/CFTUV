//! `random.Random(seed)` streams recorded from CPython (see `tools/native_canon_vectors.py`).

mod common;

use cftuv_canon::pyrandom::PyRandom;
use common::*;
use dashu_int::{IBig, UBig};

fn run_op(rng: &mut PyRandom, op: &Json) -> String {
    let parts = op.arr();
    match parts[0].str() {
        "bits" => hx(&rng.getrandbits(parts[1].int() as u32)),
        "below" => hx(&rng.randbelow(&ub(parts[1].str())).unwrap()),
        "range" => hx(&rng.randrange(&ub(parts[1].str()), &ub(parts[2].str())).unwrap()),
        "u32s" => {
            let mut bytes = Vec::new();
            for _ in 0..parts[1].int() {
                bytes.extend_from_slice(&rng.next_u32().to_le_bytes());
            }
            format!("{:x}", fnv1a64(&bytes))
        }
        other => panic!("unknown op {other}"),
    }
}

#[test]
fn every_recorded_stream_is_reproduced() {
    let vectors = load_vectors("pyrandom");
    let mut checked = 0;
    for case in vectors.get("cases").arr() {
        let seed = ib(case.get("seed").str());
        let magnitude = UBig::try_from(if seed < IBig::ZERO { -seed } else { seed }).unwrap();
        let mut rng = PyRandom::from_seed(&magnitude);
        for (op, expected) in case.get("ops").arr().iter().zip(case.get("out").arr()) {
            assert_eq!(run_op(&mut rng, op), expected.str(), "seed {} op {}", case.get("seed").str(), op.text());
            checked += 1;
        }
    }
    assert!(checked > 500, "only {checked} operations were checked");
}
