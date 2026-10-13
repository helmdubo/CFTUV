//! Независимый replay: HASH/TUPLE/SET/SORT/CANON, TAB-поля, integer-only.
//! Выводит весь comparator trace, включая вызов, который отказал.

use std::cmp::Ordering;
use std::io::{self, BufRead};

use cftuv_density::farey::{Endpoint, FareyOrdering, FareyPredicates, PrimitiveVector};
use cftuv_density::order::{CanonicalOrdering, sorted_as_cpython311};
use cftuv_density::order_set::{TupleSet, integer_hash, tuple_hash};
use cftuv_density::{IBig, UBig};

fn integers(text: &str) -> Vec<IBig> {
    if text == "-" {
        Vec::new()
    } else {
        text.split(';')
            .map(|value| value.parse().expect("integer"))
            .collect()
    }
}

fn vectors(text: &str) -> Vec<PrimitiveVector> {
    if text == "-" {
        return Vec::new();
    }
    text.split(';')
        .map(|value| {
            let (x, y) = value.split_once(',').expect("pair");
            PrimitiveVector(x.parse().expect("x"), y.parse().expect("y"))
        })
        .collect()
}

fn vector_text(value: &PrimitiveVector) -> String {
    format!("{},{}", value.0, value.1)
}
fn list<T: ToString>(items: impl IntoIterator<Item = T>) -> String {
    let text = items
        .into_iter()
        .map(|value| value.to_string())
        .collect::<Vec<_>>()
        .join(";");
    if text.is_empty() { "-".into() } else { text }
}
fn sign(order: Ordering) -> i8 {
    match order {
        Ordering::Less => -1,
        Ordering::Equal => 0,
        Ordering::Greater => 1,
    }
}
fn trace_text(trace: &[(usize, usize)]) -> String {
    list(trace.iter().map(|(a, b)| format!("{a},{b}")))
}

fn sorted(values: Vec<IBig>, fail_at: usize) -> String {
    let mut log = Vec::new();
    let answer = sorted_as_cpython311(
        values
            .into_iter()
            .enumerate()
            .map(|(at, value)| (value, at))
            .collect(),
        |left, right| {
            log.push((left.1, right.1));
            if fail_at != 0 && log.len() == fail_at {
                Err("ORDER_COMPARE_REFUSED")
            } else {
                Ok(sign(left.0.cmp(&right.0)))
            }
        },
    );
    let (outcome, answer) = match answer {
        Ok(items) => ("OK", list(items.iter().map(|item| item.1))),
        Err(error) => (error, "-".into()),
    };
    format!("{outcome}\t{answer}\t{}", trace_text(&log))
}

struct Predicates {
    input: Vec<PrimitiveVector>,
    fail_at: usize,
    trace: Vec<(usize, usize)>,
}

impl FareyPredicates for Predicates {
    type Error = &'static str;
    fn local_candidate(&mut self, _: usize, _: &PrimitiveVector) -> Result<bool, Self::Error> {
        unreachable!("ordering only")
    }
    fn edge_valid(
        &mut self,
        _: usize,
        _: Endpoint<'_>,
        _: Endpoint<'_>,
    ) -> Result<bool, Self::Error> {
        unreachable!("ordering only")
    }
    fn candidate_compare(
        &mut self,
        ordinal: usize,
        left: &PrimitiveVector,
        right: &PrimitiveVector,
    ) -> Result<i8, Self::Error> {
        assert_eq!(ordinal, 1);
        let left_at = self.input.iter().position(|item| item == left).unwrap();
        let right_at = self.input.iter().position(|item| item == right).unwrap();
        self.trace.push((left_at, right_at));
        if self.fail_at != 0 && self.trace.len() == self.fail_at {
            return Err("ORDER_COMPARE_REFUSED");
        }
        let rank = |value: &IBig| {
            let result = value % UBig::from(7u8);
            if result < IBig::ZERO {
                result + 7u8
            } else {
                result
            }
        };
        Ok(sign(rank(&left.0).cmp(&rank(&right.0))))
    }
}

fn canonical(input: Vec<PrimitiveVector>, fail_at: usize) -> String {
    let mut ordering = CanonicalOrdering;
    let indices =
        <CanonicalOrdering as FareyOrdering<Predicates>>::iteration_order(&mut ordering, 1, &input)
            .unwrap();
    let items: Vec<_> = indices.iter().map(|index| input[*index].clone()).collect();
    let mut predicates = Predicates {
        input,
        fail_at,
        trace: Vec::new(),
    };
    let result = ordering.sort(1, &items, &mut predicates);
    let (outcome, result) = match result {
        Ok(sorted) => ("OK", list(sorted.iter().map(|at| indices[*at]))),
        Err(error) => (error, "-".into()),
    };
    format!(
        "{}\t{outcome}\t{result}\t{}",
        list(indices),
        trace_text(&predicates.trace)
    )
}

fn run(fields: &[&str]) -> String {
    match fields[0] {
        "HASH" => list(integers(fields[2]).iter().map(integer_hash)),
        "TUPLE" => list(vectors(fields[2]).iter().map(tuple_hash)),
        "SET" => {
            let mut set = TupleSet::new();
            let mut phases = Vec::new();
            for phase in fields[2].split('|') {
                set.update(vectors(phase).iter());
                phases.push(format!(
                    "{}\t{}",
                    list(set.iter().map(vector_text)),
                    set.capacity()
                ));
            }
            phases.join("|")
        }
        "SORT" => sorted(integers(fields[3]), fields[2].parse().expect("fail at")),
        "CANON" => canonical(vectors(fields[3]), fields[2].parse().expect("fail at")),
        op => panic!("unknown operation {op}"),
    }
}

fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.expect("stdin");
        if line.is_empty() {
            continue;
        }
        let fields: Vec<_> = line.split('\t').collect();
        println!("{}\t{}\t{}", fields[0], fields[1], run(&fields));
    }
}
