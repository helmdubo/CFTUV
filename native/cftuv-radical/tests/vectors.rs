//! The oracle's outcomes, replayed: `tools/native_radical_v2_vectors.py write` runs programs on `reference/radical_v2.py` and writes every instruction with the outcome the oracle gave it
//! (values as wires, texts, signs, enclosures, the three work counters, the named refusals with their work and detail); the crate must render the same line for every one.
//!
//! This is the whole of the contract in one place: prices, refusals, texts and wires are compared as text, so a counter that is off by one, a detail that differs by a character or a
//! term in the wrong order fails here with the case, the instruction and both lines.

use cftuv_radical::program::{parse_instruction, render, run};

const VECTORS: &str = include_str!("vectors.txt");

struct Case {
    name: String,
    instructions: Vec<String>,
    outcomes: Vec<String>,
}

fn cases() -> Vec<Case> {
    let mut out: Vec<Case> = Vec::new();
    let mut current: Option<Case> = None;
    for (number, line) in VECTORS.lines().enumerate() {
        if line.starts_with('#') || line.is_empty() {
            continue;
        }
        let (tag, rest) = line.split_at(1);
        match tag {
            "P" => current = Some(Case { name: rest.trim_start().to_string(), instructions: Vec::new(), outcomes: Vec::new() }),
            "I" => current.as_mut().unwrap_or_else(|| panic!("line {}: an instruction outside a case", number + 1)).instructions.push(rest.strip_prefix(' ').unwrap_or(rest).to_string()),
            "O" => current.as_mut().unwrap_or_else(|| panic!("line {}: an outcome outside a case", number + 1)).outcomes.push(rest.strip_prefix(' ').unwrap_or(rest).to_string()),
            "E" => out.push(current.take().unwrap_or_else(|| panic!("line {}: the end of no case", number + 1))),
            other => panic!("line {}: unknown tag {other:?}", number + 1),
        }
    }
    assert!(current.is_none(), "the last case is not closed");
    out
}

#[test]
fn every_instruction_gives_the_outcome_the_oracle_gave() {
    let cases = cases();
    assert!(cases.len() > 200, "the file holds the cases: {}", cases.len());
    let mut instructions = 0usize;
    let mut refusals = 0usize;
    let mut wires = 0usize;
    let mut failures: Vec<String> = Vec::new();
    for case in &cases {
        assert_eq!(case.instructions.len(), case.outcomes.len(), "{}: one outcome per instruction", case.name);
        let program: Vec<_> = case.instructions.iter().map(|line| parse_instruction(line).unwrap_or_else(|why| panic!("{}: {line:?}: {why}", case.name))).collect();
        let done = run(&program);
        for (index, (outcome, expected)) in done.iter().zip(&case.outcomes).enumerate() {
            let got = render(outcome);
            instructions += 1;
            refusals += expected.starts_with("r ") as usize;
            wires += expected.starts_with("v ") as usize;
            if &got != expected && failures.len() < 8 {
                let shown = |text: &str| if text.len() > 300 { format!("{}...({} chars)", &text[..300], text.len()) } else { text.to_string() };
                failures.push(format!("{} instruction {index} {}\n   oracle: {}\n   crate:  {}", case.name, shown(&case.instructions[index]), shown(expected), shown(&got)));
            }
        }
    }
    assert!(failures.is_empty(), "{} of the first differences:\n{}", failures.len(), failures.join("\n"));
    assert!(instructions > 2500 && refusals > 100 && wires > 1000, "the file is a real cover: {instructions} instructions, {refusals} refusals, {wires} values");
}

/// `PRICE_VECTORS` of `kernel/tests/test_radical_sum_v2.py`, copied: the counting rules on fixed inputs (canonical text of the result and the work wire).
const PRICE_VECTORS: [(&str, &str, &str); 8] = [
    ("from_terms", "t Sum(Sqrt(Rational(18, 1)), Sqrt(Rational(27, 1)))", "8,0,0"),
    ("mul", "t Sum(Integer(1), Sqrt(Rational(2, 1)), Sqrt(Rational(3, 1)), Sqrt(Rational(6, 1)))", "4,0,0"),
    ("inverse3", "t Sum(Rational(1, 2), Sqrt(Rational(1, 8)), -Sqrt(Rational(3, 8)))", "3,2,0"),
    (
        "inverse3_norm2",
        "t Sum(Rational(-183554, 9207267), -Sqrt(Rational(4216540160, 84773765609289)), Sqrt(Rational(112494000080, 84773765609289)), Sqrt(Rational(3305475984832, 84773765609289)))",
        "6,4,0",
    ),
    ("div", "t Sum(Rational(-1, 2), Sqrt(Rational(1, 2)), -Sqrt(Rational(28, 1)), Sqrt(Rational(56, 1)), Sqrt(Rational(45, 49)), -Sqrt(Rational(90, 49)))", "12,2,0"),
    ("add_equal_m", "t Sqrt(Rational(8, 1))", "0,0,0"),
    ("sqrt", "t Sqrt(Rational(8, 9))", "1,0,0"),
    ("sqrt_square", "t Rational(2, 3)", "1,0,0"),
];

#[test]
fn the_price_vectors_of_the_kernel_test_are_in_the_file_and_in_the_crate() {
    let cases = cases();
    for (name, text, work) in PRICE_VECTORS {
        let case = cases.iter().find(|case| case.name == format!("price/{name}")).unwrap_or_else(|| panic!("price/{name} is not in the vector file"));
        let n = case.outcomes.len();
        assert_eq!(case.outcomes[n - 1], text, "{name}: the oracle's text");
        assert!(case.outcomes[n - 2].ends_with(&format!(" {work}")), "{name}: the oracle's work {work} in {}", case.outcomes[n - 2]);
        let program: Vec<_> = case.instructions.iter().map(|line| parse_instruction(line).unwrap()).collect();
        let done = run(&program);
        assert_eq!(render(&done[n - 1]), text, "{name}: the crate's text");
        assert!(render(&done[n - 2]).ends_with(&format!(" {work}")), "{name}: the crate's work {work} in {}", render(&done[n - 2]));
    }
}
