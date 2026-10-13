//! Numeric-only registers: fresh q/wire/sqrt/arithmetic -> SCALAR id + literal JSON.
//! Этот test-only parser не принимает symbolic AST и не вызывает legacy controller.
use cftuv_density::scalar_v2::{LiteralNode, ScalarError, ScalarV2};
use cftuv_density::{IBig, Radical, Rational};
use cftuv_radical::Failure;
use std::io::{self, BufRead, Write};
fn record(value: &ScalarV2) -> String {
    let literal = value.literal_ast();
    let wire = value
        .value()
        .wire()
        .iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join(",");
    let rows = literal
        .nodes()
        .iter()
        .map(|node| match node {
            LiteralNode::Q(n, d) => format!("[\"Q\",{n},{d}]"),
            LiteralNode::Pow { base, exponent } => format!("[\"Pow\",{base},{exponent}]"),
            LiteralNode::Mul { coefficient, root } => format!("[\"Mul\",{coefficient},{root}]"),
            LiteralNode::Add(args) => format!(
                "[\"Add\",{}]",
                args.iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(",")
            ),
        })
        .collect::<Vec<_>>()
        .join(",");
    format!(
        "{{\"canon\":\"{}\",\"v2_wire\":[{wire}],\"nodes\":[{rows}],\"root\":{}}}",
        literal.canon(),
        literal.root()
    )
}
fn run(registers: &mut Vec<ScalarV2>, line: &str) -> Result<String, String> {
    let f: Vec<_> = line.split('\t').collect();
    let reference = |text: &str| {
        let id = text.parse::<usize>().map_err(|e| e.to_string())?;
        registers
            .get(id)
            .ok_or_else(|| format!("invalid scalar register{id}"))
    };
    let rational = |n: &str, d: &str| {
        Rational::new(
            n.parse().map_err(|e| format!("{e:?}"))?,
            d.parse().map_err(|e| format!("{e:?}"))?,
        )
        .map_err(|e| format!("{e:?}"))
    };
    let result = match f.as_slice() {
        ["clear"] => {
            registers.clear();
            return Ok("CLEARED".into());
        }
        ["q", n, d] => Ok(ScalarV2::rational(&rational(n, d)?)),
        ["wire", wire] => {
            let words = wire
                .split(',')
                .filter(|v| !v.is_empty())
                .map(|v| v.parse::<IBig>().map_err(|e| format!("{e:?}")))
                .collect::<Result<Vec<_>, _>>()?;
            Ok(ScalarV2::from_value(
                Radical::from_wire(&words).map_err(|e| format!("{e:?}"))?,
            ))
        }
        ["sqrt", id] => reference(id)?.sqrt(),
        ["inverse", id] => reference(id)?.inverse(),
        ["neg", id] => Ok(reference(id)?.neg()),
        ["pow", id, n] => reference(id)?.integer_power(n.parse().map_err(|e| format!("{e:?}"))?),
        ["scaled", id, n, d] => Ok(reference(id)?.scaled(&rational(n, d)?)),
        ["add", a, b] => Ok(reference(a)?.add(reference(b)?)),
        ["sub", a, b] => Ok(reference(a)?.sub(reference(b)?)),
        ["mul", a, b] => Ok(reference(a)?.mul(reference(b)?)),
        ["div", a, b] => reference(a)?.div(reference(b)?),
        _ => return Err("closed numeric scalar wire".into()),
    };
    Ok(match result {
        Ok(value) => {
            let id = registers.len();
            let observation = record(&value);
            registers.push(value);
            format!("SCALAR\t{id}\t{observation}")
        }
        Err(ScalarError::Profile(v)) => format!("PROFILE\t{}\t{}", v.code.code(), v.detail),
        Err(ScalarError::Algebra(Failure::ZeroDivision)) => "INTERNAL\tZERO_DIVISION".into(),
        Err(ScalarError::ExponentProfile { .. }) => "INTERNAL\tEXPONENT_PROFILE".into(),
        Err(v) => format!("INTERNAL\t{v:?}"),
    })
}
fn main() {
    let mut registers = Vec::new();
    let mut out = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let answer = line
            .map_err(|e| e.to_string())
            .and_then(|line| run(&mut registers, &line))
            .unwrap_or_else(|e| format!("PARSE_ERROR\t{e}"));
        writeln!(out, "{answer}").unwrap();
        out.flush().unwrap(); // Interactive harness читает ответ до следующей команды.
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn canonical_zero_first_ids_and_errors_do_not_append() {
        let mut registers = Vec::new();
        assert_eq!(run(&mut registers,"q\t0\t1").unwrap(),"SCALAR\t0\t{\"canon\":\"DENSITY_SOURCE_CONSTRUCTION_V1\",\"v2_wire\":[],\"nodes\":[[\"Q\",0,1]],\"root\":0}");
        assert_eq!(
            run(&mut registers, "inverse\t0").unwrap(),
            "INTERNAL\tZERO_DIVISION"
        );
        assert_eq!(registers.len(), 1);
        assert!(run(&mut registers, "q\t2\t1")
            .unwrap()
            .starts_with("SCALAR\t1\t"));
        assert!(run(&mut registers, "sqrt\t1")
            .unwrap()
            .contains("[\"Pow\",0,1]"));
    }
}
