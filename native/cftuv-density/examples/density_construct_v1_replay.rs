//! Test-only fresh owned operations; JSON integers остаются arbitrary precision.
use cftuv_density::density_construct_v1::OwnedSignErrorV1;
use cftuv_density::density_construct_v1::{
    ConstructErrorV1, ConstructorV1, DecisionNodeV1, OwnedExprV1,
};
use cftuv_density::density_expand_v1::{
    expand, lazy_sign, ExpandErrorV1, ExpandLimits, LazySignErrorV1,
};
use cftuv_density::density_expression::Expressions;
use cftuv_density::density_sign::{SignFailure, SignPath};
use cftuv_density::scalar_v2::{ScalarError, ScalarV2};
use cftuv_density::{IBig, Radical, Rational};
use cftuv_radical::Failure;
use std::io::{self, BufRead, Write};

fn record(c: &ConstructorV1<'_>, value: &OwnedExprV1) -> Result<String, String> {
    let literal = c.export_raw(value).map_err(|e| format!("{e:?}"))?;
    let rows = literal
        .nodes()
        .iter()
        .map(|node| match node {
            DecisionNodeV1::Q(n, d) => format!("[\"Q\",{n},{d}]"),
            DecisionNodeV1::Pi => "[\"pi\"]".into(),
            DecisionNodeV1::Add(args) | DecisionNodeV1::Mul(args) => format!(
                "[\"{}\",{}]",
                if matches!(node, DecisionNodeV1::Add(_)) {
                    "Add"
                } else {
                    "Mul"
                },
                args.iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(",")
            ),
            DecisionNodeV1::Pow(a, b) => format!("[\"Pow\",{a},{b}]"),
            DecisionNodeV1::Sin(a) => format!("[\"sin\",{a}]"),
            DecisionNodeV1::Cos(a) => format!("[\"cos\",{a}]"),
            DecisionNodeV1::Atan2(a, b) => format!("[\"atan2\",{a},{b}]"),
        })
        .collect::<Vec<_>>()
        .join(",");
    let numeric = c
        .numeric_value(value)
        .map_err(|e| format!("{e:?}"))?
        .map_or("null".into(), |value| {
            format!(
                "[{}]",
                value
                    .value()
                    .wire()
                    .iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(",")
            )
        });
    Ok(format!(
        "{{\"canon\":\"{}\",\"nodes\":[{rows}],\"root\":{},\"numeric_v2_wire\":{numeric}}}",
        literal.canon(),
        literal.root()
    ))
}
fn error(error: ConstructErrorV1) -> String {
    match error {
        ConstructErrorV1::Profile(code) => format!("PROFILE\t{}", code.code()),
        ConstructErrorV1::ForeignArena => "PROFILE\tDENSITY_CONSTRUCT_V1_ARENA_MISMATCH".into(),
        ConstructErrorV1::ExponentProfile => "PROFILE\tDENSITY_CONSTRUCT_V1_POWER_PROFILE".into(),
        ConstructErrorV1::Scalar(ScalarError::Profile(v)) => {
            format!("PROFILE\t{}\t{}", v.code.code(), v.detail)
        }
        ConstructErrorV1::Scalar(ScalarError::Algebra(Failure::ZeroDivision)) => {
            "INTERNAL\tZERO_DIVISION".into()
        }
        ConstructErrorV1::Scalar(ScalarError::ExponentProfile { .. }) => {
            "PROFILE\tDENSITY_CONSTRUCT_V1_POWER_PROFILE".into()
        }
        error => format!("INTERNAL\t{error:?}"),
    }
}
fn expand_error(error_value: ExpandErrorV1) -> String {
    match error_value {
        ExpandErrorV1::Profile(code) => format!("PROFILE\t{}\t{code:?}", code.code()),
        ExpandErrorV1::Construction(value) => error(value),
    }
}
fn run(
    c: &mut ConstructorV1<'_>,
    registers: &mut Vec<OwnedExprV1>,
    line: &str,
) -> Result<String, String> {
    let fields: Vec<_> = line.split('\t').collect();
    let reference = |text: &str| {
        let id = text.parse::<usize>().map_err(|e| e.to_string())?;
        registers
            .get(id)
            .cloned()
            .ok_or_else(|| format!("invalid owned register{id}"))
    };
    let rational = |n: &str, d: &str| {
        Rational::new(
            n.parse().map_err(|e| format!("{e:?}"))?,
            d.parse().map_err(|e| format!("{e:?}"))?,
        )
        .map_err(|e| format!("{e:?}"))
    };
    let limits = |terms: &str, exponent: &str, bits: &str| -> Result<ExpandLimits, String> {
        Ok(ExpandLimits {
            max_terms: terms.parse().map_err(|e| format!("{e:?}"))?,
            max_polynomial_exponent: exponent.parse().map_err(|e| format!("{e:?}"))?,
            max_rational_power_bits: bits.parse().map_err(|e| format!("{e:?}"))?,
        })
    };
    if let ["lazy_sign", id, terms, exponent, bits] = fields.as_slice() {
        return Ok(
            match lazy_sign(c, &reference(id)?, limits(terms, exponent, bits)?) {
                Ok(value) => format!(
                    "SIGN\t{}\t{}\t{}",
                    value.result.sign,
                    match value.result.path {
                        SignPath::Rational => "RATIONAL",
                        SignPath::Interval => "INTERVAL",
                        SignPath::Quadratic => "QUADRATIC",
                        SignPath::Memo => "MEMO",
                    },
                    u8::from(value.expanded.is_some())
                ),
                Err(LazySignErrorV1::Expand(value)) => expand_error(value),
                Err(LazySignErrorV1::Sign(OwnedSignErrorV1::Construction(value))) => error(value),
                Err(LazySignErrorV1::Sign(OwnedSignErrorV1::Sign(SignFailure::Undecidable))) => {
                    "LEGACY\tUNDECIDABLE".into()
                }
                Err(value) => format!("INTERNAL\t{value:?}"),
            },
        );
    }
    if let ["expand", id, terms, exponent, bits] = fields.as_slice() {
        return Ok(
            match expand(c, &reference(id)?, limits(terms, exponent, bits)?) {
                Ok(value) => {
                    let observation = record(c, &value.expression)?;
                    let id = registers.len();
                    registers.push(value.expression);
                    format!("EXPR\t{id}\t{observation}")
                }
                Err(value) => expand_error(value),
            },
        );
    }
    let result = match fields.as_slice() {
        ["q", n, d] => c.rational(&rational(n, d)?),
        ["numeric", wire] => {
            let words = wire
                .split(',')
                .filter(|v| !v.is_empty())
                .map(|v| v.parse::<IBig>().map_err(|e| format!("{e:?}")))
                .collect::<Result<Vec<_>, _>>()?;
            c.numeric(ScalarV2::from_value(
                Radical::from_wire(&words).map_err(|e| format!("{e:?}"))?,
            ))
        }
        ["pi"] => c.pi(),
        ["sin", id] => c.raw_sin(&reference(id)?),
        ["cos", id] => c.raw_cos(&reference(id)?),
        ["atan2", y, x] => c.raw_atan2(&reference(y)?, &reference(x)?),
        ["sqrt", id] => c.sqrt(&reference(id)?),
        ["reciprocal", id] => c.formal_reciprocal(&reference(id)?),
        ["partial_div", a, b] => c.partial_div(&reference(a)?, &reference(b)?),
        ["neg", id] => c.neg(&reference(id)?),
        ["scaled", id, n, d] => c.scaled(&reference(id)?, &rational(n, d)?),
        ["pow", id, e] => {
            c.integer_power(&reference(id)?, e.parse().map_err(|e| format!("{e:?}"))?)
        }
        ["raw_pow", id, n, d] => c.raw_opaque_pow(&reference(id)?, &rational(n, d)?),
        ["sub", a, b] => c.sub(&reference(a)?, &reference(b)?),
        ["div", a, b] => c.div(&reference(a)?, &reference(b)?),
        [op, args @ ..] if matches!(*op, "add" | "mul") => {
            let args = args
                .iter()
                .map(|text| reference(text))
                .collect::<Result<Vec<_>, _>>()?;
            if *op == "add" {
                c.add(&args)
            } else {
                c.mul(&args)
            }
        }
        _ => return Err("closed owned construction wire".into()),
    };
    Ok(match result {
        Ok(value) => {
            let observation = record(c, &value)?;
            let id = registers.len();
            registers.push(value);
            format!("EXPR\t{id}\t{observation}")
        }
        Err(e) => error(e),
    })
}
fn main() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let mut registers = Vec::new();
    let mut out = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let answer = match line {
            Ok(line) if line == "clear" => {
                // Drop borrow и замену graph делаем вместе: old handles не переиспользуются.
                registers.clear();
                drop(c);
                graph = Expressions::new();
                c = ConstructorV1::new(&mut graph);
                "CLEARED".into()
            }
            Ok(line) => {
                run(&mut c, &mut registers, &line).unwrap_or_else(|e| format!("PARSE_ERROR\t{e}"))
            }
            Err(e) => format!("PARSE_ERROR\t{e}"),
        };
        writeln!(out, "{answer}").unwrap();
        out.flush().unwrap();
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn transport_integer_json_and_failure_no_append() {
        let mut graph = Expressions::new();
        let mut c = ConstructorV1::new(&mut graph);
        let mut registers = Vec::new();
        assert_eq!(run(&mut c,&mut registers,"q\t0\t1").unwrap(),"EXPR\t0\t{\"canon\":\"DENSITY_SOURCE_CONSTRUCTION_V1\",\"nodes\":[[\"Q\",0,1]],\"root\":0,\"numeric_v2_wire\":[]}");
        assert_eq!(
            run(&mut c, &mut registers, "pow\t0\t-1").unwrap(),
            "INTERNAL\tZERO_DIVISION"
        );
        assert_eq!(registers.len(), 1);
        assert!(run(&mut c, &mut registers, "pi")
            .unwrap()
            .contains("[\"pi\"]"));
        assert_eq!(
            run(&mut c, &mut registers, "pow\t1\t-1").unwrap(),
            "PROFILE\tDENSITY_CONSTRUCT_V1_NEGATIVE_OPAQUE_POWER"
        );
    }
}
