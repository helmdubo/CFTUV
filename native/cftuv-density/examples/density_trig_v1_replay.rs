//! Test-only finite trig replay: pi p/d; q density; ray ratio sign n j (TAB).
use cftuv_density::density_construct_v1::{ConstructorV1, DecisionNodeV1, OwnedExprV1};
use cftuv_density::density_expand_v1::{lazy_sign, ExpandLimits, LazySignErrorV1};
use cftuv_density::density_expression::Expressions;
use cftuv_density::density_sign::SignFailure;
use cftuv_density::density_trig_v1::*;
use cftuv_density::{IBig, Radical, Rational, UBig};
use std::io::{self, BufRead, Write};

fn rational(text: &str) -> Result<Rational, String> {
    let (n, d) = text.split_once('/').ok_or("expected n/d")?;
    Rational::new(
        n.parse::<IBig>().map_err(|e| format!("{e:?}"))?,
        d.parse::<UBig>().map_err(|e| format!("{e:?}"))?,
    )
    .map_err(|e| format!("{e:?}"))
}
fn record(c: &mut ConstructorV1<'_>, value: &OwnedExprV1) -> Result<String, String> {
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
        .map_or("null".into(), |v| {
            format!(
                "[{}]",
                v.value()
                    .wire()
                    .iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(",")
            )
        });
    let sign = match lazy_sign(
        c,
        value,
        ExpandLimits {
            max_terms: 4096,
            max_polynomial_exponent: 4,
            max_rational_power_bits: 65536,
        },
    ) {
        Ok(v) => v.result.sign.to_string(),
        Err(LazySignErrorV1::Sign(
            cftuv_density::density_construct_v1::OwnedSignErrorV1::Sign(SignFailure::Undecidable),
        )) => "\"UNDECIDABLE\"".into(),
        Err(e) => return Err(format!("sign {e:?}")),
    };
    Ok(format!("{{\"canon\":\"{}\",\"nodes\":[{rows}],\"root\":{},\"numeric_v2_wire\":{numeric},\"sign\":{sign}}}",literal.canon(),literal.root()))
}
fn run(line: &str) -> Result<String, String> {
    let fields: Vec<_> = line.split('\t').collect();
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let pair = match fields.as_slice() {
        ["pi", value] => owned_rational_pi_trig(&mut c, &rational(value)?).map(Some),
        ["axis", value, ..] if fields.len() == 6 => {
            owned_rational_pi_trig(&mut c, &rational(value)?).map(Some)
        }
        ["q", q] => owned_exact_q_trig(&mut c, q.parse().map_err(|e| format!("{e:?}"))?).map(Some),
        ["ray", ratio, sign, n, j] => owned_principal_ray(
            &mut c,
            &rational(ratio)?,
            sign.parse().map_err(|e| format!("{e:?}"))?,
            n.parse().map_err(|e| format!("{e:?}"))?,
            j.parse().map_err(|e| format!("{e:?}"))?,
        ),
        _ => return Err("invalid operation/fields".into()),
    };
    match pair {
        Err(error) => Ok(format!("{{\"error\":\"{error:?}\"}}")),
        Ok(None) => Ok("{\"not_known\":true}".into()),
        Ok(Some(pair)) => {
            let a = record(&mut c, &pair[0])?;
            let b = record(&mut c, &pair[1])?;
            let aa = c.integer_power(&pair[0], 2).map_err(|e| format!("{e:?}"))?;
            let bb = c.integer_power(&pair[1], 2).map_err(|e| format!("{e:?}"))?;
            let sum = c.add(&[aa, bb]).map_err(|e| format!("{e:?}"))?;
            let one = c
                .rational(&Rational::integer(IBig::ONE))
                .map_err(|e| format!("{e:?}"))?;
            let residual = c.sub(&sum, &one).map_err(|e| format!("{e:?}"))?;
            let norm = record(&mut c, &residual)?;
            let mut axes = String::new();
            if fields[0] == "axis" {
                let mut basis = Vec::new();
                for text in &fields[2..] {
                    let wire = text
                        .split(',')
                        .filter(|v| !v.is_empty())
                        .map(|v| v.parse::<IBig>().map_err(|e| format!("{e:?}")))
                        .collect::<Result<Vec<_>, _>>()?;
                    let scalar = cftuv_density::scalar_v2::ScalarV2::from_value(
                        Radical::from_wire(&wire).map_err(|e| format!("{e:?}"))?,
                    );
                    basis.push(c.numeric(scalar).map_err(|e| format!("{e:?}"))?);
                }
                let mut result = Vec::new();
                for index in 0..2 {
                    let left = c
                        .mul(&[pair[0].clone(), basis[index].clone()])
                        .map_err(|e| format!("{e:?}"))?;
                    let right = c
                        .mul(&[pair[1].clone(), basis[index + 2].clone()])
                        .map_err(|e| format!("{e:?}"))?;
                    let value = c.add(&[left, right]).map_err(|e| format!("{e:?}"))?;
                    result.push(record(&mut c, &value)?);
                }
                axes = format!(",\"axes\":[{}]", result.join(","));
            }
            Ok(format!(
                "{{\"cosine\":{a},\"sine\":{b},\"unit_residual\":{norm}{axes}}}"
            ))
        }
    }
}
fn main() -> io::Result<()> {
    let mut output = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let line = line?;
        if line.is_empty() {
            continue;
        }
        match run(&line) {
            Ok(value) => writeln!(output, "{value}")?,
            Err(e) => writeln!(output, "READ_ERROR\t{e}")?,
        }
    }
    Ok(())
}
