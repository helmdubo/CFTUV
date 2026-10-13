//! Source DAG import; expand TAB id; sign TAB id сначала проверяет source probe.
//! source_sqrt — только caller declaration, не native/source squarefree proof.
use cftuv_density::density_construct::{Constructor, SourceSqrtContract};
use cftuv_density::density_expand::{expand, ExpandLimits};
use cftuv_density::density_expression::{Expressions, Node, NodeId};
use cftuv_density::density_sign::{SignFailure, SourceSignProbe};
use cftuv_density::Rational;
use std::io::{self, BufRead, Write};
fn tree(g: &Expressions, id: NodeId) -> String {
    let (tag, args) = match g.node(id).unwrap() {
        Node::Rational(q) => {
            let (n, d) = q.parts();
            return format!("[\"Q\",\"{n}\",\"{d}\"]");
        }
        Node::Pi => return "[\"pi\"]".into(),
        Node::Add(v) => ("Add", v.clone()),
        Node::Mul(v) => ("Mul", v.clone()),
        Node::Pow(a, b) => ("Pow", vec![*a, *b]),
        Node::Atan2(a, b) => ("atan2", vec![*a, *b]),
        Node::Sin(a) => ("sin", vec![*a]),
        Node::Cos(a) => ("cos", vec![*a]),
        Node::Atan(a) => ("atan", vec![*a]),
        Node::Tan(a) => ("tan", vec![*a]),
    };
    format!(
        "[\"{tag}\",{}]",
        args.into_iter()
            .map(|id| tree(g, id))
            .collect::<Vec<_>>()
            .join(",")
    )
}
fn run(c: &mut Constructor<'_>, line: &str) -> Result<String, String> {
    let f: Vec<_> = line.split('\t').collect();
    let id = |s: &str| s.parse::<usize>().map_err(|e| e.to_string());
    let children = |v: &[&str]| v.iter().map(|s| id(s)).collect::<Result<Vec<_>, _>>();
    let node = match f.as_slice() {
        [op @ ("expand" | "sign"), source] => {
            let source = id(source)?;
            if *op == "sign" {
                match c.probe_sign(source) {
                    Ok(SourceSignProbe::Decided(v)) => {
                        return Ok(format!("SIGN\t{}\tProbe", v.sign));
                    }
                    Ok(SourceSignProbe::NeedsExpanded) => {}
                    Err(e) => return Ok(format!("ERROR\t{e:?}")),
                }
            }
            let expanded = match expand(
                c,
                source,
                ExpandLimits {
                    max_terms: 65536,
                    max_rational_power_bits: 1 << 20,
                    max_polynomial_exponent: 64,
                },
            ) {
                Ok(v) => v,
                Err(e) => return Ok(format!("ERROR\t{e:?}")),
            };
            return Ok(if *op == "expand" {
                format!("TREE\t{}", tree(c.graph(), expanded))
            } else {
                match c.sign_with_expanded_fallback(source, expanded) {
                    Ok(v) => format!("SIGN\t{}\tExpanded", v.sign),
                    Err(SignFailure::Undecidable) => "UNDECIDABLE\tExpanded".into(),
                    Err(e) => format!("ERROR\t{e:?}"),
                }
            });
        }
        ["source_sqrt", node, n] => {
            let node = id(node)?;
            return Ok(
                match c.declare_source_sqrt(
                    node,
                    n.parse().map_err(|e| format!("{e:?}"))?,
                    SourceSqrtContract::Sympy114CanonicalPositiveIntegerSqrt,
                ) {
                    Ok(()) => format!("DECLARED\t{node}"),
                    Err(e) => format!("ERROR\t{e:?}"),
                },
            );
        }
        ["Q", n, d] => Node::Rational(
            Rational::new(
                n.parse().map_err(|e| format!("{e:?}"))?,
                d.parse().map_err(|e| format!("{e:?}"))?,
            )
            .map_err(|e| format!("{e:?}"))?,
        ),
        ["pi"] => Node::Pi,
        ["Add", v @ ..] => Node::Add(children(v)?),
        ["Mul", v @ ..] => Node::Mul(children(v)?),
        ["Pow", a, b] => Node::Pow(id(a)?, id(b)?),
        ["atan2", a, b] => Node::Atan2(id(a)?, id(b)?),
        ["sin", a] => Node::Sin(id(a)?),
        ["cos", a] => Node::Cos(id(a)?),
        ["atan", a] => Node::Atan(id(a)?),
        ["tan", a] => Node::Tan(id(a)?),
        _ => return Err("closed expand wire".into()),
    };
    Ok(format!(
        "NODE\t{}",
        c.push_raw(node).map_err(|e| format!("{e:?}"))?
    ))
}
fn main() {
    let input = io::stdin();
    let mut lines = input.lock().lines();
    let mut out = io::BufWriter::new(io::stdout().lock());
    loop {
        let mut graph = Expressions::new();
        let mut c = Constructor::new(&mut graph);
        loop {
            let Some(line) = lines.next() else {
                return;
            };
            if matches!(&line,Ok(text)if text=="clear") {
                writeln!(out, "CLEARED").unwrap();
                break;
            }
            let answer = line
                .map_err(|e| e.to_string())
                .and_then(|v| run(&mut c, &v))
                .unwrap_or_else(|e| format!("PARSE_ERROR\t{e}"));
            writeln!(out, "{answer}").unwrap();
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn wire_preserves_raw_add_and_nonpolynomial_refusal() {
        let mut g = Expressions::new();
        let mut c = Constructor::new(&mut g);
        assert_eq!(run(&mut c, "Q\t1\t1").unwrap(), "NODE\t0");
        assert_eq!(run(&mut c, "Add\t0\t0").unwrap(), "NODE\t1");
        assert_eq!(
            run(&mut c, "expand\t1").unwrap(),
            "TREE\t[\"Add\",[\"Q\",\"1\",\"1\"],[\"Q\",\"1\",\"1\"]]"
        );
    }
}
