//! Raw source DAG как density_expression_replay; add/mul строят source expression.
//! Ответ TREE TAB recursive JSON; unsupported возвращается отдельно от product outcome.
use cftuv_density::density_construct::{Constructor, SourceSqrtContract};
use cftuv_density::density_expression::{Expressions, Node, NodeId};
use cftuv_density::Rational;
use std::io::{self, BufRead, Write};

fn tree(graph: &Expressions, id: NodeId) -> String {
    let node = graph.node(id).expect("validated arena");
    let (tag, children) = match node {
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
        children
            .into_iter()
            .map(|id| tree(graph, id))
            .collect::<Vec<_>>()
            .join(",")
    )
}
fn run(context: &mut Constructor<'_>, line: &str) -> Result<String, String> {
    let f: Vec<_> = line.split('\t').collect();
    let id = |s: &str| s.parse::<usize>().map_err(|e| e.to_string());
    let children = |args: &[&str]| args.iter().map(|s| id(s)).collect::<Result<Vec<_>, _>>();
    let node = match f.as_slice() {
        ["source_sqrt", node, n] => {
            let node = id(node)?;
            let result = context.declare_source_sqrt(
                node,
                n.parse().map_err(|e| format!("{e:?}"))?,
                SourceSqrtContract::Sympy114CanonicalPositiveIntegerSqrt,
            );
            return Ok(match result {
                Ok(()) => format!("DECLARED\t{node}"),
                Err(e) => format!("ERROR\t{e:?}"),
            });
        }
        ["fixed_sqrt", n] => {
            let result = context.fixed_sqrt(n.parse().map_err(|e| format!("{e:?}"))?);
            return Ok(match result {
                Ok(root) => format!("TREE\t{}", tree(context.graph(), root)),
                Err(e) => format!("ERROR\t{e:?}"),
            });
        }
        [operation @ ("add" | "mul"), args @ ..] => {
            let args = children(args)?;
            let answer = if *operation == "add" {
                context.add(&args)
            } else {
                context.mul(&args)
            };
            return Ok(match answer {
                Ok(root) => format!("TREE\t{}", tree(context.graph(), root)),
                Err(e) => format!("ERROR\t{e:?}"),
            });
        }
        ["Q", n, d] => Node::Rational(
            Rational::new(
                n.parse().map_err(|e| format!("{e:?}"))?,
                d.parse().map_err(|e| format!("{e:?}"))?,
            )
            .map_err(|e| format!("{e:?}"))?,
        ),
        ["pi"] => Node::Pi,
        ["Add", args @ ..] => Node::Add(children(args)?),
        ["Mul", args @ ..] => Node::Mul(children(args)?),
        ["Pow", a, b] => Node::Pow(id(a)?, id(b)?),
        ["atan2", a, b] => Node::Atan2(id(a)?, id(b)?),
        ["sin", a] => Node::Sin(id(a)?),
        ["cos", a] => Node::Cos(id(a)?),
        ["atan", a] => Node::Atan(id(a)?),
        ["tan", a] => Node::Tan(id(a)?),
        _ => return Err("closed constructor wire".into()),
    };
    Ok(format!(
        "NODE\t{}",
        context.push_raw(node).map_err(|e| format!("{e:?}"))?
    ))
}
fn main() {
    let mut out = io::BufWriter::new(io::stdout().lock());
    let input = io::stdin();
    let mut lines = input.lock().lines();
    loop {
        let mut graph = Expressions::new();
        let mut context = Constructor::new(&mut graph);
        loop {
            let Some(line) = lines.next() else {
                return;
            };
            if matches!(&line,Ok(text) if text=="clear") {
                writeln!(out, "CLEARED").unwrap();
                break;
            }
            let answer = line
                .map_err(|e| e.to_string())
                .and_then(|line| run(&mut context, &line))
                .unwrap_or_else(|e| format!("PARSE_ERROR\t{e}"));
            writeln!(out, "{answer}").unwrap();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn wire_compares_tree_without_node_identity() {
        let mut g = Expressions::new();
        let mut c = Constructor::new(&mut g);
        assert_eq!(run(&mut c, "pi").unwrap(), "NODE\t0");
        assert_eq!(
            run(&mut c, "add\t0\t0").unwrap(),
            "TREE\t[\"Mul\",[\"Q\",\"2\",\"1\"],[\"pi\"]]"
        );
    }
}
