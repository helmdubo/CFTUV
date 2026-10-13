//! Дифференциальный stdin: nodes в source-порядке, независимый query interval.
use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::density_quadratic::{QuadraticFailureKind, QuadraticReader};
use cftuv_density::legacy_mpf::{Mpf, Precision};
use cftuv_density::Rational;
use std::io::{self, BufRead, Write};
fn raw(v: &Mpf) -> String {
    let (s, m, e, b) = v.raw_tuple();
    format!("{s},{m},{e},{b}")
}
fn tree(graph: &Expressions, id: usize) -> String {
    let (name, children) = match graph.node(id).unwrap() {
        Node::Rational(value) => {
            let (n, d) = value.parts();
            return format!("Q({n},{d})");
        }
        Node::Pi => return "pi".into(),
        Node::Add(args) => ("Add", args.clone()),
        Node::Mul(args) => ("Mul", args.clone()),
        Node::Pow(a, b) => ("Pow", vec![*a, *b]),
        Node::Atan2(a, b) => ("atan2", vec![*a, *b]),
        Node::Sin(a) => ("sin", vec![*a]),
        Node::Cos(a) => ("cos", vec![*a]),
        Node::Atan(a) => ("atan", vec![*a]),
        Node::Tan(a) => ("tan", vec![*a]),
    };
    format!(
        "{name}({})",
        children
            .iter()
            .map(|&child| tree(graph, child))
            .collect::<Vec<_>>()
            .join(",")
    )
}
fn run(graph: &mut Expressions, line: &str) -> Result<String, String> {
    let fields: Vec<_> = line.split('\t').collect();
    let id = |s: &str| s.parse::<usize>().map_err(|e| e.to_string());
    let node = match fields.as_slice() {
        ["vector_objective", a, b, c, d, ix, iy, x, y] => {
            let matrix = [[id(a)?, id(b)?], [id(c)?, id(d)?]];
            let ideal = [id(ix)?, id(iy)?];
            let candidate = (
                x.parse().map_err(|e| format!("{e:?}"))?,
                y.parse().map_err(|e| format!("{e:?}"))?,
            );
            let mut constructor = cftuv_density::density_construct::Constructor::new(graph);
            for root in [2, 3, 5, 6] {
                constructor.fixed_sqrt(root).map_err(|e| format!("{e:?}"))?;
            }
            let result = cftuv_density::density_vector_nodes::objective(
                &mut constructor,
                matrix,
                ideal,
                &candidate,
            );
            return Ok(match result {
                Ok(node) => format!("VECTOR_AST\t{}", tree(constructor.graph(), node)),
                Err(e) => format!("CONSTRUCT_ERROR\t{e:?}"),
            });
        }
        ["matrix_vector", a, b, c, d, x, y] => {
            let matrix = [[id(a)?, id(b)?], [id(c)?, id(d)?]];
            let vector = [id(x)?, id(y)?];
            let mut constructor = cftuv_density::density_construct::Constructor::new(graph);
            for root in [2, 3, 5, 6] {
                constructor.fixed_sqrt(root).map_err(|e| format!("{e:?}"))?;
            }
            let result = cftuv_density::density_vector_nodes::matrix_vector(
                &mut constructor,
                matrix,
                vector,
            );
            return Ok(match result {
                Ok([x, y]) => format!(
                    "VECTOR_AST\t{}\t{}",
                    tree(constructor.graph(), x),
                    tree(constructor.graph(), y)
                ),
                Err(e) => format!("CONSTRUCT_ERROR\t{e:?}"),
            });
        }
        ["vector_dot", a, b, c, d, lx, ly, rx, ry] => {
            let matrix = [[id(a)?, id(b)?], [id(c)?, id(d)?]];
            let left = [id(lx)?, id(ly)?];
            let right = [id(rx)?, id(ry)?];
            let mut constructor = cftuv_density::density_construct::Constructor::new(graph);
            for root in [2, 3, 5, 6] {
                constructor.fixed_sqrt(root).map_err(|e| format!("{e:?}"))?;
            }
            let result =
                cftuv_density::density_vector_nodes::dot(&mut constructor, matrix, left, right);
            return Ok(match result {
                Ok(node) => format!("VECTOR_AST\t{}", tree(constructor.graph(), node)),
                Err(e) => format!("CONSTRUCT_ERROR\t{e:?}"),
            });
        }
        ["vector_cross", lx, ly, rx, ry, orientation] => {
            let left = [id(lx)?, id(ly)?];
            let right = [id(rx)?, id(ry)?];
            let orientation = orientation.parse().map_err(|e| format!("{e:?}"))?;
            let mut constructor = cftuv_density::density_construct::Constructor::new(graph);
            for root in [2, 3, 5, 6] {
                constructor.fixed_sqrt(root).map_err(|e| format!("{e:?}"))?;
            }
            let result = cftuv_density::density_vector_nodes::oriented_cross(
                &mut constructor,
                left,
                right,
                orientation,
            );
            return Ok(match result {
                Ok(node) => format!("VECTOR_AST\t{}", tree(constructor.graph(), node)),
                Err(e) => format!("CONSTRUCT_ERROR\t{e:?}"),
            });
        }
        ["q_trig", q] => {
            return Ok(
                match cftuv_density::density_trig_nodes::exact_q_trig(graph, id(q)?) {
                    Ok([c, s]) => format!("Q_TRIG\t{}\t{}", tree(graph, c), tree(graph, s)),
                    Err(
                        cftuv_density::density_trig_nodes::TrigNodeFailure::UnsupportedDensityQ(_),
                    ) => "Q_TRIG_UNSUPPORTED".into(),
                    Err(error) => format!("ERROR\t{error:?}"),
                },
            );
        }
        ["residual_filter", dot, left, right, q] => {
            return Ok(
                match cftuv_density::density_residual::filtered_sign(
                    graph,
                    id(dot)?,
                    id(left)?,
                    id(right)?,
                    id(q)?,
                ) {
                    Ok(Some(sign)) => format!("FILTER\t{sign}"),
                    Ok(None) => "FILTER\tNONE".into(),
                    Err(error) => format!("ERROR\t{error:?}"),
                },
            );
        }
        ["compare", left, right] => {
            let result =
                cftuv_density::density_expression_order::compare(graph, id(left)?, id(right)?)
                    .map_err(|e| format!("{e:?}"))?;
            return Ok(format!(
                "CMP\t{}",
                match result {
                    std::cmp::Ordering::Less => -1,
                    std::cmp::Ordering::Equal => 0,
                    std::cmp::Ordering::Greater => 1,
                }
            ));
        }
        ["sign", source, expanded] => {
            return Ok(
                match graph.sign_with_expanded_fallback(id(source)?, id(expanded)?) {
                    Ok(v) => format!("SIGN\t{}", v.sign),
                    Err(cftuv_density::density_sign::SignFailure::Undecidable) => {
                        "UNDECIDABLE".into()
                    }
                    Err(error) => format!("ERROR\t{error:?}"),
                },
            );
        }
        ["clear"] => {
            *graph = Expressions::new();
            return Ok("CLEARED".into());
        }
        ["exact", node] => {
            return Ok(match QuadraticReader::new(graph).read(id(node)?) {
                Ok((value, _)) => format!(
                    "RADICAL\t{}",
                    value
                        .wire()
                        .iter()
                        .map(ToString::to_string)
                        .collect::<Vec<_>>()
                        .join(",")
                ),
                Err(error) => match error.kind {
                    QuadraticFailureKind::UnsupportedSyntax(_) => "UNSUPPORTED".into(),
                    _ => format!("ERROR\t{error:?}"),
                },
            });
        }
        ["eval", p, node] => {
            let p = Precision::new(p.parse().map_err(|e| format!("{e:?}"))?)
                .map_err(|e| format!("{e:?}"))?;
            return Ok(match graph.interval(id(node)?, p) {
                Ok(v) => format!("INTERVAL\t{}\t{}", raw(v.lower()), raw(v.upper())),
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
        ["Add", children @ ..] => {
            Node::Add(children.iter().map(|x| id(x)).collect::<Result<_, _>>()?)
        }
        ["Mul", children @ ..] => {
            Node::Mul(children.iter().map(|x| id(x)).collect::<Result<_, _>>()?)
        }
        ["Pow", a, b] => Node::Pow(id(a)?, id(b)?),
        ["sin", a] => Node::Sin(id(a)?),
        ["cos", a] => Node::Cos(id(a)?),
        ["atan", a] => Node::Atan(id(a)?),
        ["atan2", a, b] => Node::Atan2(id(a)?, id(b)?),
        ["tan", a] => Node::Tan(id(a)?),
        _ => return Err("closed Density expression wire".into()),
    };
    Ok(match graph.push(node) {
        Ok(id) => format!("NODE\t{id}"),
        Err(e) => format!("ERROR\t{e:?}"),
    })
}
fn main() {
    let mut graph = Expressions::new();
    let mut out = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let answer = line
            .map_err(|e| e.to_string())
            .and_then(|line| run(&mut graph, &line))
            .unwrap_or_else(|e| format!("PARSE_ERROR\t{e}"));
        writeln!(out, "{answer}").unwrap();
    }
}
