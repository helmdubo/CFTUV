//! Закрытый node wire, shell TAB id; каждый shell вызов имеет холодный local reader.
//! exact/sign команды нужны только для существующих regression gates.
use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::density_quadratic::{QuadraticFailureKind, QuadraticReader};
use cftuv_density::density_shell_v2::integer_shell;
use cftuv_density::density_shell_v2::{density_shell_v2, ShellError, ShellResult};
use cftuv_density::interval::RationalInterval;
use cftuv_density::{Rational, Work};
use cftuv_radical::Failure;
use std::io::{self, BufRead, Write};
fn quoted(v: &str) -> String {
    format!("\"{}\"", v.replace('\\', "\\\\").replace('"', "\\\""))
}
fn work(v: Work) -> String {
    format!(
        "[{}, {}, {}]",
        v.class_merge_checks, v.conjugation_products, v.enclosure_rounds
    )
}
fn shell(graph: &Expressions, id: usize) -> String {
    match density_shell_v2(graph,id) {
        Ok(ShellResult::Applied(v)) => {
            let wire=v.value.wire().iter().map(|n|quoted(&n.to_string())).collect::<Vec<_>>().join(",");
            let (ln,ld)=v.enclosure160.lower().parts(); let (hn,hd)=v.enclosure160.upper().parts();
            let codes=v.outcomes.codes.iter().map(|c|quoted(c.code())).collect::<Vec<_>>().join(",");
            let rational=match &v.outcomes.rational { None=>"null".into(), Some(q)=>format!("{{\"numerator\":{},\"denominator\":{},\"grid_units\":{}}}",
                quoted(&q.numerator.to_string()),quoted(&q.denominator.to_string()),q.grid_units.as_ref().map_or("null".into(),|v|quoted(&v.to_string()))) };
            format!("SHELL\t{{\"kind\":\"applied\",\"canon\":{},\"value\":[{}],\"bounds\":[[{},{}],[{},{}]],\"units\":[{},{},{}],\"quantum\":{},\"codes\":[{}],\"rational\":{},\"legacy_projected_min_width_units\":{},\"work\":{}}}",
                quoted(v.canon),wire,quoted(&ln.to_string()),quoted(&ld.to_string()),quoted(&hn.to_string()),quoted(&hd.to_string()),
                quoted(&v.shell.lower_units.to_string()),quoted(&v.shell.upper_units.to_string()),quoted(&v.shell.width_units.to_string()),v.shell.quantum,codes,rational,
                v.outcomes.legacy_projected_min_width_units.map_or("null".into(),|n|n.to_string()),work(v.work))
        }
        Ok(ShellResult::NotApplicable(v)) => format!("SHELL\t{{\"kind\":\"not_applicable\",\"canon\":{},\"code\":{},\"detail\":{},\"work\":{}}}",quoted(v.canon),quoted(v.code.code()),quoted(&v.detail),work(v.work)),
        Err(ShellError::Reader(v)) => {
            let code=match &v.kind { QuadraticFailureKind::ExponentProfile(_)=>"EXPONENT_PROFILE",
                QuadraticFailureKind::InvalidReference(_)=>"INVALID_REFERENCE",
                QuadraticFailureKind::InternalAlgebra(Failure::ZeroDivision)=>"ZERO_DIVISION", _=>"READER_INTERNAL" };
            format!("SHELL\t{{\"kind\":\"internal\",\"code\":{},\"work\":{}}}",quoted(code),work(v.work))
        }
    }
}
fn run(g: &mut Expressions, line: &str) -> Result<String, String> {
    let f: Vec<_> = line.split('\t').collect();
    let id = |v: &str| v.parse::<usize>().map_err(|e| e.to_string());
    let children = |args: &[&str]| args.iter().map(|a| id(a)).collect::<Result<Vec<_>, _>>();
    let node = match f.as_slice() {
        ["bounds", ln, ld, hn, hd] => {
            let rational = |n: &str, d: &str| {
                Rational::new(
                    n.parse().map_err(|e| format!("{e:?}"))?,
                    d.parse().map_err(|e| format!("{e:?}"))?,
                )
                .map_err(|e| format!("{e:?}"))
            };
            let interval = RationalInterval::new(rational(ln, ld)?, rational(hn, hd)?)
                .map_err(|e| format!("{e:?}"))?;
            let value = integer_shell(&interval);
            return Ok(format!(
                "UNITS\t{}\t{}\t{}\t{}",
                value.lower_units, value.upper_units, value.width_units, value.quantum
            ));
        }
        ["clear"] => {
            *g = Expressions::new();
            return Ok("CLEARED".into());
        }
        ["shell", source] => return Ok(shell(g, id(source)?)),
        ["exact", source] => {
            return Ok(match QuadraticReader::new(g).read(id(source)?) {
                Ok((v, _)) => format!(
                    "RADICAL\t{}",
                    v.wire()
                        .iter()
                        .map(ToString::to_string)
                        .collect::<Vec<_>>()
                        .join(",")
                ),
                Err(e) => match e.kind {
                    QuadraticFailureKind::UnsupportedSyntax(_) => "UNSUPPORTED".into(),
                    _ => format!("ERROR\t{e:?}"),
                },
            })
        }
        ["sign", source, expanded] => {
            return Ok(
                match g.sign_with_expanded_fallback(id(source)?, id(expanded)?) {
                    Ok(v) => format!("SIGN\t{}", v.sign),
                    Err(cftuv_density::density_sign::SignFailure::Undecidable) => {
                        "UNDECIDABLE".into()
                    }
                    Err(e) => format!("ERROR\t{e:?}"),
                },
            )
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
        ["sin", a] => Node::Sin(id(a)?),
        ["cos", a] => Node::Cos(id(a)?),
        ["tan", a] => Node::Tan(id(a)?),
        ["atan", a] => Node::Atan(id(a)?),
        ["atan2", a, b] => Node::Atan2(id(a)?, id(b)?),
        _ => return Err("closed shell node wire".into()),
    };
    Ok(format!(
        "NODE\t{}",
        g.push(node).map_err(|e| format!("{e:?}"))?
    ))
}
fn main() {
    let mut g = Expressions::new();
    let mut out = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let answer = line
            .map_err(|e| e.to_string())
            .and_then(|s| run(&mut g, &s))
            .unwrap_or_else(|e| format!("PARSE_ERROR\t{e}"));
        writeln!(out, "{answer}").unwrap();
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn zero_shell_quantum_is_explicit_and_zero_inverse_is_internal() {
        let mut g = Expressions::new();
        run(&mut g, "Q\t0\t1").unwrap();
        assert!(run(&mut g, "shell\t0").unwrap().contains("\"quantum\":-27"));
        run(&mut g, "Q\t-1\t1").unwrap();
        run(&mut g, "Pow\t0\t1").unwrap();
        assert!(run(&mut g, "shell\t2")
            .unwrap()
            .contains("\"code\":\"ZERO_DIVISION\""));
    }
}
