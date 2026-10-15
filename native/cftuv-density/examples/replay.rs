//! Независимый test-only replay фундаментальных операций (не продуктовый ABI).
//! TAB-поля: op, g00=n/d, g01=n/d, g11=n/d, owner=+-1,
//! lx, ly, rx, ry (Radical canonical_text), arg (q/turn orientation или `-`).
//! Одинаковая метрика подряд сохраняет memo; отдельная строка `reset` её очищает.
//! VALUE: wire, text, Work; VECTOR: xwire, ywire, xtext, ytext, Work;
//! BOOL/SIGN: значение, Work; errors: отдельная категория, причина, Work.
//! Цена чтения входа не входит в диагностическую цену операции.

use std::io::{self, BufRead, Write};

use cftuv_density::{
    DensityFailure, DensityFailureKind, DensityTransaction, IBig, Metric2, Radical, Rational, UBig,
    Vector2, Work,
};
use cftuv_radical::Failure;

fn work_text(work: Work) -> String {
    work.wire()
        .iter()
        .map(u64::to_string)
        .collect::<Vec<_>>()
        .join(",")
}

fn wire_text(value: &Radical) -> String {
    let wire = value.wire();
    if wire.is_empty() {
        "-".into()
    } else {
        wire.iter()
            .map(IBig::to_string)
            .collect::<Vec<_>>()
            .join(",")
    }
}

fn value_text(value: Radical, work: Work) -> String {
    format!(
        "VALUE\t{}\t{}\t{}",
        wire_text(&value),
        value.text(),
        work_text(work)
    )
}

fn failure_text(failure: DensityFailure) -> String {
    let price = work_text(failure.work);
    match failure.kind {
        DensityFailureKind::InvalidInput(reason) => format!("INPUT_ERROR\t{reason:?}\t{price}"),
        DensityFailureKind::InternalAlgebra(reason) => {
            let reason = match reason {
                Failure::Refused(refusal) => refusal.message(),
                Failure::ZeroDivision => "ZeroDivision: division by an exact zero".into(),
                Failure::SeparationBoundViolated(detail) => {
                    format!("SeparationBoundViolated: {detail}")
                }
            };
            format!("INTERNAL_ALGEBRA\t{reason}\t{price}")
        }
    }
}

fn read_rational(text: &str) -> Result<Rational, String> {
    let (n, d) = text
        .split_once('/')
        .ok_or_else(|| format!("expected n/d: {text:?}"))?;
    let numerator = IBig::from_str_radix(n, 10).map_err(|error| format!("numerator: {error:?}"))?;
    let denominator =
        UBig::from_str_radix(d, 10).map_err(|error| format!("denominator: {error:?}"))?;
    Rational::new(numerator, denominator).map_err(failure_text)
}

fn read_scalar(text: &str) -> Result<Radical, String> {
    Radical::from_text(text)
        .map(|priced| priced.0)
        .map_err(|error| format!("scalar: {error:?}"))
}

fn run(line: &str, transaction: &mut Option<DensityTransaction>) -> Result<String, String> {
    if line == "reset" {
        *transaction = None;
        return Ok("RESET".into());
    }
    let fields: Vec<_> = line.split('\t').collect();
    if fields.len() != 10 {
        return Err(format!("expected 10 TAB fields, got {}", fields.len()));
    }
    let a = read_rational(fields[1])?;
    let b = read_rational(fields[2])?;
    let c = read_rational(fields[3])?;
    let owner: i8 = fields[4]
        .parse()
        .map_err(|error| format!("owner: {error:?}"))?;
    let (metric, _) = match Metric2::from_gram([[a, b.clone()], [b, c]], owner) {
        Ok(priced) => priced,
        Err(failure) => return Ok(failure_text(failure)),
    };
    if !transaction
        .as_ref()
        .is_some_and(|current| current.metric() == &metric)
    {
        *transaction = Some(DensityTransaction::new(metric));
    }
    let current = transaction.as_mut().expect("metric transaction is set");
    let left = Vector2::new(read_scalar(fields[5])?, read_scalar(fields[6])?);
    let right = Vector2::new(read_scalar(fields[7])?, read_scalar(fields[8])?);
    match fields[0] {
        "gram_dot" => {
            let (value, work) = current.gram_dot(&left, &right);
            Ok(value_text(value, work))
        }
        "dual_dot" => {
            let (value, work) = current.dual_dot(&left, &right);
            Ok(value_text(value, work))
        }
        "cross" => {
            let (value, work) = current.oriented_cross(&left, &right);
            Ok(value_text(value, work))
        }
        "quarter_turn" => {
            let orientation = fields[9]
                .parse()
                .map_err(|error| format!("orientation: {error:?}"))?;
            Ok(match current.quarter_turn(&left, orientation) {
                Ok((value, work)) => format!(
                    "VECTOR\t{}\t{}\t{}\t{}\t{}",
                    wire_text(&value.x),
                    wire_text(&value.y),
                    value.x.text(),
                    value.y.text(),
                    work_text(work)
                ),
                Err(failure) => failure_text(failure),
            })
        }
        "subturn" | "subturn_boundary" => {
            let q = fields[9].parse().map_err(|error| format!("q: {error:?}"))?;
            let answer = if fields[0] == "subturn" {
                current.subturn(&left, &right, q)
            } else {
                current.subturn_boundary(&left, &right, q)
            };
            Ok(match answer {
                Ok((value, work)) => format!("BOOL\t{}\t{}", u8::from(value), work_text(work)),
                Err(failure) => failure_text(failure),
            })
        }
        "sign" => Ok(match current.sign(&left.x) {
            Ok((value, work)) => format!("SIGN\t{value}\t{}", work_text(work)),
            Err(failure) => failure_text(failure),
        }),
        operation => Err(format!("unknown operation {operation:?}")),
    }
}

fn main() -> io::Result<()> {
    let mut transaction = None;
    let mut output = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let line = line?;
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        match run(&line, &mut transaction) {
            Ok(answer) => writeln!(output, "{answer}")?,
            Err(reason) => writeln!(output, "READ_ERROR\t{reason}")?,
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replay_keeps_memo_until_reset_and_preserves_the_error_category() {
        let mut transaction = None;
        let row =
            "quarter_turn\t2/1\t1/1\t3/1\t1\tInteger(1)\tInteger(2)\tInteger(0)\tInteger(0)\t1";
        let cold = run(row, &mut transaction).unwrap();
        let warm = run(row, &mut transaction).unwrap();
        assert!(cold.starts_with("VECTOR\t45,-1,5\t5,1,5\t"));
        assert!(warm.ends_with("\t0,0,0"));
        assert_eq!(run("reset", &mut transaction).unwrap(), "RESET");
        assert_eq!(run(row, &mut transaction).unwrap(), cold);
        let unsupported = row.replacen("quarter_turn", "subturn", 1);
        assert_eq!(
            run(&unsupported, &mut transaction).unwrap(),
            "INPUT_ERROR\tUnsupportedDensityQ(1)\t0,0,0"
        );
    }
}
