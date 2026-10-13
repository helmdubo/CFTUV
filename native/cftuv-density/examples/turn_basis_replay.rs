//! Test-only value replay, TAB: op,id,g00,g01,g11,owner,ix,iy,ox,oy,arg.
//! turn/h0: arg=orientation; unit: ox=squared; left: outgoing/arg не читаются.
//! Одинаковая метрика подряд сохраняет transaction memo; строка reset очищает его.
//! Это не product ABI и не legacy expression-DAG replay. V2 Work только диагностика.

use std::io::{self, BufRead, Write};

use cftuv_density::turn_basis::{TurnBasisTransaction, TurnFailure, TurnFailureKind, TurnResult};
use cftuv_density::{DensityTransaction, IBig, Metric2, Radical, Rational, UBig, Vector2, Work};

fn rational(text: &str) -> Result<Rational, String> {
    let (n, d) = text.split_once('/').ok_or("expected n/d")?;
    Rational::new(
        n.parse::<IBig>().map_err(|e| format!("{e:?}"))?,
        d.parse::<UBig>().map_err(|e| format!("{e:?}"))?,
    )
    .map_err(|e| format!("{e:?}"))
}

fn scalar(text: &str) -> Result<Radical, String> {
    Radical::from_text(text)
        .map(|value| value.0)
        .map_err(|error| format!("{error:?}"))
}

fn vector_text(vector: &Vector2) -> String {
    format!("{}~{}", vector.x.text(), vector.y.text())
}

fn work_text(work: Work) -> String {
    work.wire()
        .iter()
        .map(u64::to_string)
        .collect::<Vec<_>>()
        .join(",")
}

fn failure_text(error: &TurnFailure) -> String {
    match &error.kind {
        TurnFailureKind::Product(reason) => {
            format!("PRODUCT:{}:{}", reason.outcome(), reason.message())
        }
        TurnFailureKind::InternalUnsupported(reason) => format!("INTERNAL_UNSUPPORTED:{reason:?}"),
        TurnFailureKind::InternalDensity(reason) => format!("INTERNAL_DENSITY:{reason:?}"),
    }
}

fn answer<T>(result: TurnResult<T>, text: impl FnOnce(T) -> String) -> (String, Work) {
    match result {
        Ok((value, work)) => (text(value), work),
        Err(error) => (failure_text(&error), error.work),
    }
}

fn run(line: &str, cached: &mut Option<TurnBasisTransaction>) -> Result<String, String> {
    if line == "reset" {
        *cached = None;
        return Ok("RESET".into());
    }
    let fields: Vec<_> = line.split('\t').collect();
    if fields.len() != 11 {
        return Err(format!("expected11fields,got{}", fields.len()));
    }
    let (a, b, c) = (
        rational(fields[2])?,
        rational(fields[3])?,
        rational(fields[4])?,
    );
    let owner = fields[5].parse::<i8>().map_err(|e| format!("{e:?}"))?;
    let metric = Metric2::from_gram([[a, b.clone()], [b, c]], owner)
        .map_err(|e| format!("{e:?}"))?
        .0;
    if cached.as_ref().is_none_or(|tx| tx.metric() != &metric) {
        *cached = Some(TurnBasisTransaction::new(DensityTransaction::new(metric)));
    }
    let tx = cached.as_mut().unwrap();
    let incoming = Vector2::new(scalar(fields[6])?, scalar(fields[7])?);
    let outgoing = Vector2::new(scalar(fields[8])?, scalar(fields[9])?);
    let (output, work) = match fields[0] {
        "turn" => answer(
            tx.prepare_turn(
                &incoming,
                &outgoing,
                fields[10].parse().map_err(|e| format!("{e:?}"))?,
            ),
            |atom| {
                format!(
                    "ATOM:{}~{}~{}|{}|{}",
                    atom.incoming_squared.text(),
                    atom.outgoing_squared.text(),
                    atom.raw_dot.text(),
                    vector_text(&atom.incoming),
                    vector_text(&atom.outgoing)
                )
            },
        ),
        "h0" => answer(
            tx.prepare_turn(
                &incoming,
                &outgoing,
                fields[10].parse().map_err(|e| format!("{e:?}"))?,
            ),
            |atom| {
                format!(
                    "VECTORS:{}|{}",
                    vector_text(&atom.incoming),
                    vector_text(&atom.outgoing)
                )
            },
        ),
        "unit" => answer(tx.unit_from_squared(&incoming, &outgoing.x), |vector| {
            format!("VECTOR:{}", vector_text(&vector))
        }),
        "left" => answer(tx.left_unit_normal(&incoming), |vector| {
            format!("VECTOR:{}", vector_text(&vector))
        }),
        operation => return Err(format!("unsupported operation: {operation}")),
    };
    Ok(format!(
        "{}\t{}\t{}\tV2_WORK:{}\tSEARCH_WORK:0,0",
        fields[0],
        fields[1],
        output,
        work_text(work)
    ))
}

fn main() {
    let mut transaction = None;
    let mut stdout = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let line = line.expect("stdin line");
        if line.is_empty() {
            continue;
        }
        match run(&line, &mut transaction) {
            Ok(output) => writeln!(stdout, "{output}").unwrap(),
            Err(error) => {
                eprintln!("INPUT_ERROR:{error}");
                std::process::exit(1);
            }
        }
    }
}
