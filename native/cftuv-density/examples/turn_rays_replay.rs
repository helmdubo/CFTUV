//! Test-only certified-fact replay, TAB: op,id,norm_in,norm_out,dot²,raw_dot,
//! canonical,ordinal,subdivision,incoming,left,orientation,rotations,expected,raw_context.
//! Rational=n/d; vector=V2~V2; rotations=a,b|a,b or '-'. Цена чтения не входит в Work.
//! Нет legacy admission/DAG/memo orchestration; raw_context сохраняется в oracle evidence.

use std::io::{self, BufRead, Write};

use cftuv_density::turn_rays::{
    cosine_squared, principal_turn, table_rays, CertifiedGramSquares, PiMultiple, RayFailure,
    RayFailureKind, RayResult,
};
use cftuv_density::{IBig, Radical, Rational, UBig, Vector2, Work};

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
        .map_err(|e| format!("{e:?}"))
}
fn vector(text: &str) -> Result<Vector2, String> {
    let (x, y) = text.split_once('~').ok_or("expected vector")?;
    Ok(Vector2::new(scalar(x)?, scalar(y)?))
}
fn ratio_text(value: &Rational) -> String {
    let (n, d) = value.parts();
    format!("{n}/{d}")
}
fn vector_text(value: &Vector2) -> String {
    format!("{}~{}", value.x.text(), value.y.text())
}
fn work_text(work: Work) -> String {
    work.wire()
        .iter()
        .map(u64::to_string)
        .collect::<Vec<_>>()
        .join(",")
}
fn failure_text(error: &RayFailure) -> String {
    match &error.kind {
        RayFailureKind::Product(reason) => {
            format!("PRODUCT:{}:{}", reason.outcome(), reason.message())
        }
        RayFailureKind::InternalInvalid(reason) => format!("INTERNAL_INVALID:{reason:?}"),
        RayFailureKind::InternalAlgebra(reason) => format!("INTERNAL_ALGEBRA:{reason:?}"),
    }
}
fn answer<T>(result: RayResult<T>, text: impl FnOnce(T) -> String) -> (String, Work) {
    match result {
        Ok((value, work)) => (text(value), work),
        Err(error) => (failure_text(&error), error.work),
    }
}
fn rotations(text: &str) -> Result<Vec<(IBig, IBig)>, String> {
    if text == "-" {
        return Ok(Vec::new());
    }
    text.split('|')
        .map(|row| {
            let (a, b) = row.split_once(',').ok_or("expected a,b")?;
            Ok((
                a.parse::<IBig>().map_err(|e| format!("{e:?}"))?,
                b.parse::<IBig>().map_err(|e| format!("{e:?}"))?,
            ))
        })
        .collect()
}

fn run(fields: &[&str]) -> Result<String, String> {
    if fields.len() != 15 {
        return Err(format!("expected15fields,got{}", fields.len()));
    }
    let (output, work) = match fields[0] {
        "ratio" => answer(
            cosine_squared(&CertifiedGramSquares {
                incoming_squared: rational(fields[2])?,
                outgoing_squared: rational(fields[3])?,
                dot_squared: rational(fields[4])?,
            }),
            |value| format!("RATIO:{}", ratio_text(&value)),
        ),
        "principal" => {
            let facts = CertifiedGramSquares {
                incoming_squared: rational(fields[2])?,
                outgoing_squared: rational(fields[3])?,
                dot_squared: rational(fields[4])?,
            };
            let raw_dot = scalar(fields[5])?;
            let result = cosine_squared(&facts).and_then(|(ratio, prefix)| {
                principal_turn(&ratio, &raw_dot)
                    .map(|(value, work)| (value, prefix + work))
                    .map_err(|mut error| {
                        error.work = prefix + error.work;
                        error
                    })
            });
            answer(result, |value| {
                format!(
                    "PRINCIPAL:{}~{}",
                    ratio_text(value.source().cosine_squared()),
                    value.source().dot_sign()
                )
            })
        }
        "canonical" => answer(
            PiMultiple::ray_angle(
                &rational(fields[6])?,
                fields[7].parse().map_err(|e| format!("{e:?}"))?,
                fields[8].parse().map_err(|e| format!("{e:?}"))?,
            ),
            |value| format!("PI_MULTIPLE:{}", ratio_text(value.coefficient())),
        ),
        "table" => answer(
            table_rays(
                &vector(fields[9])?,
                &vector(fields[10])?,
                fields[11].parse().map_err(|e| format!("{e:?}"))?,
                fields[13].parse().map_err(|e| format!("{e:?}"))?,
                &rotations(fields[12])?,
            ),
            |rays| {
                format!(
                    "RAYS:{}",
                    rays.iter().map(vector_text).collect::<Vec<_>>().join("|")
                )
            },
        ),
        other => return Err(format!("unknown operation:{other}")),
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
    let mut stdout = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let line = line.expect("stdin");
        if line.is_empty() {
            continue;
        }
        let fields: Vec<_> = line.split('\t').collect();
        match run(&fields) {
            Ok(answer) => writeln!(stdout, "{answer}").unwrap(),
            Err(error) => {
                eprintln!("INPUT_ERROR:{error}");
                std::process::exit(1);
            }
        }
    }
}
