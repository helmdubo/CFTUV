//! Только дифференциальный wire: scalar/interval/reduce, исходные MPF tuples.
use cftuv_density::legacy_atan::{atan_cached, mpf_atan, mpf_atan2, mpi_atan, mpi_atan2_raw};
use cftuv_density::legacy_interval::LegacyInterval;
use cftuv_density::legacy_mpf::{Mpf, Precision, Rounding};
use cftuv_density::legacy_trig_eval::{mod_pi2, mpf_cos_sin, mpi_cos_sin};
use std::io::{self, BufRead, Write};

fn raw(text: &str) -> Result<Mpf, String> {
    let parts: Vec<_> = text.split(',').collect();
    if parts.len() != 4 {
        return Err("four MPF fields required".into());
    }
    Mpf::from_raw_tuple(
        parts[0].parse().map_err(|e| format!("{e:?}"))?,
        parts[1].parse().map_err(|e| format!("{e:?}"))?,
        parts[2].parse().map_err(|e| format!("{e:?}"))?,
        parts[3].parse().map_err(|e| format!("{e:?}"))?,
    )
    .map_err(|e| format!("{e:?}"))
}
fn wire(value: &Mpf) -> String {
    let (s, m, e, b) = value.raw_tuple();
    format!("{s},{m},{e},{b}")
}
fn atan_run(fields: &[&str]) -> Result<String, String> {
    if let ["cache", p, n] = fields {
        return Ok(
            match atan_cached(
                n.parse().map_err(|e| format!("{e:?}"))?,
                p.parse().map_err(|e| format!("{e:?}"))?,
            ) {
                Ok((a, b)) => format!("FIXED_PAIR\t{a}\t{b}"),
                Err(e) => format!("ERROR\t{e:?}"),
            },
        );
    }
    if fields.len() < 3 {
        return Err("atan fields".into());
    }
    let p = Precision::new(fields[1].parse().map_err(|e| format!("{e:?}"))?)
        .map_err(|e| format!("{e:?}"))?;
    if fields[0] == "atan" || fields[0] == "atan2" {
        let r = match fields[2] {
            "f" => Rounding::Floor,
            "c" => Rounding::Ceiling,
            "d" => Rounding::TowardZero,
            "u" => Rounding::AwayFromZero,
            "n" => Rounding::Nearest,
            _ => return Err("rounding".into()),
        };
        let result = match fields {
            ["atan", _, _, x] => mpf_atan(&raw(x)?, p, r),
            ["atan2", _, _, y, x] => mpf_atan2(&raw(y)?, &raw(x)?, p, r),
            _ => return Err("atan scalar fields".into()),
        };
        return Ok(match result {
            Ok(v) => format!("MPF\t{}", wire(&v)),
            Err(e) => format!("ERROR\t{e:?}"),
        });
    }
    let interval =
        |a: &str, b: &str| LegacyInterval::new(raw(a)?, raw(b)?).map_err(|e| format!("{e:?}"));
    let result = match fields {
        ["iatan", _, a, b] => {
            mpi_atan(&interval(a, b)?, p).map(|v| (v.lower().clone(), v.upper().clone()))
        }
        ["iatan2", _, ya, yb, xa, xb] => mpi_atan2_raw(&interval(ya, yb)?, &interval(xa, xb)?, p),
        _ => return Err("atan interval fields".into()),
    };
    Ok(match result {
        Ok((a, b)) => format!("INTERVAL\t{}\t{}", wire(&a), wire(&b)),
        Err(e) => format!("ERROR\t{e:?}"),
    })
}
fn run(line: &str) -> Result<String, String> {
    let fields: Vec<_> = line.split('\t').collect();
    match fields.as_slice() {
        ["decimal", p, a, b] => {
            let interval = LegacyInterval::new(raw(a)?, raw(b)?).map_err(|e| format!("{e:?}"))?;
            Ok(
                match cftuv_density::decimal_projection::project(
                    &interval,
                    p.parse().map_err(|e| format!("{e:?}"))?,
                ) {
                    Ok(v) => format!(
                        "DECIMAL\t{}\t{}\t{}",
                        v.lower.text(),
                        v.upper.text(),
                        v.width.text()
                    ),
                    Err(e) => format!("ERROR\t{e:?}"),
                },
            )
        }
        ["scalar", p, rounding, x] => {
            let precision = Precision::new(p.parse().map_err(|e| format!("{e:?}"))?)
                .map_err(|e| format!("{e:?}"))?;
            let rounding = match *rounding {
                "f" => Rounding::Floor,
                "c" => Rounding::Ceiling,
                "d" => Rounding::TowardZero,
                "u" => Rounding::AwayFromZero,
                "n" => Rounding::Nearest,
                _ => return Err("rounding".into()),
            };
            match mpf_cos_sin(&raw(x)?, precision, rounding) {
                Ok((c, s)) => Ok(format!("PAIR\t{}\t{}", wire(&c), wire(&s))),
                Err(e) => Ok(format!("ERROR\t{e:?}")),
            }
        }
        ["interval", p, a, b] => {
            let precision = Precision::new(p.parse().map_err(|e| format!("{e:?}"))?)
                .map_err(|e| format!("{e:?}"))?;
            let interval = LegacyInterval::new(raw(a)?, raw(b)?).map_err(|e| format!("{e:?}"))?;
            match mpi_cos_sin(&interval, precision) {
                Ok((c, s)) => Ok(format!(
                    "INTERVAL_PAIR\t{}\t{}\t{}\t{}",
                    wire(c.lower()),
                    wire(c.upper()),
                    wire(s.lower()),
                    wire(s.upper())
                )),
                Err(e) => Ok(format!("ERROR\t{e:?}")),
            }
        }
        ["reduce", p, x] => match mod_pi2(&raw(x)?, p.parse().map_err(|e| format!("{e:?}"))?) {
            Ok(v) => Ok(format!(
                "REDUCED\t{}\t{}\t{}",
                v.fixed_argument, v.quadrant, v.work_bits
            )),
            Err(e) => Ok(format!("ERROR\t{e:?}")),
        },
        _ => atan_run(&fields),
    }
}
fn main() {
    let mut out = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let answer = line
            .map_err(|e| e.to_string())
            .and_then(|line| run(&line))
            .unwrap_or_else(|e| format!("PARSE_ERROR\t{e}"));
        writeln!(out, "{answer}").unwrap();
    }
}
