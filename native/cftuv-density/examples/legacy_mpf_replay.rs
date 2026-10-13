//! Тестовый legacy replay: op TAB precision TAB rounding TAB A TAB B-or-exponent.
//! Raw MPF=sign,mantissa,exponent,bitcount; raw-интервал=lo|hi.
//! Rational A/B — знаковые целые; normalize A=signed-mantissa,exponent.

use std::io::{self, BufRead, Write};

use cftuv_density::legacy_interval::LegacyInterval;
#[cfg(test)]
use cftuv_density::legacy_mpf::LegacyError;
use cftuv_density::legacy_mpf::{LegacyResult, Mpf, Precision, Rounding};
use cftuv_density::{IBig, UBig};

fn raw_text(value: &Mpf) -> String {
    let (sign, man, exp, bc) = value.raw_tuple();
    format!("{sign},{man},{exp},{bc}")
}

fn interval_text(value: &LegacyInterval) -> String {
    format!("{}|{}", raw_text(value.lower()), raw_text(value.upper()))
}

fn read_mpf(text: &str) -> Result<Mpf, String> {
    let fields: Vec<_> = text.split(',').collect();
    if fields.len() != 4 {
        return Err("raw MPF needs four comma fields".into());
    }
    let sign = fields[0]
        .parse()
        .map_err(|error| format!("sign: {error:?}"))?;
    let man =
        UBig::from_str_radix(fields[1], 10).map_err(|error| format!("mantissa: {error:?}"))?;
    let exp = fields[2]
        .parse()
        .map_err(|error| format!("exponent: {error:?}"))?;
    let bc = fields[3]
        .parse()
        .map_err(|error| format!("bitcount: {error:?}"))?;
    Mpf::from_raw_tuple(sign, man, exp, bc).map_err(|error| format!("{error:?}"))
}

fn read_interval(text: &str) -> Result<LegacyInterval, String> {
    let (lo, hi) = text
        .split_once('|')
        .ok_or_else(|| "raw interval needs lo|hi".to_string())?;
    LegacyInterval::new(read_mpf(lo)?, read_mpf(hi)?).map_err(|error| format!("{error:?}"))
}

fn rounding(text: &str) -> Result<Rounding, String> {
    match text {
        "f" => Ok(Rounding::Floor),
        "c" => Ok(Rounding::Ceiling),
        "d" => Ok(Rounding::TowardZero),
        "u" => Ok(Rounding::AwayFromZero),
        "n" => Ok(Rounding::Nearest),
        _ => Err(format!("unsupported rounding {text:?}")),
    }
}

fn scalar_answer(answer: LegacyResult<Mpf>) -> String {
    match answer {
        Ok(value) => format!("MPF\t{}", raw_text(&value)),
        Err(error) => format!("ERROR\t{error:?}"),
    }
}

fn interval_answer(answer: LegacyResult<LegacyInterval>) -> String {
    match answer {
        Ok(value) => format!("MPI\t{}", interval_text(&value)),
        Err(error) => format!("ERROR\t{error:?}"),
    }
}

fn run(line: &str) -> Result<String, String> {
    let fields: Vec<_> = line.split('\t').collect();
    if fields.len() != 5 {
        return Err(format!("expected 5 TAB fields, got {}", fields.len()));
    }
    let bits = fields[1]
        .parse()
        .map_err(|error| format!("precision: {error:?}"))?;
    let precision = match Precision::new(bits) {
        Ok(p) => p,
        Err(error) => return Ok(format!("ERROR\t{error:?}")),
    };
    if fields[0].starts_with("i_") {
        let a = read_interval(fields[3])?;
        return Ok(interval_answer(match fields[0] {
            "i_pos" => a.pos(precision),
            "i_neg" => a.neg(precision),
            "i_abs" => a.abs(precision),
            "i_square" => a.square(precision),
            "i_sqrt" => a.sqrt(precision),
            "i_pow" => a.pow_int(
                fields[4]
                    .parse()
                    .map_err(|error| format!("power: {error:?}"))?,
                precision,
            ),
            "i_add" => a.add(&read_interval(fields[4])?, precision),
            "i_sub" => a.sub(&read_interval(fields[4])?, precision),
            "i_mul" => a.mul(&read_interval(fields[4])?, precision),
            "i_div" => a.div(&read_interval(fields[4])?, precision),
            operation => return Err(format!("unknown interval operation {operation:?}")),
        }));
    }
    let round = rounding(fields[2])?;
    if fields[0] == "rational" {
        let n =
            IBig::from_str_radix(fields[3], 10).map_err(|error| format!("numerator: {error:?}"))?;
        let d = IBig::from_str_radix(fields[4], 10)
            .map_err(|error| format!("denominator: {error:?}"))?;
        return Ok(scalar_answer(Mpf::from_rational(n, d, precision, round)));
    }
    if fields[0] == "normalize" {
        let (man, exp) = fields[3]
            .split_once(',')
            .ok_or_else(|| "normalize expects man,exp".to_string())?;
        let man = IBig::from_str_radix(man, 10).map_err(|error| format!("mantissa: {error:?}"))?;
        let exp = exp
            .parse()
            .map_err(|error| format!("exponent: {error:?}"))?;
        return Ok(scalar_answer(Mpf::from_man_exp(
            man,
            exp,
            Some(precision),
            round,
        )));
    }
    let a = read_mpf(fields[3])?;
    Ok(scalar_answer(match fields[0] {
        "pos" => a.pos(precision, round),
        "neg" => a.neg(precision, round),
        "square" => a.square(precision, round),
        "sqrt" => a.sqrt(precision, round),
        "pow" => a.pow_int(
            fields[4]
                .parse()
                .map_err(|error| format!("power: {error:?}"))?,
            precision,
            round,
        ),
        "add" => a.add(&read_mpf(fields[4])?, precision, round),
        "sub" => a.sub(&read_mpf(fields[4])?, precision, round),
        "mul" => a.mul(&read_mpf(fields[4])?, precision, round),
        "div" => a.div(&read_mpf(fields[4])?, precision, round),
        operation => return Err(format!("unknown scalar operation {operation:?}")),
    }))
}

fn main() -> io::Result<()> {
    let mut output = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let line = line?;
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        match run(&line) {
            Ok(answer) => writeln!(output, "{answer}")?,
            Err(error) => writeln!(output, "ERROR\tReadError({error})")?,
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replay_preserves_special_tuples_and_typed_errors() {
        assert_eq!(
            run("mul\t160\tf\t0,0,-456,-2\t0,0,0,0").unwrap(),
            "MPF\t0,0,-123,-1"
        );
        assert_eq!(
            run("div\t160\tf\t0,1,0,1\t0,0,0,0").unwrap(),
            "ERROR\tZeroDivision"
        );
        assert_eq!(
            run("i_pow\t160\t-\t0,0,-456,-2|0,0,-456,-2\t0").unwrap(),
            "MPI\t0,1,0,1|0,1,0,1"
        );
        assert_eq!(
            run("sqrt\t160\tf\t1,1,0,1\t-").unwrap(),
            format!("ERROR\t{:?}", LegacyError::ComplexResult)
        );
    }
}
