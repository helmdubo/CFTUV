//! Тестовый π replay: pi_fixed TAB bits; pi TAB precision TAB f/c/d/u/n.
//! Ответ: FIXED TAB integer либо MPF TAB sign,mantissa,exponent,bitcount.

use std::io::{self, BufRead, Write};

use cftuv_density::legacy_mpf::{Precision, Rounding};
use cftuv_density::legacy_trig::{mpf_pi, pi_fixed};

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

fn run(line: &str) -> Result<String, String> {
    let fields: Vec<_> = line.split('\t').collect();
    match fields.as_slice() {
        ["pi_fixed", bits] => match pi_fixed(bits.parse().map_err(|e| format!("bits: {e:?}"))?) {
            Ok(value) => Ok(format!("FIXED\t{value}")),
            Err(error) => Ok(format!("ERROR\t{error:?}")),
        },
        ["pi", bits, round] => {
            let precision = match Precision::new(bits.parse().map_err(|e| format!("bits: {e:?}"))?)
            {
                Ok(value) => value,
                Err(error) => return Ok(format!("ERROR\t{error:?}")),
            };
            match mpf_pi(precision, rounding(round)?) {
                Ok(value) => {
                    let (sign, mantissa, exponent, bit_count) = value.raw_tuple();
                    Ok(format!("MPF\t{sign},{mantissa},{exponent},{bit_count}"))
                }
                Err(error) => Ok(format!("ERROR\t{error:?}")),
            }
        }
        _ => Err("expected pi_fixed TAB bits, or pi TAB precision TAB rounding".into()),
    }
}

fn main() {
    let mut stdout = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let answer = match line {
            Ok(line) => run(&line).unwrap_or_else(|error| format!("PARSE_ERROR\t{error}")),
            Err(error) => format!("IO_ERROR\t{error}"),
        };
        writeln!(stdout, "{answer}").expect("replay output");
    }
}

#[cfg(test)]
mod tests {
    use super::run;

    #[test]
    fn wire_preserves_raw_pi_and_named_limits() {
        assert_eq!(run("pi_fixed\t8").unwrap(), "FIXED\t804");
        assert_eq!(run("pi\t53\tn").unwrap(), "MPF\t0,884279719003555,-48,50");
        assert!(run("pi_fixed\t901")
            .unwrap()
            .starts_with("ERROR\tUnsupportedProfile"));
    }
}
