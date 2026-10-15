//! Test-only A2 replay: five TAB fields `bound|count|poly`, n, p/d, dot_sign, argument.
//! bound argument=bits; count argument=x=n/d; poly argument=`-`; ray argument=j,bits.
//! BOUND: lower, upper, rank, degree, Sturm length, bisections, variation evaluations,
//! G(lower), G(upper), lower-is-root, upper-is-root. Это rank enclosure, не isolation.

use std::io::{self, BufRead, Write};

use cftuv_density::angle::AngleAtom;
use cftuv_density::interval::RationalInterval;
use cftuv_density::{IBig, Rational, UBig};

fn read_rational(text: &str) -> Result<Rational, String> {
    let (n, d) = text
        .split_once('/')
        .ok_or_else(|| format!("expected n/d: {text:?}"))?;
    let numerator = IBig::from_str_radix(n, 10).map_err(|error| format!("numerator: {error:?}"))?;
    let denominator =
        UBig::from_str_radix(d, 10).map_err(|error| format!("denominator: {error:?}"))?;
    Rational::new(numerator, denominator).map_err(|error| format!("rational: {error:?}"))
}

fn rational_text(value: &Rational) -> String {
    let (n, d) = value.parts();
    format!("{n}/{d}")
}

fn run(line: &str) -> Result<String, String> {
    let fields: Vec<_> = line.split('\t').collect();
    if fields.len() != 5 {
        return Err(format!("expected 5 TAB fields, got {}", fields.len()));
    }
    if fields[0].starts_with("interval_") {
        return run_interval(&fields);
    }
    let n = fields[1].parse().map_err(|error| format!("n: {error:?}"))?;
    let ratio = read_rational(fields[2])?;
    let dot_sign = fields[3]
        .parse()
        .map_err(|error| format!("sign: {error:?}"))?;
    let atom = match AngleAtom::new(n, ratio, dot_sign) {
        Ok(atom) => atom,
        Err(error) => return Ok(format!("INPUT_ERROR\t{error:?}")),
    };
    match fields[0] {
        "ray" => {
            let (ordinal, bits) = fields[4]
                .split_once(',')
                .ok_or_else(|| "ray expects j,bits".to_string())?;
            let ordinal = ordinal
                .parse()
                .map_err(|error| format!("ordinal: {error:?}"))?;
            let bits = bits.parse().map_err(|error| format!("bits: {error:?}"))?;
            Ok(match atom.ray_enclosure(ordinal, bits) {
                Ok(ray) => format!(
                    "RAY\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                    rational_text(ray.cosine.lower()),
                    rational_text(ray.cosine.upper()),
                    rational_text(ray.sine.lower()),
                    rational_text(ray.sine.upper()),
                    rational_text(&ray.root.lower),
                    rational_text(&ray.root.upper),
                    ray.root.stats.bisections
                ),
                Err(error) => format!("RAY_ERROR\t{error:?}"),
            })
        }
        "bound" => {
            let bits = fields[4]
                .parse()
                .map_err(|error| format!("bits: {error:?}"))?;
            Ok(match atom.enclose(bits) {
                Ok(bound) => format!(
                    "BOUND\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                    rational_text(&bound.lower),
                    rational_text(&bound.upper),
                    atom.rank(),
                    bound.stats.polynomial_degree,
                    bound.stats.sturm_polynomials,
                    bound.stats.bisections,
                    bound.stats.variation_evaluations,
                    atom.roots_strictly_above(&bound.lower),
                    atom.roots_strictly_above(&bound.upper),
                    u8::from(atom.is_root(&bound.lower)),
                    u8::from(atom.is_root(&bound.upper))
                ),
                Err(error) => format!("INTERNAL_ERROR\t{error:?}"),
            })
        }
        "count" => {
            let value = read_rational(fields[4])?;
            if value.parts().0 < &-IBig::from(value.parts().1.clone())
                || value.parts().0 > &IBig::from(value.parts().1.clone())
            {
                return Err("count point must be in [-1,1]".into());
            }
            Ok(format!(
                "COUNT\t{}\t{}\t{}",
                atom.roots_strictly_above(&value),
                u8::from(atom.is_root(&value)),
                atom.total_roots()
            ))
        }
        "poly" => Ok(format!(
            "POLY\t{}",
            atom.polynomial_coefficients()
                .iter()
                .map(rational_text)
                .collect::<Vec<_>>()
                .join(",")
        )),
        operation => Err(format!("unknown operation {operation:?}")),
    }
}

// Binary fields: op, lo, hi, other-lo, other-hi; unary: op, lo, hi, bits-or-'-', '-'.
fn run_interval(fields: &[&str]) -> Result<String, String> {
    let value = match RationalInterval::new(read_rational(fields[1])?, read_rational(fields[2])?) {
        Ok(value) => value,
        Err(error) => return Ok(format!("INPUT_ERROR\t{error:?}")),
    };
    let output = match fields[0] {
        "interval_square" => Ok(value.square()),
        "interval_scale" => Ok(value),
        "interval_sqrt" => value.positive_sqrt(
            fields[3]
                .parse()
                .map_err(|error| format!("bits: {error:?}"))?,
        ),
        "interval_add" | "interval_sub" | "interval_mul" | "interval_div" => {
            let other =
                match RationalInterval::new(read_rational(fields[3])?, read_rational(fields[4])?) {
                    Ok(value) => value,
                    Err(error) => return Ok(format!("INPUT_ERROR\t{error:?}")),
                };
            match fields[0] {
                "interval_add" => Ok(value.add(&other)),
                "interval_sub" => Ok(value.sub(&other)),
                "interval_mul" => Ok(value.mul(&other)),
                _ => value.div(&other),
            }
        }
        operation => return Err(format!("unknown operation {operation:?}")),
    };
    Ok(match output {
        Ok(interval) => {
            let (scaled_lower, scaled_upper) = interval.scale10_27_envelope();
            format!(
                "INTERVAL\t{}\t{}\t{scaled_lower}\t{scaled_upper}",
                rational_text(interval.lower()),
                rational_text(interval.upper())
            )
        }
        Err(error) => format!("INTERVAL_ERROR\t{error:?}"),
    })
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
            Err(reason) => writeln!(output, "READ_ERROR\t{reason}")?,
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replay_distinguishes_rank_count_and_invalid_input() {
        assert_eq!(run("count\t3\t81/256\t1\t3/4").unwrap(), "COUNT\t1\t1\t6");
        let bound = run("bound\t3\t81/256\t-1\t32").unwrap();
        assert!(bound.starts_with("BOUND\t3/4\t3/4\t2\t6\t"));
        assert_eq!(
            run("bound\t3\t1/1\t1\t32").unwrap(),
            "INPUT_ERROR\tRatioOutsideHalfOpenUnitInterval"
        );
    }

    #[test]
    fn ray_and_interval_protocols_preserve_closed_bounds_and_error_categories() {
        let ray = run("ray\t3\t81/256\t-1\t2,32").unwrap();
        assert!(ray.starts_with("RAY\t1/8\t1/8\t"));
        assert_eq!(
            run("interval_div\t0/1\t0/1\t-1/1\t1/1").unwrap(),
            "INTERVAL_ERROR\tDenominatorContainsZero"
        );
        assert_eq!(
            run("interval_square\t-1/1\t1/1\t-\t-").unwrap(),
            "INTERVAL\t0/1\t1/1\t0\t1000000000000000000000000000"
        );
        assert_eq!(
            run("interval_scale\t-1/3\t-1/3\t-\t-").unwrap(),
            "INTERVAL\t-1/3\t-1/3\t-333333333333333333333333334\t-333333333333333333333333333"
        );
    }
}
