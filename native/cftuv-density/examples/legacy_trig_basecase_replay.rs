//! Тестовый wire: basecase TAB P TAB x; anchor TAB index. Целые в decimal.
//! Ответ FIXED_PAIR TAB cos TAB sin; за профилем ERROR TAB typed error.

use std::io::{self, BufRead, Write};

use cftuv_density::legacy_trig_basecase::{anchor, cos_sin_basecase, BasecaseResult};
use cftuv_density::IBig;

fn pair_text(answer: BasecaseResult<(IBig, IBig)>) -> String {
    match answer {
        Ok((cos, sin)) => format!("FIXED_PAIR\t{cos}\t{sin}"),
        Err(error) => format!("ERROR\t{error:?}"),
    }
}

fn run(line: &str) -> Result<String, String> {
    let fields: Vec<_> = line.split('\t').collect();
    match fields.as_slice() {
        ["basecase", bits, argument] => {
            let bits = bits.parse().map_err(|e| format!("bits: {e:?}"))?;
            let argument =
                IBig::from_str_radix(argument, 10).map_err(|e| format!("argument: {e:?}"))?;
            Ok(pair_text(cos_sin_basecase(&argument, bits)))
        }
        ["anchor", index] => Ok(pair_text(anchor(
            index.parse().map_err(|e| format!("index: {e:?}"))?,
        ))),
        _ => Err("expected basecase TAB P TAB x, or anchor TAB index".into()),
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
    fn replay_reports_pairs_and_preserves_typed_profile_errors() {
        assert_eq!(run("basecase\t8\t0").unwrap(), "FIXED_PAIR\t256\t0");
        assert_eq!(
            run("basecase\t17\t45055").unwrap(),
            "FIXED_PAIR\t123404\t44171"
        );
        assert!(run("basecase\t401\t0")
            .unwrap()
            .starts_with("ERROR\tUnsupportedProfile"));
        assert!(run("anchor\t403")
            .unwrap()
            .starts_with("ERROR\tUnsupportedProfile"));
    }
}
