//! Direct calls of the factorization layer against the Python oracle: answer, six budget articles and the
//! exhaustion point (operation, radicand), unlimited and under caps around every spend boundary.

mod common;

use cftuv_canon::factor::{coprime_basis, is_prime, pollard_rho, pollard_rho_brent_attempt, rho_factors, CanonError};
use cftuv_canon::{Exhausted, WorkBudget};
use common::*;
use dashu_int::UBig;

fn pairs_json(pairs: &[(UBig, u64)]) -> Json {
    jarr(pairs.iter().map(|(prime, power)| jarr(vec![jhex(prime), jint(*power)])).collect())
}

fn exhausted_json(error: &Exhausted) -> Json {
    jarr(vec![jstr("exh"), jstr(error.operation.as_str()), jhex(&error.radicand)])
}

fn outcome<T>(result: Result<T, CanonError>, ok: impl FnOnce(T) -> Json) -> Json {
    match result {
        Ok(value) => jarr(vec![jstr("ok"), ok(value)]),
        Err(CanonError::Exhausted(error)) => exhausted_json(&error),
        Err(other) => panic!("unexpected refusal {other:?}"),
    }
}

fn call(function: &str, args: &Json, budget: &mut WorkBudget) -> Json {
    match function {
        "is_prime" => outcome(is_prime(&ub(args.get("n").str()), budget).map_err(CanonError::from), Json::Bool),
        "rho" => outcome(pollard_rho(&ub(args.get("n").str()), budget), |divisor| jhex(&divisor)),
        "brent" => outcome(
            pollard_rho_brent_attempt(
                &ub(args.get("n").str()),
                &ub(args.get("y").str()),
                &ub(args.get("c").str()),
                args.get("batch").int() as u64,
                budget,
            ),
            |divisor| divisor.map_or(Json::Null, |value| jhex(&value)),
        ),
        "rho_factors" => outcome(rho_factors(&ub(args.get("n").str()), budget), |pairs| pairs_json(&pairs)),
        "coprime" => {
            let values: Vec<UBig> = args.get("values").arr().iter().map(|item| ub(item.str())).collect();
            outcome(coprime_basis(&values, budget).map_err(CanonError::from), |basis| {
                jarr(basis.iter().map(jhex).collect())
            })
        }
        other => panic!("unknown function {other}"),
    }
}

fn check_file(name: &str) -> usize {
    let vectors = load_vectors(name);
    let mut runs_checked = 0;
    for case in vectors.get("cases").arr() {
        let function = case.get("fn").str();
        let args = case.get("args");
        for run in case.get("runs").arr() {
            let mut budget = match run.get("cap") {
                Json::Int(cap) => WorkBudget::bounded(*cap as u64),
                _ => WorkBudget::unlimited(),
            };
            let result = call(function, args, &mut budget);
            let articles = jarr(budget.articles().iter().map(|article| jint(*article)).collect());
            assert_eq!(&result, run.get("r"), "{name}/{function} {} cap {}", args.text(), run.get("cap").text());
            assert_eq!(&articles, run.get("a"), "{name}/{function} {} cap {} articles", args.text(), run.get("cap").text());
            runs_checked += 1;
        }
    }
    runs_checked
}

#[test]
fn miller_rabin_matches_the_oracle() {
    assert!(check_file("primality") > 500);
}

#[test]
fn pollard_brent_matches_the_oracle() {
    assert!(check_file("brent") > 1000);
}

#[test]
fn rho_factors_matches_the_oracle() {
    assert!(check_file("rho_factors") > 100);
}

#[test]
fn coprime_basis_matches_the_oracle() {
    assert!(check_file("coprime") > 20);
}
