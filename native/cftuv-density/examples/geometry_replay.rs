//! Exact geometry replay, TAB: op,id,g00,g01,g11,owner,expected,q,law,ideals,ordinal,args.
//! ideals: V2-text~V2-text|...; primitive args: x,y|x,y; neighbours/rotate are exact.
//! best args: cap:row-of-candidates/row; actual CanonicalOrdering, не transcript stub.

use std::io::{self, BufRead};

use cftuv_density::farey::{
    Endpoint, FareyFailure, FareyPredicates, PrimitiveVector, WorkBudget, best_fan,
};
use cftuv_density::geometry::{GeometryAdapter, GeometryFailure, GeometryFailureKind, WindowLaw};
use cftuv_density::order::CanonicalOrdering;
use cftuv_density::{DensityTransaction, IBig, Metric2, Radical, Rational, UBig, Vector2};

fn rational(text: &str) -> Rational {
    let (n, d) = text.split_once('/').expect("n/d");
    Rational::new(n.parse::<IBig>().unwrap(), d.parse::<UBig>().unwrap()).unwrap()
}

fn scalar(text: &str) -> Radical {
    Radical::from_text(text).expect("V2 text").0
}

fn ideal(text: &str) -> Vec<Vector2> {
    text.split('|')
        .map(|item| {
            let (x, y) = item.split_once('~').expect("ideal coordinates");
            Vector2::new(scalar(x), scalar(y))
        })
        .collect()
}

fn primitive(text: &str) -> PrimitiveVector {
    let (x, y) = text.split_once(',').expect("primitive pair");
    PrimitiveVector(x.parse().unwrap(), y.parse().unwrap())
}

fn endpoint<'a>(text: &str, storage: &'a mut Option<PrimitiveVector>) -> Endpoint<'a> {
    match text {
        "IN" => Endpoint::Incoming,
        "OUT" => Endpoint::Outgoing,
        text => {
            *storage = Some(primitive(text));
            Endpoint::Primitive(storage.as_ref().unwrap())
        }
    }
}

fn vector_text(vector: &Vector2) -> String {
    format!("{}~{}", vector.x.text(), vector.y.text())
}
fn primitive_text(vector: &PrimitiveVector) -> String {
    format!("{},{}", vector.0, vector.1)
}

fn failure(error: GeometryFailure) -> String {
    match error.kind {
        GeometryFailureKind::ZeroSlopeDenominator => "ERROR:ZeroSlopeDenominator".into(),
        kind => format!("INTERNAL:{kind:?}"),
    }
}

fn run(fields: &[&str]) -> Result<(String, GeometryAdapter), String> {
    if fields.len() != 12 {
        return Err(format!("expected12fields, got{}", fields.len()));
    }
    let a = rational(fields[2]);
    let b = rational(fields[3]);
    let c = rational(fields[4]);
    let owner = fields[5].parse().unwrap();
    let (metric, _) = Metric2::from_gram([[a, b.clone()], [b, c]], owner)
        .map_err(|error| format!("{error:?}"))?;
    let law = match fields[8] {
        "voronoi" => WindowLaw::Voronoi,
        "narrow" => WindowLaw::NarrowBand,
        _ => return Err("invalid law".into()),
    };
    let mut geometry = GeometryAdapter::new(
        DensityTransaction::new(metric),
        ideal(fields[9]),
        fields[7].parse().unwrap(),
        fields[6].parse().unwrap(),
        law,
    )
    .map_err(failure)?;
    let ordinal = fields[10].parse().unwrap();
    let args = fields[11];
    let answer = match fields[0] {
        "inside" => geometry
            .inside_ordinal(ordinal, &primitive(args))
            .map(|value| format!("BOOL:{}", u8::from(value))),
        "local" => geometry
            .local_candidate(ordinal, &primitive(args))
            .map(|value| format!("BOOL:{}", u8::from(value))),
        "objective" => geometry
            .objective(ordinal, &primitive(args))
            .map(|value| format!("VALUE:{}", value.text())),
        "compare" => {
            let (left, right) = args.split_once('|').unwrap();
            geometry
                .candidate_compare(ordinal, &primitive(left), &primitive(right))
                .map(|value| format!("SIGN:{value}"))
        }
        "edge" => {
            let (left, right) = args.split_once('|').unwrap();
            let (mut a, mut b) = (None, None);
            geometry
                .edge_valid(ordinal, endpoint(left, &mut a), endpoint(right, &mut b))
                .map(|value| format!("BOOL:{}", u8::from(value)))
        }
        "neighbours" => geometry
            .window_neighbours(ordinal)
            .map(|(a, b)| format!("VECTORS:{}|{}", vector_text(&a), vector_text(&b))),
        "bandinside" => {
            let center = geometry.ideal_covectors()[ordinal].clone();
            let real = geometry.ideal_covectors()[args.parse::<usize>().unwrap()].clone();
            geometry
                .neighbour_is_inside_double_band(&center, &real)
                .map(|value| format!("BOOL:{}", u8::from(value)))
        }
        "rotate" => {
            let center = geometry.ideal_covectors()[ordinal].clone();
            geometry
                .rotated_by_double_band(&center, args.parse().unwrap())
                .map(|value| format!("VECTOR:{}", vector_text(&value)))
        }
        "best" => {
            let (cap, rows) = args.split_once(':').unwrap();
            let candidates = if rows == "-" {
                Vec::new()
            } else {
                rows.split('/')
                    .map(|row| row.split(';').map(primitive).collect())
                    .collect()
            };
            let mut budget = WorkBudget::new(cap.parse().unwrap());
            let result = best_fan(
                &candidates,
                &mut budget,
                &mut geometry,
                &mut CanonicalOrdering,
            );
            let answer = match result {
                Ok(Some(fan)) => format!(
                    "FAN:{}",
                    if fan.is_empty() {
                        "-".into()
                    } else {
                        fan.iter().map(primitive_text).collect::<Vec<_>>().join(";")
                    }
                ),
                Ok(None) => "FAN:NONE".into(),
                Err(FareyFailure::Work(error)) => format!("BUDGET:{error}"),
                Err(FareyFailure::Predicate(error)) => failure(error),
                Err(error) => format!("INTERNAL:{error:?}"),
            };
            Ok(format!(
                "{answer}\tSEARCH_WORK:{},{}",
                budget.shell_probes, budget.order_steps
            ))
        }
        op => return Err(format!("unknown operation{op}")),
    };
    Ok((answer.unwrap_or_else(failure), geometry))
}

fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        if line.is_empty() {
            continue;
        }
        let fields: Vec<_> = line.split('\t').collect();
        match run(&fields) {
            Ok((answer, geometry)) => println!(
                "{}\t{}\t{}\tV2_WORK:{:?}",
                fields[0],
                fields[1],
                answer,
                geometry.work().wire()
            ),
            Err(error) => {
                eprintln!("{error}");
                std::process::exit(1);
            }
        }
    }
}
