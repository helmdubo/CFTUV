//! Test-only replay oracle callback journal: никакого default set/sort здесь нет.
//! Строки TAB: CASE id cap stop mode windows; W chart sign четыре rational;
//! TRACE; L/C/E (вход и ответ), I (set order), S (sort output); EXPECT; END.
//! Rust выполняет shell/search/budget/visit, внешний журнал — только predicates
//! и явно ещё НЕ перенесённый CPython порядок. Не продуктовый ABI.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::io::{self, BufRead};
use std::rc::Rc;

use cftuv_density::farey::{
    Endpoint, FareyFailure, FareyOrdering, FareyPredicates, PrimitiveVector, SearchMode,
    SearchResult, SingleChartWindow, WorkBudget, search,
};
use cftuv_density::{IBig, Rational, UBig};

#[derive(Default)]
struct Journal {
    rows: VecDeque<Vec<String>>,
    consumed: usize,
}

impl Journal {
    fn next(&mut self, kind: &str) -> Result<Vec<String>, String> {
        let row = self
            .rows
            .pop_front()
            .ok_or_else(|| format!("missing {kind}"))?;
        if row.first().map(String::as_str) != Some(kind) {
            return Err(format!("expected {kind}, received {row:?}"));
        }
        self.consumed += 1;
        Ok(row)
    }
}

fn vector_text(vector: &PrimitiveVector) -> String {
    format!("{},{}", vector.0, vector.1)
}

fn vectors_text(vectors: &[PrimitiveVector]) -> String {
    if vectors.is_empty() {
        "-".into()
    } else {
        vectors
            .iter()
            .map(vector_text)
            .collect::<Vec<_>>()
            .join(";")
    }
}

fn parse_vector(text: &str) -> Result<PrimitiveVector, String> {
    let (x, y) = text.split_once(',').ok_or("invalid vector")?;
    Ok(PrimitiveVector(
        x.parse().map_err(|_| "invalid x")?,
        y.parse().map_err(|_| "invalid y")?,
    ))
}

fn permutation(items: &[PrimitiveVector], text: &str) -> Result<Vec<usize>, String> {
    if text == "-" {
        return Ok(Vec::new());
    }
    text.split(';')
        .map(|item| {
            let vector = parse_vector(item)?;
            items
                .iter()
                .position(|candidate| *candidate == vector)
                .ok_or_else(|| format!("foreign vector {item}"))
        })
        .collect()
}

fn endpoint_text(endpoint: Endpoint<'_>) -> String {
    match endpoint {
        Endpoint::Incoming => "IN".into(),
        Endpoint::Outgoing => "OUT".into(),
        Endpoint::Primitive(vector) => vector_text(vector),
    }
}

fn answer(text: &str) -> Result<i8, String> {
    if let Some(message) = text.strip_prefix('!') {
        Err(message.into())
    } else {
        text.parse().map_err(|_| "invalid answer".into())
    }
}

struct Predicates(Rc<RefCell<Journal>>);
struct Ordering(Rc<RefCell<Journal>>);

fn check(row: &[String], expected: &[String]) -> Result<(), String> {
    if row[..expected.len()] == *expected {
        Ok(())
    } else {
        Err(format!("callback inputs differ: {row:?} vs {expected:?}"))
    }
}

impl FareyPredicates for Predicates {
    type Error = String;

    fn local_candidate(
        &mut self,
        ordinal: usize,
        vector: &PrimitiveVector,
    ) -> Result<bool, String> {
        let row = self.0.borrow_mut().next("L")?;
        check(
            &row,
            &["L".into(), ordinal.to_string(), vector_text(vector)],
        )?;
        Ok(answer(&row[3])? != 0)
    }

    fn candidate_compare(
        &mut self,
        ordinal: usize,
        left: &PrimitiveVector,
        right: &PrimitiveVector,
    ) -> Result<i8, String> {
        let row = self.0.borrow_mut().next("C")?;
        check(
            &row,
            &[
                "C".into(),
                ordinal.to_string(),
                vector_text(left),
                vector_text(right),
            ],
        )?;
        answer(&row[4])
    }

    fn edge_valid(
        &mut self,
        ordinal: usize,
        left: Endpoint<'_>,
        right: Endpoint<'_>,
    ) -> Result<bool, String> {
        let row = self.0.borrow_mut().next("E")?;
        check(
            &row,
            &[
                "E".into(),
                ordinal.to_string(),
                endpoint_text(left),
                endpoint_text(right),
            ],
        )?;
        Ok(answer(&row[4])? != 0)
    }
}

impl FareyOrdering<Predicates> for Ordering {
    fn iteration_order(
        &mut self,
        ordinal: usize,
        items: &[PrimitiveVector],
    ) -> Result<Vec<usize>, String> {
        let row = self.0.borrow_mut().next("I")?;
        check(&row, &["I".into(), ordinal.to_string()])?;
        permutation(items, &row[2])
    }

    fn sort(
        &mut self,
        ordinal: usize,
        items: &[PrimitiveVector],
        predicates: &mut Predicates,
    ) -> Result<Vec<usize>, String> {
        // Не реализация сортировки: исполняются ВОПРОСЫ Python-журнала в его
        // порядке; первый отказ обрывает их, последующий S уже не читается.
        loop {
            let next = self
                .0
                .borrow()
                .rows
                .front()
                .cloned()
                .ok_or("missing sort output")?;
            if next[0] != "C" {
                break;
            }
            let left = parse_vector(&next[2])?;
            let right = parse_vector(&next[3])?;
            if !items.contains(&left) || !items.contains(&right) {
                return Err("foreign comparator input".into());
            }
            predicates.candidate_compare(ordinal, &left, &right)?;
        }
        let row = self.0.borrow_mut().next("S")?;
        check(&row, &["S".into(), ordinal.to_string()])?;
        permutation(items, &row[2])
    }
}

fn counts_text(counts: &[usize]) -> String {
    if counts.is_empty() {
        "-".into()
    } else {
        counts
            .iter()
            .map(usize::to_string)
            .collect::<Vec<_>>()
            .join(",")
    }
}

fn result_text(result: Result<SearchResult, FareyFailure<String>>, budget: &WorkBudget) -> String {
    match result {
        Ok(result) => format!(
            "OK\t{}\t{}\t{}\t{}\t{}\t{}",
            result
                .height
                .map_or_else(|| "-".into(), |height| height.to_string()),
            result
                .fan
                .as_deref()
                .map_or_else(|| "-".into(), vectors_text),
            counts_text(&result.previous_counts),
            counts_text(&result.counts),
            budget.shell_probes,
            budget.order_steps
        ),
        Err(error) => {
            let reason = match error {
                FareyFailure::Work(error) => error.to_string(),
                FareyFailure::Predicate(error) => error,
                FareyFailure::AtlasNotPorted => "ATLAS_NOT_PORTED".into(),
                error => format!("{error:?}"),
            };
            format!(
                "ERR\t{reason}\t{}\t{}",
                budget.shell_probes, budget.order_steps
            )
        }
    }
}

fn rational(text: &str) -> Result<Rational, String> {
    let (n, d) = text.split_once('/').ok_or("missing rational denominator")?;
    Rational::new(
        n.parse::<IBig>().map_err(|_| "invalid numerator")?,
        d.parse::<UBig>().map_err(|_| "invalid denominator")?,
    )
    .map_err(|error| format!("{error:?}"))
}

fn run(lines: &[String]) -> Result<String, String> {
    let header: Vec<_> = lines[0].split('\t').collect();
    if header.len() != 6 || header[0] != "CASE" {
        return Err("invalid CASE".into());
    }
    let mut budget = WorkBudget::new(header[2].parse().map_err(|_| "invalid cap")?);
    let stop: UBig = header[3].parse().map_err(|_| "invalid stop")?;
    let mode = match header[4] {
        "single" => SearchMode::SingleChart,
        "atlas" => SearchMode::Atlas,
        _ => return Err("invalid mode".into()),
    };
    let count: usize = header[5].parse().map_err(|_| "invalid count")?;
    let mut windows = Vec::new();
    for line in &lines[1..=count] {
        let fields: Vec<_> = line.split('\t').collect();
        if fields.len() != 7 || fields[0] != "W" {
            return Err("invalid W".into());
        }
        windows.push(SingleChartWindow {
            use_x: fields[1] == "1",
            denominator_sign: fields[2].parse().map_err(|_| "invalid sign")?,
            sealed: [
                rational(fields[3])?,
                rational(fields[4])?,
                rational(fields[5])?,
                rational(fields[6])?,
            ],
        });
    }
    if lines[count + 1] != "TRACE" {
        return Err("missing TRACE".into());
    }
    let end = lines
        .iter()
        .position(|line| line.starts_with("EXPECT\t"))
        .ok_or("missing EXPECT")?;
    let shared = Rc::new(RefCell::new(Journal {
        rows: lines[count + 2..end]
            .iter()
            .map(|line| line.split('\t').map(str::to_owned).collect())
            .collect(),
        consumed: 0,
    }));
    let mut predicates = Predicates(shared.clone());
    let mut ordering = Ordering(shared.clone());
    let result = search(
        mode,
        &windows,
        &stop,
        &mut budget,
        &mut predicates,
        &mut ordering,
    );
    let result = result_text(result, &budget);
    let expected = lines[end].strip_prefix("EXPECT\t").unwrap();
    if result != expected {
        return Err(format!(
            "{}: result {result:?} differs from oracle {expected:?}",
            header[1]
        ));
    }
    let journal = shared.borrow();
    if !journal.rows.is_empty() {
        return Err(format!(
            "{}: {} callbacks omitted",
            header[1],
            journal.rows.len()
        ));
    }
    Ok(format!(
        "{}\t{result}\tTRACE={}",
        header[1], journal.consumed
    ))
}

fn main() {
    let mut current = Vec::new();
    for line in io::stdin().lock().lines() {
        let line = line.expect("stdin");
        if line == "END" {
            match run(&current) {
                Ok(result) => println!("{result}"),
                Err(error) => {
                    eprintln!("{error}");
                    std::process::exit(1);
                }
            }
            current.clear();
        } else if !line.is_empty() {
            current.push(line);
        }
    }
    if !current.is_empty() {
        eprintln!("unterminated CASE");
        std::process::exit(1);
    }
}
