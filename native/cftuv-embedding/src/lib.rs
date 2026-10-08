//! The source-snap embedding certificate (`kernel/src/cftuv_envelope/_embedding.py`, `_compute_source_snap_embedding_certificate` and its leaf helpers), bit-equal to the Python oracle.
//!
//! The oracle counts, over EXACT `Fraction` positions before and after the snap of the source vertices to the grid: vertex pairs the snap made coincident, nonzero edges it collapsed,
//! pairs of non-adjacent edges that did not meet and now do, intended right corners it flattened, unclassifiable corners it left unchanged. It is a pure function (no budget, no memory, no
//! float): the answer is twelve integers and the sorted vertex ids, and an inconsistent physical edge is the one exception it raises.
//!
//! What this port does differently, with the same answer:
//!
//! * integers, not `Fraction`s: each map of positions is scaled to numerators over one common denominator (`scale.rs`), and every predicate is a sign or a zero test of a polynomial in
//!   them (`relation.rs`), on `i128` when the magnitude bound proves it fits and on `IBig` otherwise (`num.rs`);
//! * a box test before a relation: two segments whose boxes are apart do not meet (the relation is `NONE` in every branch of the oracle), and the pair loop only needs
//!   `before == NONE and after != NONE`, so the relation of the snapped segments is asked first and the one of the original segments only when the snapped ones meet;
//! * the vertex-pair count is the number of pairs equal after the snap minus the number equal before AND after, from one sort (the oracle loops over all pairs).
//!
//! Failures: [`Failure::InconsistentEndpoints`] is the oracle's `ValueError`; [`Failure::MissingVertex`] is a vertex the positions lack (the oracle's `KeyError`), which the port does not answer
//! (the caller lets the oracle raise it).

mod edges;
pub mod num;
pub mod relation;
pub mod scale;

use std::cmp::Ordering;

use dashu_int::IBig;
use num::{Num, FIXED_BITS};
use relation::{corner_is_flat, relation3, Bounds, NONE};

pub use scale::{RPoint, Rational};

/// One face as the oracle reads it: the value of its id and its two cycles, as numbers (`Input::edge_names` for the edges, vertex numbers for the vertices).
#[derive(Clone, Debug)]
pub struct Face {
    pub key: String,
    pub vertices: Vec<u32>,
    pub edges: Vec<u32>,
}

/// The positions after the snap: the very same map (`UNSNAPPED_EXACT_V1`: `after is before`) or their own.
#[derive(Clone, Debug)]
pub enum After {
    Same,
    Points(Vec<RPoint>),
}

/// A corner: three vertex numbers, `None` for a vertex that is not in the positions.
pub type Corner = [Option<u32>; 3];

/// The whole input of one call, numbered. Vertices of `before` are numbered `0..before.len()` in the order of their ids' values (UTF-8 byte order, which is `str` order); a face vertex the
/// positions lack has a number at or above `before.len()` (the same vertex has the same number). `after`, when it is its own map, covers exactly the vertices of `before`, in the same order.
#[derive(Clone, Debug)]
pub struct Input {
    pub before: Vec<RPoint>,
    pub after: After,
    pub faces: Vec<Face>,
    pub edge_names: Vec<String>,
    pub intended: Vec<Corner>,
    pub unclassifiable: Vec<Corner>,
}

/// The counts of the certificate that the oracle computes (the other four fields are lengths the caller knows).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Counts {
    pub source_edge_count: u64,
    pub newly_coincident_vertex_pair_count: u64,
    pub collapsed_nonzero_source_edge_count: u64,
    pub new_nonadjacent_edge_intersection_count: u64,
    pub unchanged_unclassifiable_source_corner_count: u64,
    pub degenerated_intended_right_corner_count: u64,
    pub exact_pair_test_count: u64,
}

/// How a map was run: `i128` or `IBig`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tier {
    Fixed,
    Big,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Report {
    pub counts: Counts,
    pub before_tier: Tier,
    pub after_tier: Tier,
    /// `after` holds the same values as `before` at every vertex: the snap changed nothing and no relation was asked.
    pub unchanged: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Failure {
    /// The oracle's `ValueError("physical edge <value> has inconsistent endpoints")`; `edge` numbers `Input::edge_names`.
    InconsistentEndpoints { edge: u32 },
    /// A vertex of an edge is not in the positions: the oracle raises `KeyError`; the port declines.
    MissingVertex,
}

fn tier_of(bits: usize) -> Tier {
    if bits <= FIXED_BITS {
        Tier::Fixed
    } else {
        Tier::Big
    }
}

fn arrays<N: Num>(scaled: &scale::Scaled) -> Vec<[N; 3]> {
    scaled
        .coords
        .iter()
        .map(|point| [0, 1, 2].map(|k| N::from_big(&point[k]).expect("the magnitude bound says the coordinate fits")))
        .collect()
}

fn pairs(count: u64) -> u64 {
    count * count.saturating_sub(1) / 2
}

/// Everything the loops need, in the representations they run on.
struct Run<'a, B: Num, A: Num> {
    before: Vec<[B; 3]>,
    after: Vec<[A; 3]>,
    same: &'a [bool],
    edges: &'a [[u32; 2]],
    unchanged: bool,
}

/// `newly_coincident_vertex_pair_count`: pairs equal after the snap and different before it. One sort by `(after, before)`: pairs equal after = sum of `C(n, 2)` over the runs of equal
/// `after`; pairs equal in both = the same over the runs of equal `(after, before)`.
fn coincident_pairs<B: Num, A: Num>(run: &Run<B, A>) -> u64 {
    if run.unchanged {
        return 0;
    }
    let mut order: Vec<usize> = (0..run.before.len()).collect();
    order.sort_unstable_by(|&left, &right| run.after[left].cmp(&run.after[right]).then_with(|| run.before[left].cmp(&run.before[right])));
    let (mut total, mut start) = (0u64, 0usize);
    while start < order.len() {
        let mut end = start + 1;
        while end < order.len() && run.after[order[end]] == run.after[order[start]] {
            end += 1;
        }
        let mut same_before = 0u64;
        let mut inner = start;
        while inner < end {
            let mut stop = inner + 1;
            while stop < end && run.before[order[stop]] == run.before[order[inner]] {
                stop += 1;
            }
            same_before += pairs((stop - inner) as u64);
            inner = stop;
        }
        total += pairs((end - start) as u64) - same_before;
        start = end;
    }
    total
}

fn collapsed_edges<B: Num, A: Num>(run: &Run<B, A>) -> u64 {
    if run.unchanged {
        return 0;
    }
    run.edges
        .iter()
        .filter(|[start, end]| run.before[*start as usize] != run.before[*end as usize] && run.after[*start as usize] == run.after[*end as usize])
        .count() as u64
}

/// `(non-adjacent pairs, new intersections)`: the pairs of edges that share no endpoint, and those among them whose relation was `NONE` and is not any more.
fn edge_pairs<B: Num, A: Num>(run: &Run<B, A>) -> (u64, u64) {
    let (mut nonadjacent, mut fresh) = (0u64, 0u64);
    let boxes_after: Vec<Bounds<A>> = if run.unchanged { Vec::new() } else { run.edges.iter().map(|[s, e]| Bounds::of(&run.after[*s as usize], &run.after[*e as usize])).collect() };
    let mut boxes_before: Vec<Option<Bounds<B>>> = Vec::new();
    if !run.unchanged {
        boxes_before.resize_with(run.edges.len(), || None);
    }
    for (left_index, [a, b]) in run.edges.iter().enumerate() {
        for (offset, [c, d]) in run.edges[left_index + 1..].iter().enumerate() {
            if a == c || a == d || b == c || b == d {
                continue;
            }
            nonadjacent += 1;
            if run.unchanged {
                continue;
            }
            let right_index = left_index + 1 + offset;
            if boxes_after[left_index].apart_from(&boxes_after[right_index]) {
                continue;
            }
            let (pa, pb, pc, pd) = (&run.after[*a as usize], &run.after[*b as usize], &run.after[*c as usize], &run.after[*d as usize]);
            if relation3(pa, pb, pc, pd) == NONE {
                continue;
            }
            let (qa, qb, qc, qd) = (&run.before[*a as usize], &run.before[*b as usize], &run.before[*c as usize], &run.before[*d as usize]);
            let left_box = boxes_before[left_index].get_or_insert_with(|| Bounds::of(qa, qb)).clone();
            let right_box = boxes_before[right_index].get_or_insert_with(|| Bounds::of(qc, qd));
            if left_box.apart_from(right_box) || relation3(qa, qb, qc, qd) == NONE {
                fresh += 1;
            }
        }
    }
    (nonadjacent, fresh)
}

fn degenerated_corners<B: Num, A: Num>(run: &Run<B, A>, corners: &[Corner]) -> u64 {
    corners
        .iter()
        .filter(|corner| match corner {
            [Some(previous), Some(vertex), Some(following)] => corner_is_flat(&run.after[*previous as usize], &run.after[*vertex as usize], &run.after[*following as usize]),
            _ => true,
        })
        .count() as u64
}

fn unchanged_corners<B: Num, A: Num>(run: &Run<B, A>, corners: &[Corner]) -> u64 {
    corners.iter().filter(|corner| corner.iter().all(|item| item.is_some_and(|vertex| run.same[vertex as usize]))).count() as u64
}

fn analyse<B: Num, A: Num>(run: &Run<B, A>, input: &Input, vertices: u64) -> Counts {
    let (nonadjacent, fresh) = edge_pairs(run);
    Counts {
        source_edge_count: run.edges.len() as u64,
        newly_coincident_vertex_pair_count: coincident_pairs(run),
        collapsed_nonzero_source_edge_count: collapsed_edges(run),
        new_nonadjacent_edge_intersection_count: fresh,
        unchanged_unclassifiable_source_corner_count: unchanged_corners(run, &input.unclassifiable),
        degenerated_intended_right_corner_count: degenerated_corners(run, &input.intended),
        exact_pair_test_count: pairs(vertices) + nonadjacent,
    }
}

/// The certificate's counts for one call, or the oracle's `ValueError` / a vertex the port does not answer for.
pub fn compute(input: &Input) -> Result<Report, Failure> {
    let vertices = input.before.len();
    let known = u32::try_from(vertices).expect("a vertex number fits u32");
    let edges = edges::source_edges(&input.faces, &input.edge_names, known)?;
    let before = scale::scale(&input.before);
    let after = match &input.after {
        After::Same => None,
        After::Points(points) => {
            assert_eq!(points.len(), vertices, "`after` covers the vertices of `before`");
            Some(scale::scale(points))
        }
    };
    let same: Vec<bool> = match &after {
        None => vec![true; vertices],
        Some(after) => (0..vertices).map(|index| scale::same_value(&before, after, index)).collect(),
    };
    let unchanged = same.iter().all(|item| *item);
    let after_scaled = after.as_ref().unwrap_or(&before);
    let (before_tier, after_tier) = (tier_of(before.bits), tier_of(after_scaled.bits));
    let counts = match (before_tier, after_tier) {
        (Tier::Fixed, Tier::Fixed) => analyse(&Run { before: arrays::<i128>(&before), after: arrays::<i128>(after_scaled), same: &same, edges: &edges, unchanged }, input, vertices as u64),
        (Tier::Fixed, Tier::Big) => analyse(&Run { before: arrays::<i128>(&before), after: arrays::<IBig>(after_scaled), same: &same, edges: &edges, unchanged }, input, vertices as u64),
        (Tier::Big, Tier::Fixed) => analyse(&Run { before: arrays::<IBig>(&before), after: arrays::<i128>(after_scaled), same: &same, edges: &edges, unchanged }, input, vertices as u64),
        (Tier::Big, Tier::Big) => analyse(&Run { before: arrays::<IBig>(&before), after: arrays::<IBig>(after_scaled), same: &same, edges: &edges, unchanged }, input, vertices as u64),
    };
    Ok(Report { counts, before_tier, after_tier, unchanged })
}

/// Python's order of two ids' values, for callers that number vertices: the order of the UTF-8 bytes.
pub fn id_order(left: &str, right: &str) -> Ordering {
    left.as_bytes().cmp(right.as_bytes())
}
