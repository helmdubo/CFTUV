//! `_embedding._source_edge_occurrences`: one canonical occurrence of every physical source edge, in the oracle's order.
//!
//! Python walks the faces in the order given, makes one `_EdgeOccurrence(face key, index, edge id, start, end)` per cycle position and groups them by edge id in order of first sight
//! (`_edge_occurrence_groups`). A group whose occurrences do not all join the same two vertices (unordered) raises `ValueError` naming the FIRST such group in that order; then each group
//! gives its smallest occurrence by the key `(face key, index, edge id value, start value, end value)` and the result is sorted by that key. Strings compare as Python `str`, which is the order of
//! their UTF-8 bytes; vertices are numbered by the sorted order of their ids (`Input::before`), so a vertex number compares as its id does.

use crate::{Face, Failure};

/// Ranks of byte strings: equal strings share a rank, ranks follow the byte order.
fn ranks<'a>(items: impl Iterator<Item = &'a [u8]>) -> Vec<u32> {
    let items: Vec<&[u8]> = items.collect();
    let mut order: Vec<usize> = (0..items.len()).collect();
    order.sort_unstable_by(|&left, &right| items[left].cmp(items[right]));
    let mut out = vec![0u32; items.len()];
    let mut rank = 0u32;
    for (position, &index) in order.iter().enumerate() {
        if position > 0 && items[order[position - 1]] != items[index] {
            rank += 1;
        }
        out[index] = rank;
    }
    out
}

#[derive(Clone, Copy)]
struct Candidate {
    face: u32,
    index: u32,
    start: u32,
    end: u32,
}

impl Candidate {
    fn key(&self) -> (u32, u32, u32, u32) {
        (self.face, self.index, self.start, self.end)
    }
}

struct Group {
    pair: (u32, u32),
    inconsistent: bool,
    best: Candidate,
}

/// The canonical edges as `(start, end)` vertex numbers, sorted. `known` is the number of vertices of `before`: a vertex number at or above it is a vertex the positions do not have.
pub fn source_edges(faces: &[Face], edge_names: &[String], known: u32) -> Result<Vec<[u32; 2]>, Failure> {
    let face_rank = ranks(faces.iter().map(|face| face.key.as_bytes()));
    let edge_rank = ranks(edge_names.iter().map(|name| name.as_bytes()));
    let mut groups: Vec<Option<Group>> = (0..edge_names.len()).map(|_| None).collect();
    let mut first_sight: Vec<u32> = Vec::new();
    for (face_index, face) in faces.iter().enumerate() {
        let size = face.vertices.len();
        debug_assert_eq!(size, face.edges.len());
        for index in 0..size {
            let (start, end, edge) = (face.vertices[index], face.vertices[(index + 1) % size], face.edges[index] as usize);
            let candidate = Candidate { face: face_rank[face_index], index: index as u32, start, end };
            let pair = (start.min(end), start.max(end));
            match &mut groups[edge] {
                Some(group) => {
                    group.inconsistent |= group.pair != pair;
                    if candidate.key() < group.best.key() {
                        group.best = candidate;
                    }
                }
                slot @ None => {
                    first_sight.push(edge as u32);
                    *slot = Some(Group { pair, inconsistent: false, best: candidate });
                }
            }
        }
    }
    if let Some(&edge) = first_sight.iter().find(|&&edge| groups[edge as usize].as_ref().is_some_and(|group| group.inconsistent)) {
        return Err(Failure::InconsistentEndpoints { edge });
    }
    let mut chosen: Vec<(u32, u32, u32, u32, u32)> = first_sight
        .iter()
        .map(|&edge| {
            let best = groups[edge as usize].as_ref().expect("a first-sight edge has a group").best;
            (best.face, best.index, edge_rank[edge as usize], best.start, best.end)
        })
        .collect();
    if chosen.iter().any(|item| item.3 >= known || item.4 >= known) {
        return Err(Failure::MissingVertex);
    }
    chosen.sort_unstable();
    Ok(chosen.into_iter().map(|item| [item.3, item.4]).collect())
}
