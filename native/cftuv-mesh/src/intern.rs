//! `intern_vertices` (`mesh.py`): the key of every point of the merged faces, and the contour of every face by keys.
//!
//! One vertex per point: a lattice node that is the image of a source vertex is `src:<id>`, every other point is `node:<k>` by order of first
//! appearance. The NAME comes from the FIRST region that names the point; a later region that names it differently loses its name to the weld,
//! and that is now recorded (`SOURCE_VERTEX_NAME_DROPPED`, one note per `(point, name)`). Two different points under one `src:` name are a refusal.

use std::collections::{HashMap, HashSet};

use cftuv_canon::fxhash::FxBuild;
use cftuv_clip::point::{point_hash, same_point, Point};
use cftuv_core::num::IBig;

use crate::data::{ChordNames, Face, Key, MeshInput};
use crate::error::{MeshError, MeshResult};

/// The identity of the points seen so far (`interned`): hash buckets of entries compared by value (`same_point`), so no key is built per point.
#[derive(Default)]
pub struct Interner {
    buckets: HashMap<u64, Vec<u32>, FxBuild>,
    /// `(face, vertex, key)` of the first point of each identity.
    entries: Vec<(u32, u32, u32)>,
}

impl Interner {
    /// The entry whose point equals `point`.
    fn find(&self, faces: &[Face], point: &Point) -> Option<u32> {
        let bucket = self.buckets.get(&point_hash(point))?;
        bucket.iter().copied().find(|entry| {
            let (face, vertex, _) = self.entries[*entry as usize];
            same_point(&faces[face as usize].points[vertex as usize], point)
        })
    }

    fn add(&mut self, point: &Point, face: u32, vertex: u32, key: u32) -> u32 {
        let entry = self.entries.len() as u32;
        self.entries.push((face, vertex, key));
        self.buckets.entry(point_hash(point)).or_default().push(entry);
        entry
    }

    /// `key_of[point_key(point)]`: the key of a point that appears in the contours.
    pub fn key_of(&self, faces: &[Face], point: &Point) -> Option<u32> {
        self.find(faces, point).map(|entry| self.entries[entry as usize].2)
    }
}

/// `lattice_node(point)`: the integer coordinates of a point that lies on a lattice node.
pub fn lattice_node(point: &Point) -> Option<(IBig, IBig)> {
    let x = point.0.as_rational()?;
    let y = point.1.as_rational()?;
    if !x.value().is_integer() || !y.value().is_integer() {
        return None;
    }
    Some((x.value().numerator().clone(), y.value().numerator().clone()))
}

fn chord_name<'a>(chords: &'a ChordNames, region: &str, point: &Point) -> Option<&'a str> {
    chords.entries.iter().find(|(name, found, _)| name == region && same_point(found, point)).map(|(_, _, vertex)| vertex.as_str())
}

/// What `intern_vertices` answers besides the keys: the contours and the identity of the points (the second pass of the tessellation asks it).
pub struct Interned {
    pub interner: Interner,
}

pub fn intern_vertices(input: &MeshInput, keys: &mut Vec<Key>, cycles: &mut Vec<Vec<(u32, u32)>>, notes: &mut Vec<String>) -> MeshResult<Interned> {
    let mut interner = Interner::default();
    let mut names: HashMap<String, u32, FxBuild> = HashMap::default();
    let mut counter = 0u64;
    let mut reported: HashSet<(u32, String), FxBuild> = HashSet::default();
    for (face_index, face) in input.faces.iter().enumerate() {
        let region_id = &input.region_ids[face_index];
        let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(face.points.len());
        for (vertex, point) in face.points.iter().enumerate() {
            let node = lattice_node(point);
            let vertex_id: Option<&str> = match &node {
                Some(node) => input.table.node_vertex_ids.get(region_id).and_then(|nodes| nodes.get(node)).map(String::as_str),
                None => chord_name(input.chords, region_id, point),
            };
            let place = |node: &Option<(IBig, IBig)>| match node {
                None => "its chord station".to_string(),
                Some((x, y)) => format!("node ({x}, {y})"),
            };
            let found = interner.find(input.faces, point);
            let key = match found {
                None => {
                    let name = match vertex_id {
                        Some(id) => {
                            let name = format!("src:{id}");
                            if names.contains_key(&name) {
                                return Err(MeshError::VertexKeyCollision { key: name, region: region_id.clone(), place: place(&node) });
                            }
                            name
                        }
                        None => {
                            let name = format!("node:{counter}");
                            counter += 1;
                            name
                        }
                    };
                    let key = keys.len() as u32;
                    names.insert(name.clone(), key);
                    keys.push(Key { name, first: (face_index as u32, vertex as u32) });
                    interner.add(point, face_index as u32, vertex as u32, key);
                    key
                }
                Some(entry) => {
                    let key = interner.entries[entry as usize].2;
                    if let Some(id) = vertex_id {
                        let welded = &keys[key as usize].name;
                        let is_own = welded.strip_prefix("src:") == Some(id);
                        if !is_own && reported.insert((entry, id.to_string())) {
                            notes.push(format!("SOURCE_VERTEX_NAME_DROPPED: {id} at {} of region {region_id} is welded into {welded}", place(&node)));
                        }
                    }
                    key
                }
            };
            pairs.push((key, vertex as u32));
        }
        cycles.push(cycle_of(pairs));
    }
    Ok(Interned { interner })
}

/// `_cycle(pairs)`: no repeated key in a row, and the last item dropped when it repeats the first.
fn cycle_of(pairs: Vec<(u32, u32)>) -> Vec<(u32, u32)> {
    let mut out: Vec<(u32, u32)> = Vec::with_capacity(pairs.len());
    for item in pairs {
        if out.last().is_some_and(|last| last.0 == item.0) {
            continue;
        }
        out.push(item);
    }
    if out.len() > 1 && out[0].0 == out[out.len() - 1].0 {
        out.pop();
    }
    out
}
