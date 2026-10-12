//! The input and the output of the exact mesh in native form: what `FrameFaceV1`, `CoveredFaceV1`, `StationRunV1`, `SupportLineV1` and
//! `ChainStationTableV1` carry that `mesh.py` reads, and what it answers.

use std::collections::HashMap;
use std::rc::Rc;

use cftuv_canon::fxhash::FxBuild;
use cftuv_clip::point::Point;
use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::tally::Tally;

/// `SupportLineV1`: the integer line `a x + b y = c` and the squared speed `q` of its front (an `int` or a `Fraction`; only the value is read).
#[derive(Debug, Clone)]
pub struct Line {
    pub a: Rat,
    pub b: Rat,
    pub c: SqrtSum,
    pub q: Rat,
}

/// `StationRunV1`: the one formula `s = s_origin + <p - origin, direction>_G / |direction|_G` of a run of collinear edges, as `station_of` reads it.
#[derive(Debug, Clone)]
pub struct Run {
    pub id: String,
    /// `SqrtSumV1.rational(origin[i])`, made once.
    pub origin: (SqrtSum, SqrtSum),
    pub covector: (Rat, Rat),
    pub s_origin: SqrtSum,
    pub inverse_length: SqrtSum,
}

/// A part of a merged run (`CoveredFaceV1.parts[i]`): its contour and its doubled area.
#[derive(Debug, Clone)]
pub struct Part {
    pub points: Vec<Point>,
    pub area: SqrtSum,
}

/// One merged face with its frame (`FrameFaceV1` and the `CoveredFaceV1` inside it).
#[derive(Debug, Clone)]
pub struct Face {
    /// `layout.region_of(frame_face)`: the number of the face's region in the layout.
    pub region: u32,
    pub points: Vec<Point>,
    /// `face.doubled_area`.
    pub area: SqrtSum,
    /// `owner[0], owner[1]` as integers (the node of a fan); `None` when the owner has fewer than two items or they are not integers.
    pub apex: Option<(IBig, IBig)>,
    /// `face.parts` when it has at least two (`_rung_pieces` and the `MERGED_RUNS_*` numbers read nothing else of it).
    pub parts: Vec<Part>,
    pub line: Line,
    pub run: Rc<Run>,
    /// `fan_station` (`Some` makes the face a fan, `FrameFaceV1.is_fan`).
    pub fan_station: Option<SqrtSum>,
    /// `flow_key is not None`.
    pub in_flow: bool,
}

impl Face {
    pub fn is_fan(&self) -> bool {
        self.fan_station.is_some()
    }
}

/// `ChainStationTableV1`: what `station_values` and `intern_vertices` read of it.
#[derive(Debug, Default)]
pub struct Table {
    /// `node_vertex_ids[(region_id, node)]`: region -> node -> vertex id.
    pub node_vertex_ids: HashMap<String, HashMap<(IBig, IBig), String>, FxBuild>,
    /// `joins[(run_a, run_b)]`.
    pub joins: HashMap<(String, String), SqrtSum, FxBuild>,
    /// `frame_of_run`.
    pub frame_of_run: HashMap<String, String, FxBuild>,
}

impl Table {
    /// `table.join_station(run_a, run_b)`.
    pub fn join_station(&self, run_a: &str, run_b: &str) -> Option<&SqrtSum> {
        let found = self.joins.get(&(run_a.to_string(), run_b.to_string()));
        match found {
            Some(_) => found,
            None => self.joins.get(&(run_b.to_string(), run_a.to_string())),
        }
    }

    /// `table.cut_join_station(run_a, run_b)`: the join of two runs that lie in DIFFERENT frames, else `None`.
    pub fn cut_join_station(&self, run_a: &str, run_b: &str) -> Option<&SqrtSum> {
        if self.frame_of_run.get(run_a) == self.frame_of_run.get(run_b) {
            return None;
        }
        self.join_station(run_a, run_b)
    }
}

/// The names of the vertices at chord stations (`chord_names[(region_id, point_key)]`): a handful at most, searched by point equality.
#[derive(Debug, Default)]
pub struct ChordNames {
    pub entries: Vec<(String, Point, String)>,
}

/// The law of the tessellation (`DecalTopologyLawV1`), as the code the shim passes: 0 planar polygons, 1 quad strips, 2 any other (triangles).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Law {
    PlanarPolygons,
    QuadStrips,
    Triangles,
}

/// Everything `exact_mesh` reads.
pub struct MeshInput<'a> {
    pub faces: &'a [Face],
    /// `region_ids[i]`: the coverage region of face `i` (the key of the table's node names).
    pub region_ids: &'a [String],
    pub table: &'a Table,
    pub chords: &'a ChordNames,
    pub lattice_alpha: &'a Rat,
    pub chart_cw: bool,
    pub law: Law,
    pub exact_plane: bool,
}

/// One interned vertex name: `src:<vertex id>` or `node:<k>`, and the first point that carried it.
#[derive(Debug, Clone)]
pub struct Key {
    pub name: String,
    /// `(face, vertex)` of the point the dictionary `points` holds for this key.
    pub first: (u32, u32),
}

/// What `exact_mesh` answers.
#[derive(Debug, Default)]
pub struct MeshOutput {
    pub keys: Vec<Key>,
    /// `cycles[face]`: `(key, vertex of the face)` without the repeats `_cycle` drops.
    pub cycles: Vec<Vec<(u32, u32)>>,
    /// `facts` in dictionary order: `((layout region, key), (s, r))`.
    pub facts: Vec<((u32, u32), Rc<(SqrtSum, SqrtSum)>)>,
    /// `polygons[face]`: faces as lists of key indices.
    pub polygons: Vec<Vec<Vec<u32>>>,
    pub tally: Tally,
    /// The `(region, key)` of the vertices that got a rung station.
    pub rungs: Vec<(u32, u32)>,
    /// The lines `SOURCE_VERTEX_NAME_DROPPED` appended to `notes`.
    pub notes: Vec<String>,
}
