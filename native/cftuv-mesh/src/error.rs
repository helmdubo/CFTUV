//! What the exact mesh can refuse with. The host shim raises the matching Python exception and composes the texts that need Python objects
//! (the `repr` of an owner tuple, the `Decimal` text of a station): the native side never formats a text Python compares except where it
//! is built from plain integers and names.
//!
//! Every refusal is NAMED (rule 4 of `AGENTS.md`): there is no silent fallback. `Unsupported` is the one refusal the oracle does not have: the
//! port was asked for something it does not carry; the host decides what to do, it is never an answer.

use cftuv_clip::error::ClipError;
use cftuv_core::exact::ExactError;
use cftuv_core::sqrt_sum::SqrtSum;

/// What a failed area check is about (the `parts` word of `_closes_the_area`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Closure {
    TriangleAreas,
    QuadAreas,
    FanPolygonAreas,
    PolygonAreas,
    RunPartAreas,
}

impl Closure {
    /// The words of the oracle's text: `owner ...: {parts} differ from the face`.
    pub fn parts(self) -> &'static str {
        match self {
            Closure::TriangleAreas => "triangle areas",
            Closure::QuadAreas => "quad areas",
            Closure::FanPolygonAreas => "fan polygon areas",
            Closure::PolygonAreas => "polygon areas",
            Closure::RunPartAreas => "run part areas",
        }
    }
}

/// Whose `owner` a text names: a merged face, or one part of its merged run (`FrameFaceV1.face.parts`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Whose {
    pub face: usize,
    pub part: Option<usize>,
}

#[derive(Debug, Clone)]
pub enum MeshError {
    /// Budget exhaustion and the other refusals of the exact layer.
    Exact(ExactError),
    /// `VERTEX_KEY_COLLISION`: `key` names two different points (`region` is the coverage region, `place` is `node (x, y)` or `its chord station`).
    VertexKeyCollision { key: String, region: String, place: String },
    /// `STATION_VALUE_CONFLICT`: vertex `key` of the layout region `region` has two `(s, r)` answers from the owner of `face`. `answered` is the
    /// oracle's `(*given.items(), (run_id, value))` in order (the new answer last), and `station_conflict` is `(run_id, *given)`.
    StationConflict { key: u32, region: u32, face: usize, answered: Vec<(String, SqrtSum, SqrtSum)> },
    /// `TESSELLATION_DID_NOT_CLOSE`: `owner {owner}: {parts} differ from the face`.
    NotClosed { whose: Whose, what: Closure },
    /// `TESSELLATION_DID_NOT_CLOSE`: `owner {owner}: {points} contour points` (no triangulation).
    NoTriangulation { whose: Whose, points: usize },
    /// The native port does not cover this input (never an answer; the host must see it).
    Unsupported(String),
}

impl From<ExactError> for MeshError {
    fn from(error: ExactError) -> MeshError {
        MeshError::Exact(error)
    }
}

impl From<ClipError> for MeshError {
    fn from(error: ClipError) -> MeshError {
        match error {
            ClipError::Exact(error) => MeshError::Exact(error),
            other => MeshError::Unsupported(format!("a predicate of the clip port answered {other:?}")),
        }
    }
}

pub type MeshResult<T> = Result<T, MeshError>;
