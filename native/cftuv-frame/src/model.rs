//! What goes in and what comes out of the frame admission, and the named declines.
//!
//! The port is ACCEPT-OR-DECLINE (see the crate note): [`crate::resolve`] returns the frame the Python builder would return, or a [`Decline`] that names why it did not. A decline is always safe: the
//! caller runs the Python builder, which then produces whatever it produces (the metric, a near-planar projection, an unfolding, a refusal with its text, an exception).

use std::fmt;

use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;

/// One face of the owner patch as the oracle reads it: the value of its id and its two cycles, as numbers (vertex numbers are ranks in the sorted values of the ids, edge numbers index
/// [`Input::edge_names`]).
#[derive(Clone, Debug)]
pub struct FaceIn {
    pub key: String,
    pub vertices: Vec<u32>,
    pub edges: Vec<u32>,
}

/// The whole input of one call, numbered. `positions[i]` is the position of the vertex of rank `i` among `required_ids` (the vertices of the owner faces, sorted by the value of their id: UTF-8
/// byte order, which is the order of Python's `str`); `faces` are the owner faces sorted by the value of their id.
#[derive(Clone, Debug)]
pub struct Input {
    pub positions: Vec<[f64; 3]>,
    pub faces: Vec<FaceIn>,
    pub edge_names: Vec<String>,
    /// `grid_policy.snaps_source` (`SOURCE_ONLY_GRID_SNAP_V1` and `INTEGER_GRID_SNAP_V1` move the source the same way). A law that does not snap is declined by name.
    pub snaps_source: bool,
}

/// One scale the search tried (`GridScaleTrialV1`): its scale `2^exponent`, how many intended right corners it restored, and whether it passed (all of them).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Trial {
    pub exponent: u32,
    pub restored: u32,
    pub passed: bool,
}

/// What the grid did (`IntegerGridCertificateV1`): the window and the search. Constants of the oracle (`AUTHOR_ANGULAR_ERROR`, `DECAL_DETAIL`, `GridSpecV1.magnitude_bound`) are in [`crate::grid`].
#[derive(Clone, Debug)]
pub struct GridFacts {
    pub extent: Rat,
    pub window_lower: Rat,
    pub window_upper: Rat,
    /// The chosen scale is `2^exponent` (the last trial; the step is its inverse).
    pub exponent: u32,
    pub intended_right_corners: u32,
    pub trials: Vec<Trial>,
}

/// The frame of an exactly planar owner patch (`RationalAffinePlanarMetricV2` with an `ExactSourcePlaneCertificateV1` on the canonical source-vertex basis).
#[derive(Clone, Debug)]
pub struct Frame {
    pub grid: GridFacts,
    /// The canonical primitive plane normal (integers, gcd 1, first non-zero coordinate positive).
    pub normal: [IBig; 3],
    pub origin: [Rat; 3],
    pub basis_a: [Rat; 3],
    pub basis_b: [Rat; 3],
    /// `g00, g01, g11` (the matrix is symmetric).
    pub gram: [Rat; 3],
    /// `inverse[0][0], inverse[0][1] (= inverse[1][0]), inverse[1][1]`.
    pub inverse: [Rat; 3],
    /// `(u, v)` of every vertex, by rank.
    pub coordinates: Vec<[Rat; 2]>,
    /// `+1`: the coordinates run counter-clockwise on the owner faces, `-1`: clockwise.
    pub orientation_sign: i8,
}

/// Why the port did not produce the frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    /// A call the port does not carry: a grid law that does not snap, the plane-preserving retry law.
    Law,
    /// The input is not the canonical shape of its field (set by the reader at the boundary).
    Shape,
    /// The snapped vertices do not lie in one plane (the oracle projects them: near-planar), or the faces define no plane (the oracle unfolds).
    Plane,
    /// The grid does not exist or does not restore the intended relations, or the snap breaks the embedding (the oracle refuses by name, or tries the unfolding).
    Grid,
    /// The vertices define no canonical basis, or the chart has no oriented face (the oracle raises `ValueError`).
    Frame,
    /// A number the port does not run (out of the exactness range of its fast paths and of the declared guard).
    Range,
}

impl Reason {
    pub const fn name(self) -> &'static str {
        match self {
            Reason::Law => "LAW",
            Reason::Shape => "SHAPE",
            Reason::Plane => "PLANE",
            Reason::Grid => "GRID",
            Reason::Frame => "FRAME",
            Reason::Range => "RANGE",
        }
    }

    pub const fn code(self) -> u8 {
        match self {
            Reason::Law => 1,
            Reason::Shape => 2,
            Reason::Plane => 3,
            Reason::Grid => 4,
            Reason::Frame => 5,
            Reason::Range => 6,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Decline {
    pub reason: Reason,
    pub detail: String,
}

impl fmt::Display for Decline {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.reason.name(), self.detail)
    }
}

pub type Outcome<T> = Result<T, Decline>;

pub fn decline<T>(reason: Reason, detail: impl Into<String>) -> Outcome<T> {
    Err(Decline { reason, detail: detail.into() })
}
