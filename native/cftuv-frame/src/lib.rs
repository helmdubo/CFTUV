//! `frame`: the exact planar-grid admission of one owner patch, native (`planar_metric.build_rational_affine_planar_metric` for a patch that is exactly planar after the source snap).
//!
//! The Python builder is a ladder: the exact plane on the source snapped to the grid, then (named refusals) a near-planar projection, then an unfolding of the whole patch, then a band around the
//! chosen chains. This crate is the FIRST RUNG on the grid-snapped source, and nothing else. It returns the frame the Python rung would return, or it DECLINES by name ([`Decline`]) and the caller runs
//! the Python builder exactly as it ran before the port existed:
//!
//! * the vertices of the snapped patch do not lie in one plane (the oracle projects them: near-planar), the faces define no plane (it unfolds);
//! * the window of the grid does not exist, the search finds no scale that restores the intended right corners, or the snap breaks the embedding (the oracle refuses by name, or unfolds);
//! * the vertices define no canonical basis or the chart has no oriented face (the oracle raises `ValueError`);
//! * a call the port does not carry (a grid law that does not snap, the plane-preserving retry law).
//!
//! Native never refuses a patch itself and never answers where the oracle would not answer the same: the text of a refusal, the exception, the curved and the banded charts are the oracle's. The numbers
//! are the oracle's bit for bit: [`resolve`] is an exact computation on integers (`dyadic.rs`, `grid.rs`, `affine.rs`), every rational it returns is in lowest terms.

pub mod affine;
pub mod dyadic;
pub mod embed;
pub mod grid;
pub mod model;

pub use model::{Decline, Frame, GridFacts, Input, Outcome, Reason, Trial};

/// The frame of the owner patch, or the named decline.
pub fn resolve(input: &Input) -> Outcome<Frame> {
    use model::decline;

    if !input.snaps_source {
        return decline(Reason::Law, "a grid law that does not snap the source");
    }
    if input.positions.len() < 3 {
        return decline(Reason::Frame, "exact affine metric requires at least three total source vertices");
    }
    let resolved = grid::resolve(&input.positions, &input.faces)?;
    embed::check(&resolved, &input.faces, &input.edge_names)?;
    let normal = affine::plane_normal(&resolved.snapped, &input.faces)?;
    if !affine::is_exact_plane(&resolved.snapped, &normal) {
        return decline(Reason::Plane, "NEAR_PLANAR: the snapped vertices do not lie in the plane of the faces");
    }
    let found = affine::frame(&resolved.snapped, resolved.facts.exponent, &input.faces)?;
    Ok(Frame {
        grid: resolved.facts,
        normal,
        origin: found.origin,
        basis_a: found.basis_a,
        basis_b: found.basis_b,
        gram: found.gram,
        inverse: found.inverse,
        coordinates: found.coordinates,
        orientation_sign: found.orientation_sign,
    })
}
