//! The source-snap embedding gate: `source_grid._source_grid_result` builds the certificate over the positions before and after the snap and, with embedding enforced (the product), refuses
//! when `source_snap_violation` finds a count. The certificate is a pure function of its arguments (`cftuv-embedding` is its port); the host keeps only its verdict here, because
//! `build_rational_affine_planar_metric` returns the metric alone and drops the certificate.

use cftuv_core::num::{IBig, UBig};
use cftuv_embedding::{compute, After, Corner, Face, Input, RPoint, Rational};

use crate::grid::Resolved;
use crate::model::{decline, FaceIn, Outcome, Reason};

fn corner(found: &[u32; 3]) -> Corner {
    [Some(found[0]), Some(found[1]), Some(found[2])]
}

/// `Ok(())` when the snap breaks nothing: no vertices made coincident, no non-zero edge collapsed, no new crossing of non-adjacent edges, no intended right corner flattened.
pub fn check(resolved: &Resolved, faces: &[FaceIn], edge_names: &[String]) -> Outcome<()> {
    let e_min = resolved.e_min;
    let before_denominator = if e_min < 0 { UBig::ONE << (-e_min) as usize } else { UBig::ONE };
    let lift = |value: &IBig| -> Rational {
        if e_min < 0 {
            Rational::new(value.clone(), before_denominator.clone())
        } else {
            Rational::new(value.clone() << e_min as usize, UBig::ONE)
        }
    };
    let before: Vec<RPoint> = resolved.lifted.iter().map(|point| [lift(&point[0]), lift(&point[1]), lift(&point[2])]).collect();
    let after_denominator = UBig::ONE << resolved.facts.exponent as usize;
    let after: Vec<RPoint> = resolved
        .snapped
        .iter()
        .map(|point| {
            [
                Rational::new(point[0].clone(), after_denominator.clone()),
                Rational::new(point[1].clone(), after_denominator.clone()),
                Rational::new(point[2].clone(), after_denominator.clone()),
            ]
        })
        .collect();
    let input = Input {
        before,
        after: After::Points(after),
        faces: faces.iter().map(|face| Face { key: face.key.clone(), vertices: face.vertices.clone(), edges: face.edges.clone() }).collect(),
        edge_names: edge_names.to_vec(),
        intended: resolved.corners.intended.iter().map(corner).collect(),
        unclassifiable: resolved.corners.unclassifiable.iter().map(corner).collect(),
    };
    match compute(&input) {
        Ok(report) => {
            let counts = report.counts;
            if counts.newly_coincident_vertex_pair_count != 0
                || counts.collapsed_nonzero_source_edge_count != 0
                || counts.new_nonadjacent_edge_intersection_count != 0
                || counts.degenerated_intended_right_corner_count != 0
            {
                return decline(Reason::Grid, "SOURCE_SNAP_EMBEDDING_VIOLATED");
            }
            Ok(())
        }
        Err(failure) => decline(Reason::Grid, format!("the embedding certificate does not exist: {failure:?}")),
    }
}
