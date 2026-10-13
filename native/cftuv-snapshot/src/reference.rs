//! `reference.validation.validate_reference_geometry_payload(snapshot, patch_domain_id)` for every domain, as `envelope_angle_certificate._reject_snapshot` calls it after the snapshot
//! validator: payload mode, regime, exactly one planar metric, and the angle certificates against the metric.
//!
//! The metric check inside it (`validate_rational_affine_planar_metric(frame)` without a budget) is the one `structural::analysis` already made on the same metric; the port does not repeat it.

use std::collections::HashMap;

use cftuv_canon::fxhash::FxBuild;

use crate::angle::{contains_turn, Class};
use crate::model::{Affine, Direction, Id, Measure, Metric, Snapshot};
use crate::verdict::{decline, issue, unsupported, Attestation, Check, Context, Reason};

/// `ExactPlanarMetric.from_descriptor(frame)` builds a grid when the law snaps the constructions, and that is where the oracle can raise or loop: the port requires what it needs.
fn chart_grid_is_buildable(frame: &Affine) -> Check<()> {
    let grid = &frame.grid;
    if !grid.snaps_source {
        return Ok(());
    }
    let Some(step) = &grid.window_step else { return decline(Reason::Shape, "a snapping law without a window step (the oracle reads window_step.numerator)") };
    if !grid.snaps_constructions {
        return Ok(());
    }
    if step.signum() <= 0 {
        return decline(Reason::Shape, "a window step that is not positive: chart_grid_for never ends");
    }
    // `g00 + 2 |g01| + g11 > 0`
    let gram = &frame.gram;
    let off_diagonal = if gram[0][1].signum() < 0 { gram[0][1].neg() } else { gram[0][1].clone() };
    let squared = gram[0][0].add(&off_diagonal.add(&off_diagonal)).add(&gram[1][1]);
    if squared.signum() <= 0 {
        return decline(Reason::Shape, "a Gram form whose chart grid cannot be built (chart_grid_for raises)");
    }
    Ok(())
}

/// The payload checks of every domain, in the order `_reject_snapshot` makes them; adds the corners it decides to the attestation.
pub fn payload(snapshot: &Snapshot<'_>, context: &Context, attestation: &mut Attestation) -> Check<()> {
    if !context.native_exact {
        return decline(Reason::Mode, "the symbolic backend is not NATIVE_EXACT: the validator takes another road");
    }
    let sectors: HashMap<Id, usize, FxBuild> = snapshot.sectors.iter().enumerate().map(|(index, sector)| (sector.id, index)).collect();
    let certificates: HashMap<Id, usize, FxBuild> = snapshot.certificates.iter().enumerate().map(|(index, certificate)| (certificate.id, index)).collect();
    for domain in &snapshot.domains {
        if !snapshot.surface.full {
            return issue("coordinate-free EC0 fixtures cannot be evaluated as geometry");
        }
        if !domain.planar {
            return issue("reference evaluator supports SurfaceRegime.PLANAR only");
        }
        // `frames`: the metric descriptors that are a planar frame or a rational affine metric of this domain
        let mut frames = snapshot.metrics.iter().filter(|metric| match metric {
            Metric::Affine(affine) => affine.domain == domain.id,
            Metric::Other { domain: owner, class } => *owner == domain.id && *class == "PlanarPatchFrameV1",
        });
        let Some(frame) = frames.next() else { return issue("PlanarPatchFrameV1 or RationalAffinePlanarMetricV2 is required") };
        if frames.next().is_some() {
            return issue("exactly one planar metric descriptor must own the domain");
        }
        let affine = match frame {
            Metric::Affine(affine) => affine,
            Metric::Other { class, .. } => return unsupported(format!("a metric descriptor of class {class}")),
        };
        chart_grid_is_buildable(affine)?;
        attestation.domains += 1;

        let owner_sign: i8 = if affine.ccw { 1 } else { -1 };
        for relation in &snapshot.corners {
            let Some(&sector_index) = sectors.get(&relation.sector) else { continue };
            let sector = &snapshot.sectors[sector_index];
            if sector.domain != domain.id {
                continue;
            }
            let Some(&certificate_index) = certificates.get(&relation.certificate) else { continue };
            let Measure::Certified { delta, .. } = &snapshot.certificates[certificate_index].measure else { continue };
            let (Direction::Affine(ix, iy), Direction::Affine(ox, oy)) = (&sector.incoming.direction, &sector.outgoing.direction) else {
                match (&sector.incoming.direction, &sector.outgoing.direction) {
                    (Direction::Unavailable, _) | (_, Direction::Unavailable) => continue,
                    _ => return unsupported("a floating-point support direction (CertifiedPlanarSupportDirectionV1)"),
                }
            };
            let class = contains_turn(&affine.gram, owner_sign, &[ix.clone(), iy.clone()], &[ox.clone(), oy.clone()], sector.turn, delta)?;
            attestation.corners += 1;
            match class {
                Class::Certified => attestation.certified += 1,
                Class::Straddled => attestation.straddled += 1,
            }
        }
    }
    Ok(())
}
