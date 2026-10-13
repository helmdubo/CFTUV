//! `validate_analysis_snapshot(snapshot, developable_stretch_budget=...)` (`kernel/src/cftuv_envelope/validation_snapshot.py`) with `source_edge_zero_length_issues` and
//! `seam_neighbour_face_issues`, mirrored statement by statement: every place where the Python function adds an issue (or raises) is a decline here.
//!
//! The order follows the Python function, so that a reader can lay the two side by side. Where Python indexes (`face.vertex_cycle[index]`, `triangle.vertex_ids[2]`) the port first
//! checks the length: an `IndexError` or a `ZeroDivisionError` of the oracle is not a clean snapshot either.

use std::collections::{HashMap, HashSet};

use cftuv_canon::fxhash::FxBuild;

use crate::metric;
use crate::model::{
    capability, Constraint, Direction, JunctionKind, Measure, Metric, Role, Snapshot, Target, Topology, Turn, Id, Kind,
};
use crate::verdict::{decline, issue, unsupported, Check, Context, Reason};

type Set = HashSet<Id, FxBuild>;
type Map<V> = HashMap<Id, V, FxBuild>;

/// The surface schema the oracle compares against (a literal inside `validate_analysis_snapshot`).
pub const SURFACE_SCHEMA: &[u8] = b"cftuv.envelope.patch_surface_ir.v1";
/// The suffix of the id of the right copy of a cut vertex of a ring (`contracts/metric.CUT_RIGHT_COPY_MARK`).
const CUT_RIGHT_COPY_MARK: &[u8] = b"|cut:R";

fn set_of(ids: impl Iterator<Item = Id>, capacity: usize) -> Set {
    let mut set = Set::with_capacity_and_hasher(capacity, FxBuild::default());
    set.extend(ids);
    set
}

/// `_check_unique`: the ids are pairwise different.
fn unique<T>(items: &[T], id: impl Fn(&T) -> Id, what: &'static str) -> Check<Set> {
    let set = set_of(items.iter().map(id), items.len());
    if set.len() != items.len() {
        return issue(what);
    }
    Ok(set)
}

/// `_require_refs`: every reference is among the targets.
fn require(refs: impl Iterator<Item = Id>, targets: &Set, what: &'static str) -> Check<()> {
    for reference in refs {
        if !targets.contains(&reference) {
            return issue(what);
        }
    }
    Ok(())
}

/// `{a, b} == {c, d}` as sets.
fn same_pair(a: Id, b: Id, c: Id, d: Id) -> bool {
    (a == c && b == d) || (a == d && b == c) || (a == b && c == d && a == c)
}

fn index_by<T>(items: &[T], id: impl Fn(&T) -> Id) -> Map<usize> {
    let mut map = Map::with_capacity_and_hasher(items.len(), FxBuild::default());
    for (index, item) in items.iter().enumerate() {
        map.insert(id(item), index);
    }
    map
}

/// `source_vertex_of_chart_vertex`: the id of the right copy of a cut vertex names its source vertex; `Err` when that vertex is not even a string of the snapshot (then it cannot be in `vertex_ids`).
fn source_vertex_of_chart_vertex(snapshot: &Snapshot<'_>, id: Id) -> Option<Id> {
    let text = snapshot.strings[id.s as usize];
    if !text.ends_with(CUT_RIGHT_COPY_MARK) {
        return Some(id);
    }
    let stripped = &text[..text.len() - CUT_RIGHT_COPY_MARK.len()];
    snapshot.strings.iter().position(|candidate| *candidate == stripped).map(|index| Id::new(Kind::SourceVertex, index as u32))
}

fn schema_matches(snapshot: &Snapshot<'_>, sid: u32, expected: &[u8]) -> bool {
    snapshot.strings[sid as usize] == expected
}

pub fn analysis(snapshot: &Snapshot<'_>, context: &Context) -> Check<()> {
    if !schema_matches(snapshot, snapshot.schema, &context.analysis_schema) {
        return issue("unsupported snapshot schema");
    }
    if !schema_matches(snapshot, snapshot.surface.schema, SURFACE_SCHEMA) {
        return issue("unsupported surface schema");
    }
    if snapshot.revision != snapshot.surface.revision {
        return issue("surface and snapshot revisions differ");
    }
    if capability::REQUIRED_V1 & !snapshot.capabilities != 0 {
        return issue("missing required v1 capabilities");
    }

    let patch_ids = unique(&snapshot.patches, |id| *id, "duplicate patch_id")?;
    let domain_ids = unique(&snapshot.domains, |item| item.id, "duplicate patch_domain_id")?;
    let vertex_ids = unique(&snapshot.vertices, |item| item.id, "duplicate vertex_id")?;
    let chain_ids = unique(&snapshot.chains, |item| item.id, "duplicate physical_chain_id")?;
    let use_ids = unique(&snapshot.uses, |item| item.id, "duplicate chain_use_id")?;
    let loop_ids = unique(&snapshot.loops, |item| item.id, "duplicate boundary_loop_id")?;
    let boundary_ids = unique(&snapshot.constraints, |item| item.id, "duplicate boundary_constraint_id")?;
    let angular_sector_ids = unique(&snapshot.sectors, |item| item.id, "duplicate owner_sector_id")?;
    let angle_certificate_ids = unique(&snapshot.certificates, |item| item.id, "duplicate certificate_id")?;
    unique(&snapshot.corners, |item| item.id, "duplicate corner_relation_id")?;
    unique(&snapshot.junctions, |item| item.id, "duplicate junction_relation_id")?;
    unique(&snapshot.terminals, |item| item.id, "duplicate terminal_relation_id")?;

    let metric_domain_ids = set_of(snapshot.metrics.iter().map(Metric::domain), snapshot.metrics.len());
    if metric_domain_ids.len() != snapshot.metrics.len() {
        return issue("duplicate patch_domain_id of the metric descriptors");
    }
    require(metric_domain_ids.iter().copied(), &domain_ids, "a metric descriptor names a domain that is not in the snapshot")?;
    if metric_domain_ids != domain_ids {
        return issue("exactly one metric descriptor is required per PatchDomain");
    }

    require(snapshot.domains.iter().map(|domain| domain.owner_patch), &patch_ids, "a domain names an owner patch that is not in the snapshot")?;
    let surface = &snapshot.surface;
    require(surface.vertex_ids.iter().copied(), &vertex_ids, "the surface names a vertex that is not in the snapshot")?;
    if set_of(surface.vertex_ids.iter().copied(), surface.vertex_ids.len()) != vertex_ids {
        return issue("surface vertex set must equal snapshot vertex set");
    }

    // `full_surface`; the other payload mode (the coordinate-free fixture) is not a product snapshot
    if !surface.full {
        return unsupported("a coordinate-free fixture payload");
    }
    if snapshot.capabilities & capability::EC0_COORDINATE_FREE_FIXTURE_V5 != 0 {
        return issue("fixture capability cannot label full host surface");
    }
    if capability::REQUIRED_FULL_SURFACE & !snapshot.capabilities != 0 {
        return issue("full host surface lacks R1 geometry-boundary capabilities");
    }
    if snapshot.vertices.iter().any(|vertex| vertex.position.is_none()) {
        return issue("full surface requires local coordinates");
    }

    let domain_by_id = index_by(&snapshot.domains, |item| item.id);
    let use_by_id = index_by(&snapshot.uses, |item| item.id);
    let chain_by_id = index_by(&snapshot.chains, |item| item.id);
    let loop_by_id = index_by(&snapshot.loops, |item| item.id);
    let boundary_by_id = index_by(&snapshot.constraints, |item| item.id);
    let edge_ids = unique(&surface.edges, |item| item.id, "duplicate edge_id")?;
    let edge_by_id = index_by(&surface.edges, |item| item.id);
    let face_ids = unique(&surface.faces, |item| item.id, "duplicate face_id")?;
    let triangle_ids = unique(&surface.triangles, |item| item.id, "duplicate triangle_id")?;
    let triangle_by_id = index_by(&surface.triangles, |item| item.id);

    for edge in &surface.edges {
        require([edge.a, edge.b].into_iter(), &vertex_ids, "an edge endpoint is not a vertex of the snapshot")?;
    }
    for face in &surface.faces {
        require(std::iter::once(face.patch), &patch_ids, "a face names a patch that is not in the snapshot")?;
        require(face.vertex_cycle.iter().copied(), &vertex_ids, "a face cycle names a vertex that is not in the snapshot")?;
        require(face.edge_cycle.iter().copied(), &edge_ids, "a face cycle names an edge that is not in the snapshot")?;
        require(face.triangles.iter().copied(), &triangle_ids, "a face lists a triangle that is not in the snapshot")?;
        for (index, edge_id) in face.edge_cycle.iter().enumerate() {
            let Some(&edge_index) = edge_by_id.get(edge_id) else { continue };
            let count = face.vertex_cycle.len();
            if count == 0 || index >= count {
                return decline(Reason::Shape, "a face whose edge cycle is longer than its vertex cycle (the oracle indexes past it)");
            }
            let edge = &surface.edges[edge_index];
            if !same_pair(edge.a, edge.b, face.vertex_cycle[index], face.vertex_cycle[(index + 1) % count]) {
                return issue("physical edge endpoints do not match the face vertex cycle");
            }
        }
    }
    for triangle in &surface.triangles {
        require(std::iter::once(triangle.face), &face_ids, "a triangle names a face that is not in the snapshot")?;
        require(triangle.vertices.iter().copied(), &vertex_ids, "a triangle names a vertex that is not in the snapshot")?;
        require(triangle.edges.iter().flatten().copied(), &edge_ids, "a triangle names an edge that is not in the snapshot")?;
        if triangle.vertices.len() != 3 || triangle.edges.len() > 3 {
            return decline(Reason::Shape, "a triangle that is not three vertices and at most three edges");
        }
        let (v0, v1, v2) = (triangle.vertices[0], triangle.vertices[1], triangle.vertices[2]);
        let pairs = [(v0, v1), (v1, v2), (v2, v0)];
        for (index, edge_id) in triangle.edges.iter().enumerate() {
            let Some(edge_id) = edge_id else { continue };
            let Some(&edge_index) = edge_by_id.get(edge_id) else { continue };
            let edge = &surface.edges[edge_index];
            if !same_pair(edge.a, edge.b, pairs[index].0, pairs[index].1) {
                return issue("triangle physical edge does not match its endpoint pair");
            }
        }
    }

    // each triangle is listed by exactly one face, and the face lists all and only the triangles that name it
    let mut listed_owners: Map<u32> = Map::with_capacity_and_hasher(surface.triangles.len(), FxBuild::default());
    let mut by_face: Map<Vec<Id>> = Map::with_capacity_and_hasher(surface.faces.len(), FxBuild::default());
    for triangle in &surface.triangles {
        by_face.entry(triangle.face).or_default().push(triangle.id);
    }
    for face in &surface.faces {
        let expected = set_of(by_face.get(&face.id).into_iter().flatten().copied(), 4);
        if set_of(face.triangles.iter().copied(), face.triangles.len()) != expected {
            return issue("face must list all and only triangles whose source_face_id names it");
        }
        for triangle_id in &face.triangles {
            *listed_owners.entry(*triangle_id).or_insert(0) += 1;
            if let Some(&triangle_index) = triangle_by_id.get(triangle_id) {
                if surface.triangles[triangle_index].face != face.id {
                    return issue("triangle source_face_id disagrees with face listing");
                }
            }
        }
    }
    for triangle in &surface.triangles {
        if listed_owners.get(&triangle.id).copied().unwrap_or(0) != 1 {
            return issue("triangle must be listed by exactly one source face");
        }
    }

    let mut patch_vertices: Map<Set> = Map::with_capacity_and_hasher(patch_ids.len(), FxBuild::default());
    for face in &surface.faces {
        patch_vertices.entry(face.patch).or_default().extend(face.vertex_cycle.iter().copied());
    }
    let empty = Set::default();
    let mut planarity_ids = Set::default();
    for descriptor in &snapshot.metrics {
        let domain = &snapshot.domains[*domain_by_id.get(&descriptor.domain()).expect("the descriptor domains are the snapshot domains")];
        let affine = match descriptor {
            Metric::Other { class, .. } => return unsupported(format!("a metric descriptor of class {class}")),
            Metric::Affine(affine) => affine,
        };
        metric::validate_affine(affine)?;
        if !planarity_ids.insert(affine.certificate.id) {
            return issue("duplicate planarity certificate ID");
        }
        if !domain.planar {
            return issue("RationalAffinePlanarMetricV2 requires a PLANAR PatchDomain");
        }
        if affine.revision != snapshot.revision {
            return issue("metric and snapshot revisions differ");
        }
        let mut chart_vertices = Set::default();
        for (id, _) in &affine.coordinates {
            match source_vertex_of_chart_vertex(snapshot, *id) {
                Some(vertex) => {
                    chart_vertices.insert(vertex);
                }
                None => return issue("a chart vertex names a source vertex that is not in the snapshot"),
            }
        }
        require(chart_vertices.iter().copied(), &vertex_ids, "exact_source_vertex_coordinates names a vertex that is not in the snapshot")?;
        let required = patch_vertices.get(&domain.owner_patch).unwrap_or(&empty);
        // `metric_covers_patch` for a plane certificate: the coordinates cover ALL AND ONLY the vertices of the owner patch
        if &chart_vertices != required {
            return issue("exact affine coordinates must cover all and only owner-patch surface vertices");
        }
        metric::against_source(snapshot, affine)?;
    }

    for domain in &snapshot.domains {
        require(domain.boundary_constraints.iter().copied(), &boundary_ids, "a domain names a boundary constraint that is not in the snapshot")?;
        require(domain.sectors.iter().map(|(_, chain_use)| *chain_use), &use_ids, "a domain sector names a ChainUse that is not in the snapshot")?;
        for constraint_id in &domain.boundary_constraints {
            if let Some(&index) = boundary_by_id.get(constraint_id) {
                if snapshot.constraints[index].domain != domain.id {
                    return issue("boundary constraint belongs to another PatchDomain");
                }
            }
        }
        for (_, chain_use) in &domain.sectors {
            if let Some(&index) = use_by_id.get(chain_use) {
                let used = &snapshot.uses[index];
                if used.domain != domain.id || used.owner_patch != domain.owner_patch {
                    return issue("domain sector ChainUse has another owner/domain");
                }
            }
        }
    }
    // the domain sector ids join the angular sector ids for the terminal relations below
    let mut all_sector_ids = angular_sector_ids.clone();
    for domain in &snapshot.domains {
        all_sector_ids.extend(domain.sectors.iter().map(|(sector, _)| *sector));
    }
    for chain in &snapshot.chains {
        require(chain.vertices.iter().copied(), &vertex_ids, "a chain names a vertex that is not in the snapshot")?;
        require(chain.edges.iter().copied(), &edge_ids, "a chain names an edge that is not in the snapshot")?;
        let vertex_count = chain.vertices.len();
        let edge_count = chain.edges.len();
        let expected_edge_count = if chain.closed { vertex_count } else { vertex_count.wrapping_sub(1) };
        if vertex_count < 2 || edge_count != expected_edge_count {
            return issue("open chains require E=V-1 and closed chains require E=V");
        }
        for (index, edge_id) in chain.edges.iter().enumerate() {
            let Some(&edge_index) = edge_by_id.get(edge_id) else { continue };
            let next = (index + 1) % vertex_count;
            let edge = &surface.edges[edge_index];
            if !same_pair(edge.a, edge.b, chain.vertices[index], chain.vertices[next]) {
                return issue("chain edge endpoints do not match the ordered vertex path");
            }
        }
    }
    for used in &snapshot.uses {
        require(std::iter::once(used.chain), &chain_ids, "a ChainUse names a chain that is not in the snapshot")?;
        require(std::iter::once(used.owner_patch), &patch_ids, "a ChainUse names an owner patch that is not in the snapshot")?;
        require(std::iter::once(used.domain), &domain_ids, "a ChainUse names a domain that is not in the snapshot")?;
        require(std::iter::once(used.launch_constraint), &boundary_ids, "a ChainUse names a launch constraint that is not in the snapshot")?;
        if let Some(&index) = domain_by_id.get(&used.domain) {
            if snapshot.domains[index].owner_patch != used.owner_patch {
                return issue("ChainUse owner differs from PatchDomain owner");
            }
        }
        if let Some(&index) = boundary_by_id.get(&used.launch_constraint) {
            if snapshot.constraints[index].domain != used.domain {
                return issue("launch constraint belongs to another PatchDomain");
            }
        }
        match used.boundary_loop {
            Some(boundary_loop) => require(std::iter::once(boundary_loop), &loop_ids, "a ChainUse names a boundary loop that is not in the snapshot")?,
            None => return issue("full host ChainUse requires a boundary loop"),
        }
    }
    for boundary_loop in &snapshot.loops {
        require(std::iter::once(boundary_loop.domain), &domain_ids, "a boundary loop names a domain that is not in the snapshot")?;
        require(boundary_loop.uses.iter().copied(), &use_ids, "a boundary loop names a ChainUse that is not in the snapshot")?;
        for use_id in &boundary_loop.uses {
            if let Some(&index) = use_by_id.get(use_id) {
                let used = &snapshot.uses[index];
                if used.domain != boundary_loop.domain || used.boundary_loop != Some(boundary_loop.id) {
                    return issue("BoundaryLoop and ChainUse references are not reciprocal");
                }
            }
        }
    }
    for constraint in &snapshot.constraints {
        constraint_references(snapshot, constraint, &domain_ids, &use_ids, &loop_ids, &edge_ids, &use_by_id, &loop_by_id)?;
    }
    for sector in &snapshot.sectors {
        require(std::iter::once(sector.owner_patch), &patch_ids, "an angular sector names an owner patch that is not in the snapshot")?;
        require(std::iter::once(sector.domain), &domain_ids, "an angular sector names a domain that is not in the snapshot")?;
        require(sector.incident_uses.iter().copied(), &use_ids, "an angular sector names a ChainUse that is not in the snapshot")?;
        for use_id in &sector.incident_uses {
            if let Some(&index) = use_by_id.get(use_id) {
                let used = &snapshot.uses[index];
                if used.owner_patch != sector.owner_patch || used.domain != sector.domain {
                    return issue("oriented sector support has another owner/domain");
                }
            }
        }
        if sector.incident_uses.len() < 2 {
            return issue("ordered incident supports are required");
        } else if sector.incoming.chain_use != sector.incident_uses[0] || sector.outgoing.chain_use != *sector.incident_uses.last().expect("two or more") {
            return issue("incoming/outgoing refs must match ordered supports");
        }
        require([sector.incoming.launch_boundary, sector.outgoing.launch_boundary].into_iter(), &boundary_ids, "a support names a launch boundary that is not in the snapshot")?;
        for support in [&sector.incoming, &sector.outgoing] {
            match &support.direction {
                // FULL_HOST_SURFACE requires an authoritative planar support direction
                Direction::Unavailable => return issue("FULL_HOST_SURFACE requires an authoritative planar support direction"),
                Direction::Affine(x, y) => {
                    if x.is_zero() && y.is_zero() {
                        return issue("support direction cannot be zero");
                    }
                }
                Direction::Planar(x, y) => {
                    if *x == 0.0 && *y == 0.0 {
                        return issue("support direction cannot be zero");
                    }
                }
            }
        }
    }
    let sector_turn: Map<Turn> = snapshot.sectors.iter().map(|sector| (sector.id, sector.turn)).collect();
    for certificate in &snapshot.certificates {
        require(std::iter::once(certificate.sector), &angular_sector_ids, "a reflex certificate names a sector that is not in the snapshot")?;
        if certificate.exact_two_pi {
            return issue("2*pi is terminal/junction, never Angular");
        }
        match &certificate.measure {
            Measure::Unavailable => return issue("FULL_HOST_SURFACE requires a certified angle payload"),
            Measure::Certified { phi, delta, orientation } => {
                crate::interval::check_measure(phi, delta, context)?;
                if let Some(turn) = sector_turn.get(&certificate.sector) {
                    if turn != orientation {
                        return issue("angle orientation differs from owner sector");
                    }
                }
            }
        }
    }
    for corner in &snapshot.corners {
        require(std::iter::once(corner.vertex), &vertex_ids, "a corner names a vertex that is not in the snapshot")?;
        require(std::iter::once(corner.sector), &angular_sector_ids, "a corner names a sector that is not in the snapshot")?;
        require(std::iter::once(corner.certificate), &angle_certificate_ids, "a corner names a certificate that is not in the snapshot")?;
        if corner.exact_two_pi {
            return issue("2*pi CornerRelation is forbidden");
        }
    }
    let mut all_route_pairing_ids = Set::default();
    for junction in &snapshot.junctions {
        require(std::iter::once(junction.vertex), &vertex_ids, "a junction names a vertex that is not in the snapshot")?;
        require(junction.incident_uses.iter().copied(), &use_ids, "a junction names a ChainUse that is not in the snapshot")?;
        let expected_variant = match junction.kind {
            JunctionKind::T => 0,
            JunctionKind::X => 1,
            JunctionKind::Y => 2,
            JunctionKind::CrossPatch | JunctionKind::MixedAlpha => 3,
        };
        let (variant, pairs): (i8, Vec<&crate::model::RoutePair>) = match &junction.topology {
            Topology::Unavailable => return issue("FULL_HOST_SURFACE requires typed junction route topology"),
            Topology::T { through, .. } => (0, vec![through]),
            Topology::X { pairs } => (1, pairs.iter().collect()),
            Topology::Y { pairs, .. } => (2, pairs.iter().collect()),
            Topology::Declared { pairs } => (3, pairs.iter().collect()),
        };
        if variant != expected_variant {
            return issue("route topology variant does not match junction kind");
        }
        let pair_ids: Vec<Id> = pairs.iter().map(|pair| pair.id).collect();
        let distinct = set_of(pair_ids.iter().copied(), pair_ids.len());
        if distinct.len() != pair_ids.len() {
            return issue("duplicate route pairing ID");
        }
        if distinct.iter().any(|id| all_route_pairing_ids.contains(id)) {
            return issue("route pairing ID is reused by another JunctionRelation");
        }
        all_route_pairing_ids.extend(pair_ids.iter().copied());
        let mut referenced = Set::default();
        for pair in &pairs {
            if pair.first == pair.second {
                return issue("route pair endpoints must differ");
            }
            referenced.insert(pair.first);
            referenced.insert(pair.second);
        }
        match &junction.topology {
            Topology::T { branches, .. } => referenced.extend(branches.iter().copied()),
            Topology::Y { trunk, branches, .. } => {
                referenced.insert(*trunk);
                referenced.extend(branches.iter().copied());
            }
            _ => {}
        }
        let incident = set_of(junction.incident_uses.iter().copied(), junction.incident_uses.len());
        require(referenced.iter().copied(), &incident, "the route topology names a ChainUse that is not incident")?;
        if referenced != incident {
            return issue("typed route topology must cover every incident ChainUse");
        }
    }
    for terminal in &snapshot.terminals {
        require(std::iter::once(terminal.vertex), &vertex_ids, "a terminal names a vertex that is not in the snapshot")?;
        require(std::iter::once(terminal.chain_use), &use_ids, "a terminal names a ChainUse that is not in the snapshot")?;
        require(std::iter::once(terminal.owner_patch), &patch_ids, "a terminal names an owner patch that is not in the snapshot")?;
        require(std::iter::once(terminal.domain), &domain_ids, "a terminal names a domain that is not in the snapshot")?;
        if let Some(sector) = terminal.sector {
            require(std::iter::once(sector), &all_sector_ids, "a terminal names a sector that is not in the snapshot")?;
        }
        if let Some(&index) = use_by_id.get(&terminal.chain_use) {
            let used = &snapshot.uses[index];
            if used.owner_patch != terminal.owner_patch || used.domain != terminal.domain {
                return issue("terminal owner patch/domain differs from ChainUse");
            }
            if let Some(&chain_index) = chain_by_id.get(&used.chain) {
                let chain = &snapshot.chains[chain_index];
                if let (Some(first), Some(last)) = (chain.vertices.first(), chain.vertices.last()) {
                    if terminal.role == Role::Start && terminal.vertex != *first {
                        return issue("START terminal is not the first physical-chain vertex");
                    }
                    if terminal.role == Role::End && terminal.vertex != *last {
                        return issue("END terminal is not the last physical-chain vertex");
                    }
                }
            }
        }
    }
    zero_length_edges(snapshot)?;
    neighbour_faces(snapshot, &patch_ids)?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn constraint_references(
    snapshot: &Snapshot<'_>,
    constraint: &Constraint,
    domain_ids: &Set,
    use_ids: &Set,
    loop_ids: &Set,
    edge_ids: &Set,
    use_by_id: &Map<usize>,
    loop_by_id: &Map<usize>,
) -> Check<()> {
    require(std::iter::once(constraint.domain), domain_ids, "a boundary constraint names a domain that is not in the snapshot")?;
    if let Some(originating) = constraint.originating_use {
        require(std::iter::once(originating), use_ids, "a boundary constraint names an originating ChainUse that is not in the snapshot")?;
        if let Some(&index) = use_by_id.get(&originating) {
            if snapshot.uses[index].domain != constraint.domain {
                return issue("originating ChainUse belongs to another PatchDomain");
            }
        }
    }
    match &constraint.target {
        Target::Loop(boundary_loop) => {
            require(std::iter::once(*boundary_loop), loop_ids, "a boundary target names a loop that is not in the snapshot")?;
            if let Some(&index) = loop_by_id.get(boundary_loop) {
                if snapshot.loops[index].domain != constraint.domain {
                    return issue("target BoundaryLoop belongs to another PatchDomain");
                }
            }
        }
        Target::Use(chain_use) => {
            require(std::iter::once(*chain_use), use_ids, "a boundary target names a ChainUse that is not in the snapshot")?;
            if let Some(&index) = use_by_id.get(chain_use) {
                if snapshot.uses[index].domain != constraint.domain {
                    return issue("target ChainUse belongs to another PatchDomain");
                }
            }
        }
        Target::Edges(edges) => {
            require(edges.iter().copied(), edge_ids, "a boundary target names an edge that is not in the snapshot")?;
            if edges.is_empty() {
                return issue("physical-edge boundary target cannot be empty");
            }
        }
        Target::Unavailable => return issue("FULL_HOST_SURFACE requires a concrete boundary target"),
    }
    Ok(())
}

/// `source_edge_zero_length_issues`: no edge joins two vertices at one local position.
fn zero_length_edges(snapshot: &Snapshot<'_>) -> Check<()> {
    let positions: Map<[f64; 3]> = snapshot.vertices.iter().filter_map(|vertex| vertex.position.map(|position| (vertex.id, position))).collect();
    for edge in &snapshot.surface.edges {
        let Some(first) = positions.get(&edge.a) else { continue };
        match positions.get(&edge.b) {
            Some(second) if first == second => return issue("an edge joins two vertices at one local position"),
            _ => {}
        }
    }
    Ok(())
}

/// `seam_neighbour_face_issues`.
fn neighbour_faces(snapshot: &Snapshot<'_>, patch_ids: &Set) -> Check<()> {
    let own = set_of(snapshot.surface.faces.iter().map(|face| face.id), snapshot.surface.faces.len());
    let mut names = Set::default();
    for face in &snapshot.neighbours {
        if own.contains(&face.face) {
            return issue("neighbour face repeats a face of the snapshot surface");
        }
        if patch_ids.contains(&face.patch) {
            return issue("neighbour face belongs to a patch of the snapshot");
        }
        if set_of(face.vertices.iter().copied(), face.vertices.len()).len() != face.vertices.len() {
            return issue("neighbour face repeats a vertex");
        }
        if !names.insert(face.face) {
            return issue("two neighbour faces share a name");
        }
    }
    Ok(())
}
