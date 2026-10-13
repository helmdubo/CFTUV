//! The snapshot as the validators read it: the records and fields `validate_analysis_snapshot` and `validate_reference_geometry_payload` look at, nothing else.
//!
//! The PyO3 layer builds this from the Python graph (`cftuv-python/src/snapshot_check.rs`); the checks (`structural`, `metric`, `reference`) run on it without Python. Two rules
//! of the reading, both for the soundness of a "clean" verdict:
//!
//! * IDENTITY. An id is equal to another id of ITS OWN CLASS and value only (`SourceVertexId('x') != PhysicalEdgeId('x')`), so an [`Id`] carries its [`Kind`]; the reader checks
//!   that every id field holds exactly the class the contract names, and the strings are interned once per call ([`Sid`]).
//! * SHAPE. Containers are read as the contract types them (a `frozenset` of records is a `Vec`, a duplicate is NOT collapsed: two records with one id are exactly what
//!   `DUPLICATE_ID` is about), and anything that is not the canonical shape is a decline of the reader, never a guess.

use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;

/// An interned string (one per distinct content, per call).
pub type Sid = u32;

/// The class of an id.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Kind {
    Patch,
    PatchDomain,
    SourceVertex,
    PhysicalChain,
    ChainUse,
    BoundaryLoop,
    BoundaryConstraint,
    OwnerSector,
    AngleCertificate,
    CornerRelation,
    JunctionRelation,
    TerminalRelation,
    PhysicalEdge,
    SourceFace,
    SurfaceTriangle,
    RoutePairing,
    PlanarityCertificate,
    ReferenceMetric,
    SourceRevision,
}

impl Kind {
    /// The class name in `cftuv_envelope.ids` (the reader looks the class up by it).
    pub const fn class_name(self) -> &'static str {
        match self {
            Kind::Patch => "PatchId",
            Kind::PatchDomain => "PatchDomainId",
            Kind::SourceVertex => "SourceVertexId",
            Kind::PhysicalChain => "PhysicalChainId",
            Kind::ChainUse => "ChainUseId",
            Kind::BoundaryLoop => "BoundaryLoopId",
            Kind::BoundaryConstraint => "BoundaryConstraintId",
            Kind::OwnerSector => "OwnerSectorId",
            Kind::AngleCertificate => "AngleCertificateId",
            Kind::CornerRelation => "CornerRelationId",
            Kind::JunctionRelation => "JunctionRelationId",
            Kind::TerminalRelation => "TerminalRelationId",
            Kind::PhysicalEdge => "PhysicalEdgeId",
            Kind::SourceFace => "SourceFaceId",
            Kind::SurfaceTriangle => "SurfaceTriangleId",
            Kind::RoutePairing => "RoutePairingId",
            Kind::PlanarityCertificate => "PlanarityCertificateId",
            Kind::ReferenceMetric => "ReferenceMetricId",
            Kind::SourceRevision => "SourceRevision",
        }
    }

    pub const ALL: [Kind; 19] = [
        Kind::Patch,
        Kind::PatchDomain,
        Kind::SourceVertex,
        Kind::PhysicalChain,
        Kind::ChainUse,
        Kind::BoundaryLoop,
        Kind::BoundaryConstraint,
        Kind::OwnerSector,
        Kind::AngleCertificate,
        Kind::CornerRelation,
        Kind::JunctionRelation,
        Kind::TerminalRelation,
        Kind::PhysicalEdge,
        Kind::SourceFace,
        Kind::SurfaceTriangle,
        Kind::RoutePairing,
        Kind::PlanarityCertificate,
        Kind::ReferenceMetric,
        Kind::SourceRevision,
    ];
}

/// An id: its class and its interned value.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Id {
    pub kind: Kind,
    pub s: Sid,
}

impl Id {
    pub fn new(kind: Kind, s: Sid) -> Id {
        Id { kind, s }
    }
}

/// The capabilities the validators ask about (a bit each); the other members of the enum never matter to a verdict.
pub mod capability {
    pub const PATCH_DOMAIN_V1: u32 = 1 << 0;
    pub const PHYSICAL_CHAIN_DIRECTED_USES_V1: u32 = 1 << 1;
    pub const PATCH_SURFACE_IR_V1: u32 = 1 << 2;
    pub const ORIENTED_OWNER_SECTOR_V1: u32 = 1 << 3;
    pub const REFLEX_ANGLE_CERTIFICATE_V1: u32 = 1 << 4;
    pub const RELATION_LINEAGE_V1: u32 = 1 << 5;
    pub const AUTHORITATIVE_SURFACE_METRIC_V1: u32 = 1 << 6;
    pub const PHYSICAL_EDGE_TABLE_V1: u32 = 1 << 7;
    pub const BOUNDARY_CONSTRAINT_TARGET_V1: u32 = 1 << 8;
    pub const TYPED_JUNCTION_ROUTE_TOPOLOGY_V1: u32 = 1 << 9;
    pub const TERMINAL_RELATION_V1: u32 = 1 << 10;
    pub const EC0_COORDINATE_FREE_FIXTURE_V5: u32 = 1 << 11;

    /// `(member name, bit)`: the reader maps a member to its bit by name.
    pub const NAMES: [(&str, u32); 12] = [
        ("PATCH_DOMAIN_V1", PATCH_DOMAIN_V1),
        ("PHYSICAL_CHAIN_DIRECTED_USES_V1", PHYSICAL_CHAIN_DIRECTED_USES_V1),
        ("PATCH_SURFACE_IR_V1", PATCH_SURFACE_IR_V1),
        ("ORIENTED_OWNER_SECTOR_V1", ORIENTED_OWNER_SECTOR_V1),
        ("REFLEX_ANGLE_CERTIFICATE_V1", REFLEX_ANGLE_CERTIFICATE_V1),
        ("RELATION_LINEAGE_V1", RELATION_LINEAGE_V1),
        ("AUTHORITATIVE_SURFACE_METRIC_V1", AUTHORITATIVE_SURFACE_METRIC_V1),
        ("PHYSICAL_EDGE_TABLE_V1", PHYSICAL_EDGE_TABLE_V1),
        ("BOUNDARY_CONSTRAINT_TARGET_V1", BOUNDARY_CONSTRAINT_TARGET_V1),
        ("TYPED_JUNCTION_ROUTE_TOPOLOGY_V1", TYPED_JUNCTION_ROUTE_TOPOLOGY_V1),
        ("TERMINAL_RELATION_V1", TERMINAL_RELATION_V1),
        ("EC0_COORDINATE_FREE_FIXTURE_V5", EC0_COORDINATE_FREE_FIXTURE_V5),
    ];

    pub const REQUIRED_V1: u32 = PATCH_DOMAIN_V1 | PHYSICAL_CHAIN_DIRECTED_USES_V1 | PATCH_SURFACE_IR_V1 | ORIENTED_OWNER_SECTOR_V1 | REFLEX_ANGLE_CERTIFICATE_V1 | RELATION_LINEAGE_V1;
    pub const REQUIRED_FULL_SURFACE: u32 =
        AUTHORITATIVE_SURFACE_METRIC_V1 | PHYSICAL_EDGE_TABLE_V1 | BOUNDARY_CONSTRAINT_TARGET_V1 | TYPED_JUNCTION_ROUTE_TOPOLOGY_V1 | TERMINAL_RELATION_V1;
}

/// `IntervalEndpointKind`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Endpoint {
    Open,
    Closed,
}

/// `TurnOrientation`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Turn {
    Ccw,
    Cw,
}

/// `TerminalEndpointRole`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Start,
    End,
    NonChain,
}

/// `JunctionRelationKind`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JunctionKind {
    T,
    X,
    Y,
    CrossPatch,
    MixedAlpha,
}

/// A finite `Decimal` as `coefficient * 10^exponent` (the signed coefficient the string shows, trailing zeros kept: the digit count matters to a context).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Dec {
    pub coefficient: IBig,
    pub exponent: i64,
}

/// `CertifiedDecimalIntervalV1` (the error bound is not read by any validator).
#[derive(Clone, Debug)]
pub struct Interval {
    pub lower: Dec,
    pub upper: Dec,
    pub lower_kind: Endpoint,
    pub upper_kind: Endpoint,
}

#[derive(Clone, Debug)]
pub struct Domain {
    pub id: Id,
    pub owner_patch: Id,
    pub planar: bool,
    pub boundary_constraints: Vec<Id>,
    /// `(owner_sector_id, chain_use_id)` of each `PatchSectorV1`.
    pub sectors: Vec<(Id, Id)>,
}

#[derive(Clone, Debug)]
pub struct Vertex {
    pub id: Id,
    /// `LocalPoint3V1` as binary64, or `None` for any other position payload.
    pub position: Option<[f64; 3]>,
}

#[derive(Clone, Debug)]
pub struct Edge {
    pub id: Id,
    pub a: Id,
    pub b: Id,
}

#[derive(Clone, Debug)]
pub struct Face {
    pub id: Id,
    pub patch: Id,
    pub vertex_cycle: Vec<Id>,
    pub edge_cycle: Vec<Id>,
    pub triangles: Vec<Id>,
}

#[derive(Clone, Debug)]
pub struct Triangle {
    pub id: Id,
    pub face: Id,
    pub vertices: Vec<Id>,
    pub edges: Vec<Option<Id>>,
}

#[derive(Clone, Debug)]
pub struct Surface {
    pub schema: Sid,
    pub revision: Id,
    /// `payload_mode is FULL_HOST_SURFACE` (the other member, the coordinate-free fixture, is `false`).
    pub full: bool,
    pub vertex_ids: Vec<Id>,
    pub edges: Vec<Edge>,
    pub faces: Vec<Face>,
    pub triangles: Vec<Triangle>,
}

#[derive(Clone, Debug)]
pub struct Chain {
    pub id: Id,
    pub closed: bool,
    pub vertices: Vec<Id>,
    pub edges: Vec<Id>,
}

#[derive(Clone, Debug)]
pub struct Use {
    pub id: Id,
    pub chain: Id,
    pub owner_patch: Id,
    pub domain: Id,
    pub boundary_loop: Option<Id>,
    pub launch_constraint: Id,
}

#[derive(Clone, Debug)]
pub struct Loop {
    pub id: Id,
    pub domain: Id,
    pub uses: Vec<Id>,
}

#[derive(Clone, Debug)]
pub enum Target {
    Loop(Id),
    Use(Id),
    Edges(Vec<Id>),
    Unavailable,
}

#[derive(Clone, Debug)]
pub struct Constraint {
    pub id: Id,
    pub domain: Id,
    pub target: Target,
    pub originating_use: Option<Id>,
}

/// The direction payload of a support: only the exact affine one carries numbers.
#[derive(Clone, Debug)]
pub enum Direction {
    /// `CertifiedAffineSupportDirectionV2`: `direction_in_domain` as two exact rationals.
    Affine(Rat, Rat),
    /// `CertifiedPlanarSupportDirectionV1`: floats; the port does not carry it (the zero test of the structural pass can still be made, the angle test cannot).
    Planar(f64, f64),
    Unavailable,
}

#[derive(Clone, Debug)]
pub struct SupportRef {
    pub chain_use: Id,
    pub launch_boundary: Id,
    pub direction: Direction,
}

#[derive(Clone, Debug)]
pub struct Sector {
    pub id: Id,
    pub owner_patch: Id,
    pub domain: Id,
    pub incident_uses: Vec<Id>,
    pub incoming: SupportRef,
    pub outgoing: SupportRef,
    pub turn: Turn,
}

#[derive(Clone, Debug)]
pub enum Measure {
    Certified { phi: Interval, delta: Interval, orientation: Turn },
    Unavailable,
}

#[derive(Clone, Debug)]
pub struct Certificate {
    pub id: Id,
    pub sector: Id,
    pub measure: Measure,
    pub exact_two_pi: bool,
}

#[derive(Clone, Debug)]
pub struct Corner {
    pub id: Id,
    pub vertex: Id,
    pub sector: Id,
    pub certificate: Id,
    pub exact_two_pi: bool,
}

/// `JunctionRoutePairV1` (the station continuity law is not read by any validator).
#[derive(Clone, Debug)]
pub struct RoutePair {
    pub id: Id,
    pub first: Id,
    pub second: Id,
}

#[derive(Clone, Debug)]
pub enum Topology {
    T { through: RoutePair, branches: Vec<Id> },
    X { pairs: Vec<RoutePair> },
    Y { trunk: Id, branches: Vec<Id>, pairs: Vec<RoutePair> },
    Declared { pairs: Vec<RoutePair> },
    Unavailable,
}

#[derive(Clone, Debug)]
pub struct Junction {
    pub id: Id,
    pub vertex: Id,
    pub incident_uses: Vec<Id>,
    pub kind: JunctionKind,
    pub topology: Topology,
}

#[derive(Clone, Debug)]
pub struct Terminal {
    pub id: Id,
    pub vertex: Id,
    pub chain_use: Id,
    pub owner_patch: Id,
    pub domain: Id,
    pub role: Role,
    pub sector: Option<Id>,
}

#[derive(Clone, Debug)]
pub struct Neighbour {
    pub face: Id,
    pub patch: Id,
    pub vertices: Vec<Id>,
}

/// `ExactSourcePlaneCertificateV1`: the only planarity certificate the port carries.
#[derive(Clone, Debug)]
pub struct PlaneCertificate {
    pub id: Id,
    pub domain: Id,
    /// `admission_law is EXACT_SOURCE_PLANE_V1`.
    pub exact_law: bool,
    pub exact: bool,
    pub normal: [Rat; 3],
    pub vertex_ids: Vec<Id>,
    /// `reconstruction_law is O_PLUS_U_A_PLUS_V_B_V1`.
    pub reconstruction_ok: bool,
}

/// `IntegerGridCertificateV1`: the three fields the validators read.
#[derive(Clone, Debug)]
pub struct Grid {
    /// `GridSnappingLawV1.snaps_source`.
    pub snaps_source: bool,
    /// `GridSnappingLawV1.snaps_constructions`.
    pub snaps_constructions: bool,
    pub window_step: Option<Rat>,
    pub source_scale: Option<IBig>,
}

/// A `RationalAffinePlanarMetricV2` whose certificate is an `ExactSourcePlaneCertificateV1` and whose frame law is the canonical source-vertex basis.
#[derive(Clone, Debug)]
pub struct Affine {
    pub reference_metric: Id,
    pub domain: Id,
    pub revision: Id,
    pub origin: [Rat; 3],
    pub basis_a: [Rat; 3],
    pub basis_b: [Rat; 3],
    pub gram: [[Rat; 2]; 2],
    pub inverse_gram: [[Rat; 2]; 2],
    /// `(source_vertex_id, (u, v))` of each `ExactSourceVertexCoordinateV2`.
    pub coordinates: Vec<(Id, [Rat; 2])>,
    /// `chart_orientation is COORDINATE_CCW_MATCHES_OWNER_PATCH`.
    pub ccw: bool,
    pub certificate: PlaneCertificate,
    pub grid: Grid,
}

/// What the port knows about a metric descriptor.
#[derive(Clone, Debug)]
pub enum Metric {
    Affine(Box<Affine>),
    /// A descriptor the port does not carry: its domain and the name of its class (for the decline).
    Other { domain: Id, class: &'static str },
}

impl Metric {
    pub fn domain(&self) -> Id {
        match self {
            Metric::Affine(metric) => metric.domain,
            Metric::Other { domain, .. } => *domain,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Snapshot<'a> {
    /// The interned strings (by [`Sid`]): slices of the Python strings the snapshot holds, alive for the call.
    pub strings: Vec<&'a [u8]>,
    pub schema: Sid,
    pub revision: Id,
    pub capabilities: u32,
    pub patches: Vec<Id>,
    pub domains: Vec<Domain>,
    pub metrics: Vec<Metric>,
    pub surface: Surface,
    pub vertices: Vec<Vertex>,
    pub chains: Vec<Chain>,
    pub uses: Vec<Use>,
    pub loops: Vec<Loop>,
    pub constraints: Vec<Constraint>,
    pub sectors: Vec<Sector>,
    pub certificates: Vec<Certificate>,
    pub corners: Vec<Corner>,
    pub junctions: Vec<Junction>,
    pub terminals: Vec<Terminal>,
    pub neighbours: Vec<Neighbour>,
}
