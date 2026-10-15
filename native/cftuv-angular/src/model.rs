use cftuv_core::rat::Rat;

pub type Ref = (i64, i64, i64);
pub type Vec2 = [Rat; 2];
#[derive(Clone, Debug)]
pub struct Point { pub id: i64, pub xy: Vec2 }
#[derive(Clone, Debug)]
pub struct Chain { pub reference: Ref, pub vertices: Vec<i64>, pub edges: Vec<i64> }
#[derive(Clone, Debug)]
pub struct Face {
    pub id: i64, pub vertices: Vec<i64>, pub edges: Vec<i64>, pub triangles: Vec<i64>,
}
#[derive(Clone, Debug)]
pub struct Site {
    pub key: (i64, i64, i64), pub cut: bool, pub anchor: i64,
    pub incoming: Option<Ref>, pub outgoing: Option<Ref>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chart { RationalAffine, LegacyPlanar, Unsupported }
#[derive(Clone, Debug)]
pub struct PatchInput {
    pub version: u32, pub patch_id: i64, pub owner_sign: i8,
    pub gram: [[Rat; 2]; 2], pub chart: Chart,
    pub band_or_ring_cut: bool,
    pub points: Vec<Point>, pub chains: Vec<Chain>, pub faces: Vec<Face>,
    pub sites: Vec<Site>, pub present_vertices: Vec<i64>,
    pub support_triangles: Option<Vec<i64>>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    Version, Chart, BandOrRingCut, InvalidMetric, DuplicateId, InvalidTopology,
    MissingAnchor, MissingChain, MissingPoint, NotEndpoint, DegenerateSupport,
    NoOwnerFace, OwnerDirectionsDisagree, FullTurn, InteriorSidesDisagree,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Decline { pub site: Option<usize>, pub reason: Reason }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Skip { OffJunction, BeyondChartReach, ForwardCollinear, NonReflex }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SymbolicEvent { Canonical, Normalize }
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Reflex {
    pub incoming_normal: Vec2, pub outgoing_normal: Vec2,
    pub turn_orientation: i8, pub norm_left: Rat, pub norm_right: Rat,
    pub dot: Rat, pub owner_cross: Rat, pub gram_det: Rat,
    // Только пропущенная геометрия; порядок ДО вызова меры, не после патча.
    pub before_measure: Vec<SymbolicEvent>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Decision { Skip(Skip), Reflex(Reflex) }
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SitePlan {
    pub ordinal: usize, pub key: (i64, i64, i64), pub decision: Decision,
    // Имена в порядке исходных изменений; RELATIONS_BUILT после меры/assembly.
    pub stage_before: Vec<&'static str>, pub stage_after_success: Vec<&'static str>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PatchPlan { pub sites: Vec<SitePlan> }
