//! Named declines of the whole-patch admission and of the support lookup. The recorded corpus never reaches them (the host
//! wire always writes version 1, a RationalAffine chart, a symmetric SPD gram and well-formed chains), so only these
//! witnesses keep each guard alive: a single-site mutation that drops one of them is killed here (ANGULAR mutation gate).
use cftuv_angular::*;
use cftuv_core::rat::Rat;
fn q(n:i64)->Rat {Rat::from_i64(n)}
fn fixture()->PatchInput {
    let xy=[[-2,-2],[0,-2],[0,0],[2,0],[2,2],[-2,2]];
    PatchInput{version:1,patch_id:7,owner_sign:1,
        gram:[[q(2),q(1)],[q(1),q(3)]],chart:Chart::RationalAffine,band_or_ring_cut:false,
        points:xy.iter().enumerate().map(|(id,p)|Point{id:id as i64,xy:[q(p[0]),q(p[1])]}).collect(),
        chains:vec![Chain{reference:(7,0,0),vertices:vec![1,2],edges:vec![11]},
            Chain{reference:(7,0,1),vertices:vec![2,3],edges:vec![12]}],
        faces:vec![Face{id:9,vertices:vec![0,1,2,3,4,5],edges:vec![10,11,12,13,14,15],triangles:vec![90]}],
        sites:vec![Site{key:(0,0,0),cut:false,anchor:2,incoming:Some((7,0,0)),outgoing:Some((7,0,1))}],
        present_vertices:vec![0,1,2,3,4,5],support_triangles:None}
}
fn whole(p:&PatchInput)->Reason {
    let decline=evaluate_patch(p).unwrap_err();
    assert_eq!(decline.site,None,"{decline:?}");
    decline.reason
}
fn at_site(p:&PatchInput)->Reason {
    let decline=evaluate_patch(p).unwrap_err();
    assert_eq!(decline.site,Some(0),"{decline:?}");
    decline.reason
}
#[test]
fn the_fixture_is_admitted() {
    assert!(matches!(evaluate_patch(&fixture()).unwrap().sites[0].decision,Decision::Reflex(_)));
}
#[test]
fn version_chart_and_band_decline_before_metric() {
    let mut p=fixture();p.version=2;assert_eq!(whole(&p),Reason::Version);
    let mut p=fixture();p.version=0;assert_eq!(whole(&p),Reason::Version);
    let mut p=fixture();p.chart=Chart::Unsupported;assert_eq!(whole(&p),Reason::Chart);
    let mut p=fixture();p.band_or_ring_cut=true;assert_eq!(whole(&p),Reason::BandOrRingCut);
    // Порядок проверок: версия раньше карты и пояса, пояс раньше метрики.
    let mut p=fixture();p.version=2;p.chart=Chart::Unsupported;p.band_or_ring_cut=true;assert_eq!(whole(&p),Reason::Version);
    let mut p=fixture();p.band_or_ring_cut=true;p.owner_sign=0;assert_eq!(whole(&p),Reason::BandOrRingCut);
}
#[test]
fn metric_must_be_symmetric_positive_definite_with_unit_owner() {
    let mut p=fixture();p.gram[0][1]=q(2);assert_eq!(whole(&p),Reason::InvalidMetric);
    let mut p=fixture();p.gram[1][0]=q(0);assert_eq!(whole(&p),Reason::InvalidMetric);
    let mut p=fixture();p.gram[0][0]=q(-2);p.gram[1][1]=q(-3);assert_eq!(whole(&p),Reason::InvalidMetric);
    let mut p=fixture();p.gram=[[q(1),q(1)],[q(1),q(1)]];assert_eq!(whole(&p),Reason::InvalidMetric);
    for sign in [0,2,-2] {let mut p=fixture();p.owner_sign=sign;assert_eq!(whole(&p),Reason::InvalidMetric);}
}
#[test]
fn duplicate_ids_decline() {
    let mut p=fixture();p.points.push(Point{id:0,xy:[q(9),q(9)]});assert_eq!(whole(&p),Reason::DuplicateId);
    let mut p=fixture();let f=p.faces[0].clone();p.faces.push(f);assert_eq!(whole(&p),Reason::DuplicateId);
    let mut p=fixture();let c=p.chains[0].clone();p.chains.push(c);assert_eq!(whole(&p),Reason::DuplicateId);
}
#[test]
fn malformed_topology_declines() {
    let mut p=fixture();p.chains[0].reference=(8,0,0);p.sites[0].incoming=Some((8,0,0));assert_eq!(whole(&p),Reason::InvalidTopology);
    let mut p=fixture();p.chains[0].vertices=vec![2];p.chains[0].edges=vec![];assert_eq!(whole(&p),Reason::InvalidTopology);
    let mut p=fixture();p.chains[0].edges=vec![11,12];assert_eq!(whole(&p),Reason::InvalidTopology);
    let mut p=fixture();p.chains[1].edges=vec![];assert_eq!(whole(&p),Reason::InvalidTopology);
    let mut p=fixture();p.faces[0].edges.pop();assert_eq!(whole(&p),Reason::InvalidTopology);
    let mut p=fixture();p.faces[0].vertices=vec![0,1];p.faces[0].edges=vec![10,11];assert_eq!(whole(&p),Reason::InvalidTopology);
}
#[test]
fn site_lookups_decline_by_name() {
    let mut p=fixture();p.sites[0].incoming=Some((7,0,9));assert_eq!(at_site(&p),Reason::MissingChain);
    let mut p=fixture();p.sites[0].outgoing=None;assert_eq!(at_site(&p),Reason::MissingChain);
    let mut p=fixture();p.sites[0].outgoing=Some((7,0,9));assert_eq!(at_site(&p),Reason::MissingChain);
    let mut p=fixture();p.points.retain(|x|x.id!=1);p.present_vertices.retain(|x|*x!=1);assert_eq!(at_site(&p),Reason::MissingPoint);
    // Якорь на середине цепи — не конец ни одной опоры.
    let mut p=fixture();p.chains[0].vertices=vec![1,2,3];p.chains[0].edges=vec![11,12];p.sites[0].anchor=2;assert_eq!(at_site(&p),Reason::NotEndpoint);
}
