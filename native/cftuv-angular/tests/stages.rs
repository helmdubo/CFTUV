//! Stage names of every site decision (the profile counters the shim writes per domain). The recorded corpus has no
//! off-junction and no beyond-chart-reach site (exact-plane charts carry no support triangles), so these witnesses are
//! what kills a mutation that drops one of those names or ignores a second support beyond the chart (ANGULAR mutation gate).
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
fn only(p:&PatchInput)->SitePlan {evaluate_patch(p).unwrap().sites.remove(0)}
#[test]
fn reflex_stages_plain_and_cut() {
    let s=only(&fixture());
    assert!(matches!(s.decision,Decision::Reflex(_)));
    assert_eq!(s.stage_before,vec!["ANGULAR_CORNERS_CONSIDERED","ANGULAR_REFLEX_CORNERS"]);
    assert_eq!(s.stage_after_success,vec!["ANGULAR_RELATIONS_BUILT"]);
    let mut p=fixture();p.sites[0].cut=true;
    let s=only(&p);
    assert_eq!(s.stage_before,vec!["ANGULAR_CUT_VERTICES_CONSIDERED","ANGULAR_REFLEX_CORNERS","ANGULAR_CUT_REFLEX_RELATIONS"]);
    assert_eq!(s.stage_after_success,vec!["ANGULAR_RELATIONS_BUILT"]);
}
#[test]
fn off_junction_stages_plain_and_cut() {
    for (cut,first) in [(false,"ANGULAR_CORNERS_CONSIDERED"),(true,"ANGULAR_CUT_VERTICES_CONSIDERED")] {
        let mut p=fixture();p.sites[0].incoming=None;p.sites[0].cut=cut;
        let s=only(&p);
        assert_eq!(s.decision,Decision::Skip(Skip::OffJunction));
        assert_eq!(s.stage_before,vec![first,"ANGULAR_CORNERS_OFF_JUNCTION"]);
        assert!(s.stage_after_success.is_empty());
    }
}
#[test]
fn incoming_support_beyond_chart_reach_stages() {
    let mut p=fixture();p.support_triangles=Some(vec![999]);
    let s=only(&p);
    assert_eq!(s.decision,Decision::Skip(Skip::BeyondChartReach));
    assert_eq!(s.stage_before,vec!["ANGULAR_CORNERS_CONSIDERED","ANGULAR_CORNERS_BEYOND_CHART_REACH"]);
    assert!(s.stage_after_success.is_empty());
}
#[test]
fn outgoing_support_beyond_chart_reach_after_ready_incoming() {
    let mut p=fixture();
    // Ребро 12 (выход) лежит только в треугольнике вне карты; ребро 11 (вход) — в L-грани карты.
    p.faces[0].edges=vec![10,11,112,13,14,15];
    p.points.push(Point{id:6,xy:[q(1),q(-1)]});
    p.faces.push(Face{id:10,vertices:vec![2,6,3],edges:vec![20,21,12],triangles:vec![91]});
    p.support_triangles=Some(vec![90]);
    let s=only(&p);
    assert_eq!(s.decision,Decision::Skip(Skip::BeyondChartReach));
    assert_eq!(s.stage_before,vec!["ANGULAR_CORNERS_CONSIDERED","ANGULAR_CORNERS_BEYOND_CHART_REACH"]);
    // Вход готов, выхода нет вовсе: это именованный отказ, а не пропуск.
    p.sites[0].outgoing=None;
    assert_eq!(evaluate_patch(&p).unwrap_err(),Decline{site:Some(0),reason:Reason::MissingChain});
}
#[test]
fn long_chains_take_the_edge_at_the_anchor_end() {
    // Цепь входа 5->0->1->2 кончается в якоре 2 (опора по ребру 11; первое ребро 15 идёт против касательной); выход 2->3->4 начинается в нём (ребро 12).
    let mut p=fixture();
    p.chains[0].vertices=vec![5,0,1,2];p.chains[0].edges=vec![15,10,11];
    p.chains[1].vertices=vec![2,3,4];p.chains[1].edges=vec![12,13];
    let plain=only(&fixture());let long=only(&p);
    assert_eq!(long.decision,plain.decision);
    assert_eq!(long.stage_before,plain.stage_before);
}
