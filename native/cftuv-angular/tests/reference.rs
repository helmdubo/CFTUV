//! Ручные геометрические свидетели; компиляция/исполнение ещё не проверены.
use cftuv_angular::*;
use cftuv_core::rat::Rat;
fn q(n:i64)->Rat {Rat::from_i64(n)}
fn fixture()->PatchInput {
    // CCW L-контур: в(0,0) вход вверх, выход вправо, внутренний угол270°.
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
fn reflex(p:&PatchInput)->Reflex {
    match evaluate_patch(p).unwrap().sites.remove(0).decision {Decision::Reflex(x)=>x,other=>panic!("{other:?}")}
}
#[test]
fn skew_gram_owner_and_endpoint_witness() {
    let p=fixture(); let r=reflex(&p);
    assert_eq!(r.incoming_normal,[q(-6),q(2)]);
    assert_eq!(r.outgoing_normal,[q(-2),q(4)]);
    assert_eq!((r.norm_left,r.norm_right,r.dot,r.owner_cross,r.gram_det),(q(60),q(40),q(20),q(-20),q(5)));
    assert_eq!(r.turn_orientation,-1);
    assert_eq!(r.before_measure,vec![SymbolicEvent::Canonical;4].into_iter()
        .chain(vec![SymbolicEvent::Normalize;3]).collect::<Vec<_>>());
    let mut flipped=p; flipped.owner_sign=-1;
    let f=reflex(&flipped);
    assert_eq!(f.incoming_normal,[q(-6),q(2)]);
    assert_eq!(f.turn_orientation,1); assert_eq!(f.owner_cross,q(20));
}
#[test]
fn hole_face_clockwise_boundary() {
    let mut p=fixture();
    // Обратная грань, вход справа, выход вниз: внутренность справа.
    p.faces[0].vertices=vec![5,4,3,2,1,0];
    p.faces[0].edges=vec![14,13,12,11,10,15];
    p.chains[0].vertices=vec![3,2]; p.chains[0].edges=vec![12];
    p.chains[1].vertices=vec![2,1]; p.chains[1].edges=vec![11];
    let r=reflex(&p);
    assert_eq!(r.turn_orientation,1);
    assert_eq!(r.incoming_normal,[q(-2),q(4)]);
    assert_eq!(r.outgoing_normal,[q(-6),q(2)]);
    assert_eq!(r.owner_cross,q(20));
}
#[test]
fn no_owner_and_disagreeing_owner_are_named() {
    let mut p=fixture(); p.faces.clear();
    assert_eq!(evaluate_patch(&p).unwrap_err(),Decline{site:Some(0),reason:Reason::NoOwnerFace});
    let mut p=fixture();
    // Другая физическая сторона ребра1->2, а не просто иной winding.
    p.points.push(Point{id:6,xy:[q(2),q(-1)]});
    p.faces.push(Face{id:10,vertices:vec![1,6,2],edges:vec![20,21,11],triangles:vec![90]});
    assert_eq!(evaluate_patch(&p).unwrap_err().reason,Reason::OwnerDirectionsDisagree);
}
#[test]
fn collinear_forward_and_backward_are_distinct() {
    let mut p=fixture();
    // Прямая нижняя граница прямоугольника с промежуточной вершиной.
    p.points=vec![Point{id:0,xy:[q(-2),q(0)]},Point{id:1,xy:[q(0),q(0)]},
        Point{id:2,xy:[q(2),q(0)]},Point{id:3,xy:[q(2),q(2)]},Point{id:4,xy:[q(-2),q(2)]}];
    p.faces[0].vertices=vec![0,1,2,3,4];p.faces[0].edges=vec![11,12,13,14,15];
    p.chains[0].vertices=vec![0,1];p.chains[1].vertices=vec![1,2];p.sites[0].anchor=1;p.sites[0].cut=true;
    let plan=evaluate_patch(&p).unwrap();
    assert_eq!(plan.sites[0].decision,Decision::Skip(Skip::ForwardCollinear));
    assert_eq!(plan.sites[0].stage_before,vec!["ANGULAR_CUT_VERTICES_CONSIDERED","ANGULAR_COLLINEAR_SKIPPED","ANGULAR_CUT_CHART_COLLINEAR"]);
    p.chains[1].vertices=vec![2,1];
    assert_eq!(evaluate_patch(&p).unwrap_err().reason,Reason::FullTurn);
}
#[test]
fn site_prefix_is_plan_only_until_whole_patch_admitted() {
    let mut p=fixture(); let mut off=p.sites[0].clone();off.incoming=None;off.anchor=999;
    p.sites.insert(0,off);
    let plan=evaluate_patch(&p).unwrap();
    assert_eq!(plan.sites[0].decision,Decision::Skip(Skip::OffJunction));
    assert_eq!(plan.sites[1].stage_after_success,vec!["ANGULAR_RELATIONS_BUILT"]);
    let mut late=p.sites[1].clone();late.anchor=999;p.sites.push(late);
    assert_eq!(evaluate_patch(&p).unwrap_err(),Decline{site:Some(2),reason:Reason::MissingAnchor});
    // Result содержит только отказ, а не частичный patch/effect handle.
}
#[test]
fn chart_reach_and_non_spd_admission() {
    let mut p=fixture();p.support_triangles=Some(vec![999]);
    assert_eq!(evaluate_patch(&p).unwrap().sites[0].decision,Decision::Skip(Skip::BeyondChartReach));
    p.gram=[[q(1),q(2)],[q(2),q(1)]];
    assert_eq!(evaluate_patch(&p).unwrap_err(),Decline{site:None,reason:Reason::InvalidMetric});
}
#[test]
fn zero_components_and_square_product_do_not_pay_constant_prices() {
    let mut p=fixture();p.gram=[[q(1),q(0)],[q(0),q(1)]];
    assert!(reflex(&p).before_measure.is_empty());
    p.gram=[[q(2),q(0)],[q(0),q(2)]];
    let r=reflex(&p);
    assert_eq!((r.norm_left,r.norm_right,r.dot),(q(32),q(32),q(0)));
    assert_eq!(r.before_measure,vec![SymbolicEvent::Canonical;2]);
    p.gram=[[q(2),q(0)],[q(0),q(3)]];
    let r=reflex(&p);
    assert_eq!((r.norm_left,r.norm_right,r.gram_det),(q(72),q(48),q(6)));
    assert_eq!(r.before_measure,vec![SymbolicEvent::Canonical,SymbolicEvent::Canonical,SymbolicEvent::Normalize]);
}
#[test]
fn late_beyond_precedes_disagreement_during_owner_scan() {
    let mut p=fixture();
    p.points.push(Point{id:6,xy:[q(2),q(-1)]});
    let opposite=Face{id:10,vertices:vec![1,6,2],edges:vec![20,21,11],triangles:vec![90]};
    let mut beyond=p.faces[0].clone();beyond.id=11;beyond.triangles=vec![999];
    p.faces.push(opposite);p.faces.push(beyond);p.support_triangles=Some(vec![90]);
    assert_eq!(evaluate_patch(&p).unwrap().sites[0].decision,Decision::Skip(Skip::BeyondChartReach));
}
#[test]
fn zero_area_candidate_is_ignored_before_valid_owner() {
    let mut p=fixture();
    p.points.push(Point{id:6,xy:[q(0),q(-1)]});
    p.faces.insert(0,Face{id:10,vertices:vec![1,2,6],edges:vec![11,20,21],triangles:vec![90]});
    // Все три x=0: первая грань имеет нулевую площадь. У L-грани
    // внутренняя сторона входного ребра западная: JG(0,2)=(-6,2).
    assert_eq!(reflex(&p).incoming_normal,[q(-6),q(2)]);
}
#[test]
fn multiple_agreeing_owner_faces_keep_same_support() {
    let mut p=fixture();
    p.points.push(Point{id:6,xy:[q(-1),q(-1)]});
    p.faces.push(Face{id:10,vertices:vec![1,2,6],edges:vec![11,20,21],triangles:vec![90]});
    // Треугольник и L лежат западнее ориентированного ребра1->2.
    // Оба дают один ненормированный JG(0,2), независимо от площади.
    assert_eq!(reflex(&p).incoming_normal,[q(-6),q(2)]);
}
#[test]
fn coincident_endpoint_declines_before_face_scan() {
    let mut p=fixture();p.points[1].xy=[q(0),q(0)];
    // Концы входного ребра совпали; SPD-норма нулевого касательного=0.
    assert_eq!(evaluate_patch(&p).unwrap_err(),Decline{site:Some(0),reason:Reason::DegenerateSupport});
}
#[test]
fn independently_owned_edges_with_opposite_interior_sides_decline() {
    let mut p=fixture();
    p.points.push(Point{id:6,xy:[q(-1),q(-1)]});
    p.points.push(Point{id:7,xy:[q(1),q(-1)]});
    p.faces=vec![Face{id:10,vertices:vec![1,2,6],edges:vec![11,20,21],triangles:vec![90]},
        Face{id:11,vertices:vec![2,7,3],edges:vec![22,23,12],triangles:vec![91]}];
    // Вход вверх: владелец слева(запад). Выход вправо: владелец
    // справа(юг). Каждое ребро имеет ровно одну ненулевую грань,
    // но знаки cross(tangent,normal) противоположны.
    assert_eq!(evaluate_patch(&p).unwrap_err(),Decline{site:Some(0),reason:Reason::InteriorSidesDisagree});
}
#[test]
fn convex_turn_emits_no_measure_or_relation_success() {
    let mut p=fixture();p.points[3].xy=[q(-1),q(1)];
    // Вход(0,2), выход(-1,1): cross=+2, CCW-грань слева.
    // Произведение turn*interior положительно: угол не reflex.
    let plan=evaluate_patch(&p).unwrap();
    assert_eq!(plan.sites[0].decision,Decision::Skip(Skip::NonReflex));
    assert_eq!(plan.sites[0].stage_before,vec!["ANGULAR_CORNERS_CONSIDERED"]);
    assert!(plan.sites[0].stage_after_success.is_empty());
}
#[test]
fn mixed_square_norms_price_one_component_and_only_cross() {
    let mut p=fixture();p.gram=[[q(1),q(0)],[q(0),q(2)]];
    let r=reflex(&p);
    // JG(0,2)=(-4,0), JG(2,0)=(0,2): A=16,B=8,D=0,C=-8,T=2.
    // Только правая ненулевая unit-компонента иррациональна;
    // cross иррационален, но sin имеет sqrt(T/AB)=sqrt(1/64).
    assert_eq!((r.norm_left,r.norm_right,r.dot,r.owner_cross,r.gram_det),(q(16),q(8),q(0),q(-8),q(2)));
    assert_eq!(r.before_measure,vec![SymbolicEvent::Canonical,SymbolicEvent::Normalize]);
}
#[test]
fn large_square_factor_preserves_nonsquare_class_and_counter_order() {
    let mut p=fixture();
    for point in &mut p.points {point.xy=[point.xy[0].mul(&q(1_000_003)),point.xy[1].mul(&q(1_000_003))];}
    let r=reflex(&p);
    // Масштаб k=1000003 умножает касательные и JG-нормали на k,
    // нормы/скалярные произведения на k²=1000006000009.
    // Квадратный множитель не меняет ни одной square-class проверки.
    assert_eq!((r.norm_left,r.norm_right,r.dot),(q(60_000_360_000_540),q(40_000_240_000_360),q(20_000_120_000_180)));
    assert_eq!(r.incoming_normal,[q(-6_000_018),q(2_000_006)]);
    assert_eq!(r.before_measure,vec![SymbolicEvent::Canonical;4].into_iter()
        .chain(vec![SymbolicEvent::Normalize;3]).collect::<Vec<_>>());
}
#[test]
fn skew_gram_exact_dot_cancellation_avoids_normalize_price() {
    let mut p=fixture();p.points[3].xy=[q(3),q(-1)];
    let r=reflex(&p);
    // G(3,-1)=(5,0), поэтому m=(0,5), n=(-6,2).
    // Gm=(5,15), n.Gm=-30+30=0, хотя G недиагональна.
    // A=60,B=75,C=-30,T=5: AB=4500 nonsquare, T/AB=1/900 square.
    // Три ненулевые unit-компоненты платят canonical; только cross normalize.
    assert_eq!(r.outgoing_normal,[q(0),q(5)]);
    assert_eq!((r.norm_left,r.norm_right,r.dot,r.owner_cross),(q(60),q(75),q(0),q(-30)));
    assert_eq!(r.before_measure,vec![SymbolicEvent::Canonical,SymbolicEvent::Canonical,SymbolicEvent::Canonical,SymbolicEvent::Normalize]);
}
