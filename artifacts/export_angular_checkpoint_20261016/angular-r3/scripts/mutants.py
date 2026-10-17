"""Single-site mutants of the Rust ANGULAR code. Each entry: (id, file relative to native/, old text (exactly once), new text)."""
R = 'cftuv-angular/src/relations.rs'
O = 'cftuv-angular/src/owner.rs'
D = 'cftuv-angular/src/decimal_text.rs'
P = 'cftuv-python/src/angular.rs'

MUTANTS = [
    # relations.rs: square-class test and prices
    ('R01_square_denominator_ignored', R, '&a*&a==n && &b*&b==*d', '&a*&a==n'),
    ('R02_square_numerator_ignored', R, '&a*&a==n && &b*&b==*d', '&b*&b==*d'),
    ('R03_price_component_inverted', R, 'if !square(q) { for x in v', 'if square(q) { for x in v'),
    ('R04_price_zero_component_paid', R, 'if !x.is_zero() {out.push(SymbolicEvent::Canonical);}', 'out.push(SymbolicEvent::Canonical);'),
    ('R05_price_dot_zero_paid', R, 'if !d.is_zero() && !square(&ab)', 'if !square(&ab)'),
    ('R06_price_cross_zero_paid', R, 'if !c.is_zero() && !square(&ab) {out', 'if !square(&ab) {out'),
    ('R07_price_sin_uses_det_only', R, 'square(&t.div(&ab).expect("SPD nonzero norms"))', 'square(t)'),
    ('R08_price_sin_dropped', R, 'if !c.is_zero() && !square(&t.div', 'if false && !square(&t.div'),
    # relations.rs: admission
    ('R09_version_accepts_any', R, 'if p.version!=1 {return Err(Reason::Version);}', ''),
    ('R10_band_cut_admitted', R, 'if p.band_or_ring_cut {return Err(Reason::BandOrRingCut);}', ''),
    ('R11_det_sign_flipped', R, 'let det=g[0][0].mul(&g[1][1]).sub(&g[0][1].mul(&g[1][0]));', 'let det=g[0][0].mul(&g[1][1]).add(&g[0][1].mul(&g[1][0]));'),
    ('R12_asymmetric_gram_admitted', R, '|| g[0][1]!=g[1][0] ', ' '),
    ('R13_topology_edge_count_ignored', R, '|| x.edges.len()+1!=x.vertices.len())', ')'),
    ('R14_chain_patch_id_ignored', R, 'x.reference.0!=p.patch_id || ', ''),
    # relations.rs: site decisions and stages
    ('R15_cut_stage_names_swapped', R, 'if s.cut {"ANGULAR_CUT_VERTICES_CONSIDERED"} else {"ANGULAR_CORNERS_CONSIDERED"}', 'if !s.cut {"ANGULAR_CUT_VERTICES_CONSIDERED"} else {"ANGULAR_CORNERS_CONSIDERED"}'),
    ('R16_off_junction_stage_dropped', R, 'before.push("ANGULAR_CORNERS_OFF_JUNCTION"); ', ''),
    ('R17_missing_anchor_admitted', R, 'if !p.present_vertices.contains(&s.anchor) {return Err(fail(Reason::MissingAnchor));}', ''),
    ('R18_full_turn_admitted_as_collinear', R, 'if owner::dot(p,&v,&w).signum()<=0 {return Err(fail(Reason::FullTurn));}', ''),
    ('R19_cut_collinear_stage_dropped', R, 'if s.cut {before.push("ANGULAR_CUT_CHART_COLLINEAR");}', ''),
    ('R20_interior_sign_check_dropped', R, ' || l.signum()!=r.signum() {return', ' {return'),
    ('R21_reflex_test_inverted', R, 'if turn.signum()==l.signum() {Decision::Skip(Skip::NonReflex)}', 'if turn.signum()!=l.signum() {Decision::Skip(Skip::NonReflex)}'),
    ('R22_cut_reflex_stage_dropped', R, 'if s.cut {before.push("ANGULAR_CUT_REFLEX_RELATIONS");}', ''),
    ('R23_dot_uses_left_twice', R, 'let d=owner::dot(p,&n,&m);', 'let d=owner::dot(p,&n,&n);'),
    ('R24_cross_operands_swapped', R, 'let c=owner::cross(p,&n,&m);', 'let c=owner::cross(p,&m,&n);'),
    ('R25_turn_orientation_from_interior', R, 'turn_orientation:turn.signum()', 'turn_orientation:l.signum()'),
    ('R26_norms_swapped', R, 'norm_left:a,norm_right:b', 'norm_left:b,norm_right:a'),
    ('R27_beyond_stage_dropped', R, '{before.push("ANGULAR_CORNERS_BEYOND_CHART_REACH");Decision', '{Decision'),
    ('R28_relations_built_never', R, 'let after=if matches!(&decision,Decision::Reflex(_))', 'let after=if false'),
    ('R29_second_support_beyond_ignored', R, 'Support::Beyond=>None, Support::Ready(m,w)', 'Support::Beyond=>return Err(fail(Reason::MissingChain)), Support::Ready(m,w)'),
    # owner.rs
    ('O01_dot_gram_d_uses_a', O, '.add(&a[1].mul(&p.gram[1][0].mul(&b[0]).add(&p.gram[1][1].mul(&b[1]))))', '.add(&a[1].mul(&p.gram[1][0].mul(&b[0]).add(&p.gram[0][0].mul(&b[1]))))'),
    ('O02_cross_ignores_owner_sign', O, '.mul(&Rat::from_i64(p.owner_sign as i64))', ''),
    ('O03_tail_edge_from_head', O, '(chain.vertices[n-2],chain.vertices[n-1],chain.edges[n-2])', '(chain.vertices[n-2],chain.vertices[n-1],chain.edges[0])'),
    ('O04_tail_tangent_reversed', O, '(chain.vertices[n-2],chain.vertices[n-1],chain.edges[n-2])', '(chain.vertices[n-1],chain.vertices[n-2],chain.edges[n-2])'),
    ('O05_head_tangent_reversed', O, '(chain.vertices[0],chain.vertices[1],chain.edges[0])', '(chain.vertices[1],chain.vertices[0],chain.edges[0])'),
    ('O06_beyond_face_skipped', O, 'return Ok(Support::Beyond); }', 'continue; }'),
    ('O07_zero_area_face_used', O, 'if area.is_zero() { continue; }', ''),
    ('O08_same_direction_nonstrict', O, 'let same=dot(p,&tangent,&direction).signum()>0;', 'let same=dot(p,&tangent,&direction).signum()>=0;'),
    ('O09_owner_left_inverted', O, 'let owner_left=coordinate_left==(p.owner_sign>0);', 'let owner_left=coordinate_left!=(p.owner_sign>0);'),
    ('O10_normal_rotated_other_way', O, 'let normal=[y.neg().mul(&s), x.mul(&s)];', 'let normal=[y.mul(&s), x.neg().mul(&s)];'),
    ('O11_disagree_allows_opposite', O, '|| dot(p,first,item).signum()<=0 {', ' {'),
    ('O12_disagree_check_dropped', O, 'if !cross(p,first,item).is_zero() || dot(p,first,item).signum()<=0 {', 'if false {'),
    ('O13_degenerate_tangent_admitted', O, 'if dot(p,&tangent,&tangent).is_zero() { return Err(Reason::DegenerateSupport); }', ''),
    ('O14_direction_from_wrong_vertex', O, 'let direction=sub(cycle[(ordinal+1)%cycle.len()],cycle[ordinal]);', 'let direction=sub(cycle[ordinal],cycle[(ordinal+1)%cycle.len()]);'),
    # decimal_text.rs (crate API, not on the product wire)
    ('D01_fraction_digits_always_counted', D, 'if dot { fractional += 1; }', 'fractional += 1;'),
    ('D02_negative_exponent_ignored', D, 'if neg { exp = -exp; }', ''),
    ('D03_sign_dropped', D, "let negative = text.starts_with('-');", 'let negative = false;'),
    ('D04_exponent_bound_dropped', D, 'if exp > MAX_DECIMAL_EXPONENT_ABS { return Err(ExponentOutOfRange); }', ''),
    # cftuv-python/src/angular.rs: the PyO3 wire reader/writer
    ('P01_skip_names_swapped', P, 'Skip::OffJunction=>"OffJunction",Skip::BeyondChartReach=>"BeyondChartReach"', 'Skip::OffJunction=>"BeyondChartReach",Skip::BeyondChartReach=>"OffJunction"'),
    ('P02_event_names_swapped', P, 'SymbolicEvent::Canonical=>"canonical",SymbolicEvent::Normalize=>"normalize"', 'SymbolicEvent::Canonical=>"normalize",SymbolicEvent::Normalize=>"canonical"'),
    ('P03_output_norms_swapped', P, 'rat_out(py,&r.norm_left)?,rat_out(py,&r.norm_right)?', 'rat_out(py,&r.norm_right)?,rat_out(py,&r.norm_left)?'),
    ('P04_site_refs_swapped', P, 'incoming:optional_ref(&row.get_item(3)?)?,outgoing:optional_ref(&row.get_item(4)?)?', 'incoming:optional_ref(&row.get_item(4)?)?,outgoing:optional_ref(&row.get_item(3)?)?'),
    ('P05_owner_sign_forced_positive', P, 'owner_sign:owner as i8', 'owner_sign:1'),
    ('P06_denominator_sign_check_dropped', P, 'if num::signum(&denominator)<=0 {return shape("positive rational denominator required");}', ''),
    ('P07_band_flag_dropped', P, 'band_or_ring_cut:boolean(&item(d,"band_or_ring_cut")?)?', 'band_or_ring_cut:false'),
    ('P08_support_triangles_ignored', P, 'support_triangles:if support.is_none(){None}else{Some(ints(&support)?)}', 'support_triangles:None'),
    ('P09_gram_rows_swapped', P, 'let gram=[vector(&g.get_item(0)?)?,vector(&g.get_item(1)?)?];', 'let gram=[vector(&g.get_item(1)?)?,vector(&g.get_item(0)?)?];'),
    ('P10_turn_orientation_negated', P, 'r.turn_orientation,', '-r.turn_orientation,'),
    ('P11_stage_after_dropped', P, 'let after=PyTuple::new(py,site.stage_after_success.iter().copied())?;', 'let after=PyTuple::empty(py);'),
    ('P12_ordinal_shifted', P, 'output.push((site.ordinal,site.key,', 'output.push((site.ordinal+1,site.key,'),
]
