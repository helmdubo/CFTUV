"""Single-site mutants of the native chain_station (R3+ S2). Each entry: (id, file relative to native/, old text (exactly once), new text).

Runner: station_mutants_run.py (a fast wheel per mutant, then tests/test_native_chain_station.py incl. the field corpus; killed = the suite fails).
Known equivalent sites (not listed): the budget comparisons `>` vs `>=` (B^2 = 1/40000 is not dyadic, a squared distance of binary64 points never equals it), the
chord of `not_straight` through `two` (cross(two, whole) = -cross(one, whole)) or through `turn` (whole = one + two, so cross(one, whole) = cross(one, two)), `reach <= 0` vs `< 0`
of the segment (at reach = 0 both branches compare |along|^2 |whole|^2 with B^2 |whole|^2), a permutation of the axes of every point (an isometry), the `near_first == near_second`
refusal of `joint_arms` (before == after then meets `not_straight` with whole = 0: NOT_MANIFOLD, never a straight joint).
"""
L = 'cftuv-station/src/lib.rs'
P = 'cftuv-python/src/chain_station.rs'
S = 'cftuv-python/python/cftuv_native/chain_station_op.py'

MUTANTS = [
    ('L01_exponent_not_shifted_by_stripped_zeros', L, '    e += zeros as i32;\n', '\n'),
    ('L02_sign_of_coordinate_dropped', L, '    if negative {\n        -value\n    } else {\n        value\n    }', '    let _ = negative;\n    value'),
    ('L03_surface_face_overwrite_keeps_the_first', L, 'Some(&at) => faces[at] = record,', 'Some(_) => {}'),
    ('L04_neighbour_face_shadows_the_surface_face', L, '            if slot.contains_key(&face.id) {\n                continue;\n            }\n', ''),
    ('L05_neighbour_position_overwrites', L, '                if entry.is_none() {', '                if true {'),
    ('L06_newell_skips_the_last_triangle', L, 'for index in 1..points.len().saturating_sub(1) {', 'for index in 1..points.len().saturating_sub(2) {'),
    ('L07_within_forgets_the_count', L, 'let lifted = [&plane.count * &point[0] - &plane.sum[0],', 'let lifted = [&point[0] - &plane.sum[0],'),
    ('L08_inert_one_way', L, '                self.within(two, self.points(first))', '                { let _ = (two, first); true }'),
    ('L09_flat_together_smallest_face', L, '(&plane.squared, self.input.face_rank[self.faces[index].id as usize]) > (&top.squared,', '(&plane.squared, self.input.face_rank[self.faces[index].id as usize]) < (&top.squared,'),
    ('L10_float_filter_unsound_low_margin', L, '    if square <= limit * (1.0 - 1e-6) {', '    if square <= limit * (1.0 + 1e-3) {'),
    ('L11_segment_start_branch_dropped', L, '        if reach <= IBig::ZERO {\n            return Ok(self.budget.beyond(&dot(&along, &along), &IBig::ONE));\n        }\n', ''),
    ('L12_segment_far_end_from_start', L, '            let far = sub(middle, end);', '            let far = sub(middle, start);'),
    ('L13_backward_turn_admitted', L, '        if dot(&one, &two) <= IBig::ZERO || dot(&turn, &turn)', '        if dot(&turn, &turn)'),
    ('L14_bend_limit_inverted', L, 'dot(&turn, &turn) * &self.bend_denominator2 > &self.bend_numerator2 *', 'dot(&turn, &turn) * &self.bend_numerator2 > &self.bend_denominator2 *'),
    ('L15_chord_against_the_far_arm', L, '        let off = cross(&one, &whole);\n        if self.budget.beyond(&dot(&off, &off), &whole_square) {', '        let off = cross(&one, &whole);\n        if self.budget.beyond(&dot(&off, &off), &two_square) {'),
    ('L16_corner_vertex_not_a_junction', L, '    if facts.corner[vertex as usize] {\n        return required(Reason::Junction);\n    }\n', ''),
    ('L17_border_expects_two_faces', L, '    if kind == ChainKind::DecalSource {\n        1\n    } else {\n        2\n    }', '    let _ = kind;\n    2'),
    ('L18_unknown_side_for_every_kind', L, 'required(if kind == ChainKind::Seam { Reason::NeighbourSideUnknown }', 'required(if kind != ChainKind::DecalSource { Reason::NeighbourSideUnknown }'),
    ('L19_chain_edges_taken_as_transverse', L, '        if ends.contains(other) {\n            continue;\n        }\n', ''),
    ('L20_two_patches_not_a_junction', L, ' || facts.faces[sides[0]].patch != facts.faces[sides[1]].patch', ''),
    ('L21_pairs_unsorted', L, '    pairs.sort_by_key(|(first, second)| (face_rank(facts, *first), face_rank(facts, *second)));\n', ''),
    ('L22_deep_groups_never_demoted', L, '                deep.insert(*key);', '                let _ = key;'),
    ('L23_joints_never_agree', L, '    if demoted.is_empty() {\n        return;\n    }', '    if true {\n        return;\n    }'),
    ('L24_ring_starts_at_the_largest_name', L, 'chain = *seen.iter().min_by_key(|item| rank(**item))', 'chain = *seen.iter().max_by_key(|item| rank(**item))'),
    ('L25_joint_vertex_listed_twice', L, '                if !line.is_empty() && ordinal == first && line.last().unwrap().0 == vertices[ordinal] {', '                if false {'),
    ('L26_carried_vertex_opens_no_window', L, '            if carried[index] {\n                anchor = index;\n                continue;\n            }', '            if carried[index] {\n                continue;\n            }'),
    ('L27_window_skips_its_last_vertex', L, 'for (point, _) in &line[anchor + 1..=index] {', 'for (point, _) in &line[anchor + 1..index] {'),
    ('L29_joint_with_a_third_chain', L, '    if ends_at != 2 || chain.closed || other.closed {', '    if chain.closed || other.closed {'),
    ('L30_partner_outside_the_domain', L, '.find(|item| *item != index && in_domain.contains(&input.chains[*item].id));', '.find(|item| *item != index);'),
    ('L31_closed_chain_keeps_the_repeated_end', L, 'let closed = if n > 1 && vertices[0] == vertices[n - 1] { n - 1 } else { n };', 'let closed = n;'),
    ('L32_window_drift_never_demotes', L, '            if beyond {\n', '            if beyond && false {\n'),
    ('P01_ids_ranked_backwards', P, 'order.sort_by(|a, b| self.values[*a as usize].cmp(self.values[*b as usize]));', 'order.sort_by(|a, b| self.values[*b as usize].cmp(self.values[*a as usize]));'),
    ('P02_point_x_read_as_y', P, 'Ok([float(&class.get(object, 0)?)?, float(&class.get(object, 1)?)?, float(&class.get(object, 2)?)?])', 'Ok([float(&class.get(object, 1)?)?, float(&class.get(object, 1)?)?, float(&class.get(object, 2)?)?])'),
    ('P03_unavailable_position_declined', P, '} else if position.get_type().as_ptr() as usize != b.unavailable_position_ptr {', '} else if true {'),
    ('P04_chain_kinds_swapped', P, '                0 => ChainKind::DecalSource,\n                1 => ChainKind::Seam,', '                0 => ChainKind::Seam,\n                1 => ChainKind::DecalSource,'),
    ('P05_closed_negated', P, '            let closed = boolean(&b.chain.get(&item, 2)?)?;', '            let closed = !boolean(&b.chain.get(&item, 2)?)?;'),
    ('P06_pair_written_backwards', P, 'pairs.push(PyTuple::new(py, [one, two])?);', 'pairs.push(PyTuple::new(py, [two, one])?);'),
    ('P07_domain_chains_dropped', P, '            domain_chains.push(self.chains.intern(chain));', '            let _ = self.chains.intern(chain);'),
    ('P08_neighbours_ignored', P, '            neighbours.push(NeighbourFace { id, patch, vertices, positions });', '            let _ = (id, patch, vertices, positions);'),
    ('S01_free_under_chart_never_named', S, 'named if reason == 0 else reasons[reason]', 'reasons[reason]'),
    ('S02_payload_mode_ignored', S, '    full = surface.payload_mode is full_mode\n', '    full = True\n'),
]
