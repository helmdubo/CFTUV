# ============================================================
# PHASE 3 — Chain-First Strongest-Frontier Builder
#
# 4 ПРАВКИ в cftuv/solve.py. Выполнять по порядку.
#
# ПРАВКА 1: Добавить поля в ScaffoldPatchPlacement
# ПРАВКА 2: Добавить build_order в ScaffoldQuiltPlacement
# ПРАВКА 3: Вставить новые функции (перед build_root_scaffold_map)
# ПРАВКА 4: Заменить build_root_scaffold_map
# ============================================================


# ============================================================
# ПРАВКА 1: Добавить поля в ScaffoldPatchPlacement
#
# Найди в solve.py:
#     notes: tuple[str, ...] = ()
# (в конце dataclass ScaffoldPatchPlacement)
#
# ПОСЛЕ этой строки добавь:
#     # Phase 3: envelope fields
#     status: str = "COMPLETE"
#     dependency_patches: tuple = ()
#     unplaced_chain_indices: tuple = ()
#     pinned: bool = False
#     origin_offset: tuple = (0.0, 0.0)  # sleeping field, Phase 5
# ============================================================


# ============================================================
# ПРАВКА 2: Добавить build_order в ScaffoldQuiltPlacement
#
# Найди в solve.py dataclass ScaffoldQuiltPlacement:
#     patches: dict[int, ScaffoldPatchPlacement] = field(default_factory=dict)
#
# ПОСЛЕ этой строки добавь:
#     build_order: list = field(default_factory=list)  # Phase 3: ChainRef tuples
# ============================================================


# ============================================================
# ПРАВКА 3: Вставить ВСЕ функции ниже
#
# Найди в solve.py строку:
#     def build_root_scaffold_map(
#
# ПЕРЕД этой строкой вставь весь блок ниже
# (от CHAIN_FRONTIER_THRESHOLD до конца _build_quilt_envelopes)
# ============================================================

CHAIN_FRONTIER_THRESHOLD = 0.3


def _cf_chain_source_points(chain):
    """Конвертирует chain.vert_cos в формат [(index, Vector)] для placement функций."""
    return [(i, co.copy()) for i, co in enumerate(chain.vert_cos)]


def _cf_determine_direction(chain, node):
    """UV direction для chain из базиса patch.

    H_FRAME → snap к (±1, 0)
    V_FRAME → snap к (0, ±1)
    FREE → проекция 3D direction на basis
    """
    if len(chain.vert_cos) < 2:
        if chain.frame_role == FrameRole.H_FRAME:
            return Vector((1.0, 0.0))
        if chain.frame_role == FrameRole.V_FRAME:
            return Vector((0.0, 1.0))
        return Vector((1.0, 0.0))

    chain_3d = chain.vert_cos[-1] - chain.vert_cos[0]

    if chain.frame_role == FrameRole.H_FRAME:
        dot = chain_3d.dot(node.basis_u)
        return Vector((1.0 if dot >= 0.0 else -1.0, 0.0))

    if chain.frame_role == FrameRole.V_FRAME:
        dot = chain_3d.dot(node.basis_v)
        return Vector((0.0, 1.0 if dot >= 0.0 else -1.0))

    # FREE: проекция на 2D basis space
    u_comp = chain_3d.dot(node.basis_u)
    v_comp = chain_3d.dot(node.basis_v)
    d = Vector((u_comp, v_comp))
    if d.length > 1e-8:
        return d.normalized()
    return Vector((1.0, 0.0))


def _cf_choose_seed_chain(graph, root_node):
    """Выбирает strongest chain root patch для seed placement.

    Предпочитает H/V с сильным соседним контекстом.
    Для WALL: FLOOR.DOWN сосед = нижняя кромка стены = лучший seed.

    Returns: (loop_index, chain_index, chain) или None
    """
    best_ref = None
    best_score = -1.0

    for loop_idx, loop in enumerate(root_node.boundary_loops):
        if loop.kind != LoopKind.OUTER:
            continue
        for chain_idx, chain in enumerate(loop.chains):
            score = 0.0

            # Role
            if chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
                score += 1.0
            else:
                score += 0.1

            # Chain length
            if len(chain.vert_cos) > 1:
                chain_len = sum(
                    (chain.vert_cos[i + 1] - chain.vert_cos[i]).length
                    for i in range(len(chain.vert_cos) - 1)
                )
                score += min(chain_len * 0.1, 0.5)

            # Neighbor semantic
            if chain.neighbor_kind == ChainNeighborKind.PATCH:
                neighbor_key = graph.get_patch_semantic_key(chain.neighbor_patch_id)
                patch_type = root_node.patch_type.value if hasattr(root_node.patch_type, 'value') else str(root_node.patch_type)
                if patch_type == 'WALL':
                    if neighbor_key == 'FLOOR.DOWN':
                        score += 0.5
                    elif neighbor_key == 'FLOOR.UP':
                        score += 0.2
                    elif neighbor_key.endswith('.SIDE'):
                        score += 0.15
                else:
                    if neighbor_key.endswith('.SIDE'):
                        score += 0.2

            if score > best_score:
                best_score = score
                best_ref = (loop_idx, chain_idx, chain)

    return best_ref


def _cf_is_adjacent(chain_a_idx, chain_b_idx, boundary_loop):
    """Два chain соседние в loop если делят corner."""
    for corner in boundary_loop.corners:
        if ((corner.prev_chain_index == chain_a_idx and corner.next_chain_index == chain_b_idx) or
                (corner.prev_chain_index == chain_b_idx and corner.next_chain_index == chain_a_idx)):
            return True
    return False


def _cf_find_anchors(chain_ref, chain, graph, point_registry, vert_to_placements, placed_refs):
    """Ищет anchor UV для start и end вершин chain.

    Same-patch: через corner adjacency (не берёт другую сторону SEAM_SELF).
    Cross-patch: через vert_index (seam connection).

    Returns: (start_uv, end_uv, known_endpoints)
    """
    patch_id, loop_index, chain_index = chain_ref
    node = graph.nodes.get(patch_id)
    if node is None or loop_index >= len(node.boundary_loops):
        return None, None, 0

    boundary_loop = node.boundary_loops[loop_index]
    start_vert = chain.start_vert_index
    end_vert = chain.end_vert_index
    start_uv = None
    end_uv = None

    # --- Same-patch: corner adjacency only ---
    for placed_ref in placed_refs:
        if placed_ref[0] != patch_id or placed_ref[1] != loop_index:
            continue
        p_chain_idx = placed_ref[2]
        if not _cf_is_adjacent(chain_index, p_chain_idx, boundary_loop):
            continue
        p_chain = boundary_loop.chains[p_chain_idx]
        p_last = len(p_chain.vert_indices) - 1

        if start_uv is None:
            if p_chain.end_vert_index == start_vert:
                key = (placed_ref[0], placed_ref[1], placed_ref[2], p_last)
                if key in point_registry:
                    start_uv = point_registry[key]
            elif p_chain.start_vert_index == start_vert:
                key = (placed_ref[0], placed_ref[1], placed_ref[2], 0)
                if key in point_registry:
                    start_uv = point_registry[key]

        if end_uv is None:
            if p_chain.end_vert_index == end_vert:
                key = (placed_ref[0], placed_ref[1], placed_ref[2], p_last)
                if key in point_registry:
                    end_uv = point_registry[key]
            elif p_chain.start_vert_index == end_vert:
                key = (placed_ref[0], placed_ref[1], placed_ref[2], 0)
                if key in point_registry:
                    end_uv = point_registry[key]

    # --- Cross-patch: vert_index match ---
    if start_uv is None and start_vert >= 0 and start_vert in vert_to_placements:
        for other_ref, pt_idx in vert_to_placements[start_vert]:
            if other_ref[0] != patch_id:
                key = (other_ref[0], other_ref[1], other_ref[2], pt_idx)
                if key in point_registry:
                    start_uv = point_registry[key]
                    break

    if end_uv is None and end_vert >= 0 and end_vert in vert_to_placements:
        for other_ref, pt_idx in vert_to_placements[end_vert]:
            if other_ref[0] != patch_id:
                key = (other_ref[0], other_ref[1], other_ref[2], pt_idx)
                if key in point_registry:
                    end_uv = point_registry[key]
                    break

    known = (1 if start_uv is not None else 0) + (1 if end_uv is not None else 0)
    return start_uv, end_uv, known


def _cf_score_candidate(chain_ref, chain, node, known, graph, placed_in_patch):
    """Chain-level score для frontier candidate.

    Scoring заморожен до стабилизации placement.
    """
    score = 0.0

    # Role (primary factor)
    if chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        score += 1.0
    else:
        score += 0.2

    # Known endpoints
    if known == 2:
        score += 0.8
    elif known == 1:
        score += 0.3

    # Neighbor semantic
    if chain.neighbor_kind == ChainNeighborKind.PATCH:
        neighbor_key = graph.get_patch_semantic_key(chain.neighbor_patch_id)
        if 'FLOOR' in neighbor_key:
            score += 0.3
        elif neighbor_key.endswith('.SIDE'):
            score += 0.15

    # Patch continuity (patch уже частично размещён)
    if placed_in_patch > 0:
        score += 0.2

    return score


def _cf_place_chain(chain, node, start_uv, end_uv, final_scale):
    """Размещает один chain в UV.

    Dispatch по role + known_endpoints к существующим placement функциям.
    H/V = snap к оси. FREE = guided interpolation/extension.

    Returns: list[Vector] UV позиций (по одной на каждую вершину chain).
    """
    source_pts = _cf_chain_source_points(chain)
    direction = _cf_determine_direction(chain, node)
    role = chain.frame_role

    if start_uv is not None and end_uv is not None:
        # Оба endpoints известны
        if role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
            return _build_frame_chain_between_anchors(source_pts, start_uv, end_uv, final_scale)
        else:
            return _build_guided_free_chain_between_anchors(
                node, source_pts, start_uv, end_uv, direction, None, final_scale)

    elif start_uv is not None:
        # Только start известен
        if role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
            return _build_frame_chain_from_one_end(source_pts, start_uv, direction, role, final_scale)
        else:
            return _build_guided_free_chain_from_one_end(
                node, source_pts, start_uv, direction, final_scale)

    elif end_uv is not None:
        # Только end известен — reverse, build from end, reverse back
        rev_pts = list(reversed(source_pts))
        rev_dir = Vector((-direction.x, -direction.y))
        if role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
            rev_uvs = _build_frame_chain_from_one_end(rev_pts, end_uv, rev_dir, role, final_scale)
        else:
            rev_uvs = _build_guided_free_chain_from_one_end(
                node, rev_pts, end_uv, rev_dir, final_scale)
        return list(reversed(rev_uvs))

    return []


def _cf_register_points(chain_ref, chain, uv_points, point_registry, vert_to_placements):
    """Регистрирует все точки chain в обоих registry."""
    patch_id, loop_index, chain_index = chain_ref
    for i, uv in enumerate(uv_points):
        key = (patch_id, loop_index, chain_index, i)
        point_registry[key] = uv.copy()

        if i < len(chain.vert_indices):
            vert_idx = chain.vert_indices[i]
            vert_to_placements.setdefault(vert_idx, []).append((chain_ref, i))


def _cf_build_envelopes(graph, quilt_patch_ids, placed_chains_map, placed_chain_refs, point_registry):
    """Группирует размещённые chains в ScaffoldPatchPlacement per patch.

    Вычисляет corners, bbox, status, dependencies.
    """
    patches = {}

    for patch_id in quilt_patch_ids:
        node = graph.nodes.get(patch_id)
        if node is None:
            continue

        # Собираем placed chains этого patch
        patch_placements = []
        for ref, placement in placed_chains_map.items():
            if ref[0] == patch_id:
                patch_placements.append(placement)

        if not patch_placements:
            patches[patch_id] = ScaffoldPatchPlacement(
                patch_id=patch_id, loop_index=-1,
                notes=('no_placed_chains',), status="EMPTY")
            continue

        # OUTER loop index
        outer_loop_index = -1
        for loop_idx, loop in enumerate(node.boundary_loops):
            if loop.kind == LoopKind.OUTER:
                outer_loop_index = loop_idx
                break

        if outer_loop_index < 0:
            patches[patch_id] = ScaffoldPatchPlacement(
                patch_id=patch_id, loop_index=-1,
                notes=('no_outer_loop',), status="UNSUPPORTED")
            continue

        boundary_loop = node.boundary_loops[outer_loop_index]

        # Corners из endpoints размещённых chains
        corner_positions = {}
        for corner_idx, corner in enumerate(boundary_loop.corners):
            prev_ref = (patch_id, outer_loop_index, corner.prev_chain_index)
            next_ref = (patch_id, outer_loop_index, corner.next_chain_index)
            if prev_ref in placed_chains_map:
                pts = placed_chains_map[prev_ref].points
                if pts:
                    corner_positions[corner_idx] = pts[-1][1].copy()
            elif next_ref in placed_chains_map:
                pts = placed_chains_map[next_ref].points
                if pts:
                    corner_positions[corner_idx] = pts[0][1].copy()

        # BBox
        all_pts = [pt for cp in patch_placements for _, pt in cp.points]
        if all_pts:
            bbox_min = Vector((min(p.x for p in all_pts), min(p.y for p in all_pts)))
            bbox_max = Vector((max(p.x for p in all_pts), max(p.y for p in all_pts)))
        else:
            bbox_min = Vector((0.0, 0.0))
            bbox_max = Vector((0.0, 0.0))

        # Status
        total_chains = len(boundary_loop.chains)
        placed_count = sum(
            1 for ci in range(total_chains)
            if (patch_id, outer_loop_index, ci) in placed_chain_refs
        )

        if placed_count >= total_chains:
            status = "COMPLETE"
        elif placed_count > 0:
            status = "PARTIAL"
        else:
            status = "EMPTY"

        # Unplaced chains
        unplaced = tuple(
            ci for ci in range(total_chains)
            if (patch_id, outer_loop_index, ci) not in placed_chain_refs
        )

        # Dependencies
        dep_set = set()
        for cp in patch_placements:
            for point_key, _ in cp.points:
                if point_key.patch_id != patch_id:
                    dep_set.add(point_key.patch_id)

        # Closure
        closure_error = 0.0
        closure_valid = True
        if status == "COMPLETE" and total_chains >= 2:
            sorted_pl = sorted(patch_placements, key=lambda cp: cp.chain_index)
            if sorted_pl[-1].points and sorted_pl[0].points:
                last_end = sorted_pl[-1].points[-1][1]
                first_start = sorted_pl[0].points[0][1]
                closure_error = (last_end - first_start).length
                closure_valid = closure_error < 0.05

        patches[patch_id] = ScaffoldPatchPlacement(
            patch_id=patch_id,
            loop_index=outer_loop_index,
            corner_positions=corner_positions,
            chain_placements=patch_placements,
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            closure_error=closure_error,
            closure_valid=closure_valid,
            notes=(),
            status=status,
            dependency_patches=tuple(sorted(dep_set)),
            unplaced_chain_indices=unplaced,
        )

    return patches


def build_quilt_scaffold_chain_frontier(graph, quilt_plan, final_scale):
    """Chain-first strongest-frontier builder.

    Строит scaffold для одного quilt chain-by-chain через единый frontier pool.
    Не различает chains одного patch и chains через seam.
    """
    quilt_scaffold = ScaffoldQuiltPlacement(
        quilt_index=quilt_plan.quilt_index,
        root_patch_id=quilt_plan.root_patch_id,
    )

    root_node = graph.nodes.get(quilt_plan.root_patch_id)
    if root_node is None:
        return quilt_scaffold

    # --- Seed chain ---
    seed_result = _cf_choose_seed_chain(graph, root_node)
    if seed_result is None:
        quilt_scaffold.patches[quilt_plan.root_patch_id] = ScaffoldPatchPlacement(
            patch_id=quilt_plan.root_patch_id, loop_index=-1,
            notes=('no_seed_chain',), status="UNSUPPORTED")
        return quilt_scaffold

    seed_loop_idx, seed_chain_idx, seed_chain = seed_result
    seed_ref = (quilt_plan.root_patch_id, seed_loop_idx, seed_chain_idx)

    # Registries
    point_registry = {}
    vert_to_placements = {}
    placed_chain_refs = set()
    placed_chains_map = {}
    build_order = []

    # Place seed от (0, 0)
    seed_src = _cf_chain_source_points(seed_chain)
    seed_dir = _cf_determine_direction(seed_chain, root_node)

    if seed_chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        seed_uvs = _build_frame_chain_from_one_end(
            seed_src, Vector((0.0, 0.0)), seed_dir, seed_chain.frame_role, final_scale)
    else:
        seed_uvs = _build_guided_free_chain_from_one_end(
            root_node, seed_src, Vector((0.0, 0.0)), seed_dir, final_scale)

    if not seed_uvs:
        quilt_scaffold.patches[quilt_plan.root_patch_id] = ScaffoldPatchPlacement(
            patch_id=quilt_plan.root_patch_id, loop_index=-1,
            notes=('seed_placement_failed',), status="UNSUPPORTED")
        return quilt_scaffold

    seed_placement = ScaffoldChainPlacement(
        patch_id=seed_ref[0], loop_index=seed_ref[1], chain_index=seed_ref[2],
        frame_role=seed_chain.frame_role, source_kind='chain',
        points=tuple(
            (ScaffoldPointKey(seed_ref[0], seed_ref[1], seed_ref[2], i), uv.copy())
            for i, uv in enumerate(seed_uvs)
        ),
    )

    placed_chain_refs.add(seed_ref)
    placed_chains_map[seed_ref] = seed_placement
    build_order.append(seed_ref)
    _cf_register_points(seed_ref, seed_chain, seed_uvs, point_registry, vert_to_placements)

    print(
        f"[CFTUV][Frontier] Seed: P{seed_ref[0]} L{seed_ref[1]}C{seed_ref[2]} "
        f"{seed_chain.frame_role.value} "
        f"({seed_uvs[0].x:.4f},{seed_uvs[0].y:.4f})"
        f"->({seed_uvs[-1].x:.4f},{seed_uvs[-1].y:.4f})"
    )

    # --- All chains in quilt (OUTER loops only) ---
    all_chain_pool = []
    for patch_id in quilt_plan.solved_patch_ids:
        node = graph.nodes.get(patch_id)
        if node is None:
            continue
        for loop_idx, loop in enumerate(node.boundary_loops):
            if loop.kind != LoopKind.OUTER:
                continue
            for chain_idx, chain in enumerate(loop.chains):
                ref = (patch_id, loop_idx, chain_idx)
                if ref != seed_ref:
                    all_chain_pool.append((ref, chain, node))

    # --- Frontier loop ---
    max_iter = len(all_chain_pool) + 10
    for iteration in range(1, max_iter + 1):
        best_ref = None
        best_score = -1.0
        best_data = None

        for ref, chain, node in all_chain_pool:
            if ref in placed_chain_refs:
                continue

            anchor_start, anchor_end, known = _cf_find_anchors(
                ref, chain, graph, point_registry, vert_to_placements, placed_chain_refs)

            if known == 0:
                continue

            placed_in_patch = sum(1 for pr in placed_chain_refs if pr[0] == ref[0])
            score = _cf_score_candidate(ref, chain, node, known, graph, placed_in_patch)

            if score > best_score:
                best_score = score
                best_ref = ref
                best_data = (chain, node, anchor_start, anchor_end)

        if best_ref is None or best_score < CHAIN_FRONTIER_THRESHOLD:
            break

        chain, node, anchor_start, anchor_end = best_data
        uv_points = _cf_place_chain(chain, node, anchor_start, anchor_end, final_scale)

        if not uv_points or len(uv_points) != len(chain.vert_cos):
            placed_chain_refs.add(best_ref)
            continue

        chain_placement = ScaffoldChainPlacement(
            patch_id=best_ref[0], loop_index=best_ref[1], chain_index=best_ref[2],
            frame_role=chain.frame_role, source_kind='chain',
            points=tuple(
                (ScaffoldPointKey(best_ref[0], best_ref[1], best_ref[2], i), uv.copy())
                for i, uv in enumerate(uv_points)
            ),
        )

        placed_chain_refs.add(best_ref)
        placed_chains_map[best_ref] = chain_placement
        build_order.append(best_ref)
        _cf_register_points(best_ref, chain, uv_points, point_registry, vert_to_placements)

        ep = (1 if anchor_start else 0) + (1 if anchor_end else 0)
        print(
            f"[CFTUV][Frontier] Step {iteration}: "
            f"P{best_ref[0]} L{best_ref[1]}C{best_ref[2]} "
            f"{chain.frame_role.value} score:{best_score:.2f} ep:{ep}"
        )

    # --- Build envelopes ---
    quilt_scaffold.patches = _cf_build_envelopes(
        graph, quilt_plan.solved_patch_ids,
        placed_chains_map, placed_chain_refs, point_registry)
    quilt_scaffold.build_order = list(build_order)

    total_placed = len(build_order)
    total_available = len(all_chain_pool) + 1
    print(
        f"[CFTUV][Frontier] Quilt {quilt_plan.quilt_index}: "
        f"placed {total_placed}/{total_available} chains"
    )

    return quilt_scaffold


# ============================================================
# ПРАВКА 4: Заменить build_root_scaffold_map
#
# Найди в solve.py ВСЮ функцию build_root_scaffold_map (от def до конца)
# и замени на:
# ============================================================

# def build_root_scaffold_map(
#     graph: PatchGraph,
#     solve_plan: Optional[SolvePlan] = None,
#     final_scale: float = 1.0,
# ) -> ScaffoldMap:
#     """Build ScaffoldMap using chain-first strongest-frontier algorithm."""
#     scaffold_map = ScaffoldMap()
#     if solve_plan is None:
#         return scaffold_map
#
#     for quilt in solve_plan.quilts:
#         quilt_scaffold = build_quilt_scaffold_chain_frontier(graph, quilt, final_scale)
#         scaffold_map.quilts.append(quilt_scaffold)
#
#     return scaffold_map
