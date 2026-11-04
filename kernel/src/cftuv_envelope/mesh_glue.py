"""Склейка видов доменов в массивы меша декали (MESH_ASSEMBLY_BUFFERS_V1, срез B4a): эталон, с которым побитово сверен нативный `cftuv_native.mesh_assemble`.

По номерам сварки (`mesh_weld.weld_core`: у каждого домена номер вершины меша для каждой локальной вершины) грани домена переписываются в номера меша, UV и
владельцы граней идут подряд в порядке доменов, у каждой грани — номер патча её домена, швы домена — пары `(меньшая, большая)` вершин меша. Ни одно число
здесь не считается: только перекладка. Порядок — порядок доменов и порядок граней внутри домена; множество швов упорядочит вызывающий вместе со складками.

Единственное новое число — UV вершины закона `WALL_EDGE_SPLIT_V1` (`mesh_weld.WallPlanV1.splits`): более короткий узел стены входит в ребро длинной стены соседа как ещё один угол его грани, а его
петля — линейно по доле вдоль ребра между петлями концов ребра. Позиции не меняются, ступень остаётся, общая часть стены становится общим ребром двух граней (складка `cross_domain_seams` помечает его как любое
такое ребро). Шов вида на разделённом ребре становится двумя швами. Деление, для которого нет ребра в гранях домена либо ребро уже поделено, не делается и считается (`unplaced`).
"""

from __future__ import annotations


def _split_plan(d_faces, entries) -> tuple:
    """`({(грань, угол): (узел домена, номер узла, петля якоря, петля вершины, доля)}, не поставлено)`: куда в гранях домена встают деления `entries` (порядок делений и граней).

    Угол `k` грани — вершина, ЗА которой встаёт новая. Ребро ищется в порядке граней и углов (первое вхождение); второе деление того же ребра не ставится.
    """

    wanted = {}
    for order, (_long, anchor, vertex, short, node, share) in enumerate(entries):
        wanted.setdefault(frozenset((anchor, vertex)), (order, anchor, vertex, short, node, share))
    unplaced = len(entries) - len(wanted)
    plan: dict = {}
    loop_at = 0
    for number, loop in enumerate(d_faces):
        size = len(loop)
        for corner in range(size):
            found = wanted.pop(frozenset((loop[corner], loop[(corner + 1) % size]))) if frozenset((loop[corner], loop[(corner + 1) % size])) in wanted else None
            if found is not None:
                _order, anchor, _vertex, short, node, share = found
                here, there = loop_at + corner, loop_at + (corner + 1) % size
                plan[(number, corner)] = (short, node, *((here, there) if loop[corner] == anchor else (there, here)), share)
        loop_at += size
    return plan, unplaced + len(wanted)


def glue_domains(patch_ids, domains, index, splits=()) -> tuple:
    """`(грани, uv, домен граней, владельцы граней, пары швов, деления по доменам, не поставлено)` по доменам `(грани, uv, владельцы, швы)`, их патчам, номерам сварки и делениям `splits` (`WallPlanV1.splits`).

    Грань — кортеж номеров вершин меша; пары швов — множество кортежей `(меньшая, большая)`. Номер вне вершин домена — `IndexError`, как у кортежа.
    Деления по доменам — кортеж на домен из `(петля вида, перед которой встала новая, петля якоря, петля вершины, доля)` по возрастанию места: так наклоны петель вида превращаются в наклоны петель меша.
    """

    faces, uvs, face_domain, face_owner = [], [], [], []
    seam_pairs: set = set()
    by_domain: dict = {}
    for entry in splits:
        by_domain.setdefault(entry[0], []).append(entry)
    placed_by_domain = []
    unplaced = 0
    for ordinal, (patch_id, (d_faces, d_uvs, d_owner, d_seams), mapping) in enumerate(zip(patch_ids, domains, index)):
        entries = by_domain.get(ordinal)
        if entries is None:
            placed_by_domain.append(())
            faces.extend(tuple(mapping[item] for item in loop) for loop in d_faces)
            uvs.extend(d_uvs)
        else:
            plan, missed = _split_plan(d_faces, entries)
            unplaced += missed
            placed = []
            loop_at = 0
            for number, loop in enumerate(d_faces):
                size = len(loop)
                corners = []
                for corner, item in enumerate(loop):
                    corners.append(mapping[item])
                    uvs.append(d_uvs[loop_at + corner])
                    found = plan.get((number, corner))
                    if found is not None:
                        short, node, from_loop, to_loop, share = found
                        start, end = d_uvs[from_loop], d_uvs[to_loop]
                        corners.append(index[short][node])
                        uvs.append((start[0] + (end[0] - start[0]) * share, start[1] + (end[1] - start[1]) * share))
                        placed.append((loop_at + corner + 1, from_loop, to_loop, share))
                faces.append(tuple(corners))
                loop_at += size
            placed_by_domain.append(tuple(placed))
        face_domain.extend([patch_id] * len(d_faces))
        face_owner.extend(d_owner)
        seam_pairs.update(tuple(sorted((mapping[a], mapping[b]))) for a, b in d_seams)
        for _long, anchor, vertex, short, node, _share in entries or ():
            whole = tuple(sorted((mapping[anchor], mapping[vertex])))
            if whole in seam_pairs:
                seam_pairs.discard(whole)
                seam_pairs.add(tuple(sorted((mapping[anchor], index[short][node]))))
                seam_pairs.add(tuple(sorted((index[short][node], mapping[vertex]))))
    return faces, uvs, face_domain, face_owner, seam_pairs, tuple(placed_by_domain), unplaced


__all__ = ("glue_domains",)
