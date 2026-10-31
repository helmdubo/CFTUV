"""Склейка видов доменов в массивы меша декали (MESH_ASSEMBLY_BUFFERS_V1, срез B4a): эталон, с которым побитово сверен нативный `cftuv_native.mesh_assemble`.

По номерам сварки (`mesh_weld.weld_core`: у каждого домена номер вершины меша для каждой локальной вершины) грани домена переписываются в номера меша, UV и
владельцы граней идут подряд в порядке доменов, у каждой грани — номер патча её домена, швы домена — пары `(меньшая, большая)` вершин меша. Ни одно число
здесь не считается: только перекладка. Порядок — порядок доменов и порядок граней внутри домена; множество швов упорядочит вызывающий вместе со складками.
"""

from __future__ import annotations


def glue_domains(patch_ids, domains, index) -> tuple:
    """`(грани, uv, домен граней, владельцы граней, пары швов)` по доменам `(грани, uv, владельцы, швы)`, их патчам и номерам сварки.

    Грань — кортеж номеров вершин меша; пары швов — множество кортежей `(меньшая, большая)`. Номер вне вершин домена — `IndexError`, как у кортежа.
    """

    faces, uvs, face_domain, face_owner = [], [], [], []
    seam_pairs: set = set()
    for patch_id, (d_faces, d_uvs, d_owner, d_seams), mapping in zip(patch_ids, domains, index):
        faces.extend(tuple(mapping[item] for item in loop) for loop in d_faces)
        uvs.extend(d_uvs)
        face_domain.extend([patch_id] * len(d_faces))
        face_owner.extend(d_owner)
        seam_pairs.update(tuple(sorted((mapping[a], mapping[b]))) for a, b in d_seams)
    return faces, uvs, face_domain, face_owner, seam_pairs


__all__ = ("glue_domains",)
