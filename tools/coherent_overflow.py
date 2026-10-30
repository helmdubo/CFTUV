"""coherent_overflow: how far a shown width-preview frame spills past where the exact result of the same width ends (diagnostic, numpy only, never writes anything).

The metric of LIVE_DRAG_COHERENT_VERTEX_CLAMPED. A patch domain holds the strips of several fronts, and the fronts meet INSIDE the domain, so a strip that runs past the meeting
line lies on the surface of the opposing strip of the same patch: distance to the exact surface does not see it. Two measures do, without matching vertices between runs (the
layouts differ past an event):

1. AREA EXCESS per patch: the area of the shown faces minus the area of the exact faces at the same width. The exact faces of a patch do not overlap, so a strip running past the
   meeting line (two strips over one place) or past the patch boundary adds area the exact result does not have; a strip held back (the plain model frame) gives a deficit, not an
   excess. Reported: the excess summed over patches, as a share of the exact area, and the patches whose excess is above `share` of their exact area.
2. FLIPPED faces: a face of the shown frame whose normal turned against the same face at the base width (a fan collapsing past its skeleton node turns over). Needs no exact run, so it
   is per frame; its vertices are "vertices past their limit" in the strict sense.

`overflow(shown_positions, base_sample, exact_sample, share)` -> dict; `flipped(shown_positions, base_sample)` -> (faces, vertices).
"""

from __future__ import annotations

import numpy as np


def _fans(sample):
    """`(triangles (T, 3) vertex numbers, patch of each triangle, face of each triangle)` of the sample's mesh image, faces fanned from their first corner."""

    image = sample.image
    starts = np.asarray(image.face_starts, dtype=np.int64)
    flat = np.asarray(image.face_vertices, dtype=np.int64)
    lengths = np.diff(np.append(starts, flat.shape[0]))
    face = np.repeat(np.arange(starts.shape[0]), lengths - 2)
    offset = np.arange(face.shape[0]) - np.repeat(np.cumsum(np.append(0, lengths[:-1] - 2)), lengths - 2)
    first = flat[starts[face]]
    second = flat[starts[face] + 1 + offset]
    third = flat[starts[face] + 2 + offset]
    return np.stack([first, second, third], axis=1), np.asarray(image.face_domain, dtype=np.int64)[face], face


def _normals(positions, triangles):
    p = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    return np.cross(p[triangles[:, 1]] - p[triangles[:, 0]], p[triangles[:, 2]] - p[triangles[:, 0]])


def _areas_by_patch(positions, triangles, patches) -> dict:
    area = 0.5 * np.linalg.norm(_normals(positions, triangles), axis=1)
    keys, inverse = np.unique(patches, return_inverse=True)
    return dict(zip(keys.tolist(), np.bincount(inverse, weights=area).tolist()))


def flipped(shown_positions, base_sample) -> tuple:
    """`(faces, vertices)` of the shown frame turned over against the base width (each face once)."""

    triangles, _patches, face = _fans(base_sample)
    before = _normals(base_sample.positions, triangles)
    after = _normals(shown_positions, triangles)
    turned = np.einsum("ij,ij->i", before, after) < 0.0
    faces = np.unique(face[turned])
    return int(faces.shape[0]), int(np.unique(triangles[turned]).shape[0])


def overflow(shown_positions, base_sample, exact_sample, share: float = 0.01) -> dict:
    base_triangles, base_patches, _face = _fans(base_sample)
    exact_triangles, exact_patches, _face = _fans(exact_sample)
    shown = _areas_by_patch(shown_positions, base_triangles, base_patches)
    exact = _areas_by_patch(exact_sample.positions, exact_triangles, exact_patches)
    excess, deficit, spilling = 0.0, 0.0, []
    for patch, area in exact.items():
        delta = shown.get(patch, 0.0) - area
        if delta > 0.0:
            excess += delta
            if delta > share * area:
                spilling.append((patch, round(delta / area, 4)))
        else:
            deficit -= delta
    total = sum(exact.values())
    faces, vertices = flipped(shown_positions, base_sample)
    spilling.sort(key=lambda item: -item[1])
    return {"excess_share": excess / total if total else 0.0, "deficit_share": deficit / total if total else 0.0, "patches_spilling": len(spilling), "worst_patches": spilling[:8],
            "patches": len(exact), "flipped_faces": faces, "flipped_vertices": vertices}
