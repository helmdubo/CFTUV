from mathutils import Vector

from cftuv.decal_surface_domain import (
    OwnerDomainEdge,
    OwnerDomainFace,
    OwnerFaceDomain,
    _unfold_domain,
    build_overlap_gated_seam_faces,
)
from cftuv.decals import _OrientedCornerRun


def _run(verts, points, normal, edge_index):
    return _OrientedCornerRun(
        vert_indices=list(verts),
        points=[Vector(point) for point in points],
        segment_normals_a=[Vector(normal)],
        segment_normals_b=[Vector(normal)],
        segment_convexities=[0.0],
        segment_edge_indices=[edge_index],
    )


def _floor_wall_domains():
    points = {
        0: Vector((0.0, -1.0, 0.0)),
        1: Vector((0.0, 1.0, 0.0)),
        2: Vector((0.2, -1.0, 0.0)),
        3: Vector((0.2, 1.0, 0.0)),
        4: Vector((0.0, -1.0, 0.2)),
        5: Vector((0.0, 1.0, 0.2)),
        6: Vector((1.0, -1.0, 0.0)),
        7: Vector((1.0, 1.0, 0.0)),
        8: Vector((0.0, -1.0, 1.0)),
        9: Vector((0.0, 1.0, 1.0)),
    }

    def face(index, verts, normal):
        return OwnerDomainFace(
            face_index=index,
            vert_indices=tuple(verts),
            points=tuple(points[vert].copy() for vert in verts),
            normal=Vector(normal),
        )

    central = OwnerFaceDomain(
        domain_id=0,
        patch_id=0,
        faces=(
            face(0, (0, 2, 3, 1), (0, 0, 1)),
            face(1, (0, 1, 5, 4), (1, 0, 0)),
        ),
        source_edges=(
            OwnerDomainEdge(100, 2, 3, 0),
            OwnerDomainEdge(101, 4, 5, 1),
        ),
    )
    floor_outer = OwnerFaceDomain(
        domain_id=1,
        patch_id=1,
        faces=(face(2, (2, 6, 7, 3), (0, 0, 1)),),
        source_edges=(OwnerDomainEdge(100, 2, 3, 2),),
    )
    wall_outer = OwnerFaceDomain(
        domain_id=2,
        patch_id=2,
        faces=(face(3, (4, 5, 9, 8), (1, 0, 0)),),
        source_edges=(OwnerDomainEdge(101, 4, 5, 3),),
    )
    runs = [
        _run((2, 3), (points[2], points[3]), (0, 0, 1), 100),
        _run((4, 5), (points[4], points[5]), (1, 0, 0), 101),
    ]
    return runs, [central, floor_outer, wall_outer]


def _signature(faces):
    return [
        (
            face.surface_id,
            tuple(face.vert_keys),
            tuple(tuple(round(value, 6) for value in point) for point in face.positions),
        )
        for face in faces
    ]


class TestOverlapGatedIntrinsicDomain:
    def test_clean_narrow_operation_returns_legacy_bit_for_bit(self):
        runs, domains = _floor_wall_domains()
        result = build_overlap_gated_seam_faces(runs, domains, 0.0, 0.18)
        assert not result.intrinsic_used
        assert not result.baseline_overlaps

        from cftuv.decal_network import build_seam_network_faces

        legacy = build_seam_network_faces(runs, 0.0, 0.18)
        assert _signature(result.faces) == _signature(legacy)

    def test_wide_crossing_activates_intrinsic_and_clears_oracle(self):
        runs, domains = _floor_wall_domains()
        result = build_overlap_gated_seam_faces(runs, domains, 0.0, 0.6)
        assert result.baseline_overlaps
        assert result.intrinsic_used
        assert not result.remaining_overlaps
        assert result.faces
        assert {face.surface_id for face in result.faces} == {0, 1, 2, 3}

    def test_width_event_changes_backend_and_topology(self):
        runs, domains = _floor_wall_domains()
        narrow = build_overlap_gated_seam_faces(runs, domains, 0.0, 0.18)
        wide = build_overlap_gated_seam_faces(runs, domains, 0.0, 0.6)
        assert not narrow.intrinsic_used
        assert wide.intrinsic_used
        assert _signature(narrow.faces) != _signature(wide.faces)


class TestIntrinsicChartCuts:
    def test_closed_non_developable_cycle_records_cut_instead_of_failing(self):
        bottom = (
            Vector((-1, -1, 0)),
            Vector((1, -1, 0)),
            Vector((1, 1, 0)),
            Vector((-1, 1, 0)),
        )
        top = tuple(point + Vector((0, 0, 1)) for point in bottom)
        points = bottom + top
        faces = []
        normals = (
            Vector((0, -1, 0)),
            Vector((1, 0, 0)),
            Vector((0, 1, 0)),
            Vector((-1, 0, 0)),
        )
        for index in range(4):
            following = (index + 1) % 4
            verts = (index, following, following + 4, index + 4)
            faces.append(
                OwnerDomainFace(
                    face_index=index,
                    vert_indices=verts,
                    points=tuple(points[vert].copy() for vert in verts),
                    normal=normals[index],
                )
            )
        domain = OwnerFaceDomain(5, 5, tuple(faces), ())
        unfolded = _unfold_domain(domain, 1e-6)
        assert unfolded is not None
        face_uv, _faces_by_edge, cuts = unfolded
        assert len(face_uv) == 4
        assert cuts
