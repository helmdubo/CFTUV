from mathutils import Vector

from cftuv.decal_network import _NetworkFace
from cftuv.decal_overlap import detect_network_overlaps


def _face(surface_id, points, keys=None, normal=(0.0, 0.0, 1.0)):
    if keys is None:
        keys = [("p", surface_id, index) for index in range(len(points))]
    return _NetworkFace(
        surface_id=surface_id,
        surface_normal=Vector(normal),
        vert_keys=list(keys),
        positions=[Vector(point) for point in points],
        u_fracs=[0.0] * len(points),
        v_lengths=[0.0] * len(points),
    )


class TestNetworkOverlapOracle:
    def test_reports_coplanar_double_coverage_inside_domain(self):
        first = _face(1, [(0, 0, 0), (2, 0, 0), (2, 1, 0), (0, 1, 0)])
        second = _face(2, [(1, 0, 0), (3, 0, 0), (3, 1, 0), (1, 1, 0)])
        overlaps = detect_network_overlaps([first, second], {0: 7, 1: 7})
        assert len(overlaps) == 1
        assert abs(overlaps[0].area - 1.0) < 1e-8

    def test_close_or_intersecting_disconnected_domains_do_not_compete(self):
        first = _face(1, [(0, 0, 0), (2, 0, 0), (2, 1, 0), (0, 1, 0)])
        second = _face(2, [(1, 0, 0), (3, 0, 0), (3, 1, 0), (1, 1, 0)])
        assert not detect_network_overlaps([first, second], {0: 7, 1: 8})

    def test_shared_fold_edge_is_not_an_overlap(self):
        keys_a = [("p", 1, 0), ("rail", 0), ("rail", 1), ("p", 1, 1)]
        keys_b = [("rail", 1), ("rail", 0), ("p", 2, 0), ("p", 2, 1)]
        floor = _face(
            1,
            [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)],
            keys=keys_a,
        )
        wall = _face(
            2,
            [(1, 1, 0), (1, 0, 0), (1, 0, 1), (1, 1, 1)],
            keys=keys_b,
            normal=(1.0, 0.0, 0.0),
        )
        assert not detect_network_overlaps([floor, wall], {0: 3, 1: 3})

    def test_reports_non_coplanar_crossing_inside_domain(self):
        floor = _face(1, [(-1, -1, 0), (1, -1, 0), (0, 1, 0)])
        wall = _face(
            2,
            [(0, -0.5, -1), (0, -0.5, 1), (0, 0.8, 0)],
            normal=(1.0, 0.0, 0.0),
        )
        overlaps = detect_network_overlaps([floor, wall], {0: 4, 1: 4})
        assert len(overlaps) == 1
        assert overlaps[0].crossing_length > 0.5


def test_baseline_adjacent_owner_faces_expose_cross_surface_intersection():
    from test_decal_network import _make_run
    from cftuv.decal_network import build_seam_network_faces

    floor = _make_run(
        [10, 11],
        [(0.2, -1.0, 0.0), (0.2, 1.0, 0.0)],
        normal_a=(0.0, 0.0, 1.0),
        normal_b=(0.0, 0.0, 1.0),
    )
    wall = _make_run(
        [20, 21],
        [(0.0, -1.0, 0.2), (0.0, 1.0, 0.2)],
        normal_a=(1.0, 0.0, 0.0),
        normal_b=(1.0, 0.0, 0.0),
        edge_start=10,
    )
    faces = build_seam_network_faces([floor, wall], offset=0.0, width=0.6)
    domains = {
        index: 0
        for index, face in enumerate(faces)
        if face.surface_id >= 0
    }
    overlaps = detect_network_overlaps(faces, domains)
    assert overlaps
    assert max(hit.crossing_length for hit in overlaps) > 0.5


def test_shared_rail_production_dump_has_no_cross_surface_intersection():
    from test_decal_network import TestConformalLift

    faces = TestConformalLift()._faces()
    domains = {
        index: 0
        for index, face in enumerate(faces)
        if face.surface_id >= 0
    }
    assert not detect_network_overlaps(faces, domains, tolerance=2e-5)
