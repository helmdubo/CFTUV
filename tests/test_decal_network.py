from __future__ import annotations

from math import cos, sin

from mathutils import Vector

from cftuv.decal_network import (
    _lift_position,
    _merge_junction_continuations,
    _branch_from_run,
    build_seam_network_faces,
)
from cftuv.decals import _OrientedCornerRun
from cftuv.model import DecalSettings


Z_UP = (0.0, 0.0, 1.0)


def _make_run(vert_indices, points, normal_a=Z_UP, normal_b=Z_UP, edge_start=0):
    segment_count = len(points) - 1
    return _OrientedCornerRun(
        vert_indices=list(vert_indices),
        points=[Vector(point) for point in points],
        segment_normals_a=[Vector(normal_a) for _ in range(segment_count)],
        segment_normals_b=[Vector(normal_b) for _ in range(segment_count)],
        segment_convexities=[0.0] * segment_count,
        segment_edge_indices=list(range(edge_start, edge_start + segment_count)),
    )


def _positions(faces):
    found = set()
    for face in faces:
        for position in face.positions:
            found.add(
                (round(position.x, 4), round(position.y, 4), round(position.z, 4))
            )
    return found


def _has_position(faces, expected, tolerance=1e-3):
    expected = Vector(expected)
    for face in faces:
        for position in face.positions:
            if (position - expected).length <= tolerance:
                return True
    return False


def _uv_at_position(faces, expected, tolerance=1e-3):
    expected = Vector(expected)
    found = []
    for face in faces:
        for position, u_frac, v_length in zip(
            face.positions, face.u_fracs, face.v_lengths
        ):
            if (position - expected).length <= tolerance:
                found.append((u_frac, v_length))
    return found


class TestContinuationMerge:
    def test_collinear_pair_merges_through_junction(self):
        runs = [
            _make_run([10, 0], [(-1, 0, 0), (0, 0, 0)]),
            _make_run([0, 11], [(0, 0, 0), (1, 0, 0)], edge_start=5),
            _make_run([0, 12], [(0, 0, 0), (0, -1, 0)], edge_start=9),
        ]
        branches = _merge_junction_continuations(
            [_branch_from_run(run) for run in runs]
        )
        assert len(branches) == 2
        merged = max(branches, key=lambda branch: len(branch.points))
        assert merged.vert_indices in ([10, 0, 11], [11, 0, 10])

    def test_wide_y_junction_keeps_branches_independent(self):
        # 120° между outgoing тангенсами: dot = -0.5 выше порога слияния.
        runs = [
            _make_run([0, 1], [(0, 0, 0), (1, 0, 0)]),
            _make_run(
                [0, 2],
                [(0, 0, 0), (cos(2.0944), sin(2.0944), 0.0)],
                edge_start=5,
            ),
            _make_run(
                [0, 3],
                [(0, 0, 0), (cos(-2.0944), sin(-2.0944), 0.0)],
                edge_start=9,
            ),
        ]
        branches = _merge_junction_continuations(
            [_branch_from_run(run) for run in runs]
        )
        assert len(branches) == 3


class TestLiftPosition:
    def test_single_plane_offsets_along_normal(self):
        lifted = _lift_position(
            Vector((1.0, 2.0, 0.0)), [Vector((0.0, 0.0, 1.0))], 0.05
        )
        assert (lifted - Vector((1.0, 2.0, 0.05))).length < 1e-9

    def test_two_planes_meet_on_offset_intersection(self):
        lifted = _lift_position(
            Vector((0.0, 0.0, 0.0)),
            [Vector((0.0, 0.0, 1.0)), Vector((0.0, 1.0, 0.0))],
            0.02,
        )
        assert abs(lifted.z - 0.02) < 1e-9
        assert abs(lifted.y - 0.02) < 1e-9

    def test_three_planes_share_junction_core(self):
        lifted = _lift_position(
            Vector((0.0, 0.0, 0.0)),
            [
                Vector((0.0, 0.0, 1.0)),
                Vector((0.0, 1.0, 0.0)),
                Vector((1.0, 0.0, 0.0)),
            ],
            0.02,
        )
        assert (lifted - Vector((0.02, 0.02, 0.02))).length < 1e-7


class TestStraightSeam:
    def test_two_half_bands_share_spine_stations(self):
        run = _make_run([0, 1, 2], [(0, 0, 0), (1, 0, 0), (2, 0, 0)])
        faces = build_seam_network_faces([run], offset=0.0, width=0.2)
        assert len(faces) == 4
        spine_keys = {
            key
            for face in faces
            for key in face.vert_keys
            if key[0] == "sv"
        }
        assert spine_keys == {("sv", 0), ("sv", 1), ("sv", 2)}
        # Обе стороны используют одни и те же spine-ключи.
        for station_key in spine_keys:
            users = sum(
                1 for face in faces if station_key in face.vert_keys
            )
            assert users >= 2

    def test_uv_spans_full_rect_and_arc_length(self):
        run = _make_run([0, 1], [(0, 0, 0), (3, 0, 0)])
        faces = build_seam_network_faces([run], offset=0.0, width=0.2)
        u_values = sorted(
            {round(u, 3) for face in faces for u in face.u_fracs}
        )
        assert u_values == [-1.0, 0.0, 1.0]
        v_values = {round(v, 3) for face in faces for v in face.v_lengths}
        assert v_values == {0.0, 3.0}


class TestPlanarTJunction:
    def _faces(self):
        runs = [
            _make_run([10, 0], [(-1, 0, 0), (0, 0, 0)]),
            _make_run([0, 11], [(0, 0, 0), (1, 0, 0)], edge_start=5),
            _make_run([0, 12], [(0, 0, 0), (0, -1, 0)], edge_start=9),
        ]
        return build_seam_network_faces(runs, offset=0.0, width=0.2)

    def test_bisector_dividers_replace_transverse_endpoint(self):
        faces = self._faces()
        assert len(faces) == 6
        # Усреднённые divider-концы между stem и обеими половинами bar.
        assert _has_position(faces, (0.1, -0.1, 0.0))
        assert _has_position(faces, (-0.1, -0.1, 0.0))
        # Поперечного endpoint-ребра stem (вершина на spine ниже узла с
        # парой боковых на той же высоте) больше не существует: stem-faces
        # доходят до самого узла.
        node = Vector((0.0, 0.0, 0.0))
        for face in faces:
            xs = {round(position.x, 4) for position in face.positions}
            if xs and max(abs(x) for x in xs) <= 0.1001 and any(
                position.y < -0.5 for position in face.positions
            ):
                assert any(
                    (position - node).length < 1e-6
                    for position in face.positions
                )

    def test_bar_uv_continuous_through_junction(self):
        faces = self._faces()
        # V на узле со стороны bar равен arc от начала merged ветви (1.0)
        # в обеих прилегающих faces: непрерывный UV сквозь junction.
        node_uvs = _uv_at_position(faces, (0.0, 0.0, 0.0))
        bar_vs = sorted({round(v, 3) for _u, v in node_uvs})
        assert 1.0 in bar_vs

    def test_stem_cells_clip_at_bisector(self):
        faces = self._faces()
        uvs = _uv_at_position(faces, (0.1, -0.1, 0.0))
        assert uvs, "divider vertex must carry UV"
        for u_frac, _v in uvs:
            assert abs(abs(u_frac) - 1.0) < 1e-3


class TestXJunction:
    def test_two_through_sites_partition_by_diagonals(self):
        runs = [
            _make_run([10, 0], [(-1, 0, 0), (0, 0, 0)]),
            _make_run([0, 11], [(0, 0, 0), (1, 0, 0)], edge_start=5),
            _make_run([12, 0], [(0, -1, 0), (0, 0, 0)], edge_start=9),
            _make_run([0, 13], [(0, 0, 0), (0, 1, 0)], edge_start=13),
        ]
        faces = build_seam_network_faces(runs, offset=0.0, width=0.2)
        assert len(faces) == 8
        for corner in (
            (0.1, 0.1, 0.0),
            (-0.1, 0.1, 0.0),
            (0.1, -0.1, 0.0),
            (-0.1, -0.1, 0.0),
        ):
            assert _has_position(faces, corner)


class TestTrihedralJunction:
    def _faces(self):
        runs = [
            _make_run(
                [0, 1], [(0, 0, 0), (1, 0, 0)], (0, 0, 1), (0, 1, 0)
            ),
            _make_run(
                [0, 2],
                [(0, 0, 0), (0, 1, 0)],
                (0, 0, 1),
                (1, 0, 0),
                edge_start=5,
            ),
            _make_run(
                [0, 3],
                [(0, 0, 0), (0, 0, 1)],
                (0, 1, 0),
                (1, 0, 0),
                edge_start=9,
            ),
        ]
        return build_seam_network_faces(runs, offset=0.02, width=0.15)

    def test_all_surfaces_share_one_offset_core(self):
        faces = self._faces()
        core = Vector((0.02, 0.02, 0.02))
        touching_normals = set()
        for face in faces:
            if ("sv", 0) not in face.vert_keys:
                continue
            index = face.vert_keys.index(("sv", 0))
            assert (face.positions[index] - core).length < 1e-6
            touching_normals.add(
                tuple(round(component, 3) for component in face.surface_normal)
            )
        assert len(touching_normals) == 3

    def test_fold_wings_share_spine_verts_across_surfaces(self):
        faces = self._faces()
        # Станция ('sv', 1) ветви X принадлежит крыльям на двух поверхностях.
        normals = set()
        for face in faces:
            if ("sv", 1) in face.vert_keys:
                normals.add(
                    tuple(
                        round(component, 3)
                        for component in face.surface_normal
                    )
                )
        assert normals == {(0.0, 0.0, 1.0), (0.0, 1.0, 0.0)}

    def test_round_caps_stay_alpha_from_core_and_split_at_bisector(self):
        faces = self._faces()
        alpha = 0.075
        cap_radii = []
        for face in faces:
            if len(face.positions) != 3:
                continue
            for position, u_frac in zip(face.positions, face.u_fracs):
                if abs(abs(u_frac)) < 1e-6:
                    continue
                radius = (position - Vector((0.02, 0.02, 0.02))).length
                cap_radii.append(radius)
        assert cap_radii
        for radius in cap_radii:
            assert abs(radius - alpha) < 5e-3


class TestCloseParallelSeams:
    def test_facing_sides_clip_at_mid_line(self):
        runs = [
            _make_run([0, 1], [(0, 0, 0), (1, 0, 0)]),
            _make_run(
                [2, 3], [(0, 0.06, 0), (1, 0.06, 0)], edge_start=10
            ),
        ]
        faces = build_seam_network_faces(runs, offset=0.0, width=0.2)
        assert len(faces) == 4
        y_values = {
            round(position.y, 4)
            for face in faces
            for position in face.positions
        }
        # Смежные стороны обрезаны на средней линии, внешние — полные α.
        assert y_values == {-0.1, 0.0, 0.03, 0.06, 0.16}
        mid_uvs = _uv_at_position(faces, (0.0, 0.03, 0.0))
        assert mid_uvs
        for u_frac, _v in mid_uvs:
            assert abs(abs(u_frac) - 0.3) < 1e-3


class TestClosedRing:
    def test_square_ring_keeps_constant_width_miters(self):
        run = _make_run(
            [0, 1, 2, 3, 0],
            [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 0)],
        )
        faces = build_seam_network_faces([run], offset=0.0, width=0.2)
        assert len(faces) == 8
        positions = _positions(faces)
        for outer in ((-0.1, -0.1, 0.0), (1.1, -0.1, 0.0), (1.1, 1.1, 0.0), (-0.1, 1.1, 0.0)):
            assert outer in positions
        for inner in ((0.1, 0.1, 0.0), (0.9, 0.1, 0.0), (0.9, 0.9, 0.0), (0.1, 0.9, 0.0)):
            assert inner in positions


class TestSurfaceChangeSplit:
    def test_side_split_adds_connector_and_keeps_other_side_continuous(self):
        run = _OrientedCornerRun(
            vert_indices=[0, 1, 2],
            points=[
                Vector((0.0, 0.4, 0.0)),
                Vector((0.0, 0.0, 0.0)),
                Vector((0.8, 0.0, 0.0)),
            ],
            segment_normals_a=[Vector((0, 0, 1)), Vector((0, 0, 1))],
            segment_normals_b=[Vector((0, 0, 1)), Vector((0, 1, 0))],
            segment_convexities=[0.0, 0.0],
            segment_edge_indices=[0, 1],
        )
        faces = build_seam_network_faces([run], offset=0.0, width=0.2)
        # 2 quads стороны A (непрерывный поворот), 1 quad пол-B, 1 quad
        # стена-B, 1 connector треугольник на станции смены поверхности.
        assert len(faces) == 5
        assert _has_position(faces, (0.1, 0.1, 0.0))  # miter стороны A
        wall_faces = [
            face
            for face in faces
            if abs(face.surface_normal.y - 1.0) < 1e-6
        ]
        assert len(wall_faces) == 1
        assert _has_position(wall_faces, (0.8, 0.0, -0.1))
        connectors = [face for face in faces if len(face.positions) == 3]
        assert len(connectors) == 1
        # Connector сшивает station spine и оба flat-cap внешних конца.
        assert ("sv", 1) in connectors[0].vert_keys


class TestOrientationInvariance:
    def test_reversed_and_swapped_runs_build_identical_geometry(self):
        # Convexity — геометрический инвариант: разворот и swap сторон run
        # (как делает stitching) не должны выворачивать крылья.
        def fold(reverse=False, swap=False):
            run = _make_run(
                [0, 1, 2],
                [(0, 0, 0), (1, 0, 0), (2, 0, 0)],
                (0, 0, 1),
                (0, 1, 0),
            )
            run.segment_convexities = [1.0, 1.0]
            if reverse:
                run = run.reversed_copy()
            if swap:
                run = run.swapped_copy()
            return run

        reference = _positions(
            build_seam_network_faces([fold()], offset=0.02, width=0.15)
        )
        assert reference
        for reverse, swap in ((True, False), (False, True), (True, True)):
            variant = _positions(
                build_seam_network_faces(
                    [fold(reverse, swap)], offset=0.02, width=0.15
                )
            )
            assert variant == reference


class TestArcWallCorner:
    def _faces(self):
        from math import pi as _pi

        # Дуговая стена с толщиной: cap-кольцо, торец, вертикальное ребро.
        r_out, r_in, height, segs = 1.0, 0.9, 2.0, 14
        arc = _pi * 0.8

        def arc_point(radius, t, z):
            a = arc * t
            return Vector((radius * cos(a), radius * sin(a), z))

        def seg_mid_radial(i, sign_value=1.0):
            a = arc * (i + 0.5) / segs
            return Vector((sign_value * cos(a), sign_value * sin(a), 0.0))

        def fold_run(vids, pts, normals_a, normals_b, edge_start):
            count = len(pts) - 1
            return _OrientedCornerRun(
                vert_indices=vids,
                points=pts,
                segment_normals_a=normals_a,
                segment_normals_b=normals_b,
                segment_convexities=[1.0] * count,  # все фолды выпуклые
                segment_edge_indices=list(
                    range(edge_start, edge_start + count)
                ),
            )

        outer = fold_run(
            [1] + list(range(101, 101 + segs)),
            [arc_point(r_out, i / segs, height) for i in range(segs + 1)],
            [Vector((0, 0, 1))] * segs,
            [seg_mid_radial(i, +1.0) for i in range(segs)],
            0,
        )
        inner = fold_run(
            [2] + list(range(201, 201 + segs)),
            [arc_point(r_in, i / segs, height) for i in range(segs + 1)],
            [Vector((0, 0, 1))] * segs,
            [seg_mid_radial(i, -1.0) for i in range(segs)],
            50,
        )
        top_end = fold_run(
            [1, 2],
            [Vector((r_out, 0, height)), Vector((r_in, 0, height))],
            [Vector((0, 0, 1))],
            [Vector((0, -1, 0))],
            90,
        )
        vertical = fold_run(
            [1, 300, 301],
            [
                Vector((r_out, 0, height)),
                Vector((r_out, 0, height / 2)),
                Vector((r_out, 0, 0)),
            ],
            [Vector((0, -1, 0))] * 2,
            [seg_mid_radial(0, +1.0)] * 2,
            95,
        )
        from cftuv.decals import _stitch_corner_runs

        runs = _stitch_corner_runs([outer, inner, top_end, vertical])
        return build_seam_network_faces(runs, offset=0.02, width=0.15)

    def test_no_geometry_escapes_the_mesh(self):
        faces = self._faces()
        assert faces
        for face in faces:
            for position in face.positions:
                radius_xy = (position.x ** 2 + position.y ** 2) ** 0.5
                assert position.y > -0.021, position
                assert radius_xy < 1.1, position
                assert position.z < 2.021, position

    def test_corner_junction_has_no_unbounded_cap_fans(self):
        faces = self._faces()
        node = Vector((1.0, 0.0, 2.0))
        fan_triangles = [
            face
            for face in faces
            if len(face.positions) == 3
            and min((p - node).length for p in face.positions) < 0.12
        ]
        # Только ограниченные биссектрисами сектора, не полные 90° веера.
        assert len(fan_triangles) <= 6


class TestSingleSiteJunctionCap:
    def test_isolated_surface_end_stays_flat(self):
        # Узел валентности 3, но на торцевой поверхности только один сайт:
        # round cap висел бы за границей меша — конец должен быть плоским.
        stem = _make_run(
            [0, 10], [(0, 0, 0), (0, -1, 0)], (0, 0, 1), (0, 0, 1)
        )
        bar_left = _make_run(
            [11, 0], [(-1, 0, 0), (0, 0, 0)], (0, 0, 1), (0, 0, 1), edge_start=5
        )
        lone_wing = _make_run(
            [0, 12],
            [(0, 0, 0), (0, 0, 1)],
            (0, -1, 0),
            (0, -1, 0),
            edge_start=9,
        )
        faces = build_seam_network_faces(
            [stem, bar_left, lone_wing], offset=0.0, width=0.2
        )
        for face in faces:
            if abs(face.surface_normal.y + 1.0) > 1e-6:
                continue
            for position in face.positions:
                # Плоский конец: никакая вершина одиночного сайта не
                # заходит за узел (z < 0) дугой vertex region.
                assert position.z >= -1e-6, position


class TestSurfaceSpans:
    def _arc_branch(self, step_degrees, segments):
        from math import radians

        radius = 1.0
        step = radians(step_degrees)
        points = [
            Vector((radius * cos(i * step), radius * sin(i * step), 0.0))
            for i in range(segments + 1)
        ]
        run = _OrientedCornerRun(
            vert_indices=list(range(segments + 1)),
            points=points,
            segment_normals_a=[Vector((0, 0, 1))] * segments,
            segment_normals_b=[
                Vector((cos((i + 0.5) * step), sin((i + 0.5) * step), 0.0))
                for i in range(segments)
            ],
            segment_convexities=[1.0] * segments,
            segment_edge_indices=list(range(segments)),
        )
        return _branch_from_run(run)

    def test_borderline_tessellation_groups_spans(self):
        # ~7° между соседними нормалями — выше DECAL_COPLANAR_DOT: сегменты
        # группируются по бегущему среднему, а не рубятся на каждой станции
        # (per-station flat caps + connectors выглядели зубцами на дуге).
        from cftuv.decal_network import _side_surface_spans

        branch = self._arc_branch(7.0, 8)
        spans = _side_surface_spans(branch, branch.normals_b)
        assert 1 < len(spans) < 8
        # Бегущее среднее не даёт схлопнуть пологую дугу в одну хорду.
        assert max(end - start for start, end, _n, _r in spans) <= 4

    def test_coarse_tessellation_splits_per_face(self):
        from cftuv.decal_network import _side_surface_spans

        branch = self._arc_branch(12.0, 6)
        spans = _side_surface_spans(branch, branch.normals_b)
        assert len(spans) == 6


class TestSettings:
    def test_network_backend_enabled_by_default(self):
        assert DecalSettings().seam_network is True
