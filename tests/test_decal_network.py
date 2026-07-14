from __future__ import annotations

from collections import Counter
from math import cos, pi, sin
from random import Random
from types import SimpleNamespace

from mathutils import Vector

from cftuv.constants import DECAL_WELD_DISTANCE
from cftuv.decal_network import (
    _Site,
    _candidate_competitors,
    _lift_position,
    _merge_junction_continuations,
    _normalize_surface_arrangement,
    _segment_intersection2,
    _branch_from_run,
    build_seam_network_faces,
    compile_seam_network_plan,
    evaluate_seam_network_plan,
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

    def test_hard_miter_cap_is_single_diagonal_vertex(self):
        # MITER-политика по умолчанию: reflex-промежуток на кубическом
        # углу закрыт одной жёсткой вершиной на биссектрисе, а не веером.
        # Два перпендикулярных крыла грани дают gap 270°, sweep 45°, miter
        # на радиусе α/cos(45°) = α·√2 — диагональный внешний угол куба.
        from math import sqrt

        faces = self._faces()
        alpha = 0.075
        core = Vector((0.02, 0.02, 0.02))
        cap_tris = [face for face in faces if len(face.positions) == 3]
        assert cap_tris, "junction cap must exist"
        radii = sorted(
            {
                round((position - core).length, 4)
                for face in cap_tris
                for position, u_frac in zip(face.positions, face.u_fracs)
                if abs(u_frac) > 1e-6
            }
        )
        # Ровно два радиуса: перпендикулярный рельс на α и жёсткая
        # miter-вершина на α·√2. Никаких промежуточных веерных радиусов.
        assert len(radii) == 2
        assert abs(radii[0] - alpha) < 5e-3
        assert abs(radii[1] - alpha * sqrt(2.0)) < 5e-3

    def test_fan_triangle_count_matches_hard_corner(self):
        # Веер из множества треугольников (ROUND) заменён на единичные
        # joint-треугольники (MITER): на каждую занятую сторону каждой
        # поверхности — ровно один cap-треугольник, а не дуга.
        faces = self._faces()
        cap_tris = [face for face in faces if len(face.positions) == 3]
        # Три поверхности × две станции ветвей у core, но каждая станция
        # даёт по одному hard cap; фан из >6 треугольников исключён.
        assert len(cap_tris) <= 6


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
    def test_side_split_shares_rail_vertex_across_fold(self):
        # Convex step: пол (+Z) переходит в стену (n = -Y, материал вниз).
        run = _OrientedCornerRun(
            vert_indices=[0, 1, 2],
            points=[
                Vector((0.0, 0.4, 0.0)),
                Vector((0.0, 0.0, 0.0)),
                Vector((0.8, 0.0, 0.0)),
            ],
            segment_normals_a=[Vector((0, 0, 1)), Vector((0, 0, 1))],
            segment_normals_b=[Vector((0, 0, 1)), Vector((0, -1, 0))],
            segment_convexities=[0.0, 1.0],
            segment_edge_indices=[0, 1],
        )
        faces = build_seam_network_faces([run], offset=0.0, width=0.2)
        # 2 quads стороны A (непрерывный поворот), quad пол-B и quad
        # стена-B, сшитые общей rail-вершиной развёрнутого miter:
        # connector-треугольник не нужен.
        assert len(faces) == 4
        assert not [face for face in faces if len(face.positions) == 3]
        assert _has_position(faces, (0.1, 0.1, 0.0))  # miter стороны A
        wall_faces = [
            face
            for face in faces
            if abs(face.surface_normal.y + 1.0) < 1e-6
        ]
        assert len(wall_faces) == 1
        assert _has_position(wall_faces, (0.8, 0.0, -0.1))
        # Rail-вершина сложена обратно на стену и разделяется обеими
        # сторонами разреза одним vertex key.
        rail_keys = {
            key
            for face in faces
            for key in face.vert_keys
            if key[0] == "rail"
        }
        assert len(rail_keys) == 1
        rail_users = [
            face for face in faces if rail_keys & set(face.vert_keys)
        ]
        assert len(rail_users) == 2
        assert _has_position(rail_users, (-0.1, 0.0, -0.1))
        for face in rail_users:
            index = face.vert_keys.index(next(iter(rail_keys)))
            assert (
                face.positions[index] - Vector((-0.1, 0.0, -0.1))
            ).length < 1e-6


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


def _topology_signature(faces):
    """Комбинаторная сигнатура: (#faces, размеры полигонов, #вершин)."""

    return (
        len(faces),
        sorted(len(face.positions) for face in faces),
        len({key for face in faces for key in face.vert_keys}),
    )


def _polyline_distance_3d(points, q):
    best = None
    for seg_a, seg_b in zip(points, points[1:]):
        delta = seg_b - seg_a
        denom = delta.dot(delta)
        t = 0.0 if denom < 1e-24 else max(
            0.0, min(1.0, (q - seg_a).dot(delta) / denom)
        )
        dist = (q - (seg_a + delta * t)).length
        if best is None or dist < best:
            best = dist
    return best


class TestDynamicTopology:
    """Oracle-сценарии: ширина α меняет комбинаторику crop-сетки.

    Ячейка ветви = VoronoiCell ∩ Offset(branch, α): при пересечении α
    критических расстояний появляются новые intersection-вершины, рёбра
    схлопываются, faces исчезают. Kernel пересобирает клиппинг при каждом
    вызове, поэтому события обязаны воспроизводиться. Если сигнатура
    топологии не меняется во всём диапазоне α — тест проваливается.
    """

    def test_staggered_parallel_seams_touch_event(self):
        def faces_at(width):
            runs = [
                _make_run([0, 1], [(0, 0, 0), (1, 0, 0)]),
                _make_run(
                    [2, 3], [(0.3, 0.2, 0), (1.3, 0.2, 0)], edge_start=10
                ),
            ]
            return build_seam_network_faces(runs, offset=0.0, width=width)

        narrow = faces_at(0.1)  # α=0.05: полосы не касаются
        wide = faces_at(0.3)  # α=0.15 > d/2=0.1: клип по бисектору
        assert _topology_signature(narrow) != _topology_signature(wide)
        narrow_ys = {
            round(p.y, 3) for f in narrow for p in f.positions
        }
        assert 0.1 not in narrow_ys
        assert any(
            abs(p.y - 0.1) < 1e-3 for f in wide for p in f.positions
        )

    def test_point_segment_boundary_is_curved_and_equidistant(self):
        # Endpoint ветви B лежит против интерьера ветви A: граница ячеек
        # состоит из прямого segment-segment участка (x > 0.5) и параболы
        # point-vs-segment (x < 0.5), уходящей к краю полосы.
        spine_a = [Vector((0, 0, 0)), Vector((1, 0, 0))]
        spine_b = [Vector((0.5, -0.15, 0)), Vector((1.5, -0.15, 0))]
        runs = [
            _make_run([0, 1], [tuple(p) for p in spine_a]),
            _make_run(
                [2, 3], [tuple(p) for p in spine_b], edge_start=10
            ),
        ]
        faces = build_seam_network_faces(runs, offset=0.0, width=0.4)
        cut_verts = []
        for face in faces:
            for position in face.positions:
                dist_a = _polyline_distance_3d(spine_a, position)
                dist_b = _polyline_distance_3d(spine_b, position)
                if abs(dist_a - dist_b) < 1e-3 and dist_a > 0.02:
                    cut_verts.append(position)
        # Кривая граница (point vs segment): минимум три неколлинеарные
        # equidistant-вершины, т.е. параболический участок отслежен.
        assert len(cut_verts) >= 3
        collinear = True
        base = cut_verts[0]
        direction = None
        for vert in cut_verts[1:]:
            delta = vert - base
            if delta.length < 1e-6:
                continue
            if direction is None:
                direction = delta.normalized()
                continue
            if direction.cross(delta.normalized()).length > 1e-3:
                collinear = False
                break
        assert not collinear

    def test_acute_v_round_join_is_bounded_and_low_res(self):
        # Острый V (поворот 160°): miter превышает лимит 4α, станция
        # получает round join. По умолчанию LOW-профиль ограничивает его
        # бюджетом вершин (ширино-независимо), а не разворачивает в веер.
        # Инвариант: ячейка ограничена (нет длинного шипа) и число вершин
        # round-join мало и стабильно по ширине.
        from math import radians

        angle = radians(20.0)

        def faces_at(width):
            v_run = _make_run(
                [1, 0, 2],
                [
                    (1, 0, 0),
                    (0, 0, 0),
                    (cos(angle), sin(angle), 0.0),
                ],
            )
            return build_seam_network_faces([v_run], offset=0.0, width=width)

        narrow = faces_at(0.04)
        wide = faces_at(0.4)
        spine = [
            Vector((1, 0, 0)),
            Vector((0, 0, 0)),
            Vector((cos(angle), sin(angle), 0.0)),
        ]
        for faces, width in ((narrow, 0.04), (wide, 0.4)):
            alpha = width / 2.0
            limit = alpha * 4.0 + 1e-6
            for face in faces:
                for position in face.positions:
                    assert _polyline_distance_3d(spine, position) <= limit
        # Число вершин round-join одинаково при малой и большой ширине —
        # дискретизация больше не разрастается с α (веер убран).
        assert sum(len(f.positions) for f in narrow) == sum(
            len(f.positions) for f in wide
        )
        # И это именно немного вершин, а не десятки (LOW бюджет ≤ 4/сегмент).
        assert max(len(f.positions) for f in wide) <= 8

    def test_short_t_branch_cell_shrinks_past_critical_width(self):
        def faces_at(width):
            runs = [
                _make_run([10, 0], [(-1, 0, 0), (0, 0, 0)]),
                _make_run([0, 11], [(0, 0, 0), (1, 0, 0)], edge_start=5),
                _make_run(
                    [0, 12], [(0, 0, 0), (0, -0.12, 0)], edge_start=9
                ),
            ]
            return build_seam_network_faces(runs, offset=0.0, width=width)

        narrow = faces_at(0.1)  # α=0.05 < длины stem
        wide = faces_at(0.4)  # α=0.2 > длины stem: cap клипится бисекторами
        assert _topology_signature(narrow) != _topology_signature(wide)
        stem_wide = [
            face
            for face in wide
            if any(p.y < -0.01 for p in face.positions)
            and all(abs(p.x) < 0.5 for p in face.positions)
        ]
        assert stem_wide
        max_x = max(
            abs(p.x) for face in stem_wide for p in face.positions
        )
        # Ячейка stem ограничена бисекторами |x| <= |y| <= 0.12, а не α=0.2.
        assert max_x < 0.13

    def test_boundary_wing_divider_without_shared_vertex(self):
        def faces_at(width):
            boundary = _OrientedCornerRun(
                vert_indices=[0, 1],
                points=[Vector((0, 0, 0)), Vector((1, 0, 0))],
                segment_normals_a=[Vector((0, 0, 0))],
                segment_normals_b=[Vector((0, 0, 1))],
                segment_convexities=[0.0],
                segment_edge_indices=[0],
            )
            seam = _make_run(
                [2, 3], [(0, 0.16, 0), (1, 0.16, 0)], edge_start=10
            )
            return build_seam_network_faces(
                [boundary, seam], offset=0.0, width=width
            )

        narrow = faces_at(0.1)
        wide = faces_at(0.3)
        assert _topology_signature(narrow) != _topology_signature(wide)
        # Крыло boundary направлено внутрь patch (n x t = +Y) и обрезано
        # общим divider на y=0.08 без общей source-вершины с seam.
        assert any(
            abs(p.y - 0.08) < 1e-3 for f in wide for p in f.positions
        )
        boundary_narrow = [
            f
            for f in narrow
            if all(-0.001 <= p.y <= 0.051 for p in f.positions)
        ]
        assert boundary_narrow  # одностороннее крыло 0..α без клипа

    def test_fold_neighbor_event_matches_planar_and_lift_is_watertight(self):
        # Floor-шов в d=0.1 от выбранного fold: то же событие, что у двух
        # параллельных планарных швов (бисектор на d/2), а fold-крыло на
        # стене продолжает жить, разделяя spine-вершины через поверхность.
        def faces_at(width):
            fold = _make_run(
                [0, 1],
                [(0, 0, 0), (1, 0, 0)],
                normal_a=(0, 0, 1),
                normal_b=(0, -1, 0),
            )
            fold.segment_convexities = [1.0]
            seam = _make_run(
                [2, 3], [(0, 0.1, 0), (1, 0.1, 0)], edge_start=10
            )
            return build_seam_network_faces([fold, seam], offset=0.0, width=width)

        narrow = faces_at(0.08)  # α=0.04 < d/2
        wide = faces_at(0.3)  # α=0.15 > d/2
        assert _topology_signature(narrow) != _topology_signature(wide)
        assert any(
            abs(p.y - 0.05) < 1e-3 and abs(p.z) < 1e-6
            for f in wide
            for p in f.positions
        )
        wall_faces = [
            f
            for f in wide
            if abs(f.surface_normal.y + 1.0) < 1e-6
        ]
        assert wall_faces
        # Крыло на стене сохраняет полную ширину α (событие floor-стороны
        # его не трогает) и разделяет spine-ключи с floor-крылом.
        assert any(
            abs(p.z + 0.15) < 1e-3 for f in wall_faces for p in f.positions
        )
        wall_keys = {
            key
            for f in wall_faces
            for key in f.vert_keys
            if key[0] == "sv"
        }
        floor_keys = {
            key
            for f in wide
            if abs(f.surface_normal.z - 1.0) < 1e-6
            for key in f.vert_keys
            if key[0] == "sv"
        }
        assert wall_keys & floor_keys == {("sv", 0), ("sv", 1)}


class TestConformalLift:
    """Production repro (cftuv_seam_dump.json): широкая лента на гранёной дуге.

    Ширина 0.7427 при offset 0.001 и тесселяции ~5°/грань: chart span'а —
    усреднённая плоскость, и без конформного лифта углы flat caps выпирали
    на ~0.015 из стены («зубцы» connector-треугольников на внешнем радиусе).
    """

    # (edge, v0, co0, v1, co1, n_a, c_a, n_b, c_b) — дамп production-сцены.
    _DUMP_EDGES = [
        (1, 3, (1.0, 1.0, 1.0), 2, (1.0, 1.0, -1.0),
         (1.0, -0.0, 0.0), (1.0, 0.878988, 0.0), (0.0, 1.0, 0.0), (0.888237, 1.0, 0.0)),
        (2, 24, (0.612561, 0.992422, 1.0), 22, (0.776474, 1.0, 1.0),
         (0.0, -0.0, 1.0), (0.709419, 0.913861, 1.0), (-0.046184, 0.998933, 0.0), (0.694517, 0.996211, 0.0)),
        (4, 5, (0.776474, 1.0, -1.0), 21, (0.612561, 0.992422, -1.0),
         (-0.0, 0.0, -1.0), (0.709419, 0.913861, -1.0), (-0.046184, 0.998933, 0.0), (0.694517, 0.996211, 0.0)),
        (6, 25, (0.450047, 0.969752, 1.0), 24, (0.612561, 0.992422, 1.0),
         (0.0, -0.0, 1.0), (0.51425, 0.89346, 1.0), (-0.138156, 0.99041, 0.0), (0.531304, 0.981087, 0.0)),
        (7, 21, (0.612561, 0.992422, -1.0), 20, (0.450047, 0.969752, -1.0),
         (-0.0, 0.0, -1.0), (0.51425, 0.89346, -1.0), (-0.138156, 0.99041, 0.0), (0.531304, 0.981087, 0.0)),
        (9, 26, (0.290318, 0.932184, 1.0), 25, (0.450047, 0.969752, 1.0),
         (0.0, -0.0, 1.0), (0.406836, 0.873381, 1.0), (-0.22895, 0.973438, 0.0), (0.370183, 0.950968, 0.0)),
        (10, 20, (0.450047, 0.969752, -1.0), 19, (0.290318, 0.932184, -1.0),
         (-0.0, 0.0, -1.0), (0.406836, 0.873381, -1.0), (-0.22895, 0.973438, 0.0), (0.370183, 0.950968, 0.0)),
        (55, 5, (0.776474, 1.0, -1.0), 2, (1.0, 1.0, -1.0),
         (0.0, 0.0, -1.0), (0.925491, 0.919326, -1.0), (0.0, 1.0, 0.0), (0.888237, 1.0, 0.0)),
        (57, 22, (0.776474, 1.0, 1.0), 3, (1.0, 1.0, 1.0),
         (-0.0, 0.0, 1.0), (0.925491, 0.919326, 1.0), (0.0, 1.0, 0.0), (0.888237, 1.0, 0.0)),
    ]

    def _runs(self):
        from cftuv.decals import _stitch_corner_runs

        runs = []
        for (
            edge_index,
            index_0,
            co_0,
            index_1,
            co_1,
            normal_a,
            center_a,
            normal_b,
            _center_b,
        ) in self._DUMP_EDGES:
            p0, p1 = Vector(co_0), Vector(co_1)
            n_a = Vector(normal_a).normalized()
            n_b = Vector(normal_b).normalized()
            mid = (p0 + p1) * 0.5
            tangent = (p1 - p0).normalized()
            if n_a.cross(tangent).dot(Vector(center_a) - mid) < 0.0:
                index_0, index_1 = index_1, index_0
                p0, p1 = p1, p0
                tangent = tangent * -1.0
            value = tangent.cross(n_a).normalized().dot(n_b)
            convexity = 0.0 if abs(value) < 0.01 else max(-1.0, min(1.0, value))
            runs.append(
                _OrientedCornerRun(
                    vert_indices=[index_0, index_1],
                    points=[p0, p1],
                    segment_normals_a=[n_a],
                    segment_normals_b=[n_b],
                    segment_convexities=[convexity],
                    segment_edge_indices=[edge_index],
                )
            )
        return _stitch_corner_runs(runs)

    def _faces(self, width=0.7427, offset=0.001):
        return build_seam_network_faces(self._runs(), offset, width)

    def test_wide_band_conforms_to_faceted_wall(self):
        faces = self._faces()
        assert faces
        wall_planes = [
            (Vector(entry[7]).normalized(), Vector(entry[8]))
            for entry in self._DUMP_EDGES
        ] + [(Vector(self._DUMP_EDGES[0][5]), Vector(self._DUMP_EDGES[0][6]))]
        for face in faces:
            if abs(face.surface_normal.z) > 0.5:
                continue  # cap-стороны проверяются ниже
            for position in face.positions:
                signed = min(
                    abs(normal.dot(position - center) - 0.001)
                    for normal, center in wall_planes
                )
                # Каждая вершина wall-ленты лежит на offset-плоскости
                # какой-то реальной грани (до фикса: выпирала на ~0.015).
                assert signed < 2e-3, (tuple(position), signed)

    def test_cap_sides_stay_at_offset_height(self):
        faces = self._faces()
        for face in faces:
            if abs(face.surface_normal.z) < 0.5:
                continue
            for position in face.positions:
                assert abs(abs(position.z) - 1.001) < 2e-3


class TestJunctionCapStyle:
    def _trihedral_faces(self):
        runs = [
            _make_run(
                [0, 1], [(0, 0, 0), (1, 0, 0)], (0, 0, 1), (0, 1, 0)
            ),
            _make_run(
                [0, 2], [(0, 0, 0), (0, 1, 0)], (0, 0, 1), (1, 0, 0),
                edge_start=5,
            ),
            _make_run(
                [0, 3], [(0, 0, 0), (0, 0, 1)], (0, 1, 0), (1, 0, 0),
                edge_start=9,
            ),
        ]
        return build_seam_network_faces(runs, offset=0.02, width=0.15)

    def test_round_smooth_branch_is_preserved_and_produces_fan(self):
        # ROUND cap style + SMOOTH профиль кривой = точный веер (обе ветки
        # законсервированы). Под дефолтным LOW дуга бюджетируется — веера нет.
        import cftuv.decal_network as network

        cap = network.DECAL_NETWORK_JUNCTION_CAP_STYLE
        curve = network.DECAL_NETWORK_CLIP_CURVE_STYLE
        try:
            network.DECAL_NETWORK_JUNCTION_CAP_STYLE = "ROUND"
            network.DECAL_NETWORK_CLIP_CURVE_STYLE = "SMOOTH"
            round_faces = self._trihedral_faces()
        finally:
            network.DECAL_NETWORK_JUNCTION_CAP_STYLE = cap
            network.DECAL_NETWORK_CLIP_CURVE_STYLE = curve
        miter_faces = self._trihedral_faces()
        round_tris = sum(1 for f in round_faces if len(f.positions) == 3)
        miter_tris = sum(1 for f in miter_faces if len(f.positions) == 3)
        # ROUND+SMOOTH полилинизирует дугу в веер: строго больше
        # треугольников, чем у жёсткого дефолтного MITER на том же угле.
        assert round_tris > miter_tris
        # Веерные вершины лежат на радиусе α от core (округлость).
        core = Vector((0.02, 0.02, 0.02))
        alpha = 0.075
        arc_radii = [
            (position - core).length
            for face in round_faces
            if len(face.positions) == 3
            for position, u_frac in zip(face.positions, face.u_fracs)
            if abs(u_frac) > 1e-6
        ]
        assert arc_radii
        assert all(abs(radius - alpha) < 5e-3 for radius in arc_radii)

    def test_round_low_profile_budgets_the_fan(self):
        # ROUND cap style под дефолтным LOW: дуга есть, но бюджетирована —
        # заметно меньше треугольников, чем ROUND+SMOOTH.
        import cftuv.decal_network as network

        cap = network.DECAL_NETWORK_JUNCTION_CAP_STYLE
        curve = network.DECAL_NETWORK_CLIP_CURVE_STYLE
        try:
            network.DECAL_NETWORK_JUNCTION_CAP_STYLE = "ROUND"
            network.DECAL_NETWORK_CLIP_CURVE_STYLE = "SMOOTH"
            smooth = self._trihedral_faces()
            network.DECAL_NETWORK_CLIP_CURVE_STYLE = "LOW"
            low = self._trihedral_faces()
        finally:
            network.DECAL_NETWORK_JUNCTION_CAP_STYLE = cap
            network.DECAL_NETWORK_CLIP_CURVE_STYLE = curve
        smooth_tris = sum(1 for f in smooth if len(f.positions) == 3)
        low_tris = sum(1 for f in low if len(f.positions) == 3)
        assert low_tris < smooth_tris


class TestCandidateBroadphase:
    @staticmethod
    def _site(site_id, branch_id, surface_id, bbox, station=0):
        site = _Site(
            site_id=site_id,
            branch_id=branch_id,
            side="A",
            surface_id=surface_id,
            start_station=station,
            end_station=station + 1,
        )
        site.bbox = bbox
        return site

    @staticmethod
    def _full_scan(sites, reach):
        from cftuv.decal_network import _bbox_distance, _sites_compete

        return {
            site.site_id: [
                other.site_id
                for other in sites
                if _sites_compete(site, other)
                and _bbox_distance(site.bbox, other.bbox) <= reach
            ]
            for site in sites
        }

    @staticmethod
    def _signature(candidates):
        return {
            site_id: [site.site_id for site in competitors]
            for site_id, competitors in candidates.items()
        }

    def test_randomized_sweep_matches_full_scan(self):
        random = Random(73451)
        sites = []
        for site_id in range(180):
            x = random.uniform(-40.0, 40.0)
            y = random.uniform(-20.0, 20.0)
            width = random.uniform(0.0, 2.0)
            height = random.uniform(0.0, 2.0)
            sites.append(
                self._site(
                    site_id,
                    branch_id=random.randrange(45),
                    surface_id=random.randrange(4),
                    bbox=(x, y, x + width, y + height),
                    station=random.randrange(4),
                )
            )

        for reach in (0.0, 0.1, 0.5, 2.0, 8.0):
            expected = self._full_scan(sites, reach)
            actual = self._signature(_candidate_competitors(sites, reach))
            assert actual == expected

    def test_input_order_does_not_change_competitor_order(self):
        sites = [
            self._site(index, index, 0, (index * 0.4, 0.0, index * 0.4 + 0.2, 0.2))
            for index in range(12)
        ]

        forward = self._signature(_candidate_competitors(sites, 0.7))
        backward = self._signature(
            _candidate_competitors(list(reversed(sites)), 0.7)
        )

        assert forward == backward
        assert all(values == sorted(values) for values in forward.values())

    def test_sparse_network_avoids_quadratic_bbox_checks(self, monkeypatch):
        import cftuv.decal_network as network

        sites = [
            self._site(index, index, 0, (index * 10.0, 0.0, index * 10.0 + 1.0, 1.0))
            for index in range(1200)
        ]
        original = network._bbox_distance
        calls = [0]

        def counted(first, second):
            calls[0] += 1
            return original(first, second)

        monkeypatch.setattr(network, "_bbox_distance", counted)
        candidates = _candidate_competitors(sites, reach=0.5)

        assert all(not competitors for competitors in candidates.values())
        assert calls[0] == 0


class TestPerformancePreview:
    def _arc_comb(self):
        from math import cos, sin

        runs = []
        edge = 0
        segs = 24
        arc = [
            (cos(i * 0.06) * 3.0, sin(i * 0.06) * 3.0, 0.0)
            for i in range(segs + 1)
        ]
        runs.append(_make_run(list(range(segs + 1)), arc, edge_start=edge))
        edge += segs
        vid = 1000
        for s in range(8):
            base = 2 * s + 1
            stem = [
                (
                    cos(base * 0.06) * (3.0 - 0.25 * k),
                    sin(base * 0.06) * (3.0 - 0.25 * k),
                    0.0,
                )
                for k in range(4)
            ]
            runs.append(
                _make_run(
                    [base] + [vid + s * 10 + k for k in range(3)],
                    stem,
                    edge_start=edge,
                )
            )
            edge += 3
        return runs

    @staticmethod
    def _face_signature(faces):
        # Полигоны сравниваем как множество с округлением ниже weld,
        # инвариантно к порядку faces и к циклическому сдвигу вершин.
        signature = []
        for face in faces:
            verts = tuple(
                (round(p.x, 6), round(p.y, 6), round(p.z, 6))
                for p in face.positions
            )
            signature.append(
                (
                    tuple(round(component, 4) for component in face.surface_normal),
                    tuple(sorted(verts)),
                    tuple(round(u, 5) for u in sorted(face.u_fracs)),
                )
            )
        return sorted(signature)

    @staticmethod
    def _exact_face_signature(faces):
        return [
            (
                face.surface_id,
                tuple(tuple(position) for position in face.positions),
                tuple(face.vert_keys),
                tuple(face.u_fracs),
                tuple(face.v_lengths),
            )
            for face in faces
        ]

    def test_bbox_gate_is_exact_vs_no_gate(self):
        # Настоящий differential: гейт ON должен давать ту же геометрию,
        # что и полный клиппинг без гейта — на многоветвевой дуге и на
        # production-дампе, для набора ширин (α меняет gate_margin).
        dump_runs = TestConformalLift()._runs()
        for runs, offset in ((self._arc_comb(), 0.02), (dump_runs, 0.001)):
            for width in (0.15, 0.3, 0.7):
                gated = build_seam_network_faces(
                    runs, offset=offset, width=width, _gate=True
                )
                ungated = build_seam_network_faces(
                    runs, offset=offset, width=width, _gate=False
                )
                assert self._face_signature(gated) == self._face_signature(
                    ungated
                ), (width, offset)

    def test_candidate_broadphase_is_exact_vs_full_scan(self):
        # Broadphase меняет только поиск пар, но не порядок half-plane clips
        # и не geometry output. Проверяем preview и финальный arrangement.
        dump_runs = TestConformalLift()._runs()
        for runs, offset in ((self._arc_comb(), 0.02), (dump_runs, 0.001)):
            for width in (0.15, 0.3, 0.7):
                for preview in (True, False):
                    swept = build_seam_network_faces(
                        runs,
                        offset=offset,
                        width=width,
                        preview=preview,
                        _broadphase=True,
                    )
                    scanned = build_seam_network_faces(
                        runs,
                        offset=offset,
                        width=width,
                        preview=preview,
                        _broadphase=False,
                    )
                    assert self._exact_face_signature(
                        swept
                    ) == self._exact_face_signature(scanned), (
                        width,
                        offset,
                        preview,
                    )

    def test_compiled_plan_is_exact_and_frame_isolation(self):
        # Compile/evaluate обязан быть побитово эквивалентен one-shot API.
        # Повтор A после другой ширины B ловит утечки shared rail/retraction.
        runs = self._arc_comb()
        plan = compile_seam_network_plan(runs, offset=0.02)

        first_a = evaluate_seam_network_plan(
            plan, width=0.3, preview=False
        )
        middle_b = evaluate_seam_network_plan(
            plan, width=0.7, preview=True
        )
        second_a = evaluate_seam_network_plan(
            plan, width=0.3, preview=False
        )
        one_shot_a = build_seam_network_faces(
            runs, offset=0.02, width=0.3, preview=False
        )
        one_shot_b = build_seam_network_faces(
            runs, offset=0.02, width=0.7, preview=True
        )

        assert self._exact_face_signature(
            first_a
        ) == self._exact_face_signature(one_shot_a)
        assert self._exact_face_signature(
            middle_b
        ) == self._exact_face_signature(one_shot_b)
        assert self._exact_face_signature(
            second_a
        ) == self._exact_face_signature(first_a)

    def test_distance_accelerators_are_exact(self):
        # Оптимизации меняют только поиск того же minimum/того же
        # implicit value. Reference отключает их независимо от broadphase.
        runs = self._arc_comb()
        plan = compile_seam_network_plan(runs, offset=0.02)
        for width in (0.3, 0.7):
            for preview in (True, False):
                reference = evaluate_seam_network_plan(
                    plan,
                    width,
                    preview=preview,
                    _distance_index=False,
                    _pair_cache=False,
                    _site_distance_cache=False,
                )
                accelerated = evaluate_seam_network_plan(
                    plan,
                    width,
                    preview=preview,
                    _distance_index=True,
                    _pair_cache=False,
                )
                assert self._exact_face_signature(
                    accelerated
                ) == self._exact_face_signature(reference)

        # Интеграционный случай, где BVH реально включён в network clip:
        # длинная дуга конкурирует с ответвлением на общей owner surface.
        segment_count = 48
        arc = [
            (
                cos(index * 0.04) * 3.0,
                sin(index * 0.04) * 3.0,
                0.0,
            )
            for index in range(segment_count + 1)
        ]
        station = segment_count // 2
        angle = station * 0.04
        stem = [
            (
                cos(angle) * (3.0 - step * 0.25),
                sin(angle) * (3.0 - step * 0.25),
                0.0,
            )
            for step in range(5)
        ]
        long_runs = [
            _make_run(range(segment_count + 1), arc),
            _make_run(
                [station, 1001, 1002, 1003, 1004],
                stem,
                edge_start=segment_count,
            ),
        ]
        long_plan = compile_seam_network_plan(long_runs, offset=0.02)
        assert any(
            site._distance_root >= 0 for site in long_plan.site_templates
        )
        reference = evaluate_seam_network_plan(
            long_plan,
            0.6,
            preview=True,
            _distance_index=False,
            _pair_cache=False,
            _site_distance_cache=False,
        )
        accelerated = evaluate_seam_network_plan(
            long_plan,
            0.6,
            preview=True,
        )
        assert self._exact_face_signature(
            accelerated
        ) == self._exact_face_signature(reference)

    def test_long_site_distance_index_matches_linear_queries(self):
        segment_count = 64
        points = [
            (cos(index * 0.04) * 3.0, sin(index * 0.04) * 3.0)
            for index in range(segment_count + 1)
        ]
        site = _Site(
            site_id=0,
            branch_id=0,
            side="A",
            surface_id=0,
            start_station=0,
            end_station=segment_count,
            pts2=points,
        )
        site.finalize(0.0)
        assert site._distance_root >= 0
        rng = Random(4117)
        indexed = [
            site.competition_distance_sq(
                (rng.uniform(-4.0, 4.0), rng.uniform(-4.0, 4.0))
            )
            for _ in range(300)
        ]
        nodes, root = site._distance_nodes, site._distance_root
        site._distance_nodes = ()
        site._distance_root = -1
        rng = Random(4117)
        linear = [
            site.competition_distance_sq(
                (rng.uniform(-4.0, 4.0), rng.uniform(-4.0, 4.0))
            )
            for _ in range(300)
        ]
        site._distance_nodes, site._distance_root = nodes, root
        assert indexed == linear

    def test_pair_value_cache_reduces_distance_queries(self, monkeypatch):
        plan = compile_seam_network_plan(self._arc_comb(), offset=0.02)
        original = _Site.competition_distance
        calls = [0]

        def counted(site, point):
            calls[0] += 1
            return original(site, point)

        monkeypatch.setattr(_Site, "competition_distance", counted)
        evaluate_seam_network_plan(
            plan,
            0.7,
            preview=True,
            _pair_cache=False,
            _site_distance_cache=False,
        )
        uncached_calls = calls[0]
        calls[0] = 0
        evaluate_seam_network_plan(
            plan,
            0.7,
            preview=True,
            _pair_cache=True,
            _site_distance_cache=False,
        )
        cached_calls = calls[0]

        assert cached_calls < uncached_calls

    def test_site_distance_cache_reuses_values_across_pairs(self, monkeypatch):
        plan = compile_seam_network_plan(self._arc_comb(), offset=0.02)
        original = _Site.competition_distance
        calls = [0]

        def counted(site, point):
            calls[0] += 1
            return original(site, point)

        monkeypatch.setattr(_Site, "competition_distance", counted)
        evaluate_seam_network_plan(
            plan,
            0.7,
            preview=True,
            _pair_cache=False,
            _site_distance_cache=False,
        )
        uncached_calls = calls[0]
        calls[0] = 0
        evaluate_seam_network_plan(
            plan,
            0.7,
            preview=True,
            _pair_cache=False,
            _site_distance_cache=True,
        )
        cached_calls = calls[0]

        assert cached_calls < uncached_calls

    def test_fast_preview_roots_stay_below_weld_error(self):
        plan = compile_seam_network_plan(self._arc_comb(), offset=0.02)
        exact_preview = evaluate_seam_network_plan(
            plan,
            0.7,
            preview=True,
            _fast_preview_roots=False,
        )
        fast_preview = evaluate_seam_network_plan(
            plan,
            0.7,
            preview=True,
            _fast_preview_roots=True,
        )

        assert len(fast_preview) == len(exact_preview)
        max_position_error = 0.0
        for exact_face, fast_face in zip(exact_preview, fast_preview):
            assert fast_face.surface_id == exact_face.surface_id
            assert len(fast_face.positions) == len(exact_face.positions)
            for exact_position, fast_position in zip(
                exact_face.positions, fast_face.positions
            ):
                max_position_error = max(
                    max_position_error,
                    (exact_position - fast_position).length,
                )
        assert max_position_error < 0.001

        # Confirm rebuild не использует preview tolerance вообще.
        exact_final = evaluate_seam_network_plan(
            plan,
            0.7,
            preview=False,
            _fast_preview_roots=False,
        )
        production_final = evaluate_seam_network_plan(
            plan,
            0.7,
            preview=False,
            _fast_preview_roots=True,
        )
        assert self._exact_face_signature(
            production_final
        ) == self._exact_face_signature(exact_final)

    def test_adaptive_gate_reduces_clip_calls(self, monkeypatch):
        from cftuv import decal_network as network

        plan = compile_seam_network_plan(self._arc_comb(), offset=0.02)
        original = network._clip_polygon
        calls = [0]

        def counted(*args, **kwargs):
            calls[0] += 1
            return original(*args, **kwargs)

        monkeypatch.setattr(network, "_clip_polygon", counted)
        constant = evaluate_seam_network_plan(
            plan,
            0.7,
            preview=True,
            _adaptive_gate=False,
        )
        constant_calls = calls[0]
        calls[0] = 0
        adaptive = evaluate_seam_network_plan(
            plan,
            0.7,
            preview=True,
            _adaptive_gate=True,
        )
        adaptive_calls = calls[0]

        assert self._exact_face_signature(
            adaptive
        ) == self._exact_face_signature(constant)
        assert adaptive_calls < constant_calls

    def test_squared_cache_matches_naive_distance(self):
        # competition_distance (быстрый scalar-кэш) обязан совпадать с
        # наивным поэлементным расстоянием точка-полилиния до 1e-9.
        from cftuv.decal_network import _segment_point_distance2

        runs = self._arc_comb()
        faces = build_seam_network_faces(runs, offset=0.02, width=0.3)
        # Восстанавливаем внутренние сайты повторным разбором той же сети.
        # Проверяем инвариант кэша напрямую на сконструированном сайте.
        from cftuv.decal_network import _Site

        site = _Site(
            site_id=0, branch_id=0, side="A", surface_id=0,
            start_station=0, end_station=3,
        )
        site.pts2 = [(0.0, 0.0), (1.0, 0.2), (2.0, -0.1), (3.0, 0.3)]
        site.vert_indices = [0, 1, 2, 3]
        site.arcs = [0.0, 1.0, 2.0, 3.0]
        site.finalize(0.0)  # без retraction

        def naive_sq(q):
            best = None
            for a, b in zip(site.pts2, site.pts2[1:]):
                distance, _t = _segment_point_distance2(a, b, q)
                if best is None or distance < best:
                    best = distance
            return best * best

        for qx in (-0.5, 0.3, 1.4, 2.7, 3.5):
            for qy in (-0.4, 0.0, 0.25, 0.6):
                q = (qx, qy)
                assert abs(
                    site.competition_distance_sq(q) - naive_sq(q)
                ) < 1e-9

    def test_preview_is_coarser_or_equal_never_finer(self):
        # Корректный контракт: preview может иметь МЕНЬШЕ вершин (кривые
        # не уточняются), но не больше; финал полностью заменяет preview.
        runs = self._arc_comb()
        full = build_seam_network_faces(runs, offset=0.02, width=0.3)
        preview = build_seam_network_faces(
            runs, offset=0.02, width=0.3, preview=True
        )
        full_verts = sum(len(f.positions) for f in full)
        preview_verts = sum(len(f.positions) for f in preview)
        assert preview_verts <= full_verts
        assert all(len(f.positions) >= 3 for f in preview)

    def test_preview_drops_curved_boundary_vertices(self):
        # На point-vs-segment границе (парабола) финал вставляет уточняющие
        # вершины, а preview — нет: доказательство, что refine действительно
        # отключается, и что контракт «не одинаковы» реален.
        runs = [
            _make_run([0, 1], [(0, 0, 0), (1, 0, 0)]),
            _make_run([2, 3], [(0.5, -0.15, 0), (1.5, -0.15, 0)], edge_start=10),
        ]
        full = build_seam_network_faces(runs, offset=0.0, width=0.4)
        preview = build_seam_network_faces(
            runs, offset=0.0, width=0.4, preview=True
        )
        full_verts = sum(len(f.positions) for f in full)
        preview_verts = sum(len(f.positions) for f in preview)
        assert preview_verts < full_verts


class TestCurveProfile:
    """Профиль детализации round-join/кривых стыка (HARD/LOW/SMOOTH)."""

    def _sharp_corner(self):
        # Одиночный резкий convex-поворот: одна сторона банда даёт round
        # join за miter-лимитом — источник веера чёрных вершин на скрине.
        return _make_run(
            [0, 1, 2],
            [(1, 0, 0), (0, 0, 0), (cos(pi - 3.05), sin(pi - 3.05), 0.0)],
        )

    def _build(self, style, width=0.4):
        import cftuv.decal_network as network

        original = network.DECAL_NETWORK_CLIP_CURVE_STYLE
        try:
            network.DECAL_NETWORK_CLIP_CURVE_STYLE = style
            return build_seam_network_faces(
                [self._sharp_corner()], offset=0.0, width=width
            )
        finally:
            network.DECAL_NETWORK_CLIP_CURVE_STYLE = original

    def test_low_reduces_fan_vs_smooth(self):
        smooth = self._build("SMOOTH")
        low = self._build("LOW")
        hard = self._build("HARD")
        smooth_verts = sum(len(f.positions) for f in smooth)
        low_verts = sum(len(f.positions) for f in low)
        hard_verts = sum(len(f.positions) for f in hard)
        # Убывающая детализация round-join: SMOOTH-веер > LOW > HARD-bevel.
        assert smooth_verts > low_verts > hard_verts
        # LOW заметно (в разы) режет веер, а не на пару вершин.
        assert low_verts * 2 <= smooth_verts

    def test_low_fan_is_width_independent(self):
        # Дискретизация round-join не разрастается с шириной (бюджет вершин).
        narrow = self._build("LOW", width=0.2)
        wide = self._build("LOW", width=0.9)
        assert sum(len(f.positions) for f in narrow) == sum(
            len(f.positions) for f in wide
        )

    def test_default_curve_style_is_low(self):
        import cftuv.decal_network as network

        assert network.DECAL_NETWORK_CLIP_CURVE_STYLE == "LOW"

    def test_preview_forces_hard_curve(self):
        # preview всегда строит HARD-контуры (самое грубое) независимо от
        # дефолтного профиля — максимально дёшево во время drag.
        preview = build_seam_network_faces(
            [self._sharp_corner()], offset=0.0, width=0.4, preview=True
        )
        hard = self._build("HARD")
        assert sum(len(f.positions) for f in preview) == sum(
            len(f.positions) for f in hard
        )


# ============================================================
# Verification harness — свойства, которые пропускали прежние тесты и
# из-за чего регрессия (исчезновение faces) не ловилась: валидность loop,
# симуляция BMesh-материализации, T-стыки, площадь.
# ============================================================


def _face_has_repeated_key(face):
    return len(set(face.vert_keys)) != len(face.vert_keys)


def _materialization_drops(faces):
    """Сколько faces исчезло бы в `_materialize_network_faces`.

    Материализатор дедуплицирует вершины по ключу; если после этого в loop
    < 3 уникальных вершин — face не создаётся. Именно так молча пропадала
    геометрия у сломанного healer'а.
    """

    dropped = 0
    for face in faces:
        unique = []
        seen = set()
        for key in face.vert_keys:
            if key in seen:
                continue
            seen.add(key)
            unique.append(key)
        if len(unique) < 3:
            dropped += 1
    return dropped


def _segments_properly_cross(p1, p2, p3, p4):
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1, d2 = cross(p3, p4, p1), cross(p3, p4, p2)
    d3, d4 = cross(p1, p2, p3), cross(p1, p2, p4)
    return (
        (d1 > 1e-9) != (d2 > 1e-9)
        and (d3 > 1e-9) != (d4 > 1e-9)
        and min(abs(d1), abs(d2), abs(d3), abs(d4)) > 1e-9
    )


def _face_self_intersects(face):
    pts = [(p.x, p.y, p.z) for p in face.positions]
    n = len(pts)
    for i in range(n):
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            if _segments_properly_cross(
                pts[i], pts[(i + 1) % n], pts[j], pts[(j + 1) % n]
            ):
                return True
    return False


def _face_area(face):
    pts = face.positions
    if len(pts) < 3:
        return 0.0
    accum = Vector((0.0, 0.0, 0.0))
    for i in range(1, len(pts) - 1):
        accum = accum + (pts[i] - pts[0]).cross(pts[i + 1] - pts[0])
    return accum.length * 0.5


def _count_materialized_crossings(faces):
    """X-пересечения после реального shared-key materialization.

    `_NetworkFace.positions` хранит per-site lift и у двух faces может
    отличаться на numerical epsilon. BMesh использует первую позицию общего
    vertex key, поэтому oracle должен канонизировать координаты так же.
    """

    positions_by_key = {}
    polygons_by_surface = {}
    for face in faces:
        polygon = []
        for key, position in zip(face.vert_keys, face.positions):
            positions_by_key.setdefault(key, position)
            canonical = positions_by_key[key]
            polygon.append((canonical.x, canonical.y))
        xs = [point[0] for point in polygon]
        ys = [point[1] for point in polygon]
        polygons_by_surface.setdefault(face.surface_id, []).append(
            (polygon, (min(xs), min(ys), max(xs), max(ys)))
        )

    crossings = 0
    for polygons in polygons_by_surface.values():
        for index, (polygon_a, bbox_a) in enumerate(polygons):
            for polygon_b, bbox_b in polygons[index + 1:]:
                if (
                    bbox_a[2] < bbox_b[0] - 1e-6
                    or bbox_b[2] < bbox_a[0] - 1e-6
                    or bbox_a[3] < bbox_b[1] - 1e-6
                    or bbox_b[3] < bbox_a[1] - 1e-6
                ):
                    continue
                found = False
                for edge_a, point_a in enumerate(polygon_a):
                    point_b = polygon_a[(edge_a + 1) % len(polygon_a)]
                    for edge_b, point_c in enumerate(polygon_b):
                        point_d = polygon_b[(edge_b + 1) % len(polygon_b)]
                        for param_ab, param_cd, _point in _segment_intersection2(
                            point_a,
                            point_b,
                            point_c,
                            point_d,
                            1e-6,
                        ):
                            if (
                                1e-6 < param_ab < 1.0 - 1e-6
                                and 1e-6 < param_cd < 1.0 - 1e-6
                            ):
                                crossings += 1
                                found = True
                                break
                        if found:
                            break
                    if found:
                        break
    return crossings


def _count_t_junctions(faces, tolerance=1e-3):
    def key(position):
        return (round(position.x, 4), round(position.y, 4), round(position.z, 4))

    all_verts = {key(p) for face in faces for p in face.positions}
    count = 0
    for face in faces:
        verts = [key(p) for p in face.positions]
        for i in range(len(verts)):
            a = Vector(verts[i])
            b = Vector(verts[(i + 1) % len(verts)])
            edge = b - a
            length_sq = edge.dot(edge)
            if length_sq < 1e-12:
                continue
            for w in all_verts:
                if w in (verts[i], verts[(i + 1) % len(verts)]):
                    continue
                wv = Vector(w)
                t = (wv - a).dot(edge) / length_sq
                if 1e-3 < t < 1 - 1e-3 and (wv - (a + edge * t)).length < tolerance:
                    count += 1
    return count


def _assert_no_vanishing(faces):
    """Ни одна face не привела бы к тихому drop в BMesh-материализации.

    Проверяет ровно причины исчезновения: повтор вершины в loop (→
    `bmesh.faces.new` ValueError) и вырождение (< 3 уникальных / нулевая
    площадь). Самопересечение (bowtie) сюда НЕ входит: bmesh создаёт такую
    face без ошибки, она не исчезает — это отдельный вопрос качества.
    """

    for face in faces:
        assert not _face_has_repeated_key(face), face.vert_keys
        assert _face_area(face) > 1e-9
    assert _materialization_drops(faces) == 0


class TestArrangementValidity:
    """Инварианты materialization-safety на перекрывающихся сайтах.

    Ловят класс регрессий, который прошлые тесты пропускали: невалидные
    loop → `bmesh.faces.new` ValueError → молчаливое исчезновение faces.
    """

    def _perpendicular(self, width):
        straight = _make_run([0, 1], [(-1, 0, 0), (2, 0, 0)])
        turning = _make_run(
            [2, 3, 4],
            [(0.5, 1.2, 0), (0.5, 0.28, 0), (1.4, 0.28, 0)],
            edge_start=10,
        )
        return build_seam_network_faces(
            [straight, turning], offset=0.0, width=width
        )

    def _fold_corner(self, width):
        from cftuv.decals import _stitch_corner_runs

        vert = _make_run([0, 1], [(0, 0, 1), (0, 0, 0)], (0, -1, 0), (-1, 0, 0))
        ledge = _make_run(
            [0, 2], [(0, 0, 0), (1, 0, 0)], (0, 0, 1), (0, -1, 0), edge_start=10
        )
        floor = _make_run(
            [0, 3], [(0, 0, 0), (0, 1, 0)], (0, 0, 1), (-1, 0, 0), edge_start=20
        )
        return build_seam_network_faces(
            _stitch_corner_runs([vert, ledge, floor]), offset=0.0, width=width
        )

    def _dense_comb(self, width):
        runs = []
        edge = 0
        segs = 24
        arc = [
            (cos(i * 0.06) * 3.0, sin(i * 0.06) * 3.0, 0.0)
            for i in range(segs + 1)
        ]
        runs.append(_make_run(list(range(segs + 1)), arc, edge_start=edge))
        edge += segs
        vid = 1000
        for s in range(8):
            base = 2 * s + 1
            stem = [
                (
                    cos(base * 0.06) * (3.0 - 0.25 * k),
                    sin(base * 0.06) * (3.0 - 0.25 * k),
                    0.0,
                )
                for k in range(4)
            ]
            runs.append(
                _make_run(
                    [base] + [vid + s * 10 + k for k in range(3)],
                    stem,
                    edge_start=edge,
                )
            )
            edge += 3
        return build_seam_network_faces(runs, offset=0.02, width=width)

    def test_no_faces_vanish_on_overlap(self):
        # Ни один из кейсов (в т.ч. плотный многосторонний) не производит
        # face, которая молча исчезнет при материализации. Это тот класс
        # регрессий, который дал «исчезающую геометрию у стыка».
        for width in (0.3, 0.5, 0.7, 0.9):
            _assert_no_vanishing(self._perpendicular(width))
        for width in (0.2, 0.4, 0.7):
            _assert_no_vanishing(self._fold_corner(width))
        for width in (0.3, 0.6):
            _assert_no_vanishing(self._dense_comb(width))

    def test_shared_boundary_watertight_on_screenshot_cases(self):
        # Перпендикулярное наложение и fold-угол (кейс со скрина): общая
        # граница без T-стыков и без bowtie — обе стороны получают
        # одинаковые вершины.
        for faces in (
            [self._perpendicular(w) for w in (0.3, 0.5, 0.7, 0.9)]
            + [self._fold_corner(w) for w in (0.2, 0.4, 0.7)]
        ):
            assert _count_t_junctions(faces) == 0
            assert not any(_face_self_intersects(face) for face in faces)

    def test_dense_wide_partition_has_one_face_per_cell(self):
        # Ширина намного больше шага comb заставляет несколько независимых
        # ribbons полностью поглотить соседей. Общая planar arrangement
        # обязана выдать одну face на atomic cell и после shared-key
        # materialization не оставить X-пересечений.
        faces = self._dense_comb(8.0)
        _assert_no_vanishing(faces)
        signatures = [frozenset(face.vert_keys) for face in faces]
        assert len(signatures) == len(set(signatures))
        assert _count_materialized_crossings(faces) == 0

    def test_overlap_area_has_no_double_coverage(self):
        # Обе стороны реально обрезаны: суммарная площадь меньше наивной
        # (band1 + band2), т.е. область наложения не покрыта дважды.
        faces = self._perpendicular(0.5)
        total = sum(_face_area(face) for face in faces)
        # Две полосы шириной 0.5: наивная сумма ~ 3*0.5 + ~1.4*0.5 ≈ 2.2.
        # Корректная partition убирает overlap → строго меньше.
        assert total < 2.2

    def test_extreme_width_collinear_sliver_is_removed(self):
        # Portal/U-case со скрина: width многократно больше проёма.
        # Последовательный clip оставлял на одном углу коллинеарную
        # collision-вершину в 0.009 от station, а на симметричном — уже
        # схлопнутую в пределах weld. Arrangement должен удалить такой
        # degree-2 sliver глобально из обеих соседних faces.
        runs = [
            _make_run([0, 1], [(0, 0, 0), (0, 2, 0)]),
            _make_run(
                [1, 2], [(0, 2, 0), (2, 2, 0)], edge_start=10
            ),
            _make_run(
                [2, 3], [(2, 2, 0), (2, 0, 0)], edge_start=20
            ),
        ]
        stations = [
            Vector((0, 0, 0)),
            Vector((0, 2, 0)),
            Vector((2, 2, 0)),
            Vector((2, 0, 0)),
        ]
        faces = build_seam_network_faces(
            runs, offset=0.0, width=20.0
        )

        nearby_collision_distances = []
        for face in faces:
            for key, position in zip(face.vert_keys, face.positions):
                if key[0] == "sv":
                    continue
                distance = min(
                    (position - station).length for station in stations
                )
                if distance < 0.05:
                    nearby_collision_distances.append(distance)
        assert all(
            distance < DECAL_WELD_DISTANCE
            for distance in nearby_collision_distances
        )
        position_counts = Counter(
            (round(position.x, 5), round(position.y, 5))
            for face in faces
            for position in face.positions
        )
        reflected_counts = Counter()
        for (x, y), count in position_counts.items():
            reflected_counts[(round(2.0 - x, 5), y)] += count
        assert position_counts == reflected_counts
        _assert_no_vanishing(faces)

    def test_arrangement_split_matches_six_digit_vertex_keys(self):
        # Реальный широкий portal-кейс даёт общие boundary samples с
        # расхождением до 8.9e-7. Это меньше одного шага принятой 6-digit
        # quantization, поэтому arrangement обязана разбить соседнее ребро
        # в этой точке, а не оставить T-junction между соседними cells.
        sample = (0.00000089, 0.5)
        polygons_by_site = {
            0: [[(-1.0, 0.0), sample, (-1.0, 1.0)]],
            1: [[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]],
        }
        sites = [
            SimpleNamespace(site_id=0, surface_id=0),
            SimpleNamespace(site_id=1, surface_id=0),
        ]

        _normalize_surface_arrangement(
            polygons_by_site,
            sites,
            1e-6,
        )

        assert sample in polygons_by_site[1][0]

    def test_site_order_invariant(self):
        straight = _make_run([0, 1], [(-1, 0, 0), (2, 0, 0)])
        turning = _make_run(
            [2, 3, 4],
            [(0.5, 1.2, 0), (0.5, 0.28, 0), (1.4, 0.28, 0)],
            edge_start=10,
        )

        def signature(faces):
            return sorted(
                tuple(
                    sorted(
                        (round(p.x, 4), round(p.y, 4), round(p.z, 4))
                        for p in face.positions
                    )
                )
                for face in faces
            )

        assert signature(
            build_seam_network_faces([straight, turning], 0.0, 0.5)
        ) == signature(
            build_seam_network_faces([turning, straight], 0.0, 0.5)
        )


class TestSettings:
    def test_network_backend_enabled_by_default(self):
        assert DecalSettings().seam_network is True

    def test_junction_cap_style_defaults_to_miter(self):
        import cftuv.decal_network as network

        assert network.DECAL_NETWORK_JUNCTION_CAP_STYLE == "MITER"
