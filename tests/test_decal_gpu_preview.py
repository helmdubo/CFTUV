"""F3 GPU preview: batch-логика и lifecycle без реального gpu.

Контракты: workplan «F3. GPU preview overlay» + «Параллельный GPU-трек»
(headless-тесты через injected adapter), A6-семантика PreviewStatus.
"""

import pytest

from mathutils import Vector

from cftuv.decal_geometry import DecalGeometryFace
from cftuv.decal_gpu_preview import (
    GpuPreviewController,
    build_preview_buffers,
    create_controller,
    topology_signature,
    triangulate_face,
)


def _face(points2d, kind="SEGMENT", side="", z=0.0):
    positions = [Vector((x, y, z)) for x, y in points2d]
    return DecalGeometryFace(
        surface_id=0,
        surface_normal=Vector((0.0, 0.0, 1.0)),
        vert_keys=[("k", index) for index in range(len(positions))],
        positions=positions,
        u_fracs=[0.0] * len(positions),
        v_lengths=[float(index) for index in range(len(positions))],
        component_kind=kind,
        component_side=side,
    )


class FakeAdapter:
    def __init__(self, fail_upload=False):
        self.handler_count = 0
        self.uploads = []
        self.draws = 0
        self.redraws = 0
        self.fail_upload = fail_upload

    def add_handler(self, draw_fn):
        self.handler_count += 1
        self.draw_fn = draw_fn

    def remove_handler(self):
        self.handler_count -= 1

    def upload(self, buffers, rebuild=True):
        if self.fail_upload:
            raise RuntimeError("upload failed")
        self.uploads.append((rebuild, buffers["face_count"]))

    def draw(self):
        self.draws += 1

    def tag_redraw(self):
        self.redraws += 1


def _tri_area(a, b, c):
    return abs(
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    ) * 0.5


def _triangulation_area(points2d):
    face = _face(points2d)
    total = 0.0
    for i0, i1, i2 in triangulate_face(face.positions, face.surface_normal):
        total += _tri_area(points2d[i0], points2d[i1], points2d[i2])
    return total


def test_triangulate_quad_and_concave_preserve_area():
    assert len(triangulate_face(_face([(0, 0), (1, 0), (1, 1)]).positions,
                                Vector((0, 0, 1)))) == 1
    quad = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]
    assert _triangulation_area(quad) == pytest.approx(2.0)
    concave_l = [
        (0.0, 0.0), (2.0, 0.0), (2.0, 1.0),
        (1.0, 1.0), (1.0, 2.0), (0.0, 2.0),
    ]
    assert _triangulation_area(concave_l) == pytest.approx(3.0)


def test_triangulate_degenerate_returns_empty():
    face = _face([(0.0, 0.0), (1.0, 0.0)])
    assert triangulate_face(face.positions, face.surface_normal) == ()


def test_buffers_colors_by_kind_and_boundary_lines():
    faces = [
        _face([(0, 0), (1, 0), (1, 1), (0, 1)], kind="SEGMENT"),
        _face([(1, 0), (2, 0), (2, 1), (1, 1)], kind="KITE"),
    ]
    buffers = build_preview_buffers(faces)
    assert len(buffers["tri_positions"]) == 12  # 2 quad = 4 tris
    assert len(buffers["tri_colors"]) == 12
    assert len(buffers["tri_uvs"]) == 12
    segment_colors = set(buffers["tri_colors"][:6])
    kite_colors = set(buffers["tri_colors"][6:])
    assert segment_colors != kite_colors
    # Общее ребро x=1 внутреннее: контур = 6 из 8 рёбер, 12 точек.
    assert len(buffers["line_positions"]) == 12
    # UV: u_frac 0 -> 0.5 после ремапа [-1..1] -> [0..1].
    assert buffers["tri_uvs"][0][0] == pytest.approx(0.5)


def test_signature_stable_under_vertex_motion_only():
    base = [_face([(0, 0), (1, 0), (1, 1), (0, 1)], kind="SEGMENT")]
    moved = [_face([(0, 0), (1.5, 0), (1.5, 1.2), (0, 1)], kind="SEGMENT")]
    flipped = [_face([(0, 0), (1, 0), (1, 1), (0, 1)], kind="KITE")]
    split = base + [_face([(2, 0), (3, 0), (3, 1)], kind="SEGMENT")]
    assert topology_signature(base) == topology_signature(moved)
    assert topology_signature(base) != topology_signature(flipped)
    assert topology_signature(base) != topology_signature(split)


def test_controller_update_vs_rebuild_decision():
    adapter = FakeAdapter()
    controller = GpuPreviewController(adapter=adapter)
    assert controller.start() is True
    faces = [_face([(0, 0), (1, 0), (1, 1), (0, 1)])]
    assert controller.update(faces) == "UPDATED"
    moved = [_face([(0, 0), (2, 0), (2, 1), (0, 1)])]
    assert controller.update(moved) == "UPDATED"
    changed = moved + [_face([(3, 0), (4, 0), (4, 1)], kind="CAP")]
    assert controller.update(changed) == "UPDATED"
    rebuild_flags = [rebuild for rebuild, _count in adapter.uploads]
    assert rebuild_flags == [True, False, True]
    assert controller.rebuilds == 2 and controller.updates == 3


def test_controller_retains_last_valid_on_bad_frames():
    adapter = FakeAdapter()
    controller = GpuPreviewController(adapter=adapter)
    controller.start()
    faces = [_face([(0, 0), (1, 0), (1, 1)])]
    controller.update(faces)
    kept = controller.last_buffers
    assert controller.update([], "UPDATED") == "RETAINED_LAST_VALID"
    assert controller.update(faces, "ERROR") == "RETAINED_LAST_VALID"
    assert (
        controller.update(faces, "RETAINED_LAST_VALID")
        == "RETAINED_LAST_VALID"
    )
    assert controller.last_buffers is kept
    assert len(adapter.uploads) == 1


def test_controller_lifecycle_idempotent_no_leaks():
    adapter = FakeAdapter()
    controller = GpuPreviewController(adapter=adapter)
    for _cycle in range(100):
        controller.start()
        controller.start()
        controller.stop()
        controller.stop()
    assert adapter.handler_count == 0
    assert controller.active is False


def test_controller_upload_failure_goes_failed_and_stops():
    adapter = FakeAdapter(fail_upload=True)
    controller = GpuPreviewController(adapter=adapter)
    controller.start()
    faces = [_face([(0, 0), (1, 0), (1, 1)])]
    assert controller.update(faces) == "FAILED"
    assert controller.failed is True
    assert controller.active is False
    assert adapter.handler_count == 0
    # Повторный старт после fail запрещён (постоянный mesh fallback).
    assert controller.start() is False


def test_draw_never_raises_and_sets_failed():
    class ExplodingAdapter(FakeAdapter):
        def draw(self):
            raise RuntimeError("shader died")

    adapter = ExplodingAdapter()
    controller = GpuPreviewController(adapter=adapter)
    controller.start()
    controller.update([_face([(0, 0), (1, 0), (1, 1)])])
    controller._draw()  # не должен бросить
    assert controller.failed is True


def test_factory_returns_none_for_mesh_mode_and_background(monkeypatch):
    assert create_controller(display_mode="MESH") is None
    import sys

    class _App:
        background = True

    class _Bpy:
        app = _App()

    monkeypatch.setitem(sys.modules, "bpy", _Bpy())
    assert create_controller(display_mode="GPU") is None
