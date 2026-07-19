"""F3.b: GPU preview routing at the modal operator boundary."""

from types import SimpleNamespace

import pytest

import tests.test_operators  # noqa: F401 - installs bpy UI stubs

from cftuv import operators as ops_module
from cftuv.decal_gpu_preview import GpuPreviewController
from cftuv.decals import ManualSeamFacesResult, PreviewStatus
from tests.test_decal_gpu_preview import FakeAdapter, _face


def _operator_with_controller():
    operator = ops_module.HOTSPOTUV_OT_GenerateDecals.__new__(
        ops_module.HOTSPOTUV_OT_GenerateDecals
    )
    operator.mode = "SEAMS"
    controller = GpuPreviewController(adapter=FakeAdapter())
    controller.start()
    operator._modal_gpu_preview = controller
    operator._modal_decal_plan = object()
    return operator, controller


def _context():
    return SimpleNamespace(scene="SCENE")


def _state():
    return ("SOURCE_OBJ", None, "BASE_SETTINGS", None, True, 1, (1,))


def test_generate_routes_preview_to_gpu_and_skips_mesh(monkeypatch):
    operator, controller = _operator_with_controller()
    faces = (_face([(0, 0), (1, 0), (1, 1)]),)
    monkeypatch.setattr(
        ops_module,
        "evaluate_manual_seam_faces",
        lambda obj, settings, plan, preview=True: ManualSeamFacesResult(
            faces=faces, policy_counts=(("MITER", 1),), evaluation_ms=1.5
        ),
    )
    mesh_calls = []
    monkeypatch.setattr(
        ops_module,
        "generate_decal_result",
        lambda *args, **kwargs: mesh_calls.append(args),
    )

    result = operator._generate(_context(), _state(), "SETTINGS", preview=True)

    assert result.status == PreviewStatus.UPDATED
    assert result.object_name is None
    assert result.policy_counts == (("MITER", 1),)
    assert mesh_calls == []
    assert controller.updates == 1


def test_generate_confirm_never_routes_to_gpu(monkeypatch):
    operator, controller = _operator_with_controller()
    monkeypatch.setattr(
        ops_module,
        "evaluate_manual_seam_faces",
        lambda *args, **kwargs: pytest.fail("GPU path used on confirm"),
    )
    sentinel = object()
    monkeypatch.setattr(
        ops_module,
        "generate_decal_result",
        lambda *args, **kwargs: sentinel,
    )

    assert operator._generate(
        _context(), _state(), "SETTINGS", preview=False
    ) is sentinel
    assert controller.updates == 0


def test_generate_gpu_evaluator_error_propagates(monkeypatch):
    operator, _controller = _operator_with_controller()

    def _boom(*_args, **_kwargs):
        raise RuntimeError("DOMAIN_BUDGET_EXCEEDED: width over budget")

    monkeypatch.setattr(ops_module, "evaluate_manual_seam_faces", _boom)
    with pytest.raises(RuntimeError, match="DOMAIN_BUDGET_EXCEEDED"):
        operator._generate(_context(), _state(), "SETTINGS", preview=True)


def test_generate_adapter_failure_falls_back_to_mesh(monkeypatch):
    operator, controller = _operator_with_controller()
    controller.adapter.fail_upload = True
    faces = (_face([(0, 0), (1, 0), (1, 1)]),)
    monkeypatch.setattr(
        ops_module,
        "evaluate_manual_seam_faces",
        lambda *args, **kwargs: ManualSeamFacesResult(faces=faces),
    )
    sentinel = object()
    monkeypatch.setattr(
        ops_module,
        "generate_decal_result",
        lambda *args, **kwargs: sentinel,
    )

    assert operator._generate(
        _context(), _state(), "SETTINGS", preview=True
    ) is sentinel
    assert operator._modal_gpu_preview is None
    assert controller.adapter.handler_count == 0


def test_empty_faces_keep_overlay_and_report_empty(monkeypatch):
    operator, controller = _operator_with_controller()
    controller.update((_face([(0, 0), (1, 0), (1, 1)]),))
    kept = controller.last_buffers
    monkeypatch.setattr(
        ops_module,
        "evaluate_manual_seam_faces",
        lambda *args, **kwargs: ManualSeamFacesResult(faces=()),
    )

    result = operator._generate(_context(), _state(), "SETTINGS", preview=True)

    assert result.status == PreviewStatus.EMPTY
    assert controller.last_buffers is kept


def test_stop_gpu_preview_is_idempotent():
    operator, controller = _operator_with_controller()
    operator._stop_gpu_preview()
    operator._stop_gpu_preview()
    assert controller.active is False
    assert controller.adapter.handler_count == 0
