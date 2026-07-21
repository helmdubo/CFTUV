from types import SimpleNamespace

import pytest

from cftuv.decal_session import (
    CapturedDecalRequest,
    DecalConfirmability,
    DecalSessionController,
    DecalSessionState,
)
from cftuv.decal_transform import metric_context_for_source
from cftuv.decals import DecalGenerationResult, PreviewStatus
from cftuv.model import DecalSettings, LocalDecalSettings


class _Matrix3:
    def __init__(self, scale):
        self.rows = (
            (float(scale), 0.0, 0.0),
            (0.0, float(scale), 0.0),
            (0.0, 0.0, float(scale)),
        )
        self.read_count = 0

    def __getitem__(self, index):
        self.read_count += 1
        return self.rows[index]

    def to_3x3(self):
        return self

    def determinant(self):
        return self.rows[0][0] * self.rows[1][1] * self.rows[2][2]


def _request(*, scale=2.0, settings=None):
    matrix = _Matrix3(scale)
    source_obj = SimpleNamespace(name="Source", matrix_world=matrix)
    request = CapturedDecalRequest(
        mode="SEAMS",
        source_obj=source_obj,
        analysis_bundle=object(),
        world_settings=settings or DecalSettings(width_seam=0.2),
        metric_context=metric_context_for_source(source_obj),
        chain_refs=((0, 0, 0),),
        manual_selection=True,
        selected_edge_indices=(7, 3),
    )
    return request, matrix


def _updated(name="Preview"):
    return DecalGenerationResult(PreviewStatus.UPDATED, name)


def _controller(request, *, compile_plan=None, evaluate=None):
    compile_calls = []
    evaluation_calls = []
    clear_calls = []
    cleanup_calls = []

    def compile_port(local_request):
        compile_calls.append(local_request)
        if compile_plan is not None:
            return compile_plan(local_request, len(compile_calls))
        return f"plan-{len(compile_calls)}"

    def evaluate_port(local_request, plan, preview):
        evaluation_calls.append((local_request, plan, preview))
        if evaluate is not None:
            return evaluate(local_request, plan, preview, len(evaluation_calls))
        return _updated()

    controller = DecalSessionController(
        request,
        compile_plan=compile_port,
        evaluate=evaluate_port,
        clear_preview=lambda: clear_calls.append(True),
        cleanup=lambda restore: cleanup_calls.append(restore),
        preview_state=object(),
    )
    return (
        controller,
        compile_calls,
        evaluation_calls,
        clear_calls,
        cleanup_calls,
    )


def test_session_owns_one_metric_context_across_drag_requests():
    request, matrix = _request()
    controller, compile_calls, evaluations, _clear, _cleanup = _controller(
        request
    )
    metric_reads = matrix.read_count

    initial = controller.begin()
    dragged = controller.preview(DecalSettings(width_seam=0.6))

    assert matrix.read_count == metric_reads
    assert controller.metric_context is controller.initial_local_request.metric_context
    assert compile_calls[0].metric_context is controller.metric_context
    assert isinstance(compile_calls[0].settings, LocalDecalSettings)
    assert compile_calls[0].settings.width_seam == pytest.approx(0.1)
    assert initial.local_request.settings.width_seam == pytest.approx(0.1)
    assert dragged.local_request.settings.width_seam == pytest.approx(0.3)
    assert evaluations[1][0].metric_context is controller.metric_context
    assert controller.current_world_settings.width_seam == pytest.approx(0.6)
    assert controller.confirmability == DecalConfirmability.READY


def test_failed_preview_clears_and_blocks_stale_confirm():
    request, _matrix = _request()

    def evaluate(_local, _plan, preview, call_index):
        if call_index == 1:
            return _updated()
        assert preview is True
        return DecalGenerationResult(
            PreviewStatus.ERROR,
            None,
            reason="invalid crop",
        )

    controller, _compiles, evaluations, clear, cleanup = _controller(
        request,
        evaluate=evaluate,
    )
    controller.begin()
    controller.preview(DecalSettings(width_seam=0.4))

    assert clear == [True]
    assert controller.last_valid_result.object_name == "Preview"
    assert controller.confirmability == DecalConfirmability.BLOCKED
    confirmed = controller.confirm()

    assert confirmed.result.status == PreviewStatus.ERROR
    assert confirmed.result.reason == "invalid crop"
    assert len(evaluations) == 2
    assert cleanup == [True]
    assert controller.state == DecalSessionState.CANCELLED


def test_budget_excess_recompiles_only_at_confirm():
    request, _matrix = _request()

    def evaluate(_local, plan, preview, call_index):
        if call_index == 2:
            assert plan == "plan-1"
            assert preview is True
            return DecalGenerationResult(
                PreviewStatus.ERROR,
                None,
                reason="DOMAIN_BUDGET_EXCEEDED: requested width",
            )
        return _updated("Final" if not preview else "Preview")

    controller, compiles, evaluations, clear, cleanup = _controller(
        request,
        evaluate=evaluate,
    )
    controller.begin()
    controller.preview(DecalSettings(width_seam=0.8))

    assert len(compiles) == 1
    assert controller.confirmability == DecalConfirmability.CONTROLLED_RECOMPILE
    assert controller.current_world_settings.width_seam == pytest.approx(0.2)
    assert clear == [True]

    confirmed = controller.confirm()

    assert len(compiles) == 2
    assert compiles[1].settings.width_seam == pytest.approx(0.4)
    assert evaluations[-1][1:] == ("plan-2", False)
    assert confirmed.result.object_name == "Final"
    assert controller.compiled_plan == "plan-2"
    assert controller.current_world_settings.width_seam == pytest.approx(0.8)
    assert cleanup == [False]
    assert controller.state == DecalSessionState.FINISHED


def test_failed_controlled_recompile_keeps_previous_plan_and_cleans_once():
    request, _matrix = _request()

    def compile_plan(_local, call_index):
        if call_index == 2:
            raise RuntimeError("compile exploded")
        return "initial-plan"

    def evaluate(_local, _plan, _preview, call_index):
        if call_index == 2:
            return DecalGenerationResult(
                PreviewStatus.ERROR,
                None,
                reason="DOMAIN_BUDGET_EXCEEDED",
            )
        return _updated()

    controller, _compiles, _evaluations, _clear, cleanup = _controller(
        request,
        compile_plan=compile_plan,
        evaluate=evaluate,
    )
    controller.begin()
    controller.preview(DecalSettings(width_seam=0.8))
    result = controller.confirm()
    controller.cancel()

    assert result.result.reason == "compile exploded"
    assert controller.compiled_plan == "initial-plan"
    assert cleanup == [True]


def test_forced_cancel_cleanup_is_idempotent():
    request, _matrix = _request()
    controller, _compiles, _evaluations, _clear, cleanup = _controller(request)
    controller.begin()

    controller.cancel()
    controller.cancel()

    assert cleanup == [True]
    assert controller.state == DecalSessionState.CANCELLED


def test_immediate_execute_uses_same_owned_metric_and_plan():
    request, _matrix = _request(scale=4.0)
    controller, compiles, evaluations, _clear, cleanup = _controller(request)

    result = controller.execute()

    assert result.result.status == PreviewStatus.UPDATED
    assert compiles[0].settings.width_seam == pytest.approx(0.05)
    assert evaluations == [(compiles[0], "plan-1", False)]
    assert cleanup == []
    assert controller.state == DecalSessionState.FINISHED
