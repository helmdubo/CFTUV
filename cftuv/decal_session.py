"""Session ownership для compiled decal preview/confirm lifecycle."""

from dataclasses import dataclass
from enum import Enum

from .decal_transform import (
    LocalDecalRequest,
    MetricContext,
    local_decal_request_for_metric_context,
)
from .decals import DecalGenerationResult, PreviewStatus
from .model import WorldDecalSettings
from .surface_ir import PreviewFailurePolicy


class DecalSessionState(str, Enum):
    CREATED = "CREATED"
    RUNNING = "RUNNING"
    FINISHED = "FINISHED"
    CANCELLED = "CANCELLED"


class DecalConfirmability(str, Enum):
    UNAVAILABLE = "UNAVAILABLE"
    READY = "READY"
    CONTROLLED_RECOMPILE = "CONTROLLED_RECOMPILE"
    BLOCKED = "BLOCKED"
    CLOSED = "CLOSED"


@dataclass(frozen=True)
class CapturedDecalRequest:
    """Immutable invoke-time request; Blender selection не перечитывается."""

    mode: str
    source_obj: object
    analysis_bundle: object
    world_settings: WorldDecalSettings
    metric_context: MetricContext
    chain_refs: tuple | None
    manual_selection: bool
    selected_edge_indices: tuple[int, ...]

    def __post_init__(self):
        if not isinstance(self.world_settings, WorldDecalSettings):
            raise TypeError("CapturedDecalRequest requires WorldDecalSettings")
        if not isinstance(self.metric_context, MetricContext):
            raise TypeError("CapturedDecalRequest requires MetricContext")
        object.__setattr__(
            self,
            "chain_refs",
            None if self.chain_refs is None else tuple(self.chain_refs),
        )
        object.__setattr__(
            self,
            "selected_edge_indices",
            tuple(int(index) for index in self.selected_edge_indices),
        )

    @property
    def edge_count(self):
        return len(self.selected_edge_indices)


@dataclass(frozen=True)
class DecalSessionEvaluation:
    """Один world/local request и его authoritative engine result."""

    world_settings: WorldDecalSettings
    local_request: LocalDecalRequest
    result: DecalGenerationResult


class DecalSessionController:
    """Единственный владелец request/metric/plan/last-valid/confirmability.

    Порты оставляют Blender-объекты и UI за адаптером, но порядок compile,
    preview, controlled recompile и terminal cleanup задаётся только здесь.
    """

    def __init__(
        self,
        request,
        *,
        compile_plan,
        evaluate,
        clear_preview,
        cleanup,
        preview_state=None,
        preview_policy=PreviewFailurePolicy.CLEAR,
    ):
        if not isinstance(request, CapturedDecalRequest):
            raise TypeError("CapturedDecalRequest required")
        policy = PreviewFailurePolicy(preview_policy)
        if policy != PreviewFailurePolicy.CLEAR:
            raise ValueError("Decal sessions support PreviewFailurePolicy.CLEAR only")

        self.request = request
        self.preview_policy = policy
        self.preview_state = preview_state
        self._compile_plan_port = compile_plan
        self._evaluate_port = evaluate
        self._clear_preview_port = clear_preview
        self._cleanup_port = cleanup

        initial_local = local_decal_request_for_metric_context(
            request.world_settings,
            request.metric_context,
        )
        self.metric_context = initial_local.metric_context
        self.initial_local_request = initial_local
        self.compiled_plan = None
        self._compiled = False
        self.last_valid_evaluation = None
        self.last_preview_failure = ""
        self.pending_recompile_request = None
        self.state = DecalSessionState.CREATED
        self.confirmability = DecalConfirmability.UNAVAILABLE
        self._cleanup_done = False

    @property
    def current_world_settings(self):
        if self.last_valid_evaluation is not None:
            return self.last_valid_evaluation.world_settings
        return self.request.world_settings

    @property
    def last_valid_result(self):
        if self.last_valid_evaluation is None:
            return None
        return self.last_valid_evaluation.result

    @property
    def is_confirmable(self):
        return self.confirmability in {
            DecalConfirmability.READY,
            DecalConfirmability.CONTROLLED_RECOMPILE,
        }

    def local_request(self, world_settings):
        """Повторный drag-конверт использует captured MetricContext."""

        return local_decal_request_for_metric_context(
            world_settings,
            self.metric_context,
        )

    @staticmethod
    def _domain_budget_exceeded(reason):
        return "DOMAIN_BUDGET_EXCEEDED" in str(reason or "")

    def _evaluate(self, world_settings, local_request, plan, *, preview):
        try:
            result = self._evaluate_port(local_request, plan, bool(preview))
        except Exception as exc:
            result = DecalGenerationResult(
                PreviewStatus.ERROR,
                None,
                reason=str(exc),
                backend_summary=getattr(plan, "backend_summary", ""),
            )
        if not isinstance(result, DecalGenerationResult):
            raise TypeError("Decal session evaluate port must return DecalGenerationResult")
        return DecalSessionEvaluation(
            world_settings=world_settings,
            local_request=local_request,
            result=result,
        )

    def _accept(self, evaluation):
        self.last_valid_evaluation = evaluation
        self.last_preview_failure = ""
        self.pending_recompile_request = None
        self.confirmability = DecalConfirmability.READY
        return evaluation

    def _reject_preview(self, evaluation):
        self._clear_preview_port()
        reason = evaluation.result.reason or evaluation.result.status.value
        self.last_preview_failure = reason
        if self._domain_budget_exceeded(reason):
            self.pending_recompile_request = evaluation
            self.confirmability = DecalConfirmability.CONTROLLED_RECOMPILE
        else:
            self.pending_recompile_request = None
            self.confirmability = DecalConfirmability.BLOCKED
        return evaluation

    def begin(self):
        if self.state != DecalSessionState.CREATED:
            raise RuntimeError("DECAL_SESSION_ALREADY_STARTED")
        self.compile()
        self.state = DecalSessionState.RUNNING
        evaluation = self._evaluate(
            self.request.world_settings,
            self.initial_local_request,
            self.compiled_plan,
            preview=True,
        )
        if evaluation.result.status == PreviewStatus.UPDATED:
            return self._accept(evaluation)
        return self._reject_preview(evaluation)

    def compile(self):
        """Явная invoke boundary до запуска display adapter."""

        if self.state != DecalSessionState.CREATED:
            raise RuntimeError("DECAL_SESSION_ALREADY_STARTED")
        if not self._compiled:
            self.compiled_plan = self._compile_plan_port(
                self.initial_local_request
            )
            self._compiled = True
        return self.compiled_plan

    def preview(self, world_settings):
        if self.state != DecalSessionState.RUNNING:
            raise RuntimeError("DECAL_SESSION_NOT_RUNNING")
        local_request = self.local_request(world_settings)
        evaluation = self._evaluate(
            world_settings,
            local_request,
            self.compiled_plan,
            preview=True,
        )
        if evaluation.result.status == PreviewStatus.UPDATED:
            return self._accept(evaluation)
        return self._reject_preview(evaluation)

    def confirm(self):
        if self.state != DecalSessionState.RUNNING:
            raise RuntimeError("DECAL_SESSION_NOT_RUNNING")
        if not self.is_confirmable:
            reason = self.last_preview_failure or "no valid preview settings"
            result = DecalGenerationResult(
                PreviewStatus.ERROR,
                None,
                reason=reason,
                backend_summary=getattr(self.compiled_plan, "backend_summary", ""),
            )
            self._finish(DecalSessionState.CANCELLED, restore_scene=True)
            return DecalSessionEvaluation(
                self.current_world_settings,
                self.local_request(self.current_world_settings),
                result,
            )

        controlled_recompile = (
            self.confirmability == DecalConfirmability.CONTROLLED_RECOMPILE
        )
        request = (
            self.pending_recompile_request
            if controlled_recompile
            else self.last_valid_evaluation
        )
        if request is None:
            result = DecalGenerationResult(
                PreviewStatus.ERROR,
                None,
                reason="no valid preview settings",
            )
            self._finish(DecalSessionState.CANCELLED, restore_scene=True)
            return DecalSessionEvaluation(
                self.request.world_settings,
                self.initial_local_request,
                result,
            )

        previous_plan = self.compiled_plan
        try:
            plan = (
                self._compile_plan_port(request.local_request)
                if controlled_recompile
                else previous_plan
            )
            evaluation = self._evaluate(
                request.world_settings,
                request.local_request,
                plan,
                preview=False,
            )
        except Exception as exc:
            evaluation = DecalSessionEvaluation(
                request.world_settings,
                request.local_request,
                DecalGenerationResult(
                    PreviewStatus.ERROR,
                    None,
                    reason=str(exc),
                    backend_summary=getattr(previous_plan, "backend_summary", ""),
                ),
            )
            plan = previous_plan

        if evaluation.result.status != PreviewStatus.UPDATED:
            self.compiled_plan = previous_plan
            self._finish(DecalSessionState.CANCELLED, restore_scene=True)
            return evaluation

        self.compiled_plan = plan
        self._accept(evaluation)
        self._finish(DecalSessionState.FINISHED, restore_scene=False)
        return evaluation

    def execute(self):
        """Immediate/headless command: compile и exact evaluate один раз."""

        if self.state != DecalSessionState.CREATED:
            raise RuntimeError("DECAL_SESSION_ALREADY_STARTED")
        self.compile()
        evaluation = self._evaluate(
            self.request.world_settings,
            self.initial_local_request,
            self.compiled_plan,
            preview=False,
        )
        if evaluation.result.status == PreviewStatus.UPDATED:
            self.last_valid_evaluation = evaluation
            self.state = DecalSessionState.FINISHED
            self.confirmability = DecalConfirmability.CLOSED
        else:
            self.state = DecalSessionState.CANCELLED
            self.confirmability = DecalConfirmability.CLOSED
        return evaluation

    def cancel(self):
        if self.state in {DecalSessionState.FINISHED, DecalSessionState.CANCELLED}:
            return
        self._finish(DecalSessionState.CANCELLED, restore_scene=True)

    def _finish(self, state, *, restore_scene):
        self._cleanup(restore_scene=restore_scene)
        self.state = state
        self.confirmability = DecalConfirmability.CLOSED

    def _cleanup(self, *, restore_scene):
        if self._cleanup_done:
            return
        self._cleanup_done = True
        self._cleanup_port(bool(restore_scene))


__all__ = [
    "CapturedDecalRequest",
    "DecalConfirmability",
    "DecalSessionController",
    "DecalSessionEvaluation",
    "DecalSessionState",
]
