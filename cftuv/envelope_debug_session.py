"""WindowManager-owned lifecycle and caches for Envelope debug.

The controller has no module-global instance.  Blender stores one controller
on the active WindowManager as ``_cftuv_envelope_debug_session``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Hashable, TYPE_CHECKING

from .envelope_debug_profile import EnvelopeDebugProfileBuilderV1
from .envelope_metric_export import (
    EnvelopeDomainGeometryExportV1,
    EnvelopePatchMetricExportV1,
    build_envelope_domain_geometry_export,
    build_envelope_patch_metric_export,
)
from .envelope_topology_export import (
    EnvelopeTopologyExportV1,
    build_envelope_topology_export,
)

if TYPE_CHECKING:
    from .surface_ir import AnalysisBundle, SourceRevision


COMPILE_CONTRACT_ALPHA_INDEPENDENT = False
WINDOW_MANAGER_SESSION_ATTRIBUTE = "_cftuv_envelope_debug_session"


@dataclass(frozen=True, slots=True)
class CompiledEnvelopeCacheKeyV1:
    """Compile-static key; intentionally excludes requested alpha."""

    source_revision_value: str
    selected_chain_use_ids: tuple[str, ...]
    policy_values: tuple[str, ...]

    @classmethod
    def from_request(cls, source_revision_value: str, request):
        def value(item) -> str:
            return str(item.value) if hasattr(item, "value") else str(item)

        return cls(
            str(source_revision_value),
            tuple(
                sorted(value(item) for item in request.selected_chain_use_ids)
            ),
            tuple(
                value(getattr(request, name))
                for name in (
                    "metric_space",
                    "angular_profile_family_id",
                    "angular_profile_selection_policy_id",
                    "max_subturn_parameter_id",
                    "max_subturn_value_id",
                    "max_subturn_exact_value",
                    "cap_policy_id",
                    "boundary_policy_id",
                    "interaction_policy_id",
                    "ownership_policy_id",
                    "material_policy_id",
                    "uv_policy_id",
                )
            ),
        )


@dataclass(frozen=True, slots=True)
class QueueSessionStateV1:
    """Последний прогон движка QUEUE: чем перерисовать при смене alpha.

    Хранится сессией, а не панелью: подготовка — самая дорогая часть, и
    ползунок обязан находить её там же, где её оставила кнопка. Сцены
    неизменяемы, поэтому лёгкий путь переиспользует их как есть.
    """

    source_object_name: str
    topology_scene: object
    exact_scenes: tuple
    #: `(patch_id, patch_domain_id, ConveyorPreparationV1)` по каждому домену,
    #: дошедшему до подготовки. Домены, отказавшие раньше, сюда не попадают.
    entries: tuple
    receipts: tuple


@dataclass(frozen=True, slots=True)
class _CachedMetricFailure:
    outcome: object
    message: str
    patch_domain_id: str | None

    def raise_error(self) -> None:
        from .envelope_request_export import EnvelopeHostAdapterError

        raise EnvelopeHostAdapterError(
            self.outcome,
            self.message,
            patch_domain_id=self.patch_domain_id,
        )


class EnvelopeDebugSessionController:
    """Explicit SourceRevision-scoped cache owner for one WindowManager."""

    def __init__(self) -> None:
        self._source_state_by_object: dict[
            Hashable, tuple[Hashable, str]
        ] = {}
        self._analysis_bundle_cache: dict[
            tuple[Hashable, str], AnalysisBundle
        ] = {}
        self._topology_export_cache: dict[str, EnvelopeTopologyExportV1] = {}
        self._patch_metric_cache: dict[
            tuple[str, str],
            EnvelopePatchMetricExportV1 | _CachedMetricFailure,
        ] = {}
        self._domain_geometry_cache: dict[
            tuple[str, str], EnvelopeDomainGeometryExportV1
        ] = {}
        self._compiled_envelope_cache: dict[
            CompiledEnvelopeCacheKeyV1, object
        ] = {}
        # Подготовка очереди alpha-НЕЗАВИСИМА (ядро доказало это побитовым
        # совпадением скелета при alpha 0.25 и 0.5), поэтому ключ её и не
        # содержит: только ревизия источника, домен и выделенные рёбра домена.
        self._conveyor_preparation_cache: dict[
            tuple[str, str, frozenset[int]], object
        ] = {}
        self._queue_session: QueueSessionStateV1 | None = None
        self._build_counts: dict[str, int] = {
            "ANALYSIS_BUNDLE": 0,
            "TOPOLOGY_EXPORT": 0,
            "PATCH_METRIC": 0,
            "DOMAIN_GEOMETRY": 0,
            "COMPILED_ENVELOPE": 0,
            "CONVEYOR_PREPARATION": 0,
        }
        self._cache_build_counts: dict[tuple[str, object], int] = {}
        self._invalidation_count = 0

    @property
    def build_counts(self) -> dict[str, int]:
        return dict(self._build_counts)

    @property
    def invalidation_count(self) -> int:
        return self._invalidation_count

    @property
    def queue_session(self) -> QueueSessionStateV1 | None:
        return self._queue_session

    def clear(self) -> None:
        self._source_state_by_object.clear()
        self._analysis_bundle_cache.clear()
        self._topology_export_cache.clear()
        self._patch_metric_cache.clear()
        self._domain_geometry_cache.clear()
        self._compiled_envelope_cache.clear()
        self._conveyor_preparation_cache.clear()
        self._queue_session = None
        self._invalidation_count += 1

    def _prepare_source(
        self,
        source_object_key: Hashable,
        source_data_key: Hashable,
        source_revision_value: str,
        profile: EnvelopeDebugProfileBuilderV1 | None,
    ) -> None:
        state = (source_data_key, source_revision_value)
        previous = self._source_state_by_object.get(source_object_key)
        if previous is not None and previous != state:
            self.clear()
            if profile is not None:
                profile.set_counter("SESSION_CACHE_INVALIDATED", 1)
        self._source_state_by_object[source_object_key] = state
        if profile is not None:
            profile.set_counter(
                "SESSION_CACHE_INVALIDATION_COUNT",
                self._invalidation_count,
            )

    def _record_cache(
        self,
        profile: EnvelopeDebugProfileBuilderV1 | None,
        layer: str,
        hit: bool,
        *,
        patch_domain_id: str | None = None,
        cache_key: object | None = None,
    ) -> None:
        if profile is None:
            return
        profile.set_counter(
            f"{layer}_CACHE_HIT",
            int(hit),
            patch_domain_id,
        )
        profile.set_counter(
            f"{layer}_CACHE_MISS",
            int(not hit),
            patch_domain_id,
        )
        profile.set_counter(
            f"{layer}_BUILD_COUNT",
            self._cache_build_counts.get(
                (layer, cache_key),
                self._build_counts[layer],
            ),
            patch_domain_id,
        )

    @staticmethod
    def _revision_value(source_revision: SourceRevision) -> str:
        from .envelope_request_export import _revision_value

        return _revision_value(source_revision)

    def get_analysis_bundle(
        self,
        source_object_key: Hashable,
        source_data_key: Hashable,
        source_revision: SourceRevision,
        build: Callable[[], AnalysisBundle],
        *,
        profile: EnvelopeDebugProfileBuilderV1 | None = None,
    ) -> AnalysisBundle:
        revision = self._revision_value(source_revision)
        self._prepare_source(
            source_object_key,
            source_data_key,
            revision,
            profile,
        )
        key = (source_object_key, revision)
        cached = self._analysis_bundle_cache.get(key)
        if cached is not None:
            if profile is not None:
                with profile.measure("ANALYSIS_BUNDLE"):
                    pass
            self._record_cache(
                profile,
                "ANALYSIS_BUNDLE",
                True,
                cache_key=key,
            )
            return cached
        if profile is None:
            bundle = build()
        else:
            with profile.measure("ANALYSIS_BUNDLE"):
                bundle = build()
        if (
            str(bundle.source_revision.source_name)
            != str(source_revision.source_name)
            or str(bundle.source_revision.digest) != str(source_revision.digest)
        ):
            raise ValueError(
                "analysis builder returned a different SourceRevision"
            )
        self._analysis_bundle_cache[key] = bundle
        self._build_counts["ANALYSIS_BUNDLE"] += 1
        self._cache_build_counts[("ANALYSIS_BUNDLE", key)] = (
            self._cache_build_counts.get(("ANALYSIS_BUNDLE", key), 0) + 1
        )
        self._record_cache(
            profile,
            "ANALYSIS_BUNDLE",
            False,
            cache_key=key,
        )
        return bundle

    def get_topology_export(
        self,
        analysis_bundle: AnalysisBundle,
        source_object_key: Hashable,
        source_data_key: Hashable,
        *,
        profile: EnvelopeDebugProfileBuilderV1 | None = None,
    ) -> EnvelopeTopologyExportV1:
        revision = self._revision_value(analysis_bundle.source_revision)
        self._prepare_source(
            source_object_key,
            source_data_key,
            revision,
            profile,
        )
        cached = self._topology_export_cache.get(revision)
        if cached is not None:
            self._record_cache(
                profile,
                "TOPOLOGY_EXPORT",
                True,
                cache_key=revision,
            )
            return cached
        if profile is None:
            export = build_envelope_topology_export(analysis_bundle)
        else:
            with profile.measure("TOPOLOGY_EXPORT"):
                export = build_envelope_topology_export(
                    analysis_bundle,
                    profile=profile,
                )
        self._topology_export_cache[revision] = export
        self._build_counts["TOPOLOGY_EXPORT"] += 1
        self._cache_build_counts[("TOPOLOGY_EXPORT", revision)] = (
            self._cache_build_counts.get(
                ("TOPOLOGY_EXPORT", revision),
                0,
            )
            + 1
        )
        self._record_cache(
            profile,
            "TOPOLOGY_EXPORT",
            False,
            cache_key=revision,
        )
        return export

    def get_patch_metric(
        self,
        topology_export: EnvelopeTopologyExportV1,
        patch_id: int,
        *,
        profile: EnvelopeDebugProfileBuilderV1 | None = None,
    ) -> EnvelopePatchMetricExportV1:
        domain_id = topology_export.patch_domain_id_by_patch[int(patch_id)]
        key = (topology_export.source_revision_value, domain_id)
        cached = self._patch_metric_cache.get(key)
        if cached is not None:
            self._record_cache(
                profile,
                "PATCH_METRIC",
                True,
                patch_domain_id=domain_id,
                cache_key=key,
            )
            if isinstance(cached, _CachedMetricFailure):
                cached.raise_error()
            return cached
        try:
            metric = build_envelope_patch_metric_export(
                topology_export,
                patch_id,
                profile=profile,
            )
        except Exception as exc:
            from .envelope_request_export import EnvelopeHostAdapterError

            self._build_counts["PATCH_METRIC"] += 1
            self._cache_build_counts[("PATCH_METRIC", key)] = (
                self._cache_build_counts.get(
                    ("PATCH_METRIC", key),
                    0,
                )
                + 1
            )
            self._record_cache(
                profile,
                "PATCH_METRIC",
                False,
                patch_domain_id=domain_id,
                cache_key=key,
            )
            if isinstance(exc, EnvelopeHostAdapterError):
                self._patch_metric_cache[key] = _CachedMetricFailure(
                    exc.outcome,
                    str(exc),
                    exc.patch_domain_id,
                )
            raise
        self._patch_metric_cache[key] = metric
        self._build_counts["PATCH_METRIC"] += 1
        self._cache_build_counts[("PATCH_METRIC", key)] = (
            self._cache_build_counts.get(("PATCH_METRIC", key), 0) + 1
        )
        self._record_cache(
            profile,
            "PATCH_METRIC",
            False,
            patch_domain_id=domain_id,
            cache_key=key,
        )
        return metric

    def get_domain_geometry(
        self,
        metric_export: EnvelopePatchMetricExportV1,
        *,
        profile: EnvelopeDebugProfileBuilderV1 | None = None,
    ) -> EnvelopeDomainGeometryExportV1:
        key = (
            metric_export.source_revision_value,
            metric_export.patch_domain_id,
        )
        cached = self._domain_geometry_cache.get(key)
        if cached is not None:
            self._record_cache(
                profile,
                "DOMAIN_GEOMETRY",
                True,
                patch_domain_id=metric_export.patch_domain_id,
                cache_key=key,
            )
            return cached
        geometry = build_envelope_domain_geometry_export(
            metric_export,
            profile=profile,
        )
        self._domain_geometry_cache[key] = geometry
        self._build_counts["DOMAIN_GEOMETRY"] += 1
        self._cache_build_counts[("DOMAIN_GEOMETRY", key)] = (
            self._cache_build_counts.get(("DOMAIN_GEOMETRY", key), 0) + 1
        )
        self._record_cache(
            profile,
            "DOMAIN_GEOMETRY",
            False,
            patch_domain_id=metric_export.patch_domain_id,
            cache_key=key,
        )
        return geometry

    def get_conveyor_preparation(
        self,
        source_revision_value: str,
        patch_domain_id: str,
        selected_edge_ids: frozenset[int],
        build,
        *,
        profile: EnvelopeDebugProfileBuilderV1 | None = None,
    ):
        """Подготовка очереди из кэша либо построенная и запомненная.

        Ключ не содержит alpha намеренно: alpha-независимость подготовки
        доказана ядром побитово, и включение alpha в ключ обнуляло бы кэш на
        каждом движении ползунка — то есть отменяло бы весь смысл разреза.
        """

        key = (
            str(source_revision_value),
            str(patch_domain_id),
            frozenset(int(item) for item in selected_edge_ids),
        )
        cached = self._conveyor_preparation_cache.get(key)
        if cached is not None:
            self._record_cache(
                profile,
                "CONVEYOR_PREPARATION",
                True,
                patch_domain_id=patch_domain_id,
                cache_key=key,
            )
            return cached
        prepared = build()
        self._conveyor_preparation_cache[key] = prepared
        self._build_counts["CONVEYOR_PREPARATION"] += 1
        self._cache_build_counts[("CONVEYOR_PREPARATION", key)] = (
            self._cache_build_counts.get(("CONVEYOR_PREPARATION", key), 0) + 1
        )
        self._record_cache(
            profile,
            "CONVEYOR_PREPARATION",
            False,
            patch_domain_id=patch_domain_id,
            cache_key=key,
        )
        return prepared

    def remember_queue_session(self, state: QueueSessionStateV1) -> None:
        self._queue_session = state

    def evaluate_staged(
        self,
        analysis_bundle: AnalysisBundle,
        selected_physical_edge_ids: frozenset[int],
        alpha: float,
        *,
        source_object_key: Hashable,
        source_data_key: Hashable,
        profile: EnvelopeDebugProfileBuilderV1 | None = None,
        engine: str = "LEGACY",
    ):
        from .envelope_queue_export import (
            ENVELOPE_DEBUG_ENGINE_QUEUE,
            evaluate_envelope_queue_staged,
        )
        from .envelope_request_export import (
            evaluate_envelope_debug_staged as _evaluate,
        )

        topology_export = self.get_topology_export(
            analysis_bundle,
            source_object_key,
            source_data_key,
            profile=profile,
        )
        if profile is not None:
            profile.set_counter(
                "COMPILED_ENVELOPE_CACHE_ENABLED",
                int(COMPILE_CONTRACT_ALPHA_INDEPENDENT),
            )
            profile.set_counter(
                "COMPILED_ENVELOPE_CACHE_BYPASS_ALPHA_DEPENDENT",
                int(not COMPILE_CONTRACT_ALPHA_INDEPENDENT),
            )
            profile.set_counter(
                "COMPILED_ENVELOPE_BUILD_COUNT",
                self._build_counts["COMPILED_ENVELOPE"],
            )

        def snapshot_provider(patch_id: int, _domain_id: str):
            metric = self.get_patch_metric(
                topology_export,
                patch_id,
                profile=profile,
            )
            return self.get_domain_geometry(
                metric,
                profile=profile,
            ).snapshot

        if str(engine) != ENVELOPE_DEBUG_ENGINE_QUEUE:
            return _evaluate(
                analysis_bundle,
                selected_physical_edge_ids,
                alpha,
                profile=profile,
                topology_export=topology_export,
                domain_snapshot_provider=snapshot_provider,
            )

        revision = topology_export.source_revision_value

        def preparation_provider(
            _patch_id,
            domain_id,
            selected_edges,
            snapshot,
            request,
        ):
            from cftuv_envelope.wavefront import prepare_conveyor

            return self.get_conveyor_preparation(
                revision,
                domain_id,
                selected_edges,
                lambda: prepare_conveyor(snapshot, request),
                profile=profile,
            )

        return evaluate_envelope_queue_staged(
            analysis_bundle,
            selected_physical_edge_ids,
            alpha,
            profile=profile,
            topology_export=topology_export,
            domain_snapshot_provider=snapshot_provider,
            preparation_provider=preparation_provider,
        )


def evaluate_envelope_debug_staged(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
    alpha: float,
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
    controller: EnvelopeDebugSessionController | None = None,
    source_object_key: Hashable | None = None,
    source_data_key: Hashable | None = None,
    engine: str = "LEGACY",
):
    """Compatibility entry point with optional persistent session reuse."""

    if controller is None:
        from .envelope_queue_export import (
            ENVELOPE_DEBUG_ENGINE_QUEUE,
            evaluate_envelope_queue_staged,
        )
        from .envelope_request_export import (
            evaluate_envelope_debug_staged as _evaluate,
        )

        topology_export = build_envelope_topology_export(
            analysis_bundle,
            profile=profile,
        )
        run = (
            evaluate_envelope_queue_staged
            if str(engine) == ENVELOPE_DEBUG_ENGINE_QUEUE
            else _evaluate
        )
        return run(
            analysis_bundle,
            selected_physical_edge_ids,
            alpha,
            profile=profile,
            topology_export=topology_export,
        )
    if source_object_key is None or source_data_key is None:
        raise ValueError(
            "persistent Envelope debug session requires source object/data keys"
        )
    return controller.evaluate_staged(
        analysis_bundle,
        selected_physical_edge_ids,
        alpha,
        source_object_key=source_object_key,
        source_data_key=source_data_key,
        profile=profile,
        engine=engine,
    )


class _WindowManagerSessionAttribute:
    """Blender RNA-compatible runtime attribute descriptor."""

    def __init__(self) -> None:
        self._controllers: dict[int, EnvelopeDebugSessionController] = {}

    def __get__(self, instance, _owner):
        if instance is None:
            return self
        return self._controllers.get(int(instance.as_pointer()))

    def __set__(self, instance, value) -> None:
        if not isinstance(value, EnvelopeDebugSessionController):
            raise TypeError(
                "WindowManager Envelope session must be "
                "EnvelopeDebugSessionController"
            )
        self._controllers[int(instance.as_pointer())] = value

    def __delete__(self, instance) -> None:
        controller = self._controllers.pop(
            int(instance.as_pointer()),
            None,
        )
        if controller is not None:
            controller.clear()

    def clear_all(self) -> None:
        for controller in self._controllers.values():
            controller.clear()
        self._controllers.clear()


def register_window_manager_session_attribute() -> None:
    """Install the runtime descriptor; no controller is module-global."""

    import bpy

    existing = getattr(
        bpy.types.WindowManager,
        WINDOW_MANAGER_SESSION_ATTRIBUTE,
        None,
    )
    if existing is not None and hasattr(existing, "clear_all"):
        existing.clear_all()
    setattr(
        bpy.types.WindowManager,
        WINDOW_MANAGER_SESSION_ATTRIBUTE,
        _WindowManagerSessionAttribute(),
    )


def unregister_window_manager_session_attribute() -> None:
    import bpy

    existing = getattr(
        bpy.types.WindowManager,
        WINDOW_MANAGER_SESSION_ATTRIBUTE,
        None,
    )
    if existing is not None and hasattr(existing, "clear_all"):
        existing.clear_all()
    if hasattr(bpy.types.WindowManager, WINDOW_MANAGER_SESSION_ATTRIBUTE):
        delattr(
            bpy.types.WindowManager,
            WINDOW_MANAGER_SESSION_ATTRIBUTE,
        )


__all__ = (
    "COMPILE_CONTRACT_ALPHA_INDEPENDENT",
    "CompiledEnvelopeCacheKeyV1",
    "EnvelopeDebugSessionController",
    "QueueSessionStateV1",
    "WINDOW_MANAGER_SESSION_ATTRIBUTE",
    "evaluate_envelope_debug_staged",
    "register_window_manager_session_attribute",
    "unregister_window_manager_session_attribute",
)
