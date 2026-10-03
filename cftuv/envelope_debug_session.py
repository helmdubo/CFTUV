"""WindowManager-owned lifecycle and caches for Envelope debug.

The controller has no module-global instance.  Blender stores one controller
on the active WindowManager as ``_cftuv_envelope_debug_session``.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from typing import Callable, Hashable, TYPE_CHECKING

from .envelope_debug_profile import EnvelopeDebugProfileBuilderV1
from .envelope_domain_pool import shutdown_domain_pool
from .envelope_export_input import (
    build_host_export_input,
    patch_metric_from_worker,
    replay_export_records,
)
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
#: Сколько результатов продуктового пути держит сессия (вытесняется давнее по обращению).
#: Результат домена — батч с сеткой, поэтому запас считан в доменах: `building` (121) в
#: четыре прогона при разных alpha.
PRODUCTION_RESULT_CACHE_LIMIT = 512
#: Предел памяти замечаний к снапшотам доменов (по давности): домены одной ревизии (сотни на `building`) в него
#: входят с запасом, а сессия с несколькими мешами не копит снапшоты без счёта.
SNAPSHOT_ISSUES_CACHE_LIMIT = 512


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
    density: int | None


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


class _WorkerExportHooks:
    """Выгрузка снапшота доменов в воркерах пула, сквозь кэш метрики сессии.

    Три вызова склейки пула (`envelope_queue_pool`): провайдер входа выгрузки
    (`export_provider`), приём ответа воркера (`export_adopter`) и провайдер
    снапшота, который родитель зовёт в любом случае (`snapshot_provider`).
    Ответ воркера ждёт здесь своего домена и проходит кэш метрики как промах со
    сборкой: счётчики, счёт сборок и запомненный отказ те же, что у выгрузки в
    родителе.
    """

    def __init__(
        self,
        controller: EnvelopeDebugSessionController,
        topology_export: EnvelopeTopologyExportV1,
        profile: EnvelopeDebugProfileBuilderV1 | None,
    ) -> None:
        self._controller = controller
        self._topology_export = topology_export
        self._profile = profile
        self._adopted: dict[int, object] = {}

    def snapshot_provider(self, patch_id: int, _domain_id: str):
        topology_export = self._topology_export
        worker_result = self._adopted.pop(int(patch_id), None)
        metric = self._controller.get_patch_metric(
            topology_export,
            patch_id,
            profile=self._profile,
            build=(
                None
                if worker_result is None
                else lambda: patch_metric_from_worker(
                    topology_export, patch_id, worker_result
                )
            ),
        )
        return self._controller.get_domain_geometry(
            metric,
            profile=self._profile,
        ).snapshot

    def export_provider(self, patch_id, alpha, request_id, density):
        # Метрика в кэше — выгрузка домена попадание, и воркеру её не отдают:
        # подготовки без метрики в кэше не бывает, поэтому домен с холодной
        # метрикой всегда идёт воркеру целиком.
        if self._controller.has_patch_metric(self._topology_export, patch_id):
            return None
        return build_host_export_input(
            self._topology_export,
            patch_id,
            alpha=alpha,
            request_id=request_id,
            density=density,
        )

    def export_adopter(self, patch_id, result) -> None:
        replay_export_records(self._profile, result)
        self._adopted[int(patch_id)] = result


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
            tuple[str, str, frozenset[int], tuple[str, ...]], object
        ] = {}
        self._queue_session: QueueSessionStateV1 | None = None
        # Результаты продуктового пути по доменам: функция подготовки (её ключ), alpha и
        # законов, поэтому тот же ключ даёт тот же ответ без единого покрытия. Вытеснение
        # — по давности обращения (`PRODUCTION_RESULT_CACHE_LIMIT`).
        self._production_result_cache: OrderedDict[tuple, object] = OrderedDict()
        # Замечания проверки снапшота домена по тождеству снапшота: он от alpha не зависит, а запрос к нему
        # собирается на КАЖДОМ нажатии. Запись держит сам снапшот (занятое тождество не уходит другому).
        self._snapshot_issues: OrderedDict[int, tuple[object, tuple]] = OrderedDict()
        # Пиклы подготовок для воркеров пула (покрытие кэшированных подготовок
        # считается в них): живут и чистятся вместе с кэшем подготовок.
        self._preparation_blobs = None
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

    @property
    def preparation_blobs(self):
        if self._preparation_blobs is None:
            from .envelope_queue_pool import PreparationBlobsV1

            self._preparation_blobs = PreparationBlobsV1()
        return self._preparation_blobs

    def snapshot_issues(self, snapshot) -> tuple:
        """`validate_analysis_snapshot(snapshot)`: считается один раз на объект снапшота за сессию."""

        known = self._snapshot_issues.get(id(snapshot))
        if known is None or known[0] is not snapshot:
            from .envelope_request_export import _load_kernel

            kernel, _ = _load_kernel()
            known = (snapshot, tuple(kernel.validate_analysis_snapshot(snapshot)))
            self._snapshot_issues[id(snapshot)] = known
            while len(self._snapshot_issues) > SNAPSHOT_ISSUES_CACHE_LIMIT:
                self._snapshot_issues.popitem(last=False)
        else:
            self._snapshot_issues.move_to_end(id(snapshot))
        return known[1]

    def slider_coverage_pool(self, workers: int, profile):
        """Пул покрытия для ползунка alpha либо `None`: тогда считает родитель.

        Пул берётся только живой, уже поднятый кнопкой на ЭТО число воркеров:
        старт воркеров посреди перетаскивания стоил бы секунд.
        """

        from .envelope_domain_pool import peek_domain_pool
        from .envelope_queue_pool import SliderCoveragePool

        pool = peek_domain_pool(workers)
        if pool is None:
            return None
        return SliderCoveragePool(pool, self.preparation_blobs, profile)

    def clear(self) -> None:
        self._source_state_by_object.clear()
        self._analysis_bundle_cache.clear()
        self._topology_export_cache.clear()
        self._patch_metric_cache.clear()
        self._domain_geometry_cache.clear()
        self._compiled_envelope_cache.clear()
        self._conveyor_preparation_cache.clear()
        self._production_result_cache.clear()
        self._snapshot_issues.clear()
        if self._preparation_blobs is not None:
            self._preparation_blobs.clear()
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
        """Ключ состояния источника для кэшей сессии: ревизия И политика лестницы кривизны.

        Метрика домена зависит от политики лестницы хоста, а ревизия источника — нет, поэтому
        ключ без политики отдавал бы метрику, построенную при другой политике. Здесь политика
        только ключ кэша (смена состояния сбрасывает все кэши); идентичности внутри снапшота
        выводятся из ревизии источника сами и этой надстройки не видят.
        """

        from . import envelope_request_export as export_module

        policy = export_module.HOST_CURVATURE_LADDER_POLICY.value
        return f"{export_module._revision_value(source_revision)}|curvature-ladder:{policy}"

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

    def has_patch_metric(
        self,
        topology_export: EnvelopeTopologyExportV1,
        patch_id: int,
    ) -> bool:
        """Есть ли метрика патча в кэше. Счётчиков не пишет: это вопрос, не сборка."""

        domain_id = topology_export.patch_domain_id_by_patch[int(patch_id)]
        key = (topology_export.source_revision_value, domain_id)
        return key in self._patch_metric_cache

    def get_patch_metric(
        self,
        topology_export: EnvelopeTopologyExportV1,
        patch_id: int,
        *,
        profile: EnvelopeDebugProfileBuilderV1 | None = None,
        build: Callable[[], EnvelopePatchMetricExportV1] | None = None,
    ) -> EnvelopePatchMetricExportV1:
        """Метрика патча из кэша либо собранная и запомненная.

        `build` — готовая выгрузка, которую сделал воркер пула (либо её отказ:
        она поднимает `EnvelopeHostAdapterError`). Она проходит кэш как промах со
        сборкой: счётчики, счёт сборок и запомненный отказ те же, что у выгрузки
        в родителе.
        """

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
            metric = (
                build()
                if build is not None
                else build_envelope_patch_metric_export(
                    topology_export,
                    patch_id,
                    profile=profile,
                )
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

    @staticmethod
    def _preparation_key(
        source_revision_value: str,
        patch_domain_id: str,
        selected_edge_ids: frozenset[int],
        request,
    ):
        from .envelope_request_policy import envelope_request_policy_signature

        return (
            str(source_revision_value),
            str(patch_domain_id),
            frozenset(int(item) for item in selected_edge_ids),
            envelope_request_policy_signature(request),
        )

    def peek_conveyor_preparation(
        self,
        source_revision_value: str,
        patch_domain_id: str,
        selected_edge_ids: frozenset[int],
        request,
    ):
        """Подготовка из кэша либо `None`. Счётчиков не пишет: это вопрос.

        Попадание записывает `get_conveyor_preparation` — когда домен принят.
        """

        return self._conveyor_preparation_cache.get(
            self._preparation_key(
                source_revision_value,
                patch_domain_id,
                selected_edge_ids,
                request,
            )
        )

    def get_conveyor_preparation(
        self,
        source_revision_value: str,
        patch_domain_id: str,
        selected_edge_ids: frozenset[int],
        request,
        build,
        *,
        profile: EnvelopeDebugProfileBuilderV1 | None = None,
    ):
        """Подготовка очереди из кэша либо построенная и запомненная.

        Ключ не содержит alpha намеренно, но содержит каноническую подпись
        angular policy: геометрия подготовки зависит от плотности веера.
        """

        key = self._preparation_key(
            source_revision_value,
            patch_domain_id,
            selected_edge_ids,
            request,
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

    def worker_export_hooks(
        self,
        topology_export: EnvelopeTopologyExportV1,
        profile: EnvelopeDebugProfileBuilderV1 | None,
    ) -> _WorkerExportHooks:
        """Выгрузка домена в воркерах пула сквозь кэш метрики этой сессии."""

        return _WorkerExportHooks(self, topology_export, profile)

    def production_result_key(
        self,
        source_revision_value: str,
        patch_domain_id: str,
        selected_edge_ids: frozenset[int],
        request,
        *laws: Hashable,
    ) -> tuple:
        """Ключ результата продуктового пути: ключ подготовки и всё, что вне её.

        Подготовка alpha-независима и от законов материализации не зависит, а
        результат домена — функция подготовки, alpha и этих законов (`laws`: alpha
        текстом, закон UV, закон топологии, закон подъёма). Числа воркеров и размещения
        в ключе нет намеренно: размещение не меняет ответ.
        """

        return (
            self._preparation_key(
                source_revision_value,
                patch_domain_id,
                selected_edge_ids,
                request,
            ),
            *laws,
        )

    def peek_production_result(self, key: tuple):
        """Результат домена под ключом либо `None`; обращение освежает давность."""

        found = self._production_result_cache.get(key)
        if found is not None:
            self._production_result_cache.move_to_end(key)
        return found

    def remember_production_result(self, key: tuple, result) -> None:
        cache = self._production_result_cache
        cache[key] = result
        cache.move_to_end(key)
        while len(cache) > PRODUCTION_RESULT_CACHE_LIMIT:
            cache.popitem(last=False)

    @property
    def production_result_count(self) -> int:
        return len(self._production_result_cache)

    def remember_queue_session(self, state: QueueSessionStateV1) -> None:
        self._queue_session = state

    def invalidate_queue_session(self) -> None:
        """Сбрасывает только warm redraw, сохраняя правильно ключённые кэши."""

        self._queue_session = None

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
        density,
        workers: int = 0,
    ):
        from .envelope_domain_pool import get_domain_pool
        from .envelope_queue_export import (
            ENVELOPE_DEBUG_ENGINE_QUEUE,
            evaluate_envelope_queue_staged,
        )
        from .envelope_request_export import (
            evaluate_envelope_debug_staged as _evaluate,
        )
        from .envelope_worker_python import read_worker_python

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

        hooks = _WorkerExportHooks(self, topology_export, profile)
        snapshot_provider = hooks.snapshot_provider

        if str(engine) != ENVELOPE_DEBUG_ENGINE_QUEUE:
            return _evaluate(
                analysis_bundle,
                selected_physical_edge_ids,
                alpha,
                profile=profile,
                topology_export=topology_export,
                domain_snapshot_provider=snapshot_provider,
                density=density,
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
                request,
                lambda: prepare_conveyor(snapshot, request),
                profile=profile,
            )

        def cached_preparation(domain_id, selected_edges, request):
            return self.peek_conveyor_preparation(
                revision, domain_id, selected_edges, request
            )

        def preparation_adopter(
            _patch_id,
            domain_id,
            selected_edges,
            request,
            prepared,
        ):
            # Подготовка воркера входит в кэш тем же путём, что и собранная
            # здесь: промах со сборкой, счётчики и ключ те же.
            return self.get_conveyor_preparation(
                revision,
                domain_id,
                selected_edges,
                request,
                lambda: prepared,
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
            domain_pool=get_domain_pool(workers, read_worker_python()),
            cached_preparation=cached_preparation,
            preparation_blobs=self.preparation_blobs,
            preparation_adopter=preparation_adopter,
            export_provider=hooks.export_provider,
            export_adopter=hooks.export_adopter,
            density=density,
        )


def remember_queue_session(
    controller: EnvelopeDebugSessionController,
    source_name: str,
    topology_scene,
    exact_scenes,
    evaluation,
    *,
    density,
):
    """Запомнить подготовки очереди, чтобы ползунок нашёл их тёплыми.

    `None` — очередь не считалась (движок LEGACY либо ни один домен до
    подготовки не дошёл), и рисовать её слои нечем.
    """

    from .envelope_queue_export import build_queue_scene
    from .envelope_request_policy import normalize_envelope_fan_density

    queue_domains = evaluation.queue_domains
    if not queue_domains:
        return None
    controller.remember_queue_session(
        QueueSessionStateV1(
            str(source_name),
            topology_scene,
            tuple(exact_scenes),
            tuple(
                (item.patch_id, item.patch_domain_id, item.preparation)
                for item in queue_domains
                if item.preparation is not None
            ),
            evaluation.receipts,
            normalize_envelope_fan_density(density),
        )
    )
    return build_queue_scene(queue_domains)


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
    density=None,
    workers: int = 0,
):
    """Compatibility entry point with optional persistent session reuse.

    `workers` (>= 2) включает пул доменов движка QUEUE. Он работает только с
    сессией: кэш подготовок, который пул обходит и пополняет, живёт в
    контроллере, поэтому без `controller` параметр ничего не меняет.
    """

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
            density=density,
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
        density=density,
        workers=workers,
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
    from .envelope_production_operator import register_production_operator
    from .envelope_worker_python import install_worker_python_preference

    install_worker_python_preference()
    register_production_operator()
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
    # Воркеры пула — подпроцессы: снятие аддона обязано их остановить. Первым
    # делом и до `import bpy`: `operators.unregister` зовёт это же ПЕРЕД чисткой
    # GP, а здесь — ещё раз, идемпотентно, если вызвали только эту функцию.
    shutdown_domain_pool()
    import bpy
    from .envelope_production_operator import unregister_production_operator

    unregister_production_operator()
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
    "remember_queue_session",
    "shutdown_domain_pool",
    "unregister_window_manager_session_attribute",
)
