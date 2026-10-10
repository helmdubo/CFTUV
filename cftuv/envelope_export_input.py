"""Выгрузка снапшота домена В ВОРКЕРЕ пула: picklable вход и его исполнение.

Фаза A пула считала `(snapshot, request)` всех доменов в родителе, по одному:
на `building` это 6 с из 34 с холодной кнопки (кадр метрики, угловые
отношения, проверка снапшота на каноническом JSON). Отдать её воркеру «как
есть» нельзя: выгрузка читает `AnalysisBundle`, а тот держит патчи с
`mathutils.Vector`, которые воркеру (обычному интерпретатору) не доехать.

Выгрузка при этом читает из патча совсем немногое, и всё это — числа и кортежи:
срез поверхности (`surface_ir`, координаты — `tuple[float, float, float]`),
цепочки хоста (`_HostChainRecord` уже неизменяемые), а из `PatchNode` — тип
патча, вид петель и углы (`prev_chain_index`, `next_chain_index`, `vert_index`)
и число цепочек петли. Этим и исчерпывается «лёгкий» вход: `HostExportInputV1`.

ТОТ ЖЕ КОД, А НЕ КОПИЯ. Воркер собирает настоящий `EnvelopeTopologyExportV1` на
лёгком пакете и зовёт ту же `build_envelope_patch_metric_export`, что и родитель:
она берёт вид патча (`AnalysisBundleIdView`) и зовёт `build_envelope_analysis_
snapshot`. Второй способ выгрузить снапшот разошёлся бы с первым молча. Равенство
снапшота родителя и воркера побитово (канонические байты) закрыто тестами.

Отказ именован. Отказ выгрузки (`EnvelopeHostAdapterError`) воркер возвращает
записью `ExportRefusalV1`, и родитель поднимает ТОТ ЖЕ отказ через кэш метрики
сессии (`_CachedMetricFailure`), как поднял бы сам. Любое другое исключение —
задача упала, и домен выгружается в родителе прежним путём.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass, replace
from fractions import Fraction

from .envelope_host_labels import record_host_tokens
from .envelope_metric_export import EnvelopePatchMetricExportV1
from .envelope_request_policy import topology_chart_reach_cap
from .envelope_seam_neighbours import narrowed_surface
from .envelope_topology_export import (
    EnvelopeTopologyExportV1,
    build_analysis_bundle_id_view,
)


@dataclass(frozen=True, slots=True)
class _LightCornerV1:
    prev_chain_index: int
    next_chain_index: int
    vert_index: int


@dataclass(frozen=True, slots=True)
class _LightLoopV1:
    kind: object
    corners: tuple
    #: Выгрузка ходит по цепочкам через `host_chains`, а у петли читает только
    #: их число, поэтому вместо цепочек здесь пустышки той же длины.
    chains: tuple


@dataclass(frozen=True, slots=True)
class _LightPatchV1:
    patch_type: object
    boundary_loops: tuple
    #: У `PatchNode` поля нет (выгрузка объявляет `MIX`); поле заведено, чтобы
    #: `getattr(patch, "shape_class", None)` ответил тем же, что и у оригинала.
    shape_class: object | None = None


@dataclass(frozen=True, slots=True)
class _LightGraphV1:
    source_revision: object
    nodes: dict
    #: Рёбра графа выгрузка снапшота не читает.
    edges: dict


@dataclass(frozen=True, slots=True)
class _LightBundleV1:
    source_revision: object
    patch_graph: _LightGraphV1
    patch_surface: object
    capabilities: object


@dataclass(frozen=True, slots=True)
class HostExportInputV1:
    """Всё, что нужно воркеру, чтобы выгрузить `(snapshot, request)` домена."""

    source_revision_value: str
    bundle: _LightBundleV1
    host_chains: tuple
    alpha: float
    request_id: str
    density: object
    #: Допуск растяжения запроса (`None`: умолчание ядра): метрика домена записывается под ним.
    developable_stretch_budget: Fraction | None = None
    #: Досягаемость полосовой карты запроса (`None`: умолчание ядра): воркер собирает запрос, и его идентичность
    #: обязана совпасть с запросом родителя. Саму полосу воркер не строит: отказ целого патча родитель разрешает полосой.
    chart_reach_cap: Fraction | None = None
    #: Допуск UV закона силуэта запроса (`None`: умолчание ядра): идентичность запроса воркера обязана совпасть с родительской.
    silhouette_uv_slide: Fraction | None = None


@dataclass(frozen=True, slots=True)
class ExportRefusalV1:
    """Отказ выгрузки, пересланный значением: исход, текст и домен."""

    outcome: object
    message: str
    patch_domain_id: str | None


def _light_patch(node) -> _LightPatchV1:
    shape = getattr(node, "shape_class", None)
    return _LightPatchV1(
        node.patch_type,
        tuple(
            _LightLoopV1(
                loop.kind,
                tuple(
                    _LightCornerV1(
                        int(corner.prev_chain_index),
                        int(corner.next_chain_index),
                        int(corner.vert_index),
                    )
                    for corner in loop.corners
                ),
                (None,) * len(loop.chains),
            )
            for loop in node.boundary_loops
        ),
        None
        if shape is None
        else (shape.value if hasattr(shape, "value") else str(shape)),
    )


def build_host_export_input(
    topology_export: EnvelopeTopologyExportV1,
    patch_id: int,
    *,
    alpha,
    request_id: str,
    density,
) -> HostExportInputV1:
    """Лёгкий вход одного патча из выгрузки топологии родителя."""

    patch_id = int(patch_id)
    view = build_analysis_bundle_id_view(
        topology_export.analysis_bundle, frozenset({patch_id})
    )
    graph = view.patch_graph
    chains = topology_export.patch_chains(patch_id)
    bundle = _LightBundleV1(
        view.source_revision,
        _LightGraphV1(
            view.source_revision,
            {
                int(key): _light_patch(node)
                for key, node in graph.nodes.items()
            },
            {},
        ),
        narrowed_surface(view.patch_surface, chains),
        view.capabilities,
    )
    return HostExportInputV1(
        topology_export.source_revision_value,
        bundle,
        chains,
        alpha,
        request_id,
        density,
        topology_export.developable_stretch_budget,
        topology_chart_reach_cap(topology_export),
        topology_export.silhouette_uv_slide,
    )


def patch_metric_from_worker(
    topology_export: EnvelopeTopologyExportV1, patch_id: int, result
) -> EnvelopePatchMetricExportV1:
    """Метрика патча из ответа воркера — либо тот же отказ, что дал бы родитель.

    Вид патча строится в родителе (он лежит в записи кэша сессии), а снапшот —
    тот, что выгрузил воркер.
    """

    from .envelope_request_export import EnvelopeHostAdapterError

    patch_id = int(patch_id)
    refusal = result.refusal
    if refusal is not None and result.snapshot is None:
        raise EnvelopeHostAdapterError(
            refusal.outcome,
            refusal.message,
            patch_domain_id=refusal.patch_domain_id,
        )
    return EnvelopePatchMetricExportV1(
        topology_export.source_revision_value,
        patch_id,
        topology_export.patch_domain_id_by_patch[patch_id],
        build_analysis_bundle_id_view(
            topology_export.analysis_bundle, frozenset({patch_id})
        ),
        result.snapshot,
        topology_export.developable_stretch_budget,
    )


def replay_export_records(profile, result) -> None:
    """Тайминги и счётчики стадий выгрузки воркера — в профиль кнопки.

    Секунды стадий при пуле — время воркеров, сложенное по доменам, как у
    `QUEUE_PREPARE`: настенное время фазы лежит в `QUEUE_POOL_WALL`.
    """

    if profile is None:
        return
    for timing in result.export_timings:
        profile.add_timing(
            timing.stage, timing.elapsed_seconds, timing.patch_domain_id
        )
    for counter in result.export_counters:
        profile.set_counter(counter.name, counter.value, counter.patch_domain_id)


def _refusal(error) -> ExportRefusalV1:
    return ExportRefusalV1(error.outcome, str(error), error.patch_domain_id)


@dataclass(frozen=True, slots=True)
class TaskInputsV1:
    """Что воркер получил на входе задачи: `(snapshot, request)` и запись стадий выгрузки.

    `profile` держит секунды и счётчики выгрузки, которую сделал ВОРКЕР, и любую стадию,
    которую воркер допишет сам (подготовка продуктового пути): `result` возвращает их
    ответом, а родитель проигрывает в профиль кнопки (`replay_export_records`).
    `exported` — снапшот выгрузил воркер: он едет обратно ответом, у родителя его нет.
    """

    snapshot: object
    request: object
    profile: object
    task_id: int
    exported: bool
    #: Записанные токены идентичностей хоста домена (`DomainLabelingV1`), когда снапшот и запрос выгрузил
    #: воркер: по ним результат переносится на другую ревизию. Нет выгрузки в воркере — нет и записи.
    labeling: object | None = None

    def result(self, **fields):
        from .envelope_domain_pool import DomainTaskResultV1

        recorded = self.profile.snapshot()
        return DomainTaskResultV1(
            self.task_id,
            export_timings=recorded.timings,
            export_counters=recorded.counters,
            **fields,
        )


def task_inputs(task):
    """Входы домена в воркере: `TaskInputsV1` либо готовый ответ-отказ выгрузки.

    Задача с выгрузкой в воркере (`task.export`) выгружает `(snapshot, request)` сама,
    тем же кодом, что и родитель; иначе оба пришли готовыми из родителя. Отказ выгрузки —
    запись `refusal` без `queue_domain`; отказ запроса — то же, но со снапшотом (родитель
    повторит сборку запроса и получит тот же отказ). Прочие исключения уходят наверх, в
    `solve_task`, и становятся ошибкой задачи.
    """

    from .envelope_debug_profile import EnvelopeDebugProfileBuilderV1
    from .envelope_metric_export import build_envelope_patch_metric_export
    from .envelope_request_export import (
        EnvelopeHostAdapterError,
        build_envelope_decal_request,
    )

    profile = EnvelopeDebugProfileBuilderV1("worker", "QUEUE")
    export = task.export
    if export is None:
        return TaskInputsV1(task.snapshot, task.request, profile, task.task_id, False)
    topology = EnvelopeTopologyExportV1(
        export.source_revision_value,
        export.bundle,
        export.host_chains,
        {int(task.patch_id): task.domain_id},
        export.developable_stretch_budget,
    )
    inputs = TaskInputsV1(None, None, profile, task.task_id, True)
    with record_host_tokens() as log:
        try:
            snapshot = build_envelope_patch_metric_export(
                topology, task.patch_id, profile=profile
            ).snapshot
        except EnvelopeHostAdapterError as error:
            return inputs.result(refusal=_refusal(error))
        try:
            request = build_envelope_decal_request(
                snapshot,
                task.selected_edges,
                export.alpha,
                decal_request_id_value=export.request_id,
                density=export.density,
                developable_stretch_budget=export.developable_stretch_budget,
                chart_reach_cap=export.chart_reach_cap,
                silhouette_uv_slide=export.silhouette_uv_slide,
            )
        except EnvelopeHostAdapterError as error:
            return inputs.result(snapshot=snapshot, refusal=_refusal(error))
    return replace(
        inputs,
        snapshot=snapshot,
        request=request,
        labeling=log.labeling(export.source_revision_value, export.request_id, task.patch_id),
    )


def solve_exported_task(task):
    """Домен целиком в воркере: выгрузка `(snapshot, request)`, затем очередь.

    Возвращает `DomainTaskResultV1` (отказы выгрузки — см. `task_inputs`).
    """

    from .envelope_queue_export import run_queue_domain

    inputs = task_inputs(task)
    if not isinstance(inputs, TaskInputsV1):
        return inputs
    prepared, domain = run_queue_domain(
        task.patch_id,
        task.domain_id,
        inputs.snapshot,
        inputs.request,
        task.alpha_text,
        selected_edges=task.selected_edges,
        profile=None,
        backend=task.backend,
        skeleton_backend=task.skeleton_backend,
    )
    return inputs.result(
        prepared=prepared,
        queue_domain=replace(domain, preparation=None),
        snapshot=inputs.snapshot,
    )


def load_export_modules() -> None:
    """Подгружает модули выгрузки воркера заранее, до его «готов».

    Первый домен воркера иначе платил бы импортом `envelope_request_export` и
    его соседей в секундах домена, то есть на критическом пути стены.
    `from . import x` здесь нельзя: под чужим именем пакета (`bl_ext.*`) он
    просит у импорта родителя, которого в воркере нет.
    """

    for name in (
        "envelope_angle_certificate",
        "envelope_metric_export",
        "envelope_request_export",
    ):
        importlib.import_module(f".{name}", __package__)
    # Первое же сложение выражений sympy (`Add.flatten`) лениво подгружает
    # `sympy.tensor.tensor` и ещё 16 модулей — ~0.35 с на процесс. Родитель
    # платил это один раз, воркер платил бы на первой задаче, то есть на
    # критическом пути стены (первые задачи — самые тяжёлые домены).
    sympy = importlib.import_module("sympy")
    sympy.Symbol("warm") + 1


__all__ = (
    "ExportRefusalV1",
    "HostExportInputV1",
    "TaskInputsV1",
    "build_host_export_input",
    "load_export_modules",
    "patch_metric_from_worker",
    "replay_export_records",
    "solve_exported_task",
    "task_inputs",
)
