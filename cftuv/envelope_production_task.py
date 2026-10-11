"""Воркер продуктового пути: домен на готовой подготовке (`produce_domain`), подготовка холодного домена и задачи пула.

Вынесено из `envelope_production_export.py` ПЕРЕНОСОМ без изменения поведения (PRODUCTION_EXPORT_SPLIT_V1). Здесь всё, что исполняется
в воркере пула и тем же кодом в родителе (малая партия, пул недоступен, задача упала): `produce_domain` (покрытие на подготовке и
материализация, общий код всех путей), `prepare_for_production_recorded` (подготовка под блоком бэкенда) и две задачи воркера
(`solve_production_task`, `solve_cold_production_task`). Сторона родителя (сканирование, досчёт, пересылка) остаётся в
`envelope_production_export` и `envelope_production_scan`; все имена по-прежнему доступны из `envelope_production_export`.
"""

from __future__ import annotations

import time
import traceback
from dataclasses import replace

from .envelope_content_store import ContentRelabelFailed, carried_to_run
from .envelope_kernel_backend import (
    DEFAULT_KERNEL_BACKEND,
    PreparationRefused,
    entered_backend,
    ledger_record,
    prepared_under_backend,
    with_kernel_backend,
    with_preparation_record,
)
from .envelope_production_result import (
    MATERIALIZED,
    PLACEMENT_WORKER,
    PRODUCTION_TOPOLOGY_LAW,
    PRODUCTION_TOPOLOGY_LAWS,
    PRODUCTION_UV_POLICY,
    ProductionDomainResultV1,
    _placed,
    _raised,
    _refusal,
    deferred_result,
)
from .envelope_request_policy import ENVELOPE_UV_POLICIES
from .envelope_snap_retry import snap_scale_notes
from .envelope_worker_store import prepared_of
from .surface_ir import HOST_NEAR_PLANAR_LIFT_POLICY

def _source_normal(prepared):
    """Нормаль первой (по имени) исходной грани патча-владельца либо `None`."""

    owner = prepared.compilation.owner_patch_id
    faces = sorted(
        (
            item
            for item in prepared.context.snapshot.surface_ir.source_faces
            if item.patch_id == owner
        ),
        key=lambda item: item.face_id.value,
    )
    normal = faces[0].polygon_normal if faces else None
    return None if normal is None else (normal.x, normal.y, normal.z)


def _summary_normal(vertex_normals):
    """Нормированное среднее нормалей вершин развёртки либо `None` (у плоского домена их нет)."""

    if not vertex_normals:
        return None
    total = [sum(item[1][axis] for item in vertex_normals) for axis in range(3)]
    length = sum(axis * axis for axis in total) ** 0.5
    if not length:
        return None
    from cftuv_envelope.numeric import LocalVector3V1

    return LocalVector3V1(*(axis / length for axis in total))


@with_kernel_backend
def produce_domain(
    patch_id: int,
    domain_id: str,
    prepared,
    alpha_text: str,
    *,
    uv_policy_id: str = PRODUCTION_UV_POLICY,
    topology_law: str = PRODUCTION_TOPOLOGY_LAW,
    defer: bool = False,
) -> ProductionDomainResultV1:
    """Покрытие на готовой подготовке и материализация: общий код всех путей.

    Его зовут воркер пула (`solve_production_task`) и родитель (малая партия,
    пул недоступен, задача упала, ноль воркеров) — размещение не меняет ответ,
    потому что код один. Память разложений сбрасывается перед доменом: статьи
    бюджета материализации не зависят от истории процесса (как у кнопки отладки).
    Исключение внутри домена не роняет пул и не теряется: оно называется.

    `defer=True` (так считает воркер) — отложенный результат (`deferred_result`): дайджесты не считаются (ядро `digests=False`),
    вид домена посчитан, батч упакован в байты. Прочитанное из него равно результату `defer=False`.
    """

    if uv_policy_id not in ENVELOPE_UV_POLICIES:
        raise ValueError(f"unknown UV policy {uv_policy_id!r}")
    if topology_law not in PRODUCTION_TOPOLOGY_LAWS:
        raise ValueError(f"unknown decal topology law {topology_law!r}")
    started = time.perf_counter()
    try:
        from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
        from cftuv_envelope.exact_sqrt_sum import (
            reset_factorization_memory,
            reset_unbudgeted_work,
        )
        from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
        from cftuv_envelope.materialize.admit import (
            admit_domain,
            materialization_request,
        )
        from cftuv_envelope.materialize.lift import plane_normal_binary64
        from cftuv_envelope.materialize.step import step_domain

        reset_factorization_memory()
        reset_unbudgeted_work()
        if prepared.outcome.value != "EXACT":
            # Ядро называет это само: подготовка без компиляции запроса не
            # имеет, и отказ приходит из `admit` до любой работы.
            refused = admit_domain(prepared, None, None)
            return _refusal(
                patch_id,
                domain_id,
                refused.outcome.value,
                refused.detail,
                time.perf_counter() - started,
            )
        # Покрытие из шаблона внутри заверенного интервала ширины, иначе полный путь с записью шаблона; ответ в обоих случаях один.
        stepped = step_domain(
            prepared,
            alpha_text,
            request=materialization_request(prepared, uv_policy_id=uv_policy_id),
            near_planar_lift_law=NearPlanarLiftLawV1(HOST_NEAR_PLANAR_LIFT_POLICY.value),
            decal_topology_law=DecalTopologyLawV1(topology_law),
            digests=not defer,
        )
        result = stepped.result
        if not result.is_materialized:
            return _refusal(
                patch_id,
                domain_id,
                result.outcome.value,
                result.detail,
                time.perf_counter() - started,
            )
        normal = _summary_normal(result.vertex_normals) or plane_normal_binary64(prepared.context.frame)
        produced = ProductionDomainResultV1(
            patch_id=int(patch_id),
            domain_id=str(domain_id),
            outcome=MATERIALIZED,
            batch=result.batch,
            detail="",
            counters=tuple((str(name), value) for name, value in result.counters),
            normal=(normal.x, normal.y, normal.z),
            content_digest=result.content_digest,
            diagnostics=(*result.diagnostics, *snap_scale_notes(prepared)),
            source_normal=_source_normal(prepared),
            chart_orientation=str(prepared.context.frame.chart_orientation.value),
            vertex_normals=tuple(result.vertex_normals),
            offset_normal_law=result.offset_normal_law,
            offset_normals_digest=result.offset_normals_digest,
            decal_topology_law=result.decal_topology_law.value,
            seconds=time.perf_counter() - started,
            clip_memo=result.clip_memo,
            alpha_interval=result.interval,
            structure_digest="" if result.structure is None else result.structure.digest,
            step_path=stepped.path,
        )
        if defer:
            produced = deferred_result(produced).with_changes(seconds=time.perf_counter() - started)
        return produced
    except Exception as exc:  # noqa: BLE001 - исход называется, а не теряется
        return _raised(exc, patch_id, domain_id, time.perf_counter() - started)


def prepare_for_production_recorded(
    snapshot,
    request,
    *,
    backend: str = DEFAULT_KERNEL_BACKEND,
    skeleton_backend: str | None = None,
    embedding_backend: str | None = None,
    borrowed_ledger=None,
    checker=None,
):
    """`(подготовка, запись бэкенда | None)`: подготовка очереди с холодной памятью разложений под блоком бэкенда; `checker` - `WorkerSnapshotCheck` задачи воркера (ту же проверку снапшота отдаёт он, `clean_issues_for`).

    Память разложений и счётчик внебюджетной работы обнуляются ПЕРЕД подготовкой
    (`run_queue_domain` делает ровно это): статьи бюджета не зависят от того, какие
    домены процесс уже видел, а в пуле — от того, как задачи легли на воркеры.

    Скелет считается ЗДЕСЬ (`prepare_conveyor` -> `_prepare_region` -> `backend.skeleton_compute`), а не в `produce_domain`, поэтому блок бэкенда стоит
    вокруг подготовки (`prepared_under_backend`) и в родителе, и в воркере пула; запись подготовки (скелет отдельно) идёт к записи материализации
    (`with_preparation_record`). Обе стадии `PYTHON` — записи нет, путь побитово равен вызову без блока. Домен, которому нативное ядро отказало по имени, отказан
    `PreparationRefused`: подготовки у него нет.
    """

    from cftuv_envelope.exact_sqrt_sum import (
        reset_factorization_memory,
        reset_unbudgeted_work,
    )
    from cftuv_envelope.wavefront import prepare_conveyor

    reset_factorization_memory()
    reset_unbudgeted_work()
    return prepared_under_backend(lambda: prepare_conveyor(snapshot, request, snapshot_issues=checker and checker.clean_issues_for(snapshot, request.developable_stretch_budget)), backend, skeleton_backend, embedding_backend, borrowed_ledger=borrowed_ledger)


def prepare_for_production(
    snapshot,
    request,
    *,
    backend: str = DEFAULT_KERNEL_BACKEND,
    skeleton_backend: str | None = None,
    embedding_backend: str | None = None,
):
    """Подготовка очереди с холодной памятью разложений (то, что делала кнопка отладки) без записи бэкенда: `prepare_for_production_recorded(...)[0]`."""

    return prepare_for_production_recorded(snapshot, request, backend=backend, skeleton_backend=skeleton_backend, embedding_backend=embedding_backend)[0]


def solve_cold_production_task(task):
    """Один журнал экспорта и подготовки, включая ранний отказ; вложенная подготовка его заимствует."""

    from .envelope_domain_pool import DomainTaskResultV1

    with entered_backend(task.backend, task.skeleton_backend, task.embedding_backend) as ledger:
        try:
            reply = _solve_cold_production_task(task, ledger)
        except Exception:  # тот же ответ ошибки пула, но запись экспорта не теряется
            reply = DomainTaskResultV1(task.task_id, error=traceback.format_exc())
    record = ledger_record(ledger)
    if reply.production is not None:
        return replace(reply, production=with_preparation_record(reply.production, record))
    return replace(reply, backend_record=record)


def _solve_cold_production_task(task, ledger):
    """Воркер пула: холодный домен целиком — выгрузка (если нужна), подготовка, материализация.

    Подготовка и материализация идут подряд тем же кодом, что и порознь (`prepare_for_production`
    и `produce_domain`, который сам обнуляет память перед покрытием), поэтому ответ тот же, что
    у домена на подготовке из кэша. Ответ несёт и подготовку: у родителя её ещё нет, и в кэш
    сессии она входит так же, как подготовка отладочного вычислителя. Отказ выгрузки приходит
    ответом-отказом (`refusal`) и разбирается родителем, как разобрал бы он сам.
    """

    from .envelope_export_input import TaskInputsV1, task_inputs

    inputs = task_inputs(task)
    if not isinstance(inputs, TaskInputsV1):
        return inputs
    started = time.perf_counter()
    try:
        prepared, preparation_record = prepare_for_production_recorded(
            inputs.snapshot,
            inputs.request,
            backend=task.backend,
            skeleton_backend=task.skeleton_backend,
            embedding_backend=task.embedding_backend,
            borrowed_ledger=ledger,
            checker=inputs.checker,
        )
    except PreparationRefused as refused:
        # Нативное ядро отказало скелету по имени: домен отказан этим именем (эталону он не отдаётся), подготовки у него нет.
        prepare_seconds = time.perf_counter() - started
        inputs.profile.add_timing("QUEUE_PREPARE", prepare_seconds, task.domain_id)
        return inputs.result(
            prepared=None,
            production=_placed(
                _refusal(task.patch_id, task.domain_id, refused.outcome, refused.detail, prepare_seconds).with_changes(
                    backend_record=refused.record, labels=inputs.labeling
                ),
                PLACEMENT_WORKER,
            ),
            snapshot=inputs.snapshot if inputs.exported else None,
        )
    prepare_seconds = time.perf_counter() - started
    inputs.profile.add_timing("QUEUE_PREPARE", prepare_seconds, task.domain_id)
    produced = produce_domain(
        task.patch_id,
        task.domain_id,
        prepared,
        task.alpha_text,
        uv_policy_id=task.cold.uv_policy_id,
        topology_law=task.cold.topology_law,
        defer=True,
        backend=task.backend,
        skeleton_backend=task.skeleton_backend,
        embedding_backend=task.embedding_backend,
    )
    return inputs.result(
        prepared=prepared,
        production=_placed(
            with_preparation_record(produced, preparation_record).with_changes(
                seconds=produced.seconds + prepare_seconds,
                labels=inputs.labeling,
            ),
            PLACEMENT_WORKER,
        ),
        snapshot=inputs.snapshot if inputs.exported else None,
    )


def solve_production_task(task):
    """Воркер пула: домен продуктового пути на присланной подготовке.

    Ответ — `DomainTaskResultV1` с полем `production`; подготовку обратно не
    шлют, она у родителя уже есть.
    """

    from .envelope_domain_pool import DomainTaskResultV1

    production = task.production
    payload = prepared_of(production, retain=not production.carried)
    result = (
        payload
        if production.carried
        else produce_domain(
            task.patch_id,
            _domain_of_preparation(task.domain_id, production.relabel),
            payload,
            task.alpha_text,
            uv_policy_id=production.uv_policy_id,
            topology_law=production.topology_law,
            defer=True,
            backend=task.backend,
            skeleton_backend=task.skeleton_backend,
            embedding_backend=task.embedding_backend,
        )
    )
    if production.relabel is not None:
        try:
            result = carried_to_run(result, production.relabel)
        except ContentRelabelFailed:
            # Родитель повторит перенос, получит тот же отказ, назовёт его и посчитает домен заново.
            pass
    return DomainTaskResultV1(
        task.task_id,
        production=_placed(deferred_result(result), PLACEMENT_WORKER),
    )


def _domain_of_preparation(domain_id: str, relabel) -> str:
    """Домен, на подготовке которого считают: подготовка из хранилища несёт идентичности СВОЕЙ ревизии.

    Результат получает id домена той ревизии, при которой подготовка посчитана, и переносится целиком
    (`relabel_result`); id домена текущей ревизии, подставленный в него до переноса, был бы единственной
    строкой результата, которой нет в записи подготовки.
    """

    if relabel is None or relabel.base is None:
        return domain_id
    return relabel.base.domain_id()
