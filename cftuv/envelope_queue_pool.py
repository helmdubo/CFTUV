"""Склейка пула доменов с движком QUEUE: входы, диспетчеризация, приём ответов.

Сам пул (`envelope_domain_pool`) ничего не знает об очереди: он пересылает
`DomainTaskV1` и возвращает ответ. Здесь — то, что знает очередь:

- фаза A: входы `(snapshot, request)` доменов, чья метрика уже в кэше сессии,
  берутся в родителе (кэш-попадание стоит доли миллисекунды); у остальных
  выгрузку считает ВОРКЕР (`envelope_export_input`: Blender-объекты воркеру не
  пересылаются, уходит лёгкий вход), и родитель принимает её ответом;
- фаза B: домены, подготовки которых нет в кэше сессии, уходят воркерам разом
  целиком, а домены с ГОТОВОЙ подготовкой в кэше — только покрытием: воркеру
  шлют пикл подготовки (один раз на подготовку, `PreparationBlobsV1`), он
  считает покрытие и запись хоста и возвращает запись БЕЗ подготовки, которую
  родитель добавляет из кэша сам. Счётчики пула и стадия `QUEUE_POOL_WALL`
  пишутся в профиль;
- фаза C (цикл `evaluate_envelope_queue_staged`): ответ воркера принимается как
  результат домена, а домен, которого воркер не дал, досчитывается прежним
  последовательным путём;
- ползунок alpha (`SliderCoveragePool`): то же покрытие на готовых подготовках,
  без выгрузки и без новых подготовок.

ПОКРЫТИЕ В ВОРКЕРЕ — ТОТ ЖЕ КОД (`cover_prepared`), что и в родителе, а размещение
не меняет ответ: пикл подготовки хранит её без памяти (`_DensityExactMemo.
__reduce__`), а покрытие пересланной подготовки при другой alpha побитово то же
(`test_conveyor_preparation_pickle`). Партия, которая стоит меньше пересылки,
остаётся в родителе (`COVERAGE_POOL_MIN_BYTES`): решает размер пиклов, потому что
стоимость покрытия растёт с ним (замер на `building`, 121 домен).

Отказ именован на обоих уровнях: пул, который не стартовал, — счётчик
`ENVELOPE_DOMAIN_POOL_UNAVAILABLE`, строка консоли и диагностика на первом
домене; задача, которая упала либо чей воркер умер, — счётчик
`ENVELOPE_DOMAIN_POOL_TASK_FALLBACK`, строка консоли и диагностика на ЭТОМ
домене. Ни то ни другое не меняет ответ: домен считается тем же кодом.

Внешний интерпретатор воркеров («Worker Python»), который не прошёл сверку с
родителем, называется третьим исходом: `ENVELOPE_DOMAIN_POOL_INTERPRETER_UNUSABLE`
либо `..._INTERPRETER_MISMATCH` — диагностика на первом домене, строка консоли,
счётчик отката и причина кодом в профиле (хвост панели). Пул при этом работает:
воркеры идут на встроенном интерпретаторе.
"""

from __future__ import annotations

import pickle
import time
from dataclasses import dataclass, replace

from .envelope_lazy_preparation import LazyPreparationV1, canonical
from .envelope_worker_store import prepared_of
from .envelope_queue_export import (
    POOL_COVERAGE_DISPATCHED,
    POOL_DISPATCHED,
    POOL_EXTERNAL_PYTHON,
    POOL_INTERPRETER_FALLBACK,
    POOL_INTERPRETER_REASON,
    POOL_PYTHON_VERSION,
    POOL_TASK_FALLBACK,
    POOL_UNAVAILABLE,
    POOL_WALL_STAGE,
    POOL_WORKERS,
    CoverageCancelled,
    _measure,
    _queue_snapshot_and_request,
    cover_prepared,
)

PICKLE_PROTOCOL = 5

#: Партия покрытий, чьи пиклы весят меньше, считается в родителе: пересылка,
#: распаковка и ответ стоят дороже самого покрытия. Замер на малых доменах
#: `building` (`artifacts/warm_coverage_parallel/threshold_probe.py`, 8 тёплых
#: воркеров): пул ~12 мс постоянных против последовательного пути — 1–2 малых
#: домена (27–55 КБ) 13 против 10–13 мс, партия 110 КБ уже 14 против 19 мс, 442 КБ
#: 21 против 57 мс. 128 КБ — с запасом над точкой окупаемости.
COVERAGE_POOL_MIN_BYTES = 128 * 1024


@dataclass(frozen=True, slots=True)
class CoverageInputV1:
    """Вход задачи «только покрытие»: пикл подготовки и режим памяти воркера.

    `reset_memory` — кнопка (`run_queue_domain` сбрасывает память разложений
    перед каждым доменом, чтобы статьи бюджета не зависели от истории
    процесса) либо ползунок (`recompute_queue_coverage` её не сбрасывает).
    """

    blob: bytes | None
    reset_memory: bool
    #: Ключ пикла в памяти подготовок воркера (`envelope_worker_store.blob_key`); пусто - подготовка не запоминается.
    key: str = ""


class PreparationBlobsV1:
    """Пикл подготовок для воркеров: один раз на подготовку.

    Ключ — идентичность объекта подготовки, а запись держит и сам объект:
    живой объект не отдаст свой `id` другому. Время жизни — как у кэша
    подготовок сессии (чистится вместе с ним), а размер — ~6 МБ на 121 домен.
    Пикл снимает состояние подготовки в момент первой отправки, и бюджет точной
    работы в нём — как на подготовке, а не накопленный покрытиями ползунка.
    """

    def __init__(self) -> None:
        #: По тождеству объекта: `[объект, пикл, ключ пикла в памяти воркера либо None, пока не считали]`.
        self._items: dict[int, list] = {}

    def __len__(self) -> int:
        return len(self._items)

    def _item_of(self, prepared) -> list:
        prepared = canonical(prepared)
        item = self._items.get(id(prepared))
        if item is None or item[0] is not prepared:
            if type(prepared) is LazyPreparationV1:
                item = [prepared, prepared.blob, prepared.key or None]  # пикл у ручки уже есть: перепикливать нечего
            else:
                item = [prepared, pickle.dumps(prepared, protocol=PICKLE_PROTOCOL), None]
            self._items[id(prepared)] = item
        return item

    def blob_of(self, prepared) -> bytes:
        return self._item_of(prepared)[1]

    def adopt(self, prepared, blob: bytes, key: str) -> None:
        """Пикл подготовки, который снял воркер и из которого получена `prepared` (ручка либо развёрнутый объект): перепикливать её родителю незачем."""

        prepared = canonical(prepared)
        self._items[id(prepared)] = [prepared, blob, key or None]

    def key_of(self, prepared) -> str:
        """Ключ пикла этой подготовки в памяти воркеров (хеш байтов и кода, считается один раз); пусто - без памяти."""

        item = self._item_of(prepared)
        if item[2] is None:
            from .envelope_worker_store import blob_key

            item[2] = blob_key(item[1])
        return item[2]

    def clear(self) -> None:
        self._items.clear()

    def discard(self, prepared) -> None:
        """Пикл одной подготовки (она ушла из хранилища по содержимому)."""

        prepared = canonical(prepared)
        item = self._items.get(id(prepared))
        if item is not None and item[0] is prepared:
            del self._items[id(prepared)]

    def retain(self, keep) -> None:
        """Оставляет пиклы подготовок, для которых `keep(подготовка)` истинно: остальные никто не держит."""

        for key, (prepared, _blob, _key) in list(self._items.items()):
            if not keep(prepared):
                del self._items[key]


def solve_coverage_task(task):
    """Воркер: покрытие и запись хоста на присланной подготовке.

    Ответ — `DomainTaskResultV1` с `prepared=None`: подготовка у родителя уже
    есть, и пересылать её обратно значило бы платить за неё дважды.
    """

    from .envelope_domain_pool import DomainTaskResultV1

    prepared = prepared_of(task.coverage)
    domain = cover_prepared(
        task.patch_id,
        task.domain_id,
        prepared,
        task.alpha_text,
        reset_memory=task.coverage.reset_memory,
    )
    return DomainTaskResultV1(task.task_id, None, replace(domain, preparation=None))


@dataclass(frozen=True, slots=True)
class StagedQueueDomainV1:
    """Домен, вход которого посчитан ЗАРАНЕЕ, и то, что вернул воркер пула.

    `inputs` — `(snapshot, request)` либо `EnvelopeHostAdapterError`: отказ
    выгрузки хранится значением и разбирается тем же обработчиком, что и в
    последовательном пути, а не пересчитывается. Повторный вызов кэша сессии
    дал бы `_CACHE_HIT` вместо `_CACHE_MISS`, и счётчики разошлись бы с
    последовательным прогоном.

    `pooled` — ответ воркера (`DomainTaskResultV1`) либо `None`: домен в кэше,
    пул недоступен или задача упала, и домен считается прежним путём.
    `pool_outcome` и `pool_message` — именованная причина такого «либо»; пусто,
    когда всё шло как задумано. `interpreter_outcome` и `interpreter_message` —
    отдельная запись о внешнем интерпретаторе воркеров, отвергнутом на этом
    прогоне: она стоит на первом домене РЯДОМ с ответом воркера, а не вместо него.
    """

    inputs: object
    pooled: object | None = None
    pool_outcome: str = ""
    pool_message: str = ""
    interpreter_outcome: str = ""
    interpreter_message: str = ""


def _pool_reason(error: str) -> str:
    lines = [item for item in str(error).strip().splitlines() if item.strip()]
    return lines[-1].strip() if lines else "no reason recorded"


def stage_pool_domains(
    domain_pool,
    analysis_bundle,
    patch_ids,
    revision,
    selected_edges_by_domain,
    alpha,
    alpha_text: str,
    request_id: str,
    *,
    density,
    profile,
    topology_export=None,
    domain_snapshot_provider=None,
    cached_preparation=None,
    preparation_blobs=None,
    export_provider=None,
    export_adopter=None,
):
    """Фазы A и B пула: входы доменов, затем воркеры на те, что не в кэше.

    Фаза C — прежний последовательный цикл, который берёт отсюда готовые входы
    и ответы.

    `export_provider(patch_id, alpha, request_id, density)` отвечает лёгким
    входом выгрузки (`HostExportInputV1`) для домена, метрики которого нет в
    кэше сессии, и `None` для остальных: у тех выгрузка — попадание в кэш, и
    идёт здесь, в родителе (`_queue_snapshot_and_request`). Без провайдера
    (прогон без сессии) выгружается всё в родителе, как и прежде.
    `export_adopter(patch_id, result)` принимает ответ воркера в кэш сессии.

    `cached_preparation(domain_id, selected_edges, request)` отдаёт подготовку
    из кэша сессии либо `None`, и счётчиков не пишет: это вопрос, а попадание
    запишет сам приём ответа. Домен с подготовкой в кэше воркеру уходит ТОЛЬКО
    покрытием (`preparation_blobs` даёт пикл подготовки); без `preparation_blobs`
    он считается в родителе, как и прежде.
    """

    from .envelope_domain_pool import DomainTaskV1
    from .envelope_request_export import EnvelopeHostAdapterError, _typed_value

    def inputs_of(patch_id, domain_id, selected):
        try:
            return _queue_snapshot_and_request(
                analysis_bundle,
                patch_id,
                domain_id,
                selected,
                alpha,
                request_id,
                density=density,
                profile=profile,
                topology_export=topology_export,
                domain_snapshot_provider=domain_snapshot_provider,
            )
        except EnvelopeHostAdapterError as exc:
            return exc

    staged: dict[str, StagedQueueDomainV1] = {}
    tasks = []
    covered = []
    for patch_id in patch_ids:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        selected = frozenset(selected_edges_by_domain[domain_id])
        export = (
            None
            if export_provider is None
            else export_provider(patch_id, alpha, request_id, density)
        )
        if export is not None:
            tasks.append(
                DomainTaskV1(
                    len(tasks),
                    patch_id,
                    domain_id,
                    None,
                    None,
                    alpha_text,
                    selected,
                    export,
                )
            )
            continue
        inputs = inputs_of(patch_id, domain_id, selected)
        staged[domain_id] = StagedQueueDomainV1(inputs)
        if isinstance(inputs, EnvelopeHostAdapterError):
            continue
        snapshot, request = inputs
        prepared = (
            None
            if cached_preparation is None
            else cached_preparation(domain_id, selected, request)
        )
        if prepared is not None:
            covered.append((patch_id, domain_id, selected, prepared))
            continue
        tasks.append(
            DomainTaskV1(
                len(tasks),
                patch_id,
                domain_id,
                snapshot,
                request,
                alpha_text,
                selected,
            )
        )
    coverage, shipping_failures = _coverage_tasks(
        covered, preparation_blobs, alpha_text, len(tasks)
    )
    tasks.extend(coverage)
    return _dispatch_to_pool(
        domain_pool,
        tasks,
        staged,
        profile,
        inputs_of,
        export_adopter,
        shipping_failures,
    )


def _coverage_tasks(covered, blobs, alpha_text, first_task_id):
    """Задачи «только покрытие» для доменов с подготовкой в кэше.

    `([задачи], {domain_id: причина})`: партия, которая стоит меньше пересылки
    (`COVERAGE_POOL_MIN_BYTES`), и пул без пиклов дают пустой список, и домены
    считаются в родителе, как до пула.
    """

    from .envelope_domain_pool import DomainTaskV1

    if blobs is None or not covered:
        return [], {}
    shipped, failures = _ship_preparations(
        covered, blobs, lambda item: item[3]
    )
    if not _worth_the_pool(shipped):
        return [], failures
    return [
        DomainTaskV1(
            first_task_id + index,
            patch_id,
            domain_id,
            None,
            None,
            alpha_text,
            selected,
            coverage=CoverageInputV1(blob, True, blobs.key_of(prepared)),
        )
        for index, ((patch_id, domain_id, selected, prepared), blob) in enumerate(
            shipped
        )
    ], failures


def _ship_preparations(items, blobs, prepared_of):
    """Пиклы подготовок: `[(item, blob)]` и `{domain_id: причина}` для неудачных.

    Подготовка, которую нельзя запикливать, остаётся в родителе и называется
    отказом задачи, а не пропадает из пула без следа.
    """

    shipped = []
    failures: dict[str, str] = {}
    for item in items:
        try:
            shipped.append((item, blobs.blob_of(prepared_of(item))))
        except Exception as exc:  # noqa: BLE001 - причина называется, а не теряется
            failures[item[1]] = (
                f"preparation cannot be shipped: {type(exc).__name__}: {exc}"
            )
    return shipped, failures


def _worth_the_pool(shipped) -> bool:
    return sum(len(blob) for _, blob in shipped) >= COVERAGE_POOL_MIN_BYTES


def _run_tasks(domain_pool, tasks, profile, cancel=None):
    """Задачи в пул под стадией стены: `(run | None, причина отказа пула | "")`.

    `cancel` (`threading.Event`) уходит в пул только когда задан: подставные пулы без этого параметра
    остаются совместимы.
    """

    from .envelope_domain_pool import DomainPoolUnavailable

    if not tasks:
        return None, ""
    run = None
    failure = ""
    try:
        with _measure(profile, POOL_WALL_STAGE):
            run = (
                domain_pool.run(tasks)
                if cancel is None
                else domain_pool.run(tasks, cancel=cancel)
            )
    except DomainPoolUnavailable as exc:
        failure = str(exc)
    except Exception as exc:  # noqa: BLE001 - пул не должен ронять Build
        failure = f"pool failed: {type(exc).__name__}: {exc}"
    if failure:
        print(
            f"[CFTUV][EnvelopeDomainPool] {POOL_UNAVAILABLE}: {failure}; "
            "QUEUE runs sequentially",
            flush=True,
        )
    return run, failure


def _task_error(run, task) -> str:
    result = None if run is None else run.results.get(task.task_id)
    if result is None:
        return "task was not executed (every worker died)"
    return result.error or "worker refused a domain the host accepted"


def _note_task_fallback(domain_id: str, error: str) -> str:
    """Строка консоли и текст диагностики для задачи, которую считал не воркер."""

    print(
        f"[CFTUV][EnvelopeDomainPool] {POOL_TASK_FALLBACK} "
        f"{domain_id[-3:]}: {error}",
        flush=True,
    )
    return (
        f"Domain pool task failed: {_pool_reason(error)}; "
        "domain recomputed sequentially"
    )


def _record_pool_counters(
    profile, run, failure, dispatched, fallbacks, covered
) -> None:
    profile.set_counter(POOL_WORKERS, 0 if run is None else run.workers)
    profile.set_counter(POOL_DISPATCHED, dispatched)
    profile.set_counter(POOL_COVERAGE_DISPATCHED, covered)
    profile.set_counter(POOL_TASK_FALLBACK, fallbacks)
    profile.set_counter(POOL_UNAVAILABLE, int(bool(failure)))
    interpreter = None if run is None else getattr(run, "interpreter", None)
    profile.set_counter(
        POOL_PYTHON_VERSION, 0 if interpreter is None else interpreter.version_code
    )
    profile.set_counter(
        POOL_EXTERNAL_PYTHON, int(interpreter is not None and interpreter.external)
    )
    profile.set_counter(
        POOL_INTERPRETER_FALLBACK, int(bool(getattr(interpreter, "outcome", "")))
    )
    profile.set_counter(
        POOL_INTERPRETER_REASON, getattr(interpreter, "reason_code", 0)
    )


def _name_interpreter_fallback(run, tasks, staged) -> None:
    """Внешний интерпретатор отвергнут: консоль и первый домен называют причину."""

    from .envelope_request_export import EnvelopeHostAdapterError

    interpreter = None if run is None else getattr(run, "interpreter", None)
    if interpreter is None or not interpreter.outcome:
        return
    print(
        f"[CFTUV][EnvelopeDomainPool] {interpreter.outcome}: {interpreter.reason}; "
        f"workers run on the bundled Python {interpreter.version_text}",
        flush=True,
    )
    for task in tasks:
        entry = staged[task.domain_id]
        if not isinstance(entry.inputs, EnvelopeHostAdapterError):
            staged[task.domain_id] = replace(
                entry,
                interpreter_outcome=interpreter.outcome,
                interpreter_message=(
                    f"Worker Python rejected: {interpreter.reason}; "
                    f"workers run on the bundled Python {interpreter.version_text}"
                ),
            )
            return


def _dispatch_to_pool(
    domain_pool,
    tasks,
    staged,
    profile,
    inputs_of,
    export_adopter,
    shipping_failures=None,
):
    """Фаза B: задачи в пул и названный исход по каждой, счётчики пула.

    Задача с выгрузкой в воркере доводится до входов ЗДЕСЬ, после пула и в
    порядке доменов: принятый ответ кладётся в кэш сессии (`export_adopter`) и
    разбирается тем же `inputs_of`, что и родительская выгрузка, — поэтому
    кэш-счётчики, отказы метрики и отказы запроса те же. Домен, чья выгрузка
    отказала, воркеру «не уходил»: он не считается ни отправленным, ни упавшим.

    Подготовка, которую не удалось запикливать (`shipping_failures`), воркеру
    не уходила: она считается отказом задачи, но не отправленной.
    """

    from .envelope_request_export import EnvelopeHostAdapterError

    run, failure = _run_tasks(domain_pool, tasks, profile)
    dispatched = 0
    covered = 0
    fallbacks = 0
    unavailable_named = False
    for task in tasks:
        result = None if run is None else run.results.get(task.task_id)
        if task.export is not None:
            if result is not None and (result.ok or result.refused):
                export_adopter(task.patch_id, result)
            staged[task.domain_id] = StagedQueueDomainV1(
                inputs_of(task.patch_id, task.domain_id, task.selected_edges)
            )
        entry = staged[task.domain_id]
        if isinstance(entry.inputs, EnvelopeHostAdapterError):
            continue
        if failure:
            if not unavailable_named:
                unavailable_named = True
                staged[task.domain_id] = replace(
                    entry,
                    pool_outcome=POOL_UNAVAILABLE,
                    pool_message=(
                        f"Domain pool unavailable: {_pool_reason(failure)}; "
                        "QUEUE domains computed sequentially"
                    ),
                )
            continue
        dispatched += 1
        covered += int(task.coverage is not None)
        if result is not None and result.ok:
            staged[task.domain_id] = replace(entry, pooled=result)
            continue
        fallbacks += 1
        staged[task.domain_id] = replace(
            entry,
            pool_outcome=POOL_TASK_FALLBACK,
            pool_message=_note_task_fallback(
                task.domain_id, _task_error(run, task)
            ),
        )
    for domain_id, reason in (shipping_failures or {}).items():
        fallbacks += 1
        staged[domain_id] = replace(
            staged[domain_id],
            pool_outcome=POOL_TASK_FALLBACK,
            pool_message=_note_task_fallback(domain_id, reason),
        )
    _name_interpreter_fallback(run, tasks, staged)
    _record_pool_counters(profile, run, failure, dispatched, fallbacks, covered)
    return staged


class SliderCoveragePool:
    """Ползунок alpha: покрытие готовых подготовок воркерами, ответ тот же.

    Подготовки лежат в сессии; воркерам уходят их пиклы (`PreparationBlobsV1`:
    снимаются один раз, а не на каждом шаге ползунка), а возвращаются записи
    домена без подготовки. Результат — `{patch_domain_id: запись}` для тех
    доменов, что посчитал воркер; остальные (малая партия, пул не отдал ответ)
    остаются вызывающему, который считает их тем же кодом в родителе.
    Именованные исходы идут счётчиками в `profile` и строкой консоли: панель
    читает их тем же `queue_timing_text`, что и после кнопки.
    """

    def __init__(self, domain_pool, blobs, profile) -> None:
        self._domain_pool = domain_pool
        self._blobs = blobs
        self._profile = profile

    def cover(self, entries, alpha_text: str, cancel=None) -> dict:
        """`{patch_domain_id: запись}` посчитанных воркерами; `cancel` снят — `CoverageCancelled`."""

        from .envelope_domain_pool import DomainTaskV1

        shipped, shipping_failures = _ship_preparations(
            tuple(entries), self._blobs, lambda item: item[2]
        )
        if not _worth_the_pool(shipped):
            # Малая партия считается в родителе, но подготовка, которую не
            # удалось запикливать, называется и здесь.
            for domain_id, reason in shipping_failures.items():
                _note_task_fallback(domain_id, reason)
            if shipping_failures:
                _record_pool_counters(
                    self._profile, None, "", 0, len(shipping_failures), 0
                )
            return {}
        tasks = [
            DomainTaskV1(
                index,
                patch_id,
                domain_id,
                None,
                None,
                alpha_text,
                frozenset(),
                coverage=CoverageInputV1(blob, False, self._blobs.key_of(prepared)),
            )
            for index, ((patch_id, domain_id, prepared), blob) in enumerate(shipped)
        ]
        run, failure = _run_tasks(self._domain_pool, tasks, self._profile, cancel)
        if cancel is not None and cancel.is_set():
            # Задачи, которых отмена не дала взять, не «упали»: они не считаются ни отказом, ни
            # откатом в родителя, а заказ остановлен целиком.
            raise CoverageCancelled("slider coverage cancelled at the pool boundary")
        done: dict = {}
        fallbacks = len(shipping_failures)
        dispatched = 0
        for domain_id, reason in shipping_failures.items():
            _note_task_fallback(domain_id, reason)
        if not failure:
            for task in tasks:
                dispatched += 1
                result = None if run is None else run.results.get(task.task_id)
                if result is not None and result.ok:
                    done[task.domain_id] = result.queue_domain
                    continue
                fallbacks += 1
                _note_task_fallback(task.domain_id, _task_error(run, task))
        _record_pool_counters(
            self._profile, run, failure, dispatched, fallbacks, dispatched
        )
        return done


def adopt_pooled_domain(
    pooled,
    patch_id: int,
    domain_id: str,
    selected_edges: frozenset[int],
    snapshot,
    request,
    profile,
    preparation_adopter,
    preparation_provider,
):
    """Ответ воркера как результат домена.

    Домен целиком: тайминги `QUEUE_PREPARE` и `QUEUE_COVERAGE` — секунды, которые
    измерил сам воркер тем же `run_queue_domain`, что и последовательный путь, а
    подготовка проходит через кэш сессии (`preparation_adopter`) как промах со
    сборкой, и ползунок alpha находит её тёплой, как после последовательной
    кнопки.

    Только покрытие (`pooled.prepared is None`): подготовка берётся из кэша тем
    же `preparation_provider`, что и в последовательном пути, то есть как
    ПОПАДАНИЕ со своим счётчиком под стадией `QUEUE_PREPARE`, а секунды
    покрытия — воркера.
    """

    queue_domain = pooled.queue_domain
    if pooled.prepared is None:
        started = time.perf_counter()
        with _measure(profile, "QUEUE_PREPARE", domain_id):
            prepared = preparation_provider(
                patch_id, domain_id, frozenset(selected_edges), snapshot, request
            )
        prepare_seconds = time.perf_counter() - started
        if profile is not None:
            profile.add_timing(
                "QUEUE_COVERAGE", queue_domain.coverage_seconds, domain_id
            )
        return prepared, replace(
            queue_domain, preparation=prepared, prepare_seconds=prepare_seconds
        )
    prepared = pooled.prepared
    if profile is not None:
        profile.add_timing(
            "QUEUE_PREPARE", queue_domain.prepare_seconds, domain_id
        )
        profile.add_timing(
            "QUEUE_COVERAGE", queue_domain.coverage_seconds, domain_id
        )
    if preparation_adopter is not None:
        prepared = preparation_adopter(
            patch_id, domain_id, selected_edges, request, prepared
        )
    return prepared, replace(queue_domain, preparation=prepared)


def pool_notice(staged, domain_id: str):
    """Именованная причина, по которой домен считался не воркером, — строкой."""

    if staged is None:
        return ()
    from .envelope_request_export import (
        EnvelopeDebugHostDiagnosticV1,
        EnvelopeDebugHostSeverity,
    )

    return tuple(
        EnvelopeDebugHostDiagnosticV1(
            outcome, EnvelopeDebugHostSeverity.UNSUPPORTED, message, domain_id
        )
        for outcome, message in (
            (staged.pool_outcome, staged.pool_message),
            (staged.interpreter_outcome, staged.interpreter_message),
        )
        if outcome
    )


__all__ = (
    "COVERAGE_POOL_MIN_BYTES",
    "CoverageInputV1",
    "PreparationBlobsV1",
    "SliderCoveragePool",
    "StagedQueueDomainV1",
    "adopt_pooled_domain",
    "pool_notice",
    "solve_coverage_task",
    "stage_pool_domains",
)
