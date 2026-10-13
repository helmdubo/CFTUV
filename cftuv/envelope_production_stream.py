"""Обход доменов ЗА пулом (SCAN_BEHIND_THE_POOL_V1): пул стартует, как только отобрана голова первого круга, остаток доменов сканируется, пока воркеры считают.

ЗАЧЕМ. На холодной `cover.008` родитель перед пулом строил входы и ключи содержимого ВСЕХ 1051 доменов (`_scan`: 0.64 с, из них ключи 0.47 с), хотя первому кругу нужны лишь двенадцать самых тяжёлых задач, а пул
в это время простаивал. `HEAD_STREAMED_FIRST_ROUND_V1` уже отдаёт пулу первый круг по отбору кандидатов (по числу цепочек), пока остаток кодируется и упорядочивается. Здесь то же для сканирования:
кандидатов (`HEAD_CANDIDATES_PER_WORKER * воркеров` патчей с наибольшим числом цепочек: оно известно из экспорта топологии без сканирования) сканирует родитель, пул получает их задачи, а остальные домены
сканируются в потоке родителя, пока воркеры уже считают, и их задачи (`LateTasksV1`) уходят в очередь пула вместе с остатком, который он и так упорядочивает.

ЧТО ОСТАЁТСЯ ТЕМ ЖЕ. Обход идёт ТОЛЬКО на холодной сессии (`EnvelopeDebugSessionController.scan_is_order_free`: ни метрик, ни результатов, ни привязок ключей, ни записей хранилища, ни пометок фона, ни записей сборки):
тогда решение по домену не зависит ни от одного другого домена, и каждый домен - холодный с лёгким входом выгрузки, а ключ содержимого - чистая функция входа (память общих записей `key_memo` не меняет ни байта ключа).
Любая другая сессия, любой пул без `late`, малая партия и не-продуктовые прогоны (lean, served, skipped) идут прежним путём. Те же записи (`entries` в порядке патчей), те же задачи (`task_id` - место в порядке патчей, кадры те же байты),
тот же первый круг и порядок остатка (пул отбирает кандидатов тем же ключом `(-число цепочек, task_id)`), тот же порядок приёма ответов, тот же порядок записей в кэши, хранилище и вытеснения. Отличается одно: когда считается скан.
Исход назван счётчиком `PRODUCTION_SCAN_STREAMED` (сколько доменов отсканировано за пулом); его нет у прогона, который шёл прежним путём.

ИНВАРИАНТ ВМЕСТО МОЛЧАНИЯ. Запись домена, не похожая на холодную с входом выгрузки, невозможна на холодной сессии, и если она всё же возникла, обход называет это (`SCAN_BEHIND_THE_POOL_INVARIANT`) и отказывает прогону: решать за порядок, которого уже нет, он не вправе.
Исключение скана остатка не теряется: пул кончает голову, и оно поднимается из прогона тем же типом, что поднял бы `_scan`.
"""

from __future__ import annotations

from .envelope_production_scan import _check_cancel, _Scan

SCAN_BEHIND_THE_POOL_OUTCOME = "SCAN_BEHIND_THE_POOL_V1"
SCAN_BEHIND_THE_POOL_INVARIANT = "SCAN_BEHIND_THE_POOL_INVARIANT"
PRODUCTION_SCAN_STREAMED = "PRODUCTION_SCAN_STREAMED"
#: Выключатель (тесты: прогон прежним путём как эталон).
STREAMED_SCAN = True
#: Сколько патчей на воркера родитель сканирует ДО пула (по числу цепочек): пул выбирает из них первый круг (`HEAD_CANDIDATES_PER_WORKER`), а остальные кладёт в очередь сразу - воркеры берут их, пока скан остатка идёт
#: (на `cover.008` скан остатка за пулом стоил 1.04 с против 0.64 до него: родитель идёт на соседнем потоке ядра, и первые задачи кончаются раньше, чем остаток готов).
STREAMED_SCAN_CANDIDATES_PER_WORKER = 4


class StreamedRunV1:
    """Один потоковый обход: голова (кандидаты первого круга), остаток за пулом, приём ответов.

    `plan` возвращает его, когда обход допустим, иначе `None`. `execute()` - `(записи доменов в порядке патчей, {домен: результат}, {домен: отказ входа})`, как `_scan` и `_dispatch_work` вместе.
    """

    def __init__(self, run, pool, order: list, candidates: int) -> None:
        self.run = run
        self.pool = pool
        self.order = order
        self.candidates = candidates
        self.entries: list = [None] * len(order)
        self.scan = None
        self.laws = None
        self.rest_tasks: list | None = None
        self.error: BaseException | None = None

    def _scanned(self, position: int):
        entry = self.scan.entry(self.order[position])
        plain = (
            entry is not None
            and entry.is_cold
            and entry.export is not None
            and entry.inputs is None
            and not entry.rebased
        )
        if not plain:
            raise RuntimeError(f"{SCAN_BEHIND_THE_POOL_INVARIANT}: patch {self.order[position]} is not a plain cold domain with an export input on a cold session")
        self.entries[position] = entry
        from .envelope_production_export import cold_task

        return cold_task(self.run, entry, position, self.laws)

    def _provide(self) -> list:
        """Остаток задач: сканирует домены, которых нет в голове (по порядку патчей). Один раз; исключение сохранено, остаток пуст."""

        if self.rest_tasks is None:
            try:
                self.rest_tasks = [self._scanned(position) for position, entry in enumerate(self.entries) if entry is None]
                self.scan.finish()
            except Exception as exc:  # noqa: BLE001 - пул кончает голову, исключение поднимется из прогона
                self.error = exc
                self.rest_tasks = []
        return self.rest_tasks

    def execute(self):
        from .envelope_domain_pool import LateTasksV1
        from .envelope_production_export import cold_laws, settle_dispatch
        from .envelope_queue_pool import _run_tasks

        run = self.run
        self.scan = _Scan(run)
        self.laws = cold_laws(run)
        chains = run.topology_export.patch_chains
        ranked = sorted(range(len(self.order)), key=lambda position: (-len(chains(self.order[position])), position))
        head = [self._scanned(position) for position in sorted(ranked[: self.candidates])]
        _check_cancel(run)
        pooled, failure = _run_tasks(self.pool, head, run.profile, cancel=run.cancel, late=LateTasksV1(len(self.order), self._provide))
        rest = self._provide()  # пул, который не стартовал, остатка не просил: он отсканирован здесь, теми же записями
        if self.error is not None:
            raise self.error
        run.profile.set_counter(PRODUCTION_SCAN_STREAMED, len(rest) if pooled is not None else 0)
        tasks = sorted([*head, *rest], key=lambda task: task.task_id)
        done, refused = settle_dispatch(run, [], self.entries, self.pool, tasks, pooled, failure, {})
        return self.entries, done, refused


def plan(run, pool) -> StreamedRunV1 | None:
    """Потоковый обход прогона либо `None` (прежний путь): условия модуля, без единого скана домена."""

    from .envelope_domain_pool import STREAMED_HEAD_MIN_TASKS_PER_WORKER

    if not STREAMED_SCAN or pool is None or not getattr(pool, "streams_late", False) or run.lean or run.served or run.skipped:
        return None
    workers = int(getattr(pool, "requested", 0))
    order = list(run.patch_ids)
    if workers < 2 or len(order) < STREAMED_HEAD_MIN_TASKS_PER_WORKER * workers or not run.controller.scan_is_order_free(run):
        return None
    return StreamedRunV1(run, pool, order, STREAMED_SCAN_CANDIDATES_PER_WORKER * workers)


__all__ = ("PRODUCTION_SCAN_STREAMED", "SCAN_BEHIND_THE_POOL_INVARIANT", "SCAN_BEHIND_THE_POOL_OUTCOME", "STREAMED_SCAN", "STREAMED_SCAN_CANDIDATES_PER_WORKER", "StreamedRunV1", "plan")
