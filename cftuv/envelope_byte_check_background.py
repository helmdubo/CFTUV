"""Сторона родителя фоновой проверки первого нажатия (SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1; решение владельца: «Фоном после кнопки»).

Политика, воркерная сторона и шаг проверки - `envelope_snapshot_byte_check`; пул уступает любому прогону между шагами - `DomainPool.run_idle`. Здесь: сколько отложено, поток, который гонит шаги, итог и что с найденным.

ЗАДАЧА. После первого нажатия, где воркеры отложили круг кодека, `launch` заводит ОДНУ задачу (`ByteCheckJobV1`) и поток-демон: он шлёт воркерам шаг за шагом (`run_idle`) и складывает ответы. Поток не трогает ни bpy,
ни кэши сессии, кроме `refuse_byte_checked_domain` (замок сессии). Итог назван:

* `CLEAN` - проверены все отложенные домены, находок нет;
* `VIOLATION` - проверены все, найдено: каждый найденный домен помечен у сессии в тот момент, когда его нашли (не по концу задачи), и следующее действие отказывает его (`_scan`) тем же исходом хоста, текстом и
  доменом, что дала бы проверка на нажатии; из хранилища по содержимому домен забыт;
* `INCOMPLETE` - задача не довела (воркер умер, пул закрыт или пересоздан, сессия сброшена, ответ не того вида): называется причина и сколько проверено, идентичность кода остаётся непроверенной.

Идентичность кода (`settle_identity`) помечается проверенной при `CLEAN` и `VIOLATION`; при `INCOMPLETE` - нет. Строку итога печатает консоль (поток) и берёт панель (`text()`; оператор обновляет статус таймером главного потока).
"""

from __future__ import annotations

import threading
import time

from .envelope_snapshot_byte_check import BYTE_CHECK_DEFERRED, ByteCheckReplyV1, ByteCheckStepV1, settle_identity

BYTE_CHECK_BACKGROUND_OUTCOME = "SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1"
STATE_RUNNING = "RUNNING"
STATE_CLEAN = "CLEAN"
STATE_VIOLATION = "VIOLATION"
STATE_INCOMPLETE = "INCOMPLETE"
FINAL_STATES = (STATE_CLEAN, STATE_VIOLATION, STATE_INCOMPLETE)


class ByteCheckJobV1:
    """Одна фоновая проверка: сколько ждали, сколько проверено, что найдено и чем кончилось; читается из любых потоков (замок)."""

    def __init__(self, token: str, identity, expected: int, domains: dict, revision: str) -> None:
        self.token = token
        self.identity = identity
        #: Сколько снапшотов воркеры отложили в этом нажатии (по ответам воркеров).
        self.expected = int(expected)
        #: `{домен: (номер патча, ключ содержимого)}` доменов нажатия: по нему найденное помечается у сессии.
        self.domains = domains
        self.revision = revision
        self.state = STATE_RUNNING
        self.checked = 0
        self.violations: list = []
        self.reason = ""
        self.started = time.perf_counter()
        self.seconds = 0.0
        self.stop = threading.Event()
        self._lock = threading.Lock()
        self._done = threading.Event()
        self.thread: threading.Thread | None = None

    def absorb(self, reply: ByteCheckReplyV1) -> list:
        """Ответ воркера в итоги; найденное этого ответа (его пометит сессия)."""

        with self._lock:
            self.checked += int(reply.checked)
            self.violations.extend(reply.violations)
        return list(reply.violations)

    def finish(self, completed: bool, reason: str = "") -> None:
        """Итог: `completed` - все воркеры довели; иначе `INCOMPLETE` с причиной. Идентичность кода и строка консоли."""

        with self._lock:
            if self.state in FINAL_STATES:
                return
            self.seconds = time.perf_counter() - self.started
            verified = completed and self.checked >= self.expected
            if not verified:
                shortfall = f"{self.checked} of {self.expected} domains verified"
                self.state = STATE_INCOMPLETE
                self.reason = f"{reason or 'the workers did not report every deferred snapshot'}; {shortfall}"
            else:
                self.state = STATE_VIOLATION if self.violations else STATE_CLEAN
        settle_identity(self.identity, self.token, verified=self.state in (STATE_CLEAN, STATE_VIOLATION))
        print(f"[CFTUV][SnapshotByteCheck] {self.text()}", flush=True)
        for violation in self.violations:
            print(f"[CFTUV][SnapshotByteCheck]   {violation.domain_id}: {violation.outcome_name}: {violation.message[:240]}", flush=True)
        self._done.set()

    def wait(self, timeout: float | None = None) -> bool:
        """Ждёт конца задачи (инструменты и тесты); `True` - кончилась."""

        return self._done.wait(timeout)

    @property
    def finished(self) -> bool:
        return self.state in FINAL_STATES

    def text(self) -> str:
        """Строка статуса: исход, сколько проверено и что с найденным."""

        name = BYTE_CHECK_BACKGROUND_OUTCOME
        if self.state == STATE_RUNNING:
            return f"{name}: verifying {self.expected} first-press snapshots in the background ({self.checked} done)"
        if self.state == STATE_CLEAN:
            return f"{name}: CLEAN, {self.checked} snapshots verified in {self.seconds:.2f} s"
        if self.state == STATE_VIOLATION:
            outcomes = sorted({item.outcome_name for item in self.violations})
            return f"{name}: VIOLATION in {len(self.violations)} snapshots ({', '.join(outcomes)}); those domains are refused at the next action"
        return f"{name}: INCOMPLETE, {self.reason}"


def deferred_domains(reply) -> list:
    """Домены, чей снапшот воркер отложил в этой задаче (счётчики выгрузки ответа): по одному на каждый."""

    if reply is None:
        return []
    return [item.patch_domain_id for item in getattr(reply, "export_counters", ()) if item.name == BYTE_CHECK_DEFERRED for _ in range(int(item.value))]


def launch(run, pool, entries):
    """Задача фоновой проверки по итогам нажатия либо `None` (отложенного нет, пула нет или он не умеет шаги): поток-демон запущен, сессия знает задачу."""

    plan = run.topology_export.snapshot_byte_check
    ledger = list(run.byte_check_ledger)
    if plan is None or not plan.defer or not ledger:
        return None
    controller = run.controller
    if not hasattr(pool, "run_idle"):
        settle_identity(plan.identity, plan.token, verified=False)
        print(f"[CFTUV][SnapshotByteCheck] {BYTE_CHECK_BACKGROUND_OUTCOME}: INCOMPLETE, the pool cannot run background steps; {len(ledger)} snapshots stay unverified", flush=True)
        return None
    domains = {entry.domain_id: (entry.patch_id, entry.content_key) for entry in entries}
    job = ByteCheckJobV1(plan.token, plan.identity, len(ledger), domains, run.revision)
    controller.start_byte_check_job(job)
    job.thread = threading.Thread(target=_run, args=(job, controller, pool), name="cftuv-byte-check", daemon=True)
    job.thread.start()
    return job


def _run(job: ByteCheckJobV1, controller, pool) -> None:
    def on_reply(_index, reply) -> bool:
        if not isinstance(reply, ByteCheckReplyV1) or reply.token != job.token:
            raise RuntimeError(f"protocol violation: {type(reply).__name__}")
        for violation in job.absorb(reply):
            controller.refuse_byte_checked_domain(job, violation)
        return reply.remaining > 0

    try:
        completed, failures = pool.run_idle(ByteCheckStepV1(job.token), on_reply, stop=job.stop)
    except Exception as exc:  # noqa: BLE001 - причина называется в итоге задачи, а не теряется в потоке
        job.finish(False, f"{type(exc).__name__}: {exc}")
        return
    reason = "; ".join(f"worker {index}: {text}" for index, text in sorted(failures.items())) or ("the order was superseded" if job.stop.is_set() else "")
    job.finish(completed, reason)


__all__ = (
    "BYTE_CHECK_BACKGROUND_OUTCOME",
    "ByteCheckJobV1",
    "FINAL_STATES",
    "STATE_CLEAN",
    "STATE_INCOMPLETE",
    "STATE_RUNNING",
    "STATE_VIOLATION",
    "deferred_domains",
    "launch",
)
