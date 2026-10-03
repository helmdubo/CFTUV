"""Фоновое превью alpha: ползунок ЗАКАЗЫВАЕТ счёт, а не считает его.

ЗАЧЕМ. Калбэк `update` ползунка alpha раньше считал покрытие точно, на месте: на
каждое движение мыши интерфейс стоял 0.3 с (`2`) и до 1.2 с (`building`). Здесь
счёт уходит из главного потока: калбэк только записывает заказ (микросекунды), таймер
`bpy.app.timers` выдерживает паузу, запускает счёт, опрашивает его и применяет
результат на главном потоке. Пока считается, на экране остаётся ПРЕЖНИЙ результат.

ЗАКОН ЗАКАЗА (всё это держат тесты `tests/test_envelope_alpha_preview.py`).

- ПАУЗА (debounce): счёт стартует через `DEBOUNCE_SECONDS` после ПОСЛЕДНЕГО
  изменения. Каждое новое значение сдвигает срок.
- СЛИЯНИЕ (coalescing): значение, замещённое до старта, не считается вовсе;
  счётчик `coalesced` называет, сколько таких было.
- ОДИН ПОЛЁТ. Воркеры пула однопоточны по кадрам, поэтому в полёте не больше
  одного счёта. Новое значение отменяет полёт кооперативно (`cancel` — событие,
  которое пул проверяет между задачами), а следующий счёт стартует, когда прежний
  вернулся.
- УСТАРЕВШЕЕ НЕ ПРИМЕНЯЕТСЯ. Результат счёта, чьё значение уже не последнее,
  выбрасывается и считается в `stale`; результат последнего значения применяется,
  только если цель за время счёта не сменилась (`valid`: другая тёплая сессия,
  исчезнувший объект) — иначе `invalid` с названной причиной.
- ПОСЛЕДНИЙ РЕЗУЛЬТАТ ТОТ ЖЕ, ЧТО У КНОПКИ: счёт — тот же вызов, что и в
  последовательном ползунке, на тех же тёплых подготовках, и применяется тем же
  кодом, что и после кнопки; равенство держит тест, а не слово.
- НИКАКИХ ДАННЫХ BLENDER В ПОТОКЕ СЧЁТА. Поток получает замкнутую на питоновские
  объекты функцию `compute(cancel)`; всё, что читает или пишет `bpy`, —
  `begin`/`valid`/`apply` — выполняется на главном потоке из `step`. Этот модуль
  `bpy` не импортирует (стена `tests/test_architecture.py`): таймеры приходят
  параметром.
- НИЧЕГО НЕ ПРОПАДАЕТ МОЛЧА: каждая отброшенная работа — счётчик, каждая причина —
  строка статуса панели (`status_text`), а ошибка потока — строка консоли и статус.

ОТМЕНА ПЕРЕД ТЯЖЁЛЫМ СИНХРОННЫМ ПУТЁМ. Кнопки считают на главном потоке на тех же
подготовках (их память растёт от покрытий), поэтому `quiesce` останавливает полёт и
ЖДЁТ его конца ДО их работы; `supersede` — то же без ожидания (калбэки свойств).
"""

from __future__ import annotations

import sys
import threading
import time
import traceback
from dataclasses import dataclass
from typing import Callable

#: Пауза после последнего изменения до старта счёта.
DEBOUNCE_SECONDS = 0.15
#: Шаг опроса таймером, пока счёт в полёте.
POLL_SECONDS = 0.03
#: Сколько `quiesce` ждёт конца отменённого полёта (кооперативная отмена: до одной задачи пула).
QUIESCE_TIMEOUT_SECONDS = 30.0
#: Нижняя граница шага таймера (нулевой шаг крутил бы главный цикл вхолостую).
MIN_DELAY_SECONDS = 0.005

STATUS_COMPUTING = "computing"
STATUS_READY = "ready"
STATUS_UNAVAILABLE = "unavailable"
STATUS_INVALID = "discarded"
STATUS_FAILED = "failed"
STATUS_CANCELLED = "cancelled"

_COUNTER_NAMES = (
    "requested",
    "coalesced",
    "started",
    "applied",
    "stale",
    "invalid",
    "unavailable",
    "failed",
    "cancelled",
)


class PreviewUnavailable(RuntimeError):
    """Счёт или применение невозможны: причина — текст для панели."""


class PreviewCancelled(RuntimeError):
    """Счёт остановлен по заказу: результата нет и не будет."""


@dataclass(frozen=True, slots=True)
class PreviewRequestV1:
    """Один заказ: порядковый номер, значение, что нужно цели, момент заказа."""

    sequence: int
    alpha: float
    payload: object
    requested_at: float


@dataclass(frozen=True, slots=True)
class PreviewCountersV1:
    """Куда ушла каждая заказанная работа (сумма исходов не теряет ни одного заказа)."""

    requested: int = 0
    #: Значение замещено до старта: не считалось вовсе.
    coalesced: int = 0
    started: int = 0
    applied: int = 0
    #: Счёт пошёл, но значение успело устареть: результат выброшен.
    stale: int = 0
    #: Счёт последнего значения готов, но цель сменилась: результат выброшен.
    invalid: int = 0
    unavailable: int = 0
    failed: int = 0
    #: Остановлено `quiesce`/`supersede` (кнопка, сброс сессии).
    cancelled: int = 0


@dataclass(frozen=True, slots=True)
class PreviewAppliedV1:
    """Последний применённый результат и его цена (`latency` — от ПОСЛЕДНЕГО заказа)."""

    sequence: int
    alpha: float
    compute_seconds: float
    apply_seconds: float
    latency_seconds: float


class ThreadedPreviewJob:
    """Счёт в потоке: `compute(cancel)` и больше ничего.

    `context` — то, что вызывающий кладёт для `valid`/`apply` (читает только главный
    поток). Поток не трогает ничего, кроме собственного слота результата.
    """

    def __init__(self, compute: Callable[[threading.Event], object], *, context=None):
        self.context = context
        self.seconds = 0.0
        self._cancel = threading.Event()
        self._value = None
        self._error: BaseException | None = None
        self._thread = threading.Thread(
            target=self._run, args=(compute,), name="CFTUV-alpha-preview", daemon=True
        )
        self._thread.start()

    def _run(self, compute) -> None:
        started = time.perf_counter()
        try:
            self._value = compute(self._cancel)
        except BaseException as exc:  # noqa: BLE001 - причина идёт в главный поток
            self._error = exc
        self.seconds = time.perf_counter() - started

    @property
    def finished(self) -> bool:
        return not self._thread.is_alive()

    def cancel(self) -> None:
        self._cancel.set()

    def join(self, timeout: float | None = None) -> None:
        self._thread.join(timeout)

    def result(self):
        """Значение либо исключение потока (`PreviewCancelled` — отмена)."""

        if not self.finished:
            raise RuntimeError("preview job is still running")
        if self._error is not None:
            raise self._error
        return self._value


class AlphaPreviewScheduler:
    """Пауза, слияние, один полёт, выбрасывание устаревшего. Только главный поток.

    `begin(request) -> job` стартует счёт (может бросить `PreviewUnavailable`);
    `valid(request, job) -> str | None` — причина, по которой результат применять
    нельзя, либо `None`; `apply(request, job, value)` пишет результат в цель
    (может бросить `PreviewUnavailable`). `job` — `ThreadedPreviewJob` либо любой
    объект с `finished`, `cancel()`, `join(timeout)`, `result()`.
    `timers` — объект с `register(function, first_interval)` и
    `is_registered(function)` (во Blender — `bpy.app.timers`).
    """

    def __init__(
        self,
        *,
        begin: Callable,
        apply: Callable,
        timers,
        valid: Callable | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        on_change: Callable[[str], None] | None = None,
        debounce: float = DEBOUNCE_SECONDS,
        poll: float = POLL_SECONDS,
        quiesce_timeout: float = QUIESCE_TIMEOUT_SECONDS,
    ) -> None:
        self._begin = begin
        self._apply = apply
        self._valid = valid
        self._timers = timers
        self._clock = clock
        self._sleep = sleep
        self._on_change = on_change
        self._debounce = float(debounce)
        self._poll = float(poll)
        self._quiesce_timeout = float(quiesce_timeout)
        # Одна связанная функция на всё время жизни: `bpy.app.timers.is_registered`
        # сравнивает ОБЪЕКТ функции, а новая связка метода при каждом обращении — другой объект.
        self._callback = self._tick
        self._sequence = 0
        # Заказы с номером не выше этого сняты `supersede`/`quiesce` и уже посчитаны как `cancelled`:
        # их поздний результат выбрасывается без второй записи.
        self._retired_through = 0
        self._pending: PreviewRequestV1 | None = None
        self._deadline = 0.0
        self._job = None
        self._job_request: PreviewRequestV1 | None = None
        self._counts = dict.fromkeys(_COUNTER_NAMES, 0)
        self._burst_dropped = 0
        self._phase = ""
        self._phase_text = ""
        self._status = ""
        self.last_applied: PreviewAppliedV1 | None = None

    # ------------------------------------------------------------------
    # Состояние для панели и тестов
    # ------------------------------------------------------------------

    @property
    def counters(self) -> PreviewCountersV1:
        return PreviewCountersV1(**self._counts)

    @property
    def busy(self) -> bool:
        """Есть заказ, который ещё не дошёл до применения или названного исхода."""

        return self._pending is not None or self._job is not None

    @property
    def in_flight(self) -> bool:
        return self._job is not None

    @property
    def status_text(self) -> str:
        return self._status

    # ------------------------------------------------------------------
    # Заказ
    # ------------------------------------------------------------------

    def request(self, alpha: float, payload=None) -> PreviewRequestV1:
        """Новое значение: запись и сдвиг срока. Микросекунды, ничего не считает."""

        now = self._clock()
        if not self.busy:
            self._burst_dropped = 0
        self._sequence += 1
        request = PreviewRequestV1(self._sequence, float(alpha), payload, now)
        self._counts["requested"] += 1
        if self._pending is not None:
            self._counts["coalesced"] += 1
            self._burst_dropped += 1
        self._pending = request
        self._deadline = now + self._debounce
        if self._job is not None:
            self._job.cancel()
        self._set_phase(STATUS_COMPUTING)
        self._arm()
        return request

    def note_unavailable(self, reason: str) -> None:
        """Именованное «счёта не будет» (нет тёплой сессии и т. п.): строка и счётчик."""

        self._counts["unavailable"] += 1
        self._set_phase(STATUS_UNAVAILABLE, reason)

    def _arm(self) -> None:
        if not self._timers.is_registered(self._callback):
            self._timers.register(self._callback, first_interval=self._debounce)

    # ------------------------------------------------------------------
    # Шаг таймера
    # ------------------------------------------------------------------

    def _tick(self):
        return self.step()

    def step(self) -> float | None:
        """Один шаг: принять готовый счёт, стартовать назревший. Следующий шаг либо `None`."""

        try:
            self._collect()
            self._start_if_due()
        except Exception as exc:  # noqa: BLE001 - таймер не должен умирать молча
            self._counts["failed"] += 1
            self._report_failure("scheduler step", exc)
        return self._next_delay()

    def _collect(self) -> None:
        job = self._job
        if job is None or not job.finished:
            return
        request = self._job_request
        self._job = None
        self._job_request = None
        if request.sequence <= self._retired_through:
            return
        try:
            value = job.result()
        except PreviewCancelled:
            self._discard_as_stale(request)
            return
        except Exception as exc:  # noqa: BLE001 - причина идёт в статус и консоль
            self._counts["failed"] += 1
            self._report_failure(f"alpha={request.alpha:.4g} compute", exc)
            return
        if request.sequence != self._sequence:
            self._discard_as_stale(request)
            return
        reason = None if self._valid is None else self._valid(request, job)
        if reason:
            self._counts["invalid"] += 1
            self._set_phase(STATUS_INVALID, f"discarded alpha={request.alpha:.4g}: {reason}")
            return
        started = self._clock()
        try:
            self._apply(request, job, value)
        except PreviewUnavailable as exc:
            self._counts["invalid"] += 1
            self._set_phase(STATUS_INVALID, f"discarded alpha={request.alpha:.4g}: {exc}")
            return
        except Exception as exc:  # noqa: BLE001
            self._counts["failed"] += 1
            self._report_failure(f"alpha={request.alpha:.4g} apply", exc)
            return
        finished = self._clock()
        self._counts["applied"] += 1
        self.last_applied = PreviewAppliedV1(
            request.sequence,
            request.alpha,
            float(getattr(job, "seconds", 0.0)),
            finished - started,
            finished - request.requested_at,
        )
        self._set_phase(STATUS_READY)

    def _discard_as_stale(self, request: PreviewRequestV1) -> None:
        self._counts["stale"] += 1
        self._burst_dropped += 1
        self._set_phase(self._phase or STATUS_COMPUTING)

    def _start_if_due(self) -> None:
        if self._job is not None or self._pending is None:
            return
        if self._clock() < self._deadline:
            return
        request, self._pending = self._pending, None
        try:
            job = self._begin(request)
        except PreviewUnavailable as exc:
            self._counts["unavailable"] += 1
            self._set_phase(STATUS_UNAVAILABLE, str(exc))
            return
        except Exception as exc:  # noqa: BLE001
            self._counts["failed"] += 1
            self._report_failure(f"alpha={request.alpha:.4g} start", exc)
            return
        self._job = job
        self._job_request = request
        self._counts["started"] += 1

    def _next_delay(self) -> float | None:
        if self._job is not None:
            return self._poll
        if self._pending is not None:
            return max(MIN_DELAY_SECONDS, min(self._poll, self._deadline - self._clock()))
        return None

    # ------------------------------------------------------------------
    # Отмена
    # ------------------------------------------------------------------

    def supersede(self, reason: str) -> bool:
        """Снимает заказ и просит полёт остановиться, НЕ ЖДА. Результат полёта будет выброшен."""

        had_work = self.busy
        self._retired_through = self._sequence
        self._sequence += 1
        self._pending = None
        if self._job is not None:
            self._job.cancel()
        if had_work:
            self._counts["cancelled"] += 1
            self._set_phase(STATUS_CANCELLED, f"cancelled: {reason}")
        return had_work

    def quiesce(self, reason: str) -> bool:
        """`supersede` и ожидание конца полёта: после возврата поток счёта не пишет в подготовки."""

        had_work = self.supersede(reason)
        job = self._job
        if job is not None:
            job.join(self._quiesce_timeout)
            if job.finished:
                self._job = None
                self._job_request = None
            else:
                print(
                    f"[CFTUV][AlphaPreview] cancelled job still running after "
                    f"{self._quiesce_timeout:.0f} s ({reason})",
                    file=sys.stderr,
                    flush=True,
                )
        return had_work

    def settle(self, timeout: float = 120.0) -> None:
        """Блокирующий слив для тестов и пакетных инструментов: без паузы, до тишины.

        Таймер сам этого не делает; сюда приходят смоки и свипы, у которых нет главного
        цикла, а ответ после изменения значения нужен до следующей строки.
        """

        if self._pending is not None:
            self._deadline = self._clock()
        end = self._clock() + timeout
        while self.busy:
            self.step()
            if not self.busy:
                return
            if self._clock() > end:
                raise TimeoutError(f"alpha preview did not settle in {timeout} s")
            self._sleep(self._poll)

    # ------------------------------------------------------------------
    # Строка статуса
    # ------------------------------------------------------------------

    def _report_failure(self, what: str, exc: BaseException) -> None:
        print(f"[CFTUV][AlphaPreview] {what} failed: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        traceback.print_exception(type(exc), exc, exc.__traceback__, file=sys.stderr)
        self._set_phase(STATUS_FAILED, f"failed ({what}): {type(exc).__name__}")

    def _set_phase(self, phase: str, text: str = "") -> None:
        self._phase = phase
        self._phase_text = text
        status = self._compose()
        if status == self._status:
            return
        self._status = status
        if self._on_change is None:
            return
        try:
            self._on_change(status)
        except Exception as exc:  # noqa: BLE001 - перерисовка косметична, счёт она не ломает
            print(f"[CFTUV][AlphaPreview] status redraw failed: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)

    def _compose(self) -> str:
        latest = self._pending or self._job_request
        if self._phase == STATUS_COMPUTING:
            alpha = "" if latest is None else f" alpha={latest.alpha:.4g}"
            text = f"computing{alpha}..."
        elif self._phase == STATUS_READY:
            applied = self.last_applied
            text = "ready" if applied is None else (
                f"ready alpha={applied.alpha:.4g} (compute {applied.compute_seconds:.2f} s, "
                f"apply {applied.apply_seconds:.2f} s)"
            )
        else:
            text = self._phase_text
        if self._burst_dropped and text:
            text += f" | superseded {self._burst_dropped}"
        return text


__all__ = (
    "AlphaPreviewScheduler",
    "DEBOUNCE_SECONDS",
    "MIN_DELAY_SECONDS",
    "POLL_SECONDS",
    "PreviewAppliedV1",
    "PreviewCancelled",
    "PreviewCountersV1",
    "PreviewRequestV1",
    "PreviewUnavailable",
    "QUIESCE_TIMEOUT_SECONDS",
    "ThreadedPreviewJob",
)
