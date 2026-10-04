"""Автомат модального инструмента «Adjust Decal Width»: события -> ширина, превью, исход.

Инструмент устроен как нативные Inset/Extrude: рука тянет мышь по экрану, ширина меняется
пропорционально пути, пока идёт перетаскивание рисуется ТОЛЬКО мгновенное превью
(`envelope_width_preview`, `PREVIEW_BINARY64_V1`), в меш ничего не пишется. ЛКМ либо Enter
подтверждают (точный пересчёт заказывается один раз на последнюю ширину), ПКМ либо Esc
отменяют: ширина остаётся прежней, превью снимается.

Автомат чистый: ни `bpy`, ни мыши Blender. События приходят значениями `WidthEventV1`, оператор
(`envelope_width_modal`) только переводит в них события Blender и исполняет итог, а тесты и смок
подают события напрямую. Законы:

- РАДИАЛЬНОЕ СМЕЩЕНИЕ. Ширина растёт с расстоянием от курсора до оси (точка выбранных цепей на экране)
  и убывает при приближении: `ширина += Δ(расстояние) * метров_на_пиксель`. Приращение, а не
  разность с началом, поэтому смена Shift посреди движения не дёргает ширину.
- CTRL — привязка к шагу (`step`); SHIFT — точно: приращение умножается на `PRECISION`.
- ЧИСЛО: цифры, точка и Backspace набирают значение, как в нативных операторах; пока число набирается,
  мышь ширину не меняет. Недопустимый набор (две точки, пусто) оставляет прежнюю ширину и называется
  в заголовке.
- Ширина не меньше `min_width` и не больше `max_width` (если задан).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

KIND_MOVE = "MOVE"
KIND_CONFIRM = "CONFIRM"
KIND_CANCEL = "CANCEL"
KIND_DIGIT = "DIGIT"
KIND_POINT = "POINT"
KIND_BACKSPACE = "BACKSPACE"

PHASE_ACTIVE = "ACTIVE"
PHASE_CONFIRMED = "CONFIRMED"
PHASE_CANCELLED = "CANCELLED"

#: Шаг привязки с Ctrl, единицы длины сцены.
DEFAULT_STEP = 0.01
#: Множитель приращения с Shift.
PRECISION = 0.1
DEFAULT_MIN_WIDTH = 0.0

HINT = "Ctrl snap, Shift fine, type a number, Enter/LMB confirm, Esc/RMB cancel"


@dataclass(frozen=True, slots=True)
class WidthEventV1:
    """Одно событие: вид, позиция мыши в пикселях региона, модификаторы, символ ввода."""

    kind: str
    x: float = 0.0
    y: float = 0.0
    ctrl: bool = False
    shift: bool = False
    char: str = ""


@dataclass(frozen=True, slots=True)
class WidthStepV1:
    """Итог одного события: что изменилось и где автомат теперь."""

    phase: str
    width: float
    changed: bool
    header: str


class WidthAdjustSessionV1:
    """Состояние одного перетаскивания ширины. Не знает Blender."""

    def __init__(
        self,
        start_width: float,
        *,
        pivot: tuple[float, float],
        start_mouse: tuple[float, float],
        metres_per_pixel: float,
        step: float = DEFAULT_STEP,
        min_width: float = DEFAULT_MIN_WIDTH,
        max_width: float | None = None,
        unit: str = "m",
    ) -> None:
        if not (metres_per_pixel > 0.0 and math.isfinite(metres_per_pixel)):
            raise ValueError("metres_per_pixel must be a positive finite number")
        self.start_width = float(start_width)
        self.step = float(step)
        self.min_width = float(min_width)
        self.max_width = None if max_width is None else float(max_width)
        self.unit = unit
        self._pivot = (float(pivot[0]), float(pivot[1]))
        self._scale = float(metres_per_pixel)
        self._raw = self._clamp(self.start_width)
        self._width = self._raw
        self._radius = self._distance(start_mouse)
        self._buffer = ""
        self._note = ""
        self.phase = PHASE_ACTIVE
        #: Сколько событий разобрано: тест называет, что автомат не «молчал».
        self.events = 0

    # ------------------------------------------------------------------

    @property
    def width(self) -> float:
        return self._width

    @property
    def numeric(self) -> bool:
        return bool(self._buffer)

    @property
    def changed(self) -> bool:
        return self._width != self.start_width

    def _distance(self, point) -> float:
        return math.hypot(float(point[0]) - self._pivot[0], float(point[1]) - self._pivot[1])

    def _clamp(self, value: float) -> float:
        value = max(self.min_width, value)
        return value if self.max_width is None else min(self.max_width, value)

    def header(self) -> str:
        """Строка заголовка области: ширина названа ПРЕВЬЮ, пока не подтверждена."""

        shown = self._buffer + "|" if self._buffer else f"{self._width:.3f}"
        note = f" [{self._note}]" if self._note else ""
        return f"Decal width: {shown} {self.unit} (preview){note}   {HINT}"

    # ------------------------------------------------------------------

    def handle(self, event: WidthEventV1) -> WidthStepV1:
        """Разбирает событие. Подтверждённый либо отменённый автомат события больше не принимает."""

        if self.phase != PHASE_ACTIVE:
            return WidthStepV1(self.phase, self._width, False, self.header())
        self.events += 1
        before = self._width
        kind = event.kind
        if kind == KIND_MOVE:
            self._move(event)
        elif kind == KIND_DIGIT:
            self._type(event.char)
        elif kind == KIND_POINT:
            self._type(".")
        elif kind == KIND_BACKSPACE:
            self._buffer = self._buffer[:-1]
            self._note = ""
            self._from_buffer()
        elif kind == KIND_CONFIRM:
            self.phase = PHASE_CONFIRMED
        elif kind == KIND_CANCEL:
            self.phase = PHASE_CANCELLED
            self._width = self.start_width
        else:
            raise ValueError(f"unknown width event kind {kind!r}")
        return WidthStepV1(self.phase, self._width, self._width != before, self.header())

    def _move(self, event: WidthEventV1) -> None:
        radius = self._distance((event.x, event.y))
        if self._buffer:
            self._radius = radius  # число набирается: мышь ширину не меняет, но опору не теряет
            return
        gain = (radius - self._radius) * self._scale * (PRECISION if event.shift else 1.0)
        self._radius = radius
        self._raw = self._clamp(self._raw + gain)
        width = self._raw
        if event.ctrl and self.step > 0.0:
            width = self._clamp(round(width / self.step) * self.step)
        self._width = width

    def _type(self, char: str) -> None:
        if char == "." and "." in self._buffer:
            self._note = "second decimal point ignored"
            return
        if len(char) != 1 or not (char.isdigit() or char == "."):
            self._note = f"unsupported character {char!r} ignored"
            return
        self._note = ""
        self._buffer += char
        self._from_buffer()

    def _from_buffer(self) -> None:
        if not self._buffer:
            self._width = self._raw
            return
        try:
            value = float(self._buffer)
        except ValueError:
            self._note = "not a number: previous width kept"
            return
        self._width = self._clamp(value)
        self._raw = self._width


__all__ = (
    "DEFAULT_MIN_WIDTH",
    "DEFAULT_STEP",
    "HINT",
    "KIND_BACKSPACE",
    "KIND_CANCEL",
    "KIND_CONFIRM",
    "KIND_DIGIT",
    "KIND_MOVE",
    "KIND_POINT",
    "PHASE_ACTIVE",
    "PHASE_CANCELLED",
    "PHASE_CONFIRMED",
    "PRECISION",
    "WidthAdjustSessionV1",
    "WidthEventV1",
    "WidthStepV1",
)
