"""Циклический сборщик мусора Python стоит на паузе на время кнопки и тика ширины (GC_PAUSED_DURING_BUTTON_V1, GC_PAUSED_DURING_WIDTH_TICK_V1).

КНОПКА. На холодном нажатии `cover.008` (1051 домен) сборщик даёт ~1.6-1.8 с пауз в родителе: он в это время строит и принимает сотни тысяч долгоживущих
объектов (снапшоты, подготовки, результаты), и каждое поколение-2 обходит их заново. Воркеры пула дают ещё ~16 процессорных секунд пауз на восьмерых
(`envelope_domain_pool.serve_task`: пауза держится на одной задаче воркера). Значения GC не меняет вовсе: он только возвращает
память циклов. Поэтому `gc_paused` отключает его на время блока и возвращает ПРЕЖНЕЕ состояние в `finally`: сборщик, выключенный до
входа, остаётся выключенным, вложенный блок не включает его раньше внешнего, исключение состояние не теряет. Принудительного `gc.collect()`
в конце нет: после блока сборщик просто работает как раньше.

ТИК ШИРИНЫ. Точное обновление ширины (`envelope_width_live`: счёт в потоке, применение на главном потоке) на `cover.008` двухмодально: каждый второй тик несёт ~+400 мс
сборки поколения 2 в родителе. Тот же блок держится на счёте тика (поток), на его применении, на затравке модели и на подтверждении инструмента (`gc_paused_function`).
Граница - работа, а не простой: между тиками (пауза планировщика, опрос таймера, рука на месте) сборщик включён.

ПРОЦЕСС ОДИН. Сборщик общий на процесс: `gc.disable()` в потоке счёта выключает его и для главного потока, пока поток внутри блока (счёт тика - секунды). Это и нужно (главный поток в это
время лишь опрашивает таймер), но два блока разных потоков перекрываются НЕ вложенно (счёт ещё идёт, а кнопка уже вошла и ждёт его в `quiesce`; применение входит, пока затравка ещё
выходит), и «каждый блок вернул то, что застал» на таком порядке выходов включил бы сборщик под ещё открытым блоком или оставил бы его выключенным. Поэтому открытые блоки СЧИТАЮТСЯ
под замком: сборщик выключает первый вошедший (он и запомнил, был ли тот включён), включает последний вышедший, и только если он был включён. Для одного потока и вложенных блоков это
прежнее поведение до бита.

ЗАМОРОЗКА ПОСЛЕ КНОПКИ (GC_FROZEN_AFTER_BUTTON_V1, `gc_paused(freeze=True)`). Выходя из паузы, первая же аллокация запускала одну сборку поколения 0 по ВСЕМУ, что нажатие
накопило (~1.4 млн живых объектов на `cover.008`): 0.31-0.36 с в конце кнопки, до того как Blender перерисует окно; потом те же объекты обходили сборки поколений 1 и 2 (полная сборка ~0.5-0.66 с
в случайный момент). Кнопка переносит накопленное в постоянное поколение (`gc.freeze()`) ПЕРЕД `gc.enable()`: обходить нечего. Цена - циклический мусор среди замороженных объектов никогда не
собирается, поэтому заморозка допустима только вместе с PRODUCTION_REFUSAL_WITHOUT_TRACEBACK_V1 (`released`): прежде нажатие оставляло ~110 тыс. объектов в циклах трасс отказов
(кадры `run_production` держали рабочий набор), и заморозка утекала бы ~18 МБ за нажатие. С разорванными циклами мусора на конце нажатия 0, а клапан (`FREEZE_VALVE_OBJECTS`) не даёт замороженному
расти без счёта: выше него заморозка пропускается, а замороженное возвращается сборщику (`gc.unfreeze()`), и обычный ход сборок догоняет его, как прежде.

ЗАМОРОЗКА ПОСЛЕ ТИКА (GC_FROZEN_AFTER_WIDTH_TICK_V1, `gc_paused_function`). Работа тика ширины выходит из паузы так же: иначе первая аллокация после блока обходила поколение 0 всего,
что тик накопил, - в стене тика (у счёта в потоке - до его конца) и у подтверждения. Замер `cover.008` w12, 4 перетаскивания с подтверждением, ~300 работ такта: 303 сборки на 1.54 с, одна до 0.11 с,
0.34 с в окнах четырёх фиксаций; с заморозкой - 0 сборок. Повторение не копит замороженного: выжившее тика умирает счётчиком ссылок на следующем тике (заморозка освобождению не мешает), а циклов работа
тика не оставляет - отмена такта держится без трассы (CANCELLED_TICK_WITHOUT_TRACE_V1, `released`; до неё заморозка удержала бы ~49 тыс. объектов за три перетаскивания). Постоянное поколение и
частная память ровные между перетаскиваниями. Клапан тик не проверяет (`valve=False`): `gc.get_freeze_count()` обходит всё постоянное поколение (~1.2 млн объектов), а это стена каждого такта;
его проверяют кнопка и подготовка в простое после неё.

ЗАМОРОЖЕННОЕ СОБИРАЕТСЯ В ПРОСТОЕ (GC_FROZEN_DURING_DRAG_COLLECTED_AT_IDLE_V1, `collect_frozen_when_idle`). `gc.freeze()` общий на процесс: он уносит в постоянное поколение и ЧУЖОЙ
циклический мусор, который ещё никто не собрал (другие аддоны, интерфейс, Python самого Blender), а наши тесты и замеры видят только свой. Поэтому замороженное живёт только до простоя:
когда ни одного блока не открыто, вызывающий не занят (перетаскивание кончилось, заказы ширины и alpha применены либо названы) и с последнего выхода из паузы прошло `quiet_seconds`, всё
замороженное возвращается сборщику (`gc.unfreeze()`) и одна полная сборка идёт здесь - рука ничего не ждёт. Таймер простоя - `envelope_width_live.collect_frozen_at_idle`.

Исходы названы: `GC_PAUSED_DURING_BUTTON_V1` и `GC_PAUSED_DURING_WIDTH_TICK_V1` - ответ от паузы не зависит (проверено равенством ответа в поле), меняется только
время и пик памяти процесса, пока кнопка или тик идут; `GC_FROZEN_AFTER_BUTTON_V1` и `GC_FROZEN_AFTER_WIDTH_TICK_V1` - постоянное поколение держит замороженное (`gc.get_freeze_count()`)
до простоя; `GC_FROZEN_DURING_DRAG_COLLECTED_AT_IDLE_V1` - простой вернул его сборщику.
"""

from __future__ import annotations

import functools
import gc
import threading
import time
from contextlib import contextmanager

GC_PAUSE_OUTCOME = "GC_PAUSED_DURING_BUTTON_V1"
GC_PAUSE_WIDTH_OUTCOME = "GC_PAUSED_DURING_WIDTH_TICK_V1"
GC_FROZEN_OUTCOME = "GC_FROZEN_AFTER_BUTTON_V1"
GC_FROZEN_WIDTH_OUTCOME = "GC_FROZEN_AFTER_WIDTH_TICK_V1"
GC_IDLE_COLLECT_OUTCOME = "GC_FROZEN_DURING_DRAG_COLLECTED_AT_IDLE_V1"
#: Сколько объектов может держать постоянное поколение, прежде чем кнопка перестанет замораживать и вернёт замороженное сборщику (клапан от неограниченного роста: см. модуль).
FREEZE_VALVE_OBJECTS = 6_000_000

_LOCK = threading.Lock()
#: Сколько блоков `gc_paused` открыто сейчас во всём процессе (все потоки).
_OPEN_BLOCKS = 0
#: Был ли сборщик включён, когда вошёл первый из открытых блоков: включит его последний вышедший.
_ENABLE_AFTER_LAST = False
#: Какой-то из открытых блоков просил заморозку (кнопка, тик): последний вышедший заморозит накопленное перед включением сборщика.
_FREEZE_ON_LAST = False
#: Какой-то из открытых блоков с заморозкой просил проверить клапан (кнопка): последний вышедший сначала сверит постоянное поколение с `FREEZE_VALVE_OBJECTS`.
_VALVE_ON_LAST = False
#: Блоки что-то заморозили после последней сборки в простое (`collect_frozen_when_idle` вернёт это сборщику).
_FROZEN_UNCOLLECTED = False
#: Когда закрылся последний блок (`time.perf_counter`): простой отсчитывается от него.
_LAST_EXIT = 0.0


@contextmanager
def gc_paused(freeze: bool = False, *, valve: bool = True):
    """Отключает циклический сборщик на время блока; возвращает прежнее состояние (`True` - был включён) и восстанавливает его в `finally`.

    Блоки разных потоков считаются: сборщик включает последний вышедший из них, и только если он был включён до первого вошедшего.
    `freeze=True` (кнопка, GC_FROZEN_AFTER_BUTTON_V1; тик, GC_FROZEN_AFTER_WIDTH_TICK_V1): последний вышедший блок, пока среди открытых был такой, переносит накопленное в постоянное поколение
    перед `gc.enable()` (если сборщик включает он сам). Клапан постоянного поколения (`_freeze_or_release`) он проверяет, если его просил хоть один из этих блоков (`valve=False` - тик:
    подсчёт постоянного поколения обходит его целиком).
    """

    global _OPEN_BLOCKS, _ENABLE_AFTER_LAST, _FREEZE_ON_LAST, _VALVE_ON_LAST, _LAST_EXIT
    with _LOCK:
        was_enabled = gc.isenabled()
        if _OPEN_BLOCKS == 0:
            _ENABLE_AFTER_LAST = was_enabled
            _FREEZE_ON_LAST = _VALVE_ON_LAST = False
            if was_enabled:
                gc.disable()
        _FREEZE_ON_LAST = _FREEZE_ON_LAST or bool(freeze)
        _VALVE_ON_LAST = _VALVE_ON_LAST or (bool(freeze) and valve)
        _OPEN_BLOCKS += 1
    try:
        yield was_enabled
    finally:
        with _LOCK:
            _OPEN_BLOCKS -= 1
            if _OPEN_BLOCKS == 0:
                _LAST_EXIT = time.perf_counter()
                freezing, _FREEZE_ON_LAST = _FREEZE_ON_LAST, False
                checking, _VALVE_ON_LAST = _VALVE_ON_LAST, False
                if _ENABLE_AFTER_LAST:
                    _ENABLE_AFTER_LAST = False
                    if freezing:
                        _freeze_or_release(checking)
                    gc.enable()


def _freeze_or_release(valve: bool = True) -> None:
    """Накопленное - в постоянное поколение, пока оно не выше клапана; выше - замороженное возвращается сборщику (догонят обычные сборки). `valve=False` - без подсчёта (тик)."""

    global _FROZEN_UNCOLLECTED
    if valve and gc.get_freeze_count() > FREEZE_VALVE_OBJECTS:
        gc.unfreeze()
        _FROZEN_UNCOLLECTED = False
        return
    gc.freeze()
    _FROZEN_UNCOLLECTED = True


def collect_frozen_when_idle(quiet_seconds: float, *, busy: bool = False) -> bool:
    """GC_FROZEN_DURING_DRAG_COLLECTED_AT_IDLE_V1: в простое замороженное кнопкой и тиками возвращается сборщику, и одна полная сборка идёт здесь; `True` - собрано.

    Простой - ни одного открытого блока, вызывающий не занят (`busy`: перетаскивание или неприменённый заказ) и `quiet_seconds` с последнего выхода из паузы (между счётом такта и его
    применением, между кнопкой и подготовкой в простое после неё - не простой). Сборка идёт вне замка: её финализаторы вправе сами войти в паузу.
    """

    global _FROZEN_UNCOLLECTED
    with _LOCK:
        if busy or _OPEN_BLOCKS or not _FROZEN_UNCOLLECTED or time.perf_counter() - _LAST_EXIT < quiet_seconds:
            return False
        _FROZEN_UNCOLLECTED = False
    gc.unfreeze()
    gc.collect()
    return True


def released(failure):
    """Исключение, которое держат ЗНАЧЕНИЕМ, без трассы (`__traceback__` у него и у причин снят): тип и текст те же, цикл разорван.

    Трасса поднятого и пойманного исключения держит кадры всей цепочки вызовов (`tb_frame` -> `f_back`) с их локальными, а держатель исключения обычно
    достижим из этих же кадров: цикл, который не умирает счётчиком ссылок, ждёт сборщика, а среди замороженного (`gc.freeze`) не дождётся никогда.
    PRODUCTION_REFUSAL_WITHOUT_TRACEBACK_V1: отказ прогона (на `cover.008` пять отказов держали ~110 тыс. объектов рабочего набора `run_production`);
    CANCELLED_TICK_WITHOUT_TRACE_V1: отмена счёта такта (`envelope_alpha_preview.ThreadedPreviewJob`).
    """

    seen = set()
    node = failure
    while node is not None and id(node) not in seen:
        seen.add(id(node))
        node.__traceback__ = None
        node = node.__cause__ or node.__context__
    return failure


def gc_paused_call(function):
    """Декоратор метода оператора `(self, context)`: вызов идёт под `gc_paused`; имя, `__wrapped__` и ДВА позиционных аргумента сохранены.

    Подпись `(self, context)` обязательна, а не вкус: Blender при регистрации класса проверяет число аргументов `execute` по коду функции, и обёртка
    `(*args, **kwargs)` отказала бы `ValueError: expected Operator ... "execute" function to have 2 args, found 0`.
    """

    @functools.wraps(function)
    def wrapper(self, context):
        with gc_paused(freeze=True):
            return function(self, context)

    return wrapper


def gc_paused_function(function):
    """Декоратор любой функции, кроме метода оператора Blender: весь вызов (со всеми аргументами и значением) идёт под `gc_paused`; имя и `__wrapped__` сохранены.

    Для счёта тика ширины в потоке, его применения и затравки модели на главном потоке и подтверждения инструмента (`envelope_width_live`, `envelope_width_session`):
    пауза стоит на самой функции, поэтому и продукт, и инструменты, что зовут её напрямую, идут под ней. Выход - с заморозкой без клапана (GC_FROZEN_AFTER_WIDTH_TICK_V1, см. модуль).
    """

    @functools.wraps(function)
    def wrapper(*args, **kwargs):
        with gc_paused(freeze=True, valve=False):
            return function(*args, **kwargs)

    return wrapper


__all__ = (
    "FREEZE_VALVE_OBJECTS",
    "GC_FROZEN_OUTCOME",
    "GC_FROZEN_WIDTH_OUTCOME",
    "GC_IDLE_COLLECT_OUTCOME",
    "GC_PAUSE_OUTCOME",
    "GC_PAUSE_WIDTH_OUTCOME",
    "collect_frozen_when_idle",
    "gc_paused",
    "gc_paused_call",
    "gc_paused_function",
    "released",
)
