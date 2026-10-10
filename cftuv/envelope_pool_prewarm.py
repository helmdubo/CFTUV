"""Прогрев пула доменов: воркеры стартуют вскоре после регистрации аддона, а не на первом нажатии кнопки.

ЗАЧЕМ. Старт воркеров (интерпретатор, импорт ядра и `sympy`, `ready` каждого) стоит ~2 с СТЕНЫ на первом нажатии сеанса
(`cover.008`, 8 воркеров: 2.0-2.3 с в `pool_start`) и ни на что, кроме ожидания, не расходуется. Кнопка, нажатая через несколько секунд
после запуска Blender, находит пул готовым.

БЕЗОПАСНОСТЬ.
* Данные Blender читает ТОЛЬКО таймер на главном потоке (число воркеров сцены и путь «Worker Python»); поток, поднимающий воркеров, получает
  два простых значения и `bpy` не касается. Ни датаблока, ни операции отмены, ни записи в сцену.
* Старт идёт под замком прогона пула (`DomainPool.warm`): кнопка, нажатая во время прогрева, ждёт его конца, а не заводит вторую партию.
* Тот же пул, что у кнопки (`get_domain_pool` с теми же числом воркеров и интерпретатором): иное число воркеров на сцене к моменту нажатия
  кнопка обслужит прежним путём (пул пересоздаётся), только без выигрыша.
* В фоновом Blender (`-b`: смоки, полевые прогоны, тесты) не делает ничего, если не заказано явно (`force`).
* Отказ старта - не ошибка аддона: строка консоли с именем, а кнопка на первом нажатии назовёт тот же отказ сама (`ENVELOPE_DOMAIN_POOL_UNAVAILABLE`).
* Выключатель на машине: переменная окружения `CFTUV_POOL_PREWARM=0` (бездействующие воркеры держат около 100 МБ каждый).
"""

from __future__ import annotations

import os
import threading

#: Через сколько секунд после регистрации таймер читает настройки и заказывает старт: интерфейс успел отрисоваться.
PREWARM_DELAY_SECONDS = 2.0
PREWARM_SWITCH = "CFTUV_POOL_PREWARM"
PREWARM_UNAVAILABLE = "ENVELOPE_DOMAIN_POOL_PREWARM_UNAVAILABLE"


def prewarm_pool(workers: int, external_python: str = "") -> float:
    """Поднимает общий пул кнопки (`get_domain_pool`) и возвращает секунды старта; 0.0 - пула нет (меньше двух воркеров) либо он не поднялся."""

    from .envelope_domain_pool import DomainPoolUnavailable, get_domain_pool

    pool = get_domain_pool(workers, external_python)
    if pool is None:
        return 0.0
    try:
        return pool.warm()
    except DomainPoolUnavailable as exc:
        print(f"[CFTUV][EnvelopeDomainPool] {PREWARM_UNAVAILABLE}: {exc}; the first press starts the workers", flush=True)
        return 0.0


def _timer():
    """Таймер на главном потоке: читает настройки и отдаёт старт воркеров потоку. Однократный (`None` - не повторять)."""

    try:
        import bpy

        from .envelope_worker_python import read_worker_python

        settings = getattr(bpy.context.scene, "hotspotuv_settings", None)
        workers = int(getattr(settings, "envelope_debug_workers", 0))
        external_python = read_worker_python()
    except Exception as exc:  # noqa: BLE001 - прогрев не ломает регистрацию: кнопка стартует воркеров сама
        print(f"[CFTUV][EnvelopeDomainPool] {PREWARM_UNAVAILABLE}: settings were not read: {type(exc).__name__}: {exc}", flush=True)
        return None
    if workers >= 2:
        threading.Thread(target=prewarm_pool, args=(workers, external_python), name="cftuv-pool-prewarm", daemon=True).start()
    return None


def schedule_pool_prewarm(*, force: bool = False) -> bool:
    """Заказывает прогрев таймером Blender; `True` - заказан. Фоновый режим без `force` и `CFTUV_POOL_PREWARM=0` не заказывают ничего."""

    import bpy

    if os.environ.get(PREWARM_SWITCH, "1") == "0" or (bpy.app.background and not force):
        return False
    if bpy.app.timers.is_registered(_timer):
        return False
    bpy.app.timers.register(_timer, first_interval=PREWARM_DELAY_SECONDS)
    return True


def cancel_pool_prewarm() -> None:
    """Снимает заказанный, но не сработавший прогрев (снятие аддона); воркеров, что уже стартовали, останавливает `shutdown_domain_pool`."""

    import bpy

    if bpy.app.timers.is_registered(_timer):
        bpy.app.timers.unregister(_timer)


__all__ = ("PREWARM_DELAY_SECONDS", "PREWARM_SWITCH", "PREWARM_UNAVAILABLE", "cancel_pool_prewarm", "prewarm_pool", "schedule_pool_prewarm")
