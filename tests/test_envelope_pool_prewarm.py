"""Прогрев пула: воркеры стартуют заранее, безопасно и только там, где это заказано (хост без Blender).

Утверждений пять: (1) прогрев поднимает ТОТ ЖЕ пул, что берёт кнопка, и кнопка находит воркеров готовыми; (2) в фоновом Blender, без
заказа и при выключателе окружения он не делает ничего; (3) таймер читает настройки на главном потоке, а старт отдаёт потоку и не
стартует пул из меньше чем двух воркеров; (4) отказ прогрева — строка с именем, а не падение регистрации; (5) тот же поток считает отпечаток
кода процесса, который иначе платило бы первое нажатие (результат тот же, отказ - строка с именем).
"""

from __future__ import annotations

import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_domain_pool as pool_module  # noqa: E402
from cftuv import envelope_pool_prewarm as prewarm  # noqa: E402
from cftuv.envelope_domain_pool import DomainPoolUnavailable, get_domain_pool, shutdown_domain_pool  # noqa: E402


class FakeTimers:
    def __init__(self):
        self.registered = {}

    def is_registered(self, function):
        return function in self.registered

    def register(self, function, first_interval=0.0, **_):
        self.registered[function] = first_interval

    def unregister(self, function):
        del self.registered[function]


def fake_blender(monkeypatch, *, background, workers=4, scene=True):
    timers = FakeTimers()
    settings = SimpleNamespace(envelope_debug_workers=workers)
    context = SimpleNamespace(scene=SimpleNamespace(hotspotuv_settings=settings) if scene else None)
    fake = SimpleNamespace(app=SimpleNamespace(background=background, timers=timers), context=context)
    monkeypatch.setitem(sys.modules, "bpy", fake)
    monkeypatch.delenv(prewarm.PREWARM_SWITCH, raising=False)
    return timers


@pytest.fixture(autouse=True)
def _no_pool_outlives_the_test():
    yield
    shutdown_domain_pool()


def test_the_prewarm_starts_the_very_pool_the_button_takes_and_the_button_finds_it_ready():
    seconds = prewarm.prewarm_pool(2, "")

    pool = get_domain_pool(2, "")
    assert seconds > 0.0 and pool.worker_count == 2
    first = list(pool._workers)  # noqa: SLF001 - воркеры прогрева
    assert pool.warm() < 1.0 and list(pool._workers) == first  # второй прогрев ничего не стартует
    assert get_domain_pool(2, "") is pool, "the press takes the same pool"


def test_the_prewarm_computes_the_code_identity_that_the_press_would_compute():
    from cftuv import envelope_content_key as key

    key._fingerprint_once.cache_clear()  # noqa: SLF001
    assert key._fingerprint_once.cache_info().currsize == 0  # noqa: SLF001

    prewarm.prewarm_pool(2, "")

    assert key._fingerprint_once.cache_info().currsize == 2, "the kernel package and the host package are fingerprinted"  # noqa: SLF001
    hits_before = key._fingerprint_once.cache_info().hits  # noqa: SLF001
    expected = key.code_identity()
    assert key._fingerprint_once.cache_info().hits == hits_before + 2, "the press takes the fingerprints from the cache"  # noqa: SLF001
    key._fingerprint_once.cache_clear()  # noqa: SLF001
    assert key.code_identity() == expected, "the same fingerprint however it is reached"


def test_a_code_identity_that_cannot_be_computed_is_a_named_line_and_the_prewarm_still_returns(monkeypatch, capsys):
    from cftuv import envelope_content_key as key

    def broken():
        raise key.ContentKeyUnsupported("the kernel is not importable")

    monkeypatch.setattr(key, "code_identity", broken)

    seconds = prewarm.prewarm_pool(2, "")

    assert seconds > 0.0 and get_domain_pool(2, "").worker_count == 2
    assert prewarm.IDENTITY_PREWARM_UNAVAILABLE in capsys.readouterr().out


def test_a_pool_that_did_not_start_computes_no_identity(monkeypatch):
    pool = get_domain_pool(2, "")
    monkeypatch.setattr(pool, "ensure_started", lambda: (_ for _ in ()).throw(DomainPoolUnavailable("no interpreter")))
    monkeypatch.setattr(prewarm, "warm_code_identity", lambda: pytest.fail("a failed prewarm is not followed by the identity"))

    assert prewarm.prewarm_pool(2, "") == 0.0


def test_a_pool_of_fewer_than_two_workers_is_never_started():
    assert prewarm.prewarm_pool(1, "") == 0.0 and prewarm.prewarm_pool(0, "") == 0.0
    assert pool_module._POOL is None  # noqa: SLF001


def test_a_warm_that_lost_its_pool_leaves_no_workers_behind(monkeypatch):
    pool = get_domain_pool(2, "")
    real = pool.ensure_started

    def shut_down_then_started():
        shutdown_domain_pool()  # пул сняли (снятие аддона), а прогрев всё же стартует воркеров на снятом пуле
        real()

    monkeypatch.setattr(pool, "ensure_started", shut_down_then_started)

    pool.warm()

    assert pool.worker_count == 0


def test_a_prewarm_that_cannot_start_is_a_named_line_and_not_a_crash(monkeypatch, capsys):
    pool = get_domain_pool(2, "")
    monkeypatch.setattr(pool, "ensure_started", lambda: (_ for _ in ()).throw(DomainPoolUnavailable("no interpreter")))

    assert prewarm.prewarm_pool(2, "") == 0.0

    assert prewarm.PREWARM_UNAVAILABLE in capsys.readouterr().out


def test_background_blender_and_the_environment_switch_order_nothing(monkeypatch):
    timers = fake_blender(monkeypatch, background=True)

    assert prewarm.schedule_pool_prewarm() is False and not timers.registered
    assert prewarm.schedule_pool_prewarm(force=True) is True and list(timers.registered) == [prewarm._timer]  # noqa: SLF001
    assert prewarm.schedule_pool_prewarm(force=True) is False, "one order at a time"
    prewarm.cancel_pool_prewarm()
    assert not timers.registered
    prewarm.cancel_pool_prewarm()  # без заказа не делает ничего

    timers = fake_blender(monkeypatch, background=False)
    monkeypatch.setenv(prewarm.PREWARM_SWITCH, "0")
    assert prewarm.schedule_pool_prewarm() is False and not timers.registered
    monkeypatch.delenv(prewarm.PREWARM_SWITCH)
    assert prewarm.schedule_pool_prewarm() is True and timers.registered[prewarm._timer] == prewarm.PREWARM_DELAY_SECONDS  # noqa: SLF001


def test_the_timer_reads_blender_on_its_own_thread_and_hands_two_plain_values_to_the_starter(monkeypatch):
    fake_blender(monkeypatch, background=False, workers=3)
    monkeypatch.setattr("cftuv.envelope_worker_python.read_worker_python", lambda: "C:/python/python.exe")
    seen = []
    done = threading.Event()

    def starter(workers, external):
        seen.append((workers, external, threading.current_thread() is threading.main_thread()))
        done.set()

    monkeypatch.setattr(prewarm, "prewarm_pool", starter)

    assert prewarm._timer() is None  # noqa: SLF001 - однократный таймер
    assert done.wait(5.0) and seen == [(3, "C:/python/python.exe", False)]


@pytest.mark.parametrize("workers", [0, 1])
def test_the_timer_starts_nothing_for_a_sequential_scene(monkeypatch, workers):
    fake_blender(monkeypatch, background=False, workers=workers)
    called = []
    monkeypatch.setattr(prewarm, "prewarm_pool", lambda *args: called.append(args))

    assert prewarm._timer() is None  # noqa: SLF001
    assert not called


def test_a_scene_without_settings_starts_nothing_and_a_context_that_cannot_be_read_is_a_named_line(monkeypatch, capsys):
    fake_blender(monkeypatch, background=False, scene=False)
    monkeypatch.setattr(prewarm, "prewarm_pool", lambda *args: pytest.fail("nothing to start"))

    assert prewarm._timer() is None  # noqa: SLF001
    assert capsys.readouterr().out == ""

    class Broken:
        @property
        def scene(self):
            raise RuntimeError("restricted context")

    sys.modules["bpy"].context = Broken()

    assert prewarm._timer() is None  # noqa: SLF001
    assert prewarm.PREWARM_UNAVAILABLE in capsys.readouterr().out
