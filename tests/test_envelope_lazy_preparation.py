"""Подготовка холодного домена у родителя - пикл с фактами, а не развёрнутый объект: тождество в таблицах сессии и факты карт-полос.

Утверждений четыре, и каждое проверяет то, что иначе осталось бы словами в докстроке:

1. РУЧКА ПРИНАДЛЕЖИТ ПРОЦЕССУ. Её нельзя запиклить и скопировать; у неё нет `__getattr__`, поэтому потребитель, которому нужен живой объект,
   падает громко, а не разворачивает молча.
2. ТОЖДЕСТВО ОДНО. Пикл для воркеров снимается с ручки как есть (без `pickle.dumps`); хранилище по содержимому держит ручку, а после разворота -
   её объект, под тем же ключом.
3. ПИКЛ ЖИВЁТ, ПОКА ПОДГОТОВКУ ДЕРЖИТ КЭШ СЕССИИ. Предел хранилища по содержимому вытесняет записи (256 на 1051 домен `cover.008`), но пикл подготовки,
   которую всё ещё держит кэш подготовок, не уходит: первый шаг ширины перепикливал их все (~6 с родителя).
4. ФАКТЫ КАРТ-ПОЛОС РАВНЫ ЧТЕНИЮ СНАПШОТА. Решения, которые родитель принимал по `prepared.context.snapshot` (суженная досягаемость, отказ по alpha),
   теперь идут по фактам, снятым воркером; сверка со слово-в-слово копией прежних функций на снапшотах всех видов.
"""

from __future__ import annotations

import copy
import pickle
import sys
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_queue_pool  # noqa: E402
from cftuv.envelope_chart_band import (  # noqa: E402
    band_certificate_type,
    band_facts_of,
    refuse_alpha_beyond_reach,
    refuse_alpha_beyond_reach_facts,
    tightened_cap_of,
    tightened_cap_of_facts,
    tightening_is_current,
    tightening_is_current_facts,
)
from cftuv.envelope_content_store import ContentStoreV1  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_lazy_preparation import LazyPreparationV1, canonical  # noqa: E402
from cftuv.envelope_queue_pool import PreparationBlobsV1  # noqa: E402
from cftuv.envelope_request_export import EnvelopeHostAdapterError  # noqa: E402


class Prep:
    """Подготовка-заглушка, которую можно запиклить."""

    def __init__(self, tag):
        self.tag = tag


def handle_of(tag, key="key"):
    return LazyPreparationV1(pickle.dumps(Prep(tag), protocol=5), key, ())


# --------------------------------------------------------------------------
# 1. Ручка принадлежит процессу
# --------------------------------------------------------------------------


def test_the_handle_is_never_serialised_and_never_pretends_to_be_the_preparation():
    handle = handle_of("a")

    for attempt in (lambda: pickle.dumps(handle), lambda: copy.copy(handle), lambda: copy.deepcopy(handle)):
        with pytest.raises(TypeError, match="process-local"):
            attempt()
    with pytest.raises(AttributeError):
        handle.context  # noqa: B018 - живой объект нужен: контроллер разворачивает его явно (`live_preparation`)
    assert canonical(handle) is handle and canonical(None) is None and canonical(7) == 7
    handle.live = Prep("live")
    assert canonical(handle) is handle.live


# --------------------------------------------------------------------------
# 2. Тождество одно: пикл для воркеров, хранилище по содержимому, разворот
# --------------------------------------------------------------------------


def test_the_blob_of_a_handle_is_its_own_bytes_and_the_pickler_is_never_called(monkeypatch):
    handle = handle_of("a", "k-a")
    blobs = PreparationBlobsV1()
    monkeypatch.setattr(envelope_queue_pool.pickle, "dumps", lambda *a, **k: pytest.fail("a handle's blob must not be pickled again"))

    assert blobs.blob_of(handle) is handle.blob and blobs.key_of(handle) == "k-a" and len(blobs) == 1
    blobs.discard(handle)
    assert len(blobs) == 0


def test_developing_a_handle_moves_the_content_store_entry_and_the_blob_to_the_live_object():
    controller = EnvelopeDebugSessionController()
    store, blobs = controller.content_store, controller.preparation_blobs
    handle = handle_of("a", "k-a")
    labeling = object()
    store.register_preparation("content-a", handle, labeling)
    blobs.blob_of(handle)
    assert store.holds(handle) and store.key_of(handle) == ("content-a", store.find("content-a"))

    live = controller.live_preparation(handle)

    assert live.tag == "a" and handle.live is live and controller.live_preparation(handle) is live and controller.lazy_unpickled[0] == 1
    assert store.find("content-a").prepared is live and store.holds(live) and store.holds(handle)  # ручка ведёт к тому же объекту
    assert store.key_of(live)[0] == store.key_of(handle)[0] == "content-a"
    assert blobs.blob_of(live) is handle.blob and blobs.blob_of(handle) is handle.blob and len(blobs) == 1
    assert controller.live_preparation(None) is None and controller.live_preparation(live) is live
    store.forget("content-a")
    assert not store.holds(live) and len(store) == 0


def test_a_handle_the_session_no_longer_holds_still_develops_into_the_same_object():
    controller = EnvelopeDebugSessionController()
    handle = handle_of("a")
    handle.cache_key = ("stale",)  # в кэше подготовок под этим ключом лежит не она (вытеснена либо заменена)

    first = controller.live_preparation(handle)

    assert first.tag == "a" and controller.live_preparation(handle) is first and controller.lazy_unpickled[0] == 1


# --------------------------------------------------------------------------
# 3. Пикл живёт, пока подготовку держит кэш подготовок
# --------------------------------------------------------------------------


def test_the_blob_outlives_the_content_store_entry_while_the_session_cache_holds_the_preparation(monkeypatch):
    from cftuv import envelope_content_store

    monkeypatch.setattr(envelope_content_store, "CONTENT_STORE_ENTRY_LIMIT", 1)
    controller = EnvelopeDebugSessionController()
    first, second = Prep("first"), Prep("second")
    for name, prepared in (("first", first), ("second", second)):
        controller._conveyor_preparation_cache[(name,)] = prepared  # noqa: SLF001 - кэш подготовок, как его пополняет `get_conveyor_item`
        controller.preparation_blobs.blob_of(prepared)

    controller.content_store.register_preparation("a", first, object())
    controller.content_store.register_preparation("b", second, object())  # предел 1: запись «a» вытеснена, хранилище зовёт `on_forget`

    assert not controller.content_store.holds(first) and controller.content_store.holds(second)
    assert len(controller.preparation_blobs) == 2, "the session cache still holds `first`: its blob stays for the width step"
    controller._conveyor_preparation_cache.clear()  # noqa: SLF001
    controller._forget_preparation_blob(first)  # noqa: SLF001
    assert len(controller.preparation_blobs) == 1, "no cache, no store entry: the blob goes"


# --------------------------------------------------------------------------
# 4. Факты карт-полос равны чтению снапшота
# --------------------------------------------------------------------------


def _legacy_tightened_cap_of(snapshot):
    """Прежний `tightened_cap_of` слово в слово."""

    for metric in snapshot.surface_metric_descriptors:
        certificate = getattr(metric, "planarity_certificate", None)
        if type(certificate) is band_certificate_type() and certificate.tightened is not None:
            return Fraction(certificate.reach_cap.numerator, certificate.reach_cap.denominator)
    return None


def _legacy_refusal(snapshot, alpha_decimal):
    """Прежний `refuse_alpha_beyond_reach` слово в слово: имя исхода, текст и домен отказа либо `None`."""

    from cftuv.envelope_request_export import EnvelopeDebugHostOutcome

    for metric in snapshot.surface_metric_descriptors:
        certificate = getattr(metric, "planarity_certificate", None)
        if type(certificate) is not band_certificate_type():
            continue
        cap = Fraction(certificate.reach_cap.numerator, certificate.reach_cap.denominator)
        if certificate.excluded_triangle_count and Fraction(alpha_decimal) > cap:
            return (
                EnvelopeDebugHostOutcome.REQUEST_ALPHA_EXCEEDS_CHART_REACH,
                f"alpha={float(alpha_decimal):.6g} m is beyond the chart reach cap {float(cap):.6g} m of the band chart: "
                "the whole Patch does not unfold, and the band around the selected chains is valid up to the cap",
                metric.patch_domain_id.value,
            )
    return None


def _band(domain, reach, *, tightened, excluded):
    certificate = object.__new__(band_certificate_type())
    for name, value in (
        ("reach_cap", Fraction(reach)),
        ("tightened", object() if tightened else None),
        ("excluded_triangle_count", 3 if excluded else 0),
    ):
        object.__setattr__(certificate, name, value)
    return SimpleNamespace(planarity_certificate=certificate, patch_domain_id=SimpleNamespace(value=domain))


def _plain(domain):
    return SimpleNamespace(planarity_certificate=None, patch_domain_id=SimpleNamespace(value=domain))


SNAPSHOTS = {
    "no metrics": (),
    "no band": (_plain("d0"),),
    "a band under the request reach": (_plain("d0"), _band("d1", "1/2", tightened=False, excluded=False)),
    "a band with discarded triangles": (_band("d1", "1/2", tightened=False, excluded=True),),
    "a tightened band": (_plain("d0"), _band("d1", "3/10", tightened=True, excluded=True)),
    "two bands, the second one is tightened": (_band("d1", "1/2", tightened=False, excluded=True), _band("d2", "1/5", tightened=True, excluded=True)),
    "two tightened bands": (_band("d1", "1/4", tightened=True, excluded=False), _band("d2", "1/5", tightened=True, excluded=True)),
}
ALPHAS = ("0.05", "0.2", "0.25", "0.3", "0.5", "0.51", "4")


@pytest.mark.parametrize("name", SNAPSHOTS)
def test_the_facts_of_a_snapshot_decide_exactly_what_its_reading_decided(name):
    from decimal import Decimal

    snapshot = SimpleNamespace(surface_metric_descriptors=SNAPSHOTS[name])
    facts = band_facts_of(snapshot)

    assert tightened_cap_of(snapshot) == _legacy_tightened_cap_of(snapshot) == tightened_cap_of_facts(facts)
    assert bool(facts) == any(item.planarity_certificate is not None for item in SNAPSHOTS[name])
    for alpha in ALPHAS:
        expected = _legacy_refusal(snapshot, Decimal(alpha))
        for refuse in (lambda: refuse_alpha_beyond_reach(snapshot, Decimal(alpha)), lambda: refuse_alpha_beyond_reach_facts(facts, Decimal(alpha))):
            try:
                refuse()
            except EnvelopeHostAdapterError as exc:
                assert expected == (exc.outcome, str(exc), exc.patch_domain_id), (name, alpha)
            else:
                assert expected is None, (name, alpha)
    # Без политики полосы суженная карта устарела всегда, не суженная годна всегда: тот же ответ по снапшоту и по фактам.
    for topology in (SimpleNamespace(chart_band=None),):
        assert tightening_is_current(topology, snapshot) is tightening_is_current_facts(topology, facts) is (_legacy_tightened_cap_of(snapshot) is None)


def test_the_facts_are_plain_values_that_survive_the_pipe():
    facts = band_facts_of(SimpleNamespace(surface_metric_descriptors=SNAPSHOTS["two bands, the second one is tightened"]))

    assert pickle.loads(pickle.dumps(facts, protocol=5)) == facts
    assert [item.domain_id for item in facts] == ["d1", "d2"] and facts[1].tightened and not facts[0].tightened
