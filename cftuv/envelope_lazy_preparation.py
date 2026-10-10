"""Подготовка холодного домена, которую родитель держит ПИКЛОМ и не разворачивает, пока живой объект не понадобился.

ЗАЧЕМ. Холодную задачу пула воркер заканчивает подготовкой, а родителю отдаёт её пикл (снят один раз, `_packed_cold_reply`).
Прежде поток-читатель родителя разворачивал каждый такой пикл сразу (`pickle.loads` под GIL, ~12 мс на домен: на `cover.008`
~12 с ЦП из ~34 с холодного нажатия), хотя живой объект у родителя почти нигде не нужен: шаг ширины шлёт воркерам тот же пикл
(`PreparationBlobsV1`), ключ памяти воркера приходит в ответе, а то, что родитель читал в снапшоте подготовки, - это факты карт-полос
(`BandFactV1`), которые воркер шлёт рядом. Живая подготовка нужна лишь там, где родитель сам считает на ней (досчёт домена, которого не
взял воркер): `EnvelopeDebugSessionController.live_preparation`.

ТОТ ЖЕ ОБЪЕКТ. Развёрнутый объект - ровно то, что получил бы прежний `_received` из этих же байтов; пока никто не считал на нём в
родителе, он равен первому разворачиванию. Разворот делает контроллер и ЗАМЕНЯЕТ ручку живым объектом везде, где её держит сессия
(кэш подготовок, хранилище по содержимому, пиклы для воркеров): тождество подготовки в каждой из этих таблиц остаётся одним.

ЯВНО, А НЕ ПРОКСИ. У ручки нет `__getattr__`: потребитель, которому нужен живой объект и который не позвал `live_preparation`, получает
`AttributeError`, а не молчаливый разворот. Ручка не пикляется и не копируется (`__reduce_ex__`): она принадлежит процессу родителя.
"""

from __future__ import annotations

#: Подготовки, которые родитель развернул из пикла воркера в ЭТОМ прогоне (досчёт домена в родителе), и ЦП потока на это (микросекунды).
#: Холодное нажатие с живым пулом держит оба нулями: пикл подготовки остаётся байтами. Пишет `run_production`, имена читают отсюда.
PRODUCTION_PREPARATIONS_UNPICKLED = "PRODUCTION_PREPARATIONS_UNPICKLED"
PRODUCTION_PREPARATION_UNPICKLE_CPU_US = "PRODUCTION_PREPARATION_UNPICKLE_CPU_US"


class LazyPreparationV1:
    """Пикл подготовки, ключ этого пикла в памяти воркеров, факты карт-полос и (после разворота) живой объект."""

    __slots__ = ("blob", "key", "facts", "live", "cache_key")

    def __init__(self, blob: bytes, key: str, facts) -> None:
        self.blob = blob
        #: Ключ пикла в памяти воркеров (`envelope_worker_store.blob_key`); пусто - подготовка не запоминается.
        self.key = key
        #: `band_facts_of(снапшот подготовки)`, снятые воркером; `None` - воркер их не прислал (потребитель разворачивает подготовку).
        self.facts = facts
        self.live = None
        #: Ключ кэша подготовок сессии, под которым лежит ручка (ставит контроллер); разворот заменяет значение под ним.
        self.cache_key = None

    def __reduce_ex__(self, protocol):
        raise TypeError("LazyPreparationV1 is process-local and is never serialised: ship its blob instead")

    def __repr__(self) -> str:
        return f"LazyPreparationV1(key={self.key[:8]!r}, bytes={len(self.blob)}, live={self.live is not None})"


def canonical(item):
    """Подготовка, под которой сессия ведёт тождество: развёрнутая ручка - её живой объект, остальное - как есть."""

    if type(item) is LazyPreparationV1 and item.live is not None:
        return item.live
    return item


def record_unpickled(profile, before: tuple, after: tuple) -> None:
    """Сколько ручек развернул родитель за прогон и во что это обошлось (`controller.lazy_unpickled` до и после)."""

    profile.set_counter(PRODUCTION_PREPARATIONS_UNPICKLED, after[0] - before[0])
    profile.set_counter(PRODUCTION_PREPARATION_UNPICKLE_CPU_US, round((after[1] - before[1]) * 1e6))


__all__ = (
    "LazyPreparationV1",
    "PRODUCTION_PREPARATIONS_UNPICKLED",
    "PRODUCTION_PREPARATION_UNPICKLE_CPU_US",
    "canonical",
    "record_unpickled",
)
