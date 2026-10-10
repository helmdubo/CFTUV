"""Проверка снапшота домена, сделанная воркером, и её принятие родителем (ADOPT_REUSES_WORKER_SNAPSHOT_ISSUES_V1).

Холодный домен пула выгружает снапшот ВОРКЕР и проверяет его там же: `build_envelope_decal_request` зовёт
`validate_analysis_snapshot(снапшот, developable_stretch_budget=<допуск запроса>)`, и непустой ответ - отказ домена.
Родитель принимает снапшот как метрику домена и собирает запрос на нём заново, а память замечаний сессии ключуется
ОБЪЕКТОМ снапшота: пришедший по трубе объект новый, и та же проверка шла второй раз. На кривых патчах это пересборка
карты развёртки в родителе (1.9 с из 5.7 с кнопки `rounded_wall_noise_top`, 1.65 с из 28 с `cover.008`).

Родитель пропускает проверку, только когда это ТА ЖЕ проверка ТОГО ЖЕ снапшота, и каждый пункт ниже проверяется кодом:

* ЧИСТО, А НЕ «ЕСТЬ ЗАМЕЧАНИЯ». Воркер кладёт в ответ `SnapshotCleanV1` лишь когда `validate_analysis_snapshot` вернула ПУСТОЙ
  кортеж. Непустые замечания не едут: их порядок зависит от обхода `frozenset`, а он у процессов разный. Запрос с ними отказан,
  и родитель повторяет проверку и получает отказ сам.
* ТОТ ЖЕ ВЫЗОВ. Воркер зовёт ту же функцию с тем же аргументом, что и родитель (`budget_key` - единственное преобразование
  допуска у обоих), и записывает допуск в `SnapshotCleanV1.budget`. Память родителя ключуется `(объект снапшота, допуск)`:
  запрос с другим допуском не находит запись и проверяет сам.
* ТОТ ЖЕ СНАПШОТ. Запись едет в одном объекте ответа со снапшотом, который воркер проверил и вернул (замороженные записи, после
  проверки он не менялся); родитель принимает запись только вместе с ЭТИМ объектом ответа и только если слепок (`snapshot_witness`:
  ревизия источника, домены, размеры всех множеств) сошёлся со слепком воркера и с тем, что родитель заказал (ревизия и домен).
  Слепок не доказательство равенства содержимого, а сторож от подмены и обрыва; равенство даёт пикл замороженных значений
  (`Decimal`, `Fraction`, перечисления, кортежи и множества восстанавливаются равными), и тест сверяет его на настоящих снапшотах.
* ТОТ ЖЕ КОД. Идентичность исполнения (`active_execution`: отпечатки ядра и хоста плюс бэкенды всех стадий с `native_build_id`)
  снимается воркером там, где шла проверка, и родителем там, где она шла бы; не равны - проверка идёт как раньше.

Любое иное - проверка как раньше, а исход назван счётчиком профиля (`SNAPSHOT_CHECK_COUNTERS`): принятая и отвергнутая с причиной.
Принимает только продуктовый путь (`worker_export_hooks`): память замечаний читает он, а профиль отладки от размещения не зависит.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from fractions import Fraction

SNAPSHOT_CHECK_ADOPTED = "PRODUCTION_SNAPSHOT_CHECK_ADOPTED"
#: Причины, по которым запись воркера не принята (каждая - счётчик профиля): записи нет, исполнение другое, слепок снапшота не
#: сошёлся, слепок называет другие ревизию либо домен, чем заказал родитель.
REASON_NOT_ATTESTED = "NOT_ATTESTED"
REASON_EXECUTION_DIFFERS = "EXECUTION_DIFFERS"
REASON_SNAPSHOT_DIFFERS = "SNAPSHOT_DIFFERS"
REASON_DOMAIN_DIFFERS = "DOMAIN_DIFFERS"
#: Исход принятия (`""` - принята, иначе причина) -> счётчик профиля прогона.
SNAPSHOT_CHECK_COUNTERS = {
    "": SNAPSHOT_CHECK_ADOPTED,
    **{
        reason: f"PRODUCTION_SNAPSHOT_CHECK_{reason}"
        for reason in (REASON_NOT_ATTESTED, REASON_EXECUTION_DIFFERS, REASON_SNAPSHOT_DIFFERS, REASON_DOMAIN_DIFFERS)
    },
}


@dataclass(frozen=True, slots=True)
class SnapshotCleanV1:
    """`validate_analysis_snapshot(снапшот, developable_stretch_budget=budget)` вернула пустой кортеж, исполнение `execution`, слепок `witness`."""

    budget: Fraction | None
    execution: tuple
    witness: tuple


def budget_key(stretch_budget) -> Fraction | None:
    """Допуск растяжения как ключ проверки: `None` - допуск самого снапшота, иначе точная дробь (любое число с `numerator`/`denominator`)."""

    return None if stretch_budget is None else Fraction(stretch_budget.numerator, stretch_budget.denominator)


def _sizes(record) -> tuple:
    return tuple(len(value) for item in fields(record) if isinstance(value := getattr(record, item.name), frozenset))


def snapshot_witness(snapshot) -> tuple:
    """Слепок снапшота: `(ревизия источника, номера доменов, размеры множеств снапшота, размеры множеств его поверхности)`."""

    return (
        snapshot.source_revision.value,
        tuple(sorted(item.patch_domain_id.value for item in snapshot.patch_domains)),
        _sizes(snapshot),
        _sizes(snapshot.surface_ir),
    )


def active_execution() -> tuple | None:
    """Идентичность исполнения ЭТОГО потока сейчас: отпечатки ядра и хоста, бэкенды стадий блока `use_backend`; `None`, если её не определить."""

    try:
        from cftuv_envelope import backend

        from .envelope_content_key import ContentKeyUnsupported, execution_identity
    except ImportError:
        return None
    try:
        return execution_identity(
            backend.active_backend().value,
            backend.active_skeleton_backend().value,
            backend.active_embedding_backend().value,
        )
    except ContentKeyUnsupported:
        return None


class WorkerSnapshotCheck:
    """`snapshot_issues_of` воркера: проверка, которую `validate_snapshot_request_references` сделала бы сама, плюс запись чистого ответа.

    Считает РОВНО ОДИН раз (вместо внутреннего вызова) и отдаёт замечания запросу как есть; `clean` - запись, если их нет.
    """

    __slots__ = ("clean",)

    def __init__(self) -> None:
        self.clean: SnapshotCleanV1 | None = None

    def __call__(self, snapshot, stretch_budget) -> tuple:
        from .envelope_request_export import _load_kernel

        kernel, _ = _load_kernel()
        budget = budget_key(stretch_budget)
        issues = tuple(kernel.validate_analysis_snapshot(snapshot, developable_stretch_budget=budget))
        execution = None if issues else active_execution()
        self.clean = None if execution is None else SnapshotCleanV1(budget, execution, snapshot_witness(snapshot))
        return issues


def refusal_reason(snapshot, clean: SnapshotCleanV1 | None, *, revision: str, domain_id: str) -> str:
    """`""` - запись воркера принимается для этого снапшота; иначе имя причины (`REASON_*`).

    `revision` и `domain_id` - то, что родитель заказал воркеру: слепок снапшота обязан называть именно их.
    """

    if clean is None:
        return REASON_NOT_ATTESTED
    if clean.execution != active_execution():
        return REASON_EXECUTION_DIFFERS
    witness = snapshot_witness(snapshot)
    if witness != clean.witness:
        return REASON_SNAPSHOT_DIFFERS
    if witness[0] != revision or witness[1] != (domain_id,):
        return REASON_DOMAIN_DIFFERS
    return ""


__all__ = (
    "REASON_DOMAIN_DIFFERS",
    "REASON_EXECUTION_DIFFERS",
    "REASON_NOT_ATTESTED",
    "REASON_SNAPSHOT_DIFFERS",
    "SNAPSHOT_CHECK_ADOPTED",
    "SNAPSHOT_CHECK_COUNTERS",
    "SnapshotCleanV1",
    "WorkerSnapshotCheck",
    "active_execution",
    "budget_key",
    "refusal_reason",
    "snapshot_witness",
)
