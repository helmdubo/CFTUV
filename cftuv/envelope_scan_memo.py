"""Записи сборки входов домена продуктового прогона по `(ревизия, выделение, политика)`: ширина меняет в них одно поле.

ЗАЧЕМ. Прогон ширины (шаг ползунка) на КАЖДОМ шаге заново собирал вход каждого домена (`envelope_production_export._scan`):
токены идентичностей, ключ полосы, запрос (`build_envelope_decal_request` с проверкой снапшота) и ключ подготовки по его
подписи, хотя при том же выделении и тех же допусках запрос отличается от прежнего ровно полем `requested_alpha`
(идентичность запроса, выбранные использования цепей и политика углов от ширины не зависят), а подготовка домена
alpha-независима. Здесь первая сборка домена запоминается записью `ScanRecordV1`, а последующие шаги берут из неё
привязку, ключ подготовки и запрос с подставленной шириной.

ЭТО НЕ ЭВРИСТИКА. Запись принимается, лишь когда ЖИВОЙ снапшот домена — тот же объект (`is`), что в записи: кэши метрики и
геометрии сессии отдали его же, а запрос и ключ подготовки — функции снапшота, выделения и политики, названных в ключе
прогона. Запрос с подставленной шириной равен собранному заново (`dataclasses.replace` одного поля; тест сверяет с
`build_envelope_decal_request`). Всё, что в сборке от ширины зависит, остаётся за прогоном: допустимость ширины (конечная,
не отрицательная) проверяется здесь на каждом шаге, а домен, чей снапшот несёт полосовую карту, запись не получает вовсе:
отказ запроса по досягаемости полосы (`REQUEST_ALPHA_EXCEEDS_CHART_REACH`) и сверка полосы с политикой запроса читают ширину,
и такой домен идёт прежним путём на каждом шаге. Память живёт и сбрасывается с кэшами ревизии сессии и ограничена числом
прогонов (`SCAN_MEMO_RUN_LIMIT`).
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, replace
from decimal import Decimal, InvalidOperation

#: Сколько прогонов (выделение и политика) держит память: ширина меняется при одном, к двум-трём возвращает отмена.
SCAN_MEMO_RUN_LIMIT = 4


@dataclass(frozen=True, slots=True)
class ScanRecordV1:
    """Вход домена, собранный первым шагом: всё, что от ширины не зависит."""

    selected: frozenset
    snapshot: object
    #: Запрос первой сборки; ширину в нём заменяет `request_at`.
    request: object
    #: Ключ подготовки (`EnvelopeDebugSessionController._preparation_key`): ревизия, домен, выделение, подпись политики запроса.
    prep_key: tuple
    #: Привязка ключа содержимого (`_binding`) либо `None` (плотность не нормализуется).
    binding: tuple | None


def request_alpha(alpha) -> Decimal | None:
    """Ширина запроса как её читает построитель запроса (`Decimal(str(float(alpha)))`), либо `None`, если она недопустима."""

    try:
        value = Decimal(str(float(alpha)))
    except (InvalidOperation, ValueError, OverflowError):
        return None
    return value if value.is_finite() and value >= 0 else None


def request_at(record: ScanRecordV1, alpha: Decimal):
    """Запрос записи при ширине `alpha`: единственное поле, в котором он от ширины зависит."""

    from .envelope_request_export import _load_kernel

    kernel, _ = _load_kernel()
    return replace(record.request, requested_alpha=kernel.LocalLengthV1(alpha))


def carries_band(snapshot) -> bool:
    """В снапшоте есть карта-полоса: её проверки читают ширину запроса, и запись для такого домена не заводится."""

    from .envelope_chart_band import band_certificate_type

    band = band_certificate_type()
    return any(
        type(getattr(metric, "planarity_certificate", None)) is band for metric in snapshot.surface_metric_descriptors
    )


class ScanMemoV1:
    """Записи входов доменов по ключу прогона; вытеснение по давности прогона."""

    def __init__(self) -> None:
        self._runs: OrderedDict[tuple, dict[int, ScanRecordV1]] = OrderedDict()
        #: Выключенная память записей не заводит и не отдаёт (сверка «с памятью и без», замер «до»).
        self.enabled = True

    def __len__(self) -> int:
        return sum(len(records) for records in self._runs.values())

    def clear(self) -> None:
        self._runs.clear()

    def records_of(self, key: tuple) -> dict[int, ScanRecordV1] | None:
        if not self.enabled:
            return None
        records = self._runs.get(key)
        if records is None:
            records = self._runs[key] = {}
            while len(self._runs) > SCAN_MEMO_RUN_LIMIT:
                self._runs.popitem(last=False)
        else:
            self._runs.move_to_end(key)
        return records


def scan_key(run) -> tuple | None:
    """Ключ прогона: код, ревизия, запрос (он несёт выделение), плотность, допуски и идентичность стадии скелета; `None` — ключа нет (память не работает)."""

    from .envelope_content_key import ContentKeyUnsupported, code_identity
    from .envelope_request_policy import normalize_envelope_fan_density, topology_chart_reach_cap

    try:
        density = normalize_envelope_fan_density(run.density)
        code = code_identity()
    except (TypeError, ValueError, ContentKeyUnsupported):
        return None
    export = run.topology_export
    return (
        code,
        run.revision,
        run.request_id,
        density,
        export.developable_stretch_budget,
        export.silhouette_uv_slide,
        topology_chart_reach_cap(export),
        # запись несёт ключ подготовки, а он — идентичность стадии скелета: запись, снятая под другим скелетом, не принимается
        getattr(run, "skeleton_id", ""),
    )
