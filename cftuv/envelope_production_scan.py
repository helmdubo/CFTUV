"""Родитель продуктового прогона: что сессия знает о домене (`_DomainEntryV1`), входы прогона (`_RunInputsV1`) и сканирование (`_scan`).

Вынесено из `envelope_production_export.py` ПЕРЕНОСОМ без изменения поведения (PRODUCTION_EXPORT_SPLIT_V1). Сканирование читает кэши
сессии БЕЗ сборки: чего в них нет, то холодно. Досчёт, пересылка воркерам и сборка итога остаются в `envelope_production_export`;
все имена по-прежнему доступны из него.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

from .envelope_chart_band import tightening_is_current_facts
from .envelope_content_key import result_slot
from .envelope_kernel_backend import (
    DEFAULT_KERNEL_BACKEND,
    backend_identity_of,
    entered_backend,
    ledger_record,
    merge_backend_records,
    settle_stage_orders,
    skeleton_identity_of,
)
from .envelope_production_result import (
    OUTCOME_PRODUCTION_CANCELLED,
    PRODUCTION_CONTENT_RELABEL_FAILED,
    PRODUCTION_SCAN_RECORDS_REUSED,
    ProductionCancelled,
)
from .envelope_scan_memo import remember_inputs
from .envelope_topology_export import metric_law_key
from .surface_ir import HOST_NEAR_PLANAR_LIFT_POLICY


@dataclass(frozen=True, slots=True)
class _DomainEntryV1:
    """Что сессия знает о домене: отказ входа, готовый результат, подготовка либо ничего.

    `inputs` — `(snapshot, request)`, когда метрика домена лежит в кэше сессии; `export` —
    лёгкий вход выгрузки, когда её там нет (снапшот тогда выгрузит воркер, вместе с
    подготовкой). `result_key` и `cached` — ключ кэша результатов и результат под ним.
    """

    patch_id: int
    domain_id: str
    selected: frozenset
    failure: object | None = None
    prepared: object | None = None
    inputs: tuple | None = None
    export: object | None = None
    result_key: tuple | None = None
    cached: object | None = None
    #: Ключ содержимого домена (`None`: не адресуется по содержимому), запись идентичностей подготовки из
    #: хранилища и что оттуда взято: `result` либо `preparation` (пусто: ничего).
    content_key: str | None = None
    labeling: object | None = None
    reuse: str = ""
    #: Готовый результат из хранилища по содержимому, ещё НЕ перенесённый на ревизию прогона: перенос — работа
    #: (воркера либо родителя), поэтому домен остаётся в `needs_work`, но не считается и не холоден.
    carried: object | None = None
    #: Журнал экспорта только этого прогона; сливается с подготовкой/материализацией один раз.
    export_records: list = field(default_factory=list, compare=False, repr=False)
    #: Вход `(снапшот, запрос)`, который родитель построил для холодного домена после ответа воркера (`_adopt_cold`): `_dispatch` пишет его в записи сборки.
    built_inputs: list = field(default_factory=list, compare=False, repr=False)
    #: WIDTH_REFRESH_INCREMENTAL_V1: `cached` - результат, сдвинутый аффинным сертификатом (`envelope_width_rebase`), а не взятый из кэша сессии: домен не считается и не сканируется.
    rebased: bool = False

    @property
    def export_record(self):
        record = None
        for part in self.export_records:
            record = merge_backend_records(record, part)
        return record

    @property
    def is_cold(self) -> bool:
        return (
            self.failure is None
            and self.prepared is None
            and self.cached is None
            and self.carried is None
        )

    @property
    def needs_work(self) -> bool:
        return self.failure is None and self.cached is None

    @property
    def computes(self) -> bool:
        """Домену нужно ПОСЧИТАТЬ ответ (а не взять готовый из кэша ревизии или из хранилища)."""

        return self.needs_work and self.carried is None


@dataclass(frozen=True, slots=True)
class _RunInputsV1:
    """Всё, что одному прогону известно о выделении, сессии и законах."""

    controller: object
    analysis_bundle: object
    topology_export: object
    revision: str
    patch_ids: tuple
    selected_by_domain: dict
    alpha: float
    alpha_text: str
    request_id: str
    density: object
    uv_policy_id: str
    topology_law: str
    hooks: object
    profile: object
    #: Что хранилище по содержимому сделало в ЭТОМ прогоне: патчи, чей результат перенесён на ревизию
    #: прогона, и переносы, которых не вышло (`(патч, причина)`).
    #: `threading.Event` заказа остановки либо `None` (кнопка не отменяется).
    cancel: object = None
    #: Главный переключатель ядра и его идентичность в ключах кэшей: результат одного бэкенда не подменяет результат другого.
    backend: str = DEFAULT_KERNEL_BACKEND
    #: Идентичности (`None` — их выводит `__post_init__` из порядков): `backend_id` несёт идентичность всех стадий (результат), `skeleton_id` — только скелета (ключ кэша подготовки сессии).
    backend_id: str | None = None
    #: Постадийные порядки (`None` — как `backend`; `__post_init__` приводит поля к именам).
    skeleton_backend: str | None = None
    skeleton_id: str | None = None
    embedding_backend: str | None = None
    relabeled: list = field(default_factory=list)
    relabel_failures: list = field(default_factory=list)
    registered: list = field(default_factory=list)
    #: `{"records": записи сборки, с которыми шёл `_scan` этого прогона (`None`: памяти нет), "key_memo": тексты общих записей для ключей содержимого этого прогона}`; повтор лотереи привязки заводит свои.
    scan_records: dict = field(default_factory=dict)
    #: WIDTH_REFRESH_LEAN_V1: прогон заказал обновление без работы, нужной только фиксации (`lean_ready` решает, какие домены его получают).
    lean: bool = False
    #: WIDTH_REFRESH_INCREMENTAL_V1: `{номер патча: результат, сдвинутый сертификатом на ширину прогона}` доменов, которых прогон не считает (`_scan` кладёт их в записи как готовые); остальные считаются как всегда.
    served: dict = field(default_factory=dict)
    #: Номера патчей, которых прогон не считает и не называет в итоге вовсе: вызывающий складывает такие домены сам (составной образец инкрементального такта, `envelope_width_composite`). Итог прогона - только остальные домены.
    skipped: frozenset = frozenset()
    #: SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1: домены, чей снапшот воркер отложил на фоновый круг кодека в ЭТОМ прогоне (по одному на каждый ответ с отложенным); общий у повтора лотереи привязки.
    byte_check_ledger: list = field(default_factory=list)

    def __post_init__(self) -> None:
        settle_stage_orders(self)
        if self.backend_id is None:
            object.__setattr__(self, "backend_id", backend_identity_of(self.backend, self.skeleton_backend, self.embedding_backend))
        if self.skeleton_id is None:
            object.__setattr__(self, "skeleton_id", skeleton_identity_of(self.skeleton_backend))

    @property
    def grid_scale_law(self):
        """Закон выбора масштаба решётки прогона (`None`: умолчание ядра; повтор после отказа лотереи привязки - `PLANE_PRESERVING_V1`): часть ключей кэшей."""

        return getattr(self.topology_export, "grid_scale_law", None)


def lean_ready(run: _RunInputsV1, entry: _DomainEntryV1) -> bool:
    """Домен получает lean-путь (WIDTH_REFRESH_LEAN_V1): прогон lean, подготовка этой ревизии уже в кэше сессии, переноса из хранилища нет.

    Перенос на ревизию читает батч (`relabel_result`), поэтому домен с переносом (`carried`, `reuse`) всегда идёт полным путём; холодный домен
    считается воркером вместе с подготовкой и тоже полным: его подготовка и результат ложатся в кэши сессии и хранилище.
    """

    return run.lean and entry.prepared is not None and entry.carried is None and not entry.reuse


def released(failure):
    """Исключение-отказ, которое прогон держит ЗНАЧЕНИЕМ, без трассы (`__traceback__` у него и у причин снят): ответ тот же, цикл разорван (PRODUCTION_REFUSAL_WITHOUT_TRACEBACK_V1).

    Трасса поднятого и пойманного исключения держит кадры (`tb_frame` -> `f_back`) всей цепочки вызовов, то есть и кадр `run_production` с его рабочим набором (задачи, ответы воркеров,
    записи доменов), а отказ лежит в этом же наборе (`refused`, `_DomainEntryV1.failure`): цикл, который не умирает счётчиком ссылок. На `cover.008` пять отказов держали ~110 тыс. объектов
    до первой сборки мусора (она шла в конце кнопки, 0.36 с). Текст, исход и домен отказа от трассы не зависят.
    """

    seen = set()
    node = failure
    while node is not None and id(node) not in seen:
        seen.add(id(node))
        node.__traceback__ = None
        node = node.__cause__ or node.__context__
    return failure


def _check_cancel(run: _RunInputsV1) -> None:
    """Точки остановки: до работы родителя над доменом и сразу после возврата пула."""

    if run.cancel is not None and run.cancel.is_set():
        raise ProductionCancelled(f"{OUTCOME_PRODUCTION_CANCELLED}: the order was superseded")


def _inputs_of(run: _RunInputsV1, entry_key, provider):
    from .envelope_queue_export import _queue_snapshot_and_request

    patch_id, domain_id, selected = entry_key
    return _queue_snapshot_and_request(
        run.analysis_bundle,
        patch_id,
        domain_id,
        selected,
        run.alpha,
        run.request_id,
        density=run.density,
        topology_export=run.topology_export,
        domain_snapshot_provider=provider,
        snapshot_issues_of=run.controller.snapshot_issues,
    )


def _result_key(run: _RunInputsV1, domain_id, selected, request) -> tuple:
    return run.controller.production_result_key(
        run.revision,
        domain_id,
        selected,
        request,
        run.alpha_text,
        run.uv_policy_id,
        run.topology_law,
        HOST_NEAR_PLANAR_LIFT_POLICY.value,
        run.backend_id,
        skeleton_id=run.skeleton_id,
        grid_scale_law=run.grid_scale_law,
    )


def _entry_with_inputs(run: _RunInputsV1, patch_id, domain_id, selected, inputs):
    """Домен с выгруженным входом: подготовка и результат из кэшей сессии, если они там есть."""

    controller = run.controller
    request = inputs[1]
    key = _result_key(run, domain_id, selected, request)
    return _DomainEntryV1(
        patch_id,
        domain_id,
        selected,
        prepared=controller.peek_conveyor_item(
            run.revision, domain_id, selected, request, skeleton_id=run.skeleton_id, grid_scale_law=run.grid_scale_law
        ),
        inputs=inputs,
        result_key=key,
        cached=controller.peek_production_result(key),
    )


def _slot(run: _RunInputsV1) -> tuple:
    """Слот результата в хранилище содержимого: alpha и законы материализации прогона."""

    return result_slot(
        run.alpha_text,
        run.uv_policy_id,
        run.topology_law,
        HOST_NEAR_PLANAR_LIFT_POLICY.value,
        run.backend_id,
    )


_NO_BINDING = object()


def _bound_entry(run: _RunInputsV1, patch_id, domain_id, selected, binding=_NO_BINDING):
    """Домен, чей ключ содержимого уже известен при этой ревизии и чей результат лежит в кэше ревизии.

    Ключ — функция входа домена, а вход при ревизии, выделении, плотности, допуске и полосе один: по привязке
    `(ревизия, домен, выделение, плотность, допуск, ключ полосы)` повторное нажатие не строит ни вход воркера, ни ключ.
    `binding` — привязка из записи сборки (`envelope_scan_memo`), когда она есть: считать её заново незачем.
    """

    if binding is _NO_BINDING:
        binding = _binding(run, patch_id, domain_id, selected)
    key = None if binding is None else run.controller.content_binding(binding)
    if key is None:
        return None
    cached = run.controller.peek_production_result(("content", key, run.revision, _slot(run)))
    if cached is None:
        return None
    return _DomainEntryV1(
        patch_id, domain_id, selected, cached=cached, content_key=key, reuse="result"
    )


def _band_key(run: _RunInputsV1, patch_id):
    """Ключ полосы домена: досягаемость запроса и выбранные рёбра патча (`None`: политики полосы нет).

    Полосу строит родитель после отказа целого патча, и она зависит ровно от этого: привязка и ключ содержимого
    несут его, поэтому смена досягаемости или выбора цепей домена при той же ревизии и тех же рёбрах домена не
    отдаёт прежний ответ.
    """

    from .envelope_metric_export import band_key_of

    return band_key_of(run.topology_export, patch_id)


def _binding(run: _RunInputsV1, patch_id, domain_id, selected):
    from .envelope_request_policy import normalize_envelope_fan_density

    try:
        density = normalize_envelope_fan_density(run.density)
    except (TypeError, ValueError):
        return None
    return (
        run.revision,
        domain_id,
        selected,
        density,
        run.topology_export.developable_stretch_budget,
        run.topology_export.silhouette_uv_slide,
        _band_key(run, patch_id),
        run.backend_id,
        *metric_law_key(run.topology_export),
    )


def _alpha_refusal(run: _RunInputsV1, facts):
    """Отказ ЗАПРОСА по alpha у домена на подготовке из хранилища: карта-полоса короче alpha прогона, либо `None`.

    Подготовка alpha-независима, но полоса действует лишь до своей досягаемости, и этот отказ строитель запроса
    даёт на КАЖДОМ нажатии (`refuse_alpha_beyond_reach`). Домен на подготовке из хранилища запроса не строит,
    поэтому тот же отказ берётся тут, по фактам карт-полос снапшота подготовки (`preparation_facts`): без него alpha выше
    досягаемости дошла бы до материализации и ответила другим, не хостовым исходом. Недопустимую alpha называет сам строитель запроса.
    """

    from .envelope_chart_band import refuse_alpha_beyond_reach_facts
    from .envelope_request_export import EnvelopeHostAdapterError
    from .envelope_request_policy import request_alpha_decimal

    try:
        refuse_alpha_beyond_reach_facts(facts, request_alpha_decimal(run.alpha))
    except EnvelopeHostAdapterError as exc:
        return released(exc)
    except (ArithmeticError, TypeError, ValueError):
        return None
    return None


def _content_entry(run: _RunInputsV1, patch_id, domain_id, selected, export) -> _DomainEntryV1:
    """Домен без метрики в кэше ревизии: из хранилища по содержимому, либо холодный с ключом.

    Правка меша меняет ревизию и сбрасывает кэши ревизии, но домен, чьё содержимое то же, имеет тот же
    ключ: готовый результат из хранилища идёт на перенос к ревизии прогона (`carried`), а готовая
    подготовка идёт воркеру как подготовка из кэша. Домен, которого в хранилище нет, считается как
    раньше — с ключом, чтобы по окончании лечь в хранилище. Домен, чей вход ключ не умеет кодировать,
    считается как раньше без ключа (счётчик `PRODUCTION_CONTENT_UNKEYED`).
    """

    from .envelope_content_key import ContentKeyUnsupported, domain_content_key

    cold = _DomainEntryV1(patch_id, domain_id, selected, export=export)
    try:
        key = domain_content_key(export, selected, _band_key(run, patch_id), run.backend, run.skeleton_backend, run.embedding_backend, run.scan_records.setdefault("key_memo", {}))
    except ContentKeyUnsupported:
        return cold
    controller = run.controller
    binding = _binding(run, patch_id, domain_id, selected)
    if binding is not None:
        controller.bind_content_key(binding, key)
    slot = _slot(run)
    cold = replace(cold, content_key=key)
    result_key = ("content", key, run.revision, slot)
    cached = controller.peek_production_result(result_key)
    if cached is not None:
        return replace(cold, export=None, cached=cached, reuse="result")
    found = controller.content_store.find(key)
    if found is None:
        return cold
    stored = controller.content_store.result(key, slot)
    if stored is not None:
        labels = stored.labels
        if labels.revision == run.revision and labels.patch_id == patch_id:
            controller.remember_production_result(result_key, stored)
            return replace(cold, export=None, cached=stored, reuse="result")
        return replace(
            cold, carried=stored, result_key=result_key, labeling=found.labeling, reuse="result"
        )
    facts = controller.preparation_facts(found.prepared)
    if not tightening_is_current_facts(run.topology_export, facts):
        # Карта подготовки сужена под ДРУГУЮ alpha: подготовка устарела (ключ содержимого от alpha не зависит), её убирают,
        # и домен считается заново с тем же ключом - новая подготовка ляжет под него по окончании.
        controller.content_store.forget(key)
        return cold
    refusal = _alpha_refusal(run, facts)
    if refusal is not None:
        return replace(cold, export=None, failure=refusal)
    return replace(
        cold,
        prepared=found.prepared,
        labeling=found.labeling,
        result_key=result_key,
        reuse="preparation",
    )


def _note_relabel_failure(run: _RunInputsV1, patch_id, exc) -> None:
    """Перенос не вышел: счётчик и строка консоли; домен при этом считается заново, а не пропадает."""

    run.relabel_failures.append((patch_id, str(exc)))
    print(
        f"[CFTUV][Production] {PRODUCTION_CONTENT_RELABEL_FAILED}: patch {patch_id}: {exc}; "
        "the domain is computed anew",
        flush=True,
    )


def _entry_from_record(run: _RunInputsV1, patch_id, domain_id, selected, record, alpha):
    """Домен по записи сборки: запрос с шириной прогона, подготовка и результат из кэшей сессии (то же, что `_entry_with_inputs`)."""

    from .envelope_scan_memo import request_at

    controller = run.controller
    key = (
        record.prep_key,
        run.alpha_text,
        run.uv_policy_id,
        run.topology_law,
        HOST_NEAR_PLANAR_LIFT_POLICY.value,
        run.backend_id,
    )
    return _DomainEntryV1(
        patch_id,
        domain_id,
        selected,
        prepared=controller.peek_conveyor_item_by_key(record.prep_key),
        inputs=(record.snapshot, request_at(record, alpha)),
        result_key=key,
        cached=controller.peek_production_result(key),
    )


class _Scan:
    """Один обход доменов прогона: состояние (`records`, счёт доменов из записей) и `entry(patch_id)` - то, что сессия знает об ОДНОМ домене (`_scan` ходит по патчам по порядку, потоковый обход - `envelope_production_stream`)."""

    __slots__ = ("run", "alpha", "records", "reused")
    def __init__(self, run: _RunInputsV1) -> None:
        from . import envelope_scan_memo as memo

        self.run = run
        self.alpha = memo.request_alpha(run.alpha)
        key = None if self.alpha is None else memo.scan_key(run)
        self.records = run.scan_records["records"] = None if key is None else run.controller.scan_memo.records_of(key)
        self.reused = 0

    def entry(self, patch_id) -> _DomainEntryV1 | None:
        """Домен патча по кэшам сессии (`None` - патч не считается этим прогоном)."""

        from .envelope_request_export import EnvelopeHostAdapterError, _typed_value

        run = self.run
        controller = run.controller
        if patch_id in run.skipped:
            return None
        domain_id = _typed_value("patch-domain", run.revision, patch_id)
        refusal = controller.byte_check_refusal(domain_id)
        if refusal is not None:
            # SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1: фон нашёл, что байты снапшота домена не прошли круг кодека; следующее действие отказывает домен тем же исходом, что дала бы проверка на нажатии.
            return _DomainEntryV1(patch_id, domain_id, frozenset(run.selected_by_domain[domain_id]), failure=refusal)
        result = run.served.get(patch_id)
        if result is not None and result.domain_id in run.selected_by_domain:
            # WIDTH_REFRESH_INCREMENTAL_V1: домен обслужен сертификатом (идентичность домена - у самого результата, он про эту ревизию), ни записей, ни кэшей, ни выгрузки.
            return _DomainEntryV1(patch_id, result.domain_id, frozenset(run.selected_by_domain[result.domain_id]), cached=result, rebased=True)
        selected = frozenset(run.selected_by_domain[domain_id])
        records = self.records
        record = None if records is None else records.get(patch_id)
        if record is not None and record.selected != selected:
            record = None
        bound = _bound_entry(run, patch_id, domain_id, selected, _NO_BINDING if record is None else record.binding)
        if bound is not None:
            return bound
        export = run.hooks.export_provider(patch_id, run.alpha, run.request_id, run.density)
        if export is not None:
            return _content_entry(run, patch_id, domain_id, selected, export)
        with entered_backend(run.backend, run.skeleton_backend, run.embedding_backend) as ledger:
            try:
                snapshot = controller.get_domain_geometry(controller.get_patch_metric(run.topology_export, patch_id)).snapshot
                if record is not None and record.snapshot is snapshot:
                    entry = _entry_from_record(run, patch_id, domain_id, selected, record, self.alpha)
                    entry.export_records.append(ledger_record(ledger))
                    self.reused += 1
                    return entry
                inputs = _inputs_of(run, (patch_id, domain_id, selected), lambda _patch, _domain: snapshot)
            except EnvelopeHostAdapterError as exc:
                return _DomainEntryV1(patch_id, domain_id, selected, failure=released(exc), export_records=[ledger_record(ledger)])
        entry = _entry_with_inputs(run, patch_id, domain_id, selected, inputs)
        entry.export_records.append(ledger_record(ledger))
        if records is not None:
            remember_inputs(records, patch_id, selected, inputs, entry.result_key[0], _binding(run, patch_id, domain_id, selected))
        return entry

    def finish(self) -> None:
        # Повтор лотереи привязки идёт тем же `_scan` в тот же профиль: счётчик копится, а не затирается числом повтора.
        self.run.profile.add_counter(PRODUCTION_SCAN_RECORDS_REUSED, self.reused)


def _scan(run: _RunInputsV1):
    """Домены по кэшам сессии, БЕЗ сборки: чего нет в кэше, то холодно (промах метрики - холодный домен с лёгким входом выгрузки: сборка метрик - работа воркеров пула).

    Вход домена с готовой записью сборки (`envelope_scan_memo`: тот же снапшот, выделение и политика) берёт из неё всё, что не зависит от ширины, а ширину подставляет в запрос; ширина, которой запрос не строится, идёт прежним путём.
    """

    scan = _Scan(run)
    entries = []
    for patch_id in run.patch_ids:
        entry = scan.entry(patch_id)
        if entry is not None:
            entries.append(entry)
    scan.finish()
    return entries
