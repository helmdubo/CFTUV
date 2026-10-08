"""Живое превью меша ширины: склейка приблизительной модели (`envelope_width_preview_model`) с Blender, сессией и планировщиками.

ЧТО ВИДИТ ВЛАДЕЛЕЦ. Пока он тянет ширину (модальный «Adjust Decal Width» либо ползунок), НАСТОЯЩИЙ меш декали двигается каждый кадр: позиции
и UV пишутся в существующий меш из модели, а точный пересчёт идёт в фоне через тот же планировщик, что и прежде, и его результат
заменяет превью. Строка статуса называет превью (`PREVIEW_MESH_FROM_INTERVAL_V1`, «approximate, not certified»), придержанные и
карантинные домены. Сертифицирован только интервал событий и структуры у ядра (R1); модель внутри него — эвристика (шапка модуля модели).

ОТКУДА МОДЕЛЬ. Точные прогоны, которые уже посчитаны: на экране лежит ОБРАЗЕЦ (`PreviewSampleV1`: массивы меша и домены прогона, чья
геометрия сейчас в меше), рядом — до трёх других точных образцов. Модель строит поток точного счёта (`finish_live_run`), НЕ главный
поток: numpy над массивами меша, домен за доменом. Первую модель после кнопки даёт «затравка» (`prime`): отдельный точный прогон на
ширине рядом (`PRIME_RELATIVE_STEP`), который в меш НЕ пишется (его результат — образец, а не отображаемая ширина); он запускается
инструментом при входе (`begin_adjust`), а не кнопкой: кнопка не считает ничего сверх обещанного. Каждый следующий точный результат даёт
модель сам: прежний образец на экране становится вторым образцом новой. Тот же поток судит прежнюю модель новым точным прогоном
(`deviation`) и ведёт журнал доверия (`advance_ledger`): опровергнутый домен в карантине, возврат — по правилу (шапка модуля модели).

ЧТО ПИШЕТСЯ И ГДЕ. Только позиции и UV, только в СУЩЕСТВУЮЩИЙ меш (`write_preview_geometry`: `foreach_set`, без пересоздания датаблоков, поэтому
таймеры и модальный оператор не ломают шаг отмены).

КОМУ ПРИНАДЛЕЖИТ МЕШ (аудит ad6074f, F2; `MeshOwnershipV1`). Размеры, ширина и ревизия в свойствах меша — не доказательство: меш с теми же числами
и свойствами мог быть подменён (копия датаблока, другая диагональ треугольников, ручная правка). Поэтому КАЖДАЯ точная запись (кнопка,
живая ширина: `capture_ownership`) фиксирует в памяти сессии, а не в свойствах меша: указатель и `session_uid` объекта и меша, число вершин, петель,
граней и рёбер, ПОКОЛЕНИЕ раскладки (счётчик записей точного пути) и отпечаток раскладки (индексы вершин петель и начала граней, `blake2b`),
снятый тут же после записи; образец, чью раскладку записал точный путь, связан с этим владением тождеством объекта. Кадр проверяет за O(1): образец модели
равен образцу владения (иначе `PREVIEW_MESH_LAYOUT_GENERATION_STALE`), указатели и `session_uid` (`PREVIEW_DECAL_OBJECT_REPLACED`,
`PREVIEW_MESH_DATABLOCK_REPLACED`), размеры, ширину и ревизию (`PREVIEW_MESH_NOT_THE_BASE`), режим Edit (`PREVIEW_DECAL_IN_EDIT_MODE`); отпечаток
раскладки перечитывается не чаще раза в `LAYOUT_RECHECK_SECONDS` (и не чаще, чем стоит `LAYOUT_RECHECK_COST_RATIO` его прочтений) — сеть на
случай пропущенного сигнала (`PREVIEW_MESH_LAYOUT_CHANGED`). СТРОГАЯ ИНВАЛИДАЦИЯ: любое внешнее изменение снимает модель сразу, до следующего кадра, —
Undo/Redo и загрузка файла (`PREVIEW_HISTORY_STEP`), обновление геометрии декали в depsgraph, которое сделали не мы
(`note_decal_updates`, `PREVIEW_DECAL_CHANGED_EXTERNALLY`; свою запись обработчик узнаёт по метке `pending_own_update`), вход в Edit, подмена
объекта или датаблока. Модель снята с названной причиной (`PREVIEW_MODEL_DROPPED:<причина>`), записи в меш нет, превью молчит, линии остаются, а точный
путь работает как прежде: следующий точный результат заводит образец и владение заново.

ТОЧНЫЙ ПУТЬ НЕ ТРОНУТ. Меш после точного результата равен кнопке побитово (общий код записи; массивы строятся тем же `build_mesh_arrays`),
а отмена модального инструмента возвращает базу ПОБИТОВО (`restore_base_mesh`: кадр на `dt = 0` — сама база). Этот модуль `bpy` лениво
импортирует только в кадре и в проверках цели (стена `tests/test_architecture.py`); построение образцов и моделей от Blender не зависит.
"""

from __future__ import annotations

import hashlib
import time
from dataclasses import dataclass, field

import numpy as np

from .envelope_width_preview_model import (
    INTERVAL_CERTIFIED,
    PREVIEW_MESH_FROM_INTERVAL_V1,
    DeviationV1,
    PreviewModelV1,
    PreviewRefusalV1,
    PreviewSampleV1,
    advance_ledger,
    base_frame,
    build_model,
    deviation,
    evaluate,
    sample_of,
)

#: Доля ширины, на которую затравка отходит от базы: один шаг, чтобы наклон был локальным, но больше шума округления.
PRIME_RELATIVE_STEP = 0.005
PRIME_RELATIVE_STEPS = (PRIME_RELATIVE_STEP, 0.001)
#: Сколько затравок на образец на экране (вторая даёт квадрат: две другие точки).
PRIME_ATTEMPTS = 2
#: Точных образцов вне экрана, которые держит сессия.
AUX_SAMPLES_LIMIT = 3

MODEL_DROPPED = "PREVIEW_MODEL_DROPPED"
MODEL_BUILD_FAILED = "PREVIEW_MODEL_BUILD_FAILED"
DECAL_GONE = "PREVIEW_DECAL_GONE"
DECAL_IN_EDIT_MODE = "PREVIEW_DECAL_IN_EDIT_MODE"
MESH_NOT_THE_BASE = "PREVIEW_MESH_NOT_THE_BASE"
PRIME_BASE_REPLACED = "PREVIEW_BASE_REPLACED_WHILE_PRIMING"
WAITING = "PREVIEW_WAITING_FOR_MODEL"
#: Принадлежность меша (F2): владения нет, образец не из этого поколения, объект и датаблок подменены, раскладка изменилась,
#: геометрию декали изменил не точный путь, шаг истории.
OWNERSHIP_UNKNOWN = "PREVIEW_MESH_OWNERSHIP_UNKNOWN"
OWNERSHIP_CAPTURE_FAILED = "PREVIEW_MESH_OWNERSHIP_CAPTURE_FAILED"
LAYOUT_GENERATION_STALE = "PREVIEW_MESH_LAYOUT_GENERATION_STALE"
DECAL_OBJECT_REPLACED = "PREVIEW_DECAL_OBJECT_REPLACED"
MESH_DATABLOCK_REPLACED = "PREVIEW_MESH_DATABLOCK_REPLACED"
MESH_LAYOUT_CHANGED = "PREVIEW_MESH_LAYOUT_CHANGED"
DECAL_CHANGED_EXTERNALLY = "PREVIEW_DECAL_CHANGED_EXTERNALLY"
HISTORY_STEP = "PREVIEW_HISTORY_STEP"
#: Как часто кадр перечитывает отпечаток раскладки меша (сеть на случай пропущенного сигнала) и сколько кадров он вправе стоить в среднем:
#: интервал не меньше `LAYOUT_RECHECK_COST_RATIO` длительностей самого чтения.
LAYOUT_RECHECK_SECONDS = 0.25
LAYOUT_RECHECK_COST_RATIO = 20.0
#: Сколько номеров придержанных патчей печатает строка статуса.
HELD_NAMES_SHOWN = 6


@dataclass(slots=True)
class PreviewLogV1:
    """Счётчики превью меша за жизнь сессии: что построено, что отказано, сколько кадров и чего они стоили, как сверка и владение."""

    models: int = 0
    refusals: dict = field(default_factory=dict)
    dropped: dict = field(default_factory=dict)
    primes_requested: int = 0
    frames: int = 0
    frame_seconds_max: float = 0.0
    frame_seconds_total: float = 0.0
    checks: int = 0
    domains_checked: int = 0
    domains_refuted: int = 0
    max_position_error: float = 0.0
    max_uv_error: float = 0.0
    #: Наибольшее отношение отклонения к пределу среди судимых доменов за все сверки (1.0 — предел).
    max_deviation_ratio: float = 0.0
    #: Исходы журнала доверия: имя -> сколько раз (карантин, возврат со сжатой областью, снятие модели домена).
    trust_events: dict = field(default_factory=dict)
    #: Последняя сверка текстом строки статуса.
    last_check: str = ""
    last_model_bytes: int = 0
    #: Владение мешем: номер поколения раскладки, сколько раз перечитан отпечаток и самое долгое чтение.
    layout_generation: int = 0
    layout_checks: int = 0
    layout_check_seconds_max: float = 0.0

    def refuse(self, name: str) -> None:
        self.refusals[name] = self.refusals.get(name, 0) + 1


@dataclass(frozen=True, slots=True)
class PreviewMeshStateV1:
    """Что показывает меш сейчас: ширина кадра, живые и придержанные домены (номера патчей), исход и цена кадра."""

    alpha: float
    live: int
    held: int
    held_patches: tuple
    outcome: str
    seconds: float
    serial: int


@dataclass(frozen=True, slots=True)
class LiveRunV1:
    """Результат потока точного счёта: прогон, массивы меша (их же пишет главный поток), образец, модель, сверка и журнал доверия."""

    run: object
    arrays: object
    sample: PreviewSampleV1
    #: `PreviewModelV1`, `PreviewRefusalV1` либо `None` (модели не на чем строиться).
    model: object | None
    check: DeviationV1 | None
    #: Журнал доверия после сверки этого прогона (`TrustLedgerV1`) и его события `((патч, домен, имя), ...)`.
    trust: object | None = None
    trust_events: tuple = ()


@dataclass(slots=True)
class MeshOwnershipV1:
    """Кому принадлежит меш декали: что точный путь записал и что кадр сверяет. Память сессии, не свойства меша (их копия сохраняет)."""

    #: Образец, чью раскладку записал точный путь (тождество): модель чужого образца этому владению не принадлежит.
    sample: object
    #: Поколение раскладки: номер записи точного пути в сессии.
    generation: int
    object_pointer: int
    object_uid: int
    mesh_pointer: int
    mesh_uid: int
    vertices: int
    loops: int
    polygons: int
    edges: int
    #: Отпечаток раскладки: индексы вершин петель и начала граней (`blake2b`), снятый сразу после записи.
    layout: bytes
    verified_at: float
    check_seconds: float
    #: Запись сделана нами, и depsgraph ещё не доложил об обновлении геометрии: первое обновление — наше, не внешнее.
    pending_own_update: bool = True
    corner_scratch: object = None
    start_scratch: object = None

    def recheck_after(self) -> float:
        return max(LAYOUT_RECHECK_SECONDS, LAYOUT_RECHECK_COST_RATIO * self.check_seconds)


# --------------------------------------------------------------------------
# Ключ и образец (поток точного счёта и кнопка)
# --------------------------------------------------------------------------


def sample_key(record, offset: float) -> tuple:
    """Чья это геометрия: источник, ревизия, бэкенд ядра, плотность, допуски, смещение и номер сброса сессии.

    Образец и модель другого ключа не смешиваются и не применяются: другой объект, другая ревизия, другой бэкенд либо
    другая политика дают другой меш (`build_model` отказывает именем `PREVIEW_KEY_MISMATCH`).
    """

    from .envelope_request_policy import normalize_envelope_fan_density

    return (
        str(record.source_name),
        str(record.source_digest),
        str(record.kernel_backend),
        str(normalize_envelope_fan_density(record.density)),
        int(record.stretch_percent),
        float(record.dissolve_percent),
        float(offset),
        int(record.invalidation_count),
    )


def build_sample(results, arrays, key: tuple, alpha: float) -> PreviewSampleV1:
    """Образец точного прогона на `alpha` (текст ширины прогона — `str(float(alpha))`, как у `run_production`)."""

    return sample_of(results, arrays, key=key, alpha_text=str(float(alpha)))


def _model_or_refusal(base, others, ledger):
    try:
        return build_model(base, tuple(others), ledger)
    except Exception as exc:  # noqa: BLE001 - модель не вправе уронить точный результат: исход назван
        return PreviewRefusalV1(MODEL_BUILD_FAILED, f"{type(exc).__name__}: {exc}")


def finish_live_run(run, *, alpha, offset, key, displayed, aux, previous, prime, trust=None) -> LiveRunV1:
    """Хвост потока точного счёта: массивы меша, образец и модель (всё в потоке, главный поток только применяет).

    Точный прогон (`prime=False`): сверка предсказания прежней модели с ним (`deviation`), журнал доверия по её итогу (`advance_ledger`:
    опровергнутый домен — в карантин), модель НОВОГО образца из прежнего образца на экране и вспомогательных, с журналом. Затравка
    (`prime=True`): её образец — вспомогательный, а модель строится для образца, который на экране сейчас (`displayed`), журнал она не
    двигает (затравка опорная, а не проверочная).
    """

    from .envelope_production_mesh import build_mesh_arrays

    arrays = build_mesh_arrays(run.results, offset)
    sample = build_sample(run.results, arrays, key, alpha)
    if prime:
        model = None if displayed is None else _model_or_refusal(displayed, (sample, *aux), trust)
        return LiveRunV1(run, arrays, sample, model, None, trust)
    check = None
    events: tuple = ()
    if previous is not None:
        try:
            check = deviation(previous, sample)
            trust, events = advance_ledger(trust, previous, check)
        except Exception as exc:  # noqa: BLE001 - сверка не роняет результат
            check = DeviationV1(sample.alpha, 0.0, 0.0, 0, 0, -1, (), f"PREVIEW_CHECK_FAILED:{type(exc).__name__}")
            events = ()
    others = (() if displayed is None else (displayed,)) + tuple(aux)
    return LiveRunV1(run, arrays, sample, _model_or_refusal(sample, others, trust), check, trust, events)


# --------------------------------------------------------------------------
# Состояние сессии (главный поток)
# --------------------------------------------------------------------------


def log_of(controller) -> PreviewLogV1:
    log = controller.width_preview_log
    if log is None:
        log = controller.width_preview_log = PreviewLogV1()
    return log


def _count_model(controller, model) -> None:
    log = log_of(controller)
    if isinstance(model, PreviewModelV1):
        log.models += 1
        log.last_model_bytes = model.own_bytes
    elif isinstance(model, PreviewRefusalV1):
        log.refuse(model.outcome)


def _check_text(check: DeviationV1) -> str:
    return (
        f"{check.domains_checked} domains at {check.relative_distance * 100:.1f}% from the base, max {check.max_position:.2e} m "
        f"({check.worst_ratio:.2f} of the limit), {len(check.refuted)} refuted, {len(check.shadow_clean)} quarantined clean"
    )


def _record_check(controller, live: LiveRunV1) -> None:
    """Результат апостериорной сверки прежней модели с этим точным прогоном и события журнала доверия — в журнал сессии.

    Отказ сверки назван, а не молчит. Опровергнутые домены прежней модели уже в карантине журнала (`live.trust`), и модель нового образца
    построена с ним: прежняя модель после этого результата не живёт.
    """

    check = live.check
    log = log_of(controller)
    for _patch, _domain, name in live.trust_events:
        log.trust_events[name] = log.trust_events.get(name, 0) + 1
    if check is None:
        return
    if check.refusal:
        log.refuse(check.refusal)
        return
    log.checks += 1
    log.domains_checked += check.domains_checked
    log.domains_refuted += len(check.refuted)
    log.max_position_error = max(log.max_position_error, check.max_position)
    log.max_uv_error = max(log.max_uv_error, check.max_uv)
    log.max_deviation_ratio = max(log.max_deviation_ratio, check.worst_ratio)
    log.last_check = _check_text(check)


def note_exact_display(controller, live: LiveRunV1) -> None:
    """Точный результат лёг в меш: образец стал экранным, прежний — вспомогательным, модель новая, превью снято, журнал доверия сдвинут.

    Владение мешем (`capture_ownership`) записывает тот, кто писал меш, СРАЗУ после записи: здесь меша нет.
    """

    _record_check(controller, live)
    previous = controller.width_displayed
    aux = controller.width_aux
    if previous is not None and previous.key == live.sample.key:
        aux = (previous, *aux)
    else:
        aux = ()
    controller.width_aux = tuple(item for item in aux if item.alpha != live.sample.alpha and item.key == live.sample.key)[:AUX_SAMPLES_LIMIT]
    controller.width_displayed = live.sample
    controller.width_trust = live.trust
    controller.width_mesh_preview = None
    model = live.model
    _count_model(controller, model)
    controller.width_model = model if isinstance(model, PreviewModelV1) else None
    controller.width_model_refusal = model if isinstance(model, PreviewRefusalV1) else None
    controller.width_prime_attempts = 0


def note_button_display(controller, sample: PreviewSampleV1) -> None:
    """Кнопка записала меш: образец на экране, вспомогательных нет (прежние говорили про другую сборку), модели ещё нет, журнал доверия забыт."""

    controller.width_displayed = sample
    controller.width_aux = ()
    controller.width_model = None
    controller.width_model_refusal = None
    controller.width_mesh_preview = None
    controller.width_trust = None
    controller.width_prime_attempts = 0


def note_prime(controller, base: PreviewSampleV1 | None, live: LiveRunV1) -> str:
    """Затравка посчитана: её образец вспомогательный, модель для базы. Пусто либо причина, по которой образец не принят."""

    if controller.width_displayed is not base or base is None:
        log_of(controller).refuse(PRIME_BASE_REPLACED)
        return PRIME_BASE_REPLACED
    if live.sample.key == base.key:
        controller.width_aux = tuple(
            item for item in (live.sample, *controller.width_aux) if item.key == base.key
        )[:AUX_SAMPLES_LIMIT]
    model = live.model
    _count_model(controller, model)
    if isinstance(model, PreviewModelV1):
        controller.width_model = model
        controller.width_model_refusal = None
    else:
        controller.width_model_refusal = model
    return ""


def drop_model(controller, reason: str) -> None:
    """Модель и образцы сняты с названной причиной: меш ей больше не принадлежит (другой состав, шаг истории, внешняя правка).

    Образец на экране описывает геометрию, которой в меше нет либо за которой больше нет присмотра, поэтому он снят вместе с моделью и владением:
    следующий точный результат (кнопка либо заказ ширины) заводит образец и владение заново, и затравка снова разрешена. Журнал доверия
    остаётся (он про ключ образца, а не про меш), и точный путь не затронут.
    """

    had = controller.width_model is not None or controller.width_displayed is not None
    if had:
        name = f"{MODEL_DROPPED}:{reason}"
        log = log_of(controller)
        log.dropped[name] = log.dropped.get(name, 0) + 1
        controller.width_model_refusal = PreviewRefusalV1(name)
    controller.width_model = None
    controller.width_displayed = None
    controller.width_aux = ()
    controller.width_mesh_preview = None
    controller.width_mesh_owner = None
    controller.width_prime_attempts = 0


def note_history(controller) -> None:
    """Undo/Redo/загрузка: что показывает меш, неизвестно (шаг мог лечь на геометрию превью, датаблок мог быть возвращён из истории), поэтому модель СНЯТА.

    Строго, а не «сверим на первом кадре»: после шага истории указатели, `session_uid` и раскладка не доказывают ничего. Расхождение ползунка и
    ширины меша заказывает точный пересчёт (`reconcile_after_history`), и его результат заводит образец и владение заново.
    """

    drop_model(controller, HISTORY_STEP)


def next_prime_alpha(displayed: PreviewSampleV1, aux) -> float | None:
    """Ширина следующей затравки: рядом с базой, внутри интервалов наибольшего числа доменов; прежние ширины не повторяются.

    Кандидаты — `PRIME_RELATIVE_STEPS` в обе стороны. Вторая затравка предпочитает другую сторону от базы, чем первая, чтобы
    три точки охватывали базу (квадрат по обе стороны точнее, чем по одну).
    """

    from fractions import Fraction

    taken = {item.alpha for item in aux} | {displayed.alpha}
    sides = [1 if item.alpha > displayed.alpha else -1 for item in aux]
    best = None
    for step in PRIME_RELATIVE_STEPS:
        for sign in (1, -1):
            candidate = float(str(float(displayed.alpha * (1.0 + sign * step))))
            if candidate <= 0.0 or candidate in taken:
                continue
            exact = Fraction(str(candidate))
            inside = sum(
                1
                for domain in displayed.domains
                if domain.status == INTERVAL_CERTIFIED and domain.interval is not None and domain.interval.contains_exact(exact)
            )
            score = (inside, 0 if sign in sides else 1, step)
            if best is None or score > best[0]:
                best = (score, candidate)
    return None if best is None else best[1]


def wants_prime(controller) -> bool:
    """Нужна ли затравка: есть образец на экране своего ключа, модели нет либо она прямая, а попыток меньше `PRIME_ATTEMPTS`."""

    displayed = controller.width_displayed
    if displayed is None or controller.width_prime_attempts >= PRIME_ATTEMPTS:
        return False
    model = controller.width_model
    if model is None:
        return True
    return model.quadratic_domains == 0 and model.modelled_domains > 0 and len(model.others) < 2


# --------------------------------------------------------------------------
# Владение мешем (F2)
# --------------------------------------------------------------------------


def _read_layout(mesh, corner, start) -> bytes:
    """Отпечаток раскладки меша: индексы вершин петель и начала граней в заранее выделенные массивы, `blake2b` (16 байт)."""

    mesh.loops.foreach_get("vertex_index", corner)
    mesh.polygons.foreach_get("loop_start", start)
    digest = hashlib.blake2b(digest_size=16)
    digest.update(memoryview(corner))
    digest.update(memoryview(start))
    return digest.digest()


def capture_ownership(controller, decal, sample) -> MeshOwnershipV1 | None:
    """Точный путь записал меш декали `decal` под образец `sample`: владение (тождество, размеры, поколение, отпечаток раскладки) в сессию.

    Зовёт тот, кто писал меш, СРАЗУ после записи (кнопка и применение живой ширины: тест архитектуры). Сбой чтения называется
    (`PREVIEW_MESH_OWNERSHIP_CAPTURE_FAILED:<тип>`) и владения не оставляет: кадр тогда молчит, названный `PREVIEW_MESH_OWNERSHIP_UNKNOWN`.
    """

    controller.width_mesh_owner = None
    if decal is None or sample is None or getattr(decal, "data", None) is None:
        return None
    mesh = decal.data
    try:
        corner = np.empty(len(mesh.loops), dtype=np.int32)
        start = np.empty(len(mesh.polygons), dtype=np.int32)
        started = time.perf_counter()
        layout = _read_layout(mesh, corner, start)
        finished = time.perf_counter()
        generation = int(controller.width_layout_generation) + 1
        owner = MeshOwnershipV1(
            sample=sample,
            generation=generation,
            object_pointer=int(decal.as_pointer()),
            object_uid=int(decal.session_uid),
            mesh_pointer=int(mesh.as_pointer()),
            mesh_uid=int(mesh.session_uid),
            vertices=len(mesh.vertices),
            loops=len(mesh.loops),
            polygons=len(mesh.polygons),
            edges=len(mesh.edges),
            layout=layout,
            verified_at=finished,
            check_seconds=finished - started,
            corner_scratch=corner,
            start_scratch=start,
        )
    except Exception as exc:  # noqa: BLE001 - владение не роняет точный результат: исход назван
        log_of(controller).refuse(f"{OWNERSHIP_CAPTURE_FAILED}:{type(exc).__name__}")
        return None
    controller.width_layout_generation = generation
    controller.width_mesh_owner = owner
    log_of(controller).layout_generation = generation
    return owner


def _original_uid(ident):
    """`session_uid` ОРИГИНАЛА датаблока: `DepsgraphUpdate.id` — вычисленная копия, и у неё `session_uid` нулевой, а у оригинала — настоящий."""

    original = getattr(ident, "original", None)
    return getattr(ident if original is None else original, "session_uid", None)


def note_decal_updates(controller, updates) -> str:
    """Обработчик depsgraph: обновление ГЕОМЕТРИИ нашего объекта или меша, которое сделали не мы, снимает модель. Пусто либо причина.

    `updates` — `depsgraph.updates` (их `id` — вычисленные копии: тождество берётся у `id.original`). Свою запись (кадр, возврат базы, точный путь) отмечает `pending_own_update`: первое обновление после
    неё — наше, и метка гаснет; следующее — внешнее. Если внешнее слилось с нашим в одном обновлении, сигнал его не увидит, но отпечаток
    раскладки в кадре увидит изменение раскладки (`PREVIEW_MESH_LAYOUT_CHANGED`).
    """

    owner = controller.width_mesh_owner
    if owner is None:
        return ""
    ours = (owner.object_uid, owner.mesh_uid)
    for update in updates:
        if getattr(update, "is_updated_geometry", False) and _original_uid(update.id) in ours:
            break
    else:
        return ""
    if owner.pending_own_update:
        owner.pending_own_update = False
        return ""
    drop_model(controller, DECAL_CHANGED_EXTERNALLY)
    return DECAL_CHANGED_EXTERNALLY


# --------------------------------------------------------------------------
# Кадр (главный поток)
# --------------------------------------------------------------------------


def _mesh_problem(controller, model, decal) -> str:
    """Почему меш не принимает кадр модели (пусто — принимает): владение, тождество, размеры, свойства точного пути и раскладка.

    Порядок — от дешёвого к дорогому: объект, режим, владение (поколение образца), тождество (указатели и `session_uid`), размеры, записанные точным
    путём ширина и ревизия, и лишь раз в `recheck_after` — отпечаток раскладки. Каждый ответ — имя исхода.
    """

    from .envelope_production_mesh import DECAL_REVISION_PROPERTY, DECAL_UV_LAYER, DECAL_WIDTH_PROPERTY

    if decal is None:
        return DECAL_GONE
    if getattr(decal, "mode", "OBJECT") == "EDIT":
        return DECAL_IN_EDIT_MODE
    mesh = decal.data
    if mesh is None:
        return MESH_NOT_THE_BASE
    owner = controller.width_mesh_owner
    base = model.base
    if owner is None:
        return OWNERSHIP_UNKNOWN
    if owner.sample is not base:
        return LAYOUT_GENERATION_STALE
    if decal.as_pointer() != owner.object_pointer or decal.session_uid != owner.object_uid:
        return DECAL_OBJECT_REPLACED
    if mesh.as_pointer() != owner.mesh_pointer or mesh.session_uid != owner.mesh_uid:
        return MESH_DATABLOCK_REPLACED
    layer = mesh.uv_layers.get(DECAL_UV_LAYER)
    if (
        layer is None
        or len(mesh.vertices) != base.vertex_count
        or len(layer.data) != base.loop_count
        or len(mesh.loops) != owner.loops
        or len(mesh.polygons) != owner.polygons
        or len(mesh.edges) != owner.edges
    ):
        return MESH_NOT_THE_BASE
    if DECAL_WIDTH_PROPERTY not in mesh.keys() or float(mesh[DECAL_WIDTH_PROPERTY]) != base.alpha:
        return MESH_NOT_THE_BASE
    if DECAL_REVISION_PROPERTY not in decal.keys() or str(decal[DECAL_REVISION_PROPERTY]) != base.source_revision:
        return MESH_NOT_THE_BASE
    started = time.perf_counter()
    if started - owner.verified_at >= owner.recheck_after():
        layout = _read_layout(mesh, owner.corner_scratch, owner.start_scratch)
        finished = time.perf_counter()
        owner.check_seconds, owner.verified_at = finished - started, finished
        log = log_of(controller)
        log.layout_checks += 1
        log.layout_check_seconds_max = max(log.layout_check_seconds_max, owner.check_seconds)
        if layout != owner.layout:
            return MESH_LAYOUT_CHANGED
    return ""


def _decal_of(controller):
    import bpy

    from .envelope_production_mesh import find_decal_object

    record = controller.width_build
    source = None if record is None else bpy.data.objects.get(record.source_name)
    return None if source is None else find_decal_object(source)


def preview_mesh_now(controller, width: float) -> PreviewMeshStateV1 | None:
    """Меш декали на ширине `width` из модели: кадр пишется сразу, на главном потоке. `None` — кадра нет (причина в статусе).

    Меш, который модель не вправе писать (подмена, правка, Edit, чужое поколение), снимает модель с именем причины и не получает записи.
    """

    model = controller.width_model
    if model is None:
        return None
    from .envelope_production_mesh import ProductionWriteError, write_preview_geometry

    started = time.perf_counter()
    decal = _decal_of(controller)
    problem = _mesh_problem(controller, model, decal)
    if problem:
        drop_model(controller, problem)
        return None
    frame = evaluate(model, width)
    try:
        write_preview_geometry(decal.data, frame.positions, frame.uvs)
    except ProductionWriteError as exc:
        drop_model(controller, exc.outcome)
        return None
    controller.width_mesh_owner.pending_own_update = True
    previous = controller.width_mesh_preview
    seconds = time.perf_counter() - started
    state = PreviewMeshStateV1(
        frame.alpha,
        frame.live_domains,
        frame.held_domains,
        tuple(patch for patch, _reason in frame.held[:HELD_NAMES_SHOWN]),
        frame.outcome,
        seconds,
        1 if previous is None else previous.serial + 1,
    )
    controller.width_mesh_preview = state
    log = log_of(controller)
    log.frames += 1
    log.frame_seconds_total += seconds
    log.frame_seconds_max = max(log.frame_seconds_max, seconds)
    return state


def restore_base_mesh(controller) -> bool:
    """Меш возвращён на геометрию базы (отмена инструмента): побитово то, что записал точный путь. `True` — было что возвращать.

    Меш, который модель не вправе писать, не получает записи, а модель снимается с именем причины (как в кадре).
    """

    if controller.width_mesh_preview is None:
        return False
    model = controller.width_model
    controller.width_mesh_preview = None
    if model is None:
        return False
    from .envelope_production_mesh import ProductionWriteError, write_preview_geometry

    decal = _decal_of(controller)
    problem = _mesh_problem(controller, model, decal)
    if problem:
        drop_model(controller, problem)
        return False
    frame = base_frame(model.base)
    try:
        write_preview_geometry(decal.data, frame.positions, frame.uvs)
    except ProductionWriteError:
        return False
    controller.width_mesh_owner.pending_own_update = True
    return True


# --------------------------------------------------------------------------
# Строки статуса
# --------------------------------------------------------------------------


def status_lines(controller) -> tuple[str, ...]:
    lines = []
    state = controller.width_mesh_preview
    if state is not None:
        held = ""
        if state.held:
            shown = ", ".join(str(item) for item in state.held_patches)
            more = "" if state.held <= len(state.held_patches) else ", ..."
            held = f", {state.held} held at the last exact geometry (patch {shown}{more})"
        refusal = "" if state.outcome == PREVIEW_MESH_FROM_INTERVAL_V1 else f" REFUSED {state.outcome}:"
        lines.append(
            f"Mesh {PREVIEW_MESH_FROM_INTERVAL_V1} preview (approximate, not certified):{refusal} width {state.alpha:.4g}, "
            f"{state.live} domains moving{held}; the exact result follows"
        )
    model = controller.width_model
    prime = controller.width_prime
    if model is not None:
        reasons = model.reason_counts()
        extra = "" if not reasons else " (" + ", ".join(f"{name} {count}" for name, count in sorted(reasons.items())) + ")"
        lines.append(
            f"Preview model (approximate, inside the certified event interval): {model.modelled_domains}/{model.domain_count} domains "
            f"modelled, {model.quadratic_domains} quadratic, {model.own_bytes / 1024:.0f} KB{extra}"
        )
    elif controller.width_model_refusal is not None:
        refusal = controller.width_model_refusal
        lines.append(f"Preview model: none ({refusal.outcome}) - the line overlay only")
    ledger = controller.width_trust
    if ledger is not None and ledger.entries:
        counts = ledger.counts()
        lines.append("Preview trust: " + ", ".join(f"{count} {name.lower()}" for name, count in sorted(counts.items())))
    log = controller.width_preview_log
    if log is not None and log.last_check:
        lines.append(f"Last exact check: {log.last_check}")
    if prime is not None and prime.status_text and prime.busy:
        lines.append(f"Preview model: {prime.status_text}")
    return tuple(lines)


__all__ = (
    "AUX_SAMPLES_LIMIT",
    "DECAL_CHANGED_EXTERNALLY",
    "DECAL_GONE",
    "DECAL_IN_EDIT_MODE",
    "DECAL_OBJECT_REPLACED",
    "HISTORY_STEP",
    "LAYOUT_GENERATION_STALE",
    "LAYOUT_RECHECK_COST_RATIO",
    "LAYOUT_RECHECK_SECONDS",
    "LiveRunV1",
    "MESH_DATABLOCK_REPLACED",
    "MESH_LAYOUT_CHANGED",
    "MESH_NOT_THE_BASE",
    "MODEL_BUILD_FAILED",
    "MODEL_DROPPED",
    "MeshOwnershipV1",
    "OWNERSHIP_CAPTURE_FAILED",
    "OWNERSHIP_UNKNOWN",
    "PRIME_ATTEMPTS",
    "PRIME_BASE_REPLACED",
    "PRIME_RELATIVE_STEP",
    "PreviewLogV1",
    "PreviewMeshStateV1",
    "build_sample",
    "capture_ownership",
    "drop_model",
    "finish_live_run",
    "log_of",
    "next_prime_alpha",
    "note_button_display",
    "note_decal_updates",
    "note_exact_display",
    "note_history",
    "note_prime",
    "preview_mesh_now",
    "restore_base_mesh",
    "sample_key",
    "status_lines",
    "wants_prime",
)
