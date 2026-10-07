"""Живое превью меша ширины: склейка сертификата (`envelope_width_certificate`) с Blender, сессией и планировщиками.

ЧТО ВИДИТ ВЛАДЕЛЕЦ. Пока он тянет ширину (модальный «Adjust Decal Width» либо ползунок), НАСТОЯЩИЙ меш декали двигается каждый кадр: позиции
и UV пишутся в существующий меш из сертификата, а точный пересчёт идёт в фоне через тот же планировщик, что и прежде, и его результат
заменяет превью. Строка статуса называет превью (`PREVIEW_MESH_FROM_INTERVAL_V1`, «не сертифицировано») и придержанные домены.

ОТКУДА СЕРТИФИКАТ. Точные прогоны, которые уже посчитаны: на экране лежит ОБРАЗЕЦ (`PreviewSampleV1`: массивы меша и домены прогона, чья
геометрия сейчас в меше), рядом — до трёх других точных образцов. Сертификат строит поток точного счёта (`finish_live_run`), НЕ главный
поток: numpy над массивами меша, домен за доменом. Первый сертификат после кнопки даёт «затравка» (`prime`): отдельный точный прогон на
ширине рядом (`PRIME_RELATIVE_STEP`), который в меш НЕ пишется (его результат — образец, а не отображаемая ширина); он запускается
инструментом при входе (`begin_adjust`), а не кнопкой: кнопка не считает ничего сверх обещанного. Каждый следующий точный результат даёт
сертификат сам: прежний образец на экране становится вторым образцом нового.

ЧТО ПИШЕТСЯ И ГДЕ. Только позиции и UV, только в СУЩЕСТВУЮЩИЙ меш (`write_preview_geometry`: `foreach_set`, без пересоздания датаблоков, поэтому
таймеры и модальный оператор не ломают шаг отмены). Состав меша — вершины и петли образца-базы — проверяется перед КАЖДЫМ кадром
(число вершин и петель, ширина и ревизия, записанные на меш точным путём); не сошлось — сертификат снят с названной причиной
(`PREVIEW_CERTIFICATE_DROPPED:<причина>`), а превью молчит. После Undo/Redo/загрузки сертификат снимается всегда (меш и память могли разойтись).

ТОЧНЫЙ ПУТЬ НЕ ТРОНУТ. Меш после точного результата равен кнопке побитово (общий код записи; массивы строятся тем же `build_mesh_arrays`),
а отмена модального инструмента возвращает базу ПОБИТОВО (`restore_base_mesh`: кадр на `dt = 0` — сама база). Этот модуль `bpy` лениво
импортирует только в кадре и в проверках цели (стена `tests/test_architecture.py`); построение образцов и сертификатов от Blender не зависит.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from .envelope_width_certificate import (
    PREVIEW_MESH_FROM_INTERVAL_V1,
    DeviationV1,
    PreviewCertificateV1,
    PreviewRefusalV1,
    PreviewSampleV1,
    base_frame,
    build_certificate,
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

CERTIFICATE_DROPPED = "PREVIEW_CERTIFICATE_DROPPED"
CERTIFICATE_BUILD_FAILED = "PREVIEW_CERTIFICATE_BUILD_FAILED"
DECAL_GONE = "PREVIEW_DECAL_GONE"
DECAL_IN_EDIT_MODE = "PREVIEW_DECAL_IN_EDIT_MODE"
MESH_NOT_THE_BASE = "PREVIEW_MESH_NOT_THE_BASE"
PRIME_BASE_REPLACED = "PREVIEW_BASE_REPLACED_WHILE_PRIMING"
WAITING = "PREVIEW_WAITING_FOR_CERTIFICATE"
#: Сколько номеров придержанных патчей печатает строка статуса.
HELD_NAMES_SHOWN = 6


@dataclass(slots=True)
class PreviewLogV1:
    """Счётчики превью меша за жизнь сессии: что построено, что отказано, сколько кадров и чего они стоили, как сверка."""

    certificates: int = 0
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
    last_certificate_bytes: int = 0

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
    """Результат потока точного счёта: прогон, массивы меша (их же пишет главный поток), образец, сертификат и сверка."""

    run: object
    arrays: object
    sample: PreviewSampleV1
    #: `PreviewCertificateV1`, `PreviewRefusalV1` либо `None` (сертификату не на чем строиться).
    certificate: object | None
    check: DeviationV1 | None


# --------------------------------------------------------------------------
# Ключ и образец (поток точного счёта и кнопка)
# --------------------------------------------------------------------------


def sample_key(record, offset: float) -> tuple:
    """Чья это геометрия: источник, ревизия, бэкенд ядра, плотность, допуски, смещение и номер сброса сессии.

    Образец и сертификат другого ключа не смешиваются и не применяются: другой объект, другая ревизия, другой бэкенд либо
    другая политика дают другой меш (`build_certificate` отказывает именем `PREVIEW_KEY_MISMATCH`).
    """

    from .envelope_kernel_backend import DEFAULT_SKELETON_BACKEND
    from .envelope_request_policy import normalize_envelope_fan_density

    return (
        str(record.source_name),
        str(record.source_digest),
        str(record.kernel_backend),
        str(getattr(record, "skeleton_backend", DEFAULT_SKELETON_BACKEND)),
        str(normalize_envelope_fan_density(record.density)),
        int(record.stretch_percent),
        float(record.dissolve_percent),
        float(offset),
        int(record.invalidation_count),
    )


def build_sample(results, arrays, key: tuple, alpha: float) -> PreviewSampleV1:
    """Образец точного прогона на `alpha` (текст ширины прогона — `str(float(alpha))`, как у `run_production`)."""

    return sample_of(results, arrays, key=key, alpha_text=str(float(alpha)))


def _certificate_or_refusal(base, others):
    try:
        return build_certificate(base, tuple(others))
    except Exception as exc:  # noqa: BLE001 - сертификат не вправе уронить точный результат: исход назван
        return PreviewRefusalV1(CERTIFICATE_BUILD_FAILED, f"{type(exc).__name__}: {exc}")


def finish_live_run(run, *, alpha, offset, key, displayed, aux, previous, prime) -> LiveRunV1:
    """Хвост потока точного счёта: массивы меша, образец и сертификат (всё в потоке, главный поток только применяет).

    Точный прогон (`prime=False`): сверка предсказания прежнего сертификата с ним (`deviation`), сертификат НОВОГО образца
    из прежнего образца на экране и вспомогательных. Затравка (`prime=True`): её образец — вспомогательный, а сертификат
    строится для образца, который на экране сейчас (`displayed`).
    """

    from .envelope_production_mesh import build_mesh_arrays

    arrays = build_mesh_arrays(run.results, offset)
    sample = build_sample(run.results, arrays, key, alpha)
    if prime:
        certificate = None if displayed is None else _certificate_or_refusal(displayed, (sample, *aux))
        return LiveRunV1(run, arrays, sample, certificate, None)
    check = None
    if previous is not None:
        try:
            check = deviation(previous, sample)
        except Exception as exc:  # noqa: BLE001 - сверка не роняет результат
            check = DeviationV1(sample.alpha, 0.0, 0.0, 0, 0, -1, (), f"PREVIEW_CHECK_FAILED:{type(exc).__name__}")
    others = (() if displayed is None else (displayed,)) + tuple(aux)
    return LiveRunV1(run, arrays, sample, _certificate_or_refusal(sample, others), check)


# --------------------------------------------------------------------------
# Состояние сессии (главный поток)
# --------------------------------------------------------------------------


def log_of(controller) -> PreviewLogV1:
    log = controller.width_preview_log
    if log is None:
        log = controller.width_preview_log = PreviewLogV1()
    return log


def _count_certificate(controller, certificate) -> None:
    log = log_of(controller)
    if isinstance(certificate, PreviewCertificateV1):
        log.certificates += 1
        log.last_certificate_bytes = certificate.own_bytes
    elif isinstance(certificate, PreviewRefusalV1):
        log.refuse(certificate.outcome)


def _record_check(controller, live: LiveRunV1) -> None:
    """Результат сверки прежнего сертификата с этим точным прогоном — в журнал сессии (отказ сверки назван, а не молчит).

    Опровергнутые домены прежнего сертификата отдельно не снимаются: его заменяет сертификат нового образца, построенный
    из свежих данных; число опровергнутых и наибольшее отклонение остаются в журнале (`PreviewLogV1`).
    """

    check = live.check
    if check is None:
        return
    log = log_of(controller)
    if check.refusal:
        log.refuse(check.refusal)
        return
    log.checks += 1
    log.domains_checked += check.domains_checked
    log.domains_refuted += len(check.refuted)
    log.max_position_error = max(log.max_position_error, check.max_position)
    log.max_uv_error = max(log.max_uv_error, check.max_uv)


def note_exact_display(controller, live: LiveRunV1) -> None:
    """Точный результат лёг в меш: образец стал экранным, прежний — вспомогательным, сертификат нового, превью снято."""

    _record_check(controller, live)
    previous = controller.width_displayed
    aux = controller.width_aux
    if previous is not None and previous.key == live.sample.key:
        aux = (previous, *aux)
    else:
        aux = ()
    controller.width_aux = tuple(item for item in aux if item.alpha != live.sample.alpha and item.key == live.sample.key)[:AUX_SAMPLES_LIMIT]
    controller.width_displayed = live.sample
    controller.width_mesh_preview = None
    certificate = live.certificate
    _count_certificate(controller, certificate)
    controller.width_certificate = certificate if isinstance(certificate, PreviewCertificateV1) else None
    controller.width_certificate_refusal = certificate if isinstance(certificate, PreviewRefusalV1) else None
    controller.width_prime_attempts = 0


def note_button_display(controller, sample: PreviewSampleV1) -> None:
    """Кнопка записала меш: образец на экране, вспомогательных нет (прежние говорили про другую сборку), сертификата ещё нет."""

    controller.width_displayed = sample
    controller.width_aux = ()
    controller.width_certificate = None
    controller.width_certificate_refusal = None
    controller.width_mesh_preview = None
    controller.width_prime_attempts = 0


def note_prime(controller, base: PreviewSampleV1 | None, live: LiveRunV1) -> str:
    """Затравка посчитана: её образец вспомогательный, сертификат для базы. Пусто либо причина, по которой образец не принят."""

    if controller.width_displayed is not base or base is None:
        log_of(controller).refuse(PRIME_BASE_REPLACED)
        return PRIME_BASE_REPLACED
    if live.sample.key == base.key:
        controller.width_aux = tuple(
            item for item in (live.sample, *controller.width_aux) if item.key == base.key
        )[:AUX_SAMPLES_LIMIT]
    certificate = live.certificate
    _count_certificate(controller, certificate)
    if isinstance(certificate, PreviewCertificateV1):
        controller.width_certificate = certificate
        controller.width_certificate_refusal = None
    else:
        controller.width_certificate_refusal = certificate
    return ""


def drop_certificate(controller, reason: str) -> None:
    """Сертификат и образцы сняты с названной причиной: меш им больше не принадлежит (другой состав, исчезнувшая декаль).

    Образец на экране описывает геометрию, которой в меше нет, поэтому он снят вместе с сертификатом: следующий точный
    результат (кнопка либо заказ ширины) заводит образец заново, и затравка снова разрешена.
    """

    if controller.width_certificate is not None:
        name = f"{CERTIFICATE_DROPPED}:{reason}"
        log = log_of(controller)
        log.dropped[name] = log.dropped.get(name, 0) + 1
    controller.width_certificate = None
    controller.width_certificate_refusal = PreviewRefusalV1(f"{CERTIFICATE_DROPPED}:{reason}")
    controller.width_displayed = None
    controller.width_aux = ()
    controller.width_mesh_preview = None
    controller.width_prime_attempts = 0


def note_history(controller) -> None:
    """Undo/Redo: что показывает меш, неизвестно (шаг мог лечь на геометрию превью), поэтому состояние превью снято.

    Образец на экране и сертификат остаются: КАЖДЫЙ кадр сверяет состав меша и записанные на него ширину и ревизию
    (`_mesh_problem`), и сертификат, которому меш не принадлежит, снимается там с названной причиной. Расхождение ползунка
    и ширины меша заказывает точный пересчёт (`reconcile_after_history`).
    """

    controller.width_mesh_preview = None


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
                if domain.status == "CERTIFIED" and domain.interval is not None and domain.interval.contains_exact(exact)
            )
            score = (inside, 0 if sign in sides else 1, step)
            if best is None or score > best[0]:
                best = (score, candidate)
    return None if best is None else best[1]


def wants_prime(controller) -> bool:
    """Нужна ли затравка: есть образец на экране своего ключа, сертификата нет либо он прямой, а попыток меньше `PRIME_ATTEMPTS`."""

    displayed = controller.width_displayed
    if displayed is None or controller.width_prime_attempts >= PRIME_ATTEMPTS:
        return False
    certificate = controller.width_certificate
    if certificate is None:
        return True
    return certificate.quadratic_domains == 0 and certificate.certified_domains > 0 and len(certificate.others) < 2


# --------------------------------------------------------------------------
# Кадр (главный поток)
# --------------------------------------------------------------------------


def _mesh_problem(controller, certificate, decal) -> str:
    """Почему меш не принимает кадр сертификата (пусто — принимает): состав и записанные точным путём ширина и ревизия."""

    from .envelope_production_mesh import DECAL_REVISION_PROPERTY, DECAL_UV_LAYER, DECAL_WIDTH_PROPERTY

    if decal is None:
        return DECAL_GONE
    if getattr(decal, "mode", "OBJECT") == "EDIT":
        return DECAL_IN_EDIT_MODE
    mesh = decal.data
    if mesh is None:
        return MESH_NOT_THE_BASE
    layer = mesh.uv_layers.get(DECAL_UV_LAYER)
    base = certificate.base
    if layer is None or len(mesh.vertices) != base.vertex_count or len(layer.data) != base.loop_count:
        return MESH_NOT_THE_BASE
    if DECAL_WIDTH_PROPERTY not in mesh.keys() or float(mesh[DECAL_WIDTH_PROPERTY]) != base.alpha:
        return MESH_NOT_THE_BASE
    if DECAL_REVISION_PROPERTY not in decal.keys() or str(decal[DECAL_REVISION_PROPERTY]) != base.source_revision:
        return MESH_NOT_THE_BASE
    return ""


def _decal_of(controller):
    import bpy

    from .envelope_production_mesh import find_decal_object

    record = controller.width_build
    source = None if record is None else bpy.data.objects.get(record.source_name)
    return None if source is None else find_decal_object(source)


def preview_mesh_now(controller, width: float) -> PreviewMeshStateV1 | None:
    """Меш декали на ширине `width` из сертификата: кадр пишется сразу, на главном потоке. `None` — кадра нет (причина в статусе)."""

    certificate = controller.width_certificate
    if certificate is None:
        return None
    from .envelope_production_mesh import ProductionWriteError, write_preview_geometry

    started = time.perf_counter()
    decal = _decal_of(controller)
    problem = _mesh_problem(controller, certificate, decal)
    if problem == DECAL_IN_EDIT_MODE:
        return None  # пока декаль в Edit, кадры молчат (писать в её меш нельзя), а сертификат цел
    if problem:
        drop_certificate(controller, problem)
        return None
    frame = evaluate(certificate, width)
    try:
        write_preview_geometry(decal.data, frame.positions, frame.uvs)
    except ProductionWriteError as exc:
        drop_certificate(controller, exc.outcome)
        return None
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
    """Меш возвращён на геометрию базы (отмена инструмента): побитово то, что записал точный путь. `True` — было что возвращать."""

    if controller.width_mesh_preview is None:
        return False
    certificate = controller.width_certificate
    controller.width_mesh_preview = None
    if certificate is None:
        return False
    from .envelope_production_mesh import ProductionWriteError, write_preview_geometry

    decal = _decal_of(controller)
    if _mesh_problem(controller, certificate, decal):
        return False
    frame = base_frame(certificate.base)
    try:
        write_preview_geometry(decal.data, frame.positions, frame.uvs)
    except ProductionWriteError:
        return False
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
            f"Mesh {PREVIEW_MESH_FROM_INTERVAL_V1} preview, not certified:{refusal} width {state.alpha:.4g}, "
            f"{state.live} domains moving{held}; the exact result follows"
        )
    certificate = controller.width_certificate
    prime = controller.width_prime
    if certificate is not None:
        reasons = certificate.reason_counts()
        extra = "" if not reasons else " (" + ", ".join(f"{name} {count}" for name, count in sorted(reasons.items())) + ")"
        lines.append(
            f"Preview certificate: {certificate.certified_domains}/{certificate.domain_count} domains, "
            f"{certificate.quadratic_domains} quadratic, {certificate.own_bytes / 1024:.0f} KB{extra}"
        )
    elif controller.width_certificate_refusal is not None:
        refusal = controller.width_certificate_refusal
        lines.append(f"Preview certificate: none ({refusal.outcome}) - the line overlay only")
    if prime is not None and prime.status_text and prime.busy:
        lines.append(f"Certificate: {prime.status_text}")
    return tuple(lines)


__all__ = (
    "AUX_SAMPLES_LIMIT",
    "CERTIFICATE_BUILD_FAILED",
    "CERTIFICATE_DROPPED",
    "DECAL_GONE",
    "DECAL_IN_EDIT_MODE",
    "LiveRunV1",
    "MESH_NOT_THE_BASE",
    "PRIME_ATTEMPTS",
    "PRIME_BASE_REPLACED",
    "PRIME_RELATIVE_STEP",
    "PreviewLogV1",
    "PreviewMeshStateV1",
    "build_sample",
    "drop_certificate",
    "finish_live_run",
    "log_of",
    "next_prime_alpha",
    "note_button_display",
    "note_exact_display",
    "note_history",
    "note_prime",
    "preview_mesh_now",
    "restore_base_mesh",
    "sample_key",
    "status_lines",
    "wants_prime",
)
