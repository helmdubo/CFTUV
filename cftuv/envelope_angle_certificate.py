"""Каноническая мера рефлексного угла и приёмка выпускаемого снапшота.

Здесь два узла, и оба про одно: величина, которую ядро потом сверяет, обязана
быть посчитана РОВНО ОДИН РАЗ, и проверять надо те байты, которые уходят, а не
объект, который остаётся в памяти.

1. `certified_reflex_measure` собирает `CertifiedReflexAngleMeasureV1` из ОДНОГО
   интервала φ/π. Ядро требует `δ == φ − 1` побитово
   (`validation.py`, `reflex_excess_over_pi`), и единственный способ это
   выдержать — вычесть единицу из того самого интервала, который уходит в поле,
   а не взять вторую, независимо посчитанную оболочку той же величины.

   Хитрость в том, что «тот самый интервал» — это интервал ПОСЛЕ канонизации.
   `codec._canonical_decimal` вызывает `Decimal.normalize()`, то есть операцию
   контекста: φ ∈ (1, 2) с 28 знаками после точки — это 29 значащих цифр, и
   контекст с `prec = 28` округляет её на записи, тогда как δ < 1 с теми же 28
   знаками — ровно 28 значащих цифр и переживает запись без потерь. Поэтому
   δ, посчитанная от НЕканонизованной φ, после round-trip перестаёт быть
   φ − 1: расхождение в последней цифре, и ядро отвергает патч целиком.

   Округление наружу (`ROUND_FLOOR` низу, `ROUND_CEILING` верху) — не допуск:
   оболочка от расширения остаётся оболочкой, истинное значение внутри. А вот
   округление к ближайшему, которое делает `normalize()` за нас, оболочку рвёт:
   низ уезжает вверх, верх вниз, и сертификат перестаёт что-либо доказывать.

2. `validate_exported_snapshot` прогоняет снапшот через оба валидатора ядра и
   делает это ДВАЖДЫ: над объектом и над результатом канонического
   round-trip. Первое ловит ошибки сборки, второе — ошибки записи. Полевой
   отказ был именно вторым: в памяти `δ == φ − 1` держалось, а выпущенные байты
   ядро отвергало, и экспортёр этого не замечал.
"""

from __future__ import annotations

from decimal import (
    ROUND_CEILING,
    ROUND_FLOOR,
    Decimal,
    getcontext,
    localcontext,
)


class SnapshotExportRejected(ValueError):
    """Отказ приёмки экспорта, несущий исход хоста и домен для диагностики."""

    def __init__(
        self,
        outcome_name: str,
        message: str,
        *,
        patch_domain_id: str | None = None,
    ):
        self.outcome_name = str(outcome_name)
        self.patch_domain_id = patch_domain_id
        super().__init__(message)


def canonical_significant_digits() -> int:
    """Сколько значащих цифр переживает канонизацию `Decimal` в кодеке ядра.

    Читается из живого контекста, а не зашивается числом: `normalize()` в
    `codec._canonical_decimal` — операция контекста, и если контекст другой, то
    и предел другой. Ошибиться здесь нельзя тихо — round-trip в
    `validate_exported_snapshot` проверяет запись на самом деле.
    """

    return getcontext().prec


def _rounded_outward(value: Decimal, *, digits: int, upward: bool) -> Decimal:
    with localcontext() as context:
        context.prec = digits
        context.rounding = ROUND_CEILING if upward else ROUND_FLOOR
        return +value


def certified_reflex_measure(kernel, angle_measure, sine, cosine, orientation):
    """`CertifiedReflexAngleMeasureV1` из одной φ: δ выведена, а не посчитана.

    `angle_measure` — уже загруженный модуль `reference.angle_measure` ядра; он
    передаётся, а не импортируется здесь, потому что отсутствие ядра — это
    именованный отказ хоста (`ENVELOPE_DEBUG_KERNEL_UNAVAILABLE`), а не ошибка
    импорта модуля адаптера.

    Второй интервал, который `reflex_angle_intervals_over_pi` возвращает рядом
    с φ, намеренно не используется: это второе вычисление той же величины, и
    именно оно расходилось с φ в последней цифре после канонической записи.
    Отказ идёт тем же именем `CertifiedAngleUnavailable` — факт один и тот же
    («заверенной меры угла нет»), просто вскрывается на шаг позже.
    """

    interval_phi, _ = angle_measure.reflex_angle_intervals_over_pi(sine, cosine)
    digits = canonical_significant_digits()
    lower = _rounded_outward(interval_phi.lower, digits=digits, upward=False)
    upper = _rounded_outward(interval_phi.upper, digits=digits, upward=True)
    if lower <= Decimal(1) or upper >= Decimal(2):
        raise angle_measure.CertifiedAngleUnavailable(
            "canonical phi enclosure does not prove pi < phi < 2*pi: "
            f"phi/pi in [{lower}, {upper}]"
        )
    with localcontext() as context:
        # Запас над `digits` затем, чтобы вычитание единицы было точным, а не
        # ещё одним округлением: у δ ∈ (0, 1) значащих цифр на одну меньше, чем
        # у φ, и в `digits` она укладывается — но полагаться на это молча
        # значило бы снова оставить величину на милость контекста.
        context.prec = digits + 4
        width = upper - lower
        delta_lower = lower - Decimal(1)
        delta_upper = upper - Decimal(1)
    interval = kernel.CertifiedDecimalIntervalV1(
        lower,
        upper,
        interval_phi.lower_kind,
        interval_phi.upper_kind,
        width,
    )
    excess = kernel.CertifiedDecimalIntervalV1(
        delta_lower,
        delta_upper,
        interval_phi.lower_kind,
        interval_phi.upper_kind,
        width,
    )
    return kernel.CertifiedReflexAngleMeasureV1(
        interval,
        excess,
        orientation,
        kernel.AngleNormalizationLaw.VALUE_OVER_SYMBOLIC_PI_V1,
    )


def _reject_snapshot(kernel, snapshot, *, stage: str) -> None:
    issues = kernel.validate_analysis_snapshot(snapshot)
    if issues:
        raise SnapshotExportRejected(
            "ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID",
            f"{stage}: "
            + "; ".join(
                f"{item.code.value}:{'.'.join(item.path)}:{item.message}"
                for item in issues
            ),
        )
    validation = _reference_validation()
    for domain_id in sorted(
        (item.patch_domain_id for item in snapshot.patch_domains),
        key=lambda item: item.value,
    ):
        _, diagnostics = validation.validate_reference_geometry_payload(
            snapshot, domain_id
        )
        if diagnostics:
            raise SnapshotExportRejected(
                "ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE",
                f"{stage}: "
                + "; ".join(
                    f"{item.outcome.value}:{item.message}"
                    for item in diagnostics
                ),
                patch_domain_id=domain_id.value,
            )


def _reference_validation():
    import importlib

    return importlib.import_module("cftuv_envelope.reference.validation")


def validate_exported_snapshot(kernel, snapshot) -> bytes:
    """Оба валидатора ядра над объектом И над каноническими байтами.

    Проверенные байты возвращаются, а не выбрасываются: тому, кто их пишет,
    незачем считать их второй раз. Сегодняшний экспортёр снапшота их не пишет
    и результат игнорирует — записью фикстур занимается `tools/`.
    """

    _reject_snapshot(kernel, snapshot, stage="snapshot")
    payload = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    decoded = kernel.AnalysisSnapshotCodecV1.loads(payload)
    if kernel.AnalysisSnapshotCodecV1.dumps(decoded) != payload:
        raise SnapshotExportRejected(
            "ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID",
            "canonical snapshot bytes are not a fixed point of the codec",
        )
    _reject_snapshot(kernel, decoded, stage="canonical bytes")
    return payload


__all__ = (
    "SnapshotExportRejected",
    "canonical_significant_digits",
    "certified_reflex_measure",
    "validate_exported_snapshot",
)
