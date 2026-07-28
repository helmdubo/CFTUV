"""Runtime receipts and performance telemetry for staged Envelope debug."""

from __future__ import annotations

import time
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from enum import Enum


ENVELOPE_DEBUG_PROFILE_SCHEMA_V1 = "cftuv.envelope.debug_profile.v1"


class EnvelopeDomainStage(str, Enum):
    TOPOLOGY_READY = "TOPOLOGY_READY"
    METRIC_REJECTED = "METRIC_REJECTED"
    COMPILE_REJECTED = "COMPILE_REJECTED"
    RAW_REJECTED = "RAW_REJECTED"
    INTERACTION_REJECTED = "INTERACTION_REJECTED"
    RESOLVED = "RESOLVED"
    # Стадии движка QUEUE. Названы отдельно от RAW/INTERACTION намеренно:
    # очередь не проходит ни союз, ни попарный крой, и «RAW_REJECTED» на её
    # отказе означал бы ступень, которой в этом пути нет вовсе.
    QUEUE_PREPARE_REJECTED = "QUEUE_PREPARE_REJECTED"
    QUEUE_COVERAGE_REJECTED = "QUEUE_COVERAGE_REJECTED"
    QUEUE_RESOLVED = "QUEUE_RESOLVED"


@dataclass(frozen=True, slots=True)
class EnvelopeDomainStageReceiptV1:
    patch_id: int
    patch_domain_id: str
    stage: EnvelopeDomainStage
    outcome: str
    message: str


@dataclass(frozen=True, slots=True)
class EnvelopeDebugTimingV1:
    stage: str
    elapsed_seconds: float
    patch_domain_id: str | None = None


@dataclass(frozen=True, slots=True)
class EnvelopeDebugCounterV1:
    name: str
    value: int | float
    patch_domain_id: str | None = None


@dataclass(frozen=True, slots=True)
class EnvelopeDebugProfileV1:
    schema_version: str
    source_name: str
    build_kind: str
    timings: tuple[EnvelopeDebugTimingV1, ...]
    counters: tuple[EnvelopeDebugCounterV1, ...]
    receipts: tuple[EnvelopeDomainStageReceiptV1, ...]

    @property
    def stage_totals(self) -> dict[str, float]:
        totals: dict[str, float] = {}
        for timing in self.timings:
            totals[timing.stage] = (
                totals.get(timing.stage, 0.0) + timing.elapsed_seconds
            )
        return totals

    @property
    def dominant_stage(self) -> tuple[str, float] | None:
        totals = self.stage_totals
        if not totals:
            return None
        return max(totals.items(), key=lambda item: (item[1], item[0]))

    def stage_summary(self) -> dict[str, int]:
        total = len(self.receipts)
        metric = sum(
            receipt.stage
            not in {
                EnvelopeDomainStage.TOPOLOGY_READY,
                EnvelopeDomainStage.METRIC_REJECTED,
            }
            for receipt in self.receipts
        )
        # QUEUE_RESOLVED засчитывается в те же два столбца, что и RESOLVED:
        # столбцы называют «домен дошёл до покрытия», а не «каким движком».
        # Новых ключей не заводится — сводка читается одинаково обоими путями.
        raw = sum(
            receipt.stage
            in {
                EnvelopeDomainStage.INTERACTION_REJECTED,
                EnvelopeDomainStage.RESOLVED,
                EnvelopeDomainStage.QUEUE_COVERAGE_REJECTED,
                EnvelopeDomainStage.QUEUE_RESOLVED,
            }
            for receipt in self.receipts
        )
        resolved = sum(
            receipt.stage
            in {
                EnvelopeDomainStage.RESOLVED,
                EnvelopeDomainStage.QUEUE_RESOLVED,
            }
            for receipt in self.receipts
        )
        return {
            "topology": total,
            "metric": metric,
            "raw": raw,
            "resolved": resolved,
            "total": total,
        }

    def to_payload(self) -> dict:
        dominant = self.dominant_stage
        return {
            "schema": self.schema_version,
            "source_name": self.source_name,
            "build_kind": self.build_kind,
            "stage_summary": self.stage_summary(),
            "dominant_stage": (
                {
                    "stage": dominant[0],
                    "elapsed_seconds": dominant[1],
                }
                if dominant is not None
                else None
            ),
            "timings": [asdict(item) for item in self.timings],
            "counters": [asdict(item) for item in self.counters],
            "receipts": [
                {
                    **asdict(item),
                    "stage": item.stage.value,
                }
                for item in self.receipts
            ],
        }


# Живой след стадий. Включён по умолчанию: до него длинный прогон было не
# отличить от зависшего, а именно так и выглядел первый успешный проход
# центрального патча building.002.
LIVE_STAGE_TRACE = True

# Только те стадии, которые могут идти долго. Остальные дают доли миллисекунды
# и превратили бы след в шум.
LIVE_TRACED_STAGES = frozenset({
    "ANGULAR_RELATIONS",
    "COMPILE",
    "DOMAIN_BUILD",
    "ENVELOPE_INSTANCE_BUILD",
    "DOMAIN_CLIP",
    "RAW_UNION",
    "INTERACTION",
    "GP_RENDER",
    "QUEUE_PREPARE",
    "QUEUE_COVERAGE",
})


# Стадия ANGULAR_RELATIONS писала только тайминг, поэтому её нулевой выход был
# неотличим от отсутствия углов: пусто и пусто. Ключи объявлены перечнем, чтобы
# стадия печатала все четыре числа даже там, где ни одно из них не выросло.
ANGULAR_STAGE_COUNTERS = (
    "ANGULAR_CORNERS_CONSIDERED",
    "ANGULAR_COLLINEAR_SKIPPED",
    "ANGULAR_REFLEX_CORNERS",
    "ANGULAR_RELATIONS_BUILT",
)


def _counter_total(profile: EnvelopeDebugProfileV1, name: str) -> int:
    return sum(int(item.value) for item in profile.counters if item.name == name)


def stage_summary_text(profile: EnvelopeDebugProfileV1) -> str:
    """Строка панели: сколько доменов дошло до какой ступени.

    Дополнение выделения дописывается той же строкой, а не остаётся только
    счётчиком в JSON: владелец нажимает кнопку одним ребром шва, а считается
    вся цепочка, и молчание об этом читалось бы как «оно так и было выделено».
    """

    summary = profile.stage_summary()
    text = (
        f"Topology: {summary['topology']}/{summary['total']} | "
        f"Metric: {summary['metric']}/{summary['total']} | "
        f"Raw: {summary['raw']}/{summary['total']} | "
        f"Resolved: {summary['resolved']}/{summary['total']}"
    )
    chains = _counter_total(profile, "SELECTION_COMPLETED_CHAINS")
    edges = _counter_total(profile, "SELECTION_COMPLETED_EDGES")
    if chains or edges:
        text += f" | Selection completed: {chains} chains, +{edges} edges"
    return text


class EnvelopeDebugProfileBuilderV1:
    """Local mutable collector; published snapshots remain immutable."""

    def __init__(self, source_name: str, build_kind: str):
        self.source_name = str(source_name)
        self.build_kind = str(build_kind)
        self._timings: list[EnvelopeDebugTimingV1] = []
        self._counters: dict[
            tuple[str, str | None], EnvelopeDebugCounterV1
        ] = {}
        self._receipts: dict[str, EnvelopeDomainStageReceiptV1] = {}

    @contextmanager
    def measure(self, stage: str, patch_domain_id: str | None = None):
        live = LIVE_STAGE_TRACE and stage in LIVE_TRACED_STAGES
        if live:
            # Строка печатается ДО работы и не закрывается до её конца. Долгий
            # прогон при этом читается, а не выглядит зависшим: последняя
            # незакрытая строка и есть место, где всё стоит.
            suffix = f" {patch_domain_id[-3:]}" if patch_domain_id else ""
            print(f"  [CFTUV] {stage}{suffix} ...", end="", flush=True)
        started = time.perf_counter()
        try:
            yield
        finally:
            elapsed = time.perf_counter() - started
            if live:
                print(f" {elapsed * 1000:.1f} ms", flush=True)
            self.add_timing(stage, elapsed, patch_domain_id)

    def add_timing(
        self,
        stage: str,
        elapsed_seconds: float,
        patch_domain_id: str | None = None,
    ) -> None:
        self._timings.append(
            EnvelopeDebugTimingV1(
                str(stage),
                max(0.0, float(elapsed_seconds)),
                patch_domain_id,
            )
        )

    def set_counter(
        self,
        name: str,
        value: int | float,
        patch_domain_id: str | None = None,
    ) -> None:
        key = (str(name), patch_domain_id)
        self._counters[key] = EnvelopeDebugCounterV1(
            key[0],
            value,
            patch_domain_id,
        )

    def add_counter(
        self,
        name: str,
        value: int | float,
        patch_domain_id: str | None = None,
    ) -> None:
        key = (str(name), patch_domain_id)
        previous = self._counters.get(key)
        self.set_counter(
            key[0],
            (previous.value if previous is not None else 0) + value,
            patch_domain_id,
        )

    def set_receipt(self, receipt: EnvelopeDomainStageReceiptV1) -> None:
        self._receipts[receipt.patch_domain_id] = receipt

    def snapshot(self) -> EnvelopeDebugProfileV1:
        return EnvelopeDebugProfileV1(
            ENVELOPE_DEBUG_PROFILE_SCHEMA_V1,
            self.source_name,
            self.build_kind,
            tuple(self._timings),
            tuple(
                self._counters[key]
                for key in sorted(
                    self._counters,
                    key=lambda item: (item[0], item[1] or ""),
                )
            ),
            tuple(
                self._receipts[key]
                for key in sorted(self._receipts)
            ),
        )


__all__ = (
    "ANGULAR_STAGE_COUNTERS",
    "ENVELOPE_DEBUG_PROFILE_SCHEMA_V1",
    "EnvelopeDebugCounterV1",
    "EnvelopeDebugProfileBuilderV1",
    "EnvelopeDebugProfileV1",
    "EnvelopeDebugTimingV1",
    "EnvelopeDomainStage",
    "EnvelopeDomainStageReceiptV1",
    "stage_summary_text",
)
