"""Сборщик телеметрии frontier-builder'а.

Модуль не изменяет поведение — только собирает данные.
Зависит только от solve_records и model (без circular imports).
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

from mathutils import Vector

try:
    from .model import (
        BoundaryChain, ChainRef, FrameRole, PlacementSourceKind,
    )
    from .solve_report_utils import (
        ReportingMode,
        ReportingOptions,
        coerce_reporting_options,
        format_chain_address,
    )
    from .solve_records import (
        ChainAnchor, ChainPoolEntry, FrontierCandidateEval,
        FrontierPlacementRecord, FrontierStallRecord, QuiltFrontierTelemetry,
        CHAIN_FRONTIER_THRESHOLD,
    )
    from .console_debug import trace_console
except ImportError:
    from model import (
        BoundaryChain, ChainRef, FrameRole, PlacementSourceKind,
    )
    from solve_report_utils import (
        ReportingMode,
        ReportingOptions,
        coerce_reporting_options,
        format_chain_address,
    )
    from solve_records import (
        ChainAnchor, ChainPoolEntry, FrontierCandidateEval,
        FrontierPlacementRecord, FrontierStallRecord, QuiltFrontierTelemetry,
        CHAIN_FRONTIER_THRESHOLD,
    )
    from console_debug import trace_console


# ============================================================
# Вспомогательные функции
# ============================================================

def _anchor_kind_label(anchor: Optional[ChainAnchor]) -> str:
    """Кодирует тип anchor в строку для FrontierPlacementRecord."""
    if anchor is None:
        return "none"
    if anchor.source_kind == PlacementSourceKind.CROSS_PATCH:
        return "cross_patch"
    return "same_patch"


def _anchor_debug_short(anchor: Optional[ChainAnchor]) -> str:
    """Краткая метка anchor для verbose-вывода (SP / XP / -)."""
    if anchor is None:
        return "-"
    return "XP" if anchor.source_kind == PlacementSourceKind.CROSS_PATCH else "SP"


def _uv_chain_length(uv_points: list[Vector]) -> float:
    """Суммарная длина UV-цепочки по последовательным точкам."""
    if len(uv_points) < 2:
        return 0.0
    return sum(
        (uv_points[i + 1] - uv_points[i]).length
        for i in range(len(uv_points) - 1)
    )


def _percentile_sorted(sorted_vals: list[float], pct: float) -> float:
    """Процентиль из уже отсортированного списка. pct в [0, 1]."""
    n = len(sorted_vals)
    if n == 0:
        return 0.0
    if n == 1:
        return sorted_vals[0]
    idx = pct * (n - 1)
    lo = int(idx)
    hi = lo + 1
    if hi >= n:
        return sorted_vals[-1]
    frac = idx - lo
    return sorted_vals[lo] + frac * (sorted_vals[hi] - sorted_vals[lo])


# ============================================================
# Промежуточный mutable stall-record (до финализации)
# ============================================================

@dataclass
class _PendingStall:
    """Промежуточный stall-record до обновления результата rescue."""
    iteration: int
    best_rejected_score: float
    best_rejected_ref: Optional[ChainRef]
    best_rejected_role: Optional[FrameRole]
    best_rejected_anchor_count: int
    available_count: int
    no_anchor_count: int
    below_threshold_count: int
    patches_with_placed: int
    patches_untouched: int
    rescue_attempted: str = "none"
    rescue_succeeded: bool = False

    def finalize(self) -> FrontierStallRecord:
        return FrontierStallRecord(
            iteration=self.iteration,
            best_rejected_score=self.best_rejected_score,
            best_rejected_ref=self.best_rejected_ref,
            best_rejected_role=self.best_rejected_role,
            best_rejected_anchor_count=self.best_rejected_anchor_count,
            available_count=self.available_count,
            no_anchor_count=self.no_anchor_count,
            below_threshold_count=self.below_threshold_count,
            rescue_attempted=self.rescue_attempted,
            rescue_succeeded=self.rescue_succeeded,
            patches_with_placed=self.patches_with_placed,
            patches_untouched=self.patches_untouched,
        )


# ============================================================
# FrontierTelemetryCollector
# ============================================================

@dataclass
class FrontierTelemetryCollector:
    """Mutable collector that accumulates records during frontier build.

    Created once per quilt. Passed to frontier loop and rescue functions.
    Finalized into immutable QuiltFrontierTelemetry at end of quilt.
    Collector is append-only — never read during the frontier loop.
    """

    quilt_index: int
    _t0: float = field(default_factory=time.perf_counter)
    _placement_records: list[FrontierPlacementRecord] = field(default_factory=list)
    _stall_records: list[FrontierStallRecord] = field(default_factory=list)
    _pending_stall: Optional[_PendingStall] = field(default=None)

    def record_placement(
        self,
        iteration: int,
        chain_ref: ChainRef,
        chain: BoundaryChain,
        placement_path: str,
        score: float,
        start_anchor: Optional[ChainAnchor],
        end_anchor: Optional[ChainAnchor],
        placed_in_patch_before: int,
        is_closure_pair: bool,
        uv_points: list[Vector],
        closure_preconstraint_applied: bool = False,
        anchor_adjustment_applied: bool = False,
        direction_inherited: bool = False,
        length_factor: float = 0.0,
        downstream_count: int = 0,
        downstream_bonus: float = 0.0,
        isolation_preview: bool = True,
        isolation_penalty: float = 0.0,
        structural_free_bonus: float = 0.0,
        hv_adjacency: int = 0,
    ) -> None:
        """Записывает одно успешное размещение chain."""
        anchor_count = (
            (1 if start_anchor is not None else 0)
            + (1 if end_anchor is not None else 0)
        )
        rec = FrontierPlacementRecord(
            iteration=iteration,
            chain_ref=chain_ref,
            frame_role=chain.frame_role,
            placement_path=placement_path,
            score=score,
            anchor_count=anchor_count,
            start_anchor_kind=_anchor_kind_label(start_anchor),
            end_anchor_kind=_anchor_kind_label(end_anchor),
            placed_in_patch_before=placed_in_patch_before,
            is_first_in_patch=(placed_in_patch_before == 0),
            is_bridge=(chain.frame_role == FrameRole.FREE and len(chain.vert_cos) <= 2),
            is_corner_split=chain.is_corner_split,
            neighbor_kind=chain.neighbor_kind.value,
            is_closure_pair=is_closure_pair,
            closure_preconstraint_applied=closure_preconstraint_applied,
            anchor_adjustment_applied=anchor_adjustment_applied,
            direction_inherited=direction_inherited,
            chain_length_uv=_uv_chain_length(uv_points),
            length_factor=length_factor,
            downstream_count=downstream_count,
            downstream_bonus=downstream_bonus,
            isolation_preview=isolation_preview,
            isolation_penalty=isolation_penalty,
            structural_free_bonus=structural_free_bonus,
            hv_adjacency=hv_adjacency,
        )
        self._placement_records.append(rec)
        trace_console(
            f"[CFTUV][Telemetry] Q{self.quilt_index} Step {iteration}: "
            f"{placement_path} {format_chain_address(chain_ref, quilt_index=self.quilt_index)} "
            f"{chain.frame_role.value} score:{score:.2f} ep:{anchor_count} "
            f"{_anchor_debug_short(start_anchor)}/{_anchor_debug_short(end_anchor)} "
            f"bridge:{'Y' if rec.is_bridge else 'N'} "
            f"closure:{'Y' if is_closure_pair else 'N'} "
            f"hv:{hv_adjacency}"
        )

    def record_stall(
        self,
        iteration: int,
        best_rejected_score: float,
        best_rejected_ref: Optional[ChainRef],
        best_rejected_role: Optional[FrameRole],
        best_rejected_anchor_count: int,
        available_count: int,
        no_anchor_count: int,
        below_threshold_count: int,
        patches_with_placed: int,
        patches_untouched: int,
    ) -> None:
        """Записывает stall-событие. Rescue-результат добавляется через update_last_stall_rescue."""
        # Финализируем предыдущий pending stall если есть (не должно быть)
        if self._pending_stall is not None:
            self._stall_records.append(self._pending_stall.finalize())

        self._pending_stall = _PendingStall(
            iteration=iteration,
            best_rejected_score=best_rejected_score,
            best_rejected_ref=best_rejected_ref,
            best_rejected_role=best_rejected_role,
            best_rejected_anchor_count=best_rejected_anchor_count,
            available_count=available_count,
            no_anchor_count=no_anchor_count,
            below_threshold_count=below_threshold_count,
            patches_with_placed=patches_with_placed,
            patches_untouched=patches_untouched,
        )

        role_str = best_rejected_role.value if best_rejected_role is not None else "NONE"
        ref_str = (
            format_chain_address(best_rejected_ref, quilt_index=self.quilt_index)
            if best_rejected_ref is not None else "-"
        )
        trace_console(
            f"[CFTUV][Telemetry] Q{self.quilt_index} Stall {iteration}: "
            f"best_rejected:{best_rejected_score:.3f} {ref_str} {role_str} "
            f"available:{available_count} no_anchor:{no_anchor_count}"
        )

    def update_last_stall_rescue(
        self,
        rescue_attempted: str,
        rescue_succeeded: bool,
    ) -> None:
        """Обновляет последний pending stall с результатом rescue-попытки."""
        if self._pending_stall is None:
            return
        self._pending_stall.rescue_attempted = rescue_attempted
        self._pending_stall.rescue_succeeded = rescue_succeeded
        self._stall_records.append(self._pending_stall.finalize())
        self._pending_stall = None

    def finalize(self) -> QuiltFrontierTelemetry:
        """Вычисляет агрегаты и возвращает immutable QuiltFrontierTelemetry."""
        # Финализируем незакрытый stall если есть
        if self._pending_stall is not None:
            self._stall_records.append(self._pending_stall.finalize())
            self._pending_stall = None

        duration = time.perf_counter() - self._t0

        # Счётчики по путям
        total = len(self._placement_records)
        main_count = sum(1 for r in self._placement_records if r.placement_path == "main")
        tree_count = sum(1 for r in self._placement_records if r.placement_path == "tree_ingress")
        free_count = sum(1 for r in self._placement_records if r.placement_path == "free_ingress")
        closure_count = sum(1 for r in self._placement_records if r.placement_path == "closure_follow")

        # Stall-агрегаты
        total_stalls = len(self._stall_records)
        stalls_resolved = sum(1 for s in self._stall_records if s.rescue_succeeded)
        stalls_unresolved = total_stalls - stalls_resolved

        # Score-статистика только для main-пути
        main_scores = sorted(
            r.score for r in self._placement_records
            if r.placement_path == "main" and r.score >= 0.0
        )
        if main_scores:
            score_min = main_scores[0]
            score_max = main_scores[-1]
            score_mean = sum(main_scores) / len(main_scores)
            score_p25 = _percentile_sorted(main_scores, 0.25)
            score_p50 = _percentile_sorted(main_scores, 0.50)
            score_p75 = _percentile_sorted(main_scores, 0.75)
        else:
            score_min = score_max = score_mean = 0.0
            score_p25 = score_p50 = score_p75 = 0.0

        best_rejected_max = max(
            (s.best_rejected_score for s in self._stall_records),
            default=0.0,
        )

        # Первая rescue-итерация
        rescue_iters = [
            r.iteration for r in self._placement_records
            if r.placement_path != "main"
        ]
        first_rescue_iteration = min(rescue_iters) if rescue_iters else -1

        rescue_total = tree_count + free_count + closure_count
        rescue_ratio = rescue_total / total if total > 0 else 0.0

        return QuiltFrontierTelemetry(
            quilt_index=self.quilt_index,
            total_placements=total,
            main_placements=main_count,
            tree_ingress_placements=tree_count,
            free_ingress_placements=free_count,
            closure_follow_placements=closure_count,
            total_stalls=total_stalls,
            stalls_resolved_by_rescue=stalls_resolved,
            stalls_unresolved=stalls_unresolved,
            score_min=score_min,
            score_max=score_max,
            score_mean=score_mean,
            score_p25=score_p25,
            score_p50=score_p50,
            score_p75=score_p75,
            best_rejected_score_max=best_rejected_max,
            first_rescue_iteration=first_rescue_iteration,
            rescue_ratio=rescue_ratio,
            frontier_duration_sec=duration,
            placement_records=tuple(self._placement_records),
            stall_records=tuple(self._stall_records),
        )


# ============================================================
# Вычисление stall-данных в момент останова frontier
# (вызывается из solve_frontier.py, не из collector)
# ============================================================

def collect_stall_snapshot(
    cached_evals: dict,
    placed_chain_refs: set,
    rejected_chain_refs: set,
    placed_count_by_patch: dict,
    quilt_patch_ids: set,
    all_chain_pool: list,
) -> tuple:
    """Snapshot состояния frontier в момент stall.

    Принимает уже вычисленные поля из FrontierRuntimePolicy без импорта
    самого класса. Возвращает (score, ref, role, anchor_count, available,
    no_anchor, below_threshold, patches_with_placed, patches_untouched).
    """
    best_score = -1.0
    best_ref: Optional[ChainRef] = None
    best_role: Optional[FrameRole] = None
    best_anchor_count = 0

    available_count = 0
    no_anchor_count = 0
    below_threshold_count = 0

    for entry in all_chain_pool:
        ref = entry.chain_ref
        if ref in placed_chain_refs or ref in rejected_chain_refs:
            continue
        available_count += 1

        eval_result: Optional[FrontierCandidateEval] = cached_evals.get(ref)
        if eval_result is None:
            no_anchor_count += 1
            continue

        known = eval_result.known
        score = eval_result.score
        if known == 0:
            no_anchor_count += 1
        elif 0.0 <= score < CHAIN_FRONTIER_THRESHOLD:
            below_threshold_count += 1

        if score > best_score:
            best_score = score
            best_ref = ref
            best_role = entry.chain.frame_role
            best_anchor_count = known

    # Patches с хотя бы одним placed chain
    patches_with_placed = len(placed_count_by_patch)
    patches_untouched = sum(
        1 for pid in quilt_patch_ids if pid not in placed_count_by_patch
    )

    return (
        best_score,
        best_ref,
        best_role,
        best_anchor_count,
        available_count,
        no_anchor_count,
        below_threshold_count,
        patches_with_placed,
        patches_untouched,
    )


# ============================================================
# Форматирование для reporting
# ============================================================

def format_quilt_telemetry_summary(
    t: QuiltFrontierTelemetry,
    reporting: Optional[ReportingOptions] = None,
    *,
    mode: Optional[ReportingMode | str] = None,
) -> list[str]:
    """Краткая сводка телеметрии для regression snapshot (одна секция)."""
    reporting = coerce_reporting_options(reporting, mode=mode)
    lines = [
        "frontier_telemetry:",
        (
            f"  placements: {t.total_placements} "
            f"main:{t.main_placements} "
            f"tree_ingress:{t.tree_ingress_placements} "
            f"free_ingress:{t.free_ingress_placements} "
            f"closure_follow:{t.closure_follow_placements}"
        ),
        (
            f"  stalls: {t.total_stalls} "
            f"resolved:{t.stalls_resolved_by_rescue} "
            f"unresolved:{t.stalls_unresolved}"
        ),
        (
            f"  scores: min:{t.score_min:.3f} "
            f"p25:{t.score_p25:.3f} "
            f"p50:{t.score_p50:.3f} "
            f"p75:{t.score_p75:.3f} "
            f"max:{t.score_max:.3f}"
        ),
        f"  rescue_ratio: {t.rescue_ratio:.3f}",
        f"  best_rejected_max: {t.best_rejected_score_max:.3f}",
        f"  duration: {t.frontier_duration_sec:.3f}s",
    ]
    return lines


def format_quilt_telemetry_detail(
    t: QuiltFrontierTelemetry,
    reporting: Optional[ReportingOptions] = None,
    *,
    mode: Optional[ReportingMode | str] = None,
) -> list[str]:
    """Подробный лог размещений и stall-событий для verbose-вывода."""
    reporting = coerce_reporting_options(reporting, mode=mode)
    lines: list[str] = []
    q = t.quilt_index

    # Объединяем placement и stall в хронологическом порядке по iteration
    stall_by_iter: dict[int, FrontierStallRecord] = {s.iteration: s for s in t.stall_records}
    placement_by_iter: dict[int, list[FrontierPlacementRecord]] = {}
    for r in t.placement_records:
        placement_by_iter.setdefault(r.iteration, []).append(r)

    all_iters = sorted(set(stall_by_iter) | set(placement_by_iter))
    for it in all_iters:
        stall = stall_by_iter.get(it)
        if stall is not None:
            ref_str = (
                format_chain_address(stall.best_rejected_ref, quilt_index=q)
                if stall.best_rejected_ref is not None else "-"
            )
            role_str = stall.best_rejected_role.value if stall.best_rejected_role is not None else "NONE"
            lines.append(
                f"[CFTUV][Telemetry] Q{q} Stall {it}: "
                f"best_rejected:{stall.best_rejected_score:.3f} "
                f"{ref_str} {role_str} "
                f"available:{stall.available_count} "
                f"no_anchor:{stall.no_anchor_count} "
                f"rescue:{stall.rescue_attempted}={'Y' if stall.rescue_succeeded else 'N'}"
            )
        for r in placement_by_iter.get(it, []):
            lines.append(
                f"[CFTUV][Telemetry] Q{q} Step {it}: "
                f"{r.placement_path} "
                f"{format_chain_address(r.chain_ref, quilt_index=q)} "
                f"{r.frame_role.value} score:{r.score:.2f} "
                f"ep:{r.anchor_count} "
                f"{r.start_anchor_kind[:2].upper()}/{r.end_anchor_kind[:2].upper()} "
                f"bridge:{'Y' if r.is_bridge else 'N'} "
                f"closure:{'Y' if r.is_closure_pair else 'N'} "
                f"[lf:{r.length_factor:.2f} ds:{r.downstream_count}:{r.downstream_bonus:.2f} "
                f"iso:{r.isolation_preview}:{r.isolation_penalty:.2f} sfb:{r.structural_free_bonus:.2f} "
                f"hv:{r.hv_adjacency}]"
            )
    return lines
