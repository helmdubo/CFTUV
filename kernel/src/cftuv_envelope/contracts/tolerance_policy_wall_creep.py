"""Запись реестра допусков для ползущих стен частичного выделения швов (WALL_CREEP_V1, `wavefront.wall_creep`).

Реестр (`tolerance_policy`) стоит на потолке размера модуля, поэтому запись лежит здесь, а реестр собирает её в конце
своего тела. Типы записи принадлежат реестру: функция берёт их при вызове, а не на импорте, и цикла нет.
"""

from __future__ import annotations

from fractions import Fraction


def wall_creep_policies() -> tuple:
    """Запись скорости ползущей стены: невыделенное ребро домена движется со скоростью `eps` самого медленного источника."""

    from .tolerance_policy import (
        _KERNEL_TESTS,
        TolerancePolicyAllowedEffectV1,
        TolerancePolicyAppliedStageV1,
        TolerancePolicyCategoryV1,
        TolerancePolicyCoordinateSpaceV1,
        TolerancePolicyIdV1,
        TolerancePolicyPipelineStageV1,
        TolerancePolicyScalingLawV1,
        TolerancePolicyUnitsV1,
        TolerancePolicyV1,
        _rational,
    )

    tests = f"{_KERNEL_TESTS}/test_wavefront_wall_creep.py"
    return (
        TolerancePolicyV1(
            id=TolerancePolicyIdV1.WALL_CREEP_SPEED_V1,
            category=TolerancePolicyCategoryV1.STRUCTURAL_QUANTIZATION,
            value=_rational(Fraction(1, 1 << 20)),
            bound_law=None,
            units=TolerancePolicyUnitsV1.DIMENSIONLESS,
            coordinate_space=TolerancePolicyCoordinateSpaceV1.DECAL_UV_ALPHA_UNITS,
            scaling_law=TolerancePolicyScalingLawV1.RELATIVE_TO_DECAL_ALPHA,
            scope=(
                "Мост очереди (`bridge_arrival_laws(creeping_walls=True)`): ребро домена без закона прихода (невыделенный "
                "шов) — не неподвижная стена q = 0, на которой взвешенный скелет оставлял фронт незакрытым "
                "(WAVEFRONT_LEFT_UNRESOLVED), а ПОЛЗУЩИЙ источник с нормальной скоростью eps * sqrt(base), base — наименьший "
                "квадрат скорости настоящих источников домена (корень снизу, рациональный, один на домен). Торец декали "
                "ложится заподлицо с невыделенным соседом с ошибкой ~eps * alpha, грани стен выбрасываются из разбиения "
                "(`drop_wall_faces`). Действует только во ВТОРОМ проходе домена, чьи неподвижные стены не закрылись "
                "(`needs_creeping_walls`): где они закрываются, и когда стен нет, ответ побайтно прежний."
            ),
            authority=(
                "wavefront.wall_creep.WALL_CREEP_EPSILON; DECISIONS.md 2026-11-02 (WALL_CREEP_V1: одиночные цепочки на "
                "замороженной сцене 288 -> 30 отказов, торцы заподлицо, невязка p50 5e-5 alpha)"
            ),
            applied_stage=TolerancePolicyAppliedStageV1.WAVEFRONT_BRIDGE_UNSELECTED_EDGE,
            allowed_effect=TolerancePolicyAllowedEffectV1.CREEP_UNSELECTED_EDGE_WITHIN_EPSILON_OF_ALPHA,
            changes_topology=True,
            preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
            telemetry_counters=("CONVEYOR_WALL_CREEP_V1_WALLS", "CONVEYOR_WALL_CREEP_V1_T_NOTCHES"),
            declaration_sites=("cftuv_envelope.wavefront.wall_creep.WALL_CREEP_EPSILON",),
            positive_fixture=f"{tests}::test_creeping_wall_speed_is_eps_of_the_slowest_source",
            negative_fixture=f"{tests}::test_all_seams_selected_is_unchanged",
        ),
    )


__all__ = ("wall_creep_policies",)
