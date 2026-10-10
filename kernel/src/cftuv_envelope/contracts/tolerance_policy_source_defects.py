"""Записи реестра допусков для двери ПРЕДПОЛЁТА ИСТОЧНИКА (`source_defects`).

Реестр (`tolerance_policy`) стоит на потолке размера модуля, поэтому записи этой двери лежат здесь, а реестр собирает их
в конце своего тела. Типы записи принадлежат реестру: функция берёт их при вызове, а не на импорте, и модуль можно
импортировать первым без цикла.
"""

from __future__ import annotations

from fractions import Fraction


def source_defect_policies() -> tuple:
    """Записи двери предполёта: допуск зазора контакта (T-вершина, самопересечение грани)."""

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

    return (
        TolerancePolicyV1(
            id=TolerancePolicyIdV1.SOURCE_CONTACT_GAP_V1,
            category=TolerancePolicyCategoryV1.AUTHORING_INTENT,
            value=_rational(Fraction(7, 10**6)),
            bound_law=None,
            units=TolerancePolicyUnitsV1.DIMENSIONLESS,
            coordinate_space=TolerancePolicyCoordinateSpaceV1.SOURCE_LOCAL_INTRINSIC,
            scaling_law=TolerancePolicyScalingLawV1.RELATIVE_TO_PATCH_EXTENT,
            scope=(
                "ЕЩЁ ОДНА дверь авторской ошибки: ПРЕДПОЛЁТ ИСТОЧНИКА хоста (`source_defects`). Зазор в долях габарита патча "
                "(`AUTHOR_ANGULAR_ERROR * extent`, то есть ровно ПОЛОВИНА нижней границы окна шага решётки): вершина патча "
                "в таком зазоре от ребра патча, которому не принадлежит (ближайшая точка строго внутри ребра, зазор "
                "положителен), — T-вершина `SOURCE_T_VERTEX`; два не смежных ребра одной грани, чьи ближайшие точки внутри "
                "рёбер разведены не дальше, — самопересечение `SOURCE_FACE_SELF_INTERSECTION`. Любая допустимая ячейка "
                "не мельче нижней границы, поэтому привязка к решётке способна замкнуть такой зазор на ЛЮБОМ допустимом "
                "масштабе: это ошибка авторства меньше половины ячейки, а не деталь модели. Поле: наименьший честный зазор "
                "`cover.008` — 2.1 мм, дефекты — 5.7 мкм и 4e-08 м. Допуск только сужает принимаемое множество до "
                "именованного отказа хоста ДО ядра; ничего не сваривает и не двигает. Значение и место те же, что у "
                "AUTHOR_ANGULAR_ERROR: копии величины не существует."
            ),
            authority=(
                "source_defects.source_contact_gap; envelope_source_contacts (хост); DECISIONS.md 2026-10-10 "
                "(SOURCE-DEFECT-PREFLIGHT: находка поля cover.008, патчи 18 и 285) и 2026-07-25 (решение владельца о "
                "величине авторской ошибки)"
            ),
            applied_stage=TolerancePolicyAppliedStageV1.SOURCE_DEFECT_PREFLIGHT,
            allowed_effect=TolerancePolicyAllowedEffectV1.NARROW_ADMITTED_SET_TOWARD_REFUSAL,
            changes_topology=False,
            preview_or_final=TolerancePolicyPipelineStageV1.FINAL_PRODUCT_PATH,
            telemetry_counters=(),
            declaration_sites=("cftuv_envelope._authoring_intent.AUTHOR_ANGULAR_ERROR",),
            positive_fixture=(
                f"{_KERNEL_TESTS}/test_source_defects.py"
                "::test_a_vertex_beside_an_edge_within_the_authoring_gap_is_named"
            ),
            negative_fixture=(
                f"{_KERNEL_TESTS}/test_source_defects.py"
                "::test_a_vertex_beyond_the_authoring_gap_is_not_a_defect"
            ),
        ),
    )


__all__ = ("source_defect_policies",)
