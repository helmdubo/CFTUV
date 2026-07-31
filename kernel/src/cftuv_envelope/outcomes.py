"""Именованные capability outcomes; silent fallback запрещён."""

from enum import Enum


class NamedOutcome(str, Enum):
    DECAL_ANALYSIS_SCHEMA_UNSUPPORTED = "DECAL_ANALYSIS_SCHEMA_UNSUPPORTED"
    BARRIER_SPLIT_REQUIRED = "BARRIER_SPLIT_REQUIRED"
    BARRIER_BYPASS_UNSUPPORTED = "BARRIER_BYPASS_UNSUPPORTED"
    SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN = "SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN"
    OWNERSHIP_PARTITION_UNPROVEN = "OWNERSHIP_PARTITION_UNPROVEN"
    PENDING_EXACT_EVALUATION = "PENDING_EXACT_EVALUATION"
    APPROXIMATE_MATERIALIZATION_PENDING = "APPROXIMATE_MATERIALIZATION_PENDING"
    JUNCTION_ROUTE_PAIRING_REQUIRED = "JUNCTION_ROUTE_PAIRING_REQUIRED"
    ANGULAR_PROFILE_SELECTION_UNCERTAIN = "ANGULAR_PROFILE_SELECTION_UNCERTAIN"
    RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED = (
        "RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED"
    )
    # Источник отклоняется от плоскости больше объявленного бюджета: это уже
    # не шум представления, а другая геометрия. Отказ именованный.
    NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED = "NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED"
    # Источник был компланарен, а закон решётки вывел его из собственной
    # плоскости патча. Причина не в геометрии автора и не в бюджете: её внёс
    # сам снап. Отдельное имя нужно потому, что прежде этот случай выходил под
    # именем бюджета и полевое сообщение винило меш вместо решётки.
    GRID_SNAP_LEFT_THE_PATCH_PLANE = "GRID_SNAP_LEFT_THE_PATCH_PLANE"
    # Границы окна шага решётки разошлись: авторская ошибка на этом габарите
    # требует шага крупнее, чем позволяет деталь декали. Крупный шаг «на глаз»
    # запрещён, поэтому исход именованный.
    GRID_WINDOW_CLOSED = "GRID_WINDOW_CLOSED"
    # Окно открыто, но между границами нет ни одной степени двойки. Причина не
    # в геометрии, поэтому и исход отдельный.
    NO_POWER_OF_TWO_STEP_IN_WINDOW = "NO_POWER_OF_TWO_STEP_IN_WINDOW"
    # Перебраны ВСЕ степени двойки внутри окна, и ни на одной угол,
    # объявленный задуманно прямым, не дал точно рациональной доли π. Значит
    # объявленная авторская ошибка не описывает ЭТОТ меш, и продолжать на веру
    # нельзя. Отменяет прежний `SOURCE_SNAP_DID_NOT_RESTORE_RELATIONS`: тот
    # называл отказ одного масштаба, а масштаб теперь не один — отказ
    # принадлежит всему окну, и назван он так, чтобы это было видно.
    NO_GRID_SCALE_RESTORES_RELATIONS = "NO_GRID_SCALE_RESTORES_RELATIONS"
