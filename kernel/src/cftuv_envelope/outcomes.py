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
    # Границы окна шага решётки разошлись: авторская ошибка на этом габарите
    # требует шага крупнее, чем позволяет деталь декали. Крупный шаг «на глаз»
    # запрещён, поэтому исход именованный.
    GRID_WINDOW_CLOSED = "GRID_WINDOW_CLOSED"
    # Окно открыто, но между границами нет ни одной степени двойки. Причина не
    # в геометрии, поэтому и исход отдельный.
    NO_POWER_OF_TWO_STEP_IN_WINDOW = "NO_POWER_OF_TWO_STEP_IN_WINDOW"
    # Привязка источника выполнена, но угол, объявленный задуманно прямым, так
    # и не дал точно рациональной доли π. Значит объявленная авторская ошибка
    # не описывает ЭТОТ меш, и продолжать на веру нельзя.
    SOURCE_SNAP_DID_NOT_RESTORE_RELATIONS = (
        "SOURCE_SNAP_DID_NOT_RESTORE_RELATIONS"
    )
