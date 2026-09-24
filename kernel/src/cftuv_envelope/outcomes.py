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
    # Перебраны ВСЕ степени двойки внутри окна, и ни на одной угол,
    # объявленный задуманно прямым, не дал точно рациональной доли π. Значит
    # объявленная авторская ошибка не описывает ЭТОТ меш, и продолжать на веру
    # нельзя. Отменяет прежний `SOURCE_SNAP_DID_NOT_RESTORE_RELATIONS`: тот
    # называл отказ одного масштаба, а масштаб теперь не один — отказ
    # принадлежит всему окну, и назван он так, чтобы это было видно.
    NO_GRID_SCALE_RESTORES_RELATIONS = "NO_GRID_SCALE_RESTORES_RELATIONS"
    SOURCE_SNAP_VERTEX_INJECTIVITY_VIOLATED = (
        "SOURCE_SNAP_VERTEX_INJECTIVITY_VIOLATED"
    )
    SOURCE_SNAP_NONZERO_EDGE_COLLAPSED = "SOURCE_SNAP_NONZERO_EDGE_COLLAPSED"
    SOURCE_SNAP_NEW_NONADJACENT_EDGE_INTERSECTION = (
        "SOURCE_SNAP_NEW_NONADJACENT_EDGE_INTERSECTION"
    )
    SOURCE_SNAP_INTENDED_RIGHT_CORNER_DEGENERATED = (
        "SOURCE_SNAP_INTENDED_RIGHT_CORNER_DEGENERATED"
    )
    NEAR_PLANAR_PROJECTION_BOUNDARY_INJECTIVITY_VIOLATED = (
        "NEAR_PLANAR_PROJECTION_BOUNDARY_INJECTIVITY_VIOLATED"
    )
    NEAR_PLANAR_PROJECTION_NONZERO_BOUNDARY_EDGE_COLLAPSED = (
        "NEAR_PLANAR_PROJECTION_NONZERO_BOUNDARY_EDGE_COLLAPSED"
    )
    NEAR_PLANAR_PROJECTION_NEW_NONADJACENT_EDGE_INTERSECTION = (
        "NEAR_PLANAR_PROJECTION_NEW_NONADJACENT_EDGE_INTERSECTION"
    )
    NEAR_PLANAR_PROJECTION_NEW_COLLINEAR_EDGE_OVERLAP = (
        "NEAR_PLANAR_PROJECTION_NEW_COLLINEAR_EDGE_OVERLAP"
    )
    NEAR_PLANAR_PROJECTION_LOOP_ORIENTATION_CHANGED = (
        "NEAR_PLANAR_PROJECTION_LOOP_ORIENTATION_CHANGED"
    )
    NEAR_PLANAR_PROJECTION_BOUNDARY_COMPONENT_COUNT_CHANGED = (
        "NEAR_PLANAR_PROJECTION_BOUNDARY_COMPONENT_COUNT_CHANGED"
    )
    NEAR_PLANAR_PROJECTION_OUTER_HOLE_NESTING_CHANGED = (
        "NEAR_PLANAR_PROJECTION_OUTER_HOLE_NESTING_CHANGED"
    )
    # СНЯТ С ПРОИЗВОДСТВА (P0-4-HARDENING), имя оставлено намеренно.
    # Циклический порядок петли границы — это последовательность
    # `PhysicalEdgeId` в обходе граней ИСТОЧНИКА. Проекция — отображение
    # ПОЗИЦИЙ вершин; комбинаторику граней она не трогает вообще, поэтому
    # входа, на котором «до» и «после» разошлись бы, не существует. Исход,
    # который нельзя выпустить ни на каком входе, — не отказ, а украшение;
    # он снят с производства, а не переименован, потому что удаление члена
    # enum ломает две уже выпущенные схемы неаддитивно.
    # Геометрическое содержание, которое имя обещало, несут два живых
    # исхода: `NEAR_PLANAR_PROJECTION_FAN_IDENTITY_CHANGED` (система вращения
    # считается независимо на карте источника и на карте проекции) и
    # `NEAR_PLANAR_PROJECTION_INTERIOR_OVERLAP` (прямая власть по инъективности).
    NEAR_PLANAR_PROJECTION_CYCLIC_ORDER_CHANGED = (
        "NEAR_PLANAR_PROJECTION_CYCLIC_ORDER_CHANGED"
    )
    NEAR_PLANAR_PROJECTION_SOURCE_ANCHOR_IDENTITY_CHANGED = (
        "NEAR_PLANAR_PROJECTION_SOURCE_ANCHOR_IDENTITY_CHANGED"
    )
    NEAR_PLANAR_PROJECTION_RESOLVED_PLANE_BASIS_UNAVAILABLE = (
        "NEAR_PLANAR_PROJECTION_RESOLVED_PLANE_BASIS_UNAVAILABLE"
    )
    NEAR_PLANAR_PROJECTION_FAN_IDENTITY_CHANGED = (
        "NEAR_PLANAR_PROJECTION_FAN_IDENTITY_CHANGED"
    )
    # Две РАЗЛИЧНЫЕ вершины источника совпали в карте проекции. Прежняя
    # проверка смотрела только вхождения ГРАНИЦЫ, поэтому схлопывание
    # внутренней вершины проходило молча.
    NEAR_PLANAR_PROJECTION_VERTEX_INJECTIVITY_VIOLATED = (
        "NEAR_PLANAR_PROJECTION_VERTEX_INJECTIVITY_VIOLATED"
    )
    # Полигон грани в карте проекции не является простым невырожденным
    # многоугольником: у него нет триангуляции отсечением ушей, то есть его
    # собственные рёбра пересекаются либо площадь нулевая. Это ровно та
    # предпосылка теоремы о вложении, которой сертификату не хватало: знак
    # ПЛОЩАДИ полигона у «бабочки» остаётся положительным.
    NEAR_PLANAR_PROJECTION_FACE_POLYGON_NOT_SIMPLE = (
        "NEAR_PLANAR_PROJECTION_FACE_POLYGON_NOT_SIMPLE"
    )
    # Интерьеры двух треугольников проекции пересеклись по положительной
    # площади. Прямая власть: инъективность доказана перебором, а не выведена
    # из предпосылок теоремы.
    NEAR_PLANAR_PROJECTION_INTERIOR_OVERLAP = (
        "NEAR_PLANAR_PROJECTION_INTERIOR_OVERLAP"
    )
