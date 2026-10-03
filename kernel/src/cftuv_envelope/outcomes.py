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
    # Ступень кривизны DEVELOPABLE (S1): лестница метрики EXACT -> NEAR_PLANAR ->
    # DEVELOPABLE пробует развёртку только после ИМЕНОВАННОГО отказа near-planar по
    # ширине, перевороту либо вложению. Все исходы развёртки названы, а числа,
    # из которых сложилось решение, лежат в тексте отказа.
    # Смежность треугольников владельца не выводится по парам сторон: сторона
    # держит больше двух носителей, соседи проходят общую пару концов в одну
    # сторону либо веер вершины не складывается в одну цепь треугольников.
    DEVELOPABLE_ADJACENCY_UNAVAILABLE = "DEVELOPABLE_ADJACENCY_UNAVAILABLE"
    # Носитель домена — не диск (эйлерова характеристика не 1 либо он несвязен):
    # дерево смежности не покрывает его однозначной картой.
    DEVELOPABLE_SUPPORT_NOT_A_DISK = "DEVELOPABLE_SUPPORT_NOT_A_DISK"
    # Носитель — кольцо (характеристика 0, две граничные петли): без разреза
    # карта не определена. Разрез и голономия — следующий срез, не этот.
    PERIODIC_CUT_REQUIRED = "PERIODIC_CUT_REQUIRED"
    # Треугольник владельца вырожден после привязки: шарнир через него не задан.
    DEVELOPABLE_SOURCE_TRIANGLE_DEGENERATE = "DEVELOPABLE_SOURCE_TRIANGLE_DEGENERATE"
    # Развёртка измеряется в шагах решётки источника, а закон решётки привязку
    # источника не выполнял: масштаба нет, карта не определена.
    DEVELOPABLE_REQUIRES_SOURCE_SNAP = "DEVELOPABLE_REQUIRES_SOURCE_SNAP"
    # Растяжение треугольника источника в карту вышло за бюджет: домен не
    # развёртываем в пределах объявленного допуска (кривизна вершины либо шум
    # привязки; отказ несёт худший треугольник и худшую вершину).
    DEVELOPABLE_STRETCH_BUDGET_EXCEEDED = "DEVELOPABLE_STRETCH_BUDGET_EXCEEDED"
    # Треугольник карты после привязки к решётке имеет обратный обход.
    DEVELOPABLE_CHART_TRIANGLE_FLIPPED = "DEVELOPABLE_CHART_TRIANGLE_FLIPPED"
    # Граница карты не простая: непримыкающие рёбра пересеклись или коснулись.
    # Погружение диска с непростой границей не вложение (спираль накрывает себя).
    DEVELOPABLE_CHART_SELF_OVERLAP = "DEVELOPABLE_CHART_SELF_OVERLAP"
    # Развёртка до привязки в бюджете, а ни на одной ступени решётки карты после
    # привязки она не уложилась: ячейка грубее тонких треугольников.
    DEVELOPABLE_CHART_LATTICE_TOO_COARSE = "DEVELOPABLE_CHART_LATTICE_TOO_COARSE"
    # Карта со свободными цепями годна, а объявленная ПРЯМОЙ цепь прямой в ней быть не
    # может: внутренняя геометрия патча у её вершин не плоская (сумма углов веера на
    # стороне патча не `π`), и выпрямление стоит растяжения за бюджетом, перевёрнутого
    # треугольника либо самонакрытия. Отказ несёт цепь, худшую вершину и оболочку её
    # боковой суммы углов против `π`. Имя ступени МЕТРИКИ: объявление цепи прямой —
    # факт хоста, а `SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR` очереди остаётся за
    # ошибкой самого объявления.
    DEVELOPABLE_DECLARED_STRAIGHT_CHAIN_BENT = (
        "DEVELOPABLE_DECLARED_STRAIGHT_CHAIN_BENT"
    )
    # Ступень кривизны NEAR_PLANAR V2: ширина декали на поверхности источника
    # отличается от ширины на карте больше объявленного относительного
    # бюджета. Ширина — внутреннее свойство поверхности: проекция на плоскость
    # карты укорачивает наклонный треугольник в `cos` его наклона, и отказ
    # несёт числа (`min_cos_squared`, порог, худший треугольник).
    NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED = (
        "NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED"
    )
    # Треугольник источника владельца вырожден после привязки к решётке
    # (нормаль нулевая): наклон не определён, и сертификат искажения не может
    # его измерить. Закрытый отказ, а не пропуск треугольника.
    NEAR_PLANAR_OWNER_TRIANGLE_DEGENERATE = "NEAR_PLANAR_OWNER_TRIANGLE_DEGENERATE"
    NEAR_PLANAR_SOURCE_TRIANGLE_FOLDED = "NEAR_PLANAR_SOURCE_TRIANGLE_FOLDED"
    # Сертификату искажения нечего мерить: у владельца нет треугольников
    # поверхности, либо треугольник называет вершину вне патча. Это дефект
    # входа хоста, а не геометрия.
    NEAR_PLANAR_OWNER_SURFACE_TRIANGLES_UNAVAILABLE = (
        "NEAR_PLANAR_OWNER_SURFACE_TRIANGLES_UNAVAILABLE"
    )
    # Приведённый целочисленный базис плоскости измеряется в шагах решётки
    # источника, а закон решётки привязку источника не выполнял: масштаба нет, и
    # базис не определён. Закон репера не подменяется молча другим.
    NEAR_PLANAR_REDUCED_FRAME_REQUIRES_SOURCE_SNAP = (
        "NEAR_PLANAR_REDUCED_FRAME_REQUIRES_SOURCE_SNAP"
    )
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

    # Материализатор (`materialize/`): исходы, которые идут В БАТЧ диагностикой,
    # а не отказом домена. Меш построен, но читатель обязан видеть, что он такое.
    #
    # Домен near-planar: меш лежит на СЕРТИФИЦИРОВАННОЙ плоскости (спроецированной
    # точно, в рациональных числах), а не на исходных вершинах; расстояние между
    # ними — невязка сертификата. Смещение над поверхностью — политика хоста.
    NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE = "NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE"
    # Домен near-planar уложен на ТРЕУГОЛЬНИКИ ИСТОЧНИКА: каждая вершина меша —
    # барицентрический образ точки карты в проекции своего треугольника, расстояние
    # до поверхности нуль по построению. Диагностика несёт числа сертификата
    # искажения ширины и записанную (не судившую) невязку плоскости.
    NEAR_PLANAR_LIFT_ONTO_SOURCE_TRIANGLES = "NEAR_PLANAR_LIFT_ONTO_SOURCE_TRIANGLES"
    # Домен уложен на РАЗВЁРНУТЫЕ треугольники источника (ступень DEVELOPABLE):
    # карта — привязанная к решётке развёртка, а не проекция на плоскость.
    # Диагностика несёт числа сертификата растяжения и ярлыки вершин.
    DEVELOPABLE_LIFT_ONTO_UNFOLDED_SOURCE_TRIANGLES = (
        "DEVELOPABLE_LIFT_ONTO_UNFOLDED_SOURCE_TRIANGLES"
    )
    # Наименьший `n_v . n_T` по углам треугольников развёртки: смещение декали по нормали
    # вершины поднимает её над треугольником на `offset * cosine`, и на острой складке внутри
    # патча зазор стремится к нулю. Число записывается, порога у него нет: решает владелец.
    DEVELOPABLE_OFFSET_MIN_GAP_COSINE = "DEVELOPABLE_OFFSET_MIN_GAP_COSINE"
    # Закон `SOURCE_TRIANGLES_CLIPPED_V1` (`materialize/clip`): грани меша порезаны рёбрами треугольников
    # источника, и каждый кусок лежит в одном замкнутом треугольнике. Диагностика несёт числа: сколько
    # вершин `clip:` вставлено, сколько граней разрезано, сколько кусков сложено обратно либо оставлено
    # отдельно, сколько граней вышло за привязанную триангуляцию («свес») и осталось ушами.
    SOURCE_EDGES_LIFTED_ONTO_SURFACE = "SOURCE_EDGES_LIFTED_ONTO_SURFACE"
    # Ребро `ChainUse` лежит вне петли домена (цепь выходит за его границу):
    # накопление станции `s` на этом месте началось заново, и `u` не продолжается
    # через границу домена.
    U_RESTARTS_AT_DOMAIN_BORDER = "U_RESTARTS_AT_DOMAIN_BORDER"
    # Замкнутая цепь из одних мягких изломов (кольцо потока JOIN) разомкнута в одном месте:
    # там `s` начинается заново, а вершина разреза несёт два набора `(s, r)` в двух регионах
    # (`FLOW_CYCLE_OPENED`). Шов один — на этом стыке; остальной поток непрерывен.
    U_RESTARTS_AT_CLOSED_FLOW_OPENING = "U_RESTARTS_AT_CLOSED_FLOW_OPENING"
    # Острая вогнутая вершина оставлена МИТРОВАННОЙ (веер не записался): мягкого
    # угла в этом месте нет, и меш показывает именно митру.
    DEGRADED_MITER_CORNER_IN_GEOMETRY = "DEGRADED_MITER_CORNER_IN_GEOMETRY"
    # Закон положения вершины `src:` (`materialize/source_lift`): вершина лежит в ТОЧНОЙ позиции
    # вершины исходника из снапшота (один binary64 во всех доменах), если подъём узла отстоит от
    # неё не больше бюджета. Диагностика пишется, только когда хоть одна вершина реально сдвинута;
    # числа — в читаемой строке, счёт — в счётчиках материализатора.
    SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1 = "SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1"
    # Подъём узла вершины `src:` отстоит от её позиции больше бюджета (внутренность объявленной
    # прямой цепи, сдвинутая вдоль хорды): вершина остаётся на подъёме, счёт и худшая названы.
    SOURCE_VERTEX_DISPLACED_BY_LATTICE = "SOURCE_VERTEX_DISPLACED_BY_LATTICE"
    # Позиция хоста перевернула бы контур грани: вершины контура остались на узлах.
    SOURCE_VERTEX_LIFT_REFUSED_BY_FACE_ORIENTATION = (
        "SOURCE_VERTEX_LIFT_REFUSED_BY_FACE_ORIENTATION"
    )
    # Закон `SOURCE_VERTEX_STATIONED_ON_CHORD_V1` (`materialize/chord_station`): внутренние вершины
    # объявленных прямых цепей стоят в точной точке хорды (проекция по Граму из привязки), а не на узле
    # решётки. Диагностика пишется, только когда хоть одна вершина реально сдвинута; числа — в строке.
    SOURCE_VERTEX_STATIONED_ON_CHORD_V1 = "SOURCE_VERTEX_STATIONED_ON_CHORD_V1"
    # Цепь осталась на узлах (станции не строго возрастают между концами хорды, сдвиг больше полушага,
    # узел вершины назван другой вершиной) либо у вершины нет грани на узле: ставить нечего. Названы
    # цепь и причина либо число таких вершин; счёт — `MATERIALIZE_CHORD_STATIONS_SKIPPED_*` и
    # `..._NOT_IN_COVERAGE`.
    SOURCE_VERTEX_CHORD_STATION_SKIPPED = "SOURCE_VERTEX_CHORD_STATION_SKIPPED"
