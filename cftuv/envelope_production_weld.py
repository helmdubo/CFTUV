"""Сварка вершин соседних доменов продуктового меша: по семантической ссылке, митра смещения.

АДАПТЕР ОТОБРАЖАЕТ КОНТРАКТ (AGENTS.md): позиции, UV, владельцы и цепи приходят из батчей, а
здесь решаются ровно две вещи политики отображения, обе — над готовыми числами ядра, и ни одна не
чинит геометрию.

СВАРКА. Батч называет вершину исходника ссылкой `semantic_location_ref = location:src:<id>`: она
ГЛОБАЛЬНА (одна на вершину во всех доменах), а `location:node:<k>` — доменная и не сваривается
никогда. Вершины разных доменов с одной ссылкой становятся ОДНОЙ вершиной меша — но только если их
позиции в батчах ПОБИТОВО равны (закон ядра `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` кладёт
вершину в одну и ту же позицию хоста). Это ПРОВЕРКА, а не ремонт: позиции не сходятся — вершины
остаются раздельными, ссылка считается в `ADAPTER_WELD_POSITION_MISMATCH` (число групп и наибольшее
расхождение), и ничто не подтягивается друг к другу. По расстоянию вершины не сливаются нигде.

МИТРА. Смещение декали над поверхностью (z-fighting) — политика хоста. У вершины одного домена оно
идёт вдоль её нормали; у общей вершины k доменов — в точку ПЕРЕСЕЧЕНИЯ k сдвинутых плоскостей, чтобы
каждая сваренная грань осталась на сдвинутой плоскости СВОЕГО домена (плоская грань остаётся
плоской): `n_i . o = d` для всех i. k = 1 — обычное смещение `d n`; k = 2 — `d (n1 + n2) /
(1 + n1 . n2)`; k = 3 — единственное решение; больше — решение, если система совместна. Вершины
домена-развёртки приносят СВОИ нормали смещения (закон ядра `SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1`).
Равные нормали (компланарные соседи) смещаются ровно как одиночная вершина: побитово прежний ответ.

КОНЕЦ СТЕНЫ (закон `WALL_MITER_OFFSET_V1`). Цепь-стена (`boundary:WALL:*`) — граница домена вдоль контура патча там, где выделенной цепи нет: с другой
стороны того же ребра источника стоит стена соседнего домена. Её вершины `src:` общие и свариваются (митра выше); конец стены — вершина `node:` (место, где
фронт доходит до ребра): она ДОМЕННАЯ, у соседа свой узел на глубине, измеренной метрикой ЕГО патча (`rounded_wall.001`: плоский торец и изогнутая грань
различаются на 0.3 % глубины, 0.7 мм при ширине 0.22 и 3.1 мм при 1.0), позиции не равны, по ссылке и по расстоянию она не сваривается. Узел смещался по нормали
одного домена, а вершина `src:` у начала той же стены — митрой: два ребра стены выходили из одной митровой точки и расходились клином `d |n1 - n2|` до конца
стены (7 мм при d = 5 мм, 28 мм при d = 20 мм на сгибе 90 градусов; от глубины и ширины не зависит) — щель вдоль невыделенного шва. Геометрия батча ни при чём (до
смещения обе стены лежат на одном ребре); клин родился вместе со сваркой (DECAL-WELD), позднего релиза за ним нет.
Закон: узел стены, у которой на том же ребре стоит стена другого домена, смещается в точку пересечения сдвинутых плоскостей ОБОИХ доменов (`n_i . o = d`), то есть
на линию сгиба, на которой уже лежит митровая вершина начала стены. Нормаль соседа в точке узла — линейная по длине его стены между его нормалью в общей вершине
и нормалью за ней (за концом его стены — нормаль конца), нормированная; каждая грань остаётся на сдвинутой плоскости СВОЕГО домена. Сосед — домен, у которого стена
выходит из той же вершины `src:` (одна ссылка, побитово равная позиция) вдоль того же ребра (синус угла не больше `WALL_DIRECTION_SINE`, направления сонаправлены).
Узлы с равными позициями (симметричные глубины; побитово либо в пределах `WALL_NODE_WELD_TOLERANCE`, закон `WALL_NODE_NEAR_WELD_V1`) свариваются в одну вершину; разные — каждый остаётся своей вершиной на той же линии, а более короткий
узел делит ребро более длинной стены (закон `WALL_EDGE_SPLIT_V1`, `cftuv_envelope.mesh_glue`): ступень остаётся, общая часть стены — общее ребро двух граней. Геометрию батча закон не трогает, это политика смещения. Правило 4 (эвристика, меняющая ответ, пишется): `ADAPTER_WALL_NODES_LIFTED`
(узлы на линии сгиба), `_WELDED` (вершины меша из пар узлов), `_ALONE` (узел без соседа на стене: край меша, сосед вне выделения; смещение прежнее),
`ADAPTER_WALL_MITER_FALLBACK` (митра отказала пределом `MITER_LIMIT` либо несовместностью: узел со своим смещением, причина и худший множитель названы).

ОТКАЗ ОТ СВАРКИ ИМЕНОВАН. Нормали почти противоположны (митра улетает: длина `d / cos(угла / 2)`
выше предела), вырождены (три плоскости без общей точки), либо система несовместна — вершины
остаются раздельными, каждая со смещением по своей нормали, и это `ADAPTER_WELD_MITER_FALLBACK`
(число групп, причина и худший множитель митры). Предел митры и допуск вырожденности — решения
хоста, объявленные ниже и записанные в исходе; они меняют ответ (сваривать или нет), поэтому ни
одно срабатывание не молчит.

ШВЫ UV. UV остаётся по петле: у вершины, сваренной из двух доменов, UV граней разные. Ребро, у
которого две грани РАЗНЫХ доменов и UV хоть на одном конце различаются, — разрыв UV на складке, и
оно помечается швом (`ADAPTER_WELD_SEAMS_MARKED`): раньше эту роль играла открытая граница.

ПЛОСКОСТЬ ПОСЛЕ СМЕЩЕНИЯ. Грань от четырёх вершин, плоская в батче (закон ядра
`SOURCE_TRIANGLES_CLIPPED_V1`: кусок лежит в одном треугольнике источника), смещается вдоль нормалей
СВОИХ вершин, и у домена развёртки они разные: смещённая грань перестаёт быть плоской. Хост этого не
чинит и не режет молча: отклонение вершин от плоскости их грани ПОСЛЕ смещения измеряется
(`off_plane_after_offset`) и пишется в квитанцию (`ADAPTER_MAX_OFF_PLANE_AFTER_OFFSET_NANOMETRES`).
Порога у числа нет — решает владелец глазами (прецедент: `DEVELOPABLE_OFFSET_MIN_GAP_COSINE` ядра).
Закон по граням (`SOURCE_FACES_CLIPPED_V1`, политика кнопки) к этому числу добавляет глубину хорды: кусок
поперёк диагонали непланарной грани источника уже в батче не плоский, на величину до допуска ядра
(четверть смещения, 5 мм; `CLIP_DIAGONAL_CHORD_BUDGET`), и это отклонение входит в то же число.

ШОВ БЕЗ T-СТЫКОВ. Соседние домены делят цепь источника, и сваривает их только `location:src:`: вершины
между двумя `src:` одной цепи (`node:`, `clip:`) в разные домены не сливаются и T-стыков хост не считает
(`half_edge_conflicts` видит лишь повторные полурёбра). Поэтому шов проверяется ПО ЦЕПЯМ БАТЧЕЙ, точно:
у каждой пары соседних `src:` на цепи `boundary:SOURCE` берётся число вершин между ними в каждом домене, и
пара, у которой оно в двух доменах различно, — T-стык (`seam_report`; запись, а не ремонт: вершины не
подтягиваются). Туда же — пара соседних `src:` одного домена, между которыми у другого домена на цепи стоит ещё `src:`, каких у первого домена нет, и ни на одной цепи они не соседи: так
выглядит точка, растворённая лишь с одной стороны (план станций цепей `CHAIN_STATION_PLAN_V1` решает вершину один раз по цепи, и оба
домена общей цепи читают одно решение; эта проверка его страхует там, где домен оставил вершину по названной структурной причине). Вершина `clip:` на цепи источника или стены — свой счёт: закон ядра их там не допускает.

ПОРЯДОК. Вершина меша получает номер первого вхождения при обходе доменов по номеру патча и вершин
по ключу, поэтому нумерация не зависит ни от воркера, ни от порядка множеств батча. Цепи батча — тоже `frozenset`: `seam_report`
идёт по ним в порядке имени цепи, а сверка пар — множества над всеми цепями, и квитанция не зависит от `PYTHONHASHSEED`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from cftuv_envelope import mesh_weld
from cftuv_envelope.mesh_weld import WALL_NODE_WELD_TOLERANCE, WELD_LOCATION_PREFIX, WallPlanV1, weld_core
from cftuv_envelope.mesh_checks import _anchors_between, _run_between, seam_report_numbers  # noqa: F401 - the two private ones are read by the weld tests

OUTCOME_WELD_POSITION_MISMATCH = "ADAPTER_WELD_POSITION_MISMATCH"
OUTCOME_WELD_MITER_FALLBACK = "ADAPTER_WELD_MITER_FALLBACK"
OUTCOME_WELD_HALF_EDGE_CONFLICT = "ADAPTER_WELD_HALF_EDGE_CONFLICT"
COUNTER_WELD_GROUPS = "ADAPTER_WELD_GROUPS"
COUNTER_WELD_VERTICES_MERGED = "ADAPTER_WELD_VERTICES_MERGED"
COUNTER_WELD_SEAMS_MARKED = "ADAPTER_WELD_SEAMS_MARKED"
#: Шов по цепям батчей (см. `seam_report`): пары соседних `src:` цепи источника с РАЗНЫМ числом вершин между
#: ними в двух доменах (T-стык шва) и вершины `clip:` на цепях источника и стены.
COUNTER_SEAM_T_JUNCTIONS = "ADAPTER_SEAM_T_JUNCTIONS"
COUNTER_SEAM_CLIP_VERTICES = "ADAPTER_SEAM_CLIP_VERTICES"
OUTCOME_SEAM_T_JUNCTIONS = "ADAPTER_SEAM_T_JUNCTIONS"

#: Предел митры: длина смещения общей вершины не больше `MITER_LIMIT` смещений одиночной (для двух
#: доменов это `1 / cos(угла между нормалями / 2) <= 4`, угол до ~151 градуса). Острее — вершины
#: остаются раздельными (`ADAPTER_WELD_MITER_FALLBACK`): митра ножевого ребра улетела бы за декаль.
MITER_LIMIT = 4.0
#: Допуск вырожденности системы митры: ведущий элемент Гаусса не больше `MITER_PIVOT_EPSILON` —
#: нормали линейно зависимы; несовместность и невязка — не больше `MITER_RESIDUAL_EPSILON`.
MITER_PIVOT_EPSILON = 1e-9
MITER_RESIDUAL_EPSILON = 1e-9
#: Закон `WALL_MITER_OFFSET_V1` (конец стены на линии сгиба; описание — в докстринге модуля). Синус наибольшего угла между направлениями стен двух
#: доменов из общей вершины `src:`, при котором это одно ребро источника: позиции батча несут шум подъёма (до одной ячейки решётки, ~1 мм у изогнутого
#: патча: узел лежит на ребре с отклонением ~4e-4 на длине 0.22, это 2e-3), а разные рёбра источника в одной вершине различаются на градусы.
WALL_DIRECTION_SINE = 1e-2
#: Закон сварки, который хост передаёт ядру (`cftuv_envelope.mesh_weld`) и порту: допуски — решения адаптера, не ядра.
WELD_LAW = (MITER_LIMIT, MITER_PIVOT_EPSILON, MITER_RESIDUAL_EPSILON, WALL_DIRECTION_SINE, WALL_NODE_WELD_TOLERANCE, 1.0)


@dataclass(frozen=True, slots=True)
class DomainVerticesV1:
    """Вершины одного домена в порядке ключей: позиция батча, нормаль смещения, ссылка (либо `None`).

    `walls` — цепи-стены домена (`boundary:WALL:*`): по цепи кортеж номеров вершин в порядке цепи (пусто — стен нет, закон конца стены не действует).
    """

    patch_id: int
    positions: tuple
    normals: tuple
    refs: tuple
    walls: tuple = ()


@dataclass(frozen=True, slots=True)
class WeldV1:
    """Итог сварки: позиции вершин меша со смещением, номера вершин каждого домена и числа."""

    positions: tuple
    #: По домену: номер вершины меша для каждой его локальной вершины.
    index: tuple
    counters: tuple
    #: `(None, исход, деталь)`: находки про весь меш (в предупреждения квитанции).
    warnings: tuple
    #: `((домен, домен соседа), ...)` закона конца стены (`WallPlanV1.partners`): чьи нормали входят в смещение узлов этого домена.
    partners: tuple = ()
    #: Закон `WALL_EDGE_SPLIT_V1` (`WallPlanV1.splits`): деления рёбер длинных стен более короткими узлами.
    splits: tuple = ()


OUTCOME_WALL_MITER_FALLBACK = "ADAPTER_WALL_MITER_FALLBACK"
COUNTER_WALL_NODES_LIFTED = "ADAPTER_WALL_NODES_LIFTED"
COUNTER_WALL_NODES_WELDED = "ADAPTER_WALL_NODES_WELDED"
COUNTER_WALL_NODES_ALONE = "ADAPTER_WALL_NODES_ALONE"
COUNTER_WALL_NODES_WELDED_NEAR = "ADAPTER_WALL_NODES_WELDED_NEAR"
COUNTER_WALL_EDGES_SPLIT = "ADAPTER_WALL_EDGES_SPLIT"
COUNTER_WALL_SPLITS_UNPLACED = "ADAPTER_WALL_SPLITS_UNPLACED"


def miter_offset(normals):
    """`(вектор смещения при d = 1, множитель митры, причина)` под законом хоста (`mesh_weld.miter_offset`)."""

    return mesh_weld.miter_offset(normals, WELD_LAW)


def plan_wall_nodes(domains) -> WallPlanV1:
    """Закон `WALL_MITER_OFFSET_V1` под законом хоста (`mesh_weld.plan_wall_nodes`)."""

    return mesh_weld.plan_wall_nodes(domains, WELD_LAW)


def weld_vertices(domains, offset: float) -> WeldV1:
    """Вершины меша из вершин доменов: сварка по ссылке при равных позициях, митра смещения (`cftuv_envelope.mesh_weld.weld_core`, нативно при заказе)."""

    from .envelope_mesh_assembly import native_answer  # not at the top: the seam report of this module also loads standalone (the hash-seed probe)

    found = native_answer("mesh_weld", domains, offset, WELD_LAW)
    return weld_of_answer(weld_core(domains, offset, WELD_LAW) if found is None else found)


def weld_of_answer(answer) -> WeldV1:
    """`WeldV1` из кортежа `weld_core` (эталона, порта `mesh_weld` или чисел `mesh_assemble`, у которого позиции и номера — буферы): тексты находок пишет хост."""

    (
        positions, index, groups, merged, mismatches, gaps, fallbacks, factor, reasons,
        lifted, welded, alone, wall_fallbacks, wall_factor, wall_reasons, partners, near, splits,
    ) = answer
    gap = 0.0
    for points in gaps:
        gap = max(gap, max(math.dist(a, b) for a in points for b in points))
    warnings = []
    if mismatches:
        warnings.append(
            (
                None,
                OUTCOME_WELD_POSITION_MISMATCH,
                f"{mismatches} shared source vertices have positions that are not bitwise "
                f"equal across domains and stay separate (largest gap {gap:.6g} m)",
            )
        )
    if fallbacks:
        warnings.append(
            (
                None,
                OUTCOME_WELD_MITER_FALLBACK,
                f"{fallbacks} shared source vertices stay separate with their own offsets: "
                f"{'; '.join(reasons)} (worst miter factor {factor:.6g}, "
                f"limit {MITER_LIMIT:g})",
            )
        )
    if wall_fallbacks:
        warnings.append(
            (
                None,
                OUTCOME_WALL_MITER_FALLBACK,
                f"{wall_fallbacks} wall end nodes keep their own offsets: {'; '.join(wall_reasons)} "
                f"(worst miter factor {wall_factor:.6g}, limit {MITER_LIMIT:g})",
            )
        )
    return WeldV1(
        positions=positions,
        index=index,
        counters=(
            (COUNTER_WELD_GROUPS, groups),
            (COUNTER_WELD_VERTICES_MERGED, merged),
            (OUTCOME_WELD_POSITION_MISMATCH, mismatches),
            (OUTCOME_WELD_MITER_FALLBACK, fallbacks),
            (COUNTER_WALL_NODES_LIFTED, lifted),
            (COUNTER_WALL_NODES_WELDED, welded),
            (COUNTER_WALL_NODES_ALONE, alone),
            (COUNTER_WALL_NODES_WELDED_NEAR, near),
            (OUTCOME_WALL_MITER_FALLBACK, wall_fallbacks),
        ),
        warnings=tuple(warnings),
        partners=partners,
        splits=splits,
    )


def boundary_chain_names(batch) -> tuple:
    """`((имя цепи, (ключ вершины, ...)), ...)` граничных цепей батча по имени цепи (то, что читает `seam_report`).

    Цепи батча — `frozenset` (порядок ходит с `PYTHONHASHSEED`), поэтому идут по имени цепи. Батч без `boundary_chains` даёт `()`.
    """

    return tuple(
        (chain.semantic_boundary_id.value, tuple(item.value for item in chain.ordered_vert_keys))
        for chain in sorted(getattr(batch, "boundary_chains", ()) or (), key=lambda item: item.semantic_boundary_id.value)
    )


def seam_report(batches) -> tuple:
    """`seam_report_of_chains` по цепям батчей (см. `boundary_chain_names`)."""

    return seam_report_of_chains([boundary_chain_names(batch) for batch in batches])


def seam_report_of_chains(domains) -> tuple:
    """`((имя, число), ...)`: T-стыки шва между доменами и вершины `clip:` на шовных цепях, по цепям доменов (`cftuv_envelope.mesh_checks.seam_report_numbers`).

    `domains` — по домену `boundary_chain_names(батч)`: ровно то, что читает отчёт, поэтому его дают и вид домена, посчитанный
    воркером, и батч. Шовные цепи — `boundary:SOURCE:*` и `boundary:WALL:*` (граница домена вдоль контура патча). Домен без
    цепей ничего не даёт. Точно, без допусков: ключи вершин, а не координаты. Цепи идут в порядке имени: ответ от порядка не зависит.
    """

    junctions, clip_vertices = seam_report_numbers(domains)
    return (
        (COUNTER_SEAM_T_JUNCTIONS, junctions),
        (COUNTER_SEAM_CLIP_VERTICES, clip_vertices),
    )


def weld_console_lines(counters) -> list:
    """Строки консоли о сварке и о законе конца стены по числам квитанции (`weld_counters`): пусто, когда сварки не было.

    Строка стены печатается, когда закон сработал или отказал: узлы без соседа (`_ALONE`) — край меша и вне выделения, сами по себе не событие.
    """

    found = dict(counters or ())
    lines = []
    if found.get(COUNTER_WELD_GROUPS):
        lines.append(
            f"[CFTUV][Production] WELD: {found[COUNTER_WELD_GROUPS]} shared vertices "
            f"({found[COUNTER_WELD_VERTICES_MERGED]} domain vertices merged), "
            f"{found[COUNTER_WELD_SEAMS_MARKED]} fold seams, "
            f"position mismatches {found[OUTCOME_WELD_POSITION_MISMATCH]}, "
            f"miter fallbacks {found[OUTCOME_WELD_MITER_FALLBACK]}"
        )
    if found.get(COUNTER_WALL_NODES_LIFTED) or found.get(OUTCOME_WALL_MITER_FALLBACK):
        lines.append(
            f"[CFTUV][Production] WALL: {found[COUNTER_WALL_NODES_LIFTED]} wall end nodes lifted onto the fold line "
            f"({found[COUNTER_WALL_NODES_WELDED]} welded), {found[COUNTER_WALL_NODES_ALONE]} without a neighbour wall, "
            f"miter fallbacks {found[OUTCOME_WALL_MITER_FALLBACK]}"
        )
    return lines


__all__ = (
    "COUNTER_WALL_NODES_ALONE",
    "COUNTER_WALL_EDGES_SPLIT",
    "COUNTER_WALL_NODES_LIFTED",
    "COUNTER_WALL_NODES_WELDED_NEAR",
    "COUNTER_WALL_SPLITS_UNPLACED",
    "COUNTER_WALL_NODES_WELDED",
    "COUNTER_WELD_GROUPS",
    "COUNTER_WELD_SEAMS_MARKED",
    "COUNTER_WELD_VERTICES_MERGED",
    "DomainVerticesV1",
    "MITER_LIMIT",
    "OUTCOME_WALL_MITER_FALLBACK",
    "OUTCOME_WELD_HALF_EDGE_CONFLICT",
    "OUTCOME_WELD_MITER_FALLBACK",
    "OUTCOME_WELD_POSITION_MISMATCH",
    "WALL_DIRECTION_SINE",
    "WALL_NODE_WELD_TOLERANCE",
    "WELD_LOCATION_PREFIX",
    "WallPlanV1",
    "WeldV1",
    "miter_offset",
    "plan_wall_nodes",
    "weld_console_lines",
    "weld_of_answer",
    "weld_vertices",
)
