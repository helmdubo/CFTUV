"""Свип материализатора по ВСЕМ доменам `building` вне Blender.

Каждый домен считается тем же маршрутом, что и кнопка (`build_envelope_analysis_
snapshot` -> `build_envelope_decal_request` -> `run_queue_domain`, alpha 0.45), и
затем — `materialize_domain` на готовых `prepared` + покрытии, под UV-законом
`UV_DIRECT_STRIP_V1`. Запрос в материализатор уходит как
`materialization_request(prepared, uv_policy_id=...)`: это СКОМПИЛИРОВАННЫЙ
запрос подготовки с заменой одного закона UV, а не заново собранный запрос
со старым id (аудит 2026-10-02: так батч нёс ключ исполнения, чей хэш не
совпадал с содержимым). Чужой запрос `admit` теперь отказывает именованно.

Что пишется на каждый домен: исход подготовки и покрытия, исход
материализации (имя и деталь), дайджесты (семантический и содержательный),
счётчики (грани, треугольники, вершины, веера, слитое, аудит сетки), секунды
по стадиям. Всё, что не секунды, обязано совпасть между прогонами:

    python sweep.py run --workers 8 --densities 1,2 --out run_a.json
    python sweep.py run --workers 8 --densities 1,2 --out run_b.json
    python sweep.py run --workers 0 --densities 1,2 --out run_seq.json
    python sweep.py compare run_a.json run_b.json run_seq.json

`--workers 0` — тот же цикл в ЭТОМ процессе (последовательно, один воркер).
`--only 6,11` — подмножество доменов. Код возврата `compare` — 1 при любом
расхождении неценовых полей, не разрешённом спецификацией.

Срез, который меняет ответ осознанно, сравнивается по СПЕЦИФИКАЦИИ закона — данным в
`specs/<имя>.json`, а не флагами с зашитым списком: `compare base.json new.json --spec chord_station`.
Что в спецификации (какие домены, какие поля, дайджесты, счётчики и диагностики вправе измениться, какие
инварианты обязаны держаться), как её писать и три вида вердикта (`IDENTICAL`, `EXPECTED-CHANGE (spec X):
N domains`, `UNEXPECTED ...`) — в `expected_change.py`. `compare --list-specs` перечисляет сохранённые.
Закон, которого у среза нет флагом (`--chord-station off`, `--source-lift off`, `--near-planar-law
SOURCE_TRIANGLES_V1` — ДО резки), даёт запись «до» на том же дереве; иначе «до» берётся из прогона основы.

Счётчики, которых нет в `COUNTER_KEYS` и `TOPOLOGY_COUNTER_KEYS` (числа резки `MATERIALIZE_CLIP_*`, станции
перекладин и прочее, что ядро добавило после списков), пишутся в `untracked_counters` строки и сравниваются
наравне с остальными: новый счётчик ядра не пропадает из ворот молча. Отсутствующий счётчик равен нулю везде.

`--topology QUAD_STRIPS_V1|PLANAR_POLYGONS_V1` — закон топологии декали (по
умолчанию `TRIANGLES_V1`, как у ядра). Закон пишется в заголовок записи, а НЕ в `ANSWER_KEYS`, и числа
закона лежат в отдельных полях строки (`topology_counters`), не в `counters`:
иначе запись под законом по умолчанию разошлась бы с прежними. Два закона
сравниваются командой `compare --across-topology`: дайджесты содержания у них
различны ПО ПОСТРОЕНИЮ, всё остальное (семантический дайджест, дайджест
нормалей, число треугольников как сумма `n - 2`, вершины, цепи, счётчики
подъёма) обязано совпасть.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
GATE = ROOT / "artifacts" / "numeric_repr"
if str(GATE) not in sys.path:
    sys.path.insert(0, str(GATE))

if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import expected_change  # noqa: E402  (спецификация ожидаемого изменения: общая с `gate.py`)
import gate  # noqa: E402  (тянет pool_sweep и пути харнесса)
import pool_sweep  # noqa: E402

SCHEMA = "materialize_sweep_v1"
ALPHA_TEXT = pool_sweep.ALPHA_TEXT
ALPHA_VALUE = pool_sweep.ALPHA_VALUE
COUNTER_KEYS = (
    "MATERIALIZE_FACES_IN",
    "MATERIALIZE_FACES_CONTOURED",
    "MATERIALIZE_FACES_EMPTY_AFTER_CLIP",
    "MATERIALIZE_FACES_LOST",
    "MATERIALIZE_FACES_LOST_CONTOUR_MISSING",
    "MATERIALIZE_FACES_LOST_OWNER_MISMATCH",
    "MATERIALIZE_FACES_LOST_SHORT_CONTOUR_WITH_AREA",
    "MATERIALIZE_CONTOURS_WITHOUT_FACE",
    "MATERIALIZE_DOMAIN_REGIONS",
    "MATERIALIZE_VERTEX_SOURCE_NAMES_DROPPED",
    "MATERIALIZE_FACES_MERGED",
    "MATERIALIZE_SEPARATORS_MERGED",
    "MATERIALIZE_MERGE_UNRESOLVED",
    "MATERIALIZE_FAN_FACES",
    "MATERIALIZE_TRIANGLES",
    "MATERIALIZE_VERTICES",
    "MATERIALIZE_REGIONS",
    "MATERIALIZE_STATION_FACTS",
    "MATERIALIZE_STATION_CONSTANT_S",
    "MATERIALIZE_BOUNDARY_CHAINS",
    "MATERIALIZE_INTERFACE_CHAINS",
    "MATERIALIZE_BOUNDARY_EDGES",
    "MATERIALIZE_TRIANGLES_FLIPPED_VS_SOURCE",
    "MATERIALIZE_TRIANGLES_UV_DEGENERATE",
    "MATERIALIZE_TRIANGLES_UV_REVERSED",
    "MATERIALIZE_SURFACE_LIFT_LOCATIONS",
    "MATERIALIZE_SURFACE_LIFT_CANDIDATE_TRIANGLES",
    "MATERIALIZE_SURFACE_LIFT_PREDICATES",
    "MATERIALIZE_SURFACE_LIFT_ON_EDGE_POINTS",
    "MATERIALIZE_SURFACE_LIFT_TRIANGLES",
    "MATERIALIZE_SURFACE_LIFT_DEGENERATE_PROJECTIONS",
    "MATERIALIZE_SURFACE_LIFT_CHART_VERTICES_SNAPPED",
    "MATERIALIZE_SOURCE_VERTICES_LIFTED_AT_HOST",
    "MATERIALIZE_SOURCE_VERTICES_DISPLACED_BY_LATTICE",
    "MATERIALIZE_SOURCE_VERTICES_HOST_POSITION_UNAVAILABLE",
    "MATERIALIZE_SOURCE_VERTICES_LIFT_REFUSED_BY_FACE_ORIENTATION",
    "MATERIALIZE_NODES_FOLLOWED_SOURCE_LIFT",
    # Закон допуска глубины противостояния нормали смещения (`SURFACE_OFFSET_OPPOSITION_DEPTH_V1`).
    "MATERIALIZE_OFFSET_OPPOSITIONS_TOLERATED",
    "MATERIALIZE_OFFSET_OPPOSITION_WORST_DEPTH_NANOMETRES",
    # Закон `SOURCE_VERTEX_STATIONED_ON_CHORD_V1`: исходы каждой внутренней вершины прямой цепи.
    "MATERIALIZE_CHORD_STATIONS_TOTAL",
    "MATERIALIZE_CHORD_STATIONS_PLACED",
    "MATERIALIZE_CHORD_STATIONS_AT_NODE",
    "MATERIALIZE_CHORD_STATIONS_NOT_IN_COVERAGE",
    "MATERIALIZE_CHORD_STATIONS_SKIPPED_NOT_MONOTONE",
    "MATERIALIZE_CHORD_STATIONS_SKIPPED_NODE_NAMES_ANOTHER_VERTEX",
    "MATERIALIZE_CHORD_STATIONS_SKIPPED_SLIDE_BEYOND_HALF_STEP",
    "MATERIALIZE_CHORD_STATIONS_FACES_RESTATIONED",
    "STATION_RUNS",
    "STATION_EDGES",
    "STATION_UNNAMED_CHAINS",
    "STATION_RESTART_CHAINS",
    "STATION_SKIPS",
)
#: Поля строки, которые обязаны совпасть между прогонами (всё остальное — цена).
#: Числа закона топологии. Отдельно от `COUNTER_KEYS`: строка закона по умолчанию
#: не получает новых ключей в `counters`, и прежние записи остаются сравнимыми.
TOPOLOGY_COUNTER_KEYS = (
    "MATERIALIZE_FACES_EMITTED",
    "MATERIALIZE_QUADS",
    "MATERIALIZE_QUADS_REFUSED_NOT_CONVEX",
    "MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES",
    "MATERIALIZE_QUADS_SPLIT_OFFSET_NORMALS_DIFFER",
    "MATERIALIZE_MERGED_RUN_FACES_TRIANGULATED",
    # Числа закона `PLANAR_POLYGONS_V1`: причины названы поимённо вместо одного
    # счётчика «слитые пробеги», который считал любой контур длиннее четырёх.
    "MATERIALIZE_POLYGON_FACES_EMITTED",
    "MATERIALIZE_POLYGON_FACES_CONCAVE_EMITTED",
    "MATERIALIZE_POLYGON_FACES_TRIANGULATED_NOT_SIMPLE",
    "MATERIALIZE_POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE",
    "MATERIALIZE_CURVED_STRIP_FACES_TRIANGULATED",
    "MATERIALIZE_MERGED_RUNS_SPLIT_AT_RUNGS",
    "MATERIALIZE_MERGED_RUNS_KEPT_WHOLE",
    # Числа закона `FAN_FACE_TRIANGULATED_FROM_APEX_V1` (веера под `PLANAR_POLYGONS_V1`).
    "MATERIALIZE_FAN_FACES_CUT_BY_NEIGHBOUR",
    "MATERIALIZE_FAN_POLYGON_FACES_EMITTED",
    "MATERIALIZE_FAN_POLYGON_FACES_CONCAVE_EMITTED",
    "MATERIALIZE_FAN_FACES_TRIANGULATED_FROM_APEX",
    "MATERIALIZE_FAN_FACES_NOT_STAR_FROM_APEX",
    "MATERIALIZE_FACES_MAX_OFF_PLANE_NANOMETRES",
    "MATERIALIZE_FACES_TRIANGULATED_AFTER_SOURCE_LIFT",
    "MATERIALIZE_TRIANGLES_FLIPPED_BY_SOURCE_LIFT",
    # Числа потоков (`CORNER_JOIN_SOFT_BEND_V1`): стыки JOIN, потоки из двух и более
    # вхождений, перекладины со станцией вершины цепи, билинейные четырёхгранья и
    # наибольший излом их UV (`_MAX_` — максимум по доменам, а не сумма). Здесь, а не в
    # `COUNTER_KEYS`: прежние записи остаются сравнимыми на доменах без потоков.
    "STATION_FLOWS",
    "STATION_JOIN_CORNERS",
    "STATION_SKIP_JOIN_CORNER_NOT_ADJACENT",
    "MATERIALIZE_RUNG_STATIONS_FROM_CHAIN_VERTEX",
    "MATERIALIZE_QUADS_UV_BILINEAR",
    "MATERIALIZE_QUADS_UV_BILINEAR_MAX_MILLI_ALPHA",
    "MATERIALIZE_POLYGONS_UV_BILINEAR",
    "MATERIALIZE_POLYGONS_UV_BILINEAR_MAX_MILLI_ALPHA",
    # Кольца потока (замкнутая цепь из одних мягких изломов разомкнута в одном месте), углы JOIN
    # вне домена, названные пропуски стыков и свободные рёбра резки внутри регионов потока.
    "STATION_FLOW_CYCLES_OPENED",
    "STATION_JOIN_CORNERS_OUT_OF_DOMAIN",
    "STATION_SKIP_JOIN_USE_NOT_IN_DOMAIN_LOOPS",
    "STATION_SKIP_JOIN_CYCLE_OF_ONE_USE",
    "MATERIALIZE_CLIP_FLOW_FREE_CUT_EDGES",
)
#: Счётчики, которые считают ГРАНИ и потому зависят от закона топологии: между
#: законами они не сравниваются (число треугольников как сумма `n - 2` — сравнивается).
LAW_DEPENDENT_COUNTERS = (
    "MATERIALIZE_TRIANGLES_FLIPPED_VS_SOURCE",
    "MATERIALIZE_TRIANGLES_UV_DEGENERATE",
    "MATERIALIZE_TRIANGLES_UV_REVERSED",
)
#: Всё, что свип пишет в `counters` и `topology_counters`; остальные счётчики ядра идут в `untracked_counters`.
TRACKED_COUNTER_KEYS = frozenset(COUNTER_KEYS) | frozenset(TOPOLOGY_COUNTER_KEYS)
#: Счётчики цены (бюджет точной работы): не ответ, их сводка — `work_spent`.
PRICE_COUNTER_PREFIXES = ("EXACT_WORK_",)
ANSWER_KEYS = (
    "prepare_outcome",
    "coverage_outcome",
    "materialization",
    "detail",
    "content_digest",
    "offset_normals_digest",
    "semantic_digest",
    "counters",
    "untracked_counters",
    "diagnostics",
    "chart",
    "planarity",
)


def compute_row(patch_id: int, density, *args, **kwargs):
    """Строка домена; под любым символьным бэкендом кроме `SYMPY` несёт его счётчики (вне `ANSWER_KEYS`)."""

    gate._reset_backend_counts()
    row = _compute_row(patch_id, density, *args, **kwargs)
    extra = gate._backend_price()
    if extra:
        row["symbolic_backend_report"] = extra
    return row


def _compute_row(
    patch_id: int,
    density,
    topology: str = "TRIANGLES_V1",
    source_lift: str = "on",
    chord_station: str = "on",
    near_planar_law: str = "",
):
    ctx = pool_sweep._CTX
    if chord_station == "off":
        # Закон `SOURCE_VERTEX_STATIONED_ON_CHORD_V1` выключен: грани остаются на узлах решётки, как до
        # закона (ворота «закон — единственное изменение»: батч и дайджесты побитово прежние, кроме чисел
        # самого закона).
        from cftuv_envelope.materialize import domain as materialize_domain_module
        from cftuv_envelope.materialize.chord_station import ChordStationsV1

        materialize_domain_module.station_chord_vertices = lambda prepared, items, table: (
            items,
            ChordStationsV1(),
        )
    if source_lift == "off":
        # Закон `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` выключен (позиций хоста нет): батч
        # обязан совпасть побитово с батчем до закона (ворота «закон — единственное изменение»).
        from cftuv_envelope.materialize import domain as materialize_domain_module

        materialize_domain_module.host_positions_of = lambda snapshot: {}
    canon = ctx["canon"]
    from cftuv.surface_ir import HOST_NEAR_PLANAR_LIFT_POLICY
    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
    from cftuv_envelope.materialize.admit import materialization_request
    from cftuv_envelope.materialize.domain import materialize_domain
    from cftuv_envelope.wavefront import conveyor_coverage

    domain_id = ctx["typed_value"]("patch-domain", ctx["revision"], patch_id)
    canon.reset_factorization_memory()
    canon.reset_unbudgeted_work()
    row: dict = {"patch_id": patch_id, "density": density}
    started = time.perf_counter()
    try:
        snapshot = ctx["build_snapshot"](
            ctx["bundle"], included_patch_ids=frozenset({patch_id})
        )
        request = ctx["build_request"](
            snapshot,
            frozenset(ctx["by_domain"][domain_id]),
            ALPHA_VALUE,
            decal_request_id_value=ctx["request_id"],
            density=density,
        )
        prepared, domain = ctx["run_queue_domain"](
            patch_id, domain_id, snapshot, request, ALPHA_TEXT
        )
    except ctx["EnvelopeHostAdapterError"] as refusal:
        row.update(prepare_outcome="HOST_ADMISSION_REFUSED", detail=str(refusal)[:300])
        row["seconds"] = round(time.perf_counter() - started, 3)
        return row
    row["prepare_outcome"] = domain.preparation_outcome
    frame = getattr(getattr(prepared, "context", None), "frame", None)
    if frame is not None:
        row["chart"] = frame.chart_orientation.value
        row["planarity"] = type(frame.planarity_certificate).__name__
    row["coverage_outcome"] = domain.coverage_outcome
    row["prepare_seconds"] = round(domain.prepare_seconds + domain.coverage_seconds, 3)
    if domain.preparation_outcome != "EXACT" or domain.coverage_outcome != "EXACT":
        row["materialization"] = "NOT_ATTEMPTED"
        row["detail"] = (domain.detail or "")[:300]
        row["seconds"] = round(time.perf_counter() - started, 3)
        return row
    cover_started = time.perf_counter()
    coverage = conveyor_coverage(prepared, ALPHA_TEXT)
    row["coverage_again_seconds"] = round(time.perf_counter() - cover_started, 3)
    request = materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1")
    work_started = time.perf_counter()
    # Закон укладки — ТОТ ЖЕ, что просит кнопка (`produce_domain`): свип обязан
    # идти продуктовым путём, а не законом по умолчанию ядра.
    result = materialize_domain(
        prepared,
        coverage,
        request=request,
        near_planar_lift_law=NearPlanarLiftLawV1(near_planar_law or HOST_NEAR_PLANAR_LIFT_POLICY.value),
        decal_topology_law=DecalTopologyLawV1(topology),
    )
    row["materialize_seconds"] = round(time.perf_counter() - work_started, 4)
    row["materialization"] = result.outcome.value
    row["detail"] = result.detail[:300]
    row["content_digest"] = result.content_digest
    row["offset_normals_digest"] = result.offset_normals_digest
    row["semantic_digest"] = (
        "" if result.batch is None else result.batch.semantic_digest.value
    )
    counters = dict(result.counters)
    row["counters"] = {key: counters[key] for key in COUNTER_KEYS if key in counters}
    row["topology_counters"] = {
        key: counters[key] for key in TOPOLOGY_COUNTER_KEYS if key in counters
    }
    row["untracked_counters"] = {
        key: value
        for key, value in sorted(counters.items())
        if key not in TRACKED_COUNTER_KEYS and not key.startswith(PRICE_COUNTER_PREFIXES)
    }
    row["work_spent"] = counters.get("EXACT_WORK_SPENT", 0)
    row["diagnostics"] = list(result.diagnostics)
    row["stage_seconds"] = {name: round(value, 4) for name, value in result.timings}
    row["leaked_unbudgeted"] = canon.UNBUDGETED_WORK.spent
    row["seconds"] = round(time.perf_counter() - started, 3)
    return row


def _task(args):
    return compute_row(*args)


def _source_lift_option(args) -> str:
    return getattr(args, "source_lift", "on")


def _chord_station_option(args) -> str:
    return getattr(args, "chord_station", "on")


def _near_planar_law_option(args) -> str:
    return getattr(args, "near_planar_law", "") or ""


def _row_arguments(args, patch_id: int, density: int) -> tuple:
    return (
        patch_id,
        density,
        args.topology,
        _source_lift_option(args),
        _chord_station_option(args),
        _near_planar_law_option(args),
    )


def _git(*args: str) -> str:
    try:
        out = subprocess.run(
            ["git", *args], cwd=ROOT, capture_output=True, text=True, timeout=30
        )
        return out.stdout.strip()
    except Exception:  # noqa: BLE001
        return ""


def summarize(rows) -> dict:
    outcomes: dict[str, int] = {}
    for row in rows:
        key = f"{row['prepare_outcome']}/{row.get('coverage_outcome', '-')}/{row.get('materialization', '-')}"
        outcomes[key] = outcomes.get(key, 0) + 1
    done = [row for row in rows if "materialize_seconds" in row]
    seconds = [row["materialize_seconds"] for row in done]
    prepare = [row["prepare_seconds"] for row in done]
    return {
        "domains": len(rows),
        "outcomes": outcomes,
        "materialized": sum(1 for row in done if row["materialization"] == "MATERIALIZED"),
        "materialize_seconds_sum": round(sum(seconds), 3),
        "materialize_seconds_max": round(max(seconds, default=0.0), 4),
        "materialize_seconds_median": round(statistics.median(seconds), 4) if seconds else 0.0,
        "prepare_seconds_sum": round(sum(prepare), 3),
        "ratio_materialize_to_prepare": (
            round(sum(seconds) / sum(prepare), 4) if sum(prepare) else None
        ),
        "triangles": sum(row["counters"].get("MATERIALIZE_TRIANGLES", 0) for row in done),
        "flipped_vs_source": sum(
            row["counters"].get("MATERIALIZE_TRIANGLES_FLIPPED_VS_SOURCE", 0)
            for row in done
        ),
        "uv_degenerate": sum(
            row["counters"].get("MATERIALIZE_TRIANGLES_UV_DEGENERATE", 0) for row in done
        ),
        "uv_reversed": sum(
            row["counters"].get("MATERIALIZE_TRIANGLES_UV_REVERSED", 0) for row in done
        ),
        "fan_faces": sum(row["counters"].get("MATERIALIZE_FAN_FACES", 0) for row in done),
        "topology": {
            key: (max if "_MAX_" in key else sum)(
                row["topology_counters"].get(key, 0) for row in done
            )
            for key in sorted({k for row in done for k in row["topology_counters"]})
        },
        # Судьба граней покрытия: вход равен контурам + пустым за фронтом +
        # потерям, и потерь на здоровом домене нет.
        "faces_in": sum(row["counters"].get("MATERIALIZE_FACES_IN", 0) for row in done),
        "faces_contoured": sum(
            row["counters"].get("MATERIALIZE_FACES_CONTOURED", 0) for row in done
        ),
        "faces_empty_after_clip": sum(
            row["counters"].get("MATERIALIZE_FACES_EMPTY_AFTER_CLIP", 0) for row in done
        ),
        "faces_lost": sum(row["counters"].get("MATERIALIZE_FACES_LOST", 0) for row in done),
        "vertex_source_names_dropped": sum(
            row["counters"].get("MATERIALIZE_VERTEX_SOURCE_NAMES_DROPPED", 0)
            for row in done
        ),
        "station_skips": sum(row["counters"].get("STATION_SKIPS", 0) for row in done),
        "charts": {
            name: sum(1 for row in done if row.get("chart") == name)
            for name in sorted({row.get("chart") for row in done})
        },
        "work_spent_max": max((row.get("work_spent", 0) for row in done), default=0),
        "leaked_unbudgeted": sum(row.get("leaked_unbudgeted", 0) for row in done),
    }


def run(args) -> dict:
    from concurrent.futures import ProcessPoolExecutor

    densities = [int(item) for item in args.densities.split(",")]
    record = {
        "schema": SCHEMA,
        "sha": _git("rev-parse", "--short", "HEAD"),
        "tree_diff_hash": _git("diff", "--stat", "--", "cftuv", "kernel")[:80],
        "alpha": ALPHA_TEXT,
        "topology": args.topology,
        "source_lift": _source_lift_option(args),
        "chord_station": _chord_station_option(args),
        "near_planar_law": _near_planar_law_option(args) or "product",
        "workers": args.workers,
        "python": sys.version.split()[0],
        "cores": os.cpu_count(),
        "runs": {},
    }
    for density in densities:
        order = [int(x) for x in args.only.split(",")] if args.only else gate._default_order()
        started = time.perf_counter()
        if args.workers == 0:
            gate.init_worker(quiet=True)
            rows = [compute_row(*_row_arguments(args, pid, density)) for pid in order]
        else:
            with ProcessPoolExecutor(
                max_workers=args.workers, initializer=gate.init_worker
            ) as pool:
                rows = list(pool.map(_task, [_row_arguments(args, pid, density) for pid in order]))
        wall = time.perf_counter() - started
        rows.sort(key=lambda row: row["patch_id"])
        summary = summarize(rows)
        summary["wall_seconds"] = round(wall, 3)
        record["runs"][str(density)] = {
            "summary": summary,
            "domains": {str(row["patch_id"]): row for row in rows},
        }
        print(f"[sweep] d{density} wall={wall:.1f}s {json.dumps(summary, ensure_ascii=False)}", flush=True)
    return record


#: Дайджесты строки (`allow.digests` спецификации) и прочие скалярные поля ответа (`allow.fields`).
DIGEST_FIELDS = ("content_digest", "offset_normals_digest", "semantic_digest")
ANSWER_FIELDS = ("prepare_outcome", "coverage_outcome", "materialization", "detail", "chart", "planarity")
VOCABULARY = expected_change.Vocabulary(
    digests=frozenset(DIGEST_FIELDS),
    fields=frozenset(ANSWER_FIELDS),
    counters=TRACKED_COUNTER_KEYS,
)
UNTRACKED_NOT_COMPARED = (
    "untracked_counters (unlisted kernel counters, e.g. MATERIALIZE_CLIP_*) are not in a compared record; "
    "they are not compared on those rows - re-run the old record with the current sweep.py"
)


def _row_view(row: dict, across_topology: bool, with_untracked: bool, notes=()) -> expected_change.RowView:
    """Ответ строки: поля, ВСЕ счётчики одним словарём (отсутствующий равен нулю), диагностики; цена не входит.

    Между законами топологии не сравниваются дайджест содержания, числа закона и закон-зависимые счётчики.
    """

    fields = {key: row.get(key) for key in ANSWER_FIELDS + DIGEST_FIELDS}
    skipped = LAW_DEPENDENT_COUNTERS if across_topology else ()
    counters = {key: value for key, value in (row.get("counters") or {}).items() if key not in skipped}
    if across_topology:
        fields.pop("content_digest")
    else:
        counters.update(row.get("topology_counters") or {})
    if with_untracked:
        counters.update(row.get("untracked_counters") or {})
    return expected_change.RowView(
        fields=fields,
        counters=counters,
        diagnostics=tuple(row.get("diagnostics") or ()),
        ok=row.get("materialization") == "MATERIALIZED",
        notes=notes,
    )


def pair_views(across_topology: bool = False):
    """`(base_row, new_row) -> (RowView, RowView)`: нераспознанные счётчики сравниваются, когда записаны в ОБЕИХ строках."""

    def views(base_row: dict, new_row: dict):
        both = "untracked_counters" in base_row and "untracked_counters" in new_row
        answered = "counters" in base_row or "counters" in new_row
        notes = (UNTRACKED_NOT_COMPARED,) if answered and not both else ()
        return (
            _row_view(base_row, across_topology, both, notes),
            _row_view(new_row, across_topology, both),
        )

    return views


def compare(paths, across_topology: bool = False, spec=None, partial: bool = False) -> int:
    """Записи сравниваются с первой: ответ побитово тот же, кроме разрешённого спецификацией (`expected_change.py`).

    Без `spec` любое расхождение неценовых полей — `UNEXPECTED`. Со `spec` изменяются ТОЛЬКО объявленные домены и
    ТОЛЬКО разрешённое им; объявленный домен, который не изменился, и лишний изменившийся домен — проблемы.
    `partial` — прогон части доменов: объявленные домены вне записей не ошибка.
    """

    records = [json.loads(Path(item).read_text(encoding="utf-8")) for item in paths]
    report = expected_change.evaluate(
        records,
        [str(item) for item in paths],
        spec,
        pair_views(across_topology),
        VOCABULARY,
        partial,
    )
    expected_change.print_report(report)
    return report.exit_code


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    runner = sub.add_parser("run")
    runner.add_argument("--workers", type=int, default=8)
    runner.add_argument("--densities", default="1,2")
    runner.add_argument("--only", default="")
    runner.add_argument("--out", required=True)
    runner.add_argument(
        "--topology",
        choices=("TRIANGLES_V1", "QUAD_STRIPS_V1", "PLANAR_POLYGONS_V1"),
        default="TRIANGLES_V1",
    )
    runner.add_argument(
        "--source-lift",
        dest="source_lift",
        choices=("on", "off"),
        default="on",
        help="off: закон SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1 выключен (ворота равенства до закона)",
    )
    runner.add_argument(
        "--chord-station",
        dest="chord_station",
        choices=("on", "off"),
        default="on",
        help="off: закон SOURCE_VERTEX_STATIONED_ON_CHORD_V1 выключен (грани на узлах решётки, как до закона)",
    )
    runner.add_argument(
        "--near-planar-law",
        dest="near_planar_law",
        choices=("SOURCE_TRIANGLES_V1", "SOURCE_TRIANGLES_CLIPPED_V1", "SOURCE_FACES_CLIPPED_V1"),
        default="",
        help="закон подъёма near-planar доменов (по умолчанию продуктовый); SOURCE_TRIANGLES_V1 — запись ДО резки",
    )
    comparer = sub.add_parser("compare")
    comparer.add_argument("paths", nargs="*")
    comparer.add_argument("--across-topology", action="store_true")
    comparer.add_argument(
        "--spec",
        default=None,
        help="спецификация ожидаемого изменения: имя из specs/ либо путь к .json (без неё любое расхождение — UNEXPECTED)",
    )
    comparer.add_argument(
        "--partial",
        action="store_true",
        help="прогон части доменов (--only): объявленные спецификацией домены вне записей — примечание, не ошибка",
    )
    comparer.add_argument("--list-specs", action="store_true", help="перечислить сохранённые спецификации")
    args = parser.parse_args()
    if args.command == "compare":
        if args.list_specs:
            expected_change.print_stored_specs("sweep")
            return 0
        if len(args.paths) < 2:
            parser.error("compare needs at least two records (the first is the base)")
        spec = None
        if args.spec is not None:
            try:
                spec = expected_change.load_spec(args.spec, "sweep", VOCABULARY)
            except expected_change.SpecError as error:
                parser.error(str(error))
        return compare(args.paths, args.across_topology, spec, args.partial)
    record = run(args)
    Path(args.out).write_text(
        json.dumps(record, ensure_ascii=False, sort_keys=True, indent=1),
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
