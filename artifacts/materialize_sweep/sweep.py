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
расхождении неценовых полей.

`--topology QUAD_STRIPS_V1` — закон топологии декали (по умолчанию `TRIANGLES_V1`,
как у ядра). Закон пишется в заголовок записи, а НЕ в `ANSWER_KEYS`, и числа
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
)
#: Счётчики, которые считают ГРАНИ и потому зависят от закона топологии: между
#: законами они не сравниваются (число треугольников как сумма `n - 2` — сравнивается).
LAW_DEPENDENT_COUNTERS = (
    "MATERIALIZE_TRIANGLES_FLIPPED_VS_SOURCE",
    "MATERIALIZE_TRIANGLES_UV_DEGENERATE",
    "MATERIALIZE_TRIANGLES_UV_REVERSED",
)
ANSWER_KEYS = (
    "prepare_outcome",
    "coverage_outcome",
    "materialization",
    "detail",
    "content_digest",
    "offset_normals_digest",
    "semantic_digest",
    "counters",
    "diagnostics",
    "chart",
    "planarity",
)


def compute_row(patch_id: int, density, topology: str = "TRIANGLES_V1"):
    ctx = pool_sweep._CTX
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
        near_planar_lift_law=NearPlanarLiftLawV1(HOST_NEAR_PLANAR_LIFT_POLICY.value),
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
    row["work_spent"] = counters.get("EXACT_WORK_SPENT", 0)
    row["diagnostics"] = list(result.diagnostics)
    row["stage_seconds"] = {name: round(value, 4) for name, value in result.timings}
    row["leaked_unbudgeted"] = canon.UNBUDGETED_WORK.spent
    row["seconds"] = round(time.perf_counter() - started, 3)
    return row


def _task(args):
    return compute_row(*args)


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
            key: sum(row["topology_counters"].get(key, 0) for row in done)
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
            rows = [compute_row(pid, density, args.topology) for pid in order]
        else:
            with ProcessPoolExecutor(
                max_workers=args.workers, initializer=gate.init_worker
            ) as pool:
                rows = list(
                    pool.map(_task, [(pid, density, args.topology) for pid in order])
                )
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


def _answer_view(row: dict, across_topology: bool) -> dict:
    """Поля строки, которые обязаны совпасть; между законами — без закон-зависимого."""

    view = {key: row.get(key) for key in ANSWER_KEYS}
    if across_topology:
        view.pop("content_digest")
        view["counters"] = {
            key: value
            for key, value in (row.get("counters") or {}).items()
            if key not in LAW_DEPENDENT_COUNTERS
        }
    return view


def compare(paths, across_topology: bool = False) -> int:
    records = [json.loads(Path(item).read_text(encoding="utf-8")) for item in paths]
    problems = []
    base = records[0]
    for other, path in zip(records[1:], paths[1:]):
        for density in sorted(set(base["runs"]) & set(other["runs"])):
            left = base["runs"][density]["domains"]
            right = other["runs"][density]["domains"]
            if set(left) != set(right):
                problems.append(f"{path} d{density}: domain sets differ")
            for patch in sorted(set(left) & set(right), key=int):
                first = _answer_view(left[patch], across_topology)
                second = _answer_view(right[patch], across_topology)
                for key in first:
                    if first[key] != second[key]:
                        problems.append(f"{path} d{density} patch{patch}: {key} differs")
    for line in problems[:40]:
        print(line)
    print("IDENTICAL" if not problems else f"DIFFERENT ({len(problems)})")
    return 1 if problems else 0


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    runner = sub.add_parser("run")
    runner.add_argument("--workers", type=int, default=8)
    runner.add_argument("--densities", default="1,2")
    runner.add_argument("--only", default="")
    runner.add_argument("--out", required=True)
    runner.add_argument(
        "--topology", choices=("TRIANGLES_V1", "QUAD_STRIPS_V1"), default="TRIANGLES_V1"
    )
    comparer = sub.add_parser("compare")
    comparer.add_argument("paths", nargs="+")
    comparer.add_argument("--across-topology", action="store_true")
    args = parser.parse_args()
    if args.command == "compare":
        return compare(args.paths, args.across_topology)
    record = run(args)
    Path(args.out).write_text(
        json.dumps(record, ensure_ascii=False, sort_keys=True, indent=1),
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
