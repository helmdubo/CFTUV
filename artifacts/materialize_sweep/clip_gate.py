"""Ворота законов резки (`SOURCE_FACES_CLIPPED_V1` по умолчанию, `SOURCE_TRIANGLES_CLIPPED_V1`) на ВСЕХ доменах `building`: плоские — побитово те же, кривые — отличие ровно резкой.

Каждый домен считается маршрутом кнопки до покрытия (как `sweep.py`) и материализуется ДВАЖДЫ на одних и тех
же `prepared` и покрытии: законом укладки `SOURCE_TRIANGLES_V1` (до резки) и законом резки `--law`.
Сравнение — по ответам, а не по обещаниям:

* плоский домен (точная плоскость): `content_digest`, семантический дайджест и дайджест нормалей РАВНЫ
  побитово — резка его не касается;
* кривой домен (near-planar на поверхности, развёртка): все вершины до резки на месте и в тех же позициях
  (исключение — вершина `src:`, которую сдвиг к позиции хоста не сделан из-за ориентации куска: число таких
  вершин равно приросту счётчика `..._LIFT_REFUSED_BY_FACE_ORIENTATION`), новые вершины — только `clip:`,
  ШОВНЫЕ цепи (источник и стена — граница домена вдоль контура патча, шов с соседним доменом) побитово те же
  БЕЗ вычёркивания вершин и без единой `clip:` (иначе шов молча открывается: хост сваривает только
  `location:src:`), остальные цепи (фронт, интерфейсы) те же без вершин `clip:`, регионы те же, факты
  `(s, r)` вершин до резки те же, суммарная UV-площадь граней та же, диагностик прибавилось одна
  (`SOURCE_EDGES_LIFTED_ONTO_SURFACE`; плюс отказ сдвига вершины по ориентации куска, если счётчик отказов вырос).
  Счёт `QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES` под резкой — тавтология (имена подъёма обнулены), поэтому он
  печатается, но вердикта не решает: доказательство «кусок в одном треугольнике» — `clip._prove`.
  Закон по граням (`SOURCE_FACES_CLIPPED_V1`) прибавляет только вершины `clip:` на настоящих рёбрах и счётчики
  `MATERIALIZE_CLIP_DIAGONAL_*`; шовные цепи и там побитово те же и без единой вершины `clip:`.

    python clip_gate.py run --workers 8 --densities 1,2,4 --topology PLANAR_POLYGONS_V1 --out gate.json
    python clip_gate.py run --law SOURCE_TRIANGLES_CLIPPED_V1 --out gate_triangles.json

Закон `SILHOUETTE_TOPOLOGY_V1` (продуктовый) — это `PLANAR_POLYGONS_V1` плюс пост-проход растворения. Ворота судят РЕЗКУ, а пост-проход
растворяет две сетки независимо (резаная и нерезаная теряют разные вершины и рёбра), поэтому для этого закона обе материализации берутся
ДО растворения: ворота запускаются на стадии `PLANAR_POLYGONS_V1` (`STAGE_BEFORE_DISSOLVE`), стадия печатается и пишется в запись
(`stage_topology`), а не молча подменяется. Сам пост-проход судят `kernel/tests/test_silhouette_topology.py` и спецификация `silhouette_topology`.

Код возврата 1 при любом расхождении. Список кривых доменов с числами (грани, четырёхгранья, треугольники,
вершины `clip:`, свес, секунды резки) печатается всегда.

Что вправе измениться и у каких доменов, судит та же спецификация, что у `sweep.py compare` и `gate.py compare`
(`specs/clip_by_faces.json` для закона по граням, `specs/clip_by_triangles.json` для закона по треугольникам;
`--spec` выбирает другую): плоские домены (не названные в спецификации) совпадают во ВСЁМ — все поля, все счётчики
ядра, диагностики, — кривые меняются только разрешённым, и каждый кривой домен обязан измениться. Геометрические
инварианты кривых (вершины на месте, шовные цепи те же, площадь UV та же, ...) остаются доказательством самой
резки (`judge`) и входят в тот же вердикт строкой GEOMETRY. Последняя строка — один из трёх вердиктов:
`IDENTICAL`, `EXPECTED-CHANGE (spec X): N domains`, `UNEXPECTED (spec X): K problems; first: ...`.
"""

from __future__ import annotations

import argparse
import json
import math
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

if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import expected_change  # noqa: E402  (спецификация ожидаемого изменения: общая с `sweep.py` и `gate.py`)
import sweep  # noqa: E402  (вид строки и словарь имён сравнения: те же слова на всех воротах)

SCHEMA = "clip_gate_v1"
ALPHA_TEXT = pool_sweep.ALPHA_TEXT
ALPHA_VALUE = pool_sweep.ALPHA_VALUE
#: Закон, чья стадия до растворения — другой закон: ворота резки судят резку, а не независимое растворение двух сеток.
STAGE_BEFORE_DISSOLVE = {"SILHOUETTE_TOPOLOGY_V1": "PLANAR_POLYGONS_V1"}
REFUSED = "MATERIALIZE_SOURCE_VERTICES_LIFT_REFUSED_BY_FACE_ORIENTATION"
SPLIT = "MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES"
CLIP_NEW = "SOURCE_EDGES_LIFTED_ONTO_SURFACE"
REFUSED_DIAGNOSTIC = "SOURCE_VERTEX_LIFT_REFUSED_BY_FACE_ORIENTATION"


def _uv_area(batch) -> float:
    total = 0.0
    for face in batch.faces:
        uv = [(fact.uv.u, fact.uv.v) for fact in face.uv_facts]
        total += abs(
            sum(
                uv[i][0] * uv[(i + 1) % len(uv)][1] - uv[(i + 1) % len(uv)][0] * uv[i][1]
                for i in range(len(uv))
            )
        ) / 2.0
    return total


SEAM_KINDS = ("SOURCE", "WALL")
#: Счётчики неаффинной UV потока: домен с любым из них сверяет UV-площадь с допуском, а не точно (см. `judge`).
NON_AFFINE_COUNTERS = (
    "MATERIALIZE_QUADS_UV_BILINEAR",
    "MATERIALIZE_POLYGONS_UV_BILINEAR",
    "MATERIALIZE_POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE",
)
#: Относительный допуск UV-площади домена с неаффинными гранями: измерено 1.5e-8 при изломе UV 0.001 alpha (building п106).
FLOW_UV_AREA_REL_TOL = 1e-4


def _chain_id(item) -> str:
    return (getattr(item, "semantic_boundary_id", None) or item.semantic_interface_id).value


def _is_seam(item) -> bool:
    """Граничная цепь вдоль контура патча (`boundary:SOURCE:...`, `boundary:WALL:...`): шов с соседним доменом."""

    parts = _chain_id(item).split(":")
    return parts[0] == "boundary" and parts[1] in SEAM_KINDS


def _seam_chains(chains) -> dict:
    """Шовные цепи как есть: ключи вершин по порядку, без вычёркивания."""

    return {_chain_id(item): tuple(key.value for key in item.ordered_vert_keys) for item in chains if _is_seam(item)}


def _stripped(chains):
    """Цепи НЕ шва без вершин `clip:`: фронт и интерфейсы вправе нести вершины резки."""

    return {
        _chain_id(item): tuple(key.value for key in item.ordered_vert_keys if not key.value.startswith("clip:"))
        for item in chains
        if not _is_seam(item)
    }


def _facts(batch, keys):
    return {
        (fact.semantic_region_id.value, fact.vert_key.value): (fact.source_s, fact.source_r)
        for fact in batch.station_facts
        if fact.vert_key.value in keys
    }


def judge(base, cut, planar: bool) -> list[str]:
    """Расхождения двух материализаций одного домена; пусто — ворота пройдены."""

    if base.outcome != cut.outcome:
        return [f"outcome {base.outcome.value} -> {cut.outcome.value}: {cut.detail[:120]}"]
    if base.batch is None:
        return []
    if planar:
        return [
            name
            for name, left, right in (
                ("content_digest", base.content_digest, cut.content_digest),
                ("semantic_digest", base.batch.semantic_digest, cut.batch.semantic_digest),
                ("offset_normals_digest", base.offset_normals_digest, cut.offset_normals_digest),
            )
            if left != right
        ]
    problems = []
    before = {item.vert_key.value: item for item in base.batch.vertices}
    after = {item.vert_key.value: item for item in cut.batch.vertices}
    if set(before) - set(after):
        problems.append(f"vertices lost: {sorted(set(before) - set(after))[:3]}")
    extra = set(after) - set(before)
    if any(not key.startswith("clip:") for key in extra):
        problems.append("a vertex that is not clip: was added")
    moved = [
        key for key in before if key in after and before[key].position != after[key].position
    ]
    refused = dict(cut.counters).get(REFUSED, 0) - dict(base.counters).get(REFUSED, 0)
    if any(not key.startswith("src:") for key in moved) or len(moved) > max(refused, 0):
        problems.append(f"{len(moved)} vertices moved, {refused} extra orientation refusals")
    seam_before, seam_after = _seam_chains(base.batch.boundary_chains), _seam_chains(cut.batch.boundary_chains)
    if seam_before != seam_after:
        problems.append(
            "seam (SOURCE/WALL) chains differ: "
            + ", ".join(sorted(name for name in set(seam_before) | set(seam_after) if seam_before.get(name) != seam_after.get(name))[:4])
        )
    if any(key.startswith("clip:") for keys in seam_after.values() for key in keys):
        problems.append("a clip: vertex lies on a seam chain")
    if _stripped(base.batch.boundary_chains) != _stripped(cut.batch.boundary_chains):
        problems.append("boundary chains differ beyond clip vertices")
    if _stripped(base.batch.interface_chains) != _stripped(cut.batch.interface_chains):
        problems.append("interface chains differ beyond clip vertices")
    if base.batch.semantic_regions != cut.batch.semantic_regions:
        problems.append("semantic regions differ")
    if _facts(base.batch, set(before)) != _facts(cut.batch, set(before)):
        problems.append("(s, r) facts of the old vertices differ")
    # Равенство суммарной UV-площади держится у АФФИННОЙ UV. Поток (`CORNER_JOIN_SAME_PCHAIN_V1`, `CORNER_JOIN_SOFT_BEND_V1`)
    # несёт неаффинную UV (билинейные грани, `UV_NOT_AFFINE`): целая грань и её куски после резки отличаются на излом
    # UV, поэтому у домена с неаффинными гранями площадь сверяется с допуском `FLOW_UV_AREA_REL_TOL`, а не точно.
    nonaffine = any(
        dict(side.counters).get(name, 0)
        for side in (base, cut)
        for name in NON_AFFINE_COUNTERS
    )
    area_tolerance = FLOW_UV_AREA_REL_TOL if nonaffine else 1e-9
    if not math.isclose(_uv_area(base.batch), _uv_area(cut.batch), rel_tol=area_tolerance, abs_tol=1e-12):
        problems.append("the covered UV area differs")
    added = {item.outcome.value for item in cut.batch.diagnostics} - {
        item.outcome.value for item in base.batch.diagnostics
    }
    # Диагностика отказа сдвига вершины по ориентации куска появляется ровно тогда, когда счётчик вырос.
    if added - ({REFUSED_DIAGNOSTIC} if refused > 0 else set()) != {CLIP_NEW}:
        problems.append(f"diagnostics added: {sorted(added)}")
    return problems


#: Спецификация по умолчанию для закона резки (`--law`).
DEFAULT_SPECS = {
    "SOURCE_FACES_CLIPPED_V1": "clip_by_faces",
    "SOURCE_TRIANGLES_CLIPPED_V1": "clip_by_triangles",
}


def _answer_row(result, planarity: str) -> dict:
    """Ответ материализации в виде строки свипа: её читает то же сравнение (`sweep.pair_views`), что и записи свипа."""

    return {
        "prepare_outcome": "EXACT",
        "coverage_outcome": "EXACT",
        "materialization": result.outcome.value,
        "detail": result.detail[:300],
        "content_digest": result.content_digest,
        "offset_normals_digest": result.offset_normals_digest,
        "semantic_digest": "" if result.batch is None else result.batch.semantic_digest.value,
        "planarity": planarity,
        "counters": {
            key: value
            for key, value in sorted(dict(result.counters).items())
            if not key.startswith(sweep.PRICE_COUNTER_PREFIXES)
        },
        "untracked_counters": {},
        "diagnostics": list(result.diagnostics),
    }


def _numbers(result) -> dict:
    counters = dict(result.counters)
    return {
        "faces": counters.get("MATERIALIZE_FACES_EMITTED", 0),
        "quads": counters.get("MATERIALIZE_QUADS", 0),
        "triangles": counters.get("MATERIALIZE_TRIANGLES", 0),
        "clip_vertices": counters.get("MATERIALIZE_CLIP_VERTICES_INSERTED", 0),
        "overhang": counters.get("MATERIALIZE_CLIP_FACES_OVERHANG_TRIANGULATED", 0),
        "seam_suppressed": counters.get("MATERIALIZE_CLIP_FACES_SEAM_CROSSINGS_SUPPRESSED", 0),
        "off_corner": counters.get("MATERIALIZE_CLIP_FACES_SOURCE_VERTEX_OFF_CORNER_SUPPRESSED", 0),
        "refused": counters.get(REFUSED, 0),
        "split": counters.get(SPLIT, 0),
        "work": counters.get("EXACT_WORK_SPENT", 0),
        "seconds": round(dict(result.timings).get("CLIP", 0.0), 3),
    }


def compute_pair(patch_id: int, density, topology: str, law_name: str = "SOURCE_FACES_CLIPPED_V1") -> dict:
    ctx = pool_sweep._CTX
    canon = ctx["canon"]
    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
    from cftuv_envelope.materialize.admit import materialization_request
    from cftuv_envelope.materialize.domain import materialize_domain
    from cftuv_envelope.wavefront import conveyor_coverage

    domain_id = ctx["typed_value"]("patch-domain", ctx["revision"], patch_id)
    canon.reset_factorization_memory()
    canon.reset_unbudgeted_work()
    row: dict = {"patch_id": patch_id, "density": density}
    try:
        snapshot = ctx["build_snapshot"](ctx["bundle"], included_patch_ids=frozenset({patch_id}))
        request = ctx["build_request"](
            snapshot,
            frozenset(ctx["by_domain"][domain_id]),
            ALPHA_VALUE,
            decal_request_id_value=ctx["request_id"],
            density=density,
        )
        prepared, domain = ctx["run_queue_domain"](patch_id, domain_id, snapshot, request, ALPHA_TEXT)
    except ctx["EnvelopeHostAdapterError"] as refusal:
        row.update(status="NOT_ATTEMPTED", detail=str(refusal)[:160], problems=[])
        return row
    if domain.preparation_outcome != "EXACT" or domain.coverage_outcome != "EXACT":
        row.update(status="NOT_ATTEMPTED", detail=(domain.detail or "")[:160], problems=[])
        return row
    frame = prepared.context.frame
    planar = type(frame.planarity_certificate).__name__ == "ExactSourcePlaneCertificateV1"
    coverage = conveyor_coverage(prepared, ALPHA_TEXT)
    request = materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1")
    results = {}
    for law in (NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1, NearPlanarLiftLawV1(law_name)):
        canon.reset_factorization_memory()
        results[law.value] = materialize_domain(
            prepared,
            coverage,
            request=request,
            near_planar_lift_law=law,
            decal_topology_law=DecalTopologyLawV1(STAGE_BEFORE_DISSOLVE.get(topology, topology)),
        )
    base, cut = results["SOURCE_TRIANGLES_V1"], results[law_name]
    row.update(
        planar=planar,
        status=cut.outcome.value,
        problems=judge(base, cut, planar),
        base=_numbers(base),
        clipped=_numbers(cut),
        chart=frame.chart_orientation.value,
        answers={
            "base": _answer_row(base, type(frame.planarity_certificate).__name__),
            "clipped": _answer_row(cut, type(frame.planarity_certificate).__name__),
        },
    )
    return row


def _task(args):
    return compute_pair(*args)


def run(args, spec) -> dict:
    from concurrent.futures import ProcessPoolExecutor

    record = {
        "schema": SCHEMA,
        "topology": args.topology,
        "stage_topology": STAGE_BEFORE_DISSOLVE.get(args.topology, args.topology),
        "law": args.law,
        "spec": spec.name,
        "alpha": ALPHA_TEXT,
        "runs": {},
    }
    answers = {"base": {"runs": {}}, "clipped": {"runs": {}}}
    geometry = []
    if record["stage_topology"] != args.topology:
        print(
            f"[clip_gate] topology {args.topology}: both materializations are taken at the stage before the dissolve pass "
            f"({record['stage_topology']}); the dissolve pass is judged by its own tests",
            flush=True,
        )
    for density in (int(item) for item in args.densities.split(",")):
        order = [int(x) for x in args.only.split(",")] if args.only else gate._default_order()
        started = time.perf_counter()
        with ProcessPoolExecutor(max_workers=args.workers, initializer=gate.init_worker) as pool:
            rows = list(pool.map(_task, [(pid, density, args.topology, args.law) for pid in order]))
        rows.sort(key=lambda row: row["patch_id"])
        record["runs"][str(density)] = {"wall": round(time.perf_counter() - started, 1), "domains": rows}
        for side in answers:
            answers[side]["runs"][str(density)] = {
                "domains": {str(row["patch_id"]): row["answers"][side] for row in rows if "answers" in row}
            }
        done = [row for row in rows if row.get("status") == "MATERIALIZED"]
        planar = [row for row in done if row["planar"]]
        curved = [row for row in done if not row["planar"]]
        failed = [row for row in rows if row.get("problems")]
        geometry += [(density, row) for row in failed]
        print(
            f"[clip_gate] d{density}: {len(rows)} domains, {len(done)} materialized, {len(planar)} planar identical-checked, "
            f"{len(curved)} curved, geometry failures {len(failed)}",
            flush=True,
        )
        for row in curved:
            b, c = row["base"], row["clipped"]
            print(
                f"    patch {row['patch_id']:3d}: faces {b['faces']}->{c['faces']} quads {b['quads']}->{c['quads']} "
                f"tris {b['triangles']}->{c['triangles']} split {b['split']}->{c['split']} "
                f"clip vertices {c['clip_vertices']} overhang {c['overhang']} seam-suppressed {c['seam_suppressed']} "
                f"off-corner {c['off_corner']} src-move refusals {b['refused']}->{c['refused']} "
                f"work {b['work']}->{c['work']} "
                f"clip s {c['seconds']}",
                flush=True,
            )
    report = expected_change.evaluate(
        [answers["base"], answers["clipped"]],
        ["SOURCE_TRIANGLES_V1", args.law],
        spec,
        sweep.pair_views(False),
        sweep.VOCABULARY,
    )
    for density, row in geometry:
        report.problems.append(f"d{density} patch{row['patch_id']}: GEOMETRY {row['problems']}")
    record["verdict"] = expected_change.verdict_line(report)
    record["problems"] = len(report.problems)
    expected_change.print_report(report)
    return record


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("run",))
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--densities", default="1,2,4")
    parser.add_argument("--only", default="")
    parser.add_argument("--topology", default="PLANAR_POLYGONS_V1")
    parser.add_argument("--law", default="SOURCE_FACES_CLIPPED_V1", choices=("SOURCE_FACES_CLIPPED_V1", "SOURCE_TRIANGLES_CLIPPED_V1"))
    parser.add_argument("--out", required=True)
    parser.add_argument(
        "--spec",
        default=None,
        help="спецификация ожидаемого изменения (имя из specs/ либо путь); по умолчанию по закону: clip_by_faces / clip_by_triangles",
    )
    args = parser.parse_args()
    try:
        spec = expected_change.load_spec(args.spec or DEFAULT_SPECS[args.law], "sweep", sweep.VOCABULARY)
    except expected_change.SpecError as error:
        parser.error(str(error))
    record = run(args, spec)
    Path(args.out).write_text(json.dumps(record, ensure_ascii=False, sort_keys=True, indent=1), encoding="utf-8")
    return 1 if record["problems"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
