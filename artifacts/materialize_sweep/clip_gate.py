"""Ворота закона `SOURCE_TRIANGLES_CLIPPED_V1` на ВСЕХ доменах `building`: плоские — побитово те же, кривые — отличие ровно резкой.

Каждый домен считается маршрутом кнопки до покрытия (как `sweep.py`) и материализуется ДВАЖДЫ на одних и тех
же `prepared` и покрытии: законом укладки `SOURCE_TRIANGLES_V1` (до резки) и `SOURCE_TRIANGLES_CLIPPED_V1`.
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

    python clip_gate.py run --workers 8 --densities 1,2,4 --topology PLANAR_POLYGONS_V1 --out gate.json

Код возврата 1 при любом расхождении. Список кривых доменов с числами (грани, четырёхгранья, треугольники,
вершины `clip:`, свес, секунды резки) печатается всегда.
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

SCHEMA = "clip_gate_v1"
ALPHA_TEXT = pool_sweep.ALPHA_TEXT
ALPHA_VALUE = pool_sweep.ALPHA_VALUE
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
    if not math.isclose(_uv_area(base.batch), _uv_area(cut.batch), rel_tol=1e-9, abs_tol=1e-12):
        problems.append("the covered UV area differs")
    added = {item.outcome.value for item in cut.batch.diagnostics} - {
        item.outcome.value for item in base.batch.diagnostics
    }
    # Диагностика отказа сдвига вершины по ориентации куска появляется ровно тогда, когда счётчик вырос.
    if added - ({REFUSED_DIAGNOSTIC} if refused > 0 else set()) != {CLIP_NEW}:
        problems.append(f"diagnostics added: {sorted(added)}")
    return problems


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


def compute_pair(patch_id: int, density, topology: str) -> dict:
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
    for law in (NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1, NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1):
        canon.reset_factorization_memory()
        results[law.value] = materialize_domain(
            prepared,
            coverage,
            request=request,
            near_planar_lift_law=law,
            decal_topology_law=DecalTopologyLawV1(topology),
        )
    base, cut = results["SOURCE_TRIANGLES_V1"], results["SOURCE_TRIANGLES_CLIPPED_V1"]
    row.update(
        planar=planar,
        status=cut.outcome.value,
        problems=judge(base, cut, planar),
        base=_numbers(base),
        clipped=_numbers(cut),
        chart=frame.chart_orientation.value,
    )
    return row


def _task(args):
    return compute_pair(*args)


def run(args) -> dict:
    from concurrent.futures import ProcessPoolExecutor

    record = {"schema": SCHEMA, "topology": args.topology, "alpha": ALPHA_TEXT, "runs": {}}
    for density in (int(item) for item in args.densities.split(",")):
        order = [int(x) for x in args.only.split(",")] if args.only else gate._default_order()
        started = time.perf_counter()
        with ProcessPoolExecutor(max_workers=args.workers, initializer=gate.init_worker) as pool:
            rows = list(pool.map(_task, [(pid, density, args.topology) for pid in order]))
        rows.sort(key=lambda row: row["patch_id"])
        record["runs"][str(density)] = {"wall": round(time.perf_counter() - started, 1), "domains": rows}
        done = [row for row in rows if row.get("status") == "MATERIALIZED"]
        planar = [row for row in done if row["planar"]]
        curved = [row for row in done if not row["planar"]]
        failed = [row for row in rows if row.get("problems")]
        print(
            f"[clip_gate] d{density}: {len(rows)} domains, {len(done)} materialized, {len(planar)} planar identical-checked, "
            f"{len(curved)} curved, failures {len(failed)}",
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
        for row in failed:
            print(f"    FAIL patch {row['patch_id']}: {row['problems']}", flush=True)
    return record


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("run",))
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--densities", default="1,2,4")
    parser.add_argument("--only", default="")
    parser.add_argument("--topology", default="PLANAR_POLYGONS_V1")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    record = run(args)
    Path(args.out).write_text(json.dumps(record, ensure_ascii=False, sort_keys=True, indent=1), encoding="utf-8")
    bad = sum(
        1 for run_ in record["runs"].values() for row in run_["domains"] if row.get("problems")
    )
    print("VERDICT", "CLEAN" if not bad else f"FAILED ({bad})")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
