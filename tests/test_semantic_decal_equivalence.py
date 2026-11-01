"""SEMANTIC_DECAL_EQUIVALENCE_V1: the comparator of two decal results under the product contract (NATIVE_PATH_PRODUCT_CONTRACT_V1).

The matrix: identical records are EQUAL; another face / vertex order, another corner rotation, another triangulation inside a region, another spelling of a
number, relabelled owner ids, other diagnostic text are REPRESENTATION_ONLY; a moved silhouette vertex, a flipped owner, a UV shift at a seam, a swapped
diagonal of a non-planar quad, an interior UV shift, an unwelded join, another named outcome are GEOMETRY_ERROR; a budget refusal on one side is RESOURCE_POLICY.
"""

from __future__ import annotations

import copy
import json
import sys
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import blender_tree_ab as ab  # noqa: E402
import semantic_decal_equivalence as sde  # noqa: E402

GEOMETRY, REPRESENTATION, RESOURCE = sde.GEOMETRY_ERROR, sde.REPRESENTATION_ONLY, sde.RESOURCE_POLICY


def _view(points, uvs_by_face, faces, owners, seams=(), refs=None):
    return {
        "positions": [[sde.wire(float(c)) for c in p] for p in points], "refs": refs if refs is not None else [None] * len(points),
        "faces": [list(face) for face in faces], "uvs": [[sde.wire(float(c)) for c in uv] for face_uvs in uvs_by_face for uv in face_uvs],
        "owners": list(owners), "seams": [list(edge) for edge in seams],
    }


#: A 2 x 1 strip of two quads: the left owned by envelope 0, the right by envelope 1, a UV seam between them (the right quad's UV shifted by 10).
POINTS = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (0, 1, 0), (1, 1, 0), (2, 1, 0)]
FACES = [(0, 1, 4, 3), (1, 2, 5, 4)]
UVS = [[(0, 0), (1, 0), (1, 1), (0, 1)], [(11, 0), (12, 0), (12, 1), (11, 1)]]
REFS = [None, "corner:a", None, None, "corner:b", None]


def strip(points=POINTS, faces=FACES, uvs=UVS, owners=(0, 1), seams=((1, 4),), refs=REFS):
    return _view(points, uvs, faces, owners, seams, list(refs))


def record(view, outcome="MATERIALIZED", detail="", mesh=None, patch="7"):
    return {"schema": sde.SCHEMA, "domains": {patch: {"outcome": outcome, "detail": detail, "diagnostics": [], "view": view}}, "mesh": mesh}


def classes(first, second):
    report = sde.compare_press_records(first, second)
    return report.verdict, {kind for kind in report.counts if report.counts[kind]}, report


def test_identical_records_are_equal():
    verdict, found, _ = classes(record(strip()), record(strip()))
    assert verdict == "EQUAL" and not found


def test_reordered_vertices_faces_and_rotated_corners_are_representation_only():
    order = [5, 3, 0, 4, 2, 1]  # new index -> old vertex
    new_of = {old: new for new, old in enumerate(order)}
    points = [POINTS[old] for old in order]
    faces = [tuple(new_of[v] for v in FACES[1][2:] + FACES[1][:2]), tuple(new_of[v] for v in FACES[0][1:] + FACES[0][:1])]
    uvs = [UVS[1][2:] + UVS[1][:2], UVS[0][1:] + UVS[0][:1]]
    refs = [REFS[old] for old in order]
    verdict, found, report = classes(record(strip()), record(strip(points, faces, uvs, (1, 0), ((new_of[4], new_of[1]),), refs)))
    assert verdict == "EQUIVALENT" and found == {REPRESENTATION}, [item.line() for item in report.findings]


def test_another_triangulation_inside_a_region_is_representation_only():
    a = strip(faces=[(0, 1, 4), (0, 4, 3), FACES[1]], uvs=[[(0, 0), (1, 0), (1, 1)], [(0, 0), (1, 1), (0, 1)], UVS[1]], owners=(0, 0, 1))
    b = strip(faces=[(0, 1, 3), (1, 4, 3), FACES[1]], uvs=[[(0, 0), (1, 0), (0, 1)], [(1, 0), (1, 1), (0, 1)], UVS[1]], owners=(0, 0, 1))
    quad = strip()
    for other in (b, quad):
        verdict, found, report = classes(record(a), record(other))
        assert verdict == "EQUIVALENT" and found == {REPRESENTATION}, [item.line() for item in report.findings]
        assert any("triangulated otherwise" in item.what for item in report.findings)


def test_a_collinear_vertex_added_on_a_boundary_with_linear_uv_is_representation_only():
    points = POINTS + [(0.5, 0, 0)]
    faces = [(0, 6, 1, 4, 3), FACES[1]]
    uvs = [[(0, 0), (0.5, 0), (1, 0), (1, 1), (0, 1)], UVS[1]]
    verdict, found, report = classes(record(strip()), record(strip(points, faces, uvs, refs=REFS + [None])))
    assert verdict == "EQUIVALENT", [item.line() for item in report.findings]


def test_int_and_fraction_spelling_of_the_same_numbers_is_representation_only():
    other = strip()
    other["positions"] = [[int(c) for c in p] for p in POINTS]
    other["uvs"] = [[str(Fraction(int(c))) if i else Fraction(int(c)) for i, c in enumerate(uv)] for face in UVS for uv in face]
    other["uvs"] = [[sde.wire(item) if isinstance(item, Fraction) else item for item in uv] for uv in other["uvs"]]
    half = strip(points=[(0, 0, 0.5)] + POINTS[1:])
    half_exact = copy.deepcopy(half)
    half_exact["positions"][0] = [0, 0, "1/2"]
    for first, second in ((strip(), other), (half, half_exact)):
        verdict, found, report = classes(record(first), record(second))
        assert verdict == "EQUIVALENT" and found == {REPRESENTATION}
        assert any("the representation of one exact value" in item.what for item in report.findings)


def test_a_binary64_and_the_decimal_fraction_it_rounds_are_two_values():
    """CONTRACT_GATES_FIX_V1, finding 4e: free is the representation of ONE exact value; float 0.1 is not Fraction(1, 10)."""

    tenth = strip(points=[(0, 0, 0.1)] + POINTS[1:])
    decimal = copy.deepcopy(tenth)
    decimal["positions"][0] = [0, 0, "1/10"]
    assert classes(record(tenth), record(decimal))[0] == GEOMETRY


def test_relabelled_owner_ids_are_representation_only():
    verdict, found, report = classes(record(strip()), record(strip(owners=(5, 3))))
    assert verdict == "EQUIVALENT" and any("relabelled" in item.what for item in report.findings)


def test_a_moved_silhouette_vertex_is_a_geometry_error():
    moved = [(0, -0.125, 0)] + POINTS[1:]
    verdict, found, report = classes(record(strip()), record(strip(points=moved)))
    assert verdict == GEOMETRY and any("outline" in item.what or "open edges" in item.what for item in report.findings)


def test_a_flipped_owner_is_a_geometry_error():
    # one region of two triangles, B gives one of them to another envelope: the region splits
    a = strip(faces=[(0, 1, 4), (0, 4, 3), FACES[1]], uvs=[[(0, 0), (1, 0), (1, 1)], [(0, 0), (1, 1), (0, 1)], UVS[1]], owners=(0, 0, 1))
    b = strip(faces=[(0, 1, 4), (0, 4, 3), FACES[1]], uvs=[[(0, 0), (1, 0), (1, 1)], [(0, 0), (1, 1), (0, 1)], UVS[1]], owners=(0, 2, 1))
    assert classes(record(a), record(b))[0] == GEOMETRY
    # the left quad goes to the right quad's envelope: two owners of A become one
    verdict, _found, report = classes(record(strip()), record(strip(owners=(1, 1))))
    assert verdict == GEOMETRY and any("merged" in item.what for item in report.findings)


def test_a_uv_shift_at_a_seam_is_a_geometry_error():
    shifted = copy.deepcopy(UVS)
    shifted[1] = [(11.25, 0)] + shifted[1][1:]
    verdict, _found, report = classes(record(strip()), record(strip(uvs=shifted)))
    assert verdict == GEOMETRY
    assert any("UV along the boundary" in item.what for item in report.findings)
    assert any("corner:a" in item.what for item in report.findings)


def test_a_swapped_diagonal_of_a_non_planar_quad_is_a_geometry_error():
    bent = [POINTS[0], POINTS[1], POINTS[2], POINTS[3], (1, 1, 1), POINTS[5]]
    a = strip(points=bent, faces=[(0, 1, 4), (0, 4, 3), FACES[1]], uvs=[[(0, 0), (1, 0), (1, 1)], [(0, 0), (1, 1), (0, 1)], UVS[1]], owners=(0, 0, 1))
    b = strip(points=bent, faces=[(0, 1, 3), (1, 4, 3), FACES[1]], uvs=[[(0, 0), (1, 0), (0, 1)], [(1, 0), (1, 1), (0, 1)], UVS[1]], owners=(0, 0, 1))
    verdict, _found, report = classes(record(a), record(b))
    assert verdict == GEOMETRY and any("inside a region" in item.what for item in report.findings)


def test_a_quad_that_is_not_affine_in_uv_may_not_be_triangulated_otherwise():
    # a planar quad with a bilinear (non-affine) UV: its two diagonals lay the texture differently, so they are not equivalent
    uvs = [[(0, 0), (1, 0), (1.5, 1), (0, 1)], UVS[1]]
    a = strip(uvs=uvs)
    b = strip(faces=[(0, 1, 4), (0, 4, 3), FACES[1]], uvs=[[(0, 0), (1, 0), (1.5, 1)], [(0, 0), (1.5, 1), (0, 1)], UVS[1]], owners=(0, 0, 1))
    verdict, _found, report = classes(record(a), record(b))
    # CONTRACT_GATES_FIX_V1, finding 4d: conservative (the gate fails), but it names what it cannot confirm, not a changed geometry
    texts = [item.what for item in report.of(GEOMETRY)]
    assert verdict == GEOMETRY and texts and all("equivalence of this transformation not confirmed" in text and "tessellation diagonal" in text for text in texts), texts
    assert not any("differs" in text or "leaves" in text for text in texts), texts
    rotated = strip(faces=[(1, 4, 3, 0), FACES[1]], uvs=[uvs[0][1:] + uvs[0][:1], UVS[1]])
    assert classes(record(a), record(rotated))[0] == "EQUIVALENT"  # the same polygon from another corner


#: A planar quad that is concave at its second corner, its UV = its XY: a fan from the first corner folds, the diagonal 1-3 does not.
CONCAVE_POINTS = [(0, 0, 0), (2, 1, 0), (4, 0, 0), (0, 4, 0)]


def test_a_concave_face_triangulated_inside_is_not_a_fold():
    """CONTRACT_GATES_FIX_V1, finding 4c: the triangulation of a face is an ear clipping in exact arithmetic, not a fan from one corner."""

    quad = [[(x, y) for x, y, _z in CONCAVE_POINTS]]
    a = _view(CONCAVE_POINTS, quad, [(0, 1, 2, 3)], (0,))
    triangles = [(0, 1, 3), (1, 2, 3)]
    b = _view(CONCAVE_POINTS, [[CONCAVE_POINTS[v][:2] for v in face] for face in triangles], triangles, (0, 0))
    verdict, found, report = classes(record(a), record(b))
    assert verdict == "EQUIVALENT" and found == {REPRESENTATION}, [item.line() for item in report.findings]
    assert any("triangulated otherwise" in item.what for item in report.findings)
    # a real fold stays a fold: the same faces with the reflex corner's UV pushed through the opposite edge
    folded = [[(0, 0), (2, 3), (0, 4)], [(2, 3), (4, 0), (0, 4)]]
    assert sde.interior_difference([tuple((*p, *uv) for p, uv in zip(CONCAVE_POINTS, quad[0]))], [tuple((*CONCAVE_POINTS[v], *uv) for v, uv in zip(face, uvs)) for face, uvs in zip(triangles, folded)]) is not None


def test_an_interior_uv_shift_is_a_geometry_error():
    points = POINTS + [(0.5, 0.5, 0)]
    fan = [(0, 1, 6), (1, 4, 6), (4, 3, 6), (3, 0, 6), FACES[1]]
    centre = (0.5, 0.5)
    uvs = [[(0, 0), (1, 0), centre], [(1, 0), (1, 1), centre], [(1, 1), (0, 1), centre], [(0, 1), (0, 0), centre], UVS[1]]
    shifted = [[uv if uv != centre else (0.5, 0.625) for uv in face] for face in uvs]
    a = strip(points=points, faces=fan, uvs=uvs, owners=(0, 0, 0, 0, 1), refs=REFS + [None])
    b = strip(points=points, faces=fan, uvs=shifted, owners=(0, 0, 0, 0, 1), refs=REFS + [None])
    assert classes(record(a), record(strip()))[0] == "EQUIVALENT"  # the centre vertex on the plane is another triangulation
    verdict, _found, report = classes(record(a), record(b))
    assert verdict == GEOMETRY and any("inside a region" in item.what for item in report.findings)


def _mesh(welded=True):
    # the strip as two DOMAINS 3 and 4 welded along x = 1, or with the join's vertices duplicated (a crack)
    points = list(POINTS) + ([] if welded else [(1, 0, 0), (1, 1, 0)])
    right = (1, 2, 5, 4) if welded else (6, 2, 5, 7)
    return {
        "positions": [[sde.wire(float(c)) for c in p] for p in points], "faces": [list(FACES[0]), list(right)],
        "uvs": [[sde.wire(float(c)) for c in uv] for face in UVS for uv in face], "face_domain": [3, 4], "face_owner": [0, 0], "seams": [], "skipped": [],
    }


def test_an_unwelded_join_between_domains_is_a_geometry_error():
    assert classes(record(strip(), mesh=_mesh()), record(strip(), mesh=_mesh()))[0] == "EQUAL"
    verdict, _found, report = classes(record(strip(), mesh=_mesh()), record(strip(), mesh=_mesh(welded=False)))
    assert verdict == GEOMETRY
    assert any("join of domains 3 and 4" in item.what for item in report.findings)
    assert any("open edges" in item.what for item in report.findings)


def test_named_outcomes_detail_text_and_budget_refusals():
    refused = record(None, outcome="MATERIALIZE_SILHOUETTE_SKIPPED_WORK_BUDGET", detail="EXACT_WORK_BUDGET_EXHAUSTED at op 812")
    assert classes(record(strip()), refused)[0] == RESOURCE
    assert classes(record(strip()), record(None, outcome="DEVELOPABLE_STRETCH_BUDGET_EXCEEDED"))[0] == GEOMETRY  # a policy of the request, not a resource
    assert classes(record(strip()), record(None, outcome="NEAR_PLANAR_REFUSED"))[0] == GEOMETRY
    verdict, found, _ = classes(record(strip(), detail="took 3 ms"), record(strip(), detail="took 4 ms"))
    assert verdict == "EQUIVALENT" and found == {REPRESENTATION}
    other_domain = record(strip(), patch="8")
    assert classes(record(strip()), other_domain)[0] == GEOMETRY


def test_a_resource_refusal_against_another_refusal_is_a_geometry_error_and_the_policy_is_named():
    """CONTRACT_GATES_FIX_V1, finding 2 for the press records: only "materialized <-> resource refusal" (exact names) is RESOURCE_POLICY."""

    budget = record(None, outcome="EXACT_WORK_BUDGET_EXHAUSTED")
    report = sde.compare_press_records(record(strip()), budget)
    assert report.verdict == RESOURCE and "EXACT_CANONICALIZATION_WORK_CAP_V1" in report.of(RESOURCE)[0].what
    assert classes(record(None, outcome="NEAR_PLANAR_REFUSED"), budget)[0] == GEOMETRY
    assert classes(record(None, outcome="EXACT_WORK_BUDGET_EXHAUSTED"), record(None, outcome="SYMBOLIC_SPLIT_BUDGET_EXHAUSTED"))[0] == GEOMETRY
    lookalike = record(None, outcome="NOT_A_WORK_BUDGET_REFUSAL", detail="EXACT_WORK_BUDGET_EXHAUSTED somewhere in the text")
    assert classes(record(strip()), lookalike)[0] == GEOMETRY


def test_a_materialized_domain_whose_view_one_side_lacks_is_not_verifiable():
    """CONTRACT_GATES_FIX_V1, finding 4a: the provenance (`semantic_location_ref`) lives only in the view; without it the mesh may be equal and the
    provenance is unchecked, which is not a full EQUIVALENT."""

    report = sde.compare_press_records(record(strip()), record(None))
    assert report.verdict == sde.NOT_VERIFIABLE and report.of(sde.PROVENANCE_UNCHECKED) and not report.of(GEOMETRY), [item.line() for item in report.findings]
    assert sde.compare_press_records(record(None), record(strip())).verdict == sde.NOT_VERIFIABLE
    assert sde.compare_press_records(record(None, outcome="NEAR_PLANAR_REFUSED"), record(None, outcome="NEAR_PLANAR_REFUSED")).verdict == "EQUAL"


def _domains_mesh(skip_right=False, moved=False):
    mesh = _mesh()
    if skip_right:
        mesh["faces"], mesh["face_domain"], mesh["face_owner"] = mesh["faces"][:1], [3], [0]
        mesh["uvs"] = mesh["uvs"][:4]
        mesh["skipped"] = [["4", "EXACT_WORK_BUDGET_EXHAUSTED"]]
    if moved:
        mesh["positions"][0] = [sde.wire(0.0), sde.wire(-0.25), sde.wire(0.0)]
    return mesh


def _domains(right_outcome="MATERIALIZED", mesh=None):
    left = {"outcome": "MATERIALIZED", "detail": "", "diagnostics": [], "view": strip()}
    right = {"outcome": right_outcome, "detail": "", "diagnostics": [], "view": strip() if right_outcome == "MATERIALIZED" else None}
    return {"schema": sde.SCHEMA, "domains": {"3": left, "4": right}, "mesh": mesh}


def test_the_mesh_consequences_of_a_resource_refused_domain_are_resource_policy_and_the_rest_is_still_checked():
    """CONTRACT_GATES_FIX_V1, finding 4b: the faces, regions, joins and open edges the refused domain takes from the mesh are not GEOMETRY_ERRORs
    on top of its RESOURCE_POLICY; every other domain of the mesh is compared as before."""

    whole = _domains(mesh=_domains_mesh())
    refused = _domains("EXACT_WORK_BUDGET_EXHAUSTED", mesh=_domains_mesh(skip_right=True))
    report = sde.compare_press_records(whole, refused)
    assert report.verdict == RESOURCE and not report.of(GEOMETRY), [item.line() for item in report.findings]
    assert sde.compare_press_records(refused, whole).verdict == RESOURCE
    broken = _domains("EXACT_WORK_BUDGET_EXHAUSTED", mesh=_domains_mesh(skip_right=True, moved=True))
    found = sde.compare_press_records(whole, broken)
    assert found.verdict == GEOMETRY and all(item.where == "mesh" for item in found.of(GEOMETRY)), [item.line() for item in found.findings]
    other = _domains("NEAR_PLANAR_REFUSED", mesh=_domains_mesh(skip_right=True))
    assert any(item.where == "mesh" for item in sde.compare_press_records(whole, other).of(GEOMETRY)), "a refusal that is not a resource refusal keeps its mesh errors"


def test_determinism_turns_every_non_representation_difference_into_a_geometry_error():
    budget = record(None, outcome="SYMBOLIC_SPLIT_BUDGET_EXHAUSTED")
    report = sde.compare_determinism({"w0": record(strip()), "w12": budget})
    assert report.verdict == GEOMETRY and "determinism w0 vs w12" in report.findings[0].where
    assert sde.compare_determinism({"w0": record(strip()), "w12": record(strip(owners=(5, 3)))}).verdict == "EQUIVALENT"


def test_press_record_of_results_and_the_cli(tmp_path):
    vertices = SimpleNamespace(positions=tuple(map(lambda p: tuple(float(c) for c in p), POINTS)), refs=tuple(REFS))
    view = SimpleNamespace(failure=None, vertices=vertices, faces=FACES, uvs=tuple(tuple(float(c) for c in uv) for face in UVS for uv in face), owners=(0, 1), seams=((1, 4),))
    result = SimpleNamespace(patch_id=7, outcome="MATERIALIZED", detail="", diagnostics=(), is_materialized=True)
    refused = SimpleNamespace(patch_id=9, outcome="EXACT_WORK_BUDGET_EXHAUSTED", detail="", diagnostics=("d",), is_materialized=False)
    arrays = SimpleNamespace(positions=vertices.positions, faces=FACES, uvs=view.uvs, face_domain=(7, 7), face_owner=(0, 1), seam_edges=((1, 4),), skipped=((9, "d9", "EXACT_WORK_BUDGET_EXHAUSTED", ""),))
    built = sde.press_record([result, refused], arrays, lambda item: view)
    assert built["domains"]["7"]["view"] == strip() and built["domains"]["9"]["view"] is None
    first, second = tmp_path / "a.json", tmp_path / "b.json"
    first.write_text(json.dumps(built), encoding="utf-8")
    moved = copy.deepcopy(built)
    moved["mesh"]["positions"][0] = [sde.wire(0.0), sde.wire(-0.5), sde.wire(0.0)]
    second.write_text(json.dumps(moved), encoding="utf-8")
    assert sde.main([str(first), str(first)]) == 0
    assert sde.main([str(first), str(second), "--json", str(tmp_path / "r.json")]) == 1
    assert json.loads((tmp_path / "r.json").read_text(encoding="utf-8"))["verdict"] == GEOMETRY


def _tree_report(path, mesh_hash):
    press = {
        "mesh_hash": mesh_hash, "mesh_counts": {}, "content_keys": [], "rows": [], "profile_counters": {}, "profile_domain_counters": "", "profile_receipts": "",
        "profile_timing_stages": [], "domains": 1, "materialized": 1, "outcomes": {"MATERIALIZED": 1}, "semantic_record": str(path),
    }
    return {"workers": 0, "case": "c", "presses": {"cold": press}, "selection_sha256_16": "s", "native_build_id": "n"}


def test_tree_ab_semantic_mode_reclassifies_byte_differences(tmp_path):
    same, relabelled, moved = tmp_path / "same.json", tmp_path / "relabelled.json", tmp_path / "moved.json"
    same.write_text(json.dumps(record(strip())), encoding="utf-8")
    relabelled.write_text(json.dumps(record(strip(owners=(5, 3)))), encoding="utf-8")
    moved.write_text(json.dumps(record(strip(points=[(0, -1, 0)] + POINTS[1:]))), encoding="utf-8")
    in_bytes = ab.compare_runs(_tree_report(same, "aa"), _tree_report(relabelled, "bb"))
    assert not in_bytes.identical and ab.verdict_of(in_bytes).startswith("DIFFERENT")
    equivalent = ab.compare_runs_semantic(_tree_report(same, "aa"), _tree_report(relabelled, "bb"))
    assert equivalent.identical and ab.verdict_of(equivalent) == "EQUIVALENT", equivalent.differences
    assert any("REPRESENTATION_ONLY (bytes)" in line and "MESH HASH" in line for line in equivalent.notes)
    broken = ab.compare_runs_semantic(_tree_report(same, "aa"), _tree_report(moved, "aa"))
    assert ab.verdict_of(broken).startswith("GEOM_ERROR") and broken.classes[GEOMETRY]
    missing = _tree_report(same, "aa")
    del missing["presses"]["cold"]["semantic_record"]
    assert not ab.compare_runs_semantic(_tree_report(same, "aa"), missing).identical
    other_inputs = _tree_report(relabelled, "aa")
    other_inputs["selection_sha256_16"] = "t"
    assert not ab.compare_runs_semantic(_tree_report(same, "aa"), other_inputs).identical


def test_tree_ab_does_not_call_an_unverified_provenance_equivalent(tmp_path):
    """CONTRACT_GATES_FIX_V1, finding 4a in the driver: a press whose provenance could not be checked is NOT_VERIFIABLE, and the run's last line says so."""

    same, without = tmp_path / "same.json", tmp_path / "without.json"
    same.write_text(json.dumps(record(strip())), encoding="utf-8")
    without.write_text(json.dumps(record(None)), encoding="utf-8")
    found = ab.compare_runs_semantic(_tree_report(same, "aa"), _tree_report(without, "aa"))
    assert ab.verdict_of(found) == "NOT_VERIFIABLE" and found.unverified and not found.differences, (found.differences, found.notes)
    assert ab.verdict_of(ab.compare_runs_semantic(_tree_report(same, "aa"), _tree_report(same, "aa"))) == "EQUIVALENT"
    results = [{"verdict": "EQUIVALENT"}, {"verdict": "NOT_VERIFIABLE"}]
    line, code = ab.final_verdict(results, [], semantic=True)
    assert line.startswith("TREE_AB_NOT_VERIFIABLE ") and "TREE_AB_EQUIVALENT" not in line and code == ab.EXIT_NOT_VERIFIABLE
    assert ab.final_verdict(results[:1], [], semantic=True) == ("TREE_AB_EQUIVALENT cases=1 differing=0", ab.EXIT_IDENTICAL)
    assert ab.final_verdict(results + [{"verdict": "GEOM_ERROR(1)"}], [], semantic=True)[0].startswith("TREE_AB_DIFFERENT ")


def test_tree_ab_semantic_determinism_across_worker_counts(tmp_path):
    same, moved = tmp_path / "same.json", tmp_path / "moved.json"
    same.write_text(json.dumps(record(strip())), encoding="utf-8")
    moved.write_text(json.dumps(record(strip(points=[(0, -1, 0)] + POINTS[1:]))), encoding="utf-8")
    [(press, found)] = ab.semantic_determinism("c", {0: _tree_report(same, "a"), 12: _tree_report(same, "a")})
    assert press == "cold" and found.verdict == "EQUAL"
    [(_press, found)] = ab.semantic_determinism("c", {0: _tree_report(same, "a"), 12: _tree_report(moved, "a")})
    assert found.verdict == GEOMETRY


@pytest.mark.parametrize("value, expected", [(1, Fraction(1)), (0.5, Fraction(1, 2)), ("3/4", Fraction(3, 4)), ((0.1).hex(), Fraction(0.1)), (Fraction(2, 3), Fraction(2, 3))])
def test_exact_reads_every_spelling(value, expected):
    assert sde.exact(value) == expected
