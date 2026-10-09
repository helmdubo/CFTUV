"""Судья полевых случаев (`artifacts/materialize_sweep/field_judge.py`) и три спецификации объявленных сдвигов выпуска.

Спецификации — данные (`specs/*.json`), и проза в `DECISIONS.md` о них ничего не доказывает; здесь они исполняются:

* `cone_angle_numeric_windows` (инструмент `field`): у `rounded_wall_noise_top` 0.25/d2/20 и 0.5/d2/42 меш не двигается вовсе, остальные случаи тоже; красные контроли на
  синтетических записях полевого прогона (двинулась геометрия объявленного случая, двинулся чужой случай, случай исчез);
* `exact_scalar_text_canon_v2` (`gate`) и `canon_v2_interface_representation_shift` (`sweep`) объявляют ОДНИ И ТЕ ЖЕ 24 строки building (8 патчей x d1, d2, d4), а вторая делит их
  на 15 строк с развёрнутыми интерфейсными цепями и 9 строк с именами: разбиение, сумма и согласие двух спецификаций проверяются, а не пересказываются.
"""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SWEEP_DIR = ROOT / "artifacts" / "materialize_sweep"
SPECS = SWEEP_DIR / "specs"
CONE_CASES = ("rounded_wall_noise_top:0.25:2:20", "rounded_wall_noise_top:0.5:2:42")


def _load(name: str, path: Path):
    if str(path.parent) not in sys.path:
        sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def judge():
    return _load("field_judge_under_test_2", SWEEP_DIR / "field_judge.py")


def _row(case: str, **over) -> dict:
    row = {
        "case": case,
        "operator": ["FINISHED"],
        "status": "MATERIALIZED 5 / refused 0",
        "src_faces": 40,
        "verts": 100,
        "edges": 180,
        "faces": 80,
        "face_sizes": {"3": 10, "4": 70},
        "domain_outcomes": {"MATERIALIZED": 5},
        "refused": [],
        "op_error": None,
        "mesh_digest": "m-" + case,
        "geometry_sha256": "g-" + case,
        "button_seconds": 1.0,
    }
    row.update(over)
    return row


CASES = ("2:0.25:2:20", "building:0.25:2:20", "sagging_wall:0.987:2:42") + CONE_CASES


def _record(**changed) -> dict:
    return {"cases": [_row(case, **changed.get(case, {})) for case in CASES]}


def _verdict(judge, base, new, spec="cone_angle_numeric_windows"):
    report = judge.compare(base, new, judge.load_spec(spec))
    return report.exit_code, judge.expected_change.verdict_line(report)


def test_identical_records_with_the_cone_spec_are_identical_and_the_clock_is_not_an_answer(judge):
    base, new = _record(), _record()
    new["cases"][3]["button_seconds"] = 99.0
    code, verdict = _verdict(judge, base, new)
    assert (code, verdict) == (0, "IDENTICAL")


@pytest.mark.parametrize("case", CONE_CASES)
@pytest.mark.parametrize("name", ("geometry_sha256", "mesh_digest", "verts", "faces"))
def test_the_mesh_of_a_cone_window_case_does_not_move_even_by_one_name(judge, case, name):
    value = 101 if name in ("verts", "faces") else "changed"
    code, verdict = _verdict(judge, _record(), _record(**{case: {name: value}}))
    assert code == 1 and verdict.startswith("UNEXPECTED (spec cone_angle_numeric_windows)"), verdict


def test_another_case_moving_is_unexpected_under_the_cone_spec(judge):
    code, verdict = _verdict(judge, _record(), _record(**{"building:0.25:2:20": {"geometry_sha256": "moved"}}))
    assert code == 1 and "UNEXPECTED" in verdict


def test_without_a_spec_every_answer_change_is_unexpected(judge):
    report = judge.compare(_record(), _record(**{"2:0.25:2:20": {"verts": 1}}))
    assert report.exit_code == 1 and "UNEXPECTED" in judge.expected_change.verdict_line(report)


def test_a_refused_button_is_a_failed_domain_not_a_pass(judge):
    code, _verdict_text = _verdict(judge, _record(), _record(**{CONE_CASES[0]: {"operator": ["CANCELLED"]}}))
    assert code == 1


def test_a_case_missing_from_the_new_record_is_a_problem_unless_the_run_is_partial(judge):
    new = _record()
    del new["cases"][-1]
    assert judge.compare(_record(), new, judge.load_spec("cone_angle_numeric_windows")).exit_code == 1
    assert judge.compare(_record(), new, judge.load_spec("cone_angle_numeric_windows"), partial=True).exit_code == 0


def test_the_spec_names_exactly_the_two_cases_of_the_decision():
    raw = json.loads((SPECS / "cone_angle_numeric_windows.json").read_text(encoding="utf-8"))
    (group,) = raw["groups"]
    (clause,) = group["domains"]["where"]
    assert clause["field"] == "case" and tuple(clause["value"]) == CONE_CASES
    assert group["must_change"] == "none" and group["allow"] == {}
    assert raw["tool"] == "field"


# --------------------------------------------------------------------------
# Две спецификации канона V2: одни и те же 24 строки building
# --------------------------------------------------------------------------


def _by_density(spec: dict) -> dict:
    return {density: sorted(ids) for density, ids in spec["domains"]["by_density"].items()}


def test_the_gate_and_the_sweep_specs_of_canon_v2_declare_the_same_24_rows():
    gate = json.loads((SPECS / "exact_scalar_text_canon_v2.json").read_text(encoding="utf-8"))
    sweep = json.loads((SPECS / "canon_v2_interface_representation_shift.json").read_text(encoding="utf-8"))
    declared = _by_density(gate)
    assert set(declared) == {"1", "2", "4"} and all(ids == [0, 1, 6, 7, 17, 105, 106, 109] for ids in declared.values())
    groups = {item["name"]: _by_density({"domains": item["domains"]}) for item in sweep["groups"]}
    assert set(groups) == {"interface_chains_reversed", "names_only"}
    for density in declared:
        reversed_ids, named_ids = groups["interface_chains_reversed"][density], groups["names_only"][density]
        assert not set(reversed_ids) & set(named_ids), "a row is either reversed or names-only"
        assert sorted(reversed_ids + named_ids) == declared[density]
    assert sum(len(ids) for ids in groups["interface_chains_reversed"].values()) == 15
    assert sum(len(ids) for ids in groups["names_only"].values()) == 9


def test_canon_v2_specs_allow_only_digests_and_judge_nothing_else():
    gate = json.loads((SPECS / "exact_scalar_text_canon_v2.json").read_text(encoding="utf-8"))
    sweep = json.loads((SPECS / "canon_v2_interface_representation_shift.json").read_text(encoding="utf-8"))
    assert gate["allow"] == {"digests": ["fp_geometry", "fp_geometry_value"]}
    assert all(item["allow"] == {"digests": ["content_digest", "semantic_digest"]} for item in sweep["groups"])
    assert all(item["must_change"] == "each" for item in sweep["groups"]) and gate["must_change"] == "each"
    assert "allow_everywhere" not in gate and "allow_everywhere" not in sweep


def test_the_sweep_spec_of_canon_v2_fails_when_a_counter_moves_on_a_declared_row():
    sweep = _load("sweep_for_field_judge_tests", SWEEP_DIR / "sweep.py")
    ec = sweep.expected_change
    spec = ec.load_spec("canon_v2_interface_representation_shift", "sweep", sweep.VOCABULARY)

    def row(**over):
        base = {
            "patch_id": 7, "density": 1, "prepare_outcome": "EXACT", "coverage_outcome": "EXACT", "materialization": "MATERIALIZED", "detail": "",
            "content_digest": "c", "offset_normals_digest": "", "semantic_digest": "s", "counters": {"MATERIALIZE_FACES_IN": 3},
            "topology_counters": {}, "untracked_counters": {}, "diagnostics": [], "chart": None, "planarity": "ExactSourcePlaneCertificateV1",
        }
        base.update(over)
        return base

    def record(item):
        # d1: патч 7 - в группе развёрнутых цепей, патч 0 - в группе имён; обе группы обязаны найти своих
        return {"runs": {"1": {"domains": {"7": item, "0": {**item, "patch_id": 0}}}}}

    ok = ec.evaluate([record(row()), record(row(content_digest="c2", semantic_digest="s2"))], ["base", "new"], spec, sweep.pair_views(False), sweep.VOCABULARY, True)
    assert ok.exit_code == 0 and ec.verdict_line(ok).startswith("EXPECTED-CHANGE")
    moved = row(content_digest="c2", semantic_digest="s2", counters={"MATERIALIZE_FACES_IN": 4})
    bad = ec.evaluate([record(row()), record(moved)], ["base", "new"], spec, sweep.pair_views(False), sweep.VOCABULARY, True)
    assert bad.exit_code == 1
    same = ec.evaluate([record(row()), record(copy.deepcopy(row()))], ["base", "new"], spec, sweep.pair_views(False), sweep.VOCABULARY, True)
    assert same.exit_code == 1, "a declared row that did not change fails: must_change is 'each'"
