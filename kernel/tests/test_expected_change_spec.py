"""Ожидаемое изменение ответа ворот (`artifacts/materialize_sweep/expected_change.py`): спецификация закона и её проверка.

Раньше `sweep.py compare --expect-changed --ignore-counters` держал разрешённое поимённо в коде под один закон, и
следующие релизы сверяли «по полям» отдельными скриптами: ворота молча слабели. Теперь разрешённое — данные
(`specs/*.json`), проверка одна на `sweep.py`, `gate.py` и `clip_gate.py`. Здесь:

* КРАСНЫЕ КОНТРОЛИ на синтетических строках: спецификация, разрешающая поле A, валит изменение поля B; список
  объявленных доменов валит лишний изменившийся домен и объявленный, который не изменился; отсутствующий счётчик
  равен нулю, новый ненулевой — расхождение; инварианты `require`; схема спецификации отказывает по имени;
* НАСТОЯЩИЕ записи (`kernel/fixtures/expected_change/*.json`, обрезанные прогоны `building`): закон станции на
  хорде (запись «до» — `--chord-station off`), резка по граням (`--near-planar-law SOURCE_TRIANGLES_V1`), JOIN по
  тождеству цепи (деревья 4404607 -> 80f0366), предсказуемые веера (ворота `numeric_repr`, деревья 85ac03b -> 1b55bf0)
  проходят по сохранённым спецификациям ровно теми доменами, что названы в DECISIONS;
* пара настоящих записей `numeric_repr` (`baseline_18d7197.json`, `baseline_e548fb5.json`), где ответ законно разошёлся
  на одном домене в двух полях: спецификация на одно из двух полей валит.
"""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
SWEEP_DIR = ROOT / "artifacts" / "materialize_sweep"
FIXTURES = ROOT / "kernel" / "fixtures" / "expected_change"
NUMERIC_REPR = ROOT / "artifacts" / "numeric_repr"
EXACT_PLANE = "ExactSourcePlaneCertificateV1"


def _load(name: str, path: Path):
    if str(path.parent) not in sys.path:
        sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def sweep():
    return _load("materialize_sweep_under_test", SWEEP_DIR / "sweep.py")


@pytest.fixture(scope="module")
def ec(sweep):
    return sweep.expected_change


@pytest.fixture(scope="module")
def gate(sweep):
    return sweep.gate


# --------------------------------------------------------------------------
# Синтетические строки
# --------------------------------------------------------------------------


def _row(**over):
    row = {
        "patch_id": 1,
        "density": 1,
        "prepare_outcome": "EXACT",
        "coverage_outcome": "EXACT",
        "materialization": "MATERIALIZED",
        "detail": "",
        "content_digest": "c",
        "offset_normals_digest": "o",
        "semantic_digest": "s",
        "counters": {"MATERIALIZE_FACES_IN": 3},
        "topology_counters": {"MATERIALIZE_FACES_EMITTED": 4},
        "untracked_counters": {},
        "diagnostics": [],
        "chart": None,
        "planarity": EXACT_PLANE,
    }
    row.update(over)
    return row


def _record(rows: dict, density: str = "1") -> dict:
    return {"runs": {density: {"domains": {str(patch): row for patch, row in rows.items()}}}}


def _spec(**over) -> dict:
    spec = {
        "schema": "expected_change_spec_v1",
        "name": "t",
        "tool": "sweep",
        "law": "T",
        "about": "test",
        "domains": {"explicit": [1]},
    }
    spec.update(over)
    return spec


def _judge(sweep, ec, base: dict, new: dict, spec: dict | None = None, partial: bool = False, across: bool = False):
    parsed = None if spec is None else ec.parse_spec(spec, "sweep", sweep.VOCABULARY)
    return ec.evaluate([base, new], ["base", "new"], parsed, sweep.pair_views(across), sweep.VOCABULARY, partial)


def _shifted(**over):
    """Строка, чей ответ сдвинулся: оба дайджеста новые, прочее по умолчанию."""

    return _row(**{"content_digest": "c2", "semantic_digest": "s2", **over})


SHIFT_ALLOW = {"digests": ["semantic_digest", "content_digest"]}


# --------------------------------------------------------------------------
# Вердикты и красные контроли разрешённого
# --------------------------------------------------------------------------


def test_identical_records_give_identical_and_the_exit_code_is_zero(sweep, ec):
    record = _record({1: _row(), 2: _row(patch_id=2)})
    report = _judge(sweep, ec, record, copy.deepcopy(record))
    assert ec.verdict_line(report) == "IDENTICAL"
    assert report.exit_code == 0


def test_without_a_spec_any_answer_change_is_unexpected(sweep, ec):
    report = _judge(sweep, ec, _record({1: _row()}), _record({1: _shifted()}))
    assert ec.verdict_line(report).startswith("UNEXPECTED: 2 problems; first: ")
    assert report.exit_code == 1


def test_a_declared_domain_changing_only_what_the_spec_allows_is_an_expected_change(sweep, ec):
    spec = _spec(allow=SHIFT_ALLOW)
    report = _judge(sweep, ec, _record({1: _row(), 2: _row()}), _record({1: _shifted(), 2: _row()}), spec)
    assert report.problems == []
    assert ec.verdict_line(report) == "EXPECTED-CHANGE (spec t): 1 domains"
    assert report.changed_domains == ["1"]


def test_a_spec_that_allows_field_a_fails_when_field_b_changes(sweep, ec):
    spec = _spec(allow={"digests": ["semantic_digest"]})
    report = _judge(sweep, ec, _record({1: _row()}), _record({1: _shifted()}), spec)
    assert report.exit_code == 1
    assert any("field content_digest" in line and "not allowed for group t" in line for line in report.problems)
    assert not any("semantic_digest" in line and "UNEXPECTED" in line for line in report.problems)


def test_a_spec_that_allows_counter_a_fails_when_counter_b_changes(sweep, ec):
    spec = _spec(allow={"digests": ["semantic_digest", "content_digest"], "counters": ["MATERIALIZE_FACES_IN"]})
    new = _shifted(counters={"MATERIALIZE_FACES_IN": 9}, topology_counters={"MATERIALIZE_FACES_EMITTED": 5})
    report = _judge(sweep, ec, _record({1: _row()}), _record({1: new}), spec)
    assert [line for line in report.problems if "UNEXPECTED" in line and "MATERIALIZE_FACES_EMITTED" in line]
    assert not [line for line in report.problems if "MATERIALIZE_FACES_IN" in line]


def test_a_changed_outcome_field_is_never_covered_by_a_digest_allowance(sweep, ec):
    spec = _spec(allow=SHIFT_ALLOW)
    new = _shifted(materialization="REFUSED")
    report = _judge(sweep, ec, _record({1: _row()}), _record({1: new}), spec)
    assert any("field materialization" in line for line in report.problems)


def test_a_declared_domain_list_fails_on_an_extra_changed_domain(sweep, ec):
    spec = _spec(allow=SHIFT_ALLOW, domains={"explicit": [1]})
    report = _judge(sweep, ec, _record({1: _row(), 2: _row()}), _record({1: _shifted(), 2: _shifted()}), spec)
    assert report.exit_code == 1
    assert all("patch2" in line and "domain is not declared" in line for line in report.problems)
    assert ec.verdict_line(report).startswith("UNEXPECTED (spec t): ")


def test_a_declared_domain_that_did_not_change_fails_when_the_spec_says_it_must(sweep, ec):
    spec = _spec(allow=SHIFT_ALLOW, domains={"explicit": [1, 2]})
    report = _judge(sweep, ec, _record({1: _row(), 2: _row()}), _record({1: _shifted(), 2: _row()}), spec)
    assert report.exit_code == 1
    assert any("patch2: expected change is absent" in line for line in report.problems)


def test_must_change_any_and_none_relax_the_declared_domain_that_stayed(sweep, ec):
    base, new = _record({1: _row(), 2: _row()}), _record({1: _shifted(), 2: _row()})
    assert _judge(sweep, ec, base, new, _spec(allow=SHIFT_ALLOW, domains={"explicit": [1, 2]}, must_change="any")).problems == []
    assert _judge(sweep, ec, base, new, _spec(allow=SHIFT_ALLOW, domains={"explicit": [1, 2]}, must_change="none")).problems == []
    nothing = _judge(sweep, ec, base, base, _spec(allow=SHIFT_ALLOW, domains={"explicit": [1, 2]}, must_change="any"))
    assert any("must_change is 'any' and no declared domain changed" in line for line in nothing.problems)


def test_a_declared_domain_set_that_the_records_do_not_contain_fails_unless_the_run_is_partial(sweep, ec):
    spec = _spec(allow=SHIFT_ALLOW, domains={"explicit": [1, 7]})
    base, new = _record({1: _row()}), _record({1: _shifted()})
    strict = _judge(sweep, ec, base, new, spec)
    assert any("declares domains absent from the compared records: [7]" in line for line in strict.problems)
    partial = _judge(sweep, ec, base, new, spec, partial=True)
    assert partial.problems == []
    assert any("(partial run)" in note for note in partial.notes)


def test_different_domain_sets_fail_and_are_a_note_in_a_partial_run(sweep, ec):
    base, new = _record({1: _row(), 2: _row()}), _record({1: _row()})
    assert any("domain sets differ" in line for line in _judge(sweep, ec, base, new).problems)
    assert _judge(sweep, ec, base, new, partial=True).problems == []


def test_a_density_present_in_one_record_only_is_a_printed_note_not_a_silent_skip(sweep, ec):
    base = {"runs": {**_record({1: _row()})["runs"], "2": {"domains": {"1": _row()}}}}
    new = _record({1: _row()})
    report = _judge(sweep, ec, base, new)
    assert report.problems == []
    assert any("density 2 is present in only one record" in note for note in report.notes)


# --------------------------------------------------------------------------
# Отсутствующий счётчик равен нулю; новый счётчик объявляется
# --------------------------------------------------------------------------


@pytest.mark.parametrize("group", ["counters", "topology_counters", "untracked_counters"])
def test_a_missing_counter_equals_zero_in_every_counter_group(sweep, ec, group):
    old = _row()
    new = _row()
    new[group] = {**new[group], "STATION_FLOW_CYCLES_OPENED": 0}
    assert _judge(sweep, ec, _record({1: old}), _record({1: new})).problems == []
    assert _judge(sweep, ec, _record({1: new}), _record({1: old})).problems == []


@pytest.mark.parametrize("group", ["counters", "topology_counters", "untracked_counters"])
def test_a_new_nonzero_counter_is_a_difference_until_the_spec_declares_it(sweep, ec, group):
    old = _row()
    new = _row()
    new[group] = {**new[group], "MATERIALIZE_CLIP_FACES_CUT": 2}
    undeclared = _judge(sweep, ec, _record({1: old}), _record({1: new}), _spec(allow={"digests": ["semantic_digest"]}, must_change="none"))
    assert any("counter MATERIALIZE_CLIP_FACES_CUT" in line and "0 -> 2" in line for line in undeclared.problems)
    by_name = _judge(sweep, ec, _record({1: old}), _record({1: new}), _spec(allow={"counters": ["MATERIALIZE_CLIP_FACES_CUT"]}))
    assert by_name.problems == []
    by_prefix = _judge(sweep, ec, _record({1: old}), _record({1: new}), _spec(allow={"counter_prefixes": ["MATERIALIZE_CLIP_"]}))
    assert by_prefix.problems == []
    gone = _judge(sweep, ec, _record({1: new}), _record({1: old}))
    assert any("2 -> 0" in line for line in gone.problems)


def test_unlisted_kernel_counters_are_compared_and_one_sided_records_are_skipped_with_a_note(sweep, ec):
    with_counter = _row(untracked_counters={"MATERIALIZE_CLIP_FACES_CUT": 2})
    differing = _judge(sweep, ec, _record({1: _row()}), _record({1: with_counter}))
    assert any("MATERIALIZE_CLIP_FACES_CUT" in line for line in differing.problems)
    old_format = _row()
    del old_format["untracked_counters"]
    skipped = _judge(sweep, ec, _record({1: old_format}), _record({1: with_counter}))
    assert skipped.problems == []
    assert any("untracked_counters" in note and "not compared" in note for note in skipped.notes)


def test_the_two_counter_groups_and_the_untracked_names_never_overlap(sweep):
    assert not set(sweep.COUNTER_KEYS) & set(sweep.TOPOLOGY_COUNTER_KEYS)
    assert sweep.TRACKED_COUNTER_KEYS == set(sweep.COUNTER_KEYS) | set(sweep.TOPOLOGY_COUNTER_KEYS)


# --------------------------------------------------------------------------
# allow_everywhere, диагностики, переходы исхода
# --------------------------------------------------------------------------


def test_law_bookkeeping_is_allowed_everywhere_but_an_answer_change_on_an_undeclared_domain_is_not(sweep, ec):
    spec = _spec(allow=SHIFT_ALLOW, allow_everywhere={"counter_prefixes": ["MATERIALIZE_CHORD_STATIONS_"]})
    bookkeeping = _row(counters={"MATERIALIZE_FACES_IN": 3, "MATERIALIZE_CHORD_STATIONS_TOTAL": 2})
    ok = _judge(sweep, ec, _record({1: _row(), 2: _row()}), _record({1: _shifted(), 2: bookkeeping}), spec)
    assert ok.problems == []
    assert ok.bookkeeping_rows == 1
    assert ok.changed_domains == ["1"]
    bad = _judge(sweep, ec, _record({1: _row(), 2: _row()}), _record({1: _shifted(), 2: _shifted()}), spec)
    assert any("patch2" in line and "digest" in line for line in bad.problems)


def test_a_declared_domain_whose_only_change_is_law_bookkeeping_did_not_change(sweep, ec):
    spec = _spec(allow=SHIFT_ALLOW, allow_everywhere={"counter_prefixes": ["MATERIALIZE_CHORD_STATIONS_"]})
    bookkeeping = _row(counters={"MATERIALIZE_FACES_IN": 3, "MATERIALIZE_CHORD_STATIONS_TOTAL": 2})
    report = _judge(sweep, ec, _record({1: _row()}), _record({1: bookkeeping}), spec)
    assert any("expected change is absent" in line for line in report.problems)


def test_diagnostics_change_only_by_prefix_and_masked_numbers_never_the_rest_of_the_line(sweep, ec):
    line = "SOURCE_EDGES_LIFTED_ONTO_SURFACE: clip_vertices=4 faces_cut=5 predicates=184 max_chord_kept_nm=29951"
    spec = _spec(allow={**SHIFT_ALLOW, "diagnostics": [{"prefix": "SOURCE_EDGES_LIFTED_ONTO_SURFACE", "numbers": ["faces_cut", "predicates"]}]})
    base = _record({1: _row(diagnostics=[line])})
    masked = _shifted(diagnostics=[line.replace("faces_cut=5", "faces_cut=6").replace("predicates=184", "predicates=179")])
    assert _judge(sweep, ec, base, _record({1: masked}), spec).problems == []
    other = _shifted(diagnostics=[line.replace("clip_vertices=4", "clip_vertices=5")])
    report = _judge(sweep, ec, base, _record({1: other}), spec)
    assert any("diagnostic SOURCE_EDGES_LIFTED_ONTO_SURFACE" in problem for problem in report.problems)
    whole = _spec(allow={**SHIFT_ALLOW, "diagnostics": [{"prefix": "SOURCE_EDGES_LIFTED_ONTO_SURFACE"}]})
    assert _judge(sweep, ec, base, _record({1: other}), whole).problems == []
    appeared = _shifted(diagnostics=[line, "NEW_NAMED_OUTCOME: 1"])
    assert any("diagnostic NEW_NAMED_OUTCOME" in problem for problem in _judge(sweep, ec, base, _record({1: appeared}), whole).problems)


def test_an_outcome_change_is_allowed_only_for_the_declared_pair(sweep, ec):
    refused = _row(materialization="NOT_ATTEMPTED", prepare_outcome="REFUSED", semantic_digest="")
    built = _row(prepare_outcome="EXACT", materialization="MATERIALIZED", semantic_digest="s2", content_digest="c2")
    transitions = [
        {"field": "prepare_outcome", "from": "REFUSED", "to": "EXACT"},
        {"field": "materialization", "from": "NOT_ATTEMPTED", "to": "MATERIALIZED"},
    ]
    spec = _spec(allow={**SHIFT_ALLOW, "outcome_transitions": transitions})
    assert _judge(sweep, ec, _record({1: refused}), _record({1: built}), spec).problems == []
    backwards = _judge(sweep, ec, _record({1: built}), _record({1: refused}), spec)
    assert any("field materialization" in line for line in backwards.problems)
    wrong = _spec(allow={**SHIFT_ALLOW, "outcome_transitions": [transitions[0]]})
    assert any("field materialization" in line for line in _judge(sweep, ec, _record({1: refused}), _record({1: built}), wrong).problems)


# --------------------------------------------------------------------------
# Предикат, группы
# --------------------------------------------------------------------------


def _where(side: str, op: str, value, **target):
    return {"where": [{"side": side, "op": op, "value": value, **target}]}


def test_a_predicate_selects_by_the_side_it_names(sweep, ec):
    base = _record({1: _row(topology_counters={"STATION_JOIN_CORNERS": 0}), 2: _row()})
    new = _record({1: _shifted(topology_counters={"STATION_JOIN_CORNERS": 2}), 2: _row()})
    allow = {**SHIFT_ALLOW, "counters": ["STATION_JOIN_CORNERS"]}
    on_new = _spec(allow=allow, domains=_where("new", "gt", 0, counter="STATION_JOIN_CORNERS"))
    assert _judge(sweep, ec, base, new, on_new).problems == []
    on_base = _spec(allow=allow, domains=_where("base", "gt", 0, counter="STATION_JOIN_CORNERS"))
    report = _judge(sweep, ec, base, new, on_base)
    assert any("group t declares no domain in the compared records" in line for line in report.problems)
    assert any("patch1" in line and "domain is not declared" in line for line in report.problems)
    either = _spec(allow=allow, domains=_where("either", "gt", 0, counter="STATION_JOIN_CORNERS"))
    assert _judge(sweep, ec, base, new, either).problems == []


@pytest.mark.parametrize(
    ("op", "value", "expected"),
    [
        ("eq", "x", True), ("ne", "x", False), ("in", ["x", "y"], True), ("not_in", ["x"], False),
        ("gt", "a", True), ("lt", "a", False), ("ge", "x", True), ("le", "x", True),
    ],
)
def test_every_predicate_operator_on_a_field(sweep, ec, op, value, expected):
    row = _row(chart="x")
    clause = {"side": "new", "field": "chart", "op": op, "value": value}
    parsed = ec.parse_spec(_spec(domains={"where": [clause]}, allow=SHIFT_ALLOW), "sweep", sweep.VOCABULARY)
    views = sweep.pair_views(False)(row, row)
    assert parsed.groups[0].selects("1", "1", *views) is expected


def test_a_predicate_over_a_missing_field_never_selects_with_an_ordering_operator(sweep, ec):
    parsed = ec.parse_spec(_spec(domains={"where": [{"field": "no_such_field", "op": "gt", "value": 0}]}), "sweep", sweep.VOCABULARY)
    views = sweep.pair_views(False)(_row(), _row())
    assert parsed.groups[0].selects("1", "1", *views) is False


def test_a_domain_takes_the_first_matching_group_and_only_that_groups_allowance(sweep, ec):
    spec = {
        "schema": "expected_change_spec_v1", "name": "t", "tool": "sweep", "law": "T", "about": "test",
        "groups": [
            {"name": "digests_only", "domains": {"explicit": [1]}, "allow": SHIFT_ALLOW},
            {"name": "digests_and_faces", "domains": {"explicit": [2]}, "allow": {**SHIFT_ALLOW, "counters": ["MATERIALIZE_FACES_IN"]}},
        ],
    }
    more_faces = {"MATERIALIZE_FACES_IN": 9}
    report = _judge(
        sweep, ec,
        _record({1: _row(), 2: _row()}),
        _record({1: _shifted(counters=more_faces), 2: _shifted(counters=more_faces)}),
        spec,
    )
    assert [line for line in report.problems if "patch1" in line and "MATERIALIZE_FACES_IN" in line]
    assert not [line for line in report.problems if "patch2" in line]
    assert report.declared_rows == {"digests_only": 1, "digests_and_faces": 1}


def test_a_domain_named_by_two_groups_takes_the_first_one_only(sweep, ec):
    spec = {
        "schema": "expected_change_spec_v1", "name": "t", "tool": "sweep", "law": "T", "about": "test",
        "groups": [
            {"name": "first", "domains": {"all": True}, "must_change": "none", "allow": SHIFT_ALLOW},
            {"name": "second", "domains": {"explicit": [1]}, "must_change": "none", "allow": {**SHIFT_ALLOW, "counters": ["MATERIALIZE_FACES_IN"]}},
        ],
    }
    report = _judge(sweep, ec, _record({1: _row()}), _record({1: _shifted(counters={"MATERIALIZE_FACES_IN": 9})}), spec)
    assert any("counter MATERIALIZE_FACES_IN" in line and "not allowed for group first" in line for line in report.problems)
    assert report.declared_rows == {"first": 1}


# --------------------------------------------------------------------------
# Инварианты require
# --------------------------------------------------------------------------


def test_require_zero_non_increasing_and_non_decreasing_judge_the_new_values(sweep, ec):
    allow = {**SHIFT_ALLOW, "counters": ["MATERIALIZE_FACES_LOST", "MATERIALIZE_FACES_IN"]}
    base = _record({1: _row()})
    lost = _judge(sweep, ec, base, _record({1: _shifted(counters={"MATERIALIZE_FACES_IN": 3, "MATERIALIZE_FACES_LOST": 1})}),
                  _spec(allow=allow, require={"zero": ["MATERIALIZE_FACES_LOST"]}))
    assert any("MATERIALIZE_FACES_LOST = 1, must be 0" in line for line in lost.problems)
    grew = _judge(sweep, ec, base, _record({1: _shifted(counters={"MATERIALIZE_FACES_IN": 4})}),
                  _spec(allow=allow, require={"non_increasing": ["MATERIALIZE_FACES_IN"]}))
    assert any("grew 3 -> 4" in line for line in grew.problems)
    fell = _judge(sweep, ec, base, _record({1: _shifted(counters={"MATERIALIZE_FACES_IN": 2})}),
                  _spec(allow=allow, require={"non_decreasing": ["MATERIALIZE_FACES_IN"]}))
    assert any("fell 3 -> 2" in line for line in fell.problems)
    fine = _judge(sweep, ec, base, _record({1: _shifted(counters={"MATERIALIZE_FACES_IN": 2})}),
                  _spec(allow=allow, require={"non_increasing": ["MATERIALIZE_FACES_IN"], "zero": ["MATERIALIZE_FACES_LOST"]}))
    assert fine.problems == []


def test_require_zero_is_checked_on_undeclared_domains_too(sweep, ec):
    lost = _row(counters={"MATERIALIZE_FACES_IN": 3, "MATERIALIZE_FACES_LOST": 1})
    report = _judge(sweep, ec, _record({1: _row(), 2: lost}), _record({1: _shifted(), 2: lost}),
                    _spec(allow=SHIFT_ALLOW, require={"zero": ["MATERIALIZE_FACES_LOST"]}))
    assert any("patch2" in line and "must be 0" in line for line in report.problems)


def test_require_no_refusal_and_non_empty_and_changed_any_of(sweep, ec):
    allow = {**SHIFT_ALLOW, "fields": ["materialization"]}
    refusing = _shifted(materialization="REFUSED", semantic_digest="")
    report = _judge(sweep, ec, _record({1: _row()}), _record({1: refusing}),
                    _spec(allow=allow, require={"no_refusal": True, "non_empty": ["semantic_digest"]}))
    assert any("no longer does (no_refusal)" in line for line in report.problems)
    assert any("field semantic_digest is empty" in line for line in report.problems)
    only_detail = _row(detail="new text")
    unchanged_digest = _judge(sweep, ec, _record({1: _row()}), _record({1: only_detail}),
                              _spec(allow={"fields": ["detail"]}, require={"changed_any_of": ["semantic_digest", "content_digest"]}))
    assert any("changed_any_of" in line for line in unchanged_digest.problems)


def test_require_unchanged_pins_a_name_even_when_allow_would_cover_it(sweep, ec):
    spec = _spec(allow={**SHIFT_ALLOW, "counter_prefixes": ["MATERIALIZE_FACES_"]}, require={"unchanged": ["MATERIALIZE_FACES_EMITTED"]})
    report = _judge(sweep, ec, _record({1: _row()}), _record({1: _shifted(topology_counters={"MATERIALIZE_FACES_EMITTED": 5})}), spec)
    assert any("pinned by require.unchanged" in line and "MATERIALIZE_FACES_EMITTED" in line for line in report.problems)


def test_across_topology_drops_the_content_digest_and_the_law_counters(sweep, ec):
    base = _record({1: _row()})
    other_law = _record({1: _row(content_digest="other", topology_counters={"MATERIALIZE_FACES_EMITTED": 99})})
    assert _judge(sweep, ec, base, other_law, across=True).problems == []
    assert _judge(sweep, ec, base, other_law).problems != []


# --------------------------------------------------------------------------
# Схема спецификации отказывает по имени
# --------------------------------------------------------------------------


def _refused(sweep, ec, spec: dict, tool: str = "sweep") -> str:
    with pytest.raises(ec.SpecError) as error:
        ec.parse_spec(spec, tool, sweep.VOCABULARY)
    return str(error.value)


def test_the_spec_schema_refuses_by_name_instead_of_ignoring(sweep, ec):
    assert "unknown key 'allows'" in _refused(sweep, ec, _spec(allows={}))
    assert "exactly one of explicit / by_density / where / all" in _refused(sweep, ec, _spec(domains={"explicit": [1], "all": True}))
    assert "must end with '_' and hold at least two words" in _refused(sweep, ec, _spec(allow={"counter_prefixes": ["MATERIALIZE_"]}))
    assert "must end with '_' and hold at least two words" in _refused(sweep, ec, _spec(allow={"counter_prefixes": ["STATION"]}))
    assert "spec is for the 'sweep' tool, not 'gate'" in _refused(sweep, ec, _spec(), tool="gate")
    assert "must_change: one of" in _refused(sweep, ec, _spec(must_change="sometimes"))
    assert "'semantic_digest' is a digest; name it under digests" in _refused(sweep, ec, _spec(allow={"fields": ["semantic_digest"]}))
    assert "'detail' is not a digest of this tool" in _refused(sweep, ec, _spec(allow={"digests": ["detail"]}))
    assert "require: unknown key 'must_be'" in _refused(sweep, ec, _spec(require={"must_be": []}))
    assert "op must be one of" in _refused(sweep, ec, _spec(domains={"where": [{"field": "chart", "op": "like", "value": "x"}]}))
    assert "are alternatives: put domains/must_change/allow inside each group" in _refused(
        sweep, ec, _spec(groups=[{"name": "g", "domains": {"all": True}}], allow={})
    )
    assert "schema must be" in _refused(sweep, ec, _spec(schema="v0"))


def test_allow_everywhere_never_admits_digests_fields_or_outcome_changes(sweep, ec):
    for key, value in (("digests", ["semantic_digest"]), ("fields", ["detail"]), ("outcome_transitions", [])):
        assert f"allow_everywhere: key '{key}' is not allowed here" in _refused(sweep, ec, _spec(allow_everywhere={key: value}))


def test_a_spec_naming_a_counter_nobody_records_is_a_printed_note(sweep, ec):
    spec = _spec(allow={**SHIFT_ALLOW, "counters": ["MATERIALIZE_NO_SUCH_COUNTER"]}, allow_everywhere={"counter_prefixes": ["NO_SUCH_PREFIX_"]})
    report = _judge(sweep, ec, _record({1: _row()}), _record({1: _shifted()}), spec)
    assert report.problems == []
    assert any("MATERIALIZE_NO_SUCH_COUNTER" in note for note in report.notes)
    assert any("NO_SUCH_PREFIX_" in note for note in report.notes)


# --------------------------------------------------------------------------
# Сохранённые спецификации и инструменты
# --------------------------------------------------------------------------

STORED = {
    "chord_station": "sweep",
    "clip_by_faces": "sweep",
    "clip_by_triangles": "sweep",
    "convex_partition": "sweep",
    "join_same_pchain": "sweep",
    "right_angle_stable": "gate",
}


def test_every_stored_spec_parses_for_its_tool_and_is_named_after_its_file(sweep, ec, gate):
    assert ec.stored_spec_names() == sorted(STORED)
    for name, tool in STORED.items():
        vocabulary = sweep.VOCABULARY if tool == "sweep" else gate.VOCABULARY
        spec = ec.load_spec(name, tool, vocabulary)
        assert spec.name == name and spec.tool == tool and spec.about and spec.law


def test_a_stored_sweep_spec_is_refused_by_the_gate_and_the_other_way_round(ec, sweep, gate):
    with pytest.raises(ec.SpecError, match="spec is for the 'sweep' tool, not 'gate'"):
        ec.load_spec("chord_station", "gate", gate.VOCABULARY)
    with pytest.raises(ec.SpecError, match="spec is for the 'gate' tool, not 'sweep'"):
        ec.load_spec("right_angle_stable", "sweep", sweep.VOCABULARY)
    with pytest.raises(ec.SpecError, match="not found"):
        ec.load_spec("no_such_law", "sweep", sweep.VOCABULARY)


def _fixture(name: str) -> dict:
    return json.loads((FIXTURES / name).read_text(encoding="utf-8"))


def _real_sweep(sweep, ec, base: str, new: str, spec_name: str | None, spec_override: dict | None = None, partial: bool = False):
    records = [_fixture(base), _fixture(new)]
    if spec_override is not None:
        spec = ec.parse_spec(spec_override, "sweep", sweep.VOCABULARY)
    else:
        spec = None if spec_name is None else ec.load_spec(spec_name, "sweep", sweep.VOCABULARY)
    return ec.evaluate(records, ["base", "new"], spec, sweep.pair_views(False), sweep.VOCABULARY, partial)


def _stored_raw(name: str) -> dict:
    return json.loads((SWEEP_DIR / "specs" / f"{name}.json").read_text(encoding="utf-8"))


CHORD_DOMAINS = [0, 1, 3, 7, 11, 15, 17, 19, 89, 105, 106, 109, 114, 115]


def test_real_records_the_chord_station_law_moves_exactly_the_released_domains_and_89(sweep, ec):
    report = _real_sweep(sweep, ec, "sweep_chord_off.json", "sweep_product.json", "chord_station")
    assert report.problems == []
    assert report.changed_domains == [str(item) for item in sorted(CHORD_DOMAINS)]
    assert ec.verdict_line(report) == f"EXPECTED-CHANGE (spec chord_station): {len(CHORD_DOMAINS)} domains"
    # Домены 20 и 110: закон считал вершины, но ответ не изменился: бухгалтерия закона, не сдвиг ответа.
    assert report.bookkeeping_rows == 2


def test_real_records_without_a_spec_the_same_pair_is_unexpected_on_every_changed_domain(sweep, ec):
    report = _real_sweep(sweep, ec, "sweep_chord_off.json", "sweep_product.json", None)
    assert report.exit_code == 1
    assert {line.split(" patch")[1].split(":")[0] for line in report.problems} >= {str(item) for item in CHORD_DOMAINS}


def test_real_records_a_chord_spec_without_domain_89_fails_on_89_alone(sweep, ec):
    raw = _stored_raw("chord_station")
    raw["groups"] = [group for group in raw["groups"] if group["name"] != "domain_reached_by_later_laws"]
    report = _real_sweep(sweep, ec, "sweep_chord_off.json", "sweep_product.json", None, raw)
    assert report.exit_code == 1
    assert {line.split(" d1 ")[1].split(":")[0] for line in report.problems} == {"patch89"}
    assert report.unexpected_rows == {("1", "89"): len(report.problems)}
    assert all("domain is not declared" in line for line in report.problems)


def test_real_records_a_chord_spec_with_a_wrong_domain_declared_fails_on_it(sweep, ec):
    raw = _stored_raw("chord_station")
    raw["groups"][0]["domains"] = {"explicit": [0, 1, 2, 3, 7, 11, 15, 17, 19, 105, 114, 115]}
    report = _real_sweep(sweep, ec, "sweep_chord_off.json", "sweep_product.json", None, raw)
    assert any("patch2: expected change is absent" in line for line in report.problems)


def test_real_records_a_chord_spec_allowing_only_the_semantic_digest_fails_on_the_content_digest(sweep, ec):
    raw = _stored_raw("chord_station")
    for group in raw["groups"]:
        group["allow"]["digests"] = ["semantic_digest"]
    report = _real_sweep(sweep, ec, "sweep_chord_off.json", "sweep_product.json", None, raw)
    assert report.exit_code == 1
    assert any("field content_digest" in line for line in report.problems)
    assert not any("field semantic_digest" in line for line in report.problems)


def test_real_records_a_chord_spec_without_the_law_bookkeeping_prefix_fails_on_the_station_counters(sweep, ec):
    raw = _stored_raw("chord_station")
    del raw["allow_everywhere"]
    report = _real_sweep(sweep, ec, "sweep_chord_off.json", "sweep_product.json", None, raw)
    assert any("counter MATERIALIZE_CHORD_STATIONS_TOTAL" in line for line in report.problems)


def test_real_records_the_chord_spec_on_two_equal_records_is_a_failed_expectation_not_a_pass(sweep, ec):
    report = _real_sweep(sweep, ec, "sweep_product.json", "sweep_product.json", "chord_station")
    assert report.exit_code == 1
    assert sum("expected change is absent" in line for line in report.problems) == len(CHORD_DOMAINS)


CLIP_DOMAINS = [89, 106, 109, 120, 121]


def test_real_records_the_clip_law_moves_exactly_the_curved_domains_found_by_the_predicate(sweep, ec):
    report = _real_sweep(sweep, ec, "sweep_before_clip.json", "sweep_product.json", "clip_by_faces")
    assert report.problems == []
    assert report.changed_domains == [str(item) for item in CLIP_DOMAINS]
    assert ec.verdict_line(report) == "EXPECTED-CHANGE (spec clip_by_faces): 5 domains"
    flat = [patch for patch, row in _fixture("sweep_product.json")["runs"]["1"]["domains"].items() if row["planarity"] == EXACT_PLANE]
    assert flat and not set(flat) & set(report.changed_domains)


def test_real_records_the_clip_spec_without_the_clip_counter_prefix_names_the_unlisted_kernel_counters(sweep, ec):
    raw = _stored_raw("clip_by_faces")
    raw["allow"]["counter_prefixes"] = ["MATERIALIZE_SURFACE_LIFT_"]
    report = _real_sweep(sweep, ec, "sweep_before_clip.json", "sweep_product.json", None, raw)
    assert report.exit_code == 1
    assert any("counter MATERIALIZE_CLIP_PREDICATES" in line for line in report.problems)
    assert not any("counter MATERIALIZE_SURFACE_LIFT_" in line for line in report.problems)


def test_real_records_the_clip_law_is_not_a_chord_station_change(sweep, ec):
    report = _real_sweep(sweep, ec, "sweep_before_clip.json", "sweep_product.json", "chord_station")
    assert report.exit_code == 1


JOIN_DOMAINS = [0, 1, 2, 3, 6, 7, 15, 92, 106, 109]


def test_real_records_the_join_law_moves_exactly_the_domains_the_predicate_names(sweep, ec):
    report = _real_sweep(sweep, ec, "sweep_before_join.json", "sweep_after_join.json", "join_same_pchain")
    assert report.problems == []
    assert report.changed_domains == [str(item) for item in sorted(JOIN_DOMAINS)]
    assert report.declared_rows == {"flow_domains_planar": 8, "flow_domains_curved": 2}
    assert ec.verdict_line(report) == "EXPECTED-CHANGE (spec join_same_pchain): 10 domains"


def test_real_records_the_join_spec_with_a_narrower_allowance_names_the_counter_it_cannot_cover(sweep, ec):
    raw = _stored_raw("join_same_pchain")
    for group in raw["groups"]:
        group["allow"]["counters"] = [name for name in group["allow"]["counters"] if name != "MATERIALIZE_REGIONS"]
    report = _real_sweep(sweep, ec, "sweep_before_join.json", "sweep_after_join.json", None, raw)
    assert report.exit_code == 1
    assert all("counter MATERIALIZE_REGIONS" in line for line in report.problems)
    assert len(report.problems) == len(JOIN_DOMAINS)


def test_real_records_a_join_spec_whose_predicate_misses_a_changed_domain_fails_on_it(sweep, ec):
    raw = _stored_raw("join_same_pchain")
    for group in raw["groups"]:
        group["domains"]["where"][0]["value"] = 5
    report = _real_sweep(sweep, ec, "sweep_before_join.json", "sweep_after_join.json", None, raw)
    assert report.exit_code == 1
    assert any("domain is not declared" in line for line in report.problems)


# --------------------------------------------------------------------------
# numeric_repr gate: тот же механизм, ответ ворот
# --------------------------------------------------------------------------

FAN_D1 = [0, 1, 4, 6, 7, 10, 11, 15, 17, 105, 106, 109, 110, 114, 115]
FAN_D2 = [6, 10, 11, 15, 105, 106, 109, 110]


def _gate_result(gate, base: dict, new: dict, spec: str | dict | None, partial: bool = False):
    if isinstance(spec, dict):
        parsed = gate.expected_change.parse_spec(spec, "gate", gate.VOCABULARY)
    else:
        parsed = gate.load_spec(spec)
    return gate.compare_records(base, new, parsed, partial)


def test_gate_real_records_the_fan_law_moves_exactly_the_documented_domains_per_density(gate):
    result = _gate_result(gate, _fixture("gate_fans_before.json"), _fixture("gate_fans_after.json"), "right_angle_stable", partial=True)
    report = result["report"]
    assert report.problems == []
    assert result["verdict"] == "EXPECTED-CHANGE (spec right_angle_stable): 15 domains"
    per_density = {}
    for density, patch in report.changed_rows:
        per_density.setdefault(density, []).append(int(patch))
    assert {density: sorted(items) for density, items in per_density.items()} == {"1": FAN_D1, "2": FAN_D2}


def test_gate_real_records_the_declared_densities_the_run_does_not_hold_are_problems_unless_partial(gate):
    result = _gate_result(gate, _fixture("gate_fans_before.json"), _fixture("gate_fans_after.json"), "right_angle_stable")
    lines = [line for line in result["report"].problems if "declares density" in line]
    assert len(lines) == 2 and "density 3" in lines[0] and "density 4" in lines[1]


def test_gate_real_records_without_a_spec_the_fan_law_is_unexpected(gate):
    result = _gate_result(gate, _fixture("gate_fans_before.json"), _fixture("gate_fans_after.json"), None)
    assert result["verdict"].startswith("UNEXPECTED: ")
    assert result["answer_diffs"]


def test_gate_real_records_a_fan_spec_that_does_not_allow_a_deep_digest_fails_on_it(gate, ec):
    raw = json.loads((SWEEP_DIR / "specs" / "right_angle_stable.json").read_text(encoding="utf-8"))
    raw["allow"]["digests"] = [name for name in raw["allow"]["digests"] if not name.startswith("fp_deep_skeleton")]
    result = _gate_result(gate, _fixture("gate_fans_before.json"), _fixture("gate_fans_after.json"), raw, partial=True)
    assert result["report"].exit_code == 1
    assert any("field fp_deep_skeleton" in line for line in result["report"].problems)
    assert not any("field fp_geometry" in line for line in result["report"].problems)


def test_gate_real_records_a_fan_spec_declaring_a_domain_the_law_did_not_move_fails_on_it(gate):
    raw = json.loads((SWEEP_DIR / "specs" / "right_angle_stable.json").read_text(encoding="utf-8"))
    raw["domains"]["by_density"]["2"] = [*FAN_D2, 5]
    result = _gate_result(gate, _fixture("gate_fans_before.json"), _fixture("gate_fans_after.json"), raw, partial=True)
    assert any("d2 patch5: expected change is absent" in line for line in result["report"].problems)


def test_gate_real_records_identical_copies_are_identical(gate):
    record = _fixture("gate_fans_before.json")
    result = _gate_result(gate, record, copy.deepcopy(record), None)
    assert result["verdict"] == "IDENTICAL"
    assert result["report"].exit_code == 0


def test_gate_price_is_listed_and_never_judged_by_a_spec(gate):
    record = _fixture("gate_fans_before.json")
    twin = copy.deepcopy(record)
    for run in twin["runs"].values():
        for domain in run["domains"].values():
            domain["price"]["seconds"] = domain["price"]["seconds"] * 7 + 1
    result = _gate_result(gate, record, twin, None)
    assert result["verdict"] == "IDENTICAL"
    assert "d1:seconds" in result["price_changes"]


STORED_BASELINES = (NUMERIC_REPR / "baseline_18d7197.json", NUMERIC_REPR / "baseline_e548fb5.json")


def _baseline_pair() -> tuple:
    return tuple(json.loads(path.read_text(encoding="utf-8")) for path in STORED_BASELINES)


def _one_domain_spec(allow: dict, patch: int = 17, density: str = "2") -> dict:
    return {
        "schema": "expected_change_spec_v1", "name": "baseline_pair", "tool": "gate", "law": "STORED_BASELINES",
        "about": "two stored numeric_repr baselines whose answer legitimately differs on one domain",
        "domains": {"by_density": {density: [patch]}},
        "allow": allow,
    }


def test_gate_stored_baselines_differ_on_exactly_one_domain_in_two_fields(gate):
    base, new = _baseline_pair()
    plain = gate.compare_records(base, new)
    assert plain["verdict"].startswith("UNEXPECTED: ")
    assert [(item["density"], item["patch_id"], item["keys"]) for item in plain["answer_diffs"]] == [("2", 17, ["detail", "fp_meta"])]


def test_gate_stored_baselines_pass_a_spec_that_names_both_fields_and_fail_one_that_names_one(gate):
    base, new = _baseline_pair()
    both = _gate_result(gate, base, new, _one_domain_spec({"fields": ["detail"], "digests": ["fp_meta"]}), partial=True)
    assert both["verdict"] == "EXPECTED-CHANGE (spec baseline_pair): 1 domains"
    only_digest = _gate_result(gate, base, new, _one_domain_spec({"digests": ["fp_meta"]}), partial=True)
    assert only_digest["report"].exit_code == 1
    assert any("field detail" in line for line in only_digest["report"].problems)
    only_field = _gate_result(gate, base, new, _one_domain_spec({"fields": ["detail"]}), partial=True)
    assert any("field fp_meta" in line for line in only_field["report"].problems)
    wrong_domain = _gate_result(gate, base, new, _one_domain_spec({"fields": ["detail"], "digests": ["fp_meta"]}, patch=18), partial=True)
    assert any("patch17" in line and "domain is not declared" in line for line in wrong_domain["report"].problems)
    assert any("patch18" in line and "expected change is absent" in line for line in wrong_domain["report"].problems)


# --------------------------------------------------------------------------
# Командные строки и вердикт последней строкой
# --------------------------------------------------------------------------


def _write(tmp_path: Path, name: str, record: dict) -> str:
    path = tmp_path / name
    path.write_text(json.dumps(record), encoding="utf-8")
    return str(path)


def _last_line(capsys) -> str:
    return capsys.readouterr().out.strip().splitlines()[-1]


def test_sweep_compare_prints_one_of_three_verdicts_last(sweep, tmp_path, capsys, monkeypatch):
    base = _write(tmp_path, "base.json", _fixture("sweep_chord_off.json"))
    new = _write(tmp_path, "new.json", _fixture("sweep_product.json"))

    def run(*arguments: str) -> int:
        monkeypatch.setattr(sys, "argv", ["sweep.py", "compare", *arguments])
        return sweep.main()

    assert run(base, base) == 0
    assert _last_line(capsys) == "IDENTICAL"
    assert run(base, new, "--spec", "chord_station") == 0
    assert _last_line(capsys) == "EXPECTED-CHANGE (spec chord_station): 14 domains"
    assert run(base, new) == 1
    assert _last_line(capsys).startswith("UNEXPECTED: ")
    assert run(base, new, "--spec", "clip_by_faces") == 1
    assert _last_line(capsys).startswith("UNEXPECTED (spec clip_by_faces): ")


def test_sweep_compare_lists_the_stored_specs_and_refuses_a_missing_or_foreign_spec(sweep, tmp_path, capsys, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["sweep.py", "compare", "--list-specs"])
    assert sweep.main() == 0
    listing = capsys.readouterr().out
    assert "chord_station [SOURCE_VERTEX_STATIONED_ON_CHORD_V1]" in listing and "right_angle_stable" not in listing
    base = _write(tmp_path, "base.json", _fixture("sweep_chord_off.json"))
    for spec in ("no_such_law", "right_angle_stable"):
        monkeypatch.setattr(sys, "argv", ["sweep.py", "compare", base, base, "--spec", spec])
        with pytest.raises(SystemExit) as stopped:
            sweep.main()
        assert stopped.value.code == 2


def test_the_old_hard_coded_flags_are_gone(sweep, tmp_path, monkeypatch):
    base = _write(tmp_path, "base.json", _fixture("sweep_chord_off.json"))
    for flag in (["--expect-changed", "0,1"], ["--ignore-counters", "MATERIALIZE_CHORD_STATIONS_"]):
        monkeypatch.setattr(sys, "argv", ["sweep.py", "compare", base, base, *flag])
        with pytest.raises(SystemExit) as stopped:
            sweep.main()
        assert stopped.value.code == 2
    assert not hasattr(sweep, "CHANGED_COUNTERS") and not hasattr(sweep, "IGNORABLE_COUNTER_PREFIXES")


def test_gate_compare_prints_one_of_three_verdicts_last(gate, tmp_path, capsys, monkeypatch):
    base = _write(tmp_path, "base.json", _fixture("gate_fans_before.json"))
    new = _write(tmp_path, "new.json", _fixture("gate_fans_after.json"))

    def run(*arguments: str) -> int:
        monkeypatch.setattr(sys, "argv", ["gate.py", "compare", *arguments])
        return gate.main()

    assert run(base, base) == 0
    assert _last_line(capsys) == "IDENTICAL"
    assert run(base, new, "--spec", "right_angle_stable", "--partial") == 0
    assert _last_line(capsys) == "EXPECTED-CHANGE (spec right_angle_stable): 15 domains"
    assert run(base, new) == 1
    assert _last_line(capsys).startswith("UNEXPECTED: ")
    assert run(base, new, "--spec", "right_angle_stable") == 1
    assert _last_line(capsys).startswith("UNEXPECTED (spec right_angle_stable): ")
    monkeypatch.setattr(sys, "argv", ["gate.py", "compare", "--list-specs"])
    assert gate.main() == 0
    assert "right_angle_stable" in capsys.readouterr().out


def test_the_gate_selftest_controls_still_see_a_changed_answer_and_ignore_price(gate, tmp_path, capsys):
    record = _fixture("gate_fans_before.json")
    path = Path(_write(tmp_path, "baseline.json", record))
    assert gate.selftest(path, live=False, live_patch=None) == 0
    assert "SELFTEST PASSED" in capsys.readouterr().out


# --------------------------------------------------------------------------
# clip_gate: те же слова, тот же вердикт
# --------------------------------------------------------------------------


def test_clip_gate_default_specs_are_stored_sweep_specs_for_each_law(sweep, ec):
    clip_gate = _load("clip_gate_under_test", SWEEP_DIR / "clip_gate.py")
    assert clip_gate.DEFAULT_SPECS == {
        "SOURCE_FACES_CLIPPED_V1": "clip_by_faces",
        "SOURCE_TRIANGLES_CLIPPED_V1": "clip_by_triangles",
    }
    for law, name in clip_gate.DEFAULT_SPECS.items():
        assert ec.load_spec(name, "sweep", sweep.VOCABULARY).law == law


def test_clip_gate_answer_rows_are_read_by_the_sweep_views_without_the_work_price(sweep, ec):
    clip_gate = _load("clip_gate_under_test_rows", SWEEP_DIR / "clip_gate.py")

    def result(counters, faces_digest):
        return SimpleNamespace(
            outcome=SimpleNamespace(value="MATERIALIZED"),
            detail="ok",
            content_digest=faces_digest,
            offset_normals_digest="",
            batch=SimpleNamespace(semantic_digest=SimpleNamespace(value="sem")),
            counters=tuple(counters.items()),
            diagnostics=("SOURCE_EDGES_LIFTED_ONTO_SURFACE: clip_vertices=1",),
        )

    base = clip_gate._answer_row(result({"MATERIALIZE_FACES_IN": 3, "EXACT_WORK_SPENT": 10}, "c1"), "NearPlanarProjectionCertificateV1")
    cut = clip_gate._answer_row(result({"MATERIALIZE_FACES_IN": 3, "EXACT_WORK_SPENT": 99, "MATERIALIZE_CLIP_FACES_CUT": 2}, "c2"), "NearPlanarProjectionCertificateV1")
    assert "EXACT_WORK_SPENT" not in base["counters"]
    views = sweep.pair_views(False)(base, cut)
    changes = {(change.kind, change.name) for change in ec.diff_views(*views)}
    assert changes == {("field", "content_digest"), ("counter", "MATERIALIZE_CLIP_FACES_CUT")}
    spec = ec.load_spec("clip_by_faces", "sweep", sweep.VOCABULARY)
    assert spec.group_of("1", "5", *views) is not None
    planar = clip_gate._answer_row(result({}, "c"), EXACT_PLANE)
    assert spec.group_of("1", "5", *sweep.pair_views(False)(planar, planar)) is None
