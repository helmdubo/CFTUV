"""Нативное символьное замыкание пакета (WP-S5) равно эталону на Python (`native/cftuv-skeleton`: `overlay`, `component`, `contacts`, `generations`, `coordinator`, `omap`, `pyval`).

Эталон — `kernel/src/cftuv_envelope/wavefront`: `symbolic_superlevel_coordinator.py` (внешний фиксированный пункт, двойной прогон `plan_mixed_generations`), `symbolic_overlay.py`,
`symbolic_f0_overlay.py`, `symbolic_sparse_ports.py`, `symbolic_component.py` (дельты, подпись), `superlevel_fixed_point.py` (контакты, компиляция), `symbolic_edge_closure.py`,
`symbolic_split_endpoint.py`, `symbolic_junction_contacts.py`, `symbolic_junction_normalize.py`, `symbolic_mixed_generation.py`, `symbolic_junction_fixed_point.py`,
`symbolic_edge_fixed_point.py` (`_valid_contact`). Фиксацию в строителя (`symbolic_runtime_commit.py`, S6) этот модуль не проверяет.

Метод тот же, что у `test_native_skeleton_builder.py` (`tools/native_closure_gate.py`): состояние ДО вызова копируется, эталон исполняется на живом, нативный шов — на копии, сравнение ТОЧНОЕ:
КАНОНИЧЕСКИЙ ТЕКСТ результата (`repr` с множествами и `spans` наложения в порядке `repr` их членов: эталон печатает их в порядке хэш-таблицы), исключение `(класс, текст)`,
сдвиг `SIGN_COUNTS`, шесть статей бюджета, журнал памяти канонизации против живых таблиц ПОСЛЕ, рост памяти мест. Проходов три (вложенный вызов скрыт обёрнутым): `closure` (целое замыкание
на состоянии, которое эталон имеет в вызове изнутри настоящей транзакции), `parts` (`with_line_ports`, `build_f0_overlay`, `initial_interior_contacts`, `build_symbolic_overlay`,
`discover_interior_split_contacts`, `plan_mixed_generations`) и `inner` (`discover_junction_contacts`, `apply_component_deltas`, `overlay_signature`).

Естественные прогоны достигают поколений замыкания редко (в полевых и синтетических полигонах `outer_iterations` и число поколений почти всегда нули), поэтому четвёртый проход, `fabricated`,
даёт эталону и шву контакты и наложения, которых прогон не делает (`tools/native_closure_fabricate.py`): разрез листа несколькими внутренними контактами, пары по лучам, устаревшие и
пересекающиеся контакты, испорченные наложения, скрипт обходов `plan_mixed_generations` (цепочка поколений, повтор, исчерпание бюджета). Отказ эталона на испорченном входе, который не
контракт (`KeyError`), считается мусором и не сверяется.

Модуль пропускается с названной причиной, пока расширение не собрано (`python tools/native_build.py`) либо дерево ядра ушло от закреплённых файлов листа.
"""

from __future__ import annotations

import contextlib
import random
import sys
from collections import Counter
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools", ROOT / "tests"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка символьного замыкания с эталоном пропущена)",
        allow_module_level=True,
    )

from cftuv_native import closure_seams as cseams  # noqa: E402
from cftuv_native import skeleton_seams as wire  # noqa: E402

_STALE = wire.stale_leaf_files()
if _STALE:
    pytest.skip(
        f"файлы эталона, которые зеркалит нативный лист скелета, ушли от закрепления: {', '.join(_STALE)} (перенос дельты и новое закрепление — отдельный шаг)",
        allow_module_level=True,
    )

import native_closure_coverage as coverage_tool  # noqa: E402
import native_closure_fabricate as fabricate  # noqa: E402
import native_closure_gate as gate  # noqa: E402
import native_corpus as nc  # noqa: E402
import native_leaf_gate as leaf  # noqa: E402
import native_motorcycle_gate as motorcycle_gate  # noqa: E402
import test_native_skeleton_builder as builder_tests  # noqa: E402
import test_native_skeleton_motorcycle as motorcycle  # noqa: E402
import test_native_skeleton_parts as parts  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.wavefront.skeleton as skeleton  # noqa: E402
from cftuv_envelope.wavefront.polygon import PolygonV1, with_edge_speeds  # noqa: E402
from cftuv_envelope.wavefront.skeleton import build_skeleton  # noqa: E402

CHECKED: Counter = Counter()
FIELD_POLYGONS = motorcycle_gate.FIELD_POLYGONS
FIELD_FAST = parts.FIELD_FAST
ORACLE_OUTCOMES = builder_tests.ORACLE_OUTCOMES
WHOLE_LINES = {"symbolic_overlay.py", "symbolic_component.py", "symbolic_mixed_generation.py", "symbolic_superlevel_coordinator.py"}

#: the lead's example of a polygon whose weights make the oracle's closure leave the empty-contact paths: the points, and the speeds (`q`) of its edges in order
EXAMPLE_POINTS = ((-6, -1), (-4, -6), (6, -1), (3, 0), (3, 1), (2, 3), (-1, 3), (-5, 5), (-6, 5))
EXAMPLE_SPEEDS = (261, Fraction(125, 4), Fraction(5, 2), Fraction(1, 4), 0, Fraction(9, 4), 5, 4, 9)


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    """Эталон пишет в процессные счётчики и память ядра; тест их не оставляет."""

    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    audit = exact.set_canonical_audit(False)
    with exact.isolated_factorization_memory():
        yield
    exact.set_canonical_audit(audit)
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(nc._ARTICLES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)


COVERAGE: dict = {}


@pytest.fixture(scope="module", autouse=True)
def _watch_the_lines_of_the_symbolic_closure_and_report_the_comparison_count(request):
    """Монитор строк включён на всё время сверок (Python 3.12 и новее): достигнутые строки модулей эталона — часть доказательства, и итоговый тест их проверяет."""

    monitor = None
    if hasattr(sys, "monitoring"):
        try:
            monitor = coverage_tool.LineCoverage().__enter__()
        except ValueError:
            monitor = None
    COVERAGE["monitor"] = monitor
    yield
    if monitor is not None:
        monitor.__exit__(None, None, None)
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line("native symbolic closure compared with the Python oracle: " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items())))
        if monitor is not None:
            reporter.write_line("lines of the mirrored modules reached by the comparisons:\n" + monitor.format())


def everything(**per_seam) -> leaf.Sampling:
    return leaf.Sampling(head=10**9, stride=1, cap=10**9, per_seam=per_seam)


def verifier_of(passes=("closure",), **options) -> gate.ClosureVerifier:
    return gate.ClosureVerifier(everything(), passes=passes, **options)


def settle(verifier, *, internal: int | None = None) -> None:
    """Ничего не разошлось, ничего не отказано портом; учёт сверенных вызовов в итог модуля. `internal`: сколько пар «`TypeError` эталона — отказ порта» ожидается."""

    assert not verifier.mismatches, parts.explain(verifier)
    assert not verifier.unsupported, f"the port refused calls it should carry: {dict(verifier.unsupported)} {verifier.unsupported_detail[:3]}"
    if internal is not None:
        assert sum(verifier.internal_agreed.values()) == internal, dict(verifier.internal_agreed)
    for seam, count in verifier.checked.items():
        CHECKED[seam] += count
    CHECKED["internal errors named by the port"] += sum(verifier.internal_agreed.values())
    CHECKED["garbage inputs of the oracle (not compared)"] += sum(verifier.garbage.values())


def population() -> list:
    return builder_tests.population()


def run_oracle(verifier, polygon, budget, *, level=None, dense=False):
    """`build_skeleton` на живом состоянии с обёртками прохода; отказы эталона — исходы (`TypeError` порт называет отказом)."""

    original = skeleton.level_budget
    if level is not None:
        skeleton.level_budget = lambda _polygon, level=level: level
    try:
        with verifier.installed():
            try:
                return build_skeleton(polygon, work_budget=budget, dense_hydration=dense)
            except ORACLE_OUTCOMES as error:
                return error
    finally:
        skeleton.level_budget = original


def example_polygon(speeds=EXAMPLE_SPEEDS) -> PolygonV1:
    figure = PolygonV1.build(EXAMPLE_POINTS)
    return with_edge_speeds(figure, tuple((start, end, speed) for (start, end, _), speed in zip(figure.edges(), speeds)))


def neighbours_of_the_example(count: int, seed: int = 311) -> list:
    """The example's figure with other weights: a wall, a quarter, one, four, nine times the unit speed of every edge (at least one edge is a source)."""

    from cftuv_envelope.wavefront.event_time import SupportLineV1

    rng = random.Random(seed)
    figure = PolygonV1.build(EXAMPLE_POINTS)
    found = []
    for _ in range(count):
        speeds = []
        for start, end, _ in figure.edges():
            unit = SupportLineV1.through(start, end).q
            speeds.append((start, end, rng.choice((0, unit, unit / 4 if isinstance(unit, Fraction) else Fraction(unit, 4), unit * 4, unit * 9, unit))))
        if any(speed for _, _, speed in speeds):
            found.append(with_edge_speeds(figure, tuple(speeds)))
    return found


# --------------------------------------------------------------------------
# the table, the pins, the canonical text
# --------------------------------------------------------------------------


def test_the_seam_table_of_the_shim_is_the_extensions():
    assert wire.SEAMS == cftuv_native.skeleton_seam_table()
    assert {name for _, name in wire.SEAMS} >= {"PLAN_SYMBOLIC_CLOSURE", "BUILD_SYMBOLIC_OVERLAY", "DISCOVER_INTERIOR_CONTACTS", "PLAN_MIXED_GENERATIONS", "DISCOVER_JUNCTION_CONTACTS", "APPLY_COMPONENT_DELTAS"}


def test_the_files_of_the_symbolic_closure_are_pinned_and_the_pins_are_the_trees():
    for name in coverage_tool.FILES:
        assert f"wavefront/{name}" in wire.LEAF_FILES and f"wavefront/{name}" in wire.LEAF_PINS, name
    assert wire.stale_leaf_files() == ()


def test_the_canonical_text_orders_what_the_oracle_prints_in_hash_order_and_nothing_else():
    first, second = frozenset({(1, None), (2, "b")}), frozenset({(2, "b"), (1, None)})
    assert cseams.canon_text(first) == cseams.canon_text(second) == "frozenset({(1, None), (2, 'b')})"
    assert cseams.canon_text(set()) == "set()" and cseams.canon_text(frozenset()) == "frozenset()"
    assert cseams.canon_text({3: (1,), 1: [2]}) == "{3: (1,), 1: [2]}", "a dictionary keeps its insertion order"
    assert cseams.canon_text(Fraction(1, 2)) == repr(Fraction(1, 2)) and cseams.canon_text(None) == "None"


# --------------------------------------------------------------------------
# the closure of every packet of a real run
# --------------------------------------------------------------------------


def test_the_closure_equals_the_oracle_on_every_named_weighted_generated_and_fan_polygon():
    verifier = verifier_of()
    polygons = population()
    for _name, polygon in polygons:
        run_oracle(verifier, polygon, leaf.fresh_process_state())
    settle(verifier)
    assert verifier.checked["PLAN_SYMBOLIC_CLOSURE"] > 600


def test_the_parts_of_the_closure_equal_the_oracle_on_the_places_the_oracle_calls_them_from():
    verifier = verifier_of(("parts",))
    for _name, polygon in population()[::4]:
        run_oracle(verifier, polygon, leaf.fresh_process_state())
    settle(verifier)
    for seam in ("BUILD_SYMBOLIC_OVERLAY", "DISCOVER_INTERIOR_CONTACTS", "PLAN_MIXED_GENERATIONS", "CLOSURE_PART"):
        assert verifier.checked[seam] > 100, dict(verifier.checked)
    assert {name for name in verifier.part_calls if verifier.part_calls[name]} >= {"with_line_ports", "build_f0_overlay", "initial_interior_contacts"}


def test_the_junction_discoveries_the_deltas_and_the_signature_equal_the_oracle():
    verifier = verifier_of(("inner",))
    for _name, polygon in population()[::4]:
        run_oracle(verifier, polygon, leaf.fresh_process_state())
    settle(verifier)
    for seam in ("DISCOVER_JUNCTION_CONTACTS", "DISCOVER_INTERIOR_CONTACTS", "CLOSURE_PART"):
        assert verifier.checked[seam] > 100, dict(verifier.checked)


def test_the_closure_equals_the_oracle_unbudgeted_and_without_the_memory_of_places():
    verifier = verifier_of()
    for _name, polygon in population()[::5]:
        leaf.fresh_process_state()
        run_oracle(verifier, polygon, None)
        run_oracle(verifier, polygon, leaf.fresh_process_state(), dense=True)
    settle(verifier)
    assert verifier.checked["PLAN_SYMBOLIC_CLOSURE"] > 200


def test_the_closure_runs_out_of_budget_where_the_oracle_does_and_leaves_the_oracles_state():
    """A cap swept under what the run spends: the exhaustion falls in the snapshot (the builder seams), in the F0 overlay, in a rebuild of the splits, in a discovery, in a generation,
    in the replay; every call that ends in it is compared with the cost it left: the articles, the memory of canonicalisation, the signs, the text of the refusal."""

    verifier = verifier_of()
    swept = 0
    for _name, polygon in parts.named_polygons()[8:40]:
        probe = leaf.fresh_process_state()
        build_skeleton(polygon, work_budget=probe)
        for percent in (30, 50, 62, 74, 85, 92, 97):
            leaf.fresh_process_state()
            run_oracle(verifier, polygon, exact.exact_work_budget(stage="PREPARE", domain_id="starved", cap=max(1, probe.spent * percent // 100)))
            swept += 1
    settle(verifier)
    assert swept > 200 and verifier.checked["PLAN_SYMBOLIC_CLOSURE"] > 100
    assert verifier.raised["PLAN_SYMBOLIC_CLOSURE"] >= 10, f"the starved runs rarely ran out inside the closure: {dict(verifier.raised)}"


def test_the_closure_of_the_example_polygon_and_of_polygons_with_its_figure_and_other_weights_equals_the_oracle():
    verifier = verifier_of(("closure", "parts"))
    polygons = [example_polygon(), *neighbours_of_the_example(48)]
    for polygon in polygons:
        run_oracle(verifier, polygon, leaf.fresh_process_state())
    settle(verifier)
    assert verifier.checked["PLAN_SYMBOLIC_CLOSURE"] > 150, dict(verifier.checked)


def test_the_closure_equals_the_oracle_on_the_fast_field_polygons():
    if not FIELD_POLYGONS.exists():
        pytest.skip(f"нет записанных полевых полигонов ({FIELD_POLYGONS}): их кладёт `python tools/native_leaf_gate.py fetch`")
    verifier = verifier_of(("closure", "parts"))
    polygons = leaf.load_polygons(FIELD_POLYGONS, only=FIELD_FAST)
    assert polygons
    for _name, polygon in polygons:
        run_oracle(verifier, polygon, leaf.fresh_process_state())
    settle(verifier)
    CHECKED["field polygons"] += len(polygons)


def test_the_closure_equals_the_oracle_on_the_skeleton_corpus_and_names_the_internal_errors_of_the_oracle():
    pool = builder_tests.corpus_polygons()
    if not pool:
        pytest.skip(f"нет корпуса скелета под ядро {nc.clip_memo.kernel_code_identity()}: {builder_tests.sc.describe_missing('synthetic')}")
    chosen = [item for item in pool if item[3]] + [item for index, item in enumerate(pool) if not item[3] and index % 9 == 0 and sum(len(each.points) for each in item[1].loops) <= 40]
    verifier = verifier_of(("closure", "inner"))
    for _name, polygon, level, _internal, _outcome in chosen:
        run_oracle(verifier, polygon, leaf.fresh_process_state(), level=level)
    settle(verifier)
    assert len(chosen) > 100 and verifier.checked["PLAN_SYMBOLIC_CLOSURE"] > 500


# --------------------------------------------------------------------------
# what no run makes: contacts for the generations, spoiled overlays, scripted rounds
# --------------------------------------------------------------------------


def test_the_generations_equal_the_oracle_on_contacts_that_were_made_for_them():
    """The cut of a leaf by interior contacts (distinct and tied projections, one point twice, a leaf that is gone, an emitter that is not in the overlay), the pairing of dead
    ports (two by the cross, three or more by rays), stale and overlapping contacts; then the births applied as one generation; then the spoiled overlays; then rounds of
    `plan_mixed_generations` whose discoveries answer made-up contacts (a chain of generations, its replay, the budget that runs out)."""

    verifier = verifier_of(("fabricated",), cases=6, spoiled=4, scripted=3)
    for _name, polygon in population()[::3]:
        run_oracle(verifier, polygon, leaf.fresh_process_state())
    settle(verifier)
    assert verifier.checked["NORMALIZE_MIXED_GENERATION"] > 400 and verifier.checked["PLAN_MIXED_GENERATIONS"] > 400, dict(verifier.checked)
    assert verifier.checked["DISCOVER_INTERIOR_CONTACTS"] > 100 and verifier.checked["DISCOVER_JUNCTION_CONTACTS"] > 100


def test_the_generations_equal_the_oracle_on_contacts_made_over_the_overlays_of_the_example_and_its_neighbours():
    verifier = verifier_of(("fabricated",), seed=7, cases=8, spoiled=3, scripted=4)
    for polygon in [example_polygon(), *neighbours_of_the_example(16, seed=5)]:
        run_oracle(verifier, polygon, leaf.fresh_process_state())
    settle(verifier)
    assert verifier.checked["NORMALIZE_MIXED_GENERATION"] > 100, dict(verifier.checked)


# --------------------------------------------------------------------------
# the comparison itself: a wrong text, a wrong cost, a wrong state is reported
# --------------------------------------------------------------------------


def first_polygon_with_a_closure() -> object:
    return dict(parts.named_polygons())["ell"]


def test_the_comparison_reports_a_wrong_closure_text(monkeypatch):
    import dataclasses

    import cftuv_envelope.wavefront.symbolic_superlevel_coordinator as coordinator

    original = coordinator.plan_symbolic_superlevel_closure

    def tampered(builder, snapshot, *, outer_budget, junction_budget):
        found = original(builder, snapshot, outer_budget=outer_budget, junction_budget=junction_budget)
        return dataclasses.replace(found, canonical_batch_count=found.canonical_batch_count + 1)

    verifier = verifier_of()
    monkeypatch.setattr(coordinator, "plan_symbolic_superlevel_closure", tampered)
    run_oracle(verifier, first_polygon_with_a_closure(), leaf.fresh_process_state())
    assert any("PLAN_SYMBOLIC_CLOSURE" in str(item) and "result" in str(item) for item in verifier.mismatches), "a wrong count of batches passed the comparison"


def test_the_comparison_reports_a_sign_the_native_closure_never_asked(monkeypatch):
    import cftuv_envelope.wavefront.symbolic_component as component

    original = component.overlay_signature

    def paying(overlay):
        exact.SIGN_COUNTS["total"] += 1
        return original(overlay)

    monkeypatch.setattr(component, "overlay_signature", paying)
    import cftuv_envelope.wavefront.symbolic_mixed_generation as mixed
    import cftuv_envelope.wavefront.symbolic_superlevel_coordinator as coordinator

    monkeypatch.setattr(mixed, "overlay_signature", paying)
    monkeypatch.setattr(coordinator, "overlay_signature", paying)
    verifier = verifier_of()
    run_oracle(verifier, first_polygon_with_a_closure(), leaf.fresh_process_state())
    assert any("sign_counts" in str(item) for item in verifier.mismatches), "a sign the native closure never asked passed the cost comparison"


def test_the_comparison_reports_a_wrong_overlay_in_a_part(monkeypatch):
    import cftuv_envelope.wavefront.symbolic_overlay as overlay_module
    import cftuv_envelope.wavefront.symbolic_superlevel_coordinator as coordinator

    original = overlay_module.build_symbolic_overlay

    def tampered(builder, snapshot, materialization, *, include_line_ports=False):
        found = original(builder, snapshot, materialization, include_line_ports=include_line_ports)
        if found is not None:
            found.changed.clear()
        return found

    monkeypatch.setattr(overlay_module, "build_symbolic_overlay", tampered)
    monkeypatch.setattr(coordinator, "build_symbolic_overlay", tampered)
    verifier = verifier_of(("parts",))
    run_oracle(verifier, first_polygon_with_a_closure(), leaf.fresh_process_state())
    assert any("BUILD_SYMBOLIC_OVERLAY" in str(item) for item in verifier.mismatches), "an overlay without its changed leaves passed the comparison"


# --------------------------------------------------------------------------
# the lines of the oracle the comparisons reached (the last test of the module)
# --------------------------------------------------------------------------


def test_zz_the_comparisons_reached_the_live_lines_of_the_symbolic_modules():
    monitor = COVERAGE.get("monitor")
    if monitor is None:
        pytest.skip("монитор строк — `sys.monitoring` (Python 3.12 и новее): под 3.11 охват этого модуля не измеряется")
    report = monitor.report()
    reached = sum(found for found, _total, _missed in report.values())
    total = sum(count for _found, count, _missed in report.values())
    assert total > 600
    # the floor of what the comparisons of this module reach; a regression of the corpora or of the fabricator shows here as a drop
    floors = {"symbolic_overlay.py": 0.90, "symbolic_component.py": 0.88, "symbolic_f0_overlay.py": 0.80, "symbolic_sparse_ports.py": 0.85, "symbolic_edge_closure.py": 0.95,
              "symbolic_junction_normalize.py": 0.85, "symbolic_mixed_generation.py": 0.80, "symbolic_superlevel_coordinator.py": 0.80}
    for name, floor in floors.items():
        found, count, missed = report[name]
        assert count and found / count >= floor, f"{name}: {found}/{count} lines reached, floor {floor:.0%}; missed {missed}"
    assert reached / total >= 0.80, f"{reached}/{total} live lines reached"
