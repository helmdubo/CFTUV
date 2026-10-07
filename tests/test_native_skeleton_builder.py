"""Нативные `_Builder` (WP-S3) и слой снимка и планов (WP-S4) равны эталону на Python (`native/cftuv-skeleton`: `builder`, `skeleton`, `superlevel`, `snapshot`, `plans`,
`germ`, `closure`, `composition`, `pyval`).

Эталон — `kernel/src/cftuv_envelope/wavefront`: `skeleton.py` (`_Builder`: `__init__` с `_seed`, цикл `run`, `_finish`, примитивы фронта: `_twin`, `_new_vertex`, `_emit`,
`_enqueue_*`, `_front_vertex_met_by`, живость кандидата), `superlevel.py` (голова `apply_superlevel_transaction`, записи отказов, эмиссия узлов, `accumulate_nodes`,
`plan_superlevel_components`), `superlevel_snapshot.py`, `superlevel_germ.py`, `superlevel_closure.py` (`plan_split_materialization`), `symbolic_initial_composition.py`,
`digest.py` (счётчики дубликатов). Сама ТРАНЗАКЦИЯ (символьное замыкание S5 и фиксация S6) — граница `builder::Transaction`: здесь она не запускается нативно.

Метод тот же, что у `test_native_skeleton_motorcycle.py` (`tools/native_builder_gate.py`): состояние ДО вызова копируется, эталон исполняется на живом, нативный шов — на копии,
сравнение ТОЧНОЕ (состояние строителя целиком: рёбра и их прямые, вершины, куча очереди МАССИВОМ с порядковыми номерами, трассы, граф, индекс, счётчики; текст `repr` снимка
и планов; исключение `(класс, текст)`; сдвиг `SIGN_COUNTS`; шесть статей бюджета; журнал памяти канонизации против живых таблиц ПОСЛЕ):

* `__init__` строителя целиком (`BUILDER_INIT`);
* ЦИКЛ по шагам (`BUILDER_RUN`): нативный цикл стартует с состояния, которое оставила ПРЕДЫДУЩАЯ транзакция эталона, и обязан встать у следующего вызова транзакции в том же
  состоянии (либо закончить тем же `SkeletonV1`), с той же ценой шагов между ними (извлечение уровня, обход `_count_at_time`, остаток того же времени, закрытие коротких LAV, `_finish`);
* `collect_superlevel_snapshot`, `plan_split_materialization` (каждый вызов символьного замыкания на тех снимках, что оно делает) и `plan_superlevel_components`;
* примитивы строителя и помощники `superlevel` (`BUILDER_PRIMITIVE`), каждый на состоянии, которое эталон имеет в вызове изнутри настоящей транзакции;
* голова `apply_superlevel_transaction`: где эталон возвращается до символьного слоя, нативная голова делает то же.

Источники вызовов: настоящий `build_skeleton` на именованных, взвешенных, сгенерированных и веерных полигонах, полевых полигонах и на корпусе скелета (синтетика тестов ядра,
сгенерированные полигоны, поле), все бюджеты. Эталон падает `TypeError` на части синтетики (сортировка рождений `_meeting_plans` сравнивает `None` с ключом); порт отвечает на это
ИМЕНОВАННЫМ отказом до состояния (`Unsupported`), и пара «`TypeError` эталона — отказ порта» считается совпадением.

Модуль пропускается с названной причиной, пока расширение не собрано (`python tools/native_build.py`) либо дерево ядра ушло от закреплённых файлов листа.
"""

from __future__ import annotations

import contextlib
import sys
from collections import Counter
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
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка строителя, снимка и планов с эталоном пропущена)",
        allow_module_level=True,
    )

from cftuv_native import builder_seams as bseams  # noqa: E402
from cftuv_native import skeleton_seams as wire  # noqa: E402

_STALE = wire.stale_leaf_files()
if _STALE:
    pytest.skip(
        f"файлы эталона, которые зеркалит нативный лист скелета, ушли от закрепления: {', '.join(_STALE)} (перенос дельты и новое закрепление — отдельный шаг)",
        allow_module_level=True,
    )

import native_builder_gate as gate  # noqa: E402
import native_corpus as nc  # noqa: E402
import native_leaf_gate as leaf  # noqa: E402
import native_motorcycle_gate as motorcycle_gate  # noqa: E402
import native_skeleton_corpus as sc  # noqa: E402
import test_native_skeleton_motorcycle as motorcycle  # noqa: E402
import test_native_skeleton_parts as parts  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.wavefront.skeleton as skeleton  # noqa: E402
from cftuv_envelope.wavefront.skeleton import build_skeleton  # noqa: E402

CHECKED: Counter = Counter()
FIELD_POLYGONS = motorcycle_gate.FIELD_POLYGONS
#: the field polygons that the real `build_skeleton` runs through here in seconds (the heavy ones are the gate's: `tools/native_builder_gate.py gate`)
FIELD_FAST = parts.FIELD_FAST
HEAVY_PRIMITIVES = {name: (12, 40, 10**9) for name in ("_position", "_edge_event_is_live", "_split_is_live", "_front_vertex_met_by")}


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


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line("native builder, snapshot and plans compared with the Python oracle: " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items())))


def everything(**per_seam) -> leaf.Sampling:
    return leaf.Sampling(head=10**9, stride=1, cap=10**9, per_seam=per_seam)


def verifier_of(**options) -> gate.BuilderVerifier:
    return gate.BuilderVerifier(everything(), **options)


def settle(verifier, *, internal: int | None = None) -> None:
    """Ничего не разошлось, ничего не отказано портом; учёт сверенных вызовов в итог модуля. `internal`: сколько пар «`TypeError` эталона — отказ порта» ожидается."""

    assert not verifier.mismatches, parts.explain(verifier)
    assert not verifier.unsupported, f"the port refused calls it should carry: {dict(verifier.unsupported)} {verifier.unsupported_detail[:3]}"
    if internal is not None:
        assert sum(verifier.internal_agreed.values()) == internal, dict(verifier.internal_agreed)
    for seam, count in verifier.checked.items():
        CHECKED[seam] += count
    for operation, count in verifier.prims.items():
        CHECKED[f"primitive {operation}"] += count
    CHECKED["internal errors named by the port"] += sum(verifier.internal_agreed.values())


def population() -> list:
    """The named, the partial-source, the weighted, the generated and the fan polygons of the kernel's own cases."""

    return parts.named_polygons() + motorcycle.generated_polygons()


ORACLE_OUTCOMES = (*motorcycle.ORACLE_OUTCOMES, TypeError)


def run_oracle(verifier, polygon, budget, *, level=None, dense=False, with_loop=True, with_primitives=False):
    """`build_skeleton` on the live state with the wrappers of the verifier; the oracle's own refusals are outcomes (a `TypeError` is the one the port names)."""

    original = skeleton.level_budget
    if level is not None:
        skeleton.level_budget = lambda _polygon, level=level: level
    try:
        with contextlib.ExitStack() as stack:
            if with_loop:
                stack.enter_context(verifier.stepping())
            stack.enter_context(verifier.primitives_installed() if with_primitives else verifier.installed())
            try:
                return build_skeleton(polygon, work_budget=budget, dense_hydration=dense)
            except ORACLE_OUTCOMES as error:
                return error
    finally:
        skeleton.level_budget = original


# --------------------------------------------------------------------------
# the table, the pins
# --------------------------------------------------------------------------


def test_the_seam_table_of_the_shim_is_the_extensions():
    assert wire.SEAMS == cftuv_native.skeleton_seam_table()
    assert {name for _, name in wire.SEAMS} >= {"BUILDER_INIT", "BUILDER_RUN", "COLLECT_SNAPSHOT", "PLAN_COMPONENTS", "PLAN_SPLIT_MATERIALIZATION", "BUILDER_PRIMITIVE"}


def test_the_files_of_the_builder_and_the_plans_are_pinned_and_the_pins_are_the_trees():
    for name in ("skeleton", "superlevel", "superlevel_snapshot", "superlevel_germ", "superlevel_closure", "symbolic_initial_composition", "exact_identity", "digest"):
        assert f"wavefront/{name}.py" in wire.LEAF_FILES and f"wavefront/{name}.py" in wire.LEAF_PINS
    assert wire.stale_leaf_files() == ()


def test_a_polygon_the_port_cannot_carry_is_refused_by_the_shim_not_computed():
    from cftuv_envelope.wavefront.polygon import PolygonV1

    polygon = PolygonV1.build(((0, 0), (1 << 60, 0), (1 << 60, 8), (0, 8)))
    with pytest.raises(wire.SeamUnsupported):
        wire.enc_polygon(polygon)


# --------------------------------------------------------------------------
# the init
# --------------------------------------------------------------------------


def init_pass(verifier, polygon, budget, **options):
    try:
        return verifier.check_init(polygon, budget, **options)
    except ORACLE_OUTCOMES as error:
        return error


def test_the_builder_after_init_equals_the_oracle_on_every_named_weighted_generated_and_fan_polygon():
    verifier = verifier_of()
    polygons = population()
    for _name, polygon in polygons:
        init_pass(verifier, polygon, leaf.fresh_process_state())
    settle(verifier)
    assert verifier.checked["BUILDER_INIT"] == len(polygons)


@pytest.mark.parametrize("kind", ("unbudgeted", "dense"))
def test_the_builder_after_init_equals_the_oracle_unbudgeted_and_without_the_memory_of_places(kind):
    verifier = verifier_of()
    for _name, polygon in population()[::2]:
        leaf.fresh_process_state()
        if kind == "unbudgeted":
            init_pass(verifier, polygon, None)
        else:
            init_pass(verifier, polygon, leaf.fresh_process_state(), dense=True)
    settle(verifier)
    assert verifier.checked["BUILDER_INIT"] > 60


def test_the_builder_after_init_equals_the_oracle_on_translated_and_scaled_polygons():
    """Negative and far origins, offsets of 90 bits, cells of other sizes."""

    verifier = verifier_of()
    chosen = [(name, polygon) for name, polygon in parts.named_polygons() if name in ("ell", "comb_4", "hole_1", "holes_2", "cross", "double_notch", "staircase", "star_9_seed_4")]
    offsets = ((-17, -5, 1), (-10**6, 3 * 10**9, 1), (2**40, -(2**41), 2), (-(2**44), 2**44 + 7, 3), (0, -1, 8))
    for _name, polygon in chosen:
        for dx, dy, factor in offsets:
            init_pass(verifier, motorcycle.moved(polygon, dx, dy, factor), leaf.fresh_process_state())
    settle(verifier)
    assert verifier.checked["BUILDER_INIT"] == len(chosen) * len(offsets)


def test_the_builder_init_runs_out_of_budget_at_the_operation_the_oracle_does():
    """A cap under what the init spends, swept: the exhaustion falls in the factorization of the speeds, a radical of a velocity, a hydration, a sign of the queue."""

    verifier = verifier_of()
    swept = 0
    for _name, polygon in parts.named_polygons()[8:] + motorcycle.generated_polygons()[:14]:
        probe = leaf.fresh_process_state()
        skeleton._Builder(polygon, skeleton.SplitSearch.MOTORCYCLE, work_budget=probe)
        spent = probe.spent
        for numerator in (1, 3, 5, 7, 9):
            leaf.fresh_process_state()
            budget = exact.exact_work_budget(stage="PREPARE", domain_id="starved", cap=max(1, spent * numerator // 10))
            init_pass(verifier, polygon, budget)
            swept += 1
    settle(verifier)
    assert verifier.checked["BUILDER_INIT"] == swept
    assert verifier.raised["BUILDER_INIT"] >= 40, f"the starved inits rarely ran out inside the builder: {dict(verifier.raised)}"


def test_the_builder_after_init_equals_the_oracle_on_the_field_polygons_when_they_are_in_the_corpus():
    if not FIELD_POLYGONS.exists():
        pytest.skip(f"нет записанных полевых полигонов ({FIELD_POLYGONS}): их кладёт `python tools/native_leaf_gate.py fetch`")
    verifier = verifier_of()
    polygons = leaf.load_polygons(FIELD_POLYGONS)
    for _name, polygon in polygons:
        init_pass(verifier, polygon, leaf.fresh_process_state())
    settle(verifier)
    assert verifier.checked["BUILDER_INIT"] == len(polygons)


# --------------------------------------------------------------------------
# the loop, the snapshot and the plans, on a real run
# --------------------------------------------------------------------------


def test_the_loop_the_snapshots_and_the_plans_equal_the_oracle_on_every_named_weighted_generated_and_fan_polygon():
    verifier = verifier_of()
    polygons = population()
    for _name, polygon in polygons:
        run_oracle(verifier, polygon, leaf.fresh_process_state())
    settle(verifier)
    assert verifier.steps["finished"] + verifier.steps["raised"] == len(polygons)
    assert verifier.steps["stopped"] > 1000 and verifier.checked["COLLECT_SNAPSHOT"] > 1000 and verifier.checked["PLAN_SPLIT_MATERIALIZATION"] > 400


def test_the_plans_of_the_components_alone_equal_the_oracle():
    verifier = verifier_of(plans="components")
    polygons = population()[::3]
    for _name, polygon in polygons:
        run_oracle(verifier, polygon, leaf.fresh_process_state(), with_loop=False)
    settle(verifier)
    assert verifier.checked["PLAN_COMPONENTS"] > 100


def test_the_loop_equals_the_oracle_unbudgeted_and_without_the_memory_of_places():
    verifier = verifier_of()
    polygons = population()[::4]
    for _name, polygon in polygons:
        leaf.fresh_process_state()
        run_oracle(verifier, polygon, None)
        run_oracle(verifier, polygon, leaf.fresh_process_state(), dense=True)
    settle(verifier)
    assert verifier.steps["stopped"] > 200


def test_the_loop_runs_out_of_budget_where_the_oracle_does_and_leaves_the_oracles_state():
    """A cap swept under what the run spends: the exhaustion falls in the init (the init seam), in a hydration of the snapshot, in the plans, in the commit (the oracle's own:
    the port is not asked); every call that ends in it is compared with the cost it left, the articles and the memory of canonicalisation."""

    verifier = verifier_of()
    swept = 0
    for _name, polygon in parts.named_polygons()[8:30]:
        probe = leaf.fresh_process_state()
        build_skeleton(polygon, work_budget=probe)
        for percent in (30, 50, 70, 85, 92, 97):
            leaf.fresh_process_state()
            budget = exact.exact_work_budget(stage="PREPARE", domain_id="starved", cap=max(1, probe.spent * percent // 100))
            run_oracle(verifier, polygon, budget)
            swept += 1
    settle(verifier)
    assert swept > 100 and verifier.checked["BUILDER_RUN"] > 60
    assert verifier.raised["COLLECT_SNAPSHOT"] >= 20, f"the starved runs rarely ran out inside a snapshot: {dict(verifier.raised)}"


def test_a_level_budget_replaced_by_a_test_is_the_oracles_named_outcome():
    """The kernel tests replace `skeleton.level_budget` to force `LEVEL_BUDGET_EXHAUSTED`: the host reads the live function and hands the number to the native loop."""

    verifier = verifier_of()
    for name, polygon in parts.named_polygons():
        if name not in ("ell", "comb_4", "cross", "staircase"):
            continue
        for level in (1, 2, 3):
            result = run_oracle(verifier, polygon, leaf.fresh_process_state(), level=level)
            assert not isinstance(result, Exception) and result.outcome.value == "LEVEL_BUDGET_EXHAUSTED"
    settle(verifier)


def test_the_loop_and_the_plans_equal_the_oracle_on_the_fast_field_polygons():
    if not FIELD_POLYGONS.exists():
        pytest.skip(f"нет записанных полевых полигонов ({FIELD_POLYGONS}): их кладёт `python tools/native_leaf_gate.py fetch`")
    verifier = verifier_of()
    polygons = leaf.load_polygons(FIELD_POLYGONS, only=FIELD_FAST)
    assert polygons
    for _name, polygon in polygons:
        run_oracle(verifier, polygon, leaf.fresh_process_state())
    settle(verifier)
    CHECKED["field polygons"] += len(polygons)


# --------------------------------------------------------------------------
# the primitives of the builder and the head of the transaction
# --------------------------------------------------------------------------


def test_the_primitives_of_the_builder_and_the_head_of_the_transaction_equal_the_oracle_inside_real_transactions():
    verifier = gate.BuilderVerifier(everything(**HEAVY_PRIMITIVES))
    polygons = population()[::3]
    for _name, polygon in polygons:
        run_oracle(verifier, polygon, leaf.fresh_process_state(), with_loop=False, with_primitives=True)
    settle(verifier)
    for operation in ("_twin", "_new_vertex", "_enqueue_for", "_enqueue_edge_event", "_enqueue_splits_against", "_register", "_emit_component_nodes", "_position", "transaction head: done", "transaction head: into the closure"):
        assert verifier.prims[operation] > 0, f"no call of {operation} was checked: {dict(verifier.prims)}"


def test_the_records_of_the_refusals_and_the_emission_of_contacts_equal_the_oracle_on_constructed_packets():
    """The refusals the real runs reach rarely or never (an event of a kind the transaction does not carry, two live owners of one edge, an unproven span, a named
    reason of the closure) and the emission of a contact that absorbed a split, on the front of a real builder."""

    import dataclasses

    import cftuv_envelope.wavefront.superlevel as superlevel
    from cftuv_envelope.wavefront.candidate_refusal import CandidateRefusal
    from cftuv_envelope.wavefront.events import CandidateEventV1, EventKind

    verifier = gate.BuilderVerifier(everything())
    for name in ("ell", "comb_4", "cross"):
        polygon = dict(parts.named_polygons())[name]
        budget = leaf.fresh_process_state()
        builder = skeleton._Builder(polygon, skeleton.SplitSearch.MOTORCYCLE, work_budget=budget)
        level = builder.queue.pop_level()
        builder.now = level[0].time
        edge = CandidateEventV1(EventKind.EDGE, level[0].time, level[0].point, 0, builder.vertices[0].next, -1)
        with verifier.primitives_installed():
            snapshot = superlevel.collect_superlevel_snapshot(builder, level)
            superlevel._record_unsupported(builder, CandidateEventV1(EventKind.SWITCH, edge.time, edge.point, edge.vertex, edge.peer, -1))
            superlevel._record_unsupported(builder, CandidateEventV1(EventKind.START, edge.time, edge.point, 10**6, -1, 10**6))
            superlevel._record_duplicate_live_owner(builder, dataclasses.replace(snapshot, duplicate_live_owner_edge_ids=(0, 2)), level)
            superlevel._record_symbolic_unresolvable(builder, snapshot, "SYMBOLIC_SPLIT_OVERLAY_UNRESOLVABLE")
            superlevel._record_symbolic_unresolvable(builder, snapshot, "SYMBOLIC_SPLIT_OVERLAY_UNRESOLVABLE")
            superlevel._record_edge_span_debts(builder, (dataclasses.replace(edge, span_unproven=True), edge))
            builder._refuse(CandidateRefusal.NO_RULE_SPAN_VANISHED, vertex_ids=(0, 1), participant_edge_keys=(builder.edges[0].key,), target_edge_keys=())
            builder._record_obligation(
                cause=CandidateRefusal.NO_RULE_JOINT_IS_ANTIPARALLEL,
                disposition=superlevel.ProofObligationDisposition.OBSERVED,
                vertex_ids=(2, 1, 2),
                participant_edge_keys=(builder.edges[1].key, builder.edges[0].key),
                level=edge.time,
            )
            participants = tuple(sorted({key for index in range(2) for key in (builder.edges[index].key,)}))
            contact = superlevel.EdgeContactPlanV1(
                events=(edge, dataclasses.replace(edge, kind=EventKind.SPLIT, peer=-1, edge=3)),
                time=edge.time,
                point=edge.point,
                point_key=(),
                participants=participants,
                dead_vertex_ids=(0, 1),
                chains=((0, 1),),
                births=(),
                kinds=(EventKind.EDGE, EventKind.SPLIT),
            )
            superlevel._emit_edge_contact(builder, contact)
            superlevel._emit_edge_contact(builder, dataclasses.replace(contact, kinds=(EventKind.EDGE,), events=(edge,)))
    settle(verifier)
    for operation in ("_record_unsupported", "_record_duplicate_live_owner", "_record_symbolic_unresolvable", "_record_edge_span_debts", "_refuse", "_record_obligation", "_emit_edge_contact"):
        assert verifier.prims[operation] > 0, f"no call of {operation} was checked: {dict(verifier.prims)}"


# --------------------------------------------------------------------------
# the corpus of the skeleton: the kernel's tests, the generator, the field
# --------------------------------------------------------------------------


def corpus_polygons() -> list:
    """`[(id, polygon, recorded level budget or None, the oracle ends in an internal error, outcome)]` of the polygons of the skeleton corpus (one of each shape)."""

    found: dict = {}
    for kind in ("synthetic", "field"):
        root = sc.matching(kind)
        if root is None:
            continue
        for row in sc.rows_of(root, derived=False):
            record = sc.read(root, row)
            call = nc.prepare_call(nc.OP_SKELETON, record.call_blob, record.before())
            found.setdefault(repr(call.args[0]), (f"{kind}:{row['id']}", call.args[0], call.kwargs.get("level_budget"), sc.is_internal_error(row["exception"]), row["outcome"]))
    return list(found.values())


def test_the_loop_the_snapshots_and_the_plans_equal_the_oracle_on_the_skeleton_corpus_and_name_the_internal_errors_of_the_oracle():
    pool = corpus_polygons()
    if not pool:
        pytest.skip(f"нет корпуса скелета под ядро {nc.clip_memo.kernel_code_identity()}: {sc.describe_missing('synthetic')}")
    chosen = [item for item in pool if item[3]] + [item for index, item in enumerate(pool) if not item[3] and index % 11 == 0 and sum(len(each.points) for each in item[1].loops) <= 40]
    verifier = verifier_of()
    for name, polygon, level, internal, outcome in chosen:
        run_oracle(verifier, polygon, leaf.fresh_process_state(), level=level)
    settle(verifier)
    assert len(chosen) > 80
    assert sum(verifier.internal_agreed.values()) >= 1, "no oracle TypeError of the corpus fell in the plans: the named refusal was never checked"


# --------------------------------------------------------------------------
# the comparison itself: a wrong state, a wrong text, a wrong cost is reported
# --------------------------------------------------------------------------


def test_the_comparison_reports_a_wrong_counter_in_the_init(monkeypatch):
    original_init = skeleton._Builder.__init__

    def counted_init(self, *arguments, **keywords):
        original_init(self, *arguments, **keywords)
        self.counters["edge_events"] += 1

    verifier = verifier_of()
    monkeypatch.setattr(skeleton._Builder, "__init__", counted_init)
    init_pass(verifier, dict(parts.named_polygons())["comb_4"], leaf.fresh_process_state())
    assert verifier.mismatches and "BUILDER_INIT" in str(verifier.mismatches[0])


def test_the_comparison_reports_a_wrong_step_of_the_loop_and_a_cost_the_native_side_does_not_pay(monkeypatch):
    polygon = dict(parts.named_polygons())["ell"]
    original_count = skeleton.EventQueueV1._count_at_time
    monkeypatch.setattr(skeleton.EventQueueV1, "_count_at_time", lambda self, time: original_count(self, time) + 1)
    verifier = verifier_of()
    run_oracle(verifier, polygon, leaf.fresh_process_state())
    assert any("BUILDER_RUN" in str(item) for item in verifier.mismatches), "a wrong count of the same-time events passed the step comparison"
    monkeypatch.setattr(skeleton.EventQueueV1, "_count_at_time", original_count)

    original_close = skeleton._Builder._close_short_lavs

    def paying(self):
        exact.SIGN_COUNTS["total"] += 1
        original_close(self)

    monkeypatch.setattr(skeleton._Builder, "_close_short_lavs", paying)
    verifier = verifier_of()
    run_oracle(verifier, polygon, leaf.fresh_process_state())
    assert any("sign_counts" in str(item) for item in verifier.mismatches), "a sign the native loop never asked passed the cost comparison"


def test_the_comparison_reports_a_wrong_snapshot_and_a_wrong_plan_text():
    import dataclasses

    import cftuv_envelope.wavefront.superlevel as superlevel_module
    import cftuv_envelope.wavefront.superlevel_closure as closure_module

    polygon = dict(parts.named_polygons())["ell"]

    def tamper_snapshot(original):
        def collect(builder, level):
            found = original(builder, level)
            return dataclasses.replace(found, stale_candidates=found.stale_candidates + 1)

        return collect

    def tamper_plans(original):
        return lambda snapshot, budget=None: dataclasses.replace(original(snapshot, budget), unresolved_reason="TAMPERED")

    verifier = verifier_of()
    with leaf.swapped([("collect_superlevel_snapshot", tamper_snapshot, superlevel_module)]):
        run_oracle(verifier, polygon, leaf.fresh_process_state(), with_loop=False)
    assert any("COLLECT_SNAPSHOT" in str(item) for item in verifier.mismatches)
    verifier = verifier_of()
    with leaf.swapped([("plan_split_materialization", tamper_plans, closure_module)]):
        run_oracle(verifier, polygon, leaf.fresh_process_state(), with_loop=False)
    assert any("PLAN_SPLIT_MATERIALIZATION" in str(item) for item in verifier.mismatches)
