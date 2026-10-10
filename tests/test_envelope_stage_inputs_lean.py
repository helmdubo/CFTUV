"""Пролог кнопки без сцены топологии (`stage_production_inputs`) даёт ТО ЖЕ, что полный пролог (`stage_domain_inputs`), и не молчит о том, чего не может.

Кнопка читала из пролога четыре величины: ревизию, патчи, id запроса и выделенные рёбра по доменам. Сцена топологии (объект на каждую петлю,
цепочку и сторону шва) ей не нужна и отбрасывалась, но строилась и стоила на `cover.008` около половины секунды родителя до первой задачи пула.
Облегчённый пролог считает те же четыре величины тем же обходом без объектов сцены, а всё, что сцена проверяла, строя пути и принимая себя,
проверяет сам (вершина есть и конечна, нормаль конечна, у пути не меньше двух точек и допустимое число рёбер).

Утверждения, и у каждого оракул - полный пролог:

1. ОТВЕТ ТОТ ЖЕ на фикстурах и на сотнях выделений (все рёбра, каждое ребро отдельно, случайные подмножества - частичные выделения цепочек
   дополняются до целых): четыре величины равны.
2. ЗАПИСИ ПРОФИЛЯ ТЕ ЖЕ: счётчики (в том числе по доменам), квитанции доменов и имена стадий со временем в том же порядке.
3. ОТКАЗ ТОТ ЖЕ. Облегчённый счёт не бросает исключений вместо полного: на входе, где полный пролог бросает (отказ выделения, непарная сторона шва,
   бесконечная вершина, нечисловая нормаль, вершина вне поверхности, путь с неверным числом рёбер или точек), облегчённый счёт сам отказывается, а пролог
   отдаёт слово полному - то же исключение, тот же текст. Где полный пролог ответил, а входы расходятся (ревизия экспорта и пакета), - тот же ответ.
4. ПАМЯТЬ ПО ВИДАМ: облегчённый и полный прологи одного выделения лежат отдельными записями и не отвечают друг за друга.
5. КНОПКА НЕ СТРОИТ СЦЕНУ: прогон не вызывает `build_envelope_topology_debug_scene`, а записи профиля пролога в профиле прогона те же, что у полного пролога.
6. ПОЛЕВОЙ СЛЕПОК `building` (122 домена) сверяется в отдельном процессе (`artifacts/host_parent_prologue/field_probe.py`).
"""

from __future__ import annotations

import dataclasses
import json
import random
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
KERNEL_SRC = ROOT / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_request_export as request_export  # noqa: E402
from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import shutdown_domain_pool  # noqa: E402
from cftuv.envelope_production_export import run_production  # noqa: E402
from cftuv.envelope_topology_export import (  # noqa: E402
    STAGE_INPUTS_HIT,
    STAGE_INPUTS_LEAN_DECLINED,
    STAGE_INPUTS_MISS,
    StageInputsMemoV1,
    _production_inputs,
    build_envelope_topology_export,
    stage_domain_inputs,
    stage_production_inputs,
)
from content_fixtures import moved_vertex, with_revision  # noqa: E402
from envelope_fixture_bundles import (  # noqa: E402
    bundle_from_exported_snapshot,
    host_exported_snapshot_paths,
    planar_quad_bundle,
    quad_row_bundle,
    square_hole_bundle,
    u_route_bundle,
)
from surface_adjacency_field_corpus import load_snapshot  # noqa: E402

FIELD_PROBE = ROOT / "artifacts" / "host_parent_prologue" / "field_probe.py"
BUILDING_DOMAINS = 122


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


def fixture_bundles():
    """`(имя, пакет)`: сконструированные меши и пакеты, выпущенные хостом по фикстурам корпуса."""

    found = [
        ("quad_row_5", quad_row_bundle(5)),
        ("quad_row_3_lifted", quad_row_bundle(3, lifted_corner=0.3)),
        ("planar_quad", planar_quad_bundle()),
        ("planar_quad_kinked", planar_quad_bundle((2.0, 0.4, 0.0))),
        ("u_route", u_route_bundle()),
        ("u_route_split", u_route_bundle(split_route=True)),
        ("square_hole", square_hole_bundle()),
    ]
    for path in host_exported_snapshot_paths():
        bundle, _patch = bundle_from_exported_snapshot(load_snapshot(path.parent))
        found.append((f"snapshot:{path.parent.name}", bundle))
    return found


BUNDLES = fixture_bundles()


def chain_edges(topology):
    return sorted({int(edge) for record in topology.host_chains for edge in record.canonical_edge_ids})


def selections(topology, seed=3, random_count=12):
    """Все рёбра цепочек, каждое ребро отдельно (частичное выделение цепочки дополняется), случайные подмножества."""

    edges = chain_edges(topology)
    rng = random.Random(seed)
    found = [frozenset(edges)]
    found += [frozenset({edge}) for edge in edges[:40]]
    for _ in range(random_count):
        if len(edges) > 1:
            found.append(frozenset(rng.sample(edges, rng.randint(1, len(edges)))))
    return found


def run(function, bundle, selection, topology):
    """`("ok", значение, запись профиля)` либо `("raised", тип, текст)`."""

    profile = EnvelopeDebugProfileBuilderV1("fixture", "PRODUCTION")
    try:
        value = function(bundle, selection, profile=profile, topology_export=topology)
    except Exception as exc:  # noqa: BLE001 - сравнивается именно исключение
        return ("raised", type(exc).__name__, str(exc))
    return ("ok", value, records(profile))


def records(profile):
    """Записи профиля; счётчик отказа облегчённого пролога вынесен в `declined` (в записях полного пролога его нет по определению)."""

    snapshot = profile.snapshot()
    return {
        "counters": tuple(item for item in snapshot.counters if item.name != STAGE_INPUTS_LEAN_DECLINED),
        "receipts": tuple(snapshot.receipts),
        "stages": tuple((item.stage, item.patch_domain_id) for item in snapshot.timings),
        "declined": any(item.name == STAGE_INPUTS_LEAN_DECLINED and item.value == 1 for item in snapshot.counters),
    }


def full_four(bundle, selection, *, profile, topology_export):
    return stage_domain_inputs(bundle, selection, profile=profile, topology_export=topology_export)[1:]


def assert_same(bundle, selection, topology, *, declined=None):
    """Облегчённый пролог равен полному (ответ либо исключение, записи профиля); `declined` - ждали ли отказа облегчённого счёта (None: не проверять)."""

    full = run(full_four, bundle, selection, topology)
    lean = run(
        lambda b, s, *, profile, topology_export: stage_production_inputs(b, s, profile=profile, topology_export=topology_export),
        bundle,
        selection,
        topology,
    )
    lean_declined = lean[2]["declined"] if lean[0] == "ok" else None  # на исключении записи профиля не сверяются: их никто не читает
    if lean[0] == "ok":
        assert lean[:2] == full[:2], sorted(selection)[:6]
        assert {**lean[2], "declined": False} == full[2], sorted(selection)[:6]
    else:
        assert lean == full, sorted(selection)[:6]
    if declined is not None and lean[0] == "ok":
        assert lean_declined is declined
    return full


# --------------------------------------------------------------------------
# 1 + 2. Ответ и записи профиля те же
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name,bundle", BUNDLES, ids=[name for name, _ in BUNDLES])
def test_the_answer_and_the_profile_records_equal_the_full_prologue(name, bundle):
    topology = build_envelope_topology_export(bundle)
    for selection in selections(topology):
        assert_same(bundle, selection, topology, declined=False)


def test_the_fixtures_exercise_both_answers_and_refusals():
    """Сверка выше не пуста: на фикстурах есть выделения, на которые пролог отвечает, и такие, на которые он отказывает."""

    answered = refused = 0
    for _name, bundle in BUNDLES:
        topology = build_envelope_topology_export(bundle)
        for selection in selections(topology, random_count=4):
            kind = run(full_four, bundle, selection, topology)[0]
            answered += kind == "ok"
            refused += kind == "raised"
    assert answered >= 50 and refused >= 1, (answered, refused)


def test_the_comparison_is_not_vacuous_a_domain_carries_edges_and_the_records_are_written():
    bundle = quad_row_bundle(5)
    topology = build_envelope_topology_export(bundle)
    outcome = assert_same(bundle, frozenset(chain_edges(topology)), topology)
    assert outcome[0] == "ok"
    revision, patch_ids, request_id, by_domain = outcome[1]
    assert patch_ids and request_id and any(by_domain.values())
    assert outcome[2]["counters"] and outcome[2]["receipts"]
    assert [stage for stage, _domain in outcome[2]["stages"]] == ["SELECTION_SCOPE", "TOPOLOGY_SCENE"]


def test_a_partial_selection_of_a_chain_is_completed_exactly_as_the_full_prologue_does():
    completions = 0
    for _name, bundle in BUNDLES:
        topology = build_envelope_topology_export(bundle)
        for record in topology.host_chains:
            if len(record.canonical_edge_ids) < 2:
                continue
            for edge in record.canonical_edge_ids[:2]:
                outcome = assert_same(bundle, frozenset({int(edge)}), topology)
                if outcome[0] == "ok":
                    counters = {item.name: item.value for item in outcome[2]["counters"] if item.patch_domain_id is None}
                    completions += bool(counters.get("SELECTION_COMPLETED_EDGES"))
    assert completions, "no selection exercised the completion of a partial chain"


def test_a_prologue_without_a_profile_answers_the_same():
    bundle = quad_row_bundle(5)
    topology = build_envelope_topology_export(bundle)
    selection = frozenset(chain_edges(topology))
    assert stage_production_inputs(bundle, selection, topology_export=topology) == stage_domain_inputs(bundle, selection, topology_export=topology)[1:]


# --------------------------------------------------------------------------
# 3. Отказ тот же: облегчённый счёт отказывается сам, исключение даёт полный пролог
# --------------------------------------------------------------------------


def test_the_refusals_of_the_selection_are_the_refusals_of_the_full_prologue():
    bundle = quad_row_bundle(5)
    topology = build_envelope_topology_export(bundle)
    for selection in (frozenset(), frozenset({10**6}), frozenset({int(max(chain_edges(topology))) + 1})):
        outcome = assert_same(bundle, selection, topology)
        assert outcome[0] == "raised", sorted(selection)


def test_every_input_the_scene_would_reject_is_declined_by_the_lean_count_and_answered_by_the_full_prologue():
    """Облегчённый счёт (`_production_inputs`) на таких входах САМ бросает - пролог отдаёт слово полному, и ответ полного не меняется."""

    base = with_revision(quad_row_bundle(5), "row")
    topology = build_envelope_topology_export(base)
    selection = frozenset(chain_edges(topology))
    vertex_id = int(base.patch_surface.vertices[0].vertex_id)

    infinite = dataclasses.replace(
        base,
        patch_surface=dataclasses.replace(
            base.patch_surface,
            vertices=tuple(
                dataclasses.replace(item, position=(float("inf"), 0.0, 0.0)) if int(item.vertex_id) == vertex_id else item
                for item in base.patch_surface.vertices
            ),
        ),
    )
    missing = dataclasses.replace(
        base,
        patch_surface=dataclasses.replace(
            base.patch_surface, vertices=tuple(item for item in base.patch_surface.vertices if int(item.vertex_id) != vertex_id)
        ),
    )
    not_a_number = with_revision(quad_row_bundle(5), "row")  # той же ревизии, свои узлы графа
    assert not_a_number.source_revision == base.source_revision
    next(iter(not_a_number.patch_graph.nodes.values())).normal = (float("nan"), 0.0, 1.0)

    bad = {"an infinite vertex": infinite, "a vertex outside the surface": missing, "a non-finite normal": not_a_number}
    for label, bundle in bad.items():
        full = run(full_four, bundle, selection, topology)
        assert full[0] == "raised", f"{label}: the full prologue must refuse for this comparison to mean anything"
        with pytest.raises(Exception):
            _production_inputs(bundle, selection, profile=EnvelopeDebugProfileBuilderV1("x", "y"), topology_export=topology)
        assert assert_same(bundle, selection, topology) == full, label


def spoiled_topology(case):
    """Экспорт, в котором запись первой цепочки нарушает то, что сцена проверяла, принимая себя; пакет тот же."""

    bundle = with_revision(quad_row_bundle(5), "row")
    topology = build_envelope_topology_export(bundle)
    record = topology.host_chains[0]
    if case == "a chain use with a wrong number of edges":
        broken = dataclasses.replace(record, chain=dataclasses.replace(record.chain, edge_indices=record.chain.edge_indices[:-1] + (7, 7, 7)))
    elif case == "a chain use with a single point":
        broken = dataclasses.replace(
            record, chain=dataclasses.replace(record.chain, vert_indices=record.chain.vert_indices[:1], edge_indices=())
        )
    else:
        assert case == "a physical chain with a wrong number of edges"
        broken = dataclasses.replace(record, canonical_edge_ids=record.canonical_edge_ids + (998, 999))
    spoiled = dataclasses.replace(topology, host_chains=(broken, *topology.host_chains[1:]))
    return bundle, topology, spoiled


SPOILED = (
    "a chain use with a wrong number of edges",
    "a chain use with a single point",
    "a physical chain with a wrong number of edges",
)


@pytest.mark.parametrize("case", SPOILED)
def test_a_spoiled_record_is_declined_by_the_lean_count_and_answered_by_the_full_prologue(case):
    bundle, topology, spoiled = spoiled_topology(case)
    selection = frozenset(chain_edges(topology))
    full = run(full_four, bundle, selection, spoiled)
    assert full[0] == "raised", "the full prologue must refuse for this comparison to mean anything"
    with pytest.raises(Exception):
        _production_inputs(bundle, selection, profile=EnvelopeDebugProfileBuilderV1("x", "y"), topology_export=spoiled)
    assert assert_same(bundle, selection, spoiled) == full


def test_an_export_of_another_revision_is_declined_and_the_answer_is_the_full_prologues():
    bundle = quad_row_bundle(5)
    topology = build_envelope_topology_export(bundle)
    other = dataclasses.replace(topology, source_revision_value=topology.source_revision_value + "+other")
    selection = frozenset(chain_edges(topology))
    with pytest.raises(Exception):
        _production_inputs(bundle, selection, profile=EnvelopeDebugProfileBuilderV1("x", "y"), topology_export=other)
    outcome = assert_same(bundle, selection, other, declined=True)
    assert outcome[0] == "ok", "the full prologue answers here, and the lean count names that it handed the word over"


@pytest.mark.parametrize("case", SPOILED)
def test_a_failed_lean_count_leaves_no_partial_records_in_the_profile_of_the_run(case):
    bundle, topology, spoiled = spoiled_topology(case)
    selection = frozenset(chain_edges(topology))
    lean_profile = EnvelopeDebugProfileBuilderV1("fixture", "PRODUCTION")
    full_profile = EnvelopeDebugProfileBuilderV1("fixture", "PRODUCTION")
    with pytest.raises(ValueError) as lean:
        stage_production_inputs(bundle, selection, profile=lean_profile, topology_export=spoiled)
    with pytest.raises(ValueError) as full:
        stage_domain_inputs(bundle, selection, profile=full_profile, topology_export=spoiled)
    assert str(lean.value) == str(full.value)
    lean_records = records(lean_profile)
    assert lean_records["declined"] is True and {**lean_records, "declined": False} == records(full_profile)


# --------------------------------------------------------------------------
# 4. Память по видам
# --------------------------------------------------------------------------


def test_the_memo_keeps_the_lean_and_the_full_prologue_apart_and_hands_out_copies():
    bundle = quad_row_bundle(5)
    topology = build_envelope_topology_export(bundle)
    selection = frozenset(chain_edges(topology))
    memo = StageInputsMemoV1()
    first = stage_production_inputs(bundle, selection, topology_export=topology, memo=memo)
    assert (memo.misses, memo.hits, memo.last) == (1, 0, STAGE_INPUTS_MISS)
    again = stage_production_inputs(bundle, selection, topology_export=topology, memo=memo)
    assert (memo.misses, memo.hits, memo.last) == (1, 1, STAGE_INPUTS_HIT)
    assert again == first and again[3] is not first[3], "the per-domain selection is handed out as a copy"
    again[3][next(iter(again[3]))].add(10**6)
    assert stage_production_inputs(bundle, selection, topology_export=topology, memo=memo)[3] == first[3]

    full = stage_domain_inputs(bundle, selection, topology_export=topology, memo=memo)
    assert memo.last == STAGE_INPUTS_MISS, "a lean record never answers the full prologue"
    assert len(full) == 5 and full[1:] == first and len(memo) == 2
    assert stage_domain_inputs(bundle, selection, topology_export=topology, memo=memo)[0] is full[0]
    stage_production_inputs(bundle, selection, topology_export=topology, memo=memo)
    assert memo.last == STAGE_INPUTS_HIT, "and a full record never answers the lean one"


def test_a_hit_writes_into_the_profile_what_the_lean_count_wrote():
    bundle = quad_row_bundle(5)
    topology = build_envelope_topology_export(bundle)
    selection = frozenset(chain_edges(topology))
    direct = EnvelopeDebugProfileBuilderV1("row", "PRODUCTION")
    stage_domain_inputs(bundle, selection, profile=direct, topology_export=topology)
    memo = StageInputsMemoV1()
    seen = []
    for _ in range(2):
        profile = EnvelopeDebugProfileBuilderV1("row", "PRODUCTION")
        stage_production_inputs(bundle, selection, profile=profile, topology_export=topology, memo=memo)
        seen.append(profile.snapshot())
    assert (memo.misses, memo.hits) == (1, 1)
    reference = direct.snapshot()
    for snapshot in seen:
        assert snapshot.counters == reference.counters and snapshot.receipts == reference.receipts
        assert snapshot.counters and snapshot.receipts


def test_a_disabled_memo_counts_the_lean_prologue_every_time():
    bundle = quad_row_bundle(5)
    topology = build_envelope_topology_export(bundle)
    selection = frozenset(chain_edges(topology))
    memo = StageInputsMemoV1()
    memo.enabled = False
    first = stage_production_inputs(bundle, selection, topology_export=topology, memo=memo)
    second = stage_production_inputs(bundle, selection, topology_export=topology, memo=memo)
    assert first == second == stage_domain_inputs(bundle, selection, topology_export=topology)[1:]
    assert memo.last == "OFF" and len(memo) == 0


# --------------------------------------------------------------------------
# 5. Кнопка не строит сцену
# --------------------------------------------------------------------------


def press(bundle, controller, alpha=0.25, selected=frozenset(range(5))):
    return run_production(
        controller,
        bundle,
        selected,
        alpha,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=0,
    )


@pytest.mark.parametrize("memo", (True, False))
def test_the_button_builds_no_topology_scene_and_its_profile_carries_the_prologue_records(monkeypatch, memo):
    calls = []
    real = request_export.build_envelope_topology_debug_scene

    def spy(*args, **kwargs):
        calls.append(1)
        return real(*args, **kwargs)

    monkeypatch.setattr(request_export, "build_envelope_topology_debug_scene", spy)
    bundle = quad_row_bundle(5)
    controller = EnvelopeDebugSessionController()
    controller.stage_inputs_memo.enabled = memo
    controller.scan_memo.enabled = memo
    run = press(bundle, controller)
    run_again = press(bundle, controller, 0.26)
    assert not calls, "the production button never needs the topology scene"
    assert all(item.is_materialized for item in run.results) and all(item.is_materialized for item in run_again.results)

    topology = controller.get_topology_export(bundle, "object", "mesh").with_chart_band(None, frozenset(range(5)))
    oracle = EnvelopeDebugProfileBuilderV1("row", "PRODUCTION")
    stage_domain_inputs(bundle, frozenset(range(5)), profile=oracle, topology_export=topology)
    assert calls, "the debug path still builds the scene (the oracle call above)"
    reference = oracle.snapshot()
    for produced in (run.profile, run_again.profile):
        counters = {(item.name, item.patch_domain_id): item.value for item in produced.counters}
        assert (STAGE_INPUTS_LEAN_DECLINED, None) not in counters, "an ordinary press never needs the full prologue"
        for item in reference.counters:
            assert counters[(item.name, item.patch_domain_id)] == item.value, item.name


def test_presses_across_a_mesh_edit_give_the_same_answers_with_and_without_the_memos():
    """Правка меша (другая ревизия): новый пролог на новом пакете, результаты доменов - как у сессии без памяти."""

    bundle = with_revision(quad_row_bundle(5), "row")
    edited = moved_vertex(bundle, int(bundle.patch_surface.vertices[0].vertex_id), (0.0, 0.05, 0.0), "row")
    with_memo, without = EnvelopeDebugSessionController(), EnvelopeDebugSessionController()
    without.stage_inputs_memo.enabled = False
    without.scan_memo.enabled = False
    for subject in (bundle, edited, bundle):
        left, right = press(subject, with_memo), press(subject, without)
        assert tuple(left.results) == tuple(right.results)


# --------------------------------------------------------------------------
# 6. Полевой слепок
# --------------------------------------------------------------------------


def test_field_snapshot_building_lean_prologue_and_encoder_equal_the_references():
    finished = subprocess.run(
        [sys.executable, str(FIELD_PROBE)],
        capture_output=True,
        text=True,
        timeout=900,
        cwd=str(ROOT),
    )
    assert finished.returncode == 0, finished.stderr[-3000:]
    report = json.loads(finished.stdout.strip().splitlines()[-1])
    assert report["domains"] == BUILDING_DOMAINS
    assert report["mismatches"] == [], report["mismatches"][:5]
    assert report["lean_selections"] >= 20 and report["keys_compared"] >= BUILDING_DOMAINS
    assert report["lean_profile_records_equal"] is True
