"""The native `chain_station` (`CHAIN_STATION_PLAN_V1`, `cftuv-station`) answers exactly what the Python law answers, or declines by name where the law raises.

The law (`cftuv_envelope._chain_station.chain_station_plans`) is the ORACLE. The answer is a frozenset of records; "exactly" is: equal plans, and the pickle of every plan equal
(exact types, exact values; object sharing between plans is not part of the answer). A decline (`NativePortUnsupported`) is allowed only where the oracle raises, and where the oracle raises
the port must decline (the dispatcher then runs the oracle, which raises its own exception). The comparisons:

* the kernel's own worlds of the law (`kernel/tests/test_chain_station_plan.py`, every test run with the law SHADOWED: each call is answered by the oracle and compared with the port);
* the tracked seed (`tests/data/native_chain_station_seed.xz`: real domains of the field scenes, canonical codec bytes; written by
  `artifacts/r3plus_20261018/scripts/station_corpus.py seed`) and seeded MUTANTS of it: positions moved across the chord budget and the bend limit, positions dropped, the other side of
  the seams removed, chains closed or of another kind, faces without area;
* the owner's FIELD corpus (every domain of the cold press of the six scenes; `native_field` tier, not in CI).

The pins are approved by the gates (CHAIN_STATION_PINS_APPROVED_V1, `chain_station_approved_pins.py`); a stale or missing table refuses by name.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import inspect
import lzma
import math
import os
import pickle
import random
import sys
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "kernel" / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "kernel" / "src"))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip("расширение cftuv_native не собрано: сверка операции chain_station с эталоном пропущена", allow_module_level=True)

import cftuv_envelope as kernel  # noqa: E402
from cftuv_envelope import _chain_station as law  # noqa: E402
from cftuv_envelope import backend  # noqa: E402
from cftuv_envelope.contracts.analysis import PhysicalChainKind  # noqa: E402
from cftuv_envelope.numeric import LocalPoint3V1, SurfaceCoordinateUnavailableReason, UnavailableSourcePositionV1  # noqa: E402
from cftuv_native import chain_station_op as op  # noqa: E402
from cftuv_native import pin  # noqa: E402

from native_gate import field_tier  # noqa: E402

SEED = ROOT / "tests" / "data" / "native_chain_station_seed.xz"
FIELD = Path(os.environ.get("CFTUV_NATIVE_CORPUS") or "E:/cftuv_native_corpus") / "chain_station"
FIELD_FILES = sorted(FIELD.glob("*.pkl.xz")) if FIELD.is_dir() else []
needs_field = field_tier(bool(FIELD_FILES), f"нет полевого корпуса chain_station ({FIELD}): `station_corpus.py build <snapshot_check corpus> <out>`")
MUTANTS_PER_SNAPSHOT = int(os.environ.get("CFTUV_CHAIN_STATION_MUTANTS", "24"))
CHECKED: Counter = Counter()
ORACLE = law.chain_station_plans


@pytest.fixture(scope="module", autouse=True)
def _report(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line(f"native chain_station compared with the Python law (python {sys.version.split()[0]}): " + ", ".join(f"{k} {v}" for k, v in sorted(CHECKED.items())))


@pytest.fixture
def approved():
    """The stage answers under its approved pins against this tree (a moved oracle is a failure here: a delta port and new pins are a separate step)."""

    op.refresh()
    assert op.status() == "available", op.status()
    yield
    op.refresh()


def _bytes(plans) -> list:
    return [pickle.dumps(plan, protocol=5) for plan in sorted(plans, key=lambda plan: plan.physical_chain_id.value)]


def compare(snapshot, domain, label: str = "") -> str:
    """`answered` / `declined` (where the oracle raises); any other outcome fails with the case named."""

    try:
        expected = ORACLE(snapshot, domain)
    except Exception as error:  # noqa: BLE001 - the oracle's exception is an outcome the port must decline
        with pytest.raises(cftuv_native.NativePortUnsupported):
            cftuv_native.chain_station_plans(snapshot, domain)
        return f"oracle raised {type(error).__name__}, port declined"
    try:
        found = cftuv_native.chain_station_plans(snapshot, domain)
    except cftuv_native.NativePortUnsupported as error:
        pytest.fail(f"{label}: the port declined an input the oracle answers: {error}")
    assert found == expected, label
    assert _bytes(found) == _bytes(expected), label
    return "answered"


# --------------------------------------------------------------------------
# the pins: approved by the gates (CHAIN_STATION_PINS_APPROVED_V1); without them, or stale, the stage refuses by name and the dispatcher names its fallback
# --------------------------------------------------------------------------


def test_the_approved_pins_cover_the_law_and_the_records_whole_and_answer_against_this_tree():
    op.refresh()
    assert set(op.APPROVED_PINS) == {"_chain_station.py", "contracts/chain_station.py"}
    assert cftuv_native.native_status()["chain_station"] == "available"


def test_without_approved_pins_the_stage_is_unavailable_and_refuses_by_name(monkeypatch):
    monkeypatch.setattr(op, "APPROVED_PINS", {})
    op.refresh()
    try:
        assert cftuv_native.native_status()["chain_station"] == "unavailable"
        with pytest.raises(pin.NativePortStale, match="CHAIN_STATION_PINS_NOT_APPROVED"):
            cftuv_native.chain_station_plans(SimpleNamespace(), None)
    finally:
        monkeypatch.undo()
        op.refresh()


def test_outside_a_native_order_the_dispatcher_is_the_oracle_and_a_stale_port_falls_back_by_name(monkeypatch):
    calls = []

    def oracle(snapshot, domain):
        calls.append((snapshot, domain))
        return frozenset()

    with backend.use_backend("PYTHON", chain_station_backend="PYTHON"):
        assert backend.chain_station_compute("s", "d", oracle) == frozenset() and calls == [("s", "d")]
    monkeypatch.setattr(op, "APPROVED_PINS", {**op.APPROVED_PINS, "_chain_station.py": "0" * 64})
    op.refresh()
    try:
        assert cftuv_native.native_status()["chain_station"] == "stale(_chain_station.py)"
        with backend.use_backend("PYTHON", chain_station_backend="NATIVE") as ledger:
            assert backend.chain_station_compute("s", "d", oracle) == frozenset()
        record = ledger.record()
        assert (record.chain_station_requested, record.chain_station_native_calls, record.chain_station_python_calls) == ("NATIVE", 0, 1)
        assert record.chain_station_outcomes == ("NATIVE_PORT_STALE",)
    finally:
        monkeypatch.undo()
        op.refresh()


def test_under_a_native_order_the_dispatcher_answers_natively_and_counts_it(approved):
    snapshot, domain = SEED_ROWS[0][2], SEED_ROWS[0][3]
    with backend.use_backend("PYTHON", chain_station_backend="NATIVE") as ledger:
        found = backend.chain_station_compute(snapshot, domain, ORACLE)
    assert found == ORACLE(snapshot, domain)
    assert (ledger.record().chain_station_native_calls, ledger.record().chain_station_python_calls) == (1, 0)


# --------------------------------------------------------------------------
# the kernel's own worlds, shadowed
# --------------------------------------------------------------------------


def _kernel_law_tests():
    spec = importlib.util.spec_from_file_location("kernel_chain_station_plan_tests", ROOT / "kernel" / "tests" / "test_chain_station_plan.py")
    module = sys.modules.get(spec.name)
    if module is None:
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module  # the module's dataclasses resolve their annotations through it
        spec.loader.exec_module(module)
    cases = []
    for name, function in sorted(vars(module).items()):
        if not (name.startswith("test_") and callable(function)):
            continue
        parameters = list(inspect.signature(function).parameters)
        if not parameters:
            cases.append((name, function, {}))
            continue
        for mark in getattr(function, "pytestmark", ()):
            if mark.name == "parametrize" and mark.args[0] == parameters[0] and len(parameters) == 1:
                for value in mark.args[1]:
                    cases.append((f"{name}[{value}]", function, {parameters[0]: value}))
    return module, cases


def test_every_world_of_the_kernel_law_tests_is_answered_alike(approved, monkeypatch):
    module, cases = _kernel_law_tests()
    outcomes: Counter = Counter()

    def shadow(snapshot, domain, facts=None):
        outcomes[compare(snapshot, domain, "kernel world")] += 1
        return ORACLE(snapshot, domain, facts)

    monkeypatch.setattr(law, "chain_station_plans", shadow)
    for name, function, arguments in cases:
        if name == "test_a_compilation_carries_the_plan_and_the_recount_accepts_it_and_refuses_a_forged_one":
            continue  # the compiled fixture: its plans are compared through the seed and the field corpus
        function(**arguments)
    assert len(cases) >= 15 and outcomes["answered"] >= 40, (len(cases), outcomes)
    CHECKED["kernel worlds"] += sum(outcomes.values())


# --------------------------------------------------------------------------
# the seed and its mutants
# --------------------------------------------------------------------------


def _read_seed() -> list:
    rows = []
    with lzma.open(SEED, "rb") as handle:
        while True:
            try:
                case, value, blob = pickle.load(handle)
            except EOFError:
                break
            snapshot = kernel.AnalysisSnapshotCodecV1.loads(blob)
            (domain,) = [item.patch_domain_id for item in snapshot.patch_domains if item.patch_domain_id.value == value]
            rows.append((case, value, snapshot, domain))
    return rows


SEED_ROWS = _read_seed() if SEED.is_file() else []


def test_the_seed_shows_the_outcomes_the_port_has_to_get_right():
    assert len(SEED_ROWS) >= 20, "the tracked seed is missing or thin"
    reasons = Counter(station.reason.name for _c, _v, snapshot, domain in SEED_ROWS for plan in ORACLE(snapshot, domain) for station in plan.stations)
    for name in ("TRANSVERSE_EDGES_INERT", "FREE_UNDER_CHART", "FOLD", "JUNCTION", "NEIGHBOUR_SIDE_UNKNOWN"):
        assert reasons[name], (name, reasons)
    kinds = Counter(station.kind.name for _c, _v, snapshot, domain in SEED_ROWS for plan in ORACLE(snapshot, domain) for station in plan.stations)
    assert kinds["CHAIN_JOINT"] and kinds["INTERIOR_VERTEX"], kinds


@pytest.mark.parametrize("row", SEED_ROWS, ids=lambda row: f"{row[0]}:{row[1][-8:]}")
def test_every_seed_domain_is_answered_alike(row, approved):
    case, value, snapshot, domain = row
    assert compare(snapshot, domain, f"{case} {value}") == "answered"
    CHECKED["seed"] += 1


def _root(snapshot, **changes):
    """A root the law reads like the snapshot (attributes only), with some containers replaced; the records inside stay the contract's."""

    fields = {name: getattr(snapshot, name) for name in ("surface_ir", "source_vertices", "physical_chains", "chain_uses", "corner_relations", "junction_relations", "seam_neighbour_faces", "surface_metric_descriptors")}
    fields.update(changes)
    return SimpleNamespace(**fields)


def _mutant(snapshot, domain, rng: random.Random):
    """One seeded mutant and the name of what changed."""

    chains = [item for item in snapshot.physical_chains if any(use.physical_chain_id == item.physical_chain_id and use.patch_domain_id == domain for use in snapshot.chain_uses)]
    on_chains = sorted({vertex for chain in chains for vertex in chain.ordered_source_vertex_ids}, key=lambda item: item.value)
    vertices = {item.vertex_id: item for item in snapshot.source_vertices}
    near = sorted({vertex for face in snapshot.surface_ir.source_faces if set(face.vertex_cycle) & set(on_chains) for vertex in face.vertex_cycle}, key=lambda item: item.value)
    kind = rng.choice(("move", "move", "move", "move", "line", "far", "drop", "neighbours", "close", "kind", "flat_face", "move_neighbour", "duplicate_face", "shadow_face", "twice"))
    if kind == "twice":
        once, first = _mutant(snapshot, domain, rng)
        again, _second = _mutant(once, domain, rng)
        return again, "twice"
    if kind in ("duplicate_face", "shadow_face") and near:
        faces = sorted((face for face in snapshot.surface_ir.source_faces if set(face.vertex_cycle) & set(on_chains)), key=lambda item: item.face_id.value)
        if faces:
            face = rng.choice(faces)
            if kind == "duplicate_face":
                # two records of ONE face id in the surface (the oracle's dict keeps the place of the first and the value of the last, in iteration order): the second record
                # reversed, without its last vertex (another plane and other edges) or in another patch
                cycle = face.vertex_cycle
                other = rng.choice((
                    dataclasses.replace(face, vertex_cycle=tuple(reversed(cycle))),
                    dataclasses.replace(face, vertex_cycle=cycle[:-1], edge_cycle=face.edge_cycle[:-1]) if len(cycle) > 3
                    else dataclasses.replace(face, vertex_cycle=cycle[1:] + cycle[:1]),
                    dataclasses.replace(face, patch_id=type(face.patch_id)(face.patch_id.value + ":other")),
                ))
                surface = SimpleNamespace(payload_mode=snapshot.surface_ir.payload_mode, source_faces=snapshot.surface_ir.source_faces | {other})
                return _root(snapshot, surface_ir=surface), kind
            # a neighbour record of a face the surface already has: the oracle ignores it (positions included)
            from cftuv_envelope.contracts.analysis import SeamNeighbourFaceV1

            ghost = SeamNeighbourFaceV1(face.face_id, face.patch_id, face.vertex_cycle, tuple(LocalPoint3V1(9.0, 9.0, float(k)) for k in range(len(face.vertex_cycle))))
            return _root(snapshot, seam_neighbour_faces=snapshot.seam_neighbour_faces | {ghost}), kind
    if kind == "far" and chains:
        # both chain neighbours of an interior vertex 20 m away, the chain bent by 1e-3 rad: a bend under the artist error, a chord of about 10 mm over the budget
        chain = rng.choice(sorted(chains, key=lambda item: item.physical_chain_id.value))
        ids = chain.ordered_source_vertex_ids
        if len(ids) >= 3:
            at = rng.randrange(1, len(ids) - 1)
            records = [vertices.get(ids[at + k]) for k in (-1, 0, 1)]
            if all(record is not None and isinstance(record.position, LocalPoint3V1) for record in records):
                p, q = records[1].position, records[2].position
                u = [q.x - p.x, q.y - p.y, q.z - p.z]
                length = sum(c * c for c in u) ** 0.5
                if length > 0:
                    u = [c / length for c in u]
                    w = [-u[1], u[0], 0.0] if abs(u[2]) < 0.9 else [0.0, -u[2], u[1]]
                    bend = rng.choice((1e-3, 1.7e-3, 3e-4))
                    before = LocalPoint3V1(*(p_ - 20.0 * c for p_, c in zip((p.x, p.y, p.z), u)))
                    after = LocalPoint3V1(*(p_ + 20.0 * (c + bend * d) for p_, c, d in zip((p.x, p.y, p.z), u, w)))
                    moved = dict(vertices)
                    moved[ids[at - 1]] = dataclasses.replace(records[0], position=before)
                    moved[ids[at + 1]] = dataclasses.replace(records[2], position=after)
                    return _root(snapshot, source_vertices=frozenset(moved.values())), kind
    if kind in ("move", "line") and near:
        moved = dict(vertices)
        for vertex in rng.sample(near, min(len(near), rng.choice((1, 1, 2, 3)))):
            record = vertices.get(vertex)
            if record is None or not isinstance(record.position, LocalPoint3V1):
                continue
            p = record.position
            if kind == "line" and vertex in on_chains:
                chain = next(chain for chain in chains if vertex in chain.ordered_source_vertex_ids)
                ids = chain.ordered_source_vertex_ids
                at = ids.index(vertex)
                if 0 < at < len(ids) - 1 and all(isinstance(getattr(vertices.get(ids[at + k]), "position", None), LocalPoint3V1) for k in (-1, 1)):
                    a, b = vertices[ids[at - 1]].position, vertices[ids[at + 1]].position
                    t = rng.choice((0.5, 0.25, 0.0, 1.0, -0.5, 1.5))
                    p = LocalPoint3V1(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z))
                    moved[vertex] = dataclasses.replace(record, position=p)
                    continue
            delta = rng.choice((1e-4, 1e-3, 3e-3, 4.9e-3, 5.0e-3, 5.1e-3, 8e-3, 2e-2, 0.1)) * rng.choice((1, -1))
            axis = rng.randrange(3)
            coordinates = [p.x, p.y, p.z]
            coordinates[axis] += delta
            moved[vertex] = dataclasses.replace(record, position=LocalPoint3V1(*coordinates))
        return _root(snapshot, source_vertices=frozenset(moved.values())), kind
    if kind == "drop" and near:
        vertex = rng.choice(near)
        if vertex in vertices:
            changed = dict(vertices)
            changed[vertex] = dataclasses.replace(vertices[vertex], position=UnavailableSourcePositionV1(next(iter(SurfaceCoordinateUnavailableReason))))
            return _root(snapshot, source_vertices=frozenset(changed.values())), kind
    if kind == "neighbours":
        if snapshot.seam_neighbour_faces and rng.random() < 0.5:
            # a neighbour face without positions: the oracle raises on `points[0]`, the port must decline
            face = sorted(snapshot.seam_neighbour_faces, key=lambda item: item.face_id.value)[0]
            try:
                empty = dataclasses.replace(face, positions=())
            except (ValueError, TypeError):
                return snapshot, "unchanged"
            return _root(snapshot, seam_neighbour_faces=(snapshot.seam_neighbour_faces - {face}) | {empty}), "empty_neighbour"
        return _root(snapshot, seam_neighbour_faces=frozenset()), kind
    if kind == "move_neighbour" and snapshot.seam_neighbour_faces:
        faces = sorted(snapshot.seam_neighbour_faces, key=lambda item: item.face_id.value)
        face = rng.choice(faces)
        if face.positions:
            index = rng.randrange(len(face.positions))
            p = face.positions[index]
            moved = LocalPoint3V1(p.x, p.y, p.z + rng.choice((2e-3, 6e-3, 3e-2)))
            changed = dataclasses.replace(face, positions=(*face.positions[:index], moved, *face.positions[index + 1:]))
            return _root(snapshot, seam_neighbour_faces=(snapshot.seam_neighbour_faces - {face}) | {changed}), kind
    if kind in ("close", "kind") and chains:
        chain = rng.choice(sorted(chains, key=lambda item: item.physical_chain_id.value))
        try:
            if kind == "close":
                changed = dataclasses.replace(chain, is_closed=not chain.is_closed)
            else:
                changed = dataclasses.replace(chain, kind=rng.choice([item for item in PhysicalChainKind if item is not chain.kind]))
        except (ValueError, TypeError):
            return snapshot, "unchanged"
        return _root(snapshot, physical_chains=(snapshot.physical_chains - {chain}) | {changed}), kind
    if kind == "flat_face" and near:
        faces = sorted((face for face in snapshot.surface_ir.source_faces if set(face.vertex_cycle) & set(on_chains)), key=lambda item: item.face_id.value)
        if faces:
            face = rng.choice(faces)
            anchor = vertices.get(face.vertex_cycle[0])
            if anchor is not None and isinstance(anchor.position, LocalPoint3V1):
                changed = dict(vertices)
                for vertex in face.vertex_cycle:
                    if vertex in changed:
                        changed[vertex] = dataclasses.replace(changed[vertex], position=anchor.position)
                return _root(snapshot, source_vertices=frozenset(changed.values())), kind
    return snapshot, "unchanged"


@pytest.mark.parametrize("row", SEED_ROWS, ids=lambda row: f"{row[0]}:{row[1][-8:]}")
def test_seeded_mutants_of_every_seed_domain_are_answered_alike_or_declined_where_the_oracle_raises(row, approved):
    case, value, snapshot, domain = row
    rng = random.Random(f"{case}:{value}")
    for index in range(MUTANTS_PER_SNAPSHOT):
        mutant, kind = _mutant(snapshot, domain, rng)
        outcome = compare(mutant, domain, f"{case} {value} mutant {index} ({kind})")
        CHECKED[f"mutant {kind}: {outcome}"] += 1


def test_the_worlds_and_the_mutants_reach_every_reason_the_field_does_not_show(approved, monkeypatch):
    """The kernel worlds and the mutants are a SELECTION too: the guard is on what they reach together (the arc and the drifting strip of the worlds reach what a random move does not)."""

    reasons: Counter = Counter()

    def recording(snapshot, domain, facts=None):
        plans = ORACLE(snapshot, domain, facts)
        reasons.update(station.reason.name for plan in plans for station in plan.stations)
        return plans

    module, cases = _kernel_law_tests()
    monkeypatch.setattr(law, "chain_station_plans", recording)
    for name, function, arguments in cases:
        if name != "test_a_compilation_carries_the_plan_and_the_recount_accepts_it_and_refuses_a_forged_one":
            function(**arguments)
    monkeypatch.setattr(law, "chain_station_plans", ORACLE)
    for case, value, snapshot, domain in SEED_ROWS:
        rng = random.Random(f"{case}:{value}")
        for _index in range(MUTANTS_PER_SNAPSHOT):
            mutant, _kind = _mutant(snapshot, domain, rng)
            try:
                plans = ORACLE(mutant, domain)
            except Exception:  # noqa: BLE001
                reasons["(oracle raised)"] += 1
                continue
            for plan in plans:
                for station in plan.stations:
                    reasons[station.reason.name] += 1
    for name in ("FOLD", "GROUP_NOT_FLAT", "BEND_BEYOND_STRAIGHT", "CHORD_BEYOND_BUDGET", "RUN_CHORD_BEYOND_BUDGET", "NEIGHBOUR_SIDE_UNKNOWN", "NOT_MANIFOLD", "POSITIONS_UNAVAILABLE", "CLOSED_CHAIN"):
        assert reasons[name], (name, reasons)


# --------------------------------------------------------------------------
# random strips: arcs and rings of chain pieces, the run windows and the joints
# --------------------------------------------------------------------------


def _strip_world(rng: random.Random, long: bool = False):
    """A chain path (open arc or closed ring) cut into pieces (some listed backwards), a strip of quads on each side; bends, edge lengths and the heights of the outer
    edges drawn around the bend limit, the chord budget and the drift that the run windows catch. Returns `(snapshot, domain)` with the contract's records."""

    from cftuv_envelope.contracts.analysis import PhysicalChainV1, SeamNeighbourFaceV1, SourceVertexV1
    from cftuv_envelope.contracts.surface import SourceFaceV1, SurfacePayloadMode
    from cftuv_envelope.ids import LineageId, PatchDomainId, PatchId, PhysicalChainId, PhysicalEdgeId, SourceFaceId, SourceVertexId
    from cftuv_envelope.numeric import LocalVector3V1

    ring = rng.random() < (0.5 if long else 0.3)
    # a ring is a regular polygon (every vertex bent) or a square with straight sides (the pieces meet in free joints on the sides: the start of the line matters)
    side = rng.randint(2, 5) if ring and (long or rng.random() < 0.7) else 0
    count = rng.randint(4, 14) if not ring else (4 * side if side else rng.randint(5, 14))
    # long edges: a chord over two edges leaves the budget while every bend stays under the limit (the window must read its last vertex)
    edge = rng.choice((8.0, 20.0) if long else (0.05, 0.3, 1.0, 3.0, 8.0, 20.0))
    turn = rng.choice((0.0, 3e-4, 1e-3, 1.7e-3, 2.5e-3, 1e-2))
    if ring:
        turn = 2 * math.pi / count
    lateral = rng.choice((1e-3, 4e-3) if long else (0.0, 1e-3, 4e-3))
    heading, x, y = 0.0, 0.0, 0.0
    path = []
    for index in range(count):
        path.append((x, y, rng.choice((0.0, 0.0, lateral, -lateral))))
        if not ring:
            heading += turn * rng.choice((1.0, 1.0, 0.5, -1.0))
        elif side:
            heading += math.pi / 2 if (index + 1) % side == 0 else 0.0
        else:
            heading += turn
        x, y = x + edge * math.cos(heading), y + edge * math.sin(heading)
    if not ring:
        path.append((x, y, 0.0))
    nodes = len(path)
    position = {}
    height_a, height_b = 0.0, 0.0
    for index, (px, py, pz) in enumerate(path):
        height_a += rng.choice((0.0, 0.0, 2e-3, 4.5e-3, -3e-3, 0.02))
        height_b += rng.choice((0.0, 0.0, 2e-3, -4.5e-3))
        position[f"c{index}"] = (px, py, pz)
        position[f"a{index}"] = (px, py - 1.0, height_a)
        position[f"b{index}"] = (px, py + 1.0, height_b)
    a_patch, b_patch = PatchId("patch:A"), PatchId("patch:B")
    links = list(range(nodes)) + ([0] if ring else [])

    def face(name, patch, names):
        return SourceFaceV1(SourceFaceId(name), patch, tuple(SourceVertexId(item) for item in names), tuple(PhysicalEdgeId(f"{name}:e{k}") for k in range(len(names))), LocalVector3V1(0.0, 0.0, 1.0), ())

    faces_a, faces_b = [], []
    for k in range(len(links) - 1):
        i, j = links[k], links[k + 1]
        faces_a.append(face(f"fa{k:02d}", a_patch, (f"c{i}", f"c{j}", f"a{j}", f"a{i}")))
        faces_b.append(face(f"fb{k:02d}", b_patch, (f"c{i}", f"b{i}", f"b{j}", f"c{j}")))
    cuts = sorted(set(rng.sample(range(1, len(links) - 1), rng.randint(0 if not ring else 1, min(3, len(links) - 3)))))
    bounds = (0, *cuts, len(links) - 1)
    kind = rng.choice((PhysicalChainKind.PHYSICAL_SEAM,) * 4 + (PhysicalChainKind.PHYSICAL_DECAL_SOURCE, PhysicalChainKind.SEAM_SELF))
    chains = []
    for number, (start, end) in enumerate(zip(bounds, bounds[1:])):
        names = [f"c{links[k]}" for k in range(start, end + 1)]
        if rng.random() < 0.4:
            names.reverse()
        chains.append(PhysicalChainV1(PhysicalChainId(f"chain:{rng.choice('PQRS')}{number}"), kind, False, tuple(SourceVertexId(item) for item in names),
                                      tuple(PhysicalEdgeId(f"chain:{number}:e{k}") for k in range(len(names) - 1)), frozenset({LineageId("lineage:C")}), frozenset({LineageId("lineage:C")})))
    if ring and rng.random() < 0.25:
        # the ring as ONE closed chain, listed with or without the repeated first vertex
        names = [f"c{k}" for k in links] if rng.random() < 0.5 else [f"c{k}" for k in links[:-1]]
        chains = [PhysicalChainV1(PhysicalChainId("chain:O"), kind, True, tuple(SourceVertexId(item) for item in names),
                                  tuple(PhysicalEdgeId(f"chain:O:e{k}") for k in range(len(names) - 1)), frozenset({LineageId("lineage:C")}), frozenset({LineageId("lineage:C")}))]
    if rng.random() < 0.2:
        # a spur along a transverse edge, from a cut of the pieces when there is one: three chains end there, so no joint
        at = links[rng.choice(cuts)] if cuts else rng.randrange(nodes)
        chains.append(PhysicalChainV1(PhysicalChainId("chain:T"), kind, False, (SourceVertexId(f"c{at}"), SourceVertexId(f"a{at}")),
                                      (PhysicalEdgeId("chain:T:e0"),), frozenset({LineageId("lineage:C")}), frozenset({LineageId("lineage:C")})))
    if len({item.physical_chain_id for item in chains}) != len(chains):
        return None
    own = rng.choice(("A", "A", "AB"))
    mine = faces_a if own == "A" else faces_a + faces_b
    other = faces_b if own == "A" and kind is not PhysicalChainKind.PHYSICAL_DECAL_SOURCE and rng.random() < 0.85 else []
    if kind is PhysicalChainKind.PHYSICAL_DECAL_SOURCE:
        mine = faces_a
    names = sorted({v.value for item in mine for v in item.vertex_cycle})
    dropped = {rng.choice(names)} if rng.random() < 0.1 else set()
    domain = PatchDomainId("domain:A")
    corners = tuple(SimpleNamespace(source_vertex_id=SourceVertexId(f"c{rng.randrange(nodes)}")) for _ in range(1 if rng.random() < 0.15 else 0))
    snapshot = SimpleNamespace(
        surface_ir=SimpleNamespace(payload_mode=SurfacePayloadMode.FULL_HOST_SURFACE, source_faces=frozenset(mine)),
        source_vertices=frozenset(
            SourceVertexV1(SourceVertexId(n), LocalPoint3V1(*position[n]) if n not in dropped else UnavailableSourcePositionV1(next(iter(SurfaceCoordinateUnavailableReason))))
            for n in names
        ),
        seam_neighbour_faces=frozenset(SeamNeighbourFaceV1(item.face_id, item.patch_id, item.vertex_cycle, tuple(LocalPoint3V1(*position[v.value]) for v in item.vertex_cycle)) for item in other),
        corner_relations=corners,
        junction_relations=frozenset(),
        chain_uses=tuple(SimpleNamespace(patch_domain_id=domain, physical_chain_id=item.physical_chain_id) for item in chains if rng.random() < 0.95),
        physical_chains=frozenset(chains),
        surface_metric_descriptors=frozenset(),
    )
    return snapshot, domain


STRIP_WORLDS = int(os.environ.get("CFTUV_CHAIN_STATION_STRIPS", "600"))


def test_random_strips_arcs_and_rings_are_answered_alike_and_reach_the_windows_and_the_joints(approved):
    rng = random.Random(20261018)
    reasons: Counter = Counter()
    joints = 0
    compared = 0
    while compared < STRIP_WORLDS:
        world = _strip_world(rng)
        if world is None:
            continue
        snapshot, domain = world
        CHECKED[f"strip world: {compare(snapshot, domain, f'strip world {compared}')}"] += 1
        compared += 1
        for plan in ORACLE(snapshot, domain):
            for station in plan.stations:
                reasons[station.reason.name] += 1
                joints += station.kind.name == "CHAIN_JOINT"
    for name in ("RUN_CHORD_BEYOND_BUDGET", "GROUP_NOT_FLAT", "BEND_BEYOND_STRAIGHT", "FOLD", "TRANSVERSE_EDGES_INERT", "JUNCTION", "NEIGHBOUR_SIDE_UNKNOWN", "CLOSED_CHAIN"):
        assert reasons[name], (name, reasons)
    assert joints >= 50, joints


LONG_WORLDS = int(os.environ.get("CFTUV_CHAIN_STATION_LONG_STRIPS", "2000"))


def test_long_strips_and_square_rings_drift_through_the_run_windows_alike(approved):
    """Long edges and a lateral drift under the per-vertex chord: only the run windows demote, so the window's last vertex and the start of a ring of pieces decide the answer
    (a window that skips its last vertex, a ring started at another piece: each changes about one world in two hundred here)."""

    rng = random.Random(20261019)
    runs = 0
    for index in range(LONG_WORLDS):
        world = _strip_world(rng, long=True)
        if world is None:
            continue
        snapshot, domain = world
        CHECKED[f"long strip world: {compare(snapshot, domain, f'long strip world {index}')}"] += 1
        runs += sum(station.reason.name == "RUN_CHORD_BEYOND_BUDGET" for plan in ORACLE(snapshot, domain) for station in plan.stations)
    assert runs >= 100, runs


# --------------------------------------------------------------------------
# the field corpus
# --------------------------------------------------------------------------


@needs_field
def test_every_domain_of_the_field_corpus_is_answered_alike(approved):
    count = 0
    for path in FIELD_FILES:
        with lzma.open(path, "rb") as handle:
            while True:
                try:
                    case, domain, blob, _plans = pickle.load(handle)
                except EOFError:
                    break
                snapshot = kernel.AnalysisSnapshotCodecV1.loads(blob)
                assert compare(snapshot, domain, f"{case} {domain.value}") == "answered"
                count += 1
    assert count >= 1000, count
    CHECKED["field"] += count
