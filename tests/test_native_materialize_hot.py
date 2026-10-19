"""The native materialize_hot operations against the Python oracle: `silhouette.apply_silhouette`, `domain._sourced` (`source_lift`), `assemble.lift_vertices` on a plane.

The seed (`tests/data/native_materialize_hot_seed.recs.xz`) holds real calls recorded on the field scenes (`building`, `rounded`, `sagging0987`, `half_sphere`, `two`): the inputs the
oracle reads, nothing else. Every record runs the oracle on the state before the call, then the state is put back and the native operation runs on the same state: the answer (canonical form,
floats by `hex`, `int` and `Fraction` distinct), the exception, the budget articles, the four canonicalization tables with their order, `SIGN_COUNTS` and `UNBUDGETED_WORK` must be the same.
Derived records reach the roads the field rarely takes: a budget the pass cannot finish in (`SKIPPED_WORK_BUDGET`), a face twice (`SKIPPED_NOT_MANIFOLD`), a source step under the lattice
(`DISPLACED`), no host positions (`UNAVAILABLE`), no step at all.
"""

from __future__ import annotations

import ast
import dataclasses
import enum
import hashlib
import json
import lzma
import pickle
import sys
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "kernel" / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "kernel" / "src"))

PACKAGE = ROOT / "native/cftuv-python/python/cftuv_native"
SEED = ROOT / "tests" / "data" / "native_materialize_hot_seed.recs.xz"

#: MATERIALIZE_HOT_PINS_APPROVED_V1 (by the gates, PIN_APPROVAL_BY_GATES_V1).
APPROVED = {
    "materialize/silhouette.py": "99d156d9f2930c722a3fd651a75f05961f58beb72ef212dd230663a1e536cdb4",
    "materialize/assemble.py": "6a5ea27d6269fc2c0942b011251544c15ba24450c5355cc3cf6be132a438239b",
    "materialize/source_lift.py": "2ebb0e8e7f8690cd6f37887129d4efae6c50f59c61f83b237b7a6373bd30698f",
    "materialize/uv_law.py": "f419ea42a3fb789412fab745b2dc249417e40e2e0c4ab16984f5cb8c025ef308",
    "materialize/lift.py": "ed91b42e5d6149d30469501922f8d7abfa65168d98043eccb3bdd2a87a78412b",
    "materialize/tessellate.py": "f31338338e71822bcf5c0619f8cb4dabdbe7b636bdaecba03b00414cf9493c56",
    "_cpython311.py": "d0ce9f6eccb090f0ef1e0615830f0534ac5211615da0d04103934d017e97e034",
    "float_filter.py": "ab188717ec92dd5ed45425af9a9f98f7c4e663af82bf01b8ea7e3177775b6346",
    "exact_sqrt_sum.py": "80abe9927dd193609d32b53103aa6dd663a6bec963549fa90896c0c354fb1741",
    "exact_sqrt_sum_fused.py": "1f7f7d50a56159090eb5c33633221c7ff69cb5a02f4a0e373df949b887e9a9a4",
    "_radicand_products.py": "9d64abad54bd202442384fe79cc9c2f45582bcebf0781043c36968c4cf979f70",
    "wavefront/faces.py": "ab52d0a44599a902ec293c278b200300cc4140bc61cb33a2bcadf4c67bad700e",
    "numeric.py": "bbe162cbaab350b31928c5f2e2d8818e9c898195d1bc529c3062f4104c7de9b9",
}


# --------------------------------------------------------------------------
# the pins (mechanical: no extension import)
# --------------------------------------------------------------------------


def test_production_table_is_exactly_the_approved_pins():
    tree = ast.parse((PACKAGE / "materialize_hot_approved_pins.py").read_text(encoding="utf-8"))
    body = [node for node in tree.body if not (isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant))]
    assert len(body) == 1 and isinstance(body[0], ast.Assign) and [target.id for target in body[0].targets] == ["APPROVED"], "data only: one literal, no code"
    assert ast.literal_eval(body[0].value) == APPROVED


def test_the_operation_answers_under_exactly_that_table():
    tree = ast.parse((PACKAGE / "materialize_hot_op.py").read_text(encoding="utf-8"))
    statement = next(node for node in tree.body if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "APPROVED_PINS" for target in node.targets))
    assert ast.dump(statement.value) == ast.dump(ast.parse("MappingProxyType(_APPROVED)", mode="eval").body)
    imports = [node for node in tree.body if isinstance(node, ast.ImportFrom) and node.module == "materialize_hot_approved_pins"]
    assert len(imports) == 1 and [(alias.name, alias.asname) for alias in imports[0].names] == [("APPROVED", "_APPROVED")] and imports[0].level == 1


def test_the_pinned_files_of_this_tree_are_the_approved_ones():
    kernel = ROOT / "kernel/src/cftuv_envelope"
    for name, digest in APPROVED.items():
        assert hashlib.sha256((kernel / name).read_bytes().replace(b"\r\n", b"\n")).hexdigest() == digest, name


def test_the_domain_routes_the_hot_calls_through_the_dispatcher():
    """`materialize.domain` calls the oracle of these three stages only through `backend` (MATERIALIZE_HOT_UNDER_W1_SWITCH_V1): a direct call would skip the native operation silently."""

    source = (ROOT / "kernel/src/cftuv_envelope/materialize/domain.py").read_text(encoding="utf-8")
    assert "backend.silhouette_compute(" in source and "backend.lift_vertices_compute(" in source and "backend.source_lift_compute(" in source
    tree = ast.parse(source)
    direct = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in ("apply_silhouette", "lift_vertices", "lift_source_vertices", "settle_emitted_faces")
    }
    sourced = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_sourced")
    inside = {node.func.id for node in ast.walk(sourced) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
    assert direct <= {"lift_source_vertices", "settle_emitted_faces"} and direct <= inside, direct


# --------------------------------------------------------------------------
# the operations against the oracle
# --------------------------------------------------------------------------

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "apply_silhouette"), reason="расширение cftuv_native не собрано (или собрано без materialize_hot): сверка с эталоном пропущена"
)


def canon(value):
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, float):
        return ["f", value.hex()]
    if isinstance(value, int):
        return ["i", int(value)]
    if isinstance(value, Fraction):
        return ["F", value.numerator, value.denominator]
    if isinstance(value, str):
        return value
    if isinstance(value, enum.Enum):
        return ["E", type(value).__name__, value.name]
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return ["D", type(value).__name__, [[item.name, canon(getattr(value, item.name))] for item in dataclasses.fields(value)]]
    if isinstance(value, tuple):
        return ["t", [canon(item) for item in value]]
    if isinstance(value, list):
        return ["l", [canon(item) for item in value]]
    if isinstance(value, dict):
        return ["d", [[canon(key), canon(item)] for key, item in value.items()]]
    if isinstance(value, (set, frozenset)):
        return ["s", type(value).__name__, sorted(json.dumps(canon(item), sort_keys=True) for item in value)]
    return ["r", type(value).__name__, repr(value)]


def _exact():
    from cftuv_envelope import exact_sqrt_sum

    return exact_sqrt_sum


def _articles():
    from cftuv_native import cost

    return cost.ARTICLES


def capture(budget):
    exact = _exact()
    return (
        list(exact._KNOWN_PRIMES),
        sorted(exact._KNOWN_PRIME_SET),
        list(exact._FACTORIZATION_MEMO.items()),
        list(exact._SQUAREFREE_MEMO.items()),
        list(exact._PRIME_SUPPORT_MEMO.items()),
        None if budget is None else tuple(budget.spent_by_article()),
        sorted(dict(exact.SIGN_COUNTS).items()),
        tuple(exact.UNBUDGETED_WORK.spent_by_article()),
    )


def restore(state, budget):
    exact = _exact()
    primes, prime_set, factorization, squarefree, support, articles, counts, unbudgeted = state
    exact._KNOWN_PRIMES[:] = primes
    exact._KNOWN_PRIME_SET.clear()
    exact._KNOWN_PRIME_SET.update(prime_set)
    for table, items in ((exact._FACTORIZATION_MEMO, factorization), (exact._SQUAREFREE_MEMO, squarefree), (exact._PRIME_SUPPORT_MEMO, support)):
        table.clear()
        table.update(items)
    exact.SIGN_COUNTS.clear()
    exact.SIGN_COUNTS.update(dict(counts))
    for name, value in zip(_articles(), unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)
    if budget is not None:
        for name, value in zip(_articles(), articles):
            setattr(budget, name, value)


def _run(function, *args):
    try:
        return "ok", function(*args)
    except Exception as error:  # noqa: BLE001 - an exception of the oracle is an outcome
        return "raise", error


def _view(outcome):
    kind, value = outcome
    if kind == "ok":
        return ["ok", canon(value)]
    return ["raise", type(value).__name__, str(value), canon(getattr(value, "counters", None))]


def same(label, oracle, native, budget, *args):
    """The oracle and the native operation on one state: the same answer and the same state after; `answered` or `declined` (a refusal of the port, nothing moved)."""

    before = capture(budget)
    expected = _run(oracle, *args)
    after = capture(budget)
    restore(before, budget)
    actual = _run(native, *args)
    if actual[0] == "raise" and isinstance(actual[1], cftuv_native.NATIVE_REFUSALS):
        assert capture(budget) == before, f"{label}: a refusal of the port moved the state"
        restore(after, budget)
        return "declined"
    assert _view(actual) == _view(expected), label
    assert capture(budget) == after, f"{label}: the state after the call differs"
    return "answered"


class _Layout:
    """The two questions the law asks of `assemble.Layout`, answered from a record."""

    def __init__(self, regions, cut) -> None:
        self.regions = list(regions)
        self.cut = frozenset(frozenset(pair) for pair in cut)

    def region_of(self, face) -> int:
        return self.regions[face]

    def cut_pairs(self, faces) -> frozenset:
        return self.cut


class _Plane:
    """A lift without offset normals (`rebind_position` absent), as `PlaneLiftV1`."""


@pytest.fixture(scope="module")
def seed():
    with lzma.open(SEED, "rb") as handle:
        return pickle.load(handle)


def _radical(points):
    """Every rational chart coordinate `c` as `c + c*sqrt(2)`: the same shape (a similarity of the chart), and the collinear triples the binary64 filter cannot decide pay a radical."""

    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

    def moved(value):
        return SqrtSumV1(((1, value.terms[0][1]), (2, value.terms[0][1]))) if len(value.terms) == 1 and value.terms[0][0] == 1 else value

    return {key: (moved(x), moved(y)) for key, (x, y) in points.items()}


def _silhouette_args(row, cap=None, polygons=None, points=None, uv_slide=...):
    from cftuv_envelope.exact_sqrt_sum import exact_work_budget
    from cftuv_envelope.materialize.silhouette import SilhouetteInputV1

    cycles = [[(key, None) for key in cycle] for cycle in row["cycles"]]
    vertex_cycles = None if row["vertex_cycles"] is None else [[(key, None) for key in cycle] for cycle in row["vertex_cycles"]]
    source = SilhouetteInputV1(
        row["polygons"] if polygons is None else polygons, cycles, vertex_cycles, row["positions"], row["points"] if points is None else points, row["facts"], list(range(len(row["regions"]))),
        _Layout(row["regions"], row["cut"]), row["lattice_alpha"], row["uv_slide"] if uv_slide is ... else uv_slide, row["reverse"], row["free"],
    )
    budget = exact_work_budget(stage="MATERIALIZE", domain_id="seed", cap=row["cap"] if cap is None else cap)
    return source, budget


@pytest.fixture
def approved():
    from cftuv_native import materialize_hot_op as op

    op.refresh()
    assert op.status() == "available", op.status()
    yield
    op.refresh()


@needs_native
def test_silhouette_matches_the_oracle_on_the_seed(seed, approved):
    from cftuv_envelope.materialize.silhouette import apply_silhouette

    rows = [row for row in seed if row["kind"] == "silhouette"]
    assert len(rows) >= 40
    outcomes = [same(f"silhouette {row['case']} #{index}", apply_silhouette, cftuv_native.apply_silhouette, budget, source, budget) for index, row in enumerate(rows) for source, budget in (_silhouette_args(row),)]
    assert outcomes.count("answered") == len(rows), outcomes


@needs_native
def test_silhouette_under_small_caps_and_cold_memory(seed, approved):
    """Caps 0, 1, 3 on a cold canonicalization memory and radical chart points: the inner budget of the pass, its replay into the domain's budget and (where a sign pays) the exhaustion
    of the pass (`SKIPPED_WORK_BUDGET`, the domain's budget untouched) are the oracle's."""

    from cftuv_envelope.materialize.silhouette import apply_silhouette

    for index, row in enumerate(row for row in seed if row["kind"] == "silhouette"):
        points = _radical(row["points"])
        for cap in (0, 1, 3):
            _exact().reset_factorization_memory()
            source, budget = _silhouette_args(row, cap=cap, points=points)
            assert same(f"silhouette cap {cap} #{index}", apply_silhouette, cftuv_native.apply_silhouette, budget, source, budget) == "answered"


@needs_native
def test_silhouette_keeps_a_mesh_that_is_not_manifold(seed, approved):
    from cftuv_envelope.materialize.silhouette import SKIPPED_NOT_MANIFOLD, apply_silhouette

    for index, row in enumerate(row for row in seed if row["kind"] == "silhouette"):
        polygons = [list(face) for face in row["polygons"]]
        first = next(position for position, face in enumerate(polygons) if face)
        polygons[first] = polygons[first] + [polygons[first][0]]
        source, budget = _silhouette_args(row, polygons=polygons)
        assert same(f"silhouette not manifold #{index}", apply_silhouette, cftuv_native.apply_silhouette, budget, source, budget) == "answered"
        assert (SKIPPED_NOT_MANIFOLD, 1) in apply_silhouette(*_silhouette_args(row, polygons=polygons)).counters


def _sourced(plane, positions, triangles, hosts, step, polygons, points, budget, reverse):
    from cftuv_envelope.materialize.domain import _sourced as oracle

    return oracle(plane, positions, triangles, hosts, step, polygons, points, budget, reverse)


def _source_args(row, *, hosts=None, step=..., cap=...):
    from cftuv_envelope.exact_sqrt_sum import exact_work_budget

    budget = exact_work_budget(stage="MATERIALIZE", domain_id="seed", cap=row["cap"] if cap is ... else cap)
    return (_Plane(), row["positions"], row["triangles"], row["hosts"] if hosts is None else hosts, row["step"] if step is ... else step, row["polygons"], row["points"], budget, row["reverse"])


@needs_native
def test_source_lift_matches_the_oracle_on_the_seed(seed, approved):
    rows = [row for row in seed if row["kind"] == "source"]
    assert len(rows) >= 40
    for index, row in enumerate(rows):
        args = _source_args(row)
        assert same(f"source {row['case']} #{index}", _sourced, cftuv_native.source_lift, args[7], *args) == "answered"


@needs_native
def test_source_lift_names_what_it_does_not_lift(seed, approved):
    """No step (`UNAVAILABLE`), no host positions, a step far under the lattice (`DISPLACED`), a budget the triangulations run out of (the oracle's exhaustion)."""

    from cftuv_envelope.materialize.source_lift import DISPLACED, UNAVAILABLE

    seen = set()
    for index, row in enumerate(row for row in seed if row["kind"] == "source"):
        for label, kwargs in (("no step", {"step": None}), ("no hosts", {"hosts": {}}), ("tiny step", {"step": Fraction(1, 10**12)}), ("cap 0", {"cap": 0})):
            args = _source_args(row, **kwargs)
            assert same(f"source {label} #{index}", _sourced, cftuv_native.source_lift, args[7], *args) == "answered"
            outcome = _run(_sourced, *_source_args(row, **kwargs))
            if outcome[0] == "ok":
                seen.update(name for name, value in outcome[1][1].counters() if value and name in (DISPLACED, UNAVAILABLE))
            else:
                seen.add(type(outcome[1]).__name__)
    assert {DISPLACED, UNAVAILABLE} <= seen, seen


@needs_native
def test_plane_lift_matches_the_oracle_on_the_seed(seed, approved):
    from cftuv_envelope.materialize.assemble import lift_vertices
    from cftuv_envelope.materialize.lift import PlaneLiftV1

    rows = [row for row in seed if row["kind"] == "lift"]
    assert len(rows) >= 20
    for index, row in enumerate(rows):
        plane = PlaneLiftV1(*row["plane"])
        assert canon(cftuv_native.lift_vertices(row["points"], plane)) == canon(lift_vertices(row["points"], plane)), index


@needs_native
def test_the_dispatcher_runs_the_port_and_names_a_decline(seed, approved, monkeypatch):
    """Under a ledger that orders the W1 stage NATIVE the dispatcher answers with the port; an input the port does not carry is the oracle's, under a named fallback."""

    from cftuv_envelope import backend
    from cftuv_envelope.materialize.assemble import lift_vertices
    from cftuv_envelope.materialize.lift import PlaneLiftV1
    from cftuv_envelope.materialize.silhouette import apply_silhouette

    row = next(row for row in seed if row["kind"] == "silhouette")
    calls = []
    monkeypatch.setattr(cftuv_native, "apply_silhouette", lambda source, budget: calls.append("native") or materialize_hot_apply(source, budget))
    with backend.use_backend("PYTHON", materialize_backend="NATIVE") as ledger:
        source, budget = _silhouette_args(row)
        found = backend.silhouette_compute(apply_silhouette, source, budget)
    assert calls == ["native"] and ledger.record().materialize_native_calls >= 1
    assert canon(found) == canon(apply_silhouette(*_silhouette_args(row)))
    lift = next(row for row in seed if row["kind"] == "lift")
    points = dict(lift["points"])
    points[next(iter(points))] = ("not a pair",)
    with backend.use_backend("PYTHON", materialize_backend="NATIVE") as ledger:
        with pytest.raises(Exception):
            lift_vertices(points, PlaneLiftV1(*lift["plane"]))
        with pytest.raises(Exception):
            backend.lift_vertices_compute(lift_vertices, points, PlaneLiftV1(*lift["plane"]))
    assert any(item[0] == "NATIVE_PORT_UNSUPPORTED" for item in ledger.record().materialize_fallbacks), ledger.record().materialize_fallbacks


def materialize_hot_apply(source, budget):
    from cftuv_native import materialize_hot_op

    return materialize_hot_op.apply_silhouette(cftuv_native.default_mirror(), source, budget)


@needs_native
def test_the_worker_start_binds_the_port_once(approved):
    """`warm_materialize_hot` is what a pool worker does in its start when the W1 stage is ordered NATIVE: the digests of the mirrored functions and the class table are bound once per
    process, not on the first domain (MATERIALIZE_HOT_UNDER_W1_SWITCH_V1)."""

    from cftuv_envelope import backend

    mirror = cftuv_native.default_mirror()
    assert backend.warm_materialize_hot() is True and getattr(mirror, "_hot_bound", False)
    assert backend.warm_materialize_hot() is True
    source = (ROOT / "cftuv" / "envelope_domain_pool.py").read_text(encoding="utf-8")
    assert "warm_materialize_hot()" in source and ".materialize_w1 == \"NATIVE\"" in source

@needs_native
def test_silhouette_under_other_uv_tolerances(seed, approved):
    """The UV slide tolerance from none to everything: the planned station edges, the kept-by-UV edges and the residual of the faces the law keeps decide as the oracle's."""

    from cftuv_envelope.materialize.silhouette import apply_silhouette

    for index, row in enumerate(row for row in seed if row["kind"] == "silhouette"):
        for uv_slide in (Fraction(0), Fraction(1, 10**12), Fraction(1, 2000), Fraction(1, 50), Fraction(10)):
            source, budget = _silhouette_args(row, uv_slide=uv_slide)
            assert same(f"silhouette uv_slide {uv_slide} #{index}", apply_silhouette, cftuv_native.apply_silhouette, budget, source, budget) == "answered"


@needs_native
def test_source_lift_at_exactly_the_budget(seed, approved):
    """A host exactly one budget away (a distance the step makes exact) is allowed, as the oracle's `squared <= budget_squared`."""

    from cftuv_envelope.materialize.source_lift import SOURCE_VERTEX_LIFT_BUDGET_CELLS, source_vertex_key
    from cftuv_envelope.numeric import LocalPoint3V1

    tried = 0
    for index, row in enumerate(row for row in seed if row["kind"] == "source"):
        key = next((key for key in row["positions"] if source_vertex_key(key) in row["hosts"]), None)
        if key is None:
            continue
        position = row["positions"][key]
        moved = LocalPoint3V1(position.x + 0.25, position.y, position.z)
        hosts = dict(row["hosts"], **{source_vertex_key(key): moved})
        step = (Fraction(moved.x) - Fraction(position.x)) / SOURCE_VERTEX_LIFT_BUDGET_CELLS
        args = _source_args(row, hosts=hosts, step=step)
        assert same(f"source at the budget #{index}", _sourced, cftuv_native.source_lift, args[7], *args) == "answered"
        outcome = _run(_sourced, *_source_args(row, hosts=hosts, step=step))
        tried += outcome[0] == "ok" and outcome[1][1].positions[key] == moved  # the orientation settle may still revert it
    assert tried >= 10, tried


@needs_native
def test_source_lift_cuts_a_face_without_an_exact_triangulation(seed, approved):
    """A face of four or more vertices whose chart is not simple (two corners on one chart point) has no exact triangulation: the oracle cuts it into the fan even when no ear flipped."""

    seen = 0
    for index, row in enumerate(row for row in seed if row["kind"] == "source"):
        points = dict(row["points"])
        for face_polygons in row["polygons"]:
            for polygon in face_polygons:
                if len(polygon) >= 4:
                    points[polygon[1]] = points[polygon[0]]
        args = list(_source_args(row))
        args[6] = points
        assert same(f"source not simple #{index}", _sourced, cftuv_native.source_lift, args[7], *args) == "answered"
        args = list(_source_args(row))
        args[6] = points
        outcome = _run(_sourced, *args)
        if outcome[0] == "ok" and outcome[1][2].triangulated:
            seen += 1
    assert seen, "no record cut a face"

@needs_native
def test_silhouette_on_a_coarse_grid(seed, approved):
    """Positions snapped to a coarse grid (1/8 m to 1/65536 m): exactly equal areas and distances, so the ties of the law (the bigger face of a merge, the first maximum) decide as the oracle's."""

    from cftuv_envelope.materialize.silhouette import apply_silhouette
    from cftuv_envelope.numeric import LocalPoint3V1

    for index, row in enumerate(row for row in seed if row["kind"] == "silhouette"):
        for grid in (8.0, 64.0, 1024.0, 4096.0, 65536.0):
            snapped = {key: LocalPoint3V1(round(p.x * grid) / grid, round(p.y * grid) / grid, round(p.z * grid) / grid) for key, p in row["positions"].items()}
            source, budget = _silhouette_args(dict(row, positions=snapped))
            assert same(f"silhouette grid {grid} #{index}", apply_silhouette, cftuv_native.apply_silhouette, budget, source, budget) in ("answered", "declined")