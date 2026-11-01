"""BATCH_READERS_NATIVE_V1: the worker's readers of a materialized batch in Rust (`cftuv_native.batch_checks`, `batch_checks.rs`), against the oracle
(`validate_geometry_batch`, `audit_batch`, `batch_structure`; `materialize/batch_readers.py` is the one caller).

Real domains (flat and developed, with and without seams) give the oracle's audit and structure field for field and are proved clean exactly when the oracle finds no
issue; every spoiled batch the oracle names an issue on is NOT proved clean (the oracle then names it, as before), and every input the oracle raises on is declined.
The oracle's sources are pinned here.
"""

from __future__ import annotations

import copy
import dataclasses
import functools
import hashlib
import inspect
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "kernel" / "src", ROOT / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cftuv.envelope_production_export import produce_domain  # noqa: E402
from cftuv_envelope import _cpython311, validation, validation_issues  # noqa: E402
from cftuv_envelope import backend  # noqa: E402
from cftuv_envelope.contracts.geometry_batch import GEOMETRY_BATCH_SCHEMA_V1, GeometryInterfaceChainV1  # noqa: E402
from cftuv_envelope.contracts.metric import CUT_RIGHT_COPY_MARK  # noqa: E402
from cftuv_envelope.ids import SemanticDigestValue, SemanticInterfaceId, VertexKey  # noqa: E402
from cftuv_envelope.materialize import audit as audit_module  # noqa: E402
from cftuv_envelope.materialize import batch_readers as readers_module  # noqa: E402
from cftuv_envelope.materialize import structure as structure_module  # noqa: E402
from content_fixtures import cold_domain, with_revision  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "batch_checks"), reason="расширение cftuv_native не собрано (или собрано без BATCH_READERS_NATIVE_V1)"
)

#: The oracle of the port: its functions, in this order (a change here is a change to port to `batch_checks.rs`, then to this digest).
ORACLE = (
    (validation, ("validate_geometry_batch",)),
    (validation_issues, ("check_unique", "require_refs", "add_issue")),
    (audit_module, ("BatchAuditV1", "location_key", "_cross", "_area_vector", "_uv_area", "_normal_spread", "audit_batch")),
    (_cpython311, ("left_fold_sum",)),
    (structure_module, ("StructureSignatureV1", "_ring", "_chain_edges", "_part_digest", "batch_structure")),
    (readers_module, ("_oracle", "batch_readers")),
)
ORACLE_DIGEST = "e72697311d31de7fcd354a6b2029b2e5883fd6467d907b3ecde89a2553682065"


def _oracle_digest() -> str:
    digest = hashlib.sha256()
    for module, names in ORACLE:
        for name in names:
            digest.update(inspect.getsource(getattr(module, name)).replace("\r\n", "\n").encode("utf-8"))
    digest.update(repr((structure_module._SEPARATOR, structure_module._RECORD, CUT_RIGHT_COPY_MARK, GEOMETRY_BATCH_SCHEMA_V1)).encode("utf-8"))
    return digest.hexdigest()


def test_the_oracle_of_the_port_did_not_move():
    assert _oracle_digest() == ORACLE_DIGEST, "a reader of the batch changed: port the change to native/cftuv-python/src/batch_checks.rs, then update ORACLE_DIGEST"


def test_the_materializer_reads_the_batch_through_the_dispatcher_only():
    domain = (ROOT / "kernel/src/cftuv_envelope/materialize/domain.py").read_text(encoding="utf-8")
    for name in ("validate_geometry_batch(", "audit_batch(", "batch_structure("):
        assert name not in domain, f"_materialize_domain calls {name} past batch_readers"
    assert "batch_readers(" in domain


ROW = 5
EDGES = frozenset(range(ROW))
CASES = [(patch, lifted, alpha) for patch in range(ROW) for lifted in (0.0, 1.0) for alpha in (0.25, 0.4)]


@functools.lru_cache(maxsize=None)
def _eager(patch: int, lifted: float, alpha: float):
    source = with_revision(quad_row_bundle(ROW, lifted_corner=lifted), "row")
    deferred, prepared, _revision, _request = cold_domain(source, patch, EDGES, alpha=alpha)
    return produce_domain(patch, deferred.domain_id, prepared, str(float(alpha)))


def _inputs(result):
    normals = dict(getattr(result, "vertex_normals", ()) or ())
    return result.source_normal, normals or None


def _port(batch, source_normal, vertex_normals, validate=True, audit=True, structure=True):
    return cftuv_native.batch_checks(batch, GEOMETRY_BATCH_SCHEMA_V1, SemanticDigestValue, source_normal, vertex_normals, CUT_RIGHT_COPY_MARK, validate, audit, structure)


def _same(batch, source_normal, vertex_normals) -> bool:
    """The port against the oracle on one batch: the audit and the structure field for field, and "proved clean" only when the oracle finds no issue."""

    issues, audited, signature = readers_module._oracle(batch, source_normal, vertex_normals, True, True, True, False)
    clean, found_audit, found_structure = _port(batch, source_normal, vertex_normals)
    assert audit_module.BatchAuditV1(*found_audit) == audited
    assert structure_module.StructureSignatureV1(found_structure[0], tuple(found_structure[1])) == signature
    assert clean == (not issues)
    return clean


def _seamed(batch):
    chains = frozenset(
        GeometryInterfaceChainV1(SemanticInterfaceId(f"interface:{number}"), keys if number % 2 else tuple(reversed(keys)))
        for number, keys in enumerate(chain.ordered_vert_keys for chain in sorted(batch.boundary_chains, key=lambda item: item.semantic_boundary_id.value))
    )
    return dataclasses.replace(batch, interface_chains=chains)


def _results():
    return [result for result in (_eager(*case) for case in CASES) if result.batch is not None]


@needs_native
def test_real_domains_give_the_oracles_audit_and_structure_and_are_proved_clean():
    results = _results()
    assert len(results) >= 10
    assert any(getattr(result, "vertex_normals", ()) for result in results)  # a developed domain: one offset normal per vertex
    assert any(not getattr(result, "vertex_normals", ()) for result in results)
    for result in results:
        assert _same(result.batch, *_inputs(result))
        assert _same(_seamed(result.batch), *_inputs(result))


def _reversed_face(batch, index):
    faces = list(batch.faces)
    face = faces[index]
    faces[index] = dataclasses.replace(face, ordered_vert_keys=tuple(reversed(face.ordered_vert_keys)), uv_facts=tuple(reversed(face.uv_facts)))
    return dataclasses.replace(batch, faces=tuple(faces))


@needs_native
def test_a_cracked_or_flipped_mesh_gives_the_oracles_audit():
    for result in _results():
        source_normal, normals = _inputs(result)
        batch = result.batch
        variants = [_reversed_face(batch, 0), _reversed_face(batch, len(batch.faces) - 1), dataclasses.replace(batch, faces=batch.faces[1:]), dataclasses.replace(batch, boundary_chains=frozenset())]
        for variant in variants:
            _same(variant, source_normal, normals)


@needs_native
def test_every_batch_the_oracle_names_an_issue_on_is_not_proved_clean():
    result = next(item for item in _results() if not getattr(item, "vertex_normals", ()))
    batch = result.batch
    source_normal, normals = _inputs(result)
    vertices = sorted(batch.vertices, key=lambda item: item.vert_key.value)
    faces = list(batch.faces)
    facts = sorted(batch.station_facts, key=lambda item: item.station_fact_id.value)
    regions = sorted(batch.semantic_regions, key=lambda item: item.semantic_region_id.value)
    first_uv = faces[0].uv_facts[0]
    other_region = type(regions[0].semantic_region_id)("region:elsewhere")
    spoiled = {
        "schema": dataclasses.replace(batch, schema_version="cftuv.geometry_batch.v0"),
        "coordinate key": dataclasses.replace(batch, vertices=frozenset([dataclasses.replace(vertices[0], vert_key=VertexKey("coord:1,2,3")), *vertices[1:]])),
        "duplicate vertex key": dataclasses.replace(batch, vertices=frozenset([*vertices, dataclasses.replace(vertices[0], position=dataclasses.replace(vertices[0].position, x=vertices[0].position.x + 1.0))])),
        "missing face vertex": dataclasses.replace(batch, faces=(dataclasses.replace(faces[0], ordered_vert_keys=(VertexKey("nowhere"), *faces[0].ordered_vert_keys[1:]), uv_facts=(dataclasses.replace(first_uv, vert_key=VertexKey("nowhere")), *faces[0].uv_facts[1:])), *faces[1:])),
        "missing region": dataclasses.replace(batch, faces=(dataclasses.replace(faces[0], semantic_region_id=other_region), *faces[1:])),
        "conflicting UV": dataclasses.replace(batch, faces=(dataclasses.replace(faces[0], uv_facts=(dataclasses.replace(first_uv, uv=dataclasses.replace(first_uv.uv, u=first_uv.uv.u + 0.5)), *faces[0].uv_facts[1:])), *faces[1:]) if len(faces) > 1 else faces),
        "missing station fact": dataclasses.replace(batch, station_facts=frozenset(facts[1:])),
        "duplicate face id": dataclasses.replace(batch, faces=(*faces, faces[0])),
        "material differs": dataclasses.replace(batch, semantic_regions=frozenset([dataclasses.replace(regions[0], material_id=type(regions[0].material_id)("material:other")), *regions[1:]])),
        "missing chain vertex": dataclasses.replace(batch, interface_chains=frozenset([GeometryInterfaceChainV1(SemanticInterfaceId("interface:x"), (VertexKey("nowhere"), vertices[0].vert_key))])),
        "digest class": dataclasses.replace(batch, semantic_digest=VertexKey(batch.semantic_digest.value)),
    }
    named = 0
    for name, variant in spoiled.items():
        issues = validation.validate_geometry_batch(variant, check_semantic_digest=False)
        try:
            found = _port(variant, source_normal, normals, audit=False, structure=False)
        except cftuv_native.NATIVE_REFUSALS:
            continue  # declined: the oracle answers
        if issues:
            assert found[0] is False, f"{name}: the oracle names {issues[:2]} but the port proved the batch clean"
            named += 1
    assert named >= 9, named


@needs_native
def test_an_input_the_oracle_raises_on_is_declined():
    result = next(item for item in _results() if not getattr(item, "vertex_normals", ()))
    batch = result.batch
    faces = list(batch.faces)
    two = copy.copy(faces[0])  # the contract refuses a face of two corners: built past its check, as a broken writer would
    object.__setattr__(two, "ordered_vert_keys", faces[0].ordered_vert_keys[:2])
    object.__setattr__(two, "uv_facts", faces[0].uv_facts[:2])
    short = (two, *faces[1:])
    for variant, source_normal in ((batch, None), (dataclasses.replace(batch, faces=short), result.source_normal)):
        with pytest.raises(Exception):
            audit_module.audit_batch(variant, source_normal)
        with pytest.raises(cftuv_native.NATIVE_REFUSALS):
            _port(variant, source_normal, None, validate=False, structure=False)


@needs_native
def test_the_dispatcher_runs_the_port_under_the_w1_switch_and_the_oracle_names_what_the_port_did_not_prove(monkeypatch):
    result = next(item for item in _results() if not getattr(item, "vertex_normals", ()))
    batch = result.batch
    source_normal, normals = _inputs(result)
    calls = []
    real = cftuv_native.batch_checks
    monkeypatch.setattr(cftuv_native, "batch_checks", lambda *arguments: calls.append(arguments) or real(*arguments))
    expected = readers_module._oracle(batch, source_normal, normals, True, True, True, False)
    broken = dataclasses.replace(batch, schema_version="cftuv.geometry_batch.v0")
    with backend.use_backend("PYTHON", materialize_backend="NATIVE") as ledger:
        assert readers_module.batch_readers(batch, source_normal, normals, validate=True, audit=True, structure=True, check_semantic_digest=False) == expected
        named = readers_module.batch_readers(broken, source_normal, normals, validate=True, audit=True, structure=True, check_semantic_digest=False)
        lean = readers_module.batch_readers(batch, None, None, validate=False, audit=False, structure=True)
    assert named == readers_module._oracle(broken, source_normal, normals, True, True, True, False) and named[0]
    assert lean == ((), None, expected[2])
    assert len(calls) == 3 and ledger.record().materialize_native_calls >= 3
    with backend.use_backend("PYTHON", materialize_backend="PYTHON"):
        assert readers_module.batch_readers(batch, source_normal, normals, validate=True, audit=True, structure=True, check_semantic_digest=False) == expected
    assert len(calls) == 3
