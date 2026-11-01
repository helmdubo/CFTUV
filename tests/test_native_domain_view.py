"""VIEW_BYTES_NATIVE_V1: the bytes of the view of a domain in Rust (`cftuv_native.domain_view_bytes`, `domain_view.rs`), against the oracle
(`encode_view(build_domain_view(result), structure_token(view))`).

Real domains (flat and developed) give the oracle's bytes byte for byte, through `put_view` too; every view the oracle refuses to the writer and every value the encoder would
not repeat is declined by the port and answered by the oracle as before; the oracle's source is pinned here.
"""

from __future__ import annotations

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

from cftuv import envelope_production_view as view_module  # noqa: E402
from cftuv import envelope_production_weld as weld_module  # noqa: E402
from cftuv import envelope_view_bytes as codec  # noqa: E402
from cftuv.envelope_production_export import deferred_result, lean_result, produce_domain  # noqa: E402
from content_fixtures import cold_domain, with_revision  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "domain_view_bytes"), reason="расширение cftuv_native не собрано (или собрано без VIEW_BYTES_NATIVE_V1)"
)

#: The oracle of the port: its functions, in this order (a change here is a change to port to `domain_view.rs`, then to this digest).
ORACLE = (
    (view_module, ("build_domain_view", "structure_token", "_claim_ordinals", "_finite", "_failed")),
    (weld_module, ("boundary_chain_names",)),
    (codec, ("encode_view", "_encode", "_float_rows", "_index_rows", "_pack")),
)
ORACLE_DIGEST = "ba7cb72fd8db9884f0468810fce23e0b8b9cd2c3691ffe77265f535998fd97e2"


def _oracle_digest() -> str:
    digest = hashlib.sha256()
    for module, names in ORACLE:
        for name in names:
            digest.update(inspect.getsource(getattr(module, name)).replace("\r\n", "\n").encode("utf-8"))
    digest.update(repr((codec._HEADER.format, codec._MAGIC, codec.VIEW_BYTES_VERSION, codec._TOKEN_BYTES, codec._FAILED)).encode("utf-8"))
    return digest.hexdigest()


def test_the_oracle_of_the_port_did_not_move():
    assert _oracle_digest() == ORACLE_DIGEST, "the view or its encoder changed: port the change to native/cftuv-python/src/domain_view.rs, then update ORACLE_DIGEST"


def test_put_view_asks_the_port_first_and_the_oracle_after_a_decline():
    source = inspect.getsource(codec.put_view)
    assert source.index("native_view_bytes(result)") < source.index("build_domain_view(result)")


ROW = 5
EDGES = frozenset(range(ROW))
CASES = [(patch, lifted, alpha) for patch in range(ROW) for lifted in (0.0, 1.0) for alpha in (0.25, 0.4)]


@functools.lru_cache(maxsize=None)
def _eager(patch: int, lifted: float, alpha: float):
    source = with_revision(quad_row_bundle(ROW, lifted_corner=lifted), "row")
    deferred, prepared, _revision, _request = cold_domain(source, patch, EDGES, alpha=alpha)
    return produce_domain(patch, deferred.domain_id, prepared, str(float(alpha)))


def _oracle(result) -> bytes:
    view = view_module.build_domain_view(result)
    return codec.encode_view(view, "" if view.failure is not None else view_module.structure_token(view))


def _port(result) -> bytes:
    return cftuv_native.domain_view_bytes(result.batch, result.normal, getattr(result, "vertex_normals", ()), getattr(result, "source_normal", None), result.patch_id)


@needs_native
@pytest.mark.parametrize("patch, lifted, alpha", CASES)
def test_the_bytes_of_a_real_domain_are_the_oracles(patch, lifted, alpha):
    eager = _eager(patch, lifted, alpha)
    if eager.batch is None:
        pytest.skip("the domain did not materialize")
    assert _port(eager) == _oracle(eager)


def _seamed(eager):
    """The domain with interface chains (the quad row has none): every boundary chain also an interface chain, and one pair of each chain reversed."""

    from cftuv_envelope.contracts.geometry_batch import GeometryInterfaceChainV1
    from cftuv_envelope.ids import SemanticInterfaceId

    chains = frozenset(
        GeometryInterfaceChainV1(SemanticInterfaceId(f"interface:{number}"), keys if number % 2 else tuple(reversed(keys)))
        for number, keys in enumerate(chain.ordered_vert_keys for chain in sorted(eager.batch.boundary_chains, key=lambda item: item.semantic_boundary_id.value))
    )
    return dataclasses.replace(eager, batch=dataclasses.replace(eager.batch, interface_chains=chains))


@needs_native
def test_the_cases_cover_flat_and_developed_domains_with_chains_and_seams():
    results = [_eager(*case) for case in CASES]
    batches = [result.batch for result in results if result.batch is not None]
    assert len(batches) >= 10
    assert any(getattr(result, "vertex_normals", ()) for result in results)  # a developed domain: one offset normal per vertex
    assert any(not getattr(result, "vertex_normals", ()) for result in results if result.batch is not None)
    assert all(batch.boundary_chains for batch in batches)
    seamed = 0
    for result in results:
        if result.batch is None:
            continue
        found = _seamed(result)
        expected = _oracle(found)
        assert _port(found) == expected
        seamed += bool(codec.decode_view(expected).seams)
    assert seamed >= 10

@needs_native
@pytest.mark.parametrize("make", [deferred_result, lean_result])
def test_put_view_writes_the_oracles_bytes_under_both_orders(make, monkeypatch):
    for case in CASES:
        eager = _eager(*case)
        if eager.batch is None:
            continue
        monkeypatch.setenv("CFTUV_MESH_ASSEMBLY_BACKEND", "NATIVE")
        native = make(eager).__dict__["view_bytes"]
        monkeypatch.setenv("CFTUV_MESH_ASSEMBLY_BACKEND", "PYTHON")
        assert make(eager).__dict__["view_bytes"] == native


def _spoiled(eager, **changes):
    return dataclasses.replace(eager.batch, **changes)


def _first_vertex(batch, **changes):
    vertices = sorted(batch.vertices, key=lambda item: item.vert_key.value)
    vertices[0] = dataclasses.replace(vertices[0], **changes)
    return type(batch.vertices)(vertices)


@needs_native
def test_a_view_the_writer_refuses_or_the_encoder_would_not_repeat_is_declined_and_the_oracle_answers():
    eager = next(result for result in (_eager(*case) for case in CASES) if result.batch is not None and getattr(result, "source_normal", None) is not None and not getattr(result, "vertex_normals", ()))
    batch = eager.batch
    position = sorted(batch.vertices, key=lambda item: item.vert_key.value)[0].position
    faces = list(batch.faces)
    claim = faces[0].ownership_claim_id
    odd_claim = [dataclasses.replace(faces[0], ownership_claim_id=type(claim)("claim:x")), *faces[1:]]
    spoiled = {
        "empty": (_spoiled(eager, faces=type(batch.faces)()), eager.normal),
        "int-coordinate": (_spoiled(eager, vertices=_first_vertex(batch, position=dataclasses.replace(position, x=int(position.x)))), eager.normal),
        "no-normal": (batch, None),
        "no-normal-finite": (batch, (float("nan"), 0.0, 1.0)),
        "opposed": (batch, tuple(-value for value in eager.normal)),
        "claim-not-digits": (_spoiled(eager, faces=type(batch.faces)(odd_claim)), eager.normal),
    }
    outcomes = {}
    for name, (spoilt, normal) in spoiled.items():
        with pytest.raises(cftuv_native.NATIVE_REFUSALS):
            cftuv_native.domain_view_bytes(spoilt, normal, getattr(eager, "vertex_normals", ()), eager.source_normal, eager.patch_id)
        try:
            outcomes[name] = codec.decode_view(_oracle(dataclasses.replace(eager, batch=spoilt, normal=normal))).failure
        except (codec.ViewBytesUnsupported, ValueError) as raised:
            outcomes[name] = type(raised).__name__
    # the oracle answers each by itself: a failure view of the writer, a value the encoder refuses by name, or an answer the port only declined to read
    assert outcomes["empty"][0] == view_module.OUTCOME_EMPTY_BATCH
    assert outcomes["no-normal-finite"][0] == view_module.OUTCOME_NORMAL_MISSING
    assert outcomes["no-normal"][0] == view_module.OUTCOME_NORMAL_MISSING
    assert outcomes["opposed"][0] == view_module.OUTCOME_NORMAL_OPPOSES_SOURCE
    assert outcomes["int-coordinate"] == "ViewBytesUnsupported"
    assert outcomes["claim-not-digits"] is None