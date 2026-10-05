"""Общее решение по точкам на прямых цепях источника и стены (`envelope_production_dots`): упаковка закона ядра `SILHOUETTE_SOURCE_DOTS_V1`.

Закон, его правило и независимая проверка доказаны в ядре (`kernel/tests/test_source_dots.py`); здесь доказано, что хост ничего не
решает сам: результаты доменов идут в ядро как есть, обратно получают батч без точек, числа батча и числа закона, прогон не под
законом силуэта и отказанные домены не трогаются, кэш прогона (результат до решения) остаётся нетронутым.
"""

from __future__ import annotations

import sys
from hashlib import sha256
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from source_dots_factories import NORMAL, XS, strip  # noqa: E402

from cftuv.envelope_production_dots import dissolve_source_dots  # noqa: E402
from cftuv.envelope_production_export import MATERIALIZED, ProductionDomainResultV1  # noqa: E402
from cftuv_envelope.codec import canonical_json_bytes  # noqa: E402


def _result(patch_id, batch, law="SILHOUETTE_TOPOLOGY_V1", normals=()):
    return ProductionDomainResultV1(
        patch_id=patch_id,
        domain_id=f"domain:{patch_id}",
        outcome=MATERIALIZED,
        batch=batch,
        counters=(("MATERIALIZE_VERTICES", len(batch.vertices)), ("MATERIALIZE_FACES_EMITTED", len(batch.faces))),
        normal=(0.0, 0.0, 1.0),
        content_digest="old",
        diagnostics=("NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE: x",),
        source_normal=NORMAL,
        vertex_normals=normals,
        decal_topology_law=law,
    )


def _pair():
    return _result(0, strip("a", XS)), _result(1, strip("b", XS, side=-1))


def test_two_domains_of_a_run_get_their_shared_dots_dissolved_with_their_own_numbers():
    before = _pair()

    results, found = dissolve_source_dots(before)

    assert found is not None and found.changed and not found.problems
    for old, new in zip(before, results):
        assert len(new.batch.vertices) == len(old.batch.vertices) - 3
        counters = dict(new.counters)
        assert counters["MATERIALIZE_VERTICES"] == len(old.batch.vertices) - 3
        assert counters["MATERIALIZE_FACES_EMITTED"] == len(old.batch.faces)
        assert counters["MATERIALIZE_SILHOUETTE_SOURCE_DOTS_DISSOLVED"] == 3
        assert new.content_digest == sha256(canonical_json_bytes(new.batch)).hexdigest()
        assert new.diagnostics[0] == old.diagnostics[0] and new.diagnostics[-1].startswith("SILHOUETTE_SOURCE_DOTS_V1")
        assert [name for name, _value in new.counters[:2]] == ["MATERIALIZE_VERTICES", "MATERIALIZE_FACES_EMITTED"]
    assert before[0].batch is not results[0].batch and len(before[0].batch.vertices) == len(strip("a", XS).vertices)  # вход не тронут


def test_a_run_under_another_topology_law_or_without_materialized_domains_is_returned_as_it_is():
    older = tuple(_result(number, strip(name, XS, side=side), law="PLANAR_POLYGONS_V1") for number, name, side in ((0, "a", 1), (1, "b", -1)))
    refused = (ProductionDomainResultV1(patch_id=3, domain_id="domain:3", outcome="COVERAGE_FACE_LOST", batch=None),)

    assert dissolve_source_dots(older) == (older, None)
    assert dissolve_source_dots(refused) == (refused, None)


def test_a_refused_domain_keeps_its_place_in_the_results_and_the_materialized_ones_are_still_decided():
    refused = ProductionDomainResultV1(patch_id=2, domain_id="domain:2", outcome="COVERAGE_FACE_LOST", batch=None)
    one, other = _pair()

    results, found = dissolve_source_dots((one, refused, other))

    assert results[1] is refused and found.changed
    assert [item.patch_id for item in results] == [0, 2, 1]


def test_the_offset_normals_of_the_dissolved_vertices_leave_with_them():
    batch = strip("a", XS)
    normals = tuple((vertex.vert_key.value, (0.0, 0.0, 1.0)) for vertex in sorted(batch.vertices, key=lambda item: item.vert_key.value))

    results, _found = dissolve_source_dots((_result(0, batch, normals=normals),))

    (item,) = results
    assert {key for key, _normal in item.vertex_normals} == {vertex.vert_key.value for vertex in item.batch.vertices}
    assert item.offset_normals_digest and item.offset_normals_digest != _result(0, batch).offset_normals_digest


def test_an_exception_inside_the_law_leaves_the_results_as_they_were_and_names_the_outcome(monkeypatch, capsys):
    from cftuv import envelope_production_dots as dots
    from cftuv_envelope.materialize import source_dots

    def broken(*_args):
        raise RuntimeError("synthetic")

    monkeypatch.setattr(source_dots, "reconcile_source_dots", broken)
    before = _pair()

    results, found = dots.dissolve_source_dots(before)

    assert results == before and found.problems == (dots.OUTCOME_RAISED,) and not found.changed
    assert dots.OUTCOME_RAISED in capsys.readouterr().out


def test_the_slide_of_the_request_reaches_the_law():
    from fractions import Fraction

    skewed = (_result(0, strip("a", XS, skew={2: 0.003})),)

    assert dissolve_source_dots(skewed, Fraction(1, 1000))[1].changed is False
    assert dissolve_source_dots(skewed, Fraction(1, 100))[1].changed is True
