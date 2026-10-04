"""Идентичности хоста, привязанные к ревизии: запись вывода токена и ТОЧНОЕ переименование.

Результат домена, посчитанный при одной ревизии, переносится на другую только если все идентичности хоста в
нём переписываются точно. Точность здесь доказывается без ядра: токены, записанные при ревизии A и
переписанные на ревизию B, равны токенам, которые выдаёт ТА ЖЕ выгрузка снапшота, запущенная на меше при
ревизии B (с другим id запроса и другим номером патча). Токен вне записи не переименовывается молча.
"""

from __future__ import annotations

import hashlib
import json
import sys
import threading
from fractions import Fraction
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv.envelope_host_labels import (  # noqa: E402
    PATCH_SCOPED_KINDS,
    REQUEST_SCOPED_KINDS,
    DomainLabelingV1,
    HostTokenV1,
    LabelMapV1,
    RelabelIncomplete,
    record_host_tokens,
    relabeled,
    stable_token,
    token_hash,
    typed_id,
)
from cftuv.envelope_metric_export import build_envelope_patch_metric_export  # noqa: E402
from cftuv.envelope_request_export import (  # noqa: E402
    EnvelopeHostAdapterError,
    _stable_token,
    _typed_value,
    build_envelope_decal_request,
)
from cftuv.envelope_topology_export import (  # noqa: E402
    build_envelope_topology_export,
    stage_domain_inputs,
)
from content_fixtures import renumbered, with_revision  # noqa: E402
from envelope_fixture_bundles import (  # noqa: E402
    bundle_from_exported_snapshot,
    host_exported_snapshot_paths,
    quad_row_bundle,
    square_hole_bundle,
    u_route_bundle,
)
from surface_adjacency_field_corpus import load_snapshot  # noqa: E402

ROW = 5
EDGES = frozenset(range(ROW))

#: Виды токенов, чьи части НЕ начинаются с номера патча домена. Новый вид, которого нет ни здесь, ни в
#: `PATCH_SCOPED_KINDS`, роняет тест: решение «патчевой или нет» принимается явно.
NOT_PATCH_SCOPED = frozenset(
    {
        "decal-request",
        "decal-request-density",
        "decal-request-reach",
        "decal-request-stretch",
        "metric-source",
        "physical-chain",
        "physical-lineage",
    }
)


def _reference_token(kind, revision, *parts):
    """Прежняя реализация `_stable_token`, дословно: что хэшировалось до записи токенов."""

    payload = json.dumps(
        (kind, revision, parts), ensure_ascii=False, sort_keys=True, separators=(",", ":")
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:24]


def _chain_edges(bundle):
    """Рёбра ВСЕХ цепочек хоста: выделение целиком, годное любому мешу корпуса."""

    topology = build_envelope_topology_export(bundle)
    return frozenset(edge for record in topology.host_chains for edge in record.canonical_edge_ids)


def _recorded(bundle, patch_id, edges=EDGES, density="2", budget=None, reach=None):
    """`DomainLabelingV1` выгрузки снапшота и запроса домена, как её пишет воркер.

    `budget` и `reach` — допуск растяжения и досягаемость полосовой карты запроса: каждый добавляет к id запроса
    свой производный токен (`decal-request-stretch`, `decal-request-reach`), и цепочка переименовывается по порядку записи.
    """

    topology = build_envelope_topology_export(bundle)
    _scene, revision, _ids, request_id, by_domain = stage_domain_inputs(
        bundle, edges, topology_export=topology
    )
    domain_id = topology.patch_domain_id_by_patch[patch_id]
    with record_host_tokens() as log:
        snapshot = build_envelope_patch_metric_export(topology, patch_id).snapshot
        build_envelope_decal_request(
            snapshot,
            frozenset(by_domain[domain_id]),
            0.25,
            decal_request_id_value=request_id,
            density=density,
            developable_stretch_budget=budget,
            chart_reach_cap=reach,
        )
    return log.labeling(revision, request_id, patch_id)


# --------------------------------------------------------------------------
# Хэш и запись
# --------------------------------------------------------------------------


def test_the_token_hash_is_the_one_the_host_always_used():
    cases = (
        ("patch-domain", "host-source:abc:name", 3),
        ("chain-use", "host-source:abc:имя", 0, 1, 2, (4, 5, 6)),
        ("physical-chain", "r", True, (1, 2), (3, 4)),
    )
    for kind, revision, *parts in cases:
        expected = _reference_token(kind, revision, *parts)
        assert stable_token(kind, revision, *parts) == expected
        assert token_hash(kind, revision, tuple(parts)) == expected
        assert _stable_token(kind, revision, *parts) == expected
        assert _typed_value(kind, revision, *parts) == typed_id(kind, expected) == f"host-v0:{kind}:{expected}"


def test_a_token_is_recorded_with_its_derivation_only_inside_the_block():
    stable_token("outside", "r", 1)
    with record_host_tokens() as log:
        first = stable_token("patch-domain", "r", 3)
        again = stable_token("patch-domain", "r", 3)
        second = stable_token("chain-use", "r", 3, 0, 1, (7, 8))
    stable_token("outside", "r", 2)

    labeling = log.labeling("r", "request", 3)
    assert first == again
    assert [(item.kind, item.parts, item.token) for item in labeling.tokens] == [
        ("patch-domain", (3,), first),
        ("chain-use", (3, 0, 1, (7, 8)), second),
    ]
    assert labeling.has_domain_token()
    assert not DomainLabelingV1("r", "q", 0, ()).has_domain_token()


def test_recording_nests_and_does_not_leak_between_threads():
    seen = {}

    def other():
        with record_host_tokens() as inner:
            stable_token("thread", "r", 1)
        seen["tokens"] = [item.kind for item in inner.labeling("r", "q", 0).tokens]

    with record_host_tokens() as outer:
        stable_token("outer", "r", 1)
        worker = threading.Thread(target=other)
        worker.start()
        worker.join()
        with record_host_tokens() as nested:
            stable_token("nested", "r", 1)
        stable_token("outer", "r", 2)

    assert seen["tokens"] == ["thread"]
    assert [item.kind for item in nested.labeling("r", "q", 0).tokens] == ["nested"]
    assert [item.kind for item in outer.labeling("r", "q", 0).tokens] == ["outer", "outer"]


# --------------------------------------------------------------------------
# Переименование: равно холодной выгрузке на новой ревизии
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "shift,budget,reach",
    (
        (0, None, None),
        (3, None, None),
        # BAND-CHART C1: производный токен досягаемости (после токена допуска) переименовывается тем же законом.
        (0, None, Fraction(3, 4)),
        (3, Fraction(1, 4), Fraction(3, 4)),
    ),
)
def test_relabeled_tokens_equal_the_tokens_of_a_cold_export_at_the_target(shift, budget, reach):
    """Ревизия, id запроса и номер патча меняются вместе: токены те же, что выдала бы выгрузка на цели."""

    source = with_revision(quad_row_bundle(ROW), "row")
    target = with_revision(
        renumbered(source, {patch: patch + shift for patch in range(ROW)}) if shift else source,
        "another-object",
        "e" * 64,
    )
    patch_from, patch_to = 2, 2 + shift
    before = _recorded(source, patch_from, budget=budget, reach=reach)
    cold = _recorded(target, patch_to, budget=budget, reach=reach)
    if reach is not None:
        assert "decal-request-reach" in {item.kind for item in before.tokens}

    moved, mapper = relabeled(before, cold.revision, cold.request_id, patch_to)

    assert moved.revision == cold.revision and moved.patch_id == patch_to
    assert [(item.kind, item.token) for item in moved.tokens] == [
        (item.kind, item.token) for item in cold.tokens
    ]
    assert [(item.revision, item.parts) for item in moved.tokens] == [
        (item.revision, item.parts) for item in cold.tokens
    ]
    # Карта строк знает каждый записанный токен и переписывает строки идентичностей так же.
    assert all(mapper.tokens[old.token] == new.token for old, new in zip(before.tokens, moved.tokens))
    host_vertex = f"host-vertex:{before.revision}:5"
    assert mapper(host_vertex) == f"host-vertex:{cold.revision}:5"
    assert mapper(f"host-patch:{before.revision}:2") == f"host-patch:{cold.revision}:{patch_to}"
    assert mapper(before.request_id) == cold.request_id


def test_relabeling_to_the_same_place_changes_nothing():
    labeling = _recorded(with_revision(quad_row_bundle(ROW), "row"), 1)

    moved, _mapper = relabeled(labeling, labeling.revision, labeling.request_id, labeling.patch_id)

    assert moved == labeling


def test_the_patch_literal_does_not_touch_a_longer_number():
    labeling = DomainLabelingV1("R", "Q", 1, ())

    mapper = LabelMapV1(labeling, "R2", "Q2", 7, {})

    assert mapper("host-patch:R:1") == "host-patch:R2:7"
    assert mapper("host-patch:R:12") == "host-patch:R2:12"
    assert mapper("a host-patch:R:1, b host-patch:R:12") == "a host-patch:R2:7, b host-patch:R2:12"


# --------------------------------------------------------------------------
# Остаток называется, а не переименовывается молча
# --------------------------------------------------------------------------


def test_a_host_token_outside_the_record_is_named_not_renamed():
    labeling = DomainLabelingV1("R", "Q", 0, (HostTokenV1("patch-domain", "R", (0,), "a" * 24),))
    _moved, mapper = relabeled(labeling, "R2", "Q2", 0)

    assert mapper("host-v0:patch-domain:" + "a" * 24) == "host-v0:patch-domain:" + _moved.tokens[0].token
    with pytest.raises(RelabelIncomplete):
        mapper("host-v0:chain-use:" + "b" * 24)
    with pytest.raises(RelabelIncomplete):
        mapper("chain-source:host-patch:R:0:" + "c" * 24)
    # Токен вида, не принадлежащего ревизии (id запроса прогона), подставляется литерально, а не по записи.
    assert mapper("host-v0:decal-request:" + "d" * 24) == "host-v0:decal-request:" + "d" * 24


def test_every_host_id_prefix_names_its_unrecorded_token():
    """`host-debug-diagnostic:<токен>` и любой другой префикс `host-<имя>:` с токеном — не молчаливый остаток."""

    token = "a" * 24
    recorded = DomainLabelingV1("R", "Q", 0, (HostTokenV1("patch-domain", "R", (0,), token),))
    _moved, mapper = relabeled(recorded, "R2", "Q2", 0)

    assert mapper(f"host-debug-diagnostic:{token}") == f"host-debug-diagnostic:{_moved.tokens[0].token}"
    with pytest.raises(RelabelIncomplete, match="host-debug-diagnostic"):
        mapper("host-debug-diagnostic:" + "e" * 24)
    with pytest.raises(RelabelIncomplete, match="host-anything-new"):
        mapper("host-anything-new:" + "f" * 24)
    # Идентичности с ревизией и индексом токена не несут и остатком не называются.
    digest = "9" * 64
    for text in (
        f"host-vertex:host-source:{digest}:name:5",
        f"host-edge:host-source:{digest}:name:7",
        f"host-source:{digest}:name",
        f"host-face:R:3",
    ):
        mapper(text)


def test_a_patch_token_that_does_not_start_with_the_patch_of_the_domain_is_refused():
    labeling = DomainLabelingV1("R", "Q", 2, (HostTokenV1("chain-use", "R", (9, 0, 0, ()), "a" * 24),))

    with pytest.raises(RelabelIncomplete):
        relabeled(labeling, "R2", "Q2", 5)


# --------------------------------------------------------------------------
# Перечень патчевых видов полон
# --------------------------------------------------------------------------


def _field_bundles():
    bundles = [with_revision(quad_row_bundle(ROW, lifted_corner=1.0), "row")]
    bundles.append(with_revision(square_hole_bundle(), "hole"))
    bundles.append(with_revision(u_route_bundle(), "u-route"))
    for path in host_exported_snapshot_paths()[:6]:
        bundles.append(bundle_from_exported_snapshot(load_snapshot(path.parent))[0])
    return bundles


def test_every_recorded_kind_is_declared_patch_scoped_or_not():
    seen = {}
    for bundle in _field_bundles():
        for patch_id in sorted(bundle.patch_graph.nodes):
            try:
                labeling = _recorded(
                    bundle, patch_id, edges=_chain_edges(bundle), budget=Fraction(1, 4), reach=Fraction(3, 4)
                )
            except EnvelopeHostAdapterError:
                continue  # домен, который хост отвергает на выгрузке, токенов результата не даёт
            for item in labeling.tokens:
                seen.setdefault(item.kind, []).append((item.parts, patch_id))

    assert set(seen) <= PATCH_SCOPED_KINDS | NOT_PATCH_SCOPED, sorted(set(seen) - PATCH_SCOPED_KINDS - NOT_PATCH_SCOPED)
    for kind in PATCH_SCOPED_KINDS & set(seen):
        assert all(parts and parts[0] == patch_id for parts, patch_id in seen[kind]), kind
    assert REQUEST_SCOPED_KINDS <= NOT_PATCH_SCOPED
    # Перечни не пересекаются, а виды выгрузки патчевого домена в корпусе действительно встречены.
    assert not PATCH_SCOPED_KINDS & NOT_PATCH_SCOPED
    assert {"patch-domain", "chain-use", "boundary-loop", "owner-sector"} <= set(seen)
    assert {"decal-request-stretch", "decal-request-reach"} <= set(seen)



def test_the_host_issues_its_typed_ids_in_one_place_and_through_the_recorded_hash():
    """Перенос результата точен, пока КАЖДЫЙ токен хоста выдан через `stable_token` (и потому записан).

    Токен, выданный мимо записи, в переносимом результате назван остатком (`RelabelIncomplete`), но
    лучше не дать ему появиться: `host-v0:` собирает ровно одна функция, а хэш у неё один.
    """

    package = Path(__file__).resolve().parents[1] / "cftuv"
    issuing = []
    for path in sorted(package.glob("*.py")):
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            code = line.split("#", 1)[0]
            if 'f"host-v0:' in code or "f'host-v0:" in code or '"host-v0:" +' in code:
                issuing.append((path.name, number))
    assert [name for name, _ in issuing] == ["envelope_host_labels.py"], issuing
    source = (package / "envelope_request_export.py").read_text(encoding="utf-8")
    assert "return stable_token(kind, revision, *parts)" in source
    assert "return typed_id(kind, _stable_token(kind, revision, *parts))" in source
    assert "hashlib" not in source
