"""`materialize_domain(digests=False)`: дайджесты - чистые функции батча, и отложенные равны eager-путю побитово.

Живая ширина не читает дайджестов (ни семантического дайджеста батча, ни `content_digest`), а считали их на каждом домене
каждого шага (~18 % CPU домена). Отложить можно только то, что при чтении даёт то же значение, поэтому здесь держится
именно это: `finalize_digests` отложенного результата равен eager-результату по каждому полю ответа, а батч без дайджеста
не выдаёт `PENDING_SEMANTIC_DIGEST` за настоящий (валидатор по умолчанию его отвергает).
"""

from __future__ import annotations

import dataclasses

import pytest

from cftuv_envelope.canonical import PENDING_SEMANTIC_DIGEST, geometry_batch_semantic_digest, sealed_geometry_batch
from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.materialize.domain import finalize_digests, materialize_domain
from cftuv_envelope.validation import validate_geometry_batch

from test_materialize_domain import CASES, _run
from test_materialize_full_path import _strip_checks

LAWS = (DecalTopologyLawV1.TRIANGLES_V1, DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)
ANSWER_FIELDS = (
    "outcome",
    "detail",
    "counters",
    "diagnostics",
    "vertex_normals",
    "offset_normal_law",
    "offset_normals_digest",
    "decal_topology_law",
)


def _assert_the_deferred_result_finalizes_to_the_eager_answer(name, law):
    eager = _run(name, decal_topology_law=law)
    lazy = _run(name, decal_topology_law=law, digests=False)

    assert eager.outcome is MaterializationOutcome.MATERIALIZED, eager.detail
    assert lazy.digests_deferred and not eager.digests_deferred
    assert lazy.content_digest == "" and lazy.batch.semantic_digest.value == PENDING_SEMANTIC_DIGEST
    # Всё, кроме дайджестов, готово сразу и равно eager-ответу.
    for field in ANSWER_FIELDS:
        assert getattr(lazy, field) == getattr(eager, field), field
    assert dataclasses.replace(lazy.batch, semantic_digest=eager.batch.semantic_digest) == eager.batch

    finalized = finalize_digests(lazy)
    assert finalized.batch == eager.batch
    assert finalized.batch.semantic_digest == eager.batch.semantic_digest
    assert finalized.content_digest == eager.content_digest
    assert canonical_json_bytes(finalized.batch) == canonical_json_bytes(eager.batch)
    assert not finalized.digests_deferred


@pytest.mark.parametrize("law", LAWS)
@pytest.mark.parametrize("name", CASES)
def test_a_deferred_result_finalizes_to_the_eager_answer(name, law):
    _assert_the_deferred_result_finalizes_to_the_eager_answer(name, law)


@pytest.mark.parametrize("order", ("full_path_then_deferred", "deferred_then_full_path"))
def test_the_full_path_and_the_deferred_digests_agree_in_either_order_in_one_process(order):
    """Регрессия порядка: шесть отложенных сравнений падали, когда `test_materialize_full_path` шёл раньше в том же процессе.

    Причиной была процессная память недавних покрытий (цена попадания и промаха различалась); её больше нет, контуры едут в
    записи региона, и цена не зависит от порядка. Здесь оба пути идут в одном процессе в обоих порядках, чтобы порядок файлов не
    прятал зависимость, если она вернётся.
    """

    def full_path():
        for name in CASES:
            _strip_checks(name)

    def deferred():
        for name in CASES:
            _assert_the_deferred_result_finalizes_to_the_eager_answer(name, LAWS[0])

    steps = {"full_path": full_path, "deferred": deferred}
    for step in order.split("_then_"):
        steps[step]()


@pytest.mark.parametrize("name", CASES[:3])
def test_the_validator_rejects_a_pending_digest_unless_told_the_digest_is_deferred(name):
    lazy = _run(name, digests=False)
    pending = lazy.batch

    assert validate_geometry_batch(pending, check_semantic_digest=False) == ()
    assert any(issue.path == ("semantic_digest",) for issue in validate_geometry_batch(pending))
    assert validate_geometry_batch(sealed_geometry_batch(pending)) == ()
    assert sealed_geometry_batch(pending).semantic_digest.value == geometry_batch_semantic_digest(pending).sha256_hex


def test_finalizing_leaves_eager_results_and_refusals_alone():
    eager = _run(CASES[0])
    assert finalize_digests(eager) is eager

    refused = dataclasses.replace(eager, outcome=MaterializationOutcome.BATCH_DID_NOT_VALIDATE, batch=None, content_digest="")
    assert finalize_digests(dataclasses.replace(refused, digests_deferred=True)) == dataclasses.replace(refused, digests_deferred=True)


def test_the_deferred_flag_follows_the_argument_and_the_default_stays_eager():
    assert _run(CASES[0]).digests_deferred is False
    assert _run(CASES[0], digests=True).content_digest != ""
    assert _run(CASES[0], digests=False).digests_deferred is True
    assert "digests" in materialize_domain.__code__.co_varnames
