"""Подготовка очереди пересылается между процессами и отвечает так же.

Зачем это нужно. Домены очереди независимы по ключу исполнения
`(DecalRequestId, PatchDomainId)`, и хост считает их пулом процессов (срез
PARALLEL-DOMAINS). Результат домена — `ConveyorPreparationV1` — обязан доехать
обратно в процесс хоста: по нему же считается покрытие при другой alpha
(ползунок), поэтому одного ответа `EnvelopeQueueDomainV1` мало.

Что мешало. `ExactPlanarMetric._density_exact_memo.intervals` держит интервалы
`mpmath` (`ivmpf`), которые не пиклятся, и вместе с ними не пиклилась вся
подготовка. Memo — кэш, а не часть значения: `_DensityExactMemo.__reduce__`
пересылает его пустым.

Что проверяется, и почему именно так:

1. препятствие НАСТОЯЩЕЕ — memo засеян настоящим интервалом, и сам он не
   пиклится (иначе тест прошёл бы и без починки: на малых фикстурах memo
   интервалов не накапливает — замерено, см. отпечаток ниже);
2. подготовка с засеянным memo пиклится, а на выходе memo ПУСТОЙ;
3. покрытие копии при двух разных alpha совпадает с покрытием оригинала
   ТОЧНО — по исходу, площадям (`SqrtSumV1` канонична), владельцам, счётчикам;
4. два ответа различаются между собой: иначе равенство ничего не различало бы.
"""

from __future__ import annotations

import pickle

import pytest
import sympy as sp
from mpmath import iv

import cftuv_envelope as kernel
from cftuv_envelope._density_policy import density_interval_enclosure
from cftuv_envelope.wavefront import (
    ConveyorOutcome,
    conveyor_coverage,
    prepare_conveyor,
)

from wavefront_cases import FIELD_FIXTURE


ALPHAS = ("0.25", "0.5")


def _field_preparation():
    root = FIELD_FIXTURE.parent
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    return prepare_conveyor(snapshot, request)


def _seed_interval(memo) -> None:
    """Кладёт в memo настоящий интервал `mpmath`, как это делает плотность."""

    saved = iv.prec
    iv.prec = 160
    try:
        density_interval_enclosure(sp.sqrt(2) + sp.sqrt(3), memo.intervals)
    finally:
        iv.prec = saved


def _answer(coverage):
    """То, что считает ядро: без секунд и без ссылки на саму подготовку."""

    return (
        coverage.outcome,
        coverage.alpha,
        coverage.lattice_alpha,
        coverage.doubled_area,
        coverage.polygon_doubled_area,
        coverage.counters,
        coverage.detail,
        tuple(
            (
                region.region_id,
                region.outcome,
                region.doubled_area,
                region.polygon_doubled_area,
                region.wall_spans,
                tuple(
                    (
                        face.region_id,
                        face.owner,
                        face.envelope_spec_id,
                        face.envelope_instance_id,
                        face.doubled_area,
                    )
                    for face in region.faces
                ),
            )
            for region in coverage.regions
        ),
    )


def _memo_entries(prepared) -> int:
    memo = prepared.context.metric._density_exact_memo
    return sum(len(getattr(memo, slot)) for slot in memo.__slots__)


def test_the_obstacle_is_a_real_interval_in_the_memo_and_not_the_preparation():
    prepared = _field_preparation()
    memo = prepared.context.metric._density_exact_memo
    _seed_interval(memo)

    assert memo.intervals
    with pytest.raises(pickle.PicklingError):
        pickle.dumps(memo.intervals, protocol=pickle.HIGHEST_PROTOCOL)


def test_a_preparation_crosses_a_process_boundary_empty_memo_and_same_answers():
    prepared = _field_preparation()
    assert prepared.outcome is ConveyorOutcome.EXACT, prepared.detail
    _seed_interval(prepared.context.metric._density_exact_memo)
    assert _memo_entries(prepared) > 0

    before = {
        alpha: _answer(conveyor_coverage(prepared, alpha)) for alpha in ALPHAS
    }
    assert all(item[0] is ConveyorOutcome.EXACT for item in before.values())

    clone = pickle.loads(
        pickle.dumps(prepared, protocol=pickle.HIGHEST_PROTOCOL)
    )

    assert clone is not prepared
    # Копия пришла с ПУСТЫМ memo, а оригинал своё сохранил: кэш — не значение.
    assert _memo_entries(clone) == 0
    assert _memo_entries(prepared) > 0
    assert clone.outcome is prepared.outcome
    assert clone.counters == prepared.counters
    assert clone.law_names == prepared.law_names
    after = {
        alpha: _answer(conveyor_coverage(clone, alpha)) for alpha in ALPHAS
    }
    assert after == before
    # Контроль различения: разные alpha дают РАЗНЫЕ ответы, поэтому равенство
    # выше не выполнялось бы, перепутай копия alpha или потеряй владельцев.
    assert before[ALPHAS[0]] != before[ALPHAS[1]]


def test_an_unpickled_memo_is_usable_and_refills():
    prepared = _field_preparation()
    clone = pickle.loads(
        pickle.dumps(prepared, protocol=pickle.HIGHEST_PROTOCOL)
    )
    memo = clone.context.metric._density_exact_memo

    _seed_interval(memo)

    assert memo.intervals
    assert _memo_entries(clone) > 0
