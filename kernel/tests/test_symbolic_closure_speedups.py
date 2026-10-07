"""Замыкание пакета скелета делает меньше проходов, а ответ тот же.

Каждый пункт здесь — удалённый проход чистой функции, и его проверка одна: ответ скелета (исход, число уровней,
семантический дайджест узлов, счётчики) с включённым проходом и без него побитово один и тот же. Цена — другое дело:
она падает и в ворота равенства ответа входит отдельной строкой (`SIGN_COUNTS`), а не ответом.

1. Повторный проход замыкания (`SYMBOLIC_SUPERLEVEL_REPEATED_CONTACT_SET_CHANGED_SIGNATURE`) — самопроверка
   детерминизма, а не закон. Продукт её не выполняет; набор тестов ядра выполняет всегда (`conftest.py`), а тест ниже
   доказывает на настоящих многоугольниках, что без неё ответ тот же.
"""

from __future__ import annotations


import pytest

from cftuv_envelope.wavefront import symbolic_superlevel_coordinator as outer
from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.polygon import PolygonV1
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome, build_skeleton
from wavefront_cases import named_corpus, partial_source_corpus
from weighted_wall_differential_cases import weighted_wall_differential_corpus

REPLAY_REASON = "SYMBOLIC_SUPERLEVEL_REPEATED_CONTACT_SET_CHANGED_SIGNATURE"


def _cases():
    named = dict(named_corpus()) | dict(partial_source_corpus())
    cases = [
        (name, named[name])
        for name in ("cross", "u_shape", "ell", "comb_2", "staircase_source_edges_3_4")
        if name in named
    ]
    cases.extend(
        (case.name, case.polygon)
        for case in weighted_wall_differential_corpus()
        if case.name in {"cross_full_q_1"}
    )
    # Два отказа пакета (антипараллельный шип и складка): отказ — такой же ответ, и он тоже не должен зависеть от прохода.
    cases.append(("diagonal_spike_notch", PolygonV1.build(
        ((0, -4), (4, 0), (0, 4), (-3, 1), (-3, -1), (-4, 0))
    )))
    cases.append(("diagonal_fold", PolygonV1.build(
        ((0, -2), (2, -1), (2, -2), (2, 0), (0, 0), (0, 2), (-2, 0))
    )))
    assert len(cases) >= 7, [name for name, _ in cases]
    return tuple(cases)


CASES = _cases()


def _answer(skeleton):
    return (
        skeleton.outcome,
        skeleton.levels,
        semantic_digest(skeleton),
        skeleton.counters,
    )


def _build(polygon, monkeypatch, *, replay):
    """(ответ, число проходов `plan_mixed_generations` за построение) при включённой либо выключенной самопроверке."""

    monkeypatch.setenv(outer.ENVIRONMENT_REPLAY_CHECK, "1" if replay else "0")
    real = outer.plan_mixed_generations
    calls = []

    def counted(*args, **kwargs):
        calls.append(1)
        return real(*args, **kwargs)

    monkeypatch.setattr(outer, "plan_mixed_generations", counted)
    skeleton = build_skeleton(polygon)
    return skeleton, len(calls)


@pytest.mark.parametrize("name,polygon", CASES, ids=[item[0] for item in CASES])
def test_the_closure_replay_changes_the_price_and_never_the_answer(name, polygon, monkeypatch):
    product, product_calls = _build(polygon, monkeypatch, replay=False)
    checked, checked_calls = _build(polygon, monkeypatch, replay=True)
    assert _answer(product) == _answer(checked), name
    # Самопроверка прошла: расхождения подписи нет ни на одном замыкании пакета.
    assert checked.counter(f"superlevel_unresolvable_reason::{REPLAY_REASON}") == 0
    assert product_calls >= 1, "случай не дошёл до замыкания пакета: он ничего не доказывает"
    assert checked_calls >= product_calls
    if product.outcome is SkeletonOutcome.EXACT:
        # Закрылось каждое замыкание, поэтому самопроверка повторила каждое ровно один раз.
        assert checked_calls == 2 * product_calls


def test_the_product_path_has_no_replay_flag_by_default(monkeypatch):
    monkeypatch.delenv(outer.ENVIRONMENT_REPLAY_CHECK, raising=False)
    assert outer.replay_check_enabled() is False
    for value, expected in (("1", True), ("on", True), ("true", True), ("0", False), ("", False), ("off", False)):
        monkeypatch.setenv(outer.ENVIRONMENT_REPLAY_CHECK, value)
        assert outer.replay_check_enabled() is expected, value


def test_the_kernel_suite_runs_with_the_replay_check_on():
    """`conftest.py` включает самопроверку для набора: её снимают тесты пути продукта явно, а не забывают."""

    import os

    assert os.environ.get(outer.ENVIRONMENT_REPLAY_CHECK) in ("1", "0")
    if os.environ.get(outer.ENVIRONMENT_REPLAY_CHECK) == "1":
        assert outer.replay_check_enabled() is True
