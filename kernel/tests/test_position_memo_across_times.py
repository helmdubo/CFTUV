"""Память мест держится через точные времена ОДНОЙ сборки скелета, и цена домена от этого не становится историей.

Правило цены (одна строка DECISIONS 2026-10-07, SKELETON-POSITION-MEMO; решение владельца): память точных мест
(`PositionMemoV1`) больше не стирается при смене точного времени. Место на будущее время гидратирует закон кандидата, а на
своём уровне его спрашивает снимок суперуровня; раньше смена времени стирала запись, и то же место платилось заново.
Меняется ровно одна статья цены, `EXACT_WORK_EXACT_POSITION_HYDRATIONS`, и она падает по построению. Ответ не меняется:
ключ записи несёт `t`, значение не зависит от возраста записи.

Память живёт ВНУТРИ вычисления: её владелец — строитель одного скелета. Это не кэш между вычислениями, поэтому цена домена
остаётся функцией его входа: холодная, тёплая, повторная сборка и сборка в свежем процессе платят одно и то же.

Ничего не принимается на веру. Прежнее правило (чистить при смене времени) воспроизведено здесь как ЭТАЛОН СРАВНЕНИЯ
(`Build(clear_at_each_time=True)`): тот же строитель, тот же вход, и единственное отличие — стёртая память. Эталон нужен и
как доказательство, что проверки не пусты: у него есть то нарушение, которого настоящий строитель не допускает.
"""

from __future__ import annotations

import functools
import gc
import json
import os
import subprocess
import sys
import weakref
from pathlib import Path

import pytest

from cftuv_envelope.exact_sqrt_sum import (
    exact_work_budget,
    reset_factorization_memory,
    set_canonical_audit,
)
from cftuv_envelope.wavefront import exact_candidate_view as view_module
from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront.event_time import EventTimeV1
from cftuv_envelope.wavefront.exact_candidate_view import PositionMemoV1
from cftuv_envelope.wavefront.skeleton import SplitSearch, build_skeleton

from shadow_axes import skeleton_axes
from wavefront_cases import named_corpus, partial_source_corpus
from weighted_wall_differential_cases import weighted_wall_differential_corpus

HERE = Path(__file__).resolve().parent
KERNEL_SOURCE = HERE.parent / "src"

CORPUS = (
    tuple(named_corpus())
    + tuple(partial_source_corpus())
    + tuple((case.name, case.polygon) for case in weighted_wall_differential_corpus())
)
POLYGONS = dict(CORPUS)
NAMES = tuple(name for name, _ in CORPUS)

#: Порядок статей в `ExactWorkBudgetV1.spent_by_article()`: пять статей факторизации и гидратации мест шестая.
HYDRATIONS = 5


class Build:
    """Одна сборка скелета: ответ, цена по шести статьям, память мест, как она осталась.

    `clear_at_each_time=True` воспроизводит УДАЛЁННОЕ правило: перед первым уровнем каждого нового точного времени память
    стирается. Все прочие движения строителя те же.
    """

    def __init__(self, polygon, *, clear_at_each_time: bool = False):
        reset_factorization_memory()
        self.budget = exact_work_budget(stage="TEST", domain_id="position-memo")
        builder = skeleton_module._Builder(polygon, SplitSearch.MOTORCYCLE, work_budget=self.budget)
        if clear_at_each_time:
            apply_level = builder._apply_level
            last = []

            def apply_level_clearing(level):
                if not last or builder.now != last[0]:
                    builder._position_memo.entries.clear()
                    last[:] = [builder.now]
                return apply_level(level)

            builder._apply_level = apply_level_clearing
        self.skeleton = builder.run()
        self.price = self.budget.spent_by_article()
        self.entries = builder._position_memo.entries

    def exact_times_of_places(self):
        """Различные точные времена, на которые в памяти остались места (ключ места - `(прямая, прямая, скольжение, t)`)."""

        return {
            key[3]
            for key in self.entries
            if isinstance(key, tuple) and len(key) == 4 and isinstance(key[3], EventTimeV1)
        }


@functools.lru_cache(maxsize=None)
def built(name: str, clear_at_each_time: bool) -> Build:
    return Build(POLYGONS[name], clear_at_each_time=clear_at_each_time)


@pytest.fixture(autouse=True)
def _audited_cold_state():
    set_canonical_audit(True)
    reset_factorization_memory()
    yield
    reset_factorization_memory()


def test_the_corpus_is_eighty_six_cases():
    """Размер корпуса - часть ворот: молча усохший корпус не ворота."""

    assert len(CORPUS) == len(POLYGONS) == 86


# ---------------------------------------------------------------- ответ: память через времена его не меняет


@pytest.mark.parametrize("name", NAMES)
def test_keeping_the_memo_across_exact_times_changes_no_answer(name):
    kept, cleared = built(name, False), built(name, True)
    assert skeleton_axes(kept.skeleton) == skeleton_axes(cleared.skeleton), name


# ---------------------------------------------------------------- цена: меняется одна статья, и вниз


@pytest.mark.parametrize("name", NAMES)
def test_keeping_the_memo_changes_only_the_hydration_article_and_never_up(name):
    kept, cleared = built(name, False).price, built(name, True).price
    assert kept[:HYDRATIONS] == cleared[:HYDRATIONS], (name, kept, cleared)
    assert kept[HYDRATIONS] <= cleared[HYDRATIONS], (name, kept, cleared)


def test_the_hydration_price_drops_on_the_corpus_by_design():
    """Правило цены - не слово: на корпусе статья гидратации падает в большинстве случаев и в сумме, прочие пять статей те же."""

    kept = [built(name, False).price for name in NAMES]
    cleared = [built(name, True).price for name in NAMES]
    lower = sum(1 for a, b in zip(kept, cleared) if a[HYDRATIONS] < b[HYDRATIONS])
    assert lower * 2 > len(NAMES), (lower, len(NAMES))
    assert sum(a[HYDRATIONS] for a in kept) < sum(b[HYDRATIONS] for b in cleared)
    for article in range(HYDRATIONS):
        assert sum(a[article] for a in kept) == sum(b[article] for b in cleared), article


# ---------------------------------------------------------------- сама память: она действительно переживает смену времени


@pytest.mark.parametrize("name", ("cross", "cross_full_q_1/4", "mirror_direct", "weighted_holes_2"))
def test_the_finished_memo_holds_places_of_more_than_one_exact_time(name):
    """Настоящий строитель, без подмен: к концу сборки в памяти места с РАЗНЫХ точных времён (эталон со стиранием - не больше одного)."""

    assert len(built(name, False).exact_times_of_places()) > 1, name
    assert len(built(name, True).exact_times_of_places()) <= 1, name


def test_most_of_the_corpus_keeps_places_of_several_exact_times():
    several = sum(1 for name in NAMES if len(built(name, False).exact_times_of_places()) > 1)
    assert several >= 50, several


def test_the_memo_never_holds_more_places_than_the_build_computed(monkeypatch):
    """Память растёт не быстрее работы: каждое хранимое место оплачено ровно одной гидратацией этой же сборки."""

    calls = []
    original = view_module._hydrate_position

    def counting(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)

    monkeypatch.setattr(view_module, "_hydrate_position", counting)
    for name in ("cross", "cross_full_q_1/4", "weighted_holes_2", "same_time_weighted_collapse"):
        del calls[:]
        reset_factorization_memory()
        budget = exact_work_budget(stage="TEST", domain_id=name)
        builder = skeleton_module._Builder(POLYGONS[name], SplitSearch.MOTORCYCLE, work_budget=budget)
        builder.run()
        assert builder._position_memo.admits(builder._prime_universe)
        places = sum(
            1
            for key in builder._position_memo.entries
            if isinstance(key, tuple) and len(key) == 4 and isinstance(key[3], EventTimeV1)
        )
        assert places == len(calls) > 0, name


# ---------------------------------------------------------------- память внутри вычисления, а не кэш между вычислениями


def test_the_memo_dies_with_its_build(monkeypatch):
    """Память строится заново на каждую сборку и вместе со строителем уходит: никакой реестр её не держит, и скелет её не несёт."""

    class Watched(PositionMemoV1):
        pass

    created = []

    def make(prime_universe):
        memo = Watched(prime_universe)
        created.append(weakref.ref(memo))
        return memo

    monkeypatch.setattr(skeleton_module, "PositionMemoV1", make)
    polygon = POLYGONS["cross"]
    first = build_skeleton(polygon)
    second = build_skeleton(polygon)
    gc.collect()
    assert len(created) == 2
    assert all(reference() is None for reference in created), "the memo outlived its build"
    assert skeleton_axes(first) == skeleton_axes(second)


def test_a_second_build_inherits_nothing_from_the_first_and_pays_the_same():
    """Вторая сборка того же входа получает свою память: ни одного места от первой, та же цена, тот же ответ."""

    polygon = POLYGONS["cross_full_q_1/4"]
    memos, prices, answers, at_start = [], [], [], []
    for _ in range(2):
        reset_factorization_memory()
        budget = exact_work_budget(stage="TEST", domain_id="again")
        builder = skeleton_module._Builder(polygon, SplitSearch.MOTORCYCLE, work_budget=budget)
        at_start.append(len(builder._position_memo.entries))
        memos.append(builder._position_memo)
        answers.append(skeleton_axes(builder.run()))
        prices.append(budget.spent_by_article())
    assert memos[0] is not memos[1]
    assert at_start[0] == at_start[1], "the second build started with places of the first"
    assert len(memos[0].entries) > at_start[0]
    assert prices[0] == prices[1]
    assert answers[0] == answers[1]


CHILD = """
import json, sys
from cftuv_envelope.exact_sqrt_sum import exact_work_budget, reset_factorization_memory, set_canonical_audit
from cftuv_envelope.wavefront.skeleton import build_skeleton
from wavefront_cases import named_corpus, partial_source_corpus
from weighted_wall_differential_cases import weighted_wall_differential_corpus

set_canonical_audit(True)
corpus = dict(named_corpus()) | dict(partial_source_corpus()) | {c.name: c.polygon for c in weighted_wall_differential_corpus()}
prices = {}
for name in json.loads(sys.argv[1]):
    reset_factorization_memory()
    budget = exact_work_budget(stage="TEST", domain_id="child")
    build_skeleton(corpus[name], work_budget=budget)
    prices[name] = budget.spent_by_article()
print("PRICES " + json.dumps(prices))
"""


def test_the_price_is_the_same_cold_warm_and_in_a_fresh_process():
    """Холодная, тёплая (после чужих сборок, в обратном порядке) и сборка в свежем процессе платят одну цену.

    Тёплая сборка не сбрасывает память факторизации процесса (статьи факторизации она законно меняет, и их сбрасывает
    каждый домен конвейера), поэтому у неё сверяется статья гидратации: она от истории процесса зависеть не вправе. Холодная
    и свежий процесс сверяются по ВСЕМ шести статьям.
    """

    names = NAMES
    cold = {}
    for name in names:
        reset_factorization_memory()
        budget = exact_work_budget(stage="TEST", domain_id="cold")
        build_skeleton(POLYGONS[name], work_budget=budget)
        cold[name] = budget.spent_by_article()
    for name in reversed(names):
        budget = exact_work_budget(stage="TEST", domain_id="warm")
        build_skeleton(POLYGONS[name], work_budget=budget)
        assert budget.spent_by_article()[HYDRATIONS] == cold[name][HYDRATIONS], name
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join([str(KERNEL_SOURCE), str(HERE)])
    completed = subprocess.run(
        [sys.executable, "-c", CHILD, json.dumps(list(names))],
        capture_output=True,
        text=True,
        env=environment,
        timeout=300,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr[-2000:]
    (line,) = [row for row in completed.stdout.splitlines() if row.startswith("PRICES ")]
    fresh = {name: tuple(value) for name, value in json.loads(line[len("PRICES "):]).items()}
    assert fresh == cold
