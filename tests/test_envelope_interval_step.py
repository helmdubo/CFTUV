"""Шаг ширины на хосте (без Blender): покрытие из шаблона внутри заверенного интервала не меняет ответа прогона.

Утверждений четыре, и каждое стоит на сверке «с быстрым путём и без него» (`CFTUV_INTERVAL_STEP=0`):

1. ОТВЕТ ТОТ ЖЕ. Серия ширин (вверх, вниз, повтор, недопустимые): результаты доменов (батчи, исходы, счётчики, дайджесты) равны, а
   различаются только метки пути (`step_path`) и счётчики шага ширины в профиле.
2. ПУТЬ НАЗВАН. Профиль называет попадания и полные счёты по причинам (`PRODUCTION_INTERVAL_FAST_HITS`,
   `PRODUCTION_INTERVAL_FALLBACK_<ПРИЧИНА>`); с выключателем все домены - `DISABLED`.
3. СВЕРКА МОЛЧИТ. С `CFTUV_INTERVAL_VERIFY=1` попаданий столько же, расхождений ноль.
4. МЕТКА НЕ ОТВЕТ. `step_path` в сравнение результата не входит и в квитанции домена записана.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_production_export as production  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import shutdown_domain_pool  # noqa: E402
from cftuv.envelope_production_export import run_production  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 4
#: Шаги по 0.4 %, назад и вперёд, повтор, шаг на 20 % и снова мелкие: большинство внутри интервала, один шаг за его границей.
WIDTHS = (0.25, 0.251, 0.252, 0.253, 0.252, 0.251, 0.25, 0.249, 0.248, 0.3, 0.301, 0.302)
SWITCH = "CFTUV_INTERVAL_STEP"
VERIFY = "CFTUV_INTERVAL_VERIFY"


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


def press(bundle, controller, alpha):
    return run_production(
        controller,
        bundle,
        frozenset(range(ROW)),
        alpha,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=0,
    )


def step_counters(run) -> dict:
    return {item.name: item.value for item in run.profile.counters if item.name.startswith("PRODUCTION_INTERVAL_") and item.patch_domain_id is None}


def test_the_answer_of_a_run_is_the_same_with_the_fast_step_and_without_it(monkeypatch):
    bundle = quad_row_bundle(ROW)
    fast, plain = EnvelopeDebugSessionController(), EnvelopeDebugSessionController()
    hits = 0
    for width in WIDTHS:
        monkeypatch.delenv(SWITCH, raising=False)
        left = press(bundle, fast, width)
        monkeypatch.setenv(SWITCH, "0")
        right = press(bundle, plain, width)
        assert tuple(left.results) == tuple(right.results), width
        assert [item.outcome for item in left.results] == [item.outcome for item in right.results]
        hits += step_counters(left).get(production.PRODUCTION_INTERVAL_FAST_HITS, 0)
        computed = [item for item in right.results if item.is_materialized and item.placement != production.PLACEMENT_CACHED]
        assert all(item.step_path == "FALLBACK:DISABLED" for item in computed), width
        assert step_counters(right).get(production.PRODUCTION_INTERVAL_FALLBACK_PREFIX + "DISABLED", 0) == len(computed), width
    assert hits >= 1, "a series of small widths on a prepared domain must hit the interval at least once"


def test_the_profile_names_the_paths_and_the_receipt_keeps_the_label(monkeypatch):
    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    monkeypatch.delenv(SWITCH, raising=False)
    first = press(bundle, controller, 0.25)
    second = press(bundle, controller, 0.251)
    counters = step_counters(second)
    assert counters.get(production.PRODUCTION_INTERVAL_VERIFY_MISMATCH) == 0
    paths = [item.step_path for item in second.results if item.is_materialized]
    assert paths and all(path == "FAST_HIT" or path.startswith("FALLBACK:") for path in paths), paths
    assert counters.get(production.PRODUCTION_INTERVAL_FAST_HITS, 0) == sum(1 for path in paths if path == "FAST_HIT")
    for item in first.results:
        assert item.step_path == "" or item.step_path.startswith("FALLBACK:") or item.step_path == "FAST_HIT"
    # Метка запуска не входит в сравнение результата.
    one = second.results[0]
    assert one == one.with_changes(step_path="FALLBACK:SOMETHING_ELSE")


def test_the_verification_switch_finds_no_mismatch_on_a_series(monkeypatch):
    bundle = quad_row_bundle(ROW)
    controller = EnvelopeDebugSessionController()
    monkeypatch.delenv(SWITCH, raising=False)
    monkeypatch.setenv(VERIFY, "1")
    mismatches = hits = 0
    for width in WIDTHS:
        run = press(bundle, controller, width)
        counters = step_counters(run)
        mismatches += counters.get(production.PRODUCTION_INTERVAL_VERIFY_MISMATCH, 0)
        hits += counters.get(production.PRODUCTION_INTERVAL_FAST_HITS, 0)
    assert mismatches == 0 and hits >= 1
