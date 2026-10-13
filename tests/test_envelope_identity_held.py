"""Два дешёвых места родителя холодной кнопки (PARENT_PACK_2, рычаг B): вопрос «держит ли кэш эту подготовку» за O(1) и отпечаток источника, взятый один раз.

1. `IdentityHeldDict.holds` равен прежнему ответу `any(item is value for item in cache.values())` после ЛЮБОЙ последовательности записей, замен, удалений, `pop`, `popitem`, `setdefault`, `update`, `|=` и `clear`
   (случайные последовательности против эталонного словаря); значение под двумя ключами держится, пока не уйдут оба; равное, но не тождественное значение не считается;
2. `_forget_preparation_blob` сессии решает тем же вопросом и не обходит значения кэша (ни цикла, ни генератора в теле); поведение то же: пикл убирается, когда подготовку не держит кэш сессии;
3. `build_analysis_bundle(..., source_revision=...)` берёт отпечаток вызывающего и не берёт второй; без него берёт свой, как прежде.
"""

from __future__ import annotations

import ast
import random
import sys
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import analysis_surface as surface_module  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_identity_held import IdentityHeldDict  # noqa: E402

HOST = Path(__file__).resolve().parents[1] / "cftuv"


class _Item:
    """Значение кэша: равные по `==` объекты не тождественны."""

    def __init__(self, tag: int) -> None:
        self.tag = tag

    def __eq__(self, other) -> bool:
        return isinstance(other, _Item) and other.tag == self.tag

    def __hash__(self) -> int:
        return hash(self.tag)


def _reference_holds(model: dict, value) -> bool:
    return any(item is value for item in tuple(model.values()))


def _agree(held: IdentityHeldDict, model: dict, pool: list) -> None:
    assert dict(held) == model and list(held) == list(model)
    for value in pool:
        assert held.holds(value) == _reference_holds(model, value)


@pytest.mark.parametrize("seed", range(12))
def test_holds_equals_the_walk_over_the_values_after_any_sequence_of_writes(seed):
    rng = random.Random(seed)
    pool = [_Item(index % 5) for index in range(9)]  # среди них равные, но не тождественные
    keys = [f"k{index}" for index in range(6)]
    held, model = IdentityHeldDict(), {}
    for _step in range(400):
        action = rng.choice(("set", "set", "set", "del", "pop", "pop_default", "popitem", "setdefault", "update", "ior", "clear"))
        key, value = rng.choice(keys), rng.choice(pool)
        if action == "set":
            held[key] = model[key] = value
        elif action == "del":
            if key in model:
                del held[key]
                del model[key]
            else:
                with pytest.raises(KeyError):
                    del held[key]
        elif action == "pop":
            if key in model:
                assert held.pop(key) is model.pop(key)
            else:
                with pytest.raises(KeyError):
                    held.pop(key)
        elif action == "pop_default":
            assert held.pop(key, "none") is model.pop(key, "none") or key not in model
        elif action == "popitem":
            if model:
                assert held.popitem() == model.popitem()
            else:
                with pytest.raises(KeyError):
                    held.popitem()
        elif action == "setdefault":
            assert held.setdefault(key, value) is model.setdefault(key, value)
        elif action == "update":
            other = {rng.choice(keys): rng.choice(pool), rng.choice(keys): rng.choice(pool)}
            held.update(other)
            model.update(other)
        elif action == "ior":
            other = {rng.choice(keys): rng.choice(pool)}
            held |= other
            model |= other
        elif rng.random() < 0.1:
            held.clear()
            model.clear()
        _agree(held, model, pool)


def test_a_value_under_two_keys_is_held_until_both_are_gone_and_an_equal_copy_is_not_the_value():
    first, equal_copy = _Item(1), _Item(1)
    held = IdentityHeldDict()
    held["a"] = held["b"] = first

    assert held.holds(first) and not held.holds(equal_copy)
    del held["a"]
    assert held.holds(first)
    held["b"] = equal_copy  # замена: прежнее значение ушло, новое пришло
    assert not held.holds(first) and held.holds(equal_copy)
    assert held.pop("b") is equal_copy and not held.holds(equal_copy) and not held


def test_the_constructor_takes_the_items_like_a_dict_and_a_plain_copy_is_a_dict():
    value = _Item(3)
    held = IdentityHeldDict({"a": value}, b=value)

    assert held.holds(value) and sorted(held) == ["a", "b"]
    assert type(held.copy()) is dict


# --------------------------------------------------------------------------
# Сессия
# --------------------------------------------------------------------------


class _Blobs:
    def __init__(self) -> None:
        self.discarded: list = []

    def discard(self, prepared) -> None:
        self.discarded.append(prepared)


def test_the_session_forgets_the_blob_of_a_preparation_only_when_its_cache_does_not_hold_it():
    controller = EnvelopeDebugSessionController()
    controller._preparation_blobs = _Blobs()  # noqa: SLF001
    held, gone = _Item(1), _Item(2)
    controller._conveyor_preparation_cache[("held",)] = held  # noqa: SLF001

    controller._forget_preparation_blob(held)  # noqa: SLF001
    controller._forget_preparation_blob(gone)  # noqa: SLF001

    assert controller._preparation_blobs.discarded == [gone]  # noqa: SLF001
    controller._conveyor_preparation_cache.clear()  # noqa: SLF001
    controller._forget_preparation_blob(held)  # noqa: SLF001
    assert controller._preparation_blobs.discarded == [gone, held]  # noqa: SLF001


def test_the_question_is_not_answered_by_a_walk_over_the_values():
    """Эта проверка ловит возврат к обходу: тело не содержит ни цикла, ни генератора, ни обращения к `values`."""

    tree = ast.parse((HOST / "envelope_debug_session.py").read_text(encoding="utf-8"))
    function = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "_forget_preparation_blob")
    walks = [node for node in ast.walk(function) if isinstance(node, (ast.For, ast.While, ast.GeneratorExp, ast.ListComp, ast.SetComp, ast.DictComp))]
    values = [node for node in ast.walk(function) if isinstance(node, ast.Attribute) and node.attr in {"values", "items"}]
    assert not walks and not values and any(isinstance(node, ast.Attribute) and node.attr == "holds" for node in ast.walk(function))
    assert isinstance(EnvelopeDebugSessionController()._conveyor_preparation_cache, IdentityHeldDict)  # noqa: SLF001


# --------------------------------------------------------------------------
# Отпечаток источника
# --------------------------------------------------------------------------


def _stub_bundle_parts(monkeypatch):
    calls = []
    monkeypatch.setattr(surface_module, "source_revision_from_bmesh", lambda *args, **kwargs: calls.append(args) or "taken")
    monkeypatch.setattr(surface_module, "build_patch_surface_ir", lambda bm, graph, revision: ("surface", revision))
    monkeypatch.setattr(surface_module, "AnalysisBundle", lambda **fields: fields)
    monkeypatch.setattr(surface_module, "AnalysisCapabilities", lambda: "capabilities")
    monkeypatch.setattr(surface_module, "validate_analysis_bundle", lambda bundle: None)
    seen = []

    def graph(bm, indices, obj, source_revision):
        seen.append(source_revision)
        return "graph"

    return calls, seen, graph


def test_the_bundle_takes_the_fingerprint_of_the_caller_and_not_a_second_one(monkeypatch):
    calls, seen, graph = _stub_bundle_parts(monkeypatch)

    bundle = surface_module.build_analysis_bundle("bm", [0, 1], "obj", graph, source_revision="given")

    assert calls == [] and seen == ["given"] and bundle["source_revision"] == "given"
    assert bundle["patch_surface"] == ("surface", "given")


def test_the_bundle_without_a_given_fingerprint_takes_its_own_as_before(monkeypatch):
    calls, seen, graph = _stub_bundle_parts(monkeypatch)

    bundle = surface_module.build_analysis_bundle("bm", [0, 1], "obj", graph)

    assert calls == [("bm", "obj", (0, 1))] and seen == ["taken"] and bundle["source_revision"] == "taken"


def test_the_button_takes_the_fingerprint_once_and_hands_it_to_the_bundle_builder():
    tree = ast.parse((HOST / "envelope_production_operator.py").read_text(encoding="utf-8"))
    taken = [node for node in ast.walk(tree) if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "source_revision_from_bmesh"]
    built = [node for node in ast.walk(tree) if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "build_analysis_bundle"]
    assert len(taken) == 1 and len(built) == 1
    assert [getattr(item, "id", None) for item in built[0].args][-1] == "source_revision"
