"""Семантика CPython 3.11 названа явно: порядок сравнений `list.sort` и левая свёртка float (`_cpython311`).

Цена ядра (`SIGN_COUNTS`, `EXACT_WORK_*`) и float-ответы не вправе зависеть от интерпретатора: Blender 4.5 исполняет
CPython 3.11, хост-тесты — 3.13, а `list.sort` (3.13) и `sum()` над float (3.12) в них ведут себя по-разному.

  * сортировка: журнал сравнений `sorted_as_cpython311` равен журналу `sorted(key=cmp_to_key)` ИЗ 3.11 вызов в вызов. Эталон
    записан под CPython 3.11.11 как дайджесты журналов по семействам входов (`GOLDEN_311`); под самим 3.11 тест дополнительно
    сверяет журнал с библиотекой напрямую, на каждом входе. Входы строит собственный генератор (не `random`), размеры
    пересекают `minrun` (64), стратегию слияний Powersort, галоп `min_gallop` и обе ветви `merge_lo`/`merge_hi`;
  * `left_fold_sum` — простая левая свёртка, как `sum()` 3.11 (в 3.12+ у float с компенсацией);
  * ответы и цена резки, знак-счётчики и точная работа — одни и те же в подпроцессах под ОБОИМИ интерпретаторами
    (`cpython311_probe.py`); без интерпретатора 3.11 тест пропускается названной причиной.
"""

from __future__ import annotations

import ast
import hashlib
import json
import os
import subprocess
import sys
from array import array
from functools import cmp_to_key
from pathlib import Path

from kernel_test_paths import PACKAGE_ROOT

from cftuv_envelope._cpython311 import left_fold_sum, sorted_as_cpython311

HERE = Path(__file__).resolve().parent
KERNEL_SOURCE = PACKAGE_ROOT.parent
BLENDER_PYTHON_GLOBS = ("C:/Program Files/Blender Foundation/Blender */*/python/bin/python.exe",)
CPYTHON_311_UNAVAILABLE = "CPYTHON_311_UNAVAILABLE"

#: Дайджесты журналов сравнений `sorted(key=cmp_to_key(compare))` под CPython 3.11.11 по семействам входов `_FAMILIES`.
GOLDEN_311 = {
    "random_wide": "c3e0c73f7eebb7bf2daeca84969c1d77570c0cf89a88dbc367de13a9c26c5495",
    "random_dupes": "9ffe2f9729170a301b7f9afde2c0de169dfbfdebaedda290b41dffb9e3958c79",
    "random_pairs": "4502b6d11d55623234eec3c55639a643c323af68654526e278c24c942f3a6143",
    "ascending_dupes": "85f3a8adc0d5eda282eaf182cccd6ba2de3c5f6be6af6458daff8b1d66e779d9",
    "descending_strict": "85f3a8adc0d5eda282eaf182cccd6ba2de3c5f6be6af6458daff8b1d66e779d9",
    "descending_dupes": "63aee155785824919277d318deaef4e62377472b6c283a769c6e1f58528cfad4",
    "ascending_noise": "682f77ca221a4650120f0cd9a7895e9e441e5bb9f7a116b4748971d0f272c242",
    "organ_pipe": "3ebefb90e80fbd47a46694a49f165f1693a02ed730e864c6604e8d84a43590b5",
    "sawtooth": "2c0940ea913a6c91d74f82c3bc475bad815e4d3c750b1dd2179e4ca32ebf785f",
    "short_blocks": "512a0b5e81f41be4d24ebe884853d73c4854c699bf6aa4be48287d229684d1e8",
    "long_blocks": "f3f8ba106c66d6d988ddce4eb980718198c794b191001022c73bd8bb31ace9f3",
    "clusters": "8ee550cf17c0e4d101036fb0c4866dcfa6df2d814fb28a4c26c359ef5aac011a",
}

SIZES = (*range(0, 131), 150, 191, 192, 193, 255, 256, 257, 300, 383, 384, 385, 511, 512, 513, 640, 1000, 1023, 1025, 1500, 2049, 4097)


def _stream(seed: int):
    state = seed
    while True:
        state = (state * 6364136223846793005 + 1442695040888963407) & 0xFFFFFFFFFFFFFFFF
        yield state >> 33


def _random_wide(size, draw):
    return [next(draw) % 1000003 for _ in range(size)]


def _random_dupes(size, draw):
    return [next(draw) % 5 for _ in range(size)]


def _random_pairs(size, draw):
    return [next(draw) % 17 for _ in range(size)]


def _ascending_dupes(size, draw):
    return [index // 3 for index in range(size)]


def _descending_strict(size, draw):
    return [-index for index in range(size)]


def _descending_dupes(size, draw):
    return [-(index // 3) for index in range(size)]


def _ascending_noise(size, draw):
    return [index * 4 + next(draw) % 11 for index in range(size)]


def _organ_pipe(size, draw):
    return [min(index, size - index) for index in range(size)]


def _sawtooth(size, draw):
    return [index % 13 for index in range(size)]


def _blocks(size, draw, shortest, longest):
    """Участки случайной длины, каждый строго возрастает либо строго убывает от случайного начала."""

    out: list[int] = []
    while len(out) < size:
        length = shortest + next(draw) % (longest - shortest + 1)
        start = next(draw) % 100000
        step = 1 + next(draw) % 3
        if next(draw) % 2:
            step = -step
        out.extend(start + step * index for index in range(length))
    return out[:size]


def _short_blocks(size, draw):
    return _blocks(size, draw, 1, 40)


def _long_blocks(size, draw):
    return _blocks(size, draw, 30, 300)


def _clusters(size, draw):
    """Участки из плотных кластеров подряд идущих чисел: при слиянии один участок «выигрывает» длинными сериями (галоп)."""

    out: list[int] = []
    while len(out) < size:
        base = next(draw) % 50
        run: list[int] = []
        for _ in range(2 + next(draw) % 6):
            width = 5 + next(draw) % 60
            base += 10 + next(draw) % 80
            run.extend(range(base, base + width))
            base += width
        out.extend(run)
    return out[:size]


_FAMILIES = {
    "random_wide": _random_wide,
    "random_dupes": _random_dupes,
    "random_pairs": _random_pairs,
    "ascending_dupes": _ascending_dupes,
    "descending_strict": _descending_strict,
    "descending_dupes": _descending_dupes,
    "ascending_noise": _ascending_noise,
    "organ_pipe": _organ_pipe,
    "sawtooth": _sawtooth,
    "short_blocks": _short_blocks,
    "long_blocks": _long_blocks,
    "clusters": _clusters,
}


def _case(family: str, size: int):
    values = _FAMILIES[family](size, _stream(1_000_003 * size + len(family)))
    return [(value, position) for position, value in enumerate(values)]


def _compare_logged(log):
    def compare(left, right):
        log.append(left[1])
        log.append(right[1])
        return (left[0] > right[0]) - (left[0] < right[0])

    return compare


def _digest_of(logs) -> str:
    digest = hashlib.sha256()
    for log in logs:
        digest.update(len(log).to_bytes(8, "little"))
        digest.update(array("I", log).tobytes())
    return digest.hexdigest()


def library_logs(family: str):
    """Журналы `sorted(key=cmp_to_key)` текущего интерпретатора по размерам `SIZES`."""

    logs = []
    for size in SIZES:
        log: list[int] = []
        sorted(_case(family, size), key=cmp_to_key(_compare_logged(log)))
        logs.append(log)
    return logs


def explicit_logs(family: str):
    logs = []
    for size in SIZES:
        log: list[int] = []
        items = _case(family, size)
        ordered = sorted_as_cpython311(items, _compare_logged(log))
        assert ordered == sorted(items, key=lambda item: item[0]), (family, size)  # результат и устойчивость
        logs.append(log)
    return logs


def test_the_comparison_sequence_equals_cpython_311_for_every_input_family():
    assert set(GOLDEN_311) == set(_FAMILIES)
    mismatched = []
    for family in _FAMILIES:
        explicit = explicit_logs(family)
        if sys.version_info[:2] == (3, 11):
            direct = [size for size, mine, theirs in zip(SIZES, explicit, library_logs(family)) if mine != theirs]
            assert not direct, f"{family}: the log differs from list.sort at sizes {direct}"
        if _digest_of(explicit) != GOLDEN_311[family]:
            mismatched.append(family)
    assert not mismatched, f"the comparison sequence differs from the CPython 3.11 record: {mismatched}"


def test_sizes_cross_minrun_and_force_merges_with_galloping():
    """Входы не вырождены: бывает слияние (`n > 64`) и оба направления галопа дают свой журнал."""

    log: list[int] = []
    sorted_as_cpython311(_case("clusters", 4097), _compare_logged(log))
    assert len(log) // 2 > 4097
    assert max(SIZES) > 64 and 64 in SIZES


def test_left_fold_sum_is_the_cpython_311_float_sum():
    terms = [1e16, 1.0, -1e16, 1.0]
    assert left_fold_sum(terms) == 1.0  # 3.12+ даёт 2.0: компенсация
    assert left_fold_sum(x for x in terms) == 1.0
    assert str(left_fold_sum([-0.0])) == "0.0"  # `0 + -0.0`, как у `sum()` 3.11
    assert left_fold_sum([]) == 0 and isinstance(left_fold_sum([]), int)
    if sys.version_info[:2] < (3, 12):
        assert left_fold_sum(terms) == sum(terms)


#: Единственное место ядра, где библиотечная сортировка с компаратором допустима, и причина.
LIBRARY_COMPARATOR_SORT_ALLOWED = {
    "_embedding.py": "компаратор — целочисленные произведения и полуплоскость луча: ни один счётный вызов точной работы внутри не стоит",
}


def test_no_library_comparator_sort_in_the_kernel():
    """`cmp_to_key` в ядре запрещён: порядок сравнений библиотечной сортировки зависит от версии CPython, а компаратор ядра
    считает знаки и тратит бюджет точной работы. Сортировка с компаратором идёт через `sorted_as_cpython311`."""

    violations = []
    for path in sorted((KERNEL_SOURCE / "cftuv_envelope").rglob("*.py")):
        if path.name in LIBRARY_COMPARATOR_SORT_ALLOWED:
            continue
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            named = isinstance(node, ast.ImportFrom) and node.module == "functools" and any(
                alias.name == "cmp_to_key" for alias in node.names
            )
            attribute = isinstance(node, ast.Attribute) and node.attr == "cmp_to_key"
            if named or attribute:
                violations.append(f"{path.relative_to(KERNEL_SOURCE)}:{node.lineno}")
    assert not violations, f"library comparator sort (use _cpython311.sorted_as_cpython311): {violations}"


def _interpreter_311():
    override = os.environ.get("CFTUV_CPYTHON311")
    candidates = [override] if override else []
    for pattern in BLENDER_PYTHON_GLOBS:
        anchor = Path(pattern.split("Blender */")[0])
        if anchor.exists():
            candidates.extend(str(path) for path in sorted(anchor.glob("Blender */*/python/bin/python.exe")))
    for candidate in candidates:
        try:
            out = subprocess.run(
                [candidate, "-c", "import sys; print(sys.version_info[0], sys.version_info[1])"],
                capture_output=True,
                text=True,
                timeout=60,
            )
        except (OSError, subprocess.SubprocessError):
            continue
        if out.returncode == 0 and out.stdout.split() == ["3", "11"]:
            return candidate
    return None


def _run_probe(interpreter: str) -> dict:
    env = {key: value for key, value in os.environ.items() if key not in {"PYTHONPATH", "PYTHONHOME"}}
    env["PYTHONPATH"] = str(KERNEL_SOURCE)
    env["PYTHONSAFEPATH"] = "1"
    out = subprocess.run(
        [interpreter, str(HERE / "cpython311_probe.py")],
        capture_output=True,
        text=True,
        timeout=540,
        env=env,
    )
    assert out.returncode == 0, out.stderr[-2000:]
    return json.loads(out.stdout.splitlines()[-1])


def test_clip_and_offset_cases_agree_across_interpreters():
    import pytest

    other = _interpreter_311()
    if other is None:
        pytest.skip(f"{CPYTHON_311_UNAVAILABLE}: no CPython 3.11 (Blender's bundled Python or CFTUV_CPYTHON311)")
    here = _run_probe(sys.executable)
    there = _run_probe(other)
    assert there["python"][:2] == [3, 11]
    assert set(here) == set(there)
    for name in here:
        if name == "python" or name.startswith("info_"):
            continue
        assert here[name] == there[name], f"{name}: answers or prices differ between CPython {here['python'][:2]} and 3.11"
    # Зонд не пуст: узлы резки доходят до слияний (n >= 64), точная работа и знаки с сопряжением оплачены, а встроенный `sum`
    # этого интерпретатора на тех же числах расходится с левой свёрткой (тогда сходство ответов — заслуга ядра, а не совпадение).
    assert max(max(there[name]["ordered_sizes"]) for name in there if name.startswith("clip_")) >= 64
    work = dict(map(tuple, there["sqrt_sorted"]["exact_work"]))
    assert work["EXACT_WORK_SPENT"] > 0 and there["sqrt_sorted"]["sign_counts"]["closed_by_conjugation"] > 0
    assert there["info_builtin_sum_differs"] == 0
    if tuple(here["python"][:2]) >= (3, 12):
        assert here["info_builtin_sum_differs"] > 0
