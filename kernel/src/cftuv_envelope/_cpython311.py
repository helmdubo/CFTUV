"""Семантика CPython 3.11, названная явно: порядок сравнений `list.sort` и левая свёртка float.

ЗАЧЕМ. Цена ядра (`SIGN_COUNTS`, `EXACT_WORK_*`) и float-ответы не вправе зависеть от версии интерпретатора, а зависели:
Blender 4.5 исполняет CPython 3.11, хост-тесты — 3.13. Два места библиотеки, чьё поведение менялось между ними:

  * `list.sort` / `sorted`: в 3.13 переписаны `count_run` и `binarysort`; ответ при согласованном порядке тот же, но
    ПОСЛЕДОВАТЕЛЬНОСТЬ вызовов сравнения иная, а сравнение узлов отрезка и проекций событий считает знаки и тратит бюджет
    точной работы. 44 из 117 полевых записей резки различались числом знаков между 3.11 и 3.13.
  * `sum()` над float: с 3.12 суммирует с компенсацией Ньюмайера, в 3.11 — простая левая свёртка.

`sorted_as_cpython311(items, compare)` повторяет `list.sort` ИЗ CPython 3.11 (`Objects/listobject.c`) для каждого `n`:
`count_run` (строго убывающий участок разворачивается, неубывающий нет), бинарная вставка до `minrun`, стратегия слияния
Powersort (`found_new_run`, `powerloop`: в 3.11 она уже заменила прежний `merge_collapse`), `merge_force_collapse` ПРЕЖНЕЙ
формы (сливает меньшего соседа), `merge_at` с галопом `gallop_left`/`gallop_right` и `merge_lo`/`merge_hi` со счётчиком
`min_gallop`, который живёт между слияниями одной сортировки. Последовательность вызовов `compare` совпадает с вызовами
`sorted(key=cmp_to_key(compare))` в 3.11 вызов в вызов; `compare(a, b) < 0` — «a меньше b», как у `cmp_to_key`; порядок
равных элементов устойчив, как у `sorted`. `left_fold_sum` — `sum()` 3.11: `0 + t0 + t1 + ...` слева направо без компенсации.

Здесь нет состояния и классов: каждая функция берёт всё, что ей нужно, аргументами (`min_gallop` — один элемент списка
`state`, как поле `MergeState` в C; запись стека `[начало, длина, мощность]`). Доказательство совпадения —
`kernel/tests/test_cpython311.py`: журнал сравнений записан под 3.11.11 и сверяется по дайджесту на любом интерпретаторе.
"""

from __future__ import annotations

MIN_GALLOP = 7


def left_fold_sum(terms):
    """`sum(terms)` из CPython 3.11: левая свёртка от целого нуля, без компенсации (в 3.12+ у float иная)."""

    total = 0
    for term in terms:
        total += term
    return total


def _min_run(count: int) -> int:
    carry = 0
    while count >= 64:
        carry |= count & 1
        count >>= 1
    return count + carry


def _count_run(keys, low: int, high: int, compare):
    """`(длина, убывает)` естественного участка с `low`: строго убывающий либо неубывающий (`count_run` 3.11)."""

    low += 1
    if low == high:
        return 1, False
    count = 2
    if compare(keys[low], keys[low - 1]) < 0:
        low += 1
        while low < high and compare(keys[low], keys[low - 1]) < 0:
            low += 1
            count += 1
        return count, True
    low += 1
    while low < high and not compare(keys[low], keys[low - 1]) < 0:
        low += 1
        count += 1
    return count, False


def _binary_insertion(keys, low: int, high: int, start: int, compare) -> None:
    """Бинарная вставка (`binarysort` 3.11): `keys[low:start]` отсортирован, `keys[start:high]` встаёт в него по одному."""

    if low == start:
        start += 1
    while start < high:
        left, right, pivot = low, start, keys[start]
        while left < right:
            middle = left + ((right - left) >> 1)
            if compare(pivot, keys[middle]) < 0:
                right = middle
            else:
                left = middle + 1
        keys[left + 1:start + 1] = keys[left:start]
        keys[left] = pivot
        start += 1


def _gallop_left(key, array, base: int, count: int, hint: int, compare) -> int:
    """Самая левая позиция `key` в отсортированном `array[base:base + count]`, галоп от `hint` (`gallop_left` 3.11)."""

    last, offset, at = 0, 1, base + hint
    if compare(array[at], key) < 0:
        limit = count - hint
        while offset < limit:
            if compare(array[at + offset], key) < 0:
                last = offset
                offset = (offset << 1) + 1
            else:
                break
        offset = min(offset, limit)
        last += hint
        offset += hint
    else:
        limit = hint + 1
        while offset < limit:
            if compare(array[at - offset], key) < 0:
                break
            last = offset
            offset = (offset << 1) + 1
        offset = min(offset, limit)
        last, offset = hint - offset, hint - last
    last += 1
    while last < offset:
        middle = last + ((offset - last) >> 1)
        if compare(array[base + middle], key) < 0:
            last = middle + 1
        else:
            offset = middle
    return offset


def _gallop_right(key, array, base: int, count: int, hint: int, compare) -> int:
    """Самая правая позиция `key` в отсортированном `array[base:base + count]`, галоп от `hint` (`gallop_right` 3.11)."""

    last, offset, at = 0, 1, base + hint
    if compare(key, array[at]) < 0:
        limit = hint + 1
        while offset < limit:
            if compare(key, array[at - offset]) < 0:
                last = offset
                offset = (offset << 1) + 1
            else:
                break
        offset = min(offset, limit)
        last, offset = hint - offset, hint - last
    else:
        limit = count - hint
        while offset < limit:
            if compare(key, array[at + offset]) < 0:
                break
            last = offset
            offset = (offset << 1) + 1
        offset = min(offset, limit)
        last += hint
        offset += hint
    last += 1
    while last < offset:
        middle = last + ((offset - last) >> 1)
        if compare(key, array[base + middle]) < 0:
            offset = middle
        else:
            last = middle + 1
    return offset


def _merge_low(keys, base_a: int, count_a: int, base_b: int, count_b: int, state, compare) -> None:
    """Слияние `a` (короткий, копия) и `b` слева направо (`merge_lo` 3.11)."""

    held = keys[base_a:base_a + count_a]
    dest, in_a, in_b = base_a, 0, base_b
    keys[dest] = keys[in_b]
    dest += 1
    in_b += 1
    count_b -= 1
    if count_b == 0:
        keys[dest:dest + count_a] = held[in_a:in_a + count_a]
        return
    if count_a == 1:
        keys[dest:dest + count_b] = keys[in_b:in_b + count_b]
        keys[dest + count_b] = held[in_a]
        return
    min_gallop = state[0]
    while True:
        wins_a = wins_b = 0
        while True:
            if compare(keys[in_b], held[in_a]) < 0:
                keys[dest] = keys[in_b]
                dest += 1
                in_b += 1
                wins_b += 1
                wins_a = 0
                count_b -= 1
                if count_b == 0:
                    keys[dest:dest + count_a] = held[in_a:in_a + count_a]
                    return
                if wins_b >= min_gallop:
                    break
            else:
                keys[dest] = held[in_a]
                dest += 1
                in_a += 1
                wins_a += 1
                wins_b = 0
                count_a -= 1
                if count_a == 1:
                    keys[dest:dest + count_b] = keys[in_b:in_b + count_b]
                    keys[dest + count_b] = held[in_a]
                    return
                if wins_a >= min_gallop:
                    break
        min_gallop += 1
        while True:
            min_gallop -= min_gallop > 1
            state[0] = min_gallop
            run = _gallop_right(keys[in_b], held, in_a, count_a, 0, compare)
            wins_a = run
            if run:
                keys[dest:dest + run] = held[in_a:in_a + run]
                dest += run
                in_a += run
                count_a -= run
                if count_a == 1:
                    keys[dest:dest + count_b] = keys[in_b:in_b + count_b]
                    keys[dest + count_b] = held[in_a]
                    return
                if count_a == 0:
                    return
            keys[dest] = keys[in_b]
            dest += 1
            in_b += 1
            count_b -= 1
            if count_b == 0:
                keys[dest:dest + count_a] = held[in_a:in_a + count_a]
                return
            run = _gallop_left(held[in_a], keys, in_b, count_b, 0, compare)
            wins_b = run
            if run:
                keys[dest:dest + run] = keys[in_b:in_b + run]
                dest += run
                in_b += run
                count_b -= run
                if count_b == 0:
                    keys[dest:dest + count_a] = held[in_a:in_a + count_a]
                    return
            keys[dest] = held[in_a]
            dest += 1
            in_a += 1
            count_a -= 1
            if count_a == 1:
                keys[dest:dest + count_b] = keys[in_b:in_b + count_b]
                keys[dest + count_b] = held[in_a]
                return
            if not (wins_a >= MIN_GALLOP or wins_b >= MIN_GALLOP):
                break
        min_gallop += 1
        state[0] = min_gallop


def _merge_high(keys, base_a: int, count_a: int, base_b: int, count_b: int, state, compare) -> None:
    """Слияние `a` и `b` (короткий, копия) справа налево (`merge_hi` 3.11)."""

    held = keys[base_b:base_b + count_b]
    dest, in_a, in_b = base_b + count_b - 1, base_a + count_a - 1, count_b - 1
    keys[dest] = keys[in_a]
    dest -= 1
    in_a -= 1
    count_a -= 1
    if count_a == 0:
        keys[dest - count_b + 1:dest + 1] = held[0:count_b]
        return
    if count_b == 1:
        keys[dest - count_a + 1:dest + 1] = keys[in_a - count_a + 1:in_a + 1]
        keys[dest - count_a] = held[in_b]
        return
    min_gallop = state[0]
    while True:
        wins_a = wins_b = 0
        while True:
            if compare(held[in_b], keys[in_a]) < 0:
                keys[dest] = keys[in_a]
                dest -= 1
                in_a -= 1
                wins_a += 1
                wins_b = 0
                count_a -= 1
                if count_a == 0:
                    keys[dest - count_b + 1:dest + 1] = held[0:count_b]
                    return
                if wins_a >= min_gallop:
                    break
            else:
                keys[dest] = held[in_b]
                dest -= 1
                in_b -= 1
                wins_b += 1
                wins_a = 0
                count_b -= 1
                if count_b == 1:
                    keys[dest - count_a + 1:dest + 1] = keys[in_a - count_a + 1:in_a + 1]
                    keys[dest - count_a] = held[in_b]
                    return
                if wins_b >= min_gallop:
                    break
        min_gallop += 1
        while True:
            min_gallop -= min_gallop > 1
            state[0] = min_gallop
            run = count_a - _gallop_right(held[in_b], keys, base_a, count_a, count_a - 1, compare)
            wins_a = run
            if run:
                dest -= run
                in_a -= run
                keys[dest + 1:dest + 1 + run] = keys[in_a + 1:in_a + 1 + run]
                count_a -= run
                if count_a == 0:
                    keys[dest - count_b + 1:dest + 1] = held[0:count_b]
                    return
            keys[dest] = held[in_b]
            dest -= 1
            in_b -= 1
            count_b -= 1
            if count_b == 1:
                keys[dest - count_a + 1:dest + 1] = keys[in_a - count_a + 1:in_a + 1]
                keys[dest - count_a] = held[in_b]
                return
            run = count_b - _gallop_left(keys[in_a], held, 0, count_b, count_b - 1, compare)
            wins_b = run
            if run:
                dest -= run
                in_b -= run
                keys[dest + 1:dest + 1 + run] = held[in_b + 1:in_b + 1 + run]
                count_b -= run
                if count_b == 1:
                    keys[dest - count_a + 1:dest + 1] = keys[in_a - count_a + 1:in_a + 1]
                    keys[dest - count_a] = held[in_b]
                    return
                if count_b == 0:
                    return
            keys[dest] = keys[in_a]
            dest -= 1
            in_a -= 1
            count_a -= 1
            if count_a == 0:
                keys[dest - count_b + 1:dest + 1] = held[0:count_b]
                return
            if not (wins_a >= MIN_GALLOP or wins_b >= MIN_GALLOP):
                break
        min_gallop += 1
        state[0] = min_gallop


def _merge_at(keys, pending, index: int, state, compare) -> None:
    """Слить участки `pending[index]` и `pending[index + 1]` (`merge_at` 3.11)."""

    base_a, count_a = pending[index][0], pending[index][1]
    base_b, count_b = pending[index + 1][0], pending[index + 1][1]
    pending[index][1] = count_a + count_b
    if index == len(pending) - 3:
        pending[index + 1] = pending[index + 2]
    pending.pop()
    skip = _gallop_right(keys[base_b], keys, base_a, count_a, 0, compare)
    base_a += skip
    count_a -= skip
    if count_a == 0:
        return
    count_b = _gallop_left(keys[base_a + count_a - 1], keys, base_b, count_b, count_b - 1, compare)
    if count_b <= 0:
        return
    if count_a <= count_b:
        _merge_low(keys, base_a, count_a, base_b, count_b, state, compare)
    else:
        _merge_high(keys, base_a, count_a, base_b, count_b, state, compare)


def _power(start: int, first: int, second: int, size: int) -> int:
    """«Мощность» границы двух соседних участков (`powerloop` 3.11): глубина в двоичном дереве слияний Мунро—Уайлда."""

    result = 0
    left = 2 * start + first
    right = left + first + second
    while True:
        result += 1
        if left >= size:
            left -= size
            right -= size
        elif right >= size:
            break
        left <<= 1
        right <<= 1
    return result


def _found_new_run(keys, pending, count: int, size: int, state, compare) -> None:
    """Новый участок длины `count` найден: слить участки стека с большей мощностью границы (`found_new_run` 3.11)."""

    if pending:
        power = _power(pending[-1][0], pending[-1][1], count, size)
        while len(pending) > 1 and pending[-2][2] > power:
            _merge_at(keys, pending, len(pending) - 2, state, compare)
        pending[-1][2] = power


def _merge_force_collapse(keys, pending, state, compare) -> None:
    while len(pending) > 1:
        index = len(pending) - 2
        if index > 0 and pending[index - 1][1] < pending[index + 1][1]:
            index -= 1
        _merge_at(keys, pending, index, state, compare)


def sorted_as_cpython311(items, compare) -> list:
    """`sorted(items, key=cmp_to_key(compare))` CPython 3.11 с ТОЙ ЖЕ последовательностью вызовов `compare` (список)."""

    keys = list(items)
    size = remaining = len(keys)
    if remaining < 2:
        return keys
    state, pending, low = [MIN_GALLOP], [], 0
    min_run = _min_run(remaining)
    while remaining:
        run, descending = _count_run(keys, low, low + remaining, compare)
        if descending:
            keys[low:low + run] = keys[low:low + run][::-1]
        if run < min_run:
            forced = min(remaining, min_run)
            _binary_insertion(keys, low, low + forced, low + run, compare)
            run = forced
        _found_new_run(keys, pending, run, size, state, compare)
        pending.append([low, run, 0])
        low += run
        remaining -= run
    _merge_force_collapse(keys, pending, state, compare)
    return keys
