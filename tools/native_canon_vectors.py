"""Векторы эталона канонизации радикалов для нативного крейта `cftuv-canon`: пишет JSON, который читают тесты Rust.

Эталон — питон-ядро (`exact_sqrt_sum.py`), и оно НЕ правится. Скрипт не знает ни Blender, ни нативной сборки: он гоняет
функции эталона на входах и записывает всё, что нативная сторона обязана повторить побитово — ответ, шесть статей бюджета
после каждого вызова, точку исчерпания (операция, радикант, статьи в этот момент) и четыре таблицы памяти канонизации с
порядком вставки (включая LRU-перенос разложений, вытеснение на 8192 записях и очистку реестра простых на 8192).

Числа в файлах — только шестнадцатеричные строки без `0x` (знак `-` впереди у отрицательных), десятичных нет: питоновский
`int_max_str_digits` на длинных числах не должен влиять на формат. Размер каждого файла < 400 КБ; большие состояния
(пороги 8192) записаны отпечатком FNV-1a 64 канонического текста плюс головой и хвостом.

Запуск (системный python, ядро берётся из `kernel/src`):

    PYTHONSAFEPATH=1 python tools/native_canon_vectors.py            # все файлы векторов
    PYTHONSAFEPATH=1 python tools/native_canon_vectors.py --only pyrandom thresholds
    PYTHONSAFEPATH=1 python tools/native_canon_vectors.py --timing   # цена единицы бюджета эталона, нс
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "kernel" / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "kernel" / "src"))

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402

OUT_DIR = ROOT / "native" / "cftuv-canon" / "tests" / "vectors"
MAX_FILE_BYTES = 400_000
SCHEMA = "cftuv.canon-vectors.v1"
FACTORIZATION_LIMIT = exact._FACTORIZATION_MEMO_ENTRIES
REGISTRY_LIMIT = exact._KNOWN_PRIME_REGISTRY_ENTRIES
BOUNDED = exact.ExactWorkBudgetModeV1.BOUNDED
UNLIMITED = exact.ExactWorkBudgetModeV1.UNLIMITED_REFERENCE
Exhausted = exact.ExactCanonicalizationWorkBudgetExhausted


# --------------------------------------------------------------------------
# Кодирование: числа — hex, состояние памяти — упорядоченные списки
# --------------------------------------------------------------------------


def hx(value: int) -> str:
    return format(value, "x")


def unhx(text: str) -> int:
    return int(text, 16)


def fnv1a64(data: bytes) -> int:
    value = 0xCBF29CE484222325
    for byte in data:
        value = ((value ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return value


def state_texts(state: dict) -> dict:
    """Канонический текст каждой таблицы. Его же строит Rust-сторона: порядок записей входит."""

    return {
        "p": ",".join(state["p"]),
        "f": ";".join(f"{key}:{pairs_text_hex(pairs)}" for key, pairs in state["f"]),
        "s": ";".join(f"{key}:{pair[0]},{pair[1]}" for key, pair in state["s"]),
        "u": ";".join(f"{key}:{','.join(primes)}" for key, primes in state["u"]),
    }


def pairs_text_hex(pairs) -> str:
    return ",".join(f"{prime}^{power}" for prime, power in pairs)


def digest_state(state: dict, heads: bool = True) -> dict:
    """Отпечаток состояния: длины и FNV-1a 64 текстов таблиц; `heads` — ещё голова и хвост ключей реестра и разложений."""

    texts = state_texts(state)
    digest = {
        "n": [len(state["p"]), len(state["f"]), len(state["s"]), len(state["u"])],
        "d": [hx(fnv1a64(texts[name].encode())) for name in ("p", "f", "s", "u")],
    }
    if heads:
        digest["pk"] = state["p"][:3] + state["p"][-3:]
        digest["fk"] = [key for key, _ in state["f"]][:3] + [key for key, _ in state["f"]][-3:]
    return digest


def capture_state() -> dict:
    """Память канонизации эталона: упорядоченные списки, ключи и значения — hex."""

    assert set(exact._KNOWN_PRIMES) == exact._KNOWN_PRIME_SET
    return {
        "p": [hx(prime) for prime in exact._KNOWN_PRIMES],
        "f": [[hx(key), [[hx(p), e] for p, e in pairs]] for key, pairs in exact._FACTORIZATION_MEMO.items()],
        "s": [[hx(key), [hx(a), hx(b)]] for key, (a, b) in exact._SQUAREFREE_MEMO.items()],
        "u": [[hx(key), [hx(p) for p in support]] for key, support in exact._PRIME_SUPPORT_MEMO.items()],
    }


def expand_before(before: dict | None) -> dict:
    """Явное начальное состояние + правила заполнения (`fill`): ключи подряд, значения синтетические."""

    before = before or {}
    primes = [unhx(item) for item in before.get("p", [])]
    factorizations = [(unhx(key), tuple((unhx(p), e) for p, e in pairs)) for key, pairs in before.get("f", [])]
    splits = [(unhx(key), (unhx(a), unhx(b))) for key, (a, b) in before.get("s", [])]
    supports = [(unhx(key), tuple(unhx(p) for p in support)) for key, support in before.get("u", [])]
    fill = before.get("fill", {})
    if "p" in fill:
        start, step, count = unhx(fill["p"]["start"]), fill["p"]["step"], fill["p"]["count"]
        primes += [start + step * index for index in range(count)]
    if "f" in fill:
        start, count = unhx(fill["f"]["start"]), fill["f"]["count"]
        factorizations += [(start + index, ((start + index, 1),)) for index in range(count)]
    return {"p": primes, "f": factorizations, "s": splits, "u": supports}


def load_state(before: dict | None) -> None:
    """Ставит память эталона в `before` (холодная, если `None`). Порядок вставки сохраняется."""

    expanded = expand_before(before)
    exact.reset_factorization_memory()
    exact._KNOWN_PRIMES.extend(expanded["p"])
    exact._KNOWN_PRIME_SET.update(expanded["p"])
    exact._FACTORIZATION_MEMO.update(expanded["f"])
    exact._SQUAREFREE_MEMO.update(expanded["s"])
    exact._PRIME_SUPPORT_MEMO.update(expanded["u"])


def q_text(q: Fraction) -> list:
    return [hx(q.numerator), hx(q.denominator)]


def q_from(items) -> tuple:
    return tuple(Fraction(unhx(num), unhx(den)) for num, den in items)


# --------------------------------------------------------------------------
# Бюджет-самописец: исчерпание — это (операция, радикант), а не строка
# --------------------------------------------------------------------------


class RecordingBudget(exact.ExactWorkBudgetV1):
    """Тот же бюджет эталона; запоминает, на какой операции и каком радиканде он отказал."""

    __slots__ = ("exhaustion",)

    def __init__(self, *, cap):
        mode = UNLIMITED if cap is None else BOUNDED
        super().__init__(mode=mode, cap=cap, stage="S", domain_id="D", superlevel="L")
        self.exhaustion = None

    def _enforce(self, operation, radicand):
        try:
            super()._enforce(operation, radicand)
        except Exhausted:
            self.exhaustion = [operation.value, hx(int(radicand))]
            raise


def articles_of(budget) -> list:
    return list(budget.spent_by_article())


# --------------------------------------------------------------------------
# Интерпретатор программ: последовательность вызовов над общей памятью и бюджетом
# --------------------------------------------------------------------------


def encode_pairs(pairs) -> list:
    return [[hx(prime), power] for prime, power in pairs]


def parse_negative(error: Exception) -> list:
    """`NegativeRadicandError("под корнем -3/4")` -> `["neg", "-3", "4"]` (числитель со знаком, знаменатель)."""

    token = str(error).rsplit(" ", 1)[-1]
    numerator, _, denominator = token.partition("/")
    return ["neg", hx(int(numerator)), hx(int(denominator or 1))]


def encode_delta(delta) -> dict:
    return {
        "f": [[hx(key), encode_pairs(pairs)] for key, pairs in delta.factorizations],
        "s": [[hx(key), [hx(a), hx(b)]] for key, (a, b) in delta.squarefree],
        "u": [[hx(key), [hx(p) for p in support]] for key, support in delta.supports],
        "p": [hx(prime) for prime in delta.primes],
    }


def encode_record(record) -> list:
    """`(universe, delta, price, memory)` of a `prime_universe_remembered` store entry, without the universe (the answer carries it)."""

    _universe, delta, price, memory = record
    return [[[hx(number), encode_pairs(pairs)] for number, pairs in delta], None if price is None else list(price), encode_delta(memory)]


def decode_delta(items: dict):
    return exact.FactorizationMemoryDeltaV1(
        tuple((unhx(key), tuple((unhx(p), e) for p, e in pairs)) for key, pairs in items["f"]),
        tuple((unhx(key), (unhx(a), unhx(b))) for key, (a, b) in items["s"]),
        tuple((unhx(key), tuple(unhx(p) for p in support)) for key, support in items["u"]),
        tuple(unhx(prime) for prime in items["p"]),
    )


class Run:
    """Один прогон программы: память эталона, бюджет, `store`-словари, маркеры и дельты."""

    def __init__(self, program: dict, cap):
        self.program = program
        self.budget = RecordingBudget(cap=cap)
        self.stores: dict = {}
        self.marks: dict = {}
        self.deltas: dict = {}
        # Прогоны под потолком пишут отпечаток вместо полного состояния (`state_capped`): файл остаётся малым.
        self.mode = program.get("state", "full")
        if cap is not None:
            self.mode = program.get("state_capped", self.mode)
        exact.reset_unbudgeted_work()
        load_state(program.get("before"))

    def state(self):
        if self.mode == "none":
            return None
        state = capture_state()
        return state if self.mode == "full" else digest_state(state)

    def target(self, call: dict):
        return None if call.get("none") else self.budget

    def op_split(self, call):
        return [hx(part) for part in exact.squarefree_split(unhx(call["n"]), self.target(call))]

    def op_support(self, call):
        return [hx(prime) for prime in exact.prime_support(unhx(call["n"]), self.target(call))]

    def op_pairs(self, call):
        return encode_pairs(exact._factorization_pairs(unhx(call["n"]), self.target(call)))

    def op_seed(self, call):
        exact._seed_factorization_basis(tuple(unhx(item) for item in call["radicands"]), self.target(call))
        return None

    def op_basis(self, call):
        return [hx(item) for item in exact._coprime_basis(tuple(unhx(item) for item in call["values"]), self.target(call))]

    def op_universe(self, call):
        universe = exact._prime_universe_from_q_values(q_from(call["q"]), self.target(call))
        return [hx(prime) for prime in universe]

    def op_remembered(self, call):
        store = self.stores.setdefault(call["store"], {})
        q_values = q_from(call["q"])
        key = ("prime-universe", tuple(Fraction(value) for value in q_values))
        found = store.get(key)
        universe = exact.prime_universe_remembered(q_values, self.target(call), store)
        # a hit leaves the record it found; a miss (or a hit the budget could not afford) writes a NEW record: its factorizations, its price and its memory delta
        hit = store[key] is found
        record = None if hit else encode_record(store[key])
        return ["hit" if hit else "miss", [hx(prime) for prime in universe], record]

    def op_reset(self, call):
        exact.reset_factorization_memory()
        return None

    def op_fill(self, call):
        """Дописать синтетические записи (ключи подряд) в конец реестра `p` или таблицы разложений `f`."""

        expanded = expand_before({"fill": {call["table"]: call["rule"]}})
        exact._KNOWN_PRIMES.extend(expanded["p"])
        exact._KNOWN_PRIME_SET.update(expanded["p"])
        exact._FACTORIZATION_MEMO.update(expanded["f"])
        return None

    def op_mark(self, call):
        self.marks[call["name"]] = exact.factorization_memory_marker()
        return None

    def op_delta(self, call):
        delta = exact.factorization_memory_delta(self.marks[call["name"]])
        encoded = encode_delta(delta)
        self.deltas[call["as"]] = delta
        return encoded

    def op_replay(self, call):
        exact.replay_factorization_memory(self.deltas[call["name"]])
        return None

    def op_isolated(self, call):
        inner = []
        with exact.isolated_factorization_memory():
            for item in call["calls"]:
                inner.append(self.record(item))
        return inner

    def call(self, call: dict):
        try:
            return getattr(self, "op_" + call["op"])(call)
        except Exhausted:
            return ["exh", *self.budget.exhaustion]
        except exact.NegativeRadicandError as error:
            return parse_negative(error)

    def record(self, call: dict) -> dict:
        result = self.call(call)
        entry = {"r": result, "a": articles_of(self.budget)}
        if self.program.get("unbudgeted"):
            entry["u"] = articles_of(exact.UNBUDGETED_WORK)
        entry["s"] = self.state()
        return entry

    def execute(self, stop_on_exhaustion: bool = False) -> list:
        records = []
        for call in self.program["calls"]:
            entry = self.record(call)
            records.append(entry)
            if stop_on_exhaustion and self.budget.exhaustion is not None:
                break
        return records


def run_program(program: dict, cap) -> dict:
    run = Run(program, cap)
    calls = run.execute()
    return {"cap": cap, "calls": calls, "final": run.state()}


def total_spent(program: dict) -> int:
    run = Run(program, None)
    run.execute()
    return run.budget.spent


# --------------------------------------------------------------------------
# Простые и составные для входов
# --------------------------------------------------------------------------


def unbounded() -> RecordingBudget:
    return RecordingBudget(cap=None)


def is_prime_value(value: int) -> bool:
    return exact._is_prime(value, unbounded())


def random_prime(bits: int, rng: random.Random) -> int:
    while True:
        candidate = rng.getrandbits(bits) | (1 << (bits - 1)) | 1
        if is_prime_value(candidate):
            return candidate


def distinct_primes(bit_sizes, rng: random.Random) -> list:
    found: list = []
    for bits in bit_sizes:
        prime = random_prime(bits, rng)
        while prime in found:
            prime = random_prime(bits, rng)
        found.append(prime)
    return found


def product(values) -> int:
    result = 1
    for value in values:
        result *= value
    return result


# --------------------------------------------------------------------------
# Файлы
# --------------------------------------------------------------------------


def write_vectors(name: str, header: dict, cases: list) -> None:
    """Пишет `{схема, шапка, cases}` по одному случаю на строку. Больше `MAX_FILE_BYTES` — отказ записи."""

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    lines = [json.dumps(case, separators=(",", ":")) for case in cases]
    meta = {"schema": SCHEMA, "name": name, "python": sys.version.split()[0], **header}
    text = json.dumps(meta, separators=(",", ":"))[:-1] + ',"cases":[\n' + ",\n".join(lines) + "\n]}\n"
    if len(text.encode()) >= MAX_FILE_BYTES:
        raise SystemExit(f"{name}: {len(text.encode())} bytes exceeds {MAX_FILE_BYTES}")
    (OUT_DIR / f"{name}.json").write_text(text, encoding="utf-8")
    print(f"{name}.json: {len(cases)} cases, {len(text.encode()) // 1024} KiB")


# --------------------------------------------------------------------------
# pyrandom: MT19937 и целочисленная часть `random.Random(n)`
# --------------------------------------------------------------------------

SEED_BITS = (0, 1, 31, 32, 33, 63, 64, 65, 96, 127, 128, 129, 245, 250, 320)
BIT_COUNTS = (1, 2, 17, 31, 32, 33, 47, 63, 64, 65, 96, 100, 129, 250, 320)
BELOW_BOUNDS = ("1", "2", "3", "5", "7ffffffff", "100000000", "100000001", "8000000000000001", "10000000000000000", "3b9aca0000000000000000000000f", "3fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffb", "400000000000000000000000000000000000000000000000000000000000001")


def rng_op(rng: random.Random, op: list):
    kind = op[0]
    if kind == "bits":
        return hx(rng.getrandbits(op[1]))
    if kind == "below":
        return hx(rng._randbelow(unhx(op[1])))
    if kind == "range":
        return hx(rng.randrange(unhx(op[1]), unhx(op[2])))
    if kind == "u32s":
        words = [rng.getrandbits(32) for _ in range(op[1])]
        return hx(fnv1a64(b"".join(word.to_bytes(4, "little") for word in words)))
    raise ValueError(kind)


def random_rng_ops(chooser: random.Random) -> list:
    ops: list = [["bits", 32], ["bits", 32], ["bits", 1], ["bits", 64], ["u32s", 700], ["bits", 32]]
    for _ in range(40):
        pick = chooser.randrange(3)
        if pick == 0:
            ops.append(["bits", chooser.choice(BIT_COUNTS)])
        elif pick == 1:
            ops.append(["below", chooser.choice(BELOW_BOUNDS)])
        else:
            bound = unhx(chooser.choice(BELOW_BOUNDS))
            ops.append(["range", chooser.choice(["0", "1"]), hx(bound + 1 + chooser.randrange(3))])
    ops.append(["u32s", 1300])
    return ops


def pyrandom_case(seed_text: str, ops: list) -> dict:
    rng = random.Random(unhx(seed_text))
    return {"seed": seed_text, "ops": ops, "out": [rng_op(rng, op) for op in ops]}


def build_pyrandom() -> None:
    chooser = random.Random(0xC0FFEE)
    cases = []
    for bits in SEED_BITS:
        seed = 0 if bits == 0 else chooser.getrandbits(bits) | (1 << (bits - 1))
        cases.append(pyrandom_case(hx(seed), random_rng_ops(chooser)))
    cases.append(pyrandom_case("-" + hx(chooser.getrandbits(80)), random_rng_ops(chooser)))
    for bits in (40, 90, 140):
        cases.append(pyrandom_case(hx(chooser.getrandbits(bits) | 1 << (bits - 1)), random_rng_ops(chooser)))
    # Сидирование так, как его делает `_pollard_rho`: `Random(n)`, затем `randrange(1, n)` и `randrange(0, n)`.
    for bits in (30, 64, 65, 120, 250):
        modulus = chooser.getrandbits(bits) | 1 << (bits - 1) | 1
        ops = [["range", "1", hx(modulus)], ["range", "0", hx(modulus)]] * 6
        cases.append(pyrandom_case(hx(modulus), ops))
    write_vectors("pyrandom", {}, cases)


# --------------------------------------------------------------------------
# Прямые вызовы: _is_prime, _pollard_rho, _pollard_rho_brent_attempt, _rho_factors, _coprime_basis
# --------------------------------------------------------------------------


def _do_is_prime(args: dict, budget):
    return exact._is_prime(unhx(args["n"]), budget)


def _do_rho(args: dict, budget):
    return hx(exact._pollard_rho(unhx(args["n"]), budget))


def _do_brent(args: dict, budget):
    divisor = exact._pollard_rho_brent_attempt(
        unhx(args["n"]), unhx(args["y"]), unhx(args["c"]), batch_size=args["batch"], budget=budget
    )
    return None if divisor is None else hx(divisor)


def _do_rho_factors(args: dict, budget):
    return encode_pairs(exact._rho_factors(unhx(args["n"]), budget).items())


def _do_coprime(args: dict, budget):
    return [hx(item) for item in exact._coprime_basis(tuple(unhx(item) for item in args["values"]), budget)]


DIRECT = {
    "is_prime": _do_is_prime,
    "rho": _do_rho,
    "brent": _do_brent,
    "rho_factors": _do_rho_factors,
    "coprime": _do_coprime,
}


def direct_run(function: str, args: dict, cap) -> dict:
    budget = RecordingBudget(cap=cap)
    try:
        result = ["ok", DIRECT[function](args, budget)]
    except Exhausted:
        result = ["exh", *budget.exhaustion]
    return {"cap": cap, "r": result, "a": articles_of(budget)}


def cap_choices(total: int, full: bool = True) -> list:
    """Потолки для прогона: вокруг границы траты (0, 1, 2, треть, половина, total-1, total, total+1); короткий набор — `total-1`, `total`."""

    caps = {0, 1, 2, total // 3, total // 2, total - 1, total, total + 1} if full else {total // 2, total - 1, total}
    return sorted(cap for cap in caps if cap >= 0)


def direct_case(function: str, args: dict, sweep_caps: bool = True, full_caps: bool = True) -> dict:
    """Случай: без потолка, затем под потолками вокруг итоговой траты. Нулевая трата — только прогон без потолка."""

    base = direct_run(function, args, None)
    runs = [base]
    total = sum(base["a"])
    if sweep_caps and total > 0:
        runs += [direct_run(function, args, cap) for cap in cap_choices(total, full_caps)]
    return {"fn": function, "args": args, "runs": runs}


PSEUDOPRIMES = (
    2047, 1373653, 25326001, 3215031751, 2152302898747, 3474749660383, 341550071728321, 3825123056546413051,
    318665857834031151167461, 3317044064679887385961981,
)
CARMICHAEL = (561, 1105, 1729, 2465, 2821, 6601, 8911, 10585, 15841, 29341, 41041, 46657, 52633, 62745, 63973, 75361, 101101)
PRIME_BIT_SIZES = (
    20, 31, 32, 33, 47, 61, 63, 64, 65, 80, 100, 120, 127, 128, 129, 150, 191, 192, 193, 200, 250, 255, 256, 257,
    300, 319, 320, 321, 383, 384, 385, 448, 449, 511, 512, 513, 600,
)


def primality_values(rng: random.Random) -> list:
    values = [0, 1, 2, 3, 4, 5, 9, 25, 36, 37, 38, 39, 41, 43, 49, 1681, 1763, 65537, (1 << 31) - 1, (1 << 61) - 1]
    values += list(PSEUDOPRIMES) + list(CARMICHAEL)
    primes = [random_prime(bits, rng) for bits in PRIME_BIT_SIZES]
    values += primes
    small_primes = [random_prime(20, rng) for _ in range(3)]
    values += [small_primes[0] ** 2, small_primes[0] * small_primes[1], product(small_primes)]
    values += [primes[3] * primes[4], primes[9] * primes[10], primes[17] * primes[0], primes[22] ** 2]
    values += [2 * primes[5], 3 * primes[6], 37 * primes[7], 41 * primes[8], 123456789 * 2, 10**30 + 1]
    return values


def build_primality() -> None:
    rng = random.Random(1006)
    cases = [direct_case("is_prime", {"n": hx(value)}) for value in primality_values(rng)]
    write_vectors("primality", {}, cases)


def semiprime(small_bits: int, large_bits: int, rng: random.Random) -> int:
    small = random_prime(small_bits, rng)
    large = random_prime(large_bits, rng)
    while large == small:
        large = random_prime(large_bits, rng)
    return small * large


def full_width_semiprime(small_bits: int, words: int, rng: random.Random) -> int:
    """`p * q` с точной шириной `64 * words` бит (старший бит слова установлен): пути переноса монтгомери-умножения."""

    small = random_prime(small_bits, rng)
    low, high = -(-(1 << (64 * words - 1)) // small), ((1 << (64 * words)) - 1) // small
    while True:
        large = rng.randrange(low, high) | 1
        if large != small and is_prime_value(large):
            return small * large


def brent_moduli(rng: random.Random) -> list:
    shapes = [
        (14, 14), (16, 16), (18, 40), (20, 60), (22, 98), (24, 100), (20, 230), (22, 300), (20, 450), (20, 520),
        (31, 31), (12, 12), (15, 15), (17, 47), (22, 150), (22, 270), (22, 400),
    ]
    values = [semiprime(small, large, rng) for small, large in shapes]
    values += [full_width_semiprime(22, words, rng) for words in range(1, 9)]
    primes = distinct_primes((18, 18, 18, 22, 26), rng)
    values += [primes[0] ** 2, primes[1] ** 3, primes[0] * primes[1] * primes[2], primes[3] * primes[3] * primes[4]]
    # Крошечные модули: орбита замыкается внутри одного пакета, значит `gcd(произведение, n) == n` и играет проигрыш.
    values += [101 * 103, 211 * 223, 1009 * 1013, 1009 * 1009, 3 * 5 * 7 * 11 * 13 * 17 * 19, 7**9, 10007 * 10009]
    return values


def brent_cases(n: int) -> list:
    """Пары `(c, y)` ровно так, как их берёт `_pollard_rho` (`Random(n)`), плюс размеры пакета и `y >= n`."""

    rng = random.Random(n)
    pairs = [(rng.randrange(1, n), rng.randrange(0, n)) for _ in range(8 if n.bit_length() < 30 else 4)]
    cases = [{"n": hx(n), "c": hx(c), "y": hx(y), "batch": 64} for c, y in pairs]
    c, y = pairs[0]
    cases += [{"n": hx(n), "c": hx(c), "y": hx(y), "batch": batch} for batch in (1, 7, 1000)]
    cases.append({"n": hx(n), "c": hx(c), "y": hx(y + n), "batch": 64})
    return cases


def attempts_without_divisor(limit: int) -> list:
    """Параметры `(n, c, y)`, на которых попытка Брента возвращает `None` (проигрыш не нашёл делителя): ищем у крошечных `n`."""

    rng = random.Random(1013)
    found: list = []
    for modulus in (101 * 103, 1009 * 1013, 7**9, 13 * 13 * 17, 1009**2, 3 * 5 * 7 * 11 * 13 * 17 * 19, 3 * 3 * 3 * 3 * 5 * 5, 97 * 97 * 97):
        for _ in range(400):
            c, y = rng.randrange(1, modulus), rng.randrange(0, modulus)
            if exact._pollard_rho_brent_attempt(modulus, y, c, batch_size=64, budget=unbounded()) is None:
                found.append({"n": hx(modulus), "c": hx(c), "y": hx(y), "batch": 64})
                break
        if len(found) >= limit:
            break
    return found


class GcdTally:
    """Считает вызовы `gcd(x, n) == n` эталона: каждый такой — вход в проигрыш пакета Брента."""

    def __init__(self):
        self.hits = 0
        self.original = exact.gcd

    def __call__(self, left, right):
        result = self.original(left, right)
        self.hits += result == right
        return result


def build_brent() -> None:
    rng = random.Random(1007)
    tally = GcdTally()
    exact.gcd = tally
    try:
        cases = []
        for modulus in brent_moduli(rng):
            cases.append(direct_case("rho", {"n": hx(modulus)}))
            for index, args in enumerate(brent_cases(modulus)):
                cases.append(direct_case("brent", args, sweep_caps=index < 3 or modulus.bit_length() >= 30, full_caps=index < 3))
        cases += [direct_case("brent", args) for args in attempts_without_divisor(8)]
    finally:
        exact.gcd = tally.original
    failures = sum(1 for case in cases if case["fn"] == "brent" and case["runs"][0]["r"][1] is None)
    print(f"brent: {failures} attempts without divisor, {tally.hits} gcd == n events (batch replays and closed orbits)")
    write_vectors("brent", {}, cases)


def rho_factor_values(rng: random.Random) -> list:
    p = distinct_primes((12, 14, 16, 18, 20, 22, 25), rng)
    values = [1, 2, 3, 4, 6, 12, 1 << 20, 3 * 5 * 7 * 11 * 13, p[0], p[0] * p[0], p[0] ** 3 * p[1], p[0] ** 2 * p[1] ** 3]
    values += [product(p[:4]), product(p[:5]), product(p), (p[0] * p[1] * p[2]) ** 2, (p[1] * p[2]) ** 4, product(p[:3]) ** 2 * p[5]]
    values += [2 ** 5 * p[3] * p[4], product(distinct_primes([25] * 8, rng)), p[6] * p[6] * p[5] * p[5] * p[4]]
    return values


def build_rho_factors() -> None:
    rng = random.Random(1008)
    write_vectors("rho_factors", {}, [direct_case("rho_factors", {"n": hx(value)}) for value in rho_factor_values(rng)])


def coprime_sets(rng: random.Random) -> list:
    p = distinct_primes((10, 12, 14, 16, 18, 20), rng)
    sets = [
        [], [1], [6], [12, 18], [12, 18, 1, 35], [4, 4, 4], [6, 10, 15], [p[0] * p[1], p[1] * p[2], p[2] * p[0]],
        [product(p), product(p[:3]), product(p[2:]), p[0] ** 2 * p[1]], [30, 42, 70, 105, 210], [2**20, 2**10, 2**5, 3**7, 3**3],
        [p[0] * p[1] * p[2], p[0] * p[1] * p[2]], [p[3] * p[4] * 1009, p[4] * p[5] * 1013, 1009 * 1013 * 1019],
    ]
    first = [q for q in range(3, 800) if all(q % d for d in range(2, int(q**0.5) + 1))][:100]
    sets.append([first[i] * first[j] for i in range(100) for j in range(i + 1, 100)])
    return sets


def build_coprime() -> None:
    rng = random.Random(1009)
    cases = [direct_case("coprime", {"values": [hx(value) for value in values]}, sweep_caps=len(values) < 50) for values in coprime_sets(rng)]
    big = cases[-1]["runs"][0]["r"][1]
    coprime = all(unhx(a) != unhx(b) and exact.gcd(unhx(a), unhx(b)) == 1 for i, a in enumerate(big) for b in big[i + 1:])
    print(f"coprime: split budget {'NOT ' if coprime else ''}reached on the large set ({len(big)} atoms)")
    write_vectors("coprime", {}, cases)


# --------------------------------------------------------------------------
# Последовательности: реалистичное семейство радикандов, память с порядком после каждого вызова
# --------------------------------------------------------------------------


def split_call(value: int, **extra) -> dict:
    return {"op": "split", "n": hx(value), **extra}


def support_call(value: int, **extra) -> dict:
    return {"op": "support", "n": hx(value), **extra}


def pairs_call(value: int, **extra) -> dict:
    return {"op": "pairs", "n": hx(value), **extra}


def universe_call(q_values, op: str = "universe", **extra) -> dict:
    return {"op": op, "q": [q_text(q) for q in q_values], **extra}


def sequence_case(program: dict, caps=(0, 3, -1)) -> dict:
    """Программа + прогоны: без потолка и под потолками на долях итоговой траты (исчерпание в середине).

    Элемент `caps`: 0 — потолок 0; `-1` — итог минус единица; `k > 0` — итог, делённый на `k`.
    """

    program = dict(program, state_capped="digest")
    total = total_spent(program)
    runs = [run_program(program, None)]
    for item in caps if total else ():
        cap = 0 if item == 0 else total - 1 if item < 0 else total // item
        runs.append(run_program(program, cap))
    return {"program": program, "runs": runs}


def walls_program(rng: random.Random) -> dict:
    p = distinct_primes((18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 20, 22, 24), rng)
    r = [p[0] * p[1] * p[2], p[1] * p[3] * p[4] * p[5], p[0] * p[2] ** 2 * p[6], product(p[7:11]), product([p[0], p[3], *p[11:14]]), p[4] ** 3 * p[6] * p[12]]
    q = [Fraction(value, den) for value, den in zip(r, (3, 5, 1, 7, 2, 6))]
    calls = [
        universe_call(q[:4]),
        universe_call(q[:4], "remembered", store="s"),
        universe_call(q[:4], "remembered", store="s"),
        universe_call(q, "remembered", store="s"),
        universe_call([q[1], Fraction(0), q[1], Fraction(7), q[2]]),
    ]
    calls += [split_call(value) for value in r]
    calls += [support_call(p[0] * p[1] * p[2]), support_call(p[1] * p[3]), support_call(product(p[7:11])), support_call(p[4] * p[6] * p[12])]
    calls += [pairs_call(r[2]), pairs_call(r[5]), pairs_call(p[0] * p[1] * 1000003), split_call(p[9] ** 2), split_call(1), split_call(0)]
    return {"name": "walls_like", "state": "full", "calls": calls}


def fresh_program(rng: random.Random) -> dict:
    p = distinct_primes((16, 17, 18, 19, 20, 22), rng)
    fresh = [random_prime(bits, rng) for bits in (36, 44, 52, 60, 61, 62)]
    calls = [split_call(product(p[:3])), split_call(product(p[2:])), pairs_call(product(p))]
    calls += [split_call(product(p[:3]) * value) for value in fresh]
    calls += [split_call(fresh[0] * fresh[1]), split_call(fresh[2] ** 2 * p[0]), support_call(fresh[3] * p[1]), pairs_call(fresh[4] * fresh[5] * p[2])]
    calls += [split_call(random_prime(18, rng) * random_prime(21, rng) * fresh[0]), split_call(fresh[0])]
    return {"name": "fresh_cofactors", "state": "full", "calls": calls}


def big_program(rng: random.Random) -> dict:
    p = distinct_primes((22, 23, 24, 25, 26, 22, 23, 24, 25, 26), rng)
    r = [product(p[:8]), product(p[2:10]) * p[3], product(p[:4]) * product(p[5:9]) ** 2, product([p[1], p[4], p[7], p[9]]), p[0] ** 5 * product(p[5:8])]
    q = [Fraction(value, den) for value, den in zip(r, (1, 3, 5, 7, 1))]
    calls = [universe_call(q), universe_call(q, "remembered", store="big"), {"op": "seed", "radicands": [hx(value) for value in r]}]
    calls += [split_call(value) for value in r] + [support_call(value) for value in (product(p[:3]), product(p[3:8]))]
    calls.append({"op": "basis", "values": [hx(value) for value in r]})
    return {"name": "big_radicands", "state": "full", "calls": calls}


def errors_program(rng: random.Random) -> dict:
    p = distinct_primes((19, 21, 23, 25), rng)
    calls = [split_call(-5), support_call(-7), support_call(-1), support_call(0), support_call(1), split_call(product(p[:3]), none=True)]
    calls += [pairs_call(product(p), none=True), pairs_call(product(p)), pairs_call(-12), pairs_call(1)]
    calls += [universe_call([Fraction(5, 3), Fraction(-3, 4), Fraction(2)]), universe_call([Fraction(-6)], "remembered", store="e")]
    calls += [universe_call([Fraction(0), Fraction(1), Fraction(2, 2)]), universe_call([Fraction(0)], "remembered", store="e")]
    calls += [universe_call([Fraction(p[0] * p[1], 5), Fraction(p[1] * p[2], 7)], none=True), split_call(p[0] * p[3], none=True)]
    calls += [split_call(1000003 * 1000033), universe_call([Fraction(1000039 * 1000037, 3)], "remembered", store="e")]
    return {"name": "errors_and_unbudgeted", "state": "full", "unbudgeted": True, "calls": calls}


def memory_program(rng: random.Random) -> dict:
    p = distinct_primes((18, 20, 22, 24, 19, 21), rng)
    first = [p[0] * p[1], p[1] * p[2] * p[3]]
    second = [p[4] * p[5], p[0] * p[4] * 1000003, p[2] ** 2 * p[5]]
    inner = [split_call(first[0]), pairs_call(p[4] * 1000033), support_call(p[5] * p[0])]
    calls = [{"op": "mark", "name": "m0"}] + [split_call(value) for value in first] + [{"op": "mark", "name": "m1"}]
    calls += [split_call(value) for value in second] + [{"op": "delta", "name": "m0", "as": "d0"}, {"op": "delta", "name": "m1", "as": "d1"}]
    calls += [{"op": "isolated", "calls": inner}, split_call(first[0]), pairs_call(first[1]), {"op": "reset"}, split_call(first[0])]
    calls += [{"op": "replay", "name": "d1"}, split_call(second[0]), {"op": "replay", "name": "d0"}, pairs_call(second[1])]
    calls += [{"op": "reset"}, {"op": "replay", "name": "d0"}, split_call(second[2]), {"op": "isolated", "calls": [{"op": "reset"}, split_call(first[1])]}]
    return {"name": "reset_isolated_marker_delta", "state": "full", "calls": calls}


def store_program(rng: random.Random) -> dict:
    p = distinct_primes((18, 20, 22, 21, 19), rng)
    q1 = [Fraction(p[0] * p[1], 3), Fraction(p[1] * p[2], 1)]
    q2 = [Fraction(p[2] * p[3] * p[4], 1), Fraction(p[0] * p[4], 7)]
    calls = [universe_call(q1, "remembered", store="a"), universe_call(q2, "remembered", store="a"), {"op": "reset"}]
    calls += [universe_call(q1, "remembered", store="a"), split_call(p[0] * p[1] * 3), universe_call(q2, "remembered", store="a")]
    calls += [{"op": "reset"}, universe_call(q1, "remembered", store="b"), universe_call(q1, "remembered", store="a"), universe_call(q1, "universe")]
    return {"name": "store_hit_after_reset", "state": "full", "calls": calls}


def build_sequences() -> None:
    rng = random.Random(1010)
    makers = (walls_program, fresh_program, big_program, errors_program, memory_program, store_program)
    cases = [sequence_case(maker(rng)) for maker in makers]
    write_vectors("sequences", {}, cases)


# --------------------------------------------------------------------------
# Свипы потолка: для каждого потолка 0..итог+1 точка исчерпания и частичная память (сжато подряд идущими группами)
# --------------------------------------------------------------------------


def sweep_outcome(program: dict, cap: int):
    run = Run(program, cap)
    records = run.execute(stop_on_exhaustion=True)
    exhausted = run.budget.exhaustion is not None
    last = records[-1]
    state = capture_state()
    key = json.dumps([len(records) - 1 if exhausted else -1, last["r"], last["a"], state])
    return key, {"i": len(records) - 1 if exhausted else -1, "r": last["r"], "a": last["a"], "state": state}


def sweep_runs(program: dict) -> list:
    """Группы подряд идущих потолков с одним и тем же исходом: `{caps: [lo, hi], i, r, a, s}`.

    `i` — индекс вызова, на котором бюджет отказал (-1 — отказа нет); `r` — результат этого вызова, `a` — шесть статей на
    этот момент, `s` — отпечаток памяти в этот момент. Прогон останавливается на первом отказе.
    """

    quiet = dict(program, state="none")
    total = total_spent(quiet)
    groups: list = []
    previous_key = None
    for cap in range(0, total + 2):
        key, outcome = sweep_outcome(quiet, cap)
        if key == previous_key:
            groups[-1]["caps"][1] = cap
            continue
        previous_key = key
        groups.append({"caps": [cap, cap], "i": outcome["i"], "r": outcome["r"], "a": outcome["a"], "s": digest_state(outcome["state"], heads=False)})
    return groups


def sweep_case(program: dict) -> dict:
    groups = sweep_runs(program)
    last = groups[-1]
    print(f"sweep {program['name']}: caps 0..{last['caps'][1]}, {len(groups)} distinct outcomes")
    return {"program": dict(program, state="none"), "sweep": groups}


def sweep_programs(rng: random.Random) -> list:
    p = distinct_primes((14, 15, 14, 15, 13), rng)
    fresh = random_prime(31, rng)
    q = [Fraction(p[0] * p[1] * p[2], 3), Fraction(p[1] * p[3], 1), Fraction(p[0] * p[3] * p[2], 5)]
    first = {"name": "sweep_universe", "calls": [universe_call(q), split_call(p[0] * p[1] * p[2]), support_call(p[1] * p[3]), split_call(p[0] ** 2 * p[3])]}
    second = {"name": "sweep_remembered", "calls": [universe_call(q[:2], "remembered", store="s"), universe_call(q[:2], "remembered", store="s"), split_call(p[0] * p[1] * fresh)]}
    third = {"name": "sweep_basis", "calls": [{"op": "basis", "values": [hx(p[0] * p[1] * p[2]), hx(p[1] * p[3]), hx(p[2] * p[3] * p[4])]}, {"op": "seed", "radicands": [hx(p[0] * p[1] * p[4]), hx(p[1] * p[2] * p[4]), hx(p[3] * p[4] * 1000003)]}, pairs_call(p[3] * p[4] * 1000003)]}
    q4 = [Fraction(product(p[:3]) * fresh, 3), Fraction(p[1] * p[3] * fresh, 1), Fraction(p[2] * p[4] * fresh, 7)]
    fourth = {"name": "sweep_shared_fresh_prime", "calls": [universe_call(q4), universe_call(q4, "remembered", store="s"), split_call(product(p[:3]) * fresh), support_call(p[2] * p[4] * fresh)]}
    big = distinct_primes((17, 18, 19, 18), rng)
    fresh_large = random_prime(44, rng)
    fifth = {"name": "sweep_realistic", "calls": [universe_call([Fraction(product(big[:3]), 3), Fraction(big[1] * big[3] * fresh_large, 1), Fraction(big[0] * big[3], 5)]), split_call(product(big[:3]) * fresh_large), pairs_call(big[2] * big[3]), split_call(big[0] ** 3 * big[2])]}
    heavy = distinct_primes((21, 22, 23), rng)
    sixth = {"name": "sweep_heavy", "calls": [universe_call([Fraction(product(heavy), 3), Fraction(heavy[0] * heavy[1], 1)]), split_call(heavy[1] * heavy[2]), support_call(product(heavy))]}
    return [first, second, third, fourth, fifth, sixth]


def build_sweeps() -> None:
    rng = random.Random(1011)
    write_vectors("sweeps", {}, [sweep_case(program) for program in sweep_programs(rng)])


# --------------------------------------------------------------------------
# Пороги: 8191 / 8192 записей (LRU-вытеснение разложений, очистка реестра простых) на синтетическом заполнении
# --------------------------------------------------------------------------


def fill_call(table: str, start: int, count: int, step: int = 1) -> dict:
    return {"op": "fill", "table": table, "rule": {"start": hx(start), "step": step, "count": count}}


def threshold_programs(rng: random.Random) -> list:
    s = distinct_primes((10, 10, 11, 11, 12, 12, 10, 11), rng)
    key0 = 10**12
    products = [s[i] * s[(i + 1) % len(s)] for i in range(len(s))]
    lru = {
        "name": "lru_eviction_at_limit",
        "before": {"fill": {"f": {"start": hx(key0), "count": FACTORIZATION_LIMIT - 1}}},
        "calls": [pairs_call(products[0]), pairs_call(products[1]), pairs_call(key0 + 1), pairs_call(products[2]), split_call(products[3]), pairs_call(products[0]), pairs_call(key0 + 3), pairs_call(products[4]), pairs_call(products[5]), pairs_call(key0 + 1), pairs_call(products[6])],
    }
    registry = {
        "name": "registry_clear_at_limit",
        "before": {"fill": {"p": {"start": hx(1 << 300), "step": 2, "count": REGISTRY_LIMIT - 1}}},
        "calls": [pairs_call(products[0]), pairs_call(products[1]), pairs_call(products[2]), split_call(products[3]), support_call(products[4]), universe_call([Fraction(products[5], 3), Fraction(products[6])], "remembered", store="s"), universe_call([Fraction(products[5], 3), Fraction(products[6])], "remembered", store="s")],
    }
    hit_full = {
        "name": "store_hit_into_full_tables",
        "calls": [universe_call([Fraction(products[0], 3), Fraction(products[1], 1)], "remembered", store="s"), {"op": "reset"}, fill_call("p", 1 << 300, REGISTRY_LIMIT, 2), fill_call("f", key0, FACTORIZATION_LIMIT), universe_call([Fraction(products[0], 3), Fraction(products[1], 1)], "remembered", store="s"), split_call(products[0]), pairs_call(products[2])],
    }
    delta = {
        "name": "delta_replay_at_limit",
        "before": {"fill": {"f": {"start": hx(key0), "count": FACTORIZATION_LIMIT - 2}, "p": {"start": hx(1 << 300), "step": 2, "count": REGISTRY_LIMIT - 2}}},
        "calls": [{"op": "mark", "name": "m"}, pairs_call(products[0]), pairs_call(products[1]), pairs_call(products[2]), {"op": "delta", "name": "m", "as": "d"}, {"op": "reset"}, fill_call("f", key0, FACTORIZATION_LIMIT - 1), fill_call("p", 1 << 300, REGISTRY_LIMIT - 1, 2), {"op": "replay", "name": "d"}, split_call(products[3])],
    }
    return [lru, registry, hit_full, delta]


def threshold_case(program: dict) -> dict:
    program = dict(program, state="digest")
    total = total_spent(program)
    caps = [None] + ([total // 3, total - 1] if total else [])
    return {"program": program, "runs": [run_program(program, cap) for cap in caps]}


def build_thresholds() -> None:
    rng = random.Random(1012)
    write_vectors("thresholds", {}, [threshold_case(program) for program in threshold_programs(rng)])


# --------------------------------------------------------------------------
# Корпус (только чтение): настоящие q-векторы покрытия и настоящая память до вызова
# --------------------------------------------------------------------------

CORPUS_BASE = "E:/cftuv_native_corpus"
CORPUS_MAX_RECORDS = 9
CORPUS_MAX_SECONDS = 1.0


def newest_corpus_index() -> Path | None:
    import os

    base = Path(os.environ.get("CFTUV_NATIVE_CORPUS", CORPUS_BASE))
    indexes = [path for path in base.glob("*/index.json") if path.is_file()]
    return max(indexes, key=lambda path: path.stat().st_mtime) if indexes else None


def corpus_q_lists(state) -> list:
    return [key[1] for key, _value in (state.store or []) if isinstance(key, tuple) and len(key) == 2 and key[0] == "prime-universe"]


def corpus_domains(index: dict, mesh: str) -> list:
    """Домены сетки по убыванию суммарного времени покрытия (до секунды на запись); записи домена — по порядку вызовов."""

    domains: dict = {}
    for item in index["records"]:
        if item["mesh"] == mesh and item["op"] == "coverage_at" and item["seconds"] <= CORPUS_MAX_SECONDS and not item["exception"]:
            domains.setdefault(item["domain_id"], []).append(item)
    ordered = sorted(domains.values(), key=lambda items: -sum(item["seconds"] for item in items))
    return [sorted(items, key=lambda item: item["seq"]) for items in ordered]


def corpus_program(label: str, q_lists: list) -> dict:
    """Настоящие q-векторы одного домена по порядку вызовов: промах, затем разложение радикандов, затем повтор (попадание)."""

    calls: list = []
    for q_values in q_lists:
        calls.append(universe_call(q_values, "remembered", store="c"))
        radicands = sorted({q.numerator * q.denominator for q in q_values} - {0, 1})
        for radicand in radicands[:3]:
            calls += [split_call(radicand), support_call(radicand)]
    calls.append(universe_call(q_lists[0], "remembered", store="c"))
    return {"name": label, "state": "digest", "calls": calls}


def corpus_pick(index_path: Path, index: dict, mesh: str, read_record) -> list:
    """Для сетки: три самых дорогих домена; q-векторы их записей — различные, не больше четырёх на домен."""

    programs: list = []
    for items in corpus_domains(index, mesh)[:3]:
        lists: list = []
        seen: set = set()
        for item in items[:12]:
            for q_values in corpus_q_lists(read_record(index_path.parent / item["path"]).expected().after):
                text = json.dumps([q_text(q) for q in q_values])
                if q_values and text not in seen and len(lists) < 4:
                    seen.add(text)
                    lists.append(q_values)
        if lists:
            programs.append(corpus_program(f"{mesh}:{items[0]['domain_id'][-8:]}", lists))
    return programs


def build_corpus() -> None:
    index_path = newest_corpus_index()
    if index_path is None:
        print("corpus: no index.json found, corpus.json NOT written")
        return
    sys.path.insert(0, str(ROOT / "tools"))
    import native_corpus

    index = json.loads(index_path.read_text(encoding="utf-8"))
    programs: list = []
    for mesh in sorted({item["mesh"] for item in index["records"]}):
        programs += corpus_pick(index_path, index, mesh, native_corpus.read_record)
    cases = [sequence_case(program, caps=(3, -1)) for program in programs[:CORPUS_MAX_RECORDS]]
    print(f"corpus: {len(cases)} programs from {index_path.parent.name}")
    header = {"corpus_git_head": index.get("git_head"), "kernel_identity": index.get("kernel_identity")}
    write_vectors("corpus", header, cases)


# --------------------------------------------------------------------------
# Цена единицы бюджета эталона (нс): боевой путь `_pollard_rho_brent_attempt` и `_is_prime`, а не голый цикл
# --------------------------------------------------------------------------


def brent_ns_per_unit(modulus: int, cap: int) -> tuple:
    rng = random.Random(modulus)
    c, y = rng.randrange(1, modulus), rng.randrange(0, modulus)
    budget = RecordingBudget(cap=cap)
    started = time.perf_counter()
    try:
        exact._pollard_rho_brent_attempt(modulus, y, c, batch_size=64, budget=budget)
    except Exhausted:
        pass
    return (time.perf_counter() - started) * 1e9 / budget.spent, budget.spent


def mr_ns_per_unit(prime: int, repeats: int) -> tuple:
    spent = 0
    started = time.perf_counter()
    for _ in range(repeats):
        budget = unbounded()
        exact._is_prime(prime, budget)
        spent += budget.spent
    return (time.perf_counter() - started) * 1e9 / spent, spent // repeats


def run_timing() -> None:
    rng = random.Random(20261006)
    print(f"python {sys.version.split()[0]}: {'case':<34} {'ns/unit':>10} {'units/call':>12}")
    for label, half_bits in (("120-bit", 60), ("250-bit", 125)):
        modulus = random_prime(half_bits, rng) * random_prime(half_bits, rng)
        ns, units = brent_ns_per_unit(modulus, 300_000)
        print(f"{'':>{len(sys.version.split()[0]) + 9}}brent orbit {label} semiprime{'':<10}{ns:>10.1f} {units:>12}")
        ns, units = mr_ns_per_unit(random_prime(half_bits * 2, rng), 100)
        print(f"{'':>{len(sys.version.split()[0]) + 9}}miller-rabin {label} prime{'':<12}{ns:>10.1f} {units:>12}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--only", nargs="*", default=None)
    parser.add_argument("--timing", action="store_true")
    arguments = parser.parse_args(argv)
    if arguments.timing:
        run_timing()
        return 0
    for name, builder in BUILDERS.items():
        if arguments.only is None or name in arguments.only:
            builder()
    return 0


BUILDERS = {
    "pyrandom": build_pyrandom,
    "primality": build_primality,
    "brent": build_brent,
    "rho_factors": build_rho_factors,
    "coprime": build_coprime,
    "sequences": build_sequences,
    "sweeps": build_sweeps,
    "thresholds": build_thresholds,
    "corpus": build_corpus,
}

if __name__ == "__main__":
    raise SystemExit(main())
