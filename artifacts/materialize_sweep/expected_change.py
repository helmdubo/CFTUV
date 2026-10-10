"""Ожидаемое изменение ответа ворот: спецификация закона и её проверка (одна на `sweep.py compare` и `gate.py compare`).

Срез, который меняет ответ осознанно (закон станции на хорде, закон резки, закон JOIN, закон веера), раньше не имел
в воротах места: `--expect-changed` и `--ignore-counters` держали разрешённое поимённо в коде под ОДИН закон, и
следующие релизы шли сверкой «по полям» отдельными скриптами. Ворота молча слабели. Теперь разрешённое — ДАННЫЕ,
лежащие рядом с инструментом (`specs/<имя>.json`), а проверка одна:

    ответ домена == ответу в базе во ВСЁМ, кроме того, что спецификация разрешила именно ЭТОМУ домену.

Как пользоваться (Python 3.10+, без Blender, только stdlib):

    python sweep.py run --out base.json ...            # прогон ДО среза (на основе или флагом закона `--chord-station off`)
    python sweep.py run --out new.json ...             # прогон ПОСЛЕ
    python sweep.py compare base.json new.json                      # без спецификации: любое расхождение — UNEXPECTED
    python sweep.py compare base.json new.json --spec chord_station # закон по спецификации specs/chord_station.json
    python sweep.py compare --list-specs                            # сохранённые спецификации
    python numeric_repr/gate.py compare base.json new.json --spec right_angle_stable

Вердикт — ровно одна строка из трёх (код возврата 0, 0, 1):

    IDENTICAL                                       ни одно поле ответа ни одного домена не изменилось
    EXPECTED-CHANGE (spec X): N domains             изменились ТОЛЬКО объявленные домены и ТОЛЬКО разрешённым
    UNEXPECTED (spec X): K problems; first: ...     всё остальное; каждая проблема напечатана строкой выше

Новая спецификация — копия ближайшей из `specs/` (схема `expected_change_spec_v1` в JSON; неизвестный ключ — отказ,
а не «молча игнорируется»). Сначала снять запись «до» и «после», затем перечислить, что изменилось на самом деле
(`compare` без спецификации печатает каждое различие), и записать в спецификацию ровно это, с объяснением в `about`.

    name, tool ("sweep" | "gate"), law, about      имя равно имени файла; `tool` — для какого инструмента
    domains    РОВНО одно из:
                 {"explicit": [0, 1, 3]}                    номера патчей (на всех плотностях)
                 {"by_density": {"1": [..], "2": [..]}}     свои списки на плотность
                 {"where": [{"side": "base|new|either", "field"|"counter": "...", "op": "eq|ne|gt|ge|lt|le|in|not_in",
                             "value": ...}]}                предикат (И) над строкой: кривые домены, домены со стыком JOIN
                 {"all": true}                              закон меняет каждый домен
    must_change  "each" (по умолчанию: каждый объявленный домен ОБЯЗАН измениться) | "any" (хотя бы один) | "none"
    allow        что вправе измениться у ОБЪЯВЛЕННОГО домена: fields (точные имена полей), digests (виды дайджестов),
                 counters (точные имена), counter_prefixes (приставки из >= двух слов), diagnostics
                 ([{"prefix": "ИМЯ"}] — строки с этим именем меняются как угодно; [{"prefix": "ИМЯ", "numbers": ["k"]}] —
                 меняются только числа `k=...`), outcome_transitions ([{"field", "from", "to"}] — смена исхода ТОЛЬКО по паре)
    groups       вместо `domains`/`must_change`/`allow` на верхнем уровне — список групп `{"name", "domains", "must_change",
                 "allow"}`: у разных доменов закона разные разрешения (домен берёт ПЕРВУЮ подходящую группу)
    allow_everywhere  то же, но на ЛЮБОМ домене — только счётчики и диагностики самого закона (его бухгалтерия);
                 дайджесты, поля и исходы сюда не допускаются
    require      ИНВАРИАНТЫ: changed_any_of (хоть одно из имён у объявленного домена обязано измениться), non_empty (поле
                 у объявленного домена непусто: отказ — не сдвиг ответа), unchanged (имя неизменно, даже если allow его
                 разрешает), zero / non_increasing / non_decreasing (счётчики НОВОЙ записи на каждом домене),
                 no_refusal (домен, давший ответ в базе, даёт ответ и в новой записи)

Отсутствующий счётчик равен нулю ВЕЗДЕ (`counters`, `topology_counters`, `untracked_counters`): новый нулевой счётчик
записи не делает разной, а новый НЕнулевой, не объявленный в спецификации, — расхождение. Имя в спецификации, которого
нет ни в словаре инструмента, ни в одной из записей, печатается строкой NOTE (опечатка либо счётчик, который свип не пишет).
Домен вне объявленных групп обязан совпасть во всём, кроме `allow_everywhere`; лишний изменившийся домен валит ворота.
"""

from __future__ import annotations

import dataclasses
import json
import re
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
SPECS_DIR = HERE / "specs"
SPEC_SCHEMA = "expected_change_spec_v1"
TOOLS = ("sweep", "gate", "field")
MUST_CHANGE_MODES = ("each", "any", "none")
OPERATORS = ("eq", "ne", "gt", "ge", "lt", "le", "in", "not_in")
SIDES = ("base", "new", "either")
_MAX_SHOWN = 100


class SpecError(ValueError):
    """Спецификация нарушает схему: отказ ДО сравнения (код возврата 2), а не молчаливое «ничего не разрешено»."""


@dataclasses.dataclass(frozen=True)
class Vocabulary:
    """Что инструмент пишет в строку: проверка имён спецификации и ответ на «это дайджест или поле»."""

    digests: frozenset
    fields: frozenset
    counters: frozenset


@dataclasses.dataclass
class RowView:
    """Ответ домена, приведённый к одному виду: поля, счётчики (отсутствующий равен нулю), диагностики по строкам."""

    fields: dict
    counters: dict
    diagnostics: tuple = ()
    ok: bool = True
    notes: tuple = ()


@dataclasses.dataclass(frozen=True)
class AllowList:
    fields: frozenset = frozenset()
    digests: frozenset = frozenset()
    counters: frozenset = frozenset()
    counter_prefixes: tuple = ()
    diagnostics: tuple = ()
    outcome_transitions: tuple = ()

    def allows_field(self, name: str) -> bool:
        return name in self.fields or name in self.digests

    def allows_counter(self, name: str) -> bool:
        return name in self.counters or any(name.startswith(prefix) for prefix in self.counter_prefixes)

    def allows_diagnostic(self, prefix: str, before: tuple, after: tuple) -> bool:
        for entry_prefix, numbers in self.diagnostics:
            if entry_prefix != prefix:
                continue
            if numbers is None or _masked(before, numbers) == _masked(after, numbers):
                return True
        return False

    def allows_transition(self, name: str, before, after) -> bool:
        return any(
            field == name and before in froms and after in tos
            for field, froms, tos in self.outcome_transitions
        )

    def names(self) -> set:
        return set(self.fields) | set(self.digests) | set(self.counters)


@dataclasses.dataclass(frozen=True)
class Require:
    changed_any_of: tuple = ()
    non_empty: tuple = ()
    unchanged: frozenset = frozenset()
    zero: tuple = ()
    non_increasing: tuple = ()
    non_decreasing: tuple = ()
    no_refusal: bool = False


@dataclasses.dataclass(frozen=True)
class Group:
    """Объявленные домены и то, что вправе измениться ИМ."""

    name: str
    selector: str
    selector_value: object
    must_change: str
    allow: AllowList

    def selects(self, density: str, patch: str, base: RowView, new: RowView) -> bool:
        if self.selector == "all":
            return True
        if self.selector == "explicit":
            return int(patch) in self.selector_value
        if self.selector == "by_density":
            return int(patch) in self.selector_value.get(str(density), ())
        return all(_clause_holds(clause, base, new) for clause in self.selector_value)

    def declared_ids(self, density: str) -> frozenset:
        if self.selector == "explicit":
            return self.selector_value
        if self.selector == "by_density":
            return self.selector_value.get(str(density), frozenset())
        return frozenset()


@dataclasses.dataclass(frozen=True)
class Spec:
    name: str
    tool: str
    law: str
    about: str
    groups: tuple
    everywhere: AllowList
    require: Require

    def group_of(self, density: str, patch: str, base: RowView, new: RowView):
        return next((group for group in self.groups if group.selects(density, patch, base, new)), None)


@dataclasses.dataclass(frozen=True)
class Change:
    kind: str  # "field" | "counter" | "diagnostic"
    name: str
    before: object
    after: object


@dataclasses.dataclass
class Report:
    """Итог сравнения: проблемы (пусто — ворота пройдены), изменённые объявленные строки, примечания."""

    spec_name: str | None
    problems: list = dataclasses.field(default_factory=list)
    changed_rows: list = dataclasses.field(default_factory=list)
    bookkeeping_rows: int = 0
    rows_compared: int = 0
    declared_rows: Counter = dataclasses.field(default_factory=Counter)
    changed_by_group: Counter = dataclasses.field(default_factory=Counter)
    unexpected_rows: Counter = dataclasses.field(default_factory=Counter)
    touched: Counter = dataclasses.field(default_factory=Counter)
    notes: Counter = dataclasses.field(default_factory=Counter)

    @property
    def exit_code(self) -> int:
        return 1 if self.problems else 0

    @property
    def changed_domains(self) -> list:
        return sorted({patch for _, patch in self.changed_rows}, key=int)


# --------------------------------------------------------------------------
# Схема спецификации
# --------------------------------------------------------------------------

_TOP_KEYS = {
    "schema", "name", "tool", "law", "about", "domains", "must_change", "allow", "groups", "allow_everywhere", "require",
}
_GROUP_KEYS = {"name", "domains", "must_change", "allow"}
_ALLOW_KEYS = {"fields", "digests", "counters", "counter_prefixes", "diagnostics", "outcome_transitions"}
_EVERYWHERE_KEYS = {"counters", "counter_prefixes", "diagnostics"}
_REQUIRE_KEYS = {"changed_any_of", "non_empty", "unchanged", "zero", "non_increasing", "non_decreasing", "no_refusal"}
_CLAUSE_KEYS = {"side", "field", "counter", "op", "value"}
_NAME = re.compile(r"^[A-Za-z0-9_.]+$")


def _strings(value, where: str, errors: list) -> tuple:
    if not isinstance(value, list) or not all(isinstance(item, str) and _NAME.match(item) for item in value):
        errors.append(f"{where}: expected a list of names")
        return ()
    if len(set(value)) != len(value):
        errors.append(f"{where}: duplicate names")
    return tuple(value)


def _prefix_problem(prefix) -> str:
    if not isinstance(prefix, str) or not prefix.endswith("_") or prefix.rstrip("_").count("_") < 1:
        return f"counter prefix {prefix!r} must end with '_' and hold at least two words (a short prefix hides the whole answer)"
    return ""


def _parse_allow(raw, where: str, allowed_keys: set, errors: list) -> AllowList:
    if raw is None:
        return AllowList()
    if not isinstance(raw, dict):
        errors.append(f"{where}: expected an object")
        return AllowList()
    for key in sorted(set(raw) - allowed_keys):
        errors.append(f"{where}: key {key!r} is not allowed here (allowed: {sorted(allowed_keys)})")
    fields = _strings(raw.get("fields", []), f"{where}.fields", errors)
    digests = _strings(raw.get("digests", []), f"{where}.digests", errors)
    counters = _strings(raw.get("counters", []), f"{where}.counters", errors)
    prefixes = _strings(raw.get("counter_prefixes", []), f"{where}.counter_prefixes", errors)
    for prefix in prefixes:
        problem = _prefix_problem(prefix)
        if problem:
            errors.append(f"{where}: {problem}")
    diagnostics = []
    for index, entry in enumerate(raw.get("diagnostics", [])):
        entry_where = f"{where}.diagnostics[{index}]"
        if not isinstance(entry, dict) or set(entry) - {"prefix", "numbers"} or not isinstance(entry.get("prefix"), str):
            errors.append(f"{entry_where}: expected {{\"prefix\": NAME, \"numbers\": [names]?}}")
            continue
        numbers = None if "numbers" not in entry else _strings(entry["numbers"], f"{entry_where}.numbers", errors)
        diagnostics.append((entry["prefix"], numbers))
    transitions = []
    for index, entry in enumerate(raw.get("outcome_transitions", [])):
        entry_where = f"{where}.outcome_transitions[{index}]"
        if not isinstance(entry, dict) or set(entry) != {"field", "from", "to"} or not isinstance(entry["field"], str):
            errors.append(f"{entry_where}: expected {{\"field\", \"from\", \"to\"}}")
            continue
        sides = [frozenset(item if isinstance(item, list) else [item]) for item in (entry["from"], entry["to"])]
        transitions.append((entry["field"], sides[0], sides[1]))
    return AllowList(
        fields=frozenset(fields),
        digests=frozenset(digests),
        counters=frozenset(counters),
        counter_prefixes=prefixes,
        diagnostics=tuple(diagnostics),
        outcome_transitions=tuple(transitions),
    )


def _parse_clause(clause, where: str, errors: list) -> dict:
    if not isinstance(clause, dict) or set(clause) - _CLAUSE_KEYS:
        errors.append(f"{where}: expected an object with keys {sorted(_CLAUSE_KEYS)}")
        return {}
    if ("field" in clause) == ("counter" in clause):
        errors.append(f"{where}: exactly one of 'field' and 'counter'")
    if clause.get("op") not in OPERATORS:
        errors.append(f"{where}: op must be one of {list(OPERATORS)}")
    if clause.get("side", "either") not in SIDES:
        errors.append(f"{where}: side must be one of {list(SIDES)}")
    if "value" not in clause:
        errors.append(f"{where}: 'value' is required")
    elif clause.get("op") in ("in", "not_in") and not isinstance(clause["value"], list):
        errors.append(f"{where}: op {clause['op']!r} needs a list value")
    return clause


def _parse_selector(raw, where: str, errors: list):
    if not isinstance(raw, dict) or len(raw) != 1 or next(iter(raw)) not in ("explicit", "by_density", "where", "all"):
        errors.append(f"{where}: exactly one of explicit / by_density / where / all")
        return "all", None
    kind, value = next(iter(raw.items()))
    if kind == "explicit":
        if not isinstance(value, list) or not value or not all(type(item) is int for item in value):
            errors.append(f"{where}.explicit: a non-empty list of patch ids")
            return kind, frozenset()
        return kind, frozenset(value)
    if kind == "by_density":
        ok = isinstance(value, dict) and value and all(
            str(key).isdigit() and isinstance(items, list) and items and all(type(item) is int for item in items)
            for key, items in value.items()
        )
        if not ok:
            errors.append(f"{where}.by_density: {{\"<density>\": [patch ids]}} with non-empty lists")
            return kind, {}
        return kind, {str(key): frozenset(items) for key, items in value.items()}
    if kind == "where":
        if not isinstance(value, list) or not value:
            errors.append(f"{where}.where: a non-empty list of clauses")
            return kind, ()
        return kind, tuple(_parse_clause(clause, f"{where}.where[{index}]", errors) for index, clause in enumerate(value))
    if value is not True:
        errors.append(f"{where}.all: must be true")
    return kind, True


def _parse_group(raw, where: str, default_name: str, errors: list) -> Group:
    selector, selector_value = _parse_selector(raw.get("domains"), f"{where}.domains", errors)
    must_change = raw.get("must_change", "each")
    if must_change not in MUST_CHANGE_MODES:
        errors.append(f"{where}.must_change: one of {list(MUST_CHANGE_MODES)}")
    allow = _parse_allow(raw.get("allow"), f"{where}.allow", _ALLOW_KEYS, errors)
    name = raw.get("name", default_name)
    if not isinstance(name, str) or not name:
        errors.append(f"{where}.name: a non-empty string")
        name = default_name
    return Group(name, selector, selector_value, must_change, allow)


def _parse_groups(raw: dict, errors: list) -> tuple:
    flat = {"domains", "must_change", "allow"} & set(raw)
    if "groups" in raw and flat:
        errors.append(f"groups and {sorted(flat)} are alternatives: put domains/must_change/allow inside each group")
        return ()
    if "groups" not in raw:
        return (_parse_group(raw, "spec", "declared", errors),)
    items = raw["groups"]
    if not isinstance(items, list) or not items:
        errors.append("groups: a non-empty list")
        return ()
    groups = []
    for index, item in enumerate(items):
        where = f"groups[{index}]"
        if not isinstance(item, dict) or set(item) - _GROUP_KEYS:
            errors.append(f"{where}: expected an object with keys {sorted(_GROUP_KEYS)}")
            continue
        groups.append(_parse_group(item, where, f"group{index}", errors))
    if len({group.name for group in groups}) != len(groups):
        errors.append("groups: duplicate names")
    return tuple(groups)


def _parse_require(raw, errors: list) -> Require:
    if raw is None:
        return Require()
    if not isinstance(raw, dict):
        errors.append("require: expected an object")
        return Require()
    for key in sorted(set(raw) - _REQUIRE_KEYS):
        errors.append(f"require: unknown key {key!r} (allowed: {sorted(_REQUIRE_KEYS)})")
    if "no_refusal" in raw and not isinstance(raw["no_refusal"], bool):
        errors.append("require.no_refusal: expected true or false")
    return Require(
        changed_any_of=_strings(raw.get("changed_any_of", []), "require.changed_any_of", errors),
        non_empty=_strings(raw.get("non_empty", []), "require.non_empty", errors),
        unchanged=frozenset(_strings(raw.get("unchanged", []), "require.unchanged", errors)),
        zero=_strings(raw.get("zero", []), "require.zero", errors),
        non_increasing=_strings(raw.get("non_increasing", []), "require.non_increasing", errors),
        non_decreasing=_strings(raw.get("non_decreasing", []), "require.non_decreasing", errors),
        no_refusal=raw.get("no_refusal") is True,
    )


def _check_vocabulary(allow: AllowList, vocabulary: Vocabulary, where: str, errors: list) -> None:
    """Дайджесты называются дайджестами, поля — полями: так «кто разрешает сдвиг дайджеста» находится поиском одного слова."""

    for name in sorted(allow.digests - vocabulary.digests):
        errors.append(f"{where}.digests: {name!r} is not a digest of this tool ({sorted(vocabulary.digests)})")
    for name in sorted(allow.fields & vocabulary.digests):
        errors.append(f"{where}.fields: {name!r} is a digest; name it under digests")


def parse_spec(raw: dict, tool: str, vocabulary: Vocabulary | None = None, source: str = "<spec>") -> Spec:
    """Спецификацию из JSON-объекта; все нарушения схемы собираются в один `SpecError`."""

    errors: list[str] = []
    if not isinstance(raw, dict):
        raise SpecError(f"{source}: expected an object")
    if raw.get("schema") != SPEC_SCHEMA:
        errors.append(f"schema must be {SPEC_SCHEMA!r}")
    for key in sorted(set(raw) - _TOP_KEYS):
        errors.append(f"unknown key {key!r}")
    for key in ("name", "law", "about"):
        if not isinstance(raw.get(key), str) or not raw.get(key):
            errors.append(f"{key} is required (a non-empty string)")
    if raw.get("tool") not in TOOLS:
        errors.append(f"tool must be one of {list(TOOLS)}")
    elif raw.get("tool") != tool:
        errors.append(f"spec is for the {raw.get('tool')!r} tool, not {tool!r}")
    groups = _parse_groups(raw, errors)
    everywhere = _parse_allow(raw.get("allow_everywhere"), "allow_everywhere", _EVERYWHERE_KEYS, errors)
    require = _parse_require(raw.get("require"), errors)
    if vocabulary is not None:
        for group in groups:
            _check_vocabulary(group.allow, vocabulary, f"{group.name}.allow", errors)
    if errors:
        raise SpecError(f"{source}:\n  " + "\n  ".join(errors))
    return Spec(raw["name"], raw["tool"], raw["law"], raw["about"], groups, everywhere, require)


def load_spec(ref: str, tool: str, vocabulary: Vocabulary | None = None) -> Spec:
    """Спецификация по имени из `specs/` либо по пути к `.json`."""

    path = Path(ref)
    stored = path.suffix != ".json" and path.name == ref
    if stored:
        path = SPECS_DIR / f"{ref}.json"
    if not path.is_file():
        raise SpecError(f"spec {ref!r} not found at {path}; stored specs: {stored_spec_names()}")
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise SpecError(f"{path}: not JSON ({error})") from error
    spec = parse_spec(raw, tool, vocabulary, str(path))
    if stored and spec.name != ref:
        raise SpecError(f"{path}: name {spec.name!r} must equal the file name {ref!r}")
    return spec


def stored_spec_names() -> list:
    return sorted(path.stem for path in SPECS_DIR.glob("*.json"))


def print_stored_specs(tool: str) -> None:
    """`compare --list-specs`: имя, закон и одна строка «о чём» каждой сохранённой спецификации инструмента."""

    for name in stored_spec_names():
        try:
            raw = json.loads((SPECS_DIR / f"{name}.json").read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            print(f"{name}: NOT JSON")
            continue
        if raw.get("tool") == tool:
            print(f"{name} [{raw.get('law')}]: {raw.get('about')}")


# --------------------------------------------------------------------------
# Предикат выбора доменов
# --------------------------------------------------------------------------


def _value_of(clause: dict, view: RowView):
    if "field" in clause:
        return view.fields.get(clause["field"])
    return view.counters.get(clause["counter"], 0)


def _compare(op: str, actual, expected) -> bool:
    if op == "eq":
        return actual == expected
    if op == "ne":
        return actual != expected
    if op == "in":
        return actual in expected
    if op == "not_in":
        return actual not in expected
    if actual is None or isinstance(actual, str) != isinstance(expected, str):
        return False
    return {"gt": actual > expected, "ge": actual >= expected, "lt": actual < expected, "le": actual <= expected}[op]


def _clause_holds(clause: dict, base: RowView, new: RowView) -> bool:
    views = {"base": (base,), "new": (new,), "either": (base, new)}[clause.get("side", "either")]
    return any(_compare(clause["op"], _value_of(clause, view), clause["value"]) for view in views)


# --------------------------------------------------------------------------
# Различия одного домена
# --------------------------------------------------------------------------


def _prefix_of(line: str) -> str:
    return line.split(":", 1)[0]


def _by_prefix(lines) -> dict:
    grouped: dict[str, list] = {}
    for line in lines:
        grouped.setdefault(_prefix_of(line), []).append(line)
    return {prefix: tuple(items) for prefix, items in grouped.items()}


def _masked(lines: tuple, numbers) -> tuple:
    if not numbers:
        return lines
    pattern = re.compile(r" (?:" + "|".join(re.escape(name) for name in numbers) + r")=\S+")
    return tuple(pattern.sub("", line) for line in lines)


def diff_views(base: RowView, new: RowView) -> list:
    """Все различия двух видов одного домена; цена (секунды) в виде не живёт, отсутствующий счётчик равен нулю."""

    changes = []
    for name in sorted(set(base.fields) | set(new.fields)):
        before, after = base.fields.get(name), new.fields.get(name)
        if before != after:
            changes.append(Change("field", name, before, after))
    for name in sorted(set(base.counters) | set(new.counters)):
        before, after = base.counters.get(name, 0), new.counters.get(name, 0)
        if before != after:
            changes.append(Change("counter", name, before, after))
    left, right = _by_prefix(base.diagnostics), _by_prefix(new.diagnostics)
    for prefix in sorted(set(left) | set(right)):
        if left.get(prefix, ()) != right.get(prefix, ()):
            changes.append(Change("diagnostic", prefix, left.get(prefix, ()), right.get(prefix, ())))
    return changes


def _covered(allow: AllowList, change: Change) -> bool:
    if change.kind == "field":
        return allow.allows_field(change.name) or allow.allows_transition(change.name, change.before, change.after)
    if change.kind == "counter":
        return allow.allows_counter(change.name)
    return allow.allows_diagnostic(change.name, change.before, change.after)


def _classify(spec: Spec | None, group: Group | None, change: Change) -> str:
    """`pinned` (запрещено инвариантом), `domain` (разрешено объявленному), `everywhere` (бухгалтерия закона), `unexpected`."""

    if spec is None:
        return "unexpected"
    if change.name in spec.require.unchanged:
        return "pinned"
    if _covered(spec.everywhere, change):
        return "everywhere"
    if group is not None and _covered(group.allow, change):
        return "domain"
    return "unexpected"


def _shown(value) -> str:
    text = repr(value)
    return text if len(text) <= _MAX_SHOWN else text[: _MAX_SHOWN - 3] + "..."


def _describe(change: Change, kind: str, spec: Spec | None, group: Group | None) -> str:
    if spec is None:
        where = "outside the spec"
    elif kind == "pinned":
        where = "pinned by require.unchanged"
    elif group is None:
        where = "domain is not declared"
    else:
        where = f"not allowed for group {group.name}"
    return f"UNEXPECTED {change.kind} {change.name} ({where}): {_shown(change.before)} -> {_shown(change.after)}"


# --------------------------------------------------------------------------
# Сравнение записей
# --------------------------------------------------------------------------


def _counter(view: RowView, name: str):
    return view.counters.get(name, 0)


def _check_values(spec: Spec, base: RowView, new: RowView, declared: bool, changes: list, prefix: str, report: Report) -> None:
    """Инварианты `require`: значения в НОВОЙ записи, а не только равенство."""

    require = spec.require
    for name in require.zero:
        if _counter(new, name) != 0:
            report.problems.append(f"{prefix}: counter {name} = {_counter(new, name)}, must be 0")
    for name in require.non_increasing:
        if _counter(new, name) > _counter(base, name):
            report.problems.append(f"{prefix}: counter {name} grew {_counter(base, name)} -> {_counter(new, name)}, must not")
    for name in require.non_decreasing:
        if _counter(new, name) < _counter(base, name):
            report.problems.append(f"{prefix}: counter {name} fell {_counter(base, name)} -> {_counter(new, name)}, must not")
    if require.no_refusal and base.ok and not new.ok:
        report.problems.append(f"{prefix}: the domain answered in the base and no longer does (no_refusal)")
    if not declared:
        return
    for name in require.non_empty:
        if not new.fields.get(name):
            report.problems.append(f"{prefix}: field {name} is empty in the new record (a refusal is not a shifted answer)")
    if require.changed_any_of and not any(change.name in require.changed_any_of for change in changes):
        report.problems.append(f"{prefix}: none of {list(require.changed_any_of)} changed (changed_any_of)")


def _evaluate_domain(spec, density, patch, vb, vn, prefix, report) -> None:
    group = None if spec is None else spec.group_of(density, patch, vb, vn)
    changes = diff_views(vb, vn)
    report.rows_compared += 1
    if group is not None:
        report.declared_rows[group.name] += 1
    domain_level = False
    bookkeeping = False
    for change in changes:
        kind = _classify(spec, group, change)
        if kind in ("unexpected", "pinned"):
            report.problems.append(f"{prefix}: {_describe(change, kind, spec, group)}")
            report.unexpected_rows[(density, patch)] += 1
            continue
        report.touched[f"{change.kind}:{change.name}"] += 1
        if kind == "everywhere":
            bookkeeping = True
        else:
            domain_level = True
    report.bookkeeping_rows += bookkeeping and not domain_level
    if domain_level:
        report.changed_rows.append((density, patch))
        report.changed_by_group[group.name] += 1
    if spec is None:
        return
    if group is not None and group.must_change == "each" and not domain_level:
        report.problems.append(
            f"{prefix}: expected change is absent (group {group.name} declares this domain and must_change is 'each')"
        )
    _check_values(spec, vb, vn, group is not None, changes, prefix, report)


def _note_or_problem(report: Report, message: str, partial: bool) -> None:
    if partial:
        report.notes[message + " (partial run)"] += 1
    else:
        report.problems.append(message)


def _check_declared_present(spec: Spec, density: str, common_patches: set, report: Report, label: str, partial: bool) -> None:
    for group in spec.groups:
        missing = sorted(item for item in group.declared_ids(density) if str(item) not in common_patches)
        if missing:
            _note_or_problem(
                report, f"{label} d{density}: group {group.name} declares domains absent from the compared records: {missing}", partial
            )


def evaluate(records, labels, spec: Spec | None, pair_views, vocabulary: Vocabulary | None = None, partial: bool = False) -> Report:
    """Сравнить `records[0]` (база) с каждой из остальных; `pair_views(base_row, new_row) -> (RowView, RowView)`.

    `partial` — прогон части доменов (`--only`): объявленные домены вне записей не ошибка, а примечание.
    """

    report = Report(spec_name=None if spec is None else spec.name)
    base = records[0]
    seen_names: set = set()
    for other, label in zip(records[1:], labels[1:]):
        densities = sorted(set(base["runs"]) & set(other["runs"]), key=int)
        for density in sorted(set(base["runs"]) ^ set(other["runs"]), key=int):
            report.notes[f"{label}: density {density} is present in only one record; not compared"] += 1
        if not densities:
            report.problems.append(f"{label}: the records share no density")
        for group in () if spec is None else spec.groups:
            if group.selector == "by_density":
                for density in sorted(set(group.selector_value) - set(densities), key=int):
                    _note_or_problem(
                        report,
                        f"{label}: group {group.name} declares density {density}, which is not among the compared densities {densities}",
                        partial,
                    )
        for density in densities:
            left = base["runs"][density]["domains"]
            right = other["runs"][density]["domains"]
            if set(left) != set(right):
                _note_or_problem(
                    report,
                    f"{label} d{density}: domain sets differ (only in base: {sorted(set(left) - set(right), key=int)[:6]}, "
                    f"only in new: {sorted(set(right) - set(left), key=int)[:6]})",
                    partial,
                )
            common = set(left) & set(right)
            if spec is not None:
                _check_declared_present(spec, density, common, report, label, partial)
            for patch in sorted(common, key=int):
                vb, vn = pair_views(left[patch], right[patch])
                for view in (vb, vn):
                    for note in view.notes:
                        report.notes[note] += 1
                    seen_names |= set(view.counters) | set(view.fields)
                _evaluate_domain(spec, density, patch, vb, vn, f"{label} d{density} patch{patch}", report)
    if spec is not None:
        _post_checks(spec, report, vocabulary, seen_names)
    return report


def _post_checks(spec: Spec, report: Report, vocabulary: Vocabulary | None, seen_names: set) -> None:
    for group in spec.groups:
        if group.must_change == "none":
            continue
        if not report.declared_rows[group.name]:
            report.problems.append(
                f"group {group.name} declares no domain in the compared records (must_change is {group.must_change!r})"
            )
        elif group.must_change == "any" and not report.changed_by_group[group.name]:
            report.problems.append(f"group {group.name}: must_change is 'any' and no declared domain changed")
    known = set(seen_names)
    if vocabulary is not None:
        known |= vocabulary.counters | vocabulary.fields | vocabulary.digests
    named = set(spec.everywhere.names())
    for group in spec.groups:
        named |= group.allow.names()
    for items in (spec.require.unchanged, spec.require.zero, spec.require.changed_any_of, spec.require.non_empty,
                  spec.require.non_increasing, spec.require.non_decreasing):
        named |= set(items)
    unknown = sorted(name for name in named if name not in known)
    if unknown:
        report.notes[
            f"the spec names {len(unknown)} fields/counters present in neither the tool vocabulary nor the records: {unknown[:6]}"
        ] += 1
    prefixes = list(spec.everywhere.counter_prefixes)
    for group in spec.groups:
        prefixes += list(group.allow.counter_prefixes)
    dead = sorted({prefix for prefix in prefixes if not any(name.startswith(prefix) for name in known)})
    if dead:
        report.notes[f"the spec names counter prefixes that match no recorded counter: {dead}"] += 1


# --------------------------------------------------------------------------
# Вывод
# --------------------------------------------------------------------------


def verdict_line(report: Report) -> str:
    if report.problems:
        spec = "" if report.spec_name is None else f" (spec {report.spec_name})"
        return f"UNEXPECTED{spec}: {len(report.problems)} problems; first: {report.problems[0]}"
    if report.changed_rows:
        return f"EXPECTED-CHANGE (spec {report.spec_name}): {len(report.changed_domains)} domains"
    return "IDENTICAL"


def print_report(report: Report, limit: int = 40) -> None:
    """Проблемы, примечания, сводка изменённого и ровно одна строка вердикта в конце."""

    for line in report.problems[:limit]:
        print(line)
    if len(report.problems) > limit:
        print(f"... and {len(report.problems) - limit} more problems")
    if report.unexpected_rows:
        rows = sorted(report.unexpected_rows.items(), key=lambda item: (int(item[0][0]), int(item[0][1])))
        shown = ", ".join(f"d{density} patch{patch} x{count}" for (density, patch), count in rows[:12])
        print(f"unexpected changes on {len(rows)} domain rows: {shown}" + (" ..." if len(rows) > 12 else ""))
    for note, count in sorted(report.notes.items()):
        print(f"NOTE: {note}" + (f" (x{count})" if count > 1 else ""))
    if report.changed_rows:
        per_density: dict[str, list] = {}
        for density, patch in report.changed_rows:
            per_density.setdefault(density, []).append(int(patch))
        for density in sorted(per_density, key=int):
            print(f"changed d{density}: {len(per_density[density])} domains {sorted(per_density[density])}")
        touched = ", ".join(f"{name} x{count}" for name, count in sorted(report.touched.items()))
        print(f"touched (domain rows): {touched}")
    if report.bookkeeping_rows:
        print(f"law bookkeeping (allow_everywhere) differs on {report.bookkeeping_rows} domain rows with an unchanged answer")
    declared = ", ".join(f"{name} {count}" for name, count in sorted(report.declared_rows.items())) or "none"
    print(f"rows compared {report.rows_compared}; declared by group: {declared}")
    print(verdict_line(report))
