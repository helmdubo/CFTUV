"""Память наклонов сертификата перетаскивания (PARAMETRIC_SLOPE_MEMO_V1, `materialize.parametric.SlopeMemoV1`).

Что утверждается, и чем оно проверено:

1. КЛЮЧ ПОЛОН. Ключ записи - перечень входов чистых функций наклонов (`_fact_key` для `_fact_slopes`; плоскость, треугольник, масштаб и наклон точки для `_plane_slope` / `_triangle_slope`), ширины в нём нет.
   Поля, которые `_fact_slopes` читает у грани кадра, пробега и прямой, сверяются с `FACT_KEY_READS` по исходнику: новое чтение без новой части ключа роняет тест.
2. РАВНЫЙ КЛЮЧ - ТОТ ЖЕ ОТВЕТ ПОБИТОВО. Сертификат с памятью и без неё (`CFTUV_PARAMETRIC_MEMO=0`) на одной подготовке при ширинах alpha0, трёх выше и трёх ниже внутри заверенного
   интервала и при случайных ширинах интервала (с семенем): байты float-массивов, причины, текст, счётчики, `approximate`, `approximate_rate`, ширина и масштаб равны; память при этом попадает.
3. НИЧЕГО НЕ МОЛЧИТ. Каждый сертификат несёт метку (`MEMO_LABELS`); точный режим и выключатели названы своими метками.
"""

from __future__ import annotations

import ast
import inspect
import random
import textwrap
from fractions import Fraction

import pytest

import parametric_theorem as theorem
from test_parametric_certificate import CASES, IDS, alpha_of, step

from cftuv_envelope.materialize import parametric

SEED = 20261019
RANDOM_WIDTHS = 6


def certificate_bytes(certificate) -> tuple:
    """Всё, что сертификат отдаёт хосту, кроме метки памяти: float-массивы байтами, `approximate_rate` - шестнадцатеричной записью."""

    return (
        certificate.law, certificate.alpha, certificate.scale, certificate.markers, certificate.detail, certificate.vertex_count, certificate.loop_count,
        bytes(certificate.position_slopes), bytes(certificate.loop_slopes), certificate.counters, certificate.approximate, float(certificate.approximate_rate).hex(),
    )


def inside_widths(base, alpha0: Fraction) -> list:
    """Три ширины выше и три ниже alpha0 внутри заверенного интервала (как в полевом профиле: 0.1 %, 1 %, 1/5 пути до края, не дальше 25 %) и случайные ширины интервала (семя)."""

    interval = base.result.interval
    if interval is None or interval.status != "CERTIFIED":
        return []
    high = alpha0 * Fraction(5, 4) if interval.high is None else min(Fraction(interval.high), alpha0 * Fraction(5, 4))
    low = max(Fraction(interval.low), alpha0 * Fraction(3, 4))
    found = []
    for share in (Fraction(1, 1000), Fraction(1, 100), Fraction(1, 5)):
        found.append(alpha0 + (high - alpha0) * share)
        found.append(alpha0 - (alpha0 - low) * share)
    generator = random.Random(SEED)
    found.extend(low + (high - low) * Fraction(generator.randint(1, 999), 1000) for _ in range(RANDOM_WIDTHS))
    texts = []
    for value in found:
        text = theorem.decimal_text(value, 9)
        if Fraction(text) > 0 and interval.contains_exact(Fraction(text)) and text not in texts:
            texts.append(text)
    return texts


@pytest.mark.parametrize("name,build,laws,given,clipped", CASES, ids=IDS)
def test_equal_key_gives_the_same_certificate_byte_for_byte_at_every_width(name, build, laws, given, clipped, monkeypatch):
    prepared = build()
    text = alpha_of(prepared, given)
    base = step(prepared, text, laws, exact=False)
    widths = [text, *inside_widths(base, Fraction(text))]
    assert len(widths) >= 4, f"{name}: the interval left no width to compare"
    labels = []
    for width in (*widths, *widths):  # второй круг - те же ширины на тёплой памяти
        monkeypatch.delenv(parametric.MEMO_SWITCH, raising=False)
        remembered = step(prepared, width, laws, exact=False).result.parametric
        monkeypatch.setenv(parametric.MEMO_SWITCH, "0")
        computed = step(prepared, width, laws, exact=False).result.parametric
        assert computed.memo == parametric.MEMO_OFF
        assert certificate_bytes(remembered) == certificate_bytes(computed), (name, width, remembered.memo)
        labels.append(remembered.memo)
    assert set(labels) <= set(parametric.MEMO_LABELS), labels
    if base.result.parametric.vertex_count:
        assert parametric.MEMO_HIT in labels or parametric.MEMO_PARTIAL in labels, f"{name}: the memo never hit: {labels}"


def test_the_exact_mode_does_not_take_the_memo_and_says_so():
    name, build, laws, given, _clipped = CASES[3]
    prepared = build()
    found = step(prepared, alpha_of(prepared, given), laws, exact=True).result.parametric
    assert found.memo == parametric.MEMO_BYPASS_EXACT and found.exact is not None


def test_the_materialize_memo_switch_also_turns_the_slope_memo_off():
    name, build, laws, given, _clipped = CASES[3]
    prepared = build()
    from cftuv_envelope.materialize import memo

    request = parametric.ParametricRequestV1({}, False)
    assert parametric._slope_source(prepared, request).memo is not None
    # Без памяти подготовки шаг ширины не заказывает сертификат вовсе (полный путь без шаблонов), поэтому выключатель проверяется на источнике наклонов.
    with memo.memo_disabled():
        source = parametric._slope_source(prepared, request)
    assert source.memo is None and source.final_label() == parametric.MEMO_OFF


def _attributes_read(function, names) -> dict:
    tree = ast.parse(textwrap.dedent(inspect.getsource(function)))
    read: dict = {name: set() for name in names}
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id in read:
            read[node.value.id].add(node.attr)
    return read


def test_the_fact_key_names_every_field_the_fact_slopes_read():
    """`_fact_slopes` читает у грани кадра, пробега и прямой ровно `FACT_KEY_READS`, и `_fact_key` кладёт в ключ каждое из этих полей; аргументы обеих функций те же (кроме бюджета)."""

    read = _attributes_read(parametric._fact_slopes, parametric.FACT_KEY_READS)
    assert {name: tuple(sorted(found)) for name, found in read.items()} == {name: tuple(sorted(fields)) for name, fields in parametric.FACT_KEY_READS.items()}, read
    keyed = _attributes_read(parametric._fact_key, parametric.FACT_KEY_READS)
    for name, fields in parametric.FACT_KEY_READS.items():
        assert set(fields) <= keyed[name], (name, set(fields) - keyed[name])
    slopes = list(inspect.signature(parametric._fact_slopes).parameters)
    assert slopes[-1] == "budget" and list(inspect.signature(parametric._fact_key).parameters) == slopes[:-1]


def test_every_memo_label_is_listed_once():
    constants = [value for name, value in vars(parametric).items() if name.startswith("MEMO_") and name not in ("MEMO_LABELS", "MEMO_SWITCH") and isinstance(value, str)]
    assert sorted(constants) == sorted(parametric.MEMO_LABELS) and len(set(parametric.MEMO_LABELS)) == len(parametric.MEMO_LABELS)
