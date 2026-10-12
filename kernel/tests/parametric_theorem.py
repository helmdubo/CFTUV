"""Теорема аффинного сертификата перетаскивания, выраженная кодом: значение при alpha0 плюс наклон на сдвиг РАВНЫ точному прогону на сдвинутой ширине.

Один модуль для двух читателей: тесты ядра (`test_parametric_certificate.py`, фикстуры) и полевой зонд (`tools/...` и `E:\\cftuv_s6a\\field_theorem.py`, домены `building`,
`cover.008`, `rounded_wall_noise_top`). Равенство ТОЧНОЕ: `SqrtSumV1` каноничен, поэтому равные величины равны побитово; допусков нет.

Что сравнивается для домена, у которого нет названных причин (`markers` пусто; у развёртки допустима единственная `PARAM_NON_AFFINE_DEVELOPABLE_OFFSET`: позиции ДО смещения аффинны, а
смещение идёт мимо ядра):
* точка карты каждой вершины (единицы решётки): `point(alpha1) == point(alpha0) + наклон * (alpha1 - alpha0) * scale`;
* точный подъём точки (метры, без позиции хоста `src:` и переноса узлов: это константы): `position(alpha1) == position(alpha0) + наклон * (alpha1 - alpha0)`;
* факты `(s, r)` каждой петли UV (единицы решётки): `fact(alpha1) == fact(alpha0) + наклон * (alpha1 - alpha0) * scale`;
* наклоны не меняются от ширины к ширине (прямая, а не касательная).
Сравниваются две материализации ОДНОЙ структуры (подпись структуры и ключи вершин равны); другая структура - не опровержение, а «не судится» (решения тесселяции и силуэта в интервал не входят).
"""

from __future__ import annotations

from fractions import Fraction

DEVELOPABLE_MARKER = "PARAM_NON_AFFINE_DEVELOPABLE_OFFSET"

#: Исходы сравнения.
JUDGED = "JUDGED"
SKIPPED_MARKED = "SKIPPED_MARKED"
SKIPPED_STRUCTURE = "SKIPPED_STRUCTURE"
SKIPPED_KEYS = "SKIPPED_KEYS"


def decimal_text(alpha: Fraction, digits: int = 12) -> str:
    """Десятичная запись ширины (так ширину принимает ядро: `conveyor_coverage(prepared, "0.25")`)."""

    return f"{float(alpha):.{digits}f}"


def usable(certificate) -> bool:
    """Сертификат с точными числами: без причин либо только с развёрткой (позиции до смещения)."""

    return certificate is not None and certificate.exact is not None and set(certificate.markers) <= {DEVELOPABLE_MARKER}


def theorem_differences(base_step, other_step, alpha0: Fraction, alpha1: Fraction):
    """`(исход, список расхождений)` двух материализаций одного домена на ширинах `alpha0` и `alpha1` (метры, точные дроби); пусто - теорема выполнена."""

    base_certificate, other_certificate = base_step.result.parametric, other_step.result.parametric
    if not usable(base_certificate) or not usable(other_certificate):
        return SKIPPED_MARKED, []
    if base_step.result.structure.digest != other_step.result.structure.digest:
        return SKIPPED_STRUCTURE, []
    first, second = base_certificate.exact, other_certificate.exact
    if first.vertex_keys != second.vertex_keys or sorted(first.loop_ids) != sorted(second.loop_ids):
        return SKIPPED_KEYS, []
    delta = alpha1 - alpha0
    lattice = delta * first.scale
    found = []
    for index, key in enumerate(first.vertex_keys):
        for axis in range(2):
            if second.points0[index][axis] != first.points0[index][axis] + first.point_slopes[index][axis].scaled(lattice):
                found.append(("point", key, axis))
        if second.point_slopes[index] != first.point_slopes[index]:
            found.append(("point_slope", key))
        for axis in range(3):
            if second.positions0[index][axis] != first.positions0[index][axis] + first.position_slopes[index][axis].scaled(delta):
                found.append(("position", key, axis))
        if second.position_slopes[index] != first.position_slopes[index]:
            found.append(("position_slope", key))
    twin = {name: index for index, name in enumerate(second.loop_ids)}
    for index, name in enumerate(first.loop_ids):
        other = twin[name]
        for axis in range(2):
            if second.loops0[other][axis] != first.loops0[index][axis] + first.loop_slopes[index][axis].scaled(lattice):
                found.append(("loop", name, axis))
        if second.loop_slopes[other] != first.loop_slopes[index]:
            found.append(("loop_slope", name))
    return JUDGED, found


def float_errors(base_step, other_step, alpha0: Fraction, alpha1: Fraction):
    """`(наибольшая ошибка позиции батча, наибольшая ошибка u * alpha и v * alpha)` модели `значение + наклон * сдвиг` против настоящего батча на `alpha1` (float, метры и единицы UV).

    Это ОКРУГЛЕНИЕ: позиция батча - одно округление точного (плюс позиция хоста и перенос узлов, константы), наклон - float от точного. Вершины и петли в порядке вида домена.
    """

    base, other = base_step.result, other_step.result
    certificate = base.parametric
    if certificate is None or certificate.markers and set(certificate.markers) != {DEVELOPABLE_MARKER}:
        return None
    delta = float(alpha1 - alpha0)
    alpha_first, alpha_second = float(alpha0), float(alpha1)
    first = {item.vert_key.value: item.position for item in base.batch.vertices}
    second = {item.vert_key.value: item.position for item in other.batch.vertices}
    keys = sorted(first)
    if keys != sorted(second):
        return None
    worst_position = 0.0
    for index, key in enumerate(keys):
        for axis, name in enumerate(("x", "y", "z")):
            predicted = getattr(first[key], name) + certificate.position_slopes[index * 3 + axis] * delta
            worst_position = max(worst_position, abs(predicted - getattr(second[key], name)))
    ids_first, ids_second = _loops(base.batch), _loops(other.batch)
    if sorted(ids_first) != sorted(ids_second):
        return None
    worst_uv = 0.0
    for index, name in enumerate(_loop_names(base.batch)):
        pair_first, pair_second = ids_first[name], ids_second[name]
        for axis in range(2):
            predicted = (pair_first[axis] * alpha_first + certificate.loop_slopes[index * 2 + axis] * delta) / alpha_second
            worst_uv = max(worst_uv, abs(predicted - pair_second[axis]))
    return worst_position, worst_uv


def _loop_names(batch) -> list:
    """`(кольцо грани с наименьшего ключа, ключ вершины)` каждой петли в порядке граней батча (порядок сертификата)."""

    names = []
    for face in batch.faces:
        keys = [item.value for item in face.ordered_vert_keys]
        start = keys.index(min(keys))
        ring = tuple(keys[start:] + keys[:start])
        names.extend((ring, key) for key in keys)
    return names


def _loops(batch) -> dict:
    values = [(fact.uv.u, fact.uv.v) for face in batch.faces for fact in face.uv_facts]
    return dict(zip(_loop_names(batch), values))
