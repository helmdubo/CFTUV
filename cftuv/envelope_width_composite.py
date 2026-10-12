"""Составной образец инкрементального такта: обслуженные домены двигает модель (numpy), посчитанные заново ложатся поверх точными (WIDTH_REFRESH_INCREMENTAL_V1, этап B; чистая, без Blender).

ЗАЧЕМ. Инкрементальный такт этапа A считал заново только домены вне сертификата, но образец всё равно собирал по результатам ВСЕХ доменов: сдвигал вид каждого (`envelope_width_rebase`), сваривал
вершины всех (`build_mesh_arrays`), строил массивы образца заново. На `cover.008` это ~0.2 с родителя на тик, хотя изменились единицы доменов. Здесь образец тика складывается из образца на экране:
  * позиции и UV всех обслуженных доменов - кадр модели на ширине тика (`positions_and_uvs`: одна векторная операция над массивами, те же формулы, что при движении мыши);
  * домены, посчитанные заново, ложатся поверх точными: UV - из вида результата, позиции - из сварки ТОЛЬКО тех доменов, чьи вершины от них зависят (`weld_vertices` над группой), в ту же нумерацию вершин меша;
  * раскладка меша (грани, швы, домены, образ) - образца на экране, номера владельцев (`claim:N`) посчитанных доменов - их новые, остальных - прежние.

ЧТО ЗАВИСИТ ОТ ПОСЧИТАННОГО ДОМЕНА (группа). Позиция вершины после сварки зависит от доменов, делящих с ней ссылку `location:src:*` (митра их нормалей) и, для узлов стены, от соседей по стене (`WALL_MITER_OFFSET_V1`: нормаль соседа вдоль
его стены). Поэтому группа - это посчитанные домены `R`, домены, чьи узлы стены считаются от `R` (`rev`: те, у кого `R` в соседях), и всё, что нужно, чтобы их вершины посчитать верно: соседи по ссылке и по стене. Позиции принимаются только для
вершин `R` и `rev(R)`; у остальных доменов группы (их данные нужны, а своих соседей в группе нет) заменяются лишь вершины, общие с принимаемыми (один номер вершины меша): остальное остаётся кадром модели.

ЧТО НЕ ПЕРЕСЧИТЫВАЕТСЯ. Швы на складках между доменами (`cross_domain_seams`: решение по равенству UV на общем ребре) остаются швами образца на экране: сдвиг на ширину держит равенство UV соседей так же, как сертификат держит его у точного
прогона, а расхождение в последнем бите дало бы шов, которого точный прогон не ставит. Раскладка, швы и составы доменов - факты структуры, их сверяет полный такт (политика сверки).

ОТКАЗЫ ИМЕНОВАНЫ. Составить образец не вышло - возвращается ИМЯ причины, и тик собирается путём этапа A (результаты всех доменов, тот же ответ, дороже): домен образца отказан либо появился новый (`COMPOSITE_DOMAIN_SET_CHANGED`), у
посчитанного домена другая локальная структура (`COMPOSITE_LAYOUT_CHANGED`: токен, число вершин или петель), домен не двигается моделью, но не посчитан (`COMPOSITE_HELD_NOT_RECOMPUTED`), сварка группы раскладывает вершины иначе,
чем меш на экране, либо соседи по стене другие (`COMPOSITE_WELD_CHANGED`). Ничего не подменяется молча.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, replace

import numpy as np

from .envelope_production_result import PLACEMENT_REBASED
from .envelope_production_weld import WELD_LOCATION_PREFIX, weld_vertices
from .envelope_width_preview_model import PreviewSampleV1, domain_sample_of, positions_and_uvs
from .envelope_width_rebase import certificate_usable, rebased_view, view_of

COMPOSITE_DOMAIN_SET_CHANGED = "INCREMENTAL_COMPOSITE_DOMAIN_SET_CHANGED"
COMPOSITE_LAYOUT_CHANGED = "INCREMENTAL_COMPOSITE_LAYOUT_CHANGED"
COMPOSITE_HELD_NOT_RECOMPUTED = "INCREMENTAL_COMPOSITE_HELD_NOT_RECOMPUTED"
COMPOSITE_WELD_CHANGED = "INCREMENTAL_COMPOSITE_WELD_CHANGED"
COMPOSITE_NO_VIEW = "INCREMENTAL_COMPOSITE_NO_VIEW"


@dataclass(frozen=True, slots=True)
class CompositeV1:
    """Составной образец тика и что о нём знать: сколько доменов посчитано, сколько вошло в группу сварки и сколько вершин сварка заменила; секунды."""

    sample: PreviewSampleV1
    recomputed: int
    group: int
    adopted_vertices: int
    seconds: float


def structure_token_of(view) -> str:
    """Токен локальной структуры вида домена (тот же, что кладёт в образец `build_mesh_arrays`): грани, швы, ссылки вершин, число петель UV."""

    from .envelope_production_mesh import _structure_token

    return _structure_token(view)


def _layout_cache(displayed: PreviewSampleV1) -> dict:
    return displayed.cache


def _patch_index(displayed: PreviewSampleV1) -> dict:
    cache = _layout_cache(displayed)
    found = cache.get("patch_index")
    if found is None:
        found = cache["patch_index"] = {domain.patch_id: number for number, domain in enumerate(displayed.domains)}
    return found


def _reverse_partners(displayed: PreviewSampleV1) -> dict:
    """`{домен: (домены, у которых он в соседях по стене)}`: чьи узлы стены считаются от него (раскладка образца, считается один раз)."""

    cache = _layout_cache(displayed)
    found = cache.get("reverse_partners")
    if found is None:
        built: dict = {}
        for number, domain in enumerate(displayed.domains):
            for other in domain.partners:
                built.setdefault(int(other), []).append(number)
        found = cache["reverse_partners"] = {key: tuple(value) for key, value in built.items()}
    return found


def _reference_index(displayed: PreviewSampleV1, fresh: dict) -> dict:
    """`{ссылка location:src:*: (домены с этой вершиной)}` только для общих ссылок (раскладка образца, считается один раз): ссылки доменов берутся из их видов, у посчитанных - из нового вида."""

    cache = _layout_cache(displayed)
    found = cache.get("reference_index")
    if found is None:
        built: dict = {}
        for number, domain in enumerate(displayed.domains):
            view = fresh[number][1] if number in fresh else view_of(displayed.origins[domain.patch_id])
            for ref in view.vertices.refs:
                if ref is not None and ref.startswith(WELD_LOCATION_PREFIX):
                    built.setdefault(ref, []).append(number)
        found = cache["reference_index"] = {ref: tuple(value) for ref, value in built.items() if len(value) > 1}
    return found


def _same_partition(first: np.ndarray, second: np.ndarray) -> bool:
    """Два нумерования одних и тех же вершин задают одно и то же разбиение (взаимно однозначное соответствие)."""

    if first.shape != second.shape:
        return False
    if first.size == 0:
        return True
    width = int(second.max()) + 1
    pairs = np.unique(first.astype(np.int64) * width + second.astype(np.int64))
    return pairs.size == np.unique(first).size == np.unique(second).size


def _fresh_domains(displayed: PreviewSampleV1, results) -> dict | str:
    """`{номер домена образца: (результат, вид)}` посчитанных доменов либо имя причины: домен образца отказан, появился новый, у домена другая локальная структура."""

    index_of = _patch_index(displayed)
    fresh: dict = {}
    for result in results:
        if result.placement == PLACEMENT_REBASED:
            continue
        number = index_of.get(int(result.patch_id))
        if not result.is_materialized:
            if number is not None:
                return COMPOSITE_DOMAIN_SET_CHANGED
            continue
        if number is None:
            return COMPOSITE_DOMAIN_SET_CHANGED
        view = view_of(result)
        if view is None or view.failure is not None or view.vertices is None:
            return COMPOSITE_NO_VIEW
        domain = displayed.domains[number]
        if len(view.vertices.positions) != domain.vertices.shape[0] or len(view.uvs) != domain.loop_count or structure_token_of(view) != domain.token:
            return COMPOSITE_LAYOUT_CHANGED
        fresh[number] = (result, view)
    return fresh


def _refs_of(displayed: PreviewSampleV1, fresh: dict, number: int):
    """Ссылки вершин домена: из нового вида посчитанного, из вида прежнего результата обслуженного; `None` - результата для сдвига нет."""

    if number in fresh:
        return fresh[number][1].vertices.refs
    origin = displayed.origins.get(displayed.domains[number].patch_id)
    return None if origin is None else view_of(origin).vertices.refs


def _group_of(displayed: PreviewSampleV1, fresh: dict):
    """`(принимаемые домены, группа сварки по порядку образца)` либо имя причины. Принимаемые - посчитанные и те, чьи узлы стены считаются от них; группа - они, соседи по ссылке и по стене."""

    try:
        reference_index = _reference_index(displayed, fresh)
    except KeyError:
        return COMPOSITE_HELD_NOT_RECOMPUTED  # домен без результата для сдвига не двигается моделью и не посчитан: ссылки его вершин неизвестны
    reverse = _reverse_partners(displayed)
    taken = set(fresh)
    for number in fresh:
        taken.update(reverse.get(number, ()))
    group = set(taken)
    for number in taken:
        group.update(displayed.domains[number].partners)
        refs = _refs_of(displayed, fresh, number)
        if refs is None:
            return COMPOSITE_HELD_NOT_RECOMPUTED
        for ref in refs:
            group.update(reference_index.get(ref, ()) if ref is not None else ())
    return taken, sorted(group)


def _weld_into(displayed: PreviewSampleV1, fresh: dict, taken: set, ordered: list, positions: np.ndarray, *, alpha: float, offset: float) -> int | str:
    """Позиции вершин принимаемых доменов из сварки группы - в `positions` (нумерация меша на экране); число принятых вершин либо имя причины (сварка легла иначе, чем меш на экране)."""

    inputs = []
    for number in ordered:
        if number in fresh:
            inputs.append(fresh[number][1].vertices)
            continue
        origin = displayed.origins.get(displayed.domains[number].patch_id)
        if origin is None or not certificate_usable(origin):
            return COMPOSITE_HELD_NOT_RECOMPUTED  # домен группы не двигается моделью и не посчитан: сдвигать нечем
        inputs.append(rebased_view(view_of(origin), origin.parametric, alpha).vertices)
    weld = weld_vertices(inputs, offset)
    base_index = np.concatenate([displayed.domains[number].vertices for number in ordered]).astype(np.int64)
    weld_index = np.concatenate([np.asarray(item, dtype=np.int64) for item in weld.index])
    if not _same_partition(weld_index, base_index):
        return COMPOSITE_WELD_CHANGED
    place = {number: at for at, number in enumerate(ordered)}
    found = {(ordered[int(owner)], ordered[int(other)]) for owner, other in weld.partners}
    for number in taken:
        if {pair for pair in found if pair[0] == number} != {(number, int(other)) for other in displayed.domains[number].partners if int(other) in place}:
            return COMPOSITE_WELD_CHANGED
    weld_positions = np.asarray(weld.positions, dtype=np.float64).reshape(-1, 3)
    chosen = sorted(taken)
    mesh_index = np.concatenate([displayed.domains[number].vertices for number in chosen]).astype(np.int64)
    positions[mesh_index] = weld_positions[np.concatenate([np.asarray(weld.index[place[number]], dtype=np.int64) for number in chosen])]
    return int(mesh_index.size)


def _rows_after(displayed: PreviewSampleV1, fresh: dict, alpha_text: str):
    """`(строки доменов, результаты образца, образ меша)` после тика либо имя причины: строки посчитанных заново, их результаты вместо прежних, номера владельцев их граней."""

    domains = list(displayed.domains)
    origins = dict(displayed.origins)
    image = displayed.image
    owners = None
    for number, (result, view) in fresh.items():
        domain = displayed.domains[number]
        domains[number] = domain_sample_of(
            result, token=domain.token, vertices=domain.vertices, loop_start=domain.loop_start, loop_count=domain.loop_count, partners=domain.partners, alpha_text=alpha_text
        )
        if certificate_usable(result):
            origins[domain.patch_id] = result
        else:
            origins.pop(domain.patch_id, None)
        faces = np.flatnonzero(image.face_domain == domain.patch_id)
        fresh_owners = np.asarray(view.owners, dtype=np.int32)
        if faces.size != fresh_owners.size:
            return COMPOSITE_LAYOUT_CHANGED
        if not np.array_equal(image.face_owner[faces], fresh_owners):
            owners = image.face_owner.copy() if owners is None else owners
            owners[faces] = fresh_owners
    return domains, origins, image if owners is None else replace(image, face_owner=owners)


def compose_sample(displayed: PreviewSampleV1, model, results, *, alpha: float, offset: float) -> CompositeV1 | str:
    """Образец тика на ширине `alpha` либо ИМЯ причины, по которой составить его не вышло (тик собирается путём этапа A).

    `results` - результаты доменов, которые тик посчитал (lean-результаты подмножества); `model` - модель образца на экране; `offset` - смещение декали, с которым сварены позиции образца.
    """

    started = time.perf_counter()
    alpha = float(alpha)
    fresh = _fresh_domains(displayed, results)
    if isinstance(fresh, str):
        return fresh
    positions, uvs, live = positions_and_uvs(model, alpha)
    if any(int(number) not in fresh for number in np.flatnonzero(~live)):
        return COMPOSITE_HELD_NOT_RECOMPUTED
    if positions is displayed.positions:
        positions, uvs = positions.copy(), uvs.copy()
    for number, (_result, view) in fresh.items():
        domain = displayed.domains[number]
        uvs[domain.loop_start : domain.loop_start + domain.loop_count] = np.asarray(view.uvs, dtype=np.float64)
    adopted = group_size = 0
    if fresh:
        chosen = _group_of(displayed, fresh)
        if isinstance(chosen, str):
            return chosen
        taken, ordered = chosen
        adopted = _weld_into(displayed, fresh, taken, ordered, positions, alpha=alpha, offset=offset)
        if isinstance(adopted, str):
            return adopted
        group_size = len(ordered)
    rows = _rows_after(displayed, fresh, str(alpha))
    if isinstance(rows, str):
        return rows
    domains, origins, image = rows
    sample = replace(displayed, alpha_text=str(alpha), alpha=alpha, positions=positions, uvs=uvs, domains=tuple(domains), arrays_digest="", image=image, origins=origins)
    return CompositeV1(sample, len(fresh), group_size, adopted, time.perf_counter() - started)


__all__ = (
    "COMPOSITE_DOMAIN_SET_CHANGED",
    "COMPOSITE_HELD_NOT_RECOMPUTED",
    "COMPOSITE_LAYOUT_CHANGED",
    "COMPOSITE_NO_VIEW",
    "COMPOSITE_WELD_CHANGED",
    "CompositeV1",
    "compose_sample",
    "structure_token_of",
)
