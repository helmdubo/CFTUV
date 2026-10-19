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

ПЕРЕКЛАДКА (COMPOSITE_SPLICE_V1, только потоковый такт живого меша: `splice=True`). Две последние причины - не откат на массивы всего меша (на `cover.008` 0.35-0.45 с питона, держащего GIL): раскладка
перекладывается только в группе. Сварка группы (`weld_vertices` над видами группы на ширине такта - те же входы, что у пути этапа A) даёт классы её вершин; классы, задевающие принимаемые домены, - по ней
(вместе со всем, что с ними связано прежним классом), остальные - прежние; нумерация вершин меша - та же, что у `build_mesh_arrays` (класс получает номер по первой записи в порядке доменов и вершин);
грани и швы нетронутых доменов переходят срезами образа в новую нумерацию, перекладываемых - из их видов, швы складок на рёбрах перекладываемых вершин - заново (`cross_domain_seams` по их граням).
Не вышло - имя: класс уходит за группу (`COMPOSITE_SPLICE_CLASS_OUTSIDE`), группа не устоялась (`COMPOSITE_SPLICE_UNSETTLED`), раскладку образца не прочесть (`COMPOSITE_SPLICE_LAYOUT_UNREADABLE`).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, replace

import numpy as np

from .envelope_production_result import PLACEMENT_REBASED
from .envelope_production_weld import WELD_LOCATION_PREFIX, cross_domain_seams, weld_vertices
from .envelope_width_preview_model import MeshImageV1, PreviewSampleV1, domain_sample_of, layout_digest_of, positions_and_uvs
from .envelope_width_rebase import certificate_usable, rebased_view, view_of

COMPOSITE_DOMAIN_SET_CHANGED = "INCREMENTAL_COMPOSITE_DOMAIN_SET_CHANGED"
COMPOSITE_LAYOUT_CHANGED = "INCREMENTAL_COMPOSITE_LAYOUT_CHANGED"
COMPOSITE_HELD_NOT_RECOMPUTED = "INCREMENTAL_COMPOSITE_HELD_NOT_RECOMPUTED"
COMPOSITE_WELD_CHANGED = "INCREMENTAL_COMPOSITE_WELD_CHANGED"
COMPOSITE_NO_VIEW = "INCREMENTAL_COMPOSITE_NO_VIEW"
#: COMPOSITE_SPLICE_V1: группа не устоялась за `SPLICE_ROUNDS` расширений (сварка всё ещё находит домены вне принимаемых, чьи узлы стены считаются от них).
COMPOSITE_SPLICE_UNSETTLED = "INCREMENTAL_COMPOSITE_SPLICE_UNSETTLED"
#: COMPOSITE_SPLICE_V1: класс сваренных вершин, который перекладывается, выходит за группу сварки (его член вне группы): переложить его по группе нельзя.
COMPOSITE_SPLICE_CLASS_OUTSIDE = "INCREMENTAL_COMPOSITE_SPLICE_CLASS_OUTSIDE_GROUP"
#: COMPOSITE_SPLICE_V1: у образца на экране грани доменов не идут подряд в порядке доменов либо вершина неперекладываемой грани не нашла номера (раскладка не та, что строит `build_mesh_arrays`).
COMPOSITE_SPLICE_LAYOUT_UNREADABLE = "INCREMENTAL_COMPOSITE_SPLICE_LAYOUT_UNREADABLE"
SPLICE_ROUNDS = 3


@dataclass(frozen=True, slots=True)
class CompositeV1:
    """Составной образец тика и что о нём знать: сколько доменов посчитано, сколько вошло в группу сварки и сколько вершин сварка заменила; секунды."""

    sample: PreviewSampleV1
    recomputed: int
    group: int
    adopted_vertices: int
    seconds: float
    #: COMPOSITE_SPLICE_V1: имя причины, по которой раскладка группы переложена (`INCREMENTAL_COMPOSITE_LAYOUT_CHANGED` / `..._WELD_CHANGED`), пусто - раскладка образца на экране.
    spliced: str = ""


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


def _fresh_domains(displayed: PreviewSampleV1, results, changed: set | None = None) -> dict | str:
    """`{номер домена образца: (результат, вид)}` посчитанных доменов либо имя причины: домен образца отказан, появился новый, у домена другая локальная структура.

    `changed` (множество, COMPOSITE_SPLICE_V1) - домен с другой локальной структурой не отказ: его номер ложится в `changed`, раскладку перекладывает `_splice`.
    """

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
            if changed is None:
                return COMPOSITE_LAYOUT_CHANGED
            changed.add(number)
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


# --------------------------------------------------------------------------
# Перекладка раскладки группы (COMPOSITE_SPLICE_V1)
# --------------------------------------------------------------------------


def _layout_tables(displayed: PreviewSampleV1):
    """Раскладочные таблицы образца (считаются один раз на раскладку): начала доменов в плоском списке вершин доменов, сам список (номера вершин меша), домен каждой записи, кратность
    вершины меша, первая и последняя+1 грань каждого домена. `None` - грани доменов не идут подряд в порядке доменов (раскладка не та, что строит `build_mesh_arrays`)."""

    cache = _layout_cache(displayed)
    found = cache.get("splice_tables", False)
    if found is not False:
        return found
    domains = displayed.domains
    counts = np.fromiter((item.vertices.shape[0] for item in domains), dtype=np.int64, count=len(domains))
    offsets = np.zeros(len(domains) + 1, dtype=np.int64)
    np.cumsum(counts, out=offsets[1:])
    flat = np.concatenate([np.asarray(item.vertices, dtype=np.int64) for item in domains]) if domains else np.zeros(0, dtype=np.int64)
    owner = np.repeat(np.arange(len(domains), dtype=np.int64), counts)
    face_domain = np.asarray(displayed.image.face_domain, dtype=np.int64)
    patches = np.fromiter((item.patch_id for item in domains), dtype=np.int64, count=len(domains))
    found = None
    if bool(np.all(face_domain[1:] >= face_domain[:-1])) and bool(np.all(patches[1:] > patches[:-1])):
        first = np.searchsorted(face_domain, patches, side="left")
        last = np.searchsorted(face_domain, patches, side="right")
        found = (offsets, flat, owner, np.bincount(flat, minlength=displayed.vertex_count), first, last)
    cache["splice_tables"] = found
    return found


def _reference_all(displayed: PreviewSampleV1, fresh: dict, changed: set) -> dict:
    """`{ссылка location:src:*: [домены с этой вершиной]}` по ВСЕМ ссылкам раскладки образца (и без пары): к ней ищутся соседи новых ссылок домена, сменившего структуру.

    Считается один раз на раскладку; ссылки домена, сменившего структуру, берутся из вида его ПРЕЖНЕГО результата (таблица - про раскладку на экране), без результата - не берутся
    (его прежние соседи и так найдены по общим вершинам меша).
    """

    cache = _layout_cache(displayed)
    found = cache.get("reference_all")
    if found is None:
        found = {}
        for number, domain in enumerate(displayed.domains):
            if number in fresh and number not in changed:
                view = fresh[number][1]
            else:
                origin = displayed.origins.get(domain.patch_id)
                if origin is None:
                    if number in changed:
                        continue
                    raise KeyError(domain.patch_id)
                view = view_of(origin)
            for ref in view.vertices.refs:
                if ref is not None and ref.startswith(WELD_LOCATION_PREFIX):
                    found.setdefault(ref, []).append(number)
        cache["reference_all"] = found
    return found


def _splice_group(displayed: PreviewSampleV1, fresh: dict, changed: set, owner, flat, *, alpha: float, offset: float):
    """`(принимаемые домены, группа по порядку образца, виды группы на ширине такта, сварка группы)` либо имя причины.

    Принимаемые - посчитанные и те, чьи узлы стены считаются от них; группа - они, домены с общими вершинами меша (прежние классы сварки), соседи по стене и по ссылкам НОВЫХ видов.
    Сварка группы нашла домен вне принимаемых, чьи узлы стены теперь считаются от принимаемого, - он принимается, и группа собирается заново (не дольше `SPLICE_ROUNDS`).
    """

    try:
        refs_all = _reference_all(displayed, fresh, changed)
    except KeyError:
        return COMPOSITE_HELD_NOT_RECOMPUTED
    reverse = _reverse_partners(displayed)
    taken = set(fresh)
    for number in fresh:
        taken.update(reverse.get(number, ()))
    views: dict = {}
    for _round in range(SPLICE_ROUNDS):
        group = set(taken)
        mark = np.zeros(displayed.vertex_count, dtype=bool)
        mark[np.concatenate([np.asarray(displayed.domains[number].vertices, dtype=np.int64) for number in taken])] = True
        group.update(int(number) for number in np.unique(owner[mark[flat]]))
        for number in taken:
            group.update(int(other) for other in displayed.domains[number].partners)
            refs = _refs_of(displayed, fresh, number)
            if refs is None:
                return COMPOSITE_HELD_NOT_RECOMPUTED
            for ref in refs:
                if ref is not None and ref.startswith(WELD_LOCATION_PREFIX):
                    group.update(refs_all.get(ref, ()))
        ordered = sorted(group)
        for number in ordered:
            if number in views:
                continue
            if number in fresh:
                views[number] = fresh[number][1]
                continue
            origin = displayed.origins.get(displayed.domains[number].patch_id)
            if origin is None or not certificate_usable(origin):
                return COMPOSITE_HELD_NOT_RECOMPUTED  # домен группы не двигается моделью и не посчитан: сдвигать нечем
            views[number] = rebased_view(view_of(origin), origin.parametric, alpha)
        weld = weld_vertices([views[number].vertices for number in ordered], offset)
        grown = {ordered[int(own)] for own, other in weld.partners if ordered[int(other)] in taken and ordered[int(own)] not in taken}
        if not grown:
            return taken, ordered, views, weld
        taken |= grown
    return COMPOSITE_SPLICE_UNSETTLED


def _splice_labels(displayed: PreviewSampleV1, taken: set, ordered: list, weld, multiplicity):
    """`({домен группы: метки его вершин}, перекладываемые домены, перекладываемые вершины меша на экране)` либо имя причины.

    Метка записи вершины домена - номер вершины меша на экране (класс сварки не тронут) либо `N + номер вершины сварки группы` (класс перекладывается по сварке группы). Перекладываются
    все вершины принимаемых доменов и, до замыкания, всё, что связано с ними прежним классом либо новым классом сварки. Член перекладываемого прежнего класса вне группы - имя причины.
    """

    count = displayed.vertex_count
    place = {number: at for at, number in enumerate(ordered)}
    group_index = {number: np.asarray(weld.index[place[number]], dtype=np.int64) for number in ordered}
    moved_group = np.zeros(len(weld.positions), dtype=bool)
    moved_mesh = np.zeros(count, dtype=bool)
    taken_old = np.concatenate([np.asarray(displayed.domains[number].vertices, dtype=np.int64) for number in sorted(taken)])
    moved_mesh[taken_old] = True
    for number in taken:
        moved_group[group_index[number]] = True
    others = [number for number in ordered if number not in taken]
    empty = np.zeros(0, dtype=np.int64)
    mesh_side = np.concatenate([np.asarray(displayed.domains[number].vertices, dtype=np.int64) for number in others]) if others else empty
    group_side = np.concatenate([group_index[number] for number in others]) if others else empty
    while True:
        chosen = moved_group[group_side] | moved_mesh[mesh_side]
        size = (int(moved_group.sum()), int(moved_mesh.sum()))
        moved_group[group_side[chosen]] = True
        moved_mesh[mesh_side[chosen]] = True
        if (int(moved_group.sum()), int(moved_mesh.sum())) == size:
            break
    inside = np.bincount(mesh_side[chosen], minlength=count) + np.bincount(taken_old, minlength=count)
    if np.any(inside[moved_mesh] != multiplicity[moved_mesh]):
        return COMPOSITE_SPLICE_CLASS_OUTSIDE
    labels: dict = {}
    for number in ordered:
        shifted = group_index[number] + count
        if number in taken:
            labels[number] = shifted
            continue
        mesh = np.asarray(displayed.domains[number].vertices, dtype=np.int64)
        moved = moved_mesh[mesh]
        if moved.any():
            labels[number] = np.where(moved, shifted, mesh)
    return labels, set(labels), moved_mesh


def _runs(count: int, special: dict, base, offsets):
    """Плоский массив по доменам `0..count-1`: у доменов из `special` - их массив, у остальных - срез `base[offsets[n]:offsets[n+1]]` (подряд идущие - одним срезом)."""

    parts, cursor = [], 0
    for number in sorted(special):
        if number > cursor:
            parts.append(base[offsets[cursor] : offsets[number]])
        parts.append(special[number])
        cursor = number + 1
    if cursor < count:
        parts.append(base[offsets[cursor] : offsets[count]])
    return np.concatenate(parts) if parts else base[:0]


def _splice_numbering(labels, count: int, weld, positions):
    """Нумерация вершин меша по меткам записей (как у `weld_vertices`: класс получает номер по первой своей записи в порядке доменов и вершин), позиции, перекладываемые новые вершины
    и `{номер вершины меша на экране: новый номер}` для нетронутых классов (`-1` - вершины больше нет)."""

    size = labels.size
    first = np.full(count + len(weld.positions), size, dtype=np.int64)
    np.minimum.at(first, labels, np.arange(size, dtype=np.int64))
    pair_first = first[labels]
    head = np.zeros(size, dtype=bool)
    head[pair_first] = True
    new_of_pair = (np.cumsum(head) - 1)[pair_first]
    head_labels = labels[head]
    kept = head_labels < count
    out = np.empty((head_labels.size, 3), dtype=np.float64)
    out[kept] = positions[head_labels[kept]]
    out[~kept] = np.asarray(weld.positions, dtype=np.float64).reshape(-1, 3)[head_labels[~kept] - count]
    old_to_new = np.full(count, -1, dtype=np.int64)
    plain = labels < count
    old_to_new[labels[plain]] = new_of_pair[plain]
    return new_of_pair, out, ~kept, old_to_new


def _splice_faces(displayed: PreviewSampleV1, tables, rebuilt: set, views: dict, fresh: dict, vertices_of: dict, old_to_new):
    """`(длины граней, вершины граней, владельцы, домены граней)` меша после перекладки либо имя причины: нетронутые домены - срезами образа на экране в новой нумерации, перекладываемые - из видов."""

    image = displayed.image
    _offsets, _flat, _owner, _multiplicity, first, last = tables
    starts = np.asarray(image.face_starts, dtype=np.int64)
    ends = np.append(starts[1:], image.face_vertices.shape[0]).astype(np.int64)
    lengths, corners, owners, domains = [], [], [], []

    def plain(low: int, high: int) -> bool:
        if low >= high or int(first[low]) >= int(last[high - 1]):
            return True
        f0, f1 = int(first[low]), int(last[high - 1])
        mapped = old_to_new[np.asarray(image.face_vertices[int(starts[f0]) : int(ends[f1 - 1])], dtype=np.int64)]
        if mapped.size and int(mapped.min()) < 0:
            return False
        lengths.append(ends[f0:f1] - starts[f0:f1])
        corners.append(mapped)
        owners.append(np.asarray(image.face_owner[f0:f1], dtype=np.int32))
        domains.append(np.asarray(image.face_domain[f0:f1], dtype=np.int32))
        return True

    cursor = 0
    for number in sorted(rebuilt):
        if not plain(cursor, number):
            return COMPOSITE_SPLICE_LAYOUT_UNREADABLE
        faces = views[number].faces
        sizes = np.fromiter(map(len, faces), dtype=np.int64, count=len(faces))
        local = np.fromiter((item for face in faces for item in face), dtype=np.int64, count=int(sizes.sum()))
        if number in fresh:
            own = np.asarray(views[number].owners, dtype=np.int32)
        else:
            own = np.asarray(image.face_owner[int(first[number]) : int(last[number])], dtype=np.int32)
            if own.size != len(faces):
                return COMPOSITE_LAYOUT_CHANGED
        lengths.append(sizes)
        corners.append(vertices_of[number][local])
        owners.append(own)
        domains.append(np.full(len(faces), displayed.domains[number].patch_id, dtype=np.int32))
        cursor = number + 1
    if not plain(cursor, len(displayed.domains)):
        return COMPOSITE_SPLICE_LAYOUT_UNREADABLE
    return tuple(np.concatenate(parts) if parts else np.zeros(0, dtype=np.int64) for parts in (lengths, corners, owners, domains))


def _splice_seams(displayed: PreviewSampleV1, rebuilt: set, views: dict, vertices_of: dict, old_to_new, moved_mesh, moved_new):
    """Швы после перекладки (пары `(меньшая, большая)` по возрастанию) либо имя причины.

    Швы, не задевающие перекладываемых вершин, - швы образа на экране в новой нумерации (швы складок между нетронутыми доменами остаются, как у составного образца). Задевающие -
    заново: швы видов перекладываемых доменов и складки (`cross_domain_seams`) по их граням с UV их видов на ширине такта (как у пути этапа A); все грани такого ребра - у перекладываемых доменов.
    """

    old = np.asarray(displayed.image.seams, dtype=np.int64).reshape(-1, 2)
    keep = ~(moved_mesh[old[:, 0]] | moved_mesh[old[:, 1]])
    kept = old_to_new[old[keep]]
    if kept.size and int(kept.min()) < 0:
        return COMPOSITE_SPLICE_LAYOUT_UNREADABLE
    fresh_pairs = []
    faces, uvs, face_domain = [], [], []
    for number in sorted(rebuilt):
        view = views[number]
        index = vertices_of[number]
        if view.seams:
            fresh_pairs.append(index[np.asarray(view.seams, dtype=np.int64).reshape(-1, 2)])
        mapping = index.tolist()
        faces.extend(tuple(mapping[item] for item in face) for face in view.faces)
        uvs.extend(view.uvs)
        face_domain.extend([displayed.domains[number].patch_id] * len(view.faces))
    fresh_pairs.append(np.asarray(cross_domain_seams(faces, uvs, face_domain), dtype=np.int64).reshape(-1, 2))
    added = np.concatenate(fresh_pairs)
    added = added[moved_new[added[:, 0]] | moved_new[added[:, 1]]]
    pairs = np.concatenate([kept, added])
    low, high = np.minimum(pairs[:, 0], pairs[:, 1]), np.maximum(pairs[:, 0], pairs[:, 1])
    width = max(int(moved_new.size), 1)
    keys = np.unique(low * width + high)
    return np.stack([keys // width, keys % width], axis=1).astype(np.int32)


def _splice(displayed: PreviewSampleV1, fresh: dict, changed: set, positions, *, alpha: float, offset: float, started: float, reason: str, uvs) -> CompositeV1 | str:
    """COMPOSITE_SPLICE_V1: образец тика, когда раскладка группы сменилась (другая структура посчитанного домена либо сварка группы легла иначе), либо имя причины.

    Перекладывается только группа: сварка группы (`weld_vertices` над её видами на ширине такта, как в пути этапа A) даёт классы её вершин, нумерация меша - та же, что у `build_mesh_arrays`
    (класс - по первой записи в порядке доменов), нетронутые домены переходят срезами образа в новую нумерацию, их позиции и UV - кадр модели. Откат на весь меш не нужен.
    """

    tables = _layout_tables(displayed)
    if tables is None:
        return COMPOSITE_SPLICE_LAYOUT_UNREADABLE
    offsets, flat, owner, multiplicity, _first, _last = tables
    chosen = _splice_group(displayed, fresh, changed, owner, flat, alpha=alpha, offset=offset)
    if isinstance(chosen, str):
        return chosen
    taken, ordered, views, weld = chosen
    labelled = _splice_labels(displayed, taken, ordered, weld, multiplicity)
    if isinstance(labelled, str):
        return labelled
    labels, rebuilt, moved_mesh = labelled
    count = len(displayed.domains)
    sizes = np.diff(offsets)
    for number in changed:
        sizes[number] = len(fresh[number][1].vertices.positions)
    new_offsets = np.zeros(count + 1, dtype=np.int64)
    np.cumsum(sizes, out=new_offsets[1:])
    new_of_pair, positions_new, moved_new, old_to_new = _splice_numbering(_runs(count, labels, flat, offsets), displayed.vertex_count, weld, positions)
    vertices_of = {number: new_of_pair[new_offsets[number] : new_offsets[number + 1]] for number in rebuilt}
    faces = _splice_faces(displayed, tables, rebuilt, views, fresh, vertices_of, old_to_new)
    if isinstance(faces, str):
        return faces
    seams = _splice_seams(displayed, rebuilt, views, vertices_of, old_to_new, moved_mesh, moved_new)
    if isinstance(seams, str):
        return seams
    lengths, corners, owners, face_domain = faces
    face_starts = np.zeros(lengths.size, dtype=np.int32)
    if lengths.size > 1:
        np.cumsum(lengths[:-1], out=face_starts[1:])
    image = MeshImageV1(face_starts=face_starts, face_vertices=corners.astype(np.int32), face_domain=face_domain.astype(np.int32), face_owner=owners.astype(np.int32), seams=seams)
    loop_offsets = np.fromiter((item.loop_start for item in displayed.domains), dtype=np.int64, count=count)
    loop_offsets = np.append(loop_offsets, displayed.loop_count)
    uvs_new = _runs(count, {number: np.asarray(view.uvs, dtype=np.float64).reshape(-1, 2) for number, (_result, view) in fresh.items()}, uvs, loop_offsets)
    renumbered = _rows_to_renumber(displayed, tables, changed, new_of_pair, new_offsets)
    domains, origins = _splice_rows(displayed, fresh, changed, taken, ordered, weld, new_of_pair, new_offsets, str(alpha), renumbered)
    sample = replace(
        displayed, alpha_text=str(alpha), alpha=alpha, positions=positions_new, uvs=uvs_new, domains=domains, arrays_digest="", image=image, origins=origins,
        layout_digest=layout_digest_of(image, int(positions_new.shape[0]), int(uvs_new.shape[0])), cache={"patch_index": _patch_index(displayed)},
    )
    return CompositeV1(sample, len(fresh), len(ordered), int(moved_new.sum()), time.perf_counter() - started, spliced=reason)


def _rows_to_renumber(displayed: PreviewSampleV1, tables, changed: set, new_of_pair, new_offsets) -> np.ndarray:
    """Булев массив по доменам: номера вершин домена в меше сменились (у доменов той же формы - сравнение одной векторной операцией; сменившие форму - всегда)."""

    offsets, flat, owner = tables[0], tables[1], tables[2]
    count = len(displayed.domains)
    same_shape = np.ones(count, dtype=bool)
    same_shape[list(changed)] = False
    pairs = same_shape[owner]
    at = np.flatnonzero(pairs)
    moved = new_of_pair[at + (new_offsets[:-1] - offsets[:-1])[owner[at]]] != flat[at]
    renumbered = np.bincount(owner[at][moved], minlength=count) > 0
    renumbered[~same_shape] = True
    return renumbered


def _splice_rows(displayed: PreviewSampleV1, fresh: dict, changed: set, taken: set, ordered: list, weld, new_of_pair, new_offsets, alpha_text: str, renumbered):
    """Строки доменов после перекладки (номера вершин меша и начала петель - новые, соседи по стене принимаемых - из сварки группы) и результаты образца. Строка домена, у которого не
    сменилось ничего, переходит как есть."""

    partners: dict = {}
    for own, other in weld.partners:
        partners.setdefault(ordered[int(own)], []).append(ordered[int(other)])
    rows = []
    origins = dict(displayed.origins)
    numbers = new_of_pair.astype(np.int32)
    loop_start = 0
    for number, domain in enumerate(displayed.domains):
        if number not in fresh and number not in taken and not renumbered[number] and domain.loop_start == loop_start:
            rows.append(domain)
            loop_start += domain.loop_count
            continue
        vertices = numbers[new_offsets[number] : new_offsets[number + 1]]
        mine = tuple(partners.get(number, ())) if number in taken else domain.partners
        if number in fresh:
            result, view = fresh[number]
            token = structure_token_of(view) if number in changed else domain.token
            rows.append(domain_sample_of(result, token=token, vertices=vertices, loop_start=loop_start, loop_count=len(view.uvs), partners=mine, alpha_text=alpha_text))
            if certificate_usable(result):
                origins[domain.patch_id] = result
            else:
                origins.pop(domain.patch_id, None)
            loop_start += len(view.uvs)
            continue
        rows.append(replace(domain, vertices=vertices, loop_start=loop_start, partners=mine))
        loop_start += domain.loop_count
    return tuple(rows), origins


def compose_sample(displayed: PreviewSampleV1, model, results, *, alpha: float, offset: float, splice: bool = False) -> CompositeV1 | str:
    """Образец тика на ширине `alpha` либо ИМЯ причины, по которой составить его не вышло (тик собирается путём этапа A).

    `results` - результаты доменов, которые тик посчитал (lean-результаты подмножества); `model` - модель образца на экране; `offset` - смещение декали, с которым сварены позиции образца.
    `splice` (COMPOSITE_SPLICE_V1, потоковый такт живого меша): другая локальная структура посчитанного домена и разошедшаяся сварка группы - не откат, а перекладка раскладки группы (`_splice`).
    """

    started = time.perf_counter()
    alpha = float(alpha)
    changed: set | None = set() if splice else None
    fresh = _fresh_domains(displayed, results, changed)
    if isinstance(fresh, str):
        return fresh
    positions, uvs, live = positions_and_uvs(model, alpha)
    if any(int(number) not in fresh for number in np.flatnonzero(~live)):
        return COMPOSITE_HELD_NOT_RECOMPUTED
    if changed:
        return _splice(displayed, fresh, changed, positions, alpha=alpha, offset=offset, started=started, reason=COMPOSITE_LAYOUT_CHANGED, uvs=uvs)
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
            if splice and adopted == COMPOSITE_WELD_CHANGED:
                return _splice(displayed, fresh, set(), positions, alpha=alpha, offset=offset, started=started, reason=adopted, uvs=uvs)
            return adopted
        group_size = len(ordered)
    rows = _rows_after(displayed, fresh, str(alpha))
    if isinstance(rows, str):
        if splice and rows == COMPOSITE_LAYOUT_CHANGED:
            return _splice(displayed, fresh, set(), positions, alpha=alpha, offset=offset, started=started, reason=rows, uvs=uvs)
        return rows
    domains, origins, image = rows
    sample = replace(displayed, alpha_text=str(alpha), alpha=alpha, positions=positions, uvs=uvs, domains=tuple(domains), arrays_digest="", image=image, origins=origins)
    return CompositeV1(sample, len(fresh), group_size, adopted, time.perf_counter() - started)


__all__ = (
    "COMPOSITE_DOMAIN_SET_CHANGED",
    "COMPOSITE_HELD_NOT_RECOMPUTED",
    "COMPOSITE_LAYOUT_CHANGED",
    "COMPOSITE_NO_VIEW",
    "COMPOSITE_SPLICE_CLASS_OUTSIDE",
    "COMPOSITE_SPLICE_LAYOUT_UNREADABLE",
    "COMPOSITE_SPLICE_UNSETTLED",
    "COMPOSITE_WELD_CHANGED",
    "CompositeV1",
    "compose_sample",
    "structure_token_of",
)
