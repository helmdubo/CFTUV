"""Равенство ответа домена с точностью до меток, которые ядро выводит от ревизии.

Ядро нумерует грани, регионы, огибающие и УЗЛЫ (`node:N`) по ОТСОРТИРОВАННЫМ именам, а имена выводятся
хэшем от идентичностей хоста, то есть от ревизии источника: два холодных прогона ОДНОГО содержимого при двух
ревизиях различаются порядком граней, номерами `claim:N`/`region:N`/`node:N`, хэшами
`claim:envelope-instance:*` и числом операций точной арифметики (на `building` из 122 доменов так различаются
все, где есть больше одной огибающей; `test_two_cold_runs_of_one_content_agree_up_to_kernel_labels` держит
равенство проекций при разных дайджестах). Поэтому результат, перенесённый на новую
ревизию, нельзя сравнить с холодным прогоном побайтово, и сравнивается он ПРОЕКЦИЕЙ ответа, из которой метки
ядра убраны, а структура меток сохранена:

* узел `node:N` называется своей позицией (две вершины с одной позицией дают одинаковое имя: сравнение
  становится сравнением мультимножеств), вершина исходника `src:<идентичность хоста>` — собой;
* вершины целиком (имя, позиция, ссылка, происхождение без хэшей ядра);
* грани как множество (цикл имён вершин по минимальному вращению, UV по тому же вращению, материал);
* разбиение граней на регионы и на огибающие — множество множеств граней (сами номера не сравниваются);
* станции, цепи границы и интерфейса (как ломаные без направления) — по именам вершин; исход, детали,
  нормали, диагностики и счётчики ответа (счётчики ЦЕНЫ `EXACT_WORK_*` — нет: это работа, а не ответ);
* идентичности хоста (ревизия, домен, id запроса, рёбра, грани, цепи) — как есть: их перенос и проверяется.

Дайджесты (`content_digest`, `semantic_digest`) в проекцию не входят: они покрывают метки ядра.
"""

from __future__ import annotations

import re

#: Идентичности, которые ядро выводит хэшем от идентичностей хоста (`stable_id` ядра): метки ядра.
KERNEL_DERIVED_LABEL = re.compile(r"^claim:(envelope-instance|spec):")
PRICE_COUNTER = "EXACT_WORK_"


def _rotated(values):
    values = tuple(values)
    if not values:
        return values, 0
    start = min(range(len(values)), key=lambda index: values[index:] + values[:index])
    return values[start:] + values[:start], start


def _polyline(names):
    """Ломаная без направления; замкнутая — без начала."""

    names = tuple(names)
    if len(names) > 2 and names[0] == names[-1]:
        body = names[:-1]
        return min(_rotated(body)[0], _rotated(body[::-1])[0])
    return min(names, names[::-1])


def _namer(batch):
    where = {item.vert_key.value: repr(item.position) for item in batch.vertices}

    def name(key: str) -> str:
        return key if key.startswith("src:") else "@" + where[key]

    return name


def _provenance(item):
    return (
        tuple(sorted(value.value for value in item.source_face_ids)),
        tuple(sorted(value.value for value in item.physical_edge_ids)),
        tuple(sorted(value.value for value in item.chain_use_ids)),
        tuple(
            sorted(
                value.value
                for value in item.lineage_ids
                if not KERNEL_DERIVED_LABEL.match(value.value)
            )
        ),
    )


def batch_projection(batch) -> dict:
    name = _namer(batch)

    def signature(face):
        keys = tuple(name(key.value) for key in face.ordered_vert_keys)
        uv = tuple(repr(fact.uv) for fact in face.uv_facts)
        rotated, start = _rotated(keys)
        return (rotated, uv[start:] + uv[:start], face.material_id.value)

    signatures = {id(face): signature(face) for face in batch.faces}

    def grouped(identity):
        groups: dict[str, list] = {}
        for face in batch.faces:
            groups.setdefault(identity(face), []).append(signatures[id(face)])
        return tuple(sorted(tuple(sorted(group)) for group in groups.values()))

    return {
        "source_revision": batch.source_revision.value,
        "decal_request_id": batch.decal_request_id.value,
        "patch_domain_id": batch.patch_domain_id.value,
        "vertices": tuple(
            sorted(
                (
                    name(item.vert_key.value),
                    repr(item.position),
                    item.semantic_location_ref.value.replace(
                        item.vert_key.value, name(item.vert_key.value), 1
                    ),
                    _provenance(item.provenance),
                )
                for item in batch.vertices
            )
        ),
        "faces": tuple(sorted(signatures.values())),
        "face_provenance": tuple(
            sorted((signatures[id(face)], _provenance(face.provenance)) for face in batch.faces)
        ),
        "regions": grouped(lambda face: face.semantic_region_id.value),
        "claims": grouped(lambda face: face.ownership_claim_id.value),
        "stations": tuple(
            sorted(
                (
                    name(item.vert_key.value),
                    repr(item.source_s),
                    repr(item.source_r),
                    item.station_model_id.value,
                )
                for item in batch.station_facts
            )
        ),
        "boundary": tuple(
            sorted(
                (
                    item.semantic_boundary_id.value.split(":")[1],
                    _polyline(name(key.value) for key in item.ordered_vert_keys),
                )
                for item in batch.boundary_chains
            )
        ),
        "interface": tuple(
            sorted(
                _polyline(name(key.value) for key in item.ordered_vert_keys)
                for item in batch.interface_chains
            )
        ),
        "diagnostics": tuple(
            sorted(
                (item.diagnostic_id.value, item.severity.value, item.outcome.value)
                for item in batch.diagnostics
            )
        ),
        "contract_versions": tuple(sorted(item.value for item in batch.contract_versions)),
        "schema_version": batch.schema_version,
    }


def result_projection(result) -> dict:
    """Ответ одного домена без меток ядра, выводимых от ревизии (см. модуль)."""

    name = _namer(result.batch) if result.batch is not None else (lambda key: key)
    projection = {
        "patch_id": result.patch_id,
        "domain_id": result.domain_id,
        "outcome": result.outcome,
        "detail": result.detail,
        "counters": tuple(
            item for item in result.counters if not item[0].startswith(PRICE_COUNTER)
        ),
        "normal": result.normal,
        "source_normal": result.source_normal,
        "chart_orientation": result.chart_orientation,
        "vertex_normals": tuple(sorted((name(key), value) for key, value in result.vertex_normals)),
        "offset_normal_law": result.offset_normal_law,
        "decal_topology_law": result.decal_topology_law,
        "diagnostics": tuple(sorted(result.diagnostics)),
    }
    if result.batch is not None:
        projection["batch"] = batch_projection(result.batch)
    return projection


def mesh_projection(arrays) -> dict:
    """Меш писателя без порядка граней и номеров владельцев: что видит владелец, а не нумерация ядра."""

    faces = []
    offset = 0
    for index, loop in enumerate(arrays.faces):
        points = tuple(arrays.positions[item] for item in loop)
        start = min(range(len(points)), key=lambda at: points[at:] + points[:at])
        # `uvs` лежат подряд по кругу каждой грани: смещение — сумма длин предыдущих кругов.
        corner_uv = tuple(arrays.uvs[offset : offset + len(loop)])
        offset += len(loop)
        faces.append(
            (
                points[start:] + points[:start],
                corner_uv[start:] + corner_uv[:start],
                arrays.face_domain[index],
            )
        )
    owners: dict = {}
    for face, owner in zip(faces, arrays.face_owner):
        owners.setdefault((face[2], owner), []).append(face)
    seams = {
        tuple(sorted((arrays.positions[a], arrays.positions[b]))) for a, b in arrays.seam_edges
    }
    return {
        "faces": tuple(sorted(faces)),
        "owner_partition": tuple(sorted(tuple(sorted(group)) for group in owners.values())),
        "seams": tuple(sorted(seams)),
        "positions": tuple(sorted(arrays.positions)),
        "domains": tuple(arrays.domains),
        "skipped": tuple(arrays.skipped),
        "source_revision": arrays.source_revision,
    }
