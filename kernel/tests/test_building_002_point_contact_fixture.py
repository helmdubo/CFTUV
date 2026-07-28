from __future__ import annotations

from collections import defaultdict
import hashlib
import json
from pathlib import Path

import cftuv_envelope as kernel


ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "kernel" / "fixtures" / "building_002_point_contact_v1"
EVIDENCE = ROOT / "artifacts" / "envelope_c_r2c_fixture"
HISTORICAL_SHA = "df587ed166cfb0e0b615148f08c583b4477c5ac4"
SELECTED_SHA = "c2622d07020338e5231b81f41655fe6c74cdca72"


def _canonical_json(payload) -> bytes:
    return json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _load_contracts():
    snapshot_bytes = (FIXTURE / "analysis_snapshot.json").read_bytes()
    request_bytes = (FIXTURE / "decal_request.json").read_bytes()
    return (
        snapshot_bytes,
        request_bytes,
        kernel.AnalysisSnapshotCodecV1.loads(snapshot_bytes),
        kernel.DecalRequestCodecV1.loads(request_bytes),
    )


def _point_topology_from_historical(report: dict) -> list[dict]:
    result = []
    for vertex in report["evaluation"]["rejected_topology"]["rejected_vertices"]:
        chains = {
            chain_id
            for edge in vertex["outgoing_edges"]
            for chain_id in edge["provenance"]["coverage_contributors"][
                "chain_use_ids"
            ]
        }
        result.append(
            {
                "point": (
                    vertex["exact_point"]["x"]["expression"],
                    vertex["exact_point"]["y"]["expression"],
                ),
                "chain_use_ids": tuple(sorted(chains)),
            }
        )
    return sorted(result, key=lambda item: item["point"])


def _point_topology_from_selected(report: dict) -> list[dict]:
    raw = report["evaluation"]["raw_coverage"]
    occurrences_by_point = defaultdict(list)
    for occurrence in raw["shared_boundary_occurrences"]:
        occurrences_by_point[occurrence["arrangement_point_id"]].append(occurrence)
    result = []
    for contact in raw["point_contacts"]:
        chains = {
            chain_id
            for occurrence in occurrences_by_point[contact["arrangement_point_id"]]
            for chain_id in occurrence["provenance"]["coverage_contributors"][
                "chain_use_ids"
            ]
        }
        result.append(
            {
                "point": (
                    contact["exact_point"]["x"]["expression"],
                    contact["exact_point"]["y"]["expression"],
                ),
                "chain_use_ids": tuple(sorted(chains)),
            }
        )
    return sorted(result, key=lambda item: item["point"])


def test_fixture_contracts_hashes_and_ids_are_stable():
    manifest = json.loads((FIXTURE / "manifest.json").read_text(encoding="utf-8"))
    snapshot_bytes, request_bytes, snapshot, request = _load_contracts()
    host_mesh_bytes = (FIXTURE / "host_mesh.json").read_bytes()

    assert kernel.AnalysisSnapshotCodecV1.dumps(snapshot) == snapshot_bytes
    assert kernel.DecalRequestCodecV1.dumps(request) == request_bytes
    assert kernel.validate_analysis_snapshot(snapshot) == ()
    assert kernel.validate_decal_request(request) == ()
    assert kernel.validate_snapshot_request_references(snapshot, request) == ()

    actual_hashes = {
        "analysis_snapshot.json": _sha256(snapshot_bytes),
        "decal_request.json": _sha256(request_bytes),
        "host_mesh.json": _sha256(host_mesh_bytes),
    }
    assert actual_hashes == manifest["files"]
    assert manifest["fixture_hash"] == _sha256(
        _canonical_json(
            [[name, actual_hashes[name]] for name in sorted(actual_hashes)]
        )
    )
    assert manifest["source_mesh_fingerprint"] == (
        "3205834b7ead22cf1eb8985902144ca1c7295a88b8015556b364be17d8ac9021"
    )
    assert sorted(item.value for item in request.selected_chain_use_ids) == (
        manifest["selected_chain_use_ids"]
    )
    assert sorted(item.physical_chain_id.value for item in snapshot.physical_chains) == (
        manifest["physical_chain_ids"]
    )


def test_the_metric_descriptor_is_a_rebuild_of_the_fixtures_own_source():
    """Дескриптор метрики — не отдельные данные, а функция от вершин и граней.

    Проверяется побитово, и это же делает фикстуру перегенерируемой без
    Blender: когда контракт метрики меняется, дескриптор пересобирается ровно
    этим вызовом, а не правится руками.

    ЗАКОН РЕШЁТКИ БЕРЁТСЯ У САМОЙ ФИКСТУРЫ, а не вписан литералом, и это
    починка, а не удобство. Литерал стоял `UNSNAPPED_EXACT_V1`, и после
    пересборки фикстуры хостом он стал ложью: снапшот объявляет
    `SOURCE_ONLY_GRID_SNAP_V1`, дескриптор собран под ним, и тест падал на
    несовпадении, которое ничего про фикстуру не говорило — говорило про
    устаревшую константу в тесте. Закон и политика планарности — часть
    ОБЪЯВЛЕНИЯ дескриптора, поэтому читать их надо из него; тогда утверждение
    остаётся тем же побитовым и переживает следующее переключение политики хоста
    без правки руками.

    Что закон при этом действительно РАЗЛИЧАЕТ ответы (а чтение из дескриптора
    не выродилось в «подставить то, что подойдёт»), проверяется тут же: под
    двумя другими законами перестройка даёт другие байты.
    """

    _, _, snapshot, _ = _load_contracts()
    descriptor = next(iter(snapshot.surface_metric_descriptors))
    domain = next(
        item
        for item in snapshot.patch_domains
        if item.patch_domain_id == descriptor.patch_domain_id
    )

    def rebuild(grid_policy):
        return kernel.build_rational_affine_planar_metric(
            source_revision=snapshot.source_revision,
            patch_domain_id=descriptor.patch_domain_id,
            owner_patch_id=domain.owner_patch_id,
            source_vertices=snapshot.source_vertices,
            source_faces=snapshot.surface_ir.source_faces,
            planarity_policy=kernel.PlanarityAdmissionLawV1(
                descriptor.planarity_certificate.admission_law.value
            ),
            grid_policy=grid_policy,
            source_lineage=descriptor.source_lineage,
        )

    declared = descriptor.grid_certificate.snapping_law
    assert declared is kernel.GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1
    assert kernel.canonical_json_bytes(
        rebuild(declared)
    ) == kernel.canonical_json_bytes(descriptor)

    # Отрицательный контроль: закон решётки — не декорация, он двигает байты.
    for other in kernel.GridSnappingLawV1:
        if other is declared:
            continue
        assert kernel.canonical_json_bytes(
            rebuild(other)
        ) != kernel.canonical_json_bytes(descriptor), other


def test_selected_kernel_reaches_accepted_raw_coverage_v2():
    """Текущее ядро на ОТГРУЖЕННЫХ байтах, и почему оракулом тут не манифест.

    `expected_kernel_results` в манифесте ключуется РЕВИЗИЕЙ ЯДРА, то есть
    описывает пару (ядро, байты). Байты пересобраны, и пара распалась: ядро
    `c2622d0` эти байты не прочитало бы вовсе — в его контракте метрики нет поля
    `grid_certificate`, а отгруженный дескриптор его несёт. Значит блок под
    ключом `c2622d0` есть ИСТОРИЧЕСКАЯ запись о прежних байтах, а не ожидание
    для нынешних, и подставлять в него новые числа значило бы приписать ядру
    `c2622d0` ответ, которого оно дать не может.

    Поэтому числа отгруженных байтов заморожены ЗДЕСЬ, а расхождение с
    историческим блоком объявлено ИСПОЛНЯЕМО — иначе устаревание блока осталось
    бы молчанием ровно того рода, каким уехала транскрипция полевого контура.

    | величина | исторический блок (c2622d0) | отгруженные байты (HEAD) |
    |---|---|---|
    | `semantic_digest` | `622e1f6e...` | `60910ef0...` |
    | площадь | `122766786560/373821260323` | `64/1426085 * (1024*sqrt(2) + 6293)` |
    | вхождений границы | 12 | 14 |
    | точечных контактов | 2 | **0** |
    | петель / регионов | 3 / 3 | **1 / 1** |

    ТОЧЕЧНЫХ КОНТАКТОВ НОЛЬ, и это стоит назвать прямо: фикстура носит имя
    `point_contact`, а отгруженная больше не воспроизводит явление, ради которого
    заведена. Явление не потеряно — оно достижимо на тех же байтах снятием
    угловых отношений (`test_wavefront_differential.py::test_the_field_patch_pairwise_clipping_destroys_all_of_its_own_coverage`
    держит его вторым краем: 2 контакта, 3 региона, прежний отказ кроя). Веер
    вогнутого угла заполняет вырезы, три петли сливаются в одну, и контактам
    сходиться в точку становится нечему.

    Числа ниже проверены НЕ СОБОЙ: площадь эталона сверена с независимым путём
    очереди (`test_wavefront_conveyor.py`) — 0.34740839 против 0.34740932, то
    есть согласие двух реализаций до 2.7e-06 относительных, — а смена топологии
    объяснена контролем с той же решёткой и снятыми углами, который даёт РОВНО
    прежние (3, 3, 2).
    """

    manifest = json.loads((FIXTURE / "manifest.json").read_text(encoding="utf-8"))
    _, _, snapshot, request = _load_contracts()
    compiled = kernel.compile_reference_envelopes(snapshot, request)

    assert compiled.compilation is not None
    result = kernel.evaluate_reference_raw_coverage(
        compiled.compilation,
        request.requested_alpha,
    )
    raw = result.raw_coverage
    historical = manifest["expected_kernel_results"][SELECTED_SHA]

    # Структурная часть исторического блока ПЕРЕЖИЛА пересборку, и её оракулом
    # остаётся манифест: исход и версия схемы результата те же.
    assert result.outcome.value == historical["outcome"] == "EXACT"
    assert raw is not None
    assert raw.schema_version == historical["raw_schema_version"]

    # Числовая часть заморожена здесь, потому что она про НЫНЕШНИЕ байты.
    assert raw.semantic_digest == (
        "60910ef04eb82db25ee5c5049c3021f801e15ea81f887b5ea31666f67bd3f23f"
    )
    assert raw.exact_area_expression == (
        "Mul(Rational(64, 1426085), Add(Mul(Integer(1024), "
        "Pow(Integer(2), Rational(1, 2))), Integer(6293)))"
    )
    assert len(raw.boundary_vertex_occurrences) == 14
    assert len(raw.point_contacts) == 0
    assert len(raw.loops) == 1
    assert len(raw.regions) == 1

    # И само устаревание — исполняемым утверждением, а не примечанием. Когда
    # экспортёр перережет свои литералы `expected_kernel_results`, этот тест
    # покраснеет и потребует решить, чем блок стал.
    assert historical["semantic_digest"] != raw.semantic_digest
    assert historical["exact_area_expression"] != raw.exact_area_expression
    assert (
        historical["boundary_occurrence_count"],
        historical["point_contact_count"],
        historical["loop_count"],
        historical["region_count"],
    ) == (12, 2, 3, 3)


def test_historical_failure_and_selected_contacts_are_the_same_exact_topology():
    """Сверка ДВУХ ЗАПИСАННЫХ отчётов, и оба они про ПРЕЖНИЕ байты фикстуры.

    Тест не зовёт ядро вовсе — он читает `artifacts/envelope_c_r2c_fixture/`,
    снятые на байтах до пересборки. Поэтому он и остался зелёным, когда числа
    ядра сдвинулись, и поэтому же его зелёный цвет НЕ является утверждением об
    отгруженной фикстуре: отгруженная даёт 0 точечных контактов, а здесь их 2.
    Записано это явно, потому что «тест про point_contact проходит» иначе
    читалось бы как «точечный контакт на месте».

    Ценность сверки от этого не исчезает: она держит, что ОТКАЗ исторического
    ядра и КОНТАКТЫ выбранного — одна и та же точная топология, а это
    утверждение об алгоритмах, а не о том, какие байты сейчас в каталоге.
    Открытый счёт — перерезать артефакты вместе с литералами экспортёра.
    """

    historical = json.loads(
        (EVIDENCE / "historical_df587ed_result.json").read_text(encoding="utf-8")
    )
    selected = json.loads(
        (EVIDENCE / "selected_c262_result.json").read_text(encoding="utf-8")
    )
    manifest = json.loads((FIXTURE / "manifest.json").read_text(encoding="utf-8"))

    assert historical["kernel_revision"] == HISTORICAL_SHA
    assert historical["evaluation"]["stage"] == "RAW_REJECTED"
    assert historical["evaluation"]["outcome"] == manifest[
        "expected_kernel_results"
    ][HISTORICAL_SHA]["outcome"]
    assert historical["evaluation"]["diagnostics"][0]["message"] == (
        "exact union boundary is non-manifold at a construction vertex"
    )
    assert selected["kernel_revision"] == SELECTED_SHA
    assert selected["evaluation"]["stage"] == "RAW_READY"
    assert _point_topology_from_historical(historical) == (
        _point_topology_from_selected(selected)
    )
