# L0 — Legacy Planarity and ChainUse Evidence

Статус: factual evidence report

Срез: `SESSION_L0_LEGACY_PLANARITY_CHAINUSE_EVIDENCE`

Роль: Legacy Evidence Curator

Ветка: `codex/l0-legacy-planarity-chainuse-evidence`

База V0: `e95d9039c35d420b68d42b61509fc51190b01b66`

## 1. Вывод

Legacy Decal Seams содержал два полезных слоя, которые нельзя смешивать:

1. analysis-факты — owner-side boundary cycle, упорядоченные chain vertices/edges,
   тип соседа, локальный owner face и его normal, а также точная мощность uses
   выбранного physical edge;
2. product/implementation policy — допуск positional planarity, порог `4°` для
   rail planar-region/fold routing, consumer-dependent promotion selected edge
   до whole chain, grouping по неупорядоченной edge signature и fallback на
   средний Patch normal.

В analysis contract можно поднять первый слой и fail-closed cardinality. Legacy
численные пороги, pairing через `candidates[0]`, fallback-нормали, Voronoi,
corner builders, mesh emission и legacy fallback поднимать нельзя.

На `building.002` все 13 Patches прошли legacy positional planarity. Полевая
матрица дала `SUCCESS` для 1, 3 и всех 36 seam chains. Сценарий 10 chains
стабильно компилировался, но завершался
`TERMINAL_ROUTE_RECOMPILE_REQUIRED: patch=0 edge=4` на evaluation. Это отдельный
regression fixture, а не отрицание найденных analysis-фактов.

## 2. Authority и границы

Отчёт не читает и не изменяет новый kernel, Envelope evaluator или host adapter.
Никакая legacy geometry не перенесена.

Использованы три раздельных authority:

| ID | Authority | Назначение |
|---|---|---|
| `SRC-ROUTE` | `c70a98bb4965d1113520843e54c6e7e759b92de5` | Последний production-routing state до коммита удаления legacy SEAMS path. |
| `SRC-ARCHIVE` | `2831a223224c2e84bc39c9c405c8aefe459fd7c1` | Архивный source/test authority для сохранившегося planarity и ChainUse поведения. |
| `RT-B002` | Blender 4.3.2, exact module SHA-256 в `manifest.json` | Полевой trace `building.002`. Установленному runtime намеренно не приписан Git commit. |

`SRC-ROUTE:cftuv/decal_voronoi.py:5733-6044` доказывает, что до удаления
legacy routing активный compiler собирал selected sites, вызывал
`_patch_is_planar`, отправлял planar Patch в planar surface compile, а
non-planar Patch — в intrinsic strip charts. Определения, которые существовали
в файле, но не вызывались этим path, не считаются runtime authority.

Воспроизведение source anchors:

```powershell
git show 2831a223224c2e84bc39c9c405c8aefe459fd7c1:cftuv/decal_voronoi.py
git show 2831a223224c2e84bc39c9c405c8aefe459fd7c1:cftuv/decals.py
git show c70a98bb4965d1113520843e54c6e7e759b92de5:cftuv/decal_voronoi.py
python tools/validate_envelope_legacy_evidence.py
```

## 3. L0.1 — карта legacy planarity

### 3.1 Где Patch признавался планарным

Активный predicate — `_patch_is_planar(node)`:
`SRC-ARCHIVE:cftuv/decal_voronoi.py:3839-3853`.

Алгоритм:

```text
if mesh_verts is empty or mesh_tris is empty: NON_PLANAR
n = normalize(node.normal)
o = node.centroid
(x, y) = projection of every node.mesh_vert onto node.basis_u / node.basis_v
extent = max(max(x)-min(x), max(y)-min(y), 1.0)
tolerance = max(DECAL_WELD_DISTANCE * 0.25, extent * 1e-5)
PLANAR iff abs(dot(vertex-o, n)) <= tolerance for every Patch vertex
```

Следовательно:

- проверялся весь сериализованный Patch, не selected footprint;
- normal — нормализованный `PatchNode.normal`;
- origin — `PatchNode.centroid`;
- basis — `PatchNode.basis_u/basis_v`, применённый только для масштаба
  допуска через projected extent;
- residual — максимальное абсолютное расстояние всех Patch vertices до
  плоскости `(centroid, normal)`;
- angle в этом predicate не вычислялся;
- chart не был входом predicate: chart выбирался после результата;
- отсутствие `mesh_verts` или `mesh_tris` классифицировалось как non-planar.

Источники входов:

- `SRC-ARCHIVE:cftuv/analysis_classification.py:82-121`:
  `_classify_patch` строит area-weighted average face normal;
- `SRC-ARCHIVE:cftuv/analysis_classification.py:134-156`:
  `_build_patch_basis` строит ортогональный локальный basis;
- `SRC-ARCHIVE:cftuv/analysis_topology.py:458-539`:
  centroid, unique `mesh_verts`, fan-triangulated `mesh_tris` и provenance;
- `SRC-ARCHIVE:cftuv/analysis_topology.py:359-394`:
  эти значения собираются в `PatchNode`.

Centroid в legacy assembly усреднял face vertices по обходу faces, то есть
вершина могла иметь вес больше единицы, если входила в несколько faces
(`analysis_topology.py:458-468`). Это часть старого вычислительного path, но не
предлагаемый контракт.

### 3.2 Какие случаи были curved/non-planar

В активном compiler Patch считался non-planar, если predicate выше возвращал
`False`; затем compiler пытался построить `INTRINSIC` surface
(`SRC-ROUTE:cftuv/decal_voronoi.py:5816-5963`). То есть curved/non-planar
означал positional residual всего Patch выше допуска либо отсутствие
сериализованной геометрии, а не превышение угла между faces.

Regression evidence:

- `SRC-ARCHIVE:tests/test_decal_voronoi.py:5046-5057` фиксирует non-planar
  topology path как `INTRINSIC_DEVELOPABLE`;
- `SRC-ARCHIVE:tests/test_decal_voronoi.py:5060-5111` фиксирует стабильность
  signatures при reversed normal и микрошуме координат;
- `SRC-ARCHIVE:tests/test_decal_rails.py:260-307` фиксирует permutation
  stability rail plan для перестановок triangles/selection и shallow
  non-planar ngon.

Это regression evidence для конкретных fixtures, не математическая гарантия
инвариантности для любой triangulation или rotation.

### 3.3 Отдельный `4°` порог

Field runtime экспортировал:

```text
DECAL_COPLANAR_DOT = 0.9975640502598242
acos(value) = 4.000000000000042 degrees
```

Этот порог использовался rail policy:

- `SRC-ARCHIVE:cftuv/decal_rails.py:719-740`: двухсторонний edge — fold при
  `dot <= DECAL_COPLANAR_DOT`;
- `SRC-ARCHIVE:cftuv/decal_rails.py:842-853`:
  `_faces_are_one_planar_region` требует `dot > DECAL_COPLANAR_DOT` для всех
  faces.

Ровно `4°` поэтому попадает в non-coplanar/fold сторону. Это product routing
policy и не имеет authority над whole-Patch positional planarity. Порог нельзя
объявлять новым Envelope каноном.

### 3.4 Зависимости и инвариантность

| Изменение | Доказанное legacy поведение | Классификация |
|---|---|---|
| Object rotation/location | Predicate работал на local BMesh coordinates; object matrix в `_patch_is_planar` не участвовала. | Analysis implementation fact. |
| Применённая rigid rotation | Идеальная ортонормальная ротация сохраняет расстояния, но basis/normal пересчитываются и floating-point boundary не специфицирован. | Не считать строгой гарантией. |
| Object scale | Unapplied object scale не входит в local predicate. `SRC-ARCHIVE:tests/test_decals.py:1837-1874` фиксирует preview-обработку `NON_UNIFORM_SOURCE_SCALE`, когда transform layer уже выдал эту ошибку; сам detector этот test не доказывает. | Product/transform policy, частично regression-only. |
| Applied uniform scale | Не строго invariant: есть абсолютный floor `DECAL_WELD_DISTANCE * 0.25`, `extent >= 1.0` и относительный член `extent * 1e-5`. | Threshold implementation. |
| Triangulation | Residual проходит по `mesh_verts`; `mesh_tris` лишь должны быть non-empty. Но изменение face partition может изменить area-weighted normal и centroid. | Нельзя обещать invariance. |
| Face order | Явной сортировки в normal/centroid accumulation нет; возможен floating-point drift и изменение IDs. | Regression-only, не контракт. |
| Selected footprint | Не влияет на planarity scope: проверяется весь touched Patch. | Analysis fact. |

### 3.5 Мёртвые/неавторитетные ветви

`_canonical_plane_key` и `_provenance_owner_surfaces`
(`SRC-ARCHIVE:cftuv/decal_voronoi.py:3856-4050`) присутствовали в source, но не
были вызваны активным compile path `SRC-ROUTE`. Их per-source-face grouping и
residual нельзя выдавать за фактический Patch-planarity contract. Они
классифицируются как implementation remnants и отвергаются как authority.

## 4. L0.2 — PhysicalChain / ChainUse

### 4.1 Что реально хранил legacy analysis

Legacy не имел единственного first-class `PhysicalChain`. Analysis хранил
owner-side `BoundaryChain`:
`SRC-ARCHIVE:cftuv/model.py:142-193`.

В нём были:

- ordered `vertex_indices` и `edge_indices`;
- `side_face_indices` и `side_face_normals`;
- `is_closed`;
- `neighbor_patch_id` и derived `neighbor_kind`;
- start/end vertex и corner/junction endpoint facts.

Boundary side определялся owner face-loop topology:
`SRC-ARCHIVE:cftuv/analysis_boundary.py:49-153`. Поэтому orientation — порядок
обхода owner-side face cycle, а не сортировка vertex IDs. Два uses одного
PATCH edge на `building.002` имеют противоположные oriented vertex pairs;
это напрямую записано в `building_002_cases.json`.

### 4.2 Neighbor kinds

`_neighbor_for_side`
(`SRC-ARCHIVE:cftuv/analysis_boundary_loops.py:79-96`) давал:

- `PATCH`: другой Patch с противоположной стороны;
- `SEAM_SELF`: seam edge, обе стороны которого принадлежат одному Patch;
- `MESH_BORDER`: только один linked face либо отсутствует другой Patch.

В `BoundaryChain.neighbor_kind` legacy sentinels отображались как
`-2 -> SEAM_SELF`, `-1 -> MESH_BORDER`, иначе `PATCH`
(`SRC-ARCHIVE:cftuv/model.py:180-193`).

### 4.3 Splitting, merging, closed chains и внутренние изгибы

Первичный split происходил при смене neighbor во время циклического boundary
обхода:
`SRC-ARCHIVE:cftuv/analysis_boundary_loops.py:99-170`.

- если весь loop имел одного neighbor, получался closed chain;
- `MESH_BORDER` chains дополнительно атомизировались до single-edge
  (`analysis_boundary_loops.py:279-287`);
- после role classification соседние совместимые `MESH_BORDER` chains могли
  снова сливаться (`analysis_boundary_loops.py:432-490`);
- финальные endpoints/corners/junction facts вычислялись после chain refinement
  (`analysis_boundary_loops.py:493-600,860-880,1104-1131`).

Для `PATCH` и `SEAM_SELF` внутренний геометрический изгиб сам по себе не был
общим правилом split: chain мог содержать несколько направлений, пока neighbor
не менялся. Значит internal bend — допустимый analysis fact, а не доказательство
новой physical-chain границы.

### 4.4 Selected edge → chain

Legacy имел две разные selection semantics:

1. `chain_refs_for_edge_indices`
   (`SRC-ARCHIVE:cftuv/decals.py:707-728`) возвращал полный owner-side ChainRef,
   если selected set пересекал хотя бы один edge chain. На shared seam он
   возвращал оба patch-side refs.
2. `_collect_manual_edge_decals`
   (`SRC-ARCHIVE:cftuv/decals.py:1149-1249`) работал атомарно по каждому
   selected physical edge и собирал uses `(patch, loop, chain, segment)`.

Regression tests:

- whole-chain promotion:
  `SRC-ARCHIVE:tests/test_decals.py:2345-2373`;
- обе стороны shared edge:
  `SRC-ARCHIVE:tests/test_decals.py:2522-2546`;
- exact selected edges переживают asymmetric chain splits:
  `SRC-ARCHIVE:tests/test_decals.py:2581-2629`;
- selected `SEAM_SELF` даёт два use одного edge:
  `SRC-ARCHIVE:tests/test_decals.py:2631-2654`;
- ноль uses rejected:
  `SRC-ARCHIVE:tests/test_decals.py:2656-2665`;
- три uses rejected как non-manifold вместо выбора первых двух:
  `SRC-ARCHIVE:tests/test_decals.py:2667-2692`.

Consumer-dependent различие — случайность/долг старой реализации. Новый
contract должен явно выбрать scope; нельзя ссылаться просто на «legacy
selection semantics».

### 4.5 Physical identity и pairing

Legacy consumers реконструировали physical identity двумя способами:

- chain-level key: sorted edge IDs
  (`SRC-ARCHIVE:cftuv/decals.py:969-970`);
- edge-level key: один source edge ID с точными segment uses
  (`decals.py:1149-1249`,
  `decal_voronoi.py:4075-4157`).

`_collect_paired_chain_items`
(`SRC-ARCHIVE:cftuv/decals.py:983-1067`) находил PATCH counterpart по neighbor
Patch и той же sorted signature. Для `SEAM_SELF` требовалось ровно два
same-Patch candidate. Для PATCH path использовался `candidates[0]` без
обязательной проверки точной cardinality. Последнее — unsafe implementation
accident и должно быть отвергнуто.

Edge-level path был строже:

- 2 uses → paired PATCH/SEAM_SELF;
- 1 use → допустимый MESH_BORDER wing;
- 0 uses → `NO_BOUNDARY_CHAIN_USE`;
- более 2 → `NON_MANIFOLD_EDGE_USE`.

`_single_use_internal_seam_failures`
(`SRC-ARCHIVE:cftuv/decal_voronoi.py:4160-4195`) дополнительно fail-closed
отклонял один use, если neighbor не `MESH_BORDER`.

### 4.6 Owner side

Owner normal брался с соответствующего segment из
`BoundaryChain.side_face_normals`. При отсутствии/degenerate normal
`decals.py:690-704` мог fallback на средний Patch normal. Сам локальный
owner-side normal — analysis fact; fallback — старый geometry convenience и
не является достаточным support certificate.

`decal_rails.py:431-486` строил source chain records и проверял соответствие
ordered segment canonical endpoints physical edge. Ambiguous orientation
возвращала отсутствие результата, а не новый topology fact.

### 4.7 Четыре класса legacy поведения

| Класс | Доказанное поведение | Evidence |
|---|---|---|
| Analysis fact | Owner-side cycle, ordered vertices/edges, local owner face/normal, neighbor kind, closed flag, endpoint topology, exact use count. | `analysis_boundary.py:49-153`, `analysis_boundary_loops.py:79-170,493-600`, `model.py:142-193`, `decals.py:1149-1249`. |
| Product policy | Whole-Patch positional planarity, adaptive tolerance, `4°` rail routing, выбор whole-chain или exact-edge scope конкретным consumer. | `decal_voronoi.py:3839-3853`, `decal_rails.py:719-740,842-853`, `decals.py:707-728,1149-1249`. |
| Performance/implementation optimization | Однократный `uses_by_owner_and_edges` index для pairing и per-Patch `seen_edges`/`uses_by_edge` index для selected sites. | `decals.py:986-999`, `decal_voronoi.py:4078-4157`. |
| Случайность/долг | PATCH `candidates[0]`, sorted edge-set identity, fallback на Patch normal, consumer-dependent selected scope и неиспользуемые provenance-plane helpers. | `decals.py:969-970,1028-1055,690-704`, dead helpers `decal_voronoi.py:3856-4050`. |

Indexes и deduplication можно повторить как оптимизацию только после определения
новой canonical identity и bit-identical/full-scan gate. Их конкретные legacy
keys не являются semantic contract.

## 5. L0.3 — сравнение с V0

Колонка V0 отражает authority, заданный карточкой L0, и разрешённую analysis
границу `surface_ir.py`; kernel/evaluator/host adapter в этом срезе не
инспектировались.

| Факт | Legacy authority | Новый V0 authority | Расхождение |
|---|---|---|---|
| Patch planarity | Whole-Patch vertex residual относительно average `(normal, centroid)`, adaptive tolerance. | Exact orthonormal frame. | Legacy predicate tolerance-based, local-coordinate и использует weighted centroid; его threshold не канон V0. |
| Chain identity | Owner-side `BoundaryChain`; pairing key — sorted edge set, edge path — source edge ID. | Canonical edge sequence. | Sorted legacy signature теряет порядок/orientation и не является canonical physical identity. |
| PATCH pair | Chain path: neighbor Patch + same signature, затем `candidates[0]`; edge path: ровно 2 uses. | Exact two-use pair. | Edge-level legacy cardinality совместима; chain-level first-candidate policy должна быть отвергнута. |
| SEAM_SELF | Neighbor sentinel + ровно 2 same-Patch uses/candidates. | Exact two uses. | Семантика cardinality совпадает, но V0 не должен наследовать legacy sentinels или pairing key. |
| MESH_BORDER | Один owner-side use; border chains могли atomize/merge по role. | Один certified owner use. | Legacy merge — presentation/runtime grouping, не physical identity. |
| Selected scope | Whole chain в ChainRef consumers; exact edge в manual edge consumer. | Whole PhysicalChain. | Legacy неоднозначен по consumer; whole-chain V0 scope должен быть явным контрактом. |
| Owner side | Face-loop cycle, segment owner face/normal; Patch-normal fallback. | Face cycle/support certificate. | Cycle/owner face можно поднять; fallback normal не заменяет certificate. |
| Orientation | Порядок owner-side face-loop cycle; opposite uses обычно reversed. | Canonical physical sequence + use-local orientation. | Legacy sorted signature orientation не сохраняет. |
| Closed chain | Один-neighbor closed boundary loop, `is_closed=true`. | Explicit closed canonical sequence. | Legacy endpoint semantics для closed chain зависят от выбранного start; нужен canonical start. |
| Internal bend | Допустим внутри PATCH/SEAM_SELF chain до смены neighbor. | Явное правило canonical splitting. | Нельзя вывести physical split только из legacy local role/angle. |

`V0-BASE:cftuv/surface_ir.py:71-116` уже отделяет physical
`SourceEdge.vertex_ids` от owner-side
`SourceFace.vertex_cycle/edge_cycle` и triangle-to-physical-edge provenance.
Это правильная analysis граница; geometry authority из legacy для этого не
нужен.

## 6. L0.4 — field evidence corpus `building.002`

### 6.1 Метод

Исходное состояние:

- Blender `4.3.2`, `E:\testscene.blend`, object `building.002`;
- Edit Mode, 24 vertices, 38 edges, 15 faces;
- исходная selection `[2, 3, 7]`;
- scene уже имела `is_dirty=true` до L0.

Каждый run использовал свежий
`bmesh.from_edit_mesh(object.data).copy()`. GeometryBatch оставался in-memory:
mesh materialization, смена selection и scene save не выполнялись. После
сбора состояние осталось тем же: Edit Mode, `[2,3,7]`, `24/38/15`,
`is_dirty=true`. Authority: `RT-B002`, pre/post records в `manifest.json`.

Поздняя контрольная проверка в `23:17:30 +04:00` увидела selection `[12]` при
тех же `24/38/15`: между проверками в общей Blender session шла параллельная
активность. L0 после своего post-record использовал только BMesh copies, не
восстанавливал и не менял новое состояние. Эта поздняя selection исключена из
четырёх fixtures и сохранена только как concurrent provenance observation.

Объект дал 13 Patches, 60 patch-side chain uses и 36 seam-marked single-edge
physical chains. Edges `1..12` и `25..36` имеют по два `PATCH` use; edges
`13..24` — по одному `MESH_BORDER` use. `SEAM_SELF` на этом ассете отсутствует
и покрывается архивными regression tests, а не выдуманным field example.

Все 13 Patches прошли legacy predicate. Наибольший residual:
`9.5367431640625e-07`; во всех Patches фактический tolerance был `0.00025`
из-за absolute weld term. Полные Patch IDs, residuals, uses, owner faces и
oriented vertex pairs находятся в `building_002_cases.json`. Owner-side normal
каждого use однозначно получается через его `owner_face_id` и
`source.owner_face_normal_catalog`.

### 6.2 Результаты

Медианы пяти корректных runs:

| Case | Selected edges | Patches | Kinds | Source faces/edges | Analysis, ms | Compile, ms | Evaluation wall, ms | Result |
|---|---:|---:|---|---:|---:|---:|---:|---|
| `one_chain` | `[2]` | `[0,10]` | `PATCH×1` | `4 / 17` | `5.9981` | `7.8075` | `3.1649` | `SUCCESS`, 3 faces |
| `three_chains` | `[2,3,7]` | `[0,7,10,12]` | `PATCH×3` | `6 / 21` | `5.9574` | `16.5460` | `13.5487` | `SUCCESS`, 12 faces |
| `ten_chains` | `[1..10]` | 11 Patches | `PATCH×10` | `13 / 35` | `6.4891` | `85.4092` | `0.2275` до failure | `TERMINAL_ROUTE_RECOMPILE_REQUIRED`, patch 0 edge 4 |
| `all_seam_chains` | `[1..36]` | `[0..12]` | `PATCH×24`, `MESH_BORDER×12` | `15 / 38` | `7.4152` | `135.2895` | `89.3741` | `SUCCESS`, 68 faces |

`ten_chains` compile был успешен во всех пяти runs; короткое evaluation time
показывает ранний structured runtime failure, а не быстрый успешный result.
Сырые samples и representative log lines находятся в `timings.json`.

Analysis time на этом малом объекте почти не зависит от selection, потому что
PatchGraph строился для объекта целиком. Compile time заметно рос с selected
scope, но эти четыре точки не доказывают complexity class. L0 также не
утверждает численное превосходство над новым V0: прямой V0 benchmark был бы
чтением/исполнением вне границ этого среза.

## 7. Классификация evidence

### 7.1 Можно повысить до analysis contract

- canonical physical edge/chain sequence отдельно от patch-side use;
- owner-side face cycle и use-local orientation;
- owner Patch ID, exact owner face ID и face normal;
- explicit neighbor kind `PATCH | SEAM_SELF | MESH_BORDER`;
- exact use cardinality:
  `PATCH=2`, `SEAM_SELF=2`, `MESH_BORDER=1`, иначе fail closed;
- whole-PhysicalChain selected scope как явно названный contract;
- explicit closed flag, canonical start для closed sequence;
- source face/edge provenance и junction endpoint IDs;
- whole-Patch versus footprint scope как явное поле analysis result;
- planarity result вместе с frame, residual и tolerance provenance, не только
  boolean.

Это повышение касается данных и проверяемых topology фактов, не legacy
геометрии.

### 7.2 Только regression fixtures

- все численные residual/tolerance значения `building.002`;
- четыре selection scenarios и их exact use maps;
- timings на Blender 4.3.2 и exact installed-runtime hashes;
- стабильный `ten_chains` failure;
- микрошум/reversed-normal/permutation tests;
- legacy `DECAL_WELD_DISTANCE` и `4°` rail threshold;
- текущие Patch IDs, face IDs и chain indices: они пригодны для source revision
  digest, но не являются cross-revision semantic identity.

### 7.3 Должно быть отвергнуто

- перенос legacy Voronoi, corner builders, mesh emission или fallback;
- объявление legacy planarity tolerance новым Envelope каноном;
- смешивание positional Patch planarity и `4°` rail coplanarity;
- pairing PATCH через `candidates[0]`;
- sorted edge set как единственная canonical chain identity;
- Patch-average normal fallback вместо owner support certificate;
- зависимость selected scope от конкретного consumer;
- использование dead `_canonical_plane_key` /
  `_provenance_owner_surfaces` как runtime authority;
- вывод о строгой rotation/scale/triangulation/face-order invariance без
  отдельного proof corpus;
- исправление `ten_chains` failure внутри L0.

## 8. Gate receipt

- Каждое нормативное утверждение выше связано с `SRC-ROUTE`,
  `SRC-ARCHIVE`, конкретным test path или `RT-B002`.
- Новый kernel, evaluator, host adapter, legacy geometry и Blender production
  operators не изменены.
- L0 не сохранял сцену и не менял geometry/selection; позднее параллельное
  изменение selection отделено в provenance.
- Report не предлагает копировать legacy geometry.
- Promotion, regression-only и reject buckets разделены явно.
- Corpus проверяется без чтения legacy-кода:
  `python tools/validate_envelope_legacy_evidence.py`.
