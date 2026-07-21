# Envelope backend semantics — EC0 correction candidate

## Статус и область

Статус: `CORRECTION_CANDIDATE_READY_FOR_EXTERNAL_REVIEW`.

Это нормативная словесная спецификация EC0a/EC0b после внешнего review ветки
`codex/ec0-envelope-semantics` на commit `b16be81`. Corpus того commit отклонён
как канон из-за ошибок cardinality, дублирования графов и смешения analysis с
runtime. Он остаётся доступен в git history, но не является активной
SemanticAuthority. EC1 не начинается до явной пользовательской приёмки этого
исправления и зелёного валидатора.

Документ определяет поведение нового Blender-free envelope kernel. Он не
описывает реализацию, не переносит legacy-код и не разрешает kernel-implementer
читать `decal_voronoi.py` или legacy-части `decals.py`.

## Политика артефактов

EC0 выпускается как prose, canonical JSON, JSON Schema, metamorphic matrices и
validator/CI output. Презентационные SVG/PNG-листы, contact sheets, diagrams,
slides и interactive HTML запрещены. Они бесполезны для пользовательской
приёмки и хуже машинно-проверяемого текста для AI agents.

Разрешены диагностические скриншоты из Blender:

- viewport;
- UV Editor;
- debug overlays.

Такой скриншот фиксирует наблюдаемое runtime-поведение, но не получает
SemanticAuthority и не заменяет JSON/prose assertion.

Активный corpus: `artifacts/envelope_ec0/corpus/`.
Схема: `artifacts/envelope_ec0/corpus/schema/case.schema.json`.
Валидатор: `tools/validate_envelope_ec0.py`.

## SemanticAuthority

Каждое переносимое правило обязано иметь один тег:

- `USER_REQUIRED` — явно выбранное или подтверждённое пользователем поведение;
- `FIELD_PROVEN` — устойчиво подтверждённый производственный результат;
- `MATHEMATICALLY_REQUIRED` — необходимо для непротиворечивой topology,
  provenance или determinism;
- `LEGACY_COMPATIBILITY` — совместимость со старым поведением, ещё не принятая
  как новый канон;
- `IMPLEMENTATION_ACCIDENT` — следствие старого механизма, не семантика;
- `OPEN_RESEARCH` — гипотеза, требующая исследования.

В активную семантику входят первые три категории. `LEGACY_COMPATIBILITY`,
`IMPLEMENTATION_ACCIDENT` и `OPEN_RESEARCH` всегда находятся в явном списке
«на пересмотр» и не получают нормативной силы автоматически.

## Три раздельных контракта

### AnalysisSnapshotV1

`AnalysisSnapshotV1` содержит только host-observed facts и relations:

- PatchDescriptor и PatchDomain;
- PhysicalChain и directed ChainUse;
- BoundaryLoop, Hole, Barrier, SourceVertex;
- analysis-proven PatchSector;
- CornerRelation и JunctionRelation;
- boundary и physical lineage.

В snapshot запрещены kernel-created seeds, FrontComponents, active intervals,
runtime events, requested/effective alpha, capacity, contributions, envelopes,
ownership и GeometryBatch. `[MATHEMATICALLY_REQUIRED]`

### DecalRequestV1

`DecalRequestV1` содержит:

- `decal_request_id`;
- выбранные ChainUses;
- requested width/alpha;
- join, cap, boundary, interaction и ownership policies.

Разные requests остаются разными decal operations даже на одном Patch.
`[USER_REQUIRED]`

### CompiledPatchEvaluationPlan

План ключуется парой `(DecalRequestId, PatchDomainId)` и содержит все активные
sources этого request в этом Patch:

- FrontSeed, CornerSeed, JunctionSeed, CapSeed, EndpointClaimSeed;
- FrontComponents и active intervals;
- Strip/Corner/Junction/Cap contributions;
- boundary и interaction event ledger;
- requested/effective alpha и capacity;
- PatchCoverage, ownership, station/UV claims;
- provenance будущего GeometryBatch.

Каждый plan record сохраняет оба идентификатора. Per-pChain private plan/domain
и post-hoc material union запрещены. `[USER_REQUIRED]`

## Единственный authoritative graph

Каждый JSON case содержит ровно один ID graph:

1. `analysis_snapshot`;
2. `decal_request`;
3. `expected_compiled_plan`;
4. `acceptance` assertions.

Параллельные вручную поддерживаемые `skeleton`, `envelopes`, `region_graph` и
«pivot graph» запрещены: они неизбежно расходятся. Любая summary — только
человекочитаемое описание объектов canonical graph, не второй источник истины.
`[MATHEMATICALLY_REQUIRED]`

## Identity и cardinality

`PatchDomain` — единственное поле распространения. pChain, PhysicalChain,
ChainUse, seed и envelope не владеют собственным domain. `[USER_REQUIRED]`

`PhysicalChain` — физическая identity линии и её канонический физический
порядок. `ChainUse` — directed patch-side view этой линии.

- seam между Patch A и Patch B: одна PhysicalChain, два ChainUses в разных
  PatchDomains;
- `SEAM_SELF`: одна PhysicalChain, два разных ChainUses в одном PatchDomain;
- общий physical id не разрешает сливать uses;
- topological seam сам по себе не становится physical barrier.

Обычный directed ChainUse имеет один analysis-proven owner-interior sector и
рождает один FrontComponent. Он не получает автоматическую пару left/right.
Несколько components допустимы только когда snapshot явно доказал несколько
реальных owner-side sectors, например high-valence case. `[USER_REQUIRED]`

Все FrontComponents одного `(DecalRequestId, PatchDomainId)` вычисляются вместе,
но boundary/capacity одного component не останавливает остальные.

## Seeds и material contributions

- `FrontSeed` рождается из выбранного ChainUse.
- `CornerSeed` требует подтверждённую CornerRelation и все её incident uses.
- `JunctionSeed` требует подтверждённую JunctionRelation; per-Patch projection
  не выдаётся за отдельный global junction.
- `CapSeed` существует только на физическом конце route.
- `EndpointClaimSeed` задаёт competing endpoint ownership; фиктивный
  one-incident CornerSeed запрещён.

`StripEnvelope | CornerEnvelope | JunctionEnvelope | CapEnvelope` — tagged
union. Envelope является semantic contribution, а не independently emitted
mesh. Число faces и tessellation не входят в identity.

## Boundary-limited propagation v1

Boundary taxonomy:

- `TOPOLOGICAL_BOUNDARY_USE` — топологический факт;
- `PHYSICAL_DOMAIN_BARRIER` — неподвижное препятствие propagation;
- `SOURCE_LAUNCH_BOUNDARY` — support выбранного source, не блокирующий его
  собственный seed при launch.

Outer boundary, holes и явно помеченные barrier rails ограничивают domain.
Guide/fold/route без barrier role препятствием не становится.

Front не пересекает barrier с последующим rollback. На exact alpha контакта
возникает `BOUNDARY_CONTACT`. Разрешены exact clip, shrink/disappearance active
interval и движение endpoint contact по тому же boundary component.

Ключевое различие:

- endpoint contact может скользить и сокращаться без рождения ветви;
- interior contact, который уже требует split interval, выбора обхода или
  будущего merge, немедленно даёт `BARRIER_SPLIT_REQUIRED` на том же alpha.

Branch count в v1 не увеличивается. Затронутый component сохраняет последний
однозначный `effective_alpha`; requested alpha остаётся в diagnostics. Другие
components продолжают. Полный obstacle bypass отложен.

`FRONT_EXHAUSTED` — успешное именованное завершение.
`BARRIER_BYPASS_UNSUPPORTED` показывается как boundary capacity reached, а не
silent clamp или geometry fallback. `[USER_REQUIRED]`

## Coverage, interaction и ownership

Нормативный порядок:

```text
AnalysisSnapshotV1 + DecalRequestV1
  -> seeds/components grouped by (request, PatchDomain)
  -> local contributions
  -> BoundaryLimitedResolver
  -> PatchPrimitiveUnion
  -> PatchCoverageClaims
  -> IntraPatchFrontInteractionResolver
  -> ResolvedPatchCoverage
  -> OwnershipResolver
  -> SemanticArrangement/Coalescing
  -> GeometryBatch
```

Coverage сначала является reachability-aware union contributions одного
request в одном PatchDomain. Голый Boolean не имеет права телепортировать
материю за hole/barrier или терять provenance.

Interaction B, выбранный пользователем для case 16:

- сталкиваются крылья/юбки одной и той же decal operation;
- sources — разные pChains либо self-collision одного source;
- все участники имеют одинаковые DecalRequestId и PatchDomainId;
- до mutual arrival coverage не меняется;
- после arrival contributions клиппируются стабильным equality locus;
- новые material regions не рождаются;
- cross-request и cross-Patch collision запрещены.

Ownership идёт после resolved coverage. Он делит существующую материю на
source/station/UV claims, но не создаёт и не удаляет coverage.
`[USER_REQUIRED]`

## Corner, junction и mixed effective alpha

Corner/Junction — derived analysis relations, не новые primary topology units.
Их compile-static seeds являются атомарными входами kernel.

Cross-Patch junction имеет одну global JunctionRelation и shared anchor, затем
отдельные per-Patch projections/contributions. Между PatchDomains нет
propagation competition или collision, но topology/station coordination через
общую relation обязательна. `[MATHEMATICALLY_REQUIRED]`

Shared Corner/Junction envelope при разных effective alpha incident components
использует incident effective-alpha vector. Frozen сторона не растягивается до
alpha соседа. Если vector не определяет единственный single-cover result,
возникает `SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN`; соседние fronts продолжают.
Geometry materializer не имеет права закрывать щель по своему усмотрению.
`[USER_REQUIRED]`

## 16 канонических случаев

1. **C01 — straight owner-interior wing.** Один ChainUse -> один strip/front;
   physical ends получают caps; data split не создаёт cap.
2. **C02 — convex MITER.** Два incident uses и CornerRelation создают один
   miter core; foreign uses запрещены.
3. **C03 — convex BEVEL.** Один bevel chord/core без остаточного miter apex или
   fan behind chord.
4. **C04 — reflex corner.** Single-cover union без exterior convex wedge;
   station не сбрасывается.
5. **C05 — physical end of s.** Cap только на физическом конце, с постоянным
   terminal station; record boundary не конец s.
6. **C06 — endpoint boundary contact.** Exact contact, затем slide/shrink;
   interior split не маскируется под slide.
7. **C07 — data-chain segmentation.** Split/merge records не меняет physical
   route, cap, owner, station ledger или digest.
8. **C08 — T-junction.** Confirmed relation создаёт один junction core; порядок
   arms не назначает owner.
9. **C09 — X-junction.** Route pairing приходит из analysis; нельзя выводить
   pairing из id или угла в kernel.
10. **C10 — Y merge/split.** Один downstream single cover с сохранением полного
    upstream provenance.
11. **C11 — short segment endpoint claims.** Два EndpointClaimSeeds встречаются
    на ownerless divider; one-incident CornerSeed и smallest-id wins запрещены.
12. **C12 — curved internal divider.** Claim B имеет одну identity как
    CornerEnvelope двух incident uses. Exact curved divider остаётся внутренним
    ownership boundary и не меняет silhouette.
13. **C13 — BEVEL under another wing.** CornerSeed содержит только bevel incident
    uses; `other_wing_use` появляется лишь в same-request interaction после
    mutual arrival.
14. **C14 — proven saturation.** Один component freezes на effective alpha,
    companion продолжает; global Patch clamp запрещён.
15. **C15 — topology events during drag.** Compile-known simultaneous events
    применяются атомарно; runtime route discovery и id ordering запрещены.
16. **C16 — intra-Patch freeze B.** Same-request wings клиппируются equality
    locus; ownership-only overlap и новая материя запрещены.

Все 16 имеют status `DEFINED`. Нерешённых вариантов A/C нет: пользователь
выбрал B, а затем уточнил, что участники всегда принадлежат одной decal
operation.

## Pivot/correction cases

- **P01:** physical seam A/B -> два independent Patch plans, shared physical id.
- **P02:** `SEAM_SELF` -> два directed uses одного Patch, без auto-merge/barrier.
- **P03:** несколько sources одного request -> один Patch evaluation/union.
- **P04:** self-collision одного wing -> тот же B contract, две station readings.
- **P05:** cross-Patch junction -> shared anchor, per-Patch projections, no
  cross-Patch collision.
- **P06:** mixed-alpha Corner -> incident vector либо named unproven outcome.
- **P07:** mixed-alpha Junction -> incident vector либо named unproven outcome.

## Metamorphic matrix

Канон хранится в JSON matrices. Для каждого transform явно указано:

- `INVARIANT` — CanonicalGeometryDigest и семантический граф сохраняются;
- `SEMANTIC_CHANGE` — меняется конкретный объявленный fact/request/policy;
- `CONDITIONAL` — invariant только при сохранении указанных preconditions.

Обязательные transforms: record/id permutation, ChainUse reverse вместе с
направлением s, loop winding reverse, rigid translation, uniform scale вместе с
width, retriangulation, data-chain split/merge при той же physical line,
simultaneous event batching и малое возмущение без topology change. Ни iteration
order, ни smallest id, ни tolerance не могут выбрать topology или owner.

## CanonicalGeometryDigest

Digest сравнивает semantic graph, а не face order или float buffer:

- request/domain identity;
- coverage regions и adjacency;
- full physical/boundary/source lineage;
- owner и ownerless dividers;
- station/UV models и direction s;
- relation/seed/component/contribution identity;
- active intervals, branch count и boundary history;
- atomic events, requested/effective alpha и capacity;
- anchors и shared-junction projections.

Permutation входов, record segmentation и tessellation не меняют digest, если
не изменили семантику. `[MATHEMATICALLY_REQUIRED]`

## Named unsupported outcomes

Неподдерживаемое поведение всегда имеет имя и provenance:

- `BARRIER_SPLIT_REQUIRED` / `BARRIER_BYPASS_UNSUPPORTED`;
- `SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN`;
- `APPROXIMATE_MATERIALIZATION_PENDING` до отдельного допуска соответствующего
  geometry path;
- другие capability gaps добавляются только как explicit named outcomes.

Silent fallback, legacy geometry repair и универсальный clamp запрещены.

## На пересмотр — не семантика

Следующее не переносится в новый канон без отдельного решения:

- `[LEGACY_COMPATIBILITY]` точное совпадение face count/tessellation старого
  preview;
- `[LEGACY_COMPATIBILITY]` старые UV numeric conventions сверх сохранения
  station direction/continuity;
- `[IMPLEMENTATION_ACCIDENT]` station buckets как обязательный механизм freeze;
- `[IMPLEMENTATION_ACCIDENT]` автоматические left/right sectors на ChainUse;
- `[IMPLEMENTATION_ACCIDENT]` first/last generated vertex как criterion end;
- `[IMPLEMENTATION_ACCIDENT]` post-hoc union независимо материализованных
  pChain meshes;
- `[IMPLEMENTATION_ACCIDENT]` cap/owner divider на границе data record;
- `[OPEN_RESEARCH]` production arrangement backend и exact arithmetic package;
- `[OPEN_RESEARCH]` obstacle bypass со split/merge вокруг hole;
- `[OPEN_RESEARCH]` ownership tournament вне уже определённых local claims.

## EC0 gate

Гейт требует одновременно:

1. пользователь принимает словесное описание;
2. все 16 + 7 pivot/correction cases проходят JSON Schema и validator;
3. закрыты corner-owner, T-junction, конец s, BEVEL при чужом same-request
   столкновении, saturation, curved internal divider, topology changes при drag
   и case 16 freeze B;
4. boundary endpoint/interior distinction, launch exception, cross-Patch
   junction и mixed-alpha policy присутствуют явно;
5. нет `BLOCKED_PENDING_USER_DECISION` и «решим потом» внутри EC0 scope;
6. в активном EC0 package нет презентационных visual artifacts.

Текущий candidate удовлетворяет формальным пунктам только после зелёного запуска
`python tools/validate_envelope_ec0.py`; пользовательская приёмка остаётся
отдельным обязательным решением.

## Риски

- JSON corpus может быть внутренне согласован, но ошибочно отображён будущим
  host adapter; это закрывается cross-contract validation и Blender screenshots.
- Exact arrangement и mixed-alpha shared envelopes остаются сложными для
  реализации, хотя их outcomes уже специфицированы.
- Cross-Patch junction легко случайно превратить либо в collision, либо в два
  независимых anchors; обе ошибки должен ловить digest/validator.
- Запрет branch birth ограничивает первый backend на concave/hole domains; это
  осознанная capability boundary, не дефект молчаливого fallback.

## Особое мнение

EC0 не доказывает вычислимость всех exact geometry операций и не должен
притворяться реализационным дизайном. Его задача — сделать неверный результат
обнаружимым: один граф, тотальная identity, именованные outcomes и executable
validator. Если будущий kernel не может построить однозначный single-cover
результат, честный named failure предпочтительнее правдоподобной геометрии,
которую contract не объясняет.
