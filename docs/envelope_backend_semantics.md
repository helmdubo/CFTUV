# Envelope backend semantics — EC0 AM11 Linear-Reflex candidate

## Статус и область

Статус: `SESSION_A_ACCEPTED`.

Это нормативная словесная спецификация EC0 после Linear-Axis rebaseline AM11.
Она определяет входные факты, request policy, compiled semantic plan и
приёмочные результаты нового Blender-free envelope kernel. Ей соответствует
canonical JSON corpus v5 в `artifacts/envelope_ec0/corpus/`.

Session A закрыта: локальный validator и внешний GitHub workflow зелёные, а
пользователь принял семантику вместе с выбранным
`LINEAR_REFLEX_MAX_SUBTURN_V1 = π/3 = 60°`. EC1 разрешено открывать только как
Session B в отдельном restricted context.

Документ не описывает geometry evaluator, Blender integration или production
Boolean backend. Он не переносит legacy geometry и не разрешает исполнителю
этой роли читать `cftuv/decal_voronoi.py` либо geometry-части
`cftuv/decals.py`. `[USER_REQUIRED]`

## Политика артефактов

Нормативные EC0-артефакты — prose, JSON, JSON Schema, metamorphic matrices и
воспроизводимый validator/CI output. Презентационные SVG/PNG-листы, contact
sheets, decks, diagrams и interactive HTML запрещены. `[USER_REQUIRED]`

Разрешены только диагностические скриншоты Blender viewport, UV Editor и debug
overlays. Скриншот показывает runtime-наблюдение, но сам по себе не получает
SemanticAuthority.

## SemanticAuthority

В активную семантику разрешено переносить только утверждения с одним из тегов:

- `USER_REQUIRED` — пользователь явно выбрал продуктовое поведение;
- `FIELD_PROVEN` — результат устойчиво подтверждён производственной практикой;
- `MATHEMATICALLY_REQUIRED` — правило необходимо для непротиворечивой
  topology, provenance, exact partition или determinism.

Следующие категории не являются каноном и всегда остаются в явном списке «на
пересмотр»:

- `LEGACY_COMPATIBILITY`;
- `IMPLEMENTATION_ACCIDENT`;
- `OPEN_RESEARCH`.

Совпадение со старым preview не повышает правило до SemanticAuthority.

## Разделение authority: snapshot, request и compiled plan

### AnalysisSnapshotV1

Snapshot содержит только host-observed facts и relations:

- Patch и PatchDomain;
- PhysicalChain и directed ChainUse;
- boundary constraints, holes/barriers и SourceVertex;
- analysis-proven Patch sectors, включая `owner_sector_id`, ordered incident
  supports и turn orientation;
- CornerRelation и JunctionRelation;
- exact/certified oriented owner-sector angle certificates;
- physical и boundary lineage.

Snapshot не содержит seeds, FrontComponents, EnvelopeSpecs,
EnvelopeInstances, active intervals, runtime events, requested/effective
alpha, capacity, ownership или GeometryBatch. Это kernel-owned state.
`[MATHEMATICALLY_REQUIRED]`

### DecalRequestV1

Request содержит:

- `DecalRequestId`;
- выбранные ChainUses;
- requested alpha;
- UV/station policy;
- единый `AngularProfileId = LINEAR_REFLEX_EQUAL_V1` для подтверждённых Angular
  relations;
- named selection policy `MIN_K_FOR_MAX_SUBTURN_V1` и ссылку на её product
  parameter `LINEAR_REFLEX_MAX_SUBTURN_V1`;
- request-level interaction intent.

Angular profile не является analysis-фактом. Analysis сообщает relation,
ориентированный owner-sector и angle certificate. Request выбирает единую
profile family и named policy; compiled plan вычисляет `k` из certificate и
policy. Ручной K0/K1 selector запрещён.
`[USER_REQUIRED]`

Разные requests остаются разными decal operations даже внутри одного Patch.
Они не участвуют в collision друг с другом. `[USER_REQUIRED]`

### CompiledPatchEvaluationPlan

План ключуется `(DecalRequestId, PatchDomainId)` и содержит все активные sources
этого request в этом PatchDomain:

- compile-static seeds;
- FrontComponents, active intervals и capacity states;
- typed EnvelopeSpecs и alpha-bound EnvelopeInstances;
- boundary events и lazy deterministic event ledger;
- exact patch union и разрешённые intra-Patch interactions;
- явный ownership partition;
- station/UV claims;
- downstream-tessellation contract и GeometryBatch provenance.

Per-pChain private evaluation с последующим stitching/union запрещён.
`[USER_REQUIRED]`

## Единственный authoritative ID graph

Каждый case имеет один canonical graph:

1. `analysis_snapshot`;
2. `decal_request`;
3. `expected_compiled_plan`;
4. `acceptance` assertions.

Параллельные вручную поддерживаемые skeleton/envelope/region/pivot graphs
запрещены. Summary может только ссылаться на canonical IDs, но не создавать
второй источник истины. `[MATHEMATICALLY_REQUIRED]`

## PatchDomain, ChainUse и FrontComponent

`PatchDomain` — единственное propagation field. PhysicalChain, ChainUse, seed
и EnvelopeSpec не владеют частным domain. `[USER_REQUIRED]`

`PhysicalChain` хранит identity физической линии. `ChainUse` хранит один
directed patch-side use с owner Patch, PatchDomain, orientation и roles.

- seam между Patch A/B: одна PhysicalChain и два ChainUses в разных domains;
- `SEAM_SELF`: одна PhysicalChain и два разных ChainUses в одном domain;
- общий physical id не разрешает сливать uses;
- топологический seam-cut сам по себе не становится физическим barrier.

Обычный directed ChainUse имеет ровно один analysis-proven owner-interior
sector и рождает один FrontComponent. Дополнительные sectors/components
допустимы только при явном analysis proof реальных отдельных Patch sectors;
автоматическая пара абстрактных сторон запрещена. `[USER_REQUIRED]`

Capacity/contact одного component не останавливает остальные sources request.

## Typed EnvelopeSpec union

`EnvelopeSpec` — compile-static semantic law, а не emitted mesh. Его tagged
union содержит ровно четыре варианта: `[USER_REQUIRED]`

1. `StripEnvelopeSpec` — линейная юбка одного FrontComponent;
2. `AngularEnvelopeSpec` — профиль связи incident fronts в строгом угловом
   owner-sector;
3. `JunctionEnvelopeSpec` — material law подтверждённой JunctionRelation;
4. `CapEnvelopeSpec` — физическое terminal closure strip.

`EnvelopeInstance` — отдельный record применения spec к incident effective
alpha vector и boundary-resolved состоянию. Spec identity не меняется при drag;
instance state меняется на точных событиях. Число emitted faces не является
identity ни spec, ни instance. `[MATHEMATICALLY_REQUIRED]`

Seeds и variants соотносятся так:

- FrontSeed → StripEnvelopeSpec;
- CornerSeed → AngularEnvelopeSpec;
- JunctionSeed → JunctionEnvelopeSpec;
- CapSeed → CapEnvelopeSpec;
- EndpointClaimSeed создаёт ownership claim, но не выдумывает Angular relation.

## AngularEnvelopeSpec и LinearReflexProfile

### Геометрический сектор

AngularEnvelopeSpec допустим только для owner-interior propagation sector с
углом `π < φ < 2π`. Здесь `φ` — угол именно owner-sector, а не бытовая метка
«выпуклый/вогнутый» исходной линии. Поэтому прежние разные corner labels больше
не выбирают независимые kernel-алгоритмы. `[USER_REQUIRED]`

Определяется reflex excess:

`Δ = φ - π`, где `0 < Δ < π`.

Exact `φ = 2π` не является обычным AngularEnvelopeSpec. Это physical terminal
или Junction relation в зависимости от topology. `[MATHEMATICALLY_REQUIRED]`

### Единый профиль и angle-driven selection

Session A фиксирует один `AngularProfileId` v1:

```text
LINEAR_REFLEX_EQUAL_V1
```

Для него reflex excess делится на `k + 1` равных subturns. `k` вычисляется
named product policy, а не выбирается отдельным profile ID:

```text
selection policy: MIN_K_FOR_MAX_SUBTURN_V1
parameter:        LINEAR_REFLEX_MAX_SUBTURN_V1 = Δ_MAX
k = max(0, ceil(Δ / Δ_MAX) - 1)
```

Это наименьшее целое `k >= 0`, для которого
`Δ / (k + 1) <= Δ_MAX`. Compiled plan хранит exact/certified lower/upper
comparison certificate в machine-readable форме
`k < Δ / Δ_MAX <= k + 1` (для `k = 0` нижняя граница следует также из
`Δ > 0`). Validator выводит `k` из этого interval certificate и сверяет все
dependent fields; он не доверяет отдельно записанному `resolved_k`. Ручной
выбор `k`, fixed case mapping, fixture-specific
threshold, sampling, tessellation, face count и Boolean output не имеют
SemanticAuthority. `[USER_REQUIRED]`

Пользователь утвердил точное продуктовое значение:

`LINEAR_REFLEX_MAX_SUBTURN_V1 = Δ_MAX = π/3 = 60° = 1/6 turn`.

Это semantic default, а не implementation tolerance. После runtime tests его
можно изменить только явной правкой corpus/policy, повторной валидацией и новой
приёмкой. Молчаливый numeric retuning запрещён.

### Ориентированный owner-sector и hidden supports

Каждая Angular relation ссылается на analysis-proven `owner_sector_id` и
хранит:

- ordered incoming/outgoing `ChainUse` supports;
- owner Patch и PatchDomain;
- turn orientation в ориентации owner Patch;
- exact или certified oriented angle `φ`;
- strict certificate `π < φ < 2π`.

Порядок supports — семантический. Простая перестановка records его не меняет,
но swap incoming/outgoing без согласованного изменения orientation меняет
семантику.

Для вычисленного `k` hidden support с ordinal `j`, `1 <= j <= k`, располагается
на turn fraction `j / (k + 1)` от incoming к outgoing support внутри
ориентированного owner-sector. Hidden support:

- принадлежит только своему AngularEnvelopeSpec;
- хранит lineage relation + sector + profile + selection certificate;
- не становится PhysicalChain, ChainUse, boundary rail или глобальной линией;
- не участвует в host analysis;
- может быть переставлена как storage record без изменения semantic digest.

Профиль имеет `k + 2` active supports и `k + 2` локальных
AngularEnvelope profile segments **до** clip/union/interaction. Это не
обязательное число exposed silhouette segments: после resolution отдельные
profile segments могут стать внутренними. Изменение certified angle или
`Δ_MAX`, которое меняет вычисленный `k`, — семантическое изменение.
`[MATHEMATICALLY_REQUIRED]`

### Regression fixtures, а не product mapping

`LINEAR_REFLEX_K0_EQUAL_FIXTURE_V1` и
`LINEAR_REFLEX_K1_EQUAL_FIXTURE_V1` отмечают результаты regression cases, где
policy certificate разрешает соответственно `k = 0` и `k = 1`. Они не являются
`AngularProfileId`, пользовательским выбором или отдельной продуктовой таблицей.
C02 и C03 — именованные K0/K1 fixtures. C04, C12, C13 и P06 также получают `k`
только из angle certificate + policy и не закрепляют исторический case mapping.
P07 остаётся `JunctionEnvelopeSpec`, поэтому Angular selection к нему не
применяется.

## Strip support law

`StripEnvelopeSpec` использует закон
`PLANAR_LINEAR_NORMAL_OFFSET_V1`: `[MATHEMATICALLY_REQUIRED]`

- source support: `n · x = c`;
- moving support: `n · x = c + alpha`;
- `n` ориентирована в owner interior;
- normal speed равна единице;
- longitudinal axis задаётся semantic ChainUse station `s`;
- transverse coordinate — signed owner-interior `r`;
- terminal interfaces приходят только от ссылок на Angular, Junction или Cap
  spec либо от явно объявленного domain/request terminal.

Data-record split не меняет support law, не создаёт cap и не сбрасывает `s`.

## Cap closure law

`CapEnvelopeSpec` использует
`PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1`: `[MATHEMATICALLY_REQUIRED]`

- cap допустим только на физическом endpoint route либо явной TerminalRelation;
- cap соединяет две longitudinal boundaries incident strip;
- station на cap постоянна и равна station физического endpoint;
- effective alpha следует incident strip;
- cap не является overlay matter;
- data-record split не создаёт cap;
- exact two-pi topology разрешается как Terminal/Junction, но не как обычный
  AngularEnvelopeSpec.

## Boundary-limited propagation v1

Boundary facts различают:

- `TOPOLOGICAL_BOUNDARY_USE`;
- `PHYSICAL_DOMAIN_BARRIER`;
- `SOURCE_LAUNCH_BOUNDARY`.

Launch support не блокирует originating seed. Outer boundary, holes и явно
помеченные barrier rails ограничивают domain. Guide/fold/route без barrier role
препятствием не становится. `[USER_REQUIRED]`

Endpoint contact может точно clip/slide/shrink на том же boundary component.
Interior contact, уже требующий split active interval, выбора пути или merge,
немедленно выдаёт `BARRIER_SPLIT_REQUIRED` на exact contact alpha.

FrontComponent v1 не увеличивает branch count. Затронутый component сохраняет
requested alpha, фиксирует последний доказанный effective alpha и named
capacity reason; остальные components продолжаются. Silent disappearance,
rollback и material teleportation за obstacle запрещены. `[USER_REQUIRED]`

## Coverage, interaction и ownership

Нормативный порядок: `[USER_REQUIRED]`

```text
AnalysisSnapshotV1 + DecalRequestV1
→ seeds/fronts/specs grouped by (DecalRequestId, PatchDomainId)
→ alpha-bound EnvelopeInstances
→ boundary-limited resolution
→ exact patch single-cover union
→ permitted intra-Patch interactions
→ ResolvedCoverage
→ explicit ownership partition
→ UV/station arrangement and semantic coalescing
→ downstream tessellation
→ GeometryBatch
```

### Interaction policy B

Сталкиваются только юбки/чтения одной и той же decal operation внутри одного
PatchDomain. Это могут быть fronts разных pChains либо self-contact одного
front. До mutual arrival coverage не меняется. После arrival существующие
contributions клиппируются equality locus; новая третья материя не рождается.
Cross-request и cross-Patch collision запрещены. `[USER_REQUIRED]`

### Явные ownership claims

Каждый patch plan объявляет claims до tessellation. Они обязаны удовлетворять
`TOTAL_DISJOINT_RESOLVED_COVERAGE_V1`: `[MATHEMATICALLY_REQUIRED]`

- interiors claims попарно не пересекаются;
- union closures claims равен ResolvedCoverage;
- equality boundary ownerless;
- каждый claim ссылается на canonical EnvelopeSpec/EnvelopeInstance lineage;
- ownership не изменяет silhouette;
- недоказуемый partition возвращает `OWNERSHIP_PARTITION_UNPROVEN`.

First-wins, smallest-id, tolerance-selected owner и восстановление provenance
после Boolean запрещены.

## Junction и mixed effective alpha

JunctionRelation — derived analysis relation. JunctionEnvelopeSpec использует
только объявленные relation supports и route pairing; kernel не угадывает их по
порядку, углам или runtime IDs. `[USER_REQUIRED]`

Cross-Patch junction имеет одну global relation и shared semantic anchor, затем
отдельные per-Patch projections. Между domains нет propagation competition или
collision, но topology/station coordination обязательна.
`[MATHEMATICALLY_REQUIRED]`

Angular/Junction spec при разных incident effective alpha получает полный
incident vector. Frozen arm не растягивается до alpha соседа. Если vector не
определяет единственный exact single-cover result, outcome —
`SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN`; соседние fronts продолжаются.
`[USER_REQUIRED]`

## Lazy deterministic event ledger

Compile/startup фиксирует immutable Envelope laws, current exact state, event
predicates, transition types и немедленно достижимые candidates. Полный список
всех будущих event instances заранее не требуется. `[USER_REQUIRED]`

После atomic event batch successor candidates планируются детерминированно и
лениво. События одного exact alpha применяются одной unordered semantic batch;
runtime ID order не влияет на результат. Backward drag использует checkpoint
replay или deterministic undo. Наружу публикуется только complete exact state;
во время незавершённого exact evaluation используется
`PENDING_EXACT_EVALUATION`, а не partial mesh.

## Downstream tessellation invariants

Tessellation начинается только после resolved coverage, interactions,
ownership, UV/station arrangement и semantic coalescing.
`[MATHEMATICALLY_REQUIRED]`

Допустимо менять:

- внутреннюю triangulation/polygonization;
- face order;
- семантически эквивалентные внутренние diagonals.

Обязательно сохраняются:

- resolved coverage silhouette;
- profile-controlled Angular boundary chain;
- ownership boundaries;
- UV/station interfaces;
- source provenance;
- shared semantic vertex identities.

Tessellation не выбирает profile, hidden-edge count, silhouette, owner или
provenance и не растворяет profile-controlled boundary. Face count не является
semantic authority. Все допустимые tessellations одного arrangement имеют один
CanonicalGeometryDigest. `[USER_REQUIRED]`

## 16 canonical cases

1. **C01:** один StripEnvelopeSpec одного ChainUse; physical terminals имеют
   CapEnvelopeSpecs.
2. **C02:** `LINEAR_REFLEX_EQUAL_V1`; angle certificate + policy разрешают
   `k = 0` и фиксируют K0 regression result.
3. **C03:** тот же profile; certificate + policy разрешают `k = 1`, поэтому
   hidden support стоит на `1/2` reflex excess.
4. **C04:** angle-driven Angular contribution может internalize после exact
   union, не добавляя exterior matter.
5. **C05:** Cap только на physical end of `s`.
6. **C06:** endpoint boundary contact slide/shrink; interior split — named
   capacity event.
7. **C07:** data-chain segmentation не меняет physical route/spec/digest.
8. **C08:** T-junction получает один JunctionEnvelopeSpec; arm order не
   назначает owner.
9. **C09:** X-junction route pairing приходит из relation.
10. **C10:** Y merge/split хранит полный upstream provenance и один downstream
    cover.
11. **C11:** два EndpointClaimSeeds делят short strip; фиктивной Angular relation
    нет.
12. **C12:** claim B явно охватывает две incident strips и один angle-driven
    AngularEnvelopeSpec; curved divider остаётся ownership-only internal curve.
13. **C13:** Angular spec с policy-resolved `k` содержит только свои incident
    uses; другая юбка той же decal/Patch входит только в post-arrival
    interaction и встречает фактическую exposed profile boundary.
14. **C14:** один component насыщается на proven effective alpha, companion
    продолжает.
15. **C15:** laws/predicates compile-declared, successor events lazy, same-alpha
    transitions atomic.
16. **C16:** выбран B — same-request same-Patch contributions клиппируются
    equality locus без overlap и без новой matter.

Все case outcomes имеют `DEFINED`; открытых
`BLOCKED_PENDING_USER_DECISION` нет. Corpus fixtures используют symbolic
certified comparisons относительно утверждённого `Δ_MAX = π/3`.

## Pivot/correction cases

- **P01:** physical seam A/B — два Patch plans, shared PhysicalChain identity.
- **P02:** `SEAM_SELF` — два directed uses одного Patch, без auto-merge/barrier.
- **P03:** все sources одного request/Patch входят в один evaluation/union.
- **P04:** self-collision одного wing использует interaction policy B.
- **P05:** cross-Patch junction имеет shared anchor и per-Patch projections без
  collision.
- **P06:** angle-driven Angular spec использует mixed incident effective-alpha
  vector.
- **P07:** Junction spec использует mixed incident effective-alpha vector.

## Metamorphic matrix и CanonicalGeometryDigest

Каждое transformation получает только один verdict:

- `INVARIANT` — canonical semantic graph/digest сохраняется;
- `SEMANTIC_CHANGE` — меняется объявленный analysis/request/spec fact.

AM3 transformations покрывают permutation, storage reverse с сохранением
семантического направления, winding reverse с сохранением domain, rigid
translation, coherent scale, retriangulation, data-chain split/merge,
simultaneous-event permutation и малое возмущение без topology change.

AM11 дополнительно фиксирует:

- hidden-support record permutation — invariant;
- coherent record permutation ориентированного owner-sector — invariant;
- swap incident support order без coherent orientation update — semantic
  change;
- изменение certified angle или `Δ_MAX` через границу выбора `k` — semantic
  change;
- ручной override вычисленного hidden-edge count — semantic change и invalid
  request;
- изменение subdivision law — semantic change;
- downstream tessellation при сохранении arrangement — invariant;
- dissolve exposed profile-controlled boundary — semantic change.

Digest включает request/domain identity, specs/instances, full lineage,
coverage/adjacency, ownership, station/UV, active/capacity state, exact events и
shared anchors. Он исключает face order, internal fill diagonals, допустимую
triangulation/polygonization и runtime face count.

## Named outcomes

- `BARRIER_SPLIT_REQUIRED`;
- `BARRIER_BYPASS_UNSUPPORTED`;
- `SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN`;
- `OWNERSHIP_PARTITION_UNPROVEN`;
- `PENDING_EXACT_EVALUATION`;
- `APPROXIMATE_MATERIALIZATION_PENDING` до отдельного допуска geometry path.

Silent fallback, legacy repair, universal clamp и partial mesh publication
запрещены.

## На пересмотр — не семантика

- `[LEGACY_COMPATIBILITY]` прежние UI join labels могут существовать только как
  request presets для единой family/policy; они не являются kernel types и не
  могут вручную назначать `k`.
- `[LEGACY_COMPATIBILITY]` exact legacy face count/tessellation.
- `[LEGACY_COMPATIBILITY]` старые numeric UV conventions сверх station
  direction/continuity.
- `[IMPLEMENTATION_ACCIDENT]` station buckets как обязательный freeze mechanism.
- `[IMPLEMENTATION_ACCIDENT]` автоматические left/right sectors одного ChainUse.
- `[IMPLEMENTATION_ACCIDENT]` first/last generated vertex как terminal test.
- `[IMPLEMENTATION_ACCIDENT]` per-pChain materialization и post-hoc stitching.
- `[IMPLEMENTATION_ACCIDENT]` cap/owner divider на data-record boundary.
- `[IMPLEMENTATION_ACCIDENT]` sampled fan или absent-apex repair как source of
  Angular silhouette.
- `[OPEN_RESEARCH]` production arrangement/Boolean/exact arithmetic package.
- `[OPEN_RESEARCH]` obstacle bypass со split/path-choice/merge.
- `[OPEN_RESEARCH]` generalized straight-skeleton backend.
- `[OPEN_RESEARCH]` ownership tournament вне объявленных local claims.

Ни один пункт этого списка не разрешён как молчаливый fallback.

## EC0 gate

EC0 закрыт после выполнения всех условий:

1. v5 corpus/schema/matrices проходят локальный validator;
2. внешний GitHub workflow проходит на фактически опубликованном commit;
3. пользователь принимает `LINEAR_REFLEX_EQUAL_V1`, angle-driven
   `MIN_K_FOR_MAX_SUBTURN_V1`, exact default `Δ_MAX = π/3 = 60°`,
   ориентированный owner-sector и закон `k + 2`;
4. приняты typed EnvelopeSpec union, Strip support law и Cap closure law;
5. приняты explicit ownership claims, lazy event ledger и tessellation
   invariants;
6. закрыты corner owner, T-junction, конец `s`, C13 interaction, saturation,
   curved internal divider, topology changes during drag и case 16 policy B;
7. нет `BLOCKED_PENDING_USER_DECISION` или скрытого «решим потом»;
8. нет presentation visuals в активном EC0 package.

Условия выполнены и записаны решением `SESSION_A_FINAL_ACCEPTANCE`. Session B
разрешена, но обязательное разделение контекстов сохраняется: EC1 нельзя
начинать внутри этой Session A.

## Риски

- Утверждённые 60° влияют на вычисленный `k`; их будущая корректировка по
  runtime tests является явным продуктовым изменением, а не безопасным
  implementation tuning.
- Symbolic fixture certificates проверяют алгоритм выбора, но не заменяют
  утверждение production parameter.
- Exact curved ownership divider и Junction core требуют backend, сохраняющий
  curve/provenance; выбор backend не входит в EC0.
- Host adapter ещё должен доказать стабильные два ChainUse для `SEAM_SELF` и
  корректные relation sectors; kernel не должен реконструировать эти facts.
- UI/debug обязаны различать requested alpha, per-component effective alpha и
  pending exact evaluation.
- C13 и C16 потребуют runtime fixtures как отдельный этап, хотя их semantic
  outcomes уже определены.

## Особое мнение

Один `LINEAR_REFLEX_EQUAL_V1` лучше выражает геометрическую семью, чем перечень
K0/K1 case presets. Дискретность должна следовать из certified angle и явно
утверждённого `Δ_MAX = 60°`; она не должна зависеть от mesh density или экрана.
Отдельный новый `AngularProfileId` нужен только для действительно другого
subdivision/motion law, а не для другого результата `k` той же policy.
