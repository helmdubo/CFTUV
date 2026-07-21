# Envelope kernel pivot — обязательные инструкции исполнителю

Статус: **NORMATIVE AFTER AM7–AM8**. Этот документ уточняет
`decal_envelope_roadmap_compromise.md` и обязателен для EC0-P–EC8.
При конфликте со старой формулировкой про «компоненты» действует этот pivot.

## 1. Роль и границы

Kernel implementer работает только с новым Blender-free пакетом, контрактными
документами и fixtures. Код `cftuv/decal_voronoi.py` и legacy-части
`cftuv/decals.py` не читается. Legacy Evidence Curator и Host Adapter Author —
отдельные сессии с отдельными разрешениями и deliverables.

Kernel implementer не имеет права:

- переносить legacy mechanism вместо специфицированного результата;
- исправлять или расширять host analysis «по догадке» внутри kernel;
- материализовать разные pChains независимо и затем склеивать результат;
- добавлять fallback в старую геометрию;
- начинать EC1 до закрытия EC0-P.

## 2. Нормативная модель

Короткая формула:

> `PatchDomain` — поле. `PhysicalChain` — физическая identity.
> `ChainUse` — направленный patch-side view. pChain/ChainUse — source фронта.
> Все sources одного Patch вычисляются совместно.

### Source facts

- `PatchDescriptor` — identity Patch и ортогональные axes
  (`SurfaceRegime`, `PatchShapeClass`, context tags).
- `PhysicalChain` — source edge/vertex identity и канонический физический
  порядок. Не содержит patch-side orientation.
- `ChainUse` — `physical_chain_id`, `owner_patch_id`, `boundary_loop_id`,
  orientation, side и role set.
- `BoundaryLoop` — ordered container над uses конкретного Patch.
- Corner/Junction relation — производный analysis fact над incident uses,
  sectors и source vertex identity.

### Domain

Один `PatchDomain` на owner Patch содержит:

- surface geometry и `SurfaceMetric`;
- outer boundary и holes;
- неподвижные barriers;
- patch sectors;
- planar/curved surface regime.

Outer boundary, hole loops и `ChainUse` с явной ролью `BARRIER` являются
неподвижными non-owner ограничителями. Rail без `BARRIER` автоматически
препятствием не становится: он может быть guide/fold/route по другим roles.

У pChain, ChainUse, FrontSeed или Envelope собственного domain нет. Они только
ссылаются на owner `PatchDomain`.

### Seeds

- `FrontSeed` рождается из выбранного `ChainUse`;
- `CornerSeed`, `JunctionSeed`, `CapSeed` рождаются из подтверждённых analysis
  relations;
- Corner/Junction не являются primary topology или solve units, но их
  compile-static seeds являются атомарными входами envelope compiler.

### Front components

На каждую тройку `(ChainUse, owner Patch, sector)` создаётся отдельный
`FrontComponent`. Он хранит source seed, active continuous intervals,
boundary-event history, branch count, `requested_alpha`, `effective_alpha` и
capacity reason. Components независимы по lifecycle, но вычисляются совместно
в одной patch-level evaluation group.

### Material contributions

`StripEnvelope | CornerEnvelope | JunctionEnvelope | CapEnvelope` — tagged
union. Envelope — вклад source, а не самостоятельно материализуемая decal.
Число faces и tessellation не входят в semantic identity.

## 3. Identity и patch-side invariants

1. Physical seam между Patch A и Patch B даёт одну `PhysicalChain` и два
   `ChainUse`, по одному на Patch.
2. `SEAM_SELF` даёт два разных `ChainUse` одного `PatchDomain`; общий
   `physical_chain_id` не разрешает их объединить.
3. Два uses могут иметь противоположные orientation/normals, разные loops,
   sectors и local coordinates.
4. Одна выбранная physical chain может породить два крыла одной decal в двух
   PatchDomains. Они связаны physical identity и longitudinal `s`, но
   поперечное распространение вычисляется отдельно в каждом domain.
5. Несколько pChains одного Patch являются разными sources одного общего
   evaluator invocation и одного patch-level material plan.
6. Перестановка PhysicalChain, ChainUse, seeds и envelopes не меняет
   `CanonicalGeometryDigest`.
7. Boundary event одного FrontComponent не останавливает другой component,
   другую pChain, противоположный use physical chain или весь Patch.

## 4. Единственный допустимый pipeline

```text
Patch / PhysicalChain / ChainUse facts
    -> PatchDomain + sectors/holes/barriers
    -> FrontSeed / CornerSeed / JunctionSeed / CapSeed
    -> FrontComponents grouped by owner Patch
    -> local envelope contributions
    -> BoundaryLimitedResolver
    -> BoundaryResolvedContributions
    -> exact PatchPrimitiveUnion
    -> PatchCoverageClaims
    -> IntraPatchFrontInteractionResolver
    -> ResolvedPatchCoverage
    -> OwnershipResolver
    -> SemanticArrangement + Coalescing
    -> GeometryBatch
```

Unit of execution reference-evaluator — **один Patch со всеми его активными
sources на данном alpha**, а не одна pChain.

## 5. Coverage, interaction, ownership

### Boundary-limited propagation v1

Policy id: `BOUNDARY_LIMITED_PROPAGATION`.

- Front никогда сначала не пересекает boundary с последующим rollback. На
  точном alpha контакта возникает `BOUNDARY_CONTACT`.
- Разрешены clip, сокращение active interval, исчезновение interval и движение
  contact point вдоль того же boundary component.
- Число active branches одного FrontComponent не может увеличиваться.
- Если дальнейшее движение требует split вокруг hole/concave boundary, выбора
  левого/правого path или последующего merge, возникает capacity reason
  `BARRIER_SPLIT_REQUIRED`.
- Component сохраняет последний однозначный `effective_alpha`; новый
  `requested_alpha` остаётся в diagnostics. Остальные components продолжают.
- `FRONT_EXHAUSTED` — успешное завершение. Capability outcome
  `BARRIER_BYPASS_UNSUPPORTED` представляется пользователю как
  `BOUNDARY_CAPACITY_REACHED`, не как красная ошибка или silent clamp.

Boundary не является owner и не получает UV/station/material region. Он даёт
clip lineage, contact key, event и capacity provenance.

### Coverage

Для PatchDomain `Ω` и source contributions `E_i(alpha)`:

```text
PatchPrimitiveUnion(alpha) = Ω intersect union_i E_i(alpha)
```

Формула допустима только после reachability-aware boundary resolution.
Голый Boolean не имеет права создать material за hole/barrier, если для этого
FrontComponent должен был бы породить branches. Итог остаётся одним
single-cover material graph; contributions сохраняются в provenance/claims,
но не материализуются как overlapping meshes.

### Interaction

Interaction может законно менять coverage только по объявленной product
semantic и compile-known event. Выбранный case 16 B означает:

- встречаются юбки одной decal внутри одного Patch;
- источники — разные pChains либо два fronts self-collision одного source;
- до mutual arrival вклады растут без взаимного эффекта;
- после mutual arrival вклады клиппируются по стабильному equality locus;
- разные decal и разные Patch не входят в один interaction-system;
- freeze не является ownership-only overlap и не создаёт новую материю.

### Ownership

Ownership применяется после resolved coverage. Он назначает source/station/UV
claim каждому открытому региону, но не меняет существующую материю. Patch и
decal owner при intra-Patch collision не меняются.

## 6. Front lifecycle

Критерий lifecycle основан на active intervals, а не на «первой/последней
вершине»: вершины могут рождаться, схлопываться и сливаться. FrontComponent не
обязан единолично покрыть весь Patch, но не имеет права молча исчезнуть.
Допустимы только:

1. дальнейшее распространение;
2. именованное compile event: `boundary`, `collision`, `freeze`, `cap`,
   `collapse`, `saturation`, `birth`, `death`; `split/merge` допустимы только
   как отдельно специфицированные non-obstacle semantic events, не как обход
   hole/barrier в v1;
3. named failure, если корректное продолжение не доказано.

`FRONT_EXHAUSTED` наступает, когда active intervals пусты либо полностью
заморожены boundary/competition events. Любой путь «перестал создавать faces
без event/failure» — ошибка контракта.

## 7. Контракт SnapshotV1

SnapshotV1 принадлежит kernel. Host adapter экспортирует, но не определяет
параллельную схему. Snapshot обязан позволять валидировать:

- каждый `ChainUse` ссылается на существующие PhysicalChain, Patch и loop;
- uses одной physical chain не потеряны и не слиты;
- каждый seed имеет source relation и owner Patch;
- каждый FrontComponent имеет ChainUse/Patch/sector, active intervals и
  branch-count history;
- каждый Envelope ссылается на seed и PatchDomain;
- все sources сгруппированы по owner Patch до evaluator;
- Corner/Junction seeds имеют incident uses, sector ordering и source vertex;
- holes/barriers/domain boundaries принадлежат ровно одному domain record.
- requested/effective alpha и capacity reason принадлежат конкретному
  FrontComponent, а не неявному global clamp.

## 8. EC0-P deliverables и гейт

До EC1 обновляются визуальные листы и coordinate-free sidecars. Каждый case
показывает/описывает семь слоёв:

1. `PatchDomain` и reachable sectors;
2. physical chain identities;
3. directed `ChainUse`;
4. compile-static seeds;
5. coverage contributions и patch union;
6. interaction boundary/events;
7. ownership и UV/station flow.

Минимальные pivot fixtures:

- physical chain между Patch A/B -> два uses и два domains;
- `SEAM_SELF` -> два uses одного domain;
- 2–3 pChain sources в одном PatchDomain -> один union;
- T/X/Y через derived seed, без независимых нашлёпок;
- collision разных sources одной decal;
- self-collision одного source;
- одно касание прямой outer boundary;
- скользящий контакт с косой boundary;
- полное исчезновение front в узком sector;
- первое касание hole;
- hole, требующий split;
- concave outer boundary, требующий split;
- simultaneous contact с двумя boundaries;
- barrier rail с явной ролью `BARRIER`;
- два крыла одной physical chain: одно saturated, второе продолжает;
- другая pChain того же Patch продолжает после saturation первой.

Каждый boundary-case показывает before, exact-event after-state и
`requested_alpha > effective_alpha`.

EC0-P закрыт только после пользовательской приёмки визуалов и отсутствия
неименованных решений. Выбор B из прежнего case 16 переносится без повторного
выбора; повторно проверяется его новый domain/use/seed контекст.

Candidate corpus уже выпущен в `artifacts/envelope_ec0/v2/`; словесный маршрут
приёмки — `docs/envelope_ec0_acceptance_guide.md`. До явного ответа пользователя
статус остаётся `READY_FOR_USER_REVIEW`, и EC1 не начинается.

## 9. Запреты для review checklist

- private field/domain на pChain, ChainUse, seed или envelope;
- per-source solve + post-hoc mesh union;
- два material faces на одном открытом участке PatchCoverage;
- восстановление ownership/provenance после Boolean;
- слияние patch-side uses по `physical_chain_id`;
- junction как случайный overlap независимых strips;
- Corner/Junction как новая primary topology unit;
- subclass explosion (`PlanarBandWallPatch`, `SelectedPChain`, и т. п.);
- first-wins/smallest-id, epsilon или iteration order как semantic authority;
- silent front disappearance;
- cross-decal или cross-Patch collision;
- first/last front vertex как completion/capacity criterion;
- boundary crossing с последующим rollback;
- увеличение branch count в v1;
- material teleportation за hole/barrier;
- остановка всей pChain/Patch из-за boundary event одного component;
- runtime topology discovery между compile events;
- host-adapter geometry repair или fallback.

## 10. Обязательный handoff kernel-сессии

Исполнитель сообщает:

- активную EC-фазу и закрытый gate;
- какие PatchDomain/PhysicalChain/ChainUse invariants проверены;
- unit of execution использован patch-level или нет;
- какие named events/failures покрыты;
- изменился ли `CanonicalGeometryDigest` contract;
- прочитаны ли запрещённые legacy-файлы (должно быть «нет»);
- риски и особое мнение отдельно от канона.

## 11. Риски

1. Host analysis может пока не экспортировать стабильную PhysicalChain identity,
   полную пару ChainUses или sector ordering для `SEAM_SELF`. Это contract gap
   EC1 и named failure, а не разрешение kernel угадать недостающие facts.
2. Patch-level evaluation увеличит число simultaneous contributions. Сначала
   доказываются single-cover и provenance reference path; spatial indexing и
   event compilation начинаются только после эквивалентности.
3. Planar и curved evaluators не должны расходиться в facts/domain/seed IR.
   Различается metric/evaluator strategy, не внешний semantic graph.
4. Одна physical chain может связать два PatchDomains общей `s`, но collision
   остаётся intra-Patch. Попытка разрешать freeze поперёк Patch boundary снова
   смешает seam pairing и propagation field.
5. Роль pChain как `BARRIER` и как `SELECTED_SPINE` может сочетаться в role set;
   влияние каждой роли на reachable sectors должно быть задано до EC2.
6. Exact Boolean clip без reachability ledger визуально выглядит правдоподобно,
   но может телепортировать material за obstacle. Это отдельный EC2 gate.

## 12. Особое мнение kernel-implementer

Рекомендую сделать `PatchEvaluationPlan` явным kernel-owned aggregate с одним
domain, полным набором uses/seeds и одной таблицей source contributions. Это
лучше защищает unit of execution, чем API вида `evaluate(seed, domain)`, даже
если последний кажется удобнее для первых тестов.

Также рекомендую сохранить отдельный `PatchCoverageClaims` между union и
interaction: B меняет resolved coverage, но provenance исходных contributions
должна оставаться доступной для equality locus, ownership и digest. Это мнение
об именах/форме IR; нормативны patch-level execution и порядок стадий.

Для AM8 рекомендую хранить active intervals и capacity на FrontComponent, но
вызывать resolver одной patch-level операцией. Это сохраняет независимый
lifecycle components без возврата к запрещённому per-pChain solve.
