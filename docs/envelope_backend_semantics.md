# Семантика envelope-бэкенда до алгоритма (EC0a + EC0b)

Статус документа: **PIVOT REVISION REQUIRED (EC0-P)**. Семантические исходы
16 случаев сохраняются; для case 16 пользователь выбрал **B — coverage clip**,
а A/C отвергнуты. Но visual/sidecar corpus v1 был построен до AM7–AM8 и не
различает во всех случаях `PatchDomain`, `PhysicalChain`, `ChainUse` и seeds.
Поэтому он не открывает EC1 до повторного EC0-P выпуска и приёмки.

Основание: `docs/decal_envelope_roadmap_compromise.md` @ `a7d64ee`, включая
AM1–AM6, `SemanticAuthority` и `CanonicalGeometryDigest`. При подготовке EC0
старый backend и его implementation code не использовались. Переносится
наблюдаемая продуктовая семантика, а не прежний механизм.

Артефакты:

- визуальные листы SVG/PNG: `artifacts/envelope_ec0/sheets/`;
- coordinate-free YAML-sidecars: `artifacts/envelope_ec0/cases/`;
- метаморфная матрица AM3:
  `artifacts/envelope_ec0/metamorphic_matrix.yaml`;
- описание sidecar-формата:
  `artifacts/envelope_ec0/case_sidecar_schema.md`.
- AM8 boundary policy pack:
  `artifacts/envelope_ec0/pivot/`.
- обязательные инструкции pivot:
  `docs/envelope_kernel_pivot_instructions.md`.

## 1. Как читать визуальные листы

Текущий v1-лист содержит пять зон:

1. `SKELETON` — редкий семантический skeleton: физические линии, углы,
   junction и domain/barrier;
2. `COVERAGE` — где существует материя после union/clip, без внутренних
   ownership-разделителей;
3. `OWNERSHIP` — тотальное неперекрывающееся разбиение coverage;
4. `UV / STATION FLOW` — направление `s`, поперечное `r`, station/UV-интерфейсы;
5. `ALPHA EVOLUTION` — состояние до события, на точной высоте и после него.

Схемы не являются координатными фикстурами и не задают допуски. Их задача —
утвердить topology, lineage, owner, adjacency, направление `s` и события.

EC0-P заменяет/расширяет каждый лист так, чтобы отдельно были видны семь
семантических слоёв: `PatchDomain`, physical chain identity, directed
`ChainUse`, compile-static seeds, patch-level coverage union, interaction
boundaries/events, ownership + UV/station flow. Простая картинка «декаль вокруг
chain» больше не проходит гейт.

## 2. SemanticAuthority

У каждой переносимой нормы есть один из тегов:

- `USER_REQUIRED` — явная продуктовая воля пользователя;
- `FIELD_PROVEN` — наблюдаемый результат подтверждён полем;
- `MATHEMATICALLY_REQUIRED` — следует из single-cover, exact union,
  топологической эквивалентности или детерминизма;
- `LEGACY_COMPATIBILITY` — историческое поведение, которое ещё не заслужило
  статуса семантики;
- `IMPLEMENTATION_ACCIDENT` — след прежней структуры данных/алгоритма;
- `OPEN_RESEARCH` — вопрос, который обязан быть решён экспериментом или
  пользователем.

`LEGACY_COMPATIBILITY`, `IMPLEMENTATION_ACCIDENT` и `OPEN_RESEARCH` сами по
себе не могут создавать канон. Первые два вынесены в §11. Бывший открытый
вопрос AM4 закрыт пользовательским выбором B и уточнением scope в §9.

## 3. Общая семантика envelope-модели

### 3.1 Source facts, PatchDomain и unit of execution

- **[SemanticAuthority: USER_REQUIRED]** `PatchDomain` — единственное поле
  распространения. Он владеет surface metric, outer boundary, holes,
  barriers, sectors и surface regime.
- **[SemanticAuthority: USER_REQUIRED]** pChain не является полем. Физическая
  identity хранится в `PhysicalChain`; направленный patch-side view хранится
  в `ChainUse` с owner Patch, loop, orientation, side и roles.
- **[SemanticAuthority: USER_REQUIRED]** Physical seam между Patch A/B даёт
  два ChainUse в двух PatchDomains. `SEAM_SELF` даёт два разных ChainUse
  одного PatchDomain. Общий `physical_chain_id` не разрешает слияние uses.
- **[SemanticAuthority: USER_REQUIRED]** Все FrontSeeds одного owner Patch
  вычисляются совместно в одном evaluator invocation. Отдельный private
  domain/solve/materialization на pChain запрещён.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Corner/Junction — derived
  analysis relations над incident uses/sectors/source vertex. Они не primary
  topology units, но их compile-static seeds являются атомарными входами
  envelope compiler.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Unit of execution
  reference-evaluator — один Patch со всеми активными sources на данном alpha.

Нормативные уровни:

`Patch/PhysicalChain/ChainUse facts → PatchDomain/sectors → seeds →
FrontComponents → boundary-resolved contributions → patch union →
front interactions → ownership → GeometryBatch`.

### 3.2 Coverage и ownership

- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Для одного Patch coverage
  до front-front interactions — exact union всех boundary-resolved
  `StripEnvelope | CornerEnvelope | JunctionEnvelope | CapEnvelope` одного
  `PatchDomain`. Contribution каждого source сохраняется в provenance, но не
  материализуется отдельной overlapping mesh.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Coverage и ownership —
  разные графы над одной геометрией: сначала определяется существующая
  материя, затем тотальный и disjoint owner каждого открытого региона.
- **[SemanticAuthority: USER_REQUIRED]** Каждый примитив — отдельный вариант
  tagged union с semantic identity, provenance, законом от alpha, boundary
  curves, station map, owner patch и domain. Один класс с optional-полями не
  является эквивалентом.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Число runtime faces и
  конкретная тесселяция не входят в определение примитива или региона.
- **[SemanticAuthority: USER_REQUIRED]** Collision scope всегда ограничен
  одной decal и одним owner Patch, внутри которого выполняется offset. Его
  участники — крылья/юбки, порождённые pChains; возможны встреча юбок разных
  pChains и самостолкновение одной юбки. Разные decal и разные Patch никогда
  не участвуют в одном collision-system.
- **[SemanticAuthority: USER_REQUIRED]** После mutual arrival взаимодействие
  юбок меняет resolved coverage: вариант B обрезает участвующие покрытия по
  mutual equality locus. Это не ownership-only overlap и не новая материя.

Термин `component` в старом AM4 означает только source contribution/front
одной decal внутри одного Patch, а не отдельное поле или decal identity.

Нормативный порядок стадий:

`EnvelopeContributions → BoundaryLimitedResolver →
BoundaryResolvedContributions → PatchPrimitiveUnion → PatchCoverageClaims →
IntraPatchFrontInteractionResolver → ResolvedPatchCoverage →
OwnershipResolver → SemanticArrangement`.

Имя `IntraPatchFrontInteractionResolver` нормативно заменяет неоднозначное
roadmap-имя `IntercomponentInteractionResolver`; стадийность AM4 сохраняется.

### 3.3 Boundary-limited propagation v1

- **[SemanticAuthority: USER_REQUIRED]** Policy
  `BOUNDARY_LIMITED_PROPAGATION` создаёт отдельный `FrontComponent` для
  `(ChainUse, owner Patch, sector)`. Components имеют независимый lifecycle,
  но вычисляются одной patch-level группой.
- **[SemanticAuthority: USER_REQUIRED]** Outer boundary, holes и только
  `ChainUse` с явной ролью `BARRIER` — неподвижные non-owner constraints.
  Обычный rail автоматически barrier не становится.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Front не пересекает boundary
  и не откатывается. На точном alpha возникает `BOUNDARY_CONTACT`; разрешены
  exact clip, сокращение/исчезновение active intervals и движение contact
  point вдоль того же boundary component.
- **[SemanticAuthority: USER_REQUIRED]** Число active branches одного
  FrontComponent в v1 не увеличивается. Необходимость split/path choice/merge
  для обхода obstacle создаёт capacity reason `BARRIER_SPLIT_REQUIRED`.
- **[SemanticAuthority: USER_REQUIRED]** Capacity clamp применяется только к
  конкретному FrontComponent: сохраняются `requested_alpha`, доказанный
  `effective_alpha` и capability outcome `BARRIER_BYPASS_UNSUPPORTED`.
  Остальные components продолжают рост.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Boundary не является owner,
  не имеет UV/station/material region и даёт только clip/contact/capacity
  lineage.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Completion определяется
  пустым или полностью frozen набором active intervals (`FRONT_EXHAUSTED`),
  не первой/последней вершиной front.
- **[SemanticAuthority: USER_REQUIRED]** Boolean `Envelope ∩ PatchDomain` без
  source reachability, component count и event history не является достаточной
  семантикой: material не может телепортироваться за hole/barrier.

### 3.4 Boundary lineage и provenance

- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Каждая граница хранит
  породившие примитивы/домен/событие и свою роль: coverage silhouette,
  ownership divider, UV interface или domain boundary.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Arrangement переносит
  lineage вместе с кривой. Реконструкция provenance после Boolean запрещена.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Face/region overlay хранит
  полный набор покрывающих примитивов, domain membership и interaction state;
  curve-to-edge lineage недостаточно.
- **[SemanticAuthority: FIELD_PROVEN]** Физический boundary-контур важнее
  производной `data-chain`: переход через единственное продолжение контура
  сохраняет тот же route и station ledger.

### 3.5 Owner и adjacency

- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Owner тотален на открытых
  регионах и disjoint. Граница равенства имеет `owner = NONE`; выбирать на ней
  smallest id или first-wins запрещено.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Coalescing разрешён только
  при совпадении owner, boundary lineage, UV/station model, surface membership
  и semantic kind. Геометрическая копланарность или angle epsilon сами по себе
  недостаточны.
- **[SemanticAuthority: USER_REQUIRED]** Junction matter принадлежит
  `JunctionEnvelope`, а не случайно первому incident strip. Разные station
  модели внутри junction разделяются явным UV/ownership interface.

### 3.6 Направление s и UV

- **[SemanticAuthority: USER_REQUIRED]** `s` задаётся semantic START/END или
  явно объявленной route pairing, а не порядком хранения chain.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Storage reversal при
  сохранённых semantic endpoints не меняет `s`; намеренная смена START/END —
  семантическое преобразование.
- **[SemanticAuthority: USER_REQUIRED]** Внутренний контракт нейтрален:
  продольная координата — `s`, поперечная — `r`. Публичная trim-развёртка
  отображает их один раз; sidecar не фиксирует численные UV.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** На физическом конце `s`
  cap имеет постоянную конечную station. На data-chain split station не
  сбрасывается и cap не рождается.

### 3.7 Alpha и topology events

- **[SemanticAuthority: USER_REQUIRED]** Reference evaluator полностью
  пересобирает состояние для заданного alpha и остаётся в тестах навсегда.
- **[SemanticAuthority: USER_REQUIRED]** Runtime drag может менять connectivity
  только на событиях, объявленных compile: `birth`, `death`, `collapse`,
  `split`, `boundary`, `merge`, `saturation`, `freeze`.
- **[SemanticAuthority: USER_REQUIRED]** В boundary-limited v1 `split/merge`
  не разрешают obstacle bypass. Необходимость такого split превращается в
  `BARRIER_SPLIT_REQUIRED` capacity event.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** События одной точной высоты
  применяются атомарным пакетом и не зависят от порядка перечисления.
- **[SemanticAuthority: USER_REQUIRED]** При `alpha == event_height` действует
  after-state. Сравнение `Compiled(alpha) == Reference(alpha)` обязательно
  непосредственно до, в и после события, а также на экстремальном alpha.
- **[SemanticAuthority: FIELD_PROVEN]** Конкуренционный эффект разрешён только
  после mutual arrival; до него каждый envelope растёт без mutual effect в
  общем owner `PatchDomain`.
- **[SemanticAuthority: USER_REQUIRED]** Front не имеет права молча исчезнуть:
  он продолжает рост, достигает именованного compile event либо возвращает
  named failure.

### 3.8 Exactness

- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Exact predicate либо один
  именованный arithmetic contract определяет инцидентность, порядок событий,
  topology и owner.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Tolerance может оценивать
  ошибку представления, но никогда не выбирает topology, owner, join или
  порядок событий.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Raster/SDF и sampled curve
  допустимы как диагностика, не как authority topology.

## 4. CanonicalGeometryDigest

`CanonicalGeometryDigest` сравнивает семантический результат, не сериализацию
runtime faces.

В digest входят:

- `case/schema version`, выбранные semantic policies и active alpha interval;
- PatchDomain identity/regime, sectors, holes, barriers и reachable components;
- PhysicalChain identity, directed ChainUses и seed-to-use-to-patch provenance;
- FrontComponent identity, sector, active-interval topology, branch count,
  boundary event history, requested/effective alpha и capacity reason;
- coverage region graph и covering primitive identities;
- boundary lineage и semantic anchors;
- owner каждого открытого региона и ownerless equality boundaries;
- adjacency и domain membership;
- station/UV model identities и ориентация `s`;
- topology events, их exact heights/порядок частичного порядка и
  interaction state;
- geometric curves в exact/algebraic форме либо по одному именованному
  arithmetic contract.

Не входят:

- порядок faces/regions в памяти;
- cyclic start polygon loop;
- triangulation и число materialized faces;
- номера data-chain records, если физическая semantic line та же;
- порядок перечисления PhysicalChain, ChainUse, seeds и envelopes;
- debug colors, overlay primitives и UI last-valid state;
- float arrays, округлённые только ради сравнения.

Канонизация сортирует записи по semantic identity и lineage, а не по runtime
id. Для scale/translation metamorphic test digest сравнивается после
нормализации преобразования. Для малого perturbation без topology change
структурная часть digest совпадает, а exact geometry сравнивается с применённым
преобразованием. Никакой tolerance не может сделать два разных owner-графа
равными.

## 5. Канонические случаи 1–7: базовые envelope и физическая линия

### Case 01 — Straight open strip (`DEFINED`)

Coverage — один двухсторонний `StripEnvelope` с двумя физическими caps.
Ownership: body + start/end cap regions, без перекрытий. `s` идёт от semantic
START к END. Alpha меняет ширину без изменения combinatorics после рождения.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c01-straight-open-strip.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c01-straight-open-strip.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c01-straight-open-strip.yaml)

### Case 02 — Convex MITER corner (`DEFINED`)

Corner matter доходит до единственного support-intersection apex. Силуэт
создаёт union двух strips и одного `CornerEnvelope`; owner-dividers остаются
внутри. Join — compile semantic input.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c02-convex-miter-corner.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c02-convex-miter-corner.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c02-convex-miter-corner.yaml)

### Case 03 — Convex BEVEL corner (`DEFINED`)

Семантическая материя угла — ровно `V/P1/P2`; внешний край — chord `P1–P2`.
Отсутствующий MITER-wedge не принадлежит BEVEL. Собственные strip-regions
совпадают с MITER.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c03-convex-bevel-corner.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c03-convex-bevel-corner.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c03-convex-bevel-corner.yaml)

### Case 04 — Reflex corner (`DEFINED`)

Reflex-стык не создаёт внешний клин. Coverage есть union incident strips;
внутренний interface не является второй материей и не выходит в silhouette.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c04-reflex-corner.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c04-reflex-corner.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c04-reflex-corner.yaml)

### Case 05 — Physical end of s (`DEFINED`)

Cap рождается только на физическом конце semantic line. Это срез самой ленты,
не overlay. На cap station постоянна и равна конечной `s`; продолжения за
физический конец нет.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c05-physical-s-endpoint.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c05-physical-s-endpoint.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c05-physical-s-endpoint.yaml)

### Case 06 — Domain boundary contour (`DEFINED`)

На точном контакте возникает boundary event. Active interval клиппируется и
может сокращаться, пока contact point однозначно скользит вдоль того же
physical boundary component; хорда через contour turn запрещена. Если turn,
hole или concavity требуют рождения второй branch, component насыщается с
`BARRIER_SPLIT_REQUIRED`. Противоположная patch-side и другие sources имеют
независимый lifecycle.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c06-domain-boundary-contour.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c06-domain-boundary-contour.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c06-domain-boundary-contour.yaml)

### Case 07 — Data-chain split/merge (`DEFINED`)

Разбиение одной физической линии на records не создаёт cap, owner boundary,
station reset или topology event. После нормализации sidecar/digest совпадает
с case 01.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c07-data-chain-segmentation.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c07-data-chain-segmentation.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c07-data-chain-segmentation.yaml)

## 6. Канонические случаи 8–10: junction

### Case 08 — T-junction (`DEFINED`)

Coverage — один connected union трёх strips и `JunctionEnvelope`, без дыры и
double cover. Junction core имеет junction-owner. Trunk `s` проходит насквозь;
branch `s` начинается в junction. Разные station models разделены внутренним
UV-interface, не силуэтом.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c08-t-junction.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c08-t-junction.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c08-t-junction.yaml)

### Case 09 — X-junction with declared pairing (`DEFINED`)

Две route identities объявлены входом и сохраняют `s` через junction.
Отсутствующая/неоднозначная pairing даёт named failure
`JUNCTION_ROUTE_PAIRING_REQUIRED`; порядок arms её не угадывает.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c09-x-junction.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c09-x-junction.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c09-x-junction.yaml)

### Case 10 — Y merge/split (`DEFINED`)

Два upstream channel проходят через один junction в один downstream route.
Upstream provenance сохраняется; merged route не дублируется обратными
чтениями. Merge — counted topology event.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c10-merge-split-junction.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c10-merge-split-junction.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c10-merge-split-junction.yaml)

## 7. Канонические случаи 11–13: ownership и collision

### Case 11 — Corner-owner on a short segment (`DEFINED`)

Claims двух endpoint corners сначала клиппируются физическим divider. В
симметричном однородном случае это midpoint cross-section. На событии collapse
body исчезает, но coverage остаётся single-cover; equality boundary ownerless.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c11-short-segment-corner-ownership.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c11-short-segment-corner-ownership.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c11-short-segment-corner-ownership.yaml)

### Case 12 — Curved internal divider (`DEFINED`)

Точная кривая равенства может быть ownership/UV divider внутри coverage.
Coverage остаётся union envelopes; кривая не может стать частью silhouette.
Lineage хранит оба claims. Sampled polyline и tolerance не выбирают owner.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c12-curved-internal-divider.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c12-curved-internal-divider.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c12-curved-internal-divider.yaml)

### Case 13 — BEVEL under another-wing collision (`DEFINED`)

До mutual arrival front другой юбки той же decal и того же Patch не влияет на
BEVEL. После contact эта юбка заполняет освобождённый клин и доходит до
фактической chord `P1–P2`, не обтекая несуществующий MITER apex. Incident
strips BEVEL-угла не меняются. Collision с другой decal или другим Patch
запрещён.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c13-bevel-foreign-collision.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c13-bevel-foreign-collision.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c13-bevel-foreign-collision.yaml)

## 8. Канонические случаи 14–15: capacity и drag

### Case 14 — Proven saturation (`DEFINED`)

`SATURATE_PROVEN` фиксирует последний конструктивный state конкретного
FrontComponent и создаёт counted event. Для boundary-limited v1 сохраняются
`requested_alpha`, `effective_alpha` и `BARRIER_SPLIT_REQUIRED`; геометрия
этого component не меняется, но другие components продолжают. Если уникальный
maximal single-cover state не доказан, действуют `CONTROLLED_RECOMPILE` вне raw
frame либо `REJECT_UNPROVEN`.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c14-proven-saturation.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c14-proven-saturation.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c14-proven-saturation.yaml)

### Case 15 — Topology changes during drag (`DEFINED`)

Connectivity меняется на заранее объявленных событиях. В одном alpha могут
атомарно сработать boundary-contact + interval-collapse либо несколько
contact/collapse/freeze events. Ровно на event height действует after-state.
Obstacle-driven split/merge branches в v1 не исполняются: вместо них возникает
`BARRIER_SPLIT_REQUIRED`. Runtime discovery новых routes, supports или event
ordering запрещён.

- [PNG](../artifacts/envelope_ec0/sheets/ec0-c15-drag-topology-events.png) ·
  [SVG](../artifacts/envelope_ec0/sheets/ec0-c15-drag-topology-events.svg) ·
  [YAML](../artifacts/envelope_ec0/cases/ec0-c15-drag-topology-events.yaml)

### EC0-P boundary pack — `BOUNDARY_LIMITED_PROPAGATION`

Десять coordinate-free scenarios фиксируют straight/oblique contact,
exhaustion, первое касание hole, hole/concavity split capacity, simultaneous
contacts, barrier rail, независимость двух patch-side uses и другой pChain.
Каждый сценарий имеет before, exact after-state и requested-alpha-beyond state.
Obstacle bypass, branch birth и merge за obstacle отложены в EC8+.

- [PNG](../artifacts/envelope_ec0/pivot/ec0-p-boundary-limited-propagation.png) ·
  [SVG](../artifacts/envelope_ec0/pivot/ec0-p-boundary-limited-propagation.svg) ·
  [Policy YAML](../artifacts/envelope_ec0/pivot/boundary_limited_propagation.yaml) ·
  [Metamorphic YAML](../artifacts/envelope_ec0/pivot/boundary_metamorphic_matrix.yaml)

## 9. Case 16 — столкновение юбок одной decal внутри Patch (`DEFINED — B`)

Нормативный scope:

- **[SemanticAuthority: USER_REQUIRED]** Все участвующие юбки принадлежат одной
  decal и одному owner Patch, внутри которого идёт offset. Collision разных
  decal и collision через границу Patch запрещены.
- **[SemanticAuthority: USER_REQUIRED]** Участники — offset-крылья/юбки от
  pChains. Канон покрывает как встречу юбок разных pChains, так и
  самостолкновение одной юбки двумя её front/readings.
- **[SemanticAuthority: FIELD_PROVEN]** До mutual arrival взаимодействия нет.
- **[SemanticAuthority: FIELD_PROVEN]** После встречи freeze locus не ползёт
  при дальнейшем alpha, пока не случилось другое объявленное событие.
- **[SemanticAuthority: MATHEMATICALLY_REQUIRED]** Итоговые station claims
  disjoint; overlap faces и z-fighting запрещены. Decal owner и Patch owner
  при этом не меняются.
- **[SemanticAuthority: USER_REQUIRED]** Freeze изменяет геометрию coverage:
  выбран вариант B.
- **[SemanticAuthority: USER_REQUIRED]** B применяется к уже
  boundary-resolved contributions. Capacity одного FrontComponent не
  останавливает его конкурента, другую pChain или противоположный patch-side
  use; interaction учитывает фактически достигнутый `effective_alpha`.

### A — ownership only (`REJECTED_BY_USER`)

Primitive wing union оставался бы coverage, а equality locus только делил бы
overlap между station/UV claims. Это отвергнуто: collision обязан остановить и
обрезать геометрическую материю юбок, а не сохранить overlap под ownership.

- [PNG A](../artifacts/envelope_ec0/sheets/ec0-c16a-ownership-only.png) ·
  [SVG A](../artifacts/envelope_ec0/sheets/ec0-c16a-ownership-only.svg)

### B — wing coverage clip (`SELECTED_BY_USER`)

После mutual arrival покрытия участвующих юбок одной decal обрезаются на mutual
equality locus; ghost-контур на листе показывает материю до interaction. Это
явная стадия `IntraPatchFrontInteractionResolver`. При самостолкновении той же
юбки два её front/readings применяют тот же clip contract и сохраняют обе
station/UV readings по разные стороны locus.

- [PNG B](../artifacts/envelope_ec0/sheets/ec0-c16b-coverage-clip.png) ·
  [SVG B](../artifacts/envelope_ec0/sheets/ec0-c16b-coverage-clip.svg)

### C — new matter operator (`REJECTED_BY_USER`)

Collision порождал бы третий interaction-region с собственной semantic
identity, owner и UV/station model. Это отвергнуто: freeze не создаёт новую
материю.

- [PNG C](../artifacts/envelope_ec0/sheets/ec0-c16c-new-matter.png) ·
  [SVG C](../artifacts/envelope_ec0/sheets/ec0-c16c-new-matter.svg)

[YAML sidecar](../artifacts/envelope_ec0/cases/ec0-c16-intrapatch-wing-freeze.yaml)
фиксирует B как канонический region graph и A/C как отвергнутые альтернативы.
Выбор B и одно-decal/одно-Patch scope имеют `USER_REQUIRED` authority.

## 10. AM3 metamorphic contract

Полная бинарная матрица по всем 16 случаям находится в
`artifacts/envelope_ec0/metamorphic_matrix.yaml`. Базовые преобразования,
которые обязаны быть `INVARIANT`:

- перестановка input chains;
- reverse storage chain при сохранённых semantic endpoints;
- reverse winding/normals как storage orientation;
- rigid translation;
- uniform scale при совместном масштабе alpha/stations/lift;
- другая triangulation того же физического surface;
- split/merge data-chains на той же физической линии;
- перестановка одновременных событий внутри одного atomic batch;
- малое возмущение без изменения incidence, ordering, owner и event equalities.

Явные `SEMANTIC_CHANGE`-преобразования:

- смена semantic START/END меняет ориентацию `s`;
- `MITER ↔ BEVEL` меняет corner matter в cases 02, 03, 11, 13;
- смена `CapacityPolicy` меняет cases 14–15;
- смена выбранной intra-Patch wing policy B на отвергнутую A или C изменила бы
  case 16 семантически.

Отдельная AM8-матрица находится в
`artifacts/envelope_ec0/pivot/boundary_metamorphic_matrix.yaml`. Resampling
runtime front vertices, resegmentation одного physical boundary contour,
storage reverse и перестановка simultaneous contacts — `INVARIANT`. Toggle
роли `BARRIER`, включение obstacle bypass и превращение local capacity в
global stop — `SEMANTIC_CHANGE` только для затронутых scenarios.

## 11. Явный список «на пересмотр», не семантика

Следующие элементы не перенесены в канон:

- **[SemanticAuthority: LEGACY_COMPATIBILITY]** smallest canonical
  `chain_ref` owns exact tie. Конфликтует с общим запретом first-wins/
  smallest-id owner; в EC0 equality boundary имеет `owner = NONE`.
- **[SemanticAuthority: LEGACY_COMPATIBILITY]** Обязательность именно
  station-distance как механизма intra-Patch wing freeze. Перенесён
  наблюдаемый стабильный результат и mutual-arrival gate; representation
  механизма выбирается EC2.5.
- **[SemanticAuthority: IMPLEMENTATION_ACCIDENT]** Конец `BoundaryChain` как
  конец физического contour. Отвергнуто cases 05–07.
- **[SemanticAuthority: IMPLEMENTATION_ACCIDENT]** Cap/terminal bridge как
  дополнительные faces поверх strip. Отвергнуто single-cover контрактом.
- **[SemanticAuthority: IMPLEMENTATION_ACCIDENT]** FAN/point-site arc как
  collision boundary BEVEL. Отвергнуто case 13.
- **[SemanticAuthority: LEGACY_COMPATIBILITY]** Конкретные имена внутренних
  runtime типов (`CornerSeed`, `ResolvedCornerView`, rail reading) и старых
  ошибок. EC0 фиксирует semantic records и named-failure classes, не Python API.
- **[SemanticAuthority: IMPLEMENTATION_ACCIDENT]** Face count, polygon loop
  start, округлённые float arrays и triangulation как geometry digest.
- **[SemanticAuthority: LEGACY_COMPATIBILITY]** UI last-valid frame и
  retained-preview состояние. Это host behavior, не kernel geometry.
- **[SemanticAuthority: IMPLEMENTATION_ACCIDENT]** Любой epsilon, angle или
  id-order, который выбирает owner/topology/join/event order.

Ни один пункт этого списка не разрешено «временно» вносить в kernel под видом
совместимости. Для возвращения нужен отдельный user/reviewer decision с новым
`SemanticAuthority`.

## 12. Закрытие обязательного списка EC0

| Вопрос | Решение в candidate | Case |
|---|---|---|
| corner-owner | физический divider; ownerless tie; body collapse — event | 11 |
| T-junction | JunctionEnvelope core; trunk-through + branch-origin station sectors | 08 |
| конец s | только physical endpoint; constant cap station; no storage cap | 05/07 |
| BEVEL при столкновении с другой юбкой | та же decal/Patch; mutual-arrival; front доходит до chord | 13 |
| насыщение | `SATURATE_PROVEN`; frozen maximal single-cover; иначе named policy | 14 |
| кривой внутренний divider | exact internal-only ownership/UV curve | 12 |
| topology-изменения при drag | compile-declared, exact, atomic after-state | 15 |
| AM4 freeze | **B: intra-Patch wing coverage clip; A/C отвергнуты** | 16 |
| AM8 boundary/holes/barriers | clip + interval shrink; no branch growth; `BARRIER_SPLIT_REQUIRED` capacity | EC0-P boundary pack |

Здесь нет безымянного «решим потом» и нет blocked items. Вопрос AM4 закрыт
выбором B; разные decal и разные Patch исключены из collision scope.

## 13. Именованные unsupported outcomes

- `JUNCTION_ROUTE_PAIRING_REQUIRED` — X-junction без единственной semantic
  pairing;
- `CONTOUR_CONTINUATION_AMBIGUOUS` — physical contour имеет несколько
  продолжений без semantic mark;
- `CAPACITY_UNPROVEN` — maximal single-cover для saturation не доказан;
- `EXACT_DIVIDER_UNAVAILABLE` — backend не способен сохранить exact curved
  divider и lineage; sampled replacement запрещён;
- `EVENT_ORDER_UNRESOLVED` — simultaneous event batch не имеет единственного
  semantic after-state;
- `CROSS_DECAL_WING_INTERACTION_FORBIDDEN` — collision-system получил юбки
  разных decal;
- `CROSS_PATCH_WING_INTERACTION_FORBIDDEN` — collision-system получил юбки
  из разных owner Patch;
- `BARRIER_BYPASS_UNSUPPORTED` — физически возможное продолжение требует
  obstacle split/path choice/merge, отсутствующих в v1;
- `BOUNDARY_REACHABILITY_UNPROVEN` — Boolean clip не доказывает допустимый
  source path до material region;
- `FRONT_COMPONENT_STATE_INCOMPLETE` — нет active intervals, branch count,
  event history или requested/effective alpha для capacity decision.

Это честные failures, а не fallback в старую геометрию.

## 14. Риски

1. T/X/Y junction station sectors определены семантически, но EC2 должен
   доказать, что minimal `JunctionEnvelope` выражает их без скрытого
   first-wins. Провал — named stop, не изменение картинок задним числом.
2. Exact curved divider потребует arrangement backend с сохранением curve
   history и face claims. Polyline sampling не является запасным каноном.
3. Case 15 требует строгой atomic semantics для simultaneous events; обычная
   сортировка по runtime id не подходит.
4. Case 16 требует двух независимых EC0-P/EC1 fixtures: встреча юбок разных pChains
   одной decal и самостолкновение одной юбки. Оба должны дать B-clip с одним
   decal/Patch owner и двумя сохранёнными station readings.
5. Эти листы топологические, а не метрические. EC1 fixtures обязаны добавить
   exact anchors/curves по именованному arithmetic contract, не извлекать
   координаты из PNG.
6. V1-листы не доказывают AM7 unit of execution: на них недостаточно явно
   отделены PhysicalChain, patch-side ChainUses и seeds. Использовать их как
   единственный EC1 input запрещено до EC0-P rebaseline.
7. Host snapshot может не иметь стабильной пары uses или sector ordering для
   `SEAM_SELF`. Kernel не восстанавливает эти facts эвристически; это named
   contract failure EC1.
8. Boolean clip без reachability ledger может незаметно телепортировать
   coverage за hole/barrier. EC2 обязан сравнивать не только silhouette, но и
   component/event provenance.
9. Per-component effective alpha усложняет digest и UI: requested alpha общий,
   но saturation одного component не должна глобально заморозить Patch.

## 15. Особое мнение kernel-implementer

Пользовательский выбор **B** подтверждает отдельную coverage-стадию: freeze не
маскируется под обычный ownership и не вводит новую третью материю. После
уточнения scope считаю имя `IntraPatchFrontInteractionResolver` существенно
точнее roadmap-термина: оно не допускает ошибочного прочтения «разные decal».

Для junction считаю принципиальным отдельный semantic owner core/sector:
передача core «первому arm» снова сделает порядок данных скрытой политикой.
Для физического конца `s` constant-station cap существенно проще и чище, чем
любой extrapolation: он сохраняет trim phase и однозначно отличает физический
endpoint от data segmentation.

Для нового pivot считаю полезным явный `PatchEvaluationPlan`: один domain,
все uses/seeds Patch и таблица source contributions. Это особое мнение о форме
IR, но не о семантике; нормативен patch-level unit of execution.

Для AM8 считаю правильным хранить active intervals/capacity внутри
FrontComponent, но вызывать boundary resolver patch-level batch-операцией.
Так сохраняется независимый lifecycle без возврата к per-pChain solve.

## 16. EC0 acceptance checklist

- [ ] Пользователь просмотрел cases 01–15 и принимает silhouette, owner,
  adjacency, `s` и alpha evolution.
- [x] Пользователь выбрал B для case 16; A и C помечены как rejected.
- [x] Collision scope case 16 ограничен юбками одной decal и одного Patch;
  self-collision одной юбки включён.
- [ ] Список §11 принят как несемантический и не переносится в kernel.
- [x] Все девять вопросов §12 имеют финальное решение; blocked items отсутствуют.
- [x] После выбора обновлены status/selected alternative в case 16 sidecar и
  `CanonicalGeometryDigest` projection.
- [ ] Все cases перевыпущены как EC0-P: `PatchDomain`, PhysicalChain,
  directed ChainUse и seeds показаны отдельно от coverage/ownership.
- [ ] Добавлены pivot-cases для seam A/B, `SEAM_SELF`, нескольких sources в
  одном PatchDomain и self-collision.
- [ ] Принят EC0-P boundary pack из десяти AM8-сценариев: contact, sliding,
  exhaustion, hole/split, concavity, simultaneous contact, barrier rail и
  независимость других components.
- [ ] Sidecars мигрированы на v2 и доказывают один patch-level unit of
  execution без private per-pChain domain.
- [ ] Пользователь принял EC0-P visual corpus.

До выполнения EC0-P пунктов чек-листа **EC1 не начинается**.
