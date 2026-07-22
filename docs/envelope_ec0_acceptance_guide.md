# EC0 Session A: приёмка человеческими словами

Статус пакета: `A1_A2_CORRECTION_CANDIDATE_BLOCKED_PENDING_USER_DECISION`.

Это словесная версия Session A без geometry evaluator, Blender integration и
выбора Boolean backend. Она помогает принять продуктовый смысл JSON corpus v5.
Презентационные картинки не создаются; Blender screenshots допустимы только
как будущие runtime-доказательства.

Сейчас техническое направление A1/A2 принято, но EC0 ещё нельзя закрыть:
пользователь не задал значение `LINEAR_REFLEX_MAX_SUBTURN_V1`. Агент не имеет
права придумать число или спрятать default в коде/JSON. После этого выбора
нужны зелёный локальный validator, реальный внешний GitHub workflow и повторная
явная приёмка.

## Главная модель

Для одной decal request внутри одного PatchDomain все активные юбки считаются
вместе:

```text
host facts + request
→ seeds/fronts/specs
→ alpha-bound instances
→ boundary-limited resolution
→ single-cover union
→ разрешённые встречи юбок
→ ownership
→ UV/station
→ semantic coalescing
→ downstream tessellation
→ GeometryBatch
```

Разные decals не сталкиваются. Разные PatchDomains не сталкиваются. Встречаться
могут только contributions одной decal request в одном Patch; self-contact
одной юбки подчиняется тому же policy B.

## Девять понятий

1. **PatchDomain** — единственное поле распространения.
2. **PhysicalChain** — физическая identity линии.
3. **ChainUse** — направленное использование PhysicalChain со стороны одного
   Patch.
4. **FrontComponent** — одно owner-interior крыло обычного ChainUse.
5. **Analysis relation** — Corner/Junction/Terminal fact без геометрии.
6. **EnvelopeSpec** — неизменный аналитический закон одного из четырёх variants.
7. **EnvelopeInstance** — применение spec к effective alpha.
8. **ResolvedCoverage** — итоговая single-cover материя после разрешённых
   interactions.
9. **Ownership** — полное непересекающееся разбиение этой материи; оно не меняет
   силуэт.

## Один AngularProfileId вместо K0/K1 profiles

Для всех обычных reflex corners используется одна семья:

```text
AngularProfileId = LINEAR_REFLEX_EQUAL_V1
```

Analysis обязан передать ориентированный owner-sector:

- `owner_sector_id`;
- ordered incoming/outgoing supports;
- turn orientation относительно owner Patch;
- exact или certified oriented angle `φ`;
- доказательство `π < φ < 2π`.

Reflex excess равен `Δ = φ - π`. Request выбирает policy
`MIN_K_FOR_MAX_SUBTURN_V1` и parameter
`LINEAR_REFLEX_MAX_SUBTURN_V1 = Δ_MAX`. Compiled plan вычисляет:

```text
k = max(0, ceil(Δ / Δ_MAX) - 1)
```

Иными словами, это минимальное `k >= 0`, при котором каждый равный subturn
`Δ / (k + 1)` не превышает `Δ_MAX`.

Coordinate-free fixture хранит certified interval
`k < Δ / Δ_MAX <= k + 1`. Validator сам выводит из него `k` и только затем
сверяет `resolved_hidden_edge_count`, seed, hidden supports и profile
cardinality. Отдельная case-to-k таблица в этой проверке не участвует.

`k` нельзя выбирать отдельной case table, ручным K0/K1 preset, количеством
faces, sampling quality, epsilon или результатом Boolean. C02 и C03 сохраняют
labels `LINEAR_REFLEX_K0_EQUAL_FIXTURE_V1` и
`LINEAR_REFLEX_K1_EQUAL_FIXTURE_V1` только как regression results: policy
certificate в них дал соответственно `k = 0` и `k = 1`.

## Что делает hidden support

При вычисленном `k` скрытая support с номером `j`, `1 <= j <= k`, ставится на
долю поворота:

```text
j / (k + 1)
```

Поворот идёт от ordered incoming support к ordered outgoing support внутри
owner-sector. Hidden support локальна своему AngularEnvelopeSpec и не является
pChain, ChainUse, seam, barrier или новой host topology.

До clip/union/interaction профиль имеет `k + 2` active supports и `k + 2`
локальных profile segments. Это не число обязательных видимых рёбер. После
union часть segments может оказаться внутри общей материи; exposed silhouette
определяется только resolved coverage.

## Что ещё должен выбрать пользователь

Формула и policy уже зафиксированы. Не зафиксировано только продуктовое
значение `LINEAR_REFLEX_MAX_SUBTURN_V1` — максимально допустимый subturn.

Это намеренный `BLOCKED_PENDING_USER_DECISION`, а не просьба агенту подобрать
«разумное» число. Выбор можно оформить числовым значением или именованным
product preset с однозначным значением и единицами. После выбора corpus должен
быть обновлён и повторно проверен.

## Четыре EnvelopeSpec variants

### StripEnvelopeSpec

Обычная юбка ChainUse. Source support `n·x = c`, moving support
`n·x = c + alpha`; `n` направлена в owner interior, normal speed равна единице.
Station `s` идёт вдоль semantic ChainUse, signed `r` — поперёк. Data-record
split не создаёт cap и не сбрасывает `s`.

### AngularEnvelopeSpec

Связывает только две ordered incident юбки подтверждённой CornerRelation.
Содержит owner-sector, angle certificate, единый profile ID, policy selection
certificate, computed `k` и hidden-support lineage. Случайная третья юбка не
входит в CornerSeed/AngularEnvelopeSpec; она участвует позже как interaction
participant.

### JunctionEnvelopeSpec

Описывает подтверждённый T/X/Y или cross-Patch junction. Route pairing и shared
anchor приходят из relation, а не выбираются arm order или lowest id.
Cross-Patch projections координируются, но propagation collision между Patch
по-прежнему запрещён.

### CapEnvelopeSpec

Закрывает только физический конец strip route. Cap не появляется на конце
storage record и не создаёт overlay matter. Exact `2π` terminal — Cap/Junction,
а не обычный AngularEnvelopeSpec.

## Ownership и tessellation

Сначала вычисляется coverage, потом policy B, только потом ownership. Claims
должны образовывать total/disjoint partition; equality boundary остаётся
ownerless. First-wins/lowest-id и восстановление owner из Boolean face order
запрещены.

После semantic coalescing разрешено менять internal fill diagonals, triangles,
polygons и face order. Нельзя менять silhouette, exposed profile-controlled
segments, ownership/UV/station dividers, provenance и shared semantic vertices.
Все допустимые tessellations одного arrangement имеют один
CanonicalGeometryDigest.

## События при drag

Законы, predicates и transition types известны заранее; successor event
instances разрешено планировать лениво. Все события одного exact alpha
применяются атомарно. Наружу публикуется только complete state. Если exact state
ещё не готов, остаётся последняя полная версия и возвращается
`PENDING_EXACT_EVALUATION`.

## 16 основных cases

1. **C01:** один ChainUse → один strip/front; caps только на physical terminals.
2. **C02:** единый profile, certificate/policy дают `k = 0` (K0 fixture result).
3. **C03:** единый profile, certificate/policy дают `k = 1`; hidden support на
   `1/2` excess (K1 fixture result).
4. **C04:** локальные Angular segments могут стать внутренними после union;
   spec identity сохраняется, внешний клин не добавляется.
5. **C05:** конец `s` — только physical route end; cap на record end запрещён.
6. **C06:** endpoint contact может slide/shrink; interior split возвращает
   `BARRIER_SPLIT_REQUIRED` в exact alpha.
7. **C07:** segmentation records не меняет physical route, spec и digest.
8. **C08:** T-junction получает один JunctionEnvelopeSpec; arm order не выбирает
   owner.
9. **C09:** X route pairing приходит из JunctionRelation.
10. **C10:** Y merge/split хранит полный upstream provenance и single cover.
11. **C11:** два EndpointClaimSeeds делят short strip; фиктивного Angular spec
    нет.
12. **C12:** claim B ссылается на две strips и один angle-driven Angular spec;
    curved divider меняет только ownership/UV, не silhouette.
13. **C13:** Angular spec содержит только свои ordered incident uses. Третья
    юбка той же decal/Patch встречает фактическую exposed profile boundary лишь
    после mutual arrival.
14. **C14:** насыщается только доказанно ограниченный component; companion
    продолжает до своего effective alpha.
15. **C15:** topology transitions exact/atomic; новые semantic rules во время
    drag не изобретаются.
16. **C16:** policy B — same-request same-Patch contributions клиппируются по
    equality locus; overlap и новая третья matter отсутствуют.

## Семь pivot checks

- **P01:** физический seam A/B даёт два Patch plans и два ChainUses.
- **P02:** `SEAM_SELF` сохраняет два distinct ChainUses одного Patch.
- **P03:** все sources request/Patch входят в один plan/union.
- **P04:** self-collision одной юбки использует policy B.
- **P05:** cross-Patch junction делит anchor/provenance, но не collision field.
- **P06:** angle-driven Angular spec получает vector incident effective alpha.
- **P07:** Junction spec получает vector всех incident effective alpha.

Неединственный mixed-alpha result возвращает
`SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN`; materializer не дорисовывает щель.

## Metamorphic expectations

Инвариантны: record permutation, storage reverse при сохранении semantic
orientation, coherent winding reverse, rigid transform, coherent scale,
retriangulation, data-record split/merge, same-alpha event permutation,
oriented-sector record permutation и downstream tessellation при сохранении
arrangement.

Семантику меняют: incoherent incident-support swap, angle или `Δ_MAX`, если
изменился computed `k`, ручной override `k`, смена subdivision law и удаление
фактического exposed profile-controlled segment.

## На пересмотр — не семантика

- `[LEGACY_COMPATIBILITY]` UI MITER/BEVEL/ROUND labels как request presets;
- `[LEGACY_COMPATIBILITY]` legacy face count, triangulation и лишние numeric UV
  conventions;
- `[IMPLEMENTATION_ACCIDENT]` lowest-id owner, record-end cap, auto left/right
  fronts, overlay bridge/cap, per-pChain materialization/post-hoc stitching;
- `[IMPLEMENTATION_ACCIDENT]` sampled/repair geometry как Angular authority;
- `[OPEN_RESEARCH]` production Boolean/arrangement library;
- `[OPEN_RESEARCH]` obstacle bypass со split/path choice/merge;
- `[OPEN_RESEARCH]` generalized straight-skeleton backend.

Ни один пункт не разрешает silent fallback.

## Риски

- `Δ_MAX` заметно влияет на форму через computed `k`; это продуктовый выбор.
- Symbolic regression certificates проверяют закон, но не заменяют production
  default.
- Exact curved divider/Junction требуют backend с curve/provenance support.
- Host adapter обязан стабильно передать ChainUse, oriented sectors и angle
  certificates; kernel не должен реконструировать их эвристически.
- Runtime UI должен различать requested alpha, effective alpha и pending state.

## Особое мнение

Один `LINEAR_REFLEX_EQUAL_V1` с angle-driven `k` лучше фиксированной таблицы
K0/K1: все corners подчиняются одному закону, а рост локальной детализации
объясняется измеренным сектором и явно утверждённым продуктовым пределом. Но
этот предел нельзя выбирать исполнителю под видом технической константы.

## Как продолжить приёмку

Сначала пользователь должен сообщить значение
`LINEAR_REFLEX_MAX_SUBTURN_V1` (с единицами либо через однозначный named preset).
После обновления corpus, зелёного validator и реального GitHub workflow будет
предложена короткая финальная фраза приёмки Session A. До этого EC1 закрыт.
