# Envelope-бэкенд: КОМПРОМИССНАЯ дорожная карта + PatchDomain pivot
## Сведение планов двух ревьюверов, обновлённое AM7

Исходники: `decal_envelope_roadmap_claude.md` (ревьювер A), план
ревьювера B (в отчётах пользователя) и последующее инженерное ревью
PatchDomain/ChainUse, принятое пользователем как pivot. Согласие исходных
планов ~90%; ниже — пять решений, AM1–AM8 и обновлённые фазы EC0-P–EC8+.

**Статус после pivot:** AM7–AM8 нормативно уточняют AM4 и ранние EC-фазы.
`Patch` — единственный domain распространения; pChain — source, не domain.
EC1 не начинается до повторного EC0-P гейта.

---

## Пять решений

### 1. Граница проекта — закрыто пользователем

Оба ревьювера отвергли полный CFTUV+-rewrite по одинаковым
причинам. Расхождение: отдельный repo (B) против пакета в текущем
repo со стеной импортов (A).

**Компромисс-предложение A (рекомендация к утверждению):
моно-repo, ДВА пакета, изоляция доказывается ИСПОЛНЕНИЕМ:**
```
CFTUV repo
├── cftuv/                  # host: analysis, Blender, adapters
└── kernel/                 # cftuv_envelope_core: Blender-free
    ├── pyproject.toml      # самостоятельный пакет
    ├── src/cftuv_envelope/ # структура по плану B (contracts/
    │                       # ingest/skeleton/envelopes/
    │                       # arrangement/ownership/coalescing/
    │                       # compile/geometry_batch/diagnostics)
    ├── fixtures/           # AnalysisSnapshot JSON (см. п.2)
    └── tests/              # CI гоняет их В ОКРУЖЕНИИ БЕЗ
                            # bpy/mathutils/cftuv — изоляция
                            # доказана запуском, не соглашением
```
Это даёт ВСЁ техническое из плана B (Blender-free ядро, one-way
адаптер, A/B-сравнение, невозможность импорта старых helpers) без
издержек двух репозиториев (двойной CI, версионирование
контрактов, жонглирование ветками для одного пользователя и
агентов). Вынести kernel в отдельный repo можно В ЛЮБОЙ момент
позже — он уже изолирован; обратное слияние дороже.
**РЕШЕНИЕ ПОЛЬЗОВАТЕЛЯ: моно-repo + два пакета — УТВЕРЖДЕНО.**

**Онбординг kernel-задач (защита от «агент тонет в старом коде /
переиспользует плохие паттерны» — страх пользователя, признан
обоснованным):**
- kernel-агент читает ТОЛЬКО: этот компромисс,
  `envelope_backend_semantics.md` (EC0), инварианты брифа (§3),
  каталог болезней (§4), wavefront-заметку, контракты
  surface_ir/GeometryBatch и канон-семантику оракула
  (decal_rails.md — правила, не код), а также обязательные
  `envelope_kernel_pivot_instructions.md`;
- **чтение КОДА `decal_voronoi.py`/legacy-частей `decals.py` для
  kernel-задач ЗАПРЕЩЕНО** (плохие паттерны не переиспользуются,
  потому что не читаются); исключение — автор host-адаптера
  (отдельная host-задача);
- техническая стена: kernel-тесты в CI выполняются в окружении
  БЕЗ bpy/mathutils/cftuv — импорт старого падает по построению;
- выстраданные ПОВЕДЕНИЯ старого пути (именованные отказы,
  насыщение, обоюдное прибытие, freeze-семантика) попадают в
  ядро как СПЕЦИФИКАЦИИ через EC0-семантику, не как код.

### 2. Coverage-примитивы — СОГЛАСОВАНО (планы идентичны)
`StripEnvelope | CornerEnvelope | JunctionEnvelope | CapEnvelope`
— tagged union, не один класс с optional-полями. Каждый несёт:
semantic identity, provenance, закон от alpha, boundary curves,
station mapping, owner patch, домен. Тесселяция — downstream;
число runtime-граней не входит в определение примитива.

### 3. Coverage, interaction и ownership — СОГЛАСОВАНО после pivot
Все pChain sources одного `PatchDomain` вычисляются совместно. Сначала
строится patch-level union вкладов, затем применяются явно объявленные
interaction rules, и только после этого ownership делит уже существующую
материю между source/station claims. Термины «внутрикомпонентный» и
«межкомпонентный» больше не обозначают разные decal или отдельные поля:
это отношения вкладов/fronts одной decal внутри одного Patch.

Турнир O1–O4 остаётся главным исследовательским этапом ownership. O1
покрывает chain-local corner/cap/junction claims; O2 — nonlocal planar
claims; O3 — оракул; O4 — station claims после intra-Patch interaction.
Гейт-критерии сохраняются: тотальность, disjoint, детерминизм,
топологическая объяснимость, сохранение `s`, отсутствие скрытых приоритетов.
Требование «coverage неизменна» относится только к ownership-стадии;
выбранный interaction B законно меняет coverage раньше неё. Если policy не
найдена — модель не готова, независимо от качества coverage.

### 4. Arrangement-ядро — СОГЛАСОВАНО
Референс/оракул: CGAL Arrangement_with_history (DCEL с
пользовательскими данными — provenance проходит СКВОЗЬ
arrangement вместе с кривой; запрет «реконструкции provenance
после Boolean» — общий у обоих планов). Production-кандидат —
выход EC2 (Clipper2 отмечен B как вероятно недостаточный для
полной provenance — ownership-overlay пришлось бы строить
отдельно; собственный минимальный exact-arrangement в kernel —
альтернатива). Решение по данным, не заранее.

### 5. Модель компиляции — СОГЛАСОВАНО
Сначала reference-evaluator (полная пересборка на каждом alpha),
потом event-schedule compile (EC5). **Правило плана B принимается
навечно: reference остаётся в тестах навсегда;
Compiled(alpha) == Reference(alpha) на ширинах до/в/после каждого
события и на экстремальной.** Полный kinetic skeleton solver —
только если event-schedule окажется недостаточным (понижен до
запасного варианта обоими планами).

---

## Поправка таймлайна (план B писался против устаревшего среза)

S-CM.a/b, R2 и R2.1 УЖЕ реализованы и полево-зелёные в старом
бэкенде. Следствия: (а) старый бэкенд заморожен в maintenance
mode (уже объявлено) — только именованные fail-хотфиксы;
(б) семантика CornerSeed/CornerModel, station freeze, mutual
arrival, CapacityPolicy переносится в kernel как СЕМАНТИЧЕСКИЕ
спецификации (входы EC0), не как код; (в) соответствие плана B
«CornerModel -> CornerSeed + CornerEnvelope» совпадает с
фактической фазировкой S-CM — конфликт отсутствует.

## Контракт снапшота — ОДИН, не два

`AnalysisSnapshot` = ВЕРСИОНИРОВАННАЯ СЕРИАЛИЗАЦИЯ существующего
`AnalysisBundle`, произведённая В ОДНОМ месте host-адаптера. Второй
самостоятельный контракт анализа запрещён (болезнь Б1 на уровне
контрактов). JSON на research-фазе читается глазами и диффается в Git.

Snapshot обязан различать четыре уровня:

1. source facts: `PatchDescriptor`, `PhysicalChain`, `ChainUse`,
   `BoundaryLoop`, `SourceVertex`, Corner/Junction relations;
2. domain: `PatchDomain`, `PatchSector`, `DomainBoundary`, `Hole`, `Barrier`;
3. seeds: `FrontSeed`, `CornerSeed`, `JunctionSeed`, `CapSeed`;
4. material inputs: tagged envelope declarations.

`PhysicalChain` хранит физическую identity рёбер/вершин и канонический
физический порядок. `ChainUse` хранит directed patch-side view:
`physical_chain_id`, `owner_patch_id`, `boundary_loop_id`, orientation,
side и множество roles (`SELECTED_SPINE`, `DOMAIN_BOUNDARY`, `BARRIER`,
`FOLD_GUIDE`, `CHART_CUT`, `MARKED_ROUTE`). Одна physical seam между двумя
Patch даёт два ChainUse; `SEAM_SELF` даёт два разных ChainUse одного Patch.
Преждевременно сливать uses запрещено.

`PatchDescriptor` имеет ортогональные axes `SurfaceRegime` /
`PatchShapeClass` / `context_tags`; WALL/FLOOR — контекст, не выбор
бэкенда. Подклассы Patch/pChain отвергнуты; полиморфизм остаётся только в
операциях (`SurfaceMetric`, evaluator strategies).

---

## Объединённые фазы EC0-P–EC8

**EC0-P — Pivot rebaseline, БЕЗ КОДА.** Существующие решения 16 случаев,
включая выбранный пользователем B, сохраняются, но визуалы и sidecars
перевыпускаются на правильных атомарных уровнях. Каждый лист обязан отдельно
показывать `PatchDomain`, physical chain identity, directed `ChainUse`,
compile-static seeds, patch-level coverage union, interaction boundaries,
ownership и UV/station flow плюс alpha evolution. Sidecar v2 обязан хранить
те же уровни без координат. Обязательные новые случаи/варианты: одна physical
chain между двумя Patch; `SEAM_SELF` с двумя uses одного Patch; несколько
pChain sources в общем PatchDomain; встреча разных sources; self-collision
одного source; десять boundary-limited scenarios из AM8.
AM8 policy pack хранится в `artifacts/envelope_ec0/pivot/` и входит в
пользовательский EC0-P review corpus.
Гейт: пользователь принимает визуалы; ни одного private per-pChain domain;
PhysicalChain/ChainUse не смешаны; B остаётся patch-level coverage clip.

Текущий статус: kernel-implementer выпустил полный candidate в
`artifacts/envelope_ec0/v2/` (16 v2 sidecars/листов, AM7 pivot cases и
расширенная metamorphic matrix); AM8 pack остаётся в `pivot/`. Формальная и
визуальная генерация завершена, но EC0-P ещё не `ACCEPTED`: ожидается явная
приёмка пользователя по `docs/envelope_ec0_acceptance_guide.md`.

**EC1 — Контракты, снапшот, изоляция**: структура kernel,
SnapshotV1, `PhysicalChain`/`ChainUse`, `PatchDomain`/sectors/holes/barriers,
seed IR, `FrontComponent`/active intervals, envelopes,
Coverage/Interaction/Ownership records и EventLedger.
Ingest группирует ВСЕ sources по owner Patch и создаёт один patch-level plan;
API, компилирующий каждый pChain в отдельном поле, запрещён. Изоляция
доказывается CI-запуском без bpy/cftuv. Гейт: повторный экспорт даёт
идентичный hash; порядок PhysicalChain и ChainUse не влияет на digest;
каждый use ссылается ровно на один physical chain и один owner Patch;
SEAM_SELF сохраняет два use; полная provenance и cross-IR валидация.

**EC2 — Reference Patch Coverage Engine** (ownership отложен): для одного
`PatchDomain` одновременно строятся Strip/Corner(MITER,BEVEL)/Cap/minimal
Junction envelopes всех seeds. Каждый contribution сначала проходит
`BOUNDARY_LIMITED_PROPAGATION`: exact boundary clip + reachability/event
history без рождения branches; затем строится patch union. Два ядра
(CGAL-оракул + production-кандидат). Результат — один single-cover
`PatchCoverage`, а не набор независимо материализованных pChain meshes.
Гейт: силуэт == EC0-P; ноль double cover; перестановка sources не меняет
digest; T/X/Y не распадаются на нашлёпки; каждая boundary curve знает
primitive/source/ChainUse/Patch provenance; каждая вершина объяснена
якорем, пересечением, domain или событием.

**EC2.5 — Intra-Patch Front Interaction.** На patch-level coverage claims
применяются collision/freeze/saturation/junction rules. Для case 16 выбран B:
после mutual arrival вклады юбок одной decal в одном Patch клиппируются по
mutual equality locus. Разные decal и разные Patch не являются участниками
interaction. Self-collision одного source использует тот же contract. Гейт:
до события interaction не меняет coverage; после события нет overlap faces;
никакого first-wins и никакой новой материи; все события именованы.

**EC3 — Турнир ownership** (решение 3). Делит один resolved
`PatchCoverage` между source/station claims; не создаёт и не удаляет matter.
Гейт — тотальность/disjoint, сохранение physical-chain и ChainUse provenance,
детерминизм при перестановке sources; провал = named stop с фактами.

**EC4 — Semantic Arrangement + Coalescing**: общий arrangement
(coverage + ownership dividers + домен + UV-разрывы), RegionRecord
с тотальной provenance; merge ТОЛЬКО при совпадении
owner/lineage/UV-модели/station map/поверхности/семантики —
никаких coplanar/angle-epsilon dissolve. Гейт B (manifold,
детерминизм, независимость от триангуляции).

**EC5 — Compile-static width engine**: event heights
(birth/death/collapse/split/boundary/merge/saturation/freeze);
drag между событиями без topology discovery; правило
«reference навсегда»; пакетная обработка одновременных событий;
именованная CapacityPolicy. Для boundary-limited v1 `split/merge` вокруг
obstacle не исполняются: на `BARRIER_SPLIT_REQUIRED` конкретный
FrontComponent получает `effective_alpha`, геометрия насыщается, а остальные
components продолжают по своему plan. Перф-цель — modal preview.

**EC6 — Shadow-интеграция в CFTUV** (механизм B + гейты A):
новый бэкенд рисует debug-оверлей рядом со старым preview, ничего
не публикует; адаптер НЕ имеет права чинить GeometryBatch,
достраивать грани, менять owner, делать fallback. Сравнение
силуэт/ownership/face count/UV/события/перф/отказы на полном
полевом наборе + differential/0d.0-гейты + личная приёмка
пользователя.

**EC7 — Production cutover**: user-visible, preview==confirm на
одном GeometryBatch, старый planar-бэкенд отключён (named
failures, fallback запрещён), old/new differential — в
regression-сюиту, затем архив PatchVoronoi по S0-паттерну.

**EC8 — Curved**: тот же внешний контракт
(PatchDomain/ChainUse/seed IR/GeometryBatch), другой evaluator
(SurfaceWavefrontEvaluator, данные S-WF0: FMM-кандидат, Heat
дисквалифицирован); НЕ растягивание планарного Boolean через
chart. MSD-принцип: скелет редкий, форма в примитивах.

**EC8+ — OBSTACLE_BYPASS (отложено):** split fronts вокруг hole/concave
boundary, выбор boundary paths, merge за obstacle и новая arrival metric.
Не входит в первую реализацию planar или curved backend.

---

## Объединённые запреты (пересечение списков — 100% совпадение)

private per-pChain domain; независимая материализация каждого source;
слияние двух patch-side uses physical chain; first-wins/smallest-id owner;
round()-ключи; реконструкция
provenance после Boolean; пост-фикс силуэта; raster authority;
отдельный preview-алгоритм; silent fallback; наследование по всем
осям; преждевременная kinetic-оптимизация; нашлёпки поверх union;
join-политика вне CornerEnvelope; epsilon/k-подбор под фикстуру;
«подрежем чуть-чуть» на стыке скоупов; двойное вычисление
интерфейса примитивов; критерий first/last front vertex; пересечение boundary
с последующим rollback; branch creation в boundary-limited v1; телепортация
материи за hole/barrier. Каталог болезней Б1-Б7 — в приёмке ревью
каждого EC-среза.

## AMENDMENTS (AM1–AM6 согласованы двумя ревьюверами; AM7–AM8 —
## последующие инженерные уточнения pivot, принятые пользователем)

**AM1 — Hermetic wheel-only CI + sparse checkout (усиление моей
«CI-стены», признано: virtualenv недостаточен — корень checkout в
sys.path делает `import cftuv` рабочим без установки).**
Kernel-hermetic job: собрать kernel wheel -> чистый контейнер/
каталог БЕЗ монтирования корня repo -> установить ТОЛЬКО wheel и
declared deps -> PYTHONPATH очищен, Python isolated mode ->
тесты из собранного артефакта -> статическая проверка
import-графа. Два CI-job (kernel-hermetic, host-integration) в
одном workflow — две тестовые границы, признано честно.
Kernel-агент работает в sparse checkout/worktree: только
`kernel/`, семантика EC0, компромисс, контрактные доки, fixtures
— старого кода ФИЗИЧЕСКИ нет в рабочем каталоге.
**Extraction-readiness gate (EC1):** каталог kernel копируется в
пустой git-repo — сборка и hermetic-тесты проходят без изменения
файлов.

**AM2 — Три роли (закрывает дыру моего запрета на чтение):**
- *Kernel implementer* — старый бэкенд не читает вообще;
- *Legacy Evidence Curator* — читает старый код/тесты, kernel НЕ
  пишет; производит только: AnalysisSnapshot-fixtures, ожидаемые
  GeometryBatch, скриншоты, event-траектории, описания
  наблюдаемого поведения, список неизвестных случаев (corpus в
  EC1);
- *Host adapter author* — читает обе стороны, только конвертирует
  (AnalysisBundle -> SnapshotV1; GeometryBatch -> preview/
  materialization), геометрических исправлений не делает.
Роли — РАЗНЫЕ сессии/контексты агентов (разделение контекста —
суть механизма).

**AM3 — EC0 = картинки + формальные sidecars + метаморфная
матрица.** EC0a — визуальные листы (приёмка пользователя);
EC0b — YAML-sidecar на каждый случай: region graph, boundary
lineage, owner, adjacency, направление s, topology events,
forbidden-список — БЕЗ точных координат. Поверх каждого случая —
метаморфные преобразования (перестановка/reverse цепочек и
winding, scale/translation, другая триангуляция, разбиение/
слияние data-chains при той же физической линии, одновременные
события, малое возмущение без смены топологии): результат меняется
ТОЛЬКО там, где преобразование меняет семантику — прямой тест
против болезни Б7 («граница ChainDescriptor» != «семантический
конец физической линии»). Статусы случая: DEFINED /
UNSUPPORTED_NAMED_FAILURE / BLOCKED_PENDING_USER_DECISION —
честный BLOCKED лучше выдуманной семантики.

**AM4 — EC2.5 Intra-Patch front interaction — уточнено pivot AM7.**
В case 16 freeze: (A) только делит overlap; (B) обрезает source
contributions и меняет coverage; (C) порождает новую материю. Пользователь
выбрал **B**. Участники — фронты/юбки одной decal в одном `PatchDomain`,
порождённые разными pChain sources либо self-collision одного source.
Разные decal и разные Patch не взаимодействуют. Термин «intercomponent» в
старой формулировке означал source contributions, а не отдельные поля или
decal identities, и больше не используется нормативно. Пайплайн:
`EnvelopeContributions -> PatchPrimitiveUnion -> PatchCoverageClaims ->
IntraPatchFrontInteractionResolver -> ResolvedPatchCoverage ->
OwnershipResolver -> SemanticArrangement`.

**AM5 — SnapshotV1: kernel-owned граничная схема.** Уточнение
«одного контракта»: AnalysisBundle — ВНУТРЕННЯЯ модель хоста;
SnapshotV1 — ЕДИНСТВЕННЫЙ межпакетный контракт, владелец — kernel;
адаптер хоста мапит одно в другое; kernel не импортирует
AnalysisBundle; хост не создаёт свою копию SnapshotV1. JSON Schema
генерируется из типов SnapshotV1; адаптер валидирует каждый
экспорт. (Это не два семантических контракта: внутренняя модель +
один граничный контракт.)

**AM6 — CGAL: границы возможностей и лицензия.**
Arrangement_with_history даёт curve->edge lineage, но НЕ face
coverage/claims — те обязаны рождаться в overlay через расширенный
DCEL (не реконструироваться после Boolean); гейт EC2 дополнен:
каждая arrangement-грань знает ПОЛНЫЙ набор покрывающих
примитивов (FaceRecord.covering_primitive_ids / domain_membership
/ interaction_state). Лицензия: 2D Arrangements — GPL; CGAL-оракул
= отдельный test executable/container; production-зависимость
kernel от CGAL — только через отдельный лицензионно-архитектурный
гейт.

**AM7 — PatchDomain / PhysicalChain / ChainUse pivot — принят
пользователем по инженерному ревью.**

- `PatchDomain` — единственное поле распространения. Оно владеет surface
  geometry/metric, outer boundary, holes, barriers, sectors и surface regime.
- pChain не является полем. `PhysicalChain` хранит source identity;
  directed `ChainUse` описывает, как эту chain видит конкретный Patch.
- Physical seam между Patch A/B даёт два uses в разных domains. `SEAM_SELF`
  даёт два uses одного Patch; uses не объединяются из-за общего id.
- Все `FrontSeed` одного owner Patch живут в одном domain и подаются evaluator
  одной patch-level группой. Локальный contribution каждого source допустим;
  отдельный изолированный solve/materialization на source — запрещён.
- Corner/Junction остаются derived analysis facts и не становятся primary
  topology units, но после analysis их compile-static seeds являются
  атомарными входами envelope compiler.
- Нормативная последовательность уровней:
  `Patch/PhysicalChain/ChainUse facts -> PatchDomain + sectors -> seeds ->
  envelope contributions -> patch union -> interactions -> ownership ->
  GeometryBatch`.
- Front может продолжать рост, закончиться только именованным событием либо
  дать named failure. Silent disappearance/«перестал создавать faces» запрещён.
- Подклассы Patch/pChain не вводятся; свойства остаются ортогональными axes и
  role sets, evaluator polymorphism — в operations/strategies.

AM7 не отменяет выбранный B, single-cover, SemanticAuthority или
CanonicalGeometryDigest. Он меняет unit of execution и требует EC0-P
перевыпуска визуалов/sidecars до EC1.

**AM8 — Boundary-limited propagation v1 — принято как capability boundary.**

- Outer boundary, hole loops и только те rails/ChainUses, у которых явно есть
  role `BARRIER`, являются неподвижными non-owner ограничителями domain.
- На `(ChainUse, owner Patch, sector)` создаётся отдельный `FrontComponent` с
  набором active continuous intervals. Все components Patch всё равно
  вычисляются одной patch-level группой согласно AM7.
- Boundary contact одного component не останавливает другую сторону physical
  chain, другие pChains или весь Patch.
- Front не пересекает boundary и не откатывается: на точном alpha возникает
  `BOUNDARY_CONTACT`. Разрешены exact clip, сокращение/исчезновение intervals и
  движение contact point вдоль того же boundary component.
- В v1 число active branches одного FrontComponent не увеличивается. Если
  продолжение требует обхода hole/concave boundary, split, выбора пути или
  последующего merge, возникает capacity reason `BARRIER_SPLIT_REQUIRED`.
- Геометрия этого component фиксируется на `effective_alpha`; пользовательский
  `requested_alpha` сохраняется в diagnostics. Другие components продолжают.
- `FRONT_EXHAUSTED` — успешное завершение. Capability outcome
  `BARRIER_BYPASS_UNSUPPORTED` показывается как boundary capacity reached, а
  не silent failure или универсальный clamp.
- Exact Boolean clip без source reachability, component count и boundary event
  history недостаточен: material за obstacle без допустимого path запрещён.
- Полноценный obstacle bypass переносится в EC8+.

AM8 уточняет pipeline:
`seeds -> FrontComponents -> local contributions -> BoundaryLimitedResolver ->
BoundaryResolvedContributions -> PatchPrimitiveUnion -> PatchCoverageClaims ->
IntraPatchFrontInteractionResolver -> ResolvedPatchCoverage -> Ownership`.

**Дополнительно принято:**
- **SemanticAuthority-классификация** переносимых поведений:
  USER_REQUIRED / FIELD_PROVEN / MATHEMATICALLY_REQUIRED /
  LEGACY_COMPATIBILITY / IMPLEMENTATION_ACCIDENT / OPEN_RESEARCH.
  В EC0 переносится НАБЛЮДАЕМЫЙ результат (например, freeze), а
  не обязательность старого механизма (например, «именно
  станциями») — иначе старый дизайн проникнет через слишком
  конкретный канон. Каждая переносимая спека получает тег.
- **CanonicalGeometryDigest** для Compiled(alpha) ==
  Reference(alpha): сравнение семантического графа (coverage
  regions, lineage, ownership, adjacency, station/UV-модели,
  события, якоря, FrontComponent active intervals/branch count,
  boundary event history, requested/effective alpha/capacity reason), НЕ
  порядка faces/float-массивов; координаты —
  exact либо один именованный арифметический контракт; tolerance
  НИКОГДА не выбирает топологию или владельца.

Фазы после поправок: **EC0-P** (перевыпуск EC0a/EC0b) -> EC1
(+legacy evidence corpus) -> EC2 -> **EC2.5** -> EC3 -> EC4 -> EC5 ->
EC6 -> EC7 -> EC8 -> EC8+ (optional capability).

## Треки и текущее состояние

```
Трек CFTUV (host): maintenance mode старого бэкенда (уже);
    только адаптер AnalysisSnapshot и shadow-порт (EC6).
Трек kernel: EC0-P -> EC1 -> EC2 -> EC2.5 -> EC3 (research)
    -> АРХИТЕКТУРНЫЙ ГЕЙТ (оба ревьювера + пользователь,
       по результатам coverage/ownership)
    -> EC4/EC5 -> EC6 -> EC7 -> EC8.
```
Первый deliverable после pivot — НЕ код: EC0-P visual/sidecar rebaseline,
AM8 boundary pack и SnapshotV1 schema review на уровнях
facts/domain/seeds/FrontComponents/material inputs.
