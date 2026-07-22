# Envelope-бэкенд: КОМПРОМИССНАЯ дорожная карта
## Сведение планов двух ревьюверов (идёт исполнителю после
## решения пользователя по п.1)

Исходники: `decal_envelope_roadmap_claude.md` (ревьювер A) и план
ревьювера B (в отчётах пользователя). Согласие ~90%; ниже — пять
решений по схеме ревьювера B, затем объединённые фазы EC0-EC8.

---

## Пять решений

### 1. Граница проекта — ЕДИНСТВЕННЫЙ открытый пункт (решает
пользователь)

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
  (decal_rails.md — правила, не код);
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

### 3. Ownership — СОГЛАСОВАНО: турнир, с двухскоуповой поправкой
Турнир O1-O4 (план B) принимается как ГЛАВНЫЙ исследовательский
этап, с поправкой A: турнир решает ВНУТРИкомпонентное владение;
МЕЖкомпонентная конкуренция — канон RC1-RC3 (station freeze,
RF7) и формирует материю легально (двухскоуповое правило,
wavefront-заметка). Рабочая гипотеза (не предрешение): O1 для
chain-local corner/cap/junction + O2 для nonlocal planar +
O3 как оракул + O4 (station) — межкомпонентный слой. Гейт-критерии
плана B (тотальность, disjoint, детерминизм, топологическая
объяснимость, сохранение s, неизменность coverage, отсутствие
скрытых приоритетов) — без изменений. Если policy не находится —
модель C не готова, независимо от качества coverage (честный
критерий опровержения).

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
`AnalysisBundle` (+ PatchDescriptor/ChainDescriptor, производимые
В ОДНОМ месте адаптера). Второй самостоятельный контракт анализа
запрещён (болезнь Б1 на уровне контрактов). JSON на research-фазе
(читается глазами, диффается в Git). Модели данных плана B
принимаются: PatchDescriptor (ортогональные оси SurfaceRegime /
PatchShapeClass / context_tags; WALL/FLOOR — контекст, НЕ выбор
бэкенда), ChainDescriptor + ChainUse с МНОЖЕСТВОМ ролей
(SELECTED_SPINE, DOMAIN_BOUNDARY, BARRIER, FOLD_GUIDE, CHART_CUT,
MARKED_ROUTE). Подклассы Patch/pChain — отвергнуты обоими
планами; полиморфизм — только в операциях (SurfaceMetric,
evaluator-стратегии).

---

## Объединённые фазы EC0-EC8

**EC0 — Семантика до алгоритма** (из плана B, принято A как
улучшение): `envelope_backend_semantics.md` — для 15 канонических
случаев B + случай 16 (поправка A): ДВА КОМПОНЕНТА встречной
конкуренции (межкомпонентный freeze — RF7-семантика) — по четыре
рисунка (skeleton / coverage / ownership / UV-station flow) +
эволюция по alpha. Гейт: ни одного «решим потом» из списка B.

**EC1 — Контракты, снапшот, изоляция**: структура kernel (п.1
после решения пользователя), AnalysisSnapshot v1, все IR
(DecalSkeletonIR, примитивы, Coverage/Ownership records,
EventLedger-типы), изоляция доказана CI-запуском без
bpy/cftuv. Гейт B: повторный экспорт -> идентичный hash,
инвариантность к порядку перечисления, полная provenance,
cross-IR валидация.

**EC2 — Reference Coverage Engine** (ownership ОТЛОЖЕН — разрез
плана B принят): Strip/Corner(MITER,BEVEL)/Cap/минимальный
Junction, клип доменом, exact union; два ядра (CGAL-оракул +
production-кандидат). Гейт: силуэт == EC0-семантике; ноль
парабол; ноль double cover; независимость от порядка примитивов;
каждая граничная кромка знает породивший примитив; число вершин
силуэта объяснено (якоря/пересечения/домен/события).

**EC3 — Турнир ownership** (решение 3). Гейт — критерии B; провал
= стоп модели C с фактами.

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
именованная CapacityPolicy. Перф-цель — modal preview.

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
(SkeletonIR/интенты/GeometryBatch), другой evaluator
(SurfaceWavefrontEvaluator, данные S-WF0: FMM-кандидат, Heat
дисквалифицирован); НЕ растягивание планарного Boolean через
chart. MSD-принцип: скелет редкий, форма в примитивах.

---

## Объединённые запреты (пересечение списков — 100% совпадение)

first-wins/smallest-id owner; round()-ключи; реконструкция
provenance после Boolean; пост-фикс силуэта; raster authority;
отдельный preview-алгоритм; silent fallback; наследование по всем
осям; преждевременная kinetic-оптимизация; нашлёпки поверх union;
join-политика вне CornerEnvelope; epsilon/k-подбор под фикстуру;
«подрежем чуть-чуть» на стыке скоупов; двойное вычисление
интерфейса примитивов. Каталог болезней Б1-Б7 — в приёмке ревью
каждого EC-среза.

## AMENDMENTS (ревью компромисса ревьювером B; приняты ревьювером
## A полностью — все шесть)

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

**AM4 — EC2.5 Intercomponent interaction — главная архитектурная
поправка (принята; трёхстадийность признана).** Вопрос EC0-кейса
16: freeze (A) только делит overlap (union до == после -> это
ownership); (B) обрезает компоненты (union до != материя -> это
coverage-взаимодействие); (C) порождает новую материю (отдельный
оператор). Гипотеза канона (НЕ предрешение, проверяется в
EC0/EC2.5): вариант B — взаимный клип покрытий по локусу
станционного равенства, гейтированный обоюдным прибытием (RC5b);
подтверждает/опровергает пользователь по рисункам. Пайплайн
получает явную стадию: PrimitiveEnvelopeUnion -> ComponentCoverage
-> IntercomponentInteractionResolver -> ResolvedCoverage ->
OwnershipResolver -> SemanticArrangement.

**AM4-КОРНЕВОЕ УТОЧНЕНИЕ (пользователь; отменяет посылку case 16).**
РАЗНЫЕ декали В СТОЛКНОВЕНИИ НЕ УЧАСТВУЮТ. Декаль всегда ОДНА
(порождена набором выбранных pChains). Сталкиваются КРЫЛЬЯ (юбки)
декали: (i) крыло от одной pChain — с крылом от другой pChain
того же набора; (ii) крыло — само с собой (вогнутость, обёртка).
Следствия для A/B/C (они рисовались для несуществующего кейса
«две декали»):
- **C ОТКЛОНЁН окончательно:** третья материя/третий owner
  `INTERACTION` внутри одной декали не имеет смысла (это была бы
  выдуманная сущность). Стык-элемент между РАЗНЫМИ декалями —
  отдельная будущая фича артиста (парковка), НЕ семантика
  столкновения.
- **A и B СЛИВАЮТСЯ в один ответ:** у одной декали нет «внешнего
  bulge одного компонента поверх другого» — вся материя есть ОДИН
  union смещения набора pChains; локусы встречи ВСЕГДА внутренние
  (интерьер union). Поэтому «обрезка внешнего силуэта» (различие
  A и B) не существует — обрезать нечего.
Каноническое поведение (замена гипотезы B):
- **Coverage = ОДИН union** крыльев всех pChains, клип доменом;
  внешний силуэт — смещение набора источников (skeleton/
  medial-offset), локусы встречи в него не входят.
- **Локус встречи РАЗНЫХ pChains** = замороженный watershed
  (станционное равенство RC1, freeze RC3, S1): (а) владеет
  разделением станционной параметризации (какой pChain даёт s
  слева/справа), (б) предотвращает двойное покрытие крыльев —
  каждое крыло ограничено локусом (нет self/mutual double-cover
  одной декали = нет z-fighting с самой собой). Это НЕ два
  представления: локус — один, читается и как owner-divider, и
  как граница крыла.
- **Крыло само с собой** = single-owner, single-cover; хребет по
  medial-axis собственного источника; материя ограничена
  self-meeting локусом (double-cover запрещён).
Механизм = «B» (фронты ограничивают друг друга на замороженном
локусе, double-cover не рождается), силуэт = union одной декали
(нет обрезки, т.к. нет конкурирующего внешнего силуэта). Это
ТОЧНО то, что уже моделируют RR10 (нити между chains) + RC1-RC3
(станционная конкуренция/freeze) + RF7/RF17 — рефрейм, не рефактор.
EC0 case 16 переименовать: «встреча крыльев ОДНОЙ декали
(inter-pChain и self)»; трёхкартиночный A/B/C заменить на один
каноничный лист + негативы (нет третьей материи; нет
double-cover; внешний силуэт = union, локус внутренний).

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
  события, якоря), НЕ порядка faces/float-массивов; координаты —
  exact либо один именованный арифметический контракт; tolerance
  НИКОГДА не выбирает топологию или владельца.

Фазы после поправок: EC0a -> EC0b -> EC1 (+legacy evidence
corpus) -> EC2 -> **EC2.5** -> EC3 -> EC4 -> EC5 -> EC6 -> EC7 ->
EC8.

## AM11 — Sparse Envelope / Linear-Axis North Star (внешний
## ревьювер по MSD+LinearAxis; принято ревьювером A). Нормативно.

**Северная звезда нового движка (закрепляется как основной канон
архитектуры):**
```
редкий source-скелет (PhysicalChain/ChainUse/relations)
  + выразительные аналитические envelope-примитивы
  + exact resolved coverage (силуэт)
  + отдельный ownership (не трогает силуэт)
  + ленивый детерминированный event-ledger
  + downstream-тесселяция
```
Полный generalized straight-skeleton solver — НЕ обязательная
цель: вводится только если аналитические envelopes + exact union
+ детерминированный ledger не выражают нужную семантику.

**AM11.1 — Единая угловая семантика (заменяет MITER/BEVEL/ROUND в
ЯДРЕ).** В core вместо трёх алгоритмов — один `LinearReflexProfile`
(reflex-angle + hidden-edge pattern k): k=0 -> апекс (MITER);
малое k -> хорда/срез (BEVEL); растущее k -> полигональная дуга
(ROUND). MITER/BEVEL — только UI/migration-пресеты, компилируемые
в profile; core-поле `join_policy` -> `angular_profile`.
**ГАРД (I3): k — семантика, не скрытый порог.** k выводится из
ИМЕНОВАННОЙ политики (равное деление reflex-excess; k из явного
предела sub-turn Δ_max либо named speed-bound), НИКОГДА из
`if angle > 137.4`. Оговорка честности: ε-эквивалентность
medial-axis из статьи мы НЕ гарантируем (не гоняем глобальную
conflict-проверку) — angle-based profile принимается как
ПРОДУКТОВАЯ семантика, не как «доказанный medial-approx».

**AM11.2 — Четыре уровня угла (разделение, «один источник — много
читателей»):** `CornerRelation` (факт analysis: вершина, incident
ChainUses, сектор, угол — НЕ материя) -> `CornerSeed`
(компиляция под request) -> `AngularEnvelopeSpec` (закон формы:
hidden supports, зависимость от alpha) -> `EnvelopeInstance(t)`
(полигон при ширине). Hidden edges живут ВНУТРИ spec — НЕ
становятся PhysicalChain/ChainUse/глобальными skeleton-рёбрами
(иначе теряется sparse-skeleton). Контролируемый fan из hidden
edges — семантическая граница, не артефакт (детерминирован,
локален, с provenance).

**AM11.3 — Терминология зафиксирована (разные уровни, не
синонимы):** EnvelopeSpec (генератор E_i(t)) / EnvelopeInstance
(E_i при t) / Wavefront (движущаяся boundary-state, contribution-
front vs coverage-front) / RawCoverage (Ω ∩ union) /
ResolvedCoverage (после interaction) / Ownership (делит
ResolvedCoverage, силуэт не трогает). StripEnvelope сохраняет
ПАРАЛЛЕЛЬНЫЕ продольные supports; terminal-интерфейсы
(Strip↔Corner, Strip↔Cap) МОГУТ быть косыми — одно другому не
противоречит.

**AM11.4 — Event-стратегия: ленивый детерминированный ledger
(уточняет I1/EC5; ревьювер прав — «компилировать все высоты до
drag» слишком сильно).** До drag компилируются ЗАКОНЫ,
source-facts, event-predicates и правила смены топологии.
Инстансы будущих событий создаются ЛЕНИВО по мере роста t —
но ТОЛЬКО из заранее определённых predicates. Runtime НЕ вправе:
изобрести новый source/route/тип события, сменить hidden-profile,
применить tolerance как решение. Runtime ВПРАВЕ: после collapse
породить следующие candidate-события, расширить очередь, вычислить
точное время, закэшировать интервал. Движение между событиями —
АНАЛИТИЧНО (moving lines n·x=c+vt, аффинные траектории), не
пошаговая симуляция. Reference-evaluator (полная пересборка на
любом t) — вечный оракул. Drag назад: event-undo-stack либо
checkpoint+replay. Частично собранных faces пользователь не видит
(PENDING_COMPUTE); confirm использует тот же AdvanceTo(t).

**AM11.5 — Rails понижены до constraints/routes** (согласно
S-WF0=A): rail без роли BARRIER препятствием не становится; роли
— BARRIER / FOLD_GUIDE / SURFACE_ROUTE (station/provenance/
event-transport) / CHARACTERISTIC_HINT. Гладкий фронт НЕ обязан
двигаться по рёбрам меша (иначе зависимость от триангуляции —
болезнь S-WF0). Новая сущность — `SurfaceRoute`; legacy
rail-реализацию каноном НЕ переносить.

**AM11.6 — Cross-Patch lift = отдельная координированная стадия**
(разделение «три оффсета»: ширина — intrinsic внутри Patch; отрыв
— extrinsic после coverage/ownership). `CrossPatchLiftRelation`:
пересечение offset-плоскостей соседних патчей даёт ОДИН shared
lifted key (без щелей/overlap/двойных seam-линий); при
неединственности — именованный fail. Усреднение нормалей по
умолчанию ЗАПРЕЩЕНО (прямой урок старого dihedral-бага
patch-average). Curved-lift — зарезервировать точку архитектуры,
реализация позже.

**AM11.7 — 360°/SEAM_SELF ≠ обычный reflex.** Открытый конец
seam НЕ моделируется как `ReflexEnvelope(angle=360°)`
(вырожденный случай, hidden-count у 2π искусственно раздут,
направление fan неоднозначно). Терминальный конец ->
`TerminalRelation` -> CapEnvelope; сложный полный сектор вокруг
конца -> `Pole/JunctionRelation` -> JunctionEnvelope. У SEAM_SELF
два разных ChainUse — их terminal-топология берётся из analysis,
не из фиктивного угла.

**AM11-ГАРД ДИСЦИПЛИНЫ (ревьювер A):** внешний разбор даёт
детальный ТИП-модель (поля StripEnvelopeSpec, AngularEnvelopeSpec
и т.д.). Принимаем СЕЙЧАС: принципы AM11.1-11.7, терминологию,
разделение сущностей — это СЕМАНТИКА. Конкретные field-schema —
СИЛЬНЫЙ ВХОД EC1, но НЕ канон EC0: EC0 остаётся про семантику
(картинки + sidecar-графы) и приёмку пользователя; заморозка
схемы до одобрения картинок — нарушение дисциплины «не строить на
непроверенном фундаменте» (урок GL/атлас).

**Связь с AM4 (одна декаль).** North star ПОДТВЕРЖДАЕТ поправку
пользователя: RawCoverage одной декали = single-cover смещение
НАБОРА pChains; скелет набора (medial/straight) — внутренняя
структура, одновременно ownership-watershed и граница single-cover
(где крыло встречает чужое крыло или само себя). «Interaction» для
ОДНОЙ декали = обеспечение single-cover вдоль скелета, НЕ клип
одной декали о другую и НЕ третья материя. A/B/C-развилка EC2.5
для одной декали КОЛЛАПСИРУЕТ: ответ — RawCoverage(union,
внутренний overlap) -> ResolvedCoverage(single-cover,
скелет-раздел). EC2.5 остаётся как стадия, но её «политика» для
одной декали предопределена single-cover-инвариантом; A/B/C
всплыли бы только для будущей отдельной фичи «стык РАЗНЫХ декалей»
(парковка).

## Треки и текущее состояние

```
Трек CFTUV (host): maintenance mode старого бэкенда (уже);
    только адаптер AnalysisSnapshot и shadow-порт (EC6).
Трек kernel: EC0 -> EC1 -> EC2 -> EC3 (research)
    -> АРХИТЕКТУРНЫЙ ГЕЙТ (оба ревьювера + пользователь,
       по результатам coverage/ownership)
    -> EC4/EC5 -> EC6 -> EC7 -> EC8.
```
Первый deliverable исполнителю — НЕ код: EC0-семантика и схема
AnalysisSnapshot v1 (согласие обоих планов).
