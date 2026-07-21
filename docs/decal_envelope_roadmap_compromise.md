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
позже — он уже изолирован; обратное слияние дороже. Финальное
слово — пользователя: если психологическая жёсткость границы
важнее логистики, план работает и с отдельным repo без изменений.

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
