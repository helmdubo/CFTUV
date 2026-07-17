# Tranche E3 — Multi-chart atlas design contract

Статус: **IMPLEMENTED** (`ba75d0c`; независимое ревью пройдено,
P0 закрыт `578269a`, поправки R1–R4 реализованы). Пользователь разрешил
interior transitions, временное исключение saddle при доказанной
несостоятельности atlas и отдельный conformal research fallback.

**Предусловие P0 (жёсткое, до первого E3-коммита):** оба теста D5.3
(`test_d5_3_periodic_supporting_lines_never_disappear`,
`test_d5_3_closed_ring_keeps_resolved_partition_during_drag`) зелёные в
каноническом headless-окружении (`python -m pytest`, pyvoronoi из pip,
без Blender). На момент ревью они падают с самого коммита введения
(topology-pop merge не устранён: 14 граней вместо 8 на узкой ширине,
опорные прямые исчезают между ширинами). Acceptance §6.4 этого контракта
опирается на тот же инвариант — строить atlas поверх падающего
фундамента запрещено. Если в окружении исполнителя тесты проходят —
это environment-хрупкость ассертов; заменить точные счётчики граней на
инвариантные проверки (supporting lines, покрытие) И добиться зелёного
в каноническом окружении.

## 1. Цель и границы

Atlas решает случай, когда policy-free метрики показывают малое локальное
искажение ширины, но один глобальный hinge chart самоперекрывается внутри
materialization zone. Каноническая fixture — E.3 gentle saddle.

Atlas не ослабляет admission, не скрывает overlap и не создаёт новый modal
solve. Compile строит charts, transitions и equivalence classes; evaluate при
drag только применяет уже скомпилированное размещение и crops.

## 2. IR

`IntrinsicStripAtlas` содержит:

- упорядоченные `IntrinsicStripChart` с локальными метриками;
- `InteriorChartTransition` для каждой пары chart boundaries;
- canonical ownership source-triangle/source-region;
- общую station/V-phase систему;
- atlas-level aggregate metrics и admission tier.

`InteriorChartTransition` обязан хранить source edges, обе chart-side copies,
ориентацию, station interval и canonical `transition_key`. Ключ строится из
квантованных source/station facts по контракту D, а не из raw float modulo.

**R1 — Site images через transitions (обязательно).** Voronoi-диаграмма
каждого чарта обязана видеть конкурентов из соседних чартов: sites (и их
endpoint corners) в пределах `alpha_budget` от transition получают images
в соседнем чарте через изометрию перехода — тот же механизм, что
periodic copies D2 (images только в диаграмме, dedup при материализации,
счётчик `atlas_site_image_count`). Без этого фронты не сталкиваются
через переход и партиция чартов несовместна.

**R2 — Станции на transition вычисляются ровно один раз.** Любая
станция, лежащая на transition, вычисляется в owner-чарте и потребляется
соседом через equivalence class; пересчёт её позиции в соседнем чарте
запрещён. Обоснование: на non-developable поверхности перенос точки в
соседний чарт двумя путями расходится на holonomy (~defect * дистанция) —
если оба чарта считают одну станцию независимо, на переходе возникает
излом/щель. Канонический owner устраняет расхождение по построению;
остаточная ошибка внутренних границ вблизи перехода ограничена
E2-бюджетом ширины и отдельного допуска не требует.

## 3. Декомпозиция

1. Стартовать с deterministic hinge placement C2.
2. Найти несмежные overlap pairs внутри `alpha_budget`.
3. Построить детерминированный separator по dual graph между overlap owners;
   tie-break — canonical source edge id.
4. Разрезать только chart topology. Source surface и итоговая геометрия полосы
   не разрезаются.
5. Повторять до локальной injectivity либо вернуть канонический reject reason.

**R3 — Детерминированная граница итераций.** Цикл сепараторов ограничен:
не более `min(8, support_triangle_count)` итераций; превышение — немедленный
`ATLAS_INJECTIVITY_UNRESOLVED`, не «ещё одна попытка». Ненулевой
`atlas_unresolved_overlap_count` после лимита — тот же отказ. E.N2 crumple
обязан проходить через atlas-путь до отказа за ограниченное время (negative
acceptance, см. §6).

Нельзя выбирать переход по текущей drag-ширине: topology atlas компилируется
на `alpha_budget`, поэтому S1 и `Construct() == 0` сохраняются.

## 4. Materialization и сварка

- Каждый source-region имеет ровно одного materialization owner.
- Соседний chart может иметь overlap margin для вычисления, но не создаёт
  вторую геометрию в owned region.
- Совпадающие boundary samples свариваются только по transition equivalence
  key; координаты одной equivalence class берутся из канонического owner.
- V наследуется от общей network station; U согласуется ориентацией transition.
- На transition запрещены щель, double cover, flipped winding и UV phase jump.

## 5. Admission и диагностика

Каждый chart проходит локальные G1/G3/G4/G5/G6/G8 и organic width/normal gates.
G2 применяется к materialized atlas arrangement: overlap рабочих chart margins
не является ошибкой, double materialization является.

**R4 — Категории cuts не смешиваются.** Atlas-сепараторы — отдельная
категория (`ChartCut.reason = ATLAS_SEPARATOR`) и НЕ считаются в
single-cut правиле G7: G7 ограничивает только periodic/boundary
generating cuts. Сепаратор обязан соблюдать EP4-исключение осознанно:
он ПРОХОДИТ через materialization zone (в этом смысл E3), но материю не
режет — шов существует только в chart topology и сваривается
transition-ключами до эмиссии.

Обязательные counters:

- `atlas_chart_count`;
- `interior_transition_count`;
- `interior_weld_count`;
- `atlas_unresolved_overlap_count`;
- per-chart width error / normal variation / foldover.

Новый отказ: `ATLAS_INJECTIVITY_UNRESOLVED`. Он не маршрутизируется как
APPROXIMATE и не маскируется legacy-геометрией внутри успешно принятой части.

## 6. Acceptance

1. E.3 gentle saddle: ноль unresolved materialization overlap, cracks,
   double cover, foldover и UV phase jumps.
2. `max_width_error_sampled <= decal_chart_distortion_budget` после stitching.
3. Transition geometry бит-точна при reversed enumeration и повторном compile.
4. Width drag не меняет chart topology/supporting lines; `Construct() == 0`.
5. C7, D4 и walls differential при дефолтах байт-идентичны.
6. E.1 sphere S1 остаётся зелёным.
7. Negative: E.N2 crumple через atlas-путь -> `ATLAS_INJECTIVITY_UNRESOLVED`
   за ограниченное число итераций (R3), без частичной геометрии.
8. Фронт-коллизия через transition: фикстура с двумя цепочками в разных
   чартах atlas'а — фронты сталкиваются корректно (site images R1),
   шов коллизии S1-стабилен при drag.

## 7. Stop и разрешённые fallback decisions

Остановиться до production merge, если E.3 требует изменения B0 quantization,
double materialization или width-dependent topology. Приложить метрики и
минимальную fixture к одному из двух явных решений:

- временно перевести saddle в REJECT/deferred;
- открыть отдельный conformal research spike.

Conformal spike обязан отдельно ответить на headless execution parity,
dependency policy, determinism, compile/evaluate split и differential. Сам факт
лучшей картинки не является разрешением включить его в production.

## 8. Независимое review до implementation

Ревьюер должен проверить:

1. что atlas не превращает chart/corner в новую runtime placement unit;
2. однозначность ownership и отсутствие double cover;
3. достаточность transition key для бит-точной сварки и UV parity;
4. отсутствие width-dependent compile topology;
5. корректность G2 на atlas arrangement;
6. достижимость E.3 без скрытого conformal solve;
7. полноту negative tests и canonical reject path.

После review статус меняется на **APPROVED** с commit/reviewer evidence.

## 9. Implementation evidence

- P0: periodic T-stations нормализуются до fragment union; D5.3 проверяет
  manifold, source-spine coverage и supporting-line stability на
  `pyvoronoi 1.2.8`.
- E.3 saddle: 24 locally-injective charts, 105 interior transitions,
  6 separator iterations, ноль unresolved local overlaps.
- Public compile: 276 atlas site images, 24 compile-time `Construct()`;
  при width drag новых `Construct()` нет, interior weld count ненулевой.
- Negative crumple возвращает `ATLAS_INJECTIVITY_UNRESOLVED` на заданном
  iteration limit без partial atlas.
- Полный headless suite после implementation: `331 passed`.
