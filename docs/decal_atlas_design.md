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

## 3a. T-контракт: единое 1D-разбиение transition (поправка E4-ревью)

Предписанное решение блокера «два локальных arrangement дают
несовпадающие sliver-разбиения на transition». Ошибка предыдущей
попытки — порядок: два независимо разбитых 2D-графа сваривались
постфактум. Контракт инвертирует порядок: **сначала согласуется 1D
граница, затем каждый 2D arrangement строится уже конформным ей.**
Глобального 2D atlas-графа НЕ существует и не требуется (его нельзя
построить без глобального чарта — в этом весь смысл атласа);
atlas-wide объект — только 1D граф transitions.

**T1 — Каноническое пространство transition: 1D дуговой параметр.**
У каждого transition есть детерминированный owner-chart (по R2).
Каноническое пространство T — параметр длины дуги `t` вдоль
transition-polyline, квантованный на B0-решётке owner'а (целочисленная
1D решётка). Всё, что лежит на T, ключуется `(transition_id,
quantized_t)` — НИКОГДА 2D-координатами какого-либо чарта.

**T2 — Разбиение T вычисляется ровно один раз, owner'ом.**
T-station set = отсортированное объединение quantized t: (a) концы T;
(b) пересечения T с кривыми owner-чарта (в owner-пространстве);
(c) пересечения T с кривыми соседа: сосед ДЕКЛАРИРУЕТ свои
пересекающие кривые, отобразив их через изометрию перехода в
owner-пространство (машинерия R1 site images), owner считает параметры
пересечений в СВОЁМ пространстве; (d) R2 transition stations.
Готовый список раздаётся обоим чартам: каждый вставляет свою копию T
в локальный arrangement УЖЕ разбитой ровно в этих параметрах.
Локальным arrangement'ам ЗАПРЕЩЕНО дополнительно делить transition-
ребро. Если после отображения локальное пересечение не попадает в
канонический набор: в пределах одного кванта — снап к ближайшему
каноническому t; дальше кванта — жёсткий отказ
`ATLAS_TRANSITION_DESYNC`, не сварка. Overfull на transition-subedges
становится невозможен по построению: последовательность вершин на T
идентична с обеих сторон, каждый subedge имеет ровно одну грань с
каждой стороны.

**T3 — Кривые, пересекающие T, разрезаются НА T, не проецируются
СКВОЗЬ.** Ни одна 2D-кривая не существует в двух чартах как одна
сущность: кривая режется в каноническом t на две полукривые — часть в
A уходит в arrangement A, часть в B — в B; обе оканчиваются в общей
канонической вершине `(transition_id, t)`. Holonomy поглощается в
единственном легитимном месте: endpoint не-owner-полукривой ПЕРЕДВИГАЕТСЯ
в каноническую вершину (сдвиг ограничен E2-бюджетом) — это правка
конца кривой ДО построения arrangement, а не spatial post-weld граней.

**T4 — Грани остаются chart-local.** Face tracing — по-прежнему на
локальном arrangement каждого чарта, как в single-chart E4; обратной
«проекции traced faces» не существует, потому что грани никогда не
покидают свой чарт. Cross-chart смежность для manifold/M1-оракула
устанавливается через общие ключи subedge `(transition_id,
t-interval)` — они идентичны с обеих сторон по T2.

**T5 — Грань, «пересекающая» transition, невозможна по построению.**
Полная polyline T вставлена в оба локальных arrangement — T всегда
граница граней, никогда не внутренность. Классификация остаётся
per-chart с существующим приоритетом; transitions идут вдоль
source-рёбер, поэтому source-треугольник никогда не делится
transition'ом и владение материализацией однозначно.

**T6 — Sliver-политика на T.** Два declared-пересечения в пределах
одного кванта t детерминированно сливаются в одну каноническую
вершину (округление к решётке, слияние равных). Остаточный subedge
длиной в один квант легален и существует идентично с обеих сторон.

M1-оракул для атласа: overfull-подсчёт на transition-subedges ведётся
по каноническим `(transition_id, t-interval)` ключам — ровно 2 грани
на внутренний subedge (по одной с каждой стороны).

**T7 — Semantic transport (предписание по kind/UV-дефекту KITE↔SEGMENT).**
Диагноз: сосед не может и НЕ ДОЛЖЕН реконструировать corner-семантику
владельца — у него нет второго incident site (стратегия A добавляла
конкуренцию и ломала партицию 1->4; стратегия B давала предикат без
согласованных граничных кривых в графе и лишнюю материю 1->2; обе
верно откатаны). Принцип «владелец решает, сосед потребляет»
расширяется с геометрии на семантику:

1. Для каждой semantic-области владельца (corner component,
   junction-сектор), чья площадь пересекает T: владелец ОДИН раз
   отображает через изометрию перехода за-T-части её граничных кривых
   и передаёт соседу как **imported arrangement curves** — инертная
   геометрия (не Voronoi-сайты, не конкуренция, как рёбра
   domain-треугольников), заякоренная в канонических T-станциях и
   несущая ярлык владельца (kind, side, owner corner/site id, ссылка
   на UV-frame владельца). Import-набор compile-ограничен тем же
   радиусом alpha_budget, что R1.
2. Классификация соседних граней: грань, чья репрезентативная точка
   лежит внутри transported-области (ограниченной T-интервалом +
   imported curves — все они РЁБРА графа, потому рассинхрон
   предикат/грань невозможен), получает ярлык и UV-frame владельца
   (перенос — той же изометрией, что R2; holonomy в E2-бюджете).
   Вне области — обычная локальная классификация. Материя НЕ
   добавляется и НЕ вычитается — только переразметка уже
   существующих граней, теперь корректно ограниченных.
3. Ярлыки вычисляет владелец при каждом evaluation с текущими
   settings — threshold drag остаётся согласованным по построению
   (single source). Imported supporting-кривые статичны (S1),
   двигаются только frontier-части — compile topology
   ширино-независима.
4. Пост-трассировочная разрезка граней (вариант «separator от
   T-станции до ближайшей кривой») ЗАПРЕЩЕНА: multi-owner грань — это
   симптом ОТСУТСТВУЮЩЕЙ imported-кривой, а не задача для резака.
   С T7 разделители появляются в графе до трассировки.
5. Если transported-область пересекает более одной transition —
   механизм применяется по каждой, канонические вершины общего
   transition-графа общие; неоднозначность здесь = stop condition с
   минимальной фикстурой.

**T8 — Обязательный kind/UV-оракул через transitions.**
Ярлыки интервалов группируются в классы семантической
эквивалентности: corner-kinds эквивалентны при равных
(kind, owner corner id); SEGMENT-ярлыки эквивалентны при одной chain
и V-непрерывности в общей станции (в пределах кванта). Оракул:

- O1: для каждого внутреннего transition-subedge обе смежные грани —
  в одном классе эквивалентности; U (как u*alpha) и V в обеих общих
  станциях совпадают в пределах кванта.
- O2: грань может касаться нескольких T-интервалов ТОЛЬКО если все их
  ярлыки в одном классе (SEGMENT 12016 + SEGMENT 12018 одной chain с
  непрерывной V — легально; KITE + SEGMENT — структурный провал =
  отсутствует imported-кривая; чинится T7, не переразметкой).
- O3: смена класса вдоль T происходит только в канонической станции,
  в которой в ОБОИХ локальных графах инцидентна разделяющая кривая
  (локальная или imported).

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

## 8a. Вердикт глубокого ревью ed8e1e8 + b7817cf (T1-T6/M1)

**ACCEPT с обязательными исправлениями.** Ни одного неверного вывода
на фикстурах; EP1 подтверждён worktree-байт-сравнением с baseline
(walls 1.0x, идентичные снапшоты); overfull=0, components=1,
double-cover=0, реверс-энумерация бит-точна на sphere/saddle по 10
ширинам; скрытого spatial post-weld нет; ориентация T-станций
корректна по построению.

Обязательные исправления до полного E-acceptance:

1. **(блокирующее) COLLINEAR-drop**: `conformed_path`
   (`decal_voronoi.py:~6095`) помечает `lies_on_transition` по
   совпадению БЕСКОНЕЧНЫХ прямых без проверки перекрытия extent'ов —
   26 реальных кривых/кадр удаляются на saddle без диагностики;
   сегодня спасает случайная избыточность. Гейтить по фактическому
   перекрытию, сохранять неперекрытые коллинеарные части.
2. **(блокирующее) Учёт тихих потерь**: счётчики + ассерты (==0 в
   тестах) для sliver-rule срабатываний, no-owner drop покрытых
   in-domain граней (`:~6583` — голый continue), touch-no-token drop
   и single-declared-side drop. Инвариант 11: ни один из этих путей
   сегодня не наблюдаем.
3. Закоммитить уже проходящие усиленные оракулы: 8-10-ширинные
   sweeps с double-cover сэмплингом, реверс-энумерация через
   публичный путь, негативный тест ATLAS_TRANSITION_DESYNC (8 raise-
   сайтов — ноль тестов), UV-phase/kind непрерывность через
   transition subedges.
4. (до интерактивного APPROXIMATE) Отключить legacy clip/subtract
   для APPROXIMATE-surfaces — его выход M1-ядром не читается,
   бесплатный constant-factor.

Sliver-правило (B0-bounded nearest-boundary, 1 квант): **условно
допустимо** — детерминировано и coverage-preserving, но это отход от
буквы stop-condition E4#2; легализуется как counted-механизм с
границей в один квант; рост счётчиков или срабатывание zero-drop
ассерта эскалирует в design decision, не правится на месте.

Interval-owner форсинг у transition (обход 2D-предиката) — сверх
T5-контракта; допущен как реализация R2-канона, но кросс-сторонняя
непрерывность kind/UV обязана быть покрыта оракулом п.3.

Perf: формальный гейт E4 пройден (walls 1.0x), но APPROXIMATE-кадр —
секунды (saddle 4.3-8.5s, sphere 11-18s/frame): до F1/F2 путь
непригоден для интерактивного drag — не подключать к modal без
отдельного решения.

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
