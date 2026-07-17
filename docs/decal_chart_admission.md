# Chart Admission Oracle — допуски C3 и приёмочные фикстуры C7

Статус: **APPROVED**. Пользователь подтвердил бюджет 2%, отсутствие ручек,
single-cut policy, отрицательные fixtures и визуальный критерий C7 перед
автономным выполнением оставшейся части Tranche C.

Документ фиксирует ЧИСЛА и ПРОЦЕДУРУ приёмки для одного универсального
механизма (hinge unroll полосного чарта, C0–C2). Это не таксономия
случаев: gate один, фикстуры лишь доказывают его на представительных
формах — включая отрицательные, где gate обязан отказать.

Все допуски ссылаются на фактические поля `ChartBuildMetrics`
(`cftuv/decal_charts.py`).

---

## 1. Продуктовое обоснование: единственный видимый критерий

Игрок/артист видит ровно одно искажение: **ошибку ширины декали**
(трим-полоса шире/уже заявленного, паттерн плывёт поперёк). Порог
незаметности: при типичной ширине трима 0.2–0.5 м и плотности
1024 px/м texel ≈ 1 мм; ошибка ширины до ~2% (2–10 мм на полосе)
неразличима на архитектурных тримах. Отсюда единый бюджет:

**D_max = 2% относительной ошибки ширины.**

Маппинг на метрики: угловой дефект `delta` (рад) у вершины внутри
полосы искривляет поперечную метрику на ~`delta` относительно ширины
(малые углы, вершина в середине полосы — худший случай). Поэтому
`delta_max = 0.02 rad (~1.15°) <=> 2% ширины`. Невязка замыкания цикла
— та же holonomy, накопленная по пути: нормируется на полуширину.

---

## 2. Admission gate (C3) — один набор порогов

Чарт принимается, если выполнены ВСЕ условия. Иначе —
`ChartBuildFailure(reason)` и component-level fallback на текущий
planar/legacy путь. Молчаливая аппроксимация запрещена (инвариант 11
workplan'а).

| # | Условие | Порог | Поле / проверка |
|---|---|---|---|
| G1 | Топология после cuts — disk / open strip | binary | построение C2 |
| G2 | Нет пересечений несмежных треугольников в 2D | `== 0` | `triangle_overlap_count` |
| G3 | Максимальный дискретный угловой дефект внутренних вершин | `<= 0.02 rad` | `discrete_angle_defect` |
| G4 | Невязка замыкания циклов, нормированная на полуширину | `<= 0.02 * N` | `max_loop_closure_residual`, `N` — см. ниже |
| G5 | Относительная ошибка длин рёбер unroll (числ. точность, не distortion) | `<= 1e-5` | `max_edge_error` (relative) |
| G6 | Сохранение площади чарта | `abs(ratio - 1) <= 0.01` | `chart_area_source_area_ratio` |
| G7 | Cut policy: не более одного deterministic source-edge cut-path; только если после него чарт — disk. В periodic domain selected strip вправе пересекать cut транзверсально; запрещён только коллинеарный overlap selected source edge с generating path | binary | `cuts`, `ChartCut.reason`, `ChartCut.source_edges` |
| G8 | Однозначность размещения: каждый support-треугольник размещён ровно один раз | binary | `DISCONNECTED_CHART_SUPPORT` / dup guard C2 |

Нормировка G4: `N = alpha_budget`, если бюджет конечен
(`STRIP_BUDGET` или `PERIODIC_HALF_PERIOD`); для
`FULL_CONNECTED_COMPONENT` —
`N = max(compile-time initial alpha, средняя длина source-ребра
support'а)`. Правило детерминировано и фиксируется в коде рядом с gate.

Пороги — константы модуля, НЕ пользовательские настройки. В Tranche C
ручек не добавлять; единственная будущая ручка — distortion budget E2,
и она расширит этот gate, а не заменит его.

Смысл запаса: все шесть положительных фикстур C7 — точно developable,
на них фактический `discrete_angle_defect` — тесселяционный шум
(ожидание `<= 1e-4 rad`, см. таблицу). Порог 0.02 существует для
слабокривых поверхностей (купола) и наследуется E-траншем; в C он
просто не должен срабатывать.

---

## 3. Отказные причины (канонические reason)

- `NON_DEVELOPABLE_SUPPORT` — G3/G4 превышены;
- `CHART_SELF_OVERLAP` — G2;
- `AMBIGUOUS_PLACEMENT` / `DISCONNECTED_CHART_SUPPORT` — G8/G1;
- `CUT_CROSSES_SELECTED_CHAIN` — G7;
- `PERIODIC_HOLONOMY_UNSUPPORTED` — две стороны periodic cut не связаны
  чистой трансляцией (rotation или относительное изменение period > 0.02);
- `MULTI_CUT_REQUIRED` — цикл не сводится одним cut (v1 отдаёт в
  fallback, не импровизирует atlas);
- `DEGENERATE_SOURCE_TRIANGLE` — нулевая площадь (существующий C2).

Каждый reason виден в routing report; счётчики отказов — в benchmark
JSON (A0).

---

## 4. Измерение geodesic half-width error (функциональная проверка)

Определение для приёмки фикстур: на K >= 20 равномерных станциях вдоль
цепочки берётся поперечный луч в чарте до rail'а (`дистанция = alpha`);
поднятая на mesh точка обязана лежать на геодезическом расстоянии
`alpha * (1 ± D_max_fixture)` от цепочки. Для developable-фикстур
геодезика по развёртке точна, поэтому проверка фактически ловит ошибки
lift/provenance, а не математики.

---

## 5. Приёмочная таблица фикстур (C7)

Общие проверки для всех положительных фикстур (из C7 workplan):
один edge-connected decal component на сторону, ноль
zero-area/non-manifold/overfull edges, отсутствие junction-connector
артефактов между facets, preview == confirm signature,
`Construct() == 0` во время drag, `support_triangle_count`
пропорционален полосе (не всему patch'у) на крупной фикстуре,
детерминизм при reversed triangle enumeration.

| # | Фикстура | Что доказывает | Admission | Ожидаемые метрики | Спец-проверки |
|---|---|---|---|---|---|
| F1 | Two-plane folded strip (стена + излом 135°) | базовый hinge через один fold | ADMIT | defect <= 1e-4; residual = 0 (нет циклов); overlap = 0 | непрерывность полосы через fold; ширина 1% |
| F2 | Multi-face bevel strip (фаска 3–5 сегментов) | цепочка fold'ов; замена сегментированных мини-поверхностей одним чартом | ADMIT | defect <= 1e-4; ratio 1 ± 1e-3 | НОЛЬ junction-connectors внутри фаски (сегодняшний путь их создаёт — их исчезновение и есть ценность C) |
| F3 | Open quarter cylinder (16+ сегментов) | гладкая цилиндрическая развёртка | ADMIT | defect <= 1e-4; width err <= 1% | V вдоль дуги = длина дуги (texel density) |
| F4 | Open full cylinder, chart seam вне selected network | G7: один deterministic cut | ADMIT, `cut_count == 1` | cut.reason задан; cut не касается sites | детерминизм выбора cut при reversed enumeration |
| F5 | Densely tessellated cylindrical fillet (64+ сегментов, радиус ~ alpha) | масштаб + полосный бюджет | ADMIT | `support_triangle_count` ~ полоса, не patch | offset-safety: `abs(offset)*max_kappa` предупреждение при offset >= радиуса скругления (§4 workplan) |
| F6 | Две цепочки на одной кривой поверхности, коллизия при росте width | merge supports + Voronoi-конкуренция в одном чарте | ADMIT (один общий чарт) | supports объединены до solve | столкновение фронтов по биссектрисе; S1-стабильность швов после коллизии |
| N1 | Sphere cap (тесселяция подобрана: max defect >= 0.05 rad) | gate отказывает по G3 | **REJECT** `NON_DEVELOPABLE_SUPPORT` | — | component ушёл в legacy fallback; отчёт показывает причину; никакой частичной геометрии |
| N2 | Closed tube, selected chain коллинеарно лежит на единственном возможном cut | gate отказывает по G7 | **REJECT** `CUT_CROSSES_SELECTED_CHAIN` | — | fallback + reason в отчёте; транзверсальное пересечение для periodic domain разрешено |
| N3 | Полоса, самоперекрывающаяся в 2D (спираль/улитка с перехлёстом при заданном бюджете) | gate отказывает по G2 | **REJECT** `CHART_SELF_OVERLAP` | — | overlap_count > 0 зафиксирован в diagnostics |

Отрицательные фикстуры обязательны: admission без доказанного отказа —
это не gate, а комментарий.

---

## 6. Definition of done Tranche C

1. Все девять фикстур ведут себя по таблице; счётчики отказов N1–N3
   видны в отчёте.
2. Фаски/скругления/трубы обслуживаются ОДНИМ intrinsic-чартом:
   junction-connectors между facets исчезают (F2) — это главный
   видимый результат транша.
3. PLANAR fast path не изменился (differential walls.003/walls.001 —
   байт-в-байт).
4. S1-инварианты (`docs/decal_corner_bands.md` §5.1) действуют в чарте
   так же, как на плоскости: тесты стабильности проходят на F3.
5. Замер производительности F5 занесён в `docs/cftuv_decal_runtime.md`.

---

## 7. Что подтверждает пользователь

1. Бюджет искажения ширины **2%** (G3 = 0.02 rad, G4 = 0.02·N) — или
   строже (1% для близких камер — тогда 0.01/0.01).
2. Ноль пользовательских ручек в C: пороги — константы модуля.
3. Политика одного cut в v1 (`MULTI_CUT_REQUIRED` → fallback).
4. Состав отрицательных фикстур N1–N3.
5. Критерий №2 из DoD (исчезновение connectors на фаске) как главный
   визуальный признак приёмки транша.
