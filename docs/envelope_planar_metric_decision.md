# M-R0 — Planar Metric Contract Decision

Статус: `DECISION_COMPLETE_RESEARCH_ONLY`

Срез: `SESSION_M_R0_PLANAR_METRIC_CONTRACT_DECISION`

Ветка: `codex/m-r0-planar-metric-decision`

База: `c7bfed158802b767f7e94f6672cfb6b0181dae16`

Дата измерений: 2026-07-24

## Решение

### ReferenceMetricV2

Выбран **rational affine chart + exact Gram metric**.

`ReferenceMetricV2` должен хранить:

- exact origin `O`;
- два линейно независимых, но не обязанных быть нормированными вектора
  `A`, `B`;
- exact Gram matrix

  ```text
  G = [[A·A, A·B],
       [A·B, B·B]];
  ```

- exact affine coordinates boundary и barrier points;
- exact связь affine coordinates с host provenance.

Нормализация допустима только локально внутри закона, которому действительно
нужны физическая длина, угол или unit direction. Она не является условием
admission и не должна распространять radicals по sparse-domain boundary и
arrangement.

Algebraic orthonormal frame остаётся полезным differential research oracle и
возможной derived view, но не является authority контракта.

### RuntimeMetricV1

Выбрана архитектура **explicit certified binary64 fast path + exact fallback
to ReferenceMetricV2**. Это решение о модели, а не разрешение на production
integration.

Текущий статус:

```text
RUNTIMEMETRICV1_SELECTED
IMPLEMENTATION_STOPPED_NOT_READY_FOR_PRODUCTION
```

Runtime metric обязан:

- иметь named product policy для planar residual budget;
- использовать аналитические error bounds для filtered predicates;
- передавать любой uncertain sign, incidence, equality или contact в exact
  fallback;
- брать exact fallback facts из authoritative rational-affine chart, а не из
  повторной рационализации уже округлённого runtime frame;
- сохранять exact semantic topology, provenance, ownership и event identity;
- отдельно сообщать численную deviation геометрических координат;
- никогда не называться exact.

Spike-вариант C эти требования пока не выполняет: на двух rotation fixtures
он сохранил число output edges/loops/regions, но изменил semantic vertex
identity и atomic arrangement. Поэтому production runtime заблокирован.

## Что сравнивалось

Три metric-модели подключались к одному и тому же Blender-free evaluator:

```text
host 3D facts
→ metric/chart projection
→ один sparse outer loop
→ одна strip contribution
→ ExactSegmentArrangementBackend
→ physical 3D back-projection
→ topology/provenance/area comparison
```

Domain не строился из face triangles. Triangles использовались только как
contributors и для проверки retriangulation invariance. Arrangement,
provenance и semantic comparison одинаковы для A, B и C.

Каждая admitted model выполнялась пять раз. В отчёт вошли median timings,
`tracemalloc` peak, expression size, exact-sign batch, arrangement counters и
repeat digest. Для A/B равенство проверялось по exact physical coordinates,
exact area, topology и provenance, а не по синтаксическому виду выражений.

Исходники и результаты:

- `tools/export_envelope_metric_patch.py` — read-only Blender export host facts;
- `tools/benchmark_envelope_metric_models.py` — standalone research harness;
- `artifacts/envelope_metric_spike/results.json` — полный набор измерений;
- `artifacts/envelope_metric_spike/building_002_patch3.json` — отдельный receipt
  production fixture.

Production packages и текущие geometry contracts в этом срезе не изменялись.

Повторный запуск из committed building receipt:

```powershell
$env:PYTHONPATH = 'kernel/src'
C:\tmp\CFTUV-ec2-reference-venv\Scripts\python.exe `
  tools/benchmark_envelope_metric_models.py `
  --building-source artifacts/envelope_metric_spike/building_002_patch3.json `
  --results artifacts/envelope_metric_spike/results.json `
  --building-result artifacts/envelope_metric_spike/building_002_patch3.json `
  --iterations 5
```

Команда перезапишет timing fields новыми измерениями. Semantic gate и состав
fixtures должны остаться теми же.

## Диагноз текущего PlanarPatchFrameV1

Текущий exporter сначала рационализует host float coordinates, затем требует:

1. exact coplanarity;
2. рациональный unit normal;
3. exact binary64 round-trip для unit normal, basis и projected coordinates.

Это смешивает semantic planarity с удобством конкретного orthonormal
представления. Exact plane может быть отвергнут только потому, что длина его
normal содержит radical.

`building.002`, Patch 3 подтверждает именно этот случай:

```text
current V1: ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE
reason: host Patch 3 exact plane has no rational unit normal
```

Host normal приблизительно равен
`(0, 1, 7.563609756289225e-07)`. Mesh fingerprint до и после export совпал:

```text
7bb9661c13ee9e80060dca6bbcc4c90a6ac74b0e6537898652807141837b7671
```

Следовательно, rejection относится к frame representation, а не к отсутствию
плоскости и не к изменению source mesh.

## Fixture matrix

| Fixture | A | B | C | A/B exact semantic result | C vs B semantic topology/provenance |
|---|---:|---:|---:|---:|---:|
| axis-aligned plane | admitted | admitted | admitted | equal | equal |
| arbitrary rotated plane | admitted | admitted | admitted | equal | **mismatch** |
| 45° | admitted | admitted | admitted | equal | equal |
| irrational normal | admitted | admitted | admitted | equal | equal |
| micro-scale | admitted | admitted | admitted | equal | equal |
| huge coordinates | admitted | admitted | admitted | equal | equal |
| nearly planar outside budget | rejected | rejected | rejected | n/a | n/a |
| same plane, different triangulation | admitted | admitted | admitted | equal | equal |
| same plane after object rotation | admitted | admitted | admitted | equal | **mismatch** |
| `building.002`, Patch 3 | admitted | admitted | admitted | equal | equal |

Итог:

- A: 9 admitted, 1 rejected;
- B: 9 admitted, 1 rejected;
- C: 9 admitted, 1 rejected;
- exact A/B semantic equivalence: `true`;
- stable repeated digests: `true`;
- C semantic matches: 7;
- C semantic mismatches:
  `arbitrary_rotated_plane`, `same_plane_after_object_rotation`.

Nearly-planar fixture отвергнут явно:

```text
A/B: ALL_SOURCE_VERTICES_ON_EXACT_PLANE_FAILED
C: RUNTIME_PLANAR_RESIDUAL_BUDGET_EXCEEDED
   residual=0.0009999999444444492
   candidate_budget=4e-07
```

Этот candidate budget не является принятой product policy.

## Aggregate measurements

Значения времени — median по admitted fixtures, не end-user benchmark.

| Model | Frame build | Compile | RawCoverage | Median expression nodes | Max expression length | Max peak memory |
|---|---:|---:|---:|---:|---:|---:|
| A — algebraic orthonormal | 1.197 ms | 0.930 ms | 44.389 ms | 41 | 140 | 643,988 B |
| B — rational affine + Gram | 1.680 ms | 0.824 ms | 52.695 ms | 41 | 105 | 637,068 B |
| C — certified-float candidate | 0.283 ms | 0.811 ms | 57.992 ms | 41 | 383 | 754,860 B |

Эта aggregate-таблица сама по себе не доказывает превосходство B по времени:
B был медленнее A на простых fixtures и на 45°. Решение основано не на одной
median, а на локализации radicals, rational boundary coordinates и поведении
на сложных normals и реальном Patch 3.

### Expression growth

На 45°:

- A: 57 nodes, peak expression length 52, RawCoverage 311.386 ms;
- B: 93 nodes, peak expression length 85, RawCoverage 537.688 ms.

На irrational normal:

- A: 117 nodes, peak length 121, exact-sign batch 6.657 ms,
  RawCoverage 726.107 ms;
- B: 99 nodes, peak length 105, exact-sign batch 3.122 ms,
  RawCoverage 616.826 ms.

Вывод: affine + Gram не гарантирует меньшего выражения для любого отдельного
support law, но не требует radicals в самих domain coordinates и ограничивает
их область появления законами metric.

## `building.002`, Patch 3

Patch 3 содержит один source face, две triangles и outer loop:

```text
vertex ids: [1, 3, 15, 13]
physical edge ids: [12, 28, 24, 26]
```

| Metric | Frame | Compile | RawCoverage | Expression nodes | Peak length | Exact-sign batch |
|---|---:|---:|---:|---:|---:|---:|
| A | 1.581 ms | 2.945 ms | 163.955 ms | 77 | 140 | 4.216 ms |
| B | 2.014 ms | 0.917 ms | 54.428 ms | 41 | 76 | 0.625 ms |
| C | 0.418 ms | 1.138 ms | 57.992 ms | 41 | 51 | 1.047 ms |

На этом fixture B:

- сохранил все domain coordinates rational;
- дал exact semantic result, равный A;
- уменьшил RawCoverage примерно в 3.0 раза относительно A;
- уменьшил exact-sign batch примерно в 6.7 раза;
- не потребовал rational unit normal.

Синтаксические representation digests A и B различаются, хотя их physical
vertices, area, topology и provenance exact-равны. Поэтому долгосрочный
semantic digest не должен зависеть от выбора ортонормированного или affine
chart representation.

C на Patch 3 сохранил semantic topology/provenance и vertex identity. Max
vertex deviation равен `0.0`, absolute area deviation —
`6.661338147750939e-16`. При этом 20 из 34 filtered predicate checks потребовали
fallback. Один успешный asset case не отменяет rotation failures.

## Invariance

A и B сохранили invariant digest для:

- scale relation;
- rigid rotation relation;
- different triangulation relation.

C сохранил scale и retriangulation relation, но не rotation relation.
Причина наблюдаемого divergence: округлённая orthonormal projection сдвигает
strip endpoints относительно exact adjacent boundaries. Arrangement получает
дополнительные construction vertices: 9 atomic edges вместо 7. Простое
proximity snapping здесь недопустимо, поскольку оно реконструировало бы
topology и provenance после факта.

Для object rotation fixture использованы преобразованные local coordinates.
Не применённый Blender object transform не меняет local mesh coordinates и
потому сам по себе не тестирует metric contract, потребляющий local host facts.

## Candidate runtime policies

Spike использовал только исследовательские, непринятые policy IDs:

```text
RUNTIME_PLANAR_RESIDUAL_BUDGET_CANDIDATE_V1
status: RESEARCH_CANDIDATE_NOT_PRODUCT_POLICY

budget =
  max(1e-7 * max(planar_extent, 1e-3),
      64 * max_coordinate_ulp)
```

```text
RUNTIME_FILTERED_PREDICATE_BOUND_CANDIDATE_V1
status: RESEARCH_CANDIDATE_NOT_PRODUCT_POLICY

certified orientation if
  abs(orient2d) >
  64 * machine_epsilon * max_coordinate_delta_squared;
otherwise exact fallback
```

Они нужны, чтобы сделать spike проверяемым и показать места fallback. Они не
переносят старый hidden epsilon и не принимают новый tolerance. До отдельного
product-policy решения RuntimeMetricV1 не получает production admission.

## Stop conditions

### ReferenceMetricV2

Работа останавливается, если выполняется хотя бы одно условие:

1. `det(G) <= 0` или exact affine reconstruction host point невозможна.
2. Source points не exact-coplanar, а отдельный planarity/projection contract
   ещё не принят.
3. Нормализация снова становится admission requirement или распространяет
   radicals на все sparse boundary coordinates.
4. A/B differential oracle расходится по physical vertices, exact area,
   loops, regions, provenance или semantic digest.
5. Scale, rigid rotation или retriangulation меняют semantic result.
6. Domain silhouette начинает выводиться из face tessellation вместо
   BoundaryLoop/ChainUse/constraint authority.

### RuntimeMetricV1

Работа останавливается, если выполняется хотя бы одно условие:

1. Runtime расходится с ReferenceMetricV2 по topology, provenance, ownership,
   event identity или failure identity. Текущий spike уже остановлен этим
   условием на двух fixtures.
2. Predicate попадает внутрь error bound, но не выполняется exact fallback.
3. Exact fallback строится из округлённого runtime frame вместо authoritative
   ReferenceMetricV2 facts.
4. Residual budget не имеет принятого named product policy или не доказана его
   scale/micro/huge-coordinate устойчивость.
5. Numerical deviation превышает отдельно принятый output budget.
6. Contact/equality/incidence решаются tolerance или snapping.
7. Float path, его результаты или sidecar маркируются как exact.
8. Любой failure молча превращается в partial successful build.

### Production integration

Production integration запрещена до отдельного implementation slice, review
новых public contracts и прохождения полного differential corpus. M-R0 не
меняет `PlanarPatchFrameV1`, sparse-domain, Envelope, interaction или
arrangement semantics.

## Ограничения измерений

- C RawCoverage здесь включает exact arrangement над рационализированным
  binary64 state. Это differential research path, а не оценка скорости
  будущего production filtered backend.
- Пять повторов и малые fixtures достаточны для выявления expression growth и
  semantic divergence, но не являются hardware-independent performance SLA.
- Spike использует sparse-domain + strip evaluator, а не полный
  junction/interaction/ownership pipeline. Angle-certificate cost измерен
  отдельным exact expression surrogate.
- Timing values естественно меняются между запусками; machine-readable gate
  проверяет semantic outcomes и repeat stability внутри одного запуска.
- Existing `CanonicalGeometryDigest` не менялся. Research digest используется
  только этим spike.

## Legacy evidence boundary

Для product-policy контекста использован только factual document
`docs/envelope_legacy_planarity_chainuse_evidence.md` из Session L0. Legacy
geometry source не читался и не вызывался. Старые epsilon и fallback не
переносились в решение.
