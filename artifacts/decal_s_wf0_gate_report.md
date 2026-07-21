# S-WF0 — Surface Wavefront research gate

Статус: **research завершён; рекомендация исполнителя — исход A**.
Решение A/B/C остаётся за пользователем. Production routing, `cftuv/` и
зависимости аддона не изменялись.

Исходная точка: `05290c6`. Методика заморожена до измерений в `ce64d13`;
harness зафиксирован в `755a276`. Финальный data run выполнен после этого
коммита.

## Короткий вывод

Гипотеза «текущий rail неизбежно запекает диагональ гладкой тесселяции» на
контрольной A/B-паре **не подтвердилась**: `CURRENT_RAIL_CHART`, FMM и MMP
инвариантны до машинного шума; current rail plan одинаков в обеих
диагонализациях. Причина видна и по коду/receipt: неграничные гладкие диагонали
не стали rail routes.

FMM дал сильный intrinsic distance oracle и заметно улучшил half-sphere:
width p95 `11.01% -> 1.45%` от `alpha_reference`. Но полного labeled/
topological контракта для R2 он не доказал: на planar L/T sequence edit равен
`4/2`, locus F1 — `0.807/0.690`, максимальная width-ошибка на отдельных
planar samples — `9.66%/14.64%`, source-s p95 на half-sphere — `10%` длины
source-network. Поэтому перевод поля из оракула в генератор событий (B), тем
более в backend материализации (C), данными не обоснован.

Heat как кандидат хуже: median width p95 `9.08%`, worst `20.29%`; на ключевой
A/B-паре смена диагонали дала width p95 delta `2.52%`, max `2.86%`, source-s
delta `8.33%` и locus F1 `0.667`.

## Что именно сравнивалось

- `CURRENT_RAIL_CHART`: production `compile_decal_rail_plan` плюс production
  hinge-unroll/width chart. При отказе штатной width-параметризации сохранён
  явно помеченный диагностический hinge fallback, а не скрытый admission.
- `HEAT` и `FMM`: geometry-central bindings из
  [potpourri3d](https://pypi.org/project/potpourri3d/), unsigned и только в
  owner component; label/source-s получены отдельными solves.
- `MMP_EXACT`: точная полиэдральная геодезика
  [pygeodesic](https://pypi.org/project/pygeodesic/) от дискретного множества
  source-вершин — эталон численных сравнений.
- `STRAIGHT_SKELETON_2D`:
  [py_straight_skeleton](https://pypi.org/project/py-straight-skeleton/) только
  для простых planar polygon-fronts.

Во всех field-методах join отсутствует: поле не выбирало MITER/BEVEL и не
создавало corner geometry. Растрового представления нет; события/loci читаются
по labeled scalar values и canonical mesh keys.

## Сводная таблица

Ошибки нормированы как описано в `research/s_wf0/README.md`. В таблице девять
runs: восемь именованных фикстур, у retriangulation две версии.

| Метод | median width p95 | worst width p95 | worst width max | worst source-s p95 | min locus F1 | median compile / extract |
|---|---:|---:|---:|---:|---:|---:|
| Current rail/chart | `~0` | `11.01%` | `23.30%` | `23.32%` | `0.560` | `380.97 / 122.24 ms` |
| Heat | `9.08%` | `20.29%` | `30.76%` | `10.00%` | `0.000` | `1.17 / 1.50 ms` |
| FMM | `~0` | `1.45%` | `14.64%` | `10.00%` | `0.690` | `0.22 / 4.45 ms` |
| MMP exact | `0` | `0` | `0` | `0` | `1.000` | `0.20 / 12.33 ms` |

Timings не являются взаимозаменяемым runtime benchmark: current включает
Python production IR/rail/chart compile, field-методы — native scalar solver,
MMP extract — повторные solves по source records. Они отвечают только на
compile/extract-вопрос внутри harness.

## Ключевая ретриангуляция

Обе версии используют одинаковые вершины и одинаковые planar quads
developable surface; меняется только диагональ. Поэтому здесь не смешаны
ретриангуляция и изменение embedded geometry.

| Метод | width p95 delta | width max delta | owner disagreement | source-s delta | event kinds | locus F1 |
|---|---:|---:|---:|---:|---:|---:|
| Current rail/chart | `~0` | `~0` | `0` | `~0` | equal | `1.000` |
| Heat | `2.52%` | `2.86%` | `0` | `8.33%` | equal | `0.667` |
| FMM | `0` | `0` | `0` | `0` | equal | `1.000` |
| MMP exact | `~0` | `~0` | `0` | `0` | equal | `1.000` |

Точный CSV: `artifacts/s_wf0/retriangulation.csv`.

## Фикстурные наблюдения

| Фикстура | Current | FMM против MMP | Вывод |
|---|---|---|---|
| planar L | width p95 `~0`; native rail events `0` (RP1) | p95 `~0`, max `9.66%`; edit `4`, F1 `0.807` | FMM distance хорош почти везде, vector topology не совпала |
| planar T/X | width p95 `~0`; native rail events `0` (RP1) | p95 `~0`, max `14.64%`; edit `2`, F1 `0.690` | branching — главный отрицательный event-кейс |
| concave owner | width p95 `~0` | p95 `~0`; edit `0`, F1 `0.881` | owner-boundary leak не обнаружен, locus ещё не точен |
| cylinder + seam | width p95 `~0`, но штатная width-map non-injective; fallback задекларирован | p95/max `~0`; edit `0`, F1 `1` | periodic cut численно восстановлен, double-cover не доказан материализацией |
| half_sphere | p95 `11.01%`, max `23.30%`, source-s p95 `23.32%`; штатная width-map non-injective | p95 `1.45%`, max `1.75%`, source-s p95 `10%`; edit `0`, F1 `1` | единственный сильный выигрыш FMM на гладкой кривизне |
| близкие листы | field на чужом component отсутствует | leak `0`, p95 `~0` | intrinsic owner isolation работает; ambient nearest не использован |
| retriangulation A/B | инвариантен | инвариантен | исходная тревога edge-diagonal на этой паре не воспроизвелась |
| fold рядом с smooth | p95 `~0`; native `DAM + ALPHA` | p95/max `~0`; edit `0`, F1 `1` | fold остаётся геометрическим rail, smooth diagonal не получил эту роль |

Owner error во всех строках равен нулю после исключения exact ties. Это
положительный, но слабый результат: labels на синтетике крупные и не заменяют
полную production provenance.

## Planar straight skeleton

- `planar_l`: три точных skeleton nodes на `alpha=1.5`.
- `concave_owner`: четыре nodes на `alpha=0.75`, два на `alpha=1.5`.
- `planar_tx`: честный `unsupported`, потому что используемый backend принимает
  simple polygon, а не T/X PSLG. Подмена distance field не делалась.

Результат поддерживает отдельный exact planar backend, но не разрешает полю
выбирать join и не доказывает surface-generalization skeleton-событий.

## Рекомендация A/B/C

### Рекомендация: A — поле только оракул

Оставить vector arrangement/rails носителем поведения и материализации. FMM
или MMP можно использовать в research/acceptance как G3-подобный admission
oracle для width/source-s и как differential сигнал на smooth curvature.
Никакой новой зависимости в аддон по этому результату не вносить.

### Почему не B

FMM достаточно хорош для distance query, но на L/T не воспроизвёл event/locus
contract MMP. Кроме того, tested API возвращает scalar samples; непрерывные
segment witnesses и векторная таблица событий из него не следуют. Передача ему
R2 events сейчас означала бы достроить новую topology system поверх неполных
данных.

### Почему не C

Ни Heat, ни FMM в этом spike не материализуют GeometryBatch, не доказывают
single cover, corner provenance и стабильную границу на branching network.
Скорость scalar solve не компенсирует отсутствие backend-контракта.

## Малые пивоты, сделанные внутри среза

1. После первого smoke исправлен mixed winding half-sphere; это дефект fixture,
   не изменение метода.
2. `alpha_reference` расширен до полного owner-domain там, где прежнее значение
   обрезало cut locus и превращало support boundary в ложный freeze.
3. Retriangulation event comparison разделён на порядок kinds и отдельную
   alpha-ошибку; raw JSON equality реагировала на машинный шум.
4. Labeled tie исключён из double-cover: tie — нормальный cut locus. В proxy
   double-cover входят chart overlaps, а не равенство witnesses.

Все четыре пивота внесены до финального data run и отражены в harness/tests.

## Риски

1. **Exact-источник дискретен.** MMP точен на полиэдральном mesh от source-
   вершин, но не является точным continuous segment solver. Planar L/T max
   outliers FMM чувствительны именно к этой дискретизации.
2. **Materialization не сравнивалась симметрично.** Field-бэкенды намеренно не
   создают production geometry. `double_cover_count=0` и leak `0` — результаты
   field/chart proxy, не доказательство single-cover итогового GeometryBatch.
3. **Current fallback — диагностика.** На cylinder/half-sphere штатная
   width-параметризация вернула `WIDTH_PARAMETERIZATION_NON_INJECTIVE`; числа
   current там получены явно помеченным hinge fallback и не являются скрытым
   production admission.
4. **PL event extractor дискретен.** Он полезен для одинакового cross-method
   сравнения, но exact planar skeleton nodes авторитетнее его лишних grid
   maxima. Нельзя переносить эти sampled events в R2 буквально.
5. **Synthetic scale.** Фикстуры закрывают утверждённые классы, но не заменяют
   полевые meshes. 0d.0 не применялся: это design spike без полевого hotfix и
   без изменения рендера.
6. **Timing локален.** Windows/Python 3.13 native wheels и три повтора не
   прогнозируют Blender modal latency.
7. **Straight-skeleton охват узок.** Текущий research dependency не решает
   T/X PSLG и не должен становиться addon dependency.

## Особое мнение / предложения

Проблема в формулировке B: «поле генерирует события» объединяет два разных
продукта — scalar distance query и vector event topology. Данные подтверждают
первый для FMM, но не второй. Если пользователь захочет вернуться к B, нужен
отдельный spike с continuous segment witnesses и явным event IR; расширять
этот harness до production-прототипа нельзя.

Кроме того, Heat и FMM нельзя дальше обсуждать как один backend `Heat/FMM`:
по ключевой ретриангуляции и width они дали качественно разные результаты.
Предлагаю зафиксировать Heat как отрицательный контроль, FMM/MMP — как oracle
family, current planar exact solution — без изменений.

## Воспроизводимость и артефакты

- Полный per-vertex receipt: `artifacts/s_wf0/results.json`, schema
  `cftuv.decal_s_wf0_results.v1`.
- Сводка: `artifacts/s_wf0/summary.csv`.
- Field/native/skeleton events: `artifacts/s_wf0/events.csv`.
- A/B: `artifacts/s_wf0/retriangulation.csv`.
- Harness и pins: `research/s_wf0/`.

Environment receipt: Python `3.13.1`, NumPy `2.3.2`, potpourri3d `1.4.0`,
pygeodesic `0.1.11`, py_straight_skeleton `0.1.0`, Windows 11. Команда:

```powershell
& "$env:TEMP\cftuv_s_wf0_env_313\Scripts\python.exe" `
  research/s_wf0/run_spike.py --output-dir artifacts/s_wf0
```
