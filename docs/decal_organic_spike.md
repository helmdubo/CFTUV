# Tranche E0.b — Organic chart measurement spike

Статус: **MEASURED / THRESHOLDS CONFIRMED**. Данные сняты до изменения
admission policy. Все размеры нормированы на `alpha = 1`; K = 20 станций,
rail owner-path измеряется альтернативным triangle-strip unfold.

| Fixture | Triangles | Defect, rad | Closure residual | Width error | Normal variation, rad | Overlap | Foldover |
|---|---:|---:|---:|---:|---:|---:|---:|
| E.1 sphere cap, R=12 alpha | 232 | 0.000282 | 0.001200 | 0.000849 (0.085%) | 0.066856 | 0 | 0 |
| E.2 bounded cliff | 220 | 0.000131 | 0.000263 | 0.000277 (0.028%) | 0.060454 | 258 | 0 |
| E.3 gentle saddle, R=12 alpha | 231 | 0.000278 | 0.001166 | 0.000841 (0.084%) | 0.066524 | 361 | 0 |
| E.4 intermediate dome, R=3 alpha | 300 | 0.003241 | 0.015154 | 0.023171 (2.317%) | 0.314867 | 0 | 0 |
| E.N1 tight sphere, R=2 alpha | 480 | 0.006318 | 0.064059 | 0.117726 (11.773%) | 0.502054 | 0 | 0 |
| E.N2 high-frequency crumple | 532 | 2.936567 | 3.062613 | 0.424473 (42.447%) | 1.856321 | 1499 | 562 |

## Сверка provisional thresholds

- `max_width_error_sampled <= budget`: E.1–E.3 имеют большой запас; E.4
  лежит между 0.02 и 0.05; E.N1 превышает hard max 0.10.
- `discrete_angle_defect <= 2 * budget`: положительные fixtures проходят;
  crumple отсекается с большим запасом.
- `max_station_normal_variation <= 0.35 rad`: E.4 проходит (`0.315`),
  tight sphere и crumple отклоняются.
- E.2/E.3 подтверждают назначение E1: метрика ширины мала, но current
  single-chart placement имеет self-overlap в margin. Relief cuts должны
  убрать overlap, не вводя переход внутри `alpha_budget`.
- Foldover отличается от общего overlap намеренно: у gentle saddle/cliff
  overlap есть, но нет инверсии/встречи source-нормалей >90°; crumple имеет
  562 таких пары.

Ни один provisional threshold не расходится с данными более чем вдвое.
Stop condition E0.b не активирован; E1/E2 разрешены без калибровочной правки.

## E1 feasibility audit — STOP CONDITION

Перед реализацией margin relief выполнен отдельный audit E.3 saddle. Его
sampled width error мал (`0.000841`), но single hinge chart содержит overlap
внутри materialization zone, а не только в support margin:

| Проверка | Лучший результат |
|---|---:|
| Baseline root | 361 overlaps |
| Перебор deterministic roots вдоль chain | 296 overlaps |
| Support обрезан до `distance <= alpha` | 291 overlaps |
| Длина saddle уменьшена в 16 раз | 4 overlaps |

Margin-кандидаты определяются source-distance не меньше `alpha`. На cliff
они являются dual bridges внешних ears; на saddle одиночный margin cut снижает
счётчик максимум с 361 до 356, но все remaining pairs лежат в core. Поэтому
достижение обязательного E2 gate `triangle_overlap_count == 0` требует одного
из двух запрещённых механизмов:

1. interior cuts/transition maps ближе `alpha` к chain (нарушение EP4;
   фактически E3 atlas);
2. conformal/global solver вместо hinge unroll (нарушение EP2).

Активирован stop condition Tranche E #2. Пороги и binary overlap gate не
ослабляются; E1/E2 не имплементируются без отдельного design decision.
