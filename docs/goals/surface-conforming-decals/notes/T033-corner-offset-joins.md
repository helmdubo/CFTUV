# T033 — Constant-width corner offset joins

После stitching выбранных seam edges лента стала непрерывной, но shared wing
vertex всё ещё строился как `spine + averaged_direction * half_width`. На
повороте такая биссектриса сохраняла направление, но не расстояние до обоих
сегментов: крыло сужалось, а соседние quads визуально перекашивались.

Исправление вводит segment-local offset joins для каждого крыла:

- входящий и выходящий сегменты сохраняют собственные tangent/wing directions;
- пересечение двух offset-линий образует constant-width MITER;
- miter длиннее четырёх ширин крыла не создаёт шип и переходит в BEVEL;
- skew-линии на меняющейся геометрии также переходят в BEVEL;
- BEVEL хранит отдельные incoming/outgoing vertices и закрывается локальной
  гранью, не разрывая ручной scope и не превращая Junction в solve entity;
- открытые endpoints и прямые станции сохраняют одну общую wing vertex.

Чистые тесты покрывают прямой участок, точный 90-degree miter, острый bevel и
skew fallback. Blender 4.3.2 BMesh oracle дал для 90-degree run 9 vertices,
4 faces и minimum area `0.3971733153`; острый run дал 10 vertices, 5 faces и
minimum area `0.0034729661`. Blender 4.1.1 повторил результат 9/4 для MITER.
