"""Дамп вызовов `build_skeleton` в текст, который читает `native/cftuv-skeleton/examples/replay.rs` (замер и слепок ответа без Python).

    python tools/native_skeleton_replay_export.py field <каталог корпуса скелета> <out.txt>

Запись корпуса (`native_skeleton_export.py`, полевая или синтетическая, непроизводная) -> блок текста: полигон (петли с квадратами скоростей рёбер, веера), предел уровней `level_budget`,
потолок бюджета и флаг плотной гидратации. Состояние ДО у полевых записей холодное (память пуста, статьи нули), поэтому воспроизведению хватает этих полей; запись с тёплой памятью или с
ненулевыми статьями — отказ, а не молчаливое «почти».

Формат блока (то же самое пишет Blender-сторона, `dump_polygon`):

    polygon <имя> <меш> level_budget <n> cap <c> dense <0|1> patch <p>
    loop x,y;x,y;... | n/d;n/d;...        (точки, затем квадрат скорости каждого ребра)
    fan x,y | nx,ny,n/d;...               (вершина веера и его опоры)
    end

Запуск замера: `cargo run --release --example replay -p cftuv-skeleton -- <out.txt> 3 --out digests.txt`; два билда, давшие один `digests.txt`, отвечают на этих вызовах одинаково побитово
(результат, знаки, шесть статей, строка `superlevel`, журнал памяти).
"""

from __future__ import annotations

import sys
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))


def _rat(value) -> str:
    fraction = Fraction(value)
    return f"{fraction.numerator}/{fraction.denominator}"


def dump_polygon(name: str, mesh: str, polygon, level_budget: int, cap, dense: bool = False, patch=None) -> str:
    """Блок текста одного вызова (см. модуль)."""

    lines = [f"polygon {name} {mesh} level_budget {level_budget} cap {0 if cap is None else cap} dense {int(bool(dense))} patch {patch if patch is not None else -1}"]
    for loop in polygon.loops:
        points = ";".join(f"{x},{y}" for x, y in loop.points)
        lines.append(f"loop {points} | " + ";".join(_rat(speed) for speed in loop.edge_speeds_squared))
    for fan in polygon.vertex_fans:
        supports = ";".join(f"{support.normal_x},{support.normal_y},{_rat(support.speed_squared)}" for support in fan.supports)
        lines.append(f"fan {fan.point[0]},{fan.point[1]} | {supports}")
    lines.append("end")
    return "\n".join(lines) + "\n"


def export(root: Path, out: Path) -> int:
    import native_corpus as nc
    import native_skeleton_corpus as sc

    blocks = []
    for row in sc.rows_of(root, derived=False):
        record = sc.read(root, row)
        before = record.before()
        budget = before.budget
        if before.known_primes or before.factorization or before.squarefree or before.prime_support:
            raise SystemExit(f"{row['path']}: warm canonicalization memory before the call: the replay starts cold")
        if budget is not None and tuple(budget["articles"]) != (0,) * 6:
            raise SystemExit(f"{row['path']}: spent articles before the call: the replay starts with zero")
        call = nc.prepare_call(nc.OP_SKELETON, record.call_blob, before)
        blocks.append(dump_polygon(Path(row["path"]).stem, row["mesh"], call.args[0], call.kwargs["level_budget"], None if budget is None else budget["cap"], bool(call.kwargs["dense_hydration"]), row.get("patch_id")))
    out.write_text("".join(blocks), encoding="utf-8")
    return len(blocks)


def main(argv: list) -> int:
    if len(argv) != 3 or argv[0] != "field":
        print(__doc__)
        return 2
    print(f"NATIVE_SKELETON_REPLAY_EXPORT_OK {export(Path(argv[1]), Path(argv[2]))} calls -> {argv[2]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
