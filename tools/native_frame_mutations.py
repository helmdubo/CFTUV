"""Мутации вызова `frame`: доказательство СОЛИДНОСТИ ответа порта допуска плоской рамки.

Порт обязан строить метрику ТОЛЬКО там, где построитель ядра строит ту же самую (первая ступень лестницы: точная плоскость на источнике, привязанном к решётке), и отказывать по имени
во всём остальном. Здесь записанный вызов (`tools/native_frame_corpus.py`) ломают десятками способов, и на каждом сравнивают эталон и порт (`native_frame_corpus.compare`): НАРУШЕНИЕ -
порт ответил, а эталон отказал, исключился, ответил иначе или сдвинул другие счётчики; порт поднял не отказ. Нарушений обязано быть ноль; отказ порта там, где эталон отвечает, - цена.

Мутации: координата одной вершины сдвинута на 10^-15 ... 10^-3 и на одну единицу последнего разряда, вершина поднята из плоскости, весь патч растянут, сдвинут, повёрнут, отражён,
оси переставлены, вершины слиты и выстроены в линию, привязка к грубой решётке; грань развёрнута, повёрнута, выброшена, повторена, отдана чужому патчу, её рёбра и нормаль подменены;
владелец, имена вершин, недоступная позиция, неточные типы (целое, логическое, `Fraction`, `-0.0`, NaN, бесконечность, субнормальное, 10^300), законы привязки и масштаба, контейнеры.

    python tools/native_frame_mutations.py <файл вызовов | зерно | каталог корпуса задач> [--name ЧАСТЬ_ИМЕНИ] [--sample 60] [--seed 1]

Код выхода 1 - есть нарушения.
"""

from __future__ import annotations

import dataclasses
import math
import random
import sys
import time
from collections import Counter
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import native_frame_corpus as corpus  # noqa: E402


def make(cls, **values):
    """Запись класса ядра БЕЗ `__post_init__` (как строит родная сторона): значение, которого конструктор не пропустил бы, - тоже вход."""

    item = object.__new__(cls)
    for field in dataclasses.fields(cls):
        object.__setattr__(item, field.name, values[field.name])
    return item


def _replace(record, **changes):
    return make(type(record), **{field.name: changes.get(field.name, getattr(record, field.name)) for field in dataclasses.fields(record)})


def _point(vertex):
    position = vertex.position
    return (position.x, position.y, position.z) if hasattr(position, "x") else None


def _moved(vertex, x, y, z):
    return _replace(vertex, position=_replace(vertex.position, x=x, y=y, z=z))


def _edges_reversed(face):
    count = len(face.edge_cycle)
    return tuple(face.edge_cycle[(count - 2 - index) % count] for index in range(count))


def mutants(arguments: dict, rng: random.Random):
    """`(метка, аргументы)` мутантов одного вызова."""

    import cftuv_envelope as kernel
    from cftuv_envelope.contracts.metric import GridScaleLawV1

    vertex_kind, face_kind = type(arguments["source_vertices"]), type(arguments["source_faces"])
    vertices = sorted(arguments["source_vertices"], key=lambda item: item.vertex_id.value)
    faces = sorted(arguments["source_faces"], key=lambda item: item.face_id.value)
    placed = [vertex for vertex in vertices if _point(vertex) is not None]
    if len(placed) < 3:
        return

    def with_vertices(items):
        return {**arguments, "source_vertices": vertex_kind(items)}

    def with_faces(items):
        return {**arguments, "source_faces": face_kind(items)}

    def one(vertex, x, y, z):
        return with_vertices([_moved(item, x, y, z) if item is vertex else item for item in vertices])

    def every(transform):
        return with_vertices([_moved(item, *transform(*_point(item))) if _point(item) is not None else item for item in vertices])

    target = rng.choice(placed)
    x, y, z = _point(target)
    axis = rng.randrange(3)

    def on_axis(value):
        point = [x, y, z]
        point[axis] = value
        return point

    for epsilon in (1e-15, 1e-12, 1e-9, 1e-7, 1e-5, 1e-3):
        base = (x, y, z)[axis]
        yield f"NUDGE {epsilon:g}", one(target, *on_axis(base * (1 + epsilon) if base else epsilon))
    yield "ULP up", one(target, *on_axis(math.nextafter((x, y, z)[axis], math.inf)))
    yield "ULP down", one(target, *on_axis(math.nextafter((x, y, z)[axis], -math.inf)))
    a, b, c = (_point(item) for item in placed[:3])
    normal = ((b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]), (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]), (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    length = math.sqrt(sum(value * value for value in normal)) or 1.0
    for epsilon in (1e-9, 1e-6, 1e-4, 1e-2):
        yield f"LIFT {epsilon:g}", one(target, *(value + epsilon * n / length for value, n in zip((x, y, z), normal)))
    for factor in (2.0, 0.5, 3.0, 1e3, 1e-3, 1e6, 1e-6):
        yield f"SCALE {factor:g}", every(lambda u, v, w, k=factor: (u * k, v * k, w * k))
    for offset in (0.1, 1.0, 1e3, 1e6):
        yield f"TRANSLATE {offset:g}", every(lambda u, v, w, d=offset: (u + d, v - d, w + d))
    yield "AXES yzx", every(lambda u, v, w: (v, w, u))
    yield "MIRROR x", every(lambda u, v, w: (-u, v, w))
    for angle in (0.1, math.pi / 4, math.pi / 2, math.pi):
        cosine, sine = math.cos(angle), math.sin(angle)
        yield f"ROTATE {angle:.3f}", every(lambda u, v, w, c=cosine, s=sine: (c * u - s * v, s * u + c * v, w))
    for step in (64, 1024):
        yield f"COARSE 1/{step}", every(lambda u, v, w, q=step: (round(u * q) / q, round(v * q) / q, round(w * q) / q))
    other = rng.choice([item for item in placed if item is not target])
    yield "COLLAPSE", one(target, *_point(other))
    yield "COLLINEAR", every(lambda u, v, w: (u + v + w, 2 * (u + v + w), 0.0))
    yield "FLAT z", every(lambda u, v, w: (u, v, 0.0))
    for label, value in (("INT", int(round(x))), ("BOOL", True), ("FRACTION", Fraction(x)), ("NEGATIVE ZERO", -0.0), ("NAN", math.nan), ("INF", math.inf), ("SUBNORMAL", 5e-324), ("HUGE", 1e300)):
        yield f"VALUE {label}", one(target, value, y, z)
    unavailable = getattr(kernel, "UnavailableSourcePositionV1", None)
    reasons = list(getattr(kernel, "SurfaceCoordinateUnavailableReason", ()))
    if unavailable is not None and reasons:
        yield "UNAVAILABLE", with_vertices([make(type(item), vertex_id=item.vertex_id, position=make(unavailable, reason=reasons[0])) if item is target else item for item in vertices])
    yield "DROP vertex", with_vertices([item for item in vertices if item is not target])
    yield "EXTRA vertex", with_vertices([*vertices, make(type(target), vertex_id=type(target.vertex_id)(target.vertex_id.value + ":extra"), position=_replace(target.position, x=x + 7.0, y=y + 3.0, z=z - 5.0))])
    names = sorted(item.vertex_id.value for item in vertices)
    renamed = {old: type(target.vertex_id)(new) for old, new in zip(names, reversed(names))}
    yield "RENAME vertices", {
        **arguments,
        "source_vertices": vertex_kind(_replace(item, vertex_id=renamed[item.vertex_id.value]) for item in vertices),
    }
    face = rng.choice(faces)

    def face_swapped(new_faces):
        return with_faces([item for item in faces if item is not face] + list(new_faces))

    yield "REVERSE face", face_swapped([_replace(face, vertex_cycle=tuple(reversed(face.vertex_cycle)), edge_cycle=_edges_reversed(face))])
    yield "REVERSE face cycle only", face_swapped([_replace(face, vertex_cycle=tuple(reversed(face.vertex_cycle)))])
    yield "ROTATE face cycle", face_swapped([_replace(face, vertex_cycle=face.vertex_cycle[1:] + face.vertex_cycle[:1], edge_cycle=face.edge_cycle[1:] + face.edge_cycle[:1])])
    yield "REVERSE every face", with_faces([_replace(item, vertex_cycle=tuple(reversed(item.vertex_cycle)), edge_cycle=_edges_reversed(item)) for item in faces])
    if len(faces) > 1:
        yield "DROP face", face_swapped([])
    yield "DUPLICATE face", face_swapped([face, _replace(face, face_id=type(face.face_id)(face.face_id.value + ":copy"))])
    yield "FOREIGN face", face_swapped([_replace(face, patch_id=type(face.patch_id)(face.patch_id.value + ":foreign"))])
    yield "RENAME edge", face_swapped([_replace(face, edge_cycle=(type(face.edge_cycle[0])(face.edge_cycle[0].value + ":split"), *face.edge_cycle[1:]))])
    yield "SHARED edge", face_swapped([_replace(face, edge_cycle=(face.edge_cycle[-1], *face.edge_cycle[1:]))])
    yield "NORMAL flipped", face_swapped([_replace(face, polygon_normal=_replace(face.polygon_normal, **{name: -getattr(face.polygon_normal, name) for name in ("x", "y", "z")}))])
    yield "OWNER foreign", {**arguments, "owner_patch_id": type(arguments["owner_patch_id"])(arguments["owner_patch_id"].value + ":foreign")}
    yield "LINEAGE empty", {**arguments, "source_lineage": frozenset()}
    for law in kernel.GridSnappingLawV1:
        if law is not arguments.get("grid_policy"):
            yield f"GRID POLICY {law.value}", {**arguments, "grid_policy": law}
    for law in GridScaleLawV1:
        yield f"SCALE LAW {law.value}", {**arguments, "grid_scale_law": law}
    yield "CONTAINER tuple", {**arguments, "source_vertices": tuple(vertices), "source_faces": tuple(faces)}
    yield "CONTAINER list", {**arguments, "source_vertices": list(vertices)}


def run(native, calls: list, seed: int = 1, quiet: bool = False) -> dict:
    """Все мутанты вызовов; `{originals, mutants, equal, declined, violations, labels, ...}`."""

    rng = random.Random(seed)
    summary = {"originals": 0, "mutants": 0, "equal": 0, "declined": Counter(), "oracle": Counter(), "violations": [], "labels": Counter(), "accepted_labels": Counter(), "seconds": 0.0}
    started = time.perf_counter()
    for label, arguments in calls:
        outcome, detail = corpus.compare(native, arguments)
        summary["originals"] += 1
        if outcome == corpus.VIOLATION:
            summary["violations"].append((label, "original", detail))
        for name, mutant in mutants(arguments, rng):
            kind = name.split(" ")[0]
            outcome, detail = corpus.compare(native, mutant)
            summary["mutants"] += 1
            summary["labels"][kind] += 1
            if outcome == corpus.EQUAL:
                summary["equal"] += 1
                summary["accepted_labels"][kind] += 1
                summary["oracle"]["answered"] += 1
            elif outcome == corpus.DECLINED:
                summary["declined"][corpus.reason_of(detail)] += 1
                raised = corpus.oracle(mutant)[1]
                summary["oracle"]["raised " + raised.split(":")[0] if raised else "answered"] += 1
            else:
                summary["violations"].append((label, name, detail))
        if not quiet:
            print(f"  {summary['originals']} calls, {summary['mutants']} mutants, {len(summary['violations'])} violations", flush=True)
    summary["seconds"] = time.perf_counter() - started
    return summary


def describe(summary: dict) -> str:
    lines = [
        f"originals {summary['originals']} mutants {summary['mutants']}: the port answered {summary['equal']} (equal to the oracle), declined {sum(summary['declined'].values())} {dict(summary['declined'])}",
        f"the oracle on the mutants: {dict(summary['oracle'])}",
        f"answered by mutation: {dict(summary['accepted_labels'])}",
        f"VIOLATIONS {len(summary['violations'])} in {summary['seconds']:.1f} s",
    ]
    lines.extend(f"  {str(item)[:700]}" for item in summary["violations"][:10])
    return "\n".join(lines)


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Мутации вызова frame против построителя ядра")
    parser.add_argument("source", help="файл вызовов (`native_frame_corpus.py record`), зерно либо каталог корпуса задач")
    parser.add_argument("--name", default="")
    parser.add_argument("--sample", type=int, default=60, help="сколько вызовов взять (случайно)")
    parser.add_argument("--seed", type=int, default=1)
    arguments = parser.parse_args(argv)
    import cftuv_native

    calls = corpus.load(arguments.source, arguments.name)
    rng = random.Random(arguments.seed)
    chosen = rng.sample(calls, min(arguments.sample, len(calls)))
    print(f"{len(calls)} calls, {len(chosen)} sampled", flush=True)
    summary = run(cftuv_native, chosen, arguments.seed, quiet=True)
    print(describe(summary))
    return 1 if summary["violations"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
