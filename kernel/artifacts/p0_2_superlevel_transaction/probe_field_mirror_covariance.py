"""Полевые фигуры: исход и НАЗВАННЫЙ отказ обязаны совпасть в зеркале."""
import json, pickle, sys
from pathlib import Path
from cftuv_envelope.wavefront.polygon import (
    FanSupportV1, LoopV1, PolygonV1, VertexFanV1)
from cftuv_envelope.wavefront.skeleton import build_skeleton
from cftuv_envelope.wavefront.faces import build_faces


def mirrored(polygon):
    def flip_loop(loop):
        return LoopV1(
            tuple((-x, y) for x, y in loop.points), loop.speeds_squared
        ).oriented(counter_clockwise=loop.is_counter_clockwise)

    def flip_fan(fan):
        return VertexFanV1(
            (-fan.point[0], fan.point[1]),
            tuple(FanSupportV1(-i.normal_x, i.normal_y, i.speed_squared)
                  for i in reversed(fan.supports)))

    return PolygonV1(flip_loop(polygon.outer),
                     tuple(flip_loop(h) for h in polygon.holes),
                     tuple(flip_fan(f) for f in polygon.vertex_fans))


def axes(polygon):
    sk = build_skeleton(polygon)
    partition = build_faces(polygon, sk)
    return (sk.outcome.value, partition.outcome.value, sk.proof_status.value,
            tuple(sorted((k, v) for k, v in sk.counters
                         if "unresolvable_reason" in k)))


data = pickle.loads(Path(sys.argv[1]).read_bytes())
bad = 0
for name, polygon in data.items():
    left, right = axes(polygon), axes(mirrored(polygon))
    same = left == right
    bad += 0 if same else 1
    print(("СОВПАЛО " if same else "РАЗОШЛОСЬ "), name.replace("building_all_seams_", ""))
    print("     прямая ", left)
    if not same:
        print("     зеркало", right)
print("расхождений:", bad)
