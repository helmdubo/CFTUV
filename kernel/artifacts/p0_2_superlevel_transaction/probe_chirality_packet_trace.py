"""Почему вырожденный порт дожил до F0: что было в пакете и что отказано."""
import sys
from cftuv_envelope.wavefront import skeleton as SK
from cftuv_envelope.wavefront import superlevel as base
from cftuv_envelope.wavefront.polygon import LoopV1, PolygonV1
from cftuv_envelope.wavefront.skeleton import build_skeleton
from weighted_wall_differential_cases import weighted_wall_differential_corpus

cases = {c.name: c.polygon for c in weighted_wall_differential_corpus()}

def mirrored(polygon):
    def flip(loop):
        return LoopV1(tuple((-x, y) for x, y in loop.points), loop.speeds_squared)
    return PolygonV1.build(flip(polygon.outer), tuple(map(flip, polygon.holes)))

orig_apply = SK._Builder._apply_level
orig_refuse = SK._Builder._refuse

def apply(self, level):
    print("   ПАКЕТ t=", self.now.canonical().dividend, "событий", len(level),
          [(e.kind.value, e.vertex, e.peer, e.edge) for e in level])
    live = []
    for v in self.vertices:
        if not v.alive:
            continue
        nxt = self.vertices[v.next]
        if not nxt.alive:
            continue
        p0 = self._position(v, self.now)
        p1 = self._position(nxt, self.now)
        zero = (p0 is not None and p1 is not None
                and (p0.x - p1.x).is_zero and (p0.y - p1.y).is_zero)
        if zero:
            live.append((v.ident, nxt.ident, self.edges[v.next_edge].key))
    if live:
        print("      НУЛЕВЫЕ ПРОЛЁТЫ в prestate:", live)
    return orig_apply(self, level)

def refuse(self, reason, **fields):
    print("      ОТКАЗ", reason.value if hasattr(reason, "value") else reason,
          {k: v for k, v in fields.items() if k == "vertex_ids"})
    return orig_refuse(self, reason, **fields)

SK._Builder._apply_level = apply
SK._Builder._refuse = refuse
name = sys.argv[1]
for tag, polygon in (("=== ПРЯМАЯ", cases[name]), ("=== ЗЕРКАЛО", mirrored(cases[name]))):
    print(tag)
    sk = build_skeleton(polygon)
    print("   ", sk.outcome.value)
