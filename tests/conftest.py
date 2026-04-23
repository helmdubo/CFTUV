from __future__ import annotations

import math
import sys
import types
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


if "mathutils" not in sys.modules:
    mathutils = types.ModuleType("mathutils")

    class Vector:
        def __init__(self, values=(0.0, 0.0, 0.0)):
            self._values = tuple(float(value) for value in values)

        def copy(self):
            return Vector(self._values)

        def __iter__(self):
            return iter(self._values)

        def __len__(self):
            return len(self._values)

        def __getitem__(self, index):
            return self._values[index]

        @property
        def x(self):
            return self._values[0] if len(self._values) > 0 else 0.0

        @property
        def y(self):
            return self._values[1] if len(self._values) > 1 else 0.0

        @property
        def z(self):
            return self._values[2] if len(self._values) > 2 else 0.0

        @property
        def length_squared(self):
            return sum(value * value for value in self._values)

        @property
        def length(self):
            return math.sqrt(self.length_squared)

        def dot(self, other):
            return sum(a * b for a, b in zip(self._values, other._values))

        def cross(self, other):
            ax, ay, az = self.x, self.y, self.z
            bx, by, bz = other.x, other.y, other.z
            return Vector((ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx))

        def normalized(self):
            if self.length <= 1e-12:
                return Vector(self._values)
            return Vector(tuple(value / self.length for value in self._values))

        def normalize(self):
            normalized = self.normalized()
            self._values = normalized._values
            return self

        def angle(self, other, fallback=0.0):
            denominator = max(self.length * other.length, 1e-12)
            if denominator <= 1e-12:
                return fallback
            cosine = max(-1.0, min(1.0, self.dot(other) / denominator))
            return math.acos(cosine)

        def lerp(self, other, factor):
            return self + (other - self) * float(factor)

        def __add__(self, other):
            size = max(len(self), len(other))
            return Vector(
                tuple(
                    (self._values[index] if index < len(self) else 0.0)
                    + (other._values[index] if index < len(other) else 0.0)
                    for index in range(size)
                )
            )

        def __sub__(self, other):
            size = max(len(self), len(other))
            return Vector(
                tuple(
                    (self._values[index] if index < len(self) else 0.0)
                    - (other._values[index] if index < len(other) else 0.0)
                    for index in range(size)
                )
            )

        def __mul__(self, scalar):
            return Vector(tuple(value * scalar for value in self._values))

        def __rmul__(self, scalar):
            return self.__mul__(scalar)

        def __truediv__(self, scalar):
            return Vector(tuple(value / scalar for value in self._values))

        def __repr__(self):
            return f"Vector({self._values!r})"

    class Quaternion:
        def __init__(self, axis=(0.0, 0.0, 1.0), angle=0.0):
            self.axis = Vector(axis)
            self.angle = float(angle)

        def copy(self):
            return Quaternion(self.axis, self.angle)

        def normalized(self):
            return self.copy()

        def __matmul__(self, other):
            return other.copy() if hasattr(other, "copy") else other

    mathutils.Vector = Vector
    mathutils.Quaternion = Quaternion
    sys.modules["mathutils"] = mathutils


if "bmesh" not in sys.modules:
    sys.modules["bmesh"] = types.ModuleType("bmesh")


if "bpy" not in sys.modules:
    sys.modules["bpy"] = types.ModuleType("bpy")
