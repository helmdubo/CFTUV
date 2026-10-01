"""Сравнение двух дампов GP: побайтово, затем по первому расхождению.

    python compare_dumps.py <до.json> <после.json>

Выход 0 — дампы идентичны (и побайтово, и как деревья), 1 — нет; при
расхождении печатается путь первого различия, а не `!=` на мегабайте JSON.
"""

from __future__ import annotations

import hashlib
import json
import sys


def first_difference(left, right, path="dump"):
    if type(left) is not type(right):
        return f"{path}: {left!r} != {right!r}"
    if isinstance(left, dict):
        for key in sorted(set(left) | set(right)):
            if key not in left or key not in right:
                return f"{path}.{key}: present on one side only"
            found = first_difference(left[key], right[key], f"{path}.{key}")
            if found:
                return found
        return None
    if isinstance(left, list):
        if len(left) != len(right):
            return f"{path}: length {len(left)} != {len(right)}"
        for index, (a, b) in enumerate(zip(left, right)):
            found = first_difference(a, b, f"{path}[{index}]")
            if found:
                return found
        return None
    return None if left == right else f"{path}: {left!r} != {right!r}"


def main(argv):
    before_path, after_path = argv[1], argv[2]
    before = open(before_path, "rb").read()
    after = open(after_path, "rb").read()
    print(
        f"before: {len(before)} bytes sha256={hashlib.sha256(before).hexdigest()}"
    )
    print(
        f"after:  {len(after)} bytes sha256={hashlib.sha256(after).hexdigest()}"
    )
    if before == after:
        dump = json.loads(before)
        strokes = sum(
            len(frame["strokes"])
            for layer in dump["layers"]
            for frame in layer["frames"]
        )
        attributes = sorted(
            {
                name
                for layer in dump["layers"]
                for frame in layer["frames"]
                for name in frame.get("attributes", {})
            }
        )
        print(
            f"IDENTICAL: layers={len(dump['layers'])} strokes={strokes} "
            f"materials={len(dump['materials'])} attributes={attributes}"
        )
        return 0
    difference = first_difference(json.loads(before), json.loads(after))
    print(f"DIFFERENT: {difference}")
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
