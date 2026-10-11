"""Deterministic file-level shards of a pytest directory, and the proof that the shards are the whole suite.

CI runs a long suite as N parallel jobs; each job runs the files of ONE shard. A shard is a set of test FILES (never a subset of one
file's tests), chosen by a pure function of the file names and a weights table, so that the same checkout gives the same shards in every
job and the shards together are exactly the files `--glob` finds under `--root`:

    python tools/shard_tests.py list  --root tests --shard 2/4 [--glob "test_*.py"] [--weights tools/shard_weights.json]
    python tools/shard_tests.py check --root tests --shards 4   [--glob "test_*.py"] [--weights tools/shard_weights.json]
    python tools/shard_tests.py plan  --root tests --shards 4   [...]            # the estimated load of each shard
    python tools/shard_tests.py weights --junit report.xml | --durations-log pytest.log     # a new weights table on stdout

`list` prints one path per line (`<root>/<file>`), exit 3 when the shard is empty (an empty argument list would make pytest run the WHOLE
directory in every job). `check` fails (exit 1) unless the shards are pairwise disjoint, none is empty and their union is the full file list:
the property the CI relies on, so it is run in every shard job before the tests and is a unit test. The weights table (seconds per file)
only balances the shards: a file missing from it gets `default_seconds`, a key without a file is ignored, and neither can lose a test.

Stdlib only: the script is part of the extracted kernel (`kernel/tools`) and runs in the standalone copy.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ElementTree
from pathlib import Path

WEIGHTS_SCHEMA = "cftuv.ci-shard-weights.v1"
DEFAULT_SECONDS = 2.0
EXIT_EMPTY_SHARD = 3


def discover(root: Path, pattern: str = "test_*.py") -> list[str]:
    """Paths (POSIX, relative to `root`) of every file matching `pattern`, sorted; `__pycache__` is not a test directory."""

    root = Path(root)
    found = [path.relative_to(root).as_posix() for path in root.rglob(pattern) if path.is_file() and "__pycache__" not in path.parts]
    return sorted(found)


def load_weights(path: Path | None) -> tuple[float, dict[str, float]]:
    """`(default_seconds, {relative path: seconds})`; no table is a valid input (every file weighs the same)."""

    if path is None or not Path(path).is_file():
        return DEFAULT_SECONDS, {}
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    if document.get("schema") != WEIGHTS_SCHEMA:
        raise SystemExit(f"{path}: schema {document.get('schema')!r}, expected {WEIGHTS_SCHEMA!r}")
    return float(document.get("default_seconds", DEFAULT_SECONDS)), {name: float(value) for name, value in document["seconds"].items()}


def assign(files: list[str], default: float, weights: dict[str, float], shards: int) -> list[list[str]]:
    """Longest-processing-time-first: heaviest file to the lightest shard; ties by name, then by shard number (a pure function of the inputs)."""

    if shards < 1:
        raise ValueError(f"shards must be at least 1, not {shards}")
    bins: list[list[str]] = [[] for _ in range(shards)]
    load = [0.0] * shards
    for name in sorted(files, key=lambda item: (-weights.get(item, default), item)):
        target = min(range(shards), key=lambda number: (load[number], number))
        bins[target].append(name)
        load[target] += weights.get(name, default)
    return [sorted(group) for group in bins]


def parse_shard(text: str) -> tuple[int, int]:
    """`"2/4"` -> `(2, 4)`: the shard number (1-based) and the shard count, in ONE value so the two cannot disagree in a CI matrix."""

    match = re.fullmatch(r"(\d+)/(\d+)", text.strip())
    if match is None or not 1 <= int(match.group(1)) <= int(match.group(2)):
        raise SystemExit(f"shard must look like `2/4` with 1 <= index <= count, not {text!r}")
    return int(match.group(1)), int(match.group(2))


def shard_of(files: list[str], default: float, weights: dict[str, float], index: int, total: int) -> list[str]:
    return assign(files, default, weights, total)[index - 1]


def problems(files: list[str], groups: list[list[str]]) -> list[str]:
    """What is wrong with `groups` as shards of `files` (empty list: they are the whole suite, disjoint, none empty)."""

    found = []
    seen: dict[str, int] = {}
    for number, group in enumerate(groups, 1):
        if not group:
            found.append(f"shard {number}/{len(groups)} is empty")
        for name in group:
            if name in seen:
                found.append(f"{name} is in shard {seen[name]} and in shard {number}")
            seen[name] = number
    lost = sorted(set(files) - set(seen))
    stray = sorted(set(seen) - set(files))
    if lost:
        found.append(f"files in no shard: {lost}")
    if stray:
        found.append(f"shard members that are not files of the suite: {stray}")
    if not files:
        found.append("the suite has no files: a glob that finds nothing would pass every shard vacuously")
    return found


def weights_from_junit(text: str) -> dict[str, float]:
    """Seconds per test file from a pytest `--junitxml` report (the sum of its testcases; `classname` is the dotted module path)."""

    seconds: dict[str, float] = {}
    for case in ElementTree.fromstring(text).iter("testcase"):
        module = case.get("classname", "").split(".")
        parts = [part for part in module if re.fullmatch(r"test_\w+", part)]
        if parts:
            seconds[parts[0] + ".py"] = seconds.get(parts[0] + ".py", 0.0) + float(case.get("time", 0.0))
    return seconds


_DURATION = re.compile(r"^\s*(?:\S+Z\s+)?(\d+(?:\.\d+)?)s\s+(?:call|setup|teardown)\s+(?:\S*/)?(test_\w+\.py)::", re.MULTILINE)


def weights_from_durations_log(text: str) -> dict[str, float]:
    """Seconds per test file from `--durations` lines (`12.30s call tests/test_x.py::test_y`); only the tests the report lists are counted."""

    seconds: dict[str, float] = {}
    for value, name in _DURATION.findall(text):
        seconds[name] = seconds.get(name, 0.0) + float(value)
    return seconds


def weights_document(seconds: dict[str, float], default: float) -> str:
    rounded = {name: round(value, 1) for name, value in sorted(seconds.items())}
    return json.dumps({"schema": WEIGHTS_SCHEMA, "default_seconds": default, "seconds": rounded}, indent=1, sort_keys=False) + "\n"


def plan_lines(groups: list[list[str]], default: float, weights: dict[str, float]) -> list[str]:
    return [f"shard {number}/{len(groups)}: {len(group)} files, estimated {sum(weights.get(name, default) for name in group):.0f} s" for number, group in enumerate(groups, 1)]


def _suite_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--root", type=Path, required=True, help="the directory the test files live in")
    parser.add_argument("--glob", default="test_*.py", help="which files are the suite (default test_*.py)")
    parser.add_argument("--weights", type=Path, default=None, help="seconds per file, balances the shards only")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)
    listing = commands.add_parser("list")
    _suite_arguments(listing)
    listing.add_argument("--shard", required=True, help="`index/total`, e.g. 2/4")
    for name in ("check", "plan"):
        sub = commands.add_parser(name)
        _suite_arguments(sub)
        sub.add_argument("--shards", type=int, required=True)
    table = commands.add_parser("weights")
    table.add_argument("--junit", type=Path, default=None)
    table.add_argument("--durations-log", type=Path, default=None)
    table.add_argument("--default-seconds", type=float, default=DEFAULT_SECONDS)
    arguments = parser.parse_args(argv)

    if arguments.command == "weights":
        if (arguments.junit is None) == (arguments.durations_log is None):
            parser.error("name exactly one of --junit and --durations-log")
        if arguments.junit is not None:
            seconds = weights_from_junit(arguments.junit.read_text(encoding="utf-8"))
        else:
            seconds = weights_from_durations_log(arguments.durations_log.read_text(encoding="utf-8", errors="replace"))
        sys.stdout.write(weights_document(seconds, arguments.default_seconds))
        return 0

    files = discover(arguments.root, arguments.glob)
    default, weights = load_weights(arguments.weights)
    if arguments.command == "list":
        index, total = parse_shard(arguments.shard)
        groups = assign(files, default, weights, total)
        defects = problems(files, groups)
        if defects:
            print("SHARDS_NOT_THE_SUITE " + "; ".join(defects), file=sys.stderr)
            return EXIT_EMPTY_SHARD if not groups[index - 1] else 1
        # LF only, whatever the platform: a text-mode stdout redirected to a file on Windows would end each path with CR LF, and the CR would become part of the path in a shell `mapfile`
        sys.stdout.buffer.write("".join(f"{(arguments.root / name).as_posix()}\n" for name in groups[index - 1]).encode("utf-8"))
        return 0
    groups = assign(files, default, weights, arguments.shards)
    for line in plan_lines(groups, default, weights):
        print(line)
    if arguments.command == "plan":
        return 0
    defects = problems(files, groups)
    for defect in defects:
        print("SHARDS_NOT_THE_SUITE " + defect)
    if not defects:
        print(f"shards-are-the-suite: OK ({len(files)} files in {arguments.shards} shards)")
    return 1 if defects else 0


if __name__ == "__main__":
    raise SystemExit(main())
