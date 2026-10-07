"""Line coverage of the oracle files the native symbolic closure mirrors, over the calls a comparison makes (WP-S5).

The native closure is equal to the oracle only where a comparison RAN the oracle's code, so the number of executable lines of the mirrored modules that the compared runs reached, and
the lines they did not, are part of the evidence. This is a line monitor on `sys.monitoring` (Python 3.12 and later; the dev venv is 3.13): each line of a watched file reports once
and is then switched off, so a run costs next to nothing. Lines of function bodies only are counted: the module and class bodies run at import, before the monitor starts.

    with LineCoverage() as coverage:
        ...compared runs...
    print(coverage.format())
"""

from __future__ import annotations

import inspect
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WAVEFRONT = ROOT / "kernel" / "src" / "cftuv_envelope" / "wavefront"
#: the modules of the symbolic closure the port mirrors (`symbolic_runtime_commit.py` is the commit, WP-S6)
FILES = (
    "symbolic_overlay.py",
    "symbolic_component.py",
    "symbolic_f0_overlay.py",
    "symbolic_sparse_ports.py",
    "superlevel_fixed_point.py",
    "symbolic_edge_closure.py",
    "symbolic_split_endpoint.py",
    "symbolic_junction_contacts.py",
    "symbolic_junction_normalize.py",
    "symbolic_junction_fixed_point.py",
    "symbolic_edge_fixed_point.py",
    "symbolic_mixed_generation.py",
    "symbolic_superlevel_coordinator.py",
)

TOOL = 3


def executable_lines(path: Path) -> set:
    """The lines of the function bodies of a file that have bytecode (the `def` line itself is the module's)."""

    lines: set = set()

    def walk(each: types.CodeType) -> None:
        if each.co_flags & inspect.CO_OPTIMIZED:
            lines.update(line for _start, _end, line in each.co_lines() if line is not None and line != each.co_firstlineno)
        for const in each.co_consts:
            if isinstance(const, types.CodeType):
                walk(const)

    walk(compile(path.read_text(encoding="utf-8"), str(path), "exec"))
    return lines


class LineCoverage:
    """Collects the lines of `FILES` that run while the monitor is on."""

    def __init__(self, files=FILES) -> None:
        self.paths = {str((WAVEFRONT / name).resolve()): name for name in files}
        self.reached: dict = {name: set() for name in files}

    def __enter__(self) -> "LineCoverage":
        monitoring = sys.monitoring
        monitoring.use_tool_id(TOOL, "native-closure-coverage")
        monitoring.restart_events()
        monitoring.register_callback(TOOL, monitoring.events.LINE, self._line)
        monitoring.set_events(TOOL, monitoring.events.LINE)
        return self

    def __exit__(self, *_exception) -> None:
        monitoring = sys.monitoring
        monitoring.set_events(TOOL, 0)
        monitoring.register_callback(TOOL, monitoring.events.LINE, None)
        monitoring.free_tool_id(TOOL)

    def _line(self, code: types.CodeType, line: int):
        name = self.paths.get(str(Path(code.co_filename).resolve())) if code.co_filename.endswith(".py") else None
        if name is not None:
            self.reached[name].add(line)
        return sys.monitoring.DISABLE

    def report(self) -> dict:
        """`{file: (reached lines, executable lines, [missed lines])}` over the function bodies."""

        found = {}
        for name in self.reached:
            lines = executable_lines(WAVEFRONT / name)
            missed = sorted(lines - self.reached[name])
            found[name] = (len(lines) - len(missed), len(lines), missed)
        return found

    def format(self) -> str:
        rows = []
        covered = total = 0
        for name, (reached, executable, missed) in self.report().items():
            covered, total = covered + reached, total + executable
            ranges: list = []
            for line in missed:
                if ranges and line - ranges[-1][1] <= 1:
                    ranges[-1][1] = line
                else:
                    ranges.append([line, line])
            text = ", ".join(f"{start}" if start == end else f"{start}-{end}" for start, end in ranges)
            rows.append(f"{name:40} {reached:4}/{executable:4} {100 * reached / executable if executable else 100:5.1f}%  missed: {text or '-'}")
        rows.append(f"{'total':40} {covered:4}/{total:4} {100 * covered / total if total else 100:5.1f}%")
        return "\n".join(rows)

