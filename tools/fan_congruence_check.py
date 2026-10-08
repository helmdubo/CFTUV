"""Совместимый CLI; единственная реализация живёт вместе с извлекаемым ядром."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

_PATH = Path(__file__).resolve().parents[1] / "kernel" / "tools" / "fan_congruence_check.py"
_SPEC = importlib.util.spec_from_file_location("_kernel_fan_congruence_check", _PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _MODULE
_SPEC.loader.exec_module(_MODULE)
globals().update({name: value for name, value in vars(_MODULE).items() if not name.startswith("__")})

if __name__ == "__main__":
    raise SystemExit(_MODULE.main())
