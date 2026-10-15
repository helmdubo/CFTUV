"""Совместимый CLI/import вход; единственный владелец helper находится в kernel/tools."""
from pathlib import Path as _ForwardPath
import sys as _ForwardSys

_forward = _ForwardPath(__file__).resolve().parents[1] / "kernel" / "tools" / "density_scalar_v2_diff.py"
if str(_forward.parent) not in _ForwardSys.path:
    _ForwardSys.path.insert(0, str(_forward.parent))
# Общий namespace сохраняет private API, custom importlib alias и monkeypatch globals.
__file__ = str(_forward)
exec(compile(_forward.read_bytes(), __file__, "exec"), globals())
