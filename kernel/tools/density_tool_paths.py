"""Пути исследовательских Density tools отдельно от установленного ядра."""

from pathlib import Path
import os
import sys

KERNEL_ROOT = Path(__file__).resolve().parents[1]
TOOLS_ROOT = KERNEL_ROOT / "tools"
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))
if ("cftuv_envelope" not in sys.modules and not sys.flags.isolated
        and os.environ.get("CFTUV_TEST_REQUIRE_WHEEL") != "1"):
    source = KERNEL_ROOT / "src"
    if source.is_dir() and str(source) not in sys.path:
        sys.path.insert(0, str(source))

import cftuv_envelope  # noqa: E402

PACKAGE_ROOT = Path(cftuv_envelope.__file__).resolve().parent


def kernel_child(code, *arguments, leaf_package=None):
    """Команда дочерней проверки: тот же пакет, -I в wheel mode, origin до/после."""
    flags = ["-I"] if sys.flags.isolated or os.environ.get("CFTUV_TEST_REQUIRE_WHEEL") == "1" else []
    environment = dict(os.environ, PYTHONPATH=os.pathsep.join((str(PACKAGE_ROOT.parent), *sys.path)))
    guard = '''
import sys
from pathlib import Path as _KernelOriginPath

def _check_kernel_origin():
    expected = _KernelOriginPath(sys.argv[1]).resolve() / "cftuv_envelope"
    for name, module in tuple(sys.modules.items()):
        normal = name == "cftuv_envelope" or name.startswith("cftuv_envelope.")
        leaf = bool(_density_leaf_package) and name.startswith(_density_leaf_package + ".")
        if not (normal or leaf):
            continue
        paths = (getattr(module, "__file__", None), getattr(getattr(module, "__spec__", None), "origin", None))
        assert all(paths), f"KERNEL_CHILD_ORIGIN_INVALID: {name}: missing origin"
        for raw in paths:
            path = _KernelOriginPath(raw).resolve()
            assert path.is_file() and path.is_relative_to(expected), f"KERNEL_CHILD_ORIGIN_INVALID: {name}: {path}; expected {expected}"
'''
    prefix = f"_density_leaf_package = {leaf_package!r}\n" + guard
    if leaf_package is None:
        prefix += "\nimport cftuv_envelope\n_check_kernel_origin()\n"
    return ([sys.executable, *flags, "-c", prefix + code + "\n_check_kernel_origin()\n",
             str(PACKAGE_ROOT.parent), *arguments], environment)
