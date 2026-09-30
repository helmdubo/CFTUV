"""pytest-плагин: ставит runtime-шимы `proto_shims` ДО сбора тестов ядра.

    PYTHONSAFEPATH=1 PYTHONPATH=kernel/src;artifacts/numeric_repr \
    NUMERIC_REPR_SHIMS=sign,compare,mul,addsub,radical,floorcache \
    python -m pytest -p pytest_shims kernel/tests/<тесты с замороженными дайджестами>

Замороженные дайджесты (`FROZEN_DIGESTS`, звезда, абсолютные дайджесты P0-3) — оракул «ответ не
сдвинулся». Если прототипы их не трогают, то и настоящая правка тех же решений их не тронет.
"""

import os
import sys


def pytest_configure(config):  # noqa: ARG001
    names = os.environ.get("NUMERIC_REPR_SHIMS", "")
    if not names:
        return
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)
    import proto_shims

    installed = proto_shims.install(names.split(","))
    sys.stderr.write(f"[pytest_shims] installed {installed}\n")
