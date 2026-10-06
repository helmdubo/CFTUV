"""Догон портов за ядром (`tools/native_catchup.py`): пин переписывается из дерева ядра и называет изменённые файлы; расширение и Blender не нужны.

Шаги, которые гоняют Blender, сборку и тесты, здесь не запускаются (они — сама проверка); держится то, что можно проверить на чистом клоне: пересборка блока
`PINS` в копии `pin.py` по поддельному дереву эталона, названные изменённые файлы, отказ при пропаже файла и идемпотентность. Равенство пина ТЕКУЩЕМУ дереву ядра
здесь не проверяется: пока основная ветка двигает ядро, чистый клон обязан оставаться зелёным (нативные сверки пропускаются с названной причиной, `native_gate`).
"""

from __future__ import annotations

import importlib.util
import shutil
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def _load_tool():
    spec = importlib.util.spec_from_file_location("native_catchup", ROOT / "tools" / "native_catchup.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["native_catchup"] = module
    spec.loader.exec_module(module)
    return module


catchup = _load_tool()


def _fake_oracle(tmp_path: Path, pin) -> Path:
    root = tmp_path / "cftuv_envelope"
    for name in catchup.pinned_files(pin):
        target = root / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(f"# {name}\r\nvalue = 1\r\n".encode())
    return root


def test_rewriting_the_pin_from_a_tree_names_what_changed_and_is_idempotent(tmp_path):
    pin_file = tmp_path / "pin.py"
    shutil.copyfile(catchup.PIN_FILE, pin_file)
    pin = catchup.load_pin(pin_file)
    root = _fake_oracle(tmp_path, pin)
    changed = catchup.rewrite_pins(pin_file, root)
    assert sorted(changed) == catchup.pinned_files(pin), "every digest of the fake tree differs from the pin of the repository"
    rewritten = catchup.load_pin(pin_file)
    assert catchup.changed_files(rewritten, root) == {operation: [] for operation in rewritten.OPERATION_FILES}
    assert catchup.rewrite_pins(pin_file, root) == [], "a second rewrite changes nothing"
    # one file moves: only it is named, and only its line changes
    (root / "materialize" / "clip.py").write_bytes(b"value = 2\n")
    assert catchup.changed_files(rewritten, root)["clip"] == ["materialize/clip.py"]
    assert catchup.rewrite_pins(pin_file, root) == ["materialize/clip.py"]
    assert catchup.changed_files(catchup.load_pin(pin_file), root) == {operation: [] for operation in rewritten.OPERATION_FILES}
    text = pin_file.read_text(encoding="utf-8")
    assert text.count("\nPINS: dict = {\n") == 1 and text.count('    "materialize/clip.py": "') == 1


def test_a_missing_oracle_file_refuses_and_leaves_the_pin_alone(tmp_path):
    pin_file = tmp_path / "pin.py"
    shutil.copyfile(catchup.PIN_FILE, pin_file)
    pin = catchup.load_pin(pin_file)
    root = _fake_oracle(tmp_path, pin)
    (root / "numeric.py").unlink()
    before = pin_file.read_text(encoding="utf-8")
    with pytest.raises(SystemExit) as refusal:
        catchup.rewrite_pins(pin_file, root)
    assert "numeric.py" in str(refusal.value)
    assert pin_file.read_text(encoding="utf-8") == before
