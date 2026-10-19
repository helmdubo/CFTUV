"""CHAIN_STATION_PINS_APPROVED_V1: the production table of the native chain_station is data only and exactly the approved digests (mechanical, no extension import)."""
import ast
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "native/cftuv-python/python/cftuv_native"

#: CHAIN_STATION_PINS_APPROVED_V1 (2026-10-19, by the gates, PIN_APPROVAL_BY_GATES_V1): exactly the R3+ S2 candidate at fc817d2d.
APPROVED = {
    "_chain_station.py": "ee1394436daf4539658b63fcdba8b1d5e629fd8cc51f704d797f2a8868cb2c21",
    "contracts/chain_station.py": "7c74d5446e2aded5af0d711885fec8218dfaa5de9ef8adf9b7074afda1d47e8e",
}


def test_production_table_is_exactly_the_approved_pins():
    tree = ast.parse((PACKAGE / "chain_station_approved_pins.py").read_text(encoding="utf-8"))
    body = [node for node in tree.body if not (isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant))]
    assert len(body) == 1 and isinstance(body[0], ast.Assign) and [target.id for target in body[0].targets] == ["APPROVED"], "data only: one literal, no code"
    assert ast.literal_eval(body[0].value) == APPROVED


def test_the_operation_answers_under_exactly_that_table():
    tree = ast.parse((PACKAGE / "chain_station_op.py").read_text(encoding="utf-8"))
    statement = next(node for node in tree.body if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "APPROVED_PINS" for target in node.targets))
    assert ast.dump(statement.value) == ast.dump(ast.parse("MappingProxyType(_APPROVED)", mode="eval").body)
    imports = [node for node in tree.body if isinstance(node, ast.ImportFrom) and node.module == "chain_station_approved_pins"]
    assert len(imports) == 1 and [(alias.name, alias.asname) for alias in imports[0].names] == [("APPROVED", "_APPROVED")] and imports[0].level == 1
    assert not any(isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "CANDIDATE" for target in node.targets) for node in tree.body)


def test_the_pinned_files_of_this_tree_are_the_approved_ones():
    kernel = ROOT / "kernel/src/cftuv_envelope"
    for name, digest in APPROVED.items():
        assert hashlib.sha256((kernel / name).read_bytes().replace(b"\r\n", b"\n")).hexdigest() == digest, name
