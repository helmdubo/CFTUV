"""Диагностический свидетель двух ARAP-дайджестов; ядро и эталоны не меняет."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import platform
from pathlib import Path
import sys
import traceback


CASES = (
    ("test_developable_arap.py", "test_the_arap_metric_record_matches_its_golden_digest"),
    ("test_developable_best_proposal.py", "test_the_new_certificate_fields_are_the_only_bytes_that_changed"),
)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def source_hashes(root):
    return {
        path.relative_to(root).as_posix(): digest(path.read_bytes())
        for directory in (root / "src", root / "tests")
        for path in sorted(directory.rglob("*.py"))
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--mode", choices=("source", "wheel"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(root / "tests"))
    if args.mode == "source":
        sys.path.insert(0, str(root / "src"))

    import mpmath
    import pytest
    import sympy
    import cftuv_envelope
    from cftuv_envelope import surface_cone_angle
    from cftuv_envelope.codec import canonical_json_bytes

    package_path = Path(cftuv_envelope.__file__).resolve()
    if args.mode == "source":
        assert package_path.is_relative_to(root / "src"), package_path
    else:
        assert not package_path.is_relative_to(root), package_path
    receipt = {
        "diagnostic_only": True,
        "root": str(root),
        "mode": args.mode,
        "python": sys.version,
        "executable": sys.executable,
        "flags": str(sys.flags),
        "platform": platform.platform(),
        "libc": platform.libc_ver(),
        "float_info": str(sys.float_info),
        "versions": {"sympy": sympy.__version__, "mpmath": mpmath.__version__, "pytest": pytest.__version__},
        "hash_seed": os.environ.get("PYTHONHASHSEED"),
        "hash_probe": hash("CFTUV_ARAP_CI"),
        "environment": {key: os.environ.get(key) for key in (
            "PYTHONPATH", "PYTHONSAFEPATH", "CFTUV_CANONICAL_AUDIT", "CFTUV_SYMBOLIC_REPLAY_CHECK",
            "CFTUV_SYMBOLIC_BACKEND", "CFTUV_NATIVE_STRICT", "PYTEST_DISABLE_PLUGIN_AUTOLOAD",
        )},
        "kernel_path": str(package_path),
        "native_spec": str(importlib.util.find_spec("cftuv_native")),
        "test_order": [name for _, name in CASES],
        "source_hashes_before": source_hashes(root),
        "calls": [],
        "acos_calls": [],
        "test_reports": [],
    }

    original_acos = surface_cone_angle.acos

    def acos_spy(argument):
        result = original_acos(argument)
        receipt["acos_calls"].append({
            "argument_hex": argument.hex(),
            "argument_exact_ratio": argument.as_integer_ratio(),
            "result_hex": result.hex(),
            "result_exact_ratio": result.as_integer_ratio(),
        })
        # Передаём оригинальный float, без округления или пересчёта.
        return result

    surface_cone_angle.acos = acos_spy

    class Witness:
        def pytest_runtest_call(self, item):
            original = item.module.build_metric

            def spy(*arguments, **keywords):
                index = len(receipt["calls"])
                before = canonical_json_bytes(arguments[0])
                record = original(*arguments, **keywords)
                after = canonical_json_bytes(arguments[0])
                payload = canonical_json_bytes(record)
                stem = f"{index:02d}-{item.name}"
                for suffix, content in (("input-before", before), ("input-after", after), ("record", payload)):
                    (args.output / f"{stem}.{suffix}.json").write_bytes(content)
                entry = {
                    "test": item.name,
                    "kwargs": {key: str(value) for key, value in keywords.items()},
                    "input_before_sha256": digest(before),
                    "input_after_sha256": digest(after),
                    "record_sha256": digest(payload),
                    "record_file": f"{stem}.record.json",
                    "input_unchanged": before == after,
                }
                receipt["calls"].append(entry)
                print("ARAP_CANONICAL_INPUT", before.decode("utf-8"))
                print("ARAP_CANONICAL_RECORD", payload.decode("utf-8"))
                print("ARAP_CALL", json.dumps(entry, sort_keys=True))
                # Возвращаем ровно объект оригинального построителя; кэши и входы не трогаем.
                return record

            item.module.build_metric = spy
            item.addfinalizer(lambda: setattr(item.module, "build_metric", original))

        def pytest_runtest_logreport(self, report):
            receipt["test_reports"].append({
                "nodeid": report.nodeid, "phase": report.when, "outcome": report.outcome,
                "failure": str(report.longrepr) if report.failed else "",
            })

    code = 2
    try:
        code = int(pytest.main([
            "-q", "-s", "-p", "no:cacheprovider",
            *(f"{root / 'tests' / file}::{name}" for file, name in CASES),
        ], plugins=[Witness()]))
    except BaseException:
        receipt["exception"] = traceback.format_exc()
    finally:
        surface_cone_angle.acos = original_acos
        receipt["source_hashes_after"] = source_hashes(root)
        receipt["sources_unchanged"] = receipt["source_hashes_before"] == receipt["source_hashes_after"]
        receipt["loaded_modules"] = {
            name: {"path": str(path), "sha256": digest(path.read_bytes())}
            for name, module in sorted(sys.modules.items())
            if (name.startswith("cftuv_envelope") or name in (
                "sympy", "mpmath", "pytest", "developable_factories", "developable_route",
                "test_developable_arap", "test_developable_best_proposal",
            ))
            if (path := Path(getattr(module, "__file__", "") or "")).is_file()
        }
        receipt["exit_code"] = code
        (args.output / "manifest.json").write_text(json.dumps(receipt, ensure_ascii=False, indent=2), encoding="utf-8")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
