"""Actual libmp calls при свежем вычислении тригонометрических узлов поля."""

from collections import Counter
from contextlib import contextmanager
import argparse
import hashlib
import json
import lzma
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel" / "src"))
sys.path.insert(0, str(ROOT / "tools"))
sys.set_int_max_str_digits(0)

from mpmath import iv
from mpmath.libmp import libelefun, libmpi
from cftuv_envelope._density_policy import density_interval_enclosure
from native_density_wire import Wire, json_key

NAMES = ("cos_sin_basecase", "cos_sin_fixed", "pi_fixed", "mpf_pi",
         "atan_taylor_get_cached", "mpf_cos_sin", "mpf_atan", "mpf_atan2", "mod_pi2")
INTERVAL_NAMES = ("mpi_cos_sin", "mpi_cos", "mpi_sin", "mpi_atan", "mpi_atan2")


def plain(value):
    if value is None or type(value) in (str, bool):
        return value
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return int(value)


@contextmanager
def capture(records, counts):
    saved = []

    def wrapper(name, original):
        def traced(*args, **kwargs):
            try:
                result = original(*args, **kwargs)
            except Exception as error:
                outcome = ["raised", type(error).__name__, str(error)]
                remember(name, args, kwargs, outcome)
                raise
            remember(name, args, kwargs, ["ok", plain(result)])
            return result
        return traced

    def remember(name, args, kwargs, outcome):
        row = [name, plain(args), {key: plain(value) for key, value in kwargs.items()}, outcome]
        key = json_key(row)
        counts[key] += 1
        if counts[key] == 1:
            records.append(row)

    originals = {name: getattr(libelefun, name) for name in NAMES}
    originals.update({name: getattr(libmpi, name) for name in INTERVAL_NAMES})
    wrappers = {id(function): wrapper(name, function) for name, function in originals.items()}
    for name, module in list(sys.modules.items()):
        if name.startswith("mpmath") and module is not None:
            for attribute, value in list(vars(module).items()):
                if id(value) in wrappers:
                    saved.append((module, attribute, value))
                    setattr(module, attribute, wrappers[id(value)])
    try:
        yield
    finally:
        for module, attribute, value in saved:
            setattr(module, attribute, value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    document = json.loads(lzma.decompress(args.file.read_bytes()))
    wire = Wire(document["expressions"])
    records, counts, evaluated = [], Counter(), []
    saved = iv.prec
    try:
        iv.prec = 160
        with capture(records, counts):
            for index, node in enumerate(wire.nodes):
                if node[0] not in ("sin", "cos", "atan", "atan2"):
                    continue
                try:
                    result = density_interval_enclosure(wire.unexpression(index))
                    evaluated.append([index, "ok", plain(result._mpi_)])
                except Exception as error:
                    evaluated.append([index, "raised", type(error).__name__, str(error)])
    finally:
        iv.prec = saved
    result = {"scope": "fresh evaluation of recorded trig nodes at160; not replay of warm metric intervals",
              "file": str(args.file), "records": records, "evaluated": evaluated,
              "occurrences": [counts[json_key(row)] for row in records]}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(lzma.compress(json_key(result).encode("ascii")))
    summary = {"scope": result["scope"], "nodes": len(evaluated),
               "raised_nodes": sum(row[1] == "raised" for row in evaluated),
               "unique_calls": dict(Counter(row[0] for row in records)),
               "basecase_precisions": sorted({row[1][1] for row in records if row[0] == "cos_sin_basecase"}),
               "atan_cache_precisions": sorted({row[1][1] for row in records if row[0] == "atan_taylor_get_cached"}),
               "pi_fixed_bits": sorted({row[1][0] for row in records if row[0] == "pi_fixed"}),
               "sha256": hashlib.sha256(args.out.read_bytes()).hexdigest()}
    args.out.with_suffix(".receipt.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
