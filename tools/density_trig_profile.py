"""Границы реально достигнутых аргументов тригонометрии из iv trace."""

import argparse
from collections import Counter
from fractions import Fraction as F
import json
import lzma
from pathlib import Path
import sys

sys.set_int_max_str_digits(0)


def number(raw):
    sign, mantissa, exponent, bit_count = raw
    if bit_count < 0:
        raise ValueError("nonfinite raw MPF")
    value = F((-1 if sign else 1)*mantissa)
    return value * 2**exponent if exponent >= 0 else value / 2**(-exponent)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(lzma.decompress(args.file.read_bytes()))
    assert data["schema"] == "cftuv.density-interval-trace.v1"
    nodes = data["expressions"]
    rows = {(node, precision): outcome for node, precision, outcome in data["records"]}
    records, counts = [], Counter()
    for node, precision, outcome in data["records"]:
        tag = nodes[node][0]
        if tag not in ("sin", "cos", "atan", "atan2"):
            continue
        counts[tag] += 1
        item = {"node": node, "tag": tag, "precision": precision, "arguments": []}
        for argument in nodes[node][1:]:
            observed = rows.get((argument, precision))
            if observed is None or observed[0] != "ok":
                item["arguments"].append({"node": argument, "status": "UNRECORDED_OR_FAILED"})
                continue
            bounds = observed[1]
            try:
                values = list(map(number, bounds))
                item["arguments"].append({"node": argument, "status": "BOUNDED",
                    "absolute_at_most_8": all(abs(value) <= 8 for value in values),
                    "raw": bounds})
            except ValueError:
                item["arguments"].append({"node": argument, "status": "NONFINITE", "raw": bounds})
        records.append(item)
    result = {"scope": "observed interval trace only; no universal input-domain proof",
              "file": str(args.file), "counts": dict(counts), "records": records,
              "precisions": sorted({item["precision"] for item in records}),
              "uncovered_arguments": sum(arg["status"] != "BOUNDED" for item in records for arg in item["arguments"]),
              "arguments_outside_8": sum(not arg["absolute_at_most_8"] for item in records for arg in item["arguments"] if arg["status"] == "BOUNDED")}
    args.out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if key != "records"}), flush=True)


if __name__ == "__main__":
    main()
