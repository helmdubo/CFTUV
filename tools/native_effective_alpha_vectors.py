"""The test vectors of the Rust crate `cftuv-alpha`: pairs the extension answers, written as plain text for `cargo test` (no Python at test time).

    python tools/native_effective_alpha_vectors.py write [--from FILE ...] [--scale 0.2] [--limit 220] [--out native/cftuv-alpha/tests/vectors.txt]

A vector is a FRAME (what `effective_alpha_op._bind_frame` hands the extension: the source start, the length and the two covectors as `terms_wire` ints, the seven binary64 `(centre, bound)` pairs of the
prefilter as hex bit patterns) and the pairs asked against it, each with the oracle's answer (counter delta and contacts: alpha and station as wires, the two texts of the point). The oracle is the
live Python (`boundary._contact_candidates` with no ledger), the pairs are the generated adversarial ones (`native_effective_alpha_synthetic`) and an even stride through recorded files given by `--from`
(the field `building` is the usual one); only pairs the extension answers are written. `cargo test -p cftuv-alpha` replays them (`tests/vectors.rs`): the same contacts, term for term, and the same counts,
with the Rust arithmetic alone. The Python tests (`tests/test_native_effective_alpha.py`) compare the whole corpora with the oracle; these vectors keep the crate honest without Python.

Format (tab-separated; `-` is an empty wire or no floats; ints in decimal; floats are the 14 binary64 bit patterns in hex, comma-separated):

    F  id  start_x  start_y  floats  length  tangent_x  tangent_y  normal_x  normal_y
    P  id  start_x  start_y  end_x  end_y  rejected  passed  contact | contact | ...      contact = alpha ; station ; x text ; y text      wire = r,n,d,r,n,d,...
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import native_effective_alpha_corpus as nec  # noqa: E402

DEFAULT_OUT = ROOT / "native" / "cftuv-alpha" / "tests" / "vectors.txt"


def wire_text(terms) -> str:
    return ",".join(str(item) for item in terms) or "-"


def floats_text(floats) -> str:
    if floats is False:
        return "-"
    return ",".join(f"{struct.unpack('<Q', struct.pack('<d', number))[0]:016x}" for entry in floats for number in entry)


class Writer:
    def __init__(self) -> None:
        self.lines: list = []
        self.frames = 0
        self.pairs = 0

    def add(self, label: str, document: dict, limit: int) -> None:
        import cftuv_native
        from cftuv_envelope.reference.native_exact import terms_wire

        self.lines.append(f"# source {label}")
        objects = nec.Objects(document)
        eligible = [
            pair for pair in document["pairs"]
            if pair["frame"] and pair["mode"] == "NATIVE_EXACT" and pair["answer"][0] == "ok" and not pair["symbolic"] and not any(name in pair["counts"] for name in nec.REFUSAL_COUNTERS)
        ]
        stride = max(1, len(eligible) // limit)
        frame_ids: dict = {}
        taken = 0
        for pair in eligible[::stride]:
            if taken >= limit:
                break
            context, source, boundary, frame = objects.call(pair, "vectors")
            try:
                cftuv_native.effective_alpha_contacts(source, boundary, frame)
            except cftuv_native.NativePortUnsupported:
                continue
            if pair["source"] not in frame_ids:
                frame_ids[pair["source"]] = self.frames
                self.frames += 1
                start = source.start
                self.lines.append("\t".join([
                    "F", str(frame_ids[pair["source"]]), start.x.expression, start.y.expression, floats_text(frame._float_parts()), wire_text(terms_wire(frame.length)),
                    wire_text(terms_wire(frame.tangent_covector[0])), wire_text(terms_wire(frame.tangent_covector[1])),
                    wire_text(terms_wire(frame.normal_covector[0])), wire_text(terms_wire(frame.normal_covector[1])),
                ]))
            contacts = " | ".join(";".join([wire_text(c for term in contact[0][1] for c in term), wire_text(c for term in contact[1][1] for c in term), contact[2], contact[3]]) for contact in pair["answer"][1])
            self.lines.append("\t".join([
                "P", str(frame_ids[pair["source"]]), pair["start"][0], pair["start"][1], pair["end"][0], pair["end"][1],
                str(pair["counts"].get("contact_candidates.prefilter_rejected", 0)), str(pair["counts"].get("contact_candidates.prefilter_passed", 0)), contacts or "-",
            ]))
            taken += 1
            self.pairs += 1
        print(f"{label}: {taken} pairs of {len(eligible)} eligible ({len(document['pairs'])} recorded)")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("write",))
    parser.add_argument("--from", dest="sources", action="append", default=[])
    parser.add_argument("--scale", type=float, default=0.2)
    parser.add_argument("--limit", type=int, default=220, help="pairs per source")
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    arguments = parser.parse_args(argv)
    writer = Writer()
    writer.add("synthetic", nec.synthetic_document(arguments.scale), arguments.limit)
    for path in arguments.sources:
        writer.add(Path(path).name, nec.read_document(Path(path)), arguments.limit)
    out = Path(arguments.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(writer.lines) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {out} ({out.stat().st_size} bytes): {writer.frames} frames, {writer.pairs} pairs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
