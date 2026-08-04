"""Q5: направление сдвига долгов по каждому кейсу. Обратная сторона = дефект."""
import collections, json, sys
from pathlib import Path
from cftuv_envelope.wavefront.skeleton import build_skeleton
from wavefront_cases import named_corpus, partial_source_corpus

R = json.loads(Path(sys.argv[1]).read_text())
CASES = {c["case_id"]: c["new"] for c in R["cases"]}
rows, verdict = [], "РАСПИСКА"
for case_id, polygon in (
    *((f"named::{n}", p) for n, p in named_corpus()),
    *((f"partial_source::{n}", p) for n, p in partial_source_corpus()),
):
    sk = build_skeleton(polygon)
    got = collections.Counter(
        f"{o.cause.value}|{o.disposition.value}" for o in sk.proof_obligations)
    want = collections.Counter(CASES[case_id]["proof_counts"])
    if got == want:
        continue
    grew = {k: (want.get(k, 0), got.get(k, 0))
            for k in set(want) | set(got) if got.get(k, 0) > want.get(k, 0)}
    shrank = {k: (want.get(k, 0), got.get(k, 0))
              for k in set(want) | set(got) if got.get(k, 0) < want.get(k, 0)}
    unproven_before = sum(v for k, v in want.items()
                          if "SURVIVED" in k or "UNPROVEN" in k)
    unproven_after = sum(v for k, v in got.items()
                         if "SURVIVED" in k or "UNPROVEN" in k)
    if grew:
        verdict = "ДЕФЕКТ"
    rows.append({
        "case": case_id,
        "obligations_before": sum(want.values()),
        "obligations_after": sum(got.values()),
        "unproven_before": unproven_before,
        "unproven_after": unproven_after,
        "shrank": shrank, "grew": grew,
        "direction": "МЕНЬШЕ" if not grew else "ВЫРОСЛО — ДЕФЕКТ",
    })
print("вердикт по Q5:", verdict, "| кейсов со сдвигом:", len(rows))
print(json.dumps(rows, indent=1, ensure_ascii=False))
json.dump({"verdict": verdict, "cases": rows},
          open(sys.argv[2], "w"), indent=1, ensure_ascii=False)
