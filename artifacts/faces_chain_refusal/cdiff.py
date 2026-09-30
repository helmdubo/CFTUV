import sys
import env  # noqa: F401
from cftuv_envelope import exact_sqrt_sum as canon
from diag17c import stage


def main():
    from cftuv_envelope.wavefront import prepare_conveyor
    snap, mk = stage(17)
    res = {}
    for d in (1, 2):
        canon.reset_factorization_memory()
        p = prepare_conveyor(snap, mk(d))
        sk = p.regions[0].skeleton
        res[d] = (dict(p.counters), dict(sk.counters), sk.proof_status, len(sk.proof_obligations))
    for d in (1, 2):
        print("d", d, "proof", res[d][2], "obligations", res[d][3])
    for name, i in (("prepared", 0), ("skeleton", 1)):
        keys = sorted(set(res[1][i]) | set(res[2][i]))
        print(name, {k: (res[1][i].get(k), res[2][i].get(k)) for k in keys if res[1][i].get(k) != res[2][i].get(k)})


if __name__ == "__main__":
    main()
