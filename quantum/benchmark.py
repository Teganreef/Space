"""Compare replanning solvers on random instances.

    cd quantum && python3 benchmark.py --out ../results

Solvers: exact search, the onboard greedy policy, simulated annealing on the
QUBO, and QAOA (statevector simulation) at several depths. For QAOA the
interesting number is not the best of 4096 shots - sampling that many
bitstrings from a 10 qubit space finds the optimum by brute force alone - but
the probability that a single shot is optimal or feasible, compared against
sampling uniformly at random.
"""

from __future__ import annotations

import argparse
import itertools
import json
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "tools"))

from problem import make_instance, to_qubo  # noqa: E402
from qaoa import run_qaoa  # noqa: E402
from solvers import exact, greedy, simulated_annealing  # noqa: E402


def random_baseline(inst):
    feas = sum(1 for x in itertools.product((0, 1), repeat=inst.n) if inst.feasible(x))
    return feas / 2 ** inst.n


def bench(orbits, n_instances, qaoa_reps, qaoa_instances, seed):
    rows = []
    for k in range(n_instances):
        rng = np.random.default_rng(seed + k)
        inst = make_instance(rng, orbits)
        q = to_qubo(inst)
        t0 = time.perf_counter()
        best, _, n_opt = exact(inst)
        t_exact = time.perf_counter() - t0

        g = greedy(inst)
        sa = simulated_annealing(q, np.random.default_rng(seed + k), sweeps=500, restarts=4)
        sa_sel = tuple(int(b) for b in sa[: inst.n])

        row = {
            "orbits": orbits,
            "targets": inst.n,
            "qubits": inst.n_qubits,
            "budget_units": inst.budget,
            "optimum": best,
            "exact_s": t_exact,
            "greedy": inst.value(g) if inst.feasible(g) else 0,
            "annealing": inst.value(sa_sel) if inst.feasible(sa_sel) else 0,
            "random_p_optimal": n_opt / 2 ** inst.n,
            "random_p_feasible": random_baseline(inst),
            "qaoa": {},
        }
        if k < qaoa_instances:
            for p in qaoa_reps:
                t0 = time.perf_counter()
                r = run_qaoa(inst, q, best, reps=p, restarts=2, seed=seed + k)
                row["qaoa"][p] = {
                    "best_value": r.best_value,
                    "p_optimal": r.p_optimal,
                    "p_feasible": r.p_feasible,
                    "seconds": time.perf_counter() - t0,
                }
        rows.append(row)
        print(f"orbits={orbits} instance={k} optimum={best} greedy={row['greedy']} "
              f"annealing={row['annealing']} qaoa={ {p: round(v['p_optimal'], 3) for p, v in row['qaoa'].items()} }",
              flush=True)
    return rows


def ratio(rows, key):
    return float(np.mean([r[key] / r["optimum"] for r in rows])), sum(r[key] == r["optimum"] for r in rows)


def write_report(rows, out):
    lines = ["# Replanning benchmark", ""]
    for orbits in sorted({r["orbits"] for r in rows}):
        rs = [r for r in rows if r["orbits"] == orbits]
        lines.append(f"## {orbits} orbit{'s' if orbits > 1 else ''}: {rs[0]['targets']} targets, "
                     f"{min(r['qubits'] for r in rs)}-{max(r['qubits'] for r in rs)} qubits, {len(rs)} instances")
        lines.append("")
        lines.append("| Solver | Mean value / optimum | Instances solved optimally |")
        lines.append("|---|---:|---:|")
        lines.append(f"| Exact search | 1.000 | {len(rs)}/{len(rs)} |")
        for name, key in (("Greedy (onboard policy)", "greedy"), ("Simulated annealing", "annealing")):
            m, n = ratio(rs, key)
            lines.append(f"| {name} | {m:.3f} | {n}/{len(rs)} |")
        q = [r for r in rs if r["qaoa"]]
        if q:
            lines.append("")
            lines.append(f"QAOA on {len(q)} of these instances (statevector simulation, 4096 shots):")
            lines.append("")
            lines.append("| Method | P(single shot optimal) | P(single shot feasible) | Best of 4096 = optimum | Mean time (s) |")
            lines.append("|---|---:|---:|---:|---:|")
            lines.append(f"| Uniform random bits | {np.mean([r['random_p_optimal'] for r in q]):.3f} | "
                         f"{np.mean([r['random_p_feasible'] for r in q]):.3f} | n/a | n/a |")
            for p in sorted({p for r in q for p in r["qaoa"]}):
                vals = [r["qaoa"][p] for r in q if p in r["qaoa"]]
                hits = sum(v["best_value"] == r["optimum"] for v, r in zip(vals, q))
                lines.append(f"| QAOA p={p} | {np.mean([v['p_optimal'] for v in vals]):.3f} | "
                             f"{np.mean([v['p_feasible'] for v in vals]):.3f} | {hits}/{len(vals)} | "
                             f"{np.mean([v['seconds'] for v in vals]):.1f} |")
        lines.append("")
    (out / "quantum_benchmark.md").write_text("\n".join(lines))
    (out / "quantum_benchmark.json").write_text(json.dumps(rows, indent=1))
    print("\n".join(lines))


def plot(rows, out):
    import style

    style.apply()
    plt = style.plt
    q = [r for r in rows if r["qaoa"] and r["orbits"] == 1]
    if not q:
        return
    reps = sorted({p for r in q for p in r["qaoa"]})
    labels = ["random"] + [f"QAOA p={p}" for p in reps]
    opt = [np.mean([r["random_p_optimal"] for r in q])] + [np.mean([r["qaoa"][p]["p_optimal"] for r in q]) for p in reps]
    feas = [np.mean([r["random_p_feasible"] for r in q])] + [np.mean([r["qaoa"][p]["p_feasible"] for r in q]) for p in reps]
    fig, axes = plt.subplots(1, 2, figsize=(8, 3))
    for ax, vals, title in ((axes[0], opt, "P(single shot is optimal)"), (axes[1], feas, "P(single shot is feasible)")):
        ax.bar(labels, vals, color=["#9a9893"] + ["#6250d6"] * len(reps), width=0.6)
        for i, v in enumerate(vals):
            ax.annotate(f"{v:.2f}", (i, v), xytext=(0, 2), textcoords="offset points", ha="center", fontsize=7,
                        color=style.TEXT_2)
        ax.set_title(title, loc="left")
        ax.set_ylim(0, 1.0 if "feasible" in title else max(vals) * 1.4)
    lo, hi = min(r["qubits"] for r in q), max(r["qubits"] for r in q)
    qubits = f"{lo}" if lo == hi else f"{lo}-{hi}"
    fig.suptitle(f"QAOA vs random sampling, {len(q)} one-orbit instances ({qubits} qubits)",
                 fontsize=10, x=0.01, ha="left", color=style.TEXT)
    fig.tight_layout()
    fig.savefig(out / "quantum_benchmark.png", dpi=160)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="../results")
    ap.add_argument("--instances", type=int, default=12)
    ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--two-orbit-qaoa", type=int, default=3, help="two-orbit instances to also run QAOA p=1 on")
    ap.add_argument("--replot", action="store_true", help="redraw tables and figure from an earlier run's JSON")
    args = ap.parse_args()
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    if args.replot:
        rows = json.loads((out / "quantum_benchmark.json").read_text())
        for r in rows:
            r["qaoa"] = {int(p): v for p, v in r["qaoa"].items()}
        write_report(rows, out)
        plot(rows, out)
        return

    rows = bench(1, args.instances, (1, 2, 3), args.instances, args.seed)
    rows += bench(2, args.instances, (1,), args.two_orbit_qaoa, args.seed + 1000)
    write_report(rows, out)
    plot(rows, out)


if __name__ == "__main__":
    main()
