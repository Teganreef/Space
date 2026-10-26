"""Summarise a fault campaign.

    python3 tools/report.py results/campaign.jsonl --out results

Writes results/summary.md (tables) and results/*.png (figures). Every number
comes from the JSON lines produced by bdio_campaign; nothing is hand-entered.
"""

import argparse
import collections
import json
import pathlib

import style

SCENARIOS = ["nominal", "sensor", "power", "comms", "seu", "combined", "seu_storm"]
CONTROLLERS = ["naive", "hardened"]
VIOLATIONS = [
    "deep_discharge",
    "payload_low_power",
    "invalid_mode",
    "payload_in_safe",
    "duplicate_exec",
    "stale_exec",
]


def load(path):
    runs = collections.defaultdict(list)
    with open(path) as f:
        for line in f:
            r = json.loads(line)
            runs[(r["scenario"], r["controller"])].append(r)
    return runs


def summarise(rows):
    n = len(rows)
    scored = sum(r["faults_scored"] for r in rows)
    detected = sum(r["faults_detected"] for r in rows)
    lat = sum(r["latency_sum"] for r in rows)
    planned = sum(r["planned_science_ticks"] for r in rows)
    science = sum(r["science_ticks"] for r in rows)
    recov = sum(r["recoveries"] for r in rows)
    return {
        "runs": n,
        "runs_with_violation": sum(1 for r in rows if r["total_violations"] > 0),
        "violations": sum(r["total_violations"] for r in rows),
        "by_violation": {v: sum(r["violations"][v] for r in rows) for v in VIOLATIONS},
        "detect_rate": detected / scored if scored else None,
        "mean_latency": lat / detected if detected else None,
        "safe_entries": sum(r["safe_entries"] for r in rows),
        "false_safe_per_run": sum(r["false_safe_entries"] for r in rows) / n,
        "mean_recovery_min": (sum(r["recovery_ticks_sum"] for r in rows) / recov / 60) if recov else None,
        "science_pct": 100 * science / planned if planned else None,
        "min_soc": min(r["min_soc"] for r in rows),
    }


def by_kind(rows):
    agg = collections.defaultdict(lambda: [0, 0, 0])
    for r in rows:
        for k, v in r["by_kind"].items():
            for i in range(3):
                agg[k][i] += v[i]
    return agg


def fmt(x, spec="{:.1f}", none="n/a"):
    return none if x is None else spec.format(x)


def write_tables(runs, out):
    lines = []
    n_runs = len(next(iter(runs.values())))
    lines.append(f"Campaign: {n_runs} missions per scenario per controller, same seeds for both.\n")
    lines.append("| Scenario | Controller | Runs with a safety violation | Violation episodes | "
                 "Faults detected | Mean detection latency (s) | False safe-mode entries / run | "
                 "Science vs. plan |")
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|")
    for s in SCENARIOS:
        for c in CONTROLLERS:
            if (s, c) not in runs:
                continue
            m = summarise(runs[(s, c)])
            pct = 100 * m["runs_with_violation"] / m["runs"]
            det = None if m["detect_rate"] is None else 100 * m["detect_rate"]
            lines.append(
                f"| {s} | {c} | {pct:.1f}% | {m['violations']} | {fmt(det, '{:.1f}%')} | "
                f"{fmt(m['mean_latency'])} | {m['false_safe_per_run']:.2f} | {fmt(m['science_pct'], '{:.1f}%')} |"
            )

    lines.append("\n### Violations by type (all scenarios)\n")
    lines.append("| Controller | " + " | ".join(VIOLATIONS) + " |")
    lines.append("|---|" + "---:|" * len(VIOLATIONS))
    for c in CONTROLLERS:
        totals = collections.Counter()
        for s in SCENARIOS:
            if (s, c) in runs:
                totals.update(summarise(runs[(s, c)])["by_violation"])
        lines.append(f"| {c} | " + " | ".join(str(totals[v]) for v in VIOLATIONS) + " |")

    lines.append("\n### Detection by fault type (hardened controller, all scenarios)\n")
    lines.append("| Fault | Observable events | Detected | Mean latency (s) |")
    lines.append("|---|---:|---:|---:|")
    agg = collections.defaultdict(lambda: [0, 0, 0])
    for s in SCENARIOS:
        if (s, "hardened") in runs:
            for k, v in by_kind(runs[(s, "hardened")]).items():
                for i in range(3):
                    agg[k][i] += v[i]
    for k in sorted(agg):
        sc, d, lat = agg[k]
        lines.append(f"| {k} | {sc} | {100 * d / sc:.1f}% | {fmt(lat / d if d else None)} |")

    (out / "summary.md").write_text("\n".join(lines) + "\n")
    return "\n".join(lines)


def plot_violations(runs, out):
    plt = style.plt
    fig, ax = plt.subplots(figsize=(7.5, 3.2))
    scen = [s for s in SCENARIOS if (s, "naive") in runs]
    w = 0.38
    for i, c in enumerate(CONTROLLERS):
        vals = []
        for s in scen:
            m = summarise(runs[(s, c)])
            vals.append(100 * m["runs_with_violation"] / m["runs"])
        xs = [j + (i - 0.5) * (w + 0.02) for j in range(len(scen))]
        bars = ax.bar(xs, vals, width=w, color=style.CONTROLLER_COLOR[c], label=c)
        for b, v in zip(bars, vals):
            ax.annotate(f"{v:.0f}%", (b.get_x() + b.get_width() / 2, v), xytext=(0, 2),
                        textcoords="offset points", ha="center", fontsize=7, color=style.TEXT_2)
    ax.set_xticks(range(len(scen)), scen)
    ax.set_ylabel("missions with a safety violation (%)")
    ax.set_title("Missions that broke a safety invariant")
    ax.set_ylim(0, 110)
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(out / "violations.png", dpi=160)
    plt.close(fig)


def plot_false_safe(runs, out):
    plt = style.plt
    fig, ax = plt.subplots(figsize=(7.5, 3.0))
    scen = [s for s in SCENARIOS if (s, "naive") in runs]
    w = 0.38
    for i, c in enumerate(CONTROLLERS):
        vals = [summarise(runs[(s, c)])["false_safe_per_run"] for s in scen]
        xs = [j + (i - 0.5) * (w + 0.02) for j in range(len(scen))]
        ax.bar(xs, vals, width=w, color=style.CONTROLLER_COLOR[c], label=c)
    ax.set_xticks(range(len(scen)), scen)
    ax.set_ylabel("unjustified safe-mode entries per mission")
    ax.set_title("False alarms")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out / "false_safe.png", dpi=160)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("campaign")
    p.add_argument("--out", default="results")
    args = p.parse_args()
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    style.apply()
    runs = load(args.campaign)
    print(write_tables(runs, out))
    plot_violations(runs, out)
    plot_false_safe(runs, out)


if __name__ == "__main__":
    main()
