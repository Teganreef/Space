"""Replay one seed through both controllers and plot what happened.

    python3 tools/plot_run.py --scenario combined --seed 42 --out results/replay.png

Runs sfs_run twice (naive, hardened) with the same seed, so both see the same
faults, then draws true vs. estimated state of charge, safe-mode spans and
fault onsets on a shared time axis.
"""

import argparse
import csv
import pathlib
import subprocess
import tempfile

import style


def run(binary, scenario, seed, controller, tmp):
    csv_path = tmp / f"{controller}.csv"
    ev_path = tmp / f"{controller}_events.csv"
    subprocess.run(
        [binary, "--scenario", scenario, "--seed", str(seed), "--controller", controller,
         "--csv", str(csv_path), "--events", str(ev_path)],
        check=True, stdout=subprocess.DEVNULL,
    )
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    with open(ev_path) as f:
        events = list(csv.DictReader(f))
    return rows, events


def spans(rows, mode):
    out, start = [], None
    for r in rows:
        t = int(r["tick"])
        if r["mode"] == mode and start is None:
            start = t
        elif r["mode"] != mode and start is not None:
            out.append((start, t))
            start = None
    if start is not None:
        out.append((start, int(rows[-1]["tick"])))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--binary", default="build/sfs_run")
    p.add_argument("--scenario", default="combined")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--out", default="results/replay.png")
    args = p.parse_args()
    style.apply()
    plt = style.plt

    with tempfile.TemporaryDirectory() as d:
        tmp = pathlib.Path(d)
        data = {c: run(args.binary, args.scenario, args.seed, c, tmp) for c in ("naive", "hardened")}

    fig, axes = plt.subplots(2, 1, figsize=(8, 4.8), sharex=True)
    for ax, (c, (rows, events)) in zip(axes, data.items()):
        hours = [int(r["tick"]) / 3600 for r in rows]
        for a, b in spans(rows, "safe"):
            ax.axvspan(a / 3600, b / 3600, color=style.SAFE_SPAN, lw=0, label="_safe")
        faults = [e for e in events if e["type"] == "fault" and e["detail"] != "seu"]
        for k, e in enumerate(faults):
            x = int(e["tick"]) / 3600
            ax.axvline(x, color=style.FAULT_LINE, lw=1, ls=(0, (3, 2)))
            ax.annotate(e["detail"], (x, 1.02), xytext=(3, -2 - 22 * (k % 3)), textcoords="offset points",
                        fontsize=6.5, color=style.TEXT_2, va="top", ha="left")
        ax.plot(hours, [float(r["soc_est"]) for r in rows], color=style.CONTROLLER_COLOR[c], lw=1.4,
                label=f"{c} estimate")
        ax.plot(hours, [float(r["soc_true"]) for r in rows], color=style.TRUTH, lw=1.2, label="true charge")
        viol = [e for e in events if e["type"] == "violation"]
        n_v = len(viol)
        ax.set_title(f"{c}: {n_v} safety violation{'s' if n_v != 1 else ''}", loc="left")
        ax.set_ylim(0, 1.05)
        ax.set_ylabel("state of charge")
        ax.axhline(0.2, color=style.GRID, lw=1)
        ax.legend(loc="lower left", ncol=2)
    axes[-1].set_xlabel("mission time (hours)   grey = safe mode, dashed = fault onset")
    fig.suptitle(f"Scenario '{args.scenario}', seed {args.seed}: same faults, two controllers",
                 fontsize=11, color=style.TEXT, x=0.01, ha="left")
    fig.tight_layout()
    pathlib.Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=160)
    print(args.out)


if __name__ == "__main__":
    main()
